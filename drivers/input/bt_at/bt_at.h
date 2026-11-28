// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// Copyright (c) 2026 KI9NG
// A Bluetooth module that speaks AT lines on a UART: PTT and keys from its PTT button and its hand microphone, taken a
// byte at a time from a UART interrupt. Pure and allocation free, so the host tests run it. The board's hardware page
// has the module, its wiring and these lines (inferred, untested on a radio).
//
// The radio asks with BT_AT_PROBE ("AT+VER?"); the module answers lines that start with '+', 'O' (OK) or 'E' (ERR)
// and end with LF (a CR before it is dropped). A line longer than BT_AT_LINE_MAX is dropped whole. Any line counts as
// the module answering. A +VER line naming the APRS add-on board (which sits on the same UART) marks it: that board
// isn't a Bluetooth module.
//
// APRS board frames (ki9ng fork): the AT-D578UV PLUS's APRS add-on board decodes APRS in hardware and sends each
// frame as "+APRS:", '~', the AX.25 frame with its FCS, '~' and no line end (its firmware, M V2.00). With aprs_buf
// set, the bytes after "+APRS:" go there (in the UART interrupt) instead of the line parser, and aprs_done gets the
// length (FCS included, not checked here). Like the board, the frame ends at the first '~' after its UI header
// (03 F0): a '~' in the address field is data. Without aprs_buf they're dropped as before (errors).
//
// PTT has three sources, any one held keys the radio:
//   +PTTKEY=1 / 0          the PTT button (key fob): pressed, released
//   +HFREQ:1 / 0           the hands-free button: 1 toggles a latch, 0 clears it
//   +ADATA:nn,len          a data tunnel: the next len bytes are lines of the hand microphone's own: +PTT=P (press),
//                          +PTT=R (release), +KEY=NN (a key event, two decimal digits; 01-04 only)
// A link dropped (+DISC=, or +CONN= with the all-zero address) or the module restarting (+READY) clears every PTT
// source, so a link lost mid-transmission doesn't hold the transmitter on.
//
// Links: +CONN= with an address counts one up, +DISC= or the all-zero address one down, +READY clears them. The
// board uses the count to know the module serves nobody (links == 0) and may be powered off for a while.
//
// Raw payloads (ki9ng fork, CONFIG_BT_KISS): the same +ADATA tunnel carries what a phone writes to the module's SPP
// server (RFCOMM channel 2). A payload whose first byte isn't '+' is taken as raw bytes, all len of them, handed to
// raw_sink (in the UART interrupt) instead of being parsed as lines; a KISS frame starts with FEND (0xC0). The
// module's OK and ERR replies are counted (oks, errs): the board paces its own AT+ADATA sends on them.
#ifndef BT_AT_H
#define BT_AT_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define BT_AT_PROBE		"AT+VER?\r\n"
#define BT_AT_LINE_MAX		64u	// the longest line kept (a +CONN line with a device name)
#define BT_AT_KEY_MIN		1u	// +KEY codes acted on
#define BT_AT_KEY_MAX		4u
#define BT_AT_LINKS_MAX		8u	// a count past the module's own limit: a lost +DISC can't wrap it

typedef struct {
	char line[BT_AT_LINE_MAX];
	uint16_t n;			// bytes of the line so far (counts past BT_AT_LINE_MAX: the line is dropped)
	uint16_t payload;		// bytes of an +ADATA payload still to come
	bool present;			// the module has answered
	bool aprs_board;		// what answered is the APRS add-on board, not a Bluetooth module
	bool ver;			// a +VER line came (the answer to BT_AT_PROBE, not just +READY)
	bool fob_ptt, hf_ptt, mic_ptt;	// the three PTT sources
	uint8_t key;			// the last +KEY code (BT_AT_KEY_MIN..MAX), 0 for none yet
	uint32_t key_events;		// +KEY lines taken: a new one is a new key press
	uint8_t links;			// links up (key fob, hand mic, headset, host), BT_AT_LINKS_MAX at most
	uint32_t lines, errors;		// lines taken; bytes or lines dropped (diagnostics)
	bool payload_first;		// the next byte is an +ADATA payload's first: it decides lines or raw
	bool raw;			// the payload under way is raw bytes (its first byte wasn't '+')
	bool bare;			// a KISS frame with no +ADATA header under way: raw up to its closing FEND
	bool bare_body;		// ... and a byte past its opening FEND has come
	uint32_t bare_frames;		// such frames taken (diagnostics)
	uint32_t raw_bytes;		// raw payload bytes taken
	uint32_t oks, errs;		// OK and ERR replies from the module
	void (*raw_sink)(uint8_t c);	// where raw payload bytes go (NULL: dropped); called in the UART interrupt
	uint8_t *aprs_buf;		// where an APRS board frame goes (NULL: not taken), aprs_max bytes
	uint16_t aprs_max;
	void (*aprs_done)(uint16_t len);	// a frame is complete in aprs_buf (UART interrupt)
	bool aprs;			// an APRS board frame under way
	bool aprs_lead;			// ... its opening '~' not yet past
	bool aprs_ui;			// ... its UI header seen: the next '~' closes it
	uint16_t aprs_len;		// ... bytes so far
	uint16_t aprs_ctl;		// ... where its control byte is (0: the address field hasn't ended yet)
	uint32_t aprs_frames, aprs_overflows;	// frames handed on; frames dropped as too long (or not UI)
} bt_at_t;

static inline void bt_at_init(bt_at_t *b)
{
	memset(b, 0, sizeof(*b));
}

static inline bool bt_at_ptt(const bt_at_t *b)
{
	return b->fob_ptt || b->hf_ptt || b->mic_ptt;
}

static inline void bt_at_release(bt_at_t *b)
{
	b->fob_ptt = false;
	b->hf_ptt = false;
	b->mic_ptt = false;
}

// A UART error (overrun, framing, noise): the line under way is lost
static inline void bt_at_error(bt_at_t *b)
{
	b->n = 0;
	b->payload = 0;
	b->payload_first = false;
	b->raw = false;
	b->aprs = false;
	b->errors++;
}

static inline bool bt_at_starts(const char *s, const char *p)
{
	return strncmp(s, p, strlen(p)) == 0;
}

// A decimal number at s, -1 when s doesn't start with a digit
static inline int32_t bt_at_number(const char *s)
{
	if (*s < '0' || *s > '9') {
		return -1;
	}
	int32_t v = 0;
	while (*s >= '0' && *s <= '9' && v < 100000) {
		v = v * 10 + (*s++ - '0');
	}
	return v;
}

// A Bluetooth address of 12 hex digits all '0': a link that ended
static inline bool bt_at_no_address(const char *s)
{
	for (unsigned i = 0; i < 12u; i++) {
		if (s[i] != '0') {
			return false;
		}
	}
	return true;
}

// One line of the hand microphone's, from an +ADATA payload
static inline void bt_at_mic_line(bt_at_t *b, const char *s)
{
	if (strcmp(s, "+PTT=P") == 0) {
		b->mic_ptt = true;
	} else if (strcmp(s, "+PTT=R") == 0) {
		b->mic_ptt = false;
	} else if (bt_at_starts(s, "+KEY=") && strlen(s) == 7u) {
		int32_t k = bt_at_number(s + 5);
		if (k >= (int32_t)BT_AT_KEY_MIN && k <= (int32_t)BT_AT_KEY_MAX) {
			b->key = (uint8_t)k;
			b->key_events++;
		}
	}
}

// A whole line from the module
static inline void bt_at_line(bt_at_t *b, const char *s)
{
	b->lines++;
	b->present = true;
	if (bt_at_starts(s, "+VER")) {
		b->ver = true;
		b->aprs_board = bt_at_starts(s + 4, ":APRS") || bt_at_starts(s + 4, "=APRS");
	} else if (strcmp(s, "+PTTKEY=1") == 0) {
		b->fob_ptt = true;
	} else if (strcmp(s, "+PTTKEY=0") == 0) {
		b->fob_ptt = false;
	} else if (strcmp(s, "+HFREQ:1") == 0) {
		b->hf_ptt = !b->hf_ptt;
	} else if (strcmp(s, "+HFREQ:0") == 0) {
		b->hf_ptt = false;
	} else if (bt_at_starts(s, "+ADATA:")) {
		const char *c = strchr(s + 7, ',');
		int32_t len = c != NULL ? bt_at_number(c + 1) : -1;
		if (len > 0 && len < 1000) {
			b->payload = (uint16_t)len;
			b->payload_first = true;
			b->raw = false;
		}
	} else if (strcmp(s, "+READY") == 0) {
		bt_at_release(b);
		b->links = 0;
	} else if (bt_at_starts(s, "+DISC=") ||
		   (bt_at_starts(s, "+CONN=") && strlen(s) >= 18u && bt_at_no_address(s + 6))) {
		bt_at_release(b);
		if (b->links > 0) {
			b->links--;
		}
	} else if (bt_at_starts(s, "+CONN=") && strlen(s) >= 18u) {
		if (b->links < BT_AT_LINKS_MAX) {
			b->links++;
		}
	} else if (bt_at_starts(s, "+APRS:")) {
		b->aprs_board = true;
	} else if (strcmp(s, "OK") == 0) {
		b->oks++;
	} else if (bt_at_starts(s, "ERR")) {
		b->errs++;
	}
}

// One byte of an APRS board frame
static inline void bt_at_aprs_byte(bt_at_t *b, uint8_t c)
{
	if (b->aprs_lead) {
		if (c == 0x7Eu) {
			return;		// the opening flag
		}
		b->aprs_lead = false;
	}
	if (c == 0x7Eu && b->aprs_ui) {
		b->aprs = false;
		b->aprs_frames++;
		if (b->aprs_done != NULL) {
			b->aprs_done(b->aprs_len);
		}
		return;
	}
	if (b->aprs_len >= b->aprs_max) {
		b->aprs = false;	// too long, or no UI header: dropped (the board itself gives up at 400 bytes)
		b->aprs_overflows++;
		b->errors++;
		return;
	}
	b->aprs_buf[b->aprs_len++] = c;
	if (b->aprs_ctl == 0) {
		if (b->aprs_len % 7u == 0 && (c & 1u) != 0) {
			b->aprs_ctl = b->aprs_len;	// the last address's SSID byte has the extension bit
		}
	} else if (b->aprs_len == b->aprs_ctl + 2u) {
		b->aprs_ui = b->aprs_buf[b->aprs_ctl] == 0x03u && c == 0xF0u;
		if (!b->aprs_ui) {
			// Not an APRS frame: the board decoding noise. Waiting for the buffer's end held the parser for minutes
			// once the board stopped part way, and every module reply with it (ki9ng fork, 2026-11-27): dropped now,
			// what follows goes to the line parser
			b->aprs = false;
			b->aprs_overflows++;
			b->errors++;
		}
	}
}

// An APRS board frame that stopped arriving part way (the board's decoder lost the signal): given up, so the next
// bytes are lines again. The caller decides when (no byte for a while); with the UART interrupt held off
static inline void bt_at_aprs_drop(bt_at_t *b)
{
	if (b->aprs) {
		b->aprs = false;
		b->aprs_overflows++;
		b->errors++;
	}
}

// One received byte
static inline void bt_at_byte(bt_at_t *b, uint8_t c)
{
	if (b->payload > 0 && b->n == 0 && b->payload_first) {
		b->payload_first = false;
		b->raw = c != '+';
	}
	if (b->bare) {
		// A KISS frame the module passed on without its +ADATA header (seen on the AT-D578UV PLUS's APRS board, ki9ng
		// fork 2026-11-27): FEND opens it, the next FEND past a byte of it closes it
		b->raw_bytes++;
		if (b->raw_sink != NULL) {
			b->raw_sink(c);
		}
		if (c == 0xC0u && b->bare_body) {
			b->bare = false;
			b->bare_frames++;
		} else if (c != 0xC0u) {
			b->bare_body = true;
		}
		return;
	}
	if (b->raw) {
		b->raw_bytes++;
		if (b->raw_sink != NULL) {
			b->raw_sink(c);
		}
		if (--b->payload == 0) {
			b->raw = false;
		}
		return;
	}
	if (b->aprs) {
		bt_at_aprs_byte(b, c);
		return;
	}
	if (b->n == 0 && c == 0xC0u && b->payload == 0) {
		b->bare = true;
		b->bare_body = false;
		b->raw_bytes++;
		if (b->raw_sink != NULL) {
			b->raw_sink(c);
		}
		return;
	}
	if (b->n == 0 && c != '+' && c != 'O' && c != 'E') {
		if (b->payload > 0) {
			b->payload--;	// a payload's line ends (CR LF) don't start a line
		} else if (c != '\r' && c != '\n') {
			b->errors++;
		}
		return;
	}
	if (c == '\n') {
		bool payload = b->payload > 0;
		if (payload) {
			b->payload = b->payload > b->n + 1u ? (uint16_t)(b->payload - b->n - 1u) : 0u;
		}
		if (b->n > BT_AT_LINE_MAX) {
			b->errors++;
		} else {
			uint16_t n = b->n;
			if (n > 0 && b->line[n - 1u] == '\r') {
				n--;
			}
			b->line[n < BT_AT_LINE_MAX ? n : BT_AT_LINE_MAX - 1u] = '\0';
			if (payload) {
				bt_at_mic_line(b, b->line);
			} else {
				bt_at_line(b, b->line);
			}
		}
		b->n = 0;
		return;
	}
	if (b->n < BT_AT_LINE_MAX) {
		b->line[b->n] = (char)c;
	}
	if (b->n <= BT_AT_LINE_MAX) {
		b->n++;
	}
	if (b->n == 6u && b->aprs_buf != NULL && b->payload == 0 && memcmp(b->line, "+APRS:", 6) == 0) {
		b->n = 0;		// an APRS board frame follows, binary: not a line
		b->lines++;
		b->present = true;
		b->aprs_board = true;
		b->aprs = true;
		b->aprs_lead = true;
		b->aprs_ui = false;
		b->aprs_len = 0;
		b->aprs_ctl = 0;
	}
}

#endif
