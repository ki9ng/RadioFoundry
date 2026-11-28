// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// Copyright (c) 2026 KI9NG
// A Bluetooth module's AT lines (drivers/input/bt_at): presence, the APRS board told apart, the three PTT sources and
// the key events in the data tunnel, a dropped link or a restarted module releasing PTT, and bad bytes and long lines
// dropped
#include <stdio.h>
#include "input/bt_at/bt_at.h"

static int failures;

#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

static void feed(bt_at_t *b, const char *s)
{
	while (*s) {
		bt_at_byte(b, (uint8_t)*s++);
	}
}

// An +ADATA line and its payload, as the module sends them
static void tunnel(bt_at_t *b, const char *payload)
{
	char head[32];
	snprintf(head, sizeof(head), "+ADATA:01,%u\r\n", (unsigned)strlen(payload));
	feed(b, head);
	feed(b, payload);
}

static void test_presence(void)
{
	bt_at_t b;
	bt_at_init(&b);
	CHECK(!b.present);
	feed(&b, "\r\n+VER:ET12_AQQX_V10043\r\n");
	CHECK(b.present && !b.aprs_board && b.lines == 1 && b.errors == 0);
	bt_at_init(&b);
	feed(&b, "ERR\r\n");
	CHECK(b.present);
	bt_at_init(&b);
	feed(&b, "+VER:APRS_V200\r\n");
	CHECK(b.present && b.aprs_board);
}

static void test_ptt_sources(void)
{
	bt_at_t b;
	bt_at_init(&b);
	feed(&b, "+PTTKEY=1\r\n");
	CHECK(b.fob_ptt && bt_at_ptt(&b));
	feed(&b, "+PTTKEY=0\r\n");
	CHECK(!bt_at_ptt(&b));
	// the hands-free button toggles; 0 clears
	feed(&b, "+HFREQ:1\r\n");
	CHECK(b.hf_ptt);
	feed(&b, "+HFREQ:1\r\n");
	CHECK(!b.hf_ptt);
	feed(&b, "+HFREQ:1\r\n+HFREQ:0\r\n");
	CHECK(!b.hf_ptt);
	// the hand microphone's lines come in the tunnel; the same text outside it does nothing
	feed(&b, "+PTT=P\r\n");
	CHECK(!b.mic_ptt);
	tunnel(&b, "+PTT=P\r\n");
	CHECK(b.mic_ptt);
	tunnel(&b, "+PTT=R\r\n");
	CHECK(!b.mic_ptt);
	// a line after the payload is the module's again
	tunnel(&b, "+PTT=P\r\n");
	feed(&b, "+PTTKEY=1\r\n");
	CHECK(b.mic_ptt && b.fob_ptt && b.payload == 0);
	CHECK(b.errors == 0);
}

static void test_keys(void)
{
	bt_at_t b;
	bt_at_init(&b);
	tunnel(&b, "+KEY=03\r\n");
	CHECK(b.key == 3 && b.key_events == 1);
	tunnel(&b, "+KEY=01\r\n+KEY=02\r\n");
	CHECK(b.key == 2 && b.key_events == 3);
	// codes out of range, and malformed ones
	tunnel(&b, "+KEY=05\r\n+KEY=1\r\n+KEY=0x\r\n");
	CHECK(b.key == 2 && b.key_events == 3);
}

static void test_release(void)
{
	bt_at_t b;
	const char *ends[] = { "+DISC=0011223344AA,PTT-1\r\n", "+READY\r\n", "+CONN=000000000000,\r\n" };
	for (unsigned i = 0; i < 3; i++) {
		bt_at_init(&b);
		feed(&b, "+PTTKEY=1\r\n+HFREQ:1\r\n");
		tunnel(&b, "+PTT=P\r\n");
		CHECK(bt_at_ptt(&b));
		feed(&b, ends[i]);
		CHECK(!bt_at_ptt(&b));
	}
	// a connection to a real address doesn't
	bt_at_init(&b);
	feed(&b, "+PTTKEY=1\r\n+CONN=0011223344AA,PTT-1\r\n");
	CHECK(bt_at_ptt(&b));
}

// Links: counted up by +CONN with an address, down by +DISC or the all-zero address, cleared by +READY; never below 0
// and never past BT_AT_LINKS_MAX
static void test_links(void)
{
	bt_at_t b;
	bt_at_init(&b);
	feed(&b, "+CONN:000000000000,HOSTDEVICE\r\n");	// a query's answer, not an event
	CHECK(b.links == 0);
	feed(&b, "+CONN=0011223344AA,PTT-1\r\n+CONN=0011223344BB,BT01\r\n");
	CHECK(b.links == 2);
	feed(&b, "+DISC=0011223344AA,PTT-1\r\n");
	CHECK(b.links == 1);
	feed(&b, "+CONN=000000000000,\r\n");
	CHECK(b.links == 0);
	feed(&b, "+DISC=0011223344AA,PTT-1\r\n");
	CHECK(b.links == 0);
	for (unsigned i = 0; i < BT_AT_LINKS_MAX + 3u; i++) {
		feed(&b, "+CONN=0011223344AA,PTT-1\r\n");
	}
	CHECK(b.links == BT_AT_LINKS_MAX);
	feed(&b, "+READY\r\n");
	CHECK(b.links == 0);
}

static void test_errors(void)
{
	bt_at_t b;
	bt_at_init(&b);
	// bytes that start no line
	feed(&b, "xyz");
	CHECK(b.errors == 3 && b.lines == 0);
	// a line too long is dropped whole, and the next one is read
	char lng[BT_AT_LINE_MAX + 20];
	memset(lng, 'A', sizeof(lng));
	lng[0] = '+';
	lng[sizeof(lng) - 1] = '\0';
	feed(&b, lng);
	feed(&b, "\r\n+PTTKEY=1\r\n");
	CHECK(b.errors == 4 && b.fob_ptt && b.lines == 1);
	// a UART error ends the line under way
	feed(&b, "+PTTK");
	bt_at_error(&b);
	feed(&b, "EY=0\r\n");
	CHECK(b.fob_ptt);
	feed(&b, "+PTTKEY=0\r\n");
	CHECK(!b.fob_ptt);
}

// Raw payloads (CONFIG_BT_KISS): what a phone writes to the SPP server comes through the +ADATA tunnel as bytes
static uint8_t sunk[512];
static size_t nsunk;

static void sink(uint8_t c)
{
	if (nsunk < sizeof(sunk)) {
		sunk[nsunk] = c;
	}
	nsunk++;
}

static void tunnel_bin(bt_at_t *b, const uint8_t *d, size_t n)
{
	char head[32];
	snprintf(head, sizeof(head), "+ADATA:01,%03u\r\n", (unsigned)n);
	feed(b, head);
	for (size_t i = 0; i < n; i++) {
		bt_at_byte(b, d[i]);
	}
	feed(b, "\r\n");	// the module ends each report with CR LF after the payload
}

static void test_raw_payload(void)
{
	bt_at_t b;
	bt_at_init(&b);
	b.raw_sink = sink;
	nsunk = 0;
	// a KISS frame with bytes that would start or end a line, and a zero byte
	static const uint8_t kiss[] = { 0xC0, 0x00, '+', 'O', 'K', '\r', '\n', 'E', 0x00, 0xDB, 0xDC, 0x7E, 0xC0 };
	tunnel_bin(&b, kiss, sizeof(kiss));
	CHECK(nsunk == sizeof(kiss) && memcmp(sunk, kiss, sizeof(kiss)) == 0);
	CHECK(b.raw_bytes == sizeof(kiss) && !b.raw && b.payload == 0);
	CHECK(b.errors == 0 && b.oks == 0 && b.lines == 1);	// only the +ADATA line itself
	// lines after it parse as before
	feed(&b, "+PTTKEY=1\r\n");
	CHECK(b.fob_ptt);
	feed(&b, "+PTTKEY=0\r\n");
	// a frame split across two reports arrives whole, in order
	nsunk = 0;
	static const uint8_t a[] = { 0xC0, 0x00, 1, 2, 3 }, c[] = { 4, 5, 0xC0 };
	tunnel_bin(&b, a, sizeof(a));
	tunnel_bin(&b, c, sizeof(c));
	CHECK(nsunk == 8 && sunk[0] == 0xC0 && sunk[4] == 3 && sunk[5] == 4 && sunk[7] == 0xC0);
	CHECK(b.errors == 0);
	// the hand mic's lines in the tunnel still parse, and reach no sink
	nsunk = 0;
	tunnel(&b, "+PTT=P\r\n");
	CHECK(b.mic_ptt && nsunk == 0);
	tunnel(&b, "+PTT=R\r\n");
	CHECK(!b.mic_ptt);
	// without a sink, raw bytes are dropped quietly
	b.raw_sink = NULL;
	tunnel_bin(&b, kiss, sizeof(kiss));
	CHECK(nsunk == 0 && b.errors == 0 && !b.raw);
	// a UART error mid-payload ends it; the next line parses
	b.raw_sink = sink;
	feed(&b, "+ADATA:01,010\r\n");
	bt_at_byte(&b, 0xC0);
	bt_at_byte(&b, 0x00);
	bt_at_error(&b);
	CHECK(!b.raw && b.payload == 0);
	feed(&b, "+PTTKEY=1\r\n");
	CHECK(b.fob_ptt);
}

static void test_replies(void)
{
	bt_at_t b;
	bt_at_init(&b);
	feed(&b, "OK\r\nOK\r\nERR\r\nERROR\r\n");
	CHECK(b.oks == 2 && b.errs == 2 && b.present);
}

// APRS add-on board frames (the AT-D578UV PLUS): "+APRS:" '~' frame+FCS '~', binary, no line end
static uint8_t abuf[64];
static unsigned adone, alen;
static void aprs_done(uint16_t len)
{
	adone++;
	alen = len;
}

static void feed_bin(bt_at_t *b, const uint8_t *p, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		bt_at_byte(b, p[i]);
	}
}

static void test_aprs_board(void)
{
	// KI9NG-15>APZ578:>D578 RX test 1 03:36:04, FCS 0x973a: the bytes the board sent on the bench (2026-11-27)
	static const uint8_t fr[] = { 0x82, 0xa0, 0xb4, 0x6a, 0x6e, 0x70, 0x60, 0x96, 0x92, 0x72, 0x9c, 0x8e, 0x40, 0x7f,
		0x03, 0xf0, '>', 'D', '5', '7', '8', ' ', 'R', 'X', ' ', 't', 'e', 's', 't', ' ', '1', ' ', '0', '3', ':',
		'3', '6', ':', '0', '4', 0x3a, 0x97 };
	bt_at_t b;
	bt_at_init(&b);
	b.aprs_buf = abuf;
	b.aprs_max = sizeof(abuf);
	b.aprs_done = aprs_done;
	adone = 0;
	feed(&b, "OK\r\n+APRS:~");
	feed_bin(&b, fr, sizeof(fr));
	feed(&b, "~OK\r\n");
	CHECK(adone == 1 && alen == sizeof(fr) && memcmp(abuf, fr, sizeof(fr)) == 0);
	CHECK(b.oks == 2 && b.errors == 0 && b.aprs_frames == 1 && !b.aprs && b.aprs_board);
	// a '~' in the address field (SSID 15, not the last address) and line bytes in the info are data
	static const uint8_t f2[] = { 0x82, 0xa0, 0xb4, 0x6a, 0x6e, 0x70, 0x7e, 0x96, 0x92, 0x72, 0x9c, 0x8e, 0x40, 0x7f,
		0x03, 0xf0, '+', '\r', '\n', 'O', 'K', 0x00, 0x11, 0x22 };
	adone = 0;
	feed(&b, "+APRS:~");
	feed_bin(&b, f2, sizeof(f2));
	feed(&b, "~");
	CHECK(adone == 1 && alen == sizeof(f2) && memcmp(abuf, f2, sizeof(f2)) == 0);
	feed(&b, "OK\r\n");
	CHECK(b.oks == 3 && b.errors == 0);
	// too long for the buffer: dropped, counted, nothing handed on
	adone = 0;
	b.aprs_max = 20;
	feed(&b, "+APRS:~");
	feed_bin(&b, fr, sizeof(fr));
	CHECK(adone == 0 && b.aprs_overflows == 1 && !b.aprs);
	b.aprs_max = sizeof(abuf);
	feed(&b, "~\r\nOK\r\n");
	CHECK(b.oks == 4);
	// not a UI frame (control 0x13, the board decoding noise): dropped as soon as the header shows it, and the module's
	// replies after it parse (it used to hold the parser to the buffer's end, OKs and all: 2026-11-27)
	uint8_t f3[sizeof(fr)];
	memcpy(f3, fr, sizeof(fr));
	f3[14] = 0x13;
	adone = 0;
	feed(&b, "+APRS:~");
	feed_bin(&b, f3, 16);
	CHECK(adone == 0 && !b.aprs && b.aprs_overflows == 2);
	feed(&b, "\r\nOK\r\n");
	CHECK(b.oks == 5);
	// the bench case: noise whose 7th byte ends the address field, then nothing more from the board
	static const uint8_t junk[] = { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0x01, 0x55, 0xaa };
	feed(&b, "+APRS:~");
	feed_bin(&b, junk, sizeof(junk));
	CHECK(!b.aprs && b.aprs_overflows == 3);
	feed(&b, "OK\r\n");
	CHECK(b.oks == 6);
	// cut off before its header: held until the caller gives it up (no byte for a while), then lines again
	feed(&b, "+APRS:~");
	feed_bin(&b, fr, 5);
	CHECK(b.aprs && b.oks == 6);
	bt_at_aprs_drop(&b);
	CHECK(!b.aprs && b.aprs_overflows == 4);
	bt_at_aprs_drop(&b);
	CHECK(b.aprs_overflows == 4);
	feed(&b, "OK\r\n");
	CHECK(b.oks == 7);
	// a UART error mid-frame ends it; the next line parses
	feed(&b, "\r\n+APRS:~");
	feed_bin(&b, fr, 10);
	bt_at_error(&b);
	CHECK(!b.aprs);
	feed(&b, "+PTTKEY=1\r\n");
	CHECK(b.fob_ptt);
	// without a buffer the frame isn't taken (as before)
	bt_at_t n;
	bt_at_init(&n);
	adone = 0;
	feed(&n, "+APRS:~");
	feed_bin(&n, fr, sizeof(fr));
	feed(&n, "~");
	CHECK(adone == 0 && !n.aprs && n.aprs_frames == 0);
}

int main(void)
{
	test_presence();
	test_ptt_sources();
	test_keys();
	test_release();
	test_links();
	test_errors();
	test_raw_payload();
	test_replies();
	test_aprs_board();
	// A KISS frame without its +ADATA header (the PLUS's APRS board): FEND to FEND to the raw sink, not errors
	{
		static uint8_t got[16];
		static unsigned ng;
		bt_at_t k;
		bt_at_init(&k);
		k.raw_sink = NULL;
		const uint8_t fr[] = { 0xC0, 0x00, 0x41, 0x42, 0xC0 };
		uint32_t e0 = k.errors;
		for (unsigned j = 0; j < sizeof(fr); j++) {
			bt_at_byte(&k, fr[j]);
		}
		CHECK(k.errors == e0 && k.bare_frames == 1u && !k.bare && k.raw_bytes == sizeof(fr));
		const char *ln = "+READY\r\n";
		for (; *ln; ln++) {
			bt_at_byte(&k, (uint8_t)*ln);
		}
		CHECK(k.lines == 1u);
		(void)got; (void)ng;
	}

	if (failures) {
		printf("# btAtTest FAILED (%d)\n", failures);
		return 1;
	}
	printf("# btAtTest PASSED\n");
	return 0;
}
