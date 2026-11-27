// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// Copyright (c) 2026 KI9NG
// AT-D578UV Bluetooth PTT and keys: the module on UART4 (PC10 TX, PC11 RX), 115200
// 8N1, its AT lines read by drivers/input/bt_at. The module's supply is the shift register's bit 0x02, on from boot
// (radio_d578uv.c). Not every model has the module and no pin tells: the radio asks BT_AT_PROBE a few times at start
// and takes any answer as the module. The APRS add-on board answers on the same UART and is told apart by its answer;
// it isn't used. Pairing, audio and the module's settings aren't written: the module keeps the pairings made before
// (inferred)
#include "config.h"
#include "d578uv.h"
#include "pins.h"
#include "mcu_io.h"
#include "hal_time.h"
#include "input/bt_at/bt_at.h"
#include "debuglink/dlog.h"

#if CONFIG_BT_PTT

#define BT_BAUD			115200u
#define BT_IRQ_PRIORITY		6
#define BT_PROBES		20u	// BT_PROBE_MS apart, from BT_PROBE_FIRST_MS
#define BT_PROBE_MS		300u
// The vendor's start (ki9ng fork, AT-D578UV PLUS): the supply on, 500 ms, then EE 01 06 three times 50 ms apart and
// 200 ms more before the first AT+VER? (its probe of the APRS board, which on the PLUS sits between the radio and the
// module). Without it the module never answered here (input.bt_lines 0 on every start). What EE 01 06 does isn't known
#define BT_WAKE_MS		500u
#define BT_WAKE_GAP_MS		50u
#define BT_WAKES		3u
#define BT_PROBE_FIRST_MS	1300u	// after the module's +READY (1.14 s from power-on on the bench)
#define BT_REPORT_MS		8000u	// what came back, as text (and again at BT_REPORT2_MS)
#define BT_REPORT2_MS		20000u
#define BT_WAKE2_MS		3000u	// the wake again: a reply to it comes back right after (bench diagnostic)
#define BT_BTON_MS		5000u	// AT+BTON once: 100 ms after the +VER answer, or by this time without one
static const uint8_t bt_wake[] = { 0xEE, 0x01, 0x06 };
#define BT_HEAD			128u
static volatile uint8_t rx_head[BT_HEAD];
static volatile uint32_t rx_bytes, rx_errs;
#define BT_TAIL			96u	// the last bytes from the module, logged when errors jump (BT_TAIL_ERRS within a scan)
#define BT_TAIL_ERRS		8u
#define BT_TAIL_GAP_MS		5000u
static volatile uint8_t rx_tail[BT_TAIL];
static volatile uint32_t rx_tail_n;
static uint32_t tail_errs_seen, tail_ms;
#define BT_POWER_OFF_MS		400u	// the supply off this long at the start, so the timers below run from a power-on
static volatile bool power_want = true;
static bool cycled;
static uint8_t wakes, wakes2;
static bool bton;
static uint8_t reported;
static uint32_t ver_ms;
#define BT_KEY_MS		150u	// a key event holds its key bit this long, past the core's debounce

static bt_at_t bt;
static uint32_t start_ms, key_ms, key_seen, offs;
static uint8_t probes, key_bit_held = 0xFF;

#if D578UV_BT_APRS
#include <string.h>
#include "aprs/ax25.h"

// Frames from the PLUS's APRS add-on board (ki9ng fork). Its CMX865A decodes side B's receive audio (bench,
// 2026-11-27: side A's packets never came out of it); it sends "+APRS:~<frame + FCS>~" on this UART. bt_at assembles
// the frame into brd_rx in the UART interrupt, brd_done moves it to the ready slot, and the radio task takes it
// (d578uv_bt_aprs_frame, the core's hal_afsk_t.rx_frame), checks the FCS and gives it to the APRS engine straight from
// the slot, which frees at the next call. One frame waits; another arriving meanwhile is dropped (counted). A '~' in the FCS or info ends the board's frame early (it
// doesn't escape them): those fail the check
#define BRD_FRAME_MAX		(AX25_FRAME_MAX + 2u)
#define BRD_FRAME_MIN		(AX25_ADDR_BYTES * 2u + 2u + 2u)	// two addresses, control, PID and the FCS
static uint8_t brd_rx[BRD_FRAME_MAX], brd_ready[BRD_FRAME_MAX];
static volatile uint16_t brd_ready_len;	// 0: the slot is free
static bool brd_lent;			// the radio task has the slot's frame (until its next call)
static uint32_t brd_ok, brd_bad, brd_busy;

// UART interrupt: a whole frame in brd_rx
static void brd_done(uint16_t len)
{
	if (brd_ready_len != 0) {
		brd_busy++;
		return;
	}
	memcpy(brd_ready, brd_rx, len);
	brd_ready_len = len;
}

// Radio task: the waiting frame with a good FCS, without it (in the slot); 0 when there is none (a bad one is dropped).
// The frame lent last time is done with by now: its slot frees first
uint16_t d578uv_bt_aprs_frame(const uint8_t **frame)
{
	if (brd_lent) {
		brd_lent = false;
		brd_ready_len = 0;	// the interrupt fills the slot only while it is free
	}
	uint16_t n = brd_ready_len;
	if (n == 0) {
		return 0;
	}
	if (n >= BRD_FRAME_MIN &&
	    ax25_fcs(brd_ready, n - 2u) == (uint16_t)(brd_ready[n - 2u] | (uint16_t)brd_ready[n - 1u] << 8)) {
		brd_ok++;
		brd_lent = true;
		*frame = brd_ready;
		return (uint16_t)(n - 2u);
	}
	brd_bad++;
	brd_ready_len = 0;
	return 0;
}
#endif

#if CONFIG_BT_KISS
#include "aprs/aprs.h"
#include "aprs/aprs_host.h"
#include "aprs/kiss.h"
#include <string.h>

// KISS over the module's SPP server (RFCOMM channel 2), for APRSdroid (ki9ng fork). Wire format, from the vendor
// firmware's send path (wiki radio/anytone-d578/d578-kiss-tnc): the radio writes "AT+ADATA=01,LLL" CR LF, LLL bytes
// (1-200), CR LF, and the module answers OK or ERR. What the phone writes arrives as "+ADATA:01,LLL" CR LF, the
// bytes, CR LF: drivers/input/bt_at hands those to kiss_sink in the UART interrupt. The input task decodes them into
// the APRS engine (its station identification and rate rules apply) and, while a phone is there, sends the engine's
// KISS output to it instead of the USB port (hostlink.c asks d578uv_bt_kiss_taken).
#define KISS_RX_RING		128u	// a power of two: raw bytes from the UART interrupt to the input task
#define KISS_RX_PER_SCAN	64u	// bytes decoded per key scan, so the scan stays short
#define KISS_CHUNK		200u	// the module's largest AT+ADATA payload
#define KISS_GAP_MS		30u	// between sends (the vendor firmware waits 30 of its ticks)
#define KISS_REPLY_MS		500u	// a send with no OK or ERR by then is given up on
#define KISS_HOLD_MS		600000u	// a phone that sent KISS keeps the output this long, link count or not

static uint8_t krx[KISS_RX_RING];
static volatile uint8_t krx_head, krx_tail;
static kiss_dec_t kdec;
static uint8_t ktx[KISS_CHUNK];
static uint32_t ktx_ms, ktx_replies;
static volatile uint32_t krx_ms;	// when the phone last sent a byte (valid once krx_seen)
static volatile bool krx_seen;
static bool ktx_wait;
static uint32_t k_in, k_drop, k_out, k_out_bytes, k_timeouts;

#if D578UV_BT_GPS
// GPS out (ki9ng fork): a host that sends SetHardware 'P' 1 gets the board GPS's RMC and GGA sentences as the module
// sent them (one fix a second, PCAS02 in gps_d578uv.c), each in a SetHardware 'N' frame, while it holds the output.
// 'P' 0, or the output going back to USB, stops it, so a host that never asks (APRSdroid) never sees one. The GPS task
// writes the newest sentence of each kind, the input task takes them: interrupts off for the copies (under 200 bytes)
#define KISS_TAG_GPS_OUT	'P'
#define GPS_OUT_LINE		88u	// NMEA_LINE_MAX; a longer sentence is skipped
_Static_assert(2u * (GPS_OUT_LINE + 4u) <= KISS_CHUNK, "both GPS frames must fit one AT+ADATA send");
static char gps_line[2][GPS_OUT_LINE];	// RMC, GGA
static volatile uint8_t gps_len[2];	// 0: nothing new
static volatile bool gps_want;
static uint32_t k_gps_out;

void d578uv_bt_gps_sentence(const char *s, uint8_t len, uint32_t now_ms)
{
	(void)now_ms;
	if (!gps_want || len < 7u || len > GPS_OUT_LINE || s[0] != '$') {
		return;
	}
	uint8_t k;
	if (memcmp(s + 3, "RMC,", 4) == 0) {
		k = 0;
	} else if (memcmp(s + 3, "GGA,", 4) == 0) {
		k = 1;
	} else {
		return;
	}
	for (uint8_t i = 0; i < len; i++) {
		uint8_t c = (uint8_t)s[i];
		if (c < 0x20u || c > 0x7Eu) {
			return;		// printable only: no FEND or FESC inside the frame
		}
	}
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	memcpy(gps_line[k], s, len);
	gps_len[k] = len;
	__set_PRIMASK(pm);
}

// The new sentences as 'N' frames into out (room for 2 * (GPS_OUT_LINE + 4)); 0 when there are none
static size_t gps_take(uint8_t *out)
{
	size_t n = 0;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	for (unsigned k = 0; k < 2u; k++) {
		uint8_t m = gps_len[k];
		if (m != 0) {
			out[n++] = KISS_FEND;
			out[n++] = KISS_CMD_SETHW;
			out[n++] = KISS_TAG_NMEA;
			memcpy(out + n, gps_line[k], m);
			n += m;
			out[n++] = KISS_FEND;
			gps_len[k] = 0;
			k_gps_out++;
		}
	}
	__set_PRIMASK(pm);
	return n;
}
#endif

// UART interrupt: one raw payload byte
static void kiss_sink(uint8_t c)
{
	uint8_t h = krx_head, nh = (uint8_t)((h + 1u) & (KISS_RX_RING - 1u));
	if (nh == krx_tail) {
		k_drop++;
		return;
	}
	krx[h] = c;
	krx_head = nh;
}

// A phone takes the APRS output: the module answered, a link is up (or the phone sent KISS lately) and the APRS
// KISS setting is on. Called from the input task and from the host link task: single word reads only
bool d578uv_bt_kiss_taken(void)
{
	// krx_ms is the scan's start time, which can be a millisecond behind hal_millis() but never ahead of it
	uint32_t seen = krx_ms;
	bool phone = bt.links != 0 || (krx_seen && hal_millis() - seen < KISS_HOLD_MS);
	return bt.present && phone && aprs_host_enabled();
}

static void kiss_put_str(const char *s)
{
	while (*s) {
		mcu_uart_put(MCU_UART4, (uint8_t)*s++);
	}
}

// Input task, each key scan: what the phone sent to the APRS engine, then at most one send to the phone
static void kiss_pump(uint32_t now)
{
	for (unsigned i = 0; i < KISS_RX_PER_SCAN && krx_tail != krx_head; i++) {
		uint8_t c = krx[krx_tail];
		krx_tail = (uint8_t)((krx_tail + 1u) & (KISS_RX_RING - 1u));
		krx_ms = now;	// not now | 1: one ahead of hal_millis() read as 49 days old, and the output flickered off
		krx_seen = true;
		size_t m = kiss_dec_byte(&kdec, c);
		// Port 0 data frames only, straight to the engine: the host port's other frames (SetHardware NMEA and
		// APRS-IS lines) aren't a phone's, and the NMEA parser would take the input task past its stack
		if (m > 1u && kdec.buf[0] == KISS_CMD_DATA) {
			k_in++;
			aprs_host_frame(kdec.buf + 1, m - 1u, now);
		}
#if D578UV_BT_GPS
		if (m == 3u && kdec.buf[0] == KISS_CMD_SETHW && kdec.buf[1] == KISS_TAG_GPS_OUT) {
			gps_want = kdec.buf[2] != 0;
		}
#endif
	}
	if (!d578uv_bt_kiss_taken()) {
		ktx_wait = false;
#if D578UV_BT_GPS
		gps_want = false;	// a new host asks again
		gps_len[0] = 0;
		gps_len[1] = 0;
#endif
		return;
	}
	uint32_t replies = bt.oks + bt.errs;
	if (ktx_wait) {
		if (replies != ktx_replies) {
			ktx_wait = false;
		} else if (now - ktx_ms >= KISS_REPLY_MS) {
			ktx_wait = false;
			k_timeouts++;
		} else {
			return;
		}
	}
	if (now - ktx_ms < KISS_GAP_MS) {
		return;
	}
	size_t n = 0;
#if D578UV_BT_GPS
	n = gps_take(ktx);	// at most twice a second: the APRS output keeps the other sends
#endif
	if (n == 0) {
		n = aprs_host_output(ktx, sizeof(ktx));
	}
	if (n == 0) {
		return;
	}
	char len[6] = { (char)('0' + n / 100u), (char)('0' + n / 10u % 10u), (char)('0' + n % 10u), '\r', '\n', 0 };
	ktx_replies = replies;
	kiss_put_str("AT+ADATA=01,");
	kiss_put_str(len);
	for (size_t i = 0; i < n; i++) {
		mcu_uart_put(MCU_UART4, ktx[i]);
	}
	kiss_put_str("\r\n");
	ktx_ms = now;
	ktx_wait = true;
	k_out++;
	k_out_bytes += n;
}
#endif

void MCU_UART4_HANDLER(void)
{
	int32_t c = mcu_uart_take(MCU_UART4);
	if (c == MCU_UART_ERROR) {
		rx_errs++;
		bt_at_error(&bt);
	} else if (c != MCU_UART_NONE) {
		if (rx_bytes < BT_HEAD) {
			rx_head[rx_bytes] = (uint8_t)c;
		}
		rx_tail[rx_tail_n % BT_TAIL] = (uint8_t)c;
		rx_tail_n++;
		rx_bytes++;
		bt_at_byte(&bt, (uint8_t)c);
	}
}

void d578uv_bt_init(void)
{
	bt_at_init(&bt);
#if D578UV_BT_APRS
	bt.aprs_buf = brd_rx;
	bt.aprs_max = (uint16_t)sizeof(brd_rx);
	bt.aprs_done = brd_done;
#endif
#if CONFIG_BT_KISS
	bt.raw_sink = kiss_sink;
	kiss_dec_init(&kdec);
#endif
	mcu_pin_uart_rx(PIN_BT_RX, BT_TX_AF, MCU_PULL_UP);	// UART4 RX is AF8 on PC11 too (AT32); GD32: plain input
	mcu_pin_af(PIN_BT_TX, BT_TX_AF, false);
	mcu_uart_start(MCU_UART4, BT_BAUD, true);
	NVIC_SetPriority(MCU_UART4_IRQn, BT_IRQ_PRIORITY);
	NVIC_EnableIRQ(MCU_UART4_IRQn);
	start_ms = hal_millis();
}

// The module's key events as the radio's keys: 01 and 02 as the microphone's Up and Down (inferred), 03 and 04
// (Monitor, inferred) have no key here
static int8_t key_bit(uint8_t code)
{
	switch (code) {
	case 1:
		return KEY_MIC_UP;
	case 2:
		return KEY_MIC_DOWN;
	default:
		return -1;
	}
}

// From the key scan: the probe while nothing has answered, then PTT and the key bits
uint32_t d578uv_bt_scan(void)
{
	uint32_t now = hal_millis();
	NVIC_DisableIRQ(MCU_UART4_IRQn);
	bool present = bt.present && !bt.aprs_board;
	bool answered = bt.present;
	bool ptt = bt_at_ptt(&bt);
	uint32_t events = bt.key_events;
	uint8_t key = bt.key;
	NVIC_EnableIRQ(MCU_UART4_IRQn);
	if (!cycled) {
		if (power_want) {
			power_want = false;	// the radio task applies it within a status poll
			start_ms = now;
			return 0;
		}
		if (now - start_ms < BT_POWER_OFF_MS) {
			return 0;
		}
		power_want = true;
		cycled = true;
		NVIC_DisableIRQ(MCU_UART4_IRQn);
		rx_bytes = 0;
		rx_errs = 0;
		NVIC_EnableIRQ(MCU_UART4_IRQn);
		start_ms = now;		// from here: the power-on
		dlog("bt: supply cycled, on again");
	}
	if (wakes < BT_WAKES && now - start_ms >= BT_WAKE_MS + (uint32_t)wakes * BT_WAKE_GAP_MS) {
		for (size_t i = 0; i < sizeof(bt_wake); i++) {
			mcu_uart_put(MCU_UART4, bt_wake[i]);
		}
		wakes++;
	}
	if (wakes2 < BT_WAKES && now - start_ms >= BT_WAKE2_MS + (uint32_t)wakes2 * BT_WAKE_GAP_MS) {
		for (size_t i = 0; i < sizeof(bt_wake); i++) {
			mcu_uart_put(MCU_UART4, bt_wake[i]);
		}
		wakes2++;
	}
	if (bt.ver && ver_ms == 0) {
		ver_ms = now;
	}
	if (!bton && ((ver_ms != 0 && now - ver_ms >= 100u) || now - start_ms >= BT_BTON_MS)) {
		for (const char *t = "AT+BTON\r\n"; *t; t++) {
			mcu_uart_put(MCU_UART4, (uint8_t)*t);
		}
		bton = true;
	}
	if (bt.errors >= tail_errs_seen + BT_TAIL_ERRS && (tail_ms == 0 || now - tail_ms >= BT_TAIL_GAP_MS)) {
		tail_errs_seen = bt.errors;
		tail_ms = now | 1u;
		uint32_t end = rx_tail_n, n = end < BT_TAIL ? end : BT_TAIL;
		dlog("bt: errors %lu, last %lu bytes (bare kiss %lu):", (unsigned long)bt.errors, (unsigned long)n,
			(unsigned long)bt.bare_frames);
		static const char hx[] = "0123456789abcdef";
		for (uint32_t i = 0; i < n; i += 24u) {
			char line[49];
			uint32_t m = n - i < 24u ? n - i : 24u;
			for (uint32_t k = 0; k < m; k++) {
				uint8_t c = rx_tail[(end - n + i + k) % BT_TAIL];
				line[2 * k] = hx[c >> 4];
				line[2 * k + 1] = hx[c & 15u];
			}
			line[2 * m] = 0;
			dlog("bt: %s", line);
		}
	} else if (bt.errors < tail_errs_seen) {
		tail_errs_seen = bt.errors;
	}
	if (reported < 2u && now - start_ms >= (reported == 0u ? BT_REPORT_MS : BT_REPORT2_MS)) {
		reported++;
		uint32_t n = rx_bytes < BT_HEAD ? rx_bytes : BT_HEAD;
		dlog("bt: %lu bytes, %lu errors, %u probes, lines %lu, ver %u, links %u", (unsigned long)rx_bytes,
			(unsigned long)rx_errs, probes, (unsigned long)bt.lines, bt.ver, bt.links);
		for (uint32_t i = 0; i < n; i += 32u) {
			char txt[33];
			uint32_t m = n - i < 32u ? n - i : 32u;
			for (uint32_t k = 0; k < m; k++) {
				uint8_t c = rx_head[i + k];
				txt[k] = c == '\r' ? '<' : c == '\n' ? '|' : (c >= 0x20 && c < 0x7F) ? (char)c : '.';
			}
			txt[m] = 0;
			dlog("bt: %3lu %s", (unsigned long)i, txt);
		}
	}
	if (!bt.ver && probes < BT_PROBES && now - start_ms >= BT_PROBE_FIRST_MS + (uint32_t)probes * BT_PROBE_MS) {
		for (const char *s = BT_AT_PROBE; *s; s++) {
			mcu_uart_put(MCU_UART4, (uint8_t)*s);
		}
		probes++;
	}
#if CONFIG_BT_KISS
	// Not gated on the APRS add-on board: on the AT-D578UV PLUS it sits between the radio and the module and passes
	// the +ADATA tunnel through (our KISS patch to the vendor firmware reached APRSdroid that way, 2026-11-25)
	if (answered) {
		kiss_pump(now);
	}
#endif
	if (!present) {
		return 0;
	}
	if (events != key_seen) {
		key_seen = events;
		key_ms = now;
		int8_t b = key_bit(key);
		key_bit_held = b >= 0 ? (uint8_t)b : 0xFF;
	}
	uint32_t m = ptt ? 1u << KEY_PTT : 0u;
	if (key_bit_held != 0xFF && now - key_ms < BT_KEY_MS) {
		m |= 1u << key_bit_held;
	}
	return m;
}

void d578uv_bt_stats(hal_bt_stats_t *st)
{
	st->present = bt.present && !bt.aprs_board;
	st->ptt = bt_at_ptt(&bt);
	st->key = bt.key;
	st->keys = bt.key_events;
	st->lines = bt.lines;
	st->errors = bt.errors;
	st->links = bt.links;
	st->offs = offs;
	st->aprs_board = bt.aprs_board;
	st->oks = bt.oks;
	st->errs = bt.errs;
#if D578UV_BT_APRS
	st->aprs_frames = bt.aprs_frames;
	st->aprs_long = bt.aprs_overflows;
	st->aprs_ok = brd_ok;
	st->aprs_bad = brd_bad;
	st->aprs_busy = brd_busy;
#endif
#if CONFIG_BT_KISS
	st->kiss_taken = d578uv_bt_kiss_taken();
	st->kiss_raw = bt.raw_bytes;
	st->kiss_in = k_in;
	st->kiss_drop = k_drop;
	st->kiss_out = k_out;
	st->kiss_out_bytes = k_out_bytes;
	st->kiss_timeouts = k_timeouts;
#if D578UV_BT_GPS
	st->kiss_gps_want = gps_want;
	st->kiss_gps_out = k_gps_out;
#endif
#endif
}

// The radio powers the module off for a P25 or NXDN transmission (radio_d578uv.c, tx) only while it serves no link:
// with one up, the key fob or hand mic holding PTT would drop and end the transmission. A single byte read, no lock
bool d578uv_bt_power_want(void)
{
	return power_want;
}

bool d578uv_bt_linked(void)
{
	return bt.links != 0;
}

void d578uv_bt_off(void)
{
	offs++;
}

#endif
