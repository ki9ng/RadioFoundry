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

#if CONFIG_BT_PTT

#define BT_BAUD			115200u
#define BT_IRQ_PRIORITY		6
#define BT_PROBES		5u	// 200 ms apart
#define BT_PROBE_MS		200u
#define BT_KEY_MS		150u	// a key event holds its key bit this long, past the core's debounce

static bt_at_t bt;
static uint32_t start_ms, key_ms, key_seen, offs;
static uint8_t probes, key_bit_held = 0xFF;

#if CONFIG_BT_KISS
#include "aprs/aprs.h"
#include "aprs/aprs_host.h"
#include "aprs/kiss.h"

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
static uint32_t ktx_ms, ktx_replies, krx_ms;
static bool ktx_wait;
static uint32_t k_in, k_drop, k_out, k_out_bytes, k_timeouts;

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
	uint32_t seen = krx_ms;
	bool phone = bt.links != 0 || (seen != 0 && hal_millis() - seen < KISS_HOLD_MS);
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
		krx_ms = now | 1u;
		size_t m = kiss_dec_byte(&kdec, c);
		// Port 0 data frames only, straight to the engine: the host port's other frames (SetHardware NMEA and
		// APRS-IS lines) aren't a phone's, and the NMEA parser would take the input task past its stack
		if (m > 1u && kdec.buf[0] == KISS_CMD_DATA) {
			k_in++;
			aprs_host_frame(kdec.buf + 1, m - 1u, now);
		}
	}
	if (!d578uv_bt_kiss_taken()) {
		ktx_wait = false;
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
	size_t n = aprs_host_output(ktx, sizeof(ktx));
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
		bt_at_error(&bt);
	} else if (c != MCU_UART_NONE) {
		bt_at_byte(&bt, (uint8_t)c);
	}
}

void d578uv_bt_init(void)
{
	bt_at_init(&bt);
#if CONFIG_BT_KISS
	bt.raw_sink = kiss_sink;
	kiss_dec_init(&kdec);
#endif
	mcu_pin_in(PIN_BT_RX, MCU_PULL_UP);
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
	if (!answered && probes < BT_PROBES && now - start_ms >= (uint32_t)probes * BT_PROBE_MS) {
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
#if CONFIG_BT_KISS
	st->kiss_taken = d578uv_bt_kiss_taken();
	st->kiss_raw = bt.raw_bytes;
	st->kiss_in = k_in;
	st->kiss_drop = k_drop;
	st->kiss_out = k_out;
	st->kiss_out_bytes = k_out_bytes;
	st->kiss_timeouts = k_timeouts;
#endif
}

// The radio powers the module off for a P25 or NXDN transmission (radio_d578uv.c, tx) only while it serves no link:
// with one up, the key fob or hand mic holding PTT would drop and end the transmission. A single byte read, no lock
bool d578uv_bt_linked(void)
{
	return bt.links != 0;
}

void d578uv_bt_off(void)
{
	offs++;
}

#endif
