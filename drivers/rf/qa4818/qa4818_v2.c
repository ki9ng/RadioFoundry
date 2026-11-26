// SPDX-License-Identifier: MIT
// Copyright (c) 2026 KI9NG
// The newer RF chip in QA4818 sockets (ki9ng fork). AT-D578UV PLUS radios built with the Artery MCU carry it in
// all four RF slots. The vendor firmware (AT 2.08) tells the two apart per slot at start-up and drives this one with:
//   - an address byte of (reg << 1) | R/W, the R/W flag in bit 0 (the QA4818 takes reg | 0x80 to read)
//   - no register pages
//   - its own register program for every operation: enable 0x56, output stage 0x5C, GPIO one pin per 0x52 write,
//     frequency in Hz in 0x69 (high) / 0x68 (low), status 0x0E, RSSI 0x02
// The values here are the vendor firmware's for this chip (static analysis of AT 2.08, KI9NG 2026-11-26; fork only:
// docs/CLEANROOM.md keeps it out of RadioFoundry upstream). Receive, FM transmit and transmit sub-audio (CTCSS,
// DCS) are ported; sub-audio decode, beeps, the AF selector, DMR and the receive gain are not yet, and their entry points leave the chip alone.
#include "qa4818.h"
#include "qa4818_v2.h"
#include "rf/subaudio/subaudio.h"

#define MHZ(x)		((uint32_t)(x) * 1000000u)
#define INIT_HZ		435125000u	// both inits tune here, as the QA4818's do
#define PROBE_TRIES	100u
#define V2_PROBE_VAL	0xA888u
#define QA4818_ID	0x7813u		// the QA4818's register 0x00

static void wr(qa4818_t *c, uint8_t reg, uint16_t val)
{
	rfbus_write(c->bus, reg, val);
}

static uint16_t rd(qa4818_t *c, uint8_t reg)
{
	return rfbus_read(c->bus, reg);
}

static void wr_list(qa4818_t *c, uint8_t reg, const uint16_t *v, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		wr(c, reg, v[i]);
	}
}

bool qa4818_probe(qa4818_t *c)
{
	c->bus->addr_shift = false;
	c->v2 = false;
	wr(c, 0x7F, 0);
	for (unsigned i = 0; i < PROBE_TRIES; i++) {
		if (rd(c, 0x00) == QA4818_ID) {
			return true;
		}
		rfbus_delay_ms(c->bus, 1);
	}
	c->bus->addr_shift = true;
	wr(c, 0x50, 0x0001);
	wr(c, 0x58, 0xF1F0);
	for (unsigned i = 0; i < PROBE_TRIES; i++) {
		wr(c, 0x57, V2_PROBE_VAL);
		rfbus_delay_ms(c->bus, 1);
		if (rd(c, 0x57) == V2_PROBE_VAL) {
			c->v2 = true;
			return true;
		}
		rfbus_delay_ms(c->bus, 1);
	}
	c->bus->addr_shift = false;	// neither answered: left as a QA4818
	return false;
}

// Frequency: Hz in 0x69 (high word) then 0x68 (low word)
static void freq(qa4818_t *c, uint32_t hz)
{
	wr(c, 0x69, (uint16_t)(hz >> 16));
	wr(c, 0x68, (uint16_t)hz);
}

// A local oscillator's pair for its band, written before each tune
static void lo_band(qa4818_t *c, uint32_t hz)
{
	if (hz < MHZ(390)) {
		wr(c, 0x5E, 0x0468);
		wr(c, 0x60, 0x0306);
	} else {
		wr(c, 0x5E, 0x0569);
		wr(c, 0x60, 0x0309);
	}
}

void qa4818_v2_tune(qa4818_t *c, uint32_t hz)
{
	if (c->lo) {
		lo_band(c, hz);
	}
	freq(c, hz);
}

static const uint16_t ramp40_lo[] = { 0x70E6, 0x60FF, 0x50F4, 0x40E8, 0x30DC, 0x20CD, 0x10C3, 0x00B9 };
static const uint16_t ramp53_lo[] = { 0x773C, 0x677C, 0x56BB, 0x46BF, 0x3674, 0x2474, 0x1233, 0x0030 };
static const uint16_t ramp53_trx[] = { 0x773F, 0x673F, 0x56FF, 0x46BE, 0x34BC, 0x247B, 0x1279, 0x0076 };
static const uint16_t r49_set[] = { 0x0050, 0x1078, 0x7034, 0xB0CC };

void qa4818_v2_init_lo(qa4818_t *c, bool trx_v2)
{
	wr(c, 0x50, 0x0001);
	wr(c, 0x58, 0xF1F0);
	wr_list(c, 0x40, ramp40_lo, sizeof(ramp40_lo) / sizeof(ramp40_lo[0]));
	wr_list(c, 0x53, ramp53_lo, sizeof(ramp53_lo) / sizeof(ramp53_lo[0]));
	if (trx_v2) {
		wr(c, 0x5B, 0xB085);	// written when the transceivers are this chip too
	}
	wr(c, 0x41, 0x7152);
	wr(c, 0x43, 0x0A57);
	wr(c, 0x44, 0x1490);
	wr(c, 0x45, 0x8874);
	wr(c, 0x46, 0x006E);
	wr(c, 0x47, 0x07FE);
	wr_list(c, 0x49, r49_set, sizeof(r49_set) / sizeof(r49_set[0]));
	wr(c, 0x4D, 0x0000);
	wr(c, 0x57, 0xA008);
	wr(c, 0x5A, 0x8005);
	wr(c, 0x5D, 0xA610);
	wr(c, 0x5C, 0x0000);
	wr(c, 0x5F, 0x0768);
	wr(c, 0x2A, 0x1081);
	lo_band(c, INIT_HZ);
	wr(c, 0x30, 0x5F6B);
	qa4818_v2_tune(c, INIT_HZ);
	wr(c, 0x59, 0x221C);
	wr(c, 0x29, 0x1090);
	wr(c, 0x66, 0xAA84);
	wr(c, 0x56, 0x0000);
}

// Coefficient pairs of the analog receive path (0x20-0x23 and 0x1C-0x1F), at the vendor's indices for it
static void coef_20(qa4818_t *c)
{
	wr(c, 0x20, 0x8D15);
	wr(c, 0x21, 0x3B99);
	wr(c, 0x22, 0x8D90);
	wr(c, 0x23, 0x3B15);
}

static void coef_1c(qa4818_t *c)
{
	wr(c, 0x1C, 0x810A);
	wr(c, 0x1D, 0x3F0D);
	wr(c, 0x1E, 0x80C3);
	wr(c, 0x1F, 0x3F54);
}

// The receive and transmit path for an analog or a digital channel (the vendor's, around the QA4818's 0x44 / 0x4B / 0x73
// / 0x7E set-up): the 0x53 ramp, the path block, 0x2A, and on side A chip 0x19's analog / digital bits
static const uint16_t ramp53_trx_path[] = { 0x773F, 0x673F, 0x56FF, 0x46BE, 0x34BC, 0x247B, 0x1279, 0x0076 };
void qa4818_v2_path(qa4818_t *c, bool digital)
{
	wr_list(c, 0x53, ramp53_trx_path, sizeof(ramp53_trx_path) / sizeof(ramp53_trx_path[0]));
	wr(c, 0x43, 0x1A40);
	if (!digital) {
		wr(c, 0x11, 0x041A);
		wr(c, 0x2B, 0x708D);
		wr(c, 0x53, 0x773F);
		wr(c, 0x26, 0x6468);
		wr(c, 0x27, 0x6080);
		wr(c, 0x27, 0x199C);
		wr(c, 0x18, 0x006E);
		wr(c, 0x24, 0xF4AF);
		wr(c, 0x25, 0x0B56);
		wr(c, 0x32, 0x22A2);
		coef_20(c);
		coef_1c(c);
		wr(c, 0x2A, 0x1081);
	} else {
		wr(c, 0x2B, 0x708D);
		wr(c, 0x18, 0x026E);
		wr(c, 0x24, 0xF4AF);
		wr(c, 0x25, 0x0B56);
		wr(c, 0x26, 0x6068);
		wr(c, 0x32, 0x0622);
		wr(c, 0x53, 0x773F);
		wr(c, 0x2A, 0x108C);
	}
	if (!c->v2_side_b) {
		c->v2_r19 = digital ? (uint16_t)(c->v2_r19 | 0x0024u) : (uint16_t)(c->v2_r19 & ~0x0024u);
		wr(c, 0x19, c->v2_r19);
	}
}

void qa4818_v2_init_trx(qa4818_t *c, bool side_b)
{
	c->v2_side_b = side_b;
	c->v2_r19 = 0x0381;
	wr(c, 0x50, 0x0001);
	wr(c, 0x58, 0xF1F0);
	wr_list(c, 0x40, ramp40_lo, sizeof(ramp40_lo) / sizeof(ramp40_lo[0]));
	wr_list(c, 0x53, ramp53_trx, sizeof(ramp53_trx) / sizeof(ramp53_trx[0]));
	wr(c, 0x43, 0x1A40);
	if (side_b) {
		wr(c, 0x5B, 0xB085);
	}
	wr(c, 0x41, 0x711A);
	wr(c, 0x42, 0x4A61);
	wr(c, 0x44, 0x1490);
	wr(c, 0x45, 0x8874);
	wr(c, 0x46, 0x006E);
	c->v2_51 = rd(c, 0x51) == 3u;
	wr(c, 0x47, c->v2_51 ? 0x27FE : 0x67FE);
	wr_list(c, 0x49, r49_set, sizeof(r49_set) / sizeof(r49_set[0]));
	wr(c, 0x4D, 0x0000);
	wr(c, 0x57, 0xA008);
	wr(c, 0x5C, 0x0001);
	wr(c, 0x5D, 0x8611);
	wr(c, 0x5E, 0x0768);
	wr(c, 0x5F, 0x0768);
	wr(c, 0x60, 0x0300);
	wr(c, 0x2A, 0x1081);
	wr(c, 0x30, 0x5F6B);
	qa4818_v2_path(c, false);
	wr(c, 0x19, 0x0381);
	c->v2_r19 = 0x0381;
	wr(c, 0x48, 0x9826);
	wr(c, 0x49, 0x704F);
	wr(c, 0x43, 0x1A40);
	freq(c, INIT_HZ);
	wr(c, 0x59, 0x221C);
	wr(c, 0x29, 0x1090);
	wr(c, 0x56, 0x0000);
	wr(c, 0x3C, 0x00FC);
}

void qa4818_v2_rx_on(qa4818_t *c, uint32_t hz)
{
	wr(c, 0x58, 0xF1F0);
	wr(c, 0x5F, 0x0768);
	wr(c, 0x60, 0x0302);
	(void)hz;		// a digital channel picks 0x1090 / 0x10B0 by band (not ported)
	wr(c, 0x29, 0x1090);
	wr(c, 0x56, 0x0001);
}

void qa4818_v2_lo_on(qa4818_t *c)
{
	wr(c, 0x56, 0x0000);
	wr(c, 0x56, 0x0006);
}

void qa4818_v2_off(qa4818_t *c)
{
	wr(c, 0x56, 0x0000);
}

void qa4818_v2_out(qa4818_t *c, uint16_t value)
{
	wr(c, 0x5C, (value == QA4818_OUT_TX_VHF || value == QA4818_OUT_TX_UHF) ? 0x0001 : 0x0000);
}

void qa4818_v2_rx_setup(qa4818_t *c, bool digital)
{
	if (!digital) {
		wr(c, 0x18, 0x006E);
	}
}

// One pin per write: pin << 12 | 0x40 | level. Bit n of the QA4818's 0x25 is pin n here
void qa4818_v2_gpio(qa4818_t *c, uint16_t old, uint16_t now, bool all)
{
	for (unsigned pin = 0; pin < 8u; pin++) {
		uint16_t bit = (uint16_t)(1u << pin);
		if (all || ((old ^ now) & bit) != 0) {
			wr(c, 0x52, (uint16_t)((pin << 12) | 0x40u | ((now & bit) != 0 ? 1u : 0u)));
		}
	}
}

uint16_t qa4818_v2_reg(qa4818_t *c, uint8_t reg, uint16_t val, bool write)
{
	if (write) {
		wr(c, reg, val);
	}
	return rd(c, reg);
}

// Status: 0x0E bit 14 is the chip's signal flag; 0x02 bits 9:0 its RSSI, 4 counts a QA4818 count (the vendor's
// thresholds for it are 4x the QA4818's, so a QA4818 count is half a dB: inferred). No noise register is known: the
// core's squelch gets a low noise with the signal flag, a high one without
#define V2_NOISE_SIGNAL	8u
#define V2_NOISE_NONE	60u
void qa4818_v2_status(qa4818_t *c, qa4818_status_t *st)
{
	uint16_t s = rd(c, 0x0E);
	st->signal = (s & 0x4000u) != 0;
	st->rssi = (uint16_t)((rd(c, 0x02) & 0x3FFu) >> 2);
	st->noise = st->signal ? V2_NOISE_SIGNAL : V2_NOISE_NONE;
	st->tone_flags = 0;
}

// ---- FM transmit (the vendor's key-up order: tune, path, width, output stage, microphone, sub-audio, audio, drive,
// on) ----

// After the tune and the path: the band pair for the TX frequency, the channel width (0x48), the output stage on, the
// FM audio set-up
void qa4818_v2_fm_tx(qa4818_t *c, uint32_t hz)
{
	lo_band(c, hz);
	wr(c, 0x48, c->v2_wide ? 0x9826 : 0x082E);
	wr(c, 0x49, 0x704F);
	wr(c, 0x43, 0x1A40);
	wr(c, 0x5C, 0x0001);
	wr(c, 0x18, 0x006E);
	wr(c, 0x1A, 0x64C1);
	wr(c, 0x1B, 0x1258);
}

// The microphone's level: one of five codes in 0x10 (the vendor's table, by its microphone gain setting)
static const uint16_t mic_codes[] = { 0x001F, 0x0022, 0x0025, 0x002C, 0x002F };
void qa4818_v2_mic(qa4818_t *c, uint8_t idx)
{
	if (idx >= sizeof(mic_codes) / sizeof(mic_codes[0])) {
		idx = (uint8_t)(sizeof(mic_codes) / sizeof(mic_codes[0]) - 1u);
	}
	wr(c, 0x10, (uint16_t)(mic_codes[idx] | 0xA380u));
	wr(c, 0x11, 0x041A);
	wr(c, 0x18, 0x006E);
}

// No sub-audio
void qa4818_v2_no_tone(qa4818_t *c)
{
	wr(c, 0x2D, 0x3205);
	wr(c, 0x2F, 0x0A0F);
}

// The CTCSS encoder's word: Hz x 20.97152 (2^21 / 100000), from tenths of a hertz, truncated as the vendor's
uint16_t qa4818_v2_ctcss_word(uint16_t hz_x10)
{
	return (uint16_t)(((uint32_t)hz_x10 * 2097152u) / 1000000u);
}

// Transmit sub-audio. Register 0x2E takes several fields by its top bits: the tone word (0x0000), its latch (0x2000),
// the DCS code's two halves (0x8000, 0xA000) and the level (0xC000, written by qa4818_v2_drive after this). DCS
// inversion is a bit of 0x2D here, not the complemented code
void qa4818_v2_tx_tone(qa4818_t *c, qa4818_tone_t type, uint16_t value)
{
	switch (type) {
	case QA4818_TONE_CTCSS: {
		uint16_t w = qa4818_v2_ctcss_word(value);
		wr(c, 0x2E, w);
		wr(c, 0x2E, (uint16_t)(w | 0x2000u));
		wr(c, 0x2D, w < 0x0ACDu ? 0x0215 : 0x3215);
		wr(c, 0x2F, 0x0A0F);
		break;
	}
	case QA4818_TONE_DCS:
	case QA4818_TONE_DCS_INV: {
		uint32_t w = subaudio_dcs_word(value);
		wr(c, 0x2E, (uint16_t)(0x8000u | (w & 0xFFFu)));
		wr(c, 0x2E, (uint16_t)(0xA000u | ((w >> 12) & 0x7FFu)));
		wr(c, 0x2E, 0x0B03);
		wr(c, 0x2E, 0x2B03);
		wr(c, 0x2D, type == QA4818_TONE_DCS_INV ? 0x3214 : 0x3294);
		wr(c, 0x2F, 0x0A0F);
		break;
	}
	default:
		qa4818_v2_no_tone(c);
		break;
	}
}

// The tone generator off
void qa4818_v2_tone_off(qa4818_t *c)
{
	wr(c, 0x2A, 0x1081);
	wr(c, 0x16, 0x0000);
	wr(c, 0x3F, 0x3232);
}

// The drive (0x31, 0 = off) and the sub-audio level (0x2E), from the chip's own calibration block (the board's)
void qa4818_v2_drive(qa4818_t *c, uint16_t drive, uint16_t level)
{
	wr(c, 0x31, drive != 0 ? (uint16_t)(drive | 0x1000u) : 0);
	wr(c, 0x2E, (uint16_t)(level | 0xC000u));
}

void qa4818_v2_drive_off(qa4818_t *c)
{
	wr(c, 0x31, 0);
}

void qa4818_v2_tx_on(qa4818_t *c)
{
	wr(c, 0x2B, 0x708D);
	wr(c, 0x56, 0x0002);
}
