// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// Tools > APRS (docs/spec/aprs.md 10): what APRS is doing (callsign, frequency, position, next beacon, or why nothing
// can be sent) and the everyday choices, one row each: send now, automatic beacons, the frequency (region), the RF
// power (as the channel, or a level of its own), the digipeater path (the hops), our SSID, the interval, SmartBeaconing, the
// way out (radio / DMR / both), KISS on the host port, and the stations heard (receive builds). Where the position
// comes from is the position service's (Settings > Location, ui_position.c). The rest (symbol, comment, digital
// destination, iGate, digipeater) is set with qdmr, the debug link or tools/rfoundry_debug.py (aprs.md 11).
#include <string.h>
#include "ui_nav.h"
#include "ui_svc_aprs.h"
#include "fmt.h"
#include "i18n.h"
#include "config.h"
#include "board.h"

// The services when APRS isn't built (and in the UI simulator): nothing there
#define WEAK __attribute__((weak))
WEAK bool ui_svc_aprs_view(aprs_view_t *v) { memset(v, 0, sizeof(*v)); return false; }
WEAK void ui_svc_aprs_cfg(aprs_cfg_t *c) { memset(c, 0, sizeof(*c)); }
WEAK void ui_svc_aprs_set_cfg(const aprs_cfg_t *c) { (void)c; }
WEAK bool ui_svc_aprs_send_now(void) { return false; }
WEAK bool ui_svc_aprs_send_message(const char *to, const char *text, uint8_t flags)
{
	(void)to;
	(void)text;
	(void)flags;
	return false;
}
WEAK uint8_t ui_svc_aprs_msg_count(void) { return 0; }
WEAK bool ui_svc_aprs_msg(uint8_t i, aprs_msg_t *out, bool read)
{
	(void)i; (void)out; (void)read; return false;
}
WEAK bool ui_svc_aprs_msg_delete(uint8_t i) { (void)i; return false; }
WEAK uint8_t ui_svc_aprs_station_count(void) { return 0; }
WEAK bool ui_svc_aprs_station(uint8_t i, aprs_station_t *out) { (void)i; (void)out; return false; }
WEAK bool ui_svc_aprs_station_find(const char *call, aprs_station_t *out) { (void)call; (void)out; return false; }
WEAK bool ui_svc_aprs_range(const aprs_station_t *s, uint32_t *m, uint16_t *deg) { (void)s; (void)m; (void)deg; return false; }
WEAK uint8_t ui_svc_aprs_qsy(const radio_view_t *v, const aprs_freq_t *f) { (void)v; (void)f; return UI_QSY_NONE; }
WEAK bool ui_svc_aprs_qsy_undo(uint32_t *hz) { (void)hz; return false; }
WEAK void ui_svc_aprs_qsy_track(const radio_view_t *v) { (void)v; }
// The band plan's offset (aprs_fmt.c): none known without APRS
WEAK int32_t aprs_freq_std_offset_hz(uint32_t hz) { (void)hz; return 0; }

enum { AP_SEND, AP_AUTO, AP_FREQ, AP_POWER, AP_PATH, AP_SSID, AP_INTERVAL, AP_SMART, AP_VIA, AP_KISS, AP_HEARD, AP_MUTE,
       AP_FREQ_BCN, AP_POPUP, AP_POPUP_S, AP_UNITS, AP_COUNT };

// The rows this build shows, in order: the packet rows (SSID too) only with the AX.25 backend (CONFIG_APRS_AFSK),
// "Send via" only when there is a choice (both backends built)
static const uint8_t rows[] = {
	AP_SEND, AP_AUTO,
#if CONFIG_APRS_AFSK
	AP_FREQ, AP_POWER, AP_PATH,
#endif
	AP_INTERVAL, AP_SMART,
#if CONFIG_APRS_AFSK && CONFIG_APRS_DMR
	AP_VIA,
#endif
#if CONFIG_APRS_QSY
	AP_FREQ_BCN, AP_POPUP, AP_POPUP_S,
#endif
#if CONFIG_APRS_RX
	AP_UNITS,
#endif
	AP_KISS,
#if CONFIG_APRS_AFSK
	AP_HEARD,
#endif
#if CONFIG_APRS_RX
	AP_MUTE,	// after Stations heard: the rows above keep their digits
#endif
#if CONFIG_APRS_AFSK
	AP_SSID,	// set once: last
#endif
};
#define N_ROWS	((uint16_t)(sizeof(rows) / sizeof(rows[0])))

// Beacons go out on RF only: no DMR backend, or the setting says so
static bool rf_only(const aprs_cfg_t *c)
{
	return !CONFIG_APRS_DMR || c->via == APRS_VIA_RF;
}

static const uint16_t intervals[] = { 0, 60, 120, 300, 600, 1800 };
static const tr_id_t interval_names[] = { TRN(VALUE_OFF, "Off"), TRN(VALUE_1_MIN, "1 min"), TRN(VALUE_2_MIN, "2 min"),
	TRN(VALUE_5_MIN, "5 min"), TRN(VALUE_10_MIN, "10 min"), TRN(VALUE_30_MIN, "30 min") };
#define N_INTERVALS	(sizeof(intervals) / sizeof(intervals[0]))
// aprs_path_t order; the text is aprs_path_text()'s, kept here so the page builds without APRS (the simulator)
static const tr_id_t path_names[APRS_PATH_COUNT] = { TRN(VALUE_NONE, "None"), TRX(APRS_VALUE_WIDE1_1, "WIDE1-1"),
	TRX(APRS_VALUE_WIDE1_1_WIDE2_1, "WIDE1-1,WIDE2-1"), TRX(APRS_VALUE_WIDE2_1, "WIDE2-1"),
	TRX(APRS_VALUE_WIDE2_2, "WIDE2-2") };
static const tr_id_t via_names[] = { TRX(APRS_VALUE_AFSK, "AFSK"), TRX(APRS_VALUE_DMR, "DMR"),
	TRN(APRS_VALUE_BOTH, "Both") };
static const tr_id_t pos_names[] = { TRN(VALUE_AUTO, "Auto"), TRX(WORD_GPS, "GPS"), TRN(POS_VALUE_COMPUTER, "Computer"),
	TRN(POS_VALUE_FIXED, "Fixed") };
static const tr_id_t region_names[APRS_REGION_COUNT] = { TRN(APRS_VALUE_REGION_N_AMERICA, "N America"),
	TRN(APRS_VALUE_REGION_EUROPE, "Europe"), TRN(APRS_VALUE_REGION_AUSTRALIA, "Australia"),
	TRN(APRS_VALUE_REGION_NEW_ZEALAND, "New Zealand"), TRN(APRS_VALUE_REGION_JAPAN, "Japan"),
	TRN(APRS_VALUE_REGION_CHINA, "China"), TRN(APRS_VALUE_REGION_CUSTOM, "Custom") };

// aprs_popup_t order
static const tr_id_t popup_names[APRS_POPUP_COUNT] = { TRN(APRS_VALUE_POPUP_ALL, "All"), TRN(VALUE_OFF, "Off"),
	TRN(APRS_VALUE_POPUP_DIRECT, "Heard direct") };

#if CONFIG_APRS_QSY
// An inline number row: OK opens it, the knob and Up / Down step 1 and stop at the ends, digits type the value (it is
// kept as soon as it has as many digits as the largest value, else on OK), Back leaves the setting as it was
static struct {
	bool on;
	uint8_t typed;		// digits typed so far (0: the value shown is stepped, not typed)
	uint16_t v, lo, hi;
} num;

static void num_open(uint16_t v, uint16_t lo, uint16_t hi)
{
	num.on = true;
	num.typed = 0;
	num.lo = lo;
	num.hi = hi;
	num.v = v < lo ? lo : v > hi ? hi : v;
}

static uint16_t num_close(void)
{
	num.on = false;
	return num.v < num.lo ? num.lo : num.v > num.hi ? num.hi : num.v;
}

static uint8_t num_digits(uint16_t v)
{
	uint8_t n = 1;
	while (v >= 10u) {
		v /= 10u;
		n++;
	}
	return n;
}

// The value being edited, a typed one with a cursor while more digits can follow: "1_"
static void num_text(sbuf_t *b)
{
	sb_u32(b, num.v, 1);
	if (num.typed != 0 && num.typed < num_digits(num.hi)) {
		sb_chr(b, '_');
	}
}

// The open row's keys (aprs_key); OK is the list's (aprs_select keeps the value)
static bool num_key(const input_event_t *ev, void (*keep)(uint16_t v))
{
	bool press = ev->gesture == GESTURE_PRESS || ev->gesture == GESTURE_REPEAT;
	int dir = 0;
	if (ev->action == ACT_ENC_STEP && ev->gesture != GESTURE_UP) {
		dir = ev->value > 0 ? 1 : ev->value < 0 ? -1 : 0;
	} else if (press && ev->action == ACT_NAV_UP) {
		dir = 1;
	} else if (press && ev->action == ACT_NAV_DOWN) {
		dir = -1;
	}
	if (dir != 0) {
		if (num.typed != 0 && num.v < num.lo) {
			num.v = num.lo;
		}
		num.typed = 0;
		if (dir > 0 && num.v < num.hi) {
			num.v++;
		} else if (dir < 0 && num.v > num.lo) {
			num.v--;
		}
		ui_dirty();
		return true;
	}
	if (ev->action >= ACT_DIGIT_0 && ev->action <= ACT_DIGIT_9 && ev->gesture == GESTURE_PRESS) {
		uint16_t d = (uint16_t)(ev->action - ACT_DIGIT_0);
		num.v = num.typed == 0 ? d : (uint16_t)(num.v * 10u + d);
		num.typed++;
		if (num.typed >= num_digits(num.hi)) {
			keep(num_close());
		}
		ui_dirty();
		return true;
	}
	if (ev->action == ACT_BACK && ev->gesture == GESTURE_PRESS) {
		num.on = false;
		ui_dirty();
		return true;
	}
	return false;
}

static void popup_s_keep(uint16_t v)
{
	aprs_cfg_t c;
	ui_svc_aprs_cfg(&c);
	c.popup_s = (uint8_t)v;
	ui_svc_aprs_set_cfg(&c);
}

static bool aprs_key(const input_event_t *ev)
{
	return num.on && num_key(ev, popup_s_keep);
}
#else
static bool aprs_key(const input_event_t *ev) { (void)ev; return false; }
#endif

static void aprs_enter(void)
{
#if CONFIG_APRS_QSY
	num.on = false;
#endif
}

static const char *aprs_hint(void)
{
#if CONFIG_APRS_QSY
	if (num.on)
		return TR(APRS_HINT_NUMBER, TEXT_ALT_OPEN "Up Down 0-9  OK keep  Back" TEXT_ALT_NEXT "0-9  OK keep  Back"
			TEXT_ALT_CLOSE);
#endif
	return TR(HINT_0_9_PICK_BACK, "0-9 pick  Back");
}

// The radio can send APRS on RF (a board with an AFSK transmit path; some only receive)
static bool rf_send(void)
{
	return board.afsk != NULL && board.afsk->tx_start != NULL;
}

static const char *why_text(uint8_t block)
{
	switch (block) {
	case APRS_NO_CALL: return TR(APRS_WHY_NO_CALLSIGN, "Set your callsign (the radio name) in qdmr.");
	case APRS_NO_FREQ: return TR(APRS_WHY_NO_FREQUENCY, "No APRS frequency: pick a region.");
	case APRS_NOT_AMATEUR: return TR(APRS_WHY_NOT_HAM_BAND, "Pick a frequency in a ham band this radio sends on.");
	case APRS_NO_AFSK: return TR(APRS_WHY_NO_TONES, "No APRS tones on this radio; DMR positions work.");
	default: return NULL;
	}
}

static const char *aprs_body(void)
{
	static char t[160];
	sbuf_t b;
	aprs_view_t v;
	sb_init(&b, t, sizeof t);
	if (!ui_svc_aprs_view(&v)) {
		sb_str(&b, TR(WHY_APRS_NOT_IN_BUILD, "APRS: not in this build"));
		return t;
	}
	if (CONFIG_APRS_AFSK) {		// who and where on RF: the AX.25 backend's
		sb_str(&b, v.call[0] != '\0' ? v.call : TR(APRS_BODY_NO_CALLSIGN, "No callsign"));
		sb_str(&b, TR(APRS_BODY_ON, " on "));
		sb_freq(&b, v.freq_hz);
		if (v.rf_active) {
			sb_str(&b, TR(APRS_BODY_SENDING, " (sending)"));
		}
		const char *why = why_text(v.block);
		if (why != NULL) {
			sb_chr(&b, '\n');
			sb_str(&b, why);
		}
		sb_chr(&b, '\n');
	}
	if (v.pos_valid) {
		sb_str(&b, tr(pos_names[v.pos_src < 4 ? v.pos_src : 0]));
		sb_chr(&b, ' ');
		sb_degrees(&b, v.lat);
		sb_chr(&b, ' ');
		sb_degrees(&b, v.lon);
	} else {
		sb_str(&b, TR(APRS_BODY_NO_POSITION, "No position"));
	}
	if (v.next_s != 0xFFFFu) {
		sb_chr(&b, '\n');
		sb_str(&b, TR(APRS_BODY_NEXT_IN, "Next in "));
		sb_mmss(&b, v.next_s);
	}
	if (v.rf_sent + v.dmr_sent != 0) {
		sb_str(&b, TR(APRS_BODY_SENT, "  Sent "));
		sb_u32(&b, v.rf_sent + v.dmr_sent, 1);
	}
	return t;
}

static uint16_t aprs_count(void) { return N_ROWS; }

static uint8_t interval_index(uint16_t s)
{
	for (uint8_t i = 0; i < N_INTERVALS; i++) {
		if (intervals[i] >= s) {
			return i;
		}
	}
	return (uint8_t)(N_INTERVALS - 1u);
}

static void aprs_item(uint16_t i, ui_item_t *it)
{
	static const tr_id_t names[] = { TRN(APRS_ROW_SEND_NOW, "Send position now"),
		TRN(APRS_LABEL_BEACONS, "Automatic beacons"), TRN(APRS_LABEL_FREQUENCY, "Frequency"),
		TRN(LABEL_POWER, "Power"), TRN(APRS_LABEL_PATH, "Path"), TRX(APRS_LABEL_SSID, "SSID"),
		TRN(APRS_LABEL_INTERVAL, "Interval"),
		TRN(APRS_LABEL_SMARTBEACONING, "SmartBeaconing"), TRN(APRS_LABEL_SEND_VIA, "Send via"),
		TRN(APRS_LABEL_KISS_ON_USB, "KISS on USB"), TRN(APRS_ROW_STATIONS_HEARD, "Stations heard"),
		TRN(APRS_LABEL_MUTE, "Mute APRS audio"), TRN(APRS_LABEL_FREQ_IN_BEACON, "Freq in beacon"),
		TRN(APRS_LABEL_POPUP, "Pop-up"), TRN(APRS_LABEL_POPUP_TIME, "Pop-up time"), TRN(APRS_LABEL_UNITS, "Units") };
	aprs_view_t v;
	aprs_cfg_t c;
	bool built = ui_svc_aprs_view(&v);
	ui_svc_aprs_cfg(&c);
	it->num = i < 10u ? (char)('0' + (i + 1u) % 10u) : 0;	// rows after the 10th have no digit
	i = i < N_ROWS ? rows[i] : AP_COUNT;
	if (i >= AP_COUNT) {
		return;
	}
	// Without the AX.25 backend the host port is no TNC: it only takes NMEA positions
	str_copy(it->label, sizeof it->label, i == AP_KISS && !CONFIG_APRS_AFSK ?
		TR(APRS_LABEL_NMEA_ON_USB, "NMEA on USB") : tr(names[i]));
	sbuf_t b;
	sb_init(&b, it->value, sizeof it->value);
	if (!built) {
		it->flags |= UI_ITEM_DIM;
		return;
	}
	switch (i) {
	case AP_SEND:
		if (v.block != APRS_OK && rf_only(&c)) {
			it->flags |= UI_ITEM_DIM;
		}
		break;
	case AP_AUTO: sb_str(&b, c.tx_on ? TR(VALUE_ON, "On") : TR(VALUE_OFF, "Off")); break;
	case AP_FREQ:
		if (board.afsk == NULL && !CONFIG_APRS_RX) {
			it->flags |= UI_ITEM_DIM;	// no APRS on RF here
		} else if (c.region == APRS_REGION_CUSTOM) {
			sb_freq(&b, c.freq_hz);
		} else {
			sb_str(&b, tr(region_names[c.region < APRS_REGION_COUNT ? c.region : 0]));
		}
		break;
	case AP_POWER:
		if (!rf_send()) {
			it->flags |= UI_ITEM_DIM;	// no APRS on RF here (no transmit tone path)
		} else {
			sb_str(&b, c.power == 0 ? TR(APRS_VALUE_AS_CHANNEL, "As channel") :
				ui_power_name((uint8_t)(c.power - 1u), v.freq_hz));
		}
		break;
	case AP_PATH:
		if (!rf_send()) {
			it->flags |= UI_ITEM_DIM;	// the path is for RF packets only
		} else {
			sb_str(&b, tr(path_names[c.path < APRS_PATH_COUNT ? c.path : 0]));
		}
		break;
	case AP_SSID:
		if (!rf_send()) {
			it->flags |= UI_ITEM_DIM;	// RF packets only: DMR positions carry the network's SSID
		} else {
			sb_u32(&b, c.ssid, 1);
		}
		break;
	case AP_INTERVAL: sb_str(&b, tr(interval_names[interval_index(c.interval_s)])); break;
	case AP_SMART: sb_str(&b, c.smart ? TR(VALUE_ON, "On") : TR(VALUE_OFF, "Off")); break;
	case AP_VIA: sb_str(&b, tr(via_names[c.via < APRS_VIA_COUNT ? c.via : 0])); break;
	case AP_KISS: sb_str(&b, c.kiss ? TR(VALUE_ON, "On") : TR(VALUE_OFF, "Off")); break;
	case AP_MUTE: sb_str(&b, c.mute ? TR(VALUE_ON, "On") : TR(VALUE_OFF, "Off")); break;
	case AP_FREQ_BCN:
		if (!rf_send()) {
			it->flags |= UI_ITEM_DIM;
		} else {
			sb_str(&b, c.freq_beacon ? TR(VALUE_ON, "On") : TR(VALUE_OFF, "Off"));
		}
		break;
	case AP_POPUP: sb_str(&b, tr(popup_names[c.popup < APRS_POPUP_COUNT ? c.popup : 0])); break;
#if CONFIG_APRS_QSY
	case AP_POPUP_S:
		if (num.on) {
			num_text(&b);
			it->flags |= UI_ITEM_EDIT;
		} else {
			sb_u32(&b, c.popup_s, 1);
		}
		sb_str(&b, " s");
		break;
#endif
	case AP_UNITS: sb_str(&b, c.miles ? "mi" : "km"); break;
	case AP_HEARD:
		if (!CONFIG_APRS_RX) {
			it->flags |= UI_ITEM_DIM;
		} else {
			sb_u32(&b, v.stations, 1);
		}
		break;
	default: break;
	}
}

static void aprs_select(uint16_t i, bool hold)
{
	aprs_view_t v;
	aprs_cfg_t c;
	if (!ui_svc_aprs_view(&v)) {
		ui_toast(TR(WHY_APRS_NOT_IN_BUILD, "APRS: not in this build"));
		return;
	}
	ui_svc_aprs_cfg(&c);
	switch (i < N_ROWS ? rows[i] : AP_COUNT) {
	case AP_SEND:
		if (ui_svc_aprs_send_now()) {
			ui_toast(TR(APRS_TOAST_SENDING_POSITION, "Sending position"));
		} else {
			ui_toast(v.block == APRS_NO_CALL ?
				TR(APRS_TOAST_SET_CALLSIGN_FIRST, "Set your callsign first") :
				TR(APRS_TOAST_CANT_SEND_NOW, "Can't send now"));
		}
		return;
	case AP_AUTO:
		c.tx_on = !c.tx_on;
		ui_toast(c.tx_on ? TR(APRS_TOAST_BEACONS_ON, "Beacons on") : TR(APRS_TOAST_BEACONS_OFF, "Beacons off"));
		break;
	case AP_FREQ:
		if (board.afsk == NULL && !CONFIG_APRS_RX) {
			return;
		}
		c.region = (uint8_t)((c.region + 1u) % APRS_REGION_COUNT);
		if (c.region == APRS_REGION_CUSTOM && c.freq_hz == 0) {
			c.region = 0;	// no custom frequency set (qdmr / debug link): skip it
		}
		break;
	case AP_POWER:
		if (!rf_send()) {
			return;
		}
		c.power = (uint8_t)((c.power + 1u) % (board.power_count + 1u));	// as the channel, then each level
		break;
	case AP_PATH:
		if (!rf_send()) {
			return;
		}
		c.path = (uint8_t)((c.path + 1u) % APRS_PATH_COUNT);	// none, then more hops; WIDE2-2 wraps to none
		break;
	case AP_SSID:
		if (!rf_send()) {
			return;
		}
		c.ssid = (uint8_t)((c.ssid + (hold ? 15u : 1u)) % 16u);	// 0-15, wrapping; hold steps back
		break;
	case AP_INTERVAL: c.interval_s = intervals[(interval_index(c.interval_s) + 1u) % N_INTERVALS]; break;
	case AP_SMART: c.smart = !c.smart; break;
	case AP_VIA: c.via = (uint8_t)((c.via + 1u) % APRS_VIA_COUNT); break;
	case AP_KISS: c.kiss = !c.kiss; break;
	case AP_MUTE: c.mute = !c.mute; break;
	case AP_FREQ_BCN:
		if (!rf_send()) {
			return;
		}
		c.freq_beacon = !c.freq_beacon;
		break;
	case AP_POPUP: c.popup = (uint8_t)((c.popup + 1u) % APRS_POPUP_COUNT); break;
#if CONFIG_APRS_QSY
	case AP_POPUP_S:
		if (!num.on) {
			num_open(c.popup_s, APRS_POPUP_S_MIN, APRS_POPUP_S_MAX);
			ui_dirty();
			return;
		}
		c.popup_s = (uint8_t)num_close();
		break;
#endif
	case AP_UNITS: c.miles = !c.miles; break;
	case AP_HEARD:
		if (!CONFIG_APRS_RX) {
			ui_toast(TR(APRS_WHY_CANT_RECEIVE, "This radio can't receive APRS"));
		} else {
			ui_push(SCR_APRS_HEARD, 0);
		}
		return;
	default: return;
	}
	ui_svc_aprs_set_cfg(&c);
	ui_dirty();
}

const ui_scr_def_t ui_aprs_screen = {
	.name = "aprs", .flags = SF_LIST | SF_NUMBERED | SF_FIELD | SF_HINT_FN, .prefix = TRX(WORD_APRS, "APRS"),
	.hint_fn = aprs_hint,
	.count = aprs_count, .item = aprs_item, .select = aprs_select, .body = aprs_body, .key = aprs_key,
	.enter = aprs_enter,
};

// --- Stations heard (receive builds) ---

#if CONFIG_APRS_RX
static char station_call[AX25_CALL_LEN + 4];
#if CONFIG_APRS_QSY
static uint32_t popup_until;	// the station screen opened by itself (a pop-up) and closes then; 0: it didn't
#define POPUP_END()	(popup_until = 0)
#else
#define POPUP_END()	((void)0)
#endif

#if CONFIG_APRS_QSY
// A heard station's voice frequency as a repeater list writes it: "146.685- T100", "446.000", "147.240+ D023"
static void sb_qsy(sbuf_t *b, const aprs_freq_t *f)
{
	sb_freq(b, f->hz);
	int32_t off = f->toff ? 0 : f->offset_10k == APRS_FREQ_OFFSET_STD ? aprs_freq_std_offset_hz(f->hz) :
		f->offset_10k;
	if (off != 0) {
		sb_chr(b, off > 0 ? '+' : '-');
	}
	if (!f->toff && f->tone_kind != APRS_FREQ_TONE_NONE) {
		bool dcs = f->tone_kind == APRS_FREQ_TONE_DCS;
		sb_chr(b, ' ');
		sb_chr(b, dcs ? 'D' : f->tone_kind == APRS_FREQ_TONE_SQL ? 'C' : 'T');
		if (dcs) {
			sb_chr(b, (char)('0' + (f->tone >> 6 & 7u)));
			sb_chr(b, (char)('0' + (f->tone >> 3 & 7u)));
			sb_chr(b, (char)('0' + (f->tone & 7u)));
		} else {
			sb_u32(b, f->tone / 10u, 1);
			if (f->tone % 10u != 0) {
				sb_chr(b, '.');
				sb_u32(b, f->tone % 10u, 1);
			}
		}
	}
}
#endif
#endif

static uint16_t heard_count(void) { return ui_svc_aprs_station_count(); }

static void heard_select(uint16_t i, bool hold)
{
	(void)hold;
#if CONFIG_APRS_RX
	aprs_station_t s;
	if (ui_svc_aprs_station((uint8_t)i, &s)) {
		str_copy(station_call, sizeof station_call, s.call);
		POPUP_END();
		ui_push(SCR_APRS_STATION, 0);
	}
#else
	ui_push(SCR_APRS_STATION, i);
#endif
}

static void heard_item(uint16_t i, ui_item_t *it)
{
	aprs_station_t s;
	if (!ui_svc_aprs_station((uint8_t)i, &s)) {
		return;
	}
	aprs_cfg_t c;
	ui_svc_aprs_cfg(&c);
	sbuf_t b;
	uint32_t m, at = s.heard_ms;
	uint16_t deg;
	sb_init(&b, it->label, sizeof it->label);
	sb_str(&b, s.call);
	if (ui_svc_aprs_range(&s, &m, &deg)) {
		// How far and which way ("12.3 km NE"): an alternate, so a narrow row drops it before the call (font.h)
		sb_str(&b, TEXT_ALT_OPEN " ");
		sb_range_u(&b, m, deg, c.miles);
		sb_str(&b, TEXT_ALT_NEXT TEXT_ALT_CLOSE);
		at = s.pos_ms;	// the age of the position the range is from, not of the station's last packet
	}
	ui_sb_age(it->value, sizeof it->value, ui_st.now_ms - at);
	if (!s.direct) {
		it->flags |= UI_ITEM_TEMP;	// heard through a digipeater
	}
}

const ui_scr_def_t ui_aprs_heard_screen = {
	.name = "aprs-heard", .flags = SF_LIST | SF_FIELD, .prefix = TRN(APRS_TITLE_HEARD, "Heard"),
	.hint = TRN(HINT_BACK, "Back"),
	.empty = TRN(APRS_EMPTY_NOTHING_HEARD, "Nothing heard yet"),
	.count = heard_count, .item = heard_item, .select = heard_select,
};

#if CONFIG_APRS_RX
// A heard station (docs/spec/aprs.md 10): a compass, north up, with us in the middle and the station on the rim at its
// bearing; beside it the distance, path and age, speed, course, altitude and the voice frequency it offers; its comment
// below. The knob steps through the stations heard, OK writes it a message, CONTEXT tunes to its frequency (QSY). It
// also opens by itself on Home when a station is heard (the Pop-up setting) and closes after the Pop-up time, unless
// a key is pressed on it

// One line of the right-hand column, cut to its width; false once the column is full
static bool station_line(int x, int *y, int bottom, const char *text, color_t c)
{
	const font_t *f = ui_layout.small;
	if (*y + f->h > bottom) {
		return false;
	}
	char cut[32];
	gfx_text_fit(f, text, gfx_width() - x, cut, sizeof cut);
	gfx_text(f, x, *y, cut, c);
	*y += f->h;
	return true;
}

static void compass(int cx, int cy, int r, bool range, uint16_t deg)
{
	const font_t *t = ui_layout.tiny;
	gfx_circle(cx, cy, r, ui_col.line);
	for (int a = 0; a < 360; a += 45) {	// a tick every 45 degrees, longer at the four points
		int in = a % 90 == 0 ? r - 4 : r - 2;
		gfx_line(cx + in * gfx_isin(a) / 256, cy - in * gfx_isin(a + 90) / 256,
			 cx + r * gfx_isin(a) / 256, cy - r * gfx_isin(a + 90) / 256, ui_col.line);
	}
	gfx_text(t, cx - t->w / 2, cy - r + 4, "N", ui_col.warn);
	gfx_text(t, cx - t->w / 2, cy + r - 3 - t->h, "S", ui_col.dim);
	gfx_text(t, cx + r - 4 - t->w, cy - t->h / 2, "E", ui_col.dim);
	gfx_text(t, cx - r + 5, cy - t->h / 2, "W", ui_col.dim);
	gfx_disc(cx, cy, 1, ui_col.fg);	// us
	if (!range) {
		gfx_text(t, cx - t->w / 2, cy + 3, "?", ui_col.dim);
		return;
	}
	int d = r - 5;
	int sx = cx + d * gfx_isin(deg) / 256, sy = cy - d * gfx_isin(deg + 90) / 256;
	gfx_line(cx, cy, sx, sy, ui_col.dim);
	gfx_disc(sx, sy, 3, ui_col.rx);
}

static const char *station_hint(bool qsy)
{
	return qsy ? TR(APRS_HINT_STATION_QSY, TEXT_ALT_OPEN "# QSY  OK msg  Back" TEXT_ALT_NEXT "# QSY  OK msg"
		TEXT_ALT_CLOSE) : TR(APRS_HINT_STATION, "OK msg  Back");
}

static void station_draw(void)
{
	aprs_station_t s;
	aprs_cfg_t c;
	ui_svc_aprs_cfg(&c);
	bool found = ui_svc_aprs_station_find(station_call, &s);
	gfx_clear(ui_col.bg);
	int y = ui_draw_top(ui_view(), &ui_st, tr(TRN(APRS_TITLE_HEARD, "Heard")), station_call) + 2;
	const font_t *t = ui_layout.tiny;
	int bottom = ui_layout.hint_y - 1;
	if (!found) {
		gfx_text(ui_layout.small, 0, y, TR(APRS_BODY_STATION_GONE, "No longer in list"), ui_col.dim);
		ui_draw_bottom(&ui_st, station_hint(false));
		ui_draw_banner(&ui_st);
		return;
	}
	bool comment = s.text[0] != '\0';
	int area = bottom - y - (comment ? t->h + 1 : 0);
	// The compass leaves the column room for a frequency with its tone ("146.685- T100", 13 cells)
	int side = gfx_width() - 13 * ui_layout.small->w - 8;
	if (side > area) {
		side = area;
	}
	int r = side / 2 - 1;
	uint32_t m = 0;
	uint16_t deg = 0;
	bool range = ui_svc_aprs_range(&s, &m, &deg);
	bool dial = side >= 24;	// smaller, it says nothing: the lines get the whole width
	if (dial) {
		compass(r + 1, y + r + 1, r, range, deg);
	}

	char buf[32];
	sbuf_t b;
	int x = dial ? 2 * r + 6 : 0, ly = y, lb = y + area;
	// The most useful first: a small display has room for a few lines only
	if (range) {
		sb_init(&b, buf, sizeof buf);
		sb_range_u(&b, m, deg, c.miles);
		station_line(x, &ly, lb, buf, ui_col.fg);
	}
#if CONFIG_APRS_QSY
	bool qsy = s.freq.hz != 0;
	if (qsy) {
		sb_init(&b, buf, sizeof buf);
		sb_qsy(&b, &s.freq);
		station_line(x, &ly, lb, buf, ui_col.rx);
	}
#else
	bool qsy = false;
#endif
	sb_init(&b, buf, sizeof buf);
	sb_str(&b, s.direct ? TR(APRS_VALUE_DIRECT, "Direct") : TR(APRS_VALUE_DIGI, "Digi"));
	sb_chr(&b, ' ');
	char age[24];	// both alternates ("13s ago", "13s") and their marks: 12 cut a two-digit age's unit
	ui_sb_age(age, sizeof age, ui_st.now_ms - s.heard_ms);
	sb_str(&b, age);
	station_line(x, &ly, lb, buf, ui_col.dim);
	if (s.has_pos && s.speed_kn != APRS_SPEED_NONE) {
		sb_init(&b, buf, sizeof buf);
		sb_str(&b, TR(APRS_LABEL_SPD, "Spd "));
		uint32_t kn = s.speed_kn;
		sb_u32(&b, c.miles ? (kn * 1151u + 500u) / 1000u : (kn * 1852u + 500u) / 1000u, 1);
		sb_str(&b, c.miles ? " mph" : " km/h");
		station_line(x, &ly, lb, buf, ui_col.fg);
	}
	if (s.has_pos && s.course != APRS_COURSE_NONE) {
		sb_init(&b, buf, sizeof buf);
		sb_str(&b, TR(APRS_LABEL_CRS, "Crs "));
		sb_u32(&b, s.course % 360u, 1);
		sb_str(&b, GLYPH_DEGREE " ");
		sb_compass(&b, s.course);
		station_line(x, &ly, lb, buf, ui_col.fg);
	}
#if CONFIG_APRS_QSY
	if (s.has_pos && s.alt_m != APRS_ALT_NONE) {
		sb_init(&b, buf, sizeof buf);
		sb_str(&b, TR(APRS_LABEL_ALT, "Alt "));
		if (c.miles) {
			int32_t ft = (int32_t)(((int64_t)s.alt_m * 3281 + (s.alt_m < 0 ? -500 : 500)) / 1000);
			sb_i32(&b, ft);
			sb_str(&b, " ft");
		} else {
			sb_i32(&b, s.alt_m);
			sb_str(&b, " m");
		}
		station_line(x, &ly, lb, buf, ui_col.fg);
	}
#endif
	if (comment) {
		char cut[48];
		gfx_text_fit(t, s.text, gfx_width(), cut, sizeof cut);
		gfx_text(t, 0, bottom - t->h, cut, ui_col.dim);
	}
	ui_draw_bottom(&ui_st, station_hint(qsy));
	ui_draw_banner(&ui_st);
}

// The next or previous station in the list (most recent first), wrapping
static void station_step(int dir)
{
	uint8_t n = ui_svc_aprs_station_count();
	if (n == 0) {
		return;
	}
	aprs_station_t s;
	uint8_t at = 0;
	for (uint8_t i = 0; i < n; i++) {
		if (ui_svc_aprs_station(i, &s) && strcmp(s.call, station_call) == 0) {
			at = i;
			break;
		}
	}
	at = (uint8_t)((at + n + (dir > 0 ? 1 : n - 1)) % n);
	if (ui_svc_aprs_station(at, &s)) {
		str_copy(station_call, sizeof station_call, s.call);
	}
}

#if CONFIG_APRS_QSY
static void station_qsy(void)
{
	static char done[40];
	aprs_station_t s;
	if (!ui_svc_aprs_station_find(station_call, &s) || s.freq.hz == 0) {
		ui_toast(TR(APRS_TOAST_NO_FREQUENCY, "No frequency in its beacon"));
		return;
	}
	switch (ui_svc_aprs_qsy(ui_view(), &s.freq)) {
	case UI_QSY_OK: {
		sbuf_t b;
		sb_init(&b, done, sizeof done);
		sb_str(&b, TR(APRS_TOAST_QSY, "QSY "));
		sb_qsy(&b, &s.freq);
		popup_until = 0;
		ui_go_home();
		ui_toast(done);
		break;
	}
	case UI_QSY_BAND:
		ui_toast(TR(APRS_TOAST_QSY_BAND, "QSY: not a band this radio has"));
		break;
	case UI_QSY_DUAL:
		ui_toast(TR(APRS_TOAST_QSY_DUAL, "QSY: turn dual watch off first"));
		break;
	default:
		break;
	}
}
#endif

static bool station_key(const input_event_t *ev)
{
	bool press = ev->gesture == GESTURE_PRESS || ev->gesture == GESTURE_REPEAT;
	if (ev->action == ACT_BACK) {
		POPUP_END();
		return false;	// the list's own Back
	}
	if (ev->gesture != GESTURE_UP) {
		POPUP_END();	// a key on a pop-up keeps it open: it is the user's screen now
	}
	int dir = 0;
	if (ev->action == ACT_ENC_STEP && ev->gesture != GESTURE_UP) {
		dir = ev->value > 0 ? 1 : ev->value < 0 ? -1 : 0;
	} else if (press && (ev->action == ACT_NAV_DOWN || ev->action == ACT_NAV_RIGHT)) {
		dir = 1;
	} else if (press && (ev->action == ACT_NAV_UP || ev->action == ACT_NAV_LEFT)) {
		dir = -1;
	}
	if (dir != 0) {
		station_step(dir);
		ui_dirty();
		return true;
	}
#if CONFIG_APRS_QSY
	if (ev->action == ACT_CONTEXT && ev->gesture == GESTURE_PRESS) {
		station_qsy();
		return true;
	}
#endif
	if (ev->action == ACT_SELECT && ev->gesture == GESTURE_PRESS) {
		ui_aprs_compose(station_call);
		return true;
	}
	return false;
}

static uint16_t station_count(void) { return 1; }
static void station_item(uint16_t i, ui_item_t *it)
{
	(void)i;
	str_copy(it->label, sizeof it->label, TR(WORD_MESSAGE, "Message"));
}
static const char *station_title(void) { return station_call; }
static void station_select(uint16_t i, bool hold)
{
	(void)i;
	(void)hold;
	ui_aprs_compose(station_call);
}

// A station heard with its own position (UI_EV_APRS_HEARD): its screen opens by itself on Home, as the Pop-up setting
// says, never over a menu, a frequency being typed, a transmission or a message banner. A pop-up up already shows the
// new station and starts its time again; the same station only refreshes it. The event only notes it: the next pass
// (ui_aprs_tick) opens it, outside the event handler's frame (the push draws a list's rows: docs/dev/stack-check.md)
#if CONFIG_APRS_QSY
static char popup_call[AX25_CALL_LEN + 4];
static uint16_t popup_s;	// a pop-up to open, for this long; 0 none

__attribute__((noinline)) void ui_aprs_heard(const char *call, bool direct)
{
	aprs_cfg_t c;
	ui_svc_aprs_cfg(&c);
	if (c.popup == APRS_POPUP_OFF || (c.popup == APRS_POPUP_DIRECT && !direct)) {
		return;
	}
	str_copy(popup_call, sizeof popup_call, call);
	popup_s = c.popup_s != 0 ? c.popup_s : APRS_POPUP_S_DEFAULT;
}

// Every UI pass: a pop-up to open, the pop-up's time, and the QSY's way back
__attribute__((noinline)) void ui_aprs_tick(uint32_t now_ms)
{
	if (popup_s != 0) {
		uint16_t secs = popup_s;
		popup_s = 0;
		bool showing = popup_until != 0 && ui_frame()->scr == SCR_APRS_STATION;
		if (showing && strcmp(popup_call, station_call) == 0) {
			ui_dirty();
		} else if (showing || (ui_depth() == 0 && !ui_st.entry && ui_view()->state != RS_TX &&
					ui_st.banner.kind != UI_BANNER_APRS_MESSAGE)) {
			str_copy(station_call, sizeof station_call, popup_call);
			if (!showing) {
				ui_push(SCR_APRS_STATION, 0);
			}
			uint32_t until = now_ms + (uint32_t)secs * 1000u;
			popup_until = until != 0 ? until : 1;
			ui_dirty();
			return;
		}
	}
	if (popup_until != 0) {
		if (ui_frame()->scr != SCR_APRS_STATION) {
			popup_until = 0;	// left some other way
		} else if ((int32_t)(now_ms - popup_until) >= 0) {
			popup_until = 0;
			ui_pop();
		}
	}
	ui_svc_aprs_qsy_track(ui_view());
}

// Back on Home after a QSY: where the radio was before it. False: no QSY to undo
bool ui_aprs_qsy_back(void)
{
	static char done[32];
	uint32_t hz;
	if (!ui_svc_aprs_qsy_undo(&hz)) {
		return false;
	}
	sbuf_t b;
	sb_init(&b, done, sizeof done);
	if (hz != 0) {
		sb_str(&b, TR(APRS_TOAST_BACK_TO, "Back to "));
		sb_freq(&b, hz);
	} else {
		sb_str(&b, TR(APRS_TOAST_BACK_TO_CHANNEL, "Back to the channel"));
	}
	ui_toast(done);
	return true;
}
#else
void ui_aprs_heard(const char *call, bool direct) { (void)call; (void)direct; }
void ui_aprs_tick(uint32_t now_ms) { (void)now_ms; }
bool ui_aprs_qsy_back(void) { return false; }
#endif

const ui_scr_def_t ui_aprs_station_screen = {
	.name = "aprs-station", .flags = SF_FIELD, .prefix = TRN(APRS_TITLE_HEARD, "Heard"),
	.count = station_count, .item = station_item, .select = station_select, .title = station_title,
	.key = station_key, .draw = station_draw,
};
#else
static uint16_t station_count(void) { return 1; }
static void station_item(uint16_t i, ui_item_t *it)
{
	(void)i;
	str_copy(it->label, sizeof it->label, TR(WORD_MESSAGE, "Message"));
}
static const char *station_title(void)
{
	static char call[AX25_CALL_LEN + 4];
	aprs_station_t s;
	if (!ui_svc_aprs_station((uint8_t)ui_frame()->arg, &s))
		return "";
	str_copy(call, sizeof call, s.call);
	return call;
}
static void station_select(uint16_t i, bool hold)
{
	(void)i; (void)hold;
	aprs_station_t s;
	if (ui_svc_aprs_station((uint8_t)ui_frame()->arg, &s))
		ui_aprs_compose(s.call);
}
void ui_aprs_heard(const char *call, bool direct) { (void)call; (void)direct; }
void ui_aprs_tick(uint32_t now_ms) { (void)now_ms; }
bool ui_aprs_qsy_back(void) { return false; }
const ui_scr_def_t ui_aprs_station_screen = {
	.name = "aprs-station", .flags = SF_LIST | SF_FIELD, .prefix = TRN(APRS_TITLE_HEARD, "Heard"),
	.hint = TRN(HINT_OK_CHOOSE_BACK, "OK choose  Back"),
	.count = station_count, .item = station_item, .select = station_select, .title = station_title,
};
#endif
