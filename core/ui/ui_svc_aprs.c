// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// The UI's APRS services over core/aprs (ui_svc_aprs.h)
#include "config.h"
#if CONFIG_APRS
#include "ui_svc_aprs.h"
#include "aprs/aprs_fmt.h"
#include "ui_svc.h"
#include "board.h"
#include "storage/storage.h"
#include "hal_time.h"
#include <string.h>

bool ui_svc_aprs_view(aprs_view_t *v)
{
	aprs_get_view(v);
	return true;
}

void ui_svc_aprs_cfg(aprs_cfg_t *c)
{
	aprs_get_cfg(c);
}

// The path and the SSID also live in the codeplug's APRS record (aprs.md 11): written there by the housekeeping task
static void codeplug_job(void *ctx)
{
	(void)ctx;
	(void)aprs_to_codeplug();
}

void ui_svc_aprs_set_cfg(const aprs_cfg_t *c)
{
	aprs_cfg_t old;
	aprs_get_cfg(&old);
	aprs_set_cfg(c, hal_millis());
	if (c->path != old.path || c->ssid != old.ssid) {
		(void)storage_job_post(codeplug_job, NULL);	// busy with another job: the codeplug keeps its own
	}
}

bool ui_svc_aprs_send_now(void)
{
	return aprs_send_now();
}

bool ui_svc_aprs_send_message(const char *to, const char *text, uint8_t flags)
{
	return aprs_send_message_opts(to, text, flags);
}

#if CONFIG_APRS_RX
uint8_t ui_svc_aprs_msg_count(void) { return aprs_msg_count(); }
bool ui_svc_aprs_msg(uint8_t i, aprs_msg_t *out, bool read)
{
	return read ? aprs_msg(i, out) : aprs_msg_peek(i, out);
}
bool ui_svc_aprs_msg_delete(uint8_t i) { return aprs_msg_delete(i); }
#endif

#if CONFIG_APRS_RX
// In a frame of its own (noinline): the APRS view and an event are large, and inlined into the UI task's poll they
// would sit under every path the task takes (docs/dev/stack-check.md)
__attribute__((noinline)) void ui_svc_aprs_msg_poll(void)
{
	static uint32_t seen;
	aprs_view_t v;
	aprs_get_view(&v);
	ui_event_t ev;
#if CONFIG_APRS_QSY
	static uint32_t pos_seen;
	if (v.pos_serial != pos_seen) {	// a station heard with its position: the pop-up's (ui_aprs_heard)
		pos_seen = v.pos_serial;
		memset(&ev, 0, sizeof ev);
		ev.kind = UI_EV_APRS_HEARD;
		ev.ok = v.pos_direct;
		strncpy(ev.name, v.pos_call, sizeof ev.name - 1);
		ui_notify(&ev);
	}
#endif
	if (v.msg_serial == seen)
		return;
	seen = v.msg_serial;
	aprs_msg_t m;
	if (!aprs_msg_peek(0, &m))
		return;
	memset(&ev, 0, sizeof ev);
	ev.kind = UI_EV_APRS_MESSAGE;
	strncpy(ev.name, m.from, sizeof ev.name - 1);
	strncpy(ev.text, m.text, sizeof ev.text - 1);
	ui_notify(&ev);
}
#else
void ui_svc_aprs_msg_poll(void) { }
#endif

#if CONFIG_APRS_RX
uint8_t ui_svc_aprs_station_count(void)
{
	return aprs_station_count();
}

bool ui_svc_aprs_station(uint8_t i, aprs_station_t *out)
{
	return aprs_station(i, out);
}

bool ui_svc_aprs_station_find(const char *call, aprs_station_t *out)
{
	return aprs_station_find(call, out);
}

bool ui_svc_aprs_range(const aprs_station_t *s, uint32_t *m, uint16_t *deg)
{
	int32_t lat, lon;
	uint32_t err_m;
	bool dmr_gps;
	if (!s->has_pos || !position_now(&lat, &lon, &err_m, &dmr_gps, hal_millis())) {
		return false;
	}
	*m = aprs_distance_m(lat, lon, s->lat, s->lon);
	*deg = aprs_bearing_deg(lat, lon, s->lat, s->lon);
	return true;
}
#endif
#if CONFIG_APRS_QSY
// QSY: the station's frequency as a whole channel, its split the one it gives or the band plan's
uint8_t ui_svc_aprs_qsy(const radio_view_t *v, const aprs_freq_t *f)
{
	if (f->hz == 0) {
		return UI_QSY_NONE;
	}
	if (v->scan == 3) {
		return UI_QSY_DUAL;	// dual watch's channels are the user's two picks: not changed under it
	}
	int32_t off = f->toff ? 0 : f->offset_10k == APRS_FREQ_OFFSET_STD ? aprs_freq_std_offset_hz(f->hz) :
		(int32_t)f->offset_10k * 10000;
	ui_vfo_tune_t t;
	memset(&t, 0, sizeof t);
	t.rx_hz = f->hz;
	t.tx_hz = (uint32_t)((int32_t)f->hz + off);
	t.narrow = f->narrow;
	if (!f->toff && f->tone_kind != APRS_FREQ_TONE_NONE) {
		t.tx_tone.type = f->tone_kind == APRS_FREQ_TONE_DCS ? TONE_DCS : TONE_CTCSS;
		t.tx_tone.value = f->tone;
		if (f->tone_kind != APRS_FREQ_TONE_ENC) {
			t.rx_tone = t.tx_tone;	// 'C' and 'D' on both; 'T' on transmit only
		}
	}
	return ui_svc_vfo_tune(v, &t) ? UI_QSY_OK : UI_QSY_BAND;
}

bool ui_svc_aprs_qsy_undo(uint32_t *hz) { return ui_svc_vfo_back(hz); }
void ui_svc_aprs_qsy_track(const radio_view_t *v) { ui_svc_vfo_tune_track(v); }
#endif

#endif
