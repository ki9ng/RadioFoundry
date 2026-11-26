// SPDX-License-Identifier: MIT
// Copyright (c) 2026 KI9NG
// The newer RF chip in QA4818 sockets (qa4818_v2.c; ki9ng fork). qa4818.c calls these when c->v2
#ifndef QA4818_V2_H
#define QA4818_V2_H

#include "qa4818.h"

void qa4818_v2_tune(qa4818_t *c, uint32_t hz);
void qa4818_v2_init_lo(qa4818_t *c, bool trx_v2);
void qa4818_v2_init_trx(qa4818_t *c, bool side_b);
void qa4818_v2_path(qa4818_t *c, bool tx);
void qa4818_v2_rx_on(qa4818_t *c, uint32_t hz);
void qa4818_v2_lo_on(qa4818_t *c);
void qa4818_v2_off(qa4818_t *c);
void qa4818_v2_out(qa4818_t *c, uint16_t value);
void qa4818_v2_rx_setup(qa4818_t *c, bool digital);
void qa4818_v2_gpio(qa4818_t *c, uint16_t old, uint16_t now, bool all);
uint16_t qa4818_v2_reg(qa4818_t *c, uint8_t reg, uint16_t val, bool write);
void qa4818_v2_status(qa4818_t *c, qa4818_status_t *st);

#endif
