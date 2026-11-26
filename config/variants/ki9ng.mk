# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The RadioFoundry Authors
# Copyright (c) 2026 KI9NG
## title: KI9NG
## summary: AT-D578UV (AT32) for satellites and APRS: Doppler tracking, rigctld, KISS. No DMR trunking or zone sharing.
##
## The ki9ng fork's preset (make BOARD=d578uva VARIANT=ki9ng). Unlike the release variants it also turns features
## ON, which works because a variant is read before the board's config.mk and both use ?=. The command line still
## wins. Sized for d578uva's 128 KB of SRAM (memory.ld); RAM measured 2026-11-26 on branch d578-ki9ng:
##   default d578uva                                      130,344 B used, 456 B free
##   this variant                                         126,832 B used, 3,968 B free

# Everything the amateur variant leaves out (privacy, TMS, ARS, LRRP, control station, SNMP)
include config/variants/amateur.mk

# --- Off: what this station doesn't use, for the RAM below ---
# DMR Tier III trunking: no trunked DMR system here (about 5.7 KB of RAM)
CONFIG_DMR_TRUNK ?= n
# Zone sharing between radios (about 4.8 KB)
CONFIG_ZONE_SHARE ?= n
# The TCP services this station doesn't use: the web pages, the web control and APRS-IS (the phone or the PC is
# the iGate)
CONFIG_NET_HTTP ?= n
CONFIG_NET_WEBCTL ?= n
CONFIG_NET_APRSIS ?= n

# --- On: off by default on d578uva for RAM ---
# Satellite tracking with Doppler correction (about 1.2 KB)
CONFIG_SATELLITE_TLE ?= y
# TCP on the USB network adapter: rigctld (gpredict) and KISS over TCP follow it (about 7 KB with the two)
CONFIG_NET_TCP ?= y
