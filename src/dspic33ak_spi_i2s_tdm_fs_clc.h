#ifndef DSPIC33AK_SPI_I2S_TDM_FS_CLC_H
#define DSPIC33AK_SPI_I2S_TDM_FS_CLC_H

//===========================================================
// CLC10 50%-duty frame-sync generator (silicon helper of the SPI/I2S/TDM HAL).
//
// For a TDM MASTER configured with FS_50PCT, the SPI emits a 1-BCLK half-frame marker
// (FRMSYPW=0, FRMCNT=slots/2) on its FRMSYNC (SSx) output. This module turns that marker
// into a ~50%-duty FS by toggling CLC10 (a J-K flip-flop, J=K=1) on every marker edge, and
// puts the result on the SAME external FS pin the board already routed -- so the
// application never deals with CLC/PPS. It is SELF-CONTAINED (xc.h only): no dependency on
// the gpio/pps HAL, so the transport HAL stays vendoring-portable.
//
// What it does on engage():
//   1. Resolve this instance's FRMSYNC PPS code (hw_get_ss_pps_code).
//   2. Reverse-scan the RPORx output registers to find the PHYSICAL pin the board routed
//      that FRMSYNC to (that pin IS the external FS). (On a restart it instead finds the
//      pin already routed to CLC10OUT and reuses it.)
//   3. Route FRMSYNC (SSx) internally to virtual pin RPV8 -- no jumper, no extra pad.
//   4. Configure CLC10 as a J-K flip-flop clocked by RPV8 (DS Fig 28-3 gate roles:
//      CLK=Gate1, J=Gate2, K=Gate4, R=Gate3), with a known initial state.
//   5. Repoint that external FS pin from FRMSYNC to CLC10OUT.
//
// RESOURCE OWNERSHIP: there is ONE CLC10. It is owned by the instance/clock-domain that
// engages it. A different instance trying to engage while it is owned gets _BUSY (the core
// maps that to ERR_CLC). Sharing ONE CLC10 FS across several co-clocked, co-format,
// phase-aligned instances (fan CLC10OUT to several FS pins) is a future extension tied to
// the multi-instance clock-domain work; today a second independent domain is rejected.
//
// DEVICE: implemented for parts with CLC10 + RPV8 (dsPIC33AK512MPS512). On other supported
// parts (no CLC10) engage() returns _NO_FS_PIN and the feature is unavailable there.
//===========================================================

#include "dspic33ak_spi_i2s_tdm_hw.h"   // tdm_spi_inst_t

typedef enum {
    DSPIC33AK_SPI_I2S_TDM_FS_CLC_OK = 0,
    DSPIC33AK_SPI_I2S_TDM_FS_CLC_BUSY,       // CLC10 already owned by a different instance/domain
    DSPIC33AK_SPI_I2S_TDM_FS_CLC_NO_FS_PIN,  // FS/FRMSYNC not on any physical pin (or no CLC10 on device)
} dspic33ak_spi_i2s_tdm_fs_clc_result_t;

// Engage the CLC10 50%-FS generator for `owner` (a TDM master). Idempotent for the current
// owner (also handles re-start after release). Returns _BUSY if another owner holds CLC10,
// _NO_FS_PIN if the FS pin can't be resolved / CLC10 absent. Call AFTER the board has
// routed FRMSYNC->FS pin (open()) and BEFORE enabling the SPI module.
dspic33ak_spi_i2s_tdm_fs_clc_result_t
    dspic33ak_spi_i2s_tdm_fs_clc_engage( tdm_spi_inst_t owner );

// Release CLC10 if `owner` holds it: disables the flip-flop AND restores the external FS pin
// from CLC10OUT back to its original FRMSYNC (SSx) output, so a runtime reconfigure
// (FS_50PCT -> stop -> FS_PULSE -> start) leaves the SPI driving the FS pin directly again.
// No-op if `owner` is not the current holder.
void dspic33ak_spi_i2s_tdm_fs_clc_release( tdm_spi_inst_t owner );

#endif // DSPIC33AK_SPI_I2S_TDM_FS_CLC_H
