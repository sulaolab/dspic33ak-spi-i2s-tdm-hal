//===========================================================
// CLC10 50%-duty frame-sync generator -- see dspic33ak_spi_i2s_tdm_fs_clc.h.
//
// All register/PPS facts are transcribed from the dsPIC33AK512MPS512 Family Data Sheet
// (DS70005591C) and the device header -- nothing is guessed:
//   - CLC10 MODE=0b110 = "J-K Flip-Flop with R". Gate roles (DS Fig 28-3, table extraction,
//     HW-verified): Gate1=CLK, Gate2=J, Gate4=K, Gate3=R(async reset). Even gates carry the
//     data inputs, odd gates the controls. J=K=1 -> the FF TOGGLES on every Gate1 edge.
//   - CLC10SEL.DS1=0b110 selects "Virtual Pin 8 Output" (= RPV8) as Data1 (DS Table 28-2).
//   - RPV8 virtual output is written via RP137R (DS Table 12-20); _RP137R is its alias.
//   - PPS output code CLC10OUT = _RPOUT_CLC10OUT.
//   - RPORx pack four 7-bit RPnR fields (bits 0/8/16/24); RPORx holds RP[4x+1..4x+4];
//     registers are contiguous (RPOR0..), so &RPOR0 indexes the whole bank.
//
// A CLC gate with NO data selected outputs 0; setting that gate's GxPOL inverts it to a
// constant 1. So J and K are forced to 1 (G2POL/G4POL) without consuming an input, and R is
// pulsed to 1 (G3POL) at enable to reset the FF to a known state, then released to 0.
//===========================================================

#include "dspic33ak_spi_i2s_tdm_fs_clc.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// CLC10 + RPV8 exist on the AK512; other supported parts (AK128) have no CLC10 -> the
// 50%-FS-via-CLC feature is unavailable there and engage() reports _NO_FS_PIN.
#if (DSPIC33AK_SPI_I2S_TDM_DEVICE == DSPIC33AK_SPI_I2S_TDM_DEV_AK512)

#define FS_CLC_DS1_VIRTUAL_PIN_8   (0b110u)   // CLC10SEL.DS1 : Virtual Pin 8 (= RPV8)
#define FS_CLC_MODE_JK_FF_WITH_R   (0b110u)   // CLC10CON.MODE : J-K flip-flop with reset
#define FS_CLC_RPV8_RP             (137u)     // RPV8 == RP137R  (FRMSYNC marker -> CLC input)
#define FS_CLC_RP_PHYS_MAX         (128u)     // scan physical RP1..RP128 (RP129.. are virtual)

//===========================================================
// Variables  (ownership of the single CLC10)
//===========================================================
static bool           s_claimed;
static tdm_spi_inst_t s_owner;
static uint32_t       s_fs_rp;     // physical FS pin repointed to CLC10OUT (to restore on release)
static uint8_t        s_ss_code;   // the FRMSYNC (SSx) code that pin carried before (to restore)


//===========================================================
// Function Prototype (private)
//===========================================================
static void               fs_clc_pps_unlock(void);
static void               fs_clc_pps_lock(void);
static volatile uint32_t* fs_clc_rpor(uint32_t rp);
static uint32_t           fs_clc_rpor_pos(uint32_t rp);
static uint8_t            fs_clc_read_rp(uint32_t rp);
static void               fs_clc_write_rp(uint32_t rp, uint8_t code);
static uint32_t           fs_clc_find_rp_with_code(uint8_t code);
static void               fs_clc_configure_clc10(void);


//===========================================================
// Global Function
//===========================================================
dspic33ak_spi_i2s_tdm_fs_clc_result_t
    dspic33ak_spi_i2s_tdm_fs_clc_engage( tdm_spi_inst_t owner )
{
    uint8_t  ss_code;
    uint32_t fs_rp;

    if (s_claimed && (s_owner != owner))
    {
        return DSPIC33AK_SPI_I2S_TDM_FS_CLC_BUSY;
    }
    if (!dspic33ak_spi_i2s_tdm_hw_get_ss_pps_code(owner, &ss_code))
    {
        return DSPIC33AK_SPI_I2S_TDM_FS_CLC_NO_FS_PIN;   // instance has no FRMSYNC output
    }

    fs_rp = fs_clc_find_rp_with_code(ss_code);
    if (fs_rp != 0u)
    {
        // First engage: the board routed FRMSYNC -> this physical pin. Send FRMSYNC to RPV8
        // (CLC input) and repoint the external pin from FRMSYNC to CLC10OUT.
        fs_clc_pps_unlock();
        fs_clc_write_rp(FS_CLC_RPV8_RP, ss_code);            // FRMSYNC marker -> RPV8 (internal)
        fs_clc_write_rp(fs_rp, (uint8_t)_RPOUT_CLC10OUT);    // external FS pin <- CLC10OUT
        fs_clc_pps_lock();
    }
    else
    {
        // Re-engage (idempotent for the current owner): FRMSYNC is no longer on a physical
        // pin because a previous engage already repointed it to CLC10OUT. Find that pin and
        // keep it AS fs_rp -- otherwise s_fs_rp would stay 0 and release() would write rp=0.
        const uint32_t clc_rp = fs_clc_find_rp_with_code((uint8_t)_RPOUT_CLC10OUT);
        if (clc_rp == 0u)
        {
            return DSPIC33AK_SPI_I2S_TDM_FS_CLC_NO_FS_PIN;   // FS not on any physical pin
        }
        fs_rp = clc_rp;                                      // ensure release() restores SSx here
        fs_clc_pps_unlock();
        fs_clc_write_rp(FS_CLC_RPV8_RP, ss_code);            // keep the marker reaching RPV8
        fs_clc_pps_lock();
    }

    fs_clc_configure_clc10();
    s_fs_rp   = fs_rp;       // remembered so release() can restore SSx onto this pin
    s_ss_code = ss_code;
    s_owner   = owner;
    s_claimed = true;
    return DSPIC33AK_SPI_I2S_TDM_FS_CLC_OK;
}

void dspic33ak_spi_i2s_tdm_fs_clc_release( tdm_spi_inst_t owner )
{
    if (!s_claimed || (s_owner != owner))
    {
        return;
    }
    CLC10CONbits.ON = 0u;    // disable the flip-flop

    // Restore the external FS pin from CLC10OUT back to its original FRMSYNC (SSx) output, so
    // a runtime switch FS_50PCT -> stop -> FS_PULSE -> start leaves the pin driven by the SPI
    // directly again (RPV8 is an internal virtual pin; harmless to leave pointed at SSx).
    fs_clc_pps_unlock();
    fs_clc_write_rp(s_fs_rp, s_ss_code);
    fs_clc_pps_lock();

    s_claimed = false;
}


//===========================================================
// Local Function
//===========================================================

// PPS lock gate (RPCON.IOLOCK): self-contained, no gpio/pps HAL dependency.
static void fs_clc_pps_unlock(void) { RPCONbits.IOLOCK = 0u; }
static void fs_clc_pps_lock(void)   { RPCONbits.IOLOCK = 1u; }

// &RPOR0 indexes the contiguous RPORx bank; RPORx holds RP[4x+1..4x+4] in 7-bit fields.
static volatile uint32_t* fs_clc_rpor(uint32_t rp) { return (&RPOR0) + ((rp - 1u) / 4u); }
static uint32_t           fs_clc_rpor_pos(uint32_t rp) { return ((rp - 1u) % 4u) * 8u; }

// Read the 7-bit output-function code currently routed to physical pin RPn.
static uint8_t fs_clc_read_rp(uint32_t rp)
{
    return (uint8_t)((*fs_clc_rpor(rp) >> fs_clc_rpor_pos(rp)) & 0x7Fu);
}

// Write a 7-bit output-function code onto pin RPn (caller holds the PPS unlock).
static void fs_clc_write_rp(uint32_t rp, uint8_t code)
{
    volatile uint32_t *reg = fs_clc_rpor(rp);
    const uint32_t     pos = fs_clc_rpor_pos(rp);
    *reg = (*reg & ~(0x7FuL << pos)) | ((uint32_t)code << pos);
}

// Reverse-lookup: first PHYSICAL pin whose RPnR == code, or 0 if none.
static uint32_t fs_clc_find_rp_with_code(uint8_t code)
{
    for (uint32_t rp = 1u; rp <= FS_CLC_RP_PHYS_MAX; ++rp)
    {
        if (fs_clc_read_rp(rp) == code)
        {
            return rp;
        }
    }
    return 0u;
}

// CLC10 as a J-K flip-flop toggled by the RPV8 marker (gate roles: see file header).
static void fs_clc_configure_clc10(void)
{
    CLC10CON = 0u;                                 // ON=0 while configuring; all bits known-0
    CLC10SELbits.DS1   = FS_CLC_DS1_VIRTUAL_PIN_8; // Data1 = RPV8 (the FRMSYNC marker)
    CLC10GLS           = 0u;
    CLC10GLSbits.G1D1T = 1u;                       // Gate1 = Data1 (true) = CLK

    CLC10CONbits.MODE  = FS_CLC_MODE_JK_FF_WITH_R; // 0b110 : J-K flip-flop with R
    CLC10CONbits.G1POL = 0u;                       // CLK   : non-inverted (toggle on marker edge)
    CLC10CONbits.G2POL = 1u;                       // J = 1 (empty gate 0 -> inverted to 1)
    CLC10CONbits.G4POL = 1u;                       // K = 1 (empty gate 0 -> inverted to 1)
    CLC10CONbits.G3POL = 1u;                       // R = 1 (assert reset for a known initial Q)
    // LCPOL=0: FS starts Low after reset, so the first marker (frame start) drives it High
    // -> FS-High spans the first half-frame (active-high, matches the TDM FRMPOL convention).
    CLC10CONbits.LCPOL = 0u;
    CLC10CONbits.LCOE  = 1u;                        // drive CLC10OUT onto the (PPS-routed) pin
    CLC10CONbits.ON    = 1u;                        // enable: R still asserted -> FF held reset
    CLC10CONbits.G3POL = 0u;                        // release R: J=K=1 -> FF toggles each edge
}

#else  // device without CLC10

dspic33ak_spi_i2s_tdm_fs_clc_result_t
    dspic33ak_spi_i2s_tdm_fs_clc_engage( tdm_spi_inst_t owner )
{
    (void)owner;
    return DSPIC33AK_SPI_I2S_TDM_FS_CLC_NO_FS_PIN;   // no CLC10 on this part
}

void dspic33ak_spi_i2s_tdm_fs_clc_release( tdm_spi_inst_t owner )
{
    (void)owner;
}

#endif // device select
