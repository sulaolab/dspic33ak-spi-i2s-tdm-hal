
//===========================================================
// INCLUDES
//===========================================================
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "dspic33ak_dma.h"               // DMA channel config/enable
#include "dspic33ak_spi_i2s_tdm_reg.h"   // SPI framed-mode register masks (XC-DSC-free)
#include "dspic33ak_spi_i2s_tdm_hw.h"


//===========================================================
// Definition
//===========================================================

// ---- Debug master switch (silicon layer) ----
// Default OFF: compiles with no printf dependency. Define ENA_TDM_DBG (here or via the
// build configuration) to restore the DMA-config error printf (TDM_DBG_PRINTF -> printf
// when ON, no-op when OFF).
//#define ENA_TDM_DBG

#if defined(ENA_TDM_DBG)
  #include <stdio.h>                 // printf (debug build only; DMA-config error prints)
  #define TDM_DBG_PRINTF(...)   printf(__VA_ARGS__)
#else
  #define TDM_DBG_PRINTF(...)   ((void)0)
#endif //defined(ENA_TDM_DBG)

#define PRIO_TDM_DMA              (4)


//===========================================================
// Enum & Struct typedef (silicon facts)
//===========================================================
typedef struct {
    volatile uint32_t *ie_reg;
    uint32_t           ie_mask;
    volatile uint32_t *if_reg;
    uint32_t           if_mask;
} tdm_cpu_irq_t;

typedef struct {
    volatile void     *spi_buf;     // &SPIxBUF
    volatile uint32_t *con1;        // &SPIxCON1
    volatile uint32_t *stat;        // &SPIxSTAT (sticky error/health flags: SPIROV/SPITUR/FRMERR)
    volatile uint32_t *brg;         // &SPIxBRG
    volatile uint32_t *imsk;        // &SPIxIMSK
    uint8_t            rx_trigger;  // DMAxSELbits.CHSEL for SPIxRX (data sheet Table 13-2)
    uint8_t            tx_trigger;  // DMAxSELbits.CHSEL for SPIxTX (data sheet Table 13-2)
    tdm_cpu_irq_t      rx_irq;      // CPU IEC/IFS bit for SPIxRX interrupt
    tdm_cpu_irq_t      tx_irq;      // CPU IEC/IFS bit for SPIxTX interrupt
} tdm_spi_dev_t;


//===========================================================
// Function Prototype (private)
//===========================================================
static bool        hw_inst_valid( tdm_spi_inst_t inst );
static bool        hw_dma_config_channel( tdm_spi_inst_t inst, uint8_t dma_ch, int32_t *buffer, uint32_t count, bool is_rx );
static void        hw_spi_irq_enable( tdm_spi_inst_t inst, bool enable );
static void        hw_spi_irq_disable_clear( tdm_spi_inst_t inst );


//===========================================================
// Variables
//===========================================================

// DEVICE FACTS - data sheet transcription only (no driver/allocation values).
// Indexed by tdm_spi_inst_t. The driver's channel/buffer ALLOCATION lives in the
// transport core (s_spi_legs[]); the core passes the relevant fields down here.
static const tdm_spi_dev_t s_spi_dev[] =
{
#if DSPIC33AK_SPI_I2S_TDM_DEVICE == DSPIC33AK_SPI_I2S_TDM_DEV_AK512
    // CHSEL[7:0] values below are transcribed from the data sheet: search for
    // "Table 13-2. DMA Channel Trigger Sources" in the dsPIC33AK512MPS512
    // Family Data Sheet (DS70005591A) -> SPInRx / SPInTx rows.
    [TDM_SPI1] =
    {
        (volatile void *)&SPI1BUF, &SPI1CON1, &SPI1STAT, &SPI1BRG, &SPI1IMSK, 0x6u, 0x7u,
        { &IEC2, _IEC2_SPI1RXIE_MASK, &IFS2, _IFS2_SPI1RXIF_MASK },
        { &IEC2, _IEC2_SPI1TXIE_MASK, &IFS2, _IFS2_SPI1TXIF_MASK },
    },
    [TDM_SPI2] =
    {
        (volatile void *)&SPI2BUF, &SPI2CON1, &SPI2STAT, &SPI2BRG, &SPI2IMSK, 0x8u, 0x9u,
        { &IEC2, _IEC2_SPI2RXIE_MASK, &IFS2, _IFS2_SPI2RXIF_MASK },
        { &IEC2, _IEC2_SPI2TXIE_MASK, &IFS2, _IFS2_SPI2TXIF_MASK },
    },
    [TDM_SPI3] =
    {
        (volatile void *)&SPI3BUF, &SPI3CON1, &SPI3STAT, &SPI3BRG, &SPI3IMSK, 0xAu, 0xBu,
        { &IEC2, _IEC2_SPI3RXIE_MASK, &IFS2, _IFS2_SPI3RXIF_MASK },
        { &IEC2, _IEC2_SPI3TXIE_MASK, &IFS2, _IFS2_SPI3TXIF_MASK },
    },
    [TDM_SPI4] =
    {
        (volatile void *)&SPI4BUF, &SPI4CON1, &SPI4STAT, &SPI4BRG, &SPI4IMSK, 0xCu, 0xDu,
        { &IEC2, _IEC2_SPI4RXIE_MASK, &IFS2, _IFS2_SPI4RXIF_MASK },
        { &IEC2, _IEC2_SPI4TXIE_MASK, &IFS2, _IFS2_SPI4TXIF_MASK },
    },
#elif DSPIC33AK_SPI_I2S_TDM_DEVICE == DSPIC33AK_SPI_I2S_TDM_DEV_AK128
    // CHSEL[7:0] values below are transcribed from the data sheet: search for
    // "Table 13-2. DMA Channel Trigger Sources" in the dsPIC33AK128MC106
    // Family Data Sheet (DS70005539C) -> SPInRx / SPInTx rows.
    // (SPI1-3 only; CHSEL 0xC-0xE are Reserved -> no SPI4 on this device.)
    // SPI1 RX/TX CPU IRQ bits straddle IEC1/IEC2 and IFS1/IFS2 on AK128.
    [TDM_SPI1] =
    {
        (volatile void *)&SPI1BUF, &SPI1CON1, &SPI1STAT, &SPI1BRG, &SPI1IMSK, 0x6u, 0x7u,
        { &IEC1, _IEC1_SPI1RXIE_MASK, &IFS1, _IFS1_SPI1RXIF_MASK },
        { &IEC2, _IEC2_SPI1TXIE_MASK, &IFS2, _IFS2_SPI1TXIF_MASK },
    },
    [TDM_SPI2] =
    {
        (volatile void *)&SPI2BUF, &SPI2CON1, &SPI2STAT, &SPI2BRG, &SPI2IMSK, 0x8u, 0x9u,
        { &IEC2, _IEC2_SPI2RXIE_MASK, &IFS2, _IFS2_SPI2RXIF_MASK },
        { &IEC2, _IEC2_SPI2TXIE_MASK, &IFS2, _IFS2_SPI2TXIF_MASK },
    },
    [TDM_SPI3] =
    {
        (volatile void *)&SPI3BUF, &SPI3CON1, &SPI3STAT, &SPI3BRG, &SPI3IMSK, 0xAu, 0xBu,
        { &IEC2, _IEC2_SPI3RXIE_MASK, &IFS2, _IFS2_SPI3RXIF_MASK },
        { &IEC2, _IEC2_SPI3TXIE_MASK, &IFS2, _IFS2_SPI3TXIF_MASK },
    },
#endif
};


//===========================================================
// Global Function (silicon operations)
//===========================================================

/*
 * Program SPI framed-mode registers for one physical SPI instance.
 *
 * Writes SPIxCON1/SPIxBRG from the already-validated config and leaves DMA trigger
 * events disabled. Audio mode is intentionally off; the HAL implements I2S/TDM using
 * framed SPI plus DMA.
 */
void dspic33ak_spi_i2s_tdm_hw_apply_config( tdm_spi_inst_t inst,
                                            const dspic33ak_spi_i2s_tdm_config_t* cfg )
{
    if( !hw_inst_valid( inst ) || ( cfg == NULL ) )
    {
        return;
    }

    const tdm_spi_dev_t *dev = &s_spi_dev[inst];
    volatile uint32_t *con1 = dev->con1;

    *con1 = 0;    // just in case.
                  // note: SPI frame-sync mode, NOT Audio mode.
                  // Implement I2S/TDM with standard SPI + DMA (AUDEN = 0).

    dspic33ak_spi_i2s_tdm_reg_clear(con1, DSPIC33AK_SPI_I2S_TDM_CON1_AUDEN);    // AUDEN=0 : Audio mode off
    dspic33ak_spi_i2s_tdm_reg_set  (con1, DSPIC33AK_SPI_I2S_TDM_CON1_FRMEN);    // FRMEN=1 : framed SPI (SSx = FSYNC)

    // FS waveform shape -> FRMSYPW (pulse width) + fs_words (FRMCNT cadence). The public API
    // is the INTENT (fs_shape); the mapping to silicon lives here:
    //   FS_PULSE          : FRMSYPW=0 (1-BCLK short sync), one pulse per frame (fs_words=slots).
    //   FS_50PCT + I2S    : FRMSYPW=1 (a one-word pulse IS 50% of a 2-word frame), per frame.
    //   FS_50PCT + TDM    : FRMSYPW=0 + a HALF-frame marker (fs_words=slots/2); CLC10 toggles
    //                       it into a 50%-duty FS on the FS pin (engaged by the core, master
    //                       only). The DMA/buffer geometry stays sized by slots_per_fs.
    const bool fs_50pct  = ( cfg->fs_shape == DSPIC33AK_SPI_I2S_TDM_FS_50PCT );
    const bool is_i2s    = ( cfg->format   == DSPIC33AK_SPI_I2S_TDM_FORMAT_I2S );
    const bool is_master = ( cfg->clock_role     == DSPIC33AK_SPI_I2S_TDM_CLOCK_MASTER );
    bool    frmsypw;
    uint8_t fs_words;
    if( fs_50pct && is_i2s )
    {
        frmsypw  = true;                       // one word wide == 50% of the 2-word I2S frame
        fs_words = cfg->slots_per_fs;
    }
    else if( fs_50pct && is_master )            // FS_50PCT + TDM MASTER: half-frame marker for CLC10
    {
        frmsypw  = false;                      // 1-BCLK marker (CLC10 makes the 50% duty)
        fs_words = (uint8_t)(cfg->slots_per_fs / 2u);
    }
    else                                        // FS_PULSE (any), or a TDM FS_50PCT SLAVE.
    {                                           // (An I2S FS_50PCT leg -- master OR slave -- took
        frmsypw  = false;                      //  the first branch: FRMSYPW=1. A TDM slave receives
        fs_words = cfg->slots_per_fs;          //  FS as an input; the CLC 50% marker is master-only,
    }                                           //  so no half-frame FRMCNT here.)
    dspic33ak_spi_i2s_tdm_reg_set_or_clear(con1, DSPIC33AK_SPI_I2S_TDM_CON1_FRMSYPW, frmsypw);

    // IGNROV and IGNTUR are BOTH HARD-FORCED to 1 (not caller-selectable -- the config struct
    // deliberately omits both). If a FIFO flag becomes a critical-stop condition, it can suspend
    // the SPI leg and turn a primary DMA service failure into a permanent callback stall. Keeping
    // IGNROV/IGNTUR set is therefore continuity/secondary-fault containment, not an assertion that
    // lost data is benign. The primary cause is observed directly as DMAxSTAT.OVERRUN (dov=); the
    // downstream SPIROV/SPITUR and independent FRMERR effects remain visible as rov=/tur=/frm=.
    dspic33ak_spi_i2s_tdm_reg_set_or_clear(con1, DSPIC33AK_SPI_I2S_TDM_CON1_IGNROV, true);
    dspic33ak_spi_i2s_tdm_reg_set_or_clear(con1, DSPIC33AK_SPI_I2S_TDM_CON1_IGNTUR, true);

    // DISSDI/DISSDO/DISSCK left 0 = pins controlled by the module (already 0 after clear).

    // MODE32=1, MODE16=0 -> 32-bit.
    //   MODE32 MODE16 AUDEN  Communication
    //   1      x      0      32-bit
    //   0      1      0      16-bit
    //   0      0      0      8-bit
    //   (AUDEN=1 combinations select Audio mode; unused here.)
    dspic33ak_spi_i2s_tdm_reg_set_or_clear(con1, DSPIC33AK_SPI_I2S_TDM_CON1_MODE32, cfg->word_bits == 32u);  // MODE16 stays 0
    dspic33ak_spi_i2s_tdm_reg_set_or_clear(con1, DSPIC33AK_SPI_I2S_TDM_CON1_MCLKEN, cfg->mclk_enable);       // 1: CLKGEN9 / 0: std peripheral clock

    if( cfg->clock_role == DSPIC33AK_SPI_I2S_TDM_CLOCK_MASTER )
    {
        dspic33ak_spi_i2s_tdm_reg_set  (con1, DSPIC33AK_SPI_I2S_TDM_CON1_MSTEN);    // MSTEN=1 : Host
        dspic33ak_spi_i2s_tdm_reg_clear(con1, DSPIC33AK_SPI_I2S_TDM_CON1_FRMSYNC);  // FRMSYNC=0 : FS output (host)
    }
    else
    {
        // MSTEN=0 : Client (already 0).  FRMSYNC=1 : FS input (client)
        dspic33ak_spi_i2s_tdm_reg_set  (con1, DSPIC33AK_SPI_I2S_TDM_CON1_FRMSYNC);
    }

    // FRMPOL: 1 = FS/CS active-high, 0 = active-low. Convention: I2S active-low,
    // TDM active-high.
    dspic33ak_spi_i2s_tdm_reg_set_or_clear( con1, DSPIC33AK_SPI_I2S_TDM_CON1_FRMPOL,
                                            cfg->format != DSPIC33AK_SPI_I2S_TDM_FORMAT_I2S );

    // FRMCNT: FS pulse every N words (N = fs_words computed above). Encoding = log2(N):
    //   000 each word / 001 every 2 / 010 every 4 / 011 every 8 / 100 every 16 / 101 every 32.
    // So per-frame: I2S(2)->1, TDM4->2, TDM8->3, TDM16->4, TDM32->5; the FS_50PCT+TDM
    // half-frame marker halves N (TDM8->every 4 words->2, etc.). Defaults to TDM8 framing.
    uint32_t frmcnt;
    switch( fs_words )
    {
    case 1u:  frmcnt = 0u; break;
    case 2u:  frmcnt = 1u; break;
    case 4u:  frmcnt = 2u; break;
    case 8u:  frmcnt = 3u; break;
    case 16u: frmcnt = 4u; break;
    case 32u: frmcnt = 5u; break;
    default:  frmcnt = 3u; break;
    }
    dspic33ak_spi_i2s_tdm_reg_write_field( con1, DSPIC33AK_SPI_I2S_TDM_CON1_FRMCNT_MASK,
                                           DSPIC33AK_SPI_I2S_TDM_CON1_FRMCNT_POS, frmcnt );

    // SPIFE: 0 = FS edge precedes first BCLK (1-bit delayed) / 1 = coincides (no delay)
    dspic33ak_spi_i2s_tdm_reg_set_or_clear(con1, DSPIC33AK_SPI_I2S_TDM_CON1_SPIFE, cfg->fs_coincides_first_bclk);  // 1: no delay / 0: 1-bit delayed

    // ---- BCLK phase: CKP / CKE ----
    // (see Figure 23-9 "SPI Master Mode Operation in 8-bit Mode" in DS61106G.)
    //   CKP (BCLK polarity / idle level):
    //     0: idle Low , active High -> Idle->Active = rising,  Active->Idle = falling
    //     1: idle High, active Low  -> Idle->Active = falling, Active->Idle = rising
    //   CKE (transmit edge):
    //     0: transmit on Idle->Active edge
    //     1: transmit on Active->Idle edge
    //
    //     CKP CKE  Clock idle  Data changes on  Sample on
    //     0   0    Low         Rising           Falling
    //     0   1    Low         Falling          Rising
    //     1   0    High        Falling          Rising
    //     1   1    High        Rising           Falling
    //
    //   Typical I2S (Philips): data changes on BCLK falling, sampled on rising.
    //   This driver uses CKP=1, CKE=0 (idle High; data changes on the falling edge).
    dspic33ak_spi_i2s_tdm_reg_set_or_clear(con1, DSPIC33AK_SPI_I2S_TDM_CON1_CKP, cfg->bclk_idle_high);                 // CKP : 1=idle High
    dspic33ak_spi_i2s_tdm_reg_set_or_clear(con1, DSPIC33AK_SPI_I2S_TDM_CON1_CKE, cfg->bclk_change_on_active_to_idle);  // CKE : 1=transmit on active->idle

    // baud rate (ignored when Client / MSTEN=0)
    *dev->brg = cfg->brg;

    // Keep DMA-trigger events disabled until every SPI instance has its registers
    // programmed. The caller enables events explicitly after all register config.
    dspic33ak_spi_i2s_tdm_hw_dma_trigger_enable( inst, false );

    // SPI CPU interrupts are unused (DMA consumes the peripheral status flags).
    hw_spi_irq_disable_clear( inst );
}


/*
 * Configure + enable both DMA channels for one SPI instance.
 *
 * RX is configured first, then TX. A false return means the caller must roll back all
 * TDM DMA/SPI state because the channels may be only partially armed.
 */
bool dspic33ak_spi_i2s_tdm_hw_dma_config( tdm_spi_inst_t inst,
                                          uint8_t  rx_dma_ch,
                                          uint8_t  tx_dma_ch,
                                          int32_t* rx_buffer,
                                          int32_t* tx_buffer,
                                          uint32_t buffer_word_count )
{
    if( !hw_inst_valid( inst ) )
    {
        return false;
    }
    if( !hw_dma_config_channel( inst, rx_dma_ch, rx_buffer, buffer_word_count, true ) )
    {
        return false;
    }
    return hw_dma_config_channel( inst, tx_dma_ch, tx_buffer, buffer_word_count, false );
}


/*
 * Enable or disable SPI interrupt events used as DMA triggers.
 *
 * These are SPIxIMSK event enables, not CPU interrupt enables. The caller turns them
 * on only after every SPI instance is programmed; stop()/rollback turns them off.
 */
void dspic33ak_spi_i2s_tdm_hw_dma_trigger_enable( tdm_spi_inst_t inst, bool enable )
{
    if( !hw_inst_valid( inst ) )
    {
        return;
    }

    const tdm_spi_dev_t *dev = &s_spi_dev[inst];
    const uint32_t mask = DSPIC33AK_SPI_I2S_TDM_IMSK_SPIRBFEN |
                          DSPIC33AK_SPI_I2S_TDM_IMSK_SPITBEN;

    dspic33ak_spi_i2s_tdm_reg_set_or_clear( dev->imsk, mask, enable );
}


/*
 * Enable or disable one SPI module (SPIxCON1.ON only).
 *
 * The caller is responsible for ordering follower/timing legs so shared block timing
 * starts and stops cleanly.
 */
void dspic33ak_spi_i2s_tdm_hw_module_enable( tdm_spi_inst_t inst, bool enable )
{
    if( !hw_inst_valid( inst ) )
    {
        return;
    }

    const tdm_spi_dev_t *dev = &s_spi_dev[inst];
    dspic33ak_spi_i2s_tdm_reg_set_or_clear( dev->con1, DSPIC33AK_SPI_I2S_TDM_CON1_ON, enable );
}


/*
 * Clear CPU RX/TX interrupt flags for one SPI instance.
 *
 * Used by stop()/rollback even though the CPU interrupts are disabled, preventing
 * stale peripheral flags from leaking into later debugging or future starts.
 */
void dspic33ak_spi_i2s_tdm_hw_irq_clear_flags( tdm_spi_inst_t inst )
{
    if( !hw_inst_valid( inst ) )
    {
        return;
    }

    const tdm_spi_dev_t *dev = &s_spi_dev[inst];

    dspic33ak_spi_i2s_tdm_reg_clear( dev->rx_irq.if_reg, dev->rx_irq.if_mask );
    dspic33ak_spi_i2s_tdm_reg_clear( dev->tx_irq.if_reg, dev->tx_irq.if_mask );
}


/*
 * Soft-stop one SPI instance.
 *
 * Disables DMA-trigger event generation, masks CPU SPI IRQs, and clears the SPI ON
 * bit. It does not alter PPS/CLC routing or any saved configuration.
 */
void dspic33ak_spi_i2s_tdm_hw_soft_stop( tdm_spi_inst_t inst )
{
    // Stop SPI interrupt event generation used as DMA trigger source.
    dspic33ak_spi_i2s_tdm_hw_dma_trigger_enable( inst, false );

    // SPI CPU interrupts are unused, but mask them during teardown as well.
    hw_spi_irq_enable( inst, false );

    dspic33ak_spi_i2s_tdm_hw_module_enable( inst, false );
}


/*
 * Sample this instance's SPI framed-transport health flags (SPIxSTAT) once per completed RX
 * block, and ack the ones that are software-clearable. The HAL owns these sticky bits once this
 * is called: other code reading raw SPIxSTAT afterward will see them already acked.
 *
 * SPIROV and FRMERR are R/C/HS (software-clearable) -- acked here. SPITUR is R/HSC (hardware
 * self-clearing on SPIEN=0, NOT software-clearable) and reflects a live/dynamic underrun
 * condition, so it is only OBSERVED, never written. Returns the full observed mask (all three
 * bits as read, before any clear) -- callers get SPITUR's live state either way.
 */
uint32_t dspic33ak_spi_i2s_tdm_hw_sample_ack_errflags( tdm_spi_inst_t inst )
{
    if( !hw_inst_valid( inst ) )
    {
        return 0u;
    }
    volatile uint32_t *stat = s_spi_dev[inst].stat;
    const uint32_t status = *stat;
    const uint32_t observed = status
                        & ( DSPIC33AK_SPI_I2S_TDM_STAT_SPIROV
                          | DSPIC33AK_SPI_I2S_TDM_STAT_SPITUR
                          | DSPIC33AK_SPI_I2S_TDM_STAT_FRMERR );
    const uint32_t sw_clearable_mask = DSPIC33AK_SPI_I2S_TDM_STAT_SPIROV
                                      | DSPIC33AK_SPI_I2S_TDM_STAT_FRMERR;
    const uint32_t clearable = observed & sw_clearable_mask;
    if( clearable != 0u )
    {
        // W0C-safe ack: write only the software-clearable mask with the bits we just
        // observed cleared to 0. Never replay the stale `status` snapshot -- doing so
        // would risk silently clearing a flag hardware set between the read above and
        // this write (a torn write against sticky HW flags).
        *stat = sw_clearable_mask & ~clearable;
    }
    return observed;
}


/*
 * PPS output-function code for one SPI instance's frame-sync (SSx).
 *
 * Data-sheet fact, device-#ifdef'd: each arm compiles only where the header defines that
 * instance's SSx output (_RPOUT_SS<n>). The CLC10 50%-FS module uses this to reverse-scan
 * the RPORx registers for the physical pin the board routed FS/SS to.
 */
bool dspic33ak_spi_i2s_tdm_hw_get_ss_pps_code( tdm_spi_inst_t inst, uint8_t* code )
{
    if( code == NULL )
    {
        return false;
    }
    switch( inst )
    {
#ifdef _RPOUT_SS1
    case TDM_SPI1: *code = (uint8_t)_RPOUT_SS1; return true;
#endif
#ifdef _RPOUT_SS2
    case TDM_SPI2: *code = (uint8_t)_RPOUT_SS2; return true;
#endif
#ifdef _RPOUT_SS3
    case TDM_SPI3: *code = (uint8_t)_RPOUT_SS3; return true;
#endif
#if (DSPIC33AK_SPI_I2S_TDM_DEVICE == DSPIC33AK_SPI_I2S_TDM_DEV_AK512) && defined(_RPOUT_SS4)
    case TDM_SPI4: *code = (uint8_t)_RPOUT_SS4; return true;
#endif
    default: break;
    }
    return false;
}


//===========================================================
// Local Function
//===========================================================

/*
 * True if inst is a silicon SPI instance present on the built device.
 */
static bool hw_inst_valid( tdm_spi_inst_t inst )
{
    return ( (unsigned)inst < (unsigned)TDM_SPI_INST_COUNT );
}


/*
 * Configure and enable one RX or TX DMA channel for a SPI instance.
 *
 * The caller supplies the DMA channel and buffer; the silicon facts table supplies
 * SPIxBUF and the CHSEL trigger number. RX copies SPIxBUF into the ping-pong buffer,
 * while TX copies the ping-pong buffer into SPIxBUF.
 */
static bool hw_dma_config_channel( tdm_spi_inst_t inst, uint8_t dma_ch, int32_t *buffer, uint32_t count, bool is_rx )
{
    if( !hw_inst_valid( inst ) )
    {
        return false;
    }

    const tdm_spi_dev_t *dev = &s_spi_dev[inst];

    const dspic33ak_dma_channel_cfg_t cfg =
    {
        .src   = is_rx ? dev->spi_buf : (volatile void *)buffer,  // RX: SPIxBUF / TX: buffer
        .dst   = is_rx ? (volatile void *)buffer : dev->spi_buf,  // RX: buffer / TX: SPIxBUF
        .count = count,

        .src_mode = is_rx ? DSPIC33AK_DMA_ADDR_FIXED     : DSPIC33AK_DMA_ADDR_INCREMENT,
        .dst_mode = is_rx ? DSPIC33AK_DMA_ADDR_INCREMENT : DSPIC33AK_DMA_ADDR_FIXED,
        .size     = DSPIC33AK_DMA_SIZE_WORD,             // 32-bit
        .tr_mode  = DSPIC33AK_DMA_TRMODE_REPEAT_ONESHOT,

        .reload_count = true,
        .reload_src   = !is_rx,   // TX reloads src
        .reload_dst   =  is_rx,   // RX reloads dst

        .half_int_en  = true,
        .done_int_en  = true,

        .trigger_sel  = is_rx ? dev->rx_trigger : dev->tx_trigger,

        .irq_priority_set = true,
        .irq_priority     = PRIO_TDM_DMA,
        // RX raises the per-instance block ISR (the block-timing master); TX is
        // interrupt-less -- fire-and-forget ping-pong with auto-reload needs no ISR, so
        // the CPU IRQ is enabled on the RX channel only. (No TX "secondary" handler.)
        .irq_enable       = is_rx,
    };

    if( !dspic33ak_dma_channel_config(dma_ch, &cfg) )
    {
        TDM_DBG_PRINTF(" ERROR: DMA ch%u config failed (DMA not ready?).\n", (unsigned)dma_ch);
        return false;
    }
    if( !dspic33ak_dma_channel_enable(dma_ch, true) )
    {
        TDM_DBG_PRINTF(" ERROR: DMA ch%u enable failed (DMA not ready?).\n", (unsigned)dma_ch);
        return false;
    }
    return true;
}


/*
 * Enable or disable CPU RX/TX interrupts for one SPI instance.
 *
 * The transport normally keeps these disabled because DMA handles SPI events, but the
 * helper makes that policy explicit for both RX and TX IRQ sources during config/teardown.
 */
static void hw_spi_irq_enable( tdm_spi_inst_t inst, bool enable )
{
    if( !hw_inst_valid( inst ) )
    {
        return;
    }

    const tdm_spi_dev_t *dev = &s_spi_dev[inst];

    dspic33ak_spi_i2s_tdm_reg_set_or_clear( dev->rx_irq.ie_reg, dev->rx_irq.ie_mask, enable );
    dspic33ak_spi_i2s_tdm_reg_set_or_clear( dev->tx_irq.ie_reg, dev->tx_irq.ie_mask, enable );
}


/*
 * Disable and clear CPU SPI interrupt state for one SPI instance.
 *
 * Keeps the "CPU SPI IRQs are not part of the data path" policy in one place.
 */
static void hw_spi_irq_disable_clear( tdm_spi_inst_t inst )
{
    hw_spi_irq_enable( inst, false );
    dspic33ak_spi_i2s_tdm_hw_irq_clear_flags( inst );
}
