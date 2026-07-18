#ifndef DSPIC33AK_SPI_I2S_TDM_HW_H
#define DSPIC33AK_SPI_I2S_TDM_HW_H

//===========================================================
// dspic33ak_spi_i2s_tdm_hw.{c,h} = the SILICON layer of the SPI/I2S/TDM HAL, kept
// separate from the transport core. hw.c owns the data-sheet device-facts table
// (s_spi_dev[]: SPIxBUF/CON1/BRG/IMSK pointers, DMA trigger CHSELs, CPU IRQ bits, with
// the AK512/AK128 #if) and every function that programs one physical SPI peripheral in
// framed (I2S/TDM) mode + arms its DMA channels.
//
// DELIBERATELY DECOUPLED FROM THE INSTANCE STRUCT: these functions take a
// tdm_spi_inst_t (which physical SPI) plus the raw DMA channels/buffers, NOT a
// tdm_spi_leg_t*. The transport core owns the instance/leg struct (callback, diag,
// running, config) and passes the silicon-relevant fields down. So hw.c knows nothing
// about the engine's instance bookkeeping -- a clean "drive the silicon" boundary. The
// register-level result is identical to the previous in-core per-leg code.
//===========================================================

#include <stdint.h>
#include <stdbool.h>
#include "dspic33ak_spi_i2s_tdm.h"   // dspic33ak_spi_i2s_tdm_config_t / _role_t


// Physical SPI instances present on the device (data sheet). SPI4 is AK512 only.
// TDM_SPI_INST_COUNT is the number of silicon SPI instances on the built device
// (the size of s_spi_dev[]); the core uses it to bound-check an instance.
typedef enum {
    TDM_SPI1 = 0,
    TDM_SPI2 = 1,
    TDM_SPI3 = 2,
#if DSPIC33AK_SPI_I2S_TDM_DEVICE == DSPIC33AK_SPI_I2S_TDM_DEV_AK512
    TDM_SPI4 = 3,
#endif //DSPIC33AK_SPI_I2S_TDM_DEVICE == DSPIC33AK_SPI_I2S_TDM_DEV_AK512
    TDM_SPI_INST_COUNT
} tdm_spi_inst_t;


// ---- Per-physical-SPI silicon operations (no instance-struct knowledge) ----
// All are no-ops for an out-of-range inst. apply_config writes SPIxCON1/BRG from the
// already-validated config and leaves DMA-trigger events + the module OFF (caller arms
// triggers, then enables the module). dma_config configures + enables that SPI's RX and
// TX DMA channels (RX: SPIxBUF->rx_buf, TX: tx_buf->SPIxBUF; count = words per buffer)
// and returns false if the DMA controller rejects a channel. dma_trigger_enable toggles
// the SPIxIMSK event->DMA enables; module_enable toggles SPIxCON1.ON; irq_clear_flags
// clears the CPU RX/TX interrupt flags; soft_stop disables triggers + masks CPU IRQs +
// clears the ON bit (no PPS/CLC change).
void dspic33ak_spi_i2s_tdm_hw_apply_config( tdm_spi_inst_t inst,
                                            const dspic33ak_spi_i2s_tdm_config_t* cfg );
bool dspic33ak_spi_i2s_tdm_hw_dma_config( tdm_spi_inst_t inst,
                                          uint8_t  rx_dma_ch,
                                          uint8_t  tx_dma_ch,
                                          int32_t* rx_buffer,
                                          int32_t* tx_buffer,
                                          uint32_t buffer_word_count );
void dspic33ak_spi_i2s_tdm_hw_dma_trigger_enable( tdm_spi_inst_t inst, bool enable );
void dspic33ak_spi_i2s_tdm_hw_module_enable( tdm_spi_inst_t inst, bool enable );
void dspic33ak_spi_i2s_tdm_hw_irq_clear_flags( tdm_spi_inst_t inst );
void dspic33ak_spi_i2s_tdm_hw_soft_stop( tdm_spi_inst_t inst );

// Sample this instance's SPI framed-transport health flags (SPIxSTAT SPIROV/SPITUR/FRMERR) and
// ack the software-clearable ones (SPIROV/FRMERR). SPITUR is R/HSC (not software-clearable; it
// is only observed here, reflecting a live/dynamic underrun condition). Returns the full observed
// mask (DSPIC33AK_SPI_I2S_TDM_STAT_* bits, as read, before any ack); 0 if none set or inst out of
// range. Cheap (one SFR read + masked write-back), meant to be called once per RX-block ISR (the
// driver otherwise runs with IGNROV/IGNTUR set and never inspects these HW flags). The HAL owns
// these sticky bits once this is called -- other code reading raw SPIxSTAT afterward will see
// SPIROV/FRMERR already acked.
uint32_t dspic33ak_spi_i2s_tdm_hw_sample_ack_errflags( tdm_spi_inst_t inst );

// Return the PPS output-function code (_RPOUT_SSx) for one SPI instance's frame-sync
// (SSx = FRMSYNC output in framed master mode). Used by the CLC10 50%-FS module to find,
// by reverse PPS lookup, which physical pin the board routed this instance's FS to.
// Returns false (and leaves *code untouched) if inst is out of range or the device header
// defines no SSx output for it. (data-sheet fact, device-#ifdef'd like the rest of hw.c.)
bool dspic33ak_spi_i2s_tdm_hw_get_ss_pps_code( tdm_spi_inst_t inst, uint8_t* code );


#endif // DSPIC33AK_SPI_I2S_TDM_HW_H
