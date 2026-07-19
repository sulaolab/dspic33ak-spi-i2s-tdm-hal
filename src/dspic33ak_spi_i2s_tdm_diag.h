#ifndef DSPIC33AK_SPI_I2S_TDM_DIAG_H
#define DSPIC33AK_SPI_I2S_TDM_DIAG_H

//===========================================================
// dspic33ak_spi_i2s_tdm_diag.{c,h} = the SPI/I2S/TDM transport's DIAGNOSTICS,
// kept deliberately SEPARATE from the transport core. It owns the per-stream
// health counters (completed block count, deadline-miss count) and the per-ISR
// load/time monitor. Its external dependencies are: the hal_timer high-res API
// (used by the load monitor, runtime-gated via is_initialized() -- a normal
// sibling-HAL dependency, present in every build) and, ONLY under ENA_TDM_DBG,
// printf + scope GPIO. The transport core holds one dspic33ak_spi_i2s_tdm_diag_t
// per block-completion ISR and updates it ONLY through the functions below; it
// does not read the fields in the hot path. This separation keeps the transport
// context limited to what is required to configure/start/transfer/stop, and lets
// each physical SPI instance own an independent diag (the engine uses per-instance
// block-completion ISRs).
//
// CONCURRENCY: the counters are updated from the block-completion ISR. The 32-bit
// reads in *_get_load()/*_read_counts() are NOT atomic on this 16-bit core, so the
// CALLER must mask the updating ISR's CPU interrupt around them (the transport core
// already brackets the public readers with the DMA IE mask). The functions here do
// no masking themselves.
//===========================================================

#include <stdint.h>
#include <stdbool.h>
#include "dspic33ak_spi_i2s_tdm.h"   // dspic33ak_spi_i2s_tdm_load_t (public load-monitor type)


//===========================================================
// One block-completion ISR's diagnostics. block_count / block_deadline_miss_count
// are software/real-time stream health (NOT SPI HW over/underrun). The isr_*
// counters are raw high-res-timer ticks of ISR execution time; *_get_load()
// converts them to the public monitor (counts + 0.1us units).
//===========================================================
typedef struct {
    volatile uint32_t block_count;               // completed blocks since reset()
    volatile uint32_t block_deadline_miss_count; // HALF+DONE conflicts on this instance since reset()
    volatile uint32_t rx_dma_overrun_count;       // RX ISR snapshots with DMAxSTAT.OVERRUN since reset()
    volatile uint32_t rx_dma_other_irq_count;     // RX IRQ snapshots with neither HALF nor DONE since reset()
    volatile uint32_t rx_dma_last_status;         // raw DMAxSTAT from the most recent RX IRQ
    volatile uint32_t err_rov_block_count;        // RX blocks where SPIROV was observed, since reset()
    volatile uint32_t err_tur_block_count;        // RX blocks where SPITUR was observed set, since reset()
    volatile uint32_t err_frm_block_count;        // RX blocks where FRMERR was observed, since reset()
    volatile uint32_t frmerr_consecutive_blocks;  // consecutive RX blocks with FRMERR observed (0 on a clean block)
    volatile uint32_t isr_start_count;           // timer count at the current ISR entry
    volatile uint32_t isr_last_count;            // ticks of the last completed ISR
    volatile uint32_t isr_min_count;             // min ticks since the last clear_peak
    volatile uint32_t isr_max_count;             // max ticks since the last clear_peak
    volatile uint32_t isr_event_count;           // number of timed ISR samples since the last clear_peak
    volatile bool     isr_measure_active;        // current ISR has a valid start sample
} dspic33ak_spi_i2s_tdm_diag_t;


// Reset every counter (isr_min_count seeded to UINT32_MAX). Called by start().
void dspic33ak_spi_i2s_tdm_diag_reset( dspic33ak_spi_i2s_tdm_diag_t* d );

// Begin/end ISR load-time instrumentation. begin() snapshots the entry tick (only
// when the high-res timer HAL is initialized) and, under ENA_TDM_DBG, raises the
// scope GPIO; end() records last/min/max/event and lowers the scope GPIO.
void dspic33ak_spi_i2s_tdm_diag_isr_begin( dspic33ak_spi_i2s_tdm_diag_t* d );
void dspic33ak_spi_i2s_tdm_diag_isr_end( dspic33ak_spi_i2s_tdm_diag_t* d );

// Count one completed block for THIS instance's RX-block ISR (read via
// *_read_counts() / get_status()). Each instance keeps its own block_count.
void dspic33ak_spi_i2s_tdm_diag_note_block( dspic33ak_spi_i2s_tdm_diag_t* d );

// Fold one RX-block ISR's SPI framed-transport health-flag observation into this instance's
// diagnostics. `flags` is the mask from dspic33ak_spi_i2s_tdm_hw_sample_ack_errflags()
// (DSPIC33AK_SPI_I2S_TDM_STAT_SPIROV/SPITUR/FRMERR); each counter counts RX BLOCKS in which its
// bit was observed set, not raw event occurrences -- a flag that stays set across several blocks
// (e.g. SPITUR while an underrun condition persists) increments once per block, not once per
// underlying event. When FRMERR is absent, frmerr_consecutive_blocks is reset to zero; the other
// counters are unchanged when flags == 0. Read back via get_status().
void dspic33ak_spi_i2s_tdm_diag_note_errflags( dspic33ak_spi_i2s_tdm_diag_t* d, uint32_t flags );

// Update deadline diagnostics from THIS instance's DMA status snapshot. A HALF+DONE
// conflict means this instance's RX-block ISR fell a full block behind, so it counts
// one deadline miss in its OWN diag -- each instance keeps its own block/miss counts
// (per-instance diagnostics; there is no shared "block master" counter). dma_x is used
// only for the debug print label.
void dspic33ak_spi_i2s_tdm_diag_check_deadline( dspic33ak_spi_i2s_tdm_diag_t* d,
                                                uint8_t  dma_x,
                                                uint32_t dma_stat );

// Preserve the raw RX-DMA interrupt cause before the core resolves HALF/DONE. An
// OVERRUN-only interrupt therefore remains visible even though no audio half can be
// delivered. other_irq_count counts every snapshot lacking HALF/DONE and may overlap
// rx_dma_overrun_count for an OVERRUN-only snapshot.
void dspic33ak_spi_i2s_tdm_diag_note_dma_status( dspic33ak_spi_i2s_tdm_diag_t* d,
                                                 uint32_t dma_stat );

// Snapshot the load monitor into the public struct (and clear min/max/event when
// clear_peak). Returns false (and zeroes the monitor) until at least one timed
// sample exists and the high-res timer HAL is initialized. The caller MUST mask the
// updating ISR around this call (see CONCURRENCY note above).
bool dspic33ak_spi_i2s_tdm_diag_get_load( dspic33ak_spi_i2s_tdm_diag_t* d,
                                          dspic33ak_spi_i2s_tdm_load_t*  monitor,
                                          bool                           clear_peak );

// Read the two block counters. The caller MUST mask the updating ISR around this
// call. Either output pointer may be NULL.
void dspic33ak_spi_i2s_tdm_diag_read_counts( const dspic33ak_spi_i2s_tdm_diag_t* d,
                                             uint32_t* block_count,
                                             uint32_t* block_deadline_miss_count );


#endif // DSPIC33AK_SPI_I2S_TDM_DIAG_H
