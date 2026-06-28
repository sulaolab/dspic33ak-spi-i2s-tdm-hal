
//===========================================================
// INCLUDES
//===========================================================
#include "dspic33ak_spi_i2s_tdm_conf.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
// The silicon layer (SPIxCON1/BRG register writer, the s_spi_dev[] device-facts table,
// per-SPI DMA-channel config) lives in dspic33ak_spi_i2s_tdm_hw.*, and the block
// counters + ISR load/time monitor in dspic33ak_spi_i2s_tdm_diag.*. So the transport
// core no longer pulls the SPI register masks (reg.h), high-res timer, debug GPIO, or
// <stdio.h> -- it orchestrates instances and runs the block ISR, delegating register
// pokes to hw.* and diagnostics to diag.*.
#include "dspic33ak_dma.h"
#include "dspic33ak_spi_i2s_tdm.h"
#include "dspic33ak_spi_i2s_tdm_hw.h"      // silicon layer: tdm_spi_inst_t + register/DMA ops
#include "dspic33ak_spi_i2s_tdm_diag.h"    // block counters + ISR load/time monitor (separated)




//===========================================================
// Definition
//===========================================================
// Unknown-device guard is centralized in dspic33ak_spi_i2s_tdm.h: the
// DSPIC33AK_SPI_I2S_TDM_DEVICE derivation #error's on any unsupported device.


// DMA global address-window values are owned by the DMA HAL
// (DSPIC33AK_DMA_ADDR_WINDOW_LOW / _HIGH in dspic33ak_dma.h), set by
// dspic33ak_dma_global_init(). SPI-TDM keeps only per-channel cfg values.




// Debug switches live with their code now: the DMA-config-error printf in the silicon
// layer (dspic33ak_spi_i2s_tdm_hw.c) and the scope-GPIO/load monitor in the diagnostics
// module (dspic33ak_spi_i2s_tdm_diag.c) each carry their own ENA_TDM_DBG. The transport
// core compiles with no printf/GPIO dependency.

// Per-instance DMA channel assignment comes from conf.h (DSPIC33AK_TDM_SPIn_*_DMA);
// the core no longer hardcodes DMA channel index constants.

#define TDM_ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define TDM_COMPILEASSERT(exp) extern int tdm_compile_assert[(exp) ? 1 : -1] __attribute__((unused))




//--------------------------------------------







//===========================================================
// Enum & Struct typedef
//===========================================================

// tdm_spi_inst_t (physical SPI instances) + the silicon device-facts table live in the
// silicon layer (dspic33ak_spi_i2s_tdm_hw.*). The transport core indexes its own leg
// table with the leg-index enum below and stores the physical SPI in each leg's spi_inst.
//
// Leg-index enum, GENERATED from the instance descriptor list (conf.h): one
// TDM_SPI_LEG_<name> per row, in list order, terminated by TDM_SPI_LEG_COUNT (= the
// built-in instance count). TDM_SPI_LEG_BLOCK_REF is the first row = the block-timing
// reference. Used only inside the core (leg table / inst() / spi1()/spi2()), so it lives
// here rather than in a shared header. Adding an instance row in conf.h extends it.
typedef enum {
#define X(name, phys, rx, tx, role, slots, blk)   TDM_SPI_LEG_##name,
    DSPIC33AK_TDM_INSTANCE_LIST(X)
#undef X
    TDM_SPI_LEG_COUNT
} tdm_spi_leg_index_t;

// TDM_SPI_LEG_BLOCK_REF is the block-timing REFERENCE leg (leg 0) -- NOT a clock master.
// It owns the RX-block ISR that defines the block boundary and is what the singleton
// is_running()/get_status()/get_load() report. The clock role (master/slave) is a
// separate, per-instance concern carried in dspic33ak_spi_i2s_tdm_config_t.role. The
// current co-clocked topology requires exactly one BLOCK_REF, and it must be the first row.
#define TDM_SPI_LEG_BLOCK_REF   0   // first list row is the block-timing reference

// Map the list's role column (BLOCK_REF/FOLLOWER) to the leg's boolean flags. BLOCK_REF
// marks the block-timing reference leg only; it is NOT the clock role (see config_t.role).
#define TDM_LEG_IS_BLOCK_REF_BLOCK_REF   1
#define TDM_LEG_IS_BLOCK_REF_FOLLOWER    0
#define TDM_LEG_IS_BLOCK_REF(role)       TDM_LEG_IS_BLOCK_REF_##role

// Private SPI leg descriptor == the public opaque instance handle
// (dspic33ak_spi_i2s_tdm_inst_t). One physical SPI leg per row owns the SPI
// peripheral, RX/TX DMA channels, ping-pong buffers, its own block callback, and
// its own diagnostics. Each instance's RX-block ISR delivers ONLY this instance's
// RX/TX block to this instance's callback.
struct dspic33ak_spi_i2s_tdm_inst_s {
    tdm_spi_inst_t spi_inst;
    uint8_t        rx_dma_ch;
    uint8_t        tx_dma_ch;
    int32_t       *rx_buffer;
    int32_t       *tx_buffer;
    uint32_t       buffer_word_count;   // total words (2 * slots * blk) -- both ping+pong
    uint8_t        geom_slots_per_fs;   // THIS leg's compile-time slots/FS (buffer geometry)
    uint16_t       geom_block_frames;   // THIS leg's compile-time frames per ping/pong half
    dspic33ak_spi_i2s_tdm_config_t config;         // includes this leg's OWN clock role
    bool           config_valid;
    // Block-timing / singleton-reporting REFERENCE leg (the first list row). Each leg
    // already times itself via its own RX-block ISR (P1/P2), so this is NOT a clock-role
    // flag (clock role is per-leg in config, P3 Stage 1) and NOT a hard timing coupling --
    // it only marks which leg is_running()/get_status()/is_active() report and which RX
    // block the demo treats as primary. (When per-instance clocks land -- P3 Stage 2 --
    // the "exactly one, leg 0" topology rule generalizes; today the stream is co-clocked.)
    bool           is_block_timing_master;
    dspic33ak_spi_i2s_tdm_block_cb_t block_cb;     // this instance's block callback
    void                            *block_user;   // opaque user pointer for block_cb
    dspic33ak_spi_i2s_tdm_diag_t     diag;         // this instance's counters + ISR load monitor
    volatile bool                    running;      // this instance started (inst_start..inst_stop)
};
typedef struct dspic33ak_spi_i2s_tdm_inst_s tdm_spi_leg_t;

typedef struct
{
    tdm_spi_leg_t                      *legs;
    uint8_t                             leg_count;
    const dspic33ak_spi_i2s_tdm_port_t *port;

    // Block callback, diagnostics, and the running flag are now owned PER INSTANCE
    // (in tdm_spi_leg_t), not by the stream, so SPI1 and SPI2 each deliver their own
    // block, measure their own ISR load, and start/stop independently.
} tdm_stream_t;


//===========================================================
// Function Prototype
//===========================================================

// pin routing + CLC pass-through live on the board adapter and are reached
// through the registered port callbacks, NOT called by name -- the core no
// longer includes audio_app_board.h.

// Silicon-layer SPI register/DMA ops (tdm_spi_apply_config, *_dma_trigger_enable,
// *_module_enable, *_irq_*, *_soft_stop, *_dma_config) now live in the hw module and
// are reached as dspic33ak_spi_i2s_tdm_hw_*() taking a tdm_spi_inst_t + raw fields.

// Per-instance teardown/clear helpers (one physical SPI's own DMA buffers/channels).
static void        tdm_inst_clear_buffers( const tdm_spi_leg_t *leg );
static void        tdm_inst_soft_stop_dma( const tdm_spi_leg_t *leg );
static void        tdm_inst_clear_dma_flags( const tdm_spi_leg_t *leg );
static void        tdm_zero_memory(void *ptr, size_t bytes);
static bool        tdm_spi_leg_is_valid( const tdm_spi_leg_t *leg );
static bool        tdm_stream_topology_is_valid( const tdm_stream_t *stream );
static const tdm_spi_leg_t *tdm_stream_get_block_timing_master_leg( const tdm_stream_t *stream );
static bool        tdm_spi_leg_get_effective_config( const tdm_spi_leg_t *leg,
                                                     dspic33ak_spi_i2s_tdm_config_t *effective_cfg );

// RX-DMA IE bracket, config-envelope validation, and the per-instance RX-block ISR
// body. Definitions live in the Local Function section; forward-declared here so the
// global readers (get_load/get_status) and the ISR wrappers above their definitions
// resolve them. tdm_rx_ie_*/tdm_rx_block are static inline (folded in the hot path).
static inline bool tdm_rx_ie_disable( uint8_t rx_dma_ch );
static inline void tdm_rx_ie_restore( uint8_t rx_dma_ch, bool was_enabled );
static bool        tdm_config_is_supported( const tdm_spi_leg_t* leg, const dspic33ak_spi_i2s_tdm_config_t* cfg );
static inline void tdm_rx_block( tdm_spi_leg_t* inst, uint8_t rx_ch, uint8_t tx_ch, uint32_t half_pos );




static inline void  tdm_get_src_ptr( uint32_t        dma_stat,
                                       const int32_t*  const pRxDat,
                                       uint32_t        half_pos,
                                       const int32_t** src_pptr );
static inline void  tdm_get_dest_ptr( uint32_t dma_tx_addr, int32_t* const pTxDat, uint32_t half_pos, int32_t** dest_pptr );
// local_copy_to_CODEC / local_drc_df2t_path / local_filter_cascade_chm were
// moved to audio_app.c. The latter two are declared in audio_app.h.





//===========================================================
// Variables
//===========================================================

#define DMA_BUF_PING_POS     (0)   // ping half is always at offset 0; pong half = slots*blk per leg

// Per-instance ping/pong half size (words) = slots * blk for THIS row. Both the buffer
// size (2 * half) and the ISR's pong-half offset derive from it; passed as a compile-
// time literal into the generated ISR bodies so the hot-path pointer math stays folded.
#define TDM_LEG_HALF_WORDS(slots, blk)   ((slots) * (blk))

// Per-instance RX/TX ping-pong buffers, GENERATED one Rx_<name>/Tx_<name> pair per
// instance descriptor row (conf.h). Each is 2 * slots * blk words (Ping + Pong) using
// THIS row's slots/blk, so different legs may have different geometry. The names follow
// the leg names so the leg table can wire them by row.
#define X(name, phys, rx, tx, role, slots, blk)                                                                  \
    static int32_t    Tx_##name[ 2 * TDM_LEG_HALF_WORDS(slots, blk) ] __attribute__((aligned(4)));  /* 2: Ping/Pong */ \
    static int32_t    Rx_##name[ 2 * TDM_LEG_HALF_WORDS(slots, blk) ] __attribute__((aligned(4)));  /* 2: Ping/Pong */
DSPIC33AK_TDM_INSTANCE_LIST(X)
#undef X

// the application DSP float work buffers (f_A_Data, f_A_Data_chm,
// f_B_Data_chm) live in the demo app (audio_app.c), which owns AND
// clears them (before start()). The HAL core only owns/clears its DMA ping-pong
// buffers (the generated Rx_<name>/Tx_<name> pair per instance).




// RB15 / USB audio clock state and ISR moved to audio_app_board.c.


//===========================================================
// DMA channel + SPI allocation
//
// The DEVICE FACTS table (s_spi_dev[]: SPIxBUF/CON1/BRG/IMSK + DMA trigger CHSELs +
// CPU IRQ bits, indexed by tdm_spi_inst_t) lives in the silicon layer
// (dspic33ak_spi_i2s_tdm_hw.*). Here the transport core keeps only its DRIVER
// ALLOCATION: s_spi_legs[], one physical SPI leg per row with the SPI instance, RX/TX
// DMA channels, owned ping-pong buffers, and current logical config. start() passes
// each leg's spi_inst + DMA channels/buffers down to the hw_* ops.
//===========================================================

// GENERATED one row per instance descriptor (conf.h): physical SPI + RX/TX DMA channels
// + its own Rx_<name>/Tx_<name> ping-pong buffers. The BLOCK_REF row gets
// is_block_timing_master=1 (the singleton-reporting reference). Each leg's CLOCK role is
// NOT forced here -- it comes from the leg's own config (set by the integrator per leg:
// a follower is configured SLAVE because it rides the shared clock, not because the HAL
// forces it). Adding a row in conf.h adds a leg here automatically.
static tdm_spi_leg_t s_spi_legs[] =
{
#define X(name, phys, rx, tx, role, slots, blk)                                      \
    [TDM_SPI_LEG_##name] =                                                            \
    {                                                                                \
        .spi_inst               = phys,                                              \
        .rx_dma_ch              = (rx),                                              \
        .tx_dma_ch              = (tx),                                              \
        .rx_buffer              = Rx_##name,                                          \
        .tx_buffer              = Tx_##name,                                          \
        .buffer_word_count      = TDM_ARRAY_SIZE(Rx_##name),                          \
        .geom_slots_per_fs      = (uint8_t)(slots),                                   \
        .geom_block_frames      = (uint16_t)(blk),                                    \
        .is_block_timing_master = TDM_LEG_IS_BLOCK_REF(role),                         \
        .block_cb               = NULL,                                              \
        .block_user             = NULL,                                              \
        .diag                   = { .isr_min_count = 0xFFFFFFFFUL },  /* rest zero; start() calls diag_reset() */ \
    },
    DSPIC33AK_TDM_INSTANCE_LIST(X)
#undef X
};

static tdm_stream_t s_stream =
{
    .legs           = s_spi_legs,
    .leg_count      = (uint8_t)TDM_ARRAY_SIZE(s_spi_legs),
    .port           = NULL,
};

// The registered board/clock port is owned by the singleton stream context.
// The table is held by pointer, not copied, so the caller must provide a
// static/long-lived object. NULL is the board-free default: no pin routing,
// no external clock gate, and no clock-change events.

// Block-completion callback ownership lives PER INSTANCE (tdm_spi_leg_t.block_cb).
// Each instance's RX-block ISR invokes its own callback for each completed block;
// if NULL, that instance runs no app/DSP path and start()'s zeroed TX half stays
// silent until a callback fills it. Register before start(), do not clear while
// running; set_block_callback() updates the pair under that instance's RX DMA IE mask.

// Stream-health counters and the ISR load/time monitor live in the separated
// diagnostics module (dspic33ak_spi_i2s_tdm_diag.*); each instance (tdm_spi_leg_t)
// holds its own dspic33ak_spi_i2s_tdm_diag_t and updates it ONLY through the diag_*
// functions. Each instance's RX-block ISR feeds its diag (note_block / check_deadline
// / isr_begin / isr_end); start() resets every instance's diag. The singleton
// get_load()/get_status() report the block-timing-reference instance's diag under its
// RX DMA IE mask because 32-bit reads are non-atomic on this 16-bit core. The
// deadline metric is software/real-time, not SPIROV/SPITUR hardware status.

// Lifecycle running state is owned PER INSTANCE (tdm_spi_leg_t.running): set by a
// successful inst_start() and cleared by inst_stop(), exposed via
// inst_get_status().running and (for the block-timing reference) is_running(). This is
// deliberately separate from is_active(), which reports clock/source readiness.

// The leg table must have at least the BLOCK_REF row and not exceed the silicon SPI count.
TDM_COMPILEASSERT( TDM_ARRAY_SIZE(s_spi_legs) >= 1u );
TDM_COMPILEASSERT( TDM_ARRAY_SIZE(s_spi_legs) <= (size_t)TDM_SPI_INST_COUNT );

// Per-instance geometry sanity (compile-time), one set per instance-list row: slots/blk
// fit their leg fields (uint8_t / uint16_t), the 2*slots*blk word count cannot overflow
// int32 indexing, and the generated buffer is exactly that size. The default rows may use
// stream-wide macros already range-checked in conf.h; these asserts also cover configs
// that give each row its own slots/blk, so the per-instance geometry promise has teeth.
#define X(name, phys, rx, tx, role, slots, blk)                                          \
    TDM_COMPILEASSERT( (slots) > 0 && (slots) <= 255 );                                   \
    TDM_COMPILEASSERT( (blk)   > 0 && (blk)   <= 65535 );                                 \
    TDM_COMPILEASSERT( (slots) <= (2147483647 / (2 * (blk))) );                           \
    TDM_COMPILEASSERT( TDM_ARRAY_SIZE(Rx_##name) == (2u * (slots) * (blk)) );
DSPIC33AK_TDM_INSTANCE_LIST(X)
#undef X




//===========================================================
// Global Function
//===========================================================

// DMA global init/validate were removed in favor of the low-level DMA HAL.
// main() calls dspic33ak_dma_global_init() once at startup; the per-channel
// config below checks dspic33ak_dma_channel_config()/_enable() return values.


/*
 * Register the board/clock adapter used by this HAL.
 *
 * The table is stored by pointer, not copied, so the caller must provide a
 * static/long-lived object. Passing NULL returns the HAL to its board-free
 * default: no pin routing, no clock gate, and no clock-change events.
 */
void dspic33ak_spi_i2s_tdm_set_port( const dspic33ak_spi_i2s_tdm_port_t* port )
{
    s_stream.port = port;
}


/*
 * Report whether the external stream source is ready.
 *
 * This is a readiness gate, not a running-state check. With a registered port it
 * asks clock_source_ready(role); without a port it returns true so the HAL can
 * run as a self-clocked/no-gate transport.
 */
bool dspic33ak_spi_i2s_tdm_is_active( void )
{
    const tdm_stream_t *stream = &s_stream;

    // Stream-readiness gate routed through the clock port. No port (or no
    // clock_source_ready hook) => always ready (self-clocked, no external gate).
    // The Perseus platform wires this to the board's USB-audio clock readiness.
    // Pass the configured role. Before the first configure(), treat the transport
    // explicitly as SLAVE instead of relying on enum zero-initialization.
    if( ( stream->port != NULL ) && ( stream->port->clock_source_ready != NULL ) )
    {
        const tdm_spi_leg_t *timing_leg = tdm_stream_get_block_timing_master_leg( stream );
        dspic33ak_spi_i2s_tdm_role_t role =
            ( ( timing_leg != NULL ) && timing_leg->config_valid ) ? timing_leg->config.role
                                                                    : DSPIC33AK_SPI_I2S_TDM_ROLE_SLAVE;
        return stream->port->clock_source_ready( role );
    }
    return true;
}


/*
 * Report the engine's running state = the block-timing reference instance (SPI1).
 *
 * Set by a successful inst_start() of the block-timing reference and cleared by its inst_stop(). It
 * is separate from is_active(), which only means the clock/source gate is ready. For
 * a specific instance use inst_get_status().running.
 */
bool dspic33ak_spi_i2s_tdm_is_running( void )
{
    return s_spi_legs[TDM_SPI_LEG_BLOCK_REF].running;
}


/*
 * Read and clear the next external-clock stop/resume event.
 *
 * Clock detection lives in the board adapter and is reached through the port
 * hook. The app consumes this edge to run a mute-bounded stop/reconfigure/restart
 * sequence; without a hook, the HAL reports NONE.
 */
dspic33ak_spi_i2s_tdm_clock_event_t dspic33ak_spi_i2s_tdm_consume_clock_event( void )
{
    const tdm_stream_t *stream = &s_stream;

    // Routed through the clock port. No port (or no hook) => NONE (no external
    // clock to detect). The Perseus platform wires this to the board's RB15/CN edge.
    if( ( stream->port != NULL ) && ( stream->port->consume_clock_event != NULL ) )
    {
        return stream->port->consume_clock_event();
    }
    return DSPIC33AK_SPI_I2S_TDM_CLOCK_EVENT_NONE;
}


// The transport is RATE-AGNOSTIC: it moves 32-bit words at whatever BCLK/FS the
// configured BRG (master) or the external clock (slave) provides, and never derives
// anything from a sample-rate value. So the old rate API
// (is_supported_sample_rate / notify_sample_rate / get_sample_rate /
// set_rate_callback + the rate_state machine + rate-change callback) has been removed.
// Sample-rate POLICY is not a HAL property at all -- the supported-rate set is the
// product/board's (APP_SAMPLE_RATE_IS_SUPPORTED in the app layer, used by the CMSIS-SAI
// wrapper to validate ARM_SAI AUDIO_FREQ). Any runtime rate DETECTION (e.g. from a
// USB-audio clock notification or an FS measurement) and the stop->reconfigure->start
// it would drive belong in the application, not here.


/*
 * Number of SPI instances this build has (the size of the instance list in conf.h).
 * Pair with inst(i) to enumerate: for i in 0 .. instance_count()-1.
 */
uint8_t dspic33ak_spi_i2s_tdm_instance_count( void )
{
    return s_stream.leg_count;
}

/*
 * Return the handle for one SPI instance, or NULL if index is out of range.
 *
 * index is a leg index in list order (0 = the block-timing reference). spi1()/spi2() are
 * thin, name-stable wrappers over it (spi1() = inst(TDM_SPI_LEG_SPI1); spi2() = the SPI2
 * leg or NULL when not built); the TDM_SPI_LEG_* names are core-internal. The handle is
 * the address of the static SPI leg descriptor.
 */
dspic33ak_spi_i2s_tdm_inst_t* dspic33ak_spi_i2s_tdm_inst( uint8_t index )
{
    if( index >= (uint8_t)TDM_SPI_LEG_COUNT )
    {
        return NULL;
    }
    return &s_spi_legs[index];
}

dspic33ak_spi_i2s_tdm_inst_t* dspic33ak_spi_i2s_tdm_spi1( void )
{
    return dspic33ak_spi_i2s_tdm_inst( TDM_SPI_LEG_SPI1 );   // leg index from the instance list
}

dspic33ak_spi_i2s_tdm_inst_t* dspic33ak_spi_i2s_tdm_spi2( void )
{
#if DSPIC33AK_TDM_USE_SPI2
    return dspic33ak_spi_i2s_tdm_inst( TDM_SPI_LEG_SPI2 );
#else
    return NULL;                                            // SPI2 not built -> no such leg
#endif
}


//===========================================================
// Last-error diagnostic (debug aid; see the header contract). Plain last-writer-wins
// store -- NOT updated from the ISR hot path and NOT stream health.
//===========================================================
static volatile dspic33ak_spi_i2s_tdm_error_t s_last_error = DSPIC33AK_SPI_I2S_TDM_ERR_NONE;

static inline void tdm_set_error( dspic33ak_spi_i2s_tdm_error_t err )
{
    s_last_error = err;
}

dspic33ak_spi_i2s_tdm_error_t dspic33ak_spi_i2s_tdm_get_last_error( void )
{
    return s_last_error;
}


/*
 * Return one instance's current writable TX ping-pong half, or NULL.
 *
 * For an app that produces one instance's output from ANOTHER instance's block callback
 * -- e.g. a co-clocked dual-codec demo where SPI1's callback fills BOTH its own and
 * SPI2's TX so the two codecs stay sample-aligned (same block, same frame), with no
 * cross-ISR handoff or ordering/race dependency. The returned half is the one NOT being
 * transmitted (same selection the instance's own block callback receives as dst). Only
 * valid while inst is running, and only meaningful when called at a block boundary --
 * co-clocked siblings share the ping-pong phase, so SPI2's writable half then matches
 * SPI1's. Returns NULL if inst is NULL/stopped or the half cannot be resolved; the
 * caller must NULL-check before writing.
 */
int32_t* dspic33ak_spi_i2s_tdm_inst_tx_fill_ptr( dspic33ak_spi_i2s_tdm_inst_t* inst )
{
    int32_t* dst = NULL;

    if( ( inst == NULL ) || !inst->running )
    {
        return NULL;
    }
    // Per-leg pong-half offset = slots * blk (runtime path; the hot ISR uses a literal).
    tdm_get_dest_ptr( dspic33ak_dma_read_src( inst->tx_dma_ch ), inst->tx_buffer,
                      (uint32_t)inst->geom_slots_per_fs * inst->geom_block_frames, &dst );
    return dst;
}


/*
 * Register or clear one SPI instance's audio-block callback.
 *
 * That instance's RX-block ISR calls this hook once per completed block. The update
 * is bracketed by briefly masking the instance's RX DMA IE so the ISR cannot observe
 * a torn cb/user pair.
 *
 * Returns false (and changes nothing) if the contract is violated: NULL inst, or the
 * instance is running and the call would CHANGE the (cb,user) pair -- the callback must
 * be registered before inst_start() and not swapped/cleared mid-stream. Re-registering
 * the identical (cb,user) while running is allowed (idempotent no-op -> true).
 */
bool dspic33ak_spi_i2s_tdm_set_block_callback( dspic33ak_spi_i2s_tdm_inst_t* inst,
                                               dspic33ak_spi_i2s_tdm_block_cb_t cb,
                                               void* user )
{
    bool     rxie_bak;

    if( inst == NULL )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }

    // While running, only a no-op re-register of the same pair is permitted.
    if( inst->running && ( cb != inst->block_cb || user != inst->block_user ) )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_ALREADY_RUNNING );
        return false;
    }

    rxie_bak = tdm_rx_ie_disable( inst->rx_dma_ch );

    inst->block_cb   = cb;
    inst->block_user = user;

    tdm_rx_ie_restore( inst->rx_dma_ch, rxie_bak );
    tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NONE );
    return true;
}


/*
 * Open the shared board/clock port for the engine (role-aware), ONCE, before any
 * instance is started.
 *
 * Brings up + checks the external clock and routes pins/CLC through the registered
 * port hooks (all optional). Returns false if the clock cannot be brought up, is not
 * ready, or a pin/CLC hook rejects the role -- the caller must then not start any
 * instance. With no port registered this is a no-op success (self-clocked). It does
 * NOT block waiting for a clock (single readiness check) and does NOT touch any
 * SPI/DMA -- per-instance start arms the hardware. The role is the block-timing
 * reference's role (the app passes it; co-clocked followers ride the same clock/pins).
 */
bool dspic33ak_spi_i2s_tdm_open( dspic33ak_spi_i2s_tdm_role_t role )
{
    const tdm_stream_t *stream = &s_stream;

    // Verify the shared-engine topology ONCE here, before any clock/pin bring-up:
    // exactly one block-timing reference (the first leg), distinct physical SPIs, and
    // distinct DMA channels across all legs. This catches an instance-list misconfig
    // (e.g. two legs on the same SPI, or a crossed DMA channel) that the per-leg
    // tdm_spi_leg_is_valid() check at configure/start cannot see.
    if( !tdm_stream_topology_is_valid( stream ) )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_TOPOLOGY );
        return false;
    }

    if( stream->port != NULL )
    {
        if( ( stream->port->clock_source_init != NULL ) && !stream->port->clock_source_init( role ) )
        {
            tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_CLOCK_INIT );
            return false;   // external clock could not be brought up (e.g. unsupported role)
        }
        if( ( stream->port->clock_source_ready != NULL ) && !stream->port->clock_source_ready( role ) )
        {
            tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_CLOCK_NOT_READY );
            return false;   // clock not ready yet -- caller retries open() later
        }
        if( ( stream->port->configure_pins != NULL ) && !stream->port->configure_pins( role ) )
        {
            tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_PIN_CONFIG );
            return false;   // role this board cannot pin-route
        }
        if( ( stream->port->clc_passthrough != NULL ) && !stream->port->clc_passthrough( role ) )
        {
            tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_CLC );
            return false;
        }
    }
    tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NONE );
    return true;
}


/*
 * Close the shared port after all instances are stopped.
 *
 * Deliberately a near-no-op: like stop(), the HAL does NOT tear down PPS/CLC routing
 * or the external clock -- other peripherals (or the next open()/start()) may depend
 * on them, and the port has no deinit hook. Provided for lifecycle symmetry and as
 * the place a future clock-deinit hook would run.
 */
void dspic33ak_spi_i2s_tdm_close( void )
{
    // No hardware teardown by design (see above).
}


// The macro-derived default config lives in the platform layer -- see
// audio_app_board_get_default_config() in audio_app/. The core
// no longer fabricates a config from app macros; callers configure() explicitly.


/*
 * Store a validated configuration for ONE instance (declaration only; no HW write).
 *
 * Rejected (false) if the instance is running or invalid, or cfg is outside the
 * supported wire-format envelope (NULL-safe). Seeds the engine-shared sample-rate
 * tracking from cfg (co-clocked instances share one rate). The follower force-slave
 * policy is applied at register-write time, not stored back into the saved config.
 */
bool dspic33ak_spi_i2s_tdm_inst_configure( dspic33ak_spi_i2s_tdm_inst_t* inst,
                                           const dspic33ak_spi_i2s_tdm_config_t* cfg )
{
    // Reconfiguring a live instance would glitch (or tear) the framing mid-block; the
    // contract is stop -> configure -> start, so reject configure while running.
    if( inst == NULL )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }
    if( inst->running )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_ALREADY_RUNNING );
        return false;
    }
    if( !tdm_spi_leg_is_valid( inst ) )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }
    if( cfg == NULL )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_BAD_ARGUMENT );
        return false;
    }
    if( !tdm_config_is_supported( inst, cfg ) )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_UNSUPPORTED_CONFIG );
        return false;
    }

    inst->config       = *cfg;
    inst->config_valid = true;
    tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NONE );
    return true;
}




/*
 * Start ONE instance: arm its RX/TX DMA, then program + enable its SPI (triggers,
 * then module ON). The shared port must already be open()'d. Returns false (instance
 * left stopped, its HW rolled back) if it is not configured, already running, or DMA
 * setup fails. Does NOT touch the port or any other instance -- the caller orders
 * multi-instance starts (followers before the block-timing reference so all outputs are
 * armed when the block-timing reference's cadence begins).
 */
bool dspic33ak_spi_i2s_tdm_inst_start( dspic33ak_spi_i2s_tdm_inst_t* inst )
{
    dspic33ak_spi_i2s_tdm_config_t eff_cfg;

    // Gate FIRST -- do not arm DMA/SPI unless this instance is actually going to run.
    // config_valid implies the config already passed configure()'s envelope check.
    if( ( inst == NULL ) || !tdm_spi_leg_is_valid( inst ) )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }
    if( !inst->config_valid )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NOT_CONFIGURED );
        return false;
    }
    if( inst->running )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_ALREADY_RUNNING );
        return false;
    }
    // Resolve the register-level config (applies the follower force-slave policy).
    if( !tdm_spi_leg_get_effective_config( inst, &eff_cfg ) )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_UNSUPPORTED_CONFIG );
        return false;
    }

    // Fresh diagnostics + a deterministic first block for this instance.
    dspic33ak_spi_i2s_tdm_diag_reset( &inst->diag );
    tdm_inst_clear_buffers( inst );

    // Arm this instance's RX/TX DMA; on failure roll back ITS DMA/SPI so a partial
    // start leaves no channel with CHEN/IE set and the SPI off.
    if( !dspic33ak_spi_i2s_tdm_hw_dma_config( inst->spi_inst,
                                              inst->rx_dma_ch, inst->tx_dma_ch,
                                              inst->rx_buffer, inst->tx_buffer,
                                              inst->buffer_word_count ) )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_DMA_CONFIG );
        goto fail;
    }

    // Program + enable this instance's SPI: registers, then DMA-trigger events, then
    // the module ON (ON is the last step, after the port pins/CLC are routed).
    dspic33ak_spi_i2s_tdm_hw_apply_config( inst->spi_inst, &eff_cfg );
    dspic33ak_spi_i2s_tdm_hw_dma_trigger_enable( inst->spi_inst, true );
    dspic33ak_spi_i2s_tdm_hw_module_enable( inst->spi_inst, true );

    inst->running = true;
    tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NONE );
    return true;

fail:
    tdm_inst_soft_stop_dma( inst );
    tdm_inst_clear_dma_flags( inst );
    dspic33ak_spi_i2s_tdm_hw_soft_stop( inst->spi_inst );
    dspic33ak_spi_i2s_tdm_hw_irq_clear_flags( inst->spi_inst );
    inst->running = false;
    return false;
}




/*
 * Stop ONE instance and make its next start deterministic.
 *
 * SoftStop policy (per instance): does NOT stop DMACONbits.ON (shared controller) or
 * change PPS/CLC routing; stops only this instance's SPI module + DMA channels, masks
 * its DMA IRQs first so its ISR cannot refill mid-teardown, clears its pending
 * status, and clears its buffers so a restart is silent. Safe to call when stopped.
 */
void dspic33ak_spi_i2s_tdm_inst_stop( dspic33ak_spi_i2s_tdm_inst_t* inst )
{
    if( ( inst == NULL ) || !tdm_spi_leg_is_valid( inst ) )
    {
        return;
    }

    // Mark stopped up front so a per-instance running query sees the transition.
    inst->running = false;

    // DMA IRQs off + channels off first (no refill), then SPI module + triggers off.
    tdm_inst_soft_stop_dma( inst );
    dspic33ak_spi_i2s_tdm_hw_soft_stop( inst->spi_inst );

    // Clear pending status/flags, then buffers, before the next start.
    tdm_inst_clear_dma_flags( inst );
    dspic33ak_spi_i2s_tdm_hw_irq_clear_flags( inst->spi_inst );
    tdm_inst_clear_buffers( inst );
}




/*
 * Snapshot ONE instance's ISR load monitor.
 *
 * Values are updated from that instance's RX-block ISR, so its RX DMA IE is briefly
 * masked while reading the 32-bit counters on the 16-bit core. Returns false (NULL
 * args), or until at least one timed event exists and the high-resolution timer was
 * initialized. clear_peak resets min/max/event afterward.
 */
bool dspic33ak_spi_i2s_tdm_inst_get_load( dspic33ak_spi_i2s_tdm_inst_t* inst,
                                          dspic33ak_spi_i2s_tdm_load_t* monitor,
                                          bool clear_peak )
{
    bool     rxie_bak;
    bool     valid;

    if( ( inst == NULL ) || ( monitor == NULL ) )
    {
        return false;
    }

    // The diag counters are updated in this instance's RX-block ISR; mask its IE so
    // the snapshot + clear are consistent against the 16-bit core's non-atomic 32-bit
    // reads. The diag module itself does no masking.
    rxie_bak = tdm_rx_ie_disable( inst->rx_dma_ch );
    valid    = dspic33ak_spi_i2s_tdm_diag_get_load( &inst->diag, monitor, clear_peak );
    tdm_rx_ie_restore( inst->rx_dma_ch, rxie_bak );

    return valid;
}


/*
 * Snapshot ONE instance's health/status.
 *
 * block_count, block_deadline_miss_count, load, and running are THIS instance's.
 * active is engine-wide (the shared clock/source readiness gate).
 */
bool dspic33ak_spi_i2s_tdm_inst_get_status( dspic33ak_spi_i2s_tdm_inst_t* inst,
                                            dspic33ak_spi_i2s_tdm_status_t* status,
                                            bool clear_peak )
{
    bool     rxie_bak;

    if( ( inst == NULL ) || ( status == NULL ) )
    {
        return false;
    }

    status->active  = dspic33ak_spi_i2s_tdm_is_active();   // shared clock/source readiness gate
    status->running = inst->running;                       // this instance's running state

    // 32-bit reads on a 16-bit core are not atomic vs this instance's RX-block ISR; mask briefly.
    rxie_bak = tdm_rx_ie_disable( inst->rx_dma_ch );
    dspic33ak_spi_i2s_tdm_diag_read_counts( &inst->diag,
                                            &status->block_count,
                                            &status->block_deadline_miss_count );
    tdm_rx_ie_restore( inst->rx_dma_ch, rxie_bak );

    // load monitor (does its own RX-IE guard; honours clear_peak)
    (void)dspic33ak_spi_i2s_tdm_inst_get_load( inst, &status->load, clear_peak );

    return true;
}


/*
 * Singleton load/status readers: report the block-timing-reference instance (SPI1).
 * Thin wrappers over the per-instance readers; behaviour is unchanged from before
 * the per-instance API was added.
 */
bool dspic33ak_spi_i2s_tdm_get_load( dspic33ak_spi_i2s_tdm_load_t* monitor, bool clear_peak )
{
    return dspic33ak_spi_i2s_tdm_inst_get_load( &s_spi_legs[TDM_SPI_LEG_BLOCK_REF], monitor, clear_peak );
}

bool dspic33ak_spi_i2s_tdm_get_status( dspic33ak_spi_i2s_tdm_status_t* status, bool clear_peak )
{
    return dspic33ak_spi_i2s_tdm_inst_get_status( &s_spi_legs[TDM_SPI_LEG_BLOCK_REF], status, clear_peak );
}


//===========================================================
// DMA INTERRUPT VECTORS  --  IMPORTANT: these ARE the IVT entries the CPU jumps to.
//
// The HAL ships its own DMA interrupt vectors so the transport is turnkey: link the HAL
// and the _DMAnInterrupt slots are filled, the DMA channels are already armed by
// start(), and the integrator only registers a per-instance block callback. (Previously
// these lived in a separate optional TU dspic33ak_spi_i2s_tdm_irq.c + a forwarding
// worker; folded back here -- one X-macro, no cross-TU hop -- since the toolchain/IVT
// coupling is confined to this one section by the TDM_VEC() placeholder anyway.)
//
// GENERATED one RX vector per instance descriptor row (conf.h). The RX channel's vector
// runs that instance's block ISR (tdm_rx_block with THIS leg + its RX/TX channels + its
// slots*blk half size, ALL compile-time constants so the DMA register access / pointer
// math fold). The TX channel is INTENTIONALLY interrupt-less (hw.c enables the CPU IRQ
// on the RX channel only): TX is fire-and-forget ping-pong with auto-reload, so the RX
// completion alone defines the block boundary -- there is no TX ISR. The channel numbers
// come from the SAME list, so a conf.h remap moves the vectors too.
//
// GREP ANCHOR -- the vector names are TOKEN-PASTED (_DMA##ch##Interrupt via TDM_VEC), so
// a search for a literal name will not hit the generated definition; it lands HERE. At
// the default channel mapping (conf.h) this defines:
//     _DMA0Interrupt   (leg SPI1 RX, DMA0)
//     _DMA2Interrupt   (leg SPI2 RX, DMA2, when DSPIC33AK_TDM_USE_SPI2)
// A remap (e.g. SPI1 -> DMA6) changes which _DMA<n>Interrupt name appears.
//
// IVT slots are chip-wide exclusive. If another subsystem must own one of these
// channels, REMAP the SPI to a free channel in conf.h (preferred -- the vector follows
// by token-paste); a genuine clash surfaces as a duplicate-_DMAnInterrupt link error.
//===========================================================

// Build the XC-DSC vector name _DMAnInterrupt from a channel NUMBER. Two-level paste so
// a macro argument (e.g. DSPIC33AK_TDM_SPI1_RX_DMA) expands to its value BEFORE the ##.
// This placeholder keeps the literal compiler-reserved name out of the source text.
#define TDM_VEC_(ch)   _DMA##ch##Interrupt
#define TDM_VEC(ch)    TDM_VEC_(ch)

// One RX interrupt vector per instance row. tdm_rx_block is static inline (same TU) so
// the constant RX/TX channels + half size fold into the register access. (tx) is still
// passed -- tdm_rx_block reads the TX channel's DMA address to pick the writable TX half
// -- but the TX channel itself raises no interrupt (interrupt-less; see above).
//
// Compiled only when the HAL owns the IVT (DSPIC33AK_TDM_DEFINE_DMA_VECTORS=1, default).
// With =0 the integrator owns the vectors and calls dspic33ak_spi_i2s_tdm_inst_rx_isr()
// (below) from their own _DMA<rx>Interrupt instead.
#if DSPIC33AK_TDM_DEFINE_DMA_VECTORS
#define X(name, phys, rx, tx, role, slots, blk)                                          \
    void __attribute__((interrupt, context)) TDM_VEC(rx)(void)                           \
    {                                                                                    \
        tdm_rx_block( &s_spi_legs[TDM_SPI_LEG_##name], (rx), (tx), TDM_LEG_HALF_WORDS(slots, blk) ); \
    }
DSPIC33AK_TDM_INSTANCE_LIST(X)
#undef X
#endif // DSPIC33AK_TDM_DEFINE_DMA_VECTORS

/*
 * Public RX-block ISR entry for ONE instance (vector-ownership opt-out path).
 *
 * Runs the same block work as the HAL's own generated vector, but is a plain (non-
 * interrupt) function the integrator calls from their OWN _DMA<rx>Interrupt when
 * DSPIC33AK_TDM_DEFINE_DMA_VECTORS=0. Channels + half size come from the leg at runtime
 * (the generated turnkey vectors fold them as constants; this dispatch trades that for
 * IVT ownership). NULL inst is ignored. Call it for the instance's RX channel only --
 * TX is interrupt-less.
 */
void dspic33ak_spi_i2s_tdm_inst_rx_isr( dspic33ak_spi_i2s_tdm_inst_t* inst )
{
    if( inst == NULL )
    {
        return;
    }
    tdm_rx_block( inst, inst->rx_dma_ch, inst->tx_dma_ch,
                  (uint32_t)inst->geom_slots_per_fs * (uint32_t)inst->geom_block_frames );
}


//===========================================================
// Local Function
//===========================================================

/*
 * Clear one instance's DMA ping-pong buffers (RX + TX).
 *
 * This covers transport buffers only. Application DSP work buffers live in
 * audio_app.c and are deliberately outside the HAL's ownership boundary.
 */
static void tdm_inst_clear_buffers( const tdm_spi_leg_t *leg )
{
    if( leg == NULL )
    {
        return;
    }
    tdm_zero_memory( leg->tx_buffer, leg->buffer_word_count * sizeof(leg->tx_buffer[0]) );
    tdm_zero_memory( leg->rx_buffer, leg->buffer_word_count * sizeof(leg->rx_buffer[0]) );
}


/*
 * Tiny local memset(0) helper.
 *
 * Keeps this embedded HAL independent of the C library's memset implementation
 * and safely accepts NULL for callers that are already walking descriptor tables.
 */
static void tdm_zero_memory(void *ptr, size_t bytes)
{
    uint8_t *p = (uint8_t *)ptr;

    if( p == NULL )
    {
        return;
    }

    while( bytes > 0u )
    {
        *p = 0u;
        p++;
        bytes--;
    }
}


/*
 * Stop one instance's DMA activity.
 *
 * Its RX/TX interrupts are masked first so the ISR cannot refill buffers while the
 * channels are being disabled. The DMA controller global ON state is intentionally
 * untouched because other subsystems may be using DMA.
 */
static void tdm_inst_soft_stop_dma( const tdm_spi_leg_t *leg )
{
    if( leg == NULL )
    {
        return;
    }
    dspic33ak_dma_irq_enable( leg->rx_dma_ch, false );
    dspic33ak_dma_irq_enable( leg->tx_dma_ch, false );
    (void)dspic33ak_dma_channel_enable( leg->rx_dma_ch, false );
    (void)dspic33ak_dma_channel_enable( leg->tx_dma_ch, false );
}


/*
 * Clear one instance's pending DMA status and CPU interrupt flags.
 *
 * Status is cleared before IRQ flags so stale HALF/DONE conditions cannot
 * immediately retrigger on the next start.
 */
static void tdm_inst_clear_dma_flags( const tdm_spi_leg_t *leg )
{
    if( leg == NULL )
    {
        return;
    }
    dspic33ak_dma_clear_status( leg->rx_dma_ch );
    dspic33ak_dma_clear_status( leg->tx_dma_ch );
    dspic33ak_dma_clear_irq_flag( leg->rx_dma_ch );
    dspic33ak_dma_clear_irq_flag( leg->tx_dma_ch );
}


/*
 * Validate one private SPI leg descriptor.
 *
 * Checks only descriptor-local invariants: a known SPI instance, distinct RX/TX
 * DMA channels, non-NULL buffers, and a non-zero buffer length. Cross-leg
 * singleton rules are enforced by tdm_stream_topology_is_valid().
 */
static bool tdm_spi_leg_is_valid( const tdm_spi_leg_t *leg )
{
    if( leg == NULL )
    {
        return false;
    }
    if( (unsigned)leg->spi_inst >= (unsigned)TDM_SPI_INST_COUNT )
    {
        return false;
    }
    if( leg->rx_dma_ch == leg->tx_dma_ch )
    {
        return false;
    }
    if( ( leg->rx_buffer == NULL ) || ( leg->tx_buffer == NULL ) )
    {
        return false;
    }
    if( leg->buffer_word_count == 0u )
    {
        return false;
    }
    return true;
}


/*
 * Validate the private stream topology.
 *
 * Generic over the instance count (1..TDM_SPI_INST_COUNT): every leg is descriptor-
 * valid, EXACTLY ONE leg is the block-timing REFERENCE, and that reference is the first
 * leg (TDM_SPI_LEG_BLOCK_REF -- the row is_running()/the singleton get_status()/is_active()
 * report). All legs use distinct physical SPIs and distinct DMA channels (cross-leg loop
 * below). The physical SPI and the RX/TX DMA channels per leg come from the instance
 * descriptor list (conf.h); NOTHING here is pinned to a specific SPI number or channel,
 * so adding an instance or remapping a leg's channels needs no edit to this check.
 *
 * The "exactly one reference, at leg 0" rule is KEPT deliberately: the stream is still
 * co-clocked and the singleton reporting API needs one primary leg. It is NOT a
 * clock-role constraint (clock role is per-leg, P3 Stage 1). It generalizes to truly
 * independent per-instance timing when per-instance clocks land (P3 Stage 2), not before.
 */
static bool tdm_stream_topology_is_valid( const tdm_stream_t *stream )
{
    uint8_t timing_ref_count = 0u;
    const tdm_spi_leg_t *timing_leg = NULL;

    if( ( stream == NULL ) ||
        ( stream->legs == NULL ) ||
        ( stream->leg_count == 0u ) ||
        ( stream->leg_count > (uint8_t)TDM_SPI_INST_COUNT ) )
    {
        return false;
    }

    for( uint8_t i = 0u; i < stream->leg_count; i++ )
    {
        const tdm_spi_leg_t *leg = &stream->legs[i];

        if( !tdm_spi_leg_is_valid( leg ) )
        {
            return false;
        }
        if( leg->is_block_timing_master )
        {
            timing_ref_count++;
        }
    }
    if( timing_ref_count != 1u )
    {
        return false;
    }

    // The single block-timing reference must be the first leg (TDM_SPI_LEG_BLOCK_REF): its RX
    // ISR defines the block boundary and is what is_running()/get_status() report.
    timing_leg = tdm_stream_get_block_timing_master_leg( stream );
    if( ( timing_leg == NULL ) ||
        ( timing_leg != &stream->legs[TDM_SPI_LEG_BLOCK_REF] ) )
    {
        return false;
    }

    for( uint8_t i = 0u; i < stream->leg_count; i++ )
    {
        for( uint8_t j = (uint8_t)(i + 1u); j < stream->leg_count; j++ )
        {
            const tdm_spi_leg_t *a = &stream->legs[i];
            const tdm_spi_leg_t *b = &stream->legs[j];

            if( a->spi_inst == b->spi_inst )
            {
                return false;
            }
            if( ( a->rx_dma_ch == b->rx_dma_ch ) ||
                ( a->rx_dma_ch == b->tx_dma_ch ) ||
                ( a->tx_dma_ch == b->rx_dma_ch ) ||
                ( a->tx_dma_ch == b->tx_dma_ch ) )
            {
                return false;
            }
        }
    }

    return true;
}


/*
 * Find the single block-timing-reference SPI leg.
 *
 * Returns NULL if the table has no reference or more than one. The timing
 * leg is the row whose RX DMA interrupt defines block completion and invokes the
 * application's block callback.
 */
static const tdm_spi_leg_t *tdm_stream_get_block_timing_master_leg( const tdm_stream_t *stream )
{
    const tdm_spi_leg_t *timing_leg = NULL;

    if( ( stream == NULL ) || ( stream->legs == NULL ) )
    {
        return NULL;
    }

    for( uint8_t i = 0u; i < stream->leg_count; i++ )
    {
        if( stream->legs[i].is_block_timing_master )
        {
            if( timing_leg != NULL )
            {
                return NULL;
            }
            timing_leg = &stream->legs[i];
        }
    }

    return timing_leg;
}


/*
 * Build the hardware-applied config for one SPI leg.
 *
 * Just a validated copy of the leg's stored config -- each leg carries its OWN clock
 * role (master/slave), so there is no stream-wide role override here. A follower is
 * SLAVE because the integrator configured it SLAVE (it rides the shared clock), not
 * because the HAL forces it; an instance on an independent clock can be its own master.
 */
static bool tdm_spi_leg_get_effective_config( const tdm_spi_leg_t *leg,
                                              dspic33ak_spi_i2s_tdm_config_t *effective_cfg )
{
    if( ( effective_cfg == NULL ) || !tdm_spi_leg_is_valid( leg ) || !leg->config_valid )
    {
        return false;
    }

    *effective_cfg = leg->config;
    return true;
}








// Deadline-miss, ISR load/time, and the debug scope-GPIO/printf instrumentation
// that used to live here (local_dma_debug_check / _start / _end) now live in the
// separated diagnostics module (dspic33ak_spi_i2s_tdm_diag.*), reached through
// dspic33ak_spi_i2s_tdm_diag_check_deadline / _isr_begin / _isr_end.









/*
 * Resolve the RX ping-pong half completed by the DMA status snapshot.
 *
 * Output is NULL when the status does not identify a usable half or when inputs
 * are invalid. The returned pointer is into the HAL-owned RX buffer and is passed
 * read-only to the application block callback.
 */
static inline void tdm_get_src_ptr( uint32_t             dma_stat,
                                      const int32_t* const pRxDat,
                                      uint32_t             half_pos,
                                      const int32_t**      src_pptr )
{
    if( src_pptr == NULL )
    {
        return;
    }
    *src_pptr = NULL;

    if( pRxDat == NULL )
    {
        return;
    }

    // half_pos = this instance's ping/pong half size in words (slots * blk).
    switch( dspic33ak_dma_half_from_status( dma_stat ) )
    {
    case DSPIC33AK_DMA_HALF_FIRST:
        // SW can use Ping(A) side buffer.
        //////////////////////////////////////
        *src_pptr  = &pRxDat[ DMA_BUF_PING_POS ];
        break;

    case DSPIC33AK_DMA_HALF_SECOND:
        // SW can use Pong(B) side buffer.
        //////////////////////////////////////
        *src_pptr  = &pRxDat[ half_pos ];
        break;

    default:
        break;
    }
}

/*
 * Resolve the TX ping-pong half that software may safely fill next.
 *
 * The active DMA source address tells which half DMA is currently reading; the
 * helper returns the opposite half. Output is NULL when inputs are invalid.
 */
static inline void tdm_get_dest_ptr( uint32_t       dma_tx_addr,
                                       int32_t* const pTxDat,
                                       uint32_t       half_pos,
                                       int32_t**      dest_pptr )
{
    if( dest_pptr == NULL )
    {
        return;
    }
    *dest_pptr = NULL;

    if( pTxDat == NULL )
    {
        return;
    }

    // half_pos = this instance's ping/pong half size in words (slots * blk). The buffer
    // is [base, end) = pTxDat[0 .. 2*half_pos); the pong half starts at pTxDat[half_pos].
    // GUARD: the active TX-DMA source address must actually lie inside this buffer. At a
    // reload boundary, just after stop, on the first block, or in a fault, DMAxSRC can
    // hold an out-of-range value -- a bare ">= mid" test would then misclassify the half
    // and hand back a pointer to the half DMA is transmitting. Out of range -> NULL (the
    // caller NULL-checks and skips this block's fill).
    const uintptr_t base = (uintptr_t)&pTxDat[ DMA_BUF_PING_POS ];
    const uintptr_t mid  = (uintptr_t)&pTxDat[ half_pos ];
    const uintptr_t end  = (uintptr_t)&pTxDat[ 2u * half_pos ];
    const uintptr_t addr = (uintptr_t)dma_tx_addr;   // DMAxSRC snapshot as an address

    if( ( addr < base ) || ( addr >= end ) )
    {
        return;   // *dest_pptr stays NULL
    }

    if( addr >= mid )
    {
        // DMA is reading the Pong half -> SW fills Ping
        *dest_pptr = (int32_t*)&pTxDat[ DMA_BUF_PING_POS ];
    }
    else
    {
        // DMA is reading the Ping half -> SW fills Pong
        *dest_pptr = (int32_t*)&pTxDat[ half_pos ];
    }
}


/*
 * Validate the configuration envelope for ONE instance.
 *
 * The HAL accepts only what it can actually program/size: 32-bit words; geometry
 * (slots_per_fs / block_frames) that MATCHES this leg's compile-time buffer geometry
 * (its Rx_<name>/Tx_<name> are sized for exactly that, per the instance list); an
 * explicit role; and a framing the FRMCNT path supports -- I2S with 2 slots, or TDM
 * with 4/8/16/32 slots. This keeps configure() from programming untested framing or a
 * geometry the static buffers were not sized for.
 */
static bool tdm_config_is_supported( const tdm_spi_leg_t* leg, const dspic33ak_spi_i2s_tdm_config_t* cfg )
{
    if( ( cfg == NULL ) || ( leg == NULL ) )  return false;

    if( cfg->word_bits != 32u )         return false;
    // Geometry must match THIS leg's compile-time (statically allocated) geometry.
    if( cfg->slots_per_fs != leg->geom_slots_per_fs ) return false;
    if( cfg->block_frames != leg->geom_block_frames ) return false;

    // role must be an explicit SLAVE or MASTER -- otherwise a garbage value would be
    // silently treated as SLAVE everywhere (role == MASTER ? ... : SLAVE).
    if( ( cfg->role != DSPIC33AK_SPI_I2S_TDM_ROLE_SLAVE ) &&
        ( cfg->role != DSPIC33AK_SPI_I2S_TDM_ROLE_MASTER ) )
    {
        return false;
    }

    if( cfg->format == DSPIC33AK_SPI_I2S_TDM_FORMAT_I2S )
    {
        if( cfg->slots_per_fs != 2u )   return false;        // I2S = 2 slots (L/R)
    }
    else if( cfg->format == DSPIC33AK_SPI_I2S_TDM_FORMAT_TDM )
    {
        // TDM: FRMCNT supports FS every 4/8/16/32 words.
        if( ( cfg->slots_per_fs != 4u ) && ( cfg->slots_per_fs != 8u ) &&
            ( cfg->slots_per_fs != 16u ) && ( cfg->slots_per_fs != 32u ) )
        {
            return false;
        }
    }
    else
    {
        return false;
    }

    // Note: sample rate is NOT part of the transport envelope -- the core is
    // rate-agnostic (runs at the configured BRG / external clock). The product's
    // supported-rate policy lives in the app layer (APP_SAMPLE_RATE_IS_SUPPORTED), not here.
    return true;
}


/*
 * Read-and-clear / restore the CPU interrupt enable of an instance's RX DMA channel.
 *
 * Used to bracket non-atomic updates against the instance's RX-block ISR. Channel-
 * generic (via the DMA HAL), so it follows whatever DMA channel an instance is mapped
 * to in conf.h -- no per-channel hardcode. Returns the prior IE state; restore re-arms
 * only if it was enabled (so masking before start() leaves the IE off).
 */
static inline bool tdm_rx_ie_disable( uint8_t rx_dma_ch )
{
    return dspic33ak_dma_irq_disable_save( rx_dma_ch );
}

static inline void tdm_rx_ie_restore( uint8_t rx_dma_ch, bool was_enabled )
{
    dspic33ak_dma_irq_restore( rx_dma_ch, was_enabled );
}


/*
 * One SPI instance's RX-block ISR body (generic).
 *
 * Snapshots the instance's RX DMA status, maps the just-completed RX half and the
 * writable TX half of THIS instance, then invokes THIS instance's callback with
 * (src, dst, user). Called from the generated per-instance RX vector (the
 * _DMA<rx>Interrupt section above), which passes this instance's leg + channels.
 *
 * rx_ch / tx_ch are passed as compile-time constants from the vector so the DMA
 * register access (isr_snapshot / read_src) folds to direct register reads -- do NOT
 * call this with a runtime channel value. half_pos (= this instance's slots*blk) is
 * likewise a compile-time literal so the ping/pong pointer math stays folded.
 */
static inline void tdm_rx_block( tdm_spi_leg_t* inst, uint8_t rx_ch, uint8_t tx_ch, uint32_t half_pos )
{
          uint32_t  dma_stat;
    const int32_t*  src_ptr = NULL;
          int32_t*  dst_ptr = NULL;

    dspic33ak_spi_i2s_tdm_diag_isr_begin( &inst->diag );

    dma_stat = dspic33ak_dma_isr_snapshot( rx_ch );

    // Stream-health check; diagnostic print is debug-only. Each instance counts its
    // own deadline misses in its own diag (no shared/master counter).
    dspic33ak_spi_i2s_tdm_diag_check_deadline( &inst->diag, rx_ch, dma_stat );

    // Map this instance's completed RX half (callback input) and the TX half it may
    // fill (callback output). Each instance handles only its own RX/TX -- no dst_b.
    tdm_get_src_ptr( dma_stat, inst->rx_buffer, half_pos, &src_ptr );
    if( src_ptr == NULL )
    {
        dspic33ak_spi_i2s_tdm_diag_isr_end( &inst->diag );
        return;
    }
    tdm_get_dest_ptr( dspic33ak_dma_read_src( tx_ch ), inst->tx_buffer, half_pos, &dst_ptr );
    if( dst_ptr == NULL )
    {
        // TX-DMA source is out of the buffer envelope (reload boundary / just-stopped /
        // first block / fault): there is no writable TX half this block. Skip rather than
        // hand the callback a NULL dst -- the public contract is that dst is always valid
        // when block_cb runs. (Mirrors the src_ptr guard above.)
        dspic33ak_spi_i2s_tdm_diag_isr_end( &inst->diag );
        return;
    }

    dspic33ak_spi_i2s_tdm_diag_note_block( &inst->diag );   // one completed block (read via get_status)

    // Deliver the completed block through this instance's registered callback. The
    // callee owns its DSP work buffers; the driver passes only this instance's
    // selected RX/TX ping-pong halves (both guaranteed non-NULL here). No callback ->
    // no app/DSP path: start() zeroes the TX half so a fresh start is silent until a
    // callback fills it (clearing the callback mid-stream leaves the last TX data looping).
    if( inst->block_cb != NULL )
    {
        inst->block_cb( src_ptr, dst_ptr, inst->block_user );
    }

    dspic33ak_spi_i2s_tdm_diag_isr_end( &inst->diag );
}


// The demo/application audio path (local_copy_to_CODEC,
// local_drc_df2t_path, local_filter_cascade_chm) and the optional PWM audio
// output live in the demo app (audio_app.c). The HAL core does NOT
// call them: each instance's RX-block handler delivers its block to the registered
// block callback only (no app fallback). The HAL core owns the DMA ping-pong buffers;
// the demo app owns its f_*_Data DSP work buffers.
