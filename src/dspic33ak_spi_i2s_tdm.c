
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
#include "dspic33ak_spi_i2s_tdm_fs_clc.h"  // CLC10 50%-FS generator (TDM master + FS_50PCT)
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
// Explicit dense leg-index enum: one TDM_SPI_LEG_<name> per descriptor row, in table order,
// terminated by TDM_SPI_LEG_COUNT (= the built-in leg count). Used only inside the core
// (leg table / inst()), so it lives here rather than in a shared header. The SPI1/SPI2 row
// names are legacy logical labels; DSPIC33AK_TDM_BASE_ON_SPI34 maps those same rows to
// physical SPI3/SPI4. Public spiN() accessors search the stored physical spi_inst instead.
// Adding a leg = add an enumerator here + its buffers, s_spi_legs[] row, and RX vector below.
typedef enum {
    TDM_SPI_LEG_SPI1 = 0,
#if DSPIC33AK_TDM_USE_SPI2
    TDM_SPI_LEG_SPI2,
#endif
#if DSPIC33AK_TDM_USE_SPI3
    TDM_SPI_LEG_SPI3,
#endif
#if DSPIC33AK_TDM_USE_SPI4
    TDM_SPI_LEG_SPI4,
#endif
    TDM_SPI_LEG_COUNT
} tdm_spi_leg_index_t;

// Which leg the arg-less singleton status API reports is the stream's primary_leg_index
// (tdm_stream_t), defaulting to logical leg 0. It is NOT a clock master and NOT a hard
// timing coupling -- every leg times itself via its own RX-block ISR; the clock role
// (master/slave) is a separate per-leg concern in config.clock_role.

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
    // Sync domain id: legs sharing a domain are co-clocked and started phase-locked as a group
    // (start_domain arms all members, then releases SPIEN back-to-back so their ping-pong DMAs
    // latch one FS edge). Legs in DIFFERENT domains are independent/async (e.g. ASRC A vs B).
    // Orthogonal to config.clock_role (the clock driver). Which leg the arg-less singleton
    // status API reports is the stream's primary_leg_index (tdm_stream_t), not a per-leg flag.
    uint8_t        sync_domain;
    dspic33ak_spi_i2s_tdm_config_t config;         // includes this leg's OWN clock role
    bool           config_valid;
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
    // Index (< leg_count) of the PRIMARY leg: the one the arg-less singleton status API
    // (is_running/is_active/get_status/get_load) reports. Every leg times itself via its
    // own RX-block ISR, so this is only a reporting default, NOT a clock-role or timing
    // coupling. Replaces the former per-leg is_block_timing_master "exactly one, leg 0" flag.
    uint8_t                             primary_leg_index;
    // open() succeeded (external clock up + pins/CLC routed via the port). start/arm are
    // gated on this so a caller cannot enter SPIEN with the port unrouted / readiness
    // unchecked (a silent dead stream). Set by open(), cleared by close().
    bool                                opened;
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
static const tdm_spi_leg_t *tdm_stream_primary_leg( const tdm_stream_t *stream );
static bool        tdm_spi_leg_get_effective_config( const tdm_spi_leg_t *leg,
                                                     dspic33ak_spi_i2s_tdm_config_t *effective_cfg );

// Config-mode + lifecycle helpers (definitions below). tdm_inst_stop_impl is the
// mode-agnostic per-leg teardown that both the public inst_stop() gate and the SYSTEM
// domain teardown (stop_domain/stop_all_domains) call; tdm_stream_ready_for_start()
// re-checks the clock-readiness gate just before a start; tdm_any_leg_running() /
// tdm_inst_is_primary() back the lifecycle gates.
static void        tdm_inst_stop_impl( tdm_spi_leg_t *inst );
static bool        tdm_stream_ready_for_start( void );
static bool        tdm_any_leg_running( void );
static bool        tdm_inst_is_primary( const tdm_spi_leg_t *inst );
// tdm_set_error() is defined further down (static inline); forward-declare it because the
// lifecycle setters above its definition (set_port/close) now record an error code.
static inline void tdm_set_error( dspic33ak_spi_i2s_tdm_error_t err );

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





//===========================================================
// Variables
//===========================================================

#define DMA_BUF_PING_POS     (0)   // ping half is always at offset 0; pong half = slots*blk per leg

// Per-instance ping/pong half size (words) = slots * blk for THIS row. Both the buffer
// size (2 * half) and the ISR's pong-half offset derive from it; passed as a compile-
// time literal into the explicit per-leg ISR bodies so the hot-path pointer math stays folded.
#define TDM_LEG_HALF_WORDS(slots, blk)   ((slots) * (blk))

// Explicit per-leg RX/TX ping-pong buffers. Each is 2*slots*blk words (Ping+Pong). The macro takes
// per-leg slots/blk so a future build COULD give a leg its own geometry, but this build sizes every
// leg from the single global DSPIC33AK_TDM_SLOTS_PER_FS / _BLOCK_FRAMES. The names follow the leg
// names so the leg table can wire them; same geometry macros as the leg table + ISR so the three
// stay consistent.
// Keep each complete ping-pong array on its own-size boundary. Besides making the DMA base
// deterministic as unrelated BSS grows, this prevents one transfer array from straddling a
// data-RAM bank boundary (observed when the bidirectional ASRC state grew from 8ch to 16ch).
#define TDM_DMA_BUFFER_BYTES \
    (2u * TDM_LEG_HALF_WORDS(DSPIC33AK_TDM_SLOTS_PER_FS, DSPIC33AK_TDM_BLOCK_FRAMES) * sizeof(int32_t))
#define TDM_DMA_BUFFER_ALIGN TDM_DMA_BUFFER_BYTES
TDM_COMPILEASSERT( (TDM_DMA_BUFFER_ALIGN & (TDM_DMA_BUFFER_ALIGN - 1u)) == 0u );
static int32_t    Tx_SPI1[ 2 * TDM_LEG_HALF_WORDS(DSPIC33AK_TDM_SLOTS_PER_FS, DSPIC33AK_TDM_BLOCK_FRAMES) ] __attribute__((aligned(TDM_DMA_BUFFER_ALIGN)));
static int32_t    Rx_SPI1[ 2 * TDM_LEG_HALF_WORDS(DSPIC33AK_TDM_SLOTS_PER_FS, DSPIC33AK_TDM_BLOCK_FRAMES) ] __attribute__((aligned(TDM_DMA_BUFFER_ALIGN)));
#if DSPIC33AK_TDM_USE_SPI2
static int32_t    Tx_SPI2[ 2 * TDM_LEG_HALF_WORDS(DSPIC33AK_TDM_SLOTS_PER_FS, DSPIC33AK_TDM_BLOCK_FRAMES) ] __attribute__((aligned(TDM_DMA_BUFFER_ALIGN)));
static int32_t    Rx_SPI2[ 2 * TDM_LEG_HALF_WORDS(DSPIC33AK_TDM_SLOTS_PER_FS, DSPIC33AK_TDM_BLOCK_FRAMES) ] __attribute__((aligned(TDM_DMA_BUFFER_ALIGN)));
#endif
#if DSPIC33AK_TDM_USE_SPI3
static int32_t    Tx_SPI3[ 2 * TDM_LEG_HALF_WORDS(DSPIC33AK_TDM_SLOTS_PER_FS, DSPIC33AK_TDM_BLOCK_FRAMES) ] __attribute__((aligned(TDM_DMA_BUFFER_ALIGN)));
static int32_t    Rx_SPI3[ 2 * TDM_LEG_HALF_WORDS(DSPIC33AK_TDM_SLOTS_PER_FS, DSPIC33AK_TDM_BLOCK_FRAMES) ] __attribute__((aligned(TDM_DMA_BUFFER_ALIGN)));
#endif
#if DSPIC33AK_TDM_USE_SPI4
static int32_t    Tx_SPI4[ 2 * TDM_LEG_HALF_WORDS(DSPIC33AK_TDM_SLOTS_PER_FS, DSPIC33AK_TDM_BLOCK_FRAMES) ] __attribute__((aligned(TDM_DMA_BUFFER_ALIGN)));
static int32_t    Rx_SPI4[ 2 * TDM_LEG_HALF_WORDS(DSPIC33AK_TDM_SLOTS_PER_FS, DSPIC33AK_TDM_BLOCK_FRAMES) ] __attribute__((aligned(TDM_DMA_BUFFER_ALIGN)));
#endif

// Application processing buffers are outside the HAL's ownership. The HAL core owns and clears
// only its RX/TX DMA ping-pong buffers (the Rx_<name>/Tx_<name> pair per leg above).


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

// One row per physical SPI leg: physical SPI + RX/TX DMA channels + its own
// Rx_<name>/Tx_<name> ping-pong buffers. Each leg's CLOCK role is NOT forced here -- it
// comes from the leg's own config (set by the integrator per leg: a co-clocked follower is
// configured SLAVE because it rides the shared clock, not because the HAL forces it). The
// singleton-reporting leg is the stream's primary_leg_index, not a per-row flag.
static tdm_spi_leg_t s_spi_legs[] =
{
    [TDM_SPI_LEG_SPI1] =
    {
#if DSPIC33AK_TDM_BASE_ON_SPI34
        .spi_inst               = TDM_SPI3,
        .rx_dma_ch              = DSPIC33AK_TDM_SPI3_RX_DMA,
        .tx_dma_ch              = DSPIC33AK_TDM_SPI3_TX_DMA,
#else
        .spi_inst               = TDM_SPI1,
        .rx_dma_ch              = DSPIC33AK_TDM_SPI1_RX_DMA,
        .tx_dma_ch              = DSPIC33AK_TDM_SPI1_TX_DMA,
#endif
        .rx_buffer              = Rx_SPI1,
        .tx_buffer              = Tx_SPI1,
        .buffer_word_count      = TDM_ARRAY_SIZE(Rx_SPI1),
        .geom_slots_per_fs      = (uint8_t)(DSPIC33AK_TDM_SLOTS_PER_FS),
        .geom_block_frames      = (uint16_t)(DSPIC33AK_TDM_BLOCK_FRAMES),
        .sync_domain            = (uint8_t)(DSPIC33AK_TDM_SPI1_SYNC_DOMAIN),
        .block_cb               = NULL,
        .block_user             = NULL,
        .diag                   = { .isr_min_count = 0xFFFFFFFFUL },  /* rest zero; start() calls diag_reset() */
    },
#if DSPIC33AK_TDM_USE_SPI2
    [TDM_SPI_LEG_SPI2] =
    {
#if DSPIC33AK_TDM_BASE_ON_SPI34
        .spi_inst               = TDM_SPI4,
        .rx_dma_ch              = DSPIC33AK_TDM_SPI4_RX_DMA,
        .tx_dma_ch              = DSPIC33AK_TDM_SPI4_TX_DMA,
#else
        .spi_inst               = TDM_SPI2,
        .rx_dma_ch              = DSPIC33AK_TDM_SPI2_RX_DMA,
        .tx_dma_ch              = DSPIC33AK_TDM_SPI2_TX_DMA,
#endif
        .rx_buffer              = Rx_SPI2,
        .tx_buffer              = Tx_SPI2,
        .buffer_word_count      = TDM_ARRAY_SIZE(Rx_SPI2),
        .geom_slots_per_fs      = (uint8_t)(DSPIC33AK_TDM_SLOTS_PER_FS),
        .geom_block_frames      = (uint16_t)(DSPIC33AK_TDM_BLOCK_FRAMES),
        .sync_domain            = (uint8_t)(DSPIC33AK_TDM_SPI2_SYNC_DOMAIN),
        .block_cb               = NULL,
        .block_user             = NULL,
        .diag                   = { .isr_min_count = 0xFFFFFFFFUL },
    },
#endif // DSPIC33AK_TDM_USE_SPI2
#if DSPIC33AK_TDM_USE_SPI3
    [TDM_SPI_LEG_SPI3] =
    {
        .spi_inst               = TDM_SPI3,
        .rx_dma_ch              = DSPIC33AK_TDM_SPI3_RX_DMA,
        .tx_dma_ch              = DSPIC33AK_TDM_SPI3_TX_DMA,
        .rx_buffer              = Rx_SPI3,
        .tx_buffer              = Tx_SPI3,
        .buffer_word_count      = TDM_ARRAY_SIZE(Rx_SPI3),
        .geom_slots_per_fs      = (uint8_t)(DSPIC33AK_TDM_SLOTS_PER_FS),
        .geom_block_frames      = (uint16_t)(DSPIC33AK_TDM_BLOCK_FRAMES),
        .sync_domain            = (uint8_t)(DSPIC33AK_TDM_SPI3_SYNC_DOMAIN),
        .block_cb               = NULL,
        .block_user             = NULL,
        .diag                   = { .isr_min_count = 0xFFFFFFFFUL },
    },
#endif // DSPIC33AK_TDM_USE_SPI3
#if DSPIC33AK_TDM_USE_SPI4
    [TDM_SPI_LEG_SPI4] =
    {
        .spi_inst               = TDM_SPI4,
        .rx_dma_ch              = DSPIC33AK_TDM_SPI4_RX_DMA,
        .tx_dma_ch              = DSPIC33AK_TDM_SPI4_TX_DMA,
        .rx_buffer              = Rx_SPI4,
        .tx_buffer              = Tx_SPI4,
        .buffer_word_count      = TDM_ARRAY_SIZE(Rx_SPI4),
        .geom_slots_per_fs      = (uint8_t)(DSPIC33AK_TDM_SLOTS_PER_FS),
        .geom_block_frames      = (uint16_t)(DSPIC33AK_TDM_BLOCK_FRAMES),
        .sync_domain            = (uint8_t)(DSPIC33AK_TDM_SPI4_SYNC_DOMAIN),
        .block_cb               = NULL,
        .block_user             = NULL,
        .diag                   = { .isr_min_count = 0xFFFFFFFFUL },
    },
#endif // DSPIC33AK_TDM_USE_SPI4
};

static tdm_stream_t s_stream =
{
    .legs              = s_spi_legs,
    .leg_count         = (uint8_t)TDM_ARRAY_SIZE(s_spi_legs),
    .primary_leg_index = (uint8_t)TDM_SPI_LEG_SPI1,   // logical leg 0 = co-clock anchor / singleton-reporting leg
    .opened            = false,
    .port              = NULL,
};

// Configuration OWNERSHIP mode: how the currently-committed config was applied, which
// selects the mutually-exclusive API surface. It is a property of the committed
// CONFIGURATION, NOT the open/close lifecycle -- close() does NOT reset it (a closed
// stream keeps its committed shape so the next open()->start uses the same API family).
//   NONE   : nothing configured yet.
//   SINGLE : committed via inst_configure() -> the per-leg PRIMARY-only API
//            (inst_configure / inst_start / inst_stop) is in force.
//   SYSTEM : committed via configure_system() -> the whole-system domain API
//            (configure_system / start_domain / start_all_domains / stop_domain /
//            stop_all_domains) is in force.
// configure_system() may full-recommit from ANY mode (NONE/SINGLE/SYSTEM) while CLOSED (before
// open(); an already-open engine is rejected with ERR_ALREADY_OPEN, so recommit implies stopped);
// once SYSTEM, inst_configure() is rejected (a system caller must stay transactional).
typedef enum {
    TDM_CONFIG_MODE_NONE = 0,
    TDM_CONFIG_MODE_SINGLE,
    TDM_CONFIG_MODE_SYSTEM,
} tdm_config_mode_t;

static tdm_config_mode_t s_config_mode = TDM_CONFIG_MODE_NONE;

// Is this leg the stream's PRIMARY leg (the only leg the SINGLE per-leg API may touch)?
static bool tdm_inst_is_primary( const tdm_spi_leg_t *inst )
{
    return ( inst != NULL ) && ( inst == tdm_stream_primary_leg( &s_stream ) );
}

// Any leg currently running? Backs the close()/set_port() "not while streaming" guards.
static bool tdm_any_leg_running( void )
{
    const uint8_t n = s_stream.leg_count;
    for( uint8_t i = 0u; i < n; i++ )
    {
        const tdm_spi_leg_t *leg = &s_stream.legs[i];
        if( leg->running )
        {
            return true;
        }
    }
    return false;
}

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
// get_load()/get_status() report the primary leg's diag under its RX DMA IE mask because
// 32-bit reads are non-atomic on this 16-bit core. The deadline metric is
// software/real-time, not SPIROV/SPITUR hardware status.

// Lifecycle running state is owned PER INSTANCE (tdm_spi_leg_t.running): set by a
// successful inst_start() and cleared by inst_stop(), exposed via
// inst_get_status().running and (for the primary leg) is_running(). This is
// deliberately separate from is_active(), which reports clock/source readiness.

// The leg table must have at least one leg and not exceed the silicon SPI count.
TDM_COMPILEASSERT( TDM_ARRAY_SIZE(s_spi_legs) >= 1u );
TDM_COMPILEASSERT( TDM_ARRAY_SIZE(s_spi_legs) <= (size_t)TDM_SPI_INST_COUNT );

// Per-instance geometry sanity (compile-time), one set per leg: slots/blk
// fit their leg fields (uint8_t / uint16_t), the 2*slots*blk word count cannot overflow
// int32 indexing, and the generated buffer is exactly that size. The Perseus rows use the
// stream-wide macros (already range-checked in conf.h); these guard a standalone carve-out
// that gives a row its own slots/blk -- so the per-instance-geometry promise has teeth.
TDM_COMPILEASSERT( (DSPIC33AK_TDM_SLOTS_PER_FS) > 0 && (DSPIC33AK_TDM_SLOTS_PER_FS) <= 255 );
TDM_COMPILEASSERT( (DSPIC33AK_TDM_BLOCK_FRAMES) > 0 && (DSPIC33AK_TDM_BLOCK_FRAMES) <= 65535 );
TDM_COMPILEASSERT( (DSPIC33AK_TDM_SLOTS_PER_FS) <= (2147483647 / (2 * (DSPIC33AK_TDM_BLOCK_FRAMES))) );
TDM_COMPILEASSERT( TDM_ARRAY_SIZE(Rx_SPI1) == (2u * (DSPIC33AK_TDM_SLOTS_PER_FS) * (DSPIC33AK_TDM_BLOCK_FRAMES)) );
#if DSPIC33AK_TDM_USE_SPI2
TDM_COMPILEASSERT( TDM_ARRAY_SIZE(Rx_SPI2) == (2u * (DSPIC33AK_TDM_SLOTS_PER_FS) * (DSPIC33AK_TDM_BLOCK_FRAMES)) );
#endif
#if DSPIC33AK_TDM_USE_SPI3
TDM_COMPILEASSERT( TDM_ARRAY_SIZE(Rx_SPI3) == (2u * (DSPIC33AK_TDM_SLOTS_PER_FS) * (DSPIC33AK_TDM_BLOCK_FRAMES)) );
#endif
#if DSPIC33AK_TDM_USE_SPI4
TDM_COMPILEASSERT( TDM_ARRAY_SIZE(Rx_SPI4) == (2u * (DSPIC33AK_TDM_SLOTS_PER_FS) * (DSPIC33AK_TDM_BLOCK_FRAMES)) );
#endif




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
 *
 * Rejected (false, port unchanged) while the port is already open()'d or any leg is
 * running: open() consumes the port hooks (clock/pins/CLC), so swapping the port after
 * open -- or under a live stream -- would leave the routed hardware disagreeing with the
 * registered hooks. Call it before open() (typically once at init).
 */
bool dspic33ak_spi_i2s_tdm_set_port( const dspic33ak_spi_i2s_tdm_port_t* port )
{
    if( s_stream.opened || tdm_any_leg_running() )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_ALREADY_OPEN );
        return false;
    }
    s_stream.port = port;
    tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NONE );
    return true;
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
        const tdm_spi_leg_t *primary = tdm_stream_primary_leg( stream );
        dspic33ak_spi_i2s_tdm_clock_role_t role =
            ( ( primary != NULL ) && primary->config_valid ) ? primary->config.clock_role
                                                             : DSPIC33AK_SPI_I2S_TDM_CLOCK_SLAVE;
        return stream->port->clock_source_ready( role );
    }
    return true;
}


/*
 * Re-check the clock-readiness gate immediately before a start (arm).
 *
 * open() checks readiness ONCE; between open() and start() an external BCLK/FS could drop.
 * The start paths (inst_start / start_domain / start_all_domains) call this just before
 * arming so a stream is not entered with the source already gone. Same gate as is_active()
 * / open(): keyed on the PRIMARY leg's role and routed through the port. A self-clocked
 * master (no port, or no clock_source_ready hook) is always ready. Returns true = go.
 *
 * READINESS SCOPE (by design): this gate is ENGINE-WIDE and PRIMARY-leg-gated -- it asks the
 * port about the primary leg's clock role, NOT per-domain. A sync_domain separates only the
 * START PHASE (which legs latch one FS edge together) and the clock topology (who is master);
 * it does NOT carry its own source-readiness. So even a nominally "independent" master domain
 * (e.g. an ASRC SPI2 on sync_domain 1) is gated on the primary leg's readiness here: if the
 * primary's external clock is not ready, start_domain() on any domain returns CLOCK_NOT_READY.
 * That matches the Perseus product policy ("no A clock -> hold the whole transport"). Per-domain
 * source readiness is intentionally NOT supported; revisit (a per-domain readiness hook) before
 * relying on truly independent async domains in a generic standalone reuse.
 */
static bool tdm_stream_ready_for_start( void )
{
    return dspic33ak_spi_i2s_tdm_is_active();
}


/*
 * Report the engine's running state = the PRIMARY leg (primary_leg_index, default logical leg 0).
 *
 * Set by a successful inst_start() of the primary leg and cleared by its inst_stop(). It
 * is separate from is_active(), which only means the clock/source gate is ready. For
 * a specific instance use inst_get_status().running.
 */
bool dspic33ak_spi_i2s_tdm_is_running( void )
{
    const tdm_spi_leg_t *primary = tdm_stream_primary_leg( &s_stream );
    return ( primary != NULL ) && primary->running;
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
    // clock to detect). A board port typically wires this to an external-clock edge detector.
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
 * Number of SPI instances this build has (the size of the leg table).
 * Pair with inst(i) to enumerate: for i in 0 .. instance_count()-1.
 */
uint8_t dspic33ak_spi_i2s_tdm_instance_count( void )
{
    return s_stream.leg_count;
}

/*
 * Return the handle for one SPI instance, or NULL if index is out of range.
 *
 * index is a dense logical leg index in table order (0 = primary leg). Physical spiN()
 * accessors below search the descriptor table and return the row that actually owns SPIn;
 * therefore their meaning remains literal even when a board maps logical legs 0/1 onto
 * physical SPI3/SPI4.
 */
dspic33ak_spi_i2s_tdm_inst_t* dspic33ak_spi_i2s_tdm_inst( uint8_t index )
{
    if( index >= (uint8_t)TDM_SPI_LEG_COUNT )
    {
        return NULL;
    }
    return &s_spi_legs[index];
}

static dspic33ak_spi_i2s_tdm_inst_t* tdm_find_physical_spi( tdm_spi_inst_t spi_inst )
{
    uint8_t i;

    for( i = 0u; i < s_stream.leg_count; i++ )
    {
        if( s_stream.legs[i].spi_inst == spi_inst )
        {
            return &s_stream.legs[i];
        }
    }
    return NULL;
}

dspic33ak_spi_i2s_tdm_inst_t* dspic33ak_spi_i2s_tdm_spi1( void )
{
    return tdm_find_physical_spi( TDM_SPI1 );
}

dspic33ak_spi_i2s_tdm_inst_t* dspic33ak_spi_i2s_tdm_spi2( void )
{
    return tdm_find_physical_spi( TDM_SPI2 );
}

dspic33ak_spi_i2s_tdm_inst_t* dspic33ak_spi_i2s_tdm_spi3( void )
{
    return tdm_find_physical_spi( TDM_SPI3 );
}

dspic33ak_spi_i2s_tdm_inst_t* dspic33ak_spi_i2s_tdm_spi4( void )
{
#if DSPIC33AK_SPI_I2S_TDM_DEVICE == DSPIC33AK_SPI_I2S_TDM_DEV_AK512
    return tdm_find_physical_spi( TDM_SPI4 );
#else
    return NULL;
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
 * Return one instance's TX ping-pong half by MIRRORING a reference instance's fill half,
 * instead of reading this instance's own live DMA position.
 *
 * For the co-clocked single-producer dual-codec demo: SPI1's block callback fills BOTH
 * codecs from the SAME callback. Using inst_tx_fill_ptr(spi2) there reads SPI2's live TX
 * DMA and returns "the half not being transmitted NOW" -- a snapshot sampled at callback
 * start that can go stale (SPI2 crosses its own block boundary while the long callback runs)
 * -> the write lands in the half SPI2 is transmitting -> tearing under high load. Mirroring
 * removes the live read: the returned half is whichever half of THIS instance corresponds to
 * the reference's just-handed fill half (`ref_fill_half`, i.e. SPI1's `dst`). Because the two
 * co-clocked legs share the ping-pong phase and equal block geometry, the mirrored half is
 * the correct (safe, not-transmitting) half for the WHOLE block -- deterministic, no live-DMA
 * snapshot, A and B sample-aligned (same generation, zero skew). Restores the pre-refactor
 * single-DMA0 (src,dst_a,dst_b) behaviour on the independent-instance HAL.
 *
 * Returns a typed result (see the header enum). On OK, *dst = the writable (not-transmitting)
 * target half. On BAD_ARGUMENT (NULL arg / stopped inst / ref_fill_half outside ref's buffer),
 * UNSAFE_ACTIVE_HALF (inst is transmitting the target half NOW) or UNRESOLVED_DMA_POSITION (inst's
 * live TX-DMA address is out of buffer range -- reload boundary / just-started / fault), *dst=NULL
 * and the caller must NOT write B this block. The target half itself is deterministic (from
 * ref_fill_half); the live-DMA read is only the secondary veto that produces UNSAFE/UNRESOLVED.
 */
dspic33ak_spi_i2s_tdm_mirror_result_t dspic33ak_spi_i2s_tdm_inst_tx_fill_mirror(
        dspic33ak_spi_i2s_tdm_inst_t*       inst,
        const dspic33ak_spi_i2s_tdm_inst_t* ref,
        const int32_t*                      ref_fill_half,
        int32_t**                           dst )
{
    if( dst == NULL )
    {
        return DSPIC33AK_TDM_MIRROR_BAD_ARGUMENT;
    }
    *dst = NULL;   // fail-closed default: only OK sets a non-NULL pointer
    if( ( inst == NULL ) || ( ref == NULL ) || ( ref_fill_half == NULL ) )
    {
        return DSPIC33AK_TDM_MIRROR_BAD_ARGUMENT;
    }
    // inst and ref must be handles returned by this HAL's accessors (spiN()/inst(i)).
    // tdm_spi_leg_is_valid() checks each descriptor's local invariants (known SPI instance,
    // distinct RX/TX channels, non-NULL buffers) before use; it is not a defense against an
    // arbitrary bogus pointer. Reject on that check or a stopped inst as BAD_ARGUMENT.
    if( !tdm_spi_leg_is_valid( inst ) || !tdm_spi_leg_is_valid( ref ) || !inst->running )
    {
        return DSPIC33AK_TDM_MIRROR_BAD_ARGUMENT;
    }
    const uint32_t ref_half  = (uint32_t)ref->geom_slots_per_fs  * ref->geom_block_frames;
    const uint32_t inst_half = (uint32_t)inst->geom_slots_per_fs * inst->geom_block_frames;

    // Which half of ref is ref_fill_half? [base, base+ref_half) = ping (index 0); the pong half
    // starts at base+ref_half. Reject a pointer outside ref's [base, base+2*ref_half). Compare in
    // the integer domain (uintptr_t): comparing pointers into DIFFERENT array objects is undefined
    // in C, so a bogus ref_fill_half must be range-checked as an integer -- same idiom as the inst
    // live-DMA address check below.
    const uintptr_t rbase = (uintptr_t)&ref->tx_buffer[ 0 ];
    const uintptr_t rmid  = (uintptr_t)&ref->tx_buffer[ ref_half ];
    const uintptr_t rend  = (uintptr_t)&ref->tx_buffer[ 2u * ref_half ];
    const uintptr_t raddr = (uintptr_t)ref_fill_half;
    if( ( raddr < rbase ) || ( raddr >= rend ) )
    {
        return DSPIC33AK_TDM_MIRROR_BAD_ARGUMENT;
    }
    const bool     pong       = ( raddr >= rmid );
    const uint32_t target_off = pong ? inst_half : 0u;   // words: the half we would fill

    // Live safety veto. With a phase-locked start the mirrored (ref-fill) half is always inst's
    // NON-transmitting half. Two abnormal cases must NOT authorize a write:
    //   - live TX-DMA address OUT of inst's buffer (reload boundary / just-started / fault): the
    //     active half is UNRESOLVABLE -> fail-closed as UNRESOLVED (caller tolerates a few
    //     consecutive as a transient, resyncs only if persistent). (Was fail-OPEN before.)
    //   - live address IN range AND on the very half we'd fill: a real phase problem -> UNSAFE.
    const uintptr_t ibase = (uintptr_t)&inst->tx_buffer[ 0 ];
    const uintptr_t imid  = (uintptr_t)&inst->tx_buffer[ inst_half ];
    const uintptr_t iend  = (uintptr_t)&inst->tx_buffer[ 2u * inst_half ];
    const uintptr_t iaddr = (uintptr_t)dspic33ak_dma_read_src( inst->tx_dma_ch );
    if( ( iaddr < ibase ) || ( iaddr >= iend ) )
    {
        return DSPIC33AK_TDM_MIRROR_UNRESOLVED_DMA_POSITION;
    }
    const uint32_t active_off = ( iaddr >= imid ) ? inst_half : 0u;
    if( active_off == target_off )
    {
        return DSPIC33AK_TDM_MIRROR_UNSAFE_ACTIVE_HALF;
    }
    *dst = inst->tx_buffer + target_off;
    return DSPIC33AK_TDM_MIRROR_OK;
}


/*
 * Diagnostic: which TX ping-pong half is this instance's DMA CURRENTLY transmitting?
 *   0 = ping (first half), 1 = pong (second half), -1 = unresolved (stopped, or the live
 *   DMAxSRC snapshot is outside this buffer -- reload boundary / just-started / fault).
 *
 * Phase-probe use only (measure SPI1 vs SPI2 ping-pong alignment for the co-clocked
 * single-producer path). Reads the live DMA source address, so sample it at a
 * deterministic instant (e.g. a block-boundary ISR) and compare two co-clocked legs.
 */
int dspic33ak_spi_i2s_tdm_inst_tx_active_half( dspic33ak_spi_i2s_tdm_inst_t* inst )
{
    if( ( inst == NULL ) || !inst->running )
    {
        return -1;
    }
    const uint32_t  half = (uint32_t)inst->geom_slots_per_fs * inst->geom_block_frames;
    const uintptr_t base = (uintptr_t)&inst->tx_buffer[ 0 ];
    const uintptr_t mid  = (uintptr_t)&inst->tx_buffer[ half ];
    const uintptr_t end  = (uintptr_t)&inst->tx_buffer[ 2u * half ];
    const uintptr_t addr = (uintptr_t)dspic33ak_dma_read_src( inst->tx_dma_ch );

    if( ( addr < base ) || ( addr >= end ) )
    {
        return -1;
    }
    return ( addr >= mid ) ? 1 : 0;
}


/*
 * Diagnostic: the TX DMA's CURRENT read position as a word offset into the full ping-pong
 * buffer [0, 2*half). Returns -1 if stopped or the live DMAxSRC snapshot is out of range.
 *
 * Finer than tx_active_half(): lets a phase probe measure the SUB-block sample offset
 * between two co-clocked legs (equal half but different position => a fixed offset that can
 * still tear a late cross-fill write). Sample at a deterministic instant and diff two legs.
 */
int32_t dspic33ak_spi_i2s_tdm_inst_tx_active_pos( dspic33ak_spi_i2s_tdm_inst_t* inst )
{
    if( ( inst == NULL ) || !inst->running )
    {
        return -1;
    }
    const uint32_t  half = (uint32_t)inst->geom_slots_per_fs * inst->geom_block_frames;
    const uintptr_t base = (uintptr_t)&inst->tx_buffer[ 0 ];
    const uintptr_t end  = (uintptr_t)&inst->tx_buffer[ 2u * half ];
    const uintptr_t addr = (uintptr_t)dspic33ak_dma_read_src( inst->tx_dma_ch );

    if( ( addr < base ) || ( addr >= end ) )
    {
        return -1;
    }
    return (int32_t)( ( addr - base ) / sizeof(int32_t) );   // 0 .. 2*half-1
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
 * Open the shared board/clock port for the engine, ONCE, before any instance is started.
 *
 * Takes NO role argument: the clock role handed to the port hooks is DERIVED from the
 * committed primary leg (primary_leg_index), so the pin/clock direction can never disagree
 * with the configured stream (a caller can no longer pass a role that contradicts the
 * committed config). Brings up + checks the external clock and routes pins/CLC through the
 * registered port hooks (all optional). Returns false if the primary leg is not configured
 * (ERR_NOT_CONFIGURED), the external clock cannot be brought up / is not ready, or a pin/CLC
 * hook rejects the role -- the caller must then not start any instance. With no port
 * registered this is a no-op success (self-clocked). It does NOT block waiting for a clock
 * (single readiness check) and does NOT touch any SPI/DMA -- per-instance start arms the
 * hardware. A board hook that also routes a SECONDARY leg reads that leg's committed role via
 * dspic33ak_spi_i2s_tdm_inst_get_setup() and skips a leg left unconfigured.
 */
bool dspic33ak_spi_i2s_tdm_open( void )
{
    const tdm_stream_t *stream = &s_stream;

    // Idempotent: a second open() while already open must NOT re-run the port hooks (they
    // have side effects -- external-clock bring-up, CLC engage) -- just succeed.
    if( s_stream.opened )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NONE );
        return true;
    }

    // Verify the shared-engine topology ONCE here, before any clock/pin bring-up:
    // primary_leg_index in range, distinct physical SPIs, and distinct DMA channels across
    // all legs. This catches a leg-table misconfig (e.g. two legs on the same SPI, or a
    // crossed DMA channel) that the per-leg tdm_spi_leg_is_valid() check at configure/start
    // cannot see.
    if( !tdm_stream_topology_is_valid( stream ) )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_TOPOLOGY );
        return false;
    }

    // Derive the clock role from the COMMITTED primary leg (never from a caller argument).
    // The primary MUST be configured -- open() with an unconfigured primary is a contract
    // error, not a silent SLAVE default.
    const tdm_spi_leg_t *primary = tdm_stream_primary_leg( stream );
    if( ( primary == NULL ) || !primary->config_valid )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NOT_CONFIGURED );
        return false;
    }
    const dspic33ak_spi_i2s_tdm_clock_role_t role = primary->config.clock_role;

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
    s_stream.opened = true;   // port up: start/arm may now proceed
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
 *
 * Rejected (false, stays open) while any leg is still running: closing under a live stream
 * would make the lifecycle state (closed) disagree with the hardware (SPI/DMA still on).
 * Stop every leg first (inst_stop / stop_all_domains). Does NOT change the config mode --
 * mode is a property of the committed configuration, not of open/close, so a closed stream
 * keeps its SINGLE/SYSTEM shape for the next open()->start.
 */
bool dspic33ak_spi_i2s_tdm_close( void )
{
    if( tdm_any_leg_running() )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_ALREADY_RUNNING );
        return false;
    }
    s_stream.opened = false;   // a fresh open() is required before the next start/arm
    // No hardware teardown by design (see above). Config mode is intentionally preserved.
    tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NONE );
    return true;
}


// The board-specific config lives in the integrator's board/platform layer (a board TDM-config
// table). The core no longer fabricates a config from build macros; callers supply it explicitly
// via inst_configure() / configure_system().


/*
 * Store a validated configuration for ONE instance (declaration only; no HW write).
 *
 * This is the SINGLE-mode, PRIMARY-only per-leg entry. Rejected (false) if: the port is
 * already open()'d (ERR_ALREADY_OPEN -- configure before open); the stream was committed via
 * configure_system() (mode==SYSTEM -> ERR_CONFIG_MODE; a system caller stays transactional);
 * inst is not the primary leg (ERR_CONFIG_MODE -- a non-primary leg is only reachable through
 * configure_system()); the instance is running or invalid; or cfg is outside the supported
 * wire-format envelope (NULL-safe). On success the config is stored and the mode becomes
 * SINGLE. Each leg carries its OWN clock role (the transport is rate-agnostic; a slave leg is
 * SLAVE because it was configured SLAVE). start() applies the stored config to the hardware.
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
    // Configure happens BEFORE open(): open() consumes this config to derive the clock role
    // and route pins/CLC, so configuring under an open port would desync HW from the config.
    if( s_stream.opened )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_ALREADY_OPEN );
        return false;
    }
    // Mode ownership: a SYSTEM-committed stream must be reconfigured only transactionally
    // (configure_system); and the per-leg API only ever addresses the PRIMARY leg. A
    // non-primary leg is configured exclusively through configure_system().
    if( s_config_mode == TDM_CONFIG_MODE_SYSTEM )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_CONFIG_MODE );
        return false;
    }
    if( !tdm_inst_is_primary( inst ) )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_CONFIG_MODE );
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
    // The leg's sync_domain (from the conf.h seed here; configure_system overwrites it) must fit
    // the 0..31 range start_all_domains()'s dedup/rollback mask can track -- guard the per-leg
    // path too (configure_system already checks it). See also the conf.h SYNC_DOMAIN #error.
    if( inst->sync_domain >= 32u )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_TOPOLOGY );
        return false;
    }

    inst->config       = *cfg;
    inst->config_valid = true;
    s_config_mode      = TDM_CONFIG_MODE_SINGLE;   // per-leg primary API now owns the config
    tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NONE );
    return true;
}


/*
 * Read one leg's COMMITTED setup (stored config + sync domain) into *setup.
 *
 * PURE query: returns false -- and touches neither *setup nor the last-error -- if inst is
 * NULL, setup is NULL, or the leg is not configured. Leaving last-error alone matters so a
 * board hook calling this during open() cannot clobber the real bring-up failure code.
 * Returning false for an unconfigured leg lets the caller distinguish it from a valid SLAVE
 * (role value 0) and SKIP an optional leg not part of this run (e.g. CMSIS single-instance).
 */
bool dspic33ak_spi_i2s_tdm_inst_get_setup( const dspic33ak_spi_i2s_tdm_inst_t* inst,
                                           dspic33ak_spi_i2s_tdm_leg_setup_t* setup )
{
    if( ( inst == NULL ) || ( setup == NULL ) || !inst->config_valid )
    {
        return false;
    }
    setup->stream      = inst->config;
    setup->sync_domain = inst->sync_domain;
    return true;
}


/*
 * Do two legs sharing a sync domain agree on the BCLK/FS frame interpretation?
 *
 * Co-clocked legs (same sync_domain) ride ONE shared bit/frame clock, so the fields that
 * define HOW that shared clock is read must be identical on every member -- otherwise the
 * legs would sample the same wires with different framing. Compared: format, word_bits,
 * slots_per_fs, block_frames, fs_coincides_first_bclk (SPIFE), bclk_idle_high (CKP),
 * bclk_change_on_active_to_idle (CKE), AND fs_shape. fs_shape is compared because for I2S it
 * maps to FRMSYPW regardless of clock role (hw: FS_50PCT+I2S -> FRMSYPW=1, FS_PULSE -> 0), so
 * two co-clocked I2S legs with different fs_shape would read the SAME FS with different pulse
 * widths. Deliberately NOT compared (may legitimately differ per leg): clock_role (exactly one
 * master drives the shared clock, the rest are slaves), brg (a slave ignores it), and
 * mclk_enable. IGNROV/IGNTUR are HAL-fixed policies, not per-leg config fields.
 */
static bool tdm_domain_framing_matches( const dspic33ak_spi_i2s_tdm_config_t* a,
                                        const dspic33ak_spi_i2s_tdm_config_t* b )
{
    return ( a->format                        == b->format ) &&
           ( a->word_bits                     == b->word_bits ) &&
           ( a->slots_per_fs                  == b->slots_per_fs ) &&
           ( a->block_frames                  == b->block_frames ) &&
           ( a->fs_coincides_first_bclk       == b->fs_coincides_first_bclk ) &&
           ( a->bclk_idle_high                == b->bclk_idle_high ) &&
           ( a->bclk_change_on_active_to_idle == b->bclk_change_on_active_to_idle ) &&
           ( a->fs_shape                      == b->fs_shape );
}


/*
 * Transactional whole-system configure. setups[i] targets leg index i; setup_count must
 * equal the built leg count (TDM_SPI_LEG_COUNT). See the header for the contract.
 *
 * Two passes, all-or-nothing:
 *   1. PREFLIGHT (zero side effects) -- every leg must be a valid descriptor, STOPPED, and
 *      its stream must pass tdm_config_is_supported; each sync domain may hold at most one
 *      clock MASTER; and legs sharing a sync domain must agree on the frame interpretation
 *      (tdm_domain_framing_matches). Any failure rejects the whole set before a leg is touched.
 *   2. COMMIT -- only after a clean preflight, store each leg's config + sync_domain +
 *      config_valid together. Because preflight already validated everything, commit cannot
 *      fail, so there is never a partially-configured mix (SPI1-new + SPI2-old).
 *
 * The caller owns stop->configure->start: this does NOT stop a running transport (it
 * rejects one via the STOPPED preflight), keeping the call side-effect-free on rejection.
 */
bool dspic33ak_spi_i2s_tdm_configure_system( const dspic33ak_spi_i2s_tdm_leg_setup_t* setups,
                                             uint8_t setup_count )
{
    if( ( setups == NULL ) || ( setup_count != (uint8_t)TDM_SPI_LEG_COUNT ) )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_BAD_ARGUMENT );
        return false;
    }
    // Configure happens BEFORE open() (open() consumes the committed config). A full recommit
    // is allowed from ANY mode (NONE/SINGLE/SYSTEM) as long as the port is closed and every
    // leg is stopped (the STOPPED preflight below enforces the latter).
    if( s_stream.opened )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_ALREADY_OPEN );
        return false;
    }

    // 1a. PREFLIGHT: each leg valid, stopped, and its stream supported. Zero side effects.
    for( uint8_t i = 0u; i < setup_count; i++ )
    {
        dspic33ak_spi_i2s_tdm_inst_t* leg = dspic33ak_spi_i2s_tdm_inst( i );
        if( ( leg == NULL ) || !tdm_spi_leg_is_valid( leg ) )
        {
            tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_BAD_INSTANCE );
            return false;
        }
        if( leg->running )
        {
            tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_ALREADY_RUNNING );
            return false;
        }
        if( !tdm_config_is_supported( leg, &setups[i].stream ) )
        {
            tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_UNSUPPORTED_CONFIG );
            return false;
        }
        // sync_domain must fit the 0..31 range that start_all_domains()'s 32-bit dedup/rollback
        // mask can track. A domain id >= 32 would be silently dropped from the started-mask, so
        // its legs could be started twice or skipped on rollback. Reject at configure (fail
        // closed) rather than misbehave at start. (Perseus uses 0/1; this guards public reuse.)
        if( setups[i].sync_domain >= 32u )
        {
            tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_TOPOLOGY );
            return false;
        }
    }

    // 1b. PREFLIGHT: at most one clock MASTER per sync domain (a domain has exactly one
    // clock source; two masters would fight for BCLK/FS).
    for( uint8_t i = 0u; i < setup_count; i++ )
    {
        if( setups[i].stream.clock_role != DSPIC33AK_SPI_I2S_TDM_CLOCK_MASTER )
        {
            continue;
        }
        for( uint8_t j = i + 1u; j < setup_count; j++ )
        {
            if( ( setups[j].sync_domain == setups[i].sync_domain ) &&
                ( setups[j].stream.clock_role == DSPIC33AK_SPI_I2S_TDM_CLOCK_MASTER ) )
            {
                tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_TOPOLOGY );
                return false;
            }
        }
    }

    // 1c. PREFLIGHT: legs sharing a sync domain are co-clocked on ONE BCLK/FS, so their frame
    // interpretation must be identical (a mismatch would read the shared clock differently on
    // each leg). See tdm_domain_framing_matches for the fields compared vs allowed to differ.
    for( uint8_t i = 0u; i < setup_count; i++ )
    {
        for( uint8_t j = (uint8_t)(i + 1u); j < setup_count; j++ )
        {
            if( ( setups[j].sync_domain == setups[i].sync_domain ) &&
                !tdm_domain_framing_matches( &setups[i].stream, &setups[j].stream ) )
            {
                tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_TOPOLOGY );
                return false;
            }
        }
    }

    // 2. COMMIT: preflight guarantees success, so store all legs together (config,
    // sync_domain, config_valid) -- the topology table is now the single source of both
    // the stream and the sync domain.
    for( uint8_t i = 0u; i < setup_count; i++ )
    {
        dspic33ak_spi_i2s_tdm_inst_t* leg = dspic33ak_spi_i2s_tdm_inst( i );
        leg->config       = setups[i].stream;
        leg->sync_domain  = setups[i].sync_domain;
        leg->config_valid = true;
    }
    s_config_mode = TDM_CONFIG_MODE_SYSTEM;   // whole-system domain API now owns the config
    tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NONE );
    return true;
}




/*
 * Start ONE instance: arm its RX/TX DMA, then program + enable its SPI (triggers,
 * then module ON). The shared port must already be open()'d. Returns false (instance
 * left stopped, its HW rolled back) if it is not configured, already running, or DMA
 * setup fails. Does NOT touch the port or any other instance -- the caller orders
 * multi-instance starts (co-clocked slave legs before the clock-master leg so every
 * output is armed by the time the master's clock/FS cadence begins).
 */
// ARM stage: everything up to (but NOT including) the SPI module ON. After this the DMA is
// configured/armed and the SPI is fully programmed but held OFF, so the transfer stream has
// not started. Pair with inst_go() to release. Splitting start into arm+go lets a caller ARM
// several co-clocked legs and then release them back-to-back (both SPIEN within one FS frame)
// so their ping-pong DMAs latch the SAME first FS edge = phase-locked (wdiff=0). Returns false
// (rolled back) on any failure, same as inst_start().
static bool tdm_inst_arm( dspic33ak_spi_i2s_tdm_inst_t* inst )
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
    // open() (shared clock/pins/CLC + readiness) must have run first -- arming DMA/SPI without
    // it would enter SPIEN with the port unrouted (a silent dead stream). Fail closed.
    if( !s_stream.opened )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NOT_OPEN );
        return false;
    }
    // Resolve the register-level config (a validated copy of the leg's own stored config).
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

    // FS_50PCT on a TDM master: the SPI emits a half-frame marker (set by apply_config);
    // engage CLC10 to toggle it into a 50%-duty FS on the same FS pin BEFORE the module
    // turns on, so the very first marker is captured. I2S (native 50%), FS_PULSE, and any
    // slave (FS is an input) need no CLC -- release it in case this instance held it before.
    if( ( eff_cfg.clock_role   == DSPIC33AK_SPI_I2S_TDM_CLOCK_MASTER ) &&
        ( eff_cfg.format == DSPIC33AK_SPI_I2S_TDM_FORMAT_TDM )  &&
        ( eff_cfg.fs_shape == DSPIC33AK_SPI_I2S_TDM_FS_50PCT ) )
    {
        const dspic33ak_spi_i2s_tdm_fs_clc_result_t clc =
            dspic33ak_spi_i2s_tdm_fs_clc_engage( inst->spi_inst );
        if( clc != DSPIC33AK_SPI_I2S_TDM_FS_CLC_OK )
        {
            // BUSY = CLC10 already owned by another instance/domain; NO_FS_PIN = FS not on a
            // physical pin (or no CLC10 on this part).
            tdm_set_error( ( clc == DSPIC33AK_SPI_I2S_TDM_FS_CLC_BUSY )
                               ? DSPIC33AK_SPI_I2S_TDM_ERR_CLC
                               : DSPIC33AK_SPI_I2S_TDM_ERR_PIN_CONFIG );
            goto fail;
        }
    }
    else
    {
        dspic33ak_spi_i2s_tdm_fs_clc_release( inst->spi_inst );
    }

    // ARMED: SPI module still OFF. Caller issues inst_go() to release the transfer stream.
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


// GO stage: release an armed instance's transfer stream (SPI module ON). For a framed slave
// this starts on the next FS edge; for a master it begins generating BCLK/FS. Issue the go()
// of co-clocked legs back-to-back (slaves first, an internal FS master last) so all latch the
// same FS. No-op-safe only on an armed instance; call exactly once after inst_arm().
static void tdm_inst_go( dspic33ak_spi_i2s_tdm_inst_t* inst )
{
    if( ( inst == NULL ) || !tdm_spi_leg_is_valid( inst ) )
    {
        return;
    }
    dspic33ak_spi_i2s_tdm_hw_module_enable( inst->spi_inst, true );
    inst->running = true;
}


// Convenience: arm + go one instance (single-instance start). SINGLE-mode + PRIMARY-only:
// the per-leg start path is for the one-leg (e.g. CMSIS-SAI) driver; a co-clocked group or
// any non-primary leg must start through start_domain()/start_all_domains() so the domain
// invariants (one master, matching framing, phase-locked release) are enforced.
//
// Validation order (matches start_domain()): instance validity -> config mode -> primary ->
// opened -> clock readiness -> arm -> go. opened is checked BEFORE readiness so a start before
// open() returns ERR_NOT_OPEN (not a readiness-hook verdict), and the readiness re-check (the
// open->start drop window) only runs once open() has already brought the clock source up.
bool dspic33ak_spi_i2s_tdm_inst_start( dspic33ak_spi_i2s_tdm_inst_t* inst )
{
    if( ( inst == NULL ) || !tdm_spi_leg_is_valid( inst ) )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }
    if( ( s_config_mode != TDM_CONFIG_MODE_SINGLE ) || !tdm_inst_is_primary( inst ) )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_CONFIG_MODE );
        return false;
    }
    if( !s_stream.opened )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NOT_OPEN );
        return false;
    }
    if( !tdm_stream_ready_for_start() )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_CLOCK_NOT_READY );
        return false;
    }
    if( !tdm_inst_arm( inst ) )
    {
        return false;
    }
    tdm_inst_go( inst );
    return true;
}


// Mode-agnostic teardown of every leg in one sync domain (idempotent; safe on stopped legs).
// PRIVATE: no config-mode gate, so the internal rollback in start_domain()/start_all_domains()
// and the public SYSTEM wrappers can both use it. Tears down every member via the per-leg impl.
static void tdm_stop_domain_impl( uint8_t domain )
{
    const uint8_t n = dspic33ak_spi_i2s_tdm_instance_count();
    for( uint8_t i = 0u; i < n; i++ )
    {
        dspic33ak_spi_i2s_tdm_inst_t* leg = dspic33ak_spi_i2s_tdm_inst( i );
        if( ( leg != NULL ) && ( leg->sync_domain == domain ) )
        {
            tdm_inst_stop_impl( leg );
        }
    }
}


// Public SYSTEM-mode stop of one sync domain. Rejects (false, HW unchanged) unless the stream was
// committed via configure_system() (mode==SYSTEM) -- a SINGLE-mode stream tears down through
// inst_stop(). Symmetric with start_domain(): an out-of-range (>=32) or MEMBER-LESS domain is
// ERR_BAD_INSTANCE (not a silent success); an existing-but-already-stopped domain is idempotent true.
bool dspic33ak_spi_i2s_tdm_stop_domain( uint8_t domain )
{
    if( s_config_mode != TDM_CONFIG_MODE_SYSTEM )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_CONFIG_MODE );
        return false;
    }
    if( domain >= 32u )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }
    // Reject a domain with no member leg (an unknown id), mirroring start_domain()'s members==0 check.
    uint8_t       members = 0u;
    const uint8_t n       = dspic33ak_spi_i2s_tdm_instance_count();
    for( uint8_t i = 0u; i < n; i++ )
    {
        const tdm_spi_leg_t *leg = dspic33ak_spi_i2s_tdm_inst( i );
        if( ( leg != NULL ) && ( leg->sync_domain == domain ) )
        {
            members++;
        }
    }
    if( members == 0u )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }
    tdm_stop_domain_impl( domain );
    tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NONE );
    return true;
}


// Side-effect-free classification of one sync domain, shared by start_domain() (its preflight)
// and start_all_domains() (its whole-set preflight). A sync domain is a phase-locked UNIT whose
// INVARIANTS are re-checked at START -- not only in configure_system() -- because a member may
// have been (re)configured via the per-leg path afterwards. Sets *err ONLY for INVALID.
//   INVALID     : an unconfigured member (NOT_CONFIGURED), >1 clock MASTER (TOPOLOGY), a
//                 same-domain framing disagreement (TOPOLOGY), or no member at all (BAD_INSTANCE).
//   STOPPED     : every member configured + stopped -> startable.
//   ALL_RUNNING : every member already running -> a start is an idempotent no-op.
//   PARTIAL     : some-but-not-all members running -> reject WITHOUT teardown (a re-assert must
//                 not kill live audio; a half-running domain is a foreign/inconsistent state).
typedef enum {
    TDM_DOMAIN_INVALID = 0,
    TDM_DOMAIN_STOPPED,
    TDM_DOMAIN_ALL_RUNNING,
    TDM_DOMAIN_PARTIAL,
} tdm_domain_state_t;

static tdm_domain_state_t tdm_domain_classify( uint8_t domain, dspic33ak_spi_i2s_tdm_error_t *err )
{
    const uint8_t        n       = dspic33ak_spi_i2s_tdm_instance_count();
    const tdm_spi_leg_t *ref     = NULL;
    uint8_t              members = 0u;
    uint8_t              masters = 0u;
    uint8_t              running = 0u;

    for( uint8_t i = 0u; i < n; i++ )
    {
        const tdm_spi_leg_t *leg = dspic33ak_spi_i2s_tdm_inst( i );
        if( ( leg == NULL ) || ( leg->sync_domain != domain ) )
        {
            continue;
        }
        members++;
        if( !leg->config_valid )
        {
            *err = DSPIC33AK_SPI_I2S_TDM_ERR_NOT_CONFIGURED;
            return TDM_DOMAIN_INVALID;
        }
        if( leg->config.clock_role == DSPIC33AK_SPI_I2S_TDM_CLOCK_MASTER )
        {
            masters++;
        }
        if( leg->running )
        {
            running++;
        }
        if( ref == NULL )
        {
            ref = leg;
        }
        else if( !tdm_domain_framing_matches( &ref->config, &leg->config ) )
        {
            *err = DSPIC33AK_SPI_I2S_TDM_ERR_TOPOLOGY;
            return TDM_DOMAIN_INVALID;
        }
    }
    if( members == 0u )
    {
        *err = DSPIC33AK_SPI_I2S_TDM_ERR_BAD_INSTANCE;
        return TDM_DOMAIN_INVALID;
    }
    if( masters > 1u )
    {
        *err = DSPIC33AK_SPI_I2S_TDM_ERR_TOPOLOGY;
        return TDM_DOMAIN_INVALID;
    }
    if( running == 0u )
    {
        return TDM_DOMAIN_STOPPED;
    }
    return ( running == members ) ? TDM_DOMAIN_ALL_RUNNING : TDM_DOMAIN_PARTIAL;
}


// Start every config_valid leg of one sync domain PHASE-LOCKED: (1) ARM all members, then
// (2) release SPIEN back-to-back -- non-MASTER (slave/follower) legs first, the clock-MASTER
// leg (config.clock_role==MASTER) LAST. The adjacent go() calls make the members' ping-pong DMAs
// latch the same FS edge (external-FS domains have 0 masters -> all slaves go back-to-back;
// an internal-FS domain's master starts its BCLK/FS after the slaves are armed and listening).
// SYSTEM-mode API. Non-destructive: a fully-running domain is idempotent success; a partial/
// invalid domain is rejected WITHOUT teardown. Returns false (and rolls back only THIS call's
// arms) if a leg fails to arm. open() (shared clock/pins) must have run first; this does not.
bool dspic33ak_spi_i2s_tdm_start_domain( uint8_t domain )
{
    const uint8_t                 n = dspic33ak_spi_i2s_tdm_instance_count();
    dspic33ak_spi_i2s_tdm_error_t err;

    // SYSTEM-mode ownership: domain start is only for a configure_system()-committed stream.
    if( s_config_mode != TDM_CONFIG_MODE_SYSTEM )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_CONFIG_MODE );
        return false;
    }
    // open() (shared clock/pins/CLC + readiness) MUST have run first.
    if( !s_stream.opened )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NOT_OPEN );
        return false;
    }

    // PREFLIGHT (zero side effects) via the shared classifier.
    const tdm_domain_state_t state = tdm_domain_classify( domain, &err );
    if( state == TDM_DOMAIN_INVALID )
    {
        tdm_set_error( err );
        return false;   // no side effects (do NOT stop_domain on a preflight reject)
    }
    if( state == TDM_DOMAIN_ALL_RUNNING )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NONE );
        return true;    // already fully up -> idempotent no-op success
    }
    if( state == TDM_DOMAIN_PARTIAL )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_ALREADY_RUNNING );
        return false;   // partial running -> reject, leave the domain as-is
    }

    // STOPPED and about to arm: re-check the clock-readiness gate (the open->start drop window).
    if( !tdm_stream_ready_for_start() )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_CLOCK_NOT_READY );
        return false;
    }

    // (1) ARM every member. Preflight guaranteed all are stopped + configured, so a failure
    //     here only rolls back what THIS call armed (stop_domain is idempotent on stopped legs).
    for( uint8_t i = 0u; i < n; i++ )
    {
        dspic33ak_spi_i2s_tdm_inst_t* leg = dspic33ak_spi_i2s_tdm_inst( i );
        if( ( leg == NULL ) || ( leg->sync_domain != domain ) )
        {
            continue;
        }
        if( !tdm_inst_arm( leg ) )
        {
            tdm_stop_domain_impl( domain );   // roll back only this call's arms (mode-agnostic)
            return false;
        }
    }

    // (2a) GO the non-master legs first (adjacent SPIEN releases).
    for( uint8_t i = 0u; i < n; i++ )
    {
        dspic33ak_spi_i2s_tdm_inst_t* leg = dspic33ak_spi_i2s_tdm_inst( i );
        if( ( leg == NULL ) || ( leg->sync_domain != domain ) || !leg->config_valid )
        {
            continue;
        }
        if( leg->config.clock_role != DSPIC33AK_SPI_I2S_TDM_CLOCK_MASTER )
        {
            tdm_inst_go( leg );
        }
    }
    // (2b) then the clock-MASTER leg(s) LAST -- its BCLK/FS starts after the slaves listen.
    for( uint8_t i = 0u; i < n; i++ )
    {
        dspic33ak_spi_i2s_tdm_inst_t* leg = dspic33ak_spi_i2s_tdm_inst( i );
        if( ( leg == NULL ) || ( leg->sync_domain != domain ) || !leg->config_valid )
        {
            continue;
        }
        if( leg->config.clock_role == DSPIC33AK_SPI_I2S_TDM_CLOCK_MASTER )
        {
            tdm_inst_go( leg );
        }
    }
    tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NONE );
    return true;
}


// Start every sync domain present in the leg table (each once). Start/rollback bookkeeping is
// per-domain and there is no cross-domain start-ordering constraint here (NOTE: this is NOT full
// independence -- source-readiness is engine-wide / primary-leg-gated and shared resources such as
// CLC10 and the board clock port are not per-domain; see tdm_stream_ready_for_start()). SYSTEM-mode
// API. TWO PASSES so a later domain's failure can never tear
// down a domain that was ALREADY running before this call (nor one this call did not touch):
//   Pass 1 (side-effect-free): classify every DISTINCT domain. If ANY is PARTIAL or INVALID,
//           reject the whole call touching NOTHING. Record which domains are STOPPED (startable).
//           ALL_RUNNING domains are left running and are NOT recorded (never rolled back).
//   Pass 2: start only the STOPPED domains, tracking newly_started_mask = domains THIS call
//           actually started. On any failure, roll back ONLY newly_started_mask -- pre-existing
//           running domains and untouched domains are preserved.
// Returns false + ERR_NOT_CONFIGURED if no domain is configured. open() must run first.
bool dspic33ak_spi_i2s_tdm_start_all_domains( void )
{
    const uint8_t                 n = dspic33ak_spi_i2s_tdm_instance_count();
    uint32_t                      seen_mask    = 0u;   // distinct domains examined
    uint32_t                      stopped_mask = 0u;   // startable (all-stopped) domains
    dspic33ak_spi_i2s_tdm_error_t err;

    // SYSTEM-mode ownership + open() precondition.
    if( s_config_mode != TDM_CONFIG_MODE_SYSTEM )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_CONFIG_MODE );
        return false;
    }
    if( !s_stream.opened )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NOT_OPEN );
        return false;
    }

    // PASS 1: classify every distinct configured domain, side-effect-free. Any PARTIAL/INVALID
    // domain rejects the whole call before a single leg is touched.
    for( uint8_t i = 0u; i < n; i++ )
    {
        const tdm_spi_leg_t* leg = dspic33ak_spi_i2s_tdm_inst( i );
        if( ( leg == NULL ) || !leg->config_valid )
        {
            continue;
        }
        const uint8_t dom = leg->sync_domain;
        if( ( dom >= 32u ) || ( ( seen_mask & ( 1uL << dom ) ) != 0u ) )
        {
            continue;   // >=32 is rejected at configure; skip duplicates
        }
        seen_mask |= ( 1uL << dom );

        const tdm_domain_state_t state = tdm_domain_classify( dom, &err );
        if( state == TDM_DOMAIN_INVALID )
        {
            tdm_set_error( err );
            return false;   // touch nothing
        }
        if( state == TDM_DOMAIN_PARTIAL )
        {
            tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_ALREADY_RUNNING );
            return false;   // touch nothing (do NOT tear a half-running domain down)
        }
        if( state == TDM_DOMAIN_STOPPED )
        {
            stopped_mask |= ( 1uL << dom );
        }
        // ALL_RUNNING: leave it running, not recorded -> never rolled back.
    }
    if( seen_mask == 0u )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NOT_CONFIGURED );
        return false;   // no configured domain to start
    }

    // PASS 2: start only the STOPPED domains; roll back ONLY what this call started.
    uint32_t newly_started_mask = 0u;
    for( uint8_t dom = 0u; dom < 32u; dom++ )
    {
        if( ( stopped_mask & ( 1uL << dom ) ) == 0u )
        {
            continue;
        }
        if( !dspic33ak_spi_i2s_tdm_start_domain( dom ) )
        {
            for( uint8_t d = 0u; d < 32u; d++ )
            {
                if( ( newly_started_mask & ( 1uL << d ) ) != 0u )
                {
                    tdm_stop_domain_impl( d );   // mode-agnostic; roll back only newly-started
                }
            }
            return false;   // pre-existing running domains untouched
        }
        newly_started_mask |= ( 1uL << dom );
    }
    tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NONE );
    return true;
}


// Mode-agnostic teardown of every instance (all sync domains). PRIVATE counterpart used by the
// public SYSTEM wrapper below (and available for internal use). Covers future SPI3/SPI4 in any
// domain automatically. Idempotent; safe on already-stopped legs.
static void tdm_stop_all_domains_impl( void )
{
    const uint8_t n = dspic33ak_spi_i2s_tdm_instance_count();
    for( uint8_t i = 0u; i < n; i++ )
    {
        dspic33ak_spi_i2s_tdm_inst_t* leg = dspic33ak_spi_i2s_tdm_inst( i );
        if( leg != NULL )
        {
            tdm_inst_stop_impl( leg );
        }
    }
}


// Public SYSTEM-mode stop of every sync domain. Domain-level teardown counterpart to
// start_all_domains() so callers never enumerate individual logical legs. Rejects
// (false, HW unchanged) unless the stream was committed via configure_system() (mode==SYSTEM);
// a SINGLE-mode stream tears down through inst_stop(). Idempotent success otherwise.
bool dspic33ak_spi_i2s_tdm_stop_all_domains( void )
{
    if( s_config_mode != TDM_CONFIG_MODE_SYSTEM )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_CONFIG_MODE );
        return false;
    }
    tdm_stop_all_domains_impl();
    tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NONE );
    return true;
}


// (Startup phase-lock verification is done per-block in the app's SPI1 block callback -- it must
// observe multiple conditions across consecutive blocks, incl. tail_tear which needs the callback's
// begin/end sampling. The HAL exposes the primitives inst_tx_active_half()/inst_tx_active_pos() and
// the mirror's NULL-when-unsafe; a HAL position-only "locked?" snapshot was intentionally NOT added
// because wdiff==0 alone is necessary-but-not-sufficient, as HW showed, wdiff=0 with tail_tear=samp.)



/*
 * Stop ONE instance and make its next start deterministic (MODE-AGNOSTIC implementation).
 *
 * SoftStop policy (per instance): does NOT stop DMACONbits.ON (shared controller) or
 * change PPS/CLC routing; stops only this instance's SPI module + DMA channels, masks
 * its DMA IRQs first so its ISR cannot refill mid-teardown, clears its pending
 * status, and clears its buffers so a restart is silent. Safe to call when stopped.
 *
 * This static core carries no mode/primary gate so the SYSTEM domain teardown
 * (stop_domain/stop_all_domains) can tear down every member leg. The public inst_stop()
 * below wraps it with the SINGLE-mode + primary-only gate.
 */
static void tdm_inst_stop_impl( tdm_spi_leg_t *inst )
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

    // Release the CLC10 50%-FS generator if this instance owned it (disables the flip-flop;
    // PPS routes are left in place, like the rest of stop()). No-op if it didn't.
    dspic33ak_spi_i2s_tdm_fs_clc_release( inst->spi_inst );

    // Clear pending status/flags, then buffers, before the next start.
    tdm_inst_clear_dma_flags( inst );
    dspic33ak_spi_i2s_tdm_hw_irq_clear_flags( inst->spi_inst );
    tdm_inst_clear_buffers( inst );
}


/*
 * Public per-instance stop: SINGLE-mode + PRIMARY-only, symmetric with inst_start().
 *
 * Validation order matches inst_start(): a NULL/invalid handle is ERR_BAD_INSTANCE FIRST, then
 * the mode/primary gate. Rejected (false) when the stream was committed via configure_system()
 * (mode==SYSTEM -> tear down through stop_domain()/stop_all_domains() instead) or inst is not the
 * primary leg. Returns true after the teardown (or if the primary was already stopped -- idempotent).
 */
bool dspic33ak_spi_i2s_tdm_inst_stop( dspic33ak_spi_i2s_tdm_inst_t* inst )
{
    if( ( inst == NULL ) || !tdm_spi_leg_is_valid( inst ) )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_BAD_INSTANCE );
        return false;
    }
    if( ( s_config_mode != TDM_CONFIG_MODE_SINGLE ) || !tdm_inst_is_primary( inst ) )
    {
        tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_CONFIG_MODE );
        return false;
    }
    tdm_inst_stop_impl( inst );
    tdm_set_error( DSPIC33AK_SPI_I2S_TDM_ERR_NONE );
    return true;
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
    status->err_rov_block_count       = inst->diag.err_rov_block_count;   // masked read
    status->err_tur_block_count       = inst->diag.err_tur_block_count;
    status->err_frm_block_count       = inst->diag.err_frm_block_count;
    status->frmerr_consecutive_blocks = inst->diag.frmerr_consecutive_blocks;
    status->rx_dma_overrun_count      = inst->diag.rx_dma_overrun_count;
    status->rx_dma_other_irq_count    = inst->diag.rx_dma_other_irq_count;
    status->rx_dma_last_status        = inst->diag.rx_dma_last_status;
    tdm_rx_ie_restore( inst->rx_dma_ch, rxie_bak );

    // load monitor (does its own RX-IE guard; honours clear_peak)
    (void)dspic33ak_spi_i2s_tdm_inst_get_load( inst, &status->load, clear_peak );

    return true;
}


/*
 * Singleton load/status readers: report the PRIMARY leg (primary_leg_index, default logical leg 0).
 * Thin wrappers over the per-instance readers; behaviour is unchanged from before the
 * per-instance API was added. They go through s_stream.legs (the stream is the single
 * source for the leg table, same as is_running()/is_active()) rather than s_spi_legs
 * directly, and use a mutable leg because the per-instance readers honour clear_peak. Guard
 * the index (fail closed -> false) so an out-of-range primary never dereferences the table.
 */
bool dspic33ak_spi_i2s_tdm_get_load( dspic33ak_spi_i2s_tdm_load_t* monitor, bool clear_peak )
{
    if( s_stream.primary_leg_index >= s_stream.leg_count )
    {
        return false;
    }
    return dspic33ak_spi_i2s_tdm_inst_get_load( &s_stream.legs[s_stream.primary_leg_index], monitor, clear_peak );
}

bool dspic33ak_spi_i2s_tdm_get_status( dspic33ak_spi_i2s_tdm_status_t* status, bool clear_peak )
{
    if( s_stream.primary_leg_index >= s_stream.leg_count )
    {
        return false;
    }
    return dspic33ak_spi_i2s_tdm_inst_get_status( &s_stream.legs[s_stream.primary_leg_index], status, clear_peak );
}


//===========================================================
// DMA INTERRUPT VECTORS  --  IMPORTANT: these ARE the IVT entries the CPU jumps to.
//
// The HAL ships its own DMA interrupt vectors so the transport is turnkey: link the HAL
// and the _DMAnInterrupt slots are filled, the DMA channels are already armed by
// start(), and the integrator only registers a per-instance block callback. (Previously
// these lived in a separate optional TU dspic33ak_spi_i2s_tdm_irq.c + a forwarding
// worker; folded back here -- no cross-TU hop -- since the toolchain/IVT coupling is
// confined to this one section.)
//
// One explicit RX vector per leg. Each RX vector runs that leg's block ISR (tdm_rx_block
// with THIS leg + its RX/TX channels + its slots*blk half size, ALL compile-time
// constants so the DMA register access / pointer math fold, with NO runtime channel->leg
// lookup). The TX channel is INTENTIONALLY interrupt-less (hw.c enables the CPU IRQ on the
// RX channel only): TX is fire-and-forget ping-pong with auto-reload, so RX completion
// alone defines the block boundary -- there is no TX ISR.
//
// The vector NAME (_DMA<n>Interrupt) encodes the DMA channel, so each vector is bound to
// its conf.h RX-DMA channel by a compile-time assert: at the default mapping this defines
//     _DMA0Interrupt   (leg SPI1 RX, DMA0)
//     _DMA2Interrupt   (leg SPI2 RX, DMA2, when DSPIC33AK_TDM_USE_SPI2)
// Change an RX-DMA channel macro in conf.h and the build FAILS (assert) until the vector
// name + its assert are updated to match. IVT slots are chip-wide exclusive; a genuine
// clash surfaces as a duplicate-_DMAnInterrupt link error.
//===========================================================

// One RX interrupt vector per leg. tdm_rx_block is static inline (same TU) so
// the constant RX/TX channels + half size fold into the register access. (tx) is still
// passed -- tdm_rx_block reads the TX channel's DMA address to pick the writable TX half
// -- but the TX channel itself raises no interrupt (interrupt-less; see above).
//
// Compiled only when the HAL owns the IVT (DSPIC33AK_TDM_DEFINE_DMA_VECTORS=1, default).
// With =0 the integrator owns the vectors and calls dspic33ak_spi_i2s_tdm_inst_rx_isr()
// (below) from their own _DMA<rx>Interrupt instead.
#if DSPIC33AK_TDM_DEFINE_DMA_VECTORS
// Explicit HAL-owned RX vectors (was X-macro-generated). tdm_rx_block is same-TU static inline, so
// the literal channel numbers + half size fold into the register access -- NO runtime channel->leg
// lookup. The vector NAME encodes the DMA channel number, so bind each name to its conf.h channel
// with a compile-time assert: change the RX-DMA channel macro and the build FAILS until the vector
// name (and its assert) is updated to match. (tx) is passed so tdm_rx_block can pick the writable TX
// half; the TX channel raises no interrupt.
#if DSPIC33AK_TDM_BASE_ON_SPI34
TDM_COMPILEASSERT( DSPIC33AK_TDM_SPI3_RX_DMA == 4 );   /* logical A -> _DMA4Interrupt */
void __attribute__((interrupt, context)) _DMA4Interrupt(void)
{
    tdm_rx_block( &s_spi_legs[TDM_SPI_LEG_SPI1],
                  DSPIC33AK_TDM_SPI3_RX_DMA, DSPIC33AK_TDM_SPI3_TX_DMA,
                  TDM_LEG_HALF_WORDS(DSPIC33AK_TDM_SLOTS_PER_FS, DSPIC33AK_TDM_BLOCK_FRAMES) );
}
#else
TDM_COMPILEASSERT( DSPIC33AK_TDM_SPI1_RX_DMA == 0 );   /* _DMA0Interrupt binding */
void __attribute__((interrupt, context)) _DMA0Interrupt(void)
{
    tdm_rx_block( &s_spi_legs[TDM_SPI_LEG_SPI1],
                  DSPIC33AK_TDM_SPI1_RX_DMA, DSPIC33AK_TDM_SPI1_TX_DMA,
                  TDM_LEG_HALF_WORDS(DSPIC33AK_TDM_SLOTS_PER_FS, DSPIC33AK_TDM_BLOCK_FRAMES) );
}
#endif
#if DSPIC33AK_TDM_USE_SPI2
#if DSPIC33AK_TDM_BASE_ON_SPI34
TDM_COMPILEASSERT( DSPIC33AK_TDM_SPI4_RX_DMA == 6 );   /* logical B -> _DMA6Interrupt */
void __attribute__((interrupt, context)) _DMA6Interrupt(void)
{
    tdm_rx_block( &s_spi_legs[TDM_SPI_LEG_SPI2],
                  DSPIC33AK_TDM_SPI4_RX_DMA, DSPIC33AK_TDM_SPI4_TX_DMA,
                  TDM_LEG_HALF_WORDS(DSPIC33AK_TDM_SLOTS_PER_FS, DSPIC33AK_TDM_BLOCK_FRAMES) );
}
#else
TDM_COMPILEASSERT( DSPIC33AK_TDM_SPI2_RX_DMA == 2 );   /* _DMA2Interrupt binding */
void __attribute__((interrupt, context)) _DMA2Interrupt(void)
{
    tdm_rx_block( &s_spi_legs[TDM_SPI_LEG_SPI2],
                  DSPIC33AK_TDM_SPI2_RX_DMA, DSPIC33AK_TDM_SPI2_TX_DMA,
                  TDM_LEG_HALF_WORDS(DSPIC33AK_TDM_SLOTS_PER_FS, DSPIC33AK_TDM_BLOCK_FRAMES) );
}
#endif
#endif // DSPIC33AK_TDM_USE_SPI2
#if DSPIC33AK_TDM_USE_SPI3 && !DSPIC33AK_TDM_BASE_ON_SPI34
TDM_COMPILEASSERT( DSPIC33AK_TDM_SPI3_RX_DMA == 4 );   /* _DMA4Interrupt binding */
void __attribute__((interrupt, context)) _DMA4Interrupt(void)
{
    tdm_rx_block( &s_spi_legs[TDM_SPI_LEG_SPI3],
                  DSPIC33AK_TDM_SPI3_RX_DMA, DSPIC33AK_TDM_SPI3_TX_DMA,
                  TDM_LEG_HALF_WORDS(DSPIC33AK_TDM_SLOTS_PER_FS, DSPIC33AK_TDM_BLOCK_FRAMES) );
}
#endif // DSPIC33AK_TDM_USE_SPI3
#if DSPIC33AK_TDM_USE_SPI4 && !DSPIC33AK_TDM_BASE_ON_SPI34
TDM_COMPILEASSERT( DSPIC33AK_TDM_SPI4_RX_DMA == 6 );   /* _DMA6Interrupt binding */
void __attribute__((interrupt, context)) _DMA6Interrupt(void)
{
    tdm_rx_block( &s_spi_legs[TDM_SPI_LEG_SPI4],
                  DSPIC33AK_TDM_SPI4_RX_DMA, DSPIC33AK_TDM_SPI4_TX_DMA,
                  TDM_LEG_HALF_WORDS(DSPIC33AK_TDM_SLOTS_PER_FS, DSPIC33AK_TDM_BLOCK_FRAMES) );
}
#endif // DSPIC33AK_TDM_USE_SPI4
#endif // DSPIC33AK_TDM_DEFINE_DMA_VECTORS

/*
 * Public RX-block ISR entry for ONE instance (vector-ownership opt-out path).
 *
 * Runs the same block work as the HAL's own explicit _DMA<rx>Interrupt vector, but is a plain
 * (non-interrupt) function the integrator calls from their OWN _DMA<rx>Interrupt when
 * DSPIC33AK_TDM_DEFINE_DMA_VECTORS=0. Channels + half size come from the leg at runtime
 * (the explicit turnkey vectors fold them as constants; this dispatch trades that for
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
 * This covers transport buffers only. Application processing buffers are deliberately
 * outside the HAL's ownership boundary.
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
 * valid, the primary_leg_index is in range (it selects the leg the singleton
 * is_running()/get_status()/get_load()/is_active() API reports), and all legs use
 * distinct physical SPIs and distinct DMA channels (cross-leg loop below). The physical
 * SPI and the RX/TX DMA channels per leg are set on the leg descriptors; NOTHING here is
 * pinned to a specific SPI number or channel, so adding a leg or remapping a leg's channels
 * needs no edit to this check.
 *
 * primary_leg_index is only a reporting default -- NOT a clock-role constraint (clock role
 * is per-leg) and NOT a timing coupling (every leg times itself via its own RX-block ISR).
 */
static bool tdm_stream_topology_is_valid( const tdm_stream_t *stream )
{
    if( ( stream == NULL ) ||
        ( stream->legs == NULL ) ||
        ( stream->leg_count == 0u ) ||
        ( stream->leg_count > (uint8_t)TDM_SPI_INST_COUNT ) ||
        ( stream->primary_leg_index >= stream->leg_count ) )
    {
        return false;
    }

    for( uint8_t i = 0u; i < stream->leg_count; i++ )
    {
        if( !tdm_spi_leg_is_valid( &stream->legs[i] ) )
        {
            return false;
        }
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
 * Return the stream's PRIMARY leg -- the one the arg-less singleton status API
 * (is_running/is_active/get_status/get_load) reports.
 *
 * It is simply legs[primary_leg_index]; returns NULL if the stream/table is absent or the
 * index is out of range (fail closed). Not a clock master and not a timing coupling --
 * every leg times itself via its own RX-block ISR.
 */
static const tdm_spi_leg_t *tdm_stream_primary_leg( const tdm_stream_t *stream )
{
    if( ( stream == NULL ) ||
        ( stream->legs == NULL ) ||
        ( stream->primary_leg_index >= stream->leg_count ) )
    {
        return NULL;
    }
    return &stream->legs[stream->primary_leg_index];
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
 * (its Rx_<name>/Tx_<name> are sized for exactly that, per the leg table); an
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
    if( ( cfg->clock_role != DSPIC33AK_SPI_I2S_TDM_CLOCK_SLAVE ) &&
        ( cfg->clock_role != DSPIC33AK_SPI_I2S_TDM_CLOCK_MASTER ) )
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

    // fs_shape must be a known value -- otherwise a garbage value would be silently
    // treated as FS_PULSE by hw_apply_config (shape == FS_50PCT ? ... : ...).
    if( ( cfg->fs_shape != DSPIC33AK_SPI_I2S_TDM_FS_PULSE ) &&
        ( cfg->fs_shape != DSPIC33AK_SPI_I2S_TDM_FS_50PCT ) )
    {
        return false;
    }
    // No role/format restriction on fs_shape. A TDM SLAVE receives FS as an INPUT, so
    // fs_shape is accepted but has no generated-waveform effect (hw_apply_config treats a
    // slave as normal framing). TDM FS_50PCT is generated by CLC10 only in MASTER mode.

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
 * (src, dst, user). Called from the explicit per-instance RX vector (the
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

    // Sample + fold this instance's SPIxSTAT framed-transport health flags (SPIROV/SPITUR/
    // FRMERR) into its diag. Cheap (1 SFR read + masked ack); normally zero. The HAL only
    // records these counters -- recovery policy, if any, belongs to the application.
    dspic33ak_spi_i2s_tdm_diag_note_errflags( &inst->diag,
        dspic33ak_spi_i2s_tdm_hw_sample_ack_errflags( inst->spi_inst ) );

    dma_stat = dspic33ak_dma_isr_snapshot( rx_ch );

    // Preserve DMAxSTAT before HALF/DONE resolution. In particular, an OVERRUN-only
    // snapshot has no completed half and will return below; its root-cause evidence must
    // survive that early exit in the public diagnostics.
    dspic33ak_spi_i2s_tdm_diag_note_dma_status( &inst->diag, dma_stat );

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


// The application audio/DSP path lives above the HAL. The HAL core does NOT call any
// application function: each instance's RX-block handler delivers its block to the registered
// block callback only (no app fallback). The HAL core owns its DMA ping-pong buffers; the
// application owns its own processing buffers.
