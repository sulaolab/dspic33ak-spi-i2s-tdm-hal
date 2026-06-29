#ifndef DSPIC33AK_SPI_I2S_TDM_H
#define DSPIC33AK_SPI_I2S_TDM_H

//===========================================================
// Device identity (HAL-owned adapter). Maps the toolchain's -mcpu predefined macro to an
// opaque, module-prefixed tag in ONE place; the silicon layer (hw.{c,h}) selects the SPI
// device-facts table / instance count on it. Self-contained: depends only on the compiler
// macro, never on app config. Add a sibling part by OR-ing it into one arm here -- the
// single part-number change point. Tag values arbitrary; compare with == only (never
// order / arithmetic). This HAL hard-#errors on an unsupported device.
//===========================================================
#define DSPIC33AK_SPI_I2S_TDM_DEV_AK512   (1)
#define DSPIC33AK_SPI_I2S_TDM_DEV_AK128   (2)

#if   defined(__dsPIC33AK512MPS512__)
  #define DSPIC33AK_SPI_I2S_TDM_DEVICE    DSPIC33AK_SPI_I2S_TDM_DEV_AK512
#elif defined(__dsPIC33AK128MC106__)
  #define DSPIC33AK_SPI_I2S_TDM_DEVICE    DSPIC33AK_SPI_I2S_TDM_DEV_AK128
#else
  #error "Unsupported device -- the SPI/I2S/TDM HAL expects __dsPIC33AK512MPS512__ or __dsPIC33AK128MC106__."
#endif

//===========================================================
// dspic33ak_spi_i2s_tdm.{c,h} + dspic33ak_spi_i2s_tdm_reg.h provide a reusable
// dsPIC33AK SPI framed-mode I2S/TDM transport HAL. It owns SPI framed-mode setup,
// RX/TX DMA ping-pong buffers, per-instance block callbacks, lifecycle, status,
// deadline-miss, and load diagnostics.
//
// It does not own DSP, codec setup, PPS/CLC routing, sample-rate policy, app
// config, or CMSIS-SAI buffer semantics. Board pin/CLC and external-clock concerns
// are reached only through the registered board/clock port hook (set_port()).
//
// Compile-time stream geometry and topology come from a project-supplied
// dspic33ak_spi_i2s_tdm_conf.h. The HAL core only depends on DSPIC33AK_TDM_*
// macros from that header -- it does not read app symbols directly. Instance
// count, physical-SPI/DMA assignment, and per-instance geometry are driven from
// DSPIC33AK_TDM_INSTANCE_LIST in conf.h (leg enum, buffers, leg table, and DMA
// vectors are generated from it).
//
// Supported-device limitation: the silicon-facts paths currently cover
// __dsPIC33AK512MPS512__ and __dsPIC33AK128MC106__ only; other parts need their
// facts added in dspic33ak_spi_i2s_tdm_hw.{c,h}. Sibling dependencies are
// dspic33ak_dma (required) and dspic33ak_high_res_timer (compile/link dependency
// for the load monitor, runtime-gated via is_initialized()).
//===========================================================

//===========================================================
// INCLUDES
//===========================================================
#include <stdint.h>
#include <stdbool.h>
#include "dspic33ak_spi_i2s_tdm_conf.h"   // HAL compile-time config (geometry/topology); exposes DSPIC33AK_TDM_* to consumers


//===========================================================
// Definition
//===========================================================


//===========================================================
// Enum & Struct typedef
//===========================================================

// Block-ISR load/time monitor: execution-time stats of one instance's RX-block ISR,
// in raw timer counts and 0.1us units. (Named after the load it measures, not a fixed
// DMA channel -- each instance's RX DMA channel is configurable in conf.h.)
typedef struct
{
    uint32_t last_count;
    uint32_t min_count;
    uint32_t max_count;
    uint32_t event_count;

    // us10 means 0.1 us unit.
    // Example: 1234 means 123.4 us.
    uint32_t last_us10;
    uint32_t min_us10;
    uint32_t max_us10;
} dspic33ak_spi_i2s_tdm_load_t;



//===========================================================
// Runtime SPI/I2S/TDM stream configuration.
//
// configure() validates and stores this configuration while stopped. start()
// applies the stored configuration to the SPI peripheral and DMA engine after the
// platform pin/clock port hooks have completed successfully. Seed it from the
// platform helper (audio_app_board_get_default_config) or build it directly. Field
// comments map to the SPIxCON1 bit each one drives.
//===========================================================

// Frame format. Selects FRMCNT + the conventional FRMPOL used today.
//   I2S : FRMCNT=001 (FS every 2 words), FRMPOL active-low   (was ENA_FRMT_I2S)
//   TDM : FRMCNT=011 (FS every 8 words, TDM8), FRMPOL active-high
typedef enum {
    DSPIC33AK_SPI_I2S_TDM_FORMAT_I2S = 0,
    DSPIC33AK_SPI_I2S_TDM_FORMAT_TDM = 1,
} dspic33ak_spi_i2s_tdm_format_t;

// Bit-clock / frame-sync role.
//   SLAVE  : MSTEN=0, FRMSYNC=1 (FS input)   -- external BCLK/FS (Perseus today)
//   MASTER : MSTEN=1, FRMSYNC=0 (FS output)  -- self-clocked (starter target)
typedef enum {
    DSPIC33AK_SPI_I2S_TDM_ROLE_SLAVE  = 0,
    DSPIC33AK_SPI_I2S_TDM_ROLE_MASTER = 1,
} dspic33ak_spi_i2s_tdm_role_t;

typedef struct {
    dspic33ak_spi_i2s_tdm_format_t format;          // I2S vs TDM (FRMCNT/FRMPOL)
    dspic33ak_spi_i2s_tdm_role_t   role;            // master vs slave (MSTEN/FRMSYNC)
    uint8_t  slots_per_fs;                          // DSPIC33AK_TDM_SLOTS_PER_FS: I2S=2 / TDM8=8
    uint8_t  word_bits;                             // 32 (MODE32); only 32 validated
    uint16_t block_frames;                          // DSPIC33AK_TDM_BLOCK_FRAMES: frames per ping/pong half
    uint32_t brg;                                   // SPIxBRG (master only; ignored as slave)
    bool     mclk_enable;                           // MCLKEN (CLKGEN9 reference)
    bool     fs_one_word_wide;                      // FRMSYPW
    bool     fs_coincides_first_bclk;               // SPIFE: 1=no delay, 0=1-bit delayed (ENA_1_BIT_DELAY)
    bool     bclk_idle_high;                        // CKP
    bool     bclk_change_on_active_to_idle;         // CKE
    bool     ignore_overflow;                       // IGNROV
    bool     ignore_underrun;                       // IGNTUR
} dspic33ak_spi_i2s_tdm_config_t;

// Clock-change event reported by the slave's external-clock detector
// (board RB15 / CN). STOPPED = the external bit/frame clock has stopped (e.g. the
// source is switching sample rate) -> the app should mute + stop. RESUMED = the
// clock is back -> the app should measure the rate, reconfigure, then restart.
// NONE = nothing pending. Consumed (read-and-clear) via
// dspic33ak_spi_i2s_tdm_consume_clock_event(); always NONE when the board has no
// external-clock detect (e.g. ENA_USB_AUDIO_IN undefined).
typedef enum {
    DSPIC33AK_SPI_I2S_TDM_CLOCK_EVENT_NONE = 0,
    DSPIC33AK_SPI_I2S_TDM_CLOCK_EVENT_STOPPED, // external clock stopped (rate switch begun)
    DSPIC33AK_SPI_I2S_TDM_CLOCK_EVENT_RESUMED, // external clock back
} dspic33ak_spi_i2s_tdm_clock_event_t;

// One-completed-block callback (event hook), registered PER SPI instance. The
// instance's RX-block ISR calls this for each completed block: src = the RX ping/pong
// half just captured by THIS instance; dst = the TX ping/pong half of THIS instance
// to fill; user = opaque context. One callback handles exactly one physical SPI's
// RX/TX block -- there is no dst_b / "second output": when two SPI instances are
// running, each has its own callback, and any cross-instance routing (e.g. mirroring
// one input to both outputs) is the application's job, done explicitly through a
// shared buffer between the two callbacks.
// Contract: register it (set_block_callback) BEFORE start(); do NOT clear it while
// running. If no callback is registered for an instance, that instance runs no
// app/DSP path (its zeroed TX half stays silent).
// Contract: when this callback is invoked, src and dst are both non-NULL. If the core
// cannot resolve either half-buffer (reload boundary / just-stopped / first block /
// fault), it skips the block instead of calling the callback -- the callee never
// NULL-checks src/dst.
typedef void (*dspic33ak_spi_i2s_tdm_block_cb_t)( const int32_t* src,
                                                  int32_t*       dst,
                                                  void*          user );

// Opaque per-physical-SPI instance handle. The engine exposes the two co-clocked SPI
// legs (SPI1 + optional SPI2) through the accessors below; pass the handle to
// inst_configure()/inst_start()/inst_stop()/set_block_callback()/inst_get_status() to
// drive or query that one instance. The shared board/clock port is brought up once via
// open()/close(); the app owns the multi-instance ordering.
typedef struct dspic33ak_spi_i2s_tdm_inst_s dspic33ak_spi_i2s_tdm_inst_t;

// Stream status snapshot. block_count is the number of completed audio
// blocks delivered since the last start(); one block = block_frames (DSPIC33AK_TDM_BLOCK_FRAMES)
// frames per direction. load is the block-ISR load monitor (same data as
// get_load()). block_deadline_miss_count is the number of
// times the RX-block ISR fell a full block behind (HALF+DONE conflict) since start()
// -- the real-time/stream-health metric for this zero-copy engine. It is DISTINCT
// from SPI HW FIFO over/underrun flags.
// `running` is the true stream-running state -- set by start(), cleared by stop().
// It is DISTINCT from `active`: `active`
// (is_active()) is the clock/source-readiness gate (e.g. external USB audio clock
// present) that the Perseus main loop uses to decide whether streaming *should*
// run, and it can read true while the stream is stopped. Read `running` for "is
// the engine actually streaming", `active` for "is the clock source ready".
typedef struct {
    bool                        active;       // is_active(): clock/source readiness (NOT running)
    bool                        running;      // is_running(): stream actually started (start..stop)
    uint32_t                    block_count;  // completed blocks since start()
    uint32_t                     block_deadline_miss_count; // HALF+DONE conflicts since start()
    dspic33ak_spi_i2s_tdm_load_t load;         // block-ISR load/time monitor
} dspic33ak_spi_i2s_tdm_status_t;


//===========================================================
// Board/clock PORT (optional hooks). The HAL core is board-free: instead of
// calling the board adapter directly, it routes pin routing + external-clock
// concerns through this fn-pointer table, which the platform layer
// (audio_app) registers via set_port(). Every field is optional; the
// fallible hooks return bool (false => start() aborts and returns false) and take
// the resolved role so the platform can act differently for master vs slave:
//   - configure_pins(role)  : PPS/GPIO routing for the role. false => unsupported
//                             pin config (e.g. a role this board cannot drive) =>
//                             start() fails. NULL => core does no pin routing.
//   - clc_passthrough(role) : CLC bypass route (slave clock fan-out). false =>
//                             start() fails. NULL => skipped.
//   - clock_source_init(role): bring up an external (e.g. USB-audio) clock. false
//                             => start() fails. NULL => no external clock to bring up.
//   - clock_source_ready(role): external-clock readiness; drives is_active() and a
//                             SINGLE non-blocking check in start() (start() does NOT
//                             wait -- it returns false if not ready, leaving retry to
//                             the platform/app). NULL => always ready (no clock gate).
//   - consume_clock_event() : read-and-clear the ext-clock stop/resume edge; NULL
//                             => always NONE.
// With NO port registered the core behaves as a self-clocked transport with no
// readiness gate (is_active()==true, no events).
//===========================================================
typedef struct {
    bool (*configure_pins)( dspic33ak_spi_i2s_tdm_role_t role );
    bool (*clc_passthrough)( dspic33ak_spi_i2s_tdm_role_t role );
    bool (*clock_source_init)( dspic33ak_spi_i2s_tdm_role_t role );
    bool (*clock_source_ready)( dspic33ak_spi_i2s_tdm_role_t role );
    dspic33ak_spi_i2s_tdm_clock_event_t (*consume_clock_event)( void );
} dspic33ak_spi_i2s_tdm_port_t;


//===========================================================
// HAL API. The transport's entry points:
//   - set_port()         : register the board/clock port (above). Call before
//                          inst_configure()/open(); NULL reverts to the self-clocked,
//                          no-gate default.
//   - spi1()/spi2()      : per-physical-SPI instance handles.
//   - inst_configure()   : validate + store a config_t for one instance (no HW write).
//   - set_block_callback(): register one instance's per-block event callback.
//   - open()/close()     : shared board/clock port bring-up/teardown (once for the
//                          engine, role-aware) -- see the per-instance lifecycle block
//                          below.
//   - inst_start()/inst_stop(): per-instance transport lifecycle. The app sequences
//                          configure each -> open(role) -> start(followers) ->
//                          start(block-ref) -> ... -> stop each -> close(). inst_start()
//                          returns true only if it actually started (false, instance
//                          stopped, if not configured / already running / rate
//                          unsupported / DMA setup fails) and never blocks.
//   - is_active()        : clock/source readiness gate (NOT running).
//   - is_running()       : block-timing-reference running state (start..stop).
//   - get_load()/get_status()       : block-timing reference (SPI1) load / status.
//   - inst_get_load()/inst_get_status(): a specific instance's load / status.
//   (DMA interrupt vectors: default (DSPIC33AK_TDM_DEFINE_DMA_VECTORS=1) the HAL owns
//    the _DMAnInterrupt vectors -- the integrator writes no ISR code. Opt-out (=0): the
//    integrator owns the IVT and calls dspic33ak_spi_i2s_tdm_inst_rx_isr() from their
//    own vector. TX is interrupt-less. Channel conflict: remap in conf.h.)
//===========================================================

// Register the board/clock port (fn-pointer hooks above). Pass NULL to clear it
// (revert to the self-clocked, no-gate default). Call before inst_configure()/open()
// (e.g. from the platform layer at init). The pointer is stored, not copied -- it
// must outlive the stream (use a static/const table).
extern void dspic33ak_spi_i2s_tdm_set_port( const dspic33ak_spi_i2s_tdm_port_t* port );

// Instance handles. The HAL is built from an instance descriptor list (conf.h), so the
// count is configurable. instance_count() returns how many instances this build has;
// inst(i) returns the i-th handle in list order (0 = the block-timing reference) or NULL
// if i is out of range. Together they let a caller enumerate instances
// (for i in 0 .. instance_count()-1: inst(i)) -- e.g. a future CMSIS-SAI wrapper mapping
// Driver_SAI0 -> inst(0), Driver_SAI1 -> inst(1) (that wrapper is not built yet).
// spi1()/spi2() are name-stable convenience wrappers: spi1() is the first instance; spi2()
// is the second, or NULL when only one is built (DSPIC33AK_TDM_USE_SPI2 == 0). Use the
// handle with set_block_callback().
extern uint8_t                       dspic33ak_spi_i2s_tdm_instance_count( void );
extern dspic33ak_spi_i2s_tdm_inst_t* dspic33ak_spi_i2s_tdm_inst( uint8_t index );
extern dspic33ak_spi_i2s_tdm_inst_t* dspic33ak_spi_i2s_tdm_spi1( void );
extern dspic33ak_spi_i2s_tdm_inst_t* dspic33ak_spi_i2s_tdm_spi2( void );

// Return one instance's current writable TX ping-pong half (the half NOT being
// transmitted), or NULL if inst is NULL/stopped/unresolved. Lets an app produce one
// instance's output from ANOTHER instance's block callback so two co-clocked codecs
// stay sample-aligned (call at a block boundary; co-clocked siblings share the phase).
// NULL-check before writing.
extern int32_t* dspic33ak_spi_i2s_tdm_inst_tx_fill_ptr( dspic33ak_spi_i2s_tdm_inst_t* inst );

// Register the per-completed-block callback for one SPI instance. The callback
// receives that instance's RX half just completed and the TX half it may fill.
// Register BEFORE inst_start(). Returns false (and changes nothing) on a contract
// violation: NULL inst, or the instance is running and the (cb,user) pair would
// change -- the callback must not be swapped or cleared mid-stream. Re-registering the
// identical (cb,user) while running is a no-op and returns true.
extern bool dspic33ak_spi_i2s_tdm_set_block_callback( dspic33ak_spi_i2s_tdm_inst_t* inst,
                                                      dspic33ak_spi_i2s_tdm_block_cb_t cb,
                                                      void* user );

// ---- Per-instance lifecycle (the app owns multi-instance ordering) ----
// open() brings up the SHARED board/clock port (external clock + pins + CLC) ONCE for
// the engine, role-aware -- pass the block-timing reference's role; returns false (do not
// start any instance) if the clock can't be brought up / isn't ready or a pin/CLC hook
// rejects the role. With no port registered it is a no-op success. It never blocks and
// touches no SPI/DMA. close() is the symmetric teardown: a near-no-op, since the HAL
// deliberately never tears down PPS/CLC or the clock (other peripherals may depend on
// them; reserved for a future clock-deinit hook).
// inst_configure/inst_start/inst_stop act on ONE instance handle (spi1()/spi2()). The
// app sequences them: configure each -> open(role) -> start(followers) -> start(block-ref)
// -> ... -> stop each (block-ref first to halt the cadence) -> close(). inst_start arms
// only that instance and requires the shared port to already be open()'d. For the
// co-clocked SPI1+SPI2 engine, "followers" = SPI2 and the block-timing reference = SPI1.
extern bool dspic33ak_spi_i2s_tdm_open( dspic33ak_spi_i2s_tdm_role_t role );
extern void dspic33ak_spi_i2s_tdm_close( void );
extern bool dspic33ak_spi_i2s_tdm_inst_configure( dspic33ak_spi_i2s_tdm_inst_t* inst,
                                                  const dspic33ak_spi_i2s_tdm_config_t* cfg );
extern bool dspic33ak_spi_i2s_tdm_inst_start( dspic33ak_spi_i2s_tdm_inst_t* inst );
extern void dspic33ak_spi_i2s_tdm_inst_stop( dspic33ak_spi_i2s_tdm_inst_t* inst );

// Return the clock/source readiness gate. This can be true while the transport is
// stopped; use is_running() when the question is "is audio streaming now?"
extern bool dspic33ak_spi_i2s_tdm_is_active( void );

// Return true only after start() succeeds and before stop() begins.
extern bool dspic33ak_spi_i2s_tdm_is_running( void );   // true stream-running state (start..stop)

// Consume one external-clock stop/resume edge from the board port, or NONE when
// no event/hook exists.
extern dspic33ak_spi_i2s_tdm_clock_event_t dspic33ak_spi_i2s_tdm_consume_clock_event( void );  // external-clock stop/resume edge

// NOTE: the transport is RATE-AGNOSTIC -- there is intentionally NO sample-rate API
// (no notify / get / set-callback / is-supported / rate_state). The HAL runs at the
// configured BRG (master) or the incoming external clock (slave) and never derives
// anything from a sample-rate value. Sample-rate POLICY is NOT a HAL property: the
// product/board's supported-rate set lives in the app layer (APP_SAMPLE_RATE_IS_SUPPORTED),
// used by the CMSIS-SAI wrapper to validate ARM_SAI AUDIO_FREQ. Runtime rate DETECTION +
// the stop->reconfigure->start it drives live in the application.

// Snapshot the load monitor / status. The singleton forms report the block-timing
// reference (SPI1); the inst forms report a specific instance (use spi1()/spi2()).
// For the inst forms, block_count/deadline_miss/load AND running are that instance's;
// only active (the clock/source readiness gate) is engine-wide/shared.
// clear_peak resets that instance's min/max/event peaks after the snapshot.
extern bool dspic33ak_spi_i2s_tdm_get_load( dspic33ak_spi_i2s_tdm_load_t* monitor, bool clear_peak );
extern bool dspic33ak_spi_i2s_tdm_get_status( dspic33ak_spi_i2s_tdm_status_t* status, bool clear_peak );
extern bool dspic33ak_spi_i2s_tdm_inst_get_load( dspic33ak_spi_i2s_tdm_inst_t* inst,
                                                 dspic33ak_spi_i2s_tdm_load_t* monitor,
                                                 bool clear_peak );
extern bool dspic33ak_spi_i2s_tdm_inst_get_status( dspic33ak_spi_i2s_tdm_inst_t* inst,
                                                   dspic33ak_spi_i2s_tdm_status_t* status,
                                                   bool clear_peak );


// Last-error diagnostic. The bool-returning calls (open / inst_configure / inst_start /
// set_block_callback) collapse several failure causes into one `false`; get_last_error()
// returns the most
// specific reason recorded by the most recent such call (ERR_NONE after a success). This
// is a DEBUG aid only -- NOT stream health: deadline misses / block counts live in
// get_status(), not here. It is the "last failed API reason", not a per-instance latch,
// and is intentionally not strictly interrupt/multi-core safe (a plain last-writer-wins
// store -- adequate as a 16-bit-MCU debug hint).
typedef enum {
    DSPIC33AK_SPI_I2S_TDM_ERR_NONE = 0,
    DSPIC33AK_SPI_I2S_TDM_ERR_BAD_INSTANCE,        // NULL / out-of-range instance handle
    DSPIC33AK_SPI_I2S_TDM_ERR_BAD_ARGUMENT,        // NULL cfg / other bad argument
    DSPIC33AK_SPI_I2S_TDM_ERR_NOT_CONFIGURED,      // start before a successful configure
    DSPIC33AK_SPI_I2S_TDM_ERR_ALREADY_RUNNING,     // start/configure while running
    DSPIC33AK_SPI_I2S_TDM_ERR_UNSUPPORTED_CONFIG,  // configure envelope rejected (format/slots/blk)
    DSPIC33AK_SPI_I2S_TDM_ERR_TOPOLOGY,            // instance-list/topology validation failed
    DSPIC33AK_SPI_I2S_TDM_ERR_CLOCK_INIT,          // port clock_source_init hook failed
    DSPIC33AK_SPI_I2S_TDM_ERR_CLOCK_NOT_READY,     // port clock_source_ready hook not ready
    DSPIC33AK_SPI_I2S_TDM_ERR_PIN_CONFIG,          // port configure_pins hook failed
    DSPIC33AK_SPI_I2S_TDM_ERR_CLC,                 // port clc_passthrough hook failed
    DSPIC33AK_SPI_I2S_TDM_ERR_DMA_CONFIG,          // DMA channel setup failed
} dspic33ak_spi_i2s_tdm_error_t;

extern dspic33ak_spi_i2s_tdm_error_t dspic33ak_spi_i2s_tdm_get_last_error( void );

// DMA interrupt vectors: by default (DSPIC33AK_TDM_DEFINE_DMA_VECTORS=1, conf.h) the HAL
// DEFINES the _DMAnInterrupt vectors itself (one RX vector per instance descriptor row),
// so the integrator writes NO interrupt/DMA code -- just registers a per-instance block
// callback. TX is interrupt-less (fire-and-forget ping-pong with auto-reload; hw.c
// enables the CPU IRQ on the RX channel only). RX/TX channel numbers come from conf.h and
// are baked in as compile-time constants so the DMA register access folds. To yield an
// IVT slot to another subsystem, either remap the SPI's channel in conf.h, or take full
// vector ownership: set DSPIC33AK_TDM_DEFINE_DMA_VECTORS=0 (the HAL then defines no
// vectors) and call inst_rx_isr() below from your own _DMA<rx>Interrupt for each instance.

// RX-block ISR entry for one instance, for the DSPIC33AK_TDM_DEFINE_DMA_VECTORS=0
// (vector-ownership opt-out) path: call it from your own _DMA<rx>Interrupt for that
// instance's RX channel (TX is interrupt-less -- never call it for a TX channel). It runs
// the same block work as the HAL's generated vector. A NULL inst is ignored. In the
// default turnkey build (=1) you do not call this -- the HAL's own vectors do the work.
extern void dspic33ak_spi_i2s_tdm_inst_rx_isr( dspic33ak_spi_i2s_tdm_inst_t* inst );



#endif // DSPIC33AK_SPI_I2S_TDM_H
