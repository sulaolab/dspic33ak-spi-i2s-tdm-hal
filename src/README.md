# dspic33ak_spi_i2s_tdm — SPI framed-mode I2S/TDM transport HAL

A compact, reusable SPI/I2S/TDM **transport** HAL for dsPIC33AK, carved from the Perseus
audio project. It moves audio frames over a framed SPI peripheral with DMA ping-pong and
a per-instance block callback. It is intentionally **small**: it does not try to be a
turnkey "drop-in and forget" audio stack. Board-specific, failsafe, and CMSIS-SAI
buffer-semantics concerns stay in layers above it, so a project can extend only what it
needs.

> Want to run it on hardware first?
> Start with [dspic33ak-hal-starter](https://github.com/sulaolab/dspic33ak-hal-starter),
> which vendors validated snapshots of the dsPIC33AK HAL repositories and provides a
> ready-to-build MPLAB X project for the dsPIC33AK Curiosity board.

## 1. What this HAL does

- dsPIC33AK SPI framed mode (AUDEN=0, FRMEN=1) used as an I2S/TDM transport.
- RX/TX ping-pong DMA, double-buffered.
- One per-instance block callback per physical SPI: `cb(src, dst, user)`.
- Selectable frame-sync waveform via `config.fs_shape`: `FS_PULSE` (short ~1-BCLK sync) or
  `FS_50PCT` (50%-duty FS). I2S 50% is native; a TDM **master** gets a 50%-duty FS
  synthesized by **CLC10** on the FS pin (auto-detected by PPS reverse-lookup; no app/CLC
  code). A TDM slave receives FS as an input, so `fs_shape` is accepted but has no
  generated-waveform effect (still validated, and compared as a framing field within a sync
  domain). See `dspic33ak_spi_i2s_tdm_fs_clc.{c,h}`.
- Two configure/lifecycle paths: **system** `configure_system(setups, count)` (transactional,
  all-or-nothing) + `open()` + `start_all_domains()`, or **single-instance**
  `inst_configure(inst, cfg)` + `open()` + `inst_start(inst)`. `open()` takes **no role** —
  it derives the clock role from the committed primary leg. Plus `get_status` / `get_load`
  diagnostics (block count, deadline-miss, ISR load; the arg-less ones report the primary leg).
- Optional board/clock **port** hook (`set_port()`) for pin/CLC routing and external-clock
  bring-up/readiness — the core calls only through this registered port.
- Per-instance SPI framed-transport health diagnostics (`SPIROV` / `SPITUR` / `FRMERR`, sampled
  once per completed RX block) — see "SPI framed-transport health diagnostics" below.
- Multi-instance: leg count from `DSPIC33AK_TDM_USE_SPI2` (1 or 2). The physical-SPI mapping is
  FIXED in the core (leg 0 = SPI1, leg 1 = SPI2), NOT from `conf.h`; `conf.h` supplies each leg's
  DMA channels, geometry, and initial `SYNC_DOMAIN`. Per-leg format/role come from the runtime
  config. The core defines the leg enum/buffers/table/`_DMA<rx>Interrupt` vectors in explicit C
  (no generator macro). Enumerate with `instance_count()` + `inst(i)`. See the root README for a
  pre-refactor -> current migration map.

## 2. What this HAL does NOT do

- No codec init (e.g. WM8904) — that is board/app code.
- No general board pin routing in the core — board FS/BCLK/DATA/MCLK routing is supplied by
  the registered port hook. **Exception:** for `TDM master + FS_50PCT`, the HAL-owned CLC10
  helper (`dspic33ak_spi_i2s_tdm_fs_clc.*`) temporarily repoints the already-routed `SSx` FS
  pin to `CLC10OUT` (and routes `SSx`→RPV8) and restores it on `release()`.
- No DSP — the callback owns any processing.
- No sample-rate policy — the transport is rate-agnostic (runs at the configured BRG or
  the incoming external clock); the supported-rate set is an app concern.
- No failsafe / board-specific teardown in the core — `close()` is a near-no-op; pin/clock
  release is left to the integrator (a future optional port deinit hook, not included).
- No CMSIS-SAI types in the core — `ARM_SAI_*` must not appear here.
- No automatic recovery from SPI framed-transport health-flag events (`SPIROV` / `SPITUR` /
  `FRMERR`) — the HAL only records per-RX-block observations (see "SPI framed-transport health
  diagnostics" below); reacting to them is an app-layer policy decision.

## 3. Required project config

- The project MUST provide `dspic33ak_spi_i2s_tdm_conf.h` on the include path.
- The HAL folder ships a self-contained template: `dspic33ak_spi_i2s_tdm_conf.h_example`.
- Copy/rename the example (or supply an equivalent header) and edit the geometry, the leg
  count (`DSPIC33AK_TDM_USE_SPI2`), the per-instance DMA channels, and the per-leg
  `SYNC_DOMAIN` defaults. `*.h_example` is never compiled.
- The template is self-contained (no app-config dependency). A project MAY instead derive
  the `DSPIC33AK_TDM_*` macros from its own app config (Perseus does this in
  `src/dspic33ak_spi_i2s_tdm_conf.h`); that is the integrator's choice and does not make
  the HAL core app-dependent (dependency is app → conf.h → HAL, never HAL → app).

## 4. Required sibling HALs

- `dspic33ak_dma` — DMA channel setup/arming (required). Standalone repo:
  [dspic33ak-dma-hal](https://github.com/sulaolab/dspic33ak-dma-hal).
- `dspic33ak_high_res_timer` — compile/link sibling dependency for the load monitor.
  Runtime use is gated by `dspic33ak_high_res_timer_is_initialized()`; if the timer is
  not initialized, `get_load()` / `inst_get_load()` returns `false` and zeroes the supplied
  load struct. Standalone repo:
  [dspic33ak-timer-hal](https://github.com/sulaolab/dspic33ak-timer-hal) (the
  Timer2 high-resolution counter).
- The SPI register-mask helper (`dspic33ak_spi_i2s_tdm_reg.h`) ships inside this HAL folder.

## 5. Supported devices

Currently supported (silicon facts present):

- `__dsPIC33AK512MPS512__`
- `__dsPIC33AK128MC106__`

> Note: the `FS_50PCT`-via-**CLC10** path (TDM master) requires CLC10 + virtual pin RPV8, so
> it is **AK512-only**. On **AK128** (no CLC10) a TDM master uses `FS_PULSE`; `FS_PULSE` and
> I2S-native `FS_50PCT` work on both parts.

The HAL `#error`s on any other device. Adding a new dsPIC33AK part means adding its
silicon facts in the HW layer (`dspic33ak_spi_i2s_tdm_hw.{c,h}`):

- SPI instance count
- `SPIxBUF` / `SPIxCON1` / `SPIxBRG` / `SPIxIMSK` / `SPIxSTAT` pointers
- DMA trigger CHSEL values
- CPU IRQ `IEC`/`IFS` masks

The vendor part macro is confined to one `DSPIC33AK_SPI_I2S_TDM_DEVICE` adapter
(opaque-tag derivation); app/HW code selects on that, not on the raw `__dsPIC33AK*__`.

## 6. SPI framed-transport health diagnostics

`dspic33ak_spi_i2s_tdm_hw_sample_ack_errflags(inst)` samples `SPIxSTAT` once per completed RX
block (`SPIROV | SPITUR | FRMERR`) and is the HAL's single ack point for the two
software-clearable bits, `SPIROV`/`FRMERR` (a W0C-safe write of only the software-clearable mask
with the observed bits zeroed — never a replay of the whole status word, so a previously
unobserved clearable bit asserted after the snapshot is preserved). `SPITUR` self-clears only in hardware
(`SPIEN=0`), so it is only ever observed. `dspic33ak_spi_i2s_tdm_diag_note_errflags()` folds the
mask into four per-instance, per-RX-block counters read via `get_status()`:
`err_rov_block_count` / `err_tur_block_count` / `err_frm_block_count` /
`frmerr_consecutive_blocks`. See the root README for the full ownership contract and counter
semantics.

## 7. Interrupt ownership

- `DSPIC33AK_TDM_DEFINE_DMA_VECTORS=1` (default): the HAL defines the RX DMA interrupt
  vectors itself (turnkey). Nothing else to wire.
- `DSPIC33AK_TDM_DEFINE_DMA_VECTORS=0`: the HAL defines no vectors; the integrator owns the
  IVT and calls `dspic33ak_spi_i2s_tdm_inst_rx_isr(inst)` from their own `_DMA<rx>Interrupt`
  for each instance.

TX is interrupt-less (no TX interrupt is enabled by the transport).

## 8. Tested envelope

State honestly:

- The default Perseus configuration is stable (boot, blocks advancing, `miss=0`, audio
  unchanged).
- An exhaustive format/role matrix test is **not** complete.
- Validated / currently intended envelope:
  - 32-bit word.
  - I2S 2-slot, or TDM 4/8/16/32 from the HAL envelope.
  - In practice the default test path is I2S / TDM8 depending on project config.
  - **Slave** (external BCLK/FS) is the main tested path.
  - A **master** (self-clocked) path exists; the TDM8 master with `FS_50PCT` (CLC10-generated
    50%-duty FS) was bench-verified on a dsPIC33AK Curiosity board (BCLK/FS = 256, `miss=0`).
    Other master rate/format combinations should still be confirmed on the target board.
- This snapshot is the **system-topology** model (transactional `configure_system()`,
  `open()` with no role, per-domain framing validation), HW-verified in the upstream Perseus
  source (co-clocked A/B, 94% load, deterministic phase-locked startup, CMSIS single-instance
  loopback) and bench-verified via the starter on a dsPIC33AK Curiosity board (TDM8 master
  smoke, `FS_PULSE`/`FS_50PCT`, stop→restart, negative-config self-test matrix).

## 9. CMSIS-SAI relationship

- A CMSIS-SAI wrapper is a layer **above** this HAL, not part of it.
- `ARM_SAI_*` types must not appear in this HAL core.
- The CMSIS wrapper owns Send/Receive buffer semantics, `tx_underflow` / `rx_overflow`,
  and sample-rate policy.
- This HAL's native diagnostics use `block_deadline_miss_count`, `block_count`, `load`, and the
  framed-transport health counters `err_rov_block_count` / `err_tur_block_count` /
  `err_frm_block_count` / `frmerr_consecutive_blocks` (SPIROV/SPITUR/FRMERR, sampled once per
  RX-block; see "SPI framed-transport health diagnostics" above).
- **Naming note:** this HAL's `err_rov_block_count` / `err_tur_block_count` (hardware
  `SPIROV`/`SPITUR`, sampled per RX block) are not the same signal as the CMSIS wrapper's
  `rx_overflow` / `tx_underflow` (its own software buffer-semantics events, one layer up) — the
  names read alike but the two pairs detect different failure modes at different layers.

---

### Full API contract → root README

This folder README covers **integration essentials** only. The complete public-API specification
lives in the root README of the standalone repo and is the single canonical reference (including
when you vendor just this folder into another project):

<https://github.com/sulaolab/dspic33ak-spi-i2s-tdm-hal>

It documents, in full:

- **Configuration model** — the SINGLE vs SYSTEM config-ownership mode; the transactional
  `configure_system()` all-or-nothing preflight (running / wire-format envelope / one MASTER per
  domain / same-domain framing match / `sync_domain` < 32); the one-way SYSTEM latch
  (`configure_system()` may recommit only while **closed + stopped**; no runtime SYSTEM→SINGLE);
  `open()` deriving the clock role from the committed primary leg; and the clock-readiness re-check
  just before arming.
- **Lifecycle** — `open()` idempotent; `close()` / `set_port()` return `bool` and reject while a
  leg is running (and `set_port()` also while open); `close()` is a near-no-op (no board/clock
  teardown).
- **Block-callback contract** — register before `inst_start()`; `src`/`dst` are both non-NULL when
  the callback runs, else the block is skipped.
- **CANDIDATE co-clock API** — `inst_tx_fill_mirror()` (typed `mirror_result_t` + `int32_t** dst`
  out-param) and the `tx_active_half()` / `tx_active_pos()` probes.
- **Diagnosing a failed call** — every `bool`-returning API sets `get_last_error()` on `false`.
