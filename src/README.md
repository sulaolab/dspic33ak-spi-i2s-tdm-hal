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
  code). A TDM slave receives FS as an input and ignores `fs_shape`. See
  `dspic33ak_spi_i2s_tdm_fs_clc.{c,h}`.
- Two configure/lifecycle paths: **system** `configure_system(setups, count)` (transactional,
  all-or-nothing) + `open()` + `start_all_domains()`, or **single-instance**
  `inst_configure(inst, cfg)` + `open()` + `inst_start(inst)`. `open()` takes **no role** —
  it derives the clock role from the committed primary leg. Plus `get_status` / `get_load`
  diagnostics (block count, deadline-miss, ISR load; the arg-less ones report the primary leg).
- Optional board/clock **port** hook (`set_port()`) for pin/CLC routing and external-clock
  bring-up/readiness — the core calls only through this registered port.
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
  not initialized, `get_load()` returns `valid=false`. Standalone repo:
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
- `SPIxBUF` / `SPIxCON1` / `SPIxBRG` / `SPIxIMSK` pointers
- DMA trigger CHSEL values
- CPU IRQ `IEC`/`IFS` masks

The vendor part macro is confined to one `DSPIC33AK_SPI_I2S_TDM_DEVICE` adapter
(opaque-tag derivation); app/HW code selects on that, not on the raw `__dsPIC33AK*__`.

## 6. Interrupt ownership

- `DSPIC33AK_TDM_DEFINE_DMA_VECTORS=1` (default): the HAL defines the RX DMA interrupt
  vectors itself (turnkey). Nothing else to wire.
- `DSPIC33AK_TDM_DEFINE_DMA_VECTORS=0`: the HAL defines no vectors; the integrator owns the
  IVT and calls `dspic33ak_spi_i2s_tdm_inst_rx_isr(inst)` from their own `_DMA<rx>Interrupt`
  for each instance.

TX is interrupt-less (no TX interrupt is enabled by the transport).

## 7. Tested envelope

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
  loopback). A fresh on-board smoke of this standalone/starter snapshot is the remaining step.

## 8. CMSIS-SAI relationship

- A CMSIS-SAI wrapper is a layer **above** this HAL, not part of it.
- `ARM_SAI_*` types must not appear in this HAL core.
- The CMSIS wrapper owns Send/Receive buffer semantics, `tx_underflow` / `rx_overflow`,
  and sample-rate policy.
- This HAL's native diagnostics use `block_deadline_miss_count`, `block_count`, and `load`.

---

### Configuration model (summary)

Two ways to configure, both ending in `open()` → start:

- **System (transactional, recommended).** `configure_system(setups, count)` takes one
  `leg_setup_t` per leg (`{ stream, sync_domain }`; `count` == the built leg count) and is
  all-or-nothing. A side-effect-free PREFLIGHT rejects the whole call — touching no leg — if
  any leg is running or outside the wire-format envelope, if a sync domain holds more than one
  clock MASTER, if same-domain legs disagree on the frame interpretation (format / word_bits /
  slots / block_frames / SPIFE / CKP / CKE), or if any `sync_domain` ≥ 32. Only after a clean
  preflight are all legs committed together — no half-configured mix.
- **Single-instance.** `inst_configure(inst, cfg)` + `open()` + `inst_start(inst)` for a
  single-leg driver (e.g. a CMSIS-SAI wrapper).

`open()` takes no role: it derives the clock role from the committed **primary** leg
(`primary_leg_index`, default leg 0), failing `ERR_NOT_CONFIGURED` if the primary is
unconfigured. `inst_get_setup(inst, &out)` reads a leg's committed setup (pure query; `false`
for an unconfigured leg, distinct from a valid SLAVE). `start_all_domains()` starts each sync
domain once, phase-locked (co-clocked members' `SPIEN` released back-to-back, slaves first,
master last). The arg-less `is_running()` / `get_status()` / `get_load()` report the primary
leg. A small **CANDIDATE, non-generic** co-clock-only API (`inst_tx_fill_ptr_mirror()`,
`tx_active_half()` / `tx_active_pos()`) exists for dual-codec use and may change.

### Block-callback contract (summary)

Register the callback with `set_block_callback()` **before** `inst_start()`; do not
swap/clear it while running (an identical re-register is an idempotent no-op). When the
callback runs, **`src` and `dst` are both non-NULL** — if the core cannot resolve the RX
or TX half it skips that block instead of calling the callback. With no callback, that
instance runs no DSP path (its zeroed TX half stays silent).

### Diagnosing a failed call

`open()` / `inst_configure()` / `inst_start()` return `bool`. On `false`,
`dspic33ak_spi_i2s_tdm_get_last_error()` returns the most specific reason
(`dspic33ak_spi_i2s_tdm_error_t`). This is a debug aid only — stream health (deadline
misses, block counts) lives in `get_status()`, not here.
