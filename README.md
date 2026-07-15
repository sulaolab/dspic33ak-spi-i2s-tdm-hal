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

<img src="docs/images/tdm8-scope-mikrobus-a.png" alt="Oscilloscope capture of MikroBUS-A SPI pins during a TDM8 master smoke demo: BCLK (~12.5 MHz, yellow), a ~50%-duty frame sync FS (~49 kHz, blue, CLC10-generated), and DataOut carrying the TDM8 slot data (red); BCLK/FS = 256" width="900">

Oscilloscope capture from the dspic33ak-hal-starter TDM8 **master** smoke demo
running on a dsPIC33AK Curiosity board (MikroBUS-A pins, no codec). BCLK
~12.5 MHz (yellow); frame sync FS ~49 kHz (blue) — here the **50%-duty** LRCLK-style
FS produced by the HAL's `fs_shape = FS_50PCT` option (CLC10-generated, BCLK/FS = 256);
DataOut with 8 × 32-bit slots (red). This HAL drives the signal; board pin routing
and DMA configuration are supplied by the integrating project — see
[dspic33ak-hal-starter](https://github.com/sulaolab/dspic33ak-hal-starter) for
a complete worked example.

## 1. What this HAL does

- dsPIC33AK SPI framed mode (AUDEN=0, FRMEN=1) used as an I2S/TDM transport.
- RX/TX ping-pong DMA, double-buffered.
- One per-instance block callback per physical SPI: `cb(src, dst, user)`.
- Selectable frame-sync waveform via `config.fs_shape` (the app states intent; the HAL
  hides FRMSYPW/FRMCNT/CLC):
  - `FS_PULSE` — short ~1-BCLK frame sync (DSP/TDM short sync).
  - `FS_50PCT` — 50%-duty FS (I2S LRCLK style). I2S is native (FRMSYPW=1); a TDM **master**
    gets a ~50%-duty FS synthesized by **CLC10** (a J-K flip-flop toggled by a half-frame
    FRMSYNC marker, fed internally via virtual pin RPV8) on the same FS pin — no app/CLC code
    and no extra pin (the FS pin is auto-detected by PPS reverse-lookup). A TDM slave receives
    FS as an input, so it ignores `fs_shape`. See `dspic33ak_spi_i2s_tdm_fs_clc.{c,h}`.
- Two configure/lifecycle paths (see "Configuration model" at the end):
  - **System** (recommended, multi-leg): `configure_system(setups, count)` applies ALL legs
    transactionally (all-or-nothing), then `open()` + `start_all_domains()`.
  - **Single-instance**: `inst_configure(inst, cfg)` + `open()` + `inst_start(inst)`.
  `open()` takes **no role** — it derives the clock role from the committed primary leg.
  `inst_stop()` / `close()` tear down. Plus `get_status` / `get_load` diagnostics (block
  count, deadline-miss, ISR load) — the arg-less ones report the primary leg.
- Optional board/clock **port** hook (`set_port()`) for pin/CLC routing and external-clock
  bring-up/readiness — the core calls only through this registered port.
- Multi-instance: the leg count is chosen by `DSPIC33AK_TDM_USE_SPI2` (1 or 2). The
  physical-SPI mapping is **fixed in the core** (leg 0 = SPI1, leg 1 = SPI2) -- it does NOT
  come from `conf.h`. What `conf.h` supplies per leg is the RX/TX DMA channels, the geometry
  (`SLOTS_PER_FS` / `BLOCK_FRAMES`), and the initial `SYNC_DOMAIN`; per-leg format / clock role
  come from the runtime config (`configure_system` / `inst_configure`). The core defines the
  leg enum, ping-pong buffers, leg table, and `_DMA<rx>Interrupt` vectors in **explicit C**
  (no generator macro). Enumerate with `instance_count()` + `inst(i)`.

## 2. What this HAL does NOT do

- No codec init (e.g. WM8904) — that is board/app code.
- No general board pin routing in the core — board FS/BCLK/DATA/MCLK routing is supplied by
  the registered port hook. **Exception:** for `TDM master + FS_50PCT`, the HAL-owned CLC10
  helper (`dspic33ak_spi_i2s_tdm_fs_clc.*`) does touch PPS/CLC — it temporarily repoints the
  already-routed `SSx` FS pin to `CLC10OUT` (and routes `SSx`→RPV8) and restores it on
  `release()`. This is the one place the core itself drives PPS/CLC.
- No DSP — the callback owns any processing.
- No sample-rate policy — the transport is rate-agnostic (runs at the configured BRG or
  the incoming external clock); the supported-rate set is an app concern.
- No failsafe / board-specific teardown in the core — `close()` is a near-no-op; pin/clock
  release is left to the integrator (a future optional port deinit hook, not included).
- No CMSIS-SAI types in the core — `ARM_SAI_*` must not appear here.

## 3. Required project config

- The project MUST provide `dspic33ak_spi_i2s_tdm_conf.h` on the include path.
- The HAL folder ships a self-contained template: `dspic33ak_spi_i2s_tdm_conf.h_example`.
- Copy/rename the example (or supply an equivalent header) and edit the geometry
  (`DSPIC33AK_TDM_SLOTS_PER_FS` / `_BLOCK_FRAMES`), the leg count (`DSPIC33AK_TDM_USE_SPI2`),
  the per-instance DMA channels, and the per-leg `SYNC_DOMAIN` defaults. `*.h_example` is
  never compiled.
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

> Note: the `FS_50PCT`-via-**CLC10** path (TDM master) is a feature of parts that have CLC10 +
> virtual pin RPV8 — i.e. **AK512**. On **AK128** (no CLC10) it is unavailable: `engage()`
> reports `NO_FS_PIN` and a TDM master can use `FS_PULSE`. `FS_PULSE` and I2S-native
> `FS_50PCT` work on both parts. (The device itself is HAL-supported; only the CLC10 50%-FS
> generator is AK512-only.)

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
    50%-duty FS) was bench-verified on a dsPIC33AK Curiosity board (BCLK/FS = 256, `miss=0`,
    plus the single-codec starter demo and the Perseus dual-codec topology). Other
    master rate/format combinations should still be confirmed on the target board.
- This snapshot is the **system-topology** model (transactional `configure_system()`,
  `open()` with no role, per-domain framing validation). That code is HW-verified in the
  upstream Perseus source — co-clocked dual-codec A/B, 80-stage/94% CPU load, deterministic
  phase-locked startup (`miss=0`), and the CMSIS-SAI single-instance loopback. A fresh
  on-board smoke of THIS standalone/starter snapshot (TDM8 / `FS_PULSE` / `FS_50PCT` /
  stop→restart + the negative-config matrix) is the remaining verification step.

## 8. CMSIS-SAI relationship

- A CMSIS-SAI wrapper is a layer **above** this HAL, not part of it.
- `ARM_SAI_*` types must not appear in this HAL core.
- The CMSIS wrapper owns Send/Receive buffer semantics, `tx_underflow` / `rx_overflow`,
  and sample-rate policy.
- This HAL's native diagnostics use `block_deadline_miss_count`, `block_count`, and `load`.

---

### Migration from the pre-refactor HAL

This is a **breaking** API change from the earlier X-macro / `BLOCK_REF` HAL. A consumer built
against the old API will not compile until updated; the map:

| Before | Now |
|---|---|
| `dspic33ak_spi_i2s_tdm_role_t`, `..._ROLE_MASTER` / `..._ROLE_SLAVE` | `..._clock_role_t`, `..._CLOCK_MASTER` / `..._CLOCK_SLAVE` |
| `config_t.role` | `config_t.clock_role` |
| `open(role)` | `open(void)` — role derived from the committed primary leg |
| `DSPIC33AK_TDM_INSTANCE_LIST(X)` X-macro + `BLOCK_REF` / `FOLLOWER` | explicit SPI1 / optional SPI2 in the core; leg count via `DSPIC33AK_TDM_USE_SPI2` |
| per-leg `inst_configure()` ×N for a multi-leg stream | `configure_system(setups, count)` (transactional, all-or-nothing) |
| public `inst_arm()` / `inst_go()` | internal only — use `inst_start()` / `start_domain()` / `start_all_domains()` |

### Configuration model (summary)

Two ways to configure, both ending in `open()` → start:

- **System (transactional, recommended).** `configure_system(setups, count)` takes one
  `leg_setup_t` per leg (`{ stream, sync_domain }`; `count` == the built leg count) and is
  all-or-nothing. A side-effect-free PREFLIGHT rejects the whole call — touching no leg — if
  any leg is running or outside the wire-format envelope, if a sync domain holds more than one
  clock MASTER, if two legs sharing a sync domain disagree on the frame interpretation
  (format / word_bits / slots / block_frames / SPIFE / CKP / CKE / `fs_shape`), or if any
  `sync_domain` is ≥ 32 (the domain id range `start_all_domains()` can track). Only after a clean
  preflight are all legs committed together, so there is never a half-configured mix.
- **Single-instance.** `inst_configure(inst, cfg)` validates + stores one leg's config; use
  it with `open()` + `inst_start(inst)` for a single-leg driver (e.g. a CMSIS-SAI wrapper).

The two paths establish a mutually-exclusive **config-ownership mode** — a property of the
committed configuration, independent of the open/close lifecycle (`close()` does not reset it):
`inst_configure()` → **SINGLE**, in which the per-leg API (`inst_configure`/`inst_start`/
`inst_stop`) is legal only on the **primary** leg; `configure_system()` → **SYSTEM**, in which the
whole-system domain API (`configure_system`/`start_domain`/`start_all_domains`/`stop_domain`/
`stop_all_domains`) is legal. A call from the wrong family (or a non-primary leg via `inst_*`)
returns `ERR_CONFIG_MODE`. The latch is one-way in practice: `configure_system()` may full-recommit
from any stopped+closed mode (SINGLE→SYSTEM ok), but there is no runtime SYSTEM→SINGLE reset — a
compile-time-fixed integration never needs one, and switching a running product from the domain API
to a single-leg SAI driver takes a power cycle.
`open()` is idempotent (a second call re-runs no hooks); `close()` and `set_port()` return `bool`
and reject while a leg is running (or, for `set_port()`, while open). The start paths re-check the
clock-readiness gate immediately before arming, so a source that drops between `open()` and start
fails the start (`ERR_CLOCK_NOT_READY`).

`open()` takes no role: it derives the clock role from the committed **primary** leg
(`primary_leg_index`, default leg 0) and passes it to the port hooks; it fails
(`ERR_NOT_CONFIGURED`) if the primary is unconfigured. A board port hook that also routes a
secondary leg reads that leg's committed role via `inst_get_setup(inst, &out)` (a pure query;
returns `false` for an unconfigured leg — distinct from a valid SLAVE, role value 0 — so the
hook can skip a leg not part of this run). `start_all_domains()` starts each sync domain once,
releasing the co-clocked members' `SPIEN` back-to-back (slaves first, clock master last) so
they latch one frame edge (phase-locked). The arg-less `is_running()` / `get_status()` /
`get_load()` report the primary leg.

A small **CANDIDATE, non-generic** API supports co-clocked dual-codec use only:
`inst_tx_fill_ptr_mirror()` and the `tx_active_half()` / `tx_active_pos()` phase probes. A
generic single- or independent-instance consumer does not need them; they may change or move.

### Block-callback contract (summary)

Register the callback with `set_block_callback()` **before** `inst_start()`; do not
swap/clear it while running (an identical re-register is an idempotent no-op). When the
callback runs, **`src` and `dst` are both non-NULL** — if the core cannot resolve the RX
or TX half it skips that block instead of calling the callback. With no callback, that
instance runs no DSP path (its zeroed TX half stays silent).

### Diagnosing a failed call

The bool-returning calls — `set_port()`, `open()`, `close()`, `inst_configure()`,
`configure_system()`, `inst_start()`, `inst_stop()`, `start_domain()`, `start_all_domains()`,
`stop_domain()`, `stop_all_domains()`, `set_block_callback()` — collapse several causes into one
`false`. On `false`, `dspic33ak_spi_i2s_tdm_get_last_error()` returns the most specific reason
(`dspic33ak_spi_i2s_tdm_error_t`). This is a debug aid only — stream health (deadline
misses, block counts) lives in `get_status()`, not here.
