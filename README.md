# dspic33ak_spi_i2s_tdm — SPI framed-mode I2S/TDM transport HAL

A compact, reusable SPI/I2S/TDM **transport** HAL for dsPIC33AK, carved from the Perseus
audio project. It moves audio frames over a framed SPI peripheral with DMA ping-pong and
a per-instance block callback. It is intentionally **small**: it does not try to be a
turnkey "drop-in and forget" audio stack. Board-specific, failsafe, and CMSIS-SAI
buffer-semantics concerns stay in layers above it, so a project can extend only what it
needs.

## 1. What this HAL does

- dsPIC33AK SPI framed mode (AUDEN=0, FRMEN=1) used as an I2S/TDM transport.
- RX/TX ping-pong DMA, double-buffered.
- One per-instance block callback per physical SPI: `cb(src, dst, user)`.
- `open` / `inst_configure` / `inst_start` / `inst_stop` / `close` lifecycle, plus
  `get_status` / `get_load` diagnostics (block count, deadline-miss, ISR load).
- Optional board/clock **port** hook (`set_port()`) for pin/CLC routing and external-clock
  bring-up/readiness — the core calls only through this registered port.
- Multi-instance: instance count / physical-SPI / DMA channels / format / block size all
  come from the instance list in `conf.h`. Enumerate with `instance_count()` + `inst(i)`.

## 2. What this HAL does NOT do

- No codec init (e.g. WM8904) — that is board/app code.
- No PPS/CLC pin routing in the core — reached only through the registered port hook.
- No DSP — the callback owns any processing.
- No sample-rate policy — the transport is rate-agnostic (runs at the configured BRG or
  the incoming external clock); the supported-rate set is an app concern.
- No failsafe / board-specific teardown in the core — `close()` is a near-no-op; pin/clock
  release is left to the integrator (a future optional port deinit hook, not included).
- No CMSIS-SAI types in the core — `ARM_SAI_*` must not appear here.

## 3. Required project config

- The project MUST provide `dspic33ak_spi_i2s_tdm_conf.h` on the include path.
- The HAL folder ships a self-contained template: `dspic33ak_spi_i2s_tdm_conf.h_example`.
- Copy/rename the example (or supply an equivalent header) and edit the instance list +
  geometry. `*.h_example` is never compiled.
- The template is self-contained (no app-config dependency). A project MAY instead derive
  the `DSPIC33AK_TDM_*` macros from its own app config (Perseus does this in
  `src/dspic33ak_spi_i2s_tdm_conf.h`); that is the integrator's choice and does not make
  the HAL core app-dependent (dependency is app → conf.h → HAL, never HAL → app).

## 4. Required sibling HALs

- `dspic33ak_dma` — DMA channel setup/arming (required).
- `dspic33ak_high_res_timer` — used by the load monitor **when initialized**; if the timer
  is not initialized, `get_load()` returns `valid=false` (no hard dependency, no device
  gating).
- The SPI register-mask helper (`dspic33ak_spi_i2s_tdm_reg.h`) ships inside this HAL folder.

## 5. Supported devices

Currently supported (silicon facts present):

- `__dsPIC33AK512MPS512__`
- `__dsPIC33AK128MC106__`

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
  - A **master** (self-clocked) path exists but should be treated as less tested unless
    confirmed on the target board.

## 8. CMSIS-SAI relationship

- A CMSIS-SAI wrapper is a layer **above** this HAL, not part of it.
- `ARM_SAI_*` types must not appear in this HAL core.
- The CMSIS wrapper owns Send/Receive buffer semantics, `tx_underflow` / `rx_overflow`,
  and sample-rate policy.
- This HAL's native diagnostics use `block_deadline_miss_count`, `block_count`, and `load`.

---

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
