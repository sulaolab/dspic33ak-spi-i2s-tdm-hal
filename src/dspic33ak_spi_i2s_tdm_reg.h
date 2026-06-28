#ifndef DSPIC33AK_SPI_I2S_TDM_REG_H
#define DSPIC33AK_SPI_I2S_TDM_REG_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Internal register helper layer for the SPI framed-mode TDM/I2S transport.
 *
 * Like dspic33ak_dma_reg.h / dspic33ak_i2c_reg.h, this intentionally uses plain
 * 32-bit register pointers and bit masks instead of XC-DSC bitfield structures
 * such as SPIxCON1bits.  The goal is to keep compiler/DFP-specific details away
 * from the readable driver.
 *
 * Bit positions were checked against:
 *   Microchip dsPIC33AK-MP_DFP 1.3.185  p33AK512MPS512.h (tagSPI1CON1BITS / tagSPI1IMSKBITS)
 *   Microchip dsPIC33AK-MC_DFP 1.4.172  p33AK128MC106.h
 * SPIxCON1 / SPIxBRG / SPIxIMSK are uniform 32-bit SFRs across instances and
 * across both devices.
 *
 * Keep this file small.  Add only the bits actually used by the readable driver.
 */

/* ---- SPIxCON1 single-bit fields ---- */
#define DSPIC33AK_SPI_I2S_TDM_CON1_SPIFE    (1UL << 1)    /* SPIxCON1bits.SPIFE   */
#define DSPIC33AK_SPI_I2S_TDM_CON1_MCLKEN   (1UL << 2)    /* SPIxCON1bits.MCLKEN  */
#define DSPIC33AK_SPI_I2S_TDM_CON1_DISSCK   (1UL << 3)    /* SPIxCON1bits.DISSCK  */
#define DSPIC33AK_SPI_I2S_TDM_CON1_DISSDI   (1UL << 4)    /* SPIxCON1bits.DISSDI  */
#define DSPIC33AK_SPI_I2S_TDM_CON1_MSTEN    (1UL << 5)    /* SPIxCON1bits.MSTEN   */
#define DSPIC33AK_SPI_I2S_TDM_CON1_CKP      (1UL << 6)    /* SPIxCON1bits.CKP     */
#define DSPIC33AK_SPI_I2S_TDM_CON1_CKE      (1UL << 8)    /* SPIxCON1bits.CKE     */
#define DSPIC33AK_SPI_I2S_TDM_CON1_MODE16   (1UL << 10)   /* SPIxCON1bits.MODE16  */
#define DSPIC33AK_SPI_I2S_TDM_CON1_MODE32   (1UL << 11)   /* SPIxCON1bits.MODE32  */
#define DSPIC33AK_SPI_I2S_TDM_CON1_DISSDO   (1UL << 12)   /* SPIxCON1bits.DISSDO  */
#define DSPIC33AK_SPI_I2S_TDM_CON1_ON       (1UL << 15)   /* SPIxCON1bits.ON      */
#define DSPIC33AK_SPI_I2S_TDM_CON1_FRMSYPW  (1UL << 19)   /* SPIxCON1bits.FRMSYPW */
#define DSPIC33AK_SPI_I2S_TDM_CON1_FRMPOL   (1UL << 21)   /* SPIxCON1bits.FRMPOL  */
#define DSPIC33AK_SPI_I2S_TDM_CON1_FRMSYNC  (1UL << 22)   /* SPIxCON1bits.FRMSYNC */
#define DSPIC33AK_SPI_I2S_TDM_CON1_FRMEN    (1UL << 23)   /* SPIxCON1bits.FRMEN   */
#define DSPIC33AK_SPI_I2S_TDM_CON1_IGNTUR   (1UL << 28)   /* SPIxCON1bits.IGNTUR  */
#define DSPIC33AK_SPI_I2S_TDM_CON1_IGNROV   (1UL << 29)   /* SPIxCON1bits.IGNROV  */
#define DSPIC33AK_SPI_I2S_TDM_CON1_AUDEN    (1UL << 31)   /* SPIxCON1bits.AUDEN   */

/* ---- SPIxCON1 multi-bit field (position + mask) ---- */
#define DSPIC33AK_SPI_I2S_TDM_CON1_FRMCNT_POS   (16)
#define DSPIC33AK_SPI_I2S_TDM_CON1_FRMCNT_MASK  (0x7UL << DSPIC33AK_SPI_I2S_TDM_CON1_FRMCNT_POS) /* SPIxCON1bits.FRMCNT */

/* ---- SPIxIMSK bits (DMA-trigger event enables) ---- */
#define DSPIC33AK_SPI_I2S_TDM_IMSK_SPIRBFEN (1UL << 0)    /* SPIxIMSKbits.SPIRBFEN */
#define DSPIC33AK_SPI_I2S_TDM_IMSK_SPITBEN  (1UL << 3)    /* SPIxIMSKbits.SPITBEN  */

/* ---- Minimal generic 32-bit SFR access helpers ---- */
/*
 * Set one or more bits in a 32-bit SFR.
 *
 * `mask` is written with a read-modify-write sequence. Callers supply volatile
 * register addresses from the SPI device table.
 */
static inline void dspic33ak_spi_i2s_tdm_reg_set(volatile uint32_t *reg, uint32_t mask)
{
    *reg |= mask;
}

/*
 * Clear one or more bits in a 32-bit SFR.
 *
 * The helper mirrors reg_set() so the readable driver can avoid XC-DSC bitfield
 * syntax and stay close to data-sheet mask names.
 */
static inline void dspic33ak_spi_i2s_tdm_reg_clear(volatile uint32_t *reg, uint32_t mask)
{
    *reg &= ~mask;
}

/*
 * Set or clear a masked field based on a boolean.
 *
 * This collapses the common "if on set else clear" register pattern and keeps
 * config writers readable when mapping config_t booleans to SPIxCON1 bits.
 */
static inline void dspic33ak_spi_i2s_tdm_reg_set_or_clear(volatile uint32_t *reg, uint32_t mask, bool on)
{
    if (on)
        *reg |= mask;
    else
        *reg &= ~mask;
}

/*
 * Test whether any bit in `mask` is set.
 *
 * Used for simple SFR state checks while preserving the same mask-based access
 * style as the rest of this register helper layer.
 */
static inline bool dspic33ak_spi_i2s_tdm_reg_is_set(volatile uint32_t *reg, uint32_t mask)
{
    return ((*reg & mask) != 0u);
}

/*
 * Write a shifted multi-bit field inside a 32-bit SFR.
 *
 * Existing bits outside `mask` are preserved. `value` is unshifted on input; the
 * helper shifts it by `pos` and clips it to `mask` before writing.
 */
static inline void dspic33ak_spi_i2s_tdm_reg_write_field(volatile uint32_t *reg,
                                                     uint32_t mask,
                                                     uint32_t pos,
                                                     uint32_t value)
{
    *reg = (*reg & ~mask) | ((value << pos) & mask);
}

#endif /* DSPIC33AK_SPI_I2S_TDM_REG_H */
