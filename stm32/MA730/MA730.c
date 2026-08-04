
/**
 * Driver for a Monolithic Power Systems MA730 magnetic resolver
 *
 * Created by Adam Spencer @ 4 Aug 2026
 */
#include "MA730.h"

#include <stddef.h>
#include <string.h>

enum MA730_Error ma730_init(struct MA730_Driver* blank_driver,
    enum MA730_Error (*spi_xmit)(uint16_t* payload),
    enum MA730_Error (*spi_recv)(uint16_t* payload)
)
{
    if (blank_driver == NULL) return MA730_NULL;
    if (spi_xmit == NULL) return MA730_NULL;
    if (spi_recv == NULL) return MA730_NULL;

    memset(blank_driver, 0, sizeof(struct MA730_Driver));

    blank_driver->spi_xmit = spi_xmit;
    blank_driver->spi_recv = spi_recv;

    return MA730_OK;
}

//
//
// Read/Write Register Functions
//
//

static enum MA730_Error write_reg(struct MA730_Driver* drv, enum MA730_RegAddr addr, uint8_t value)
{
    if (NULL == drv) return MA730_NULL;
    if (!ma730_is_valid_regaddr(addr)) return MA730_BAD_ADDR;

    // Do the transmission bit

    enum MA730_Error err = MA730_OK;

    uint16_t payload_regaddr = ((uint16_t) addr) << 8;
    err = drv->spi_xmit(&payload_regaddr);
    if (MA730_OK != err) return err;

    uint16_t payload_value = ((uint16_t) value) << 8;
    err = drv->spi_xmit(&payload_value);
    if (MA730_OK != err) return err;

    return MA730_OK;
}

static enum MA730_Error read_reg(struct MA730_Driver* drv, enum MA730_RegAddr addr, uint8_t* value)
{
    if (NULL == drv) return MA730_NULL;
    if (NULL == value) return MA730_NULL;
    if (!ma730_is_valid_regaddr(addr)) return MA730_BAD_ADDR;

    //

    enum MA730_Error err = MA730_OK;

    uint16_t payload_regaddr = ((uint16_t) addr) << 8;
    err = drv->spi_xmit(&payload_regaddr);
    if (MA730_OK != err) return err;

    uint16_t payload_value;
    err = drv->spi_recv(&payload_value);
    if (MA730_OK != err) return err;

    *value = (uint8_t) (payload_value >> 8);

    return MA730_OK;
}

//
//
// Encoder Angle
//
//

enum MA730_Error ma730_read_angle(struct MA730_Driver* drv, float* angle)
{
    if (NULL == drv) return MA730_NULL;
    if (NULL == angle) return MA730_NULL;

    uint16_t angle_ticks;
    enum MA730_Error err = MA730_OK;
    err = drv->spi_recv(&angle_ticks);
    if (MA730_OK != err) return err;

    *angle = ((float) angle_ticks * 360.0f) / 65536.0f;

    return MA730_OK;
}

//
//
// Specific Registers
//
//

//
// Zero Position
//

enum MA730_Error ma730_write_zero_deg(struct MA730_Driver* drv, float zero_deg)
{
    if (NULL == drv) return MA730_NULL;
    if ((zero_deg < 0.0f) || (zero_deg > 360.0f)) return MA730_BAD_RANGE;

    uint32_t zero_ticks = (uint32_t) lroundf((zero_deg * 65536.0f) / 360.0f);
    zero_ticks &= 0xFFFFu;

    uint16_t z = (uint16_t) ((0x10000u - zero_ticks) & 0xFFFFu);

    // Datasheet maps Z low byte to register 0x00 and high byte to register 0x01.
    enum MA730_Error err = write_reg(drv, MA730_REGADDR_Z_MSB, (uint8_t) (z & 0xFFu));
    if (MA730_OK != err) return err;

    return write_reg(drv, MA730_REGADDR_Z_LSB, (uint8_t) ((z >> 8) & 0xFFu));
}

enum MA730_Error ma730_read_zero_deg(struct MA730_Driver* drv, float* zero_deg)
{
    if (NULL == drv) return MA730_NULL;
    if (NULL == zero_deg) return MA730_NULL;

    uint8_t z_lsb;
    uint8_t z_msb;

    enum MA730_Error err = read_reg(drv, MA730_REGADDR_Z_MSB, &z_lsb);
    if (MA730_OK != err) return err;

    err = read_reg(drv, MA730_REGADDR_Z_LSB, &z_msb);
    if (MA730_OK != err) return err;

    uint16_t z = ((uint16_t) z_msb << 8) | (uint16_t) z_lsb;
    uint16_t zero_ticks = (uint16_t) ((0x10000u - (uint32_t) z) & 0xFFFFu);

    *zero_deg = ((float) zero_ticks * 360.0f) / 65536.0f;

    return MA730_OK;
}

//
// Bias Current
//

enum MA730_Error ma730_write_bias_current(struct MA730_Driver* drv, float bias)
{
    if (NULL == drv) return MA730_NULL;
    if ((bias < 0.0f) || (bias > 255.0f)) return MA730_BAD_RANGE;

    uint8_t bct = (uint8_t) lroundf(bias);
    return write_reg(drv, MA730_REGADDR_BCT, bct);
}

enum MA730_Error ma730_read_bias_current(struct MA730_Driver* drv, float* bias)
{
    if (NULL == drv) return MA730_NULL;
    if (NULL == bias) return MA730_NULL;

    uint8_t bct;
    enum MA730_Error err = read_reg(drv, MA730_REGADDR_BCT, &bct);
    if (MA730_OK != err) return err;

    *bias = (float) bct;

    return MA730_OK;
}

//
// Trimming
//

enum MA730_Error ma730_write_trimming(struct MA730_Driver* drv, bool x, bool y)
{
    if (NULL == drv) return MA730_NULL;

    uint8_t reg;
    enum MA730_Error err = read_reg(drv, MA730_REGADDR_ET, &reg);
    if (MA730_OK != err) return err;

    reg &= (uint8_t) ~0x03u;
    if (x) reg |= 0x01u;
    if (y) reg |= 0x02u;

    return write_reg(drv, MA730_REGADDR_ET, reg);
}

enum MA730_Error ma730_read_trimming(struct MA730_Driver* drv, bool* x, bool* y)
{
    if (NULL == drv) return MA730_NULL;
    if (NULL == x) return MA730_NULL;
    if (NULL == y) return MA730_NULL;

    uint8_t reg;
    enum MA730_Error err = read_reg(drv, MA730_REGADDR_ET, &reg);
    if (MA730_OK != err) return err;

    *x = ((reg & 0x01u) != 0u);
    *y = ((reg & 0x02u) != 0u);

    return MA730_OK;
}

//
// Pulses Per Turn and Index Length
//

enum MA730_Error ma730_write_pulses_and_index(struct MA730_Driver* drv, uint16_t pulses_per_turn, uint8_t ilip)
{
    if (NULL == drv) return MA730_NULL;
    if ((pulses_per_turn < 1u) || (pulses_per_turn > 1024u)) return MA730_BAD_RANGE;
    if (ilip > 0x0Fu) return MA730_BAD_RANGE;

    uint16_t ppt = (uint16_t) (pulses_per_turn - 1u);

    uint8_t reg4 = (uint8_t) (((ppt & 0x03u) << 6) | ((ilip & 0x0Fu) << 2));
    uint8_t reg5 = (uint8_t) ((ppt >> 2) & 0xFFu);

    enum MA730_Error err = write_reg(drv, MA730_REGADDR_PPT_ILIP, reg4);
    if (MA730_OK != err) return err;

    return write_reg(drv, MA730_REGADDR_PPT, reg5);
}

enum MA730_Error ma730_read_pulses_and_index(struct MA730_Driver* drv, uint16_t* pulses_per_turn, uint8_t* ilip)
{
    if (NULL == drv) return MA730_NULL;
    if (NULL == pulses_per_turn) return MA730_NULL;
    if (NULL == ilip) return MA730_NULL;

    uint8_t reg4;
    uint8_t reg5;

    enum MA730_Error err = read_reg(drv, MA730_REGADDR_PPT_ILIP, &reg4);
    if (MA730_OK != err) return err;

    err = read_reg(drv, MA730_REGADDR_PPT, &reg5);
    if (MA730_OK != err) return err;

    uint16_t ppt = ((uint16_t) reg5 << 2) | ((uint16_t) (reg4 >> 6) & 0x03u);

    *pulses_per_turn = (uint16_t) (ppt + 1u);
    *ilip = (uint8_t) ((reg4 >> 2) & 0x0Fu);

    return MA730_OK;
}

//
// Low and High Field Strength Thresholds
//

enum MA730_Error ma730_write_thresholds(struct MA730_Driver* drv, enum MA730_FieldThreshold_Low low, enum MA730_FieldThreshold_High high)
{
    if (NULL == drv) return MA730_NULL;
    if (!ma730_is_valid_field_threshold_low(low)) return MA730_BAD_RANGE;
    if (!ma730_is_valid_field_threshold_high(high)) return MA730_BAD_RANGE;

    uint8_t reg = (uint8_t) (((uint8_t) low) | ((uint8_t) high));
    return write_reg(drv, MA730_REGADDR_MGHT, reg);
}

enum MA730_Error ma730_read_thresholds(struct MA730_Driver* drv, enum MA730_FieldThreshold_Low* low, enum MA730_FieldThreshold_High* high)
{
    if (NULL == drv) return MA730_NULL;
    if (NULL == low) return MA730_NULL;
    if (NULL == high) return MA730_NULL;

    uint8_t reg;
    enum MA730_Error err = read_reg(drv, MA730_REGADDR_MGHT, &reg);
    if (MA730_OK != err) return err;

    *low = (enum MA730_FieldThreshold_Low) (reg & 0xE0u);
    *high = (enum MA730_FieldThreshold_High) (reg & 0x1Cu);

    return MA730_OK;
}

//
// Rotation Direction
//

enum MA730_Error ma730_write_direction(struct MA730_Driver* drv, bool ccw)
{
    if (NULL == drv) return MA730_NULL;

    uint8_t reg;
    enum MA730_Error err = read_reg(drv, MA730_REGADDR_RD, &reg);
    if (MA730_OK != err) return err;

    if (ccw) reg |= 0x80u;
    else reg &= (uint8_t) ~0x80u;

    return write_reg(drv, MA730_REGADDR_RD, reg);
}

enum MA730_Error ma730_read_direction(struct MA730_Driver* drv, bool* ccw)
{
    if (NULL == drv) return MA730_NULL;
    if (NULL == ccw) return MA730_NULL;

    uint8_t reg;
    enum MA730_Error err = read_reg(drv, MA730_REGADDR_RD, &reg);
    if (MA730_OK != err) return err;

    *ccw = ((reg & 0x80u) != 0u);

    return MA730_OK;
}