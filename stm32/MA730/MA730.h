/**
 * Driver for a Monolithic Power Systems MA730 magnetic resolver
 *
 * Created by Adam Spencer @ 4 Aug 2026
 */

#ifndef GUARD_MA730
#define GUARD_MA730

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

struct MA730_Driver
{
    /**
     * Pointer to a function which synchronously transmits a single 16-bit
     * payload (register address + value) to the given MA730 chip.
     *
     * The bits should be put on the wire in order of descending significance:
     * 1) The most significant byte, leading with its most significant bit
     * 2) The least significant byte, leading with its most significant bit
     */
    enum MA730_Error (*spi_xmit)(uint16_t* payload);
    /**
     * Pointer to a function which synchronously recieves a single 16-bit
     * payload (2 bytes) from the given MA730 chip.
     *
     * The bytes/bits are recieved in the same order as `spi_xmit(...)` sends
     * them.
     */
    enum MA730_Error (*spi_recv)(uint16_t* payload);
};

enum MA730_Error
{
    MA730_OK,
    MA730_NULL,
    MA730_BAD_ADDR,
    MA730_BAD_RANGE
};

//
// Register Addresses
//

enum MA730_RegAddr
{
    MA730_REGADDR_Z_MSB     = 0x00,
    MA730_REGADDR_Z_LSB     = 0x01,
    MA730_REGADDR_BCT       = 0x02,
    MA730_REGADDR_ET        = 0x03,
    MA730_REGADDR_PPT_ILIP  = 0x04,
    MA730_REGADDR_PPT       = 0x05,
    MA730_REGADDR_MGHT      = 0x06,
    MA730_REGADDR_RD        = 0x09,
    MA730_REGADDR_MG        = 0x1B,
};

static const bool ma730_is_valid_regaddr(enum MA730_RegAddr addr)
{
    switch(addr)
    {
        case MA730_REGADDR_Z_MSB:
        case MA730_REGADDR_Z_LSB:
        case MA730_REGADDR_BCT:
        case MA730_REGADDR_ET:
        case MA730_REGADDR_PPT_ILIP:
        case MA730_REGADDR_PPT:
        case MA730_REGADDR_MGHT:
        case MA730_REGADDR_RD:
        case MA730_REGADDR_MG:
            return true;
        default:
            return false;
    }
}

//
// Field Thresholds (LOW)
//

enum MA730_FieldThreshold_Low
{
    MA730_FIELDTHRESHOLD_LOW_26  = 0b000 << 5,
    MA730_FIELDTHRESHOLD_LOW_41  = 0b001 << 5,
    MA730_FIELDTHRESHOLD_LOW_56  = 0b010 << 5,
    MA730_FIELDTHRESHOLD_LOW_70  = 0b011 << 5,
    MA730_FIELDTHRESHOLD_LOW_84  = 0b100 << 5,
    MA730_FIELDTHRESHOLD_LOW_98  = 0b101 << 5,
    MA730_FIELDTHRESHOLD_LOW_112 = 0b110 << 5,
    MA730_FIELDTHRESHOLD_LOW_126 = 0b111 << 5
};

static const bool ma730_is_valid_field_threshold_low(enum MA730_FieldThreshold_Low low)
{
    switch(low)
    {
        case MA730_FIELDTHRESHOLD_LOW_26:
        case MA730_FIELDTHRESHOLD_LOW_41:
        case MA730_FIELDTHRESHOLD_LOW_56:
        case MA730_FIELDTHRESHOLD_LOW_70:
        case MA730_FIELDTHRESHOLD_LOW_84:
        case MA730_FIELDTHRESHOLD_LOW_98:
        case MA730_FIELDTHRESHOLD_LOW_112:
        case MA730_FIELDTHRESHOLD_LOW_126:
            return true;
        default:
            return false;
    }
}

//
// Field Thresholds (HIGH)
//

enum MA730_FieldThreshold_High
{
    MA730_FIELDTHRESHOLD_HIGH_20  = 0b000 << 2,
    MA730_FIELDTHRESHOLD_HIGH_35  = 0b001 << 2,
    MA730_FIELDTHRESHOLD_HIGH_50  = 0b010 << 2,
    MA730_FIELDTHRESHOLD_HIGH_64  = 0b011 << 2,
    MA730_FIELDTHRESHOLD_HIGH_78  = 0b100 << 2,
    MA730_FIELDTHRESHOLD_HIGH_92  = 0b101 << 2,
    MA730_FIELDTHRESHOLD_HIGH_106 = 0b110 << 2,
    MA730_FIELDTHRESHOLD_HIGH_120 = 0b111 << 2
};

static const bool ma730_is_valid_field_threshold_high(enum MA730_FieldThreshold_High high)
{
    switch(high)
    {
        case MA730_FIELDTHRESHOLD_HIGH_20:
        case MA730_FIELDTHRESHOLD_HIGH_35:
        case MA730_FIELDTHRESHOLD_HIGH_50:
        case MA730_FIELDTHRESHOLD_HIGH_64:
        case MA730_FIELDTHRESHOLD_HIGH_78:
        case MA730_FIELDTHRESHOLD_HIGH_92:
        case MA730_FIELDTHRESHOLD_HIGH_106:
        case MA730_FIELDTHRESHOLD_HIGH_120:
            return true;
        default:
            return false;
    }
}

//
// Functions
//

enum MA730_Error ma730_init(struct MA730_Driver* blank_driver, 
    enum MA730_Error (*spi_xmit)(uint16_t* payload),
    enum MA730_Error (*spi_recv)(uint16_t* payload)
);

enum MA730_Error ma730_write_zero_deg(struct MA730_Driver* drv, float zero_deg);
enum MA730_Error ma730_read_zero_deg(struct MA730_Driver* drv, float* zero_deg);

enum MA730_Error ma730_write_bias_current(struct MA730_Driver* drv, float bias);
enum MA730_Error ma730_read_bias_current(struct MA730_Driver* drv, float* bias);

enum MA730_Error ma730_write_trimming(struct MA730_Driver* drv, bool x, bool y);
enum MA730_Error ma730_read_trimming(struct MA730_Driver* drv, bool* x, bool* y);

enum MA730_Error ma730_write_pulses_and_index(struct MA730_Driver* drv, uint16_t pulses_per_turn, uint8_t ilip);
enum MA730_Error ma730_read_pulses_and_index(struct MA730_Driver* drv, uint16_t* pulses_per_turn, uint8_t* ilip);

enum MA730_Error ma730_write_thresholds(struct MA730_Driver* drv, enum MA730_FieldThreshold_Low low, enum MA730_FieldThreshold_High high);
enum MA730_Error ma730_read_thresholds(struct MA730_Driver* drv, enum MA730_FieldThreshold_Low* low, enum MA730_FieldThreshold_High* high);

enum MA730_Error ma730_write_direction(struct MA730_Driver* drv, bool ccw);
enum MA730_Error ma730_read_direction(struct MA730_Driver* drv, bool* ccw);

#endif