/**
 * Driver for a Monolithic Power Systems MA730 magnetic resolver
 *
 * Created by Adam Spencer @ 4 Aug 2026
 */

#ifndef GUARD_MA730
#define GUARD_MA730

struct MA730_Driver
{
    /**
     * Pointer to a function which synchronously transmits a single 16-bit
     * payload to the given MA730 chip.
     */
    enum MA730_Error (*spi_xmit)(uint16_t* payload);
    /**
     * Pointer to a function which synchronously recieves a single 16-bit
     * payload from the given MA730 chip.
     */
    enum MA730_Error (*spi_recv)(uint16_t* payload);
};

enum MA730_Error
{

};



#endif