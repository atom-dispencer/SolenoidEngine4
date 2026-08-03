#include "main.h"

#include <stdbool.h>
#include <assert.h>

//
// Solenoid Control
//

enum Solenoid
{
    SOLENOID_1,
    SOLENOID_2,
    SOLENOID_3,
    SOLENOID_4,
    //
    SOLENOID_COUNT
}:

const enum Solenoid SOLENOIDS[SOLENOID_COUNT] =
{
    SOLENOID_1,
    SOLENOID_2,
    SOLENOID_3,
    SOLENOID_4
};

void is_valid_solenoid(enum Solenoid s)
{
    switch(s)
    {
        case SOLENOID_1:
        case SOLENOID_2:
        case SOLENOID_3:
        case SOLENOID_4:
            return true;
        default:
            return false;
    }
}

void energise_solenoid(enum Solenoid s, bool energised)
{
    assert("Invalid Solenoid" && is_valid_solenoid(s));

    int port;
    int pin;

    switch(s)
    {
        case SOLENOID_1:
            port = SOL_EN_1_GPIO_Port;
            pin = SOL_EN_1_Pin;
            break;
        case SOLENOID_2:
            port = SOL_EN_2_GPIO_Port;
            pin = SOL_EN_2_Pin;
            break;
        case SOLENOID_3:
            port = SOL_EN_3_GPIO_Port;
            pin = SOL_EN_3_Pin;
            break;
        case SOLENOID_4:
            port = SOL_EN_4_GPIO_Port;
            pin = SOL_EN_4_Pin;
            break;
    }

    GPIO_PinState state = energised ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(port, pin, state);
}

void energise_all_solenoids(bool energised)
{
    for (int i = 0; i < SOLENOID_COUNT; i++)
    {
        enum Solenoid s = SOLENOIDS[i];
        energise_solenoid(s, energised);
    }
}

//
// Global state
//

struct Handles
{
    ADC_HandleTypeDef *h_eng_throttle;
    SPI_HandleTypeDef *h_resolver;
};

struct SolenoidEngine4
{
    struct Handles handles;

    float crank_radians;
    float throttle_fraction;

    float calibration[SOLENOID_COUNT];
};

struct SolenoidEngine4 SE4 = { 0 };


//
// CONFIGURATION
//

void configure_engine()
{
    energise_solenoid(SOLENOID_1, false);
    energise_solenoid(SOLENOID_2, false);
    energise_solenoid(SOLENOID_3, false);
    energise_solenoid(SOLENOID_4, false);

    HAL_ADC_StartCalibration(SE4.handles.h_eng_throttle);

    HAL_Delay(1);
}

//
// CALIBRATION
//

void calibrate_engine()
{
    unsigned int revolutions = 2;
    unsigned int repeats = 50;

    //
    // Collect & sum the calibration samples
    //

    for (int i = 0; i < revolutions * SOLENOID_COUNT; i++)
    {
        energise_all_solenoids(false);

        enum Solenoid s = SOLENOIDS[i % SOLENOID_COUNT];
        energise_solenoid(s, true);

        //
        // TODO read calibration value from resolver
        //
    }

    //
    // Average the calibration samples 
    //

    unsigned int samples = revolutions * repeats;
    for (int i = 0; i < SOLENOID_COUNT; i++)
    {
        SE4.calibration[i] /= samples;
    }
}

//
// TICK
//

void tick_engine()
{

}

//
// MAIN
//

void main_SolenoidEngine4(
    ADC_HandleTypeDef *h_eng_throttle,
    SPI_HandleTypeDef *h_resolver
)
{
    SE4.handles.h_eng_throttle = h_eng_throttle;
    SE4.handles.h_resolver = h_resolver;

    configure_engine();

    calibrate_engine();

    while(1)
    {
        tick_engine();
    }
}