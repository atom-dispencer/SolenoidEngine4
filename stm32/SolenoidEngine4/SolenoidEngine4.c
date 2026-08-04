#include "main.h"
#include "stm32c092xx.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_adc.h"

#include <stdbool.h>
#include <assert.h>

#include "SolenoidEngine4.h"
#include "MA730.h"
#include "stm32c0xx_hal_adc_ex.h"

static enum MA730_Error resolver_spi_xmit(uint16_t* payload);
static enum MA730_Error resolver_spi_recv(uint16_t* payload);

static struct MA730_Driver RESOLVER = { 0 };
static volatile uint32_t THROTTLE_ADC_RAW = 0;

//
// SOLENOID CONTROL
//

void energise_solenoid(enum Solenoid s, bool energised)
{
    assert("Invalid Solenoid" && is_valid_solenoid(s));

    GPIO_TypeDef* port;
    uint16_t pin;

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
        //
        case SOLENOID_COUNT:
        default:
            assert("Bad SOLENOID enum" && false);
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
// GLOBAL STATE 
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
// RESOLVER SPI CONNECTIONS
//

static enum MA730_Error resolver_spi_xmit(uint16_t* payload)
{
    if (NULL == payload) return MA730_NULL;
    if (NULL == SE4.handles.h_resolver) return MA730_NULL;

    HAL_StatusTypeDef hal_err = HAL_SPI_Transmit(SE4.handles.h_resolver, (uint8_t*) payload, 1, HAL_MAX_DELAY);
    if (HAL_OK != hal_err) return MA730_BAD_RANGE;

    return MA730_OK;
}

static enum MA730_Error resolver_spi_recv(uint16_t* payload)
{
    if (NULL == payload) return MA730_NULL;
    if (NULL == SE4.handles.h_resolver) return MA730_NULL;

    uint16_t tx_word = 0;
    HAL_StatusTypeDef hal_err = HAL_SPI_TransmitReceive(SE4.handles.h_resolver, (uint8_t*) &tx_word, (uint8_t*) payload, 1, HAL_MAX_DELAY);
    if (HAL_OK != hal_err) return MA730_BAD_RANGE;

    return MA730_OK;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc != SE4.handles.h_eng_throttle) return;

    THROTTLE_ADC_RAW = HAL_ADC_GetValue(hadc);
}


//
// CONFIGURATION
//

void configure_engine()
{
    energise_solenoid(SOLENOID_1, false);
    energise_solenoid(SOLENOID_2, false);
    energise_solenoid(SOLENOID_3, false);
    energise_solenoid(SOLENOID_4, false);

    ma730_init(&RESOLVER, resolver_spi_xmit, resolver_spi_recv);

    HAL_ADCEx_Calibration_Start(SE4.handles.h_eng_throttle);
    HAL_Delay(1);
    HAL_ADC_Start_IT(SE4.handles.h_eng_throttle);
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

        for (int j = 0; j < repeats; j++)
        {
            float cal = 0;
            //
            // TODO read calibration value from resolver
            //
            SE4.calibration[i] += cal;
        }

        HAL_Delay(500);
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
    // TODO read crank angle from resolver
    // TODO energise solenoids based on crank angle 

    // TODO read throttle fraction from ADC
    // TODO set PWM duty cycle based on throttle fraction
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