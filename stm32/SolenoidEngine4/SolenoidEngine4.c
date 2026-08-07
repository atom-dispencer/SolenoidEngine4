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

void energise_solenoid(struct Solenoid* sol, bool energised)
{
    assert("NULL Solenoid" && (NULL != sol));

    GPIO_PinState state = energised ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(sol.gpio_port, sol.gpio_pin, state);
}

void energise_all_solenoids(bool energised)
{
    for (int i = 0; i < SOLENOID_COUNT; i++)
    {
        struct Solenoid* s = &SOLENOIDS[i];
        energise_solenoid(s, energised);
    }
}

//
// GLOBAL STATE
//

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

    HAL_TIM_PWM_Start(SE4.handles.h_pwm, TIM_CHANNEL_1);
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
            float cal = 0.0f;
            ma730_read_angle(&RESOLVER, &cal);
            SE4.calibration[i] += cal;
        }

        HAL_Delay(500);
    }

    //
    // Average the calibration samples and calculate the point opposite to the
    // solenoid so we know when to start energising it
    //

    unsigned int samples = revolutions * repeats;
    for (int i = 0; i < SOLENOID_COUNT; i++)
    {
        SE4.calibration[i] / samples;
    }
}

//
// PWM
//

static void set_sol_pwm_duty_cycle(float fraction)
{
    if (NULL == SE4.handles.h_pwm) return;

    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;

    uint32_t period  = SE4.handles.h_pwm->Init.Period;
    uint32_t compare = (uint32_t)(fraction * (float)(period + 1));
    if (compare > period) compare = period;

    __HAL_TIM_SET_COMPARE(SE4.handles.h_pwm, TIM_CHANNEL_1, compare);
}

//
// TICK
//

#define TIMESTEP_MS (1)
#define D_DEG_WINDOW (50)
#define D_DEG_THRESHOLD (10)

#define D_DEG_LEAD (10)

void tick_engine()
{
    //
    // Read resolver angle and calculate speed
    //

    float deg = 0.0f;
    ma730_read_angle(&RESOLVER, &deg);

    static float deg_measurements[D_DEG_WINDOW] = { 0 };
    static int deg_measurement_index = 0;
    deg_measurements[deg_measurement_index] = deg;
    deg_measurement_index = (deg_measurement_index + 1) % D_DEG_WINDOW;

    float newest = deg_measurements[(deg_measurement_index + D_DEG_WINDOW - 1) % D_DEG_WINDOW];
    float oldest = deg_measurements[deg_measurement_index];
    float d_deg_raw = newest - oldest;
    if (d_deg_raw >  180.0f) d_deg_raw -= 360.0f;
    if (d_deg_raw < -180.0f) d_deg_raw += 360.0f;
    float d_deg_per_tick = d_deg_raw / (D_DEG_WINDOW - 1);                          // deg/tick
    float d_deg_per_sec  = d_deg_per_tick * (1000.0f / TIMESTEP_MS);                // deg/s

    //
    // Predict where the crank will be next tick
    //

    float deg_predicted = deg + d_deg_per_tick;
    if (deg_predicted >= 360.0f) deg_predicted -= 360.0f;
    if (deg_predicted <    0.0f) deg_predicted += 360.0f;

    //
    // Calculate spin direction
    //

    enum Spin spin;
    if (d_deg_per_sec > D_DEG_THRESHOLD) spin = SPIN_POSITIVE;
    else if (d_deg_per_sec < -D_DEG_THRESHOLD) spin = SPIN_NEGATIVE;
    else spin = SPIN_NONE;

    //
    // Calculate solenoid energisation
    //
    // We use the *predicted* angle so that solenoid energisation leads reality
    // by one tick
    //

    for (int i = 0; i < SOLENOID_COUNT; i++)
    {
        assert("Invalid Solenoid Index" && IS_VALID_SOLENOID_INDEX(i));

        struct Solenoid* s = &SOLENOIDS[i];

        // Predicted angle relative to this solenoid's closest point (calibration = TDC),
        // normalised to [-180, 180].
        float rel = deg_predicted - SE4.calibration[i];
        if (rel >  180.0f) rel -= 360.0f;
        if (rel < -180.0f) rel += 360.0f;

        // Energise only when the conrod is approaching and pull is effective.
        // SPIN_POSITIVE: conrod approaches through rel [-180, 0]; window = [-(180-lead), -lead]
        // SPIN_NEGATIVE: conrod approaches through rel [+180, 0]; window = [+lead, +(180-lead)]
        bool energise = false;
        if (spin == SPIN_POSITIVE)
            energise = (rel > -(180.0f - D_DEG_LEAD)) && (rel < -(float)D_DEG_LEAD);
        else if (spin == SPIN_NEGATIVE)
            energise = (rel > (float)D_DEG_LEAD) && (rel < (180.0f - D_DEG_LEAD));

        energise_solenoid(s, energise);
    }

    //
    // If the SPIN isn't NONE, apply power proportional to the throttle ADC
    //

    if (spin == SPIN_NONE)
    {
        set_sol_pwm_duty_cycle(0.0f);
    }
    else
    {
        float fraction = (float)THROTTLE_ADC_RAW / 4095.0f;
        fraction = (fraction - 0.05f) / 0.90f;
        if (fraction < 0.0f) fraction = 0.0f;
        if (fraction > 1.0f) fraction = 1.0f;
        SE4.throttle_fraction = fraction;
        set_sol_pwm_duty_cycle(SE4.throttle_fraction);
    }
}

//
// MAIN
//

void main_SolenoidEngine4(
  ADC_HandleTypeDef *h_eng_throttle,
  SPI_HandleTypeDef *h_resolver,
  TIM_HandleTypeDef *h_pwm,
  TIM_HandleTypeDef *h_1ms
)
{
    SE4.handles.h_eng_throttle = h_eng_throttle;
    SE4.handles.h_resolver = h_resolver;
    SE4.handles.h_pwm = h_pwm;
    SE4.handles.h_1ms = h_1ms;

    configure_engine();

    calibrate_engine();

    while(1)
    {
        tick_engine();
    }
}