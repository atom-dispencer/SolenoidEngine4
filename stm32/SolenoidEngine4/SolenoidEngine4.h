#ifndef GUARD_SOLENOIDENGINE4
#define GUARD_SOLENOIDENGINE4

struct Handles
{
    ADC_HandleTypeDef *h_eng_throttle;
    SPI_HandleTypeDef *h_resolver;
    TIM_HandleTypeDef *h_pwm;
    TIM_HandleTypeDef *h_1ms;
};

struct SolenoidEngine4
{
    struct Handles handles;

    float crank_radians;
    float throttle_fraction;

    float calibration[SOLENOID_COUNT];
};

static struct Solenoid 
{
    float expected_calibration;
    GPIO_TypeDef* gpio_port;
    uint16_t gpio_pin;
};

const static struct SOLENOIDS[] = 
{
    {
        .gpio_port = SOL_EN_1_GPIO_Port,
        .gpio_pin = SOL_EN_1_Pin
    },
    {
        .gpio_port = SOL_EN_2_GPIO_Port,
        .gpio_pin = SOL_EN_2_Pin
    },
    {
        .gpio_port = SOL_EN_3_GPIO_Port,
        .gpio_pin = SOL_EN_3_Pin
    },
    {
        .gpio_port = SOL_EN_4_GPIO_Port,
        .gpio_pin = SOL_EN_4_Pin
    }
};

enum Spin
{
    SPIN_POSITIVE,
    SPIN_NONE,
    SPIN_NEGATIVE
};

#define SOLENOID_COUNT (sizeof(SOLENOIDS) / sizeof(struct Solenoid))
#define IS_VALID_SOLENOID_INDEX(idx) ( (unsigned int)(idx) < SOLENOID_COUNT )

#endif