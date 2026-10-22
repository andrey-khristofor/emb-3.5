#include <cstdio>
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "servo";

constexpr adc_unit_t ADC_UNIT = ADC_UNIT_1;
constexpr adc_channel_t ADC_CHANNEL = ADC_CHANNEL_9;

constexpr gpio_num_t SERVO_GPIO = GPIO_NUM_5;
constexpr ledc_timer_bit_t SERVO_PWM_RESOLUTION = LEDC_TIMER_12_BIT;
constexpr ledc_timer_t SERVO_TIMER = LEDC_TIMER_0;
constexpr ledc_channel_t SERVO_CHANNEL = LEDC_CHANNEL_0;
constexpr uint32_t SERVO_FREQ = 50;

constexpr int MIN_SERVO_ANGLE = 0;
constexpr int MAX_SERVO_ANGLE = 180;

constexpr int MIN_PULSE_US = 500;  // MIN_SERVO_ANGLE
constexpr int MAX_PULSE_US = 2500; // MAX_SERVO_ANGLE
constexpr int PERIOD_US = 20000;   // 50 Гц = 20 мс період

constexpr int MIN_POT_RAW = 0;    // MIN_SERVO_ANGLE на потенціометрі
constexpr int MAX_POT_RAW = 2450; // MAX_SERVO_ANGLE на потенціометрі

adc_oneshot_unit_handle_t configure_adc();
void configure_servo_gpio_with_pwm();
void set_servo_angle(int);
int get_angle_by_pot_value(int);

extern "C" void app_main(void)
{
    adc_oneshot_unit_handle_t adc = configure_adc();
    configure_servo_gpio_with_pwm();

    while (1)
    {
        int raw = 0;
        adc_oneshot_read(adc, ADC_CHANNEL, &raw);

        int angle = get_angle_by_pot_value(raw);

        ESP_LOGI(TAG, "Кут від лівого положення: %i°", MAX_SERVO_ANGLE - angle);

        set_servo_angle(angle);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

adc_oneshot_unit_handle_t configure_adc()
{
    adc_oneshot_unit_handle_t adc_handle;

    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_new_unit(&unit_config, &adc_handle);

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &channel_config);

    return adc_handle;
}

void configure_servo_gpio_with_pwm()
{
    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_config.duty_resolution = SERVO_PWM_RESOLUTION;
    timer_config.timer_num = SERVO_TIMER;
    timer_config.freq_hz = SERVO_FREQ;
    timer_config.clk_cfg = LEDC_AUTO_CLK;

    ledc_timer_config(&timer_config);

    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = SERVO_GPIO;
    channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_config.channel = SERVO_CHANNEL;
    channel_config.timer_sel = SERVO_TIMER;
    channel_config.duty = 0;
    channel_config.hpoint = 0;

    ledc_channel_config(&channel_config);
}

int get_angle_by_pot_value(int raw)
{
    raw = raw < MIN_POT_RAW ? MIN_POT_RAW : raw;
    raw = raw > MAX_POT_RAW ? MAX_POT_RAW : raw;

    return MIN_SERVO_ANGLE + (raw - MIN_POT_RAW) * (MAX_SERVO_ANGLE - MIN_SERVO_ANGLE) / (MAX_POT_RAW - MIN_POT_RAW);
}

void set_servo_angle(int angle)
{
    angle = angle < MIN_SERVO_ANGLE ? MIN_SERVO_ANGLE : angle;
    angle = angle > MAX_SERVO_ANGLE ? MAX_SERVO_ANGLE : angle;

    float pulse_us = MIN_PULSE_US + ((angle - MIN_SERVO_ANGLE) * 1.0f / (MAX_SERVO_ANGLE - MIN_SERVO_ANGLE)) * (MAX_PULSE_US - MIN_PULSE_US);

    uint32_t duty = (uint32_t)((pulse_us / PERIOD_US) * (1 << SERVO_PWM_RESOLUTION));

    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);
}