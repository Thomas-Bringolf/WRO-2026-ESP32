#include "encoder.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#define TAG "ENCODER_MODULE"

#define DEFAULT_PULSE_THRESHOLD 10

static int32_t encoder_accumulated_count = 0;


static bool IRAM_ATTR pcnt_threshold_isr(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    int count = 0;
    pcnt_unit_get_count(unit, &count);
    pcnt_unit_clear_count(unit);
    encoder_accumulated_count += count;
    return false; // no high priority task woken
}


esp_err_t encoder_init(Encoder *encoder)
{
    if (!encoder) return ESP_ERR_INVALID_ARG;

    esp_err_t err;

    // Create PCNT unit
    pcnt_unit_config_t unit_config = {
        .low_limit     = -1000,
        .high_limit    = 1000,
        .intr_priority = 1,
        .flags.accum_count = 1, // automatically accumulate on overflow
    };
    err = pcnt_new_unit(&unit_config, &encoder->pcnt_unit);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_new_unit failed: %s", esp_err_to_name(err));
        return err;
    }

    // Create PCNT channel
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num  = encoder->channelA_pin,
        .level_gpio_num = encoder->channelB_pin,
        .flags.invert_edge_input  = 0,
        .flags.invert_level_input = 0,
    };
    pcnt_channel_handle_t chan;
    err = pcnt_new_channel(encoder->pcnt_unit, &chan_config, &chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_new_channel failed: %s", esp_err_to_name(err));
        return err;
    }

    // Set channel actions
    err = pcnt_channel_set_edge_action(chan,
                                    PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                    PCNT_CHANNEL_EDGE_ACTION_DECREASE);

    err = pcnt_channel_set_level_action(chan,
                                        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);


    // 4Configure glitch filter
    pcnt_glitch_filter_config_t filter = { .max_glitch_ns = 1000 }; // 1 µs
    pcnt_unit_set_glitch_filter(encoder->pcnt_unit, &filter);

    // 5Register ISR callback
    pcnt_event_callbacks_t cbs = { .on_reach = pcnt_threshold_isr };
    pcnt_unit_register_event_callbacks(encoder->pcnt_unit, &cbs, encoder);

    // Enable unit and start counting
    pcnt_unit_enable(encoder->pcnt_unit);
    pcnt_unit_clear_count(encoder->pcnt_unit);
    pcnt_unit_start(encoder->pcnt_unit);

    // Initialize tracking variables
    encoder->accumulated_count      = 0;
    encoder->last_accumulated_count = 0;
    encoder->last_speed_calc_time   = esp_timer_get_time();

    ESP_LOGI(TAG, "Encoder initialized (A=%d, B=%d)", encoder->channelA_pin, encoder->channelB_pin);
    return ESP_OK;
}


esp_err_t encoder_reset(Encoder *encoder)
{
    if (!encoder) return ESP_ERR_INVALID_ARG;

    pcnt_unit_stop(encoder->pcnt_unit);
    encoder_accumulated_count = 0;
    pcnt_unit_clear_count(encoder->pcnt_unit);
    encoder->accumulated_count      = 0;
    encoder->last_accumulated_count = 0;
    encoder->last_speed_calc_time   = esp_timer_get_time();
    pcnt_unit_start(encoder->pcnt_unit);

    return ESP_OK;
}


int32_t encoder_read(Encoder *encoder)
{
    if (!encoder) return 0;

    int count = 0;
    pcnt_unit_get_count(encoder->pcnt_unit, &count);
    pcnt_unit_clear_count(encoder->pcnt_unit);
    encoder_accumulated_count += count;

    return encoder_accumulated_count;
}


float encoder_calc_speed_cyclic(Encoder *encoder)
{
    if (!encoder) return 0.0f;

    int32_t delta_counts, current_counts;
    int64_t time_now;
    float delta_time_s;

    do {
        current_counts = encoder_read(encoder);
        delta_counts = current_counts - encoder->last_accumulated_count;
        if (delta_counts < 10) vTaskDelay(pdMS_TO_TICKS(1));
    } while (delta_counts < 10);

    time_now     = esp_timer_get_time();
    delta_time_s = (time_now - encoder->last_speed_calc_time) / 1e6f;

    float cps = delta_counts / delta_time_s;

    encoder->last_accumulated_count = current_counts;
    encoder->last_speed_calc_time   = time_now;

    return cps;
}


float encoder_calc_speed_now(Encoder *encoder, int pulse_threshold)
{
    if (!encoder) return 0.0f;

    int32_t start_counts = encoder_read(encoder);
    int64_t start_time  = esp_timer_get_time();
    int32_t delta_counts, current_counts;

    do {
        current_counts = encoder_read(encoder);
        delta_counts = current_counts - start_counts;
        if (delta_counts < pulse_threshold) vTaskDelay(pdMS_TO_TICKS(1));
    } while (delta_counts < pulse_threshold);

    int64_t end_time = esp_timer_get_time();
    float cps = delta_counts / ((end_time - start_time) / 1e6f);

    return cps;
}


float encoder_calc_speed_now_(Encoder *encoder)
{
    return encoder_calc_speed_now(encoder, DEFAULT_PULSE_THRESHOLD);
}
