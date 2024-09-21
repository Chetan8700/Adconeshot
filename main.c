#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
static const char *TAG = "ADC_ONE_SHOT";

void app_main(void)
{
    // Configure ADC one-shot mode
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1,  // Use ADC1 (can be ADC_UNIT_1 or ADC_UNIT_2)
    };
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_new_unit(&adc_config, &adc_handle);

    // Configure ADC channel
    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,  // Default bit width (12 bits)
        .atten =  ADC_ATTEN_DB_12 ,           // 0 dB attenuation (full-scale voltage ~1.1V)
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &channel_config);  // Configure ADC Channel 0 (GPIO36)

    while (1) {
        int adc_reading = 0;
        // Read value from the ADC
        adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &adc_reading);
        
        // Log the reading
        ESP_LOGI(TAG, "ADC Reading: %d", adc_reading);

        // Delay to avoid continuous fast readings (1 second delay)
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Clean up (not usually needed in the main loop)
    adc_oneshot_del_unit(adc_handle);
}
