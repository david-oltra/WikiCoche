#include <string.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <driver/adc.h>

#ifdef MANDO
    #define JOYSTICK_SW GPIO_NUM_0

#else
    #include <driver/ledc.h>

    #define TB6612_PWMA GPIO_NUM_32
    #define TB6612_AIN1 GPIO_NUM_25
    #define TB6612_AIN2 GPIO_NUM_33
    #define TB6612_PWMB GPIO_NUM_12
    #define TB6612_BIN1 GPIO_NUM_27
    #define TB6612_BIN2 GPIO_NUM_14
    #define TB6612_STBY GPIO_NUM_26
#endif

#define ESP_CHANNEL 1
static uint8_t peer_mac [ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
//static uint8_t peer_mac [ESP_NOW_ETH_ALEN] = {0xc8, 0x2e, 0x18, 0xf1, 0x83, 0x40};
//static uint8_t peer_mac [ESP_NOW_ETH_ALEN] = {0xa0, 0xa3, 0xb3, 0x2a, 0xb5, 0xa4};

typedef struct struct_message {
    uint16_t eje_x;
    uint16_t eje_y;
    bool sw;
} struct_message;
struct_message remote_data;

extern "C" void app_main();

#ifdef MANDO
static const char * TAG = "ESP_REMOTE";
#else
static const char * TAG = "MAIN";
#endif 

static esp_err_t init_wifi(void)
{
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    
    esp_netif_init();
    esp_event_loop_create_default();
    nvs_flash_init();
    esp_wifi_init(&wifi_init_config);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    esp_wifi_start();

    ESP_LOGI(TAG, "wifi init completed");
    return ESP_OK;
}

void recv_cb(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len)
{
    //ESP_LOGI(TAG, "Data received " MACSTR "%s", MAC2STR(esp_now_info->src_addr), data);

    memcpy(&remote_data, data, sizeof(remote_data));
}

void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if(status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(TAG, "ESP_NOW_SEND_SUCCESS");
    }
    else
    {
        ESP_LOGW(TAG, "ESP_NOW_SEND_FAIL");
    }
}

static esp_err_t init_esp_now(void)
{
    esp_now_init();
    esp_now_register_recv_cb(recv_cb);
    esp_now_register_send_cb(send_cb);

    ESP_LOGI(TAG, "esp now init completed");
    return ESP_OK;
}

static esp_err_t register_peer(uint8_t *peer_addr)
{
    esp_now_peer_info_t esp_now_peer_info;
    memset(&esp_now_peer_info, 0, sizeof(esp_now_peer_info_t));
    esp_now_peer_info.channel = ESP_CHANNEL;
    esp_now_peer_info.ifidx = WIFI_IF_STA;
    esp_now_peer_info.encrypt = false;
    memcpy(esp_now_peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    esp_now_add_peer(&esp_now_peer_info);

    return ESP_OK;
}

static esp_err_t esp_now_send_data(const uint8_t *peer_addr, const uint8_t *data, size_t len)
{
    esp_now_send(peer_addr, data, len);

    return ESP_OK;
}

esp_err_t init_adc(void)
{
    adc1_config_channel_atten(ADC1_CHANNEL_1,ADC_ATTEN_DB_11); //eje Y
    adc1_config_channel_atten(ADC1_CHANNEL_2,ADC_ATTEN_DB_11); //eje X
    adc1_config_width(ADC_WIDTH_BIT_12);

    ESP_LOGI(TAG,"ADC init completed");
    return ESP_OK;
}

#ifdef MANDO
    esp_err_t init_gpios(void)
    {
        gpio_config_t in_conf = {};
        in_conf.intr_type = GPIO_INTR_DISABLE;
        in_conf.mode = GPIO_MODE_INPUT;
        in_conf.pin_bit_mask = (1ULL << JOYSTICK_SW);
        in_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        in_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&in_conf);

        ESP_LOGI(TAG,"gpios init completed");
        return ESP_OK;
    }
#else
    esp_err_t init_gpios(void)
    {
        gpio_config_t out_conf = {};
        out_conf.intr_type = GPIO_INTR_DISABLE;
        out_conf.mode = GPIO_MODE_OUTPUT;
        out_conf.pin_bit_mask = 1ULL << (TB6612_AIN1) | (1ULL << TB6612_AIN2) | (1ULL << TB6612_BIN1) | (1ULL << TB6612_BIN2) | (1ULL << TB6612_STBY);
        out_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        out_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&out_conf);

        gpio_set_level(TB6612_AIN1, 0);
        gpio_set_level(TB6612_AIN2, 0);
        gpio_set_level(TB6612_BIN1, 0);
        gpio_set_level(TB6612_BIN2, 0);
        gpio_set_level(TB6612_STBY, 1);

        ESP_LOGI(TAG,"gpios init completed");
        return ESP_OK;
    }
#endif

#ifndef MANDO
esp_err_t init_pwm(void)
{
    ledc_timer_config_t ledc_r_timer;
    ledc_r_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_r_timer.duty_resolution  = LEDC_TIMER_12_BIT;
    ledc_r_timer.timer_num        = LEDC_TIMER_0;
    ledc_r_timer.freq_hz          = 4000;
    ledc_r_timer.clk_cfg          = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_r_timer));

    ledc_channel_config_t ledc_r_channel;
    ledc_r_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_r_channel.channel        = LEDC_CHANNEL_0;
    ledc_r_channel.timer_sel      = LEDC_TIMER_0;
    ledc_r_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_r_channel.gpio_num       = TB6612_PWMA;
    ledc_r_channel.duty           = 0; // 0 -> 2^13 -1 = 8192 - 1
    ledc_r_channel.hpoint         = 0;
    ledc_r_channel.flags.output_invert  = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_r_channel));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

    ledc_timer_config_t ledc_l_timer;
    ledc_l_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_l_timer.duty_resolution  = LEDC_TIMER_12_BIT;
    ledc_l_timer.timer_num        = LEDC_TIMER_1;
    ledc_l_timer.freq_hz          = 4000;
    ledc_l_timer.clk_cfg          = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_l_timer));

    ledc_channel_config_t ledc_l_channel;
    ledc_l_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_l_channel.channel        = LEDC_CHANNEL_1;
    ledc_l_channel.timer_sel      = LEDC_TIMER_1;
    ledc_l_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_l_channel.gpio_num       = TB6612_PWMB;
    ledc_l_channel.duty           = 0; // 0 -> 2^13 -1 = 8192 - 1
    ledc_l_channel.hpoint         = 0;
    ledc_l_channel.flags.output_invert  = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_l_channel));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));

    ESP_LOGI(TAG,"pwm init completed");
    return ESP_OK;
}
#endif

void app_main()
{
    ESP_ERROR_CHECK(init_wifi());
    ESP_ERROR_CHECK(init_esp_now());
    ESP_ERROR_CHECK(init_gpios());

    #ifdef MANDO
        ESP_ERROR_CHECK(init_adc());
        
        ESP_ERROR_CHECK(register_peer(peer_mac));

        while (1)
        {
            remote_data.eje_x = adc1_get_raw(ADC1_CHANNEL_2);
            remote_data.eje_y = adc1_get_raw(ADC1_CHANNEL_1);
            remote_data.sw = gpio_get_level(JOYSTICK_SW);
            printf("eje X: %u\teje Y: %u\t sw: %u\n", remote_data.eje_x, remote_data.eje_y, remote_data.sw);
            esp_now_send_data(peer_mac, (uint8_t *) &remote_data, sizeof(remote_data));
            vTaskDelay(pdMS_TO_TICKS(100));
        }

    #else
        init_pwm();

        int32_t in_med_x = 0;
        int32_t in_med_y = 0;
        int16_t map_x = 0; //(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        int16_t map_y = 0;
        int16_t pwm_r;
        int16_t pwm_l;

        while (!remote_data.sw)
        {
            ESP_LOGI(TAG,"Waiting for signal");
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        for (int x = 0; x<50; x++)
        {
            in_med_x += remote_data.eje_x;
            in_med_y += remote_data.eje_y;
            ESP_LOGW(TAG,"in_med_x = %lu\tin_med_y = %lu",in_med_x, in_med_y);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        in_med_x = in_med_x / 50;
        in_med_y = in_med_y / 50;
        ESP_LOGE(TAG,"in_med_x = %lu\tin_med_y = %lu",in_med_x, in_med_y);

        while (1)
        {
            if (remote_data.eje_x > (in_med_x + 10))
            {
                //MOVE FRONT
                map_x = (remote_data.eje_x - (in_med_x + 10)) * (4095 - 0) / (4095 - (in_med_x + 10)) + 0; //(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
                gpio_set_level(TB6612_AIN1,1);
                gpio_set_level(TB6612_AIN2,0);
                gpio_set_level(TB6612_BIN1,1);
                gpio_set_level(TB6612_BIN2,0);
            }
            else if (remote_data.eje_x < (in_med_x - 10))
            {
                //MOVE BACK
                map_x = (remote_data.eje_x - 0) * (4095 - 0) / ((in_med_x - 10) - 0) + 0; //(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
                map_x = 4095 - map_x;
                gpio_set_level(TB6612_AIN1,0);
                gpio_set_level(TB6612_AIN2,1);
                gpio_set_level(TB6612_BIN1,0);
                gpio_set_level(TB6612_BIN2,1);
            }
            else
            {
                //STOP
                map_x = 0;
            }

            map_y = 0;
            pwm_r = map_x;
            pwm_l = map_x;
            if (remote_data.eje_y > (in_med_y + 10))
            {
                //MOVE RIGHT (LEFT 100% - RIGHT map_y)
                map_y = (remote_data.eje_y - (in_med_y + 10)) * (100 - 0) / (4095 - (in_med_y + 10)) + 0; //(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
                pwm_l = map_x;
                if (map_y == 100)
                {
                    pwm_r = 0;
                }
                else 
                {
                    pwm_r = map_x - (map_x*map_y/100);
                }
                
            }
            else if (remote_data.eje_y < (in_med_y - 10))
            {
                //MOVE LEFT (RIGHT 100% - LEFT map_y)
                map_y = (remote_data.eje_y - 0) * (100 - 0) / ((in_med_y - 10) - 0) + 0; //(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
                map_y = 100 - map_y;
                pwm_r = map_x;
                if (map_y == 100)
                {
                    pwm_l = 0;
                }
                else 
                {
                    pwm_l = map_x - (map_x*map_y/100);
                }
            }
            else
            {
                //100-100
 
            }

            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_l));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, pwm_r));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
            
            //ESP_LOGI(TAG, "Eje X: %u\tEje Y: %u\tSW: %u", remote_data.eje_x, remote_data.eje_y, remote_data.sw);
            //ESP_LOGI(TAG, "MAP X = %u\tMAP Y = %u", map_x, map_y);
            //ESP_LOGI(TAG, "MAP X = %u\tMAP Y = %u\tPWM L = %u\tPWM R= %u", map_x, map_y, pwm_l, pwm_r);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

    #endif
}