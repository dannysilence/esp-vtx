//#include <Arduino.h>
#include <algorithm>

#include "esp_camera.h"
//#include "EEPROM.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include "esp_wifi_types.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
//#include "esp_wifi_internal.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "esp_private/wifi.h"
#include "esp_task_wdt.h"
//#include "bt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "sodium.h"
#include "esp_wb.hpp"
#include "fec.h"

#include "driver/gpio.h"
#include "main.h"
#include "queue.h"
#include "packets.h"
#include "safe_printf.h"
#include "structures.h"
#include "crc.h"
#include "circular_buffer.h"


Transmitter transmitter(8,12,0);


//#define WIFI_AP

#if defined WIFI_AP
    #define ESP_WIFI_MODE WIFI_MODE_AP
    #define ESP_WIFI_IF WIFI_IF_AP
#else
    #define ESP_WIFI_MODE WIFI_MODE_STA
    #define ESP_WIFI_IF WIFI_IF_STA
#endif



/*Select Camera Model*/
#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    21
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      19
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM       5
#define Y2_GPIO_NUM       4
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

#elif defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23

#define Y9_GPIO_NUM      36
#define Y8_GPIO_NUM      37
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      39
#define Y5_GPIO_NUM      35
#define Y4_GPIO_NUM      14
#define Y3_GPIO_NUM      13
#define Y2_GPIO_NUM      34
#define VSYNC_GPIO_NUM   5
#define HREF_GPIO_NUM    27
#define PCLK_GPIO_NUM    25

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    15
#define XCLK_GPIO_NUM     27
#define SIOD_GPIO_NUM     25
#define SIOC_GPIO_NUM     23

#define Y9_GPIO_NUM       19
#define Y8_GPIO_NUM       36
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       39
#define Y5_GPIO_NUM        5
#define Y4_GPIO_NUM       34
#define Y3_GPIO_NUM       35
#define Y2_GPIO_NUM       32
#define VSYNC_GPIO_NUM    22
#define HREF_GPIO_NUM     26
#define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#elif defined(CAMERA_MODEL_M5STACK)
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    15
#define XCLK_GPIO_NUM     27
#define SIOD_GPIO_NUM     25
#define SIOC_GPIO_NUM     23

#define Y9_GPIO_NUM       19
#define Y8_GPIO_NUM       36
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       39
#define Y5_GPIO_NUM        5
#define Y4_GPIO_NUM       34
#define Y3_GPIO_NUM       35
#define Y2_GPIO_NUM       17
#define VSYNC_GPIO_NUM    22
#define HREF_GPIO_NUM     26
#define PCLK_GPIO_NUM     21

#else
#error "Camera model not selected"
#endif

Stats s_stats;
Ground2Air_Data_Packet s_ground2air_data_packet;
Ground2Air_Config_Packet s_ground2air_config_packet;     

static int s_stats_last_tp = -10000;

static TaskHandle_t s_wifi_tx_task = nullptr;
#ifdef TX_COMPLETION_CB
SemaphoreHandle_t s_wifi_tx_done_semaphore = xSemaphoreCreateBinary();
#endif

static TaskHandle_t s_wifi_rx_task = nullptr;

/////////////////////////////////////////////////////////////////////////

static size_t s_video_frame_data_size = 0;
static bool s_video_frame_started = false;

static int64_t s_video_last_sent_tp = esp_timer_get_time();
static int64_t s_video_last_acquired_tp = esp_timer_get_time();
static bool s_video_skip_frame = false;
static int64_t s_video_target_frame_dt = 0;

/////////////////////////////////////////////////////////////////////////

static int s_uart_verbose = 1;

#define LOG(...) do { if (s_uart_verbose > 0) SAFE_PRINTF(__VA_ARGS__); } while (false) 

/////////////////////////////////////////////////////////////////////////

static constexpr gpio_num_t STATUS_LED_PIN = GPIO_NUM_33;
static constexpr uint8_t STATUS_LED_ON = 0;
static constexpr uint8_t STATUS_LED_OFF = 1;

void initialize_status_led()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << STATUS_LED_PIN;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(STATUS_LED_PIN, STATUS_LED_OFF);
}

IRAM_ATTR uint64_t micros()
{
    return esp_timer_get_time();
}

IRAM_ATTR uint64_t millis()
{
    return esp_timer_get_time() / 1000ULL;
}

IRAM_ATTR void set_status_led_on()
{
    gpio_set_level(STATUS_LED_PIN, STATUS_LED_ON);
}

IRAM_ATTR void update_status_led()
{
    gpio_set_level(STATUS_LED_PIN, STATUS_LED_OFF);
}

//////////////////////////////////////////////////////////////////////////////

Queue s_wlan_incoming_queue;
Queue s_wlan_outgoing_queue;

SemaphoreHandle_t s_wlan_incoming_mux = xSemaphoreCreateBinary();
SemaphoreHandle_t s_wlan_outgoing_mux = xSemaphoreCreateBinary();
SemaphoreHandle_t s_sd_fast_buffer_mux = xSemaphoreCreateBinary();
SemaphoreHandle_t s_sd_slow_buffer_mux = xSemaphoreCreateBinary();

auto _init_result = []() -> bool
{
  xSemaphoreGive(s_wlan_incoming_mux);
  xSemaphoreGive(s_wlan_outgoing_mux);
  xSemaphoreGive(s_sd_fast_buffer_mux);
  xSemaphoreGive(s_sd_slow_buffer_mux);
  return true;
}();

bool init_queues(size_t wlan_incoming_queue_size, size_t wlan_outgoing_queue_size)
{
  s_wlan_outgoing_queue.init(new uint8_t[wlan_outgoing_queue_size], wlan_outgoing_queue_size);
  s_wlan_incoming_queue.init(new uint8_t[wlan_incoming_queue_size], wlan_incoming_queue_size);

  return true;
}

////////////////////////////////////////////////////////////////////////////////////

IRAM_ATTR bool start_writing_wlan_outgoing_packet(Wlan_Outgoing_Packet& packet, size_t size)
{
  size_t real_size = WLAN_IEEE_HEADER_SIZE + size;
  uint8_t* buffer = s_wlan_outgoing_queue.start_writing(real_size);
  if (!buffer)
  {
    packet.ptr = nullptr;
    return false;
  }
  packet.offset = 0;
  packet.size = size;
  packet.ptr = buffer;
  packet.payload_ptr = buffer + WLAN_IEEE_HEADER_SIZE;
  return true;
}
IRAM_ATTR void end_writing_wlan_outgoing_packet(Wlan_Outgoing_Packet& packet)
{
  s_wlan_outgoing_queue.end_writing();
  packet.ptr = nullptr;
}
IRAM_ATTR void cancel_writing_wlan_outgoing_packet(Wlan_Outgoing_Packet& packet)
{
  s_wlan_outgoing_queue.cancel_writing();
  packet.ptr = nullptr;
}

IRAM_ATTR bool start_reading_wlan_outgoing_packet(Wlan_Outgoing_Packet& packet)
{
  size_t real_size = 0;
  uint8_t* buffer = s_wlan_outgoing_queue.start_reading(real_size);
  if (!buffer)
  {
    packet.ptr = nullptr;
    return false;
  }
  packet.offset = 0;
  packet.size = real_size - WLAN_IEEE_HEADER_SIZE;
  packet.ptr = buffer;
  packet.payload_ptr = buffer + WLAN_IEEE_HEADER_SIZE;
  return true;
}
IRAM_ATTR void end_reading_wlan_outgoing_packet(Wlan_Outgoing_Packet& packet)
{
  s_wlan_outgoing_queue.end_reading();
  packet.ptr = nullptr;
}
IRAM_ATTR void cancel_reading_wlan_outgoing_packet(Wlan_Outgoing_Packet& packet)
{
  s_wlan_outgoing_queue.cancel_reading();
  packet.ptr = nullptr;
}

////////////////////////////////////////////////////////////////////////////////////

IRAM_ATTR bool start_writing_wlan_incoming_packet(Wlan_Incoming_Packet& packet, size_t size)
{
  uint8_t* buffer = s_wlan_incoming_queue.start_writing(size);
  if (!buffer)
  {
    packet.ptr = nullptr;
    return false;
  }
  packet.offset = 0;
  packet.size = size;
  packet.ptr = buffer;
  return true;
}
IRAM_ATTR void end_writing_wlan_incoming_packet(Wlan_Incoming_Packet& packet)
{
  s_wlan_incoming_queue.end_writing();
  packet.ptr = nullptr;
}
IRAM_ATTR void cancel_writing_wlan_incoming_packet(Wlan_Incoming_Packet& packet)
{
  s_wlan_incoming_queue.cancel_writing();
  packet.ptr = nullptr;
}

IRAM_ATTR bool start_reading_wlan_incoming_packet(Wlan_Incoming_Packet& packet)
{
  size_t size = 0;
  uint8_t* buffer = s_wlan_incoming_queue.start_reading(size);
  if (!buffer)
  {
    packet.ptr = nullptr;
    return false;
  }
  packet.offset = 0;
  packet.size = size;
  packet.ptr = buffer;
  return true;
}
IRAM_ATTR void end_reading_wlan_incoming_packet(Wlan_Incoming_Packet& packet)
{
  s_wlan_incoming_queue.end_reading();
  packet.ptr = nullptr;
}
IRAM_ATTR void cancel_reading_wlan_incoming_packet(Wlan_Incoming_Packet& packet)
{
  s_wlan_incoming_queue.cancel_reading();
  packet.ptr = nullptr;
}

/////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////

static TaskHandle_t s_sd_write_task = nullptr;
static TaskHandle_t s_sd_enqueue_task = nullptr;
static bool s_sd_initialized = false;
static size_t s_sd_file_size = 0;
static uint32_t s_sd_next_session_id = 0;
static uint32_t s_sd_next_segment_id = 0;

//the fast buffer is RAM and used to transfer data quickly from the camera callback to the slow, SPIRAM buffer. 
//Writing directly to the SPIRAM buffer is too slow in the camera callback and causes lost frames, so I use this RAM buffer and a separate task (sd_enqueue_task) for that.
static constexpr size_t SD_FAST_BUFFER_SIZE = 10000;
Circular_Buffer s_sd_fast_buffer(new uint8_t[SD_FAST_BUFFER_SIZE], SD_FAST_BUFFER_SIZE);

//this slow buffer is used to buffer data that is about to be written to SD. The reason it's this big is because SD card write speed fluctuated a lot and 
// sometimes it pauses for a few hundred ms. So to avoid lost data, I have to buffer it into a big enoigh buffer.
//The data is written to the sd card by the sd_write_task, in chunks of SD_WRITE_BLOCK_SIZE.
static constexpr size_t SD_SLOW_BUFFER_SIZE = 3 * 1024 * 1024;
Circular_Buffer s_sd_slow_buffer((uint8_t*)heap_caps_malloc(SD_SLOW_BUFFER_SIZE, MALLOC_CAP_SPIRAM), SD_SLOW_BUFFER_SIZE);

//Cannot write to SD directly from the slow, SPIRAM buffer as that causes the write speed to plummet. So instead I read from the slow buffer into
// this RAM block and write from it directly. This results in several MB/s write speed performance which is good enough.
static constexpr size_t SD_WRITE_BLOCK_SIZE = 8192;

static void shutdown_sd()
{
    if (!s_sd_initialized)
        return;

    esp_vfs_fat_sdmmc_unmount();
    s_sd_initialized = false;

    //to turn the LED off
    gpio_set_pull_mode((gpio_num_t)4, GPIO_PULLDOWN_ONLY);    // D1, needed in 4-line mode only
}

static bool init_sd()
{
    if (s_sd_initialized)
        return true;

    sdmmc_card_t* card = nullptr;

    esp_vfs_fat_sdmmc_mount_config_t mount_config;
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 2;
    mount_config.allocation_unit_size = 0;

#if 0
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    if (!s_spi_initialized)
        s_spi_initialized = init_spi((spi_host_device_t)host.slot);

    gpio_set_pull_mode((gpio_num_t)15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode((gpio_num_t)13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_13;
    slot_config.host_id = (spi_host_device_t)host.slot;

    esp_err_t ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK)
    {
        LOG("Failed to mount SD card VFAT filesystem. Error: %s\n", esp_err_to_name(ret));
        return false;
    }
#else
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    //host.max_freq_khz = SDMMC_FREQ_PROBING;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    //host.flags = SDMMC_HOST_FLAG_1BIT;

    gpio_set_pull_mode((gpio_num_t)14, GPIO_PULLUP_ONLY);   // CLK, needed in 4-line mode only
    gpio_set_pull_mode((gpio_num_t)15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode((gpio_num_t)2, GPIO_PULLUP_ONLY);    // D0, needed in 4-line mode only
    gpio_set_pull_mode((gpio_num_t)4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    gpio_set_pull_mode((gpio_num_t)12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode((gpio_num_t)13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    //slot_config.width = 1;

    LOG("Mounting SD card...\n");
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK)
    {
        LOG("Failed to mount SD card VFAT filesystem. Error: %s\n", esp_err_to_name(ret));
        //to turn the LED off
        gpio_set_pull_mode((gpio_num_t)4, GPIO_PULLDOWN_ONLY);    // D1, needed in 4-line mode only
        return false;
    }
#endif

    s_sd_initialized = true;

    //find the latest file number
    char buffer[64];
    for (uint32_t i = 0; i < 100000; i++)
    {
        sprintf(buffer, "/sdcard/session%03d_segment000.mjpeg", i);
        FILE* f = fopen(buffer, "rb");
        if (f)
        {
            fclose(f);
            continue;
        }
        s_sd_next_session_id = i;
        s_sd_next_segment_id = 0;
        break;
    }
    return true;
}

static FILE* open_sd_file()
{
    char buffer[64];
    sprintf(buffer, "/sdcard/session%03d_segment%03d.mjpeg", s_sd_next_session_id, s_sd_next_segment_id);
    FILE* f = fopen(buffer, "wb");
    if (!f)
        return nullptr;

    LOG("Opening session file '%s'\n", buffer);
    s_sd_file_size = 0;
    s_sd_next_segment_id++;

    return f;
}

//this will write data from the slow queue to file
static void sd_write_proc(void*)
{
    uint8_t* block = new uint8_t[SD_WRITE_BLOCK_SIZE];

    while (true)
    {
        if (!s_ground2air_config_packet.dvr_record)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        if (!init_sd())
        {
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            continue;
        }

        FILE* f = open_sd_file();
        if (!f)
        {
            shutdown_sd();
            vTaskDelay(1000 / portTICK_PERIOD_MS); 
            continue;
        }

        xSemaphoreTake(s_sd_slow_buffer_mux, portMAX_DELAY);
        s_sd_slow_buffer.clear();
        xSemaphoreGive(s_sd_slow_buffer_mux);

        bool error = false; 
        bool done = false;
        while (!done)
        {
            ulTaskNotifyTake(pdTRUE, 1000 / portTICK_PERIOD_MS); //wait for notification

            while (true) //consume all the buffer
            {
                if (!s_ground2air_config_packet.dvr_record)
                {
                    LOG("Done recording, closing file\n", s_sd_file_size);
                    done = true;
                    break;
                }

                xSemaphoreTake(s_sd_slow_buffer_mux, portMAX_DELAY);
                bool read = s_sd_slow_buffer.read(block, SD_WRITE_BLOCK_SIZE);
                xSemaphoreGive(s_sd_slow_buffer_mux);
                if (!read)
                    break; //not enough data, wait

                if (fwrite(block, SD_WRITE_BLOCK_SIZE, 1, f) == 0)
                {
                    LOG("Error while writing! Stopping session\n");
                    done = true;
                    error = true;
                    break;
                }
                s_stats.sd_data += SD_WRITE_BLOCK_SIZE;
                s_sd_file_size += SD_WRITE_BLOCK_SIZE;
                if (s_sd_file_size > 500 * 1024 * 1024)
                {
                    LOG("Max file size reached: %d. Restarting session\n", s_sd_file_size);
                    done = true;
                    break;
                }
            }
        }

        if (!error)
        {
            fflush(f);
            fsync(fileno(f));
            fclose(f);
        }

        if (!s_ground2air_config_packet.dvr_record || error)
            shutdown_sd();
    }
}

//this will move data from the fast queue to the slow queue
static void sd_enqueue_proc(void*)
{
    while (true)
    {
        if (!s_ground2air_config_packet.dvr_record)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        xSemaphoreTake(s_sd_fast_buffer_mux, portMAX_DELAY);
        s_sd_fast_buffer.clear();
        xSemaphoreGive(s_sd_fast_buffer_mux);

        while (true)
        {
            ulTaskNotifyTake(pdTRUE, 1000 / portTICK_PERIOD_MS); //wait for notification

            xSemaphoreTake(s_sd_fast_buffer_mux, portMAX_DELAY);
            size_t size = s_sd_fast_buffer.size();
            if (size == 0)
            {
                xSemaphoreGive(s_sd_fast_buffer_mux);
                continue; //no data? wait some more
            }

            const void* buffer = s_sd_fast_buffer.start_reading(size);
            xSemaphoreGive(s_sd_fast_buffer_mux);

            xSemaphoreTake(s_sd_slow_buffer_mux, portMAX_DELAY);
            s_sd_slow_buffer.write(buffer, size);
            xSemaphoreGive(s_sd_slow_buffer_mux);

            xSemaphoreTake(s_sd_fast_buffer_mux, portMAX_DELAY);
            s_sd_fast_buffer.end_reading(size);
            xSemaphoreGive(s_sd_fast_buffer_mux);

            if (s_sd_write_task)
                xTaskNotifyGive(s_sd_write_task); //notify task

            if (!s_ground2air_config_packet.dvr_record)
                break;
        }
    }
}


IRAM_ATTR static void add_to_sd_fast_buffer(const void* data, size_t size)
{
    xSemaphoreTake(s_sd_fast_buffer_mux, portMAX_DELAY);
    bool ok = s_sd_fast_buffer.write(data, size);
    xSemaphoreGive(s_sd_fast_buffer_mux);
    if (ok)
    {
        if (s_sd_enqueue_task)
            xTaskNotifyGive(s_sd_enqueue_task); //notify task
    }
    else
        s_stats.sd_drops += size;
}

/////////////////////////////////////////////////////////////////////

float s_wlan_power_dBm = 0;

esp_err_t set_wlan_power_dBm(float dBm)
{
    constexpr float k_min = 2.f;
    constexpr float k_max = 20.f;

    dBm = std::max(std::min(dBm, k_max), k_min);
    s_wlan_power_dBm = dBm;
    int8_t power = static_cast<int8_t>(((dBm - k_min) / (k_max - k_min)) * 80) + 8;
    return esp_wifi_set_max_tx_power(power);
}

float get_wlan_power_dBm()
{
    return s_wlan_power_dBm;
}

WIFI_Rate s_wlan_rate = WIFI_Rate::RATE_G_18M_ODFM;
esp_err_t set_wifi_fixed_rate(WIFI_Rate value)
{
    uint8_t rates[] = 
    {
        WIFI_PHY_RATE_2M_L,
        WIFI_PHY_RATE_2M_S,
        WIFI_PHY_RATE_5M_L,
        WIFI_PHY_RATE_5M_S,
        WIFI_PHY_RATE_11M_L,
        WIFI_PHY_RATE_11M_S,

        WIFI_PHY_RATE_6M,
        WIFI_PHY_RATE_9M,
        WIFI_PHY_RATE_12M,
        WIFI_PHY_RATE_18M,
        WIFI_PHY_RATE_24M,
        WIFI_PHY_RATE_36M,
        WIFI_PHY_RATE_48M,
        WIFI_PHY_RATE_54M,

        WIFI_PHY_RATE_MCS0_LGI,
        WIFI_PHY_RATE_MCS0_SGI,
        WIFI_PHY_RATE_MCS1_LGI,
        WIFI_PHY_RATE_MCS1_SGI,
        WIFI_PHY_RATE_MCS2_LGI,
        WIFI_PHY_RATE_MCS2_SGI,
        WIFI_PHY_RATE_MCS3_LGI,
        WIFI_PHY_RATE_MCS3_SGI,
        WIFI_PHY_RATE_MCS4_LGI,
        WIFI_PHY_RATE_MCS4_SGI,
        WIFI_PHY_RATE_MCS5_LGI,
        WIFI_PHY_RATE_MCS5_SGI,
        WIFI_PHY_RATE_MCS6_LGI,
        WIFI_PHY_RATE_MCS6_SGI,
        WIFI_PHY_RATE_MCS7_LGI,
        WIFI_PHY_RATE_MCS7_SGI,
    };
    //esp_err_t err = esp_wifi_internal_set_fix_rate(ESP_WIFI_IF, true, (wifi_phy_rate_t)rates[(int)value]);
    
    esp_err_t err = esp_wifi_config_80211_tx_rate(ESP_WIFI_IF, (wifi_phy_rate_t)rates[(int)value]);
    if (err == ESP_OK)
        s_wlan_rate = value;
    return err;
}

WIFI_Rate get_wifi_fixed_rate()
{
    return s_wlan_rate;
}


IRAM_ATTR void packet_received_cb(void* buf, wifi_promiscuous_pkt_type_t type)
{
    //;
}



////////////////////////////////////////////////////////////////////////

#ifdef TX_COMPLETION_CB
IRAM_ATTR static void wifi_tx_done(uint8_t ifidx, uint8_t *data, uint16_t *data_len, bool txStatus)
{
    //LOG("tx done\n");
    xSemaphoreGive(s_wifi_tx_done_semaphore);
}
#endif


void setup_wifi()
{
    init_crc8_table();

    initialize_status_led();

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    //ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_ap_handler, NULL, NULL));

    esp_wifi_internal_set_log_level(WIFI_LOG_NONE); //to try in increase bandwidth when we spam the send function and there are no more slots available

    {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        ESP_ERROR_CHECK(esp_wifi_set_mode(ESP_WIFI_MODE));
    }

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    //this reduces throughput for some reason
#ifdef TX_COMPLETION_CB
    ESP_ERROR_CHECK(esp_wifi_set_tx_done_cb(wifi_tx_done));
#endif

    ESP_ERROR_CHECK(set_wifi_fixed_rate(s_ground2air_config_packet.wifi_rate));
    ESP_ERROR_CHECK(esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE));

    wifi_promiscuous_filter_t filter = 
    {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(packet_received_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    esp_wifi_set_bandwidth(ESP_WIFI_IF,WIFI_BW_HT20);


    set_wlan_power_dBm(20.f);

    //esp_log_level_set("*", ESP_LOG_DEBUG);

    LOG("MEMORY After WIFI: \n");
    heap_caps_print_heap_info(MALLOC_CAP_8BIT);

    LOG("Initialized\n");
}

/////////////////////////////////////////////////////////////////////////

IRAM_ATTR void send_air2ground_video_packet(uint8_t * buf)
{
    
}

constexpr size_t PAYLOAD_SIZE = AIR2GROUND_MTU - sizeof(Air2Ground_Video_Packet);

uint8_t CAMERA_BUFFER[2000];

IRAM_ATTR static void camera_data_available(const void* data, size_t stride, size_t count, bool last)
{

    if (data == nullptr) //start frame
    {
        s_video_frame_started = true;        
    }
    else 
    {
        if (!s_video_skip_frame)
        {
            const uint8_t* src = (const uint8_t*)data;

            if (last) //find the end marker for JPEG. Data after that can be discarded
            {
                const uint8_t* dptr = src + (count - 2) * stride;
                while (dptr > src)
                {
                    if (dptr[0] == 0xFF && dptr[stride] == 0xD9)
                    {
                        count = (dptr - src) / stride + 2; //to include the 0xFFD9
                        if ((count & 0x1FF) == 0)
                            count += 1; 
                        if ((count % 100) == 0)
                            count += 1;
                        break;
                    }
                    dptr -= stride;
                }
            }

            uint8_t* packet_data = CAMERA_BUFFER;
            uint8_t* start_ptr = packet_data;
            uint8_t* ptr = start_ptr;
            size_t c = count;

            s_video_frame_data_size += c;

            size_t c8 = c >> 3;
            for (size_t i = c8; i > 0; i--)
            {
                *ptr++ = *src; src += stride;
                *ptr++ = *src; src += stride;
                *ptr++ = *src; src += stride;
                *ptr++ = *src; src += stride;
                *ptr++ = *src; src += stride;
                *ptr++ = *src; src += stride;
                *ptr++ = *src; src += stride;
                *ptr++ = *src; src += stride;
            }
            for (size_t i = c - (c8 << 3); i > 0; i--)
            {
                *ptr++ = *src; src += stride;
            }
            transmitter.send_packet(CAMERA_BUFFER,count,0);
            if (s_ground2air_config_packet.dvr_record)
                add_to_sd_fast_buffer(start_ptr, c);
        }
    }

}

static void init_camera()
{
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 4;
    config.fb_count = 3;

    // camera init
    esp_err_t err = esp_camera_init(&config, camera_data_available);
    if (err != ESP_OK)
    {
        LOG("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_VGA);
    s->set_saturation(s, 0);
}

//#define SHOW_CPU_USAGE

static void print_cpu_usage()
{
#ifdef SHOW_CPU_USAGE
    TaskStatus_t* pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime, ulStatsAsPercentage;

    // Take a snapshot of the number of tasks in case it changes while this
    // function is executing.
    uxArraySize = uxTaskGetNumberOfTasks();
    //LOG("%u tasks\n", uxArraySize);

    // Allocate a TaskStatus_t structure for each task.  An array could be
    // allocated statically at compile time.
    pxTaskStatusArray = (TaskStatus_t*)heap_caps_malloc(uxArraySize * sizeof(TaskStatus_t), MALLOC_CAP_SPIRAM);

    if (pxTaskStatusArray != NULL)
    {
        // Generate raw status information about each task.
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
        //LOG("%u total usage\n", ulTotalRunTime);

        // For percentage calculations.
        ulTotalRunTime /= 100UL;

        // Avoid divide by zero errors.
        if (ulTotalRunTime > 0)
        {
            // For each populated position in the pxTaskStatusArray array,
            // format the raw data as human readable ASCII data
            for (x = 0; x < uxArraySize; x++)
            {
                // What percentage of the total run time has the task used?
                // This will always be rounded down to the nearest integer.
                // ulTotalRunTimeDiv100 has already been divided by 100.
                ulStatsAsPercentage = pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;

                if (ulStatsAsPercentage > 0UL)
                {
                    LOG("%s\t\t%u\t\t%u%%\r\n", pxTaskStatusArray[x].pcTaskName, pxTaskStatusArray[x].ulRunTimeCounter, ulStatsAsPercentage);
                }
                else
                {
                    // If the percentage is zero here then the task has
                    // consumed less than 1% of the total run time.
                    LOG("%s\t\t%u\t\t<1%%\r\n", pxTaskStatusArray[x].pcTaskName, pxTaskStatusArray[x].ulRunTimeCounter);
                }
            }
        }

        // The array is no longer needed, free the memory it consumes.
        free(pxTaskStatusArray);
    }
#endif
}


// uint8_t data[1000]={1,2,4,5,15,5,5,5,5,5,50};
// uint8_t outdata[1444]={0};
// extern "C" void app_main()
// {


//     uint64_t outsize;
//     uint64_t cnt=0;
//     uint8_t key[32]={1,2,3,4,56,6};
//     srand(esp_timer_get_time());

//     printf("Initializing...\n");
    
//     while (true)
//     {
//         esp_task_wdt_reset();
//         uint64_t t1=micros();
//         crypto_aead_chacha20poly1305_encrypt(outdata,&outsize,data,sizeof(data),NULL,0,NULL,(uint8_t *)&cnt,key);
//         uint64_t t2=micros();
//         cnt++;
//         printf("encrypt time=%lld outsize:%lld \n",t2-t1,outsize); 
//         vTaskDelay(10);
        
//     }
// }



extern "C" void app_main()
{
    Ground2Air_Data_Packet& ground2air_data_packet = s_ground2air_data_packet;
    ground2air_data_packet.type = Ground2Air_Header::Type::Data;
    ground2air_data_packet.size = sizeof(ground2air_data_packet);

    Ground2Air_Config_Packet& ground2air_config_packet = s_ground2air_config_packet;
    ground2air_config_packet.type = Ground2Air_Header::Type::Config;
    ground2air_config_packet.size = sizeof(ground2air_config_packet);
    ground2air_config_packet.wifi_rate = WIFI_Rate::RATE_G_18M_ODFM;

    srand(esp_timer_get_time());

    printf("Initializing...\n");

    printf("MEMORY at start: \n");
    heap_caps_print_heap_info(MALLOC_CAP_8BIT);

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_queues(WLAN_INCOMING_BUFFER_SIZE, WLAN_OUTGOING_BUFFER_SIZE);
    setup_wifi();
    init_camera();

    {
        int core = tskNO_AFFINITY;
        BaseType_t res = xTaskCreatePinnedToCore(&sd_write_proc, "SD Write", 4096, nullptr, 1, &s_sd_write_task, core);
        if (res != pdPASS)
            LOG("Failed sd write task: %d\n", res);
    }
    {
        int core = tskNO_AFFINITY;
        BaseType_t res = xTaskCreatePinnedToCore(&sd_enqueue_proc, "SD Enq", 1024, nullptr, 1, &s_sd_enqueue_task, core);
        if (res != pdPASS)
            LOG("Failed sd enqueue task: %d\n", res);
    }
    esp_camera_fb_get(); //this will start the camera capture

    printf("MEMORY Before Loop: \n");
    heap_caps_print_heap_info(MALLOC_CAP_8BIT);

    while (true)
    {
        if (s_uart_verbose > 0 && millis() - s_stats_last_tp >= 1000)
        {
            s_stats_last_tp = millis();
            //LOG("WLAN S: %d, R: %d, E: %d, D: %d, %%: %d || FPS: %d, D: %d || D: %d, E: %d\n",
            //    s_stats.wlan_data_sent, s_stats.wlan_data_received, s_stats.wlan_error_count, s_stats.wlan_received_packets_dropped, s_wlan_outgoing_queue.size() * 100 / s_wlan_outgoing_queue.capacity(),
            //    (int)s_stats.video_frames, s_stats.video_data, s_stats.sd_data, s_stats.sd_drops);
            print_cpu_usage();

            s_stats = Stats();
        }

        vTaskDelay(10);
        esp_task_wdt_reset();

        update_status_led();
    }
}
