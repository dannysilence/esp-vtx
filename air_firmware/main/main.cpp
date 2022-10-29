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

typedef struct{
    size_t size;
}PipelineData;

class Buffer{
    public:

    void clear(){
        read_ptr=buf;
        write_ptr=buf;
        available_length=max_size; 
    }


    Buffer(size_t size){
        buf =(uint8_t *) malloc(size);
        max_size=size;
        clear();
        write_mux = xSemaphoreCreateBinary();
        read_mux = xSemaphoreCreateBinary();
    }

    ~Buffer(){
        free(buf);
    }
    void write(uint8_t * data, size_t len){
        xSemaphoreTake(write_mux, portMAX_DELAY);

        uint8_t * old_ptr=write_ptr;
        if(len>available_length){
            uint32_t wrap_cnt=len-available_length;
            write_ptr=buf+wrap_cnt;
            xSemaphoreGive(write_mux);
            memcpy(old_ptr,data,available_length);
            memcpy(buf,data+available_length,wrap_cnt);
            available_length=max_size-wrap_cnt;
        }else{
            write_ptr+=len;
            xSemaphoreGive(write_mux);
            memcpy(old_ptr,data,len);
            available_length-=len;
        }
    }

    void read_out(uint8_t * target,size_t len){
        
        xSemaphoreTake(read_mux, portMAX_DELAY);
        uint8_t * old_ptr=read_ptr;
        if(read_ptr+len>buf+max_size){
            uint32_t first_cnt=buf+max_size-read_ptr;
            read_ptr=buf+first_cnt;
            xSemaphoreGive(read_mux);
            memcpy(target,old_ptr,first_cnt);
            memcpy(target+first_cnt,buf,len-first_cnt);
        }else{
            read_ptr+=len;
            xSemaphoreGive(read_mux);
            memcpy(target,old_ptr,len);
        }
    }

    private:
    uint8_t * buf;
    size_t max_size;
    size_t available_length;
    uint8_t * write_ptr;
    uint8_t * read_ptr;
    SemaphoreHandle_t write_mux,read_mux;
    
};

QueueHandle_t cam2fec_pipeline_queue;


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
static bool buffer_ready=true;

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

uint64_t last_cap_time,cap_dts;
uint32_t cam_count;

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
    ESP_ERROR_CHECK(esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE));

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

    LOG("Initialized\n");
}

/////////////////////////////////////////////////////////////////////////

IRAM_ATTR void send_air2ground_video_packet(uint8_t * buf)
{
    
}

constexpr size_t PAYLOAD_SIZE = AIR2GROUND_MTU - sizeof(Air2Ground_Video_Packet);

uint8_t CAMERA_BUFFER[2000];
uint8_t Frame_buffer[8192];



IRAM_ATTR static void camera_data_available(const void* data, size_t stride, size_t count, bool last)
{

    static uint32_t frame_bytes_cnt=0;
    if (data == nullptr && buffer_ready) //start frame
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


            
            if(s_video_frame_started){
                memcpy(Frame_buffer+frame_bytes_cnt,CAMERA_BUFFER,count);
                frame_bytes_cnt += count;
            }

            if(last && s_video_frame_started){
                buffer_ready=false;
                s_video_frame_data_size=frame_bytes_cnt;
                frame_bytes_cnt=0;

                cap_dts=micros()-last_cap_time;
                last_cap_time=micros();
                s_video_frame_started=false;
            }
            //PipelineData temp_pipeline_item={count};
            //xQueueSendToBack(cam2fec_pipeline_queue,&temp_pipeline_item,portMAX_DELAY);
            //give  Notify to fec

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
    config.frame_size = FRAMESIZE_HQVGA;
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
    s->set_framesize(s, FRAMESIZE_HQVGA);
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


void fec_proc(void *){
    while(true){
        //ulTaskNotifyTake(pdTRUE, 1000 / portTICK_PERIOD_MS); //wait for notification
        PipelineData temp;
        xQueueReceive(cam2fec_pipeline_queue,&temp,portMAX_DELAY);
    }
}

#include "driver/uart.h"
#include "soc/uart_periph.h"
#include "esp_rom_gpio.h"
#include "driver/gpio.h"
uint8_t one_frame_buffer[8192]={0};
uint8_t final_buffer[8192]={0};

#define DEFAULT_UART_CHANNEL    (0)
#define DEFAULT_UART_RX_PIN     (GPIO_NUM_3)
#define DEFAULT_UART_TX_PIN     (GPIO_NUM_1)

#define UARTS_BAUD_RATE         (115200)
#define TASK_STACK_SIZE         (2048)
#define READ_BUF_SIZE           (1024)


static void configure_uarts(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(DEFAULT_UART_CHANNEL, READ_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(DEFAULT_UART_CHANNEL, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(DEFAULT_UART_CHANNEL, DEFAULT_UART_TX_PIN, DEFAULT_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}


void construct_test_package(uint8_t * buff,int len){
    int *p=(int * )buff;
    for(int i=0;i<len/4;++i){
        *p=i;
        ++p;
    }
}
void construct_test_package2(uint8_t * buff,int len){
    uint8_t data=0;
    for(int i=0;i<len;++i){
        buff[i]=data;
        data++;
    }
}

#define TEST_CORRECT
#undef TEST_CORRECT
extern "C" void app_main()
{
    Ground2Air_Data_Packet& ground2air_data_packet = s_ground2air_data_packet;
    ground2air_data_packet.type = Ground2Air_Header::Type::Data;
    ground2air_data_packet.size = sizeof(ground2air_data_packet);

    Ground2Air_Config_Packet& ground2air_config_packet = s_ground2air_config_packet;
    ground2air_config_packet.type = Ground2Air_Header::Type::Config;
    ground2air_config_packet.size = sizeof(ground2air_config_packet);
    ground2air_config_packet.wifi_rate = WIFI_Rate::RATE_G_6M_ODFM;

    static uint8_t temp_buffer[1024];

    srand(esp_timer_get_time());
    //configure_uarts();
    

    printf("Initializing...\n");

    printf("MEMORY at start: \n");
    heap_caps_print_heap_info(MALLOC_CAP_8BIT);
    cam2fec_pipeline_queue = xQueueCreate(30, sizeof(PipelineData));

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //init_queues(WLAN_INCOMING_BUFFER_SIZE, WLAN_OUTGOING_BUFFER_SIZE);
    setup_wifi();
    init_camera();


    esp_camera_fb_get(); //this will start the camera capture



    printf("MEMORY Before Loop: \n");
    heap_caps_print_heap_info(MALLOC_CAP_8BIT);
    uart_driver_install(DEFAULT_UART_CHANNEL, READ_BUF_SIZE * 2, 0, 0, NULL, 0);
    while (true)
    {
        static int cnt=0;
        static size_t now_size=0;
        static int len=0;
        static uint8_t data[8];
        esp_task_wdt_reset();

        vTaskDelay(10);

        do {
            len = uart_read_bytes(0, data,  1, 100 / portTICK_RATE_MS);
        } while (len == 0);

        #ifndef TEST_CORRECT
        if(!buffer_ready){
            now_size=s_video_frame_data_size;
            memcpy(one_frame_buffer,Frame_buffer,s_video_frame_data_size);
            buffer_ready=true;
            cnt++;
            transmitter.send_session_key();
        }
        #else
            construct_test_package2(one_frame_buffer,5000);
            now_size=5000;
        #endif

        size_t one_cycle_size=now_size/8;


        for(int i=0;i<8;++i){
            //temp_buffer[0]=(uint8_t )i;
            if(i==7){
                transmitter.send_session_key();
                //memcpy(temp_buffer+1,one_frame_buffer+i*one_cycle_size,now_size-i*one_cycle_size);
                transmitter.send_packet(one_frame_buffer+i*one_cycle_size,now_size-i*one_cycle_size,0);
                //transmitter.send_packet(temp_buffer,now_size-i*one_cycle_size+1,0);
                LOG("send size:%d, cnt:%d\n",now_size-i*one_cycle_size,i);
            }else{
                //memcpy(temp_buffer+1,one_frame_buffer+i*one_cycle_size,one_cycle_size);
                //transmitter.send_packet(temp_buffer,one_cycle_size+1,0);
                transmitter.send_packet(one_frame_buffer+i*one_cycle_size,one_cycle_size,0);
                LOG("send size:%d, cnt:%d\n",one_cycle_size,i);
            }
            //vTaskDelay(100);
        }

        // do {
        //     len = uart_read_bytes(0, data,  1, 250 / portTICK_RATE_MS);
        // } while (len == 0);
        // vTaskDelay(100);
        // uart_write_bytes(0,&now_size,sizeof(size_t));
        // vTaskDelay(100);
        // uart_write_bytes(0,one_frame_buffer,now_size);
        // continue;
        

        

        
        // if (s_uart_verbose > 0 && millis() - s_stats_last_tp >= 1000)
        // {
        //     s_stats_last_tp = millis();
        //     //LOG("WLAN S: %d, R: %d, E: %d, D: %d, %%: %d || FPS: %d, D: %d || D: %d, E: %d\n",
        //     //    s_stats.wlan_data_sent, s_stats.wlan_data_received, s_stats.wlan_error_count, s_stats.wlan_received_packets_dropped, s_wlan_outgoing_queue.size() * 100 / s_wlan_outgoing_queue.capacity(),
        //     //    (int)s_stats.video_frames, s_stats.video_data, s_stats.sd_data, s_stats.sd_drops);
        //     print_cpu_usage();

        //     if(s_video_frame_end){
        //         pipeline_buffer->read_out(CAMERA_BUFFER,);
        //     }
        //     transmitter.send_packet(CAMERA_BUFFER,1600,0);

        //     transmitter.send_session_key();
        //     //esp_wifi_80211_tx(ESP_WIFI_IF, CAMERA_BUFFER, 500, false);
        //     s_stats = Stats();
        // }

        // vTaskDelay(10);
        

    }
}
