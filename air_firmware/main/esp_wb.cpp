#include "esp_wb.hpp"
#include "wifibroadcast.hpp"
#include "esp_private/wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include "esp_wifi_types.h"
#include "esp_heap_caps.h"
#include "endian.h"
#include "fec.h"
#include "ieee80211_radiotap.h"

#include "key.h"

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
#include "esp_log.h"

#define LOG(...) do { SAFE_PRINTF(__VA_ARGS__); } while (false) 
#define max(a,b) (a>b?a:b)

static const char* TAG = "esp_wb";
Transmitter::Transmitter(int k, int n, uint8_t radio_port,size_t total_buffer_size,size_t line_size):  fec_k(k), fec_n(n), block_idx(0),
                                                                fragment_idx(0),
                                                                max_packet_size(0),
                                                                radio_port(radio_port)
{
    uint32_t packet_cnt_per_frame=0;

    combined_cnt = 1400/line_size;

    packet_cnt_per_frame = total_buffer_size/line_size/combined_cnt;
    packet_cnt_per_frame ++;

    fec_p = fec_new(fec_k, fec_n);

    block_cnt_per_frame = packet_cnt_per_frame / fec_k;
    block_list = new uint8_t**[block_cnt_per_frame];

    for(int i=0; i < block_cnt_per_frame;++i){
        block_list[i] = new uint8_t*[fec_n];

        for(int j=0; j < fec_n; j++)
        {
            block_list[i][j] = new uint8_t[MAX_FEC_PAYLOAD];
        }
    }

    busy=false;

    ESP_LOGI(TAG,"block_cnt_per_frame:%u  packet_cnt_per_frame:%u combined_cnt:%d \n",block_cnt_per_frame,packet_cnt_per_frame,combined_cnt);
    // aim to locate 1 frame in the blocks


    make_session_key();
}

Transmitter::~Transmitter()
{
    // this method should be rewrite 
    // while as for mcu, it's not import to do this.


    // for(int i=0; i < fec_n; i++)
    // {
    //     delete block[i];
    // }
    // delete block;

    // fec_free(fec_p);
}


void Transmitter::make_session_key(void)
{
    randombytes_buf(session_key, sizeof(session_key));
    session_key_packet.packet_type = WFB_PACKET_KEY;
    randombytes_buf(session_key_packet.session_key_nonce, sizeof(session_key_packet.session_key_nonce));
    if (crypto_box_easy(session_key_packet.session_key_data, session_key, sizeof(session_key),
                        session_key_packet.session_key_nonce, (uint8_t *)rx_pubkey, (uint8_t *)tx_secretkey) != 0)
    {
        //throw runtime_error("Unable to make session key!");
        ;
    }
}

void  Transmitter::inject_packet(const uint8_t *buf, size_t size)
{

    uint8_t *p = txbuf;

    // radiotap header
    //memcpy(p, radiotap_header, sizeof(radiotap_header));     // may be optimized later
    //p += sizeof(radiotap_header);

    // ieee80211 header
    memcpy(p, ieee80211_header, sizeof(ieee80211_header));
    p[SRC_MAC_LASTBYTE] = radio_port;
    p[DST_MAC_LASTBYTE] = radio_port;
    p[FRAME_SEQ_LB] = ieee80211_seq & 0xff;
    p[FRAME_SEQ_HB] = (ieee80211_seq >> 8) & 0xff;
    ieee80211_seq += 16;
    p += sizeof(ieee80211_header);

    // FEC data
    memcpy(p, buf, size);
    p += size;
    esp_wifi_80211_tx(WIFI_IF_STA, txbuf,p - txbuf, false);

}


void Transmitter::send_block_fragment(size_t packet_size)
{
    uint8_t** block=block_list[current_block_idx];
    wblock_hdr_t *block_hdr = (wblock_hdr_t*)ciphertext;
    long long unsigned int ciphertext_len;

    //assert(packet_size <= MAX_FEC_PAYLOAD);

    block_hdr->packet_type = WFB_PACKET_DATA;
    block_hdr->nonce = htobe64(((block_idx & BLOCK_IDX_MASK) << 8) + fragment_idx);

    LOG("block_idx:%lld   fragment_idx:%d\n",block_idx,fragment_idx);

    // encrypted payload
    // encrypt 1000 bytes ~= 400 us
    TEST_TIME_FUNC(crypto_aead_chacha20poly1305_encrypt(ciphertext + sizeof(wblock_hdr_t), &ciphertext_len,
                                         block[fragment_idx], packet_size,
                                         (uint8_t*)block_hdr, sizeof(wblock_hdr_t),
                                         NULL, (uint8_t*)(&(block_hdr->nonce)), session_key), ENCRYPT);


    //inject_packet 1000 bytes  ~= 180 us
    TEST_TIME_FUNC(inject_packet(ciphertext, sizeof(wblock_hdr_t) + ciphertext_len),INJECT);

 
    
}

void Transmitter::send_session_key(void)
{
    //fprintf(stderr, "Announce session key\n");
    inject_packet((uint8_t*)&session_key_packet, sizeof(session_key_packet));
}

void Transmitter::clean_cnts(void ){
    fragment_packet_cnt=0;
    pushed_fragment_size=0;
    push_fragment_idx=0;
    push_block_idx=0;

    busy=false;

    current_block_idx=0; 
    fragment_idx=0;
}

uint8_t * Transmitter::push(size_t size,size_t * pushd_size,bool* completed){
    
    if(push_block_idx == block_cnt_per_frame){
        return nullptr;// the buffer is full
    }

    uint8_t** block=block_list[push_block_idx];
    wpacket_hdr_t *packet_hdr = (wpacket_hdr_t *)block[push_fragment_idx];
    uint8_t * result_ptr;


    // if(push_block_idx == 0 && push_fragment_idx==0){
    //     busy=true;
    // }
    

    //memset(block[push_fragment_idx], 0, MAX_FEC_PAYLOAD);
    // don't clear the space, because we will re-enter in the same push_fragment_idx

    //memcpy(block[push_fragment_idx], &packet_hdr, sizeof(packet_hdr));
    // don't need to set packet_hdr, we will edit the size later

    //memcpy(block[push_fragment_idx] + sizeof(packet_hdr), buf, size);
    // the data would be copied by the returned ptr

    result_ptr = block[push_fragment_idx] + sizeof(packet_hdr) + pushed_fragment_size;
    pushed_fragment_size += size;
    fragment_packet_cnt++;

    *pushd_size=pushed_fragment_size;
    packet_hdr->packet_size = htobe16(pushed_fragment_size); // edit the packet_size

    ESP_LOGI(TAG,"push_fragment_idx:%u  push_block_idx:%u fragment_packet_cnt:%d \n",push_fragment_idx,push_block_idx,fragment_packet_cnt);
    if(fragment_packet_cnt != combined_cnt){
        *completed=false;
        return result_ptr;
    }

    // the fragment has been full
    *completed=true;
    fragment_packet_cnt=0;
    pushed_fragment_size=0;
    push_fragment_idx++; // change to next fragment
    

    if(push_fragment_idx == fec_k){
        // the block has been full
        // change to next block
        push_fragment_idx = 0;
        push_block_idx++;
    }
    
    return result_ptr;
}

void Transmitter::send_packet(size_t size){
    uint8_t** block=block_list[current_block_idx];

    send_block_fragment(sizeof(wpacket_hdr_t) + size);
    fragment_idx++;
    max_packet_size = max(max_packet_size, sizeof(wpacket_hdr_t) + size);
    if (fragment_idx < fec_k)  return;

    TEST_TIME_FUNC(fec_encode(fec_p, (const uint8_t**)block, block + fec_k, max_packet_size),FEC);
    while (fragment_idx < fec_n)
    {
        send_block_fragment(max_packet_size);
        fragment_idx += 1;
    }

    current_block_idx += 1;
    block_idx += 1;
    fragment_idx = 0;
    max_packet_size = 0;

    if(current_block_idx == block_cnt_per_frame){
        current_block_idx = 0;
        push_fragment_idx=0;
        push_block_idx=0;
        busy = false;
    }
        // Generate new session key after MAX_BLOCK_IDX blocks
    if (block_idx > MAX_BLOCK_IDX)
    {
        make_session_key();
        send_session_key();
        block_idx = 0;
    }
    
}

void Transmitter::send_packet(const uint8_t *buf, size_t size, uint8_t flags)
{
    wpacket_hdr_t packet_hdr;
    uint8_t** block=block_list[current_block_idx];
    //assert(size <= MAX_PAYLOAD_SIZE);

    // FEC-only packets are only for closing already opened blocks
    if(fragment_idx == 0 && flags & WFB_PACKET_FEC_ONLY)
    {
        return;
    }

    packet_hdr.packet_size = htobe16(size);
    packet_hdr.flags = flags;
    memset(block[fragment_idx], '\0', MAX_FEC_PAYLOAD);
    memcpy(block[fragment_idx], &packet_hdr, sizeof(packet_hdr));
    memcpy(block[fragment_idx] + sizeof(packet_hdr), buf, size);


    send_block_fragment(sizeof(packet_hdr) + size);


    max_packet_size = max(max_packet_size, sizeof(packet_hdr) + size);
    fragment_idx += 1;

    if (fragment_idx < fec_k)  return;



    // fec 4000 bytes (4 blocks, 1000bytes per block) ~= 900 us
    TEST_TIME_FUNC(fec_encode(fec_p, (const uint8_t**)block, block + fec_k, max_packet_size),FEC);
    while (fragment_idx < fec_n)
    {
        send_block_fragment(max_packet_size);
        fragment_idx += 1;
    }
    block_idx += 1;
    fragment_idx = 0;
    max_packet_size = 0;


    // Generate new session key after MAX_BLOCK_IDX blocks
    if (block_idx > MAX_BLOCK_IDX)
    {
        make_session_key();
        send_session_key();
        block_idx = 0;
    }
}



