#ifndef __ESP_WB_HPP__
#define __ESP_WB_HPP__

#include <stdint.h>
#include "sodium.h"
#include "fec.h"
#include "wifibroadcast.hpp"


class Transmitter
{
public:
    Transmitter(int k, int n, uint8_t radio_port,size_t total_buffer_size,size_t line_size);
    virtual ~Transmitter();
    void send_packet(const uint8_t *buf, size_t size, uint8_t flags);
    void send_packet(size_t size);
    void send_session_key(void);
    uint8_t * push(size_t size,size_t * pushd_size,bool* completed);
    void clean_cnts(void );
    bool busy;
protected:
    void inject_packet(const uint8_t *buf, size_t size);

private:
    void send_block_fragment(size_t packet_size);
    void make_session_key(void);
    uint8_t combined_cnt; // combined sevral packets into 1 block fragment. used in low resolution. for example, 320*160, then we can combine 3 packets into 1 fragment.
    uint32_t block_cnt_per_frame;
    fec_t* fec_p;
    int fec_k;  // RS number of primary fragments in block
    int fec_n;  // RS total number of fragments in block
    uint64_t block_idx; //block_idx << 8 + fragment_idx = nonce (64bit)
    uint16_t current_block_idx;
    uint8_t fragment_idx;
    uint8_t fragment_packet_cnt;   
    uint8_t push_fragment_idx;
    uint8_t push_block_idx;
    size_t pushed_fragment_size;
    //uint8_t** block;
    uint8_t *** block_list;

    size_t max_packet_size;
    uint8_t radio_port;
    // tx->rx keypair
    //uint8_t tx_secretkey[crypto_box_SECRETKEYBYTES];
    //uint8_t rx_publickey[crypto_box_PUBLICKEYBYTES];
    uint8_t session_key[crypto_aead_chacha20poly1305_KEYBYTES];
    uint8_t txbuf[MAX_PACKET_SIZE];
    wsession_key_t session_key_packet;

    uint16_t ieee80211_seq=0;

    uint8_t ciphertext[MAX_FORWARDER_PACKET_SIZE];

};

#endif
