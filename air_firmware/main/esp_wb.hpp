#ifndef __ESP_WB_HPP__
#define __ESP_WB_HPP__

#include <stdint.h>
#include "sodium.h"


class Transmitter
{
public:
    Transmitter(int k, int m, const std::string &keypair);
    virtual ~Transmitter();
    void send_packet(const uint8_t *buf, size_t size, uint8_t flags);
    void send_session_key(void);
    virtual void select_output(int idx) = 0;
protected:
    virtual void inject_packet(const uint8_t *buf, size_t size) = 0;

private:
    void send_block_fragment(size_t packet_size);
    void make_session_key(void);

    fec_t* fec_p;
    int fec_k;  // RS number of primary fragments in block
    int fec_n;  // RS total number of fragments in block
    uint64_t block_idx; //block_idx << 8 + fragment_idx = nonce (64bit)
    uint8_t fragment_idx;
    uint8_t** block;
    size_t max_packet_size;

    // tx->rx keypair
    uint8_t tx_secretkey[crypto_box_SECRETKEYBYTES];
    uint8_t rx_publickey[crypto_box_PUBLICKEYBYTES];
    uint8_t session_key[crypto_aead_chacha20poly1305_KEYBYTES];
    wsession_key_t session_key_packet;
};

#endif
