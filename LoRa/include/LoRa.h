#ifndef LORA_H
#define LORA_H

#include "driver/spi_master.h"
#include "driver/gpio.h"

class LoRa
{
private:
    spi_device_handle_t _spi;
    spi_host_device_t _spi_host;
    gpio_num_t _mosi;
    gpio_num_t _miso;
    gpio_num_t _sck;
    gpio_num_t _rst;
    gpio_num_t _cs;
    long _frequency;
    int _implicit;
    int _send_packet_lost = 0;

    void write_reg(uint8_t reg, uint8_t value);
    void write_reg_buffer(uint8_t reg, uint8_t *buffer, uint8_t size);
    uint8_t read_reg(uint8_t reg);
    void read_reg_buffer(uint8_t reg, uint8_t *buffer, uint8_t size);

public:
    LoRa(spi_host_device_t spi, gpio_num_t mosi, gpio_num_t miso, gpio_num_t sck, gpio_num_t rst, gpio_num_t cs);
    uint8_t init(long frequency);
    void reset(void);
    void explicit_header_mode(void);
    void implicit_header_mode(int size);
    void sleep(void);
    void set_tx_power(uint8_t level);
    void idle(void);
    void receive(void);
    void set_frequency(long frequency);
    void set_spreading_factor(int sf);
    int get_spreading_factor(void);
    void set_dio_mapping(int dio, int mode);
    int get_dio_mapping(int dio);
    void set_bandwidth(int sbw);
    int get_bandwidth(void);
    void set_coding_rate(int denominator);
    int get_coding_rate(void);
    void set_preamble_length(long length);
    long get_preamble_length(void);
    void set_sync_word(int sw);
    void enable_crc(void);
    void disable_crc(void);
    void send_packet(uint8_t *buf, int size);
    int receive_packet(uint8_t *buf, int size);
    int received(void);
    int get_irq(void);
    int packet_lost(void);
    int packet_rssi(void);
    float packet_snr(void);
    void close(void);
    void dump_registers(void);

};

#endif
