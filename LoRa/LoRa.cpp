#include "esp_log.h"
#include "esp_system.h"


#include "LoRa.h"

/*
 * Register definitions
 */
#define REG_FIFO                       0x00
#define REG_OP_MODE                    0x01
#define REG_FRF_MSB                    0x06
#define REG_FRF_MID                    0x07
#define REG_FRF_LSB                    0x08
#define REG_PA_CONFIG                  0x09
#define REG_LNA                        0x0c
#define REG_FIFO_ADDR_PTR              0x0d
#define REG_FIFO_TX_BASE_ADDR          0x0e
#define REG_FIFO_RX_BASE_ADDR          0x0f
#define REG_FIFO_RX_CURRENT_ADDR       0x10
#define REG_IRQ_FLAGS                  0x12
#define REG_RX_NB_BYTES                0x13
#define REG_PKT_SNR_VALUE              0x19
#define REG_PKT_RSSI_VALUE             0x1a
#define REG_MODEM_CONFIG_1             0x1d
#define REG_MODEM_CONFIG_2             0x1e
#define REG_PREAMBLE_MSB               0x20
#define REG_PREAMBLE_LSB               0x21
#define REG_PAYLOAD_LENGTH             0x22
#define REG_MODEM_CONFIG_3             0x26
#define REG_RSSI_WIDEBAND              0x2c
#define REG_DETECTION_OPTIMIZE         0x31
#define REG_DETECTION_THRESHOLD        0x37
#define REG_SYNC_WORD                  0x39
#define REG_DIO_MAPPING_1              0x40
#define REG_DIO_MAPPING_2              0x41
#define REG_VERSION                    0x42

/*
 * Transceiver modes
 */
#define MODE_LONG_RANGE_MODE           0x80
#define MODE_SLEEP                     0x00
#define MODE_STDBY                     0x01
#define MODE_TX                        0x03
#define MODE_RX_CONTINUOUS             0x05
#define MODE_RX_SINGLE                 0x06

/*
 * PA configuration
 */
#define PA_BOOST                       0x80

/*
 * IRQ masks
 */
#define IRQ_TX_DONE_MASK               0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK     0x20
#define IRQ_RX_DONE_MASK               0x40

#define PA_OUTPUT_RFO_PIN              0
#define PA_OUTPUT_PA_BOOST_PIN         1

#define TIMEOUT_RESET                  100

#define SPI_TRANSMIT                   1

#define BUFFER_IO 1


#define TAG "LoRa"

// Private methods
void LoRa::write_reg(uint8_t reg, uint8_t value)
{
    uint8_t out[2] = { static_cast<uint8_t>(0x80 | reg), static_cast<uint8_t>(value) };
    spi_transaction_t t = {};
    t.flags = 0;
    t.length = 8 * sizeof(out);
    t.tx_buffer = out;
    t.rx_buffer = NULL;

    gpio_set_level(_cs, 0);  // Control CS manually

    spi_device_transmit(_spi, &t);

    gpio_set_level(_cs, 1);  // Release CS manually
}

void LoRa::write_reg_buffer(uint8_t reg, uint8_t *buffer, uint8_t size)
{
    uint8_t *out = (uint8_t *)malloc(size + 1);
    out[0] = static_cast<uint8_t>(0x80 | reg);
    for (int i = 0; i < size; i++) {
        out[i + 1] = static_cast<uint8_t>(buffer[i]);
    }

    spi_transaction_t t = {};
    t.flags = 0;
    t.length = static_cast<size_t>(8 * (size + 1));
    t.tx_buffer = out;
    t.rx_buffer = NULL;

    gpio_set_level(_cs, 0);  // Control CS manually

#if SPI_TRANSMIT
    spi_device_transmit(_spi, &t);
#else
    spi_device_polling_transmit(_spi, &t);
#endif

    gpio_set_level(_cs, 1);  // Release CS manually
    free(out);
}

uint8_t LoRa::read_reg(uint8_t reg)
{
    esp_err_t ret;
    uint8_t out[2] = { reg, 0xff };
    uint8_t in[2];
    
    spi_transaction_t t = {};
    t.flags = 0;
    t.length = 8 * sizeof(out);
    t.tx_buffer = out;
    t.rx_buffer = in;

    //gpio_set_level(_cs, 0);  // Control CS manually

    ret = spi_device_transmit(_spi, &t);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_transmit failed, ret=%d", ret);
    }

    //gpio_set_level(_cs, 1);  // Release CS manually
    return in[1];
}

void LoRa::read_reg_buffer(uint8_t reg, uint8_t *buffer, uint8_t size)
{
    uint8_t *out = (uint8_t *)malloc(size + 1);
    uint8_t *in = (uint8_t *)malloc(size + 1);
    out[0] = static_cast<uint8_t>(reg);
    for (int i = 0; i < size; i++) {
        out[i + 1] = 0xff;
    }

    spi_transaction_t t = {};
    t.flags = 0;
    t.length = static_cast<size_t>(8 * (size + 1));
    t.tx_buffer = out;
    t.rx_buffer = in;

    gpio_set_level(_cs, 0);  // Control CS manually

#if SPI_TRANSMIT
    spi_device_transmit(_spi, &t);
#else
    spi_device_polling_transmit(_spi, &t);
#endif

    gpio_set_level(_cs, 1);  // Release CS manually
    for (int i = 0; i < size; i++) {
        buffer[i] = static_cast<int>(in[i + 1]);
    }

    free(out);
    free(in);
}

// Public methods

LoRa::LoRa(spi_host_device_t spi, gpio_num_t mosi,
                             gpio_num_t miso, gpio_num_t sck, gpio_num_t rst, gpio_num_t cs) : _spi_host(spi), _mosi(mosi), _miso(miso), _sck(sck), _rst(rst), _cs(cs)
{
}


void LoRa::reset(void)
{
    gpio_set_level(_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void LoRa::explicit_header_mode(void)
{
    _implicit = 0;
    write_reg(REG_MODEM_CONFIG_1, read_reg(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRa::implicit_header_mode(int size)
{
   _implicit = 1;
   write_reg(REG_MODEM_CONFIG_1, read_reg(REG_MODEM_CONFIG_1) | 0x01);
   write_reg(REG_PAYLOAD_LENGTH, size);
}

void LoRa::idle(void)
{
   write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRa::sleep(void)
{ 
   write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRa::receive(void)
{
    write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}


void LoRa::set_tx_power(uint8_t level)
{
    if (level < 2) level = 2;
    else if (level > 17) level = 17;
    write_reg(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

void LoRa::set_frequency(long frequency)
{
   _frequency = frequency;

   uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

   write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
   write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
   write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void LoRa::set_spreading_factor(int sf)
{
   if (sf < 6) sf = 6;
   else if (sf > 12) sf = 12;

   if (sf == 6) {
      write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
      write_reg(REG_DETECTION_THRESHOLD, 0x0c);
   } else {
      write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
      write_reg(REG_DETECTION_THRESHOLD, 0x0a);
   }

   write_reg(REG_MODEM_CONFIG_2, (read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

int LoRa::get_spreading_factor(void)
{
   return (read_reg(REG_MODEM_CONFIG_2) >> 4);
}

void LoRa::set_dio_mapping(int dio, int mode)
{
   if (dio < 4) {
      int _mode = read_reg(REG_DIO_MAPPING_1);
      if (dio == 0) {
         _mode = _mode & 0x3F;
         _mode = _mode | (mode << 6);
      } else if (dio == 1) {
         _mode = _mode & 0xCF;
         _mode = _mode | (mode << 4);
      } else if (dio == 2) {
         _mode = _mode & 0xF3;
         _mode = _mode | (mode << 2);
      } else if (dio == 3) {
         _mode = _mode & 0xFC;
         _mode = _mode | mode;
      }
      write_reg(REG_DIO_MAPPING_1, _mode);
      ESP_LOGD(TAG, "REG_DIO_MAPPING_1=0x%02x", _mode);
   } else if (dio < 6) {
      int _mode = read_reg(REG_DIO_MAPPING_2);
      if (dio == 4) {
         _mode = _mode & 0x3F;
         _mode = _mode | (mode << 6);
      } else if (dio == 5) {
         _mode = _mode & 0xCF;
         _mode = _mode | (mode << 4);
      }
      ESP_LOGD(TAG, "REG_DIO_MAPPING_2=0x%02x", _mode);
      write_reg(REG_DIO_MAPPING_2, _mode);
   }
}


int LoRa::get_dio_mapping(int dio)
{
   if (dio < 4) {
      int _mode = read_reg(REG_DIO_MAPPING_1);
      ESP_LOGD(TAG, "REG_DIO_MAPPING_1=0x%02x", _mode);
      if (dio == 0) {
         return ((_mode >> 6) & 0x03);
      } else if (dio == 1) {
         return ((_mode >> 4) & 0x03);
      } else if (dio == 2) {
         return ((_mode >> 2) & 0x03);
      } else if (dio == 3) {
         return (_mode & 0x03);
      }
   } else if (dio < 6) {
      int _mode = read_reg(REG_DIO_MAPPING_2);
      ESP_LOGD(TAG, "REG_DIO_MAPPING_2=0x%02x", _mode);
      if (dio == 4) {
         return ((_mode >> 6) & 0x03);
      } else if (dio == 5) {
         return ((_mode >> 4) & 0x03);
      }
   }
   return 0;
}

void LoRa::set_bandwidth(int sbw)
{
   if (sbw < 10) {
      write_reg(REG_MODEM_CONFIG_1, (read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (sbw << 4));
   }
}

int LoRa::get_bandwidth(void)
{
   //int bw;
   //bw = read_reg(REG_MODEM_CONFIG_1) & 0xf0;
   //ESP_LOGD(TAG, "bw=0x%02x", bw);
   //bw = bw >> 4;
   //return bw;
   return ((read_reg(REG_MODEM_CONFIG_1) & 0xf0) >> 4);
}

void LoRa::set_coding_rate(int denominator)
{
   if (denominator < 5) denominator = 5;
   else if (denominator > 8) denominator = 8;

   int cr = denominator - 4;
   write_reg(REG_MODEM_CONFIG_1, (read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

int LoRa::get_coding_rate(void)
{
   return ((read_reg(REG_MODEM_CONFIG_1) & 0x0E) >> 1);
}

void LoRa::set_preamble_length(long length)
{
   write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
   write_reg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

long LoRa::get_preamble_length(void)
{
   long preamble;
   preamble = read_reg(REG_PREAMBLE_MSB) << 8;
   preamble = preamble + read_reg(REG_PREAMBLE_LSB);
   return preamble;
}

void LoRa::set_sync_word(int sw)
{
   write_reg(REG_SYNC_WORD, sw);
}

void LoRa::enable_crc(void)
{
   write_reg(REG_MODEM_CONFIG_2, read_reg(REG_MODEM_CONFIG_2) | 0x04);
}

void LoRa::disable_crc(void)
{
   write_reg(REG_MODEM_CONFIG_2, read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}


uint8_t LoRa::init(long frequency)
{
    esp_err_t ret;
    _frequency = frequency;

    gpio_reset_pin(_rst);
    gpio_set_direction(_rst, GPIO_MODE_OUTPUT);

    gpio_reset_pin(_cs);
    gpio_set_direction(_cs, GPIO_MODE_OUTPUT);
    gpio_set_level(_cs, 1);

    ESP_LOGI(TAG, "Initializing LoRa with frequency %ld", frequency);

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = _mosi;
    buscfg.miso_io_num = _miso;
    buscfg.sclk_io_num = _sck;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 0;

    ret = spi_bus_initialize(_spi_host, &buscfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);

    spi_device_interface_config_t dev = {};
    dev.clock_speed_hz = 9000000;
    dev.mode = 0;
    dev.spics_io_num = _cs;  // CS is controlled manually
    dev.queue_size = 7;
    dev.flags = 0;
    dev.pre_cb = NULL;

    ret = spi_bus_add_device(_spi_host, &dev, &_spi);
    assert(ret == ESP_OK);

    reset();

    uint8_t version;
    uint8_t i = 0;
    while(i++ < TIMEOUT_RESET) {
        version = read_reg(REG_VERSION);
        ESP_LOGD(TAG, "version=0x%02x", version);
        if(version == 0x12) break;
        vTaskDelay(2);
    }
    ESP_LOGD(TAG, "i=%d, TIMEOUT_RESET=%d", i, TIMEOUT_RESET);
    if (i == TIMEOUT_RESET + 1) return 0;

    sleep();
    write_reg(REG_FIFO_RX_BASE_ADDR, 0);
    write_reg(REG_FIFO_TX_BASE_ADDR, 0);
    write_reg(REG_LNA, read_reg(REG_LNA) | 0x03);
    write_reg(REG_MODEM_CONFIG_3, 0x04);
    set_tx_power(17);

    idle();

    set_frequency(_frequency);
    return 1;
}


void LoRa::send_packet(uint8_t *buf, int size)
{
   /*
    * Transfer data to radio.
    */
   idle();
   write_reg(REG_FIFO_ADDR_PTR, 0);

#if BUFFER_IO
   write_reg_buffer(REG_FIFO, buf, size);
#else
   for(int i=0; i<size; i++) 
      write_reg(REG_FIFO, *buf++);
#endif
   
   write_reg(REG_PAYLOAD_LENGTH, size);
   
   /*
    * Start transmission and wait for conclusion.
    */
   write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
#if 0
   while((read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
      vTaskDelay(2);
#endif
   int loop = 0;
   while(1) {
      int irq = read_reg(REG_IRQ_FLAGS);
      ESP_LOGD(TAG, "read_reg=0x%x", irq);
      if ((irq & IRQ_TX_DONE_MASK) == IRQ_TX_DONE_MASK) break;
      loop++;
      if (loop == 10) break;
      vTaskDelay(2);
   }
   if (loop == 10) {
      _send_packet_lost++;
      ESP_LOGE(TAG, "send_packet Fail");
   }
   write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}



int LoRa::receive_packet(uint8_t *buf, int size)
{
   int len = 0;

   /*
    * Check interrupts.
    */
   int irq = read_reg(REG_IRQ_FLAGS);
   write_reg(REG_IRQ_FLAGS, irq);
   if((irq & IRQ_RX_DONE_MASK) == 0) return 0;
   if(irq & IRQ_PAYLOAD_CRC_ERROR_MASK) return 0;

   /*
    * Find packet size.
    */
   if (_implicit) len = read_reg(REG_PAYLOAD_LENGTH);
   else len = read_reg(REG_RX_NB_BYTES);

   /*
    * Transfer data from radio.
    */
   idle();   
   write_reg(REG_FIFO_ADDR_PTR, read_reg(REG_FIFO_RX_CURRENT_ADDR));
   if(len > size) len = size;
#if BUFFER_IO
   read_reg_buffer(REG_FIFO, buf, len);
#else
   for(int i=0; i<len; i++) 
      *buf++ = read_reg(REG_FIFO);
#endif

   return len;
}

int LoRa::received(void)
{
   if(read_reg(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) return 1;
   return 0;
}

int LoRa::get_irq(void)
{
   return (read_reg(REG_IRQ_FLAGS));
}


int LoRa::packet_lost(void)
{
   return (_send_packet_lost);
}

int LoRa::packet_rssi(void)
{
   return (read_reg(REG_PKT_RSSI_VALUE) - (_frequency < 868E6 ? 164 : 157));
}

float LoRa::packet_snr(void)
{
   return ((int8_t)read_reg(REG_PKT_SNR_VALUE)) * 0.25;
}

void LoRa::close(void)
{
   sleep();
//   close(__spi);  FIXME: end hardware features after close
//   close(__cs);
//   close(__rst);
//   __spi = -1;
//   __cs = -1;
//   __rst = -1;
}

void LoRa::dump_registers(void)
{
   int i;
   printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
   for(i=0; i<0x40; i++) {
      printf("%02X ", read_reg(i));
      if((i & 0x0f) == 0x0f) printf("\n");
   }
   printf("\n");
}


