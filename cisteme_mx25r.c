#define DT_DRV_COMPAT cisteme_mx25r

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>

#include <cisteme_mx25r.h>
#include "cisteme_mx25r_regmap.h"

static int mx25r_cmd(const struct device *dev, uint8_t cmd) 
{
    struct mx25r_data *data = dev->data;
    const struct mx25r_config *config = dev->config;

    uint8_t command = cmd;
    data->buf_tx = (struct spi_buf){.buf = &command, .len = 1};
    return spi_write_dt(&config->spi, &data->buf_set_tx);
}

static int write_enable(const struct device *dev) 
{
    return mx25r_cmd(dev, CMD_WREN);
}

static int write_disable(const struct device *dev) 
{
    return mx25r_cmd(dev, CMD_WRDI);
}

static int reset_enable(const struct device *dev) 
{
    return mx25r_cmd(dev, CMD_RSTEN);
}

static int reset(const struct device *dev) 
{
    return mx25r_cmd(dev, CMD_RST);
}

static int read_status(const struct device *dev, uint8_t *status) 
{
    struct mx25r_data *data = dev->data;
    const struct mx25r_config *config = dev->config;

    uint8_t command = CMD_RDSR;
    uint8_t data_rx[2];

    data->buf_tx = (struct spi_buf){.buf = &command, .len = 1};
    data->buf_rx = (struct spi_buf){.buf = data_rx, .len = sizeof(data_rx)};

    int ret = spi_transceive_dt(&config->spi, &data->buf_set_tx, &data->buf_set_rx);
    memcpy(status, &data[1], 1);
    return ret;
}

static int write_status(const struct device *dev, uint8_t status) 
{
    struct mx25r_data *data = dev->data;
    const struct mx25r_config *config = dev->config;

    uint8_t command[2] = {CMD_WRSR, status};
    
    data->buf_tx = (struct spi_buf){.buf = command, .len = sizeof(command)};

    return spi_transceive_dt(&config->spi, &data->buf_set_tx);
}

static int read_security(const struct device *dev, uint8_t *security) 
{
    struct mx25r_data *data = dev->data;
    const struct mx25r_config *config = dev->config;

    uint8_t command = CMD_RDSCUR;
    uint8_t data_rx[2];

    data->buf_tx = (struct spi_buf){.buf = &command, .len = 1};
    data->buf_rx = (struct spi_buf){.buf = data_rx, .len = sizeof(data_rx)};

    int ret = spi_transceive_dt(&config->spi, &data->buf_set_tx, &data->buf_set_rx);
    memcpy(security, &data[1], 1);
    return ret;
}

static int write_security(const struct device *dev, uint8_t security) 
{
    struct mx25r_data *data = dev->data;
    const struct mx25r_config *config = dev->config;

    uint8_t command[2] = {CMD_WRSR, security};
    
    data->buf_tx = (struct spi_buf){.buf = command, .len = sizeof(command)};

    return spi_write_dt(&config->spi, &data->buf_set_tx);
}

static int read_id(const struct device *dev, uint8_t *id) 
{
    struct mx25r_data *data = dev->data;
    const struct mx25r_config *config = dev->config;

    uint8_t command = CMD_RDID;
    uint8_t data_rx[4];

    data->buf_tx = (struct spi_buf){.buf = &command, .len = 1};
    data->buf_rx = (struct spi_buf){.buf = data_rx, .len = sizeof(data_rx)};

    int ret = spi_transceive_dt(&config->spi, &data->buf_set_tx, &data->buf_set_rx);
    memcpy(id, &data[1], 3);
    return ret;
}

static int read_rems(const struct device *dev, uint8_t *rems) 
{
    struct mx25r_data *data = dev->data;
    const struct mx25r_config *config = dev->config;

    uint8_t command[4] = {CMD_REMS, DUMMY, DUMMY, 0x00};
    uint8_t data_rx[6];

    data->buf_tx = (struct spi_buf){.buf = command, .len = 4};
    data->buf_rx = (struct spi_buf){.buf = data_rx, .len = sizeof(data_rx)};

    int ret = spi_transceive_dt(&config->spi, &data->buf_set_tx, &data->buf_set_rx);
    memcpy(rems, &data[4], 2);
    return ret;
}

static int read_res(const struct device *dev, uint8_t *res) 
{
    struct mx25r_data *data = dev->data;
    const struct mx25r_config *config = dev->config;

    uint8_t command = CMD_RES;
    uint8_t data_rx[5];

    data->buf_tx = (struct spi_buf){.buf = &command, .len = 1};
    data->buf_rx = (struct spi_buf){.buf = data_rx, .len = sizeof(data_rx)};

    int ret = spi_transceive_dt(&config->spi, &data->buf_set_tx, &data->buf_set_rx);
    memcpy(res, &data[4], 1);
    return ret;
}

static int read_config(const struct device *dev, uint8_t *conf) 
{
    struct mx25r_data *data = dev->data;
    const struct mx25r_config *config = dev->config;

    uint8_t command = CMD_RDCR;
    uint8_t data_rx[3];

    data->buf_tx = (struct spi_buf){.buf = &command, .len = 1};
    data->buf_rx = (struct spi_buf){.buf = data_rx, .len = sizeof(data_rx)};

    int ret = spi_transceive_dt(&config->spi, &data->buf_set_tx, &data->buf_set_rx);
    memcpy(conf, &data[1], 2);
    return ret;
}

static int write_mem(const struct device *dev, uint16_t addr, uint8_t *buf, uint16_t buf_size) 
{
    struct mx25r_data *data = dev->data;
    const struct mx25r_config *config = dev->config;

    uint8_t command[4] = {  CMD_PP, 
                            (uint8_t)((addr >> 16) & 0xFF), 
                            (uint8_t)((addr >> 8) & 0xFF), 
                            (uint8_t)(addr & 0xFF)};
    uint8_t data_tx[buf_size + 4];
    memcpy(data_tx, &command, 4);
    memcpy(&data_tx[4], buf, buf_size);
    
    data->buf_tx = (struct spi_buf){.buf = data_tx, .len = sizeof(data_tx)};

    return spi_write_dt(&config->spi, &data->buf_set_tx);
}

static int read_mem(const struct device *dev, uint16_t addr, uint8_t *buf, uint16_t buf_size) 
{
    struct mx25r_data *data = dev->data;
    const struct mx25r_config *config = dev->config;

    uint8_t command[4] = {  CMD_READ,
                            (uint8_t)((addr >> 16) & 0xFF), 
                            (uint8_t)((addr >> 8) & 0xFF), 
                            (uint8_t)(addr & 0xFF)};
    uint8_t data_rx[buf_size+4];

    data->buf_tx = (struct spi_buf){.buf = command, .len = sizeof(command)};
    data->buf_rx = (struct spi_buf){.buf = data_rx, .len = sizeof(data_rx)};

    int ret = spi_transceive_dt(&config->spi, &data->buf_set_tx, &data->buf_set_rx);
    memcpy(buf, &data[4], buf_size);
    return ret;
}

static int mx25r_erase(const struct device *dev, uint8_t cmd, uint16_t addr) 
{
    struct mx25r_data *data = dev->data;
    const struct mx25r_config *config = dev->config;

    uint8_t command[4] = {  cmd, 
                            (uint8_t)((addr >> 16) & 0xFF), 
                            (uint8_t)((addr >> 8) & 0xFF), 
                            (uint8_t)(addr & 0xFF)};
    
    data->buf_tx = (struct spi_buf){.buf = command, .len = sizeof(command)};

    return spi_write_dt(&config->spi, &data->buf_set_tx);
}

static int erase_block(const struct device *dev, uint16_t addr) 
{
    return mx25r_erase(dev, CMD_BE, addr);
}

static int erase_block32(const struct device *dev, uint16_t addr) 
{
    return mx25r_erase(dev, CMD_32KBE, addr);
}

static int erase_sector(const struct device *dev, uint16_t addr) 
{
    return mx25r_erase(dev, CMD_SE, addr);
}

static int erase_chip(const struct device *dev) 
{
    return mx25r_cmd(dev, CMD_CE);
}

static int fread_mem(const struct device *dev, uint16_t addr, uint8_t *buf, uint16_t buf_size) 
{
    struct mx25r_data *data = dev->data;
    const struct mx25r_config *config = dev->config;

    uint8_t command[4] = {  CMD_FREAD,
                            (uint8_t)((addr >> 16) & 0xFF), 
                            (uint8_t)((addr >> 8) & 0xFF), 
                            (uint8_t)(addr & 0xFF)};
    uint8_t data_rx[buf_size+5];

    data->buf_tx = (struct spi_buf){.buf = command, .len = sizeof(command)};
    data->buf_rx = (struct spi_buf){.buf = data_rx, .len = sizeof(data_rx)};

    int ret = spi_transceive_dt(&config->spi, &data->buf_set_tx, &data->buf_set_rx);
    memcpy(buf, &data[5], buf_size);
    return ret;
}

static bool is_writing(const struct device *dev)
{
    uint8_t status;
    read_status(dev, &status);
    return status&0x01 ? true : false;
}

static const struct mx25r_api api = {
    .mx25r_write_enable = &write_enable,
    .mx25r_write_disable = &write_disable,
    .mx25r_reset_enable = &reset_enable,
    .mx25r_reset = &reset,
    .mx25r_read_status = &read_status,
    .mx25r_write_status = &write_status,
    .mx25r_read_security = &read_security,
    .mx25r_write_security = &write_security,
    .mx25r_read_id = &read_id,
    .mx25r_read_rems = &read_rems,
    .mx25r_read_res = &read_res,
    .mx25r_read_config = &read_config,
    .mx25r_write_mem = &write_mem,
    .mx25r_read_mem = &read_mem,
    .mx25r_erase_block = &erase_block,
    .mx25r_erase_block32 = &erase_block32,
    .mx25r_erase_sector = &erase_sector,
    .mx25r_erase_chip = &erase_chip,
    .mx25r_fread_mem = &fread_mem,
    .mx25r_is_writing = &is_writing
};

static struct mx25r_data data;

static int mx25r_init(const struct device *dev)
{
    struct mx25r_data *data = dev->data;
    const struct mx25r_config *config = dev->config;

    return 0;
}

#define CISTEME_MX25R_DEFINE(inst)                                                  \
    static const const struct mx25r_config mx25r_config_##inst = {                        \
        .spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),   \
    };                                                                              \
    DEVICE_DT_INST_DEFINE(inst,                                                     \
                          mx25r_init,                                               \
                          NULL,                                                     \
                          &data,                                                    \
                          &mx25r_config_##inst,                                     \
                          POST_KERNEL,                                              \
                          90,                                                       \
                          &api);

DT_INST_FOREACH_STATUS_OKAY(CISTEME_MX25R_DEFINE)