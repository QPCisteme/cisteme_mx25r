/**
 * @file cisteme_mx25r.h
 * @author Quentin PERROCHE (perroche@cisteme.net)
 * @brief CISTEME MX25R NOR Flash driver functions header
 * @version 0.1
 * @date 2025-06-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include "cisteme_mx25r_regmap.h"

struct mx25r_config {
    struct spi_dt_spec spi;
};

struct mx25r_data {
    struct spi_buf buf_tx;
    struct spi_buf buf_rx;

    struct spi_buf_set buf_set_tx;
    struct spi_buf_set buf_set_rx;
};

__subsystem struct mx25r_api {
    int (*mx25r_write_enable)(const struct device *dev);
    int (*mx25r_write_disable)(const struct device *dev);

    int (*mx25r_reset_enable)(const struct device *dev);
    int (*mx25r_reset)(const struct device *dev);

    int (*mx25r_read_status)(const struct device *dev, uint8_t *status);
    int (*mx25r_write_status)(const struct device *dev, uint8_t status);

    int (*mx25r_read_security)(const struct device *dev, uint8_t *security);
    int (*mx25r_write_security)(const struct device *dev, uint8_t security);

    int (*mx25r_read_id)(const struct device *dev, uint8_t *id);
    int (*mx25r_read_rems)(const struct device *dev, uint8_t *rems);
    int (*mx25r_read_res)(const struct device *dev, uint8_t *res);
    int (*mx25r_read_config)(const struct device *dev, uint8_t *config);

    int (*mx25r_write_mem)(const struct device *dev, uint16_t addr, uint8_t *data, uint16_t data_size);
    int (*mx25r_read_mem)(const struct device *dev, uint16_t addr, uint8_t *data, uint16_t data_size);

    int (*mx25r_erase_block)(const struct device *dev, uint16_t addr);
    int (*mx25r_erase_block32)(const struct device *dev, uint16_t addr);
    int (*mx25r_erase_sector)(const struct device *dev, uint16_t addr);
    int (*mx25r_erase_chip)(const struct device *dev);

    int (*mx25r_fread_mem)(const struct device *dev, uint16_t addr, uint8_t *data, uint16_t data_size);
    bool (*mx25r_is_writing)(const struct device *dev);
};

__syscall int mx25r_write_enable(const struct device *dev);

static inline int z_impl_mx25r_write_enable(const struct device *dev)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_write_enable == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_write_enable(dev);
}

__syscall int mx25r_write_disable(const struct device *dev);

static inline int z_impl_mx25r_write_disable(const struct device *dev)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_write_disable == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_write_disable(dev);
}

__syscall int mx25r_reset_enable(const struct device *dev);

static inline int z_impl_mx25r_reset_enable(const struct device *dev)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_reset_enable == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_reset_enable(dev);
}

__syscall int mx25r_reset(const struct device *dev);

static inline int z_impl_mx25r_reset(const struct device *dev)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_reset == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_reset(dev);
}

__syscall int mx25r_read_status(const struct device *dev, uint8_t *status);

static inline int z_impl_mx25r_read_status(const struct device *dev, uint8_t *status)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_read_status == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_read_status(dev, status);
}

__syscall int mx25r_write_status(const struct device *dev, uint8_t status);

static inline int z_impl_mx25r_write_status(const struct device *dev, uint8_t status)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_write_status == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_write_status(dev,status);
}

__syscall int mx25r_read_security(const struct device *dev, uint8_t *security);

static inline int z_impl_mx25r_read_security(const struct device *dev, uint8_t *security)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_read_security == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_read_security(dev, security);
}

__syscall int mx25r_write_security(const struct device *dev, uint8_t security);

static inline int z_impl_mx25r_write_security(const struct device *dev, uint8_t security)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_write_security == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_write_security(dev, security);
}

__syscall int mx25r_read_id(const struct device *dev, uint8_t *id);

static inline int z_impl_mx25r_read_id(const struct device *dev, uint8_t *id)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_read_id == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_read_id(dev, id);
}

__syscall int mx25r_read_rems(const struct device *dev, uint8_t *rems);

static inline int z_impl_mx25r_read_rems(const struct device *dev, uint8_t *rems)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_read_rems == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_read_rems(dev, rems);
}

__syscall int mx25r_read_res(const struct device *dev, uint8_t *res);

static inline int z_impl_mx25r_read_res(const struct device *dev, uint8_t *res)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_read_res == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_read_res(dev, res);
}

__syscall int mx25r_read_config(const struct device *dev, uint8_t *conf);

static inline int z_impl_mx25r_read_config(const struct device *dev, uint8_t *conf)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_read_config == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_read_config(dev, conf);
}

__syscall int mx25r_write_mem(const struct device *dev, uint16_t addr, uint8_t *data, uint16_t data_size);

static inline int z_impl_mx25r_write_mem(const struct device *dev, uint16_t addr, uint8_t *data, uint16_t data_size)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_write_mem == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_write_mem(dev, addr, data, data_size);
}

__syscall int mx25r_read_mem(const struct device *dev, uint16_t addr, uint8_t *data, uint16_t data_size);

static inline int z_impl_mx25r_read_mem(const struct device *dev, uint16_t addr, uint8_t *data, uint16_t data_size)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_read_mem == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_read_mem(dev, addr, data, data_size);
}

__syscall int mx25r_erase_block(const struct device *dev, uint16_t addr);

static inline int z_impl_mx25r_erase_block(const struct device *dev, uint16_t addr)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_erase_block == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_erase_block(dev, addr);
}

__syscall int mx25r_erase_block32(const struct device *dev, uint16_t addr);

static inline int z_impl_mx25r_erase_block32(const struct device *dev, uint16_t addr)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_erase_block32 == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_erase_block32(dev, addr);
}

__syscall int mx25r_erase_sector(const struct device *dev, uint16_t addr);

static inline int z_impl_mx25r_erase_sector(const struct device *dev, uint16_t addr)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_erase_sector == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_erase_sector(dev, addr);
}

__syscall int mx25r_erase_chip(const struct device *dev);

static inline int z_impl_mx25r_erase_chip(const struct device *dev)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_erase_chip == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_erase_chip(dev);
}

__syscall int mx25r_fread_mem(const struct device *dev, uint16_t addr, uint8_t *data, uint16_t data_size);

static inline int z_impl_mx25r_fread_mem(const struct device *dev, uint16_t addr, uint8_t *data, uint16_t data_size)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_fread_mem == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_fread_mem(dev, addr, data, data_size);
}

__syscall int mx25r_is_writing(const struct device *dev);

static inline int z_impl_mx25r_is_writing(const struct device *dev)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_is_writing == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_is_writing(dev);
}

#include <syscalls/cisteme_mx25r.h>