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

/**
 * @brief MX25R Interface
 * @defgroup mx25r_interface MX25R Interface
 * @since 1.0
 * @version 1.0.0
 * @ingroup io_interfaces
 * @{
 */
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include "cisteme_mx25r_regmap.h"

/** Default SPI Config : 8bit words, MSB first, CPOL, CPHA */
#define SPIOP	SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA

/**
 * @brief MX25R Interface Configuration.
 * @param spi is the used SPI bus
 */
struct mx25r_config {
    struct spi_dt_spec spi;
};

/**
 * @brief MX25R Interface Data
 * 
 */
struct mx25r_data {
    /** SPI TX data buffer */
    struct spi_buf buf_tx;

    /** SPI RX data buffer */
    struct spi_buf buf_rx;

    /** SPI TX buffer set */
    struct spi_buf_set buf_set_tx;

    /** SPI RX buffer set */
    struct spi_buf_set buf_set_rx;
};

/**
 * @cond INTERNAL_HIDDEN
 *
 * These are for internal use only, so skip these in
 * public documentation.
 */
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
/**
 * @endcond
 *
 */

/**
 * @brief Enable the Write Enable Bit (WEB) in status register.
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @retval spi_write_dt return value.
 */
__syscall int mx25r_write_enable(const struct device *dev);

static inline int z_impl_mx25r_write_enable(const struct device *dev)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_write_enable == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_write_enable(dev);
}

/**
 * @brief Disable the Write Enable Bit (WEB) in status register.
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @retval spi_write_dt return value.
 */
__syscall int mx25r_write_disable(const struct device *dev);

static inline int z_impl_mx25r_write_disable(const struct device *dev)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_write_disable == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_write_disable(dev);
}

/**
 * @brief Enable the Reset Enable Latch.
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @retval spi_write_dt return value.
 */
__syscall int mx25r_reset_enable(const struct device *dev);

static inline int z_impl_mx25r_reset_enable(const struct device *dev)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_reset_enable == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_reset_enable(dev);
}

/**
 * @brief Reset device.
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @retval spi_write_dt return value.
 */
__syscall int mx25r_reset(const struct device *dev);

static inline int z_impl_mx25r_reset(const struct device *dev)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_reset == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_reset(dev);
}

/**
 * @brief Read device Status Register (SR).
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @param status Pointer to 1 byte array.
 * @retval spi_transceive_dt return value.
 */
__syscall int mx25r_read_status(const struct device *dev, uint8_t *status);

static inline int z_impl_mx25r_read_status(const struct device *dev, uint8_t *status)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_read_status == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_read_status(dev, status);
}

/**
 * @brief Write device Status Register (SR).
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @param status Status register value.
 * @retval spi_write_dt return value.
 */
__syscall int mx25r_write_status(const struct device *dev, uint8_t status);

static inline int z_impl_mx25r_write_status(const struct device *dev, uint8_t status)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_write_status == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_write_status(dev,status);
}

/**
 * @brief Read device Security Register (SCUR).
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @param security Pointer to 1 byte array.
 * @retval spi_transceive_dt return value.
 */
__syscall int mx25r_read_security(const struct device *dev, uint8_t *security);

static inline int z_impl_mx25r_read_security(const struct device *dev, uint8_t *security)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_read_security == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_read_security(dev, security);
}

/**
 * @brief Write device Security Register (SCUR).
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @param security Security register value
 * @retval spi_write_dt return value.
 */
__syscall int mx25r_write_security(const struct device *dev, uint8_t security);

static inline int z_impl_mx25r_write_security(const struct device *dev, uint8_t security)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_write_security == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_write_security(dev, security);
}

/**
 * @brief Read 3-bytes device ID.
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @param id Pointer to 3 bytes array
 * @retval spi_transceive_dt return value.
 */
__syscall int mx25r_read_id(const struct device *dev, uint8_t *id);

static inline int z_impl_mx25r_read_id(const struct device *dev, uint8_t *id)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_read_id == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_read_id(dev, id);
}

/**
 * @brief Read 2 bytes device and manufacturer descriptions.
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @param rems Pointer to 2 bytes array.
 * @retval spi_transceive_dt return value.
 */
__syscall int mx25r_read_rems(const struct device *dev, uint8_t *rems);

static inline int z_impl_mx25r_read_rems(const struct device *dev, uint8_t *rems)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_read_rems == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_read_rems(dev, rems);
}

/**
 * @brief Read 1 byte device manufacturer description.
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @param res Pointer to 1 byte array
 * @retval spi_transceive_dt return value.
 */
__syscall int mx25r_read_res(const struct device *dev, uint8_t *res);

static inline int z_impl_mx25r_read_res(const struct device *dev, uint8_t *res)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_read_res == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_read_res(dev, res);
}

/**
 * @brief Read device Configuration Register (CR).
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @param conf Pointer to 1 byte array
 * @retval spi_transceive_dt return value.
 */
__syscall int mx25r_read_config(const struct device *dev, uint8_t *conf);

static inline int z_impl_mx25r_read_config(const struct device *dev, uint8_t *conf)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_read_config == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_read_config(dev, conf);
}

/**
 * @brief Write memory using Page Program (PP) instruction.
 * 
 * @note Require enabling the Write Enable Latch (see mx25r_write_enable)
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @param addr 3 bytes address to write.
 * @param buf Pointer to data buffer.
 * @param buf_size Data buffer size in bytes.
 * @retval spi_write_dt return value.
 */
__syscall int mx25r_write_mem(const struct device *dev, uint16_t addr, uint8_t *data, uint16_t data_size);

static inline int z_impl_mx25r_write_mem(const struct device *dev, uint16_t addr, uint8_t *data, uint16_t data_size)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_write_mem == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_write_mem(dev, addr, data, data_size);
}

/**
 * @brief Read memory using Read instruction.
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @param addr 3 bytes address to read.
 * @param buf Pointer to data buffer.
 * @param buf_size Number of bytes to read.
 * @retval spi_transceive_dt return value.
 */
__syscall int mx25r_read_mem(const struct device *dev, uint16_t addr, uint8_t *data, uint16_t data_size);

static inline int z_impl_mx25r_read_mem(const struct device *dev, uint16_t addr, uint8_t *data, uint16_t data_size)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_read_mem == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_read_mem(dev, addr, data, data_size);
}

/**
 * @brief Erase 64KB data block
 * 
 * @note Require enabling the Write Enable Latch (see mx25r_write_enable)
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @param addr 3 bytes block address.
 * @retval spi_write_dt return value.
 */
__syscall int mx25r_erase_block(const struct device *dev, uint16_t addr);

static inline int z_impl_mx25r_erase_block(const struct device *dev, uint16_t addr)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_erase_block == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_erase_block(dev, addr);
}

/**
 * @brief Erase 32KB data block
 * 
 * @note Require enabling the Write Enable Latch (see mx25r_write_enable)
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @param addr 3 bytes block address.
 * @retval spi_write_dt return value.
 */
__syscall int mx25r_erase_block32(const struct device *dev, uint16_t addr);

static inline int z_impl_mx25r_erase_block32(const struct device *dev, uint16_t addr)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_erase_block32 == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_erase_block32(dev, addr);
}

/**
 * @brief Erase 4KB data sector
 * 
 * @note Require enabling the Write Enable Latch (see mx25r_write_enable)
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @param addr 3 bytes sector address.
 * @retval spi_write_dt return value.
 */
__syscall int mx25r_erase_sector(const struct device *dev, uint16_t addr);

static inline int z_impl_mx25r_erase_sector(const struct device *dev, uint16_t addr)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_erase_sector == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_erase_sector(dev, addr);
}

/**
 * @brief Erase whole chip data.
 * 
 * @note Require enabling the Write Enable Latch (see mx25r_write_enable)
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @retval spi_write_dt return value.
 */
__syscall int mx25r_erase_chip(const struct device *dev);

static inline int z_impl_mx25r_erase_chip(const struct device *dev)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_erase_chip == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_erase_chip(dev);
}

/**
 * @brief Read memory using Fast-Read command.
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @param addr 3 bytes address to read.
 * @param buf Pointer to data buffer.
 * @param buf_size Number of bytes to read.
 * @retval spi_transceive_dt return value.
 */
__syscall int mx25r_fread_mem(const struct device *dev, uint16_t addr, uint8_t *data, uint16_t data_size);

static inline int z_impl_mx25r_fread_mem(const struct device *dev, uint16_t addr, uint8_t *data, uint16_t data_size)
{
    const struct mx25r_api *api = (const struct mx25r_api *)dev->api;
    if (api->mx25r_fread_mem == NULL) {
        return -ENOSYS;
    }
    return api->mx25r_fread_mem(dev, addr, data, data_size);
}

/**
 * @brief Check Write In Progress bit from Status Register
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * @retval true when device is busy writing.
 * @retval false when device is available.
 */
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