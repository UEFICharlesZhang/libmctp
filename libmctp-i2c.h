/* SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later */

#ifndef _LIBMCTP_I2C_H
#define _LIBMCTP_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <libmctp.h>

struct mctp_binding_i2c;

struct mctp_binding_i2c *mctp_i2c_init(uint8_t addr); // addr is 7-bits
struct mctp_binding *mctp_binding_i2c_core(struct mctp_binding_i2c *b);

void mctp_i2c_destroy(struct mctp_binding_i2c *i2c);
int mctp_i2c_rx(struct mctp_binding_i2c *i2c, uint8_t *buf, uint32_t len);

/* direct function call IO */
typedef int (*mctp_i2c_tx_fn)(const void *data, uint8_t *buf, uint32_t len)
	__attribute__((warn_unused_result));
void mctp_i2c_set_tx_fn(struct mctp_binding_i2c *i2c,
		mctp_i2c_tx_fn fn, const void *fn_data);

#ifdef MCTP_HAVE_FILEIO
/* file-based IO */
int mctp_i2c_open_fd(struct mctp_binding_i2c *i2c, uint8_t bus);
int mctp_i2c_get_fd(struct mctp_binding_i2c *i2c);
int mctp_i2c_read(struct mctp_binding_i2c *i2c);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _LIBMCTP_I2C_H */
