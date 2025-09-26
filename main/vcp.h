#ifndef __VCP_H__
#define __VCP_H__
#include "usb/usb_types_cdc.h"
#ifdef __cplusplus
extern "C" {
#endif//__cplusplus

#include <usb/cdc_acm_host.h>

// Opaque: cpp class
typedef struct CdcAcmDevice CdcAcmDevice;

void register_ch34x();

CdcAcmDevice* vcp_open(uint32_t PID, uint32_t VID, cdc_acm_host_device_config_t *dev_config);
esp_err_t vcp_line_coding_set(CdcAcmDevice *dev, cdc_acm_line_coding_t *line_coding);
esp_err_t vcp_tx_blocking(CdcAcmDevice *dev, uint8_t *msg, size_t msg_len);
esp_err_t vcp_set_control_line_state(CdcAcmDevice* dev, bool dtr, bool rts);

#ifdef __cplusplus
}
#endif//__cplusplus
#endif//__VCP_H__
