#include <usb/vcp_ch34x.hpp>
#include <usb/vcp.hpp>

using namespace esp_usb;

#include "vcp.h"

extern "C" {
  void register_ch34x() {
    printf("Registering CH340 vcp driver...\n");
    VCP::register_driver<CH34x>();
  }

  CdcAcmDevice* vcp_open(uint32_t VID, uint32_t PID, cdc_acm_host_device_config_t *dev_config) {
    printf("Creating VCP device (open)...\n");
    auto ret = VCP::open(VID, PID, dev_config);
    printf("vcp_handler: %p%s\n", ret, ret == NULL ? " (NULL)" : "");
    return ret;
  }

  esp_err_t vcp_line_coding_set(CdcAcmDevice *dev, cdc_acm_line_coding_t *line_coding) {
    printf("Setting VCP line coding...\n");
    return dev->line_coding_set(line_coding);
  }

  esp_err_t vcp_tx_blocking(CdcAcmDevice *dev, uint8_t *msg, size_t msg_len) {
    printf("Sending vcp_tx...\n");
    return dev->tx_blocking(msg, msg_len);
  }

  esp_err_t vcp_set_control_line_state(CdcAcmDevice* dev, bool dtr, bool rts) {
    printf("VCP control line state...\n");
    return dev->set_control_line_state(dtr, rts);
  }
}

  // const cdc_acm_host_device_config_t dev_config = {
  //   .connection_timeout_ms = 5000,
  //   .out_buffer_size = 512,
  //   .in_buffer_size = 512,
  //   .event_cb = handle_event,
  //   .data_cb = handle_rx,
  //   .user_arg = NULL,
  // };
  // CdcAcmDevice *vcp = VCP::open(0x1A86, 0x7523, &dev_config);
  // cdc_acm_line_coding_t line_coding = {
  //     .dwDTERate = 38400,
  //     .bCharFormat = 0,
  //     .bParityType = 0,
  //     .bDataBits = 8,
  // };
  // ESP_ERROR_CHECK(vcp->line_coding_set(&line_coding));
