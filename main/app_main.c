// #include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <esp_event.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <freertos/idf_additions.h>
#include <nvs_flash.h>
#include <inttypes.h>
#include <esp_log.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include <soc/gpio_num.h>
#include <hal/gpio_types.h>

#include "colors.h"
#include "mqtt_client.h"
#include "wifi.h"
#include "bno055.h"
#include "neopixel.h"
#include "vcp.h"

#define DECODER_IMPLEMENTATIONS
#include "decoder.h"

#define ESC "\x1B"
#define ESC_CLEAR_LINE ESC"[2K"
#define ESC_UP_LINE(NUM) ESC"["#NUM"A"

#define delay_ms(ms) vTaskDelay(pdMS_TO_TICKS(ms))

#define LOG(STR) puts(COLOR_CYAN STR COLOR_RESET);
#define LOGF(FMT,...) printf(COLOR_CYAN FMT COLOR_RESET "\n",__VA_ARGS__);

// void assert_true(bool cond) { assert(cond); }
// #define assert_true assert
void _assert_true(bool cond, const char * const cond_str, size_t line) {
  if(cond) {
    printf(COLOR_RED"ERROR: " COLOR_RESET __FILE__":%lu:0: %s\n", (unsigned long)line, cond_str);
    exit(-1);
  }
}
#define assert_true(CONDITION) _assert_true(CONDITION, #CONDITION, __LINE__)

static const char *TAG = "Futurelab_Coleta";

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     1
#define I2C_SCL_PIN     2
#define I2C_FREQ_HZ     400000

/*#define WIFI_SSID "FL2.4"*/
/*#define WIFI_PASS "W3LC0M3T0TH3FUTURE"*/
#define WIFI_SSID "elm"
#define WIFI_PASS "senhadowifi"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define BROKER_URL "mqtt://mqtt.viniciusjo.eng.br"
#define MQTT_BROKER_URI BROKER_URL
#define MQTT_TOPIC "elm327/test"

#define MQTT_SEND_INTERVAL 1000
#define ACCEL_SAMPLE_INTERVAL 100

#define PID_LIST      \
    X(RPM, 0C)        \
    X(SPEED, 0D)      \
    X(PEDAL_POS, 11)  \
    X(FUEL_RATE, 5E)  \
    X(FUEL_TYPE, 51)  \

static char *pid_name[256] = {0};
void init_pid_names() {
#define X(name, pid) pid_name[0x##pid] = #name;
  PID_LIST
#undef X
}

#define X(name, pid) PID_##pid,
enum { PID_LIST PIDS_COUNT, };
#undef X

// #define X(name, pid) 0x01##pid,
// static const uint16_t pids[PIDS_COUNT] = { PID_LIST };
// #undef X

#define TO_STRING(VAL) #VAL

#define X(name, pid) TO_STRING(01##pid),
static const char *pids_ascii[PIDS_COUNT] = { PID_LIST };
#undef X

#define X(name, pid) const uint8_t PID_##name = 0x##pid;
PID_LIST
#undef X

typedef struct {
  float accel_x;
  float accel_y;
  float rpm;
  float speed;
  float fuel_rate;
  float pedal_pos;
} Features;
Features feats = {0};

void print_features(Features f) {
  printf("feats = {\n"
     "	.accel_x = %.4f,\n"
     "	.accel_y = %.4f,\n"
     "	.rpm = %.4f,\n"
     "	.speed = %.4f,\n"
     "	.fuel_rate = %.4f,\n"
     "	.pedal_pos = %.4f,\n"
     "}\n",
     f.accel_x,
     f.accel_y,
     f.rpm,
     f.speed,
     f.fuel_rate,
     f.pedal_pos
  );
}

bno055_9dof_t bno055_data = {0};

extern EventGroupHandle_t s_wifi_event_group;

int mqtt_connected = 0;
esp_mqtt_client_handle_t client;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  /*esp_mqtt_event_handle_t event = event_data;*/
  switch (event_id) {
    case MQTT_EVENT_CONNECTED:
      ESP_LOGI(TAG, "MQTT connected");
      /*esp_mqtt_client_publish(event->client, "/test/topic", "Hello from ESP32-S3", 0, 1, 0);*/
      break;
    case MQTT_EVENT_DISCONNECTED:
      ESP_LOGI(TAG, "MQTT disconnected");
      break;
    default:
      break;
  }
}

// static void wifi_init(void) {
//   esp_netif_init();
//   esp_event_loop_create_default();
//   esp_netif_create_default_wifi_sta();
//
//   wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//   esp_wifi_init(&cfg);
//
//   esp_wifi_set_mode(WIFI_MODE_STA);
//   wifi_config_t wifi_config = {
//     .sta = {
//       .ssid = WIFI_SSID,
//       .password = WIFI_PASS,
//     },
//   };
//   esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
//   esp_wifi_start();
//   esp_wifi_connect();
// }

static esp_mqtt_client_handle_t mqtt_app_start(void) {
  LOG("MQTT starting...");
  esp_mqtt_client_config_t mqtt_cfg = {
    .broker.address.uri = MQTT_BROKER_URI
  };

  esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
  if(client == NULL) {
    perror("Could not initialize client");
    return NULL;
  }
  /* The last argument may be used to pass data to the event handler, in this
   * example mqtt_event_handler */
  esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
  /*(esp_mqtt_client_subscribe_single(client, "elm327", 0));*/
  esp_mqtt_client_start(client);
  LOG("MQTT end");

  return client;
}

esp_err_t i2c_master_init() {
  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_SDA_PIN,
    .scl_io_num = I2C_SCL_PIN,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_FREQ_HZ,
  };
  esp_err_t ret = i2c_param_config(I2C_PORT, &conf);
  if (ret != ESP_OK) return ret;
  return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

#define K ((MQTT_SEND_INTERVAL)/(ACCEL_SAMPLE_INTERVAL))
float accel_x[K] = {0};
float accel_y[K] = {0};
size_t accel_cursor = 0;

static void accell_task(void *args) {
  (void)args;

  while(1) {
    if(bno055_read_9dof(&bno055_data) == ESP_OK) {
      // TODO: implement moving average or peak hold
      accel_x[accel_cursor] = bno055_data.accel_x;
      accel_y[accel_cursor] = bno055_data.accel_y;
      accel_cursor = (accel_cursor + 1) % K;
    }
    delay_ms(ACCEL_SAMPLE_INTERVAL);
  }
}

static float get_accell_x() {
  double sum = 0;
  for(int i = 0; i < K; i++) sum += accel_x[i];
  return sum / (int)K;
}

static float get_accell_y() {
  double sum = 0;
  for(int i = 0; i < K; i++) sum += accel_y[i];
  return sum / (int)K;
}

#undef K

static void mqtt_send_task(void *args) {
  esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)args;

  char buff[512] = {0};
  while(1) {
    sprintf(buff, "{ "
      "\"origin\": \"esp\", "
      "\"accel_x\": %f, "
      "\"accel_y\": %f, "
      "\"rpm\": %f, "
      "\"speed\": %f, "
      "\"fuel_usage\": %f, "
      "\"pedal_position\": %f "
      "}",
      get_accell_x(),
      get_accell_y(),
      feats.rpm,
      feats.speed,
      feats.fuel_rate,
      feats.pedal_pos
    );
    printf(COLOR_CYAN"%s\n"COLOR_RESET, buff);
    esp_mqtt_client_publish(client, MQTT_TOPIC, buff, 0, 1, 0);
    delay_ms(MQTT_SEND_INTERVAL);
  }
}

void wifi_init_sta(void) {
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &event_handler,
        NULL,
        &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &event_handler,
        NULL,
        &instance_got_ip));

  wifi_config_t wifi_config = {
    .sta = {
      .ssid = WIFI_SSID,
      .password = WIFI_PASS,
      .threshold.authmode = WIFI_AUTH_WPA2_PSK
    },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
  ESP_ERROR_CHECK(esp_wifi_start() );

  ESP_LOGI(TAG, "wifi_init_sta finished.");

  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", WIFI_SSID, WIFI_PASS);
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", WIFI_SSID, WIFI_PASS);
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
}

static void usb_lib_task(void *arg) {
  while (1) {
    // Start handling system events
    uint32_t event_flags;
    usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      ESP_ERROR_CHECK(usb_host_device_free_all());
    }
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
      puts("USB: All devices freed");
      // Continue handling USB events to allow device reconnection
    }
  }
}

static SemaphoreHandle_t device_disconnected_sem;

static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx) {
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        printf("CDC-ACM error has occurred, err_no = %d\n", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        puts("Device suddenly disconnected");
        xSemaphoreGive(device_disconnected_sem);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        printf("Serial state notif 0x%04X\n", event->data.serial_state.val);
        break;
    case CDC_ACM_HOST_NETWORK_CONNECTION:
    default: break;
    }
}

uint8_t char_to_byte(char c) {
  if(c >= '0' && c <= '9') return c - '0';
  if(c >= 'A' && c <= 'F') return c - 'A';
  if(c >= 'a' && c <= 'f') return c - 'a';
  return 0;
}

// TODO: replace boolean by an mutex
bool wait_elm_response = false;


void print_data_vector(char * const name, const uint8_t * const data, size_t data_len) {
  printf("%s: [ ", name);
  for(int i = 0; i < data_len; i++) printf("%X%s ", data[i], i == data_len - 1 ? "" : ",");
  printf("] = [ ");
  for(int i = 0; i < data_len; i++) printf("%d%s ", data[i], i == data_len - 1 ? "" : ",");
  printf("] = [ ");
  for(int i = 0; i < data_len; i++) printf("%c%s ", data[i], i == data_len - 1 ? "" : ",");
  printf("]\n" COLOR_RESET);
}
#define print_bytes(data, data_len) print_data_vector(#data, (uint8_t*)data, data_len)

/**
 * @brief Data received callback
 *
 * @param[in] data     Pointer to received data
 * @param[in] data_len Length of received data in bytes
 * @param[in] arg      Argument we passed to the device open function
 * @return
 *   true:  We have processed the received data
 *   false: We expect more data
 */
static bool handle_rx(const uint8_t *data, size_t data_len, void *arg) {
  if(
    data_len < 3 ||
    '\r' != data[data_len-3] ||
    '\r' != data[data_len-2] ||
    '>'  != data[data_len-1]
  ) return false;

  // Remove trailing "\r\r>"
  data_len -= 3;

  // Signals packet received
  wait_elm_response = false;

  print_data_vector(COLOR_YELLOW"Payload" COLOR_RESET, data, data_len);

  // Ignore payload on wrong shape of header and payload
  if(data_len <= 4 || (data_len & 1) ^ 0) return true;

  const uint8_t payload_len = data_len/2;

  uint8_t * const payload_ref = (uint8_t*)malloc(payload_len);

  // Converts ascii representation to numeric representation
  uint8_t *payload = payload_ref;
  for(uint8_t i = 0; i < payload_len; i++)
    payload[i] = data[i*2] << 8 | data[i*2 + 1];

  // Ignores payload if not response to service 0x01 pid (!= 0x41XX...)
  if(0x41 != payload[0]) {
    free(payload_ref);
    return true;
  }

  const uint16_t received_pid = payload[1];

  // Skips header (0x41XX)
  payload += 2;

  switch(received_pid) {
    case PID_RPM: {
      if(payload_len < 2) goto invalid_payload;
      uint8_t d[] = {payload[0], payload[1]};
      feats.rpm = rpm_decoder(d);
    } break;
    case PID_SPEED: {
      if(payload_len < 1) goto invalid_payload;
      feats.speed = speed_decoder((uint8_t*)payload);
    } break;
    case PID_PEDAL_POS: {
      if(payload_len < 1) goto invalid_payload;
      feats.pedal_pos = pedal_pos_decoder((uint8_t*)payload);
    } break;
    case PID_FUEL_RATE: {
      if(payload_len < 2) goto invalid_payload;
      uint8_t d[] = {payload[0], payload[1]};
      feats.fuel_rate = fuel_rate_decoder(d);
    } break;
    default: {
      if(pid_name[received_pid & 0xFF] == NULL)
        fprintf(stderr, "PID [0x%X] not suported\n", received_pid);
      else
        fprintf(stderr, "Decode for feature \"%s\" [%02X] not registered\n", pid_name[received_pid & 0xFF], received_pid);

      free(payload_ref);
      return true;
    }
  }

  print_features(feats);

  free(payload_ref);
  return true;

invalid_payload:
  {
    // print_data_vector(COLOR_YELLOW"Invalid payload" COLOR_RESET, data, data_len);
    const uint8_t * const invalid_payload = data;
    print_bytes(invalid_payload, data_len);
  }

  free(payload_ref);
  return true;
}

esp_err_t _send_elm_command(CdcAcmDevice *dev, const char * const cmd, bool wait, bool log) {
  while(wait_elm_response) vTaskDelay(100);

  const uint8_t msg_len = strlen(cmd) + 1;
  char * const msg = (char*)malloc(msg_len);
  sprintf(msg, "%s\r", cmd);
  if(log) printf(">> %s\n", msg);
  esp_err_t err = vcp_tx_blocking(dev, (uint8_t*)msg, msg_len);
  // ESP_ERROR_CHECK(vcp_set_control_line_state(true, true));
  if(wait) wait_elm_response = true;
  free(msg);
  return err;
}
#define send_elm_cmd(vcp, cmd) _send_elm_command(vcp, cmd, false, false)
#define send_elm_cmd_wait(vcp, cmd) _send_elm_command(vcp, cmd, true, false)
#define send_elm_cmd_debug(vcp, cmd) _send_elm_command(vcp, cmd, false, true)
#define send_elm_cmd_debug_wait(vcp, cmd) _send_elm_command(vcp, cmd, true, true)

void usb_elm327_init(CdcAcmDevice *vcp) {
  #define CMD_COUNT 7
  char *start_cmds[CMD_COUNT] = {
    (char*)"ATZ",
    (char*)"ATE1",
    (char*)"ATL0",
    (char*)"ATS0",
    (char*)"ATH0",
    (char*)"ATSP0",
    (char*)"ATST 0F",
  };

  for(uint8_t i = 0; i < CMD_COUNT; i++) {
    esp_err_t r = send_elm_cmd_debug_wait(vcp, start_cmds[i]);
    assert_true(ESP_OK == r);
    // TODO: remove delay when wait on send_elm_cmd is functional
    delay_ms(1000);
  }
};

static void elm327_query_task(void *args) {
  CdcAcmDevice *vcp = (CdcAcmDevice*)args;
  assert_true(vcp);
  uint8_t i = 0;

  send_elm_cmd_debug_wait(vcp, "AT IGN");
  
  while(1) {
    send_elm_cmd_debug_wait(vcp, pids_ascii[i]);
    i = (i + 1) % PIDS_COUNT;
    vTaskDelay(100);
  }
}

void app_main(void) {
  esp_log_level_set("*", ESP_LOG_NONE);
  esp_log_level_set("wifi", ESP_LOG_DEBUG);
  esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
  esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
  esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
  esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
  esp_log_level_set("transport", ESP_LOG_VERBOSE);
  esp_log_level_set("outbox", ESP_LOG_VERBOSE);

  ESP_ERROR_CHECK(nvs_flash_init());

  // ========= Neopixel ==========

  gpio_config_t led_config = {
    .pin_bit_mask = (1ULL << GPIO_NUM_2),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&led_config);
  gpio_set_level(GPIO_NUM_2, false);

  tNeopixelContext np_ctx = neopixel_Init(1, 48);
  if(NULL == np_ctx) {
    fprintf(stderr, "Error initializing neopixel");
    exit(1);
  };
  tNeopixel np = { 0, 0x0000FF };
  neopixel_SetPixel(np_ctx, &np, 1);

  // ========= WIFI ==========

  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

  wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS, .threshold.authmode = WIFI_AUTH_WPA2_PSK } };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_sta finished.");

  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
  if (bits & WIFI_CONNECTED_BIT) {
    LOGF("connected to ap SSID:%s password:%s", WIFI_SSID, WIFI_PASS);
    gpio_set_level(GPIO_NUM_2, true);
    np.rgb = 0x00FF00;
    neopixel_SetPixel(np_ctx, &np, 1);
  } else if (bits & WIFI_FAIL_BIT) {
    LOGF("Failed to connect to SSID:%s, password:%s", WIFI_SSID, WIFI_PASS);
    gpio_set_level(GPIO_NUM_2, false);
    np.rgb = 0xFF0000;
    neopixel_SetPixel(np_ctx, &np, 1);
    delay_ms(1000);
  } else {
    LOG("UNEXPECTED EVENT");
    gpio_set_level(GPIO_NUM_2, false);
    np.rgb = 0xFF0000;
    neopixel_SetPixel(np_ctx, &np, 1);
    delay_ms(1000);
  }

  // ========= MQTT ==========

  esp_mqtt_client_handle_t mqtt_client = mqtt_app_start();

  // ========= Accelerometer ==========
  
  ESP_ERROR_CHECK(i2c_master_init());
  ESP_ERROR_CHECK(bno055_init(I2C_PORT));

  // ========= USB ==========

  device_disconnected_sem = xSemaphoreCreateBinary();
  assert_true(device_disconnected_sem);

  usb_host_config_t host_config = {
    .skip_phy_setup = false,
    .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };
  ESP_ERROR_CHECK(usb_host_install(&host_config));

  ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

  register_ch34x();

  cdc_acm_host_device_config_t dev_config = {
    .connection_timeout_ms = 5000,
    .out_buffer_size = 512,
    .in_buffer_size = 512,
    .event_cb = handle_event,
    .data_cb = handle_rx,
    .user_arg = NULL,
  };
  CdcAcmDevice *elm327 = vcp_open(0x1A86, 0x7523, &dev_config);
  assert_true(elm327 != NULL && "Failed to open ELM327");

  cdc_acm_line_coding_t line_coding = {
      .dwDTERate = 38400,
      .bCharFormat = 0,
      .bParityType = 0,
      .bDataBits = 8,
  };
  ESP_ERROR_CHECK(vcp_line_coding_set(elm327, &line_coding));

  usb_elm327_init(elm327);

  // ========= Tasks ==========

  (xTaskCreate(mqtt_send_task, "mqtt_send_task", 4096, (void*)mqtt_client, 3, NULL));
  (xTaskCreate(accell_task, "accell_task", 4096, NULL, 3, NULL));
  (xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL));
  (xTaskCreate(elm327_query_task, "elm327_query_task", 4096, (void*)elm327, 2, NULL));
}

