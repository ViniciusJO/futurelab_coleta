#ifndef __WIFI_H__
#define __WIFI_H__

#include "esp_event.h"

void wifi_init_sta(void);
void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

#endif //__WIFI_H__
