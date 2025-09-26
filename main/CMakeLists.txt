idf_component_register(
    # SRCS "cdc_acm_vcp_example_main.cpp"
    SRCS "app_main.c" "bno055.c" "neopixel.c" "vcp.cpp" "wifi.c" 
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb nvs_flash driver mqtt esp_event esp_wifi
    )
