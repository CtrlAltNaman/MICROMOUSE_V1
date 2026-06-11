idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_http_server esp_wifi esp_event esp_netif nvs_flash esp_timer
)
