idf_component_register(SRCS "main.c"
                    REQUIRES "devunit" "esp_https_ota" "app_update" "mbedtls" "esp_timer"
                    INCLUDE_DIRS ".")
