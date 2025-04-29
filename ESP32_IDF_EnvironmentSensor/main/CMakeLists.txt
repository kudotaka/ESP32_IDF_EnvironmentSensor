idf_component_register(SRCS "main.c"
                    REQUIRES "devunit" "esp_https_ota" "app_update" "mbedtls"
                    INCLUDE_DIRS ".")
