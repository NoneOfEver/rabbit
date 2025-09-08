idf_component_register(SRCS "rtc.c" "sd_card.c" "rgb_led.c" "motor.c" "main.c" "ble.c"
                    INCLUDE_DIRS ".")

target_link_libraries(${COMPONENT_LIB} m)