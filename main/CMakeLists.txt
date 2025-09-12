idf_component_register(SRCS "cmd_storage.c" "pulse_counter.c" "motor.c" "rtc.c" "sd_card.c" "rgb_led.c" "main.c" "ble.c"
                    INCLUDE_DIRS ".")

target_link_libraries(${COMPONENT_LIB} m)