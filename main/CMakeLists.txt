idf_component_register(SRCS "blink_example_main.c" "adc_control.c" "led_control.c"  "wifi_app.c" "http_server.c"
                       INCLUDE_DIRS "."
                       EMBED_FILES webpage/app.css webpage/app.js webpage/favicon.ico webpage/index.html webpage/jquery-3.3.1.min.js)
