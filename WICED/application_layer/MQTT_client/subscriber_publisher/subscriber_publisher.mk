NAME := apps_scu_iot_projects_application_subscriber_publisher

$(NAME)_SOURCES := subscriber_publisher.c

$(NAME)_COMPONENTS := protocols/MQTT

WIFI_CONFIG_DCT_H := wifi_config_dct.h

