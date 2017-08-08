NAME := apps_scu_iot_projects_application_secure_subscriber_publisher

$(NAME)_SOURCES := secure_subscriber_publisher.c

$(NAME)_COMPONENTS := protocols/MQTT

WIFI_CONFIG_DCT_H := wifi_config_dct.h

$(NAME)_RESOURCES := apps/scu_mqtt_server/rootca.cer