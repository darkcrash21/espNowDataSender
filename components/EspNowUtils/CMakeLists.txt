set(SOURCES
    esp_now_utils.cpp
)

set(INCLUDES
    .
)

set(REQS
    esp_event
    esp_netif
    esp_wifi
)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS ${INCLUDES}
    REQUIRES ${REQS}
)
