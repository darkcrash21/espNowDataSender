set(SOURCES
    main.cpp
)

set(INCLUDES
    .
    ../components/EspNowUtils/
)

set(REQS
    nvs_flash
    EspNowUtils
)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS ${INCLUDES}
    REQUIRES ${REQS}
)
