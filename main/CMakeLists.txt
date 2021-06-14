# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

list(APPEND COMPONENT_ADD_INCLUDEDIRS "./external_driver2")
list(APPEND COMPONENT_ADD_INCLUDEDIRS "./include")

list(APPEND COMPONENT_SRCS "main.c")
list(APPEND COMPONENT_SRCS "./external_driver/bme280.c")
list(APPEND COMPONENT_SRCS "./include/i2c.c")

register_component()
