if(CONFIG_MICROPYTHON)
	set(MICROPY_DIR ${ZEPHYR_MICROPYTHON_MODULE_DIR})
	set(MICROPY_MODULE_DIR ${ZEPHYR_BASE}/modules/micropython)
	set(MICROPY_PORT_DIR ${MICROPY_DIR}/ports/zephyr)
	set(MICROPY_TARGET micropython)

	include(${MICROPY_DIR}/py/py.cmake)
	include(${MICROPY_DIR}/extmod/extmod.cmake)

    set(CONFIG_MICROPY_CONFIGFILE ${MICROPY_MODULE_DIR}/mpconfigport.h)
	set(MICROPY_SOURCE_PORT
	    help.c
	    machine_i2c.c
	    machine_spi.c
	    machine_pin.c
	    machine_uart.c
	    modmachine.c
	    modutime.c
	    modzephyr.c
	    mphalport.c
	    uart_core.c
	)
	list(TRANSFORM MICROPY_SOURCE_PORT PREPEND ${MICROPY_PORT_DIR}/)

	set(MICROPY_SOURCE_SHARED
	    libc/printf.c
	    readline/readline.c
	    runtime/interrupt_char.c
	    runtime/mpirq.c
	    runtime/pyexec.c
	    runtime/stdout_helpers.c
	    timeutils/timeutils.c
	)
	list(TRANSFORM MICROPY_SOURCE_SHARED PREPEND ${MICROPY_DIR}/shared/)

	set(MICROPY_SOURCE_QSTR
	    ${MICROPY_SOURCE_PY}
	    ${MICROPY_SOURCE_EXTMOD}
	    ${MICROPY_SOURCE_SHARED}
	    ${MICROPY_SOURCE_PORT}
	)

	zephyr_get_include_directories_for_lang(C includes)
	zephyr_get_system_include_directories_for_lang(C system_includes)
	zephyr_get_compile_definitions_for_lang(C definitions)
	zephyr_get_compile_options_for_lang(C options)

	set(MICROPY_CPP_FLAGS_EXTRA ${includes} ${system_includes} ${definitions} ${options})

	zephyr_library_named(${MICROPY_TARGET})

	target_include_directories(${MICROPY_TARGET} PUBLIC
	    ${MICROPY_INC_CORE}
	    ${MICROPY_PORT_DIR}
	    ${CMAKE_CURRENT_BINARY_DIR}
	    ${CMAKE_BINARY_DIR}
	)

	zephyr_library_compile_options(
	    -std=gnu99 -fomit-frame-pointer
	)

	target_compile_definitions(${MICROPY_TARGET} PUBLIC
	    NDEBUG
	    MP_CONFIGFILE=<${CONFIG_MICROPY_CONFIGFILE}>
	)

	zephyr_library_sources(${MICROPY_SOURCE_QSTR})

	add_dependencies(${MICROPY_TARGET} zephyr_generated_headers)

	include(${MICROPY_DIR}/py/mkrules.cmake)

	target_link_libraries(app PRIVATE ${MICROPY_TARGET})
endif()
