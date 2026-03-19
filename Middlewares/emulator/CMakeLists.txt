add_library(emulator STATIC
    emulator.c
)

target_include_directories(emulator
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(emulator
    PUBLIC
        stm32cubemx
)
