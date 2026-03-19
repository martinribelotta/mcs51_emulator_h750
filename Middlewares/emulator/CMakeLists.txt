add_library(emulator STATIC
    emulator.c
)

add_subdirectory(
    mcs51_emulator/lib
    ${CMAKE_CURRENT_BINARY_DIR}/mcs51_emulator_lib
)



target_include_directories(emulator
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/mcs51_emulator/lib
)

target_link_libraries(emulator
    PUBLIC
        stm32cubemx
        mcs51_lib
)
