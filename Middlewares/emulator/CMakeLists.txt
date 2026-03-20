set(MCS51_FW_HEX ${CMAKE_CURRENT_SOURCE_DIR}/mcs51_source/firmware.hex)

set(MCS51_FW_BIN "${CMAKE_CURRENT_BINARY_DIR}/mcs51_fw.bin")
set(MCS51_FW_OBJ "${CMAKE_CURRENT_BINARY_DIR}/mcs51_fw.o")

if(EXISTS "${MCS51_FW_HEX}")
    add_custom_command(
        OUTPUT ${MCS51_FW_BIN}
        COMMAND ${CMAKE_OBJCOPY} -I ihex -O binary "${MCS51_FW_HEX}" "${MCS51_FW_BIN}"
        DEPENDS "${MCS51_FW_HEX}"
        COMMENT "Converting Intel HEX to BIN: ${MCS51_FW_HEX}"
        VERBATIM
    )
    message(STATUS "MCS51 firmware HEX found at: ${MCS51_FW_HEX}. Will convert to BIN and embed.")
else()
    message(FATAL_ERROR "MCS51 firmware HEX not found at: ${MCS51_FW_HEX}.")
endif()

add_custom_command(
    OUTPUT ${MCS51_FW_OBJ}
    COMMAND ${CMAKE_OBJCOPY}
        -I binary
        -O elf32-littlearm
        -B arm
        --rename-section .data=.rodata.mcs51_firmware,alloc,load,readonly,data,contents
        --redefine-sym _binary_mcs51_fw_bin_start=mcs51_firmware_start
        --redefine-sym _binary_mcs51_fw_bin_end=mcs51_firmware_end
        mcs51_fw.bin
        mcs51_fw.o
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${MCS51_FW_BIN}
    COMMENT "Creating linkable object from BIN: ${MCS51_FW_BIN}"
    VERBATIM
)

set_source_files_properties(${MCS51_FW_OBJ} PROPERTIES EXTERNAL_OBJECT TRUE GENERATED TRUE)

add_library(emulator STATIC
    emulator.c
    ${MCS51_FW_OBJ}
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
