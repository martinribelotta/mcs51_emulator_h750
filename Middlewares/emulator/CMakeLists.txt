set(MCS51_FW_ASM
    ${CMAKE_CURRENT_SOURCE_DIR}/mcs51_emulator/mc51code/serial_hello.asm
    CACHE FILEPATH "Path to 8051 firmware ASM source"
)

get_filename_component(MCS51_FW_NAME "${MCS51_FW_ASM}" NAME_WE)
set(MCS51_FW_HEX "${CMAKE_CURRENT_BINARY_DIR}/${MCS51_FW_NAME}.hex")
set(MCS51_FW_BIN "${CMAKE_CURRENT_BINARY_DIR}/${MCS51_FW_NAME}.bin")
set(MCS51_FW_OBJ "${CMAKE_CURRENT_BINARY_DIR}/${MCS51_FW_NAME}.o")

find_program(ASEM_EXECUTABLE NAMES asem)

set(MCS51_FW_OBJ_INPUTS)

if(ASEM_EXECUTABLE)
    add_custom_command(
        OUTPUT ${MCS51_FW_HEX}
        COMMAND ${ASEM_EXECUTABLE} "${MCS51_FW_ASM}"
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS ${MCS51_FW_ASM}
        COMMENT "Assembling 8051 firmware: ${MCS51_FW_ASM}"
        VERBATIM
    )

    add_custom_command(
        OUTPUT ${MCS51_FW_BIN}
        COMMAND ${CMAKE_OBJCOPY} -I ihex -O binary "${MCS51_FW_NAME}.hex" "${MCS51_FW_NAME}.bin"
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS ${MCS51_FW_HEX}
        COMMENT "Converting Intel HEX to BIN: ${MCS51_FW_HEX}"
        VERBATIM
    )

    add_custom_command(
        OUTPUT ${MCS51_FW_OBJ}
        COMMAND ${CMAKE_OBJCOPY}
            -I binary
            -O elf32-littlearm
            -B arm
            --rename-section .data=.rodata.mcs51_firmware,alloc,load,readonly,data,contents
            --redefine-sym _binary_${MCS51_FW_NAME}_bin_start=mcs51_firmware_start
            --redefine-sym _binary_${MCS51_FW_NAME}_bin_end=mcs51_firmware_end
            "${MCS51_FW_NAME}.bin"
            "${MCS51_FW_NAME}.o"
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS ${MCS51_FW_BIN}
        COMMENT "Creating linkable object from BIN: ${MCS51_FW_BIN}"
        VERBATIM
    )

    set_source_files_properties(${MCS51_FW_OBJ} PROPERTIES EXTERNAL_OBJECT TRUE GENERATED TRUE)
    set(MCS51_FW_OBJ_INPUTS ${MCS51_FW_OBJ})
else()
    message(WARNING "'asem' not found: embedded MCS51 firmware generation disabled; emulator will use built-in demo program.")
endif()

add_library(emulator STATIC
    emulator.c
    ${MCS51_FW_OBJ_INPUTS}
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
