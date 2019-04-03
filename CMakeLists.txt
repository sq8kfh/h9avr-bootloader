cmake_minimum_required(VERSION 2.8.4)

project(h9avr-bootloader)

set(SOURCE_FILES bootloader.c common.c common.h h9msg.h)


set(CMAKE_SYSTEM_NAME Generic)

set(PATHS "/usr/local/bin/")

function(add_additional_clean_file FILENAME)
    get_directory_property(EXTRA_CLEAN_FILES ADDITIONAL_MAKE_CLEAN_FILES)
    set_directory_properties(
            PROPERTIES
            ADDITIONAL_MAKE_CLEAN_FILES "${EXTRA_CLEAN_FILES};${FILENAME}"
    )

    set_source_files_properties(${FILENAME} PROPERTIES GENERATED TRUE)
endfunction(add_additional_clean_file)

find_program(AVR_C_COMPILER avr-gcc PATHS ${PATHS})
find_program(AVR_OBJCOPY avr-objcopy PATHS ${PATHS})
find_program(AVR_SIZE avr-size PATHS ${PATHS})
find_program(AVRDUDE avrdude PATHS ${PATHS})

set(CMAKE_C_COMPILER ${AVR_C_COMPILER})

if(NOT BUILD_DIRECTORY)
    set(BUILD_DIRECTORY /build CACHE STRING "Setting build directory to /build")
endif(NOT BUILD_DIRECTORY)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}${BUILD_DIRECTORY}")

if(NOT SOURCE_FILES)
    message(FATAL_ERROR "SOURCE_FILES variable is not defined")
endif(NOT SOURCE_FILES)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

if(NOT FREQ)
    message(FATAL_ERROR "FREQ variable is not defined")
endif(NOT FREQ)

if(NOT PROGRAMMER)
    message(FATAL_ERROR "PROGRAMMER variable is not defined")
endif(NOT PROGRAMMER)

if(NOT MMCU)
    message(FATAL_ERROR "MMCU variable is not defined")
endif(NOT MMCU)

if(NOT BOOTSTART)
    message(FATAL_ERROR "BOOTSTART variable is not defined")
endif(NOT BOOTSTART)

message(STATUS "Building ${PROJECT_NAME} for ${MMCU} with frequency ${FREQ} at ${BOOTSTART}")
message(STATUS "Source files: ${SOURCE_FILES}")
message(STATUS "Using flags: ${CMAKE_C_FLAGS}")

set(ELF_FILENAME ${PROJECT_NAME}.elf)

set(ELF_FILE "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${ELF_FILENAME}")
set(HEX_FILE "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.hex")
set(EEP_FILE "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.eep")

add_additional_clean_file(${ELF_FILE})
add_additional_clean_file(${HEX_FILE})
add_additional_clean_file(${EEP_FILE})

set_target_properties(
        ${PROJECT_NAME}
        PROPERTIES
        OUTPUT_NAME ${ELF_FILENAME}
        COMPILE_FLAGS "-mmcu=${MMCU} -I. -gdwarf-2 -DF_CPU=${FREQ}UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wno-unknown-pragmas -Wstrict-prototypes -Wundef -std=gnu99 -Wl,--gc-section"
        LINK_FLAGS "-mmcu=${MMCU} -Wl,--section-start=.text=${BOOTSTART} -I. -gdwarf-2 -DF_CPU=${FREQ}UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wno-unknown-pragmas -Wstrict-prototypes -Wundef -std=gnu99 -Wundef -MD -MP -MF .dep/main.elf.d --output ${ELF_FILE} -Wl,-lc,--entry=main"
)

add_custom_command(
        TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMAND ${AVR_OBJCOPY} -O ihex -R .eeprom ${ELF_FILE} ${HEX_FILE}
        COMMAND ${AVR_SIZE} ${ELF_FILE} --mcu=${MMCU} --format=avr
        COMMENT "Post processsing"
)

add_custom_target("flash_bl"
        ${AVRDUDE} -p ${MMCU} -c ${PROGRAMMER} -U ${HEX_FILE}
        DEPENDS ${PROJECT_NAME}
        COMMENT "Uploading bootloader"
        )
