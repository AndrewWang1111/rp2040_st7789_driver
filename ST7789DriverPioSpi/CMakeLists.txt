
add_library(ST7789Driver driver.cpp pio_spi.cpp)
target_link_libraries(ST7789Driver
pico_stdlib
hardware_pio
)
pico_generate_pio_header(ST7789Driver ${CMAKE_CURRENT_LIST_DIR}/spi.pio)