add_library(usb
    usb_descriptors.c 
    usb.c)

target_include_directories(usb PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ../include  
)

target_link_libraries(usb
    pico_stdlib
    tinyusb_device
    pico_unique_id
    tinyusb_board
)