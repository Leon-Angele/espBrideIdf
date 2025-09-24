
#
# Main component of TF Micro project 'micro_speech'.
#

idf_component_register(
    SRCS main.cc main_functions.cc constants.cc output_handler.cc model.cc spi_interface.c spi_test.c
    PRIV_REQUIRES spi_flash driver esp_timer
    INCLUDE_DIRS "")
