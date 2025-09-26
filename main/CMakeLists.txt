
#
# Main component of TF Micro project 'micro_speech'.
#

idf_component_register(
    SRCS src/main.cc src/main_functions.cc src/constants.cc src/output_handler.cc src/model.cc src/spi_interface.c  src/trajectory_buffer.cc src/app_cycle.cc src/ml_inference.cc
    PRIV_REQUIRES spi_flash driver esp_timer spiffs
    INCLUDE_DIRS "include")

