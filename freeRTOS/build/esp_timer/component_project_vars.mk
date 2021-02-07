# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)/components/esp_timer/include
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/esp_timer -lesp_timer
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += esp_timer
COMPONENT_LDFRAGMENTS += $(IDF_PATH)/components/esp_timer/linker.lf
component-esp_timer-build: 
