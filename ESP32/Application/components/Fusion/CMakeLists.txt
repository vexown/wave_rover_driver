idf_component_register(
    SRCS "Source/FusionAhrs.c" "Source/FusionCompass.c" "Source/FusionOffset.c"
    INCLUDE_DIRS "Include"
    REQUIRES driver esp_common freertos 
)