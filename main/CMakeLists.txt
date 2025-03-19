set(priv_requires fatfs console driver)

if(CONFIG_POLO_TRAINER_STORAGE_MEDIA_SPIFLASH)
    list(APPEND priv_requires wear_levelling esp_partition)
endif()

idf_component_register(
    SRCS "polo_trainer.cpp"
    INCLUDE_DIRS .
    PRIV_REQUIRES "${priv_requires}"
)
