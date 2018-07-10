zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ADS1X9X ads1x9x.c
											ADS1x9xTI.c
											ECGInterface.c
											ADS1x9x_ECG_Processing.c
											ADS1x9x_RESP_Processing.c)
zephyr_library_sources_ifdef(CONFIG_ADS1X9X_TRIGGER ads1x9x_trigger.c)
