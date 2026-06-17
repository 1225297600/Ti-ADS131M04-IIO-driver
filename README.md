# Ti-ADS131M04-IIO-driver
Ti ADS131M04 IIO driver （Testing For Kernel 6.1.99）


Use Step:

(1) put "ads131m04.c" on "kernel/drivers/iio/adc/"

(2) "kernel/drivers/iio/adc/Makfefile" add line "obj-$(CONFIG_TI_ADS131M04) += ads131m04.o"

(3) "kernel/drivers/iio/adc/Kconfig" add :

"config TI_ADS131M04

	tristate "Ti ADS131M04 SPI ADC driver"
	
	depends on SPI_MASTER
	
	default y
	
	help
	
	  Ti ADS131M04 SPI ADC driver"

	  
(4) kernel do "make menuconfig"

serach "ADS131M04"

Enable It

(5) "make"

