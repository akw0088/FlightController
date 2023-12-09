Above is mostly auto generated from STM32CubeIDE

	Bootloader boot works
	Software runs when programming from bootloader
	Clocking issue resolved USB in non-bootloader mode
	(8mhz external clock HSE, was set to 27mhz originally as that was what was originally in the BOM, but what was built on the PCB was actually 8mhz, which is good, because the max clock I see on the CubeIDE is 26mhz)
	USB currently comes up as STMicroelectronics Virtual COM Port (COM3)
	Need to add some software to support updating when in non-bootloader mode
	(maybe serial command to enter DFU mode?)
	Idea was to just use BetaFlight unmodified, but I'm assuming clocking differences are preventing the USB from coming up



