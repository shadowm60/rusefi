#include "../stm32f7ems/efifeatures.h"

#pragma once

#undef EFI_MAP_AVERAGING
#define EFI_MAP_AVERAGING FALSE

#undef EFI_USE_FAST_ADC
#define EFI_USE_FAST_ADC FALSE

#undef EFI_MC33816
#define EFI_MC33816 FALSE

#undef EFI_CJ125
#define EFI_CJ125 FALSE

#undef BOARD_TLE6240_COUNT
#undef BOARD_MC33972_COUNT
#undef BOARD_TLE8888_COUNT
#define BOARD_TLE6240_COUNT	0
#define BOARD_MC33972_COUNT	0
#define BOARD_TLE8888_COUNT 	0

#undef EFI_MAX_31855
#define EFI_MAX_31855 FALSE

#undef BOARD_EXT_GPIOCHIPS
#define BOARD_EXT_GPIOCHIPS			(BOARD_TLE6240_COUNT + BOARD_MC33972_COUNT + BOARD_TLE8888_COUNT + BOARD_DRV8860_COUNT + BOARD_MC33810_COUNT)

#define EFI_USE_COMPRESSED_INI_MSD

// H7 has dual bank, so flash on its own (low priority) thread so as to not block any other operations
#define EFI_FLASH_WRITE_THREAD TRUE

#undef EFI_LUA
#define EFI_LUA TRUE
