#!/bin/bash

export EXTRA_PARAMS="-DDUMMY \
 -DDEFAULT_ENGINE_TYPE=MIATA_NA6_VAF \
 \
 -DEFI_SOFTWARE_KNOCK=TRUE \
 -DSTM32_ADC_USE_ADC3=TRUE \
 -Iconfig/boards/frankenso \
 \
 -DFIRMWARE_ID=\\\"frankensoNA6\\\""

bash ../common_make.sh

