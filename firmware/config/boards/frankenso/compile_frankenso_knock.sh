#!/bin/bash

export EXTRA_PARAMS="-DDUMMY \
 -DDEFAULT_ENGINE_TYPE=MIATA_NA6_VAF \
 \
 \
 \
 -DFIRMWARE_ID=\\\"frankensoNA6\\\""

bash ../common_make.sh
