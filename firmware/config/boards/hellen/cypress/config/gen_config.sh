#!/bin/bash
# This script files reads rusefi_config.txt and produces firmware persistent configuration headers
# the storage section of rusefi.ini is updated as well
#
# many of the files generated here require time consuming full compilation thus we have an aggressive caching mechanism
# to make sure that we do not regenerate for no reason
# the primary input files are rusefi_config.txt and rusefi.input, also mapping.yaml etc
# see inside cache.zip for all input files
#

cd ../../../../..

pwd

bash gen_signature.sh hellen_cypress

java \
 -DSystemOut.name=gen_config_hellen_cypress \
 -Drusefi.generator.lazyfile.enabled=true \
 -jar ../java_tools/ConfigDefinition.jar \
 -definition integration/rusefi_config.txt \
 -cache hellen_cypress \
 -cache_zip_file tunerstudio/generated/cache.zip \
 -ts_destination tunerstudio \
 -tool hellen_cypress_gen_config.bat \
 -firing_order controllers/algo/firing_order.h \
 -with_c_defines false \
 -initialize_to_zero false \
 -ts_output_name generated/rusefi_hellen_cypress.ini \
 -c_defines config/boards/hellen/cypress/config/controllers/algo/rusefi_generated.h \
 -c_destination config/boards/hellen/cypress/config/controllers/algo/engine_configuration_generated_structures.h \
 -signature tunerstudio/generated/signature_hellen_cypress.txt \
 -signature_destination controllers/generated/signature_hellen_cypress.h \
 -enumInputFile controllers/algo/rusefi_enums.h \
 -enumInputFile controllers/algo/rusefi_hw_enums.h \
 -board hellen_cypress \
 -prepend config/boards/hellen/cypress/config/tunerstudio/generated/hellen_cypress_prefix.txt

[ $? -eq 0 ] || { echo "ERROR generating TunerStudio config for hellen_cypress"; exit 1; }
