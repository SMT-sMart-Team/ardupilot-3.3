# !bin/bash
date
rm -rf APP-new
rm -f FW-v0.2* 
mkdir -p APP-new
make pxf
cp ArduCopter.elf APP-new/app_quad
make pxf-hexa
cp ArduCopter.elf APP-new/app_hexa
make pxf-octa
cp ArduCopter.elf APP-new/app_octa
cp readme APP-new/
export FW_NAME=FW-v0.2-`date +%Y%m%d`.fc
tar -cf ${FW_NAME} APP-new
echo "package FW ${FW_NAME} done\n========================================="
