# !bin/bash
echo "========================================="
date

if [ $# = 0 ]
then
    echo "Usage: $1 <FW-version>"
    echo "\n=========================================\n"
    exit 
fi

export FW_NAME=FW-V${1}-`date +%Y%m%d`.fc
echo "package ${FW_NAME} start\n"

rm -rf APP-new
rm -f FW-V* 
mkdir -p APP-new
make pxf
cp ArduCopter.elf APP-new/app_quad
make pxf-hexa
cp ArduCopter.elf APP-new/app_hexa
make pxf-octa
cp ArduCopter.elf APP-new/app_octa
cp readme APP-new/
tar -cf ${FW_NAME} APP-new
echo "package FW ${FW_NAME} done\n========================================="
