# !bin/bash
date
rm -rf APP-new
mkdir -p APP-new
make pxf
cp ArduCopter.elf APP-new/app_quad
make pxf-hexa
cp ArduCopter.elf APP-new/app_hexa
make pxf-octa
cp ArduCopter.elf APP-new/app_octa
cp readme APP-new/
tar -cf FW-v0.2.tar APP-new
echo "package FW done\n========================================="
