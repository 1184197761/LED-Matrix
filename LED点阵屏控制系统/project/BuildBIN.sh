#!/bin/bash

if [ -e tmp ]; then
files=$(ls tmp)
for f in $files
do
	ff=$(cat tmp/$f)
	mv -f $ff.bak $ff
done
rm -rf tmp
fi

cp  ./Obj/*.elf project.elf
cp  ./Lst/*.map project.map
cp  ./Obj/*.ihex project.hex

if [ -e parameter.bincfg ]; then
cp -f ./parameter.bincfg parameter.cfg
fi
if [ -e parameter_ui.setcfg ]; then
cp -f ./parameter_ui.setcfg parameter_ui.cfg
fi

${CDKPath}/CSKY/MinGW/csky-abiv2-elf-toolchain/bin/csky-elfabiv2-readelf.exe -a project.elf > project.lst
${CDKPath}/CSKY/MinGW/csky-abiv2-elf-toolchain/bin/csky-elfabiv2-objdump.exe -S project.elf > project.asm

[ ! -f BinScript.exe ] && cp ../../../../tools/makecode/BinScript.exe BinScript.exe
[ ! -f crc.exe ] && cp ../../../../tools/makecode/crc.exe crc.exe
[ ! -f makecode.exe ] && cp ../../../../tools/makecode/makecode.exe makecode.exe


cp parameter.bincfg parameter.cfg
cp ../sdk/chip/txw80x/loader.bin loader.bin

BinScript.exe BinScript.BinScript
makecode.exe
#crc.exe crc.ini
#BinScript.exe BinScript_Bin2Hex.BinScript

