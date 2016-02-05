#!/bin/bash
make distclean
make zynq_zed_config
#make clean
if make -j6  
then
 echo "*************make Success!************"
 echo "************* copy u-boot ************"
	if cp u-boot /workspace/result/BOOT_BIN/u-boot.elf
	then 
	echo "************* copy OK ************"
	else
	echo "************* copy ER ************"
	fi
else
 echo "*************make failed!************"
fi

rm /workspace/result/BOOT_BIN/BOOT.BIN	
#rm BOOT.BIN
#if bootgen -image ddddd.bif -bin -o i BOOT.BIN
if /workspace/result/BOOT_BIN/make_bin.sh
then
	echo "bootgen make BOOT.BIN ok ^_^"

fi
