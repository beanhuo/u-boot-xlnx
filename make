#!/bin/bash
#make distclean
export CROSS_COMPILE=/crosstools/zynq_tools/source/bin/arm-xilinx-linux-gnueabi-
export ARCH=arm
#make zynq_zc706_defconfig
#make clean
if make -j6  
then
 echo "*************make Success!************"
 echo "************* copy u-boot ************"
	if cp u-boot /home/bean/workspace/result/u-boot.elf
	then 
	echo "************* copy OK ************"
	else
	echo "************* copy ER ************"
	fi
else
 echo "*************make failed!************"
fi
