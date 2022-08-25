#source /opt/imx6ul/environment-setup-cortexa7t2hf-neon-poky-linux-gnueabi
source /opt/6ul-5.10sdk/environment-setup-cortexa7t2hf-neon-poky-linux-gnueabi
make distclean
#make myd_imx6ull_nand_ddr256_defconfig
#make myd_imx6ul_emmc_defconfig 
#make myd_imx6ul_nand_ddr256_defconfig
make myd_imx6ull_emmc_defconfig
make  -j16

#mkdir -p $PWD/../build
#cp $PWD/arch/arm/boot/zImage $PWD/../build/
#cp $PWD/arch/arm/boot/dts/myd-y6ull*.dtb $PWD/../build/
