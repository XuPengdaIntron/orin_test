
export CROSS_COMPILE_AARCH64_PATH=/home/user/Daniel/orin_test/toolchain
export CROSS_COMPILE_AARCH64=/home/user/Daniel/orin_test/toolchain/bin/aarch64-buildroot-linux-gnu-

mkdir -p kernel_out
./nvbuild.sh -o $PWD/kernel_out

mkdir -p my_out
cp ./kernel_out/arch/arm64/boot/Image  ./my_out/Image_test
cp ./kernel_out/arch/arm64/boot/dts/nvidia/tegra234-p3701-0004-p3737-0000.dtb ./my_out/tegra234-p3701-0004-p3737-0000_test.dtb
#cp ./kernel_out/drivers/media/i2c/nv_irs2877a.ko  ./my_out/nv_irs2877a.ko
#cp ./kernel_out/drivers/media/i2c/nv_sz220at.ko   ./my_out/nv_sz220at.ko
cp ./kernel_out/drivers/media/i2c/nv_irs2877a2.ko  ./my_out/nv_irs2877a2.ko
cp ./kernel_out/drivers/media/i2c/nv_sz220at2.ko   ./my_out/nv_sz220at2.ko
cp ./kernel_out/drivers/media/i2c/nv_floodspot.ko   ./my_out/nv_floodspot.ko
