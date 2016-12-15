make distclean
make re3android_defconfig
make -j8
cp SPL /tftpboot
cp u-boot.img /tftpboot
