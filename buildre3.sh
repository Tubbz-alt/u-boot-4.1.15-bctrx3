make distclean
make re3spl_defconfig
make -j8
cp SPL /tftpboot
cp u-boot.img /tftpboot
