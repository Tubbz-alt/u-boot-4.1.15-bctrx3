make distclean
make rm3spl_defconfig
make -j8
cp SPL /tftpboot
cp u-boot.img /tftpboot
