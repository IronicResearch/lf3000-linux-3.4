# lf3000-linux-3.4
Development snapshot of LF3000 Linux 3.4.x kernel

Refer to `build.sh` script for config options and env vars.

`CROSS_COMPILE` env var set to ARM cross-compiler prefix.
Kernel may be built with any ARM cross-compiler, not just
uclibc toolchain version used for LF userspace binaries.

```
export CROSS_COMPILE=arm-none-linux-gnueabi-
...
export CROSS_COMPILE=arm-angstrom-linux-uclibceabi-
```

`TARGET_MACH` env var set to target board configuration.
Refer to `arch/arm/configs/nxp4330_*_defconfig` files
for product and board variant options.

```
export TARGET_MACH=nxp4330_cabo_defconfig
...
export TARGET_MACH=nxp4330_bogota_defconfig
```

Development snapshot is from 2015 pre-release of BOGOTA device,
so incomplete compared to LF release tarball, though includes
developer git history for reference.

```
./build.sh
```

Equivalent to compiling BOGOTA default config:

```
make ARCH=arm nxp4330_bogota_defconfig
make ARCH=arm uImage
```

Successful build will yield kernel uImage binary + modules.tar
archive in parent ../deploy directory.
