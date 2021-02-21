FROM ubuntu:20.04

# Build tools
RUN set -ex; \
    apt-get update && apt-get install -y make wget xz-utils;

# Create build folder
RUN set -ex; \
    mkdir /build;

WORKDIR /build

# gcc
RUN set -ex; \
    wget --no-verbose https://releases.linaro.org/components/toolchain/binaries/6.5-2018.12/arm-linux-gnueabihf/gcc-linaro-6.5.0-2018.12-x86_64_arm-linux-gnueabihf.tar.xz; \
    tar xfJ gcc-linaro-6.5.0-2018.12-x86_64_arm-linux-gnueabihf.tar.xz; \
    rm gcc-linaro-6.5.0-2018.12-x86_64_arm-linux-gnueabihf.tar.xz; \
    mv gcc-linaro-6.5.0-2018.12-x86_64_arm-linux-gnueabihf gcc;

ENV ARCH=arm
ENV CROSS_COMPILE=/build/gcc/bin/arm-linux-gnueabihf-

# libusb
RUN set -ex; \
    wget --no-verbose https://github.com/libusb/libusb/releases/download/v1.0.23/libusb-1.0.23.tar.bz2; \
    tar xfj libusb-1.0.23.tar.bz2; \
    rm libusb-1.0.23.tar.bz2; \
    mv libusb-1.0.23 libusb; \
    cd libusb; \
    ./configure --prefix=/build/gcc --host=arm-linux-gnueabihf --disable-udev --disable-shared; \
    make; \
    mkdir -p /build/gcc/arm-linux-gnueabihf/include/libusb-1.0; \
    cp libusb/libusb.h /build/gcc/arm-linux-gnueabihf/include/libusb-1.0/; \
    cp libusb/.libs/libusb-1.0.a /build/gcc/lib/gcc/arm-linux-gnueabihf/6.5.0/;

# libphidgets
RUN set -ex; \
    wget --no-verbose https://www.phidgets.com/downloads/phidget22/libraries/linux/libphidget22.tar.gz; \
    tar xfz libphidget22.tar.gz; \
    rm libphidget22.tar.gz; \
    mv libphidget22-* libphidget22; \
    cd libphidget22; \
    ./configure --prefix=/build/gcc --host=arm-linux-gnueabihf --disable-shared; \
    make; \
    cp phidget22.h /build/gcc/arm-linux-gnueabihf/include/; \
    cp .libs/libphidget22.a /build/gcc/lib/gcc/arm-linux-gnueabihf/6.5.0/;

WORKDIR /project