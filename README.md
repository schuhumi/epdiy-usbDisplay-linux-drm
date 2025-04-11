# DIY E-Paper Linux Computer

This is a linux kernel driver for epaper displays. The hardware driver used is an [EPDiy board](https://github.com/vroland/epdiy), attached to an 13.3 E-Ink epaper display (ED133UT2). The board runs my [epdiy-usbDisplay](https://github.com/schuhumi/epdiy-usbDisplay) firmware, which is what this kernel driver talks to over usb.

![Image of e-paper computer](https://i.imgur.com/piu50Rt.jpeg)

### Usage

Build and install the kernel module like any other, there are some convenience shortcuts in src/Makefile. After that, the e-paper should turn up just like any other regular display.

### Attention: kernel version!

I do not maintain this module for multiple kernel versions, always just the one I happen to use at a time. Currently it is **6.1.0**.
