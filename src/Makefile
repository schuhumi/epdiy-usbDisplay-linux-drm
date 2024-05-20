CONFIG_MODULE_SIG=n
obj-m += usb2epdiy.o
ccflags-y := -std=gnu11 -Wno-declaration-after-statement

KVERSION = $(shell uname -r)

all:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules

install_headers:
	apt-get install -y linux-headers-$(KVERSION)

install:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules_install
	/sbin/depmod -a

uninstall:
	rm /lib/modules/*/extra/usb2epdiy.ko
	/sbin/depmod -a

clean:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean

load:
	/sbin/modprobe usb2epdiy

unload:
	/sbin/modprobe --remove usb2epdiy

info:
	drm_info -- /dev/dri/card1

unbind:
	echo '$(firstword $(subst :, ,$(shell cd /sys/bus/usb/drivers/usb2epdiy/ && echo *-*:*)))' | sudo tee /sys/bus/usb/drivers/usb/unbind
