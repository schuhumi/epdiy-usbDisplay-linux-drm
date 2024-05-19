#!/bin/sh
apt-get install -y \
	build-essential \
	linux-headers-$(uname -r) \
	libdrm-dev \
	libdrm2 \
	libdrm-common \
	libgbm-dev 
