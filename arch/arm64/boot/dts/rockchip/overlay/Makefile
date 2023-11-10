# SPDX-License-Identifier: GPL-2.0
dtbo-$(CONFIG_ARCH_ROCKCHIP) += \
	rk3566-bigtree-cb2-hdmi.dtbo \
	rk3566-bigtree-cb2-csi.dtbo \
	rk3566-bigtree-cb2-dsi.dtbo \
	rk3566-bigtree-cb2-dsi-double.dtbo \
	rk3566-bigtree-cb2-i2c1.dtbo \
	rk3566-bigtree-cb2-i2c3.dtbo \
	rk3566-bigtree-cb2-i2c4.dtbo \
	rk3566-bigtree-cb2-mcp2515.dtbo \
	rk3566-bigtree-cb2-pwm0.dtbo \
	rk3566-bigtree-cb2-pwm1.dtbo \
	rk3566-bigtree-cb2-pwm2.dtbo \
	rk3566-bigtree-cb2-pwm4.dtbo \
	rk3566-bigtree-cb2-pwm5.dtbo \
	rk3566-bigtree-cb2-pwm12.dtbo \
	rk3566-bigtree-cb2-pwm13.dtbo \
	rk3566-bigtree-cb2-pwm14.dtbo \
	rk3566-bigtree-cb2-pwm15.dtbo \
	rk3566-bigtree-cb2-spi1.dtbo \
	rk3566-bigtree-cb2-spi3.dtbo \
	rk3566-bigtree-cb2-tft_35.dtbo \
	rk3566-bigtree-cb2-uart0.dtbo \
	rk3566-bigtree-cb2-uart1.dtbo \
	rk3566-bigtree-cb2-uart3.dtbo \
	rk3566-bigtree-cb2-uart5.dtbo \
	rk3566-bigtree-cb2-uart7.dtbo \
	rk3566-bigtree-cb2-uart9.dtbo
	
	
	
	# \
	#bigtree-cb2-hdmi.dtbo \
	#bigtree-cb2-spi-display.dtbo

scr-$(CONFIG_ARCH_ROCKCHIP) += \
	rockchip-fixup.scr

dtbotxt-$(CONFIG_ARCH_ROCKCHIP) += \
	README.rockchip-overlays

targets += $(dtbo-y) $(scr-y) $(dtbotxt-y)

always			:= $(dtbo-y) $(scr-y) $(dtbotxt-y)
clean-files		:= *.dtbo *.scr

