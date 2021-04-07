ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_SDM450) += sdm450-cdp-s2-overlay.dtbo \
	sdm450-mtp-s3-overlay.dtbo \
	sdm450-qrd-sku4-overlay.dtbo

msm8953-mtp-overlay.dtbo-base := sdm450.dtb \
	msm8953.dtb \
	apq8053.dtb \
	sda450.dtb \
	msm8953-pmi8940.dtb \
	msm8953-pmi8937.dtb \
	sdm450-pmi8940.dtb \
	sdm450-pmi8937.dtb
msm8953-cdp-overlay.dtbo-base := sdm450.dtb \
	msm8953.dtb \
	apq8053.dtb \
	sda450.dtb \
	msm8953-pmi8940.dtb \
	msm8953-pmi8937.dtb
msm8953-rcm-overlay.dtbo-base := sdm450.dtb \
	msm8953.dtb \
	apq8053.dtb
msm8953-ipc-overlay.dtbo-base := msm8953.dtb \
	apq8053.dtb
msm8953-qrd-overlay.dtbo-base := sdm450.dtb \
	msm8953.dtb
msm8953-iot-mtp-overlay.dtbo-base := sdm450.dtb \
	msm8953.dtb \
	apq8053.dtb
msm8953-ext-codec-mtp-overlay.dtbo-base := msm8953.dtb \
	apq8053.dtb \
	msm8953-pmi8940.dtb \
	msm8953-pmi8937.dtb
msm8953-ext-codec-rcm-overlay.dtbo-base := msm8953.dtb \
	apq8053.dtb
msm8953-cdp-1200p-overlay.dtbo-base := msm8953.dtb
sdm450-cdp-s2-overlay.dtbo-base := sdm450-pmi632.dtb \
	sdm632.dtb \
	sdm632-pm8004.dtb \
	msm8953-pmi632.dtb \
	sda450-pmi632.dtb
sdm450-mtp-s3-overlay.dtbo-base := sdm450-pmi632.dtb \
	sdm632.dtb \
	sdm632-pm8004.dtb \
	sda450-pmi632.dtb
sdm450-qrd-sku4-overlay.dtbo-base := sdm450-pmi632.dtb \
	sdm632.dtb \
	sdm632-pm8004.dtb
else
dtb-$(CONFIG_ARCH_MSM8953) += msm8953-cdp.dtb \
	msm8953-mtp.dtb \
	msm8953-ext-codec-mtp.dtb \
	msm8953-qrd-sku3.dtb \
	msm8953-rcm.dtb \
	apq8053-rcm.dtb \
	msm8953-ext-codec-rcm.dtb \
	apq8053-cdp.dtb \
	apq8053-ipc.dtb \
	msm8953-ipc.dtb \
	apq8053-mtp.dtb \
	apq8053-ext-audio-mtp.dtb \
	apq8053-ext-codec-rcm.dtb \
	msm8953-cdp-1200p.dtb \
	msm8953-iot-mtp.dtb \
	apq8053-iot-mtp.dtb \
	apq8053-lite-dragon-v1.0.dtb \
	apq8053-lite-dragon-v2.0.dtb \
	apq8053-lite-dragon-v2.1.dtb \
	apq8053-lite-dragon-v2.2.dtb \
	apq8053-lite-dragon-v2.2.1.dtb \
	apq8053-lite-dragon-v2.3.dtb \
	apq8053-lite-dragon-v2.4.dtb \
	apq8053-lat-concam-dev.dtb \
	apq8053-lat-concam-proto.dtb \
	msm8953-pmi8940-cdp.dtb \
	msm8953-pmi8940-mtp.dtb \
	msm8953-pmi8937-cdp.dtb \
	msm8953-pmi8937-mtp.dtb \
	msm8953-pmi8940-ext-codec-mtp.dtb \
	msm8953-pmi8937-ext-codec-mtp.dtb \
	msm8953-pmi632-cdp-s2.dtb

dtb-$(CONFIG_ARCH_SDM450) += sdm450-rcm.dtb \
	sdm450-cdp.dtb \
	sdm450-mtp.dtb \
	sda450-cdp.dtb \
	sda450-mtp.dtb \
	sdm450-qrd.dtb \
	sdm450-pmi8940-mtp.dtb \
	sdm450-pmi8937-mtp.dtb \
	sdm450-iot-mtp.dtb \
	sdm450-qrd-sku4.dtb \
	sdm450-pmi632-cdp-s2.dtb \
	sdm450-pmi632-mtp-s3.dtb \
	sda450-pmi632-cdp-s2.dtb \
	sda450-pmi632-mtp-s3.dtb

dtb-$(CONFIG_ARCH_SDM632) += sdm632-rumi.dtb \
	sdm632-cdp-s2.dtb \
	sdm632-ext-codec-cdp-s3.dtb \
	sdm632-mtp-s3.dtb \
	sdm632-ext-codec-mtp-s4.dtb \
	sdm632-rcm.dtb \
	sdm632-qrd-sku4.dtb \
	sdm632-qrd.dtb \
	sdm632-pm8004-cdp-s2.dtb \
	sdm632-pm8004-ext-codec-cdp-s3.dtb \
	sdm632-pm8004-mtp-s3.dtb \
	sdm632-pm8004-ext-codec-mtp-s4.dtb \
	sdm632-pm8004-rcm.dtb \
	sdm632-pm8004-qrd-sku4.dtb \
	sdm632-pm8004-qrd.dtb

endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
