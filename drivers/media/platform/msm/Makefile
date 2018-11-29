#
# Makefile for the QCOM spcific video device drivers
# based on V4L2.
#

obj-$(CONFIG_MSM_VIDC_V4L2)     += vidc/
obj-$(CONFIG_MSM_WFD) += wfd/
obj-y += broadcast/
obj-$(CONFIG_DVB_MPQ) += dvb/
<<<<<<< HEAD
ifeq ($(CONFIG_HUAWEI_KK_CAMERA),y)
obj-$(CONFIG_MSMB_CAMERA) += camera_v2_cherry/
else ifeq ($(CONFIG_HUAWEI_L_CAMERA),y)
obj-$(CONFIG_MSMB_CAMERA) += camera_v2_scale/
else ifeq ($(CONFIG_MACH_HY_A16C3H),y)
obj-$(CONFIG_MSMB_CAMERA) += camera_a16c3h_v2/
else
obj-$(CONFIG_MSMB_CAMERA) += camera_v2/
endif
obj-$(CONFIG_MSM_VPU) += vpu/
