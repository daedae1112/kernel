#
# Makefile for common V4L/DVB tuners
#

tda18271-objs := tda18271-maps.o tda18271-common.o tda18271-fe.o

obj-$(CONFIG_MEDIA_TUNER_SIMPLE) += tuner-simple.o
# tuner-types will be merged into tuner-simple, in the future
obj-$(CONFIG_MEDIA_TUNER_SIMPLE) += tuner-types.o
obj-$(CONFIG_MEDIA_TUNER_TDA827X) += tda827x.o
obj-$(CONFIG_MEDIA_TUNER_TDA9887) += tda9887.o
obj-$(CONFIG_MEDIA_TUNER_TDA18271) += tda18271.o
obj-$(CONFIG_MEDIA_TUNER_MT2060) += mt2060.o
obj-$(CONFIG_MEDIA_TUNER_MT2266) += mt2266.o
obj-$(CONFIG_MEDIA_TUNER_QT1010) += qt1010.o
obj-$(CONFIG_MEDIA_TUNER_MT2131) += mt2131.o
obj-$(CONFIG_MEDIA_TUNER_MXL5005S) += mxl5005s.o
obj-$(CONFIG_MEDIA_TUNER_MXL5007T) += mxl5007t.o
obj-$(CONFIG_MEDIA_TUNER_MC44S803) += mc44s803.o

EXTRA_CFLAGS += -Idrivers/media/dvb/dvb-core
EXTRA_CFLAGS += -Idrivers/media/dvb/frontends
