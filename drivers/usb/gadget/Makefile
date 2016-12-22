#
# USB peripheral controller drivers
#
subdir-ccflags-$(CONFIG_USB_GADGET_DEBUG)	:= -DDEBUG
subdir-ccflags-$(CONFIG_USB_GADGET_VERBOSE)	+= -DVERBOSE_DEBUG
ccflags-y				+= -I$(srctree)/drivers/usb/gadget/udc

obj-$(CONFIG_USB_LIBCOMPOSITE)	+= libcomposite.o
libcomposite-y			:= usbstring.o config.o epautoconf.o
libcomposite-y			+= composite.o functions.o configfs.o u_f.o
#++ 2015/11/16 USB Team, PCN00038 ++
libcomposite-y			+= functions.o u_f.o
#-- 2015/11/16 USB Team, PCN00038 --

obj-y				+= debug.o
g_android-y			:= android.o
obj-$(CONFIG_USB_GADGET)	+= udc/ function/ legacy/
