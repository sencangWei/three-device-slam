CC ?= gcc
VENDOR_SDK ?=
ARCH ?= x86_64

TARGET := build/libtwo_uq2_xu.so
VENDOR_LIBRARY := $(VENDOR_SDK)/lib/libSPV4L2XU/GNU/$(ARCH)/libSPV4L2XU.a
CPPFLAGS := -I$(VENDOR_SDK)/include/V4L2
CFLAGS := -std=c11 -O2 -fPIC
LDFLAGS := -shared -Wl,--no-undefined

.PHONY: all clean

all: $(TARGET)

$(TARGET): bridge.c $(VENDOR_LIBRARY)
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ bridge.c $(VENDOR_LIBRARY)

clean:
	$(RM) -rf -- build
