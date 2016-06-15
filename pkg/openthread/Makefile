PKG_NAME=openthread
PKG_URL=git://github.com/openthread/openthread.git
PKG_VERSION=050365e4fcd8729d5e6b19fdc8164ce8c1627c11
#PKG_VERSION=0254cde9865f4c76ec5a4d1236c19fab58c408c4
#PKG_VERSION=95d2e8cdecc096efbf3287e6607c7b441086c115
PKG_BUILDDIR ?= $(BINDIRBASE)/pkg/$(BOARD)/$(PKG_NAME)

.PHONY: all

all: git-download
		$(MAKE) -C $(PKG_BUILDDIR)

include $(RIOTBASE)/pkg/pkg.mk
