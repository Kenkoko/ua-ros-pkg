all:build


ZIP_FILE   = PhidgetsPython_2.1.6.20100129.zip
ZIP_URL    = http://www.phidgets.com/downloads/libraries/$(ZIP_FILE)
SRC_DIR    = src
PKG_NAME   = phidgets_py_api
SRC_PATH   = $(SRC_DIR)/$(PKG_NAME)

#include $(shell rospack find mk)/download_unpack_build.mk

build:
	if test -d $(SRC_DIR); \
	then \
		echo "phidgets_py_api has already been installed."; \
	else \
		mkdir $(SRC_DIR); \
		wget $(ZIP_URL); \
		unzip $(ZIP_FILE) -d $(SRC_DIR); \
		mv $(SRC_DIR)/PhidgetsPython $(SRC_PATH); \
		touch $(SRC_PATH)/__init__.py; \
		patch -b --verbose $(SRC_PATH)/Phidgets/PhidgetException.py < $(PKG_NAME).patch; \
		rm $(ZIP_FILE); \
	fi
        
clean:
	rm -rf $(SRC_DIR)

wipe: clean

