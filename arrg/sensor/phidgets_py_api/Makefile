all:build


ZIP_FILE   = PhidgetsPython_2.1.6.20100129.zip
ZIP_URL    = http://www.phidgets.com/downloads/libraries/PhidgetsPython_2.1.6.20100129.zip
UNPACK_CMD = unzip
SRC_DIR    = src
PKG_NAME   = phidgets_py_api

#include $(shell rospack find mk)/download_unpack_build.mk

build: 
	mkdir $(SRC_DIR)
	wget $(ZIP_URL)
	$(UNPACK_CMD) $(ZIP_FILE) -d $(SRC_DIR)
	mv $(SRC_DIR)/PhidgetsPython $(SRC_DIR)/$(PKG_NAME)
	rm $(ZIP_FILE)
        
clean:
	rm -rf $(SRC_DIR)


wipe: clean
	rm -rf build
