all:build


ZIP_FILE   = PhidgetsPython_2.1.6.20100129.zip
ZIP_URL    = http://www.phidgets.com/downloads/libraries/$(ZIP_FILE)
SRC_DIR    = src
TMP_DIR    = tmp
PATCH_FILE = phidgets_py_api.patch

#include $(shell rospack find mk)/download_unpack_build.mk

build:
	if test -d $(SRC_DIR); \
	then \
		echo "phidgets_py_api has already been installed."; \
	else \
		mkdir $(SRC_DIR); \
		mkdir $(TMP_DIR); \
		wget $(ZIP_URL); \
		unzip $(ZIP_FILE) -d $(TMP_DIR); \
		mv $(TMP_DIR)/PhidgetsPython/Phidgets $(SRC_DIR); \
		patch -p1 --verbose < $(PATCH_FILE); \
		rm $(ZIP_FILE); \
		rm -rf $(TMP_DIR); \
	fi
        
clean:
	rm -rf $(SRC_DIR)

wipe: clean

