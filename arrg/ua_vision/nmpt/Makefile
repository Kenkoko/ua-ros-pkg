all:build


ZIP_FILE   = NMPT.zip
ZIP_URL    = http://mplab.ucsd.edu/~nick/$(ZIP_FILE)
SRC_DIR    = src
LIB_DIR    = lib
PATCH_FILE = nmptgcc4_4_1.patch

include $(shell rospack find mk)/cmake.mk

build:
	if test -d $(LIB_DIR)/libnmpt.so; \
	then \
		echo "NMPT has already been installed."; \
	else \
		mkdir -p $(SRC_DIR); \
		wget $(ZIP_URL); \
		unzip $(ZIP_FILE); \
		mv NMPT/src/*.h $(SRC_DIR); \
		mv NMPT/src/*.cpp $(SRC_DIR); \
		patch -b -p0 --verbose < $(PATCH_FILE); \
		rm $(ZIP_FILE); \
		rm -rf NMPT; \
	fi

