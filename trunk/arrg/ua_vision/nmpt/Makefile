all:build


ZIP_FILE   = NMPT.zip
ZIP_URL    = http://mplab.ucsd.edu/~nick/$(ZIP_FILE)
SRC_DIR    = src
PKG_NAME   = nmpt
SRC_PATH   = $(SRC_DIR)/$(PKG_NAME)

include $(shell rospack find mk)/cmake.mk

build:
	if test -d $(SRC_DIR); \
	then \
		echo "NMPT has already been installed."; \
	else \
		mkdir $(SRC_DIR); \
		wget $(ZIP_URL); \
		unzip $(ZIP_FILE); \
		mv NMPT/src/*.h $(SRC_DIR); \
		mv NMPT/src/*.cpp $(SRC_DIR); \
		#rm $(ZIP_FILE); \
		rm -rf NMPT; \
	fi
        
clean:
	rm -rf $(SRC_DIR)

wipe: clean
	rm -rf build
