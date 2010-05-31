all:build


ZIP_FILE    = NMPT.zip
ZIP_URL     = http://mplab.ucsd.edu/~nick/$(ZIP_FILE)
SRC_DIR     = src
INCLUDE_DIR = include
LIB_DIR     = lib
BIN_DIR     = bin
DATA_DIR    = data
DOC_DIR     = doc
PATCH_FILE1 = nmpt_ros.patch
PATCH_FILE2 = nmpt_box_filter_bug.patch

include $(shell rospack find mk)/cmake.mk

build:
	if test -e $(LIB_DIR)/libnmpt.so; \
	then \
		echo "NMPT has already been installed."; \
	else \
		mkdir -p $(SRC_DIR)/nmpt; \
		mkdir -p $(INCLUDE_DIR)/nmpt; \
		mkdir -p $(DOC_DIR); \
		mkdir -p $(DATA_DIR); \
		wget $(ZIP_URL); \
		unzip $(ZIP_FILE); \
		patch -d NMPT -p1 --verbose < $(PATCH_FILE1); \
		patch -d NMPT/src -p2 --verbose < $(PATCH_FILE2); \
		mv NMPT/src/*.h $(INCLUDE_DIR)/nmpt; \
		mv NMPT/src/*.cpp $(SRC_DIR)/nmpt; \
		mv NMPT/doc/* $(DOC_DIR); \
		mv NMPT/data/* $(DATA_DIR); \
		mv NMPT/ReferenceManual.pdf $(DOC_DIR); \
		rm $(ZIP_FILE); \
		rm -rf NMPT; \
	fi

clean:
	rm -rf $(INCLUDE_DIR) $(SRC_DIR) $(LIB_DIR) $(BIN_DIR) $(DOC_DIR) $(DATA_DIR) build

wipe: clean
