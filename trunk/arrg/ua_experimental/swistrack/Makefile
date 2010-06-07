all:build

SVN_URL      = https://swistrack.svn.sourceforge.net/svnroot/swistrack
SVN_REVISION = -r 1006
SVN_DIR      = swistrack_svn

BIN_DIR      = bin
INCLUDE_DIR  = include
LIB_DIR      = lib
SRC_DIR      = src

DOC_DIR      = doc
EXAMPLES_DIR = examples

PATCH_FILE   = swistrack_ros_paths.patch

include $(shell rospack find mk)/cmake.mk

build:
	if test -e $(LIB_DIR)/libswistrackcore.so; \
	then \
		echo "SwisTrack has already been installed."; \
	else \
		mkdir -p $(BIN_DIR); \
		mkdir -p $(INCLUDE_DIR)/swistrack/batch; \
		mkdir -p $(INCLUDE_DIR)/swistrack/core; \
		mkdir -p $(INCLUDE_DIR)/swistrack/gui; \
		mkdir -p $(SRC_DIR)/swistrack/batch; \
		mkdir -p $(SRC_DIR)/swistrack/core; \
		mkdir -p $(SRC_DIR)/swistrack/gui; \
		mkdir -p $(INCLUDE_DIR)/camshift_wrapper; \
		mkdir -p $(INCLUDE_DIR)/cvblobslib; \
		mkdir -p $(INCLUDE_DIR)/tsai; \
		mkdir -p $(SRC_DIR)/camshift_wrapper; \
		mkdir -p $(SRC_DIR)/cvblobslib; \
		mkdir -p $(SRC_DIR)/tsai; \
		mkdir -p $(DOC_DIR); \
		mkdir -p $(EXAMPLES_DIR); \
		svn export $(SVN_REVISION) $(SVN_URL) $(SVN_DIR); \
		mv $(SVN_DIR)/Code/batch/*.h $(INCLUDE_DIR)/swistrack/batch; \
		mv $(SVN_DIR)/Code/batch/*.cpp $(SRC_DIR)/swistrack/batch; \
		mv $(SVN_DIR)/Code/core/*.h $(INCLUDE_DIR)/swistrack/core; \
		mv $(SVN_DIR)/Code/core/*.cpp $(SRC_DIR)/swistrack/core; \
		mv $(SVN_DIR)/Code/gui/*.h $(INCLUDE_DIR)/swistrack/gui; \
		mv $(SVN_DIR)/Code/gui/*.cpp $(SRC_DIR)/swistrack/gui; \
		mv $(SVN_DIR)/Code/gui/bitmaps $(INCLUDE_DIR)/swistrack/gui; \
		mv $(SVN_DIR)/3rdParty/camshift_wrapper/*.h $(INCLUDE_DIR)/camshift_wrapper; \
		mv $(SVN_DIR)/3rdParty/camshift_wrapper/*.c $(SRC_DIR)/camshift_wrapper; \
		mv $(SVN_DIR)/3rdParty/camshift_wrapper/*.txt $(INCLUDE_DIR)/camshift_wrapper; \
		mv $(SVN_DIR)/3rdParty/cvblobslib_v6p1/*.h $(INCLUDE_DIR)/cvblobslib; \
		mv $(SVN_DIR)/3rdParty/cvblobslib_v6p1/*.cpp $(SRC_DIR)/cvblobslib; \
		mv $(SVN_DIR)/3rdParty/cvblobslib_v6p1/*.txt $(SRC_DIR)/cvblobslib; \
		mv $(SVN_DIR)/3rdParty/libtsai/include/*.h $(INCLUDE_DIR)/tsai; \
		mv $(SVN_DIR)/3rdParty/libtsai/tsaisrc/*.c $(SRC_DIR)/tsai; \
		mv $(SVN_DIR)/3rdParty/libtsai/mathsrc $(SRC_DIR)/tsai; \
		mv $(SVN_DIR)/Doc/* $(DOC_DIR); \
		mv $(SVN_DIR)/Examples/* $(EXAMPLES_DIR); \
		mv $(SVN_DIR)/SwisTrackEnvironment/* $(BIN_DIR); \
		mv $(SVN_DIR)/COPYING .; \
		mv $(SVN_DIR)/README .; \
		rm -rf $(SVN_DIR); \
		patch -p1 --verbose < $(PATCH_FILE); \
	fi

clean:
	rm -rf build $(LIB_DIR) $(BIN_DIR)/SwisTrack $(BIN_DIR)/SwisTrack-batch $(BIN_DIR)/TrackFaces

wipe: clean
	rm -rf $(BIN_DIR) $(INCLUDE_DIR) $(SRC_DIR) $(DOC_DIR) $(EXAMPLES_DIR) COPYING README

