INSTALL_DIR = gearbox

all: installed

SVN_DIR = build/gearbox-svn
SVN_URL = http://gearbox.svn.sf.net/svnroot/gearbox/gearbox/tags/RELEASE_10.11
PATCH_FILE = aist_python_mt.patch

include $(shell rospack find mk)/svn_checkout.mk

CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=RELEASE \
             -D GEARBOX_INSTALL=`rospack find gearbox`/$(INSTALL_DIR) \
             -D boostPythonLib='boost_python' \
             -D ENABLE_LIB_FLEXIPORT=ON \
             -D ENABLE_LIB_BASICEXAMPLE=OFF \
             -D ENABLE_LIB_GBXADVANCEDEXAMPLE=OFF \
             -D ENABLE_LIB_GBXGARMINACFR=OFF \
             -D ENABLE_LIB_GBXICEUTILACFR=OFF \
             -D ENABLE_LIB_GBXNOVATELACFR=OFF \
             -D ENABLE_LIB_GBXSERIALACFR=OFF \
             -D ENABLE_LIB_GBXSICKACFR=OFF \
             -D ENABLE_LIB_GBXSMARTBATTERYACFR=OFF \
             -D ENABLE_LIB_GBXUTILACFR=OFF \
             -D ENABLE_LIB_HOKUYO_AIST=OFF \
             -D FLEXIPORT_BUILD_BINDINGS=OFF \
             -D FLEXIPORT_INCLUDE_LOGGING=ON \
             -D GBX_BUILD_TESTS=OFF \
             -D HOKUYO_AIST_BUILD_BINDINGS=OFF

installed: wiped $(SVN_DIR) patched
	mkdir -p $(SVN_DIR)/build
	cd `rospack find gearbox` && patch -d $(SVN_DIR)/src/hokuyo_aist/python --verbose < `rospack find gearbox`/$(PATCH_FILE)
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	touch installed

clean:
	-cd $(SVN_DIR)/build && make clean
	rm -rf $(INSTALL_DIR) installed

wiped: Makefile $(SVN_PATCH)
	make wipe
	touch wiped

wipe: clean
	rm -rf build patched

.PHONY : clean download
