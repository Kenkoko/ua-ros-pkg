INSTALL_DIR = gearbox

all: installed

SVN_DIR = build/gearbox-svn
SVN_URL = http://gearbox.svn.sf.net/svnroot/gearbox/gearbox/tags/RELEASE_10.11
PATCH_FILE = aist_python_mt.patch

include $(shell rospack find mk)/svn_checkout.mk

CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=RELEASE \
             -D GEARBOX_INSTALL=`rospack find gearbox`/$(INSTALL_DIR) \
             -D boostPythonLib='boost_python'

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
