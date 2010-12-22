all: installed

SVN_URL	= https://wubbleworld2d.googlecode.com/svn/trunk/
SVN_DIR = wubble_svn

include $(shell rospack find mk)/svn_checkout.mk

include $(shell rospack find mk)/cmake.mk

installed: $(SVN_DIR)
	if test -e bin/data; \
	then \
		echo "WW2D data is already installed."; \
	else \
		mkdir -p bin; \
		cp -r $(SVN_DIR)/src/data bin; \
	fi

wipe: clean 
	rm -rf bin src logs build patched rospack_nosubdirs $(SVN_DIR)
