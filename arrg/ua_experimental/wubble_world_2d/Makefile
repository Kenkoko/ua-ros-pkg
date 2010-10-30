all: installed

SVN_URL	= http://wubbleworld2d.googlecode.com/svn/trunk/
SVN_DIR = wubble_svn

include $(shell rospack find mk)/svn_checkout.mk

include $(shell rospack find mk)/cmake.mk

installed: $(SVN_DIR)
	if test -e src; \
	then \
		echo "Wubble World 2D is already installed."; \
	else \
		mv $(SVN_DIR)/src src; \
		mkdir bin; \
		cp -r src/data bin; \
		mv $(SVN_DIR)/lib lib; \
		mv $(SVN_DIR)/natives natives; \
		mkdir states; \
		mkdir logs; \
	fi

wipe: clean 
	rm -rf bin build src lib natives states logs patched rospack_nosubdirs $(SVN_DIR)
