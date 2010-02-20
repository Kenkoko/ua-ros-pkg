all: checkout build

SVN_DIR = gbbopen
SVN_URL = http://GBBopen.org/svn/GBBopen/trunk/
include $(shell rospack find mk)/svn_checkout.mk

checkout: $(SVN_DIR)

build: 
	sh compile.sh
	rm -rf asdf
	mkdir asdf
	ln -s `(rospack find gbbopen)`/gbbopen/gbbopen.asd asdf/gbbopen.asd 

clean:
	rm -rf patched rospack_nosubdirs gbbopen asdf
