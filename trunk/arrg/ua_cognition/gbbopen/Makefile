all: checkout build

SVN_DIR = gbbopen
SVN_URL = http://GBBopen.org/svn/GBBopen/trunk/
SVN_REVISION = -r 906
include $(shell rospack find mk)/svn_checkout.mk

checkout: $(SVN_DIR)

build: 
	sh compile.sh
	if test -e asdf/gbbopen.asd; then echo "GBBopen already patched"; else ln -s `(rospack find gbbopen)`/gbbopen/gbbopen.asd asdf/gbbopen.asd; fi
clean:
	rm -rf rospack_nosubdirs gbbopen asdf/gbbopen.asd
