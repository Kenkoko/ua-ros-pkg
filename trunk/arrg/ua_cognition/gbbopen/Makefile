all: checkout test

SVN_DIR = gbbopen
SVN_URL = http://GBBopen.org/svn/GBBopen/trunk/
include $(shell rospack find mk)/svn_checkout.mk

checkout: $(SVN_DIR)

test:
	if test -e gbbopen/patched; then echo "GBBopen already patched"; else make build; fi

build: 
	sh compile.sh
	rm -rf asdf
	mkdir asdf
	patch gbbopen/gbbopen.asd -i gbbopen.patch -o gbbopen/gbbopen-patched.asd
	rm gbbopen/gbbopen.asd
	ln -s `(rospack find gbbopen)`/gbbopen/gbbopen-patched.asd asdf/gbbopen.asd 
	touch gbbopen/patched

clean:
	rm -rf patched rospack_nosubdirs gbbopen asdf
