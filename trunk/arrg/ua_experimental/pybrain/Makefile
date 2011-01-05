all: installed

INSTALL_DIR = `rospack find pybrain`
GIT_DIR = build
GIT_URL = https://github.com/pybrain/pybrain.git
GIT_REVISION = master

include $(shell rospack find mk)/git_checkout.mk

installed: download clean
	mkdir -p src
	cd src && ln -s $(INSTALL_DIR)/$(GIT_DIR)/pybrain pybrain
	ln -s $(INSTALL_DIR)/$(GIT_DIR)/docs docs
	ln -s $(INSTALL_DIR)/$(GIT_DIR)/examples examples
	touch installed

clean:
	rm -rf src docs examples
	rm -rf installed

wipe: clean
	rm -rf build
	touch wiped

