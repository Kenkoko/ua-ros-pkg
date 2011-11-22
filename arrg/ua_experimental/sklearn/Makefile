all: installed

INSTALL_DIR = `rospack find sklearn`
BUILD_DIR = build

prep:
	mkdir -p src
	touch rospack_nosubdirs

get_learn:
	git clone git://github.com/scikit-learn/scikit-learn.git $(BUILD_DIR)/learn
	cd $(BUILD_DIR)/learn && git checkout master

install_learn: get_learn
	cd $(BUILD_DIR)/learn && python setup.py build_ext --inplace
	cd src && ln -s $(INSTALL_DIR)/$(BUILD_DIR)/learn/sklearn sklearn

installed: clean prep install_learn
	touch installed

clean:
	rm -rf src
	rm -rf installed

wipe: clean
	rm -rf $(BUILD_DIR)
	touch wiped

