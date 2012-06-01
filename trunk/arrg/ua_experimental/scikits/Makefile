all: installed

INSTALL_DIR = `rospack find scikits`
BUILD_DIR = build

prep:
	mkdir -p src/scikits
	cd src/scikits && echo "__import__('pkg_resources').declare_namespace(__name__)" > __init__.py
	touch rospack_nosubdirs

get_learn:
	git clone https://github.com/scampion/scikit-learn.git $(BUILD_DIR)/learn
	cd $(BUILD_DIR)/learn && git checkout master

get_audiolab:
	git clone https://github.com/cournape/audiolab.git $(BUILD_DIR)/audiolab
	cd $(BUILD_DIR)/audiolab && git checkout master

install_learn: get_learn
	cd $(BUILD_DIR)/learn && python setup.py build_ext --inplace
	cd src/scikits && ln -s $(INSTALL_DIR)/$(BUILD_DIR)/learn/scikits/learn learn

install_audiolab: get_audiolab
	cd $(BUILD_DIR)/audiolab && python setup.py build_ext --inplace
	cd src/scikits && ln -s $(INSTALL_DIR)/$(BUILD_DIR)/audiolab/scikits/audiolab audiolab

installed: clean prep install_learn install_audiolab
	touch installed

clean:
	rm -rf $(BUILD_DIR) src
	rm -rf installed

wipe: clean
	touch wiped

