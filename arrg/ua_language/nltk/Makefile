DIR = nltk-2.0b8
TARBALL = $(DIR).tar.gz
TARBALL_URL = http://nltk.googlecode.com/files/$(TARBALL)

all: nltk

nltk:
	if test -e src/nltk; then echo "exists"; else make stuff; fi

stuff:
	wget $(TARBALL_URL)
	tar -xzvf $(TARBALL)
	mkdir -p src
	mv $(DIR)/nltk src
	rm $(TARBALL)
	rm -rf $(DIR)

clean:
	rm -rf $(DIR)
	rm -rf src
