GIT_VERSION := $(shell git describe --abbrev --dirty --always --tags)

version:
	echo "#define VERSION \"$(GIT_VERSION)\"" > version.h
