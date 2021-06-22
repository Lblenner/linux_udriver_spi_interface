CC = g++
CFLAGS  = -std=c++2a -Wall -Wextra

mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
mkfile_dir := $(dir $(mkfile_path))
lib_dir := $(mkfile_dir)../build/lib
head_dir := $(mkfile_dir)../build/headers

build:
	$(CC) $(CFLAGS)  maintain_position.cpp  -I$(head_dir) -L$(lib_dir) -lspi_interface -o exec -Wl,-rpath=$(lib_dir)

run:
	./exec


