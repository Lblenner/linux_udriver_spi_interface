CC = g++
CFLAGS  = -std=c++2a -Wall -Wextra

build:
	$(CC) $(CFLAGS) Spi_Interface.cpp -o spi_interface.so -shared -fpic


