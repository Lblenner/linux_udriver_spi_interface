CC = g++
CFLAGS  = -std=c++2a -Wall -Wextra



build:
	mkdir -p ../build/lib
	mkdir -p ../build/headers
	$(CC) $(CFLAGS) Spi_Interface.cpp -o ../build/lib/libspi_interface.so -shared -fpic
	chmod 0755 ../build/lib/libspi_interface.so
	cp *.hpp ../build/headers


