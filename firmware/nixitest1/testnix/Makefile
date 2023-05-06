all : testnix

testnix : testnix.c
	gcc -o $@ $^ ../../ch32v003fun/minichlink/minichlink.so -lX11 -DMINICHLINK_AS_LIBRARY

clean :
	rm -rf testnix
