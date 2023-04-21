all : testnix

testnix : testnix.c
	gcc -o $@ $^ ../../ch32v003fun/minichlink/minichlink.so -lX11

clean :
	rm -rf testnix
