all: libcrossld.so
clean:
	rm -f *.so *.o

libcrossld.so: crossld.c crossld_asm.S
	nasm -f elf64 crossld_asm.S
	gcc -shared -fPIC -o libcrossld.so crossld.c crossld_asm.o
