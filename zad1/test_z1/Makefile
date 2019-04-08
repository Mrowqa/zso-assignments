TESTS_COMMON := memsz multiargs multicall stacksize
TESTS_GENERIC := exit missing-sym long-ptr mprotect mprotect2 cleanup
TESTS := $(TESTS_COMMON) $(TESTS_GENERIC)
CROSSLD := libcrossld.so
CFLAGS := -O3
CC := gcc

.PHONY: all
all: $(TESTS:%=%-64) $(TESTS:%=%-32)

clean:
	rm -f *-32 *-64 *.o

fakelib.so: fakelib.c
	gcc -m32 -shared -fPIC -o $@ $<

$(TESTS_COMMON:%=%-64):%-64: %-64.o common.o $(CROSSLD)
	gcc  -o $@ $(CROSSLD) common.o $<

$(TESTS_GENERIC:%=%-64):%-64: %-64.o $(CROSSLD)
	gcc  -o $@ $(CROSSLD) $<

%-32.o: %-32.c
	gcc -m32 -O3 -fno-stack-protector -c $<


%-32: %-32.o fakelib.so
	ld -m elf_i386 -o $@ $< fakelib.so
