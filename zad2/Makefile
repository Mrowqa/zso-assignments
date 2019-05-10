KDIR ?= /lib/modules/`uname -r`/build

default: harddoom2_tests
	$(MAKE) -C $(KDIR) M=$$PWD

install:
	$(MAKE) -C $(KDIR) M=$$PWD modules_install

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
	rm -f harddoom2_tests

harddoom2_tests: harddoom2_tests.c
	gcc harddoom2_tests.c -o harddoom2_tests
