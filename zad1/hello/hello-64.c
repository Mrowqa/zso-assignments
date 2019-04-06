#include <stdio.h>
#include "crossld.h"

static void print(char *data) {
	printf("%s\n", data);
	// __asm__ volatile(".byte 0xcc");
}

static long inc_l(long a) { return a+1; }
static unsigned long inc_ul(unsigned long a) { return a+1; }
static void *inc_ptr(void *a) { return a+1; }
static long dec_l(long a) { return a-1; }
static unsigned long dec_ul(unsigned long a) { return a-1; }
static void *dec_ptr(void *a) { return a-1; }
static long long inc_ll(long long a) { return a+1; }
static unsigned long long inc_ull(unsigned long long a) { return a+1; }

int main(int argc, char *argv[]) {
	int res;
	enum type print_types[] = {TYPE_PTR};
	enum type ll_types[] = {TYPE_LONG_LONG};
	enum type ull_types[] = {TYPE_UNSIGNED_LONG_LONG};
	enum type l_types[] = {TYPE_LONG};
	enum type ul_types[] = {TYPE_UNSIGNED_LONG};
	enum type ptr_types[] = {TYPE_PTR};
	struct function funcs[] = {
		{"print", print_types, 1, TYPE_VOID, print},
		{"inc_ll", ll_types, 1, TYPE_LONG_LONG, inc_ll},
		{"inc_ull", ull_types, 1, TYPE_UNSIGNED_LONG_LONG, inc_ull},
		{"inc_l", l_types, 1, TYPE_LONG, inc_l},
		{"inc_ul", ul_types, 1, TYPE_UNSIGNED_LONG, inc_ul},
		{"inc_ptr", ptr_types, 1, TYPE_PTR, inc_ptr},
		{"dec_l", l_types, 1, TYPE_LONG, dec_l},
		{"dec_ul", ul_types, 1, TYPE_UNSIGNED_LONG, dec_ul},
		{"dec_ptr", ptr_types, 1, TYPE_PTR, dec_ptr},
	};
	
	 res = crossld_start(argc > 1 ? argv[1] : "hello-32", funcs, 3);
	 printf("Result: %d\n", res);
	 return res;
}
