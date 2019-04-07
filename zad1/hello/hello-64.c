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
static void multi_param(long a, long long b, int c) {
	printf("%x, %lx, %x\n", a, b, c);
}
static void lots_of_params(int a, int b, int c, int d, int e, int f, int g, int h, int i) {
	printf("%d, %d, %d, %d, %d, %d, %d, %d, %d\n", a, b, c, d, e, f, g, h, i);
}

int main(int argc, char *argv[]) {
	int res;
	enum type print_types[] = {TYPE_PTR};
	enum type ll_types[] = {TYPE_LONG_LONG};
	enum type ull_types[] = {TYPE_UNSIGNED_LONG_LONG};
	enum type l_types[] = {TYPE_LONG};
	enum type ul_types[] = {TYPE_UNSIGNED_LONG};
	enum type ptr_types[] = {TYPE_PTR};
	enum type multi_param_types[] = {TYPE_LONG, TYPE_LONG_LONG, TYPE_INT};
	enum type lots_of_params_types[] = {TYPE_INT, TYPE_INT, TYPE_INT, TYPE_INT, TYPE_INT, TYPE_INT, TYPE_INT, TYPE_INT, TYPE_INT};
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
		{"multi_param", multi_param_types, 3, TYPE_VOID, multi_param},
		{"lots_of_params", lots_of_params_types, 9, TYPE_VOID, lots_of_params},
	};
	
	 res = crossld_start(argc > 1 ? argv[1] : "hello-32",
	 	funcs, sizeof(funcs) / sizeof(struct function));
	 printf("Result: %d\n", res);
	 return res;
}
