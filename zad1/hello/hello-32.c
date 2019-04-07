asm (
	".global _start\n"
	"_start:\n"
	"call hello\n"
	"hlt\n"
);

//int some_global_var = 42;
//char some_global_buffer[42];

_Noreturn void exit(int status);
void print(char *str);

long inc_l(long a);
unsigned long inc_ul(unsigned long a);
void *inc_ptr(void *a);
long dec_l(long a);
unsigned long dec_ul(unsigned long a);
void *dec_ptr(void *a);
long long inc_ll(long long a); // { return a+1; }
long long my_inc_ll(long long a) { return a+1; }
unsigned long long inc_ull(unsigned long long a); // { return a+1; }
void multi_param(long a, long long b, int c);
void lots_of_params(int a, int b, int c, int d, int e, int f, int g, int h, int i);

unsigned long long hello()
{
	//long long a = inc_ll(0x1122334455667788);
	//unsigned long long b = inc_ull(0xffeeddccbbaa9900);
	//print("Hello world");
	//__asm__ volatile(".byte 0xcc");
	//exit((int) dec_ptr(0));
	multi_param(0xdeafbeef, 0x0123456789abcdef, 0xc0dec0de);
	lots_of_params(1, 2, 3, 4, 5, 6, 7, 8, 9);
	//exit(dec_l(0x0));
	//exit(dec_l(0x0));
	//exit(dec_l(0x0));
	//exit(dec_l(0x0));
	//exit(dec_l(0x0));
	//exit(42);
	//return a+b;
	//return inc_ll(0x1122334455667788) == my_inc_ll(0x1122334455667788);
	exit(0);
}
