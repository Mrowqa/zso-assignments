asm (
	".global _start\n"
	"_start:\n"
	// "int3\n"
	"call hello\n"
	"jmp 0x42424242\n" // debuggg harrddrrdd
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

unsigned long long hello()
{
	//long long a = inc_ll(0x1122334455667788);
	//unsigned long long b = inc_ull(0xffeeddccbbaa9900);
	print("Hello world");
	//__asm__ volatile(".byte 0xcc");
	exit(inc_ul(-3)); // todo debug
	//exit(dec_l(0x0));
	//exit(dec_l(0x0));
	//exit(dec_l(0x0));
	//exit(dec_l(0x0));
	//exit(dec_l(0x0));
	exit(42);
	//return a+b;
	//return inc_ll(0x1122334455667788) == my_inc_ll(0x1122334455667788);
}
