asm (
	".global _start\n"
	"_start:\n"
	"int3\n"
	"call hello\n"
	"jmp 0x42424242\n" // debuggg harrddrrdd
	"hlt\n"
);

//int some_global_var = 42;
//char some_global_buffer[42];

_Noreturn void exit(int status);
void print(char *str);

void hello()
{
	print("Hello world");
	//exit(0);
}
