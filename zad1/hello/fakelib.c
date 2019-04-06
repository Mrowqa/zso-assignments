void print() {}
_Noreturn void exit(int status) {__builtin_unreachable();}

long inc_l(long a) { return a+1; }
unsigned long inc_ul(unsigned long a) { return a+1; }
void *inc_ptr(void *a) { return a+1; }
long dec_l(long a) { return a-1; }
unsigned long dec_ul(unsigned long a) { return a-1; }
void *dec_ptr(void *a) { return a-1; }
long long inc_ll(long long a) { return a+1; }
unsigned long long inc_ull(unsigned long long a) { return a+1; }
