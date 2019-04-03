; todo:
;   + stack alignment
;   + saved registers

default rel

global call_32bits_code_start
global call_32bits_code_end
global call_32bits_code
global trampoline_32to64_start
global trampoline_32to64_end
global trampoline_32to64

[section .rodata]

; ---------------------- calling 32 bit process -------------------
[bits 64]
call_32bits_code_start:
; int call_32bits_code(uint32_t entry_point, uint32_t new_stack);
; args: rdi, rsi
call_32bits_code:
    ; save registers that need to be preserved
    push rbx
    push rbp
    push r12
    push r13
    push r14
    push r15
    
    ; remember rsp in 32 bits registers that has to preserved
    mov ebx, esp
    shr rsp, 32
    mov ebp, esp

    ; set new stack
    mov esp, esi

    ; and change cpu mode to 32 bits
    sub rsp, 8
    lea eax, [__call_32bits_code_step2]
    mov dword [rsp+4], 0x23
    mov dword [rsp], eax
    retf

[bits 32]
__call_32bits_code_step2:
    push 0x2b
    pop ds
    push 0x2b
    pop es
    jmp edi ; stack here is aligned (0 =(mod16))
            ; it's how it should be in _start

call_32bits_code_end:


; ---------------- 32-to-64 bit trampoline template ---------------
[bits 32]
trampoline_32to64_start:
func_ptr: dd 0
; patch args conv

trampoline_32to64:
    call __get_eip
__get_eip:
    pop eax
    push dword [eax + func_ptr - __get_eip] ; push func_ptr on stack
    add eax, __trampoline_step2 - __get_eip ; calc step2 address
    push 0x33
    push eax
    retf

; todo: stack alignment
[bits 64]
__trampoline_step2:
    ; save registers which have to preserved in i386 calling convention
    mov r12d, esi
    mov r13d, edi
    mov r14d, eax ; addr of __trampoline_step2

    ; convert args
    ; call given function
    ; convert result
    ; restore saved regs
    ; go back to 32 bits mode

    db 0xcc

[bits 32]
__trampoline_step3:
    ; set segment registers
    ; ret
    db 0xcc

trampoline_32to64_end:

