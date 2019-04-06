default rel

global call_guest_code_start
global call_guest_code_start_rsp_storage_ptr
global call_guest_code_end
global call_guest_code
global call_guest_code_exit
global call_guest_code_exit_step2
global trampoline_32to64_start
global trampoline_32to64_end
global trampoline_32to64
global trampoline_32to64_func_ptr
global trampoline_32to64_code_ptr
global trampoline_32to64_exit_step2_ptr
global trampoline_32to64_required_stack_size
global trampoline_32to64_conv_args_ptr
global trampoline_32to64_conv_ret_val_ptr

[section .rodata]

; ---------------------- calling 32 bit process -------------------
[bits 64]
call_guest_code_start:
call_guest_code_start_rsp_storage_ptr: dq 0

; int call_guest_code(uint32_t entry_point, uint32_t new_stack);
; args: rdi, rsi
call_guest_code:
    ; save registers that need to be preserved
    push rbx
    push rbp
    push r12
    push r13
    push r14
    push r15
    
    ; remember rsp in memory
    mov rax, [call_guest_code_start_rsp_storage_ptr]
    mov [rax], rsp

    ; set new stack
    mov esp, esi

    ; and change cpu mode to 32 bits
    sub rsp, 8
    lea eax, [__call_guest_code_step2]
    mov dword [rsp+4], 0x23
    mov dword [rsp], eax
    retf

[bits 32]
__call_guest_code_step2:
    push 0x2b
    pop ds
    push 0x2b
    pop es
    jmp edi ; stack here is aligned (0 =(mod16))
            ; it's how it should be in _start

call_guest_code_exit:
    pop edi ; ret addr (trash)
    pop edi ; exit code
    call __get_eip3
__get_eip3:
    pop eax
    add eax, call_guest_code_exit_step2 - __get_eip3
    push 0x33
    push eax
    retf

[bits 64]
call_guest_code_exit_step2:
    mov eax, edi  ; exit code

    ; restore original stack
    mov rsp, [call_guest_code_start_rsp_storage_ptr]
    mov rsp, [rsp]

    ; restore saved registers
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbp
    pop rbx
    ret

call_guest_code_end:


; ---------------- 32-to-64 bit trampoline template ---------------
[bits 32]
trampoline_32to64_start:
trampoline_32to64_func_ptr: dq 0
trampoline_32to64_code_ptr: dq 0
trampoline_32to64_exit_step2_ptr: dq 0
trampoline_32to64_required_stack_size: dq 0
trampoline_32to64_conv_args_ptr: dq 0
trampoline_32to64_conv_ret_val_ptr: dq 0

trampoline_32to64:
    call __get_eip
__get_eip:
    pop eax
    add eax, __trampoline_step2 - __get_eip ; calc step2 address
    push 0x33
    push eax
    retf

; todo: (analyze)
;   + stack alignment
;   + saved registers

[bits 64]
__trampoline_step2:
    ; save registers which have to preserved in i386 calling convention
    mov r12d, esi
    mov r13d, edi
    
    ; remember addr of __trampoline_step2
    mov r14d, eax
    ; remember stack for easy deallocation
    mov r15, rsp

    ; align stack 0 mod 16
    sub rsp, 0xc

    ; convert arguments
    ; void convert_arguments(uint64_t *dst, uint32_t *src, struct function *fun, call_guest_exit_t exit_step2)
    lea rsi, [rsp + 0x10] ; 32bit stack args, 2nd arg
    sub rsp, [r14 + trampoline_32to64_required_stack_size - __trampoline_step2]
    mov rdi, rsp ; 64bit stack args, 1st arg
    mov rdx, [r14 + trampoline_32to64_func_ptr - __trampoline_step2] ; struct function*, 3rd arg
    mov rcx, [r14 + trampoline_32to64_exit_step2_ptr - __trampoline_step2] ; exit_step2, 4th arg
    call [r14 + trampoline_32to64_conv_args_ptr - __trampoline_step2]

    ; call given function
    pop rdi
    pop rsi
    pop rdx
    pop rcx
    pop r8
    pop r9
    call [r14 + trampoline_32to64_code_ptr - __trampoline_step2]

    ; convert result
    ; uint32_t convert_return_value(uint64_t value, struct function *fun, call_guest_exit_t exit_step2)
    mov rdi, rax
    mov rsi, [r14 + trampoline_32to64_func_ptr - __trampoline_step2]
    mov rdx, [r14 + trampoline_32to64_exit_step2_ptr - __trampoline_step2]
    call [r14 + trampoline_32to64_conv_ret_val_ptr - __trampoline_step2]
    mov rdx, rax
    shr rdx, 32   ; result is in edx:eax

    ; restore saved regs
    mov esi, r12d
    mov edi, r13d
    mov rsp, r15

    ; go back to 32 bits mode
    sub rsp, 8
    lea ecx, [__trampoline_step3]
    mov dword [rsp+4], 0x23
    mov dword [rsp], ecx
    retf

[bits 32]
__trampoline_step3:
    push 0x2b
    pop ds
    push 0x2b
    pop es
    ret  ; eax contains already converted result

trampoline_32to64_end:
