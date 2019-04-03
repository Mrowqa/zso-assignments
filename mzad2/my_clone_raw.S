.globl my_clone_raw

my_clone_raw:
    endbr64
    mov    $0xffffffffffffffea, %rax
    test   %rdi, %rdi
    je     .error
    test   %rsi, %rsi
    je     .error
    sub    $0x10, %rsi
    mov    %rcx, 0x8(%rsi)
    mov    %rdi, (%rsi)
    mov    %rdx, %rdi
    mov    %r8, %rdx
    mov    %r9, %r8
    mov    0x8(%rsp), %r10
    mov    $0x38, %eax
    syscall
    test   %rax, %rax
    jl     .error
    je     .child
    ret
.child:
    xor    %ebp, %ebp
    pop    %rax
    pop    %rdi
    call   *%rax
    mov    %rax, %rdi
    mov    $0x3c, %eax
    syscall
.error:
    ret

