.data
    n: .word 5
    a: .word 3
    b: .word 1
    c: .word 5
    d: .word 2
    e: .word 4
.text
.globl __start



swap: 
    slli x6,x11,2   # reg x6 = k * 4 
    add x6,x10,x6   # reg x6 = v + (k * 4) 
    lw x5,0(x6)     # reg x5 (temp) = v[k] 
    lw x7,4(x6)     # reg x7 = v[k + 1] 
    sw x7,0(x6)     # v[k] = reg x7 
    sw x5,4(x6)     # v[k+1] = reg x5 (temp) 
    jalr x0,0(x1)   # return to calling routine 
    
sort:
    addi sp,sp,-20  # make room on stack for 5 regs
    sw x1,16(sp)    # save x1 on stack 
    sw x22,12(sp)   # save x22 on stack 
    sw x21,8(sp)   # save x21 on stack 
    sw x20,4(sp)    # save x20 on stack 
    sw x19,0(sp)    # save x19 on stack
    
    mv x21, x10     # copy parameter x10 into x21 
    mv x22, x11     # copy parameter x11 into x22 
    li x19,0        # i = 0 
for1tst: 
    bge x19,x22,exit1   # go to exit1 if x19 ≥ x11 (i≥n) 
    addi x20,x19,-1     # j = i −1 
for2tst: 
    blt x20,x0,exit2    # go to exit2 if X20 < 0 (j < 0) 
    slli x5,x20,2       # reg x5 = j * 4
    add x5,x21,x5       # reg x5 = v + (j * 4) 
    lw x6,0(x5)         # reg x6 = v[j] 
    lw x7,4(x5)         # reg x7 = v[j + 1] 
    ble x6,x7,exit2 # go to exit2 if x6 ≤ x7 
    mv x10, x21     # first swap parameter is v 
    mv x11, x20     # second swap parameter is j 
    jal x1,swap     # call swap 
    addi x20,x20,-1 # j –= 1 
    j for2tst       # branch to test of inner loop 
exit2: 
    addi x19,x19,1  # i += 1 
    j for1tst       # branch to test of outer loop 
exit1:
    lw x19,0(sp)    # restore x19 from stack 
    lw x20,4(sp)    # restore x20 from stack 
    lw x21,8(sp)   # restore x21 from stack 
    lw x22,12(sp)   # restore x22 from stack 
    lw x1,16(sp)    # restore x1 from stack 
    addi sp,sp, 20  # restore stack pointer 
    jalr x0,0(x1) 


# Do NOT modify this part!!!
__start:
    la   x10, a
    la   t0, n
    lw   x11, 0(t0)
    jal  x1, sort
    addi a0,x0,10
    ecall