.section ".text.startup"

.global _start
.global _get_stack_pointer
//.global _exception_table
//.global _enable_interrupts


_start:
    b start
undef_excp:
    subs pc, r14, #4 	/* return */
swi_excp:
    subs pc, r14, #0 	/* return */
prefetch_excp:
    subs pc, r14, #4	/* return */
abort_excp:
    subs pc, r14, #4 	/* return */
    nop			        /* reserver vector */
irq_excp:
    subs pc, r14, #4	/* return */
fiq_excp:
    subs pc, r14, #4


start:
    mov r0,  #0
        mov	r1,  r0
        mov r2,  r0
        mov r3,  r0
        mov r4,  r0
        mov r5,  r0
        mov r6,  r0
        mov r7,  r0
        mov r8,  r0
        mov r9,  r0
        mov r10, r0
        mov r11, r0
        mov r12, r0
    ldr sp, =(0x00000000 + (256*1024) - 4)
    b	_cstartup

_get_stack_pointer:
    // Return the stack pointer value
    str     sp, [sp]
    ldr     r0, [sp]

