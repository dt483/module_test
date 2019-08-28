//.section ".text.startup"
.section ".init"

.global _start
.global main


// See ARM section A2.2 (Processor Modes)

.equ    CPSR_MODE_USER,         0x10
.equ    CPSR_MODE_FIQ,          0x11
.equ    CPSR_MODE_IRQ,          0x12
.equ    CPSR_MODE_SVR,          0x13
.equ    CPSR_MODE_ABORT,        0x17
.equ    CPSR_MODE_UNDEFINED,    0x1B
.equ    CPSR_MODE_SYSTEM,       0x1F

// See ARM section A2.5 (Program status registers)
.equ    CPSR_IRQ_INHIBIT,       0x80
.equ    CPSR_FIQ_INHIBIT,       0x40
.equ    CPSR_THUMB,             0x20

//IRQ mdode stack size

.global IRQ_STACK_SIZE
.equ 	IRQ_STACK_SIZE, 		0x800
.global USER_STACK_SIZE
.equ 	USER_STACK_SIZE, 		0x800
.global SVR_STACK_SIZE
.equ 	SVR_STACK_SIZE, 		0x8000


_start:
    ldr pc, _reset_h
    ldr pc, _undefined_instruction_vector_h
    ldr pc, _software_interrupt_vector_h
    ldr pc, _prefetch_abort_vector_h
    ldr pc, _data_abort_vector_h
    nop //ldr pc, _unused_vector_h
    ldr pc, _interrupt_vector_h
    ldr pc, _fast_interrupt_vector_h

	//setup USER mode stack size
	ldr r0, = __stacktop
	mov sp, r0
	//generating SWI for enter to SWi modede then enter SVR mode
	swi 0


_reset_h:                           .word   _reset_;
_undefined_instruction_vector_h:    .word   _reset_;
_software_interrupt_vector_h:       .word   _reset_;
_prefetch_abort_vector_h:           .word   _prefetch_abort_handler;
_data_abort_vector_h:               .word   _data_abort_handler;
//_unused_vector_h:                   .word   _unused_vector_h
_interrupt_vector_h:                .word   _interrupt_handler;
_fast_interrupt_vector_h:           .word   _fast_interrupt_handler;



.section ".text.init"
_software_interrupt_handler:
	msr cpsr_c, #(CPSR_MODE_SVR | CPSR_IRQ_INHIBIT | CPSR_FIQ_INHIBIT )
	b _reset_

_reset_:
	// We enter execution in supervisor mode.

//	add r0, pc ,#0xC0000000
//	mov pc, r0
	// We're going to use interrupt mode, so setup the interrupt mode
    // stack pointer which differs to the application stack pointer:
	// interrupt mode stack init
    ldr r0, = __stacktop// 0x7FFFF - RAM TOP
  	sub r0, r0, #USER_STACK_SIZE

	msr cpsr_c, #(CPSR_MODE_IRQ | CPSR_IRQ_INHIBIT | CPSR_FIQ_INHIBIT )
    mov sp, r0
    sub r0, r0, #IRQ_STACK_SIZE


	// supervisor mode stack init
	msr cpsr_c, #(CPSR_MODE_SVR | CPSR_IRQ_INHIBIT | CPSR_FIQ_INHIBIT )
    mov sp, r0

	//erasing non initialized data (.bss section)
	mov     r0, #0
	ldr     r1, =__bss_start__
	ldr     r2, =__bss_end__
	LoopBss:
	cmp     r1, r2
	strlo   r0, [r1], #4
	blo     LoopBss

	   // Enable VFP ------------------------------------------------------------
 /*   // r1 = Access Control Register
    MRC p15, #0, r1, c1, c0, #2
    // enable full access for p10,11
    ORR r1, r1, #(0xf << 20)
    // ccess Control Register = r1
    MCR p15, #0, r1, c1, c0, #2
    MOV r1, #0
    // flush prefetch buffer because of FMXR below
    MCR p15, #0, r1, c7, c5, #4
    // and CP 10 & 11 were only just enabled
    // Enable VFP itself
    MOV r0,#0x40000000
    // FPEXC = r0
    FMXR FPEXC, r0*/

	//Starting main
	ldr     r0,=main
	bx      r0


_undefined_instruction_handler:
	//Infinite loop _undefined_instruction_handler
	b       _undefined_instruction_handler

_prefetch_abort_handler:
	//Infinite loop _prefetch_abort_handler
	b       _prefetch_abort_handler

_data_abort_handler:
	//Infinite loop  _data_abort_handler
	b       _data_abort_handler

_fast_interrupt_handler:
	//Infinite loop _fast_interrupt_handler
	b       _fast_interrupt_handler
_interrupt_handler:
//Infinite loop _fast_interrupt_handler
	b       _interrupt_handler
/*_software_interrupt_handler:
	b      _software_interrupt_handler*/

.global _get_stack_pointer
_get_stack_pointer:
    // Return the stack pointer value
    str     sp, [sp]
    ldr     r0, [sp]
    mov     pc, lr




