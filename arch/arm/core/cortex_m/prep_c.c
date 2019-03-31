/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Full C support initialization
 *
 *
 * Initialization of full C support: zero the .bss, copy the .data if XIP,
 * call z_cstart().
 *
 * Stack is available in this module, but not the global data/bss until their
 * initialization is performed.
 */

#include <kernel.h>
#include <zephyr/types.h>
#include <toolchain.h>
#include <linker/linker-defs.h>
#include <kernel_internal.h>
#include <arch/arm/cortex_m/cmsis.h>
#include <cortex_m/stack.h>

#if defined(__GNUC__)
/*
 * GCC can detect if memcpy is passed a NULL argument, however one of
 * the cases of relocate_vector_table() it is valid to pass NULL, so we
 * supress the warning for this case.  We need to do this before
 * string.h is included to get the declaration of memcpy.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnonnull"
#endif

#include <string.h>

static inline void switch_sp_to_psp(void)
{
	__set_CONTROL(__get_CONTROL() | CONTROL_SPSEL_Msk);
	/*
	 * When changing the stack pointer, software must use an ISB instruction
	 * immediately after the MSR instruction. This ensures that instructions
	 * after the ISB instruction execute using the new stack pointer.
	 */
	__ISB();
}

static inline void set_and_switch_to_psp(void)
{
	u32_t process_sp;
	/*
	*MSP：主堆栈指针，当程序复位后（开始运行后），一直到第一次任务切换完成前，使用的都是MSP，即：main函数运行时用的是MSP，运行OSStartHighRdy，运行PendSV程序，用的都是MSP。当main函数开始运行前，启动文件会给这个函数分配一个堆栈空间，像ucos给任务分配堆栈空间一样，用于保存main函数运行过程中变量的保存。此时MSP就指向了该堆栈的首地址。
	*
	*PSP:进程堆栈指针，切换任务之后PendSV服务程序中有ORR LR, LR, #0x04这句，意思就是PendSV中断返回后使用的PSP指针，此时PSP已经指向了所运行任务的堆栈，所以返回后就可以就接着该任务继续运行下去了。
	*由于任何一个时刻都只能使用一个堆栈指针（SP），所以，如果在某一个时刻，需要读取或者改变另外一个堆栈指针的内容就得使用特定的指令：MSR和MRS
	*/
	//_interrupt_stack 在系统内核初始化期间，作为系统栈使用，因为在前面已经disable所有的interrupt
	//将PSP置于栈顶,栈为满递减的
	process_sp = (u32_t)&_interrupt_stack + CONFIG_ISR_STACK_SIZE;
	__set_PSP(process_sp);
	//sp切换后，必须加isb指令
	switch_sp_to_psp();
}

void lock_interrupts(void)
{
#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
	__disable_irq();
#elif defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
	__set_BASEPRI(_EXC_IRQ_DEFAULT_PRIO);
#else
#error Unknown ARM architecture
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */
}

#ifdef CONFIG_INIT_STACKS
static inline void init_stacks(void)
{
	memset(&_interrupt_stack, 0xAA, CONFIG_ISR_STACK_SIZE);
}
#endif

#ifdef CONFIG_CPU_CORTEX_M_HAS_VTOR

#ifdef CONFIG_XIP
#define VECTOR_ADDRESS ((uintptr_t)_vector_start)
#else
#define VECTOR_ADDRESS CONFIG_SRAM_BASE_ADDRESS
#endif
static inline void relocate_vector_table(void)
{
	SCB->VTOR = VECTOR_ADDRESS & SCB_VTOR_TBLOFF_Msk;
	__DSB();
	__ISB();
}

#else

#if defined(CONFIG_SW_VECTOR_RELAY)
Z_GENERIC_SECTION(.vt_pointer_section) void *_vector_table_pointer;
#endif

#define VECTOR_ADDRESS 0

void __weak relocate_vector_table(void)
{
#if defined(CONFIG_XIP) && (CONFIG_FLASH_BASE_ADDRESS != 0) || \
    !defined(CONFIG_XIP) && (CONFIG_SRAM_BASE_ADDRESS != 0)
	size_t vector_size = (size_t)_vector_end - (size_t)_vector_start;
	(void)memcpy(VECTOR_ADDRESS, _vector_start, vector_size);
#elif defined(CONFIG_SW_VECTOR_RELAY)
	_vector_table_pointer = _vector_start;
#endif
}

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#endif /* CONFIG_CPU_CORTEX_M_HAS_VTOR */

#ifdef CONFIG_FLOAT
static inline void enable_floating_point(void)
{
	/*
	 * Upon reset, the Co-Processor Access Control Register is 0x00000000.
	 * Enable CP10 and CP11 co-processors to enable floating point.
	 */
	SCB->CPACR |= CPACR_CP10_FULL_ACCESS | CPACR_CP11_FULL_ACCESS;
	/*
	 * Upon reset, the FPU Context Control Register is 0xC0000000
	 * (both Automatic and Lazy state preservation is enabled).
	 * Disable lazy state preservation so the volatile FP registers are
	 * always saved on exception.
	 */
	FPU->FPCCR = FPU_FPCCR_ASPEN_Msk; /* FPU_FPCCR_LSPEN = 0 */

	/*
	 * Although automatic state preservation is enabled, the processor
	 * does not automatically save the volatile FP registers until they
	 * have first been touched. Perform a dummy move operation so that
	 * the stack frames are created as expected before any thread
	 * context switching can occur. It has to be surrounded by instruction
	 * synchronization barriers to ensure that the whole sequence is
	 * serialized.
	 */
	__asm__ volatile(
		"isb;\n\t"
		"vmov s0, s0;\n\t"
		"isb;\n\t"
		);
}
#else
static inline void enable_floating_point(void)
{
}
#endif

extern FUNC_NORETURN void z_cstart(void);
/**
 *
 * @brief Prepare to and run C code
 *
 * This routine prepares for the execution of and runs C code.
 *
 * @return N/A
 */

extern void _IntLibInit(void);

#ifdef CONFIG_BOOT_TIME_MEASUREMENT
	extern u64_t __start_time_stamp;
#endif
void _PrepC(void)
{
#ifdef CONFIG_INIT_STACKS
	init_stacks();//初始化栈空间
#endif
	/*
	 * Set PSP and use it to boot without using MSP, so that it
	 * gets set to _interrupt_stack during initialization.
	 */
	set_and_switch_to_psp();
	//重定位中断向量表，如果定义了CONFIG_XIP，那么VECTOR_ADDRESS的值就位0（ROM的起始地址），如果没有定义CONFIG_XIP，那么VECTOR_ADDRESS的值就位RAM的起始地址
	relocate_vector_table();
	//使能浮点运算
	enable_floating_point();
	//bss段清0，bss全局未初始化变量或者初始化为0的变量
	z_bss_zero();
	//将已初始化的全局变量从rom中copy到ram中
	z_data_copy();
#ifdef CONFIG_BOOT_TIME_MEASUREMENT
	__start_time_stamp = 0U;
#endif
	//初始化中断，以确保在reset时，可以正常进行interrupt lock
	_IntLibInit();
	//初始化内核，准备执行C代码，此函数不会返回
	z_cstart();
	CODE_UNREACHABLE;
}
