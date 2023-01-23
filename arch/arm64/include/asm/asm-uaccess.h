#ifndef __ASM_ASM_UACCESS_H
#define __ASM_ASM_UACCESS_H

#include <asm/alternative.h>
#include <asm/kernel-pgtable.h>
#include <asm/sysreg.h>
#include <asm/assembler.h>
#include <asm/mmu.h>

/*
 * User access enabling/disabling macros.
 */
#ifdef CONFIG_ARM64_SW_TTBR0_PAN
	.macro	__uaccess_ttbr0_disable, tmp1
	mrs	\tmp1, ttbr1_el1		// swapper_pg_dir
	bic     \tmp1, \tmp1, #TTBR_ASID_MASK
	add	\tmp1, \tmp1, #SWAPPER_DIR_SIZE	// reserved_ttbr0 at the end of swapper_pg_dir
	msr	ttbr0_el1, \tmp1		// set reserved TTBR0_EL1
	isb
	sub     \tmp1, \tmp1, #SWAPPER_DIR_SIZE
	msr     ttbr1_el1, \tmp1                // set reserved ASID
	isb
	.endm

	.macro	__uaccess_ttbr0_enable, tmp1, tmp2
	get_thread_info \tmp1
	ldr	\tmp1, [\tmp1, #TSK_TI_TTBR0]	// load saved TTBR0_EL1
	mrs     \tmp2, ttbr1_el1
	extr    \tmp2, \tmp2, \tmp1, #48
	ror     \tmp2, \tmp2, #16
	msr     ttbr1_el1, \tmp2                // set the active ASID
	isb
	msr	ttbr0_el1, \tmp1		// set the non-PAN TTBR0_EL1
	isb
	.endm

	.macro	uaccess_ttbr0_disable, tmp1, tmp2
alternative_if_not ARM64_HAS_PAN
	save_and_disable_irq \tmp2		// avoid preemption
	__uaccess_ttbr0_disable \tmp1
	restore_irq \tmp2
alternative_else_nop_endif
	.endm

	.macro	uaccess_ttbr0_enable, tmp1, tmp2, tmp3
alternative_if_not ARM64_HAS_PAN
	save_and_disable_irq \tmp3		// avoid preemption
	__uaccess_ttbr0_enable \tmp1, \tmp2
	restore_irq \tmp3
alternative_else_nop_endif
	.endm
#else
	.macro	uaccess_ttbr0_disable, tmp1, tmp2
	.endm

	.macro	uaccess_ttbr0_enable, tmp1, tmp2, tmp3
	.endm
#endif

/*
 * Remove the address tag from a virtual address, if present.
 */
	.macro	clear_address_tag, dst, addr
	tst	\addr, #(1 << 55)
	bic	\dst, \addr, #(0xff << 56)
	csel	\dst, \dst, \addr, eq
	.endm

/*
 * Generate the assembly for UAO alternatives with exception table entries.
 * This is complicated as there is no post-increment or pair versions of the
 * unprivileged instructions, and USER() only works for single instructions.
 */
#ifdef CONFIG_ARM64_UAO
	.macro uao_ldp l, reg1, reg2, addr, post_inc
		alternative_if_not ARM64_HAS_UAO
8888:			ldp	\reg1, \reg2, [\addr], \post_inc;
8889:			nop;
			nop;
		alternative_else
			ldtr	\reg1, [\addr];
			ldtr	\reg2, [\addr, #8];
			add	\addr, \addr, \post_inc;
		alternative_endif

		_asm_extable	8888b,\l;
		_asm_extable	8889b,\l;
	.endm

	.macro uao_stp l, reg1, reg2, addr, post_inc
		alternative_if_not ARM64_HAS_UAO
8888:			stp	\reg1, \reg2, [\addr], \post_inc;
8889:			nop;
			nop;
		alternative_else
			sttr	\reg1, [\addr];
			sttr	\reg2, [\addr, #8];
			add	\addr, \addr, \post_inc;
		alternative_endif

		_asm_extable	8888b,\l;
		_asm_extable	8889b,\l;
	.endm

	.macro uao_user_alternative l, inst, alt_inst, reg, addr, post_inc
		alternative_if_not ARM64_HAS_UAO
8888:			\inst	\reg, [\addr], \post_inc;
			nop;
		alternative_else
			\alt_inst	\reg, [\addr];
			add		\addr, \addr, \post_inc;
		alternative_endif

		_asm_extable	8888b,\l;
	.endm
#else
	.macro uao_ldp l, reg1, reg2, addr, post_inc
		USER(\l, ldp \reg1, \reg2, [\addr], \post_inc)
	.endm
	.macro uao_stp l, reg1, reg2, addr, post_inc
		USER(\l, stp \reg1, \reg2, [\addr], \post_inc)
	.endm
	.macro uao_user_alternative l, inst, alt_inst, reg, addr, post_inc
		USER(\l, \inst \reg, [\addr], \post_inc)
	.endm
#endif

#endif
