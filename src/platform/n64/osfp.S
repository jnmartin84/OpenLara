#define __mips 3
#if 1
#define USE_DOUBLE
#endif
#define zero $0
#define at $1
#define v0 $2
#define v1 $3
#define a0 $4
#define a1 $5
#define a2 $6
#define a3 $7
#define t0 $8
#define t1 $9
#define t2 $10
#define t3 $11
#define t4 $12
#define t5 $13
#define t6 $14
#define t7 $15
#define s0 $16
#define s1 $17
#define s2 $18
#define s3 $19
#define s4 $20
#define s5 $21
#define s6 $22
#define s7 $23
#define t8 $24
#define t9 $25
#define k0 $26
#define k1 $27
#define gp $28
#define sp $29
#define fp $30
#define ra $31


	.align 3
	.global	__osGetFpcCsr
	.set	nomips16
	.set	nomicromips
	.set	noreorder
	.set	nomacro
	.ent	__osGetFpcCsr
	.type	__osGetFpcCsr,	@function
	__osGetFpcCsr:
		cfc1  v0, $31
		jr    ra
		nop
	.set	macro
	.set	reorder
	.end	__osGetFpcCsr
    .size   __osGetFpcCsr, .-__osGetFpcCsr

	.align 3
	.global	__osSetFpcCsr
	.set	nomips16
	.set	nomicromips
	.set	noreorder
	.set	nomacro
	.ent	__osSetFpcCsr
	.type	__osSetFpcCsr,	@function
	__osSetFpcCsr:
cfc1  v0, $31
ctc1  a0, $31
jr    ra
	.set	macro
	.set	reorder
	.end	__osSetFpcCsr
    .size   __osSetFpcCsr, .-__osSetFpcCsr
