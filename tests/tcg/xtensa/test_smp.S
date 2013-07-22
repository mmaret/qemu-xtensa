.include "macros.inc"

test_suite smp

#if 1
test ipi_busy
	set_vector kernel, 12f
	movi	a2, 1
	wsr	a2, intenable
	movi	a4, 100f
	movi	a5, 1000

	rsr	a2, prid
	bnez	a2, 10f
	movi	a2, 0
	movi	a3, 0x200
	wer	a2, a3
	loop	a5, 3f
1:
	movi	a2, 0
	s32i	a2, a4, 4
	rsr	a2, ccount
	s32i	a2, a4, 0

	movi	a2, ~1
	movi	a3, 0x140
	wer	a2, a3
2:
	l32i	a2, a4, 4
	beqz	a2, 2b
	l32i	a3, a4, 0
	sub	a2, a2, a3
	l32i	a3, a4, 8
	maxu	a2, a2, a3
	s32i	a2, a4, 8
3:
	j	20f

10:
	loop	a5, 11f
	waiti	0
	nop
11:
	j	20f

12:
	rsr	a2, ccount
	s32i	a2, a4, 4
	rsr	a2, prid
	movi	a3, 0x100
	add	a3, a3, a2
	movi	a2, 1
	wer	a2, a3
	rfe

20:
	movi	a2, 101f
1:
	l32i	a3, a2, 0
	wsr	a3, scompare1
	addi	a4, a3, 1
	s32c1i	a4, a2, 0
	bne	a3, a4, 1b

1:
	l32i	a3, a2, 0
	bnei	a3, 2, 1b


.data
.align	4
100:
	.word	0, 0, 0
101:
	.word	0
.text

test_end

#endif

test ipi_rt
	movi	a2, 1
	wsr	a2, intenable
	movi	a4, 100f
	movi	a5, 1000

	rsr	a2, prid
	bnez	a2, 10f

	set_vector kernel, 4f
	movi	a2, 0
	movi	a3, 0x200
	wer	a2, a3
	loop	a5, 3f
1:
	movi	a2, 0
	s32i	a2, a4, 4
	rsr	a2, ccount
	s32i	a2, a4, 0

	movi	a2, ~1
	movi	a3, 0x140
	wer	a2, a3
2:
	waiti	0
	rsil	a2, 15
	l32i	a2, a4, 4
	beqz	a2, 2b
	l32i	a3, a4, 0
	sub	a2, a2, a3
	l32i	a3, a4, 8
	maxu	a2, a2, a3
	s32i	a2, a4, 8
3:
	j	20f
4:
	rsr	a2, prid
	movi	a3, 0x100
	add	a3, a3, a2
	movi	a2, 1
	wer	a2, a3
	rfe

10:
	set_vector kernel, 12f
	loop	a5, 11f
	waiti	0
	nop
11:
	j	20f

12:
	rsr	a2, ccount
	s32i	a2, a4, 4
	rsr	a2, prid
	movi	a3, 0x100
	add	a3, a3, a2
	movi	a2, 1
	wer	a2, a3

	movi	a2, 1
	movi	a3, 0x140
	wer	a2, a3
	rfe

20:
	movi	a2, 101f
1:
	l32i	a3, a2, 0
	wsr	a3, scompare1
	addi	a4, a3, 1
	s32c1i	a4, a2, 0
	bne	a3, a4, 1b

1:
	l32i	a3, a2, 0
	bnei	a3, 2, 1b

.data
.align	4
100:
	.word	0, 0, 0
101:
	.word	0
.text

test_end

test_suite_end
