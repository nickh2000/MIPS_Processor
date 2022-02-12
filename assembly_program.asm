addi $1, $0, -5
addi $2, $0, -4
andi $3, $2, 0x80
bne $3, $0, invertOperand
beq $3, $0, loop
invertOperand: inv $2, $2
		addi $2, $2, 1
sw $3, 0($0)
addi $3, $0, 0
loop:	add $3, $3, $1
	addi $2, $2, -1
	bne $2, $0, loop
	lw $2, 0($0)
	bne $2, $0, invertResult
	beq $2, $0, exit
invertResult: inv $3, $3
		addi $3, $3, 1
exit:
	