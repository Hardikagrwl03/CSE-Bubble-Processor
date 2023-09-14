.data
    array: .float  0.897, 0.565, 0.656, 0.1234, 0.665, 0.3434, 0.1126, 0.554, 0.3349, 0.678, 0.3656, 0.9989 
    size: .word 12
    buckets: .word 12
    inputp: .asciiz "Enter the size of the array(p):"
    inputn: .asciiz "Enter the number of buckets(n):"
    inputarr: .asciiz "Enter the array element:"
    inputnl: .asciiz "\n"
.text
.globl main
main:
    li $t0, 0       # t0 = i = 0
    move $s2, $sp   #creating the start pointer of the array

    la $s3, array
    lw $s0, size
    lw $s1, buckets

input_loop:
    beq $t0, $s0, end_input_loop      # check if 
    l.s $f0, 0($s3)
    s.s $f0, 0($sp)
    addi $sp, $sp, -4
    addi $s3, $s3, 4
    addi $t0, $t0, 1    # i=i+1
    j input_loop 

end_input_loop:
    li $t0, 0
    jal bucket_sort
    move $t1, $s2
    li $t0, 0
    jal print_arr
    
    li $v0, 10
    syscall

print_arr:
    bne $t0, $s0, print_float
    jr $ra
    
print_float:
    li $v0, 2
    l.s $f12, 0($t1)
    syscall                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
    li $v0, 4
    la $a0, inputnl
    syscall
    sub $t1, $t1, 4
    addi $t0, $t0, 1
    j print_arr

bucket_sort:
    sw $ra, 0($sp)
    move $s3, $sp
    addi $sp, $sp, -4

    move $s4, $sp
    addi $t1, $s0, 1
    mul $t2, $t1, $s1
    sll $t3, $t2, 2
    sub $sp, $sp, $t3

    li $t0, 0
    sll $t1, $t1, 2

size_init:
    beq $t0, $s1, end_size_init
    mul $t2, $t0, $t1
    sub $t3, $s4, $t2
    sw $zero, 0($t3)
    addi $t0, $t0, 1
    j size_init

end_size_init:

    li $t0, 0
    jal filling_buckets

    li $t0, 0
    move $t2, $s4

sort_bucket:
    beq $t0, $s1, end_sort_bucket

    lw $t3, 0($t2)       # (bucket[i]) $t3 stores the number of elements in the b
    move $s5, $t3
    addi $s6, $t2, -4    # $t4 first element of each bucket
        
    jal insertion_sort  

    sub $t2, $t2, $t1
    addi $t0, $t0, 1
    j sort_bucket

end_sort_bucket:

    li $t0, 0
    li $t2, 0

    jal concatenate_buckets

    lw $ra, 0($s3)
    jr $ra

filling_buckets:
    bne $t0, $s0, insert_in_bucket
    jr $ra

insert_in_bucket:
    sll $t2, $t0, 2
    sub $t3, $s2, $t2
    l.s $f2, 0($t3)
    mtc1 $s1, $f3
    cvt.s.w $f3, $f3
    mul.s $f4, $f3, $f2
    cvt.w.s $f4, $f4
    mfc1 $t4, $f4

    mul $t2, $t4, $t1
    sub $t5, $s4, $t2

    lw $t2, 0($t5)
    addi $t3, $t2, 1
    sw $t3, 0($t5)

    sll $t4, $t3, 2
    sub $t3, $t5, $t4
    s.s $f2, 0($t3) 


    addi $t0, $t0, 1
    j filling_buckets

insertion_sort:
    addi $sp, $sp, -8  # space for temp and j
    sw $t3, 0($sp)   # t0 for temp
    sw $t4, 4($sp)   # t1 for j
    li $t5,1         # t2 for i=1
    j outer_loop

outer_loop:
    ble $s5, $t5, exit_outer_loop  # if i>=p exit out of the loop
    
    sll $t7, $t5, 2
    sub $t7, $s6, $t7
    l.s $f3, 0($t7)  # temp=arr[i]
    lw $t3, 0($t7)

    addi $t4, $t5, -1  # j=i-1
    j inner_loop

inner_loop:
    blt $t4, 0, exit_inner_loop  # if j<0, exit out of the loop  
    sll $t8, $t4, 2
    sub $t8, $s6, $t8
    l.s $f6, 0($t8)
    lw $t6, 0($t8)
    ble $t6, $t3, exit_inner_loop # if arr[j]<=temp, exit out of the loop  
    addi $t8, $t8, -4
    s.s $f6, 0($t8)  # arr[j+1] = arr[j]
    
    addi $t4, $t4, -1  # j=j-1
    j inner_loop

exit_inner_loop:
    addi $t9, $t4, 1
    sll $t8, $t9, 2
    sub $t8, $s6, $t8

    s.s $f3, 0($t8)  # arr[j+1] = temp

    addi $t5, $t5, 1  # i=i+1
    j outer_loop

exit_outer_loop:
    lw $t3, 0($sp)  #temp   
    lw $t4, 4($sp)  #j
    addi $sp, $sp, 8
    jr $ra  # return

concatenate_buckets:
    li $t1, 0
    addi $t4, $s0, 1
    mul $t5, $t4, $t0
    sll $t4, $t5, 2
    sub $t3, $s4, $t4
    lw $s5, 0($t3)
        
    bne $t0, $s0, concat_bucket
    jr $ra

concat_bucket:
    beq $t1, $s5, concat_bucket_end

    addi $t4, $t1, 1
    sll $t5, $t4, 2
    sub $t4, $t3, $t5
    l.s $f2, 0($t4)

    sll $t4, $t2, 2
    sub $t5, $s2, $t4
    s.s $f2, 0($t5)

    addi $t2, $t2, 1
    addi $t1, $t1, 1
    j concat_bucket
    
concat_bucket_end:
    addi $t0, $t0, 1
    j concatenate_buckets