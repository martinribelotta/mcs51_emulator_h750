            ORG 0000h
            LJMP start

            ORG 0030h
start:      MOV SP, #70h
            MOV TMOD, #20h      ; Timer1 mode 2 (8-bit auto-reload)
            MOV TH1, #0FDh      ; 9600 baud @ 11.0592MHz, SMOD=0
            MOV TL1, #0FDh
            MOV PCON, #00h      ; SMOD=0
            MOV SCON, #50h      ; Mode 1, REN=1
            SETB TR1

            CLR P1.0

main_loop:  MOV DPTR, #msg
send_loop:  CLR A
            MOVC A, @A+DPTR
            JZ line_done
            ACALL putc
            INC DPTR
            SJMP send_loop

line_done:  CPL P1.0
            ACALL delay
            SJMP main_loop

putc:       MOV SBUF, A
wait_ti:    JNB TI, wait_ti
            CLR TI
            RET

delay:
            MOV R6, #04Ch
dly_outer:  MOV R5, #0FFh
dly_inner:  NOP
            NOP
            DJNZ R5, dly_inner
            DJNZ R6, dly_outer
            RET

msg:        DB 'Hello world', 0Dh, 0Ah, 00h
            END
