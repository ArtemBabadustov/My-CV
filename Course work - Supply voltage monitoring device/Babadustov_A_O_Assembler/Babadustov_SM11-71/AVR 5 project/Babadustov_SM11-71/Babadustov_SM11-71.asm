;*************************************
;* Title:          contr_UART.asm
;* Device          ATmega16
;* Clock frequency:������� ��.���������� 9,216 mHz
;*************************************
; �������
;*************************************
;��������/����� ������ �� USART �������������� �� ���������� �����������:
;�)�������� ������ 19200���/���;
;�)������ ������� 11 ��� ����������:
;      -  �����-���;
;      -  8 ��� ������;
;      -  ��� �������� �������� - ���������� �� ����������;
;      -  ���� ���� ���.
;������ USART(RXD) PD0 (����),(TXD) PD1(�����)
;�������� ������ : 
;1.��� � ������� ����� ����� ��������� �������� ���������� ����������, ���������� �� �������. 
;	� ������ ����� ����� ���������� �������� ���������� ������� ���������, �� ������ ����� �������� ���������� ������� ���������. 
;2.� ������ ������� ���������� ���� ���������� �� ����� �� ����������, ����� ������������ ��� �����: 
;	������ ���� ����� ��������� � ���� ��� �������, ������ ���� ����� ��������� � ���� ����� ������������ ���������. 
;3.�������� ���������� ����� �� ������� ��������� �������������� ����������� �������� �������:
;	����� ������������ ���� ���� � ������� ���������, �� ������� ����� �������������. 
;***********************************************
;***********************************************
.include "m16def.inc";"C:\Program Files (x86)\Atmel\AVR Tools\AvrAssembler\Appnotes\m16def.inc"
; ������������� ����� ��������; 
.list                   ;��������� ��������                                                                      
;*******************
;*******************
; Register Variables
;*******************
.def temp_L     =R16
.def temp_H     =R17
; 
.def Voltage1     =R18 
.def Voltage2   =R19
.def Current_Sourse = R20
.def Err_Sourse1 = R21
.def Err_Sourse2 = R22
.def Trans1 = R23
.def Trans2 = R24
.def  cou_ADC    =R25
;*******************
.def Byte_fl    =R2; ���� ������
.def  ADC_h      =R3
.def  ADC_l      =R4
.def Cou_Rec    =R5;������� �������� ����
.def Cou_Tran   =R6;������� ���������� ����
.def c_sumREC   =R7;����� ����� ����. ����
.def c_sumTRAN  =R8;����� ����� ���������� ����
;-----------------
.equ   F_receive   =1;���� ��������� �������
.equ   F_trans    =2;���� ���������� ��������
.equ  F_iz_kan    =0;���� ��������� ������:0 1-� ��������.1 2-� ��������
.equ  F_end_ADC   =1;���� ���������� (Val_N_ADC) �������������� ���
;******************* 
; Constants
;*****************
.equ Val_Voltage_Measurement=50;�������� ���������,���. ����� ����� ������� ����� ������������ �������� ���������� ���������� 
.equ   Val_N_ADC    =4;�����-�� �������������� ���(1,2,4,8.16..)
;========================================
;���������� ���� ������ � ����
;----------------------------------------
.equ	VAL_TR  =2
.equ	VAL_REC =1
.equ	VAL_time =17;35 ������������ ������������� 1 ���
.equ	Master_Adress = 0b01010101; ����� �������
.equ	My_Adress = 0b01010100; ����� �����
;***************************************
;Variable
;***************************************
.DSEG 
;���� ��� ��� ������ ���, �� ����� ������������.

;***************************************
.cseg
.org $0000
rjmp Init
;****************
;****************
.org  INT0addr;=$002	;External Interrupt0 Vector Address
reti;
;----------------
.org  INT1addr;=$004	;External Interrupt1 Vector Address
reti
.org  OC2addr; =$006	;Output Compare2 Interrupt Vector Address
reti
.org  OVF2addr;=$008	;Overflow2 Interrupt Vector Address
reti 
.org  ICP1addr;=$00A	;Input Capture1 Interrupt Vector Address
reti
.org  OC1Aaddr;=$00C	;Output Compare1A Interrupt Vector Address
//rjmp  Time_OUT
reti
.org  OC1Baddr;=$00E	;Output Compare1B Interrupt Vector Address
reti
.org  OVF1addr;=$010	;Overflow1 Interrupt Vector Address
reti
.org  OVF0addr;=$012	;Overflow0 Interrupt Vector Address
//rjmp  time_d_k ;
reti
.org  SPIaddr; =$014	;SPI Interrupt Vector Address
reti
.org  URXCaddr;=$016	;UART Receive Complete Interrupt Vector Address
rjmp  REC_date
.org  UDREaddr;=$018	;UART Data Register Empty Interrupt Vector Address
reti
.org UTXCaddr; =$01A	;UART Transmit Complete Interrupt Vector Address
rjmp  TRANdate
.org ADCCaddr; =$01C	;ADC Interrupt Vector Address
reti
.org ERDYaddr; =$01E	;EEPROM Interrupt Vector Address
reti
.org ACIaddr;  =$020	;Analog Comparator Interrupt Vector Address
reti
.org TWIaddr;  =$022    ;Irq. vector address for Two-Wire Interface
reti
.org INT2addr; =$024    ;External Interrupt2 Vector Address
reti
.org OC0addr;  =$026    ;Output Compare0 Interrupt Vector Address
reti
.org SPMRaddr; =$028    ;Store Program Memory Ready Interrupt Vector Address
reti
;***********************************
; Start Of Main Program
;***********************************
Init:
       ldi   temp_L,LOW(RAMEND);����� ������� �����
	   out   SPL, temp_L;��������� ����� 
	   ldi   temp_L,HIGH(RAMEND)
	   out   SPH,temp_L
;
; ------������������� ������ B/D
Init_B: ; � ����� ����� ���������� ����. PB0 - ���� 1, PB1 - ���� 2, PB2 - ���� 3, PB3 - ���� 4.
       ldi   temp_L,0b00011111;PC0-PC4-������, ��������� ������ �� ������������.
	   out   DDRB,temp_L
	   ldi   temp_L,0b11100000;��������� ��������� PC0-PC3 ������ ���� ������ �.� ��� �� ���������� ������ ������� V1 � V2. ���������� ������������� ���������������� �� �����.   
	   out   PORTB,temp_L;
;
Init_D:
       ldi   temp_L,0b00000010;PD1-�����(Txd0), PD0-����(Rxd0), ��������� ������ �� ������������.
	   out   DDRD,temp_L
	   ldi   temp_L,0b11111111;   
	   out   PORTD,temp_L;
;
;INIT USART
       ldi   temp_L,14;(������� ��. 9,216 ���,�������� ������ 19200),U2X0=0,
       ldi   temp_H,00 
	   out   UBRRL,temp_L;
	   out   UBRRH,temp_H
	   ldi   temp_L,(1<<RXEN)|(1<<RXCIE);UCSZ2=0,UCSZ1=1,UCSZ0=1 - 8 bit
	   out   UCSRB,temp_L
	   ldi   temp_L,(1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1)|(1<<UPM1);- 1 stop bit,
	   out   UCSRC,temp_L;
	   
; ---  �������������  ������� TCNT0

;
;      �������������  ������� TCNT1 ���� ������ ����� �������������� ��� ������� ����� ������� ��� �������� �������� ����������. 
       ldi   temp_L,00;compare A,(COM1A1,COM1A0=00) OC1A disconnect
       out   TCCR1A,temp_L;
	   ldi   temp_L,(1<<WGM12);WGM13=0,WGM12=1,WGM11=0,WGM10=0,����� CTC
;                             No clock source,CS42,CS41,CS40=000
	   out   TCCR1B,temp_L;(No prescaling CS10=1
	   ldi   temp_H,0xD8  ; time_out 6mcek - 0,006*9216000=55296(D800)
	   ldi   temp_L,0x00  ;
       out   OCR1AH,temp_H
	   out   OCR1AL,temp_L
;
       
       clr   Byte_fl



       sei             ;��������� ����������

;==================================================
;������ ����� ���������
/*� �������� ����� ��������� ������ ���������� ����������� ������ �������� ���������� ���������� �������. ����������� ������ ������ ����������� ����� ����� ������ ��������. 
�������� �������� ���������� ����� ����������� �� ���������� �� �������.
���� ������� ������������ ������� ���������� �� ����������.
��� ���������� �������: 
1.������� ������������ ������ ADC(� ��� ����� ������������ �������� Voltage1 � Voltage2 ) � ��������� ������
2.������� ������������� ������������ �������� ������(���������� ������ ��� ��������(Trans1, Trans2), � ������� ����� ���������� ������ ��� ��������) ��� ����� ���������� Transceive
3.������� ������������ ��������� ���������� � �������(� ��� ����� ����������� Trans1 � Trans2, �������� ���� ����� �� Volage1 � Voltage2. ����� ���������� ������������ ��������)
4.����������� � ����������� �������� ����������.
5.�������� ������������ voltege_drop_handler. � ��� ������������� �������� ����������, ����������� ���������� Trans1 � Trans2, ����� ���������� ������������ ��������)
6.� ������ �� ������ �� ������������ ����� ���������� ����� � ������� ���������� Trans1 ����� ��������������� ������. 
	Trans2 ����� ��������� �����: 
	�) �������� ��������� �������
	�) �������� �� ���������: ���������� �� ��������� ���� �����������
	�) �������� �� ���������: ������ ������ �������
7. ���� �������� ����������� ������� ���������� ������� ��������. ������� ����� ������������, ������� ����� ���������� ������� ����� ����� ����� ��������.
*/
;==================================================
Start: 
		sbrc   Byte_fl,F_receive;�������� ����� ��������� �������
		rjmp Receive 	
		rcall get_Voltage; � �������� ���������
		rjmp Start	

get_Voltage:
		rcall	start_ADC
		rcall	wait_ADC
		rcall	izm_Nkan; ���������� ����������� � ���������� izm_Nkan
		rcall	start_ADC
		rcall	wait_ADC
		rcall	izm_Nkan
		ret
wait_ADC:
		sbrs   byte_fl,F_end_ADC
		rjmp   wait_ADC
		rcall  out_ADC
		ret

; ������������ ����� ������ ��������������(����� �� ������)
;==================================================
izm_Nkan:
	   clt
         	   sbrs  byte_fl,F_iz_kan
	   set
        	   bld   byte_fl,F_iz_kan
;�������� � ������ ���
        	   in     temp_L,ADMUX
         	   andi   temp_L,0b01100000
ch_mux_ADC: sbrs  byte_fl,F_iz_kan
        	   rjmp  ex_c_mux
         	   ori   temp_L,(1<<MUX0)
ex_c_mux:	out   ADMUX,temp_L
        	   ret
;**************************************************
start_ADC:  
		ldi   temp_L,(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1);�-�� ������.64(125���)
		out   ADCSR,temp_L
		ret 
;������������ out_ADC ��������� ������ ��������� �� �������
;**************************************************
;������ � ��� ����������� ����� ������������ (Val_N_ADC=1,2,4,8.16) ��� � ����������
;�������� ��������, ��������� ��� ������� ����� ������
;���������� �������� Vin=(ADC*2,54)/1024=ADC/400. ���������� ������ ����� ����� ���
;������ �� �������, �.�. Vin*100. ����� ������� ���������� �������� ����� ADC/4
;���������� � 100 ��� ��� ������ �� ������� ������������ ������� ����� � ������
; ��.�����
;���-�� ������� ������ ��� ���-� �����.��������
out_ADC:    
		ldi    temp_L,Val_N_ADC;
		cpi    temp_L,8; 8 ����������
		brne   ch_4izm
		ldi    temp_L,3
		rjmp   sh_ADC
ch_4izm:    cpi    temp_L,4; 4 ����������
		brne   ch_2izm
        	ldi    temp_L,2
		rjmp   sh_ADC
ch_2izm:    cpi    temp_L,2; 2 ����������
		brne   ch_1izm 
		ldi    temp_L,1
		rjmp   sh_ADC  
ch_1izm:    cpi    temp_L,1
		breq   norm_ADC  
sh_ADC:  
		lsr    ADC_h 
		ror    ADC_l
		dec    temp_L
		cpi    temp_L,0x00
		brne   sh_ADC;� ADC_h,ADC_l ������� ��������
norm_ADC:   
		lsr    ADC_h 		   
		ror    ADC_l
		lsr    ADC_h 		   
		ror    ADC_l; � ADC_l ��������,����������� � 100 ���
		mov    temp_L,ADC_l
		clr   ADC_h
		clr   ADC_l
		sbrs  byte_fl,F_iz_kan
		mov Voltage1,temp_L
		sbrc byte_fl,F_iz_kan
		mov Voltage2,temp_L
		cpi	temp_l, 0x12; ���������� ���������� �������� ���������� � 18�.
		brlo voltege_drop_handler
		clt
		bld    byte_fl,F_end_ADC
		ret
;**************************************************
;������������ voltage_drop_handler ���������� ��������� �� ������ � ������ ������� ���������� �� ����� �� ����������
;� ����������� ����� �� ������ ��������������� ��������
voltege_drop_handler:
	sbrs  byte_fl,F_iz_kan
	rjmp Error_Sourse1
	sbrc byte_fl,F_iz_kan
	rjmp Error_Sourse2
Error_Sourse1:
	ldi Err_Sourse1, 0b00000001
	ldi Trans1, 0b11111111
	ldi Trans2, 0b00000001
	rjmp ex_voltege_drop_handler
Error_Sourse2: 
	ldi Err_Sourse2, 0b00000001
	ldi Trans1, 0b11111111
	ldi Trans2, 0b00000010
	rjmp ex_voltege_drop_handler
ex_voltege_drop_handler:
	rcall Transceive
	clt
	bld    byte_fl,F_end_ADC
	ret
;Subroutine interrupt ADC(� ������ �� ����)
;***********************************
IN_ADC:    push   temp_L
        	   push   temp_H
        	   in     temp_L,SREG
	   push   temp_L
rd_ADC:  in     temp_L,ADCL
	   in     temp_H,ADCH
	   add    ADC_l,temp_L
	   adc    ADC_h,temp_H 
         	   inc    cou_ADC
   cpi    cou_ADC,Val_N_ADC;�����-�� �������������� ���
         	   breq   end_ADC
        	   ldi   temp_L,(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADSC);;start convers.
        	   out    ADCSR,temp_L
ex_INADC:  pop    temp_L         
        	   out    SREG,temp_L
	   pop    temp_H
	   pop    temp_L
         	   reti
;-------------------------
end_ADC:   clr    cou_ADC
        	   set
	   bld    byte_fl,F_end_ADC
	   rjmp   ex_INADC
;*******************************************

;AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA       
Receive: 
       

;==================================================	   
;Subroutine interrupt USART RX Complete
;***********************************
REC_date: push   temp_L
          push   temp_H 
          in     temp_L,SREG
          push   temp_L
;
		  in    temp_H,UCSRA
rd_UDR:	  in    temp_L,UDR
		  rjmp   rt_rec
;  
Rec_Err: 
		  ldi Trans1, 0b00000001 ; � Trans1 ����������� ������������� ������ 
		  ldi Trans2, 0b00000001; � Trans2 ����������� ��� ������ ��������
		  rcall Transceive ; ���������� ������������ �������� 
          pop    temp_L
		  out    SREG,temp_L
		  pop    temp_H
		  pop    temp_L
		  reti
pop_rec: 
		pop    temp_L
		out    SREG,temp_L
		pop    temp_H
		pop    temp_L
		reti
;***********************************
rt_rec: 
          andi   temp_H,(1<<FE)|(1<<DOR)|(1<<PE)
          breq   USART2NoError
          rjmp   Rec_Err
USART2NoError:       ; ����� ����� ����������� ������������ �������� 
		cpi temp_L, 0b00000001
		breq Select1
		cpi temp_L, 0b00000010
		breq Select2
		cpi temp_L, 0b10101010
		breq Shot_Down
Select1: 
		cpi Err_Sourse1, 0b00000000
		brne Selected_Sourse_Fault
		ldi temp_L, 0b00000011
		out PORTB, temp_L
		rjmp Selection_Success
Select2: 
		cpi Err_Sourse2, 0b00000000
		brne Selected_Sourse_Fault
		ldi temp_L, 0b00001100
		out PORTB, temp_L
		rjmp Selection_Success
Shot_Down:
		ldi temp_L, 0b00000000
		out PORTB, temp_L
		rjmp Selection_Success
Selection_Success:
		ldi Trans1, 0b00000001 ; � Trans1 ����������� ������������� ������ �� ������
		ldi Trans2, 0b00000001
		rcall Transceive ; ���������� ������������ ��������
        rjmp pop_rec

Selected_Sourse_Fault:
		ldi Trans1, 0b00000001 ; � Trans1 ����������� ������������� ������ �� ������
		ldi Trans2, 0b00000010
		rcall Transceive ; ���������� ������������ �������� 
        rjmp pop_rec
rec_end: 
		com    temp_L;�������� ��������� �����
		cp     temp_L,c_sumREC; �������� ����� �����
		brne   pop_rec
		set
		bld    Byte_fl,F_receive;����� ������ ������
;          

;***********************************        
;Subroutine interrupt USART Data register Empty 
;***********************************
Transceive:  
		  sbi	PORTB, 4 ; ��������� RS485 ��������� � ����� ��������
		  ldi   temp_L,(1<<TXEN)|(1<<TXCIE);��������� ���������� TXC
	      out   UCSRB,temp_L
          bld   Byte_fl,F_receive;
		  out   UDR,Trans1
          ret 
;***********************************
;Subroutine interrupt USART, Tx Complete;
;***********************************
TRANdate: push   temp_L
          in     temp_L,SREG
          push   temp_L
		  push   temp_H
          out    UDR,Trans2
		  ldi    temp_L,(1<<RXEN)|(1<<RXCIE);
	      out    UCSRB,temp_L
		  cbi    PORTB,4; ��������� RS485 ��������� � ����� ������
		  set
		  bld    Byte_fl,F_trans
		  reti
;**********************************
;Subroutine interrupt OC1A
;**********************************
/*Time_OUT:  push   temp_L
           push   temp_H 
           in     temp_L,SREG
		   push   temp_L
;
           ldi    temp_L,(1<<WGM12);WGM13=0,WGM12=1,WGM11=0,WGM10=0,����� CTC
;                             No clock source,CS42,CS41,CS40=000 
           out    TCCR1B,temp_L;
		   in     temp_L,TIMSK
           clt
		   bld    temp_L,OCIE1A;OCIE1A ��������� ����������
           out    TIMSK,temp_L
           ldi    temp_H,0x00
		   ldi    temp_L,0x00  ;
           out    TCNT1H,temp_H;����� ��������
		   out    TCNT1L,temp_L
		   clr    Cou_Rec
		   clr    c_sumREC
		   ldi    YL,low(varBuf_Rxd)  ; Load Y register low ����� ������
           ldi    YH,high(varBuf_Rxd) ; Load Y register high ����� ������		   
;
		   pop    temp_L
		   out    SREG,temp_L
		   pop    temp_H 
           pop    temp_L
           reti*/
;**********************************