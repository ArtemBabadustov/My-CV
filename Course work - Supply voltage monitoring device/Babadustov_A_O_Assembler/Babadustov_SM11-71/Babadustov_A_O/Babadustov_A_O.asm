;*************************************
;* Device          ATmega16
;* Clock frequency:Частота кв.резонатора 9,216 mHz
;*************************************
;*************************************
;Программа  осуществляет обмен по интерфейсу RS-485 между главным компьютером
;и модулем резервирования питания. Преобразование в интерфейс 
;RS-485 выполняется микросхемой ADM2461E, установленной на модуле.
;Передача/прием данных по USART осуществляется со следующими параметрами:
;а)скорость обмена 19200бит/сек;
;б)формат посылки 11 бит информации:
;      -  старт-бит;
;      -  9 бит данных;
;      -  один стоп бит.
;Выводы USART(RXD) PD0 (вход),(TXD) PD1(выход)
;***********************************************
;***********************************************
.include "m16def.inc";"C:\Program Files (x86)\Atmel\AVR Tools\AvrAssembler\Appnotes\m16def.inc"
; присоединение файла описаний; 
.list                   ;включение листинга                                                                      
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
.def Err_Sourse = R21
//.def Err_Sourse2 = R22
.def Trans1 = R23
.def Trans2 = R24
.def  cou_ADC    =R25
;*******************
.def Byte_fl    =R2; байт флагов
.def  ADC_h      =R3
.def  ADC_l      =R4
.def Cou_Rec    =R5;счетчик принятых байт
.def Cou_Tran   =R6;счетчик переданных байт

;-----------------
.equ   F_receive   =1;флаг принятого запроса
.equ   F_trans    =2;флаг завершения передачи
.equ  F_iz_kan    =0;флаг изменения канала:0 1-й источник.1 2-й источник
.equ  F_end_ADC   =1;флаг завершения (Val_N_ADC) преобразований АЦП
;******************* 
; Constants
;*****************
.equ   Val_N_ADC    =1;колич-во преобразований АЦП(1,2,4,8.16..)
;========================================
;Количество байт обмена с ПЭВМ
;----------------------------------------
.equ	VAL_TR  =3
.equ	VAL_REC =2
//.equ	Master_Adress = 0b01010101; Адрес мастера
.equ	My_Adress = 0b01010100; Адрес платы
;***************************************
;Variable
;***************************************
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
//rjmp  One_Second_Passed
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
rjmp IN_ADC
.org ERDYaddr; =$01E	;EEPROM Interrupt Vector Address
reti
.org ACIaddr;  =$020	;Analog Comparator Interrupt Vector Address
reti
.org TWIaddr;  =$022    ;Irq. vector address for Two-Wire Interface
reti
.org INT2addr; =$024    ;External Interrupt2 Vector Address
reti
.org OC0addr;  =$026    ;Output Compare0 Interrupt Vector Address
rjmp TimeOUT
.org SPMRaddr; =$028    ;Store Program Memory Ready Interrupt Vector Address
reti
;***********************************
; Start Of Main Program
;***********************************
Init:
       ldi   temp_L,LOW(RAMEND);выбор вершины стека
	   out   SPL, temp_L;Указатель стека 
	   ldi   temp_L,HIGH(RAMEND)
	   out   SPH,temp_L
;
; ------Инициализация портов B/D
Init_B: ; К этому порту подключены реле. PB0 - реле 1, PB1 - реле 2, PB2 - реле 3, PB3 - реле 4.
       ldi   temp_L,0b00011111;PC0-PC4-выходы, остальные выводы не используются.
	   out   DDRB,temp_L
	   ldi   temp_L,0b11100000;Начальное состояние PC0-PC3 должно быть низким т.к ещё не определены уровни питания V1 и V2. Изначально устанавливаем приемопередатчик на прием.   
	   out   PORTB,temp_L;
;
Init_D:
       ldi   temp_L,0b00000010;PD1-выход(Txd0), PD0-вход(Rxd0), остальные выводы не используются.
	   out   DDRD,temp_L
	   ldi   temp_L,0b11111111;   
	   out   PORTD,temp_L;
;
;INIT USART
       ldi   temp_L,0x29;(Частота кв. 9,216 мГц,скорость обмена 19200),U2X0=0,
       ldi   temp_H,0x00 
	   out   UBRRL,temp_L;
	   out   UBRRH,temp_H
	   ldi   temp_L,(1<<RXEN)|(1<<RXCIE)|(1<<UCSZ2);
	   out   UCSRB,temp_L
	   ldi   temp_L,(1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1)|(1<<UCSZ2)|(1<<URSEL)
	   out   UCSRC,temp_L;
	   ldi   temp_L, (1<<MPCM)					
	   out   UCSRA,temp_L
	   
; ---  Инициализация  таймера TCNT0 

	   	ldi   temp_L,0x11  			;
	   	out   OCR0,temp_L
		ldi   temp_L,(1<<WGM12)|(1<<CS02)|(0<<CS01)|(1<<CS00)	;WGM13=0,WGM12=1,WGM11=0,WGM10=0 - режим CTC
	   	out   TCCR0,temp_L			;timeOUT = (11 * 1) / 19200 = 0.5 ms => возьмём с запасом 2 ms
	   								;2 ms => 0,002*9216000/1024 = 18(0xF12)
       
       clr   Byte_fl
;---  Инициализация ADC
		ldi   temp_L,0b00000000
		out   ADMUX,temp_L



       sei             ;разрешаем прерывания

;==================================================
;начало цикла программы
;==================================================
Start: 
		rcall	start_ADC
		rcall	wait_ADC
		rcall	izm_Nkan; 
		rcall	start_ADC
		rcall	wait_ADC
		rcall	izm_Nkan
		rjmp Start
wait_ADC:
		sbrs   byte_fl,F_end_ADC
		rjmp   wait_ADC
		rcall  out_ADC
		ret

; Подпрограмма смены канала преобразования
;==================================================
izm_Nkan:
	   clt
         	   	sbrs  byte_fl,F_iz_kan
	   set
        	  	bld   byte_fl,F_iz_kan
;изменить № канала АЦП
         	   	ldi   temp_L,0b00100000
ch_mux_ADC: 	sbrs  byte_fl,F_iz_kan
				ldi   temp_L,0b00100001
				out   ADMUX,temp_L
        	   	ret
;**************************************************
start_ADC:  
		ldi   temp_L,(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADSC);ч-та преобр.64(144кГц)
		out   ADCSRA,temp_L
		ret 
;**************************************************
;данные с АЦП усредняются путем суммирования (Val_N_ADC=1,2,4,8.16) раз и нахождения
;среднего значения, используя для деления сдвиг вправо
;кол-во сдвигов вправо для нах-я средн.значения
out_ADC:    
		ldi    temp_L,Val_N_ADC;
		cpi    temp_L,8; 8 накоплений
		brne   ch_4izm
		ldi    temp_L,3
		rjmp   sh_ADC
ch_4izm:    cpi    temp_L,4; 4 накоплений
		brne   ch_2izm
        	ldi    temp_L,2
		rjmp   sh_ADC
ch_2izm:    cpi    temp_L,2; 2 накоплений
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
		brne   sh_ADC;в ADC_h,ADC_l среднее значение
norm_ADC:   
		lsr    ADC_h 		   
		ror    ADC_l
		lsr    ADC_h 		   
		ror    ADC_l; после двойного сдвига вправо в ADC_l хранится значение напряжения в формате числа с фиксированной запятой, запятая стоит между 3 и 4 разрядами 
		mov    temp_L,ADC_l
		clr   ADC_h
		clr   ADC_l
		sbrs  byte_fl,F_iz_kan
		mov Voltage1,temp_L
		sbrc byte_fl,F_iz_kan
		mov Voltage2,temp_L
		cpi	temp_l, 0x90; сравниваем полученное значение напряжения с 18В. В выбраном формате числа с плавающей запятой 18 = 10010.000
		brlo voltege_drop_handler
		clt
		bld    byte_fl,F_end_ADC
		ret
;**************************************************
;Подпрограмма voltage_drop_handler отправляет сообщение об ошибке в случае падения напряжения на одном из источников
;И переключает выход на штатно функционирующий источник
voltege_drop_handler:
	sbrs  byte_fl,F_iz_kan
	rjmp Error_Sourse1
	sbrc byte_fl,F_iz_kan
	rjmp Error_Sourse2
Error_Sourse1:
	ldi temp_L, 0b00000011
	out PORTB, temp_L
	sbr Err_Sourse, 0
	rjmp ex_voltege_drop_handler
Error_Sourse2: 
	//ldi temp_L, 0b00001100
	//	out PORTB, temp_L
	sbr Err_Sourse, 1
	ldi Trans1, 0b11111111
	ldi Trans2, 0b00000010
	rjmp ex_voltege_drop_handler
ex_voltege_drop_handler:
	clt
	bld    byte_fl,F_end_ADC
	ret
;Subroutine interrupt ADC
;***********************************
IN_ADC:
	   push   temp_L
       push   temp_H
       in     temp_L,SREG
	   push   temp_L
rd_ADC:  in     temp_L,ADCL
	   in     temp_H,ADCH
	   add    ADC_l,temp_L
	   adc    ADC_h,temp_H 
         	   inc    cou_ADC
   cpi    cou_ADC,Val_N_ADC;колич-во преобразований АЦП
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
;==================================================	   
;Subroutine interrupt USART RX Complete
;***********************************
REC_date:
		push   temp_L
        push   temp_H
        in     temp_L,SREG
        push   temp_L

		in		temp_H, UCSRA		;информация об ошибках
rd_UDR:	in    temp_L,UDR
		rjmp   rt_rec
pop_rec: 
		pop    temp_L
		out    SREG,temp_L
		pop    temp_H
		pop    temp_L
		reti
;***********************************
rt_rec: 
          andi   temp_H,(1<<FE)|(1<<DOR)|(1<<PE)
          breq   USARTNoError
          rjmp   pop_rec
USARTNoError:       ; Здесь будет происходить переключение сточника  
		clr		Cou_Rec
		breq	addressReceived			;мы получили адрес
		rjmp	byteReceived		;мы получили команду

		
addressReceived:	;в UDR адрес ведомого
		cpi		temp_L, My_Adress		;сравниваем адрес из UDR с назначенным адресом модуля резервирования
		brne	pop_rec				;выходим, если адрес не совпал
		cbi		UCSRA, MPCM			;очищаем MPCM 
			;запускаем Т0(TimeOUT)		;		
		ldi   	temp_L, 0
		out		TCNT0, temp_L
		ldi   temp_L,(1<<WGM12)|(1<<CS12)|(0<<CS11)|(1<<CS10)	;WGM13=0,WGM12=1,WGM11=0,WGM10=0 - режим CTC
	   	out   TCCR0,temp_L			;timeOUT = (11 * 1) / 19600 = 0.5 ms => возьмём с запасом 2 ms
	   								;2 ms => 0,002*9216000/1024 = 18(0xF12)
		ldi   temp_L,(1<<OCIE0)|(1<<OCIE1A)	
		out   TIMSK, temp_L
			

		inc 	cou_rec ;инкрементируем счетчик принятых данных
		rjmp	pop_rec
byteReceived:						;Прием кадра 
		ldi temp_H, 0 ;Обнуляем счетчик принятых данных
		mov	Cou_Rec, temp_H
		cpi temp_L, 0b00000001
		breq Select1
		cpi temp_L, 0b00000010
		breq Select2
		cpi temp_L, 0b00000100
		breq Answer
		rjmp pop_rec; если ни одна команда не совпала, значит произошла ошибка передачи
Select1: 
		sbrs Err_Sourse, 0
		rjmp Send_Error
		ldi temp_L, 0b00000011
		out PORTB, temp_L
		rjmp Selection_Success
Select2: 
		sbrs Err_Sourse, 1
		rjmp Send_Error
		ldi temp_L, 0b00001100
		out PORTB, temp_L
		rjmp Selection_Success
Answer:
		cpi Err_Sourse, 0
		brne Send_Error
		
Send_Voltage:
		mov Trans1, Voltage1  
		mov Trans2, Voltage2
		rcall Transceive ; Вызывается подпрограмма передачи 
        rjmp pop_rec
Send_Error:
		ldi Trans1, 0b00000101 
		mov Trans2, Err_Sourse
		rcall Transceive ; Вызывается подпрограмма передачи 
        rjmp pop_rec
Selection_Success:
		ldi Trans1, 0b00000001 ; В Trans1 загружается идентификатор ответа на запрос
		ldi Trans2, 0b00000001
		rcall Transceive ; Вызывается подпрограмма передачи 
        rjmp pop_rec



;***********************************
Transceive:  
		sbi	  PORTB, 4 ; Переводим RS485 трансивер в режим передачи
		ldi   temp_L,(1<<TXEN)|(1<<TXCIE);разрешить прерывание TXC
		out   UCSRB,temp_L
		out   UDR, Trans1
		ret 
;***********************************
;Subroutine interrupt USART, Tx Complete;
;***********************************
TRANdate: 
		push   temp_L
		in     temp_L,SREG
		push   temp_L
		push   temp_H

		out    UDR,Trans2
		ldi    temp_L,(1<<RXEN)|(1<<RXCIE);
		out    UCSRB,temp_L
		cbi    PORTB,4; Переводим RS485 трансивер в режим приема
		sbi		UCSRA,(1<<MPCM)
		set
		bld    Byte_fl,F_trans
		pop    temp_L         
		out    SREG,temp_L
		pop    temp_H
		pop    temp_L
		reti	
;**********************************
;Подпрограмма обработки прерывания таймера окна приема
;===================================================
TimeOUT:	
	push   temp_L
    push   temp_H 
    in     temp_L, SREG
	push   temp_L
	
	clr		temp_L 
    out    TIMSK, temp_L
	clr    cou_rec
	sbi		UCSRA,(1<<MPCM)
		   
	pop    temp_L
	out    SREG, temp_L
	pop    temp_H 
    pop    temp_L
    reti
;=====================================================
/*;Subroutine interrupt OC1A
;**********************************
One_Second_Passed:  push   temp_L
           push   temp_H 
           in     temp_L,SREG
		   push   temp_L
;
           mov Trans1, Voltage1
		   mov Trans2, Voltage2
		   rcall Transceive
;
		   pop    temp_L
		   out    SREG,temp_L
		   pop    temp_H 
           pop    temp_L
           reti
;**********************************/
