;*************************************
;* Title:          contr_UART.asm
;* Device          ATmega16
;* Clock frequency:Частота кв.резонатора 9,216 mHz
;*************************************
; учебная
;*************************************
;Передача/прием данных по USART осуществляется со следующими параметрами:
;а)скорость обмена 19200бит/сек;
;б)формат посылки 11 бит информации:
;      -  старт-бит;
;      -  8 бит данных;
;      -  бит контроля четности - дополнение до нечетности;
;      -  один стоп бит.
;Выводы USART(RXD) PD0 (вход),(TXD) PD1(выход)
;Протокол обмена : 
;1.Раз в секунду плата будет оправлять значения напряжения источников, округлённое до десятых. 
;	В первом байте будет находиться значение напряжения первого источника, во втором байте значение напряжения второго источника. 
;2.В случае падения напряжения ниже порогового на одном из источников, будет отправляться два байта: 
;	Первый байт будет содержать в себе все еденицы, второй байт будет содержать в себе номер неисправного источника. 
;3.Головное устройство может по команде оператора самостоятельно переключить источник питания:
;	Будет отправляться один байт с номером источника, на который нужно переключиться. 
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
.def Err_Sourse1 = R21
.def Err_Sourse2 = R22
.def Trans1 = R23
.def Trans2 = R24
.def  cou_ADC    =R25
;*******************
.def Byte_fl    =R2; байт флагов
.def  ADC_h      =R3
.def  ADC_l      =R4
.def Cou_Rec    =R5;счетчик принятых байт
.def Cou_Tran   =R6;счетчик переданных байт
.def c_sumREC   =R7;контр сумма прин. байт
.def c_sumTRAN  =R8;контр сумма переданных байт
;-----------------
.equ   F_receive   =1;флаг принятого запроса
.equ   F_trans    =2;флаг завершения передачи
.equ  F_iz_kan    =0;флаг изменения канала:0 1-й источник.1 2-й источник
.equ  F_end_ADC   =1;флаг завершения (Val_N_ADC) преобразований АЦП
;******************* 
; Constants
;*****************
.equ Val_Voltage_Measurement=50;величина константы,опр. время через которое будет отправляться значение напряжения источников 
.equ   Val_N_ADC    =4;колич-во преобразований АЦП(1,2,4,8.16..)
;========================================
;Количество байт обмена с ПЭВМ
;----------------------------------------
.equ	VAL_TR  =2
.equ	VAL_REC =1
.equ	VAL_time =17;35 переполнений соответствуют 1 сек
.equ	Master_Adress = 0b01010101; Адрес мастера
.equ	My_Adress = 0b01010100; Адрес платы
;***************************************
;Variable
;***************************************
.DSEG 
;Пока что тут ничего нет, но может понадобиться.

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
       ldi   temp_L,14;(Частота кв. 9,216 мГц,скорость обмена 19200),U2X0=0,
       ldi   temp_H,00 
	   out   UBRRL,temp_L;
	   out   UBRRH,temp_H
	   ldi   temp_L,(1<<RXEN)|(1<<RXCIE);UCSZ2=0,UCSZ1=1,UCSZ0=1 - 8 bit
	   out   UCSRB,temp_L
	   ldi   temp_L,(1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1)|(1<<UPM1);- 1 stop bit,
	   out   UCSRC,temp_L;
	   
; ---  Инициализация  таймера TCNT0

;
;      Инициализация  таймера TCNT1 этот таймер будет использоваться для отсчета одной секунды для отправки значений напряжения. 
       ldi   temp_L,00;compare A,(COM1A1,COM1A0=00) OC1A disconnect
       out   TCCR1A,temp_L;
	   ldi   temp_L,(1<<WGM12);WGM13=0,WGM12=1,WGM11=0,WGM10=0,режим CTC
;                             No clock source,CS42,CS41,CS40=000
	   out   TCCR1B,temp_L;(No prescaling CS10=1
	   ldi   temp_H,0xD8  ; time_out 6mcek - 0,006*9216000=55296(D800)
	   ldi   temp_L,0x00  ;
       out   OCR1AH,temp_H
	   out   OCR1AL,temp_L
;
       
       clr   Byte_fl



       sei             ;разрешаем прерывания

;==================================================
;начало цикла программы
/*В основном цикле программы должно непрерывно происходить снятие значений напряжений источников питания. Определение ошибки должно происходить сразу после снятия значения. 
Отправка значений напряжения будет происходить по прерыванию от таймера.
Приём команды переключения питания происходит по прерыванию.
Что необходимо сделать: 
1.Сделать подпрограмму работы ADC(В ней будут записываться значения Voltage1 и Voltage2 ) и изменения канала
2.Сделать универсальную подпрограмму отправки данных(Необходимо ввести два регистра(Trans1, Trans2), в которые будут помещаться данные для передачи) Она будет называться Transceive
3.Сделать подпрограмму обработки прерывания с таймера(В ней будут заполняться Trans1 и Trans2, значения надо брать из Volage1 и Voltage2. Затем вызывается подпрограмма передачи)
4.Разобраться с округлением значений напряжения.
5.Написать подпрограмму voltege_drop_handler. В ней переключается источник напряжения, заполняются переменные Trans1 и Trans2, затем вызывается подпрограмма передачи)
6.В овтвет на запрос на переключение нужно отправлять ответ о статусе выполнения Trans1 будет идентификвтором ответа. 
	Trans2 будет содержать ответ: 
	а) Операция проведена успешно
	б) Операция не выполнена: напряжение на источнике ниже допустимого
	в) Операция не выполнена: ошибка приема команды
7. Надо добавить возможность полного отключения питания аппарата. Питание платы енезависимое, поэтому после отключения аппарат можно будет снова включить.
*/
;==================================================
Start: 
		sbrc   Byte_fl,F_receive;проверка флага принятого запроса
		rjmp Receive 	
		rcall get_Voltage; В процессе написания
		rjmp Start	

get_Voltage:
		rcall	start_ADC
		rcall	wait_ADC
		rcall	izm_Nkan; Необходимо разобраться и подпривить izm_Nkan
		rcall	start_ADC
		rcall	wait_ADC
		rcall	izm_Nkan
		ret
wait_ADC:
		sbrs   byte_fl,F_end_ADC
		rjmp   wait_ADC
		rcall  out_ADC
		ret

; Подпрограмма смены канала преобразования(Вроде бы готово)
;==================================================
izm_Nkan:
	   clt
         	   sbrs  byte_fl,F_iz_kan
	   set
        	   bld   byte_fl,F_iz_kan
;изменить № канала АЦП
        	   in     temp_L,ADMUX
         	   andi   temp_L,0b01100000
ch_mux_ADC: sbrs  byte_fl,F_iz_kan
        	   rjmp  ex_c_mux
         	   ori   temp_L,(1<<MUX0)
ex_c_mux:	out   ADMUX,temp_L
        	   ret
;**************************************************
start_ADC:  
		ldi   temp_L,(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1);ч-та преобр.64(125кГц)
		out   ADCSR,temp_L
		ret 
;Подпрограмма out_ADC изменение данных выводимых на дисплей
;**************************************************
;данные с АЦП усредняются путем суммирования (Val_N_ADC=1,2,4,8.16) раз и нахождения
;среднего значения, используя для деления сдвиг вправо
;измеренное значение Vin=(ADC*2,54)/1024=ADC/400. Используем только целые числа для
;вывода на дисплей, т.е. Vin*100. таким образом измеренное значение равно ADC/4
;увеличение в 100 раз при выводе на дисплей компенсируем выводом точки у первой
; зн.цифры
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
		ror    ADC_l; в ADC_l значение,увеличенное в 100 раз
		mov    temp_L,ADC_l
		clr   ADC_h
		clr   ADC_l
		sbrs  byte_fl,F_iz_kan
		mov Voltage1,temp_L
		sbrc byte_fl,F_iz_kan
		mov Voltage2,temp_L
		cpi	temp_l, 0x12; сравниваем полученное значение напряжения с 18В.
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
;Subroutine interrupt ADC(её менять не надо)
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
		  ldi Trans1, 0b00000001 ; В Trans1 загружается идентификатор ответа 
		  ldi Trans2, 0b00000001; В Trans2 загружается код ошибки передачи
		  rcall Transceive ; Вызывается подпрограмма передачи 
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
USART2NoError:       ; Здесь будет происходить переключение сточника 
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
		ldi Trans1, 0b00000001 ; В Trans1 загружается идентификатор ответа на запрос
		ldi Trans2, 0b00000001
		rcall Transceive ; Вызывается подпрограмма передачи
        rjmp pop_rec

Selected_Sourse_Fault:
		ldi Trans1, 0b00000001 ; В Trans1 загружается идентификатор ответа на запрос
		ldi Trans2, 0b00000010
		rcall Transceive ; Вызывается подпрограмма передачи 
        rjmp pop_rec
rec_end: 
		com    temp_L;инверсия принятого байта
		cp     temp_L,c_sumREC; проверка контр суммы
		brne   pop_rec
		set
		bld    Byte_fl,F_receive;прием верных данных
;          

;***********************************        
;Subroutine interrupt USART Data register Empty 
;***********************************
Transceive:  
		  sbi	PORTB, 4 ; Переводим RS485 трансивер в режим передачи
		  ldi   temp_L,(1<<TXEN)|(1<<TXCIE);разрешить прерывание TXC
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
		  cbi    PORTB,4; Переводим RS485 трансивер в режим приема
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
           ldi    temp_L,(1<<WGM12);WGM13=0,WGM12=1,WGM11=0,WGM10=0,режим CTC
;                             No clock source,CS42,CS41,CS40=000 
           out    TCCR1B,temp_L;
		   in     temp_L,TIMSK
           clt
		   bld    temp_L,OCIE1A;OCIE1A запретить прерывание
           out    TIMSK,temp_L
           ldi    temp_H,0x00
		   ldi    temp_L,0x00  ;
           out    TCNT1H,temp_H;сброс счетчика
		   out    TCNT1L,temp_L
		   clr    Cou_Rec
		   clr    c_sumREC
		   ldi    YL,low(varBuf_Rxd)  ; Load Y register low буфер приема
           ldi    YH,high(varBuf_Rxd) ; Load Y register high буфер приема		   
;
		   pop    temp_L
		   out    SREG,temp_L
		   pop    temp_H 
           pop    temp_L
           reti*/
;**********************************