PROCESSOR 16F887
    
; PIC16F887 Configuration Bit Settings

; Assembly source line config statements

; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

// config statements should precede project file includes.
#include <xc.inc>
  
; -------------- MACROS --------------- 
  
; Macro para reiniciar el valor del TMR0

  RESET_TMR0 MACRO TMR_VAR
 
    BANKSEL TMR0	    ; cambiamos de banco
    MOVLW   TMR_VAR
    MOVWF   TMR0	    ; configuramos tiempo de retardo
    BCF	    T0IF	    ; limpiamos bandera de interrupción
    ENDM

; Macro para reiniciar el valor del TMR1
  
; Recibe el valor a configurar en TMR1_H y TMR1_L

RESET_TMR1 MACRO TMR1_H, TMR1_L	
 
    BANKSEL TMR1H
    MOVLW   TMR1_H	    ; Literal a guardar en TMR1H
    MOVWF   TMR1H	    ; Guardamos literal en TMR1H
    MOVLW   TMR1_L	    ; Literal a guardar en TMR1L
    MOVWF   TMR1L	    ; Guardamos literal en TMR1L
    BCF	    TMR1IF	    ; Limpiamos bandera de int. TMR1
    ENDM
  
; ------- VARIABLES EN MEMORIA --------
PSECT udata_shr		    ; Memoria compartida
    W_TEMP:		DS 1
    STATUS_TEMP:	DS 1
    SEGUNDOS:           DS 1
    SEGUNDOS2:          DS 1
    
    
PSECT udata_bank0           ; memoria para display
    valor:         DS 1	    ; Contiene valor a mostrar en los displays de 7-seg
    banderas:      DS 1     ; cuando encender y apagar cada display
    nibbles:       DS 2	    ; Contiene los nibbles alto y bajo de valor
    display:       DS 2	    ; Representación de cada nibble en el display de 7-seg

PSECT resVect, class=CODE, abs, delta=2
ORG 00h			    ; posición 0000h para el reset
;------------ VECTOR RESET --------------
resetVec:
    PAGESEL MAIN	; Cambio de pagina
    GOTO    MAIN
    
PSECT intVect, class=CODE, abs, delta=2
ORG 04h			    ; posición 0004h para interrupciones
;------- VECTOR INTERRUPCIONES ----------
PUSH:
    MOVWF   W_TEMP	    ; Guardamos W
    SWAPF   STATUS, W
    MOVWF   STATUS_TEMP	    ; Guardamos STATUS
    
ISR:
    BTFSC   TMR1IF	    ; Interrupcion de TMR1
    CALL    INT_TMR1
    
    BTFSC   TMR2IF	    ; Interrupción del TMR2
    CALL    INT_TMR2
    
    BTFSC   T0IF	    ; Interrupción del TMR0
    CALL    INT_TMR0
 

POP:
    SWAPF   STATUS_TEMP, W  
    MOVWF   STATUS	    ; Recuperamos el valor de reg STATUS
    SWAPF   W_TEMP, F	    
    SWAPF   W_TEMP, W	    ; Recuperamos valor de W
    RETFIE		    ; Regresamos a ciclo principal

; ------ SUBRUTINAS DE INTERRUPCIONES ------
    
INT_TMR1:
    RESET_TMR1 0xB, 0xCD    ; Reiniciamos TMR1 para 1s
    INCF    SEGUNDOS 
    RETURN
    
INT_TMR2:
    BSF	    PORTB, 0
    BCF	    TMR2IF
    INCF    SEGUNDOS2
    MOVLW   2
    SUBWF   SEGUNDOS2, F
    BTFSS   STATUS, 0
    GOTO    $+3
    BCF	    PORTB,0
    GOTO    $-5
    
    RETURN
    
 INT_TMR0:
    RESET_TMR0 131		; Reiniciamos TMR0 para 50ms
    CALL    MOSTRAR_VALOR	; Mostramos valor en hexadecimal en los displays
    RETURN
    
    
    
    
PSECT code, delta=2, abs
ORG 100h		   
 
;------------- CONFIGURACION ------------
MAIN:
    CALL    CONFIG_IO	   
    CALL    CONFIG_RELOJ    ; Oscilador
    CALL    CONFIG_TMR0     ; TMR0
    CALL    CONFIG_TMR1	    ; TMR1
    CALL    CONFIG_TMR2     ; TMR2
    CALL    CONFIG_INT	    ; interrupciones
    
       
    
LOOP:
    MOVF    SEGUNDOS, W	
    MOVWF   valor		; Movemos W a variable valor
    
    CALL    OBTENER_NIBBLE	; Guardamos nibble alto y bajo de valor
    CALL    SET_DISPLAY		; Guardamos los valores a enviar en PORTC para mostrar valor en hex
    GOTO    LOOP	    
    
;------------- SUBRUTINAS ---------------
CONFIG_RELOJ:
    BANKSEL OSCCON	    ; cambiamos a banco 1
    BSF	    OSCCON, 0	    ; SCS -> 1, Usamos reloj interno
    BCF	    OSCCON, 6
    BSF	    OSCCON, 5
    BSF	    OSCCON, 4	    ; IRCF<2:0> -> 011 500kHz
    RETURN
    
 CONFIG_TMR0:
    BANKSEL OPTION_REG		; cambiamos de banco
    BCF	    T0CS		; TMR0 como temporizador
    BCF	    PSA			; prescaler a TMR0
    BCF	    PS2
    BCF	    PS1
    BCF	    PS0			; PS<2:0> -> 000 prescaler 1 : 2
    RESET_TMR0 131		; Reiniciamos TMR0 para 50ms
    RETURN 
    
    
CONFIG_TMR1:
    BANKSEL T1CON	    ; Cambiamos a banco 00
    BCF	    TMR1CS	    ; Reloj interno
    BCF	    T1OSCEN	    ; Apagamos LP
    
    BCF	    T1CKPS1	    ; Prescaler 1:4
    BSF	    T1CKPS0
    
    BCF	    TMR1GE	    ; TMR1 siempre contando
    BSF	    TMR1ON	    ; Encendemos TMR1
    
    RESET_TMR1 0xB, 0xCD   ; TMR1 a 500ms
    RETURN
   
    
 CONFIG_IO:
    BANKSEL ANSEL
    CLRF    ANSEL
    CLRF    ANSELH	    ; I/O digitales
    
    BANKSEL TRISA
    CLRF    TRISA	    ; PORTA como salida
    
    BANKSEL PORTA
    CLRF    PORTA	    ; Apagamos PORTA
    
    BANKSEL TRISB
    BCF     TRISB, 0
    
    BANKSEL PORTB
    CLRF    PORTB
    
    BANKSEL TRISC           ; para display 
    CLRF     TRISC
    
    BANKSEL PORTC
    CLRF    PORTC
    
    BANKSEL TRISD           ; para banderas de display
    BCF     TRISD, 0
    BCF     TRISD, 1
    
    BANKSEL PORTD
    CLRF    PORTD
    
    RETURN
    
    
    
CONFIG_INT:
    BANKSEL PIE1	    
    BSF	    TMR1IE	    ; int. TMR1
    BSF	    TMR2IE	    ; int. TMR1
    
    BANKSEL INTCON	  
    BSF	    PEIE	    ; int. perifericas 
    BSF	    GIE		    ; Habilitamos interrupciones
    BSF	    T0IE	    ; Habilitamos interrupcion TMR0
    BCF	    T0IF	    ; Limpiamos bandera de int. de TMR0
   
    BCF	    TMR1IF	    ; Limpiamos bandera de TMR1
    
    RETURN
    
    
CONFIG_TMR2:
    BANKSEL PR2		    ; Cambiamos a banco 01
    MOVLW   244		    ; Valor para interrupciones cada 500 ms
    MOVWF   PR2		    ; Cargamos litaral a PR2
    
    BANKSEL T2CON	    ; Cambiamos a banco 00
    BSF	    T2CKPS1	    ; Prescaler 1:16
    BSF	    T2CKPS0
    
    BSF	    TOUTPS3	    ;Postscaler 1:16
    BSF	    TOUTPS2
    BSF	    TOUTPS1
    BSF	    TOUTPS0
    
    BSF	    TMR2ON	    ; Encendemos TMR2
    RETURN
    
    
 ;-------- subrutinas para multiplexado --------
 
 OBTENER_NIBBLE:
    MOVLW   0x0F             ;0000 1111
    ANDWF   valor, W      
    MOVWF   nibbles          ;nibble bajo 
    
    MOVLW   0xF0             ;1111 0000
    ANDWF   valor, W      
    MOVWF   nibbles+1      ;nibble alto +1 para guardar en el otro byte de memoria
    SWAPF   nibbles+1, F
    
    RETURN
    
 SET_DISPLAY:
    MOVF    nibbles, W		; Movemos nibble bajo a W
    CALL    TABLE		; Buscamos valor a cargar en PORTC
    MOVWF   display		; Guardamos en display
    
    MOVF    nibbles+1, W	; Movemos nibble alto a W
    CALL    TABLE		; Buscamos valor a cargar en PORTC
    MOVWF   display+1		; Guardamos en display+1
    RETURN
    
 MOSTRAR_VALOR:
    BCF	    PORTD, 0		; Apagamos display de nibble alto
    BCF	    PORTD, 1		; Apagamos display de nibble bajo
    BTFSC   banderas, 0		; Verificamos bandera
    GOTO    DISPLAY_1		; 
    
 DISPLAY_0:			
    MOVF    display, W		; Movemos display a W
    MOVWF   PORTC		; Movemos Valor de tabla a PORTC
    BSF	    PORTD, 1		; Encendemos display de nibble bajo
    BSF	    banderas, 0		; Cambiamos bandera para cambiar el otro display en la siguiente interrupción
    RETURN

 DISPLAY_1:
    MOVF    display+1, W	; Movemos display+1 a W
    MOVWF   PORTC		; Movemos Valor de tabla a PORTC
    BSF	    PORTD, 0		; Encendemos display de nibble alto
    BCF	    banderas, 0		; Cambiamos bandera para cambiar el otro display en la siguiente interrupción
    RETURN

 ORG 200h
 
 TABLE:                         ; PC = PCH + PCL 
    CLRF    PCLATH		; Limpiamos registro PCLATH
    BSF	    PCLATH, 1		; Posicionamos el PC en dirección 02xxh
    ANDLW   0x0F		; no saltar más del tamaño de la tabla
    ADDWF   PCL
    RETLW   00111111B	;0
    RETLW   00000110B	;1
    RETLW   01011011B	;2
    RETLW   01001111B	;3
    RETLW   01100110B	;4
    RETLW   01101101B	;5
    RETLW   01111101B	;6
    RETLW   00000111B	;7
    RETLW   01111111B	;8
    RETLW   01101111B	;9
    RETLW   01110111B	;A
    RETLW   01111100B	;b
    RETLW   00111001B	;C
    RETLW   01011110B	;d
    RETLW   01111001B	;E
    RETLW   01110001B	;F
   
    
END 
