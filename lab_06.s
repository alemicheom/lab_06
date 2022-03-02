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
    
    BTFSC   TMR2IF
    CALL    INT_TMR2
 

POP:
    SWAPF   STATUS_TEMP, W  
    MOVWF   STATUS	    ; Recuperamos el valor de reg STATUS
    SWAPF   W_TEMP, F	    
    SWAPF   W_TEMP, W	    ; Recuperamos valor de W
    RETFIE		    ; Regresamos a ciclo principal

; ------ SUBRUTINAS DE INTERRUPCIONES ------
    
INT_TMR1:
    RESET_TMR1 0xB, 0xCD   ; Reiniciamos TMR1 para 1s
    INCF    PORTA	    ; Incremento en PORTA
    INCF    SEGUNDOS 
    RETURN
    
INT_TMR2:
    BSF	    PORTC, 0
    BCF	    TMR2IF
    INCF    SEGUNDOS2
    MOVLW   2
    SUBWF   SEGUNDOS2, F
    BTFSS   STATUS, 0
    GOTO    $+3
    BCF	    PORTC,0
    GOTO    $-5
    
    RETURN
    
    
    
    
PSECT code, delta=2, abs
ORG 100h		   
 
;------------- CONFIGURACION ------------
MAIN:
    CALL    CONFIG_IO	   
    CALL    CONFIG_RELOJ    ; Oscilador
    CALL    CONFIG_TMR1	    ; TMR1
    CALL    CONFIG_TMR2     ; TMR2
    CALL    CONFIG_INT	    ; interrupciones
    
    BANKSEL PORTD	    
    
LOOP:
    
    GOTO    LOOP	    
    
;------------- SUBRUTINAS ---------------
CONFIG_RELOJ:
    BANKSEL OSCCON	    ; cambiamos a banco 1
    BSF	    OSCCON, 0	    ; SCS -> 1, Usamos reloj interno
    BCF	    OSCCON, 6
    BSF	    OSCCON, 5
    BSF	    OSCCON, 4	    ; IRCF<2:0> -> 011 500kHz
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
    
    BANKSEL TRISC
    BCF     TRISC, 0
    
    BANKSEL PORTC
    CLRF    PORTC
    
    RETURN
    
    
    
CONFIG_INT:
    BANKSEL PIE1	    
    BSF	    TMR1IE	    ; int. TMR1
    BSF	    TMR2IE	    ; int. TMR1
    
    BANKSEL INTCON	  
    BSF	    PEIE	    ; int. perifericas 
    BSF	    GIE		    ; Habilitamos interrupciones
   
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


