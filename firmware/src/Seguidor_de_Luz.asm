        LIST    P=16F887
        INCLUDE <p16f887.inc>
        __CONFIG _CONFIG1, 0x2FF4

        CBLOCK  0x20
pulso_x
pulso_y
modo
flags
tecla
lectura_an2
lectura_an3
lectura_an4
lectura_an0
espera_adc
diferencia
paso_auto

        ENDC


        ORG     0x0000
        GOTO    inicio


        ORG     0x0004

        RETFIE

; INICIALIZACION POR MODULOS
inicio
        ; --- 1. Oscilador interno (4 MHz) ---
        BSF     STATUS,RP0
        BCF     STATUS,RP1      ; Banco 1.
        MOVLW   0x61
        MOVWF   OSCCON          ; Reloj interno a 4 MHz (1 us por instruccion).

        ; --- 2. Conversor ADC (4 sensores LDR) ---
        CLRF    ADCON1          ; Justificado a izquierda, referencias VDD y VSS.
        BSF     STATUS,RP1      ; Banco 3 (RP0=1, RP1=1).
        MOVLW   0x1D
        MOVWF   ANSEL           ; Habilita entradas analogicas AN0, AN2, AN3 y AN4.
        CLRF    ANSELH

        ; --- 3. Comunicacion Serie UART (9600 baudios, 8N1) ---
        CLRF    BAUDCTL         ; BRG16 = 0 (generador de baudios de 8 bits).
        BCF     STATUS,RP1      ; Banco 1 (RP0=1, RP1=0).
        MOVLW   .25
        MOVWF   SPBRG           ; Fosc / (16 * (25 + 1)) = 9615 baudios.
        MOVLW   0x24
        MOVWF   TXSTA           ; Asincrona, alta velocidad (BRGH=1) y TX activo (TXEN=1).
        BCF     TRISC,6         ; RC6 / TX como salida.
        BSF     TRISC,7         ; RC7 / RX como entrada.
        BCF     STATUS,RP0      ; Banco 0 (RP0=0, RP1=0).
        MOVLW   0x90
        MOVWF   RCSTA           ; Habilita puerto serie (SPEN=1) y recepcion continua (CREN=1).

        ; --- 4. Salidas PWM para Servomotores (RD0 y RD1) ---
        CLRF    PORTD           ; Inicia salidas de servos en nivel bajo.
        BSF     STATUS,RP0      ; Banco 1.
        MOVLW   0xFC
        MOVWF   TRISD           ; RD0 (Servo X) y RD1 (Servo Y) como salidas digitales.

        ; --- 5. Temporizadores (Timer0 para 20 ms, Timer1 para ancho de pulso) ---
        MOVLW   0x86
        MOVWF   OPTION_REG      ; Timer0 con reloj interno y prescaler 1:128.
        BCF     STATUS,RP0      ; Banco 0.
        MOVLW   0x30
        MOVWF   T1CON           ; Timer1 detenido, reloj Fosc/4 y prescaler 1:8 (8 us/cuenta).
        MOVLW   RECARGA_TMR0
        MOVWF   TMR0            ; Precarga Timer0 para periodo de ~20 ms.

        ; --- 6. Variables iniciales e Interrupciones ---
        MOVLW   PULSO_CENTRO
        MOVWF   pulso_x         ; Servo X centrado (1500 us).
        MOVWF   pulso_y         ; Servo Y centrado (1500 us).
        CLRF    modo            ; Arranca en modo automatico.
        CLRF    flags
        MOVLW   DIVISOR_TELEMETRIA
        MOVWF   divisor_tx      ; Inicializa contador de telemetria (cada 5 periodos).
        MOVLW   0xA0
        MOVWF   INTCON          ; Habilita interrupcion global (GIE) y de Timer0 (T0IE).

; PROGRAMA PRINCIPAL
principal


        END
