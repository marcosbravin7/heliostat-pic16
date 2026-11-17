## Diagramas de Flujo del Firmware

### 1. Flujo General: Inicialización y Programa Principal (`inicio` y `principal`)

```mermaid
flowchart TD
    A(["Reset Vector (0x0000)"]) --> B["1. Oscilador (4 MHz) y 2. ADC (AN0, AN2, AN3, AN4)"]
    B --> C["3. UART (9600 baudios, 8N1, TX y RX) y 4. Salidas PWM (RD0, RD1)"]
    C --> D["5. Timers (Timer0 1:128, Timer1 1:8) y 6. Variables e Interrupciones (GIE, T0IE)"]
    D --> F{"¿FRAME_LISTO == 1?"}
    F -- "No" --> K["CALL recibir_uart"]
    F -- "Sí (cada ~20 ms)" --> G["Limpiar bandera FRAME_LISTO = 0"]
    G --> H["CALL emitir_pwm (Generar pulsos en RD0 y RD1)"]
    H --> I{"Decrementar divisor_tx: ¿divisor_tx == 0?"}
    I -- "No" --> K
    I -- "Sí (cada 5 periodos ~100 ms)" --> J["Recargar divisor_tx = 5 y CALL enviar_telemetria: enviar [0xFF, pulso_x, pulso_y]"]
    J --> K
    K --> F
```

### 2. Rutina de Interrupción (`interrupcion` en `0x0004`) y Modo Automático (`ajustar_por_luz`)

```mermaid
flowchart TD
    ISR_IN(["Desborde de Timer0 (0x0004 - cada 19.968 ms)"]) --> ISR1["Guardar contexto en RAM común (W, STATUS, FSR)"]
    ISR1 --> ISR2["Recargar TMR0 = 100 y limpiar bandera T0IF = 0"]
    ISR2 --> ISR3{"¿MODO_MANUAL == 1?"}
    ISR3 -- "Sí (Modo Manual)" --> ISR_FIN["Activar bandera FRAME_LISTO = 1"]
    ISR3 -- "No (Modo Automático)" --> AUTO1["Leer ADC (7 bits): AN2 (Norte), AN3 (Este), AN4 (Oeste), AN0 (Sur)"]
    AUTO1 --> AUTO2["Eje X: diferencia = AN4 - AN3"]
    AUTO2 --> AUTO3{"¿|diferencia X| > ZONA_MUERTA (2)?"}
    AUTO3 -- "Sí" --> AUTO4["Ajustar pulso_x en +/- 1 cuenta y limitar entre [1, 251]"]
    AUTO3 -- "No" --> AUTO5["Mantener pulso_x"]
    AUTO4 --> AUTO6["Eje Y: diferencia = AN0 - AN2"]
    AUTO5 --> AUTO6
    AUTO6 --> AUTO7{"¿|diferencia Y| > ZONA_MUERTA (2)?"}
    AUTO7 -- "Sí" --> AUTO8["Ajustar pulso_y en +/- 1 cuenta y limitar entre [1, 251]"]
    AUTO7 -- "No" --> AUTO9["Mantener pulso_y"]
    AUTO8 --> ISR_FIN
    AUTO9 --> ISR_FIN
    ISR_FIN --> ISR_RET["Restaurar contexto (FSR, STATUS, W) y RETFIE"]
```

### 3. Emisión de Pulsos PWM con Timer1 (`emitir_pwm`)

```mermaid
flowchart TD
    PWM_IN(["Inicio emitir_pwm"]) --> P1{"Comparar pulso_x y pulso_y"}
    P1 -- "pulso_x < pulso_y" --> P2["pulso_corto = pulso_x, pulso_largo = pulso_y, mascara = 0x02 (RD1 sigue alto)"]
    P1 -- "pulso_y < pulso_x" --> P3["pulso_corto = pulso_y, pulso_largo = pulso_x, mascara = 0x01 (RD0 sigue alto)"]
    P1 -- "pulso_x == pulso_y" --> P4["pulso_corto = pulso_largo = pulso_x, mascara = 0x00 (ambos bajan juntos)"]
    P2 --> P5["Precargar TMR1H = 0, TMR1L = 195 (61 cuentas base = 488 us)"]
    P3 --> P5
    P4 --> P5
    P5 --> P6["Iniciar Timer1 (TMR1ON = 1) y subir salidas RD0 y RD1 (PORTD = 0x03)"]
    P6 --> P7{"Esperar base: ¿TMR1H,0 == 1?"}
    P7 -- "No (lazo de 8 us)" --> P7
    P7 -- "Sí (cumplidos ~500 us)" --> P8{"¿TMR1L >= pulso_corto?"}
    P8 -- "No (lazo de 8 us)" --> P8
    P8 -- "Sí" --> P9["Apagar pulso más corto (PORTD = mascara_pulso_largo)"]
    P9 --> P10{"¿TMR1L >= pulso_largo?"}
    P10 -- "No (lazo de 8 us)" --> P10
    P10 -- "Sí" --> P11["Apagar ambas salidas (PORTD = 0x00) y detener Timer1 (TMR1ON = 0)"]
    P11 --> PWM_RET(["RETURN"])
```

### 4. Recepción UART, Eco y Control Manual (`recibir_uart`)

```mermaid
flowchart TD
    U_IN(["Inicio recibir_uart"]) --> U2{"¿Dato recibido (RCIF == 1)?"}
    U2 -- "No" --> U_RET(["RETURN"])
    U2 -- "Sí" --> U4["Leer tecla = RCREG y enviar eco inmediato por TX (CALL enviar_uart)"]
    U4 --> U5{"¿tecla == '1'?"}
    U5 -- "Sí" --> U_AUTO["Activar Modo Automático (modo = 0)"] --> U_RET
    U5 -- "No" --> U6{"¿tecla == '2'?"}
    U6 -- "Sí" --> U_MAN["Activar Modo Manual (MODO_MANUAL = 1)"] --> U_RET
    U6 -- "No" --> U7{"¿Está en Modo Manual?"}
    U7 -- "No" --> U_RET
    U7 -- "Sí" --> U9{"Evaluar comando manual"}
    U9 -- "'a'" --> M_A["pulso_x = pulso_x - 4 (limitar 1..251)"] --> U_RET
    U9 -- "'d'" --> M_D["pulso_x = pulso_x + 4 (limitar 1..251)"] --> U_RET
    U9 -- "'w'" --> M_W["pulso_y = pulso_y + 4 (limitar 1..251)"] --> U_RET
    U9 -- "'s'" --> M_S["pulso_y = pulso_y - 4 (limitar 1..251)"] --> U_RET
    U9 -- "'c'" --> M_C["Centrar servos: pulso_x = 126, pulso_y = 126"] --> U_RET
    U9 -- "Otra tecla" --> U_RET
```

---