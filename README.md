\# Faraday Rocketry — Prueba de Acceso Aviónica Software



Repositorio oficial para la entrega de los ejercicios de selección del subdepartamento de software de aviónica (Faraday Rocketry).



\---



\## Arquitectura de Software de Vuelo (STM32F405)



\### Periféricos y Buses

\- \*\*I2C1 (PB6/PB7 @ 400 kHz):\*\* Doble barómetro MS5611 redundante (direcciones `0x77` y `0x76`) con conversión barométrica basada en el modelo de atmósfera estándar ISA multinivel.

\- \*\*SPI1 (PA5/PA6/PA7):\*\* Bus compartido para la IMU BNO085 (`CS: PB0`, `INT: PC4`) y el acelerómetro de alta escala H3LIS331DL (`CS: PB1`).

\- \*\*USART1 (PA9/PA10 @ 115200 bps):\*\* Recepción asíncrona y parseo continuo de tramas NMEA del GPS u-blox NEO-M9N mediante `TinyGPS++`.



\### Algoritmos y Control de Flujo

\- \*\*Estimación de Estados (EKF 1D):\*\* Filtro de Kalman lineal a 100 Hz ($z$, $v$, $b$) con estimación del sesgo del acelerómetro, rechazo de valores atípicos (\*outliers\* a > $4\\sigma$) y propagación analítica de la covarianza.

\- \*\*Conmutación Inercial Dinámica:\*\* Selección automática del acelerómetro High-G al superar los 7g ($a > 68.6\\text{ m/s}^2$) para evitar la saturación de la IMU principal.

\- \*\*Máquina de Estados de Vuelo:\*\* Detección de fases de vuelo (PAD, BOOST, COAST, APOGEE, DESCENT) con guardas temporales por persistencia (30 ms) frente a ruido transitorio.

\- \*\*Gestión No Bloqueante de Memoria:\*\* Vaciado continuo de colas UART y búferes SHTP sin retardos, desacoplando la tasa de refresco del bucle de control (100 Hz) respecto a la emisión de telemetría por puerto serie (10 Hz / 4 Hz).



\---



\## Estructura del Repositorio



\- \*\*`Inscripción\_Aviónica\_Software\_Roman Galiana Romero.pdf`:\*\* Memoria técnica completa y justificación de diseño de los ejercicios 1, 2 y 3.

\- \*\*`main/`:\*\* Firmware principal con el bucle a 100 Hz, EKF 1D y máquina de estados.

\- \*\*`inicializacion\_lectura\_sensores/`:\*\* Script de validación de buses, lectura baremetal y vaciado de colas.

\- \*\*`Inscripción\_Avionica\_Software\_1\_CubeMx\_F405/`:\*\* Proyecto y configuración de pines y relojes en STM32CubeMX (`.ioc`).

\- \*\*`Inscripción\_Avionica\_Ejercicio\_Personal/`:\*\* Documentación y código del control de un rover autónomo con sensado LiDAR ToF y desacoplo de cómputo.

