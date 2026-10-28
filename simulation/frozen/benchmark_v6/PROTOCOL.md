# Mejoras internas de Flatness: prueba acotada v6

2026-10-11. Hipótesis fijadas antes de resultados cerrados. Alcance: pérdida parcial de efectividad conocida; no frenos simulados ni recuperación catastrófica. No se modifica el manuscrito ni las campañas v1–v5.

## Variantes sin selección por escenario

1. `Flatness`: HoldCorrected de v5.
2. `StateMap`: misma corrección, pero si el error máximo de posición excede 0.02 rad o el de velocidad 0.08 rad/s, calcular la sensibilidad discreta en el estado medido. Regresar al mapa de referencia si ambos errores bajan de 0.01 rad y 0.04 rad/s. Histéresis fija. No cambia la dinámica de doble integrador del horizonte.
3. `Observer`: Flatness más estimador acotado de torque no modelado.
4. `StateObserver`: ambas modificaciones.
5. `FastStateObserver`: ambas modificaciones e integración nominal interna a 2 ms en lugar de 1 ms. La planta permanece a 1 ms.
6. `LTV`: baseline v5 con caché móvil.
7. `LTVObserver`: mismo estimador que Flatness, para separar el beneficio general del observador de la formulación flatness.

El estimador usa la diferencia entre la velocidad medida y la predicha por una integración nominal del último comando mantenido. Convierte el residuo a torque mediante la masa nominal, filtra con constante de tiempo 0.12 s y limita cada componente al 15% del torque nominal. Se inhibe la actualización cuando la efectividad cambia más de 0.015 por paso, para no aprender el transitorio de falla como error persistente. No conoce parámetros verdaderos de la planta. El torque estimado se incluye en predictor, límites, coste y fallback; nunca se añade al comando después de resolver el QP sin ajustar cotas.

Supervisor común `constant` de v5, congelado: el objetivo es aislar cambios internos. Todos reciben la misma referencia temporal, efectividad actual exacta y ruidos por semilla. No se reajustan costes ni parámetros después de comparar. El ruido de estado no implica incertidumbre sobre la falla.

## Casos

Dos recorridos conocidos (Inertia 4.5 s y GravityReverse 3 s): cuatro casos deterministas (nominal/rampa rápida temprana; masa +20%/escalón a mitad; fricción +25%/rampa durante frenado; error inicial 0.03 rad y 0.05 rad/s sin falla), y ocho realizaciones pareadas de masa ±10%, inercia ±5%, centro de masa ±2 mm, fricción viscosa ±15% y seca ±10%, con ruido de estado 1e-4 rad y 1e-3 rad/s, semillas 6001–6008. Se alterna escalón y rampa lenta, e instantes 0.5 s y 45% del tiempo nominal. Efectividades finales 0.65/0.60 respectivamente, escogidas por capacidad estática y no por rendimiento. Un tercer recorrido Transfer usa extremos Inertia escalados 0.8 y duración 4.5 s, cuatro casos deterministas con salud final 0.70. Total 28 casos × 7 variantes = 196 corridas.

Se verifica la capacidad estática de la planta perturbada antes de ejecutar y se registra como diagnóstico de alcance; no se descartan resultados cerrados por ser adversos. Si un caso queda fuera, se informa y no se cuenta como éxito. Integración física 1 ms; horizonte y periodo heredados de v5. Todas las corridas duran 10 s, salvo salida de alcance o guard numérico, para comparar esfuerzo y error sobre el mismo intervalo. Meta evaluada durante el último segundo completo, con posición y velocidad <0.02; métricas separadas de movimiento (hasta tiempo nominal) y sostenimiento.

## Validaciones y criterio de lectura

Equivalencia de variantes base con v5; observador nulo conserva QP; corrección incluida algebraicamente en límites; signo y límite del estimador; activación e histéresis del mapa; reducción de error local frente a planta bajo torque sostenido; convergencia de integración interna 1/2 ms; hashes de fuentes y resultados, ruido idéntico por caso. Tiempos incluyen supervisor, predictor, observador y controlador; no se inyecta latencia de CPU en la planta y no se afirma garantía de tiempo real.

Analizar error articular, error geométrico, error final, saturación, fallbacks, violaciones, TV, esfuerzo y tiempo total. No elegir un ganador por corrida. Una reducción de error acompañada por violaciones, fallbacks, ruido de torque o coste excesivo debe explicitarse. Estos son ensayos exploratorios, no una prueba de superioridad universal ni un teorema de estabilidad.
