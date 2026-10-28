# Piloto acotado: falla conocida, pendiente y camino fijo

2026-10-11. El autor limita explícitamente el alcance a pérdida parcial de efectividad que permita mantener la tarea/postura. No se desarrolla recuperación de fallas extremas ni búsqueda de posturas de resguardo. El modelo no tiene frenos: salir del intervalo operativo termina el ensayo con `outside_operating_envelope`, sin simular una parada segura ni un robot inmovilizado.

## Arquitectura y supuestos

- Se conoce la efectividad actual. El predictor solo usa su historial, nunca la función futura de falla del simulador. Estima pendiente en una ventana corta, detecta saltos y mesetas. Una extrapolación local tiene horizonte limitado; no se extiende una pendiente hasta cero durante todo el viaje.
- El intervalo mínimo de efectividad es un supuesto de diseño declarado antes de simular. Se obtiene de la demanda estática del camino con margen y se verifica contra la falla real. No es el valor final secreto de la falla. Una predicción se acota a ese intervalo; una observación fuera del intervalo termina el ensayo.
- Se conservan los caminos articulares rectos de los perfiles originales (sus trayectorias quínticas recorren esas rectas). El supervisor cambia exclusivamente el avance escalar. Se preservan posición y velocidad de la referencia al replanificar; la aceleración puede cambiar dentro de sus límites.
- Se precomputan coeficientes de dinámica a lo largo del camino. La evaluación del supervisor interpola torque inercial, centrífugo/Coriolis, gravedad y evalúa fricción explícita. Se valida contra el MEX físico. La reserva inicial de torque es 3%; no es una garantía robusta.
- El controlador usa una referencia local de horizonte; no se reconstruye un predictor para todo el viaje. Flatness y LTV reciben las mismas reglas, costes, límites y pronóstico. La predicción de salud cambia los límites y el coste físico de comando por etapa.

## Comparación

Tres tratamientos: referencia fija y salud congelada (`fixed`); supervisor con salud congelada (`constant`); supervisor con pronóstico de tendencia (`trend`). Ambos controladores: HoldCorrected y LTV. Tres fallas por perfil: escalón, rampa de 0.3 s y rampa de 2 s, seguidas de meseta. Modelo nominal y diagnóstico actual exacto. No se añaden incertidumbres hasta validar esta mecánica. No se recalibran los pesos seleccionados en v3.

Se eligen perfiles e intensidades por demanda física, antes de resultados cerrados, y se conservan todos los resultados. El intervalo operativo solo aporta una condición de alcance, no un certificado de seguimiento o frenado. Las fallas anteriores y sus resultados adversos permanecen archivados.

Selección tras inspeccionar únicamente demanda nominal: Inertia en 4.5 s, efectividad final 0.555 y mínimo de diseño 0.53; Gravity recorrido en sentido inverso en 3 s, efectividad final 0.50 y mínimo de diseño 0.43. La inversión se declara: aumenta demanda dinámica/fricción y evita repetir un caso donde la pérdida admitida apenas exige adaptación. No se seleccionó ningún escenario por desempeño relativo de los controladores. Total: 36 corridas.

Si se agota la familia finita de cambios de tiempo, se registra `search_exhausted` y se conserva la referencia anterior con el MPC limitado en torque. Eso no se presenta como una política de parada segura ni una garantía de completar la tarea. Las salidas del intervalo operativo detienen el ensayo y no se cuentan como éxitos.

Medir error geométrico y de seguimiento, tiempo de tarea, postura final, violaciones durante integración a 1 ms, esfuerzo, variación de comando, frecuencia de replanificación, tiempo de supervisor/predictor/controlador y tiempo total por paso. La simulación nominal usa cómputo instantáneo; si hay pasos superiores a 20 ms, no se afirma ejecución en tiempo real. El escalón histórico aparece inmediatamente después de la medición coincidente con su inicio; se registran los transitorios de reacción.

## Validación

Continuidad del avance y su velocidad; camino de referencia invariante; dinámica interpolada contrastada con MEX; pendiente de rampa y detección de meseta; salto no confundido con rampa infinita; resultado del predictor independiente del futuro; salud constante equivalente al coste y QP anteriores; fuentes y resultados con hashes; no cambios durante la campaña.

## Reutilización de modelos

La campaña inicial `74a319258df3` se conserva completa. La siguiente versión reutiliza linealizaciones coincidentes del horizonte anterior, en ambos métodos, con tolerancia 1e-11 en posición, velocidad y aceleración de referencia. LTV necesita normalmente una nueva linealización al desplazar el horizonte, en lugar de repetir quince. Se comprueba equivalencia del QP y comandos con y sin caché. Esta corrección evita una comparación temporal que penalice a LTV por trabajo redundante. Se mantienen escenarios, pesos y reglas de decisión.

## Ablación posterior a la campaña con caché

La campaña `b4e47d4138b5` conserva sus 36 corridas completas. Al anticipar tanto en supervisor como en MPC, GravityReverse/rampa rápida con LTV presenta una violación de aceleración de 2.59 rad/s², aunque llega a la meta. Se añade un cuarto tratamiento `trend_governor`: misma pendiente y mismo supervisor temporal, pero el MPC mantiene la salud actual durante su horizonte. Así se aísla el uso del pronóstico en la referencia de su uso en las restricciones/coste del QP. No se cambia ningún peso, escenario o umbral. La nueva campaña registra los cuatro tratamientos: 48 corridas. Es una ablación exploratoria motivada por ese resultado adverso, no una confirmación independiente.
