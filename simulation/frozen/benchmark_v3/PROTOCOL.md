# Corrección de torque sostenido — experimento posterior a v2

2026-10-11. La campaña v2 conserva sus resultados adversos: el preview basado en decisiones anteriores puede aumentar TV, y el punto medio con cotas futuras puede activar respaldos nominales. Esta nueva hipótesis se fija después de observar v2; no es una extensión confirmatoria de su protocolo original.

## Hipótesis fija

Usar una inversión aproximada del incremento de velocidad sobre el intervalo de torque sostenido. Para cada instante de referencia, precalcular `K = (Ts*I) / Bd_velocity`, con Bd obtenido por ZOH de la linealización física nominal. En línea, evaluar una sola integración RK4 nominal con el torque `tau_anchor = M(x)*ddq_ref + b(x)` desde el estado medido. Si produce aceleración media `a_anchor`, usar `tau(v)=tau_anchor+K*(v-a_anchor)`. Mantener este mapa fijo durante el QP, sin realimentar la solución previa en su construcción.

La predicción cinemática sigue siendo de doble integrador en aceleración media. No se afirma exactitud de posición ni inversión exacta fuera del entorno del anclaje. K se precalcula en la referencia, no en la planta verdadera. El modelo nominal y la salud estimada son la única información disponible. La integración interna de predicción usa 0.001 s, como la planta base; no conoce la salud real futura. El torque deseado se divide por la salud estimada actual y se satura con las mismas cotas.

Mantener cotas de posición, velocidad y aceleración media en todo el horizonte. Añadir aceleración nominal instantánea prevista al inicio y al final del primer intervalo, usando estados de anclaje fijos. El resto de las etapas conserva aproximación cinemática, como v1. Igual solver, coste, horizonte, respaldo y límites físicos; el coste adicional de preparación/integración interna se incluye en tiempos.

## Validación antes de correr

Equivalencia algebraica del coste y solución con interior-point; comandos finitos bajo salud cero; cotas de aceleración inicial; ruido/runner original idénticos; repetir control determinista. Contrastar error de predicción de velocidad y posición con torque sostenido. Los resultados de v2 no se sobrescriben.

## Calibración y confirmación

Tres métodos: FlatnessV1, HoldCorrected y LTV. Misma rejilla `[0.001,0.003,0.01,0.03,0.1,0.3,1]`, mismas referencias de calibración ×0.85 y escenarios sano/moderado de v2; criterio de puntuación idéntico, sin reajustarlo para favorecer HoldCorrected. 84 configuraciones de calibración. No hay selección de arquitectura dentro de esta campaña: se prueba exclusivamente HoldCorrected y se conserva aun si empeora.

Confirmación: mismos bloques E1=48, E2=30, E3=42 y E4=180 del protocolo v2, pero semillas de incertidumbre nuevas **3001:3030**. Las dos referencias siguen siendo las mismas; solo las realizaciones aleatorias son nuevas. Convergencia a 0.5 ms en los seis casos moderados nominales. Total **390 configuraciones** = 84 + 300 + 6. Congelar pesos antes de E1–E4. No ajustar de nuevo al ver la confirmación.

Evaluar error, TV/RMS, esfuerzo, respaldos y violaciones; pares por semilla, bootstrap 10 000 réplicas semilla 20261013; tiempos por replay intercalado. Distinguir resultados de v1, v2 y esta campaña, sin comparar medias de distintas semillas como pares. Una ventaja puede ser localizada; no imponer un ganador como criterio de éxito.
