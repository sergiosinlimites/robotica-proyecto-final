## Lab 05 – Tareas pendientes para finalizar

Este documento resume **qué falta por implementar** para completar el Lab 05 con el PhantomX Pincher X100, tomando como base el análisis de los paquetes y nodos ya disponibles.

---

## 1. Modelo geométrico y DH

- [ ] **Construir la tabla DH completa del brazo**
  - A partir del URDF/Xacro (`phantomx_pincher_arm.xacro`) o de mediciones físicas.
  - Incluir \(a_i\), \(\alpha_i\), \(d_i\), \(\theta_i\) para cada articulación activa.
- [ ] **Elaborar un diagrama del robot con los frames DH**
  - Mostrar claramente el sistema de referencia base y el frame del efector final (TCP).

---

## 2. Scripts ROS 2 en espacio articular

- [x] **Script 1 – Movimiento secuencial HOME → pose objetivo**
  - Implementado dentro de `pincher_control/control_servo.py` como parte de la GUI.
  - Movimiento secuencial **por articulación** usando botones de poses predefinidas:
    - HOME (retorno secuencial extremo → base).
    - Pose 1 y Pose 2 (movimiento secuencial base → extremo).
  - Incluye esperas entre cada articulación para que el movimiento se vea claramente escalonado.

- [x] **Script 2 – Publicación de comandos articulares en grados**
  - Funcionalidad cubierta por `pincher_control/control_servo.py`:
    - Sliders y campos numéricos trabajan en **grados** respecto a HOME.
    - Conversión interna a ticks Dynamixel mediante `degrees_to_dxl`.
  - **Validación de límites articulares** implementada con rangos \([-150°, 150°]\) para todas las articulaciones.
  - Integrado directamente con el nodo de control articular (`PincherController`) que envía los comandos a los motores físicos.

- [x] **Script 3 – Lectura de ángulos articulares en grados**
  - Implementado como nodo ROS 2 en `pincher_control/joint_angles_degrees.py`.
  - Se suscribe a `/joint_states` y convierte las posiciones articulares a **grados respecto a HOME**.
  - Muestra en consola los **5 ángulos en grados** en el orden:
    - 1: base, 2: shoulder, 3: elbow, 4: wrist, 5: gripper.
  - Deja explícito el mapeo nombre-de-joint ↔ índice en el log de inicio del nodo.

---

## 3. Integración Python + ROS 2 + Toolbox (cinemática directa)

- [x] **Script de cinemática directa con Toolbox**
  - Usar los parámetros DH definidos en la sección 1 en una toolbox de robótica (Matlab, Python Robotics Toolbox u otra).
  - Recibir una configuración articular (por ejemplo, las 5 poses propuestas en la guía).
  - Enviar esa configuración al robot (vía ROS 2).
  - Calcular la **cinemática directa** para obtener la pose cartesiana del TCP.
- [x] **Comparación con RViz / robot real**
  - Graficar el modelo en la toolbox y comparar visualmente con:
    - La pose mostrada en RViz.
    - La pose física del robot (si se dispone de hardware).
  - Verificar coherencia (al menos cualitativa) entre ambas representaciones.

---

## 4. Extensiones de la interfaz gráfica (HMI)

- [x] **Personalización de la GUI**
  - Añadida pestaña `Acerca de` en la HMI (`control_servo.py`) con:
    - Nombres de los integrantes del grupo.
    - Nombre de la universidad, programa e información básica de la asignatura.

- [x] **Botones para poses predefinidas**
  - Implementadas en la pestaña `Control por Pose` de `control_servo.py`:
    - Pose HOME secuencial (extremo → base), equivalente a Pose Lab 1.
    - Pose 1 y Pose 2 secuenciales (base → extremo), definidas en grados.
    - Poses de laboratorio (según la guía):  
      `[0, 0, 0, 0, 0]`, `[25, 25, 20, -20, 0]`, `[-35, 35, -30, 30, 0]`,  
      `[85, -20, 55, 25, 0]`, `[80, -35, 55, -45, 0]`.
  - Los textos de los botones indican claramente el vector `[q1..q5]` de cada pose.

- [ ] **Pestaña de control en espacio de la tarea**
  - Crear una nueva pestaña con sliders o campos numéricos para:
    - **X, Y, Z** (m).
    - **Roll, Pitch, Yaw** (rad o grados, pero documentado claramente).
  - Conectar esta pestaña al tópico `pose_command` (`phantomx_pincher_interfaces/PoseCommand`) o a un nodo equivalente.
  - Permitir elegir si se quiere:
    - Movimiento directo a la pose (planificación en espacio de configuración).
    - Trayectoria cartesiana (usando `cartesian_path = true`).

- [/] **Pestaña de visualización numérica de la pose cartesiana**
  - Mostrar en tiempo real la pose cartesiana del TCP:
    - **X, Y, Z** (m).
    - **Roll, Pitch, Yaw** (rad o grados).
  - Obtener estos datos ya sea de:
    - La cinemática directa implementada con la Toolbox, o
    - La información de MoveIt (por ejemplo, pose actual del `end_effector_link`).

---

## 5. Entregables sugeridos

- [ ] **Documento o sección de informe**
  - Explicar brevemente:
    - Cómo se obtuvo la tabla DH.
    - Cómo se validó la cinemática directa.
    - Cómo se integró ROS 2 + Toolbox + GUI.
- [ ] **Evidencias**
  - Capturas de pantalla de:
    - RViz con el modelo del robot.
    - La GUI con las nuevas pestañas y botones.
    - La toolbox mostrando el modelo y la pose.
  - Video corto (si es posible) mostrando:
    - El movimiento secuencial HOME → pose objetivo.
    - El uso del control en espacio de la tarea desde la GUI.


