# Guía 1 – Clasificación de objetos con PhantomX Pincher en ROS 2

## Objetivo

El objetivo de esta guía es crear una **rutina de robot** que pueda recoger **4 objetos distintos** que siempre se ubicarán en la **zona de recolección** y luego depositarlos en la caneca correspondiente según su tipo.

Los objetos son:

- Un **cubo**
- Un **cilindro**
- Un **pentágono**
- Un **rectángulo**

> **Nota:** Todos estos objetos tendrán **la misma altura**.

La clasificación de los objetos en las canecas será:

- El **cubo** debe dejarse en la **caneca roja**
- El **cilindro** debe dejarse en la **caneca verde**
- El **pentágono** debe dejarse en la **caneca azul**
- El **rectángulo** debe dejarse en la **caneca amarilla**

---

## Descripción general de la tarea

Se debe crear un **nodo de ROS 2** que:

1. **Reciba como parámetro** el **tipo de figura** que está en la zona de recolección.
2. **Reciba también** (es decir, el nodo debe ser **listener y publisher a la vez**) la **posición de cada caneca** según su color.
3. Con esta información, debe **mover el robot** a una serie de posiciones hasta alcanzar la posición donde debe **dejar el objeto** en la caneca correcta.
4. El nodo debe **publicar** las posiciones objetivo del efector final al tópico:

   ```bash
   /pose_command
   ```

   Más adelante se muestra cómo publicar a este tópico desde la consola.

> **Importante:** El robot tiene **dos planning groups** distintos:
>
> - Uno para el **movimiento del brazo**
> - Otro para el **movimiento del gripper**

Para probar el robot, se deben **ingresar por consola** los parámetros de:

- Tipo de figura
- Posición de la caneca correspondiente

De esta manera, el nodo moverá el brazo del robot a la posición deseada y ejecutará la rutina de clasificación.

---

## Nodo ROS 2 requerido

El nodo debe cumplir con las siguientes características:

- Ser capaz de:
  - **Suscribirse** a la información que indica:
    - Tipo de figura en la zona de recolección.
    - Posición de cada caneca (roja, verde, azul, amarilla).
  - **Publicar** comandos de posición al tópico:

    ```bash
    /pose_command
    ```

- Utilizar el mensaje:

  ```text
  phantomx_pincher_interfaces/msg/PoseCommand
  ```

- Generar una **secuencia de poses** (trayectoria) para:
  1. Ir desde una posición de reposo hasta la zona de recolección.
  2. Cerrar el gripper para recoger el objeto.
  3. Llevar el objeto a la caneca correspondiente según su tipo.
  4. Soltar el objeto y regresar a una posición segura o de reposo.

---

## Tópico `/pose_command`

El tópico al que publicará el nodo se llama:

```bash
/pose_command
```

El mensaje `PoseCommand` tiene la siguiente estructura (ejemplo de uso desde consola):

```bash
ros2 topic pub -1 /pose_command phantomx_pincher_interfaces/msg/PoseCommand \
"{x: 0.128, y: 0.0, z: 0.100, roll: 3.142, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
```

Donde:

- `x, y, z` → Posición del efector final en coordenadas del robot.
- `roll, pitch, yaw` → Orientación del efector final.
- `cartesian_path` → Indica si la trayectoria debe ser cartesiana o no.

---

## URDF y Xacro

Para el desarrollo de la guía es necesario **modificar el archivo `kit.xacro`** ubicado en:

```text
KIT_Phantom_X_Pincher_ROS2/phantom_ws/src/phantomx_pincher_description/urdf/kit.xacro
```

El archivo `kit.xacro` actual **no incluye**:

- El **mástil** que sostiene la cámara en el kit.
- La **canastilla** alrededor del kit.

### 1. Descarga de modelos 3D (STL)

Para añadir estos elementos al `kit.xacro`:

1. Descargue los archivos STL del:
   - Mástil
   - Canastilla

2. Colóquelos en la carpeta:

   ```text
   KIT_Phantom_X_Pincher_ROS2/phantom_ws/src/phantomx_pincher_description/meshes/STL/
   ```

3. Los archivos STL se encuentran en el siguiente repositorio:

   ```text
   https://github.com/labsir-un/3DModels_KIT_Phantom_Pincher_X100/tree/main/Kit/STLS
   ```

### 2. Modificación de `kit.xacro`

Una vez añadidos los archivos STL:

- Debe **referenciar estos modelos** dentro de `kit.xacro` usando las etiquetas correspondientes para:
  - Los **visuals** (apariencia del modelo).
  - Las **collisions** (colisiones del modelo).

> **Nota:** Asegúrese de añadir también las **colisiones** de cada nuevo elemento para que el planificador de movimiento las tenga en cuenta.

---

## ¿Cómo identificar las posiciones correctas para el brazo?

Para encontrar posiciones adecuadas para la rutina (recolección, transporte y depósito), se propone el siguiente procedimiento.

### 1. Lanzar el robot con MoveIt y RViz

En un primer terminal:

```bash
# Terminal 1
. install/setup.bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py
```

Esto abrirá **RViz con MoveIt**.

En RViz:

- Mueva el **brazo** a una posición que considere **apropiada** para la rutina
  (por ejemplo, posición de recolección o posición sobre una caneca).

### 2. Leer la pose del efector final con tf2

En un segundo terminal:

```bash
# Terminal 2
ros2 run tf2_ros tf2_echo phantomx_pincher_base_link phantomx_pincher_end_effector
```

> **Nota:** Este comando permite ver la **información del efector final** una vez se ha cambiado la posición en MoveIt.

Tome nota de:

- `translation` (x, y, z)
- `rotation` (en cuaterniones o convierta luego a roll, pitch, yaw)

Guarde esta información, ya que la usará para probar las poses publicando al tópico `/pose_command`.

### 3. Verificación de la pose usando `/pose_command`

Para comprobar que la posición anotada es correcta:

1. Cierre RViz y el lanzamiento anterior usando `Ctrl + C` en ambos terminales.

2. Vuelva a lanzar el robot:

   ```bash
   # Terminal 1
   . install/setup.bash
   ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py
   ```

3. En un segundo terminal, publique en el tópico `/pose_command` usando la información que anotó anteriormente. Ejemplo:

   ```bash
   # Terminal 2
   . install/setup.bash
   ros2 topic pub -1 /pose_command phantomx_pincher_interfaces/msg/PoseCommand \
   "{x: 0.128, y: 0.0, z: 0.100, roll: 3.142, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
   ```

Si todo está correcto, debería ver cómo el robot se **mueve a la posición deseada**.

---

## Resumen del flujo de trabajo

1. **Preparar el modelo del robot**
   - Descargar los STL del mástil y la canastilla.
   - Añadirlos al `kit.xacro` con sus correspondientes `visual` y `collision`.

2. **Definir las posiciones clave**
   - Usar RViz + MoveIt para ubicar el efector final en:
     - Zona de recolección.
     - Caneca roja.
     - Caneca verde.
     - Caneca azul.
     - Caneca amarilla.
   - Obtener las poses con `tf2_echo` y guardarlas.

3. **Implementar el nodo ROS 2**
   - Suscribirse a la información de:
     - Tipo de figura.
     - Posición de las canecas.
   - Publicar comandos de posición en `/pose_command` usando `PoseCommand`.
   - Implementar la lógica:
     - Si figura = cubo → ir a posición caneca roja.
     - Si figura = cilindro → ir a posición caneca verde.
     - Si figura = pentágono → ir a posición caneca azul.
     - Si figura = rectángulo → ir a posición caneca amarilla.

4. **Probar la rutina**
   - Ingresar parámetros de figura y posición de canecas por consola.
   - Verificar que el robot:
     - Toma el objeto en la zona de recolección.
     - Lo deposita en la caneca correcta según su tipo.

---

## Notas finales

- Asegúrese de tener correctamente configurado el entorno de trabajo:

  ```bash
  . install/setup.bash
  ```

- Verifique siempre las poses antes de usarlas en la rutina automática.
- Recuerde que el robot tiene **planning groups separados** para:
  - Brazo
  - Gripper

Esto debe tenerse en cuenta al momento de planear y ejecutar los movimientos.
