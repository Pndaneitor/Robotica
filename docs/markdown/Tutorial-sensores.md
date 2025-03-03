- [Tutorial de Sensores en Gazebo](#tutorial-de-sensores-en-gazebo)
- [Crear espacio de trabajo](#crear-espacio-de-trabajo)
- [Abriendo el archivo](#abriendo-el-archivo)
- [Añadiendo el sensor](#añadiendo-el-sensor)

# Tutorial de Sensores en Gazebo
El contenido está basado en el [tutorial que viene en la página de Gazebo](https://gazebosim.org/docs/jetty/sensors/), pero se cambió para que se puedan personalizar más los sensores. 

# Crear espacio de trabajo
Para este tutorial no es necesario hacer un Workspace (lo veremos después), pero pondremos los archivos en una carpeta.
```bash
mkdir -p ~/tutoriales_gazebo
cd ~/tutoriales_gazebo
wget -O sensor_tutorial.sdf https://github.com/gazebosim/docs/blob/master/jetty/tutorials/moving_robot/moving_robot.sdf 
```
Esto crea el archivo `sensor_tutorial.sdf` dentro de la carpeta `tutoriales_gazebo/`, el cual contiene todos los componentes de un robot básico tipo carro. Se modificará para añadirle los sensores.

# Abriendo el archivo
Para añadir el IMU a nuestro robot, abre el archivo con Visual Studio Code. Presionando en el botón `><` que se encuentra en la esquina inferior izquierda:

![Conexión remota](assets/Conexion_remota.png)

Si no lo habías hecho, instala la extensión y luego selecciona **conectar a WSL mediante distribución**

![Conectar a WSL](assets/WSL_VSCode.png)

Entonces selecciona Ubuntu 24.04. En el explorador de Visual Studio Code, selecciona **Abrir carpeta** y selecciona la que dice `/home/usuario/`, siendo `usuario` el que creaste.
Abre el archivo que está en la carpeta `/tutoriales_gazebo` para poder editarlo en **Visual Studio Code**.

El archivo usa un formato llamado `xml`, el cual es un lenguaje de markado que permite foemar estructuras (por ejemplo, está en las facturas electrónicas y diferentes archivos), y no importa el orden en el que se encuentren las etiquetas mientras se encuentren dentro de una. Por ejemplo, todo lo que esté dentro de 
<world>

# Añadiendo el sensor
La unidad de medición inercial (IMU, por sus siglas en inglés) proporciona la orientación de nuestro robot en cuaterniones, la velocidad angular en los tres ejes (X, Y, Z) y la aceleración lineal en los tres ejes.
Para que el IMU funcione, necesitan añadir el plugin dentro de la etiqueta `<world>`. No importa si lo pones

# Tutorial: Creación de Sensores en Gazebo con SDF

Este tutorial está basado en la documentación oficial de Gazebo:
[Gazebo Sensors](https://gazebosim.org/docs/jetty/sensors/). Se recomienda revisar dicho tutorial para obtener información detallada sobre la versión de Gazebo utilizada.

Para especificaciones adicionales sobre sensores en **SDF (Simulation Description Format)**, consulte:
[Especificaciones de Sensores en SDFormat](http://sdformat.org/spec), sección **sensors**. En este tutorial, utilizaremos **SDFormat 1.9**, ya que es la versión empleada en la documentación oficial de Gazebo para sensores.

Además, como referencia para los valores de los sensores, usaremos la IMU **MPU9250**, tomando en cuenta los valores de acelerómetro y magnetómetro indicados en la siguiente tabla.

## **1. Agregando una IMU a un Robot en SDF**

Para este ejemplo, utilizaremos como base el archivo SDF de un robot simple disponible en:
[Moving Robot SDF](https://github.com/gazebosim/docs/blob/master/jetty/tutorials/moving_robot/moving_robot.sdf).

### **1.1. Código Base del Robot**
Antes de agregar la IMU, el robot base en SDF luce de la siguiente manera:

```xml
<sdf version='1.9'>
  <model name='moving_robot'>
    <static>false</static>
    <link name='chassis'>
      <pose>0 0 0.5 0 0 0</pose>
      <visual name='visual'>
        <geometry>
          <box>
            <size>1 0.5 0.25</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

### **1.2. Agregando una IMU al Robot**
Para agregar una IMU al robot, añadiremos un nuevo sensor en el `link` del `chassis` y lo configuraremos con un plugin de Gazebo.

```xml
<link name='chassis'>
  <pose>0 0 0.5 0 0 0</pose>
  <visual name='visual'>
    <geometry>
      <box>
        <size>1 0.5 0.25</size>
      </box>
    </geometry>
  </visual>

  <!-- Sensor IMU -->
  <sensor name='imu_sensor' type='imu'>
    <always_on>1</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type='gaussian'>
            <mean>0.0</mean>
            <stddev>0.1</stddev> <!-- Basado en la IMU MPU9250 -->
          </noise>
        </x>
        <y>
          <noise type='gaussian'>
            <mean>0.0</mean>
            <stddev>0.1</stddev>
          </noise>
        </y>
        <z>
          <noise type='gaussian'>
            <mean>0.0</mean>
            <stddev>0.1</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type='gaussian'>
            <mean>0.0</mean>
            <stddev>0.008</stddev> <!-- Basado en la IMU MPU9250 -->
          </noise>
        </x>
        <y>
          <noise type='gaussian'>
            <mean>0.0</mean>
            <stddev>0.008</stddev>
          </noise>
        </y>
        <z>
          <noise type='gaussian'>
            <mean>0.0</mean>
            <stddev>0.008</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    
    <!-- Plugin de Gazebo para la IMU -->
    <plugin name='gz-sim-imu-system' filename='gz-sim-imu-system'>
    </plugin>
  </sensor>
</link>
```

### **1.3. Explicación del Código**
- **`<sensor>`**: Define un sensor de tipo `imu`.
- **`<update_rate>`**: Define la frecuencia de actualización en Hz.
- **`<noise>`**: Se agregan valores de ruido basados en las especificaciones de la **IMU MPU9250**.
- **`<plugin>`**: Se usa el plugin `gz-sim-imu-system` para integrar la IMU en Gazebo Sim.

---

### **Siguientes Pasos**
1. Ejecutar el robot con la IMU en Gazebo.
2. Obtener los datos del sensor IMU a través de ROS 2 o directamente desde Gazebo.
3. Agregar más sensores al robot, como LiDAR o cámaras.

En la siguiente sección, abordaremos la integración con ROS 2 para visualizar los datos de la IMU en tiempo real.
