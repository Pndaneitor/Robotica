# Tutorial: Control de **Turtlesim** con Tópicos en ROS Noetic

En esta práctica aprenderás a:

1. Lanzar el simulador **turtlesim**.  
2. Explorar los tópicos disponibles.  
3. Escribir un nodo **publisher** en Python para enviar velocidades (`geometry_msgs/Twist`) a la tortuga.  
4. Escribir un nodo **subscriber** en Python para leer la posición de la tortuga.  
5. Combinar ambos para controlar la tortuga desde tu código.

## 1. Lanzar el simulador

En un terminal, abre el nodo de ROS MASTER:

```bash
source /opt/ros/noetic/setup.bash
roscore
```

y en otro terminal ejecuta el nodo de turtlesim:

```bash
source /opt/ros/noetic/setup.bash
rosrun turtlesim turtlesim_node
```

Verás una ventana con la tortuga.

## 3. Explorar los tópicos

Con otro terminal ejecuta uno por uno los siguientes comandos y analiza lo que aparece:

```bash
# muestra la lista de nodos activos
rosnode list
```
```bash
# muestra los topics disponibles
rostopic list
```
Deberías ver, entre otros:

* `/turtle1/cmd_vel` — para **publicar** velocidades (tipo `geometry_msgs/Twist`).
* `/turtle1/pose`    — para **suscribirse** a la posición (tipo `turtlesim/Pose`).

Para inspeccionar los mensaje en vivo, primero tienes que ver el type del topic, o el tipo de mensaje que usan. Por ejemplo, si escribes

```bash
rostopic type /turtle1/pose
```

verás que el type es `turtlesim/Pose` y si ahora usas

```bash
rosmsg show turtlesim/Pose
```

Se mostrará el mensaje (digamos la variable enviada) y el tipo de dato que se encía, como `float32`.

Si queremos ver ahora el mensaje que envía en tiempo real, podemos ejecutar

```bash
rostopic echo /turtle1/pose
```

Y para que deje de ejecutarse, presionamos `ctrl` + `c`.

## 4. Publicar comandos de velocidad desde la línea

Prueba a mover la tortuga sin escribir código:

```bash
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

> La tortuga avanzará hacia adelante y girará.

## 5. Workspace de Robótica

Si ya crearon el Workspace, vayan al [paso 6](#6-nodo-publisher-en-python) aun no lo han hecho, importen el repositorio de ustedes del proyecto de robótica desde GitHub (cambien `UsuarioDeGitHub` y `RepositorioRobotica` y dejen al final `~/Robotica`); recomiendo guardarlo en una carpeta llamada Robotica para que sea más fácil seguir el tutorial.
```bash
git clone https://github.com/UsuarioDeGitHub/RepositorioRobotica.git ~/Robotica
```
Una vez dentro, en Visual Studio Code abran la carpeta desde `Archivo` -> `Abrir carpeta...` para trabajar con GitHub desde ahí.

Ahora, convertirán el proyecto en un Workspace de ROS. Un Workspace permite crear paquetes o bibliotecas locales que no interfieran con otros Workspaces, por lo que es algo similar al proyecto de Matlab. Para convertirlo en un Workspace, abre un terminal y asegúrate de estar en la carpeta del proyecto; la pueden llamar como `Robotica` (según este tutorial) o `moveit_ws` (según el tutorial del pdf) para no batallar en copiar y pegar.

```bash
cd ~/Robotica
catkin init
```

Verifican que no hay un error grande, como haber creado un workspace dentro de otro workspace. Después ejecuten

```bash
catkin build
```

Esto creará las siguientes carpetas:
- `src/`: donde colocamos los **paquetes** (como `mi_robot`, `tutorial` o `sensores_genericos`). Yo les dije que por estandar, ya debíamos tener esa carpeta.
- `build/`: donde se guardan archivos temporales de compilación.
- `devel/`: donde se instalan los paquetes ya compilados.
- `log/`: registros de la compilación.

Si está configurado bien `.gitignore`, deberían de verse de color gris y no deberían de aparecer como spam de cientos de modificaciones de git. 

El comando `catkin build` se usa para compilar el código máquina de C++, por lo que si un paquete usa C++, es necesario usar el comando cada vez que se cambia el código. Python utiliza algo llamado intérprete que le permite que se lea diréctamente el código en lenguaje de alto nivel, por lo que no es necesario convertirlo a código máquina y, por lo tanto, no es necesario usar `catkin build` cada vez que se modifique el código. Sin embargo, sí es necesario usarlo al crear un nuevo paquete para que ROS los encuentre automáticamente.

Ahora, cada vez que se abra una terminal, para que use el Workspace, es necesario ejecutar (dentro de la carpeta del workspace):

```bash
source devel/setup.bash
```

Pero si solo usarás un workspace y no varios, se puede añadir al archivo `.bashrc` para que siempre se ejecute al abrir el terminal con
 ```bash
 echo 'source ~/Robotica/devel/setup.bash' >> ~/.bashrc
 source ~/.bashrc
```

O puedes hacerlo desde el navegador de Windows entrando a `Linux\Ubuntu-20.04\home\$USUARIO\.bashrc`, aunque también puedes usar Visual Studio Code o abrir el editor de ubuntu con

```bash
gedit ~/.bashrc
```

Y añade `source ~/Robotica/install/setup.bash` para que asiempre se ejecute. Recuerda cerrar y abrir el terminal para que haga efecto.

### En resumen:
1. Abrir Ubuntu 20.04.
2. Clonar repositorio en la carpeta Robotica.
```bash
git clone https://github.com/UsuarioDeGitHub/RepositorioRobotica.git ~/Robotica
```

3. Abrir carpeta en Visual Studio Code.
4. Verificar que se puede crear el Workspace
```bash
cd ~/Robotica
catkin init
```
5. Crear Workspace.
```bash
catkin build
```
5.  Abrirlo por defecto en terminal.
```
echo 'source ~/Robotica/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## 6. Nodo **publisher** en Python

Crea un paquete (si no lo tienes):

```bash
cd ~/Robotica/src
catkin_create_pkg turtle_control rospy geometry_msgs
cd ~/Robotica
catkin build
sb
```

Dentro de `~/Robotica/src/turtle_control/scripts/`, crea `move_turtle.py`:

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def mover_tortuga():
    rospy.init_node('mover_tortuga', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(2)  # 2 Hz

    cmd = Twist()
    cmd.linear.x = 1.0    # velocidad hacia adelante
    cmd.angular.z = 0.5   # velocidad de giro

    while not rospy.is_shutdown():
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        mover_tortuga()
    except rospy.ROSInterruptException:
        pass
```

Hazlo ejecutable y lánzalo:

```bash
chmod +x ~/Robotica/src/turtle_control/scripts/move_turtle.py
rosrun turtle_control move_turtle.py
```

---

## 6. Nodo **subscriber** en Python

En el mismo paquete, crea `read_pose.py`:

```python
#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

def callback(pose):
    rospy.loginfo("Tortuga en x=%.2f, y=%.2f, θ=%.2f", pose.x, pose.y, pose.theta)

def leer_pose():
    rospy.init_node('leer_pose', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, callback)
    rospy.spin()

if __name__ == '__main__':
    leer_pose()
```

Hazlo ejecutable y pruébalo en paralelo con `turtlesim_node`.

---

## 7. Ejercicio integrado (documenta el proceso con capturas de pantalla y un documento)

1. Modifica `move_turtle.py` para que durante 2 s la tortuga avance y luego gire.
2. Publica velocidades variables (usa `rospy.get_time()` para controlar fases).
3. Observa la salida de `read_pose.py` para verificar que la tortuga se mueve donde esperas.
4. Lee los servicios activos con `rosservice list` e invoca otra tortuga usando `rosservice call /spawn` y el tabulador. Recuerda cambiar el nombre de la tortuga y su posición inicial.
5. Haz que se muevan en bucle
6. Haz que las dos tortugas se muevan y graba un video de 10 segundos.