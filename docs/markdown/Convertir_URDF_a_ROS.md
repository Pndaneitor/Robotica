<h1>Importar paquete de SolidWorks a ROS con MoveIt</h1>

A pesar de que sw2urdf genera varios archivos, es necesario hacer algunos cambios para simular nuestro robot.

Los paquetes de ROS tienen un estandar, donde se recomienda (aunque no es obligatorio) **empezar por una letra minúscula y contener SOLO letras minúsculas, dígitos, guiones bajos y guiones**. No recomiendo los guiones (solo los guines bajos), pero deben seguir esas instrucciones al exportar el robot en SolidWorks. 

<h2>Índice</h2>

<- El paso anterior es [Exportar SolidWorks a URDF](sw2urdf.md)

- [Importar proyecto en Ubuntu](#importar-proyecto-en-ubuntu)
  - [En resumen:](#en-resumen)
- [Copiar a Ubuntu el paquete del robot](#copiar-a-ubuntu-el-paquete-del-robot)
- [Editar CMakeLists.txt](#editar-cmakeliststxt)
- [Editar package.xml](#editar-packagexml)
- [Modificar el URDF](#modificar-el-urdf)


## Importar proyecto en Ubuntu

Los pasos descritos aquí aparecen en su mayoría en el pdf con el [tutorial de Age of Robotics](https://github.com/ageofrobotics/import_your_custom_urdf_package_to_ROS-main/blob/main/Importing_URDF_Package_from_Soloidworks_in_ROS.pdf), los cuales se pueden ver también en [youtube](https://www.youtube.com/watch?v=ZWliEJfNtlM). Aun así, recomiendo seguir los primeros pasos para crear el Workspace dentro del repositorio de GitHub de la materia.

Entren a Ubuntu e importen el repositorio de ustedes del proyecto de robótica desde GitHub (cambien `UsuarioDeGitHub` y `RepositorioRobotica` y dejen al final `~/Robotica`); recomiendo guardarlo en una carpeta llamada Robotica para que sea más fácil seguir el tutorial.
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

O puedes hacerlo desde el navegador de Windows entrando a `Linux\Ubuntu-20.02\home\$USUARIO\.bashrc`, aunque también puedes usar Visual Studio Code o abrir el editor de ubuntu con

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
echo 'source ~/Robotica/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## Copiar a Ubuntu el paquete del robot

Lo siguiente es copiar todos los archivos y carpetas del paquete generado por `sw2urdf` al generado en la carpeta `src/`.

## Editar CMakeLists.txt
El archivo `CMakeLists.txt` contiene la configuración de C++, por lo que los nodos programados en ese lenguaje deben añadirse ahí. También se añaden los paquetes necesarios para ejecutar el paquete, por lo que si su archivo se parece a esto

```cmake
cmake_minimum_required(VERSION 2.8.3)

project(MI_ROBOT)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
```
Deben cambiarlo a esto. Recuerden cambiar `MI_ROBOT` por el de su paquete.

```cmake
cmake_minimum_required(VERSION 2.8.3)

project(era_description)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	rospy
	std_msgs
	geometry_msgs
	urdf
	xacro
	message_generation
)
catkin_package(
	CATKIN_DEPENDS
		geometry_msgs
		roscpp
		rospy
		std_msgs
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
```

## Editar package.xml
También tienen que cambiar el archivo `package.xml`. Cambien donde dice `MI_ROBOT`, el autor y correo. Si quieren poner múltiples autores, solo tienen que poner la linea de <author> y <maintainer> varias veces hasta que estén todos los integrantes.

```xml
<package format="2">
  <name>MI_ROBOT</name>
  <version>1.0.0</version>
  <description>
    <p>URDF Description package for MI_ROBOT</p>
    <p>This package contains configuration data, 3D models and launch files
for era_description robot</p>
  </description>
  <author>TODO</author>
  <maintainer email="TODO@email.com" />
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>urdf</build_depend>
  <build_depend>xacro</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  <depend>roslaunch</depend>
  <depend>robot_state_publisher</depend>
  <depend>rviz</depend>
  <depend>joint_state_publisher</depend>
  <depend>joint_state_publisher_gui</depend>
  <depend>gazebo</depend>
  <depend>moveit_simple_controller_manager</depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>urdf</build_export_depend>
  <build_export_depend>xacro</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>urdf</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>message_runtime</exec_depend>
  <export>
    <architecture_independent />
  </export>
</package>
```

El archivo `package.xml` contiene una lista de los paquetes necesarios para que este paquete funcione. Si intentan compilar de nuevo con `catkin build`, probablemente aparezca un error porque no tienen instalado un paquete. Es por ello que una práctica común es ejecutar
```bash
cd ~/Robotica
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin build
```
para instalar todos los paquetes que se encuentran en `package.xml` y compilarlo.

## Modificar el URDF
Ahora se debe modificar el URDF para que se pueda simular en gazebo. Este archivo describe todos los componentes del robot usando el lenguaje xml. 

En xml, una etiqueta empieza con `<` y termina con `/>`. Si queremos que una etiqueta contenga varias etiquetas dentro, se puede poner en una línea `<etiqueta>` y en otra línea `</etiqueta>` con el mismo nombre pero con `/`, de forma que todo lo que esté dentro de las dos forme parte de `etiqueta`. 

