# Exportar a ROS 2
Los paquetes de ROS 1 y ROS 2 tienen un estandar, donde se recomienda (aunque no es obligatorio) **empezar por una letra minúscula y contener SOLO letras minúsculas, dígitos, guiones bajos y guiones**. No recomiendo los guiones (solo los guines bajos), pero deben seguir esas instrucciones al exportar el robot en SolidWorks. 

## Importar proyecto en Ubuntu

Entren a Ubuntu e importen el repositorio del proyecto de robótica desde GitHub. Una vez dentro, en Visual Studio Code abran la carpeta desde `Archivo` -> `Abrir carpeta...` para trabajar con GitHub desde ahí.

Ahora, convertirán el proyecto en un Workspace de ROS 2. Un Workspace permite crear paquetes o bibliotecas locales que no interfieran con otros Workspaces, por lo que es algo similar al proyecto de Matlab. Para convertirlo en un Workspace, abre un terminal y asegúrate de estar en la carpeta del proyecto (puedes cambiar con `cd ~/Robotica_o_carpeta_del_proyecto`) y ejecutar

```bash
colcon build --symlink-install
```

Esto creará las siguientes carpetas:
- `src/`: donde colocamos los **paquetes** (como `mi_robot`, `tutorial` o `sensores_genericos`). Yo les dije que por estandar, ya debíamos tener esa carpeta.
- `build/`: donde se guardan archivos temporales de compilación.
- `install/`: donde se instalan los paquetes ya compilados.
- `log/`: registros de la compilación.

Si está configurado bien `.gitignore`, deberían de verse de color gris y no deberían de aparecer como spam de cientos de modificaciones de git. 

Normalmente no se necesita usar la opción `--symlink-install`, pero la primera vez sirve para que no se generen copias físicas del código de los paquetes en la carpeta `install`.

El comando `colcon build` se usa para compilar el código máquina de C++, por lo que si un paquete usa C++, es necesario usar el comando cada vez que se cambia el código. Python utiliza algo llamado intérprete que le permite que se lea diréctamente el código en lenguaje de alto nivel, por lo que no es necesario convertirlo a código máquina y, por lo tanto, no es necesairo usar `colcon build` cada vez que se modifique el código. Sin embargo, sí es necesario usarlo al crear un nuevo paquete para que ROS 2 los encuentre automáticamente.

Ahora, cada vez que se abra una terminal, para que use el Workspace, es necesario ejecutar

```bash
source install/setup.bash
```

Pero si solo usarás el workspace, se puede añadir al archivo `.bashrc` para que siempre se ejecute al abrir el terminal. Puedes hacerlo desde el navegador de Windows entrando a `Linux\Ubuntu-24.02\home\$USUARIO\.bashrc`, aunque también puedes usar Visual Studio Code o abrir el editor de ubuntu con

```bash
gedit ~/.bashrc
```

Y añade `source install/setup.bash` para que asiempre se ejecute. Recuerda cerrar y abrir el terminal para que haga efecto.




Cabe destacar que es necesario importar el repositorio del proyecto de robótica a la carpeta de usuario de Ubuntu. Una vez hecho eso, abren un nuevo terminal **dentro de Ubuntu** e importan el [repositorio](https://github.com/xiaoming-sun6/sw2urdf_ros2) para convertir el proyecto de ROS 1 a ROS 2.

```bash
git clone https://github.com/xiaoming-sun6/sw2urdf_ros2.git
```

