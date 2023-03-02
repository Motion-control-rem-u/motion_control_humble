# MOTION CONTROL HUMBLE

Este repositorio contiene el trabajo desarrollado por el subsistema en 202310

TODO: Documentacion

Codigos que debemos migrar:

* control_rem.py
* --joystick_publisher.py--
* keyboard_publisher.py (creo)
* node_joystick_traction.py (creo)
* serial_writer_4motors.py
* y cosas de nav autonoma

## Documentacion

Para validar dependencias:

`rosdep install -i --from-path src --rosdistro humble -y`

Para armar el workspace (toca hacerlo despues de cada cambio):

`colcon build --packages-select motion_pkg`

Despues, en otro terminal:

`. install/setup.bash`

`ros2 run motion_pkg [node name]`
