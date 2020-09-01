# Ude_ros

## Começo rápido

** Simulação no MoveIt! e Gazebo **:

`` `
roslaunch ude_gazebo empty_world.launch
`` `

## Pacotes
[ude_control]:
Este pacote faz interface com ros_control para permitir que moveit controle o modelo Gazebo

[ude_test_description]:
Este pacote contém descrições Xacro e URDF que fazem interface com Rviz e Gazebo.

[ude_gazebo]:
Este pacote simplesmente lança os modelos Ude e relacionados com o ambiente.

[ude_test_moveit]:
Este pacote controla toda a interface do moveit com o gazebo e o robô real

[rqt_mypkg]: 

[ude_arduino] :
Este pacote faz interface entre o MoveIt! com o robô real controlado pelo arduino.
