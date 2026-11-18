# Origen de esta copia

Copia de <https://github.com/enactic/openarm_ros2>:

- Commit: `8f23aeabb372c256f7807b9a74d48e035fc68386` (2026-10-21), el mismo del que se
  importó la configuración de MoveIt de `launcher_robots_lab_robotica/openarm/moveit/`.
- Obtenida con `git archive` (sin historial).

Solo se compila `openarm_hardware`. `openarm`, `openarm_bimanual_moveit_config` y
`openarm_bringup` llevan `COLCON_IGNORE` para no duplicar la descripción ni el moveit_config
que ya trae `launcher_robots_lab_robotica` (y porque dependen de `openarm_description`,
que no está en el espacio de trabajo).

## Cambios locales

- `openarm_hardware/src/openarm_simple_hardware.cpp` (`generate_joint_names`): toma los nombres
  de las articulaciones del bloque `<ros2_control>` del URDF en lugar de construirlos como
  `openarm_<arm_prefix>jointN`. Sin este cambio el plugin exporta `openarm_izq_joint1/...`
  mientras que el URDF renombrado por `lanzador_openarm.py` y los controladores usan
  `301_izq_joint1`, y los controladores no pueden reclamar sus interfaces. Si el URDF no
  declara las articulaciones esperadas, mantiene el comportamiento original.
