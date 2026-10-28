from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'launcher_robots_lab_robotica'

# Ficheros de datos del árbol openarm/ (URDF, xacros, mallas y configuración de
# MoveIt del OpenArm) instalados respetando su estructura de carpetas
datos_openarm = []
for carpeta, _, ficheros in os.walk('openarm'):
    datos_openarm.append((
        os.path.join('share', package_name, carpeta),
        [os.path.join(carpeta, f) for f in ficheros],
    ))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/calibration', glob('calibration/*.yaml')),
        ('share/' + package_name + '/meshes/visual', glob('meshes/visual/*')),
        ('share/' + package_name + '/meshes/collision', glob('meshes/collision/*')),
    ] + datos_openarm,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ignacio',
    maintainer_email='ignacio@todo.todo',
    description='Lanzadores y configuración de los robots UR3e + RG2 del laboratorio',
    license='TODO',
    entry_points={
        'console_scripts': [
            'pinza_openarm_node = launcher_robots_lab_robotica.pinza_openarm:main',
        ],
    },
)
