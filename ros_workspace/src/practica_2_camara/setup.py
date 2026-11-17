from glob import glob

from setuptools import find_packages, setup

package_name = 'practica_2_camara'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laboratorio',
    maintainer_email='laboratorio@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'capturar_imagenes = practica_2_camara.capturar_imagenes:main',
            'tablero_simulado = practica_2_camara.tablero_simulado:main',
            'corregir_imagen = practica_2_camara.corregir_imagen:main',
        ],
    },
)
