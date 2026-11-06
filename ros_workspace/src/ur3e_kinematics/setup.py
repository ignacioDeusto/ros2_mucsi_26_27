from setuptools import find_packages, setup

package_name = 'ur3e_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Markel',
    maintainer_email='markel.crespo@opendeusto.es',
    description='Extracción y análisis de datos cinemáticos (posición, velocidad y aceleración) del robot UR3e',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lector_articulaciones = ur3e_kinematics.lector_articulaciones:main',
            'trayectorias = ur3e_kinematics.trayectorias:main',
        ],
    },
)
