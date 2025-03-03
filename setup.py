from setuptools import setup
import os
from glob import glob

package_name = 'icuas25_sm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Necessário para que o ROS2 encontre o package.xml
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Instala os arquivos de launch e parâmetros
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='root',
    author_email='luccagandra@poli.ufrj.br',
    maintainer='root',
    maintainer_email='luccagandra@poli.ufrj.br',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Data_Wrapper = icuas25_sm.Data_Wrapper:main',
            'MainStateMachine = icuas25_sm.MainStateMachine:main',
        ],
    },
)
