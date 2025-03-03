from setuptools import find_packages, setup
from os.path import join
from glob import glob

package_name = 'icuas25_sm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, 'launch'), glob('launch/*.yaml')),
        (join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pedrograca',
    maintainer_email='pedrojullian@poli.ufrj.br',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],  # Substituindo `tests_require`
    },
    entry_points={
        'console_scripts': [
            'MainStateMachine = icuas25_sm.MainStateMachine:main',
        ],
    },
)