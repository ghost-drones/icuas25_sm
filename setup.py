from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'icuas25_sm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['icuas25_sm', 'icuas25_sm.*']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Lucca Gandra',
    author_email='luccagandra@poli.ufrj.br',
    maintainer='Lucca Gandra',
    maintainer_email='luccagandra@poli.ufrj.br',
    description='Yasmin SM - Icuas25 Competition',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'MainStateMachine = icuas25_sm.MainStateMachine:main',
            'Data_Wrapper = icuas25_sm.Data_wrapper:main',
        ],
    },
)
