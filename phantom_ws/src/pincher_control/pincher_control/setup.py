"""
Setup para el paquete pincher_control.
"""
from setuptools import setup, find_packages

setup(
    name='pincher_control',
    version='2.0.0',
    packages=find_packages(),
    install_requires=[
        'rclpy',
        'dynamixel_sdk',
        'PyQt5',
        'numpy',
        'roboticstoolbox-python',
        'spatialmath-python',
    ],
    entry_points={
        'console_scripts': [
            'pincher_gui = pincher_control.main:main',
        ],
    },
    author='Samuel Sanchez & Santiago Ávila',
    author_email='',
    description='Control modular del robot PhantomX Pincher',
    license='MIT',
    python_requires='>=3.8',
)
