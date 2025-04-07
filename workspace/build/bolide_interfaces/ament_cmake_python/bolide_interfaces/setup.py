from setuptools import find_packages
from setuptools import setup

setup(
    name='bolide_interfaces',
    version='0.1.0',
    packages=find_packages(
        include=('bolide_interfaces', 'bolide_interfaces.*')),
)
