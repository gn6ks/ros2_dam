from setuptools import find_packages
from setuptools import setup

setup(
    name='lbr_bringup',
    version='2.4.3',
    packages=find_packages(
        include=('lbr_bringup', 'lbr_bringup.*')),
)
