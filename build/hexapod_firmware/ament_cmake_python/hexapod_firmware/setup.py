from setuptools import find_packages
from setuptools import setup

setup(
    name='hexapod_firmware',
    version='0.0.0',
    packages=find_packages(
        include=('hexapod_firmware', 'hexapod_firmware.*')),
)
