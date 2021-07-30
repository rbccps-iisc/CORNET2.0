from setuptools import find_packages
from setuptools import setup

setup(
    name='heartbeat',
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[],
    install_requires=['setuptools'],
    author='Alexander Roessler',
    author_email='alex@machinekoder.com',
    maintainer='Alexander Roessler',
    maintainer_email='alex@machinekoder.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Hearbeat message publisher.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heartbeat_publisher = heartbeat.heartbeat_publisher:main',
        ],
    },
)
