from setuptools import find_packages, setup
from glob import glob # glob es la abreviacion de global para retornar todos los paths que se vean de una forma especifica
import os

package_name = 'scrips_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,"launch"), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carloso',
    maintainer_email='carlosalbertolivmo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'ackerman_node = scrips_node.ackerman_node:main',
        'follow_the_gap = scrips_node.follow_the_gap:main',
        'pidWF_node = scrips_node.pidWF_node:main',
        'stop_node = scrips_node.stop_node:main',
        ],
    },
)
