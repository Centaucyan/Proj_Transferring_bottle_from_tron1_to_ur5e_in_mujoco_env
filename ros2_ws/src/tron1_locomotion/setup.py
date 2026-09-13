from setuptools import find_packages, setup

#. By Teddy_K >>>>
import os
from glob import glob
#. >>>> By Teddy_K

package_name = 'tron1_locomotion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        #. By Teddy_K >>>>
        # YAML 설정 파일들을 install 디렉토리로 설치 등록
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        #. >>>> By Teddy_K
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Teddy_K',
    maintainer_email='cencyan35@nate.com',
    description='ROS2 FSM Locomotion and Docking Controller Node for Tron1',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # tron1_controller.py의 main() 진입점 매핑
            'tron1_controller = tron1_locomotion.tron1_controller:main',
        ],
    },
)
