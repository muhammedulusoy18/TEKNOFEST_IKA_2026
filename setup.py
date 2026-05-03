import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'teknofest_ika'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch ve Config klasörlerini sisteme dahil eder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yusuf ve Muhammed Ulusoy',
    maintainer_email='yusuf@todo.todo',
    description='TEKNOFEST 2026 İnsansız Kara Aracı (İKA) ROS 2 Ana Paketi',
    license='Apache-2.0',

    # İŞTE BÜTÜN FİLONUN HARİTASI BURADA:
    entry_points={
        'console_scripts': [
            # 1. BEYİN: Tüm kararları veren ana merkez (vehicle_manager'ın ROS hali)
            'brain_node = teknofest_ika.nodes.brain_node:main',

            # 2. GÖZLER: Kamerayı okuyup OpenCV/Tesseract işleyen node (perception_node)
            'perception_node = teknofest_ika.nodes.perception_node:main',

            # 3. KASLAR: Beyinden gelen komutları PID ile motorlara ileten node (motor_node)
            'motor_node = teknofest_ika.nodes.motor_node:main',

            # 4. DUYULAR: IMU (Denge) ve UART sensör verilerini ROS ağına basan node
            'sensor_node = teknofest_ika.nodes.sensor_node:main',

            # 5. YER İSTASYONU: Hatice'nin yazacağı arayüzü (GUI) ROS ağına bağlayan node
            'gui_node = teknofest_ika.nodes.gui_node:main',
        ],
    },
)