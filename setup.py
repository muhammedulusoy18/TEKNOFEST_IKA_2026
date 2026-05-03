from setuptools import setup, find_packages

package_name = 'teknofest_ika'

setup(
    name=package_name,
    version='0.0.0',
    # find_packages sayesinde core, modules ve utils otomatik olarak dahil edilecek
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yusuf',
    description='TEKNOFEST IKA 2026 Robot Kontrol Paketi',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'beyin = teknofest_ika.run_ika:main',
        ],
    },
)
