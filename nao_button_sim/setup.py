from setuptools import setup

package_name = 'nao_button_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ijnek',
    maintainer_email='kenjibrameld@gmail.com',
    description='Allows simulating button presses through command line interface',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nao_button_sim = nao_button_sim.nao_button_sim:main'
        ],
    },
)
