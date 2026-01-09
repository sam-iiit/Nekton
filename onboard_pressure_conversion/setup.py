from setuptools import setup

package_name = 'onboard_pressure_conversion'

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
    maintainer='Matthew Woodward',
    maintainer_email='mwoodward@gatech.edu',
    description='Pressure to depth conversion node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'onboard_pressure_conversion = onboard_pressure_conversion.onboard_pressure_conversion:main'
        ],
    },
)
