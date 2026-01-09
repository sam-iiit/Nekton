from setuptools import setup

package_name = 'onboard_forward_cam'

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
    description='Computer Vision processing for the nekton forward camera',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'onboard_forward_cam = onboard_forward_cam.onboard_forward_cam:main'
        ],
    },
)
