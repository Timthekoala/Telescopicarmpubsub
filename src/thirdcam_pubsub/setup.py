from setuptools import setup

package_name = 'thirdcam_pubsub'

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
    maintainer='timothy',
    maintainer_email='timothy_lim@htx.gov.sg',
    description='Pub and Sub for keyboard control of telescopic arm',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_publisher = thirdcam_pubsub.thirdcampub:main',
            'my_subscriber = thirdcam_pubsub.thirdcamsub:main',
        ],
    },
)
