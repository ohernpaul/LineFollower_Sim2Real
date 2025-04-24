from setuptools import find_packages, setup

package_name = 'linepreprocessor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bah',
    maintainer_email='ohernpaul@gmail.com',
    description='Package for extracting the information from an image containing a line for a line following robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'line_center_finder = linepreprocessor.line_center_finder:main'
        ],
    },
)
