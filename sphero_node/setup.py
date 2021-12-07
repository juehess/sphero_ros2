from setuptools import setup
import os
from glob import glob
package_name = 'sphero_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juergen Hess',
    maintainer_email='juergenmhess@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sphero = sphero_node.sphero_node_pysphero:main',
            'sphero_tf_pub = sphero_node.odom_tf_pub:main',
        ],
    },
)
