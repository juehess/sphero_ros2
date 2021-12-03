from setuptools import setup

package_name = 'sphero_node'
transformations = "sphero_node/transformations"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, transformations],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'sphero = sphero_node.sphero_node:main',
            'sphero2 = sphero_node.sphero_node2:main',
        ],
    },
)
