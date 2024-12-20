from setuptools import setup

package_name = 'extras'

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
    maintainer='sedrica',
    maintainer_email='sedrica@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'convertor = extras.convertor:main',
            'direction = extras.direction:main',
            'frametf = extras.frametf:main',
            'tests = extras.tests:main',
            'cutter =extras.cutter:main',
            'alpp =extras.alpp:main'
        ],
    },
)
