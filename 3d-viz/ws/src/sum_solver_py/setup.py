from setuptools import setup
package_name = 'sum_solver_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kevin_KyawThiha',
    maintainer_email='k.thiha10.mail@gmail.com',
    description='Two-sum example',
    license='Apache-2.0',
    entry_points={'console_scripts': ['two_sum = sum_solver_py.cli:main']},
)
