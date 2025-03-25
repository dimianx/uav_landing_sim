from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['drone', 'uwb_interpolation', 'env_disturbances'],
    package_dir={'': 'src'})

setup(**setup_args)