import os, sys

from distutils.core import setup, Extension
from distutils import sysconfig

cpp_args = ['-std=c++11', '-I', '/home/shzhou2/project/eigen-eigen-323c052e1731']

ext_modules = [
    Extension(
    'wrap',
        ['kalman.cpp', 'wrap.cpp'],
        include_dirs=['pybind11/include'],
    language='c++',
    extra_compile_args = cpp_args,
    ),
]

setup(
    name='wrap',
    version='0.0.1',
    description='Example',
    ext_modules=ext_modules,
)