import distutils.core
import Cython.Build
import numpy 

distutils.core.setup(
    ext_modules = Cython.Build.cythonize("radar4D2xyz_complex_cython.pyx"),
    include_dirs=[numpy.get_include()]
)