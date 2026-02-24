from setuptools import setup, find_packages

setup(
    name="slvrov-tools",
    version="0.1.0",
    packages=find_packages(where='slvrov_tools/src'),
    package_dir={'': 'slvrov_tools/src'},
    package_data={
        "slvrov_tools": ["*.so"],
    },
    install_requires=[
        'setuptools',
        'numpy',
        'opencv'
        ],
    python_requires=">=3.10",
    zip_safe=True,
)
