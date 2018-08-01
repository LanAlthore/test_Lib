import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="proDVKlib",
    version="1.0.0",
    author="Alan Waite",
    author_email="Alan.Waite@emmicro-us.com",
    description="The library for working with the proDVK",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/LanAlthore/test_Lib",
    packages=setuptools.find_packages(),
    classifiers=(
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ),
)
