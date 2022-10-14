from setuptools import setup, find_packages


def parse_requirements(fname_list=[]):
    """Parse the package dependencies listed in a requirements list file."""
    requirements = []
    for fname in fname_list:
        with open(fname) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#"):
                    requirements.append(line)
    # remove duplicates
    requirements = list(set(requirements))
    return requirements


# basic
setup(
    # 描述信息
    name="wadda",
    version="0.0.4",
    description="Wind's Autonomous Driving Development Art",
    author="windzu",
    author_email="windzu1@gmail.com",
    url="https://github.com/windzu/wadda",
    license="MIT license",
    keywords="adas deeplearning",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Natural Language :: English",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
    ],
    # 主要设置
    python_requires=">=3.6",
    packages=find_packages(exclude=("docs")),
    install_requires=parse_requirements(
        [
            "requirements/requirements.txt",
            "requirements/pypcd.txt",
        ]
    ),
    entry_points={"console_scripts": ["wadda=wadda.main:main"]},
    # 次要设置
    include_package_data=True,
    zip_safe=False,
)
