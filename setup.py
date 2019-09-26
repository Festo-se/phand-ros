# coding: utf-8

import sys
from setuptools import setup, find_packages

NAME = "phand_rest_api"
VERSION = "1.0.0"
REQUIRES = [""]
setup(
    name=NAME,
    version=VERSION,
    description="BionicSoftHandAPI",
    author_email="timo.schwarzer@festo.com",
    url="",
    keywords=["Swagger", "BionicSoftHandAPI"],
    install_requires=REQUIRES,
    packages=find_packages(),
    package_data={'': ['phand_rest_api/swagger.yaml']},
    include_package_data=True,
    entry_points={
        'console_scripts': ['phand_rest_api=phand_rest_api.app']},
    long_description="""\
    The REST Api for the BionicSoftHand
    """
)
