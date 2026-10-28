"""Milestone 0 smoke test: package imports and pinned dependency versions are present."""

import numpy
import scipy
import yaml

import graduation_project


def test_package_imports() -> None:
    assert graduation_project.__version__ == "0.1.0"


def test_dependency_versions_match_pins() -> None:
    assert numpy.__version__ == "1.26.4"
    assert scipy.__version__ == "1.11.4"
    assert yaml.__version__ == "6.0.1"
