import pytest

from graduation_project.config import AircraftParams, load_aircraft

AIRCRAFT = ["research_quad", "m400_reference_quad"]


@pytest.fixture(params=AIRCRAFT)
def aircraft(request: pytest.FixtureRequest) -> AircraftParams:
    return load_aircraft(request.param)


@pytest.fixture
def research_quad() -> AircraftParams:
    return load_aircraft("research_quad")
