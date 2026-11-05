# Reference results from Python build123d

The scripts here run the original Python build123d and save its results as JSON. The .NET tests
recompute the same cases and compare. The JSON is committed, so running the tests doesn't need Python.

| Script | Output | C# test |
|---|---|---|
| `geometry_reference.py` | `Reference/geometry.json` | `Geometry/ReferenceTests.cs`: positions, directions and angles to about 1e-12 |
| `topology_reference.py` | `Reference/topology.json` | `Topology/TopologyReferenceTests.cs`: each named shape's type, topology counts, validity, volume, area, bounding box and centre |

## Setting up the reference environment

The environment uses OCP 8.0, the same OCCT version as the port. It runs build123d from a sibling
checkout at `../build123d` without installing it.

```sh
python -m venv .deps/py
.deps/py/Scripts/python -m pip install "cadquery-ocp-novtk>=8.0,<8.1" numpy scipy sympy svgpathtools \
    anytree ezdxf fonttools ipython "ocpsvg>=0.7,<0.8" "ocp_gordon>=0.3.1,<0.4" trianglesolver \
    "webcolors~=24.8.0" requests lib3mf "bd_materials>=0.2,<0.3" "threejs-materials>=1.2.1,<1.3"
```

## Regenerating

Set `PYTHONDONTWRITEBYTECODE=1` so nothing is written into the build123d checkout:

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=../build123d/src .deps/py/Scripts/python -W ignore \
    tests/reference/geometry_reference.py tests/Kerf.Modeling.Tests/Reference/geometry.json
```

Inputs come from fixed seeds, so the output changes only when build123d's results do.
