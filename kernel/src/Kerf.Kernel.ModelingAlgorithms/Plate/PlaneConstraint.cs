// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_PlaneConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_PlaneConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_PlaneConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>A constraint that the plate's deformation at a 2D point (or its derivative) lies in a plane.</summary>
/// <remarks>A class: OCCT's copies share the linear constraint's arrays (see <see cref="LinearScalarConstraint"/>).</remarks>
internal class PlaneConstraint
{
    private readonly LinearScalarConstraint _lsc;

    public PlaneConstraint(in Xy point2d, in Plane3D pln, int iu = 0, int iv = 0)
    {
        _lsc = new LinearScalarConstraint(1, 1);
        Xyz point = pln.Location.XYZ();
        _lsc.SetPPC(1, new PinpointConstraint(point2d, point, iu, iv));
        Xyz dir = pln.Axis.Direction.XYZ();
        dir.Normalize();
        _lsc.SetCoeff(1, 1, dir);
    }

    public LinearScalarConstraint LSC() => _lsc;
}
