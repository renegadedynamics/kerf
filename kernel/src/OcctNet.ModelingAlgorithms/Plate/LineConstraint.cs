// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LineConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LineConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LineConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>A constraint that the plate's deformation at a 2D point (or its derivative) lies on a line.</summary>
/// <remarks>A class: OCCT's copies share the linear constraint's arrays (see <see cref="LinearScalarConstraint"/>).</remarks>
public class LineConstraint
{
    private readonly LinearScalarConstraint _lsc;

    public LineConstraint(in Xy point2d, in Line3D lin, int iu = 0, int iv = 0)
    {
        _lsc = new LinearScalarConstraint(2, 1);
        Xyz point = lin.Location().XYZ();
        _lsc.SetPPC(1, new PinpointConstraint(point2d, point, iu, iv));
        Xyz dir = lin.Direction().XYZ();
        // one builds two directions orthogonal to dir
        Xyz dX = new(1, 0, 0);
        Xyz dY = new(0, 1, 0);

        Xyz d1 = dX ^ dir;
        Xyz d2 = dY ^ dir;
        if (d2.SquareModulus() > d1.SquareModulus())
        {
            d1 = d2;
        }
        d1.Normalize();
        d2 = dir ^ d1;
        d2.Normalize();
        _lsc.SetCoeff(1, 1, d1);
        _lsc.SetCoeff(2, 1, d2);
    }

    public LinearScalarConstraint LSC() => _lsc;
}
