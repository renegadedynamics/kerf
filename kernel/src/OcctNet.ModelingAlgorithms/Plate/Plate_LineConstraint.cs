// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LineConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LineConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LineConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A constraint that the plate's deformation at a 2D point (or its derivative) lies on a line.</summary>
/// <remarks>A class: OCCT's copies share the linear constraint's arrays (see <see cref="Plate_LinearScalarConstraint"/>).</remarks>
public class Plate_LineConstraint
{
    private readonly Plate_LinearScalarConstraint myLSC;

    public Plate_LineConstraint(in gp_XY point2d, in gp_Lin lin, int iu = 0, int iv = 0)
    {
        myLSC = new Plate_LinearScalarConstraint(2, 1);
        gp_XYZ point = lin.Location().XYZ();
        myLSC.SetPPC(1, new Plate_PinpointConstraint(point2d, point, iu, iv));
        gp_XYZ dir = lin.Direction().XYZ();
        // one builds two directions orthogonal to dir
        gp_XYZ dX = new(1, 0, 0);
        gp_XYZ dY = new(0, 1, 0);

        gp_XYZ d1 = dX ^ dir;
        gp_XYZ d2 = dY ^ dir;
        if (d2.SquareModulus() > d1.SquareModulus())
        {
            d1 = d2;
        }
        d1.Normalize();
        d2 = dir ^ d1;
        d2.Normalize();
        myLSC.SetCoeff(1, 1, d1);
        myLSC.SetCoeff(2, 1, d2);
    }

    public Plate_LinearScalarConstraint LSC() => myLSC;
}
