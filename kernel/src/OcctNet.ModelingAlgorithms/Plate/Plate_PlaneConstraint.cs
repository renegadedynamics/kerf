// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_PlaneConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_PlaneConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_PlaneConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A constraint that the plate's deformation at a 2D point (or its derivative) lies in a plane.</summary>
/// <remarks>A class: OCCT's copies share the linear constraint's arrays (see <see cref="Plate_LinearScalarConstraint"/>).</remarks>
public class Plate_PlaneConstraint
{
    private readonly Plate_LinearScalarConstraint myLSC;

    public Plate_PlaneConstraint(in gp_XY point2d, in gp_Pln pln, int iu = 0, int iv = 0)
    {
        myLSC = new Plate_LinearScalarConstraint(1, 1);
        gp_XYZ point = pln.Location().XYZ();
        myLSC.SetPPC(1, new Plate_PinpointConstraint(point2d, point, iu, iv));
        gp_XYZ dir = pln.Axis().Direction().XYZ();
        dir.Normalize();
        myLSC.SetCoeff(1, 1, dir);
    }

    public Plate_LinearScalarConstraint LSC() => myLSC;
}
