// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Interference computation between polygons, lines and polyhedra with only triangular facets (polygonal
/// representations of curves and triangulated representations of surfaces): the package's statics.
/// </summary>
public static class Intf
{
    /// <summary>The plane equation of the triangle <paramref name="P1"/> <paramref name="P2"/> <paramref name="P3"/>.</summary>
    public static void PlaneEquation(in Point3D P1,
                                     in Point3D P2,
                                     in Point3D P3,
                                     out Xyz NormalVector,
                                     out double PolarDistance)
    {
        Xyz v1 = P2.XYZ() - P1.XYZ();
        Xyz v2 = P3.XYZ() - P2.XYZ();
        Xyz v3 = P1.XYZ() - P3.XYZ();
        NormalVector = (v1 ^ v2) + (v2 ^ v3) + (v3 ^ v1);
        double aNormLen = NormalVector.Modulus();
        if (aNormLen < Constants.Resolution())
        {
            PolarDistance = 0.0;
        }
        else
        {
            NormalVector.Divide(aNormLen);
            PolarDistance = NormalVector * P1.XYZ();
        }
    }

    /// <summary>True if the triangle <paramref name="P1"/> <paramref name="P2"/> <paramref name="P3"/> contains <paramref name="ThePnt"/>.</summary>
    public static bool Contain(in Point3D P1, in Point3D P2, in Point3D P3, in Point3D ThePnt)
    {
        Xyz v1 = (P2.XYZ() - P1.XYZ()) ^ (ThePnt.XYZ() - P1.XYZ());
        Xyz v2 = (P3.XYZ() - P2.XYZ()) ^ (ThePnt.XYZ() - P2.XYZ());
        Xyz v3 = (P1.XYZ() - P3.XYZ()) ^ (ThePnt.XYZ() - P3.XYZ());
        return v1 * v2 >= 0.0 && v2 * v3 >= 0.0 && v3 * v1 >= 0.0;
    }
}
