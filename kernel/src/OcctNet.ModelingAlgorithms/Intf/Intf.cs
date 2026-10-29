// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Interference computation between polygons, lines and polyhedra with only triangular facets (polygonal
/// representations of curves and triangulated representations of surfaces): the package's statics.
/// </summary>
public static class Intf
{
    /// <summary>The plane equation of the triangle <paramref name="P1"/> <paramref name="P2"/> <paramref name="P3"/>.</summary>
    public static void PlaneEquation(in gp_Pnt P1,
                                     in gp_Pnt P2,
                                     in gp_Pnt P3,
                                     out gp_XYZ NormalVector,
                                     out double PolarDistance)
    {
        gp_XYZ v1 = P2.XYZ() - P1.XYZ();
        gp_XYZ v2 = P3.XYZ() - P2.XYZ();
        gp_XYZ v3 = P1.XYZ() - P3.XYZ();
        NormalVector = (v1 ^ v2) + (v2 ^ v3) + (v3 ^ v1);
        double aNormLen = NormalVector.Modulus();
        if (aNormLen < gp.Resolution())
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
    public static bool Contain(in gp_Pnt P1, in gp_Pnt P2, in gp_Pnt P3, in gp_Pnt ThePnt)
    {
        gp_XYZ v1 = (P2.XYZ() - P1.XYZ()) ^ (ThePnt.XYZ() - P1.XYZ());
        gp_XYZ v2 = (P3.XYZ() - P2.XYZ()) ^ (ThePnt.XYZ() - P2.XYZ());
        gp_XYZ v3 = (P1.XYZ() - P3.XYZ()) ^ (ThePnt.XYZ() - P3.XYZ());
        return v1 * v2 >= 0.0 && v2 * v3 >= 0.0 && v3 * v1 >= 0.0;
    }
}
