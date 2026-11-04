// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Polygons;

/// <summary>
/// Interference computation between polygons, lines and polyhedra with only triangular facets (polygonal
/// representations of curves and triangulated representations of surfaces): the package's statics.
/// </summary>
internal static class Intf
{
    /// <summary>The plane equation of the triangle <paramref name="p1"/> <paramref name="p2"/> <paramref name="p3"/>.</summary>
    public static void PlaneEquation(in Point3D p1,
                                     in Point3D p2,
                                     in Point3D p3,
                                     out Xyz normalVector,
                                     out double polarDistance)
    {
        Xyz v1 = p2.XYZ() - p1.XYZ();
        Xyz v2 = p3.XYZ() - p2.XYZ();
        Xyz v3 = p1.XYZ() - p3.XYZ();
        normalVector = (v1 ^ v2) + (v2 ^ v3) + (v3 ^ v1);
        double normLen = normalVector.Modulus();
        if (normLen < Constants.Resolution())
        {
            polarDistance = 0.0;
        }
        else
        {
            normalVector.Divide(normLen);
            polarDistance = normalVector * p1.XYZ();
        }
    }

    /// <summary>True if the triangle <paramref name="p1"/> <paramref name="p2"/> <paramref name="p3"/> contains <paramref name="thePnt"/>.</summary>
    public static bool Contain(in Point3D p1, in Point3D p2, in Point3D p3, in Point3D thePnt)
    {
        Xyz v1 = (p2.XYZ() - p1.XYZ()) ^ (thePnt.XYZ() - p1.XYZ());
        Xyz v2 = (p3.XYZ() - p2.XYZ()) ^ (thePnt.XYZ() - p2.XYZ());
        Xyz v3 = (p1.XYZ() - p3.XYZ()) ^ (thePnt.XYZ() - p3.XYZ());
        return v1 * v2 >= 0.0 && v2 * v3 >= 0.0 && v3 * v1 >= 0.0;
    }
}
