// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_Poly.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_Poly.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>The minimum distance between the triangulation nodes of the faces of two shapes.</summary>
public static class BRepExtrema_Poly
{
    /// <summary>
    /// The nearest pair of triangulation nodes of the faces of <paramref name="S1"/> and <paramref name="S2"/>; false
    /// if either has no triangulated face. <paramref name="P1"/> and <paramref name="P2"/> are left unchanged when the
    /// method fails, as in OCCT.
    /// </summary>
    public static bool Distance(in Shape S1, in Shape S2, ref Point3D P1, ref Point3D P2, out double dist)
    {
        dist = global::Kerf.Engine.Foundation.Precision.Infinite();
        Placement L;
        Triangulation? Tr;
        var exFace = new Explorer();

        int nbn1 = 0;
        for (exFace.Init(S1, ShapeType.Face); exFace.More(); exFace.Next())
        {
            Face F = ShapeCasts.Face(exFace.Current());
            Tr = Tool.Triangulation(F, out L);
            if (Tr is not null)
            {
                nbn1 += Tr.NodeCount;
            }
        }
        if (nbn1 == 0)
        {
            return false;
        }

        int nbn2 = 0;
        for (exFace.Init(S2, ShapeType.Face); exFace.More(); exFace.Next())
        {
            Face F = ShapeCasts.Face(exFace.Current());
            Tr = Tool.Triangulation(F, out L);
            if (Tr is not null)
            {
                nbn2 += Tr.NodeCount;
            }
        }
        if (nbn2 == 0)
        {
            return false;
        }

        int i, n;
        var TP1 = new NCollection_Array1<Point3D>(1, nbn1);
        nbn1 = 0;
        for (exFace.Init(S1, ShapeType.Face); exFace.More(); exFace.Next())
        {
            Face F = ShapeCasts.Face(exFace.Current());
            Tr = Tool.Triangulation(F, out L);
            if (Tr is not null)
            {
                Transform3D aTrsf = L.Transformation();
                n = Tr.NodeCount;
                for (i = 1; i <= n; i++)
                {
                    nbn1++;
                    TP1.SetValue(nbn1, Tr.Node(i).Transformed(aTrsf));
                }
            }
        }

        var TP2 = new NCollection_Array1<Point3D>(1, nbn2);
        nbn2 = 0;
        for (exFace.Init(S2, ShapeType.Face); exFace.More(); exFace.Next())
        {
            Face F = ShapeCasts.Face(exFace.Current());
            Tr = Tool.Triangulation(F, out L);
            if (Tr is not null)
            {
                Transform3D aTrsf = L.Transformation();
                n = Tr.NodeCount;
                for (i = 1; i <= n; i++)
                {
                    nbn2++;
                    TP2.SetValue(nbn2, Tr.Node(i).Transformed(aTrsf));
                }
            }
        }

        int i1, i2;
        for (i1 = 1; i1 <= nbn1; i1++)
        {
            Point3D PP1 = TP1[i1];
            for (i2 = 1; i2 <= nbn2; i2++)
            {
                Point3D PP2 = TP2[i2];
                double dCur = PP1.Distance(PP2);
                if (dist > dCur)
                {
                    P1 = PP1;
                    P2 = PP2;
                    dist = dCur;
                }
            }
        }
        return true;
    }
}
