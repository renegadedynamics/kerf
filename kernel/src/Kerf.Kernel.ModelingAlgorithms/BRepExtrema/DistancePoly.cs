// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_Poly.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_Poly.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>The minimum distance between the triangulation nodes of the faces of two shapes.</summary>
public static class DistancePoly
{
    /// <summary>
    /// The nearest pair of triangulation nodes of the faces of <paramref name="s1"/> and <paramref name="s2"/>; false
    /// if either has no triangulated face. <paramref name="p1"/> and <paramref name="p2"/> are left unchanged when the
    /// method fails, as in OCCT.
    /// </summary>
    public static bool Distance(in Shape s1, in Shape s2, ref Point3D p1, ref Point3D p2, out double dist)
    {
        dist = global::Kerf.Engine.Foundation.Precision.Infinite();
        Placement l;
        Triangulation? tr;
        var exFace = new Explorer();

        int nbn1 = 0;
        for (exFace.Init(s1, ShapeType.Face); exFace.More(); exFace.Next())
        {
            Face f = ShapeCasts.Face(exFace.Current());
            tr = Tool.Triangulation(f, out l);
            if (tr is not null)
            {
                nbn1 += tr.NodeCount;
            }
        }
        if (nbn1 == 0)
        {
            return false;
        }

        int nbn2 = 0;
        for (exFace.Init(s2, ShapeType.Face); exFace.More(); exFace.Next())
        {
            Face f = ShapeCasts.Face(exFace.Current());
            tr = Tool.Triangulation(f, out l);
            if (tr is not null)
            {
                nbn2 += tr.NodeCount;
            }
        }
        if (nbn2 == 0)
        {
            return false;
        }

        int i, n;
        var tp1 = new NCollection_Array1<Point3D>(1, nbn1);
        nbn1 = 0;
        for (exFace.Init(s1, ShapeType.Face); exFace.More(); exFace.Next())
        {
            Face f = ShapeCasts.Face(exFace.Current());
            tr = Tool.Triangulation(f, out l);
            if (tr is not null)
            {
                Transform3D trsf = l.Transformation;
                n = tr.NodeCount;
                for (i = 1; i <= n; i++)
                {
                    nbn1++;
                    tp1.SetValue(nbn1, tr.Node(i).Transformed(trsf));
                }
            }
        }

        var tp2 = new NCollection_Array1<Point3D>(1, nbn2);
        nbn2 = 0;
        for (exFace.Init(s2, ShapeType.Face); exFace.More(); exFace.Next())
        {
            Face f = ShapeCasts.Face(exFace.Current());
            tr = Tool.Triangulation(f, out l);
            if (tr is not null)
            {
                Transform3D trsf = l.Transformation;
                n = tr.NodeCount;
                for (i = 1; i <= n; i++)
                {
                    nbn2++;
                    tp2.SetValue(nbn2, tr.Node(i).Transformed(trsf));
                }
            }
        }

        int i1, i2;
        for (i1 = 1; i1 <= nbn1; i1++)
        {
            Point3D pp1 = tp1[i1];
            for (i2 = 1; i2 <= nbn2; i2++)
            {
                Point3D pp2 = tp2[i2];
                double dCur = pp1.DistanceTo(pp2);
                if (dist > dCur)
                {
                    p1 = pp1;
                    p2 = pp2;
                    dist = dCur;
                }
            }
        }
        return true;
    }
}
