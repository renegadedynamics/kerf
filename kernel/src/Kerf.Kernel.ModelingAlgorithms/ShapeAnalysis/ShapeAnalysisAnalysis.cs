// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>Tools for analysing shapes and their geometry: periodic adjustment, wire bounds and orientation, outer wires.</summary>
public static class ShapeAnalysisAnalysis
{
    /// <summary>
    /// Returns a shift required to move point <paramref name="val"/> to the range [ToVal-Period/2, ToVal+Period/2].
    /// The shift is a multiple of the period; 0 if the point is already in range.
    /// </summary>
    public static double AdjustByPeriod(double val, double toVal, double period)
    {
        double diff = val - toVal;
        double d    = Math.Abs(diff);
        double p    = Math.Abs(period);
        if (d <= 0.5 * p)
        {
            return 0.0;
        }

        if (p < 1e-100)
        {
            return diff;
        }

        return (diff > 0 ? -p : p) * Math.Floor(d / p + 0.5);
    }

    /// <summary>Returns a shift required to move point <paramref name="val"/> to the range [ValMin, ValMax].</summary>
    public static double AdjustToPeriod(double val, double valMin, double valMax)
    {
        return AdjustByPeriod(val, 0.5 * (valMin + valMax), valMax - valMin);
    }

    /// <summary>
    /// Finds the start and end vertices of a shape: of a wire (TopExp.Vertices), of an edge (with its orientation),
    /// or of a vertex (the vertex twice). Null vertices for other shapes.
    /// </summary>
    public static void FindBounds(in Shape shape, out Vertex v1, out Vertex v2)
    {
        v1 = new Vertex();
        v2 = new Vertex();
        EdgeAnalysis ea = new EdgeAnalysis();
        if (shape.Type == ShapeType.Wire)
        {
            Wire w = ShapeCasts.Wire(shape);
            // invalid work with reversed wires replaced on TopExp
            Exploration.Vertices(w, out v1, out v2);
        }
        else if (shape.Type == ShapeType.Edge)
        {
            v1 = ea.FirstVertex(ShapeCasts.Edge(shape));
            v2 = ea.LastVertex(ShapeCasts.Edge(shape));
        }
        else if (shape.Type == ShapeType.Vertex)
        {
            v1 = v2 = ShapeCasts.Vertex(shape);
        }
    }

    /// <summary>Returns the total signed area of the 2d wire on the face: positive for an outer (counter-clockwise) contour.</summary>
    public static double TotCross2D(ExtendWireData sewd, in Face aFace)
    {
        int      i, nbc = 0;
        Point2D fuv = new Point2D(), luv, uv0 = new Point2D();
        double   totcross = 0;
        for (i = 1; i <= sewd.EdgeCount; i++)
        {
            Edge   edge = sewd.Edge(i);
            Curve2D? c2d  = Tool.CurveOnSurface(edge, aFace, out double f2d, out double l2d);
            if (c2d is not null)
            {
                nbc++;
                NCollection_Sequence<Point2D> seqPnt = new NCollection_Sequence<Point2D>();
                CurveAnalysis.GetSamplePoints(c2d, f2d, l2d, seqPnt);
                if (edge.Orientation == Reversed)
                {
                    seqPnt.Reverse();
                }

                if (nbc == 1)
                {
                    fuv = seqPnt.Value(1);
                    uv0 = fuv;
                }

                int j = 1;
                for (; j <= seqPnt.Length(); j++)
                {
                    luv = seqPnt.Value(j);
                    totcross += (fuv.X - luv.X) * (fuv.Y + luv.Y) / 2;
                    fuv = luv;
                }
            }
        }

        totcross += (fuv.X - uv0.X) * (fuv.Y + uv0.Y) / 2;
        return totcross;
    }

    /// <summary>Returns the total area of the 3d wire, from sample points of its 3d curves.</summary>
    public static double ContourArea(in Wire wire)
    {
        int    nbc = 0;
        Point3D fuv = new Point3D(), luv, uv0 = new Point3D();
        Xyz total = new Xyz(0.0, 0.0, 0.0);
        Iterator ite = new Iterator(wire, false);
        for (; ite.More(); ite.Next())
        {
            Edge edge = ShapeCasts.Edge(ite.Value());
            Curve3D? c3d  = Tool.Curve(edge, out double first, out double last);
            if (c3d is not null)
            {
                NCollection_Sequence<Point3D> seqPnt = new NCollection_Sequence<Point3D>();
                if (!CurveAnalysis.GetSamplePoints(c3d, first, last, seqPnt))
                {
                    continue;
                }

                nbc++;
                if (edge.Orientation == Reversed)
                {
                    seqPnt.Reverse();
                }

                if (nbc == 1)
                {
                    fuv = seqPnt.Value(1);
                    uv0 = fuv;
                }

                int j = 1;
                for (; j <= seqPnt.Length(); j++)
                {
                    luv = seqPnt.Value(j);
                    total += luv.XYZ() ^ fuv.XYZ();
                    fuv = luv;
                }
            }
        }

        total += uv0.XYZ() ^ fuv.XYZ();
        double area = total.Modulus() * 0.5;
        return area;
    }

    /// <summary>Returns true if the face has an outer bound.</summary>
    public static bool IsOuterBound(in Face face)
    {
        Face f = face;
        Wire w = new Wire();
        f.Orient(Forward);
        int nbw = 0;
        for (Explorer exp = new Explorer(f, ShapeType.Wire); exp.More(); exp.Next())
        {
            w = ShapeCasts.Wire(exp.Current());
            nbw++;
        }

        // skl 08.04.2002
        if (nbw == 1)
        {
            ExtendWireData sewd = new ExtendWireData(w);
            double totcross = TotCross2D(sewd, f);
            return (totcross >= 0);
        }
        else
        {
            FaceSurfaceAdaptor ads = new FaceSurfaceAdaptor(f, false);
            double tol   = Tool.Tolerance(f);
            double toluv = Min(ads.UResolution(tol), ads.VResolution(tol));
            FClass2d fcl = new FClass2d(f, toluv);
            bool rescl = (fcl.PerformInfinitePoint() == Out);
            return rescl;
        }
    }

    /// <summary>
    /// Returns the positively oriented wire of the face (its outer wire); if there is none, the last wire of the face.
    /// A null wire for a face without wires.
    /// </summary>
    public static Wire OuterWire(in Face face)
    {
        Face f = face;
        f.Orient(Forward);

        Explorer it = new Explorer(f, ShapeType.Wire);
        while (it.More())
        {
            Wire wire = ShapeCasts.Wire(it.Value());
            it.Next();

            // if current wire is the last one, return it without analysis
            if (!it.More())
            {
                return wire;
            }

            // Check if the wire has positive area
            ExtendWireData sewd = new ExtendWireData(wire);
            double area2d = ShapeAnalysisAnalysis.TotCross2D(sewd, f);
            if (area2d >= 0.0)
            {
                return wire;
            }
        }

        return new Wire();
    }

    /// <summary>
    /// Computes the exact UV bounds of the face from the pcurves of its edges, or the surface's bounds for a face without
    /// edges.
    /// </summary>
    public static void GetFaceUVBounds(in Face F, out double uMin, out double uMax, out double vMin, out double vMax)
    {
        Face ff = F;
        ff.Orient(Forward);
        Explorer ex = new Explorer(ff, ShapeType.Edge);
        if (!ex.More())
        {
            Tool.Surface(F, out _)!.ParameterBounds(out uMin, out uMax, out vMin, out vMax);
            return;
        }

        Box2d           b   = new Box2d();
        EdgeAnalysis  sae = new EdgeAnalysis();
        CurveAnalysis sac = new CurveAnalysis();
        for (; ex.More(); ex.Next())
        {
            Edge edge = ShapeCasts.Edge(ex.Current());
            if (!sae.PCurve(edge, F, out Curve2D? c2d, out double f, out double l, false))
            {
                continue;
            }

            sac.FillBndBox(c2d!, f, l, 20, true, ref b);
        }

        b.Get(out uMin, out vMin, out uMax, out vMax);
    }
}
