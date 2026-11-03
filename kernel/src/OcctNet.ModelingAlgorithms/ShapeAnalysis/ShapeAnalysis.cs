// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>Tools for analysing shapes and their geometry: periodic adjustment, wire bounds and orientation, outer wires.</summary>
public static class ShapeAnalysis
{
    /// <summary>
    /// Returns a shift required to move point <paramref name="Val"/> to the range [ToVal-Period/2, ToVal+Period/2].
    /// The shift is a multiple of the period; 0 if the point is already in range.
    /// </summary>
    public static double AdjustByPeriod(double Val, double ToVal, double Period)
    {
        double diff = Val - ToVal;
        double D    = Math.Abs(diff);
        double P    = Math.Abs(Period);
        if (D <= 0.5 * P)
        {
            return 0.0;
        }

        if (P < 1e-100)
        {
            return diff;
        }

        return (diff > 0 ? -P : P) * Math.Floor(D / P + 0.5);
    }

    /// <summary>Returns a shift required to move point <paramref name="Val"/> to the range [ValMin, ValMax].</summary>
    public static double AdjustToPeriod(double Val, double ValMin, double ValMax)
    {
        return AdjustByPeriod(Val, 0.5 * (ValMin + ValMax), ValMax - ValMin);
    }

    /// <summary>
    /// Finds the start and end vertices of a shape: of a wire (TopExp.Vertices), of an edge (with its orientation),
    /// or of a vertex (the vertex twice). Null vertices for other shapes.
    /// </summary>
    public static void FindBounds(in TopoDS_Shape shape, out TopoDS_Vertex V1, out TopoDS_Vertex V2)
    {
        V1 = new TopoDS_Vertex();
        V2 = new TopoDS_Vertex();
        ShapeAnalysis_Edge EA = new ShapeAnalysis_Edge();
        if (shape.ShapeType() == TopAbs_WIRE)
        {
            TopoDS_Wire W = TopoDS.Wire(shape);
            // invalid work with reversed wires replaced on TopExp
            TopExp.Vertices(W, out V1, out V2);
        }
        else if (shape.ShapeType() == TopAbs_EDGE)
        {
            V1 = EA.FirstVertex(TopoDS.Edge(shape));
            V2 = EA.LastVertex(TopoDS.Edge(shape));
        }
        else if (shape.ShapeType() == TopAbs_VERTEX)
        {
            V1 = V2 = TopoDS.Vertex(shape);
        }
    }

    /// <summary>Returns the total signed area of the 2d wire on the face: positive for an outer (counter-clockwise) contour.</summary>
    public static double TotCross2D(ShapeExtend_WireData sewd, in TopoDS_Face aFace)
    {
        int      i, nbc = 0;
        Point2D fuv = new Point2D(), luv, uv0 = new Point2D();
        double   totcross = 0;
        for (i = 1; i <= sewd.NbEdges(); i++)
        {
            TopoDS_Edge   edge = sewd.Edge(i);
            Geom2d_Curve? c2d  = BRep_Tool.CurveOnSurface(edge, aFace, out double f2d, out double l2d);
            if (c2d is not null)
            {
                nbc++;
                NCollection_Sequence<Point2D> SeqPnt = new NCollection_Sequence<Point2D>();
                ShapeAnalysis_Curve.GetSamplePoints(c2d, f2d, l2d, SeqPnt);
                if (edge.Orientation() == TopAbs_REVERSED)
                {
                    SeqPnt.Reverse();
                }

                if (nbc == 1)
                {
                    fuv = SeqPnt.Value(1);
                    uv0 = fuv;
                }

                int j = 1;
                for (; j <= SeqPnt.Length(); j++)
                {
                    luv = SeqPnt.Value(j);
                    totcross += (fuv.X() - luv.X()) * (fuv.Y() + luv.Y()) / 2;
                    fuv = luv;
                }
            }
        }

        totcross += (fuv.X() - uv0.X()) * (fuv.Y() + uv0.Y()) / 2;
        return totcross;
    }

    /// <summary>Returns the total area of the 3d wire, from sample points of its 3d curves.</summary>
    public static double ContourArea(in TopoDS_Wire theWire)
    {
        int    nbc = 0;
        Point3D fuv = new Point3D(), luv, uv0 = new Point3D();
        Xyz aTotal = new Xyz(0.0, 0.0, 0.0);
        TopoDS_Iterator aIte = new TopoDS_Iterator(theWire, false);
        for (; aIte.More(); aIte.Next())
        {
            TopoDS_Edge edge = TopoDS.Edge(aIte.Value());
            Geom_Curve? c3d  = BRep_Tool.Curve(edge, out double first, out double last);
            if (c3d is not null)
            {
                NCollection_Sequence<Point3D> aSeqPnt = new NCollection_Sequence<Point3D>();
                if (!ShapeAnalysis_Curve.GetSamplePoints(c3d, first, last, aSeqPnt))
                {
                    continue;
                }

                nbc++;
                if (edge.Orientation() == TopAbs_REVERSED)
                {
                    aSeqPnt.Reverse();
                }

                if (nbc == 1)
                {
                    fuv = aSeqPnt.Value(1);
                    uv0 = fuv;
                }

                int j = 1;
                for (; j <= aSeqPnt.Length(); j++)
                {
                    luv = aSeqPnt.Value(j);
                    aTotal += luv.XYZ() ^ fuv.XYZ();
                    fuv = luv;
                }
            }
        }

        aTotal += uv0.XYZ() ^ fuv.XYZ();
        double anArea = aTotal.Modulus() * 0.5;
        return anArea;
    }

    /// <summary>Returns true if the face has an outer bound.</summary>
    public static bool IsOuterBound(in TopoDS_Face face)
    {
        TopoDS_Face F = face;
        TopoDS_Wire W = new TopoDS_Wire();
        F.Orientation(TopAbs_FORWARD);
        int nbw = 0;
        for (TopExp_Explorer exp = new TopExp_Explorer(F, TopAbs_WIRE); exp.More(); exp.Next())
        {
            W = TopoDS.Wire(exp.Current());
            nbw++;
        }

        // skl 08.04.2002
        if (nbw == 1)
        {
            ShapeExtend_WireData sewd = new ShapeExtend_WireData(W);
            double totcross = TotCross2D(sewd, F);
            return (totcross >= 0);
        }
        else
        {
            BRepAdaptor_Surface Ads = new BRepAdaptor_Surface(F, false);
            double tol   = BRep_Tool.Tolerance(F);
            double toluv = Min(Ads.UResolution(tol), Ads.VResolution(tol));
            BRepTopAdaptor_FClass2d fcl = new BRepTopAdaptor_FClass2d(F, toluv);
            bool rescl = (fcl.PerformInfinitePoint() == TopAbs_OUT);
            return rescl;
        }
    }

    /// <summary>
    /// Returns the positively oriented wire of the face (its outer wire); if there is none, the last wire of the face.
    /// A null wire for a face without wires.
    /// </summary>
    public static TopoDS_Wire OuterWire(in TopoDS_Face theFace)
    {
        TopoDS_Face aF = theFace;
        aF.Orientation(TopAbs_FORWARD);

        TopExp_Explorer anIt = new TopExp_Explorer(aF, TopAbs_WIRE);
        while (anIt.More())
        {
            TopoDS_Wire aWire = TopoDS.Wire(anIt.Value());
            anIt.Next();

            // if current wire is the last one, return it without analysis
            if (!anIt.More())
            {
                return aWire;
            }

            // Check if the wire has positive area
            ShapeExtend_WireData aSEWD = new ShapeExtend_WireData(aWire);
            double anArea2d = ShapeAnalysis.TotCross2D(aSEWD, aF);
            if (anArea2d >= 0.0)
            {
                return aWire;
            }
        }

        return new TopoDS_Wire();
    }

    /// <summary>
    /// Computes the exact UV bounds of the face from the pcurves of its edges, or the surface's bounds for a face without
    /// edges.
    /// </summary>
    public static void GetFaceUVBounds(in TopoDS_Face F, out double UMin, out double UMax, out double VMin, out double VMax)
    {
        TopoDS_Face FF = F;
        FF.Orientation(TopAbs_FORWARD);
        TopExp_Explorer ex = new TopExp_Explorer(FF, TopAbs_EDGE);
        if (!ex.More())
        {
            BRep_Tool.Surface(F, out _)!.Bounds(out UMin, out UMax, out VMin, out VMax);
            return;
        }

        Box2d           B   = new Box2d();
        ShapeAnalysis_Edge  sae = new ShapeAnalysis_Edge();
        ShapeAnalysis_Curve sac = new ShapeAnalysis_Curve();
        for (; ex.More(); ex.Next())
        {
            TopoDS_Edge edge = TopoDS.Edge(ex.Current());
            if (!sae.PCurve(edge, F, out Geom2d_Curve? c2d, out double f, out double l, false))
            {
                continue;
            }

            sac.FillBndBox(c2d!, f, l, 20, true, ref B);
        }

        B.Get(out UMin, out VMin, out UMax, out VMax);
    }
}
