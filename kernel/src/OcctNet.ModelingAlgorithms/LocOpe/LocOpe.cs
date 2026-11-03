// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Provides tools to implement local topological operations on a shape.</summary>
public static class LocOpe
{
    private const int NECHANT = 10;

    /// <summary>Returns true when the wire <paramref name="W"/> is closed on the face <paramref name="F"/>.</summary>
    public static bool Closed(in TopoDS_Wire W, in TopoDS_Face F)
    {
        TopExp.Vertices(W, out TopoDS_Vertex Vf, out TopoDS_Vertex Vl);
        if (!Vf.IsSame(Vl))
        {
            return false;
        }

        // On recherche l`edge contenant Vf FORWARD

        TopExp_Explorer exp = new(), exp2 = new();
        for (exp.Init(W.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp.More(); exp.Next())
        {
            for (exp2.Init(exp.Current(), TopAbs_VERTEX); exp2.More(); exp2.Next())
            {
                if (exp2.Current().IsSame(Vf) && exp2.Current().Orientation() == TopAbs_FORWARD)
                {
                    break;
                }
            }
            if (exp2.More())
            {
                break;
            }
        }
        TopoDS_Edge Ef = TopoDS.Edge(exp.Current());

        // On recherche l`edge contenant Vl REVERSED

        for (exp.Init(W.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp.More(); exp.Next())
        {
            for (exp2.Init(exp.Current(), TopAbs_VERTEX); exp2.More(); exp2.Next())
            {
                if (exp2.Current().IsSame(Vl) && exp2.Current().Orientation() == TopAbs_REVERSED)
                {
                    break;
                }
            }
            if (exp2.More())
            {
                break;
            }
        }
        TopoDS_Edge El = TopoDS.Edge(exp.Current());

        Point2D pf, pl;
        Geom2d_Curve C2d = BRep_Tool.CurveOnSurface(Ef, F, out double f, out double l)!;
        if (Ef.Orientation() == TopAbs_FORWARD)
        {
            pf = C2d.Value(f);
        }
        else
        {
            pf = C2d.Value(l);
        }
        C2d = BRep_Tool.CurveOnSurface(El, F, out f, out l)!;
        if (El.Orientation() == TopAbs_FORWARD)
        {
            pl = C2d.Value(l);
        }
        else
        {
            pl = C2d.Value(f);
        }

        if (pf.Distance(pl) <= global::Kerf.Engine.Foundation.Precision.PConfusion(global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            return true;
        }
        return false;
    }

    /// <summary>Returns true when the edge <paramref name="E"/> is closed on the face <paramref name="F"/>.</summary>
    public static bool Closed(in TopoDS_Edge E, in TopoDS_Face F)
    {
        BRep_Builder B = new();
        TopoDS_Wire W = new();
        B.MakeWire(ref W);
        B.Add(W, E.Oriented(TopAbs_FORWARD));
        return Closed(W, F);
    }

    /// <summary>Returns true when the faces are tangent along <paramref name="E"/>.</summary>
    public static bool TgtFaces(in TopoDS_Edge E, in TopoDS_Face F1, in TopoDS_Face F2)
    {
        BRepAdaptor_Surface bs = new(F1, false);
        double u;
        double ta = 0.0001;

        TopoDS_Edge e = E;

        BRepAdaptor_Surface HS1 = new(F1);
        BRepAdaptor_Surface HS2 = new(F2);
        e.Orientation(TopAbs_FORWARD);
        BRepAdaptor_Curve2d HC2d = new();
        BRepAdaptor_Curve2d HC2d2 = new();
        HC2d.Initialize(e, F1);
        HC2d2.Initialize(e, F2);

        bool rev1 = (F1.Orientation() == TopAbs_REVERSED);
        bool rev2 = (F2.Orientation() == TopAbs_REVERSED);
        double eps, angmin = Math.PI, angmax = -Math.PI, ang;
        BRep_Tool.Range(e, out double f, out double l);

        eps = (l - f) / 100.0;
        f += eps; // pour eviter de faire des calculs sur les
        l -= eps; // pointes des carreaux pointus.
        Vector3D d1, d2;

        double uu, vv;

        int i;
        for (i = 0; i <= 20; i++)
        {
            u = f + (l - f) * i / 20;
            HC2d.D0(u, out Point2D p);
            HS1.D1(p.X(), p.Y(), out Point3D pp1, out Vector3D du, out Vector3D dv);
            d1 = (du.Crossed(dv)).Normalized();
            if (rev1)
            {
                d1.Reverse();
            }
            HC2d2.D0(u, out p);
            p.Coord(out uu, out vv);
            HS2.D1(uu, vv, out pp1, out du, out dv);
            d2 = (du.Crossed(dv)).Normalized();
            if (rev2)
            {
                d2.Reverse();
            }
            ang = d1.Angle(d2);
            if (ang <= angmin)
            {
                angmin = ang;
            }
            if (ang >= angmax)
            {
                angmax = ang;
            }
        }
        _ = bs;
        return (angmax <= ta);
    }

    /// <summary>
    /// Fills <paramref name="theSeq"/> with points sampled on the edges of <paramref name="theShape"/> (not at their
    /// ends), then with every vertex.
    /// </summary>
    public static void SampleEdges(in TopoDS_Shape theShape, NCollection_Sequence<Point3D> theSeq)
    {
        theSeq.Clear();
        NCollection_Map<TopoDS_Shape> theMap = new(TopTools_ShapeMapHasher.Instance);

        TopExp_Explorer exp = new(theShape, TopAbs_EDGE);
        double prm;
        int i;

        // Computes points on edge, but does not take the extremities into account
        for (; exp.More(); exp.Next())
        {
            TopoDS_Edge edg = TopoDS.Edge(exp.Current());
            if (!theMap.Add(edg))
            {
                continue;
            }
            if (!BRep_Tool.Degenerated(edg))
            {
                Geom_Curve C = BRep_Tool.Curve(edg, out Placement Loc, out double f, out double l)!;
                C = (Geom_Curve)C.Transformed(Loc.Transformation());
                double delta = (l - f) / NECHANT * 0.123456;
                for (i = 1; i < NECHANT; i++)
                {
                    prm = delta + ((NECHANT - i) * f + i * l) / NECHANT;
                    theSeq.Append(C.Value(prm));
                }
            }
        }

        // Adds every vertex
        for (exp.Init(theShape, TopAbs_VERTEX); exp.More(); exp.Next())
        {
            if (theMap.Add(exp.Current()))
            {
                theSeq.Append(BRep_Tool.Pnt(TopoDS.Vertex(exp.Current())));
            }
        }
    }
}
