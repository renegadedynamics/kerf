// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>Provides tools to implement local topological operations on a shape.</summary>
public static class LocOpe
{
    private const int NECHANT = 10;

    /// <summary>Returns true when the wire <paramref name="W"/> is closed on the face <paramref name="F"/>.</summary>
    public static bool Closed(in Wire W, in Face F)
    {
        Exploration.Vertices(W, out Vertex Vf, out Vertex Vl);
        if (!Vf.IsSame(Vl))
        {
            return false;
        }

        // On recherche l`edge contenant Vf FORWARD

        Explorer exp = new(), exp2 = new();
        for (exp.Init(W.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
        {
            for (exp2.Init(exp.Current(), ShapeType.Vertex); exp2.More(); exp2.Next())
            {
                if (exp2.Current().IsSame(Vf) && exp2.Current().Orientation() == Forward)
                {
                    break;
                }
            }
            if (exp2.More())
            {
                break;
            }
        }
        Edge Ef = ShapeCasts.Edge(exp.Current());

        // On recherche l`edge contenant Vl REVERSED

        for (exp.Init(W.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
        {
            for (exp2.Init(exp.Current(), ShapeType.Vertex); exp2.More(); exp2.Next())
            {
                if (exp2.Current().IsSame(Vl) && exp2.Current().Orientation() == Reversed)
                {
                    break;
                }
            }
            if (exp2.More())
            {
                break;
            }
        }
        Edge El = ShapeCasts.Edge(exp.Current());

        Point2D pf, pl;
        Curve2D C2d = Tool.CurveOnSurface(Ef, F, out double f, out double l)!;
        if (Ef.Orientation() == Forward)
        {
            pf = C2d.Value(f);
        }
        else
        {
            pf = C2d.Value(l);
        }
        C2d = Tool.CurveOnSurface(El, F, out f, out l)!;
        if (El.Orientation() == Forward)
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
    public static bool Closed(in Edge E, in Face F)
    {
        BrepBuilder B = new();
        Wire W = new();
        B.MakeWire(ref W);
        B.Add(W, E.Oriented(Forward));
        return Closed(W, F);
    }

    /// <summary>Returns true when the faces are tangent along <paramref name="E"/>.</summary>
    public static bool TgtFaces(in Edge E, in Face F1, in Face F2)
    {
        FaceSurfaceAdaptor bs = new(F1, false);
        double u;
        double ta = 0.0001;

        Edge e = E;

        FaceSurfaceAdaptor HS1 = new(F1);
        FaceSurfaceAdaptor HS2 = new(F2);
        e.Orientation(Forward);
        EdgeCurve2DAdaptor HC2d = new();
        EdgeCurve2DAdaptor HC2d2 = new();
        HC2d.Initialize(e, F1);
        HC2d2.Initialize(e, F2);

        bool rev1 = (F1.Orientation() == Reversed);
        bool rev2 = (F2.Orientation() == Reversed);
        double eps, angmin = Math.PI, angmax = -Math.PI, ang;
        Tool.Range(e, out double f, out double l);

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
    public static void SampleEdges(in Shape theShape, NCollection_Sequence<Point3D> theSeq)
    {
        theSeq.Clear();
        NCollection_Map<Shape> theMap = new(ShapeHasher.Instance);

        Explorer exp = new(theShape, ShapeType.Edge);
        double prm;
        int i;

        // Computes points on edge, but does not take the extremities into account
        for (; exp.More(); exp.Next())
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            if (!theMap.Add(edg))
            {
                continue;
            }
            if (!Tool.Degenerated(edg))
            {
                Curve3D C = Tool.Curve(edg, out Placement Loc, out double f, out double l)!;
                C = (Curve3D)C.Transformed(Loc.Transformation());
                double delta = (l - f) / NECHANT * 0.123456;
                for (i = 1; i < NECHANT; i++)
                {
                    prm = delta + ((NECHANT - i) * f + i * l) / NECHANT;
                    theSeq.Append(C.Value(prm));
                }
            }
        }

        // Adds every vertex
        for (exp.Init(theShape, ShapeType.Vertex); exp.More(); exp.Next())
        {
            if (theMap.Add(exp.Current()))
            {
                theSeq.Append(Tool.Pnt(ShapeCasts.Vertex(exp.Current())));
            }
        }
    }
}
