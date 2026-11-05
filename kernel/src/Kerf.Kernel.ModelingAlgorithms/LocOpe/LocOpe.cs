// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features.Local;

/// <summary>Provides tools to implement local topological operations on a shape.</summary>
internal static class LocOpe
{
    private const int Nechant = 10;

    /// <summary>Returns true when the wire <paramref name="w"/> is closed on the face <paramref name="F"/>.</summary>
    public static bool Closed(in Wire w, in Face F)
    {
        Exploration.Vertices(w, out Vertex vf, out Vertex vl);
        if (!vf.IsSame(vl))
        {
            return false;
        }

        // On recherche l`edge contenant Vf FORWARD

        Explorer exp = new(), exp2 = new();
        for (exp.Init(w.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
        {
            for (exp2.Init(exp.Current(), ShapeType.Vertex); exp2.More(); exp2.Next())
            {
                if (exp2.Current().IsSame(vf) && exp2.Current().Orientation == Forward)
                {
                    break;
                }
            }
            if (exp2.More())
            {
                break;
            }
        }
        Edge ef = ShapeCasts.Edge(exp.Current());

        // On recherche l`edge contenant Vl REVERSED

        for (exp.Init(w.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
        {
            for (exp2.Init(exp.Current(), ShapeType.Vertex); exp2.More(); exp2.Next())
            {
                if (exp2.Current().IsSame(vl) && exp2.Current().Orientation == Reversed)
                {
                    break;
                }
            }
            if (exp2.More())
            {
                break;
            }
        }
        Edge el = ShapeCasts.Edge(exp.Current());

        Point2D pf, pl;
        Curve2D c2d = Tool.CurveOnSurface(ef, F, out double f, out double l)!;
        if (ef.Orientation == Forward)
        {
            pf = c2d.Value(f);
        }
        else
        {
            pf = c2d.Value(l);
        }
        c2d = Tool.CurveOnSurface(el, F, out f, out l)!;
        if (el.Orientation == Forward)
        {
            pl = c2d.Value(l);
        }
        else
        {
            pl = c2d.Value(f);
        }

        if (pf.DistanceTo(pl) <= global::Kerf.Engine.Foundation.Precision.PConfusion(global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            return true;
        }
        return false;
    }

    /// <summary>Returns true when the edge <paramref name="e"/> is closed on the face <paramref name="f"/>.</summary>
    public static bool Closed(in Edge e, in Face f)
    {
        BrepBuilder b = new();
        Wire w = new();
        b.MakeWire(ref w);
        b.Add(w, e.Oriented(Forward));
        return Closed(w, f);
    }

    /// <summary>Returns true when the faces are tangent along <paramref name="E"/>.</summary>
    public static bool TgtFaces(in Edge E, in Face f1, in Face f2)
    {
        FaceSurfaceAdaptor bs = new(f1, false);
        double u;
        double ta = 0.0001;

        Edge e = E;

        FaceSurfaceAdaptor hs1 = new(f1);
        FaceSurfaceAdaptor hs2 = new(f2);
        e.Orient(Forward);
        EdgeCurve2DAdaptor hc2d = new();
        EdgeCurve2DAdaptor hc2d2 = new();
        hc2d.Initialize(e, f1);
        hc2d2.Initialize(e, f2);

        bool rev1 = (f1.Orientation == Reversed);
        bool rev2 = (f2.Orientation == Reversed);
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
            hc2d.D0(u, out Point2D p);
            hs1.D1(p.X, p.Y, out Point3D pp1, out Vector3D du, out Vector3D dv);
            d1 = (du.Crossed(dv)).Normalized();
            if (rev1)
            {
                d1 = d1.Reversed();
            }
            hc2d2.D0(u, out p);
            p.Coord(out uu, out vv);
            hs2.D1(uu, vv, out pp1, out du, out dv);
            d2 = (du.Crossed(dv)).Normalized();
            if (rev2)
            {
                d2 = d2.Reversed();
            }
            ang = d1.AngleTo(d2);
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
    /// Fills <paramref name="seq"/> with points sampled on the edges of <paramref name="shape"/> (not at their
    /// ends), then with every vertex.
    /// </summary>
    public static void SampleEdges(in Shape shape, NCollection_Sequence<Point3D> seq)
    {
        seq.Clear();
        HashSet<Shape> theMap = new(ShapeHasher.Instance);

        Explorer exp = new(shape, ShapeType.Edge);
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
                Curve3D c = Tool.Curve(edg, out Placement loc, out double f, out double l)!;
                c = (Curve3D)c.Transformed(loc.Transformation);
                double delta = (l - f) / Nechant * 0.123456;
                for (i = 1; i < Nechant; i++)
                {
                    prm = delta + ((Nechant - i) * f + i * l) / Nechant;
                    seq.Append(c.Value(prm));
                }
            }
        }

        // Adds every vertex
        for (exp.Init(shape, ShapeType.Vertex); exp.More(); exp.Next())
        {
            if (theMap.Add(exp.Current()))
            {
                seq.Append(Tool.Pnt(ShapeCasts.Vertex(exp.Current())));
            }
        }
    }
}
