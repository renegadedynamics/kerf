// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdgesInFace.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdgesInFace.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_FindEdgesInFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Features.Local;

/// <summary>
/// Finds the edges of a shape lying in a face (planar or cylindrical): the edges the face shares, and the lines and
/// circles on its surface whose ends and middle are inside it.
/// </summary>
/// <remarks>A working object (decision 21): it only reads the shapes; the edges are the shape's own.</remarks>
internal sealed class FindEdgesInFace
{
    private Shape _shape = new();
    private Face _face = new();
    private readonly NCollection_List<Shape> _list = new();
    private readonly NCollection_List<Shape>.Iterator _it = new();

    public FindEdgesInFace()
    {
    }

    public FindEdgesInFace(in Shape s, in Face f)
    {
        Set(s, f);
    }

    public void Init()
    {
        _it.Initialize(_list);
    }

    public bool More() => _it.More();

    public Edge Edge() => ShapeCasts.Edge(_it.Value());

    public void Next()
    {
        _it.Next();
    }

    public void Set(in Shape sh, in Face F)
    {
        _shape = sh;
        _face = F;
        _list.Clear();

        HashSet<Shape> m = new(ShapeHasher.Instance);
        Explorer exp = new(), expf = new();
        Curve3D c;
        Surface s;
        double f, l;
        Type tc, ts;
        bool toAdd;
        Plane3D pl = new();
        Cylinder3D cy = new();

        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        double tolAng = global::Kerf.Engine.Foundation.Precision.Angular();

        s = Tool.Surface(F)!;
        ts = s.GetType();
        if (ts == typeof(TrimmedSurface))
        {
            s = ((TrimmedSurface)s).BasisSurface;
            ts = s.GetType();
        }

        if (ts != typeof(PlaneSurface) && ts != typeof(CylindricalSurface))
        {
            return; // pour le moment
        }

        FaceSurfaceAdaptor hs = new(_face);
        BrepTopolTool tpt = new(hs);

        for (exp.Init(_shape, global::Kerf.Topology.ShapeType.Edge); exp.More(); exp.Next())
        {
            toAdd = false;
            Edge edg = ShapeCasts.Edge(exp.Current());
            if (m.Contains(edg))
            {
                continue;
            }

            for (expf.Init(_face, global::Kerf.Topology.ShapeType.Edge); expf.More(); expf.Next())
            {
                if (expf.Current().IsSame(edg))
                {
                    break;
                }
            }
            if (expf.More())
            { // partage d`edge
                m.Add(edg);
                _list.Append(edg);
                continue;
            }

            c = Tool.Curve(edg, out Placement loc, out f, out l)!;
            c = (Curve3D)c.Transformed(loc.Transformation);
            tc = c.GetType();
            if (tc == typeof(TrimmedCurve3D))
            {
                c = ((TrimmedCurve3D)c).BasisCurve;
                tc = c.GetType();
            }
            if (tc != typeof(LineCurve3D) && tc != typeof(CircleCurve3D))
            {
                continue; // pour le moment
            }
            if (ts == typeof(PlaneSurface))
            {
                pl = ((PlaneSurface)s).Plane;
            }
            else
            {
                cy = ((CylindricalSurface)s).Cylinder;
            }

            if (tc == typeof(LineCurve3D))
            {
                Line3D li = ((LineCurve3D)c).Line;
                if (ts == typeof(PlaneSurface))
                {
                    if (pl.Contains(li, tol, tolAng))
                    {
                        toAdd = true;
                    }
                }
                else
                { // Ts == STANDARD_TYPE(Geom_CylindricalSurface)
                    if (cy.Axis.IsParallel(li.Position, tolAng) && Math.Abs(li.DistanceTo(cy.Location) - cy.Radius) < tol)
                    {
                        toAdd = true;
                    }
                }
            }
            else
            { // Tt == STANDARD_TYPE(Geom_Circle)
                Circle3D ci = ((CircleCurve3D)c).Circle;
                if (ts == typeof(PlaneSurface))
                {
                    if (pl.Position.IsCoplanar(new CoordinateSystem(ci.Position), tol, tolAng))
                    {
                        toAdd = true;
                    }
                }
                else
                {
                    if (Math.Abs(cy.Radius - ci.Radius) < tol && cy.Axis.IsCoaxial(ci.Axis, tolAng, tol))
                    {
                        toAdd = true;
                    }
                }
            }

            if (toAdd)
            {
                // On classifie 3 points.
                Point3D[] p = new Point3D[3];
                double u, v;
                p[0] = c.Value(f);
                p[1] = c.Value(l);
                p[2] = c.Value((f + l) / 2.0);
                int i;
                for (i = 0; i < 3; i++)
                {
                    if (ts == typeof(PlaneSurface))
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(pl, p[i], out u, out v);
                    }
                    else
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cy, p[i], out u, out v);
                    }
                    if (tpt.Classify(new Point2D(u, v), global::Kerf.Engine.Foundation.Precision.Confusion()) == Out)
                    {
                        break;
                    }
                }
                if (i >= 3)
                {
                    m.Add(edg);
                    _list.Append(edg);
                }
            }
        }
    }
}
