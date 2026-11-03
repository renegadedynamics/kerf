// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_FindPlane.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_FindPlane.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Api;

/// <summary>Finds the plane in which the edges of a shape lie, if there is one.</summary>
/// <remarks>
/// A plane is built only if every edge is within the tolerance of it: the larger of the given tolerance (negative by
/// default) and the largest edge tolerance. The shape is only read; <see cref="Plane"/> is frozen.
/// </remarks>
public sealed class FindPlane
{
    private PlaneSurface? _plane;

    /// <summary>An empty algorithm; give the shape with <see cref="Init"/>.</summary>
    public FindPlane()
    {
    }

    /// <summary>Finds the plane containing the edges of <paramref name="s"/>.</summary>
    public FindPlane(in Shape s, double tol = -1)
    {
        Init(s, tol);
    }

    /// <summary>Finds the plane containing the edges of <paramref name="s"/>.</summary>
    public void Init(in Shape s, double tol = -1)
    {
        double tolerance = tol;
        _plane = null;

        // compute the tolerance
        var ex = new Explorer();
        for (ex.Init(s, ShapeType.Edge); ex.More(); ex.Next())
        {
            double t = Tool.Tolerance(ShapeCasts.Edge(ex.Current()));
            if (t > tolerance)
            {
                tolerance = t;
            }
        }

        double tol2 = tolerance * tolerance;
        // try to find an analytical curve and calculate points
        bool found = false;
        PlaneSurface? p = null;
        var points = new NCollection_Sequence<Point3D>();
        int nbPnts;

        for (ex.Init(s, ShapeType.Edge); ex.More(); ex.Next())
        {
            Curve3D? c3d = Tool.Curve(ShapeCasts.Edge(ex.Current()), out Placement loc, out double first,
                                              out double last);
            if (c3d is not null)
            {
                var c3dptr = (Curve3D)c3d.Transformed(loc.Transformation());

                Type cType = c3dptr.GetType();

                if (cType == typeof(LineCurve3D))
                {
                    nbPnts = 3;
                }
                else if ((cType == typeof(CircleCurve3D)) || (cType == typeof(EllipseCurve3D))
                         || (cType == typeof(ParabolaCurve3D))
                         || (cType == typeof(HyperbolaCurve3D)))
                {
                    nbPnts = 4;
                    if (!found)
                    {
                        found = true;
                        var co = (ConicCurve3D)c3dptr;
                        p = new PlaneSurface(new CoordinateSystem(co.Position()));
                    }
                }
                else if (cType == typeof(BezierCurve3D))
                {
                    var co = (BezierCurve3D)c3dptr;
                    nbPnts = co.PoleCount;
                }
                else if (cType == typeof(BSplineCurve3D))
                {
                    var co = (BSplineCurve3D)c3dptr;
                    nbPnts = co.PoleCount;
                }
                else
                {
                    nbPnts = 10;
                }

                Point3D p0 = new();
                for (int i = 1; i <= nbPnts; i++)
                {
                    if (i == 1)
                    {
                        c3dptr.D0(first, out p0);
                    }
                    else if (i == nbPnts)
                    {
                        c3dptr.D0(last, out p0);
                    }
                    else
                    {
                        c3dptr.D0(first + (last - first) / (nbPnts - 1) * (i - 1), out p0);
                    }
                    points.Append(p0);
                }
            }
        }

        if (!found)
        {
            // try to find a plane with the points
            if (points.Length() > 2)
            {
                double disMax = 0.0;
                Point3D p0 = points[1];
                Point3D p1 = new();
                for (int i = 2; i <= points.Length(); i++)
                {
                    double dist = p0.SquareDistance(points[i]);
                    if (dist > disMax)
                    {
                        disMax = dist;
                        p1 = points[i]; // it will be faster to store the point, otherwise it is necessary to find a value in a sequence
                    }
                }

                if (disMax > tol2)
                {
                    Vector3D v1 = new(p0, p1), v3;
                    double proMax = 0.0;
                    Point3D p2 = p0;
                    for (int j = 2; j <= points.Length(); j++)
                    {
                        v3 = v1 ^ new Vector3D(p0, points[j]);
                        double pro = v3.SquareMagnitude();
                        if (pro > proMax)
                        {
                            proMax = pro;
                            p2 = points[j];
                        }
                    }

                    if (p0.SquareDistance(p2) > tol2)
                    {
                        Direction3D d1 = new(v1), d2 = new(new Vector3D(p0, p2));
                        if (!d1.IsParallel(d2, global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            p = new PlaneSurface(new CoordinateSystem(p0, d1.Crossed(d2), d1));
                            found = true;
                        }
                    }
                }
            }
        }

        if (found)
        {
            // test if all points are on the plane
            Plane3D pln = p!.Pln();
            for (int i = 1; i <= points.Length(); i++)
            {
                if (pln.SquareDistance(points[i]) > tol2)
                {
                    found = false;
                    break;
                }
            }
        }

        if (found)
        {
            p!.Freeze();
            _plane = p;
        }
    }

    /// <summary>True if a plane containing the edges of the shape was found.</summary>
    public bool Found()
    {
        return _plane is not null;
    }

    /// <summary>The plane containing the edges of the shape (frozen), or null if none was found.</summary>
    public PlaneSurface? Plane()
    {
        return _plane;
    }
}
