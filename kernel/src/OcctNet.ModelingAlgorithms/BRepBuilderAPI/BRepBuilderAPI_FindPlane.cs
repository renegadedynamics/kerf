// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_FindPlane.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_FindPlane.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Finds the plane in which the edges of a shape lie, if there is one.</summary>
/// <remarks>
/// A plane is built only if every edge is within the tolerance of it: the larger of the given tolerance (negative by
/// default) and the largest edge tolerance. The shape is only read; <see cref="Plane"/> is frozen.
/// </remarks>
public sealed class BRepBuilderAPI_FindPlane
{
    private Geom_Plane? myPlane;

    /// <summary>An empty algorithm; give the shape with <see cref="Init"/>.</summary>
    public BRepBuilderAPI_FindPlane()
    {
    }

    /// <summary>Finds the plane containing the edges of <paramref name="S"/>.</summary>
    public BRepBuilderAPI_FindPlane(in TopoDS_Shape S, double Tol = -1)
    {
        Init(S, Tol);
    }

    /// <summary>Finds the plane containing the edges of <paramref name="S"/>.</summary>
    public void Init(in TopoDS_Shape S, double Tol = -1)
    {
        double tolerance = Tol;
        myPlane = null;

        // compute the tolerance
        var ex = new TopExp_Explorer();
        for (ex.Init(S, TopAbs_EDGE); ex.More(); ex.Next())
        {
            double t = BRep_Tool.Tolerance(TopoDS.Edge(ex.Current()));
            if (t > tolerance)
            {
                tolerance = t;
            }
        }

        double tol2 = tolerance * tolerance;
        // try to find an analytical curve and calculate points
        bool found = false;
        Geom_Plane? P = null;
        var points = new NCollection_Sequence<Point3D>();
        int nbPnts;

        for (ex.Init(S, TopAbs_EDGE); ex.More(); ex.Next())
        {
            Geom_Curve? c3d = BRep_Tool.Curve(TopoDS.Edge(ex.Current()), out Placement loc, out double first,
                                              out double last);
            if (c3d is not null)
            {
                var c3dptr = (Geom_Curve)c3d.Transformed(loc.Transformation());

                Type cType = c3dptr.GetType();

                if (cType == typeof(Geom_Line))
                {
                    nbPnts = 3;
                }
                else if ((cType == typeof(Geom_Circle)) || (cType == typeof(Geom_Ellipse))
                         || (cType == typeof(Geom_Parabola))
                         || (cType == typeof(Geom_Hyperbola)))
                {
                    nbPnts = 4;
                    if (!found)
                    {
                        found = true;
                        var Co = (Geom_Conic)c3dptr;
                        P = new Geom_Plane(new CoordinateSystem(Co.Position()));
                    }
                }
                else if (cType == typeof(Geom_BezierCurve))
                {
                    var Co = (Geom_BezierCurve)c3dptr;
                    nbPnts = Co.NbPoles();
                }
                else if (cType == typeof(Geom_BSplineCurve))
                {
                    var Co = (Geom_BSplineCurve)c3dptr;
                    nbPnts = Co.NbPoles();
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
                    Vector3D V1 = new(p0, p1), V3;
                    double proMax = 0.0;
                    Point3D p2 = p0;
                    for (int j = 2; j <= points.Length(); j++)
                    {
                        V3 = V1 ^ new Vector3D(p0, points[j]);
                        double pro = V3.SquareMagnitude();
                        if (pro > proMax)
                        {
                            proMax = pro;
                            p2 = points[j];
                        }
                    }

                    if (p0.SquareDistance(p2) > tol2)
                    {
                        Direction3D D1 = new(V1), D2 = new(new Vector3D(p0, p2));
                        if (!D1.IsParallel(D2, global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            P = new Geom_Plane(new CoordinateSystem(p0, D1.Crossed(D2), D1));
                            found = true;
                        }
                    }
                }
            }
        }

        if (found)
        {
            // test if all points are on the plane
            Plane3D pln = P!.Pln();
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
            P!.Freeze();
            myPlane = P;
        }
    }

    /// <summary>True if a plane containing the edges of the shape was found.</summary>
    public bool Found()
    {
        return myPlane is not null;
    }

    /// <summary>The plane containing the edges of the shape (frozen), or null if none was found.</summary>
    public Geom_Plane? Plane()
    {
        return myPlane;
    }
}
