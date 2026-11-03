// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Construction.Library;

public static partial class BrepLibrary
{
    //=======================================================================
    // function: findNearestValidPoint
    // purpose : Starting from the appointed end of the curve, find the nearest
    //           point on the curve that is an intersection with the sphere with
    //           center theVertPnt and radius theTol.
    //=======================================================================
    private static bool FindNearestValidPoint(CurveAdaptorBase curve,
                                              double first,
                                              double last,
                                              bool isFirst,
                                              in Point3D vertPnt,
                                              double tol,
                                              double eps,
                                              ref double par)
    {
        // 1. Check that the needed end is inside the sphere

        double startU = first;
        double endU = last;
        if (!isFirst)
        {
            (startU, endU) = (endU, startU);
        }
        Point3D p = curve.Value(startU);
        double sqTol = tol * tol;
        if (p.SquareDistanceTo(vertPnt) > sqTol)
        {
            // the vertex does not cover the corresponding to this vertex end of the curve
            return false;
        }

        // 2. Find a nearest point that is outside

        // stepping along the curve by theTol till go out
        //
        // the general step is computed using general curve resolution
        double step = curve.Resolution(tol) * 1.01;
        if (step < eps)
        {
            step = eps;
        }
        // aD1Mag is a threshold to consider local derivative magnitude too small
        // and to accelerate going out of sphere
        // (inverse of resolution is the maximal derivative);
        // this is actual for bezier and b-spline types only
        double d1Mag = 0.0;
        CurveKind type = curve.GetType();
        if (type == OffsetCurve)
        {
            OffsetCurve3D offsetCurve = curve.OffsetCurve();
            Curve3D baseCurve = offsetCurve.BasisCurve;
            type = new CurveAdaptor(baseCurve).GetType();
        }
        if (type == BezierCurve || type == BSplineCurve)
        {
            d1Mag = 1.0 / curve.Resolution(1.0) * 0.01;
            d1Mag *= d1Mag;
        }
        if (!isFirst)
        {
            step = -step;
        }
        bool isOut = false;
        double uIn = startU;
        double uOut = uIn;
        while (!isOut)
        {
            uIn = uOut;
            uOut += step;
            if ((isFirst && uOut > endU) || (!isFirst && uOut < endU))
            {
                // step is too big and we go out of bounds,
                // check if the opposite bound is outside
                p = curve.Value(endU);
                isOut = (p.SquareDistanceTo(vertPnt) > sqTol);
                if (!isOut)
                {
                    // all range is inside sphere
                    return false;
                }
                uOut = endU;
                break;
            }
            if (d1Mag > 0.0)
            {
                double stepLocal = step;
                for (;;)
                {
                    // cycle to go out of local singularity
                    curve.D1(uOut, out p, out Vector3D d1);
                    isOut = (p.SquareDistanceTo(vertPnt) > sqTol);
                    if (!isOut && d1.SquareMagnitude < d1Mag)
                    {
                        stepLocal *= 2.0;
                        uOut += stepLocal;
                        if ((isFirst && uOut < endU) || (!isFirst && uOut > endU))
                        {
                            // still in range
                            continue;
                        }
                        // went out of range, so check if the end point has out state
                        uOut = endU;
                        p = curve.Value(uOut);
                        isOut = (p.SquareDistanceTo(vertPnt) > sqTol);
                        if (!isOut)
                        {
                            // all range is inside sphere
                            return false;
                        }
                    }
                    break;
                }
            }
            else
            {
                p = curve.Value(uOut);
            }
            if (!isOut)
            {
                isOut = (p.SquareDistanceTo(vertPnt) > sqTol);
            }
        }

        // 3. Precise solution with binary search

        double delta = Math.Abs(uOut - uIn);
        while (delta > eps)
        {
            double midU = (uIn + uOut) * 0.5;
            p = curve.Value(midU);
            isOut = (p.SquareDistanceTo(vertPnt) > sqTol);
            if (isOut)
            {
                uOut = midU;
            }
            else
            {
                uIn = midU;
            }
            delta = Math.Abs(uOut - uIn);
        }
        par = (uIn + uOut) * 0.5;
        return true;
    }

    /// <summary>
    /// For an edge given by a 3D curve and a tolerance, and its vertices by points, parameters and tolerances, finds
    /// the range of the curve between the vertices that their tolerances don't cover. Returns false if there is none;
    /// otherwise sets <paramref name="first"/> and <paramref name="last"/> (which may be written before a false
    /// return, as in OCCT).
    /// </summary>
    public static bool FindValidRange(CurveAdaptorBase curve,
                                      double tolE,
                                      double parV1,
                                      in Point3D pntV1,
                                      double tolV1,
                                      double parV2,
                                      in Point3D pntV2,
                                      double tolV2,
                                      ref double first,
                                      ref double last)
    {
        if (parV2 - parV1 < Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        bool isInfParV1 = Kerf.Engine.Foundation.Precision.IsInfinite(parV1), isInfParV2 = Kerf.Engine.Foundation.Precision.IsInfinite(parV2);

        double maxPar = 0.0;
        if (!isInfParV1)
        {
            maxPar = Math.Abs(parV1);
        }
        if (!isInfParV2)
        {
            maxPar = Max(maxPar, Math.Abs(parV2));
        }

        double eps = Max(Max(curve.Resolution(tolE) * 0.1, Epsilon(maxPar)), Kerf.Engine.Foundation.Precision.PConfusion());

        if (isInfParV1)
        {
            first = parV1;
        }
        else
        {
            if (!FindNearestValidPoint(curve, parV1, parV2, true, pntV1, tolV1, eps, ref first))
            {
                return false;
            }
            if (parV2 - first < eps)
            {
                return false;
            }
        }

        if (isInfParV2)
        {
            last = parV2;
        }
        else
        {
            if (!FindNearestValidPoint(curve, parV1, parV2, false, pntV2, tolV2, eps, ref last))
            {
                return false;
            }
            if (last - parV1 < eps)
            {
                return false;
            }
        }

        // check found parameters
        if (first > last)
        {
            // overlapping, not valid range
            return false;
        }

        return true;
    }

    /// <summary>
    /// The range of the edge's 3D curve that its vertices' tolerances (increased by the confusion precision) don't
    /// cover. Returns false if there is none.
    /// </summary>
    public static bool FindValidRange(in Edge edge, ref double first, ref double last)
    {
        if (Tool.Curve(edge, out _, out _, out _) is null)
        {
            return false;
        }
        var ac = new EdgeCurveAdaptor(edge);
        Span<double> parV = [ac.FirstParameter(), ac.LastParameter()];
        if (parV[1] - parV[0] < Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        // get vertices
        var v = new Vertex[2];
        Exploration.Vertices(edge, out v[0], out v[1]);

        double tolE = Tool.Tolerance(edge);
        // to have correspondence with intersection precision
        // the tolerances of vertices are increased on Precision::Confusion()
        Span<double> tolV = [Kerf.Engine.Foundation.Precision.Confusion(), Kerf.Engine.Foundation.Precision.Confusion()];
        var pntV = new Point3D[] { new(), new() };
        for (int i = 0; i < 2; i++)
        {
            if (!v[i].IsNull)
            {
                tolV[i] += Tool.Tolerance(v[i]);
                pntV[i] = Tool.Pnt(v[i]);
            }
            else if (!Kerf.Engine.Foundation.Precision.IsInfinite(parV[i]))
            {
                tolV[i] += tolE;
                pntV[i] = ac.Value(parV[i]);
            }
        }
        return FindValidRange(ac, tolE, parV[0], pntV[0], tolV[0], parV[1], pntV[1], tolV[1], ref first, ref last);
    }

    /// <summary>
    /// Stores on the edge its pcurve on the face when the face is a plane and the pcurve was only computed (in place:
    /// internal).
    /// </summary>
    internal static void BuildPCurveForEdgeOnPlane(in Edge aE, in Face aF)
    {
        bool bToUpdate;
        double tolE;
        var bb = new BrepBuilder();
        //
        BuildPCurveForEdgeOnPlane(aE, aF, out Curve2D? c2D, out bToUpdate);
        if (bToUpdate)
        {
            tolE = Tool.Tolerance(aE);
            bb.UpdateEdge(aE, c2D, aF, tolE);
        }
    }

    /// <summary>
    /// The pcurve of the edge on the face, and whether it would have to be stored on the edge: true when it was
    /// computed (the face is a plane and the edge has no pcurve on it). Reads only.
    /// </summary>
    public static void BuildPCurveForEdgeOnPlane(in Edge aE, in Face aF, out Curve2D? aC2D, out bool bToUpdate)
    {
        aC2D = Tool.CurveOnSurface(aE, aF, out _, out _, out bool isStored);
        bToUpdate = !isStored && aC2D is not null;
    }

    /// <summary>Builds the pcurves of the edges of <paramref name="le"/> on the face if it is a plane (in place: internal).</summary>
    internal static void BuildPCurveForEdgesOnPlane(IEnumerable<Shape> le, in Face f)
    {
        foreach (Shape s in le)
        {
            Edge e = ShapeCasts.Edge(s);
            if (!e.IsNull)
            {
                BuildPCurveForEdgeOnPlane(e, f);
            }
        }
    }
}
