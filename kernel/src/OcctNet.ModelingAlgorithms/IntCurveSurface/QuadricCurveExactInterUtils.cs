// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_QuadricCurveExactInterUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// The intersection of a curve with a quadric by the roots of the quadric's implicit function along the curve
/// (OCCT's namespace IntCurveSurface_QuadricCurveExactInterUtils).
/// </summary>
internal static class QuadricCurveExactInterUtils
{
    /// <summary>Tolerance values for root finding</summary>
    public const double EPSX = 0.00000000000001;
    public const double EPSDIST = 0.00000001;
    public const double EPSNUL = 0.00000001;

    /// <summary>
    /// Intersects the curve with the quadric surface: the roots (<paramref name="pnts"/>) and the intervals where
    /// the curve lies on the surface (pairs of bounds in <paramref name="intv"/>) on each C1 interval of the curve.
    /// The counts are -1 if the surface is not a quadric or a root search failed.
    /// </summary>
    public static void PerformIntersection<SurfaceType, SurfaceTool, CurveType, CurveTool, QuadCurvFuncType>(
        in SurfaceType surface, in CurveType curve, NCollection_Sequence<double> pnts,
        NCollection_Sequence<double> intv, out int nbPnts, out int nbIntv)
        where SurfaceTool : IIntersectionSurfaceTool<SurfaceType>
        where CurveTool : IIntersectionCurveTool<CurveType>
        where QuadCurvFuncType : FunctionWithDerivative, ITheQuadCurvFunc<QuadCurvFuncType, CurveType>
    {
        nbPnts = -1;
        nbIntv = -1;
        pnts.Clear();
        intv.Clear();

        SurfaceKind quadricType = SurfaceTool.GetType(surface);
        SurfaceQuadric quadric = new();

        switch (quadricType)
        {
            case Plane:
            {
                quadric.SetValue(SurfaceTool.Plane(surface));
                break;
            }
            case SurfaceKind.Cylinder:
            {
                quadric.SetValue(SurfaceTool.Cylinder(surface));
                break;
            }
            case SurfaceKind.Cone:
            {
                quadric.SetValue(SurfaceTool.Cone(surface));
                break;
            }
            case SurfaceKind.Sphere:
            {
                quadric.SetValue(SurfaceTool.Sphere(surface));
                break;
            }
            default:
            {
                return;
            }
        }

        int nbIntervals = CurveTool.NbIntervals(curve, C1);
        NCollection_Array1<double> intervals = new(1, nbIntervals + 1);
        int ii;

        CurveTool.Intervals(curve, intervals, C1);

        for (ii = 1; ii <= nbIntervals; ii++)
        {
            double u1 = intervals.Value(ii);
            double u2 = intervals.Value(ii + 1);

            FunctionSample sample = new(u1, u2, CurveTool.NbSamples(curve, u1, u2));
            QuadCurvFuncType function = QuadCurvFuncType.Create(quadric, curve);
            FunctionAllRoots roots = new(function, sample, EPSX, EPSDIST, EPSNUL);

            if (roots.IsDone)
            {
                int nbPoints = roots.PointCount;
                int nbRootIntv = roots.IntervalCount;

                for (int i = 1; i <= nbPoints; i++)
                {
                    pnts.Append(roots.GetPoint(i));
                }

                for (int i = 1; i <= nbRootIntv; i++)
                {
                    roots.GetInterval(i, out double a, out double b);
                    intv.Append(a);
                    intv.Append(b);
                }
            }
            else
            {
                break;
            }
        }

        if (ii > nbIntervals)
        {
            nbPnts = pnts.Length();
            nbIntv = intv.Length() / 2;
        }
    }
}
