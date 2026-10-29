// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_QuadricCurveExactInterUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// The intersection of a curve with a quadric by the roots of the quadric's implicit function along the curve
/// (OCCT's namespace IntCurveSurface_QuadricCurveExactInterUtils).
/// </summary>
internal static class IntCurveSurface_QuadricCurveExactInterUtils
{
    /// <summary>Tolerance values for root finding</summary>
    public const double EPSX = 0.00000000000001;
    public const double EPSDIST = 0.00000001;
    public const double EPSNUL = 0.00000001;

    /// <summary>
    /// Intersects the curve with the quadric surface: the roots (<paramref name="thePnts"/>) and the intervals where
    /// the curve lies on the surface (pairs of bounds in <paramref name="theIntv"/>) on each C1 interval of the curve.
    /// The counts are -1 if the surface is not a quadric or a root search failed.
    /// </summary>
    public static void PerformIntersection<SurfaceType, SurfaceTool, CurveType, CurveTool, QuadCurvFuncType>(
        in SurfaceType theSurface, in CurveType theCurve, NCollection_Sequence<double> thePnts,
        NCollection_Sequence<double> theIntv, out int theNbPnts, out int theNbIntv)
        where SurfaceTool : IIntCurveSurface_SurfaceTool<SurfaceType>
        where CurveTool : IIntCurveSurface_CurveTool<CurveType>
        where QuadCurvFuncType : math_FunctionWithDerivative, IIntCurveSurface_TheQuadCurvFunc<QuadCurvFuncType, CurveType>
    {
        theNbPnts = -1;
        theNbIntv = -1;
        thePnts.Clear();
        theIntv.Clear();

        GeomAbs_SurfaceType aQuadricType = SurfaceTool.GetType(theSurface);
        IntSurf_Quadric aQuadric = new();

        switch (aQuadricType)
        {
            case GeomAbs_Plane:
            {
                aQuadric.SetValue(SurfaceTool.Plane(theSurface));
                break;
            }
            case GeomAbs_Cylinder:
            {
                aQuadric.SetValue(SurfaceTool.Cylinder(theSurface));
                break;
            }
            case GeomAbs_Cone:
            {
                aQuadric.SetValue(SurfaceTool.Cone(theSurface));
                break;
            }
            case GeomAbs_Sphere:
            {
                aQuadric.SetValue(SurfaceTool.Sphere(theSurface));
                break;
            }
            default:
            {
                return;
            }
        }

        int aNbIntervals = CurveTool.NbIntervals(theCurve, GeomAbs_C1);
        NCollection_Array1<double> anIntervals = new(1, aNbIntervals + 1);
        int ii;

        CurveTool.Intervals(theCurve, anIntervals, GeomAbs_C1);

        for (ii = 1; ii <= aNbIntervals; ii++)
        {
            double U1 = anIntervals.Value(ii);
            double U2 = anIntervals.Value(ii + 1);

            math_FunctionSample aSample = new(U1, U2, CurveTool.NbSamples(theCurve, U1, U2));
            QuadCurvFuncType aFunction = QuadCurvFuncType.Create(aQuadric, theCurve);
            math_FunctionAllRoots aRoots = new(aFunction, aSample, EPSX, EPSDIST, EPSNUL);

            if (aRoots.IsDone())
            {
                int aNbPoints = aRoots.NbPoints();
                int aNbRootIntv = aRoots.NbIntervals();

                for (int i = 1; i <= aNbPoints; i++)
                {
                    thePnts.Append(aRoots.GetPoint(i));
                }

                for (int i = 1; i <= aNbRootIntv; i++)
                {
                    aRoots.GetInterval(i, out double a, out double b);
                    theIntv.Append(a);
                    theIntv.Append(b);
                }
            }
            else
            {
                break;
            }
        }

        if (ii > aNbIntervals)
        {
            theNbPnts = thePnts.Length();
            theNbIntv = theIntv.Length() / 2;
        }
    }
}
