// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Tools.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Tools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>Tools for the sampling of surfaces.</summary>
public static class IntPolyh_Tools
{
    /// <summary>Whether the surface can be enlarged in U and in V: B-spline and Bézier surfaces only, in a direction in
    /// which they are neither closed, periodic nor infinite.</summary>
    public static void IsEnlargePossible(SurfaceAdaptorBase theSurf, out bool theUEnlarge, out bool theVEnlarge)
    {
        theUEnlarge = false;
        theVEnlarge = false;

        // In the context of IntPolyh_Intersection only BSpline and Bezier surfaces
        // should be enlarged
        if (theSurf.GetType() == SurfaceKind.BSplineSurface || theSurf.GetType() == SurfaceKind.BezierSurface)
        {
            // Check U periodicity and closeness
            if (!theSurf.IsUClosed && !theSurf.IsUPeriodic)
            {
                // Check that surface is not infinite in U direction
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(theSurf.FirstUParameter()) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(theSurf.LastUParameter()))
                {
                    theUEnlarge = true;
                }
            }

            // Check V periodicity and closeness
            if (!theSurf.IsVClosed && !theSurf.IsVPeriodic)
            {
                // Check that surface is not infinite in V direction
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(theSurf.FirstVParameter()) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(theSurf.LastVParameter()))
                {
                    theVEnlarge = true;
                }
            }
        }
    }

    // Enlarges the sampling zone of the surface by 1% at each end, where possible
    private static void EnlargeZone(SurfaceAdaptorBase theSurf, ref double u0, ref double u1, ref double v0, ref double v1)
    {
        IsEnlargePossible(theSurf, out bool isToEnlargeU, out bool isToEnlargeV);
        // Enlarge U
        if (isToEnlargeU)
        {
            double delta_u = 0.01 * Math.Abs(u1 - u0);
            u0 -= delta_u;
            u1 += delta_u;
        }

        if (isToEnlargeV)
        {
            double delta_v = 0.01 * Math.Abs(v1 - v0);
            v0 -= delta_v;
            v1 += delta_v;
        }
    }

    /// <summary>
    /// Samples the surface: <paramref name="theNbSU"/> x <paramref name="theNbSV"/> evenly spaced parameters over its
    /// bounds (enlarged when <paramref name="theEnlargeZone"/>), into the arrays (resized from 1).
    /// </summary>
    public static void MakeSampling(SurfaceAdaptorBase theSurf, int theNbSU, int theNbSV, bool theEnlargeZone,
                                    NCollection_Array1<double> theUPars, NCollection_Array1<double> theVPars)
    {
        // Resize arrays
        theUPars.Resize(1, theNbSU, false);
        theVPars.Resize(1, theNbSV, false);
        //
        double u0, u1, v0, v1;
        u0 = theSurf.FirstUParameter();
        u1 = theSurf.LastUParameter();
        v0 = theSurf.FirstVParameter();
        v1 = theSurf.LastVParameter();

        // Enlarge surface intersection zone if necessary
        if (theEnlargeZone)
        {
            EnlargeZone(theSurf, ref u0, ref u1, ref v0, ref v1);
        }

        int aNbSamplesU1 = theNbSU - 1;
        int aNbSamplesV1 = theNbSV - 1;

        // U step
        double dU = (u1 - u0) / (double)aNbSamplesU1;
        // V step
        double dV = (v1 - v0) / (double)aNbSamplesV1;

        // Fill arrays
        for (int i = 0; i < theNbSU; ++i)
        {
            double aU = u0 + i * dU;
            if (i == aNbSamplesU1)
            {
                aU = u1;
            }

            theUPars.SetValue(i + 1, aU);
        }
        //
        for (int i = 0; i < theNbSV; ++i)
        {
            double aV = v0 + i * dV;
            if (i == aNbSamplesV1)
            {
                aV = v1;
            }

            theVPars.SetValue(i + 1, aV);
        }
    }

    /// <summary>The deflection tolerance of the sampling: IntCurveSurface's polyhedron's over-estimation.</summary>
    public static double ComputeDeflection(SurfaceAdaptorBase theSurf, NCollection_Array1<double> theUPars,
                                           NCollection_Array1<double> theVPars)
    {
        IntCurveSurface_ThePolyhedronOfHInter polyhedron = new(theSurf, theUPars, theVPars);
        double aDeflTol = polyhedron.DeflectionOverEstimation();
        return aDeflTol;
    }

    /// <summary>Fills the array with the surface's points at the sampling (U major) and its unit normals there (zero
    /// where the normal vanishes).</summary>
    public static void FillArrayOfPointNormal(SurfaceAdaptorBase theSurf, NCollection_Array1<double> theUPars,
                                              NCollection_Array1<double> theVPars,
                                              IntPolyh_Array<IntPolyh_PointNormal> thePoints)
    {
        int aNbU = theUPars.Length();
        int aNbV = theVPars.Length();
        int iCnt = 0;
        thePoints.Init(aNbU * aNbV);
        for (int i = 1; i <= aNbU; ++i)
        {
            double aU = theUPars[i];

            for (int j = 1; j <= aNbV; ++j)
            {
                double aV = theVPars[j];
                // Compute the point
                theSurf.D1(aU, aV, out Point3D aP, out Vector3D aDU, out Vector3D aDV);
                // Compute normal
                Vector3D aVNorm = aDU.Crossed(aDV);
                double aLength = aVNorm.Magnitude();
                if (aLength > Constants.Resolution())
                {
                    aVNorm /= aLength;
                }
                else
                {
                    aVNorm.SetCoord(0.0, 0.0, 0.0);
                }

                // Save the pair
                ref IntPolyh_PointNormal aPN = ref thePoints[iCnt];
                aPN.Point = aP;
                aPN.Normal = aVNorm;

                ++iCnt;
            }
        }

        thePoints.SetNbItems(iCnt);
    }
}
