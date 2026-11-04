// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Tools.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Tools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Intersection.Polyhedra;

/// <summary>Tools for the sampling of surfaces.</summary>
internal static class PolyhedronTools
{
    /// <summary>Whether the surface can be enlarged in U and in V: B-spline and Bézier surfaces only, in a direction in
    /// which they are neither closed, periodic nor infinite.</summary>
    public static void IsEnlargePossible(SurfaceAdaptorBase surf, out bool uEnlarge, out bool vEnlarge)
    {
        uEnlarge = false;
        vEnlarge = false;

        // In the context of IntPolyh_Intersection only BSpline and Bezier surfaces
        // should be enlarged
        if (surf.GetType() == SurfaceKind.BSplineSurface || surf.GetType() == SurfaceKind.BezierSurface)
        {
            // Check U periodicity and closeness
            if (!surf.IsUClosed && !surf.IsUPeriodic)
            {
                // Check that surface is not infinite in U direction
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(surf.FirstUParameter()) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(surf.LastUParameter()))
                {
                    uEnlarge = true;
                }
            }

            // Check V periodicity and closeness
            if (!surf.IsVClosed && !surf.IsVPeriodic)
            {
                // Check that surface is not infinite in V direction
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(surf.FirstVParameter()) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(surf.LastVParameter()))
                {
                    vEnlarge = true;
                }
            }
        }
    }

    // Enlarges the sampling zone of the surface by 1% at each end, where possible
    private static void EnlargeZone(SurfaceAdaptorBase surf, ref double u0, ref double u1, ref double v0, ref double v1)
    {
        IsEnlargePossible(surf, out bool isToEnlargeU, out bool isToEnlargeV);
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
    /// Samples the surface: <paramref name="nbSU"/> x <paramref name="nbSV"/> evenly spaced parameters over its
    /// bounds (enlarged when <paramref name="enlargeZone"/>), into the arrays (resized from 1).
    /// </summary>
    public static void MakeSampling(SurfaceAdaptorBase surf, int nbSU, int nbSV, bool enlargeZone,
                                    NCollection_Array1<double> uPars, NCollection_Array1<double> vPars)
    {
        // Resize arrays
        uPars.Resize(1, nbSU, false);
        vPars.Resize(1, nbSV, false);
        //
        double u0, u1, v0, v1;
        u0 = surf.FirstUParameter();
        u1 = surf.LastUParameter();
        v0 = surf.FirstVParameter();
        v1 = surf.LastVParameter();

        // Enlarge surface intersection zone if necessary
        if (enlargeZone)
        {
            EnlargeZone(surf, ref u0, ref u1, ref v0, ref v1);
        }

        int nbSamplesU1 = nbSU - 1;
        int nbSamplesV1 = nbSV - 1;

        // U step
        double dU = (u1 - u0) / (double)nbSamplesU1;
        // V step
        double dV = (v1 - v0) / (double)nbSamplesV1;

        // Fill arrays
        for (int i = 0; i < nbSU; ++i)
        {
            double u = u0 + i * dU;
            if (i == nbSamplesU1)
            {
                u = u1;
            }

            uPars.SetValue(i + 1, u);
        }
        //
        for (int i = 0; i < nbSV; ++i)
        {
            double v = v0 + i * dV;
            if (i == nbSamplesV1)
            {
                v = v1;
            }

            vPars.SetValue(i + 1, v);
        }
    }

    /// <summary>The deflection tolerance of the sampling: IntCurveSurface's polyhedron's over-estimation.</summary>
    public static double ComputeDeflection(SurfaceAdaptorBase surf, NCollection_Array1<double> uPars,
                                           NCollection_Array1<double> vPars)
    {
        ThePolyhedronOfHInter polyhedron = new(surf, uPars, vPars);
        double deflTol = polyhedron.DeflectionOverEstimation();
        return deflTol;
    }

    /// <summary>Fills the array with the surface's points at the sampling (U major) and its unit normals there (zero
    /// where the normal vanishes).</summary>
    public static void FillArrayOfPointNormal(SurfaceAdaptorBase surf, NCollection_Array1<double> uPars,
                                              NCollection_Array1<double> vPars,
                                              PolyhedronArray<PolyhedronPointNormal> points)
    {
        int nbU = uPars.Length();
        int nbV = vPars.Length();
        int iCnt = 0;
        points.Init(nbU * nbV);
        for (int i = 1; i <= nbU; ++i)
        {
            double u = uPars[i];

            for (int j = 1; j <= nbV; ++j)
            {
                double v = vPars[j];
                // Compute the point
                surf.D1(u, v, out Point3D p, out Vector3D du, out Vector3D dv);
                // Compute normal
                Vector3D vNorm = du.Crossed(dv);
                double length = vNorm.Magnitude;
                if (length > Constants.Resolution())
                {
                    vNorm /= length;
                }
                else
                {
                    vNorm = vNorm.WithCoord(0.0, 0.0, 0.0);
                }

                // Save the pair
                ref PolyhedronPointNormal pn = ref points[iCnt];
                pn.Point = p;
                pn.Normal = vNorm;

                ++iCnt;
            }
        }

        points.SetNbItems(iCnt);
    }
}
