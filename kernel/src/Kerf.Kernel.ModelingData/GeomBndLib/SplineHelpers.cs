// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_SplineHelpers.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>Sampling and pole-hull helpers for spline bounding boxes (OCCT's namespace GeomBndLib_SplineHelpers).</summary>
/// <remarks>
/// OCCT's templates are ported once per instantiation it makes (rules, section 9): <c>FillBox</c> over
/// (Bnd_Box, GeomAdaptor_Curve, gp_Pnt), <c>ReduceSplineBox</c> over (gp_Pnt, Bnd_Box), and <c>CurveBoxOptimal</c>
/// over (GeomAdaptor_Curve, Bnd_Box, gp_Pnt) and (Geom2dAdaptor_Curve, Bnd_Box2d, gp_Pnt2d). The templates
/// <c>BezierCurveBox</c>, <c>BSplineCurveBox</c> and the pole-range overload of <c>ReduceSplineBox</c>, and the
/// function <c>ComputePoleIndexRange</c> that only they call, are never instantiated or called in OCCT V8_0_1 and are
/// not ported; <c>PointOps</c> is written out where it is used.
/// </remarks>
internal static class SplineHelpers
{
    /// <summary>Fill box with sampled points and return maximum deflection.</summary>
    internal static double FillBox(ref Box box, CurveAdaptor curve, double first, double last, int n)
    {
        box.Add(curve.EvalD0(first));
        double p = first;
        double tol = 0.0;
        double dp = last - first;
        if (Math.Abs(dp) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            Point3D p1 = curve.EvalD0(first);
            double step = dp / (2 * n);
            for (int idx = 1; idx <= n; ++idx)
            {
                p += step;
                Point3D p2 = curve.EvalD0(p);
                box.Add(p2);
                p += step;
                Point3D p3 = curve.EvalD0(p);
                box.Add(p3);
                // PointOps<gp_Pnt>::Mid
                Point3D pc = new((p1.XYZ() + p3.XYZ()) / 2.0);
                tol = Max(tol, pc.Distance(p2));
                p1 = p3;
            }
        }
        else
        {
            box.Add(curve.EvalD0(last));
        }
        return tol;
    }

    /// <summary>Reduce box using poles convex hull.</summary>
    internal static void ReduceSplineBox(NCollection_Array1<Point3D> poles, in Box origBox, ref Box reducedBox)
    {
        Box polesBox = new();
        for (int idx = poles.Lower(); idx <= poles.Upper(); ++idx)
        {
            polesBox.Add(poles.Value(idx));
        }

        (double xMin, double xMax, double yMin, double yMax, double zMin, double zMax) = origBox.Get();
        if (!polesBox.IsVoid)
        {
            (double pxMin, double pxMax, double pyMin, double pyMax, double pzMin, double pzMax) = polesBox.Get();
            reducedBox.Update(Max(xMin, pxMin),
                                 Max(yMin, pyMin),
                                 Max(zMin, pzMin),
                                 Min(xMax, pxMax),
                                 Min(yMax, pyMax),
                                 Min(zMax, pzMax));
        }
        else
        {
            reducedBox.Update(xMin, yMin, zMin, xMax, yMax, zMax);
        }
    }

    /// <summary>Compute precise bounding box for a 3D curve using sampling + numerical optimization.</summary>
    internal static Box CurveBoxOptimal(CurveAdaptor adaptor, double u1, double u2, double tol)
    {
        const int nDim = 3;
        int nbSamples = SamplingHelpers.ComputeNbSamplesT(adaptor, u1, u2);

        Span<double> coordMin = stackalloc double[nDim];
        Span<double> coordMax = stackalloc double[nDim];
        Span<double> deflMax = stackalloc double[nDim];
        for (int k = 0; k < nDim; ++k)
        {
            coordMin[k] = RealLast();
            coordMax[k] = -RealLast();
            deflMax[k] = -RealLast();
        }

        double du = (u2 - u1) / (nbSamples - 1);
        double du2 = du / 2.0;
        NCollection_Array1<Xyz> pnts = new(1, nbSamples);
        for (int i = 1, j = 0; i <= nbSamples; i++, j++)
        {
            double u = u1 + j * du;
            Point3D p = adaptor.EvalD0(u);
            pnts[i] = p.XYZ();
            for (int k = 0; k < nDim; ++k)
            {
                if (coordMin[k] > p.Coord(k + 1))
                {
                    coordMin[k] = p.Coord(k + 1);
                }
                if (coordMax[k] < p.Coord(k + 1))
                {
                    coordMax[k] = p.Coord(k + 1);
                }
            }
            if (i > 1)
            {
                Xyz pm = 0.5 * (pnts[i - 1] + pnts[i]);
                Point3D pMid = adaptor.EvalD0(u - du2);
                Xyz d = pMid.XYZ() - pm;
                for (int k = 0; k < nDim; ++k)
                {
                    if (coordMin[k] > pMid.Coord(k + 1))
                    {
                        coordMin[k] = pMid.Coord(k + 1);
                    }
                    if (coordMax[k] < pMid.Coord(k + 1))
                    {
                        coordMax[k] = pMid.Coord(k + 1);
                    }
                    double diff = Math.Abs(d.Coord(k + 1));
                    if (deflMax[k] < diff)
                    {
                        deflMax[k] = diff;
                    }
                }
            }
        }

        double eps = Max(tol, global::Kerf.Engine.Foundation.Precision.Confusion());
        for (int k = 0; k < nDim; ++k)
        {
            double diff = deflMax[k];
            if (diff <= eps)
            {
                continue;
            }
            double cMin = coordMin[k];
            double cMax = coordMax[k];
            for (int i = 1; i <= nbSamples; ++i)
            {
                if (pnts[i].Coord(k + 1) - cMin < diff)
                {
                    double uMin = u1 + Max(0, i - 2) * du;
                    double uMax = u1 + Min(nbSamples - 1, i) * du;
                    double locMin = OptimizationHelpers.AdjustExtrT(adaptor, uMin, uMax, cMin, k + 1, eps, true);
                    if (locMin < cMin)
                    {
                        cMin = locMin;
                    }
                }
                else if (cMax - pnts[i].Coord(k + 1) < diff)
                {
                    double uMin = u1 + Max(0, i - 2) * du;
                    double uMax = u1 + Min(nbSamples - 1, i) * du;
                    double locMax = OptimizationHelpers.AdjustExtrT(adaptor, uMin, uMax, cMax, k + 1, eps, false);
                    if (locMax > cMax)
                    {
                        cMax = locMax;
                    }
                }
            }
            coordMin[k] = cMin;
            coordMax[k] = cMax;
        }

        Box box = new();
        box.Add(new Point3D(coordMin[0], coordMin[1], coordMin[2]));
        box.Add(new Point3D(coordMax[0], coordMax[1], coordMax[2]));
        box.Enlarge(eps);
        return box;
    }

    /// <summary>Compute precise bounding box for a 2D curve using sampling + numerical optimization.</summary>
    internal static Box2d CurveBoxOptimal(CurveAdaptor2D adaptor, double u1, double u2, double tol)
    {
        const int nDim = 2;
        int nbSamples = SamplingHelpers.ComputeNbSamplesT(adaptor, u1, u2);

        Span<double> coordMin = stackalloc double[nDim];
        Span<double> coordMax = stackalloc double[nDim];
        Span<double> deflMax = stackalloc double[nDim];
        for (int k = 0; k < nDim; ++k)
        {
            coordMin[k] = RealLast();
            coordMax[k] = -RealLast();
            deflMax[k] = -RealLast();
        }

        double du = (u2 - u1) / (nbSamples - 1);
        double du2 = du / 2.0;
        NCollection_Array1<Xy> pnts = new(1, nbSamples);
        for (int i = 1, j = 0; i <= nbSamples; i++, j++)
        {
            double u = u1 + j * du;
            Point2D p = adaptor.EvalD0(u);
            pnts[i] = p.XY();
            for (int k = 0; k < nDim; ++k)
            {
                if (coordMin[k] > p.Coord(k + 1))
                {
                    coordMin[k] = p.Coord(k + 1);
                }
                if (coordMax[k] < p.Coord(k + 1))
                {
                    coordMax[k] = p.Coord(k + 1);
                }
            }
            if (i > 1)
            {
                Xy pm = 0.5 * (pnts[i - 1] + pnts[i]);
                Point2D pMid = adaptor.EvalD0(u - du2);
                Xy d = pMid.XY() - pm;
                for (int k = 0; k < nDim; ++k)
                {
                    if (coordMin[k] > pMid.Coord(k + 1))
                    {
                        coordMin[k] = pMid.Coord(k + 1);
                    }
                    if (coordMax[k] < pMid.Coord(k + 1))
                    {
                        coordMax[k] = pMid.Coord(k + 1);
                    }
                    double diff = Math.Abs(d.Coord(k + 1));
                    if (deflMax[k] < diff)
                    {
                        deflMax[k] = diff;
                    }
                }
            }
        }

        double eps = Max(tol, global::Kerf.Engine.Foundation.Precision.Confusion());
        for (int k = 0; k < nDim; ++k)
        {
            double diff = deflMax[k];
            if (diff <= eps)
            {
                continue;
            }
            double cMin = coordMin[k];
            double cMax = coordMax[k];
            for (int i = 1; i <= nbSamples; ++i)
            {
                if (pnts[i].Coord(k + 1) - cMin < diff)
                {
                    double uMin = u1 + Max(0, i - 2) * du;
                    double uMax = u1 + Min(nbSamples - 1, i) * du;
                    double locMin = OptimizationHelpers.AdjustExtrT(adaptor, uMin, uMax, cMin, k + 1, eps, true);
                    if (locMin < cMin)
                    {
                        cMin = locMin;
                    }
                }
                else if (cMax - pnts[i].Coord(k + 1) < diff)
                {
                    double uMin = u1 + Max(0, i - 2) * du;
                    double uMax = u1 + Min(nbSamples - 1, i) * du;
                    double locMax = OptimizationHelpers.AdjustExtrT(adaptor, uMin, uMax, cMax, k + 1, eps, false);
                    if (locMax > cMax)
                    {
                        cMax = locMax;
                    }
                }
            }
            coordMin[k] = cMin;
            coordMax[k] = cMax;
        }

        Box2d box = new();
        box.Add(new Point2D(coordMin[0], coordMin[1]));
        box.Add(new Point2D(coordMax[0], coordMax[1]));
        box.Enlarge(eps);
        return box;
    }
}
