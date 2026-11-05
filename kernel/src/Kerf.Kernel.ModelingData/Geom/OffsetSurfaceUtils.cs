// Port of: src/ModelingData/TKG3d/Geom/Geom_OffsetSurfaceUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.NormalStatus;

namespace Kerf.Geometry;

/// <summary>
/// The point and derivatives of an offset surface from those of its basis surface:
/// P(u, v) = p(u, v) + Offset * N / ||N||, where N = dp/du ^ dp/dv is the normal. Handles the non-singular case
/// (the normal from the cross product) and the singular one (higher derivatives of the normal, and the
/// osculating surfaces of <see cref="OsculatingSurface"/>). An internal C++ namespace in OCCT, used by
/// Geom_OffsetSurface and GeomAdaptor_Surface.
/// </summary>
/// <remarks>
/// OCCT's function templates over the basis surface and the osculating-surface query are generic methods over
/// the basis (decision 212): <see cref="GeomSurfaceBasis"/> wraps a <see cref="Surface"/>
/// (Geom_OffsetSurface's instantiation) and <see cref="AdaptorBasis"/> a <see cref="SurfaceAdaptor"/>
/// (GeomAdaptor_Surface's); the bodies are written once. Both of OCCT's query types end in
/// Geom_OsculatingSurface (Geom_OffsetSurface's query returns false without one), so the query is a nullable
/// <see cref="OsculatingSurface"/>. OCCT's stack buffers (NCollection_LocalArray under an
/// NCollection_Array2) are NCollection_Array2 allocations here (decision 78).
/// </remarks>
internal static class OffsetSurfaceUtils
{
    /// <summary>The evaluation services of a basis surface that the functions use.</summary>
    internal interface IBasisSurface
    {
        void Bounds(out double u1, out double u2, out double v1, out double v2);

        bool IsUPeriodic();

        bool IsVPeriodic();

        Surface.ResD1 EvalD1(double u, double v);

        Surface.ResD2 EvalD2(double u, double v);

        Surface.ResD3 EvalD3(double u, double v);

        Vector3D EvalDN(double u, double v, int nu, int nv);
    }

    /// <summary>A Geom surface as basis (Geom_OffsetSurface).</summary>
    internal readonly struct GeomSurfaceBasis : IBasisSurface
    {
        private readonly Surface _surf;

        public GeomSurfaceBasis(Surface surf) => _surf = surf;

        public void Bounds(out double u1, out double u2, out double v1, out double v2) =>
            _surf.ParameterBounds(out u1, out u2, out v1, out v2);

        public bool IsUPeriodic() => _surf.IsUPeriodic;

        public bool IsVPeriodic() => _surf.IsVPeriodic;

        public Surface.ResD1 EvalD1(double u, double v) => _surf.EvalD1(u, v);

        public Surface.ResD2 EvalD2(double u, double v) => _surf.EvalD2(u, v);

        public Surface.ResD3 EvalD3(double u, double v) => _surf.EvalD3(u, v);

        public Vector3D EvalDN(double u, double v, int nu, int nv) => _surf.EvalDN(u, v, nu, nv);
    }

    /// <summary>A surface adaptor as basis (GeomAdaptor_Surface, bounded by the adaptor's parameter range).</summary>
    internal readonly struct AdaptorBasis : IBasisSurface
    {
        private readonly SurfaceAdaptor _surf;

        public AdaptorBasis(SurfaceAdaptor surf) => _surf = surf;

        public void Bounds(out double u1, out double u2, out double v1, out double v2) =>
            _surf.Bounds(out u1, out u2, out v1, out v2);

        public bool IsUPeriodic() => _surf.IsUPeriodic;

        public bool IsVPeriodic() => _surf.IsVPeriodic;

        public Surface.ResD1 EvalD1(double u, double v) => _surf.EvalD1(u, v);

        public Surface.ResD2 EvalD2(double u, double v) => _surf.EvalD2(u, v);

        public Surface.ResD3 EvalD3(double u, double v) => _surf.EvalD3(u, v);

        public Vector3D EvalDN(double u, double v, int nu, int nv) => _surf.EvalDN(u, v, nu, nv);
    }

    /// <summary>Default tolerance for normal magnitude check.</summary>
    internal const double THE_D1_MAGNITUDE_TOL = 1.0e-9;

    /// <summary>Osculating surface query results.</summary>
    private struct OsculatingInfo
    {
        /// <summary>True if osculating along U direction.</summary>
        public bool AlongU;

        /// <summary>True if osculating along V direction.</summary>
        public bool AlongV;

        /// <summary>True if normal direction should be reversed.</summary>
        public bool IsOpposite;

        /// <summary>The sign factor for offset calculation.</summary>
        public readonly double Sign() => ((AlongU || AlongV) && IsOpposite) ? -1.0 : 1.0;

        /// <summary>True if osculating surface is available.</summary>
        public readonly bool HasOsculating => AlongU || AlongV;
    }

    /// <summary>True if any coordinate is infinite.</summary>
    internal static bool IsInfiniteCoord(in Vector3D vec)
    {
        return global::Kerf.Engine.Foundation.Precision.IsInfinite(vec.X) || global::Kerf.Engine.Foundation.Precision.IsInfinite(vec.Y) || global::Kerf.Engine.Foundation.Precision.IsInfinite(vec.Z);
    }

    /// <summary>True if the normal magnitude is below the tolerance (singular case).</summary>
    internal static bool IsSingular(in Vector3D d1U, in Vector3D d1V, double tol = THE_D1_MAGNITUDE_TOL)
    {
        // Normalize derivatives before normal calculation for stability
        Vector3D aD1U = d1U;
        Vector3D aD1V = d1V;
        double d1UNorm2 = aD1U.SquareMagnitude;
        double d1VNorm2 = aD1V.SquareMagnitude;
        if (d1UNorm2 > 1.0)
        {
            aD1U /= Math.Sqrt(d1UNorm2);
        }
        if (d1VNorm2 > 1.0)
        {
            aD1V /= Math.Sqrt(d1VNorm2);
        }

        Vector3D norm = aD1U.Crossed(aD1V);
        return norm.SquareMagnitude <= tol * tol;
    }

    /// <summary>The normalized normal in the non-singular case; false if singular.</summary>
    internal static bool ComputeNormal(in Vector3D d1U, in Vector3D d1V, out Vector3D normal,
                                       double tol = THE_D1_MAGNITUDE_TOL)
    {
        // Normalize derivatives before normal calculation for stability
        Vector3D aD1U = d1U;
        Vector3D aD1V = d1V;
        double d1UNorm2 = aD1U.SquareMagnitude;
        double d1VNorm2 = aD1V.SquareMagnitude;
        if (d1UNorm2 > 1.0)
        {
            aD1U /= Math.Sqrt(d1UNorm2);
        }
        if (d1VNorm2 > 1.0)
        {
            aD1V /= Math.Sqrt(d1VNorm2);
        }

        normal = aD1U.Crossed(aD1V);
        if (normal.SquareMagnitude <= tol * tol)
        {
            return false;
        }
        normal = normal.Normalized();
        return true;
    }

    /// <summary>dN/du for a non-singular offset surface.</summary>
    internal static Vector3D ComputeDNormalU(in Vector3D d1U, in Vector3D d1V, in Vector3D d2U, in Vector3D d2UV,
                                           in Vector3D normal)
    {
        double scale = (d1U ^ d1V).Dot(normal);

        Vector3D n1U = new Vector3D();
        n1U = n1U.WithX(d2U.Y * d1V.Z + d1U.Y * d2UV.Z - d2U.Z * d1V.Y
                  - d1U.Z * d2UV.Y);
        n1U = n1U.WithY(-(d2U.X * d1V.Z + d1U.X * d2UV.Z - d2U.Z * d1V.X
                    - d1U.Z * d2UV.X));
        n1U = n1U.WithZ(d2U.X * d1V.Y + d1U.X * d2UV.Y - d2U.Y * d1V.X
                  - d1U.Y * d2UV.X);
        double scaleU = n1U.Dot(normal);
        n1U = n1U.Subtracted(scaleU * normal);
        n1U /= scale;

        return n1U;
    }

    /// <summary>dN/dv for a non-singular offset surface.</summary>
    internal static Vector3D ComputeDNormalV(in Vector3D d1U, in Vector3D d1V, in Vector3D d2V, in Vector3D d2UV,
                                           in Vector3D normal)
    {
        double scale = (d1U ^ d1V).Dot(normal);

        Vector3D n1V = new Vector3D();
        n1V = n1V.WithX(d2UV.Y * d1V.Z + d2V.Z * d1U.Y - d2UV.Z * d1V.Y
                  - d2V.Y * d1U.Z);
        n1V = n1V.WithY(-(d2UV.X * d1V.Z + d2V.Z * d1U.X - d2UV.Z * d1V.X
                    - d2V.X * d1U.Z));
        n1V = n1V.WithZ(d2UV.X * d1V.Y + d2V.Y * d1U.X - d2UV.Y * d1V.X
                  - d2V.X * d1U.Y);
        double scaleV = n1V.Dot(normal);
        n1V = n1V.Subtracted(scaleV * normal);
        n1V /= scale;

        return n1V;
    }

    /// <summary>
    /// The offset point in the non-singular case: theValue is the basis point on input. False if singular.
    /// </summary>
    internal static bool CalculateD0(ref Point3D value, in Vector3D d1U, in Vector3D d1V, double offset,
                                     double sign = 1.0)
    {
        if (!ComputeNormal(d1U, d1V, out Vector3D norm))
        {
            return false;
        }
        value = value.WithXYZ(value.XYZ() + offset * sign * norm.XYZ());
        return true;
    }

    /// <summary>
    /// The offset point and first derivatives in the non-singular case: the inputs are the basis surface's.
    /// False if singular.
    /// </summary>
    internal static bool CalculateD1(ref Point3D value, ref Vector3D d1U, ref Vector3D d1V, in Vector3D d2U,
                                     in Vector3D d2V, in Vector3D d2UV, double offset, double sign = 1.0)
    {
        if (!ComputeNormal(d1U, d1V, out Vector3D norm))
        {
            return false;
        }

        // Compute offset point
        value = value.WithXYZ(value.XYZ() + offset * sign * norm.XYZ());

        // Compute normal derivatives
        Vector3D n1U = ComputeDNormalU(d1U, d1V, d2U, d2UV, norm);
        Vector3D n1V = ComputeDNormalV(d1U, d1V, d2V, d2UV, norm);

        d1U += offset * sign * n1U;
        d1V += offset * sign * n1V;

        return true;
    }

    /// <summary>
    /// The derivatives of the normal at singular points (theDerNUV), from the derivatives of the basis surface
    /// (theDerSurf, completed here) and, along a degenerate iso-curve, of the osculating surface.
    /// </summary>
    internal static bool ComputeDerivatives<TBasis>(int maxOrder,
                                                    int minOrder,
                                                    double u,
                                                    double v,
                                                    in TBasis basisSurf,
                                                    int nu,
                                                    int nv,
                                                    bool alongU,
                                                    bool alongV,
                                                    BSplineSurface? oscSurf,
                                                    NCollection_Array2<Vector3D> derNUV,
                                                    NCollection_Array2<Vector3D> derSurf)
        where TBasis : struct, IBasisSurface
    {
        if (alongU || alongV)
        {
            maxOrder = 0;
            NCollection_Array2<Vector3D> derSurfL = new(0, maxOrder + nu + 1, 0, maxOrder + nv + 1);
            switch (minOrder)
            {
                case 1:
                {
                    Surface.ResD1 d1 = oscSurf!.EvalD1(u, v);
                    derSurfL.SetValue(1, 0, d1.D1U);
                    derSurfL.SetValue(0, 1, d1.D1V);
                    break;
                }
                case 2:
                {
                    Surface.ResD2 d2 = oscSurf!.EvalD2(u, v);
                    derSurfL.SetValue(1, 0, d2.D1U);
                    derSurfL.SetValue(0, 1, d2.D1V);
                    derSurfL.SetValue(1, 1, d2.D2UV);
                    derSurfL.SetValue(2, 0, d2.D2U);
                    derSurfL.SetValue(0, 2, d2.D2V);
                    break;
                }
                case 3:
                {
                    Surface.ResD3 d3 = oscSurf!.EvalD3(u, v);
                    derSurfL.SetValue(1, 0, d3.D1U);
                    derSurfL.SetValue(0, 1, d3.D1V);
                    derSurfL.SetValue(1, 1, d3.D2UV);
                    derSurfL.SetValue(2, 0, d3.D2U);
                    derSurfL.SetValue(0, 2, d3.D2V);
                    derSurfL.SetValue(3, 0, d3.D3U);
                    derSurfL.SetValue(2, 1, d3.D3UUV);
                    derSurfL.SetValue(1, 2, d3.D3UVV);
                    derSurfL.SetValue(0, 3, d3.D3V);
                    break;
                }
                default:
                    break;
            }

            if (nu <= nv)
            {
                for (int i = 0; i <= maxOrder + 1 + nu; i++)
                {
                    for (int j = i; j <= maxOrder + nv + 1; j++)
                    {
                        if (i + j > minOrder)
                        {
                            Vector3D oscDN = oscSurf!.EvalDN(u, v, i, j);
                            Vector3D basisDN = basisSurf.EvalDN(u, v, i, j);
                            derSurfL.SetValue(i, j, oscDN);
                            derSurf.SetValue(i, j, basisDN);
                            if (i != j && j <= nu + 1)
                            {
                                Vector3D basisDNji = basisSurf.EvalDN(u, v, j, i);
                                Vector3D oscDNji = oscSurf.EvalDN(u, v, j, i);
                                derSurf.SetValue(j, i, basisDNji);
                                derSurfL.SetValue(j, i, oscDNji);
                            }
                        }
                    }
                }
            }
            else
            {
                for (int j = 0; j <= maxOrder + 1 + nv; j++)
                {
                    for (int i = j; i <= maxOrder + nu + 1; i++)
                    {
                        if (i + j > minOrder)
                        {
                            Vector3D oscDN = oscSurf!.EvalDN(u, v, i, j);
                            Vector3D basisDN = basisSurf.EvalDN(u, v, i, j);
                            derSurfL.SetValue(i, j, oscDN);
                            derSurf.SetValue(i, j, basisDN);
                            if (i != j && i <= nv + 1)
                            {
                                Vector3D basisDNji = basisSurf.EvalDN(u, v, j, i);
                                Vector3D oscDNji = oscSurf.EvalDN(u, v, j, i);
                                derSurf.SetValue(j, i, basisDNji);
                                derSurfL.SetValue(j, i, oscDNji);
                            }
                        }
                    }
                }
            }
            for (int i = 0; i <= maxOrder + nu; i++)
            {
                for (int j = 0; j <= maxOrder + nv; j++)
                {
                    if (alongU)
                    {
                        derNUV.SetValue(i, j, global::Kerf.Engine.Geometry.CSLib.DNNUV(i, j, derSurfL, derSurf));
                    }
                    if (alongV)
                    {
                        derNUV.SetValue(i, j, global::Kerf.Engine.Geometry.CSLib.DNNUV(i, j, derSurf, derSurfL));
                    }
                }
            }
        }
        else
        {
            for (int i = 0; i <= maxOrder + nu + 1; i++)
            {
                for (int j = i; j <= maxOrder + nv + 1; j++)
                {
                    if (i + j > minOrder)
                    {
                        Vector3D dn = basisSurf.EvalDN(u, v, i, j);
                        derSurf.SetValue(i, j, dn);
                        if (i != j && j <= derSurf.UpperRow() && i <= derSurf.UpperCol())
                        {
                            Vector3D dNji = basisSurf.EvalDN(u, v, j, i);
                            derSurf.SetValue(j, i, dNji);
                        }
                    }
                }
            }
            for (int i = 0; i <= maxOrder + nu; i++)
            {
                for (int j = 0; j <= maxOrder + nv; j++)
                {
                    derNUV.SetValue(i, j, global::Kerf.Engine.Geometry.CSLib.DNNUV(i, j, derSurf));
                }
            }
        }
        return true;
    }

    /// <summary>
    /// Replaces a zero derivative (CSLib_InfinityOfSolutions: exactly one of theDU, theDV is zero) by the one
    /// evaluated a small step away. True if replaced.
    /// </summary>
    internal static bool ReplaceDerivative<TBasis>(double u,
                                                   double v,
                                                   double uMin,
                                                   double uMax,
                                                   double vMin,
                                                   double vMax,
                                                   ref Vector3D du,
                                                   ref Vector3D dv,
                                                   double squareTol,
                                                   in TBasis basisSurf)
        where TBasis : struct, IBasisSurface
    {
        bool isReplaceDU = du.SquareMagnitude < squareTol;
        bool isReplaceDV = dv.SquareMagnitude < squareTol;
        bool isReplaced = false;

        // Only handle case where exactly one derivative is zero
        if (isReplaceDU != isReplaceDV)
        {
            // Calculate step along non-zero derivative
            double step;
            if (isReplaceDV)
            {
                step = global::Kerf.Engine.Foundation.Precision.Confusion() * du.Magnitude;
                if (step > uMax - uMin)
                {
                    step = (uMax - uMin) / 100.0;
                }
            }
            else
            {
                step = global::Kerf.Engine.Foundation.Precision.Confusion() * dv.Magnitude;
                if (step > vMax - vMin)
                {
                    step = (vMax - vMin) / 100.0;
                }
            }

            // Step away from current parametric coordinates and calculate derivatives once again.
            // Replace zero derivative by the obtained.
            for (double stepSign = -1.0; stepSign <= 1.0 && !isReplaced; stepSign += 2.0)
            {
                double aU = u;
                double aV = v;

                if (isReplaceDV)
                {
                    aU = u + stepSign * step;
                    if (aU < uMin || aU > uMax)
                    {
                        continue;
                    }
                }
                else
                {
                    aV = v + stepSign * step;
                    if (aV < vMin || aV > vMax)
                    {
                        continue;
                    }
                }

                Surface.ResD1 d1Result = basisSurf.EvalD1(aU, aV);

                if (isReplaceDU && d1Result.D1U.SquareMagnitude > squareTol)
                {
                    du = d1Result.D1U;
                    isReplaced = true;
                }
                if (isReplaceDV && d1Result.D1V.SquareMagnitude > squareTol)
                {
                    dv = d1Result.D1V;
                    isReplaced = true;
                }
            }
        }
        return isReplaced;
    }

    /// <summary>
    /// Shifts the evaluation point towards the center of the parametric space (each time doubling its distance
    /// from the start point), for singular points near the bounds. False when the center is reached.
    /// </summary>
    internal static bool ShiftPoint(double uStart,
                                    double vStart,
                                    ref double u,
                                    ref double v,
                                    double uMin,
                                    double uMax,
                                    double vMin,
                                    double vMax,
                                    bool isUPeriodic,
                                    bool isVPeriodic,
                                    in Vector3D d1U,
                                    in Vector3D d1V)
    {
        // Check if either U or V is singular (normally one of them is)
        bool isUSingular = (d1U.SquareMagnitude < THE_D1_MAGNITUDE_TOL * THE_D1_MAGNITUDE_TOL);
        bool isVSingular = (d1V.SquareMagnitude < THE_D1_MAGNITUDE_TOL * THE_D1_MAGNITUDE_TOL);

        // Compute vector to shift from start point to center of the surface;
        // if surface is periodic or singular in some direction, take shift in that direction zero
        double dirU =
            (isUPeriodic || (isUSingular && !isVSingular) ? 0.0 : 0.5 * (uMin + uMax) - uStart);
        double dirV =
            (isVPeriodic || (isVSingular && !isUSingular) ? 0.0 : 0.5 * (vMin + vMax) - vStart);
        double dist = Math.Sqrt(dirU * dirU + dirV * dirV);

        // Shift current point from its current position towards center, by value of twice
        // current distance from it to start (but not less than Precision::PConfusion());
        // fail if center is overpassed.
        double du = u - uStart;
        double dv = v - vStart;
        double step = Math.Max(2.0 * Math.Sqrt(du * du + dv * dv), global::Kerf.Engine.Foundation.Precision.PConfusion());
        if (step >= dist)
        {
            return false;
        }

        step /= dist;
        u += dirU * step;
        v += dirV * step;

        return true;
    }

    // Queries the osculating surfaces at (U, V) as OCCT's OscSurfQueryType does.
    private static void QueryOsculating(OsculatingSurface? oscQuery, double u, double v,
                                        ref OsculatingInfo info, ref BSplineSurface? oscSurf)
    {
        if (oscQuery is not null)
        {
            info.AlongU = oscQuery.UOsculatingSurface(u, v, ref info.IsOpposite, ref oscSurf);
            info.AlongV = oscQuery.VOsculatingSurface(u, v, ref info.IsOpposite, ref oscSurf);
        }
    }

    /// <summary>
    /// The offset point from the basis point (theValue on input) and first derivatives at (theU, theV); at singular
    /// points, retries closer to the center of the parameter space. False if it fails.
    /// </summary>
    internal static bool EvaluateD0<TBasis>(double u,
                                            double v,
                                            in TBasis basisSurf,
                                            double offset,
                                            OsculatingSurface? oscQuery,
                                            ref Point3D value,
                                            in Vector3D d1U,
                                            in Vector3D d1V)
        where TBasis : struct, IBasisSurface
    {
        double uStart = u;
        double vStart = v;
        basisSurf.Bounds(out double uMin, out double uMax, out double vMin, out double vMax);
        bool isUPer = basisSurf.IsUPeriodic();
        bool isVPer = basisSurf.IsVPeriodic();

        // Use pre-computed D1 for first iteration
        Vector3D aD1U = d1U;
        Vector3D aD1V = d1V;
        bool isFirstIteration = true;

        for (;;)
        {
            // For subsequent iterations, recompute D1 at shifted point
            if (!isFirstIteration)
            {
                Surface.ResD1 d1Result = basisSurf.EvalD1(u, v);
                value = d1Result.Point;
                aD1U = d1Result.D1U;
                aD1V = d1Result.D1V;
            }
            isFirstIteration = false;

            if (IsInfiniteCoord(aD1U) || IsInfiniteCoord(aD1V))
            {
                return false;
            }

            // Try non-singular case first
            if (CalculateD0(ref value, aD1U, aD1V, offset))
            {
                return true;
            }

            // Singular case - query osculating surface and use higher order derivatives
            const int maxOrder = 3;

            OsculatingInfo oscInfo = default;
            BSplineSurface? oscSurf = null;
            QueryOsculating(oscQuery, u, v, ref oscInfo, ref oscSurf);

            NCollection_Array2<Vector3D> derNUV = new(0, maxOrder, 0, maxOrder);
            NCollection_Array2<Vector3D> derSurf = new(0, maxOrder + 1, 0, maxOrder + 1);

            derSurf.SetValue(1, 0, aD1U);
            derSurf.SetValue(0, 1, aD1V);

            // Use ComputeDerivatives which handles osculating surface properly
            if (oscInfo.HasOsculating && oscSurf is not null)
            {
                if (!ComputeDerivatives(maxOrder, 1, u, v, basisSurf, 0, 0, oscInfo.AlongU,
                                        oscInfo.AlongV, oscSurf, derNUV, derSurf))
                {
                    return false;
                }
            }
            else
            {
                if (!ComputeDerivatives(maxOrder, 1, u, v, basisSurf, 0, 0, false, false, null, derNUV,
                                        derSurf))
                {
                    return false;
                }
            }

            Direction3D normal = new Direction3D();
            global::Kerf.Engine.Geometry.CSLib.Normal(maxOrder,
                         derNUV,
                         THE_D1_MAGNITUDE_TOL,
                         u,
                         v,
                         uMin,
                         uMax,
                         vMin,
                         vMax,
                         out NormalStatus nStatus,
                         ref normal,
                         out _,
                         out _);

            // Handle CSLib_InfinityOfSolutions by replacing zero derivative
            if (nStatus == InfinityOfSolutions)
            {
                Vector3D newDU = aD1U;
                Vector3D newDV = aD1V;
                if (ReplaceDerivative(u,
                                      v,
                                      uMin,
                                      uMax,
                                      vMin,
                                      vMax,
                                      ref newDU,
                                      ref newDV,
                                      THE_D1_MAGNITUDE_TOL * THE_D1_MAGNITUDE_TOL,
                                      basisSurf))
                {
                    global::Kerf.Engine.Geometry.CSLib.Normal(newDU, newDV, THE_D1_MAGNITUDE_TOL, out nStatus, ref normal);
                }
            }

            if (nStatus == Defined)
            {
                value = value.WithXYZ(value.XYZ() + offset * oscInfo.Sign() * normal.XYZ());
                return true;
            }

            // Try shifting point towards center - returns false when center is overpassed
            if (!ShiftPoint(uStart, vStart, ref u, ref v, uMin, uMax, vMin, vMax, isUPer, isVPer, aD1U,
                            aD1V))
            {
                return false;
            }
        }
    }

    /// <summary>The offset point at (theU, theV); false if it fails at a singular point.</summary>
    internal static bool EvaluateD0<TBasis>(double u,
                                            double v,
                                            in TBasis basisSurf,
                                            double offset,
                                            OsculatingSurface? oscQuery,
                                            out Point3D value)
        where TBasis : struct, IBasisSurface
    {
        Surface.ResD1 basisD1 = basisSurf.EvalD1(u, v);
        value = basisD1.Point;
        return EvaluateD0(u, v, basisSurf, offset, oscQuery, ref value, basisD1.D1U, basisD1.D1V);
    }

    /// <summary>
    /// The offset point and first derivatives from the basis point and first derivatives (on input) and second
    /// derivatives; at singular points, retries closer to the center. False if it fails.
    /// </summary>
    internal static bool EvaluateD1<TBasis>(double u,
                                            double v,
                                            in TBasis basisSurf,
                                            double offset,
                                            OsculatingSurface? oscQuery,
                                            ref Point3D value,
                                            ref Vector3D d1U,
                                            ref Vector3D d1V,
                                            in Vector3D d2U,
                                            in Vector3D d2V,
                                            in Vector3D d2UV)
        where TBasis : struct, IBasisSurface
    {
        double uStart = u;
        double vStart = v;
        basisSurf.Bounds(out double uMin, out double uMax, out double vMin, out double vMax);
        bool isUPer = basisSurf.IsUPeriodic();
        bool isVPer = basisSurf.IsVPeriodic();

        // Use pre-computed D2 for first iteration
        Vector3D aD2U = d2U;
        Vector3D aD2V = d2V;
        Vector3D aD2UV = d2UV;
        bool isFirstIteration = true;

        for (;;)
        {
            // For subsequent iterations, recompute D2 at shifted point
            if (!isFirstIteration)
            {
                Surface.ResD2 d2Result = basisSurf.EvalD2(u, v);
                value = d2Result.Point;
                d1U = d2Result.D1U;
                d1V = d2Result.D1V;
                aD2U = d2Result.D2U;
                aD2V = d2Result.D2V;
                aD2UV = d2Result.D2UV;
            }
            isFirstIteration = false;

            if (IsInfiniteCoord(d1U) || IsInfiniteCoord(d1V))
            {
                return false;
            }

            // Check if singular by normalizing derivatives and computing cross product
            Vector3D aD1U = d1U;
            Vector3D aD1V = d1V;
            double d1UNorm2 = aD1U.SquareMagnitude;
            double d1VNorm2 = aD1V.SquareMagnitude;
            if (d1UNorm2 > 1.0)
            {
                aD1U /= Math.Sqrt(d1UNorm2);
            }
            if (d1VNorm2 > 1.0)
            {
                aD1V /= Math.Sqrt(d1VNorm2);
            }

            bool isSingular = false;
            const int maxOrder = 3;
            Vector3D norm = aD1U.Crossed(aD1V);

            // Query osculating surface only if singular
            OsculatingInfo oscInfo = default;
            BSplineSurface? oscSurf = null;
            if (norm.SquareMagnitude <= THE_D1_MAGNITUDE_TOL * THE_D1_MAGNITUDE_TOL)
            {
                QueryOsculating(oscQuery, u, v, ref oscInfo, ref oscSurf);
                isSingular = true;
            }

            // Compute sign factor
            double sign = oscInfo.Sign();

            // Non-singular case: use direct formulas
            if (!isSingular)
            {
                norm = norm.Normalized();
                value = value.WithXYZ(value.XYZ() + offset * sign * norm.XYZ());

                // Compute normal derivatives using inline formulas
                Vector3D n0 = new Vector3D(norm.XYZ()), n1U = new Vector3D(), n1V = new Vector3D();
                double scale = (d1U ^ d1V).Dot(n0);
                n1U = n1U.WithX(aD2U.Y * d1V.Z + d1U.Y * aD2UV.Z - aD2U.Z * d1V.Y
                          - d1U.Z * aD2UV.Y);
                n1U = n1U.WithY((aD2U.X * d1V.Z + d1U.X * aD2UV.Z - aD2U.Z * d1V.X
                           - d1U.Z * aD2UV.X)
                          * -1.0);
                n1U = n1U.WithZ(aD2U.X * d1V.Y + d1U.X * aD2UV.Y - aD2U.Y * d1V.X
                          - d1U.Y * aD2UV.X);
                double scaleU = n1U.Dot(n0);
                n1U = n1U.Subtracted(scaleU * n0);
                n1U /= scale;

                n1V = n1V.WithX(aD2UV.Y * d1V.Z + aD2V.Z * d1U.Y - aD2UV.Z * d1V.Y
                          - aD2V.Y * d1U.Z);
                n1V = n1V.WithY((aD2UV.X * d1V.Z + aD2V.Z * d1U.X - aD2UV.Z * d1V.X
                           - aD2V.X * d1U.Z)
                          * -1.0);
                n1V = n1V.WithZ(aD2UV.X * d1V.Y + aD2V.Y * d1U.X - aD2UV.Y * d1V.X
                          - aD2V.X * d1U.Y);
                double scaleV = n1V.Dot(n0);
                n1V = n1V.Subtracted(scaleV * n0);
                n1V /= scale;

                d1U += offset * sign * n1U;
                d1V += offset * sign * n1V;

                return true;
            }

            // Singular case - use higher order derivatives
            NCollection_Array2<Vector3D> derNUV = new(0, maxOrder + 1, 0, maxOrder + 1);
            NCollection_Array2<Vector3D> derSurf = new(0, maxOrder + 2, 0, maxOrder + 2);

            derSurf.SetValue(1, 0, d1U);
            derSurf.SetValue(0, 1, d1V);
            derSurf.SetValue(1, 1, aD2UV);
            derSurf.SetValue(2, 0, aD2U);
            derSurf.SetValue(0, 2, aD2V);

            if (oscInfo.HasOsculating && oscSurf is not null)
            {
                if (!ComputeDerivatives(maxOrder, 2, u, v, basisSurf, 1, 1, oscInfo.AlongU,
                                        oscInfo.AlongV, oscSurf, derNUV, derSurf))
                {
                    return false;
                }
            }
            else
            {
                if (!ComputeDerivatives(maxOrder, 2, u, v, basisSurf, 1, 1, false, false, null, derNUV,
                                        derSurf))
                {
                    return false;
                }
            }

            Direction3D normal = new Direction3D();
            global::Kerf.Engine.Geometry.CSLib.Normal(maxOrder,
                         derNUV,
                         THE_D1_MAGNITUDE_TOL,
                         u,
                         v,
                         uMin,
                         uMax,
                         vMin,
                         vMax,
                         out NormalStatus nStatus,
                         ref normal,
                         out int orderU,
                         out int orderV);

            // Handle CSLib_InfinityOfSolutions by replacing zero derivative
            if (nStatus == InfinityOfSolutions)
            {
                Vector3D newDU = d1U;
                Vector3D newDV = d1V;
                if (ReplaceDerivative(u,
                                      v,
                                      uMin,
                                      uMax,
                                      vMin,
                                      vMax,
                                      ref newDU,
                                      ref newDV,
                                      THE_D1_MAGNITUDE_TOL * THE_D1_MAGNITUDE_TOL,
                                      basisSurf))
                {
                    // Re-compute with replaced derivatives
                    derSurf.SetValue(1, 0, newDU);
                    derSurf.SetValue(0, 1, newDV);
                    if (oscInfo.HasOsculating && oscSurf is not null)
                    {
                        if (!ComputeDerivatives(maxOrder, 2, u, v, basisSurf, 1, 1, oscInfo.AlongU,
                                                oscInfo.AlongV, oscSurf, derNUV, derSurf))
                        {
                            return false;
                        }
                    }
                    else
                    {
                        if (!ComputeDerivatives(maxOrder, 2, u, v, basisSurf, 1, 1, false, false, null,
                                                derNUV, derSurf))
                        {
                            return false;
                        }
                    }
                    global::Kerf.Engine.Geometry.CSLib.Normal(maxOrder,
                                 derNUV,
                                 THE_D1_MAGNITUDE_TOL,
                                 u,
                                 v,
                                 uMin,
                                 uMax,
                                 vMin,
                                 vMax,
                                 out nStatus,
                                 ref normal,
                                 out orderU,
                                 out orderV);
                }
            }

            if (nStatus == Defined)
            {
                // Compute offset point
                value = value.WithXYZ(value.XYZ() + offset * sign * normal.XYZ());
                // Compute D1 using CSLib
                d1U = global::Kerf.Engine.Geometry.CSLib.DNNormal(1, 0, derNUV, orderU, orderV);
                d1V = global::Kerf.Engine.Geometry.CSLib.DNNormal(0, 1, derNUV, orderU, orderV);

                d1U = d1U.Multiplied(offset * sign);
                d1U = d1U.Added(derSurf[1, 0]);
                d1V = d1V.Multiplied(offset * sign);
                d1V = d1V.Added(derSurf[0, 1]);
                return true;
            }

            // Try shifting point towards center - returns false when center is overpassed
            if (!ShiftPoint(uStart, vStart, ref u, ref v, uMin, uMax, vMin, vMax, isUPer, isVPer, d1U,
                            d1V))
            {
                return false;
            }
        }
    }

    /// <summary>The offset point and first derivatives at (theU, theV); false if it fails at a singular point.</summary>
    internal static bool EvaluateD1<TBasis>(double u,
                                            double v,
                                            in TBasis basisSurf,
                                            double offset,
                                            OsculatingSurface? oscQuery,
                                            out Point3D value,
                                            out Vector3D d1U,
                                            out Vector3D d1V)
        where TBasis : struct, IBasisSurface
    {
        Surface.ResD2 basisD2 = basisSurf.EvalD2(u, v);
        value = basisD2.Point;
        d1U = basisD2.D1U;
        d1V = basisD2.D1V;
        return EvaluateD1(u,
                          v,
                          basisSurf,
                          offset,
                          oscQuery,
                          ref value,
                          ref d1U,
                          ref d1V,
                          basisD2.D2U,
                          basisD2.D2V,
                          basisD2.D2UV);
    }

    /// <summary>
    /// The offset point and derivatives up to the second order from the basis ones (on input) and third
    /// derivatives; at singular points, retries closer to the center. False if it fails.
    /// </summary>
    internal static bool EvaluateD2<TBasis>(double u,
                                            double v,
                                            in TBasis basisSurf,
                                            double offset,
                                            OsculatingSurface? oscQuery,
                                            ref Point3D value,
                                            ref Vector3D d1U,
                                            ref Vector3D d1V,
                                            ref Vector3D d2U,
                                            ref Vector3D d2V,
                                            ref Vector3D d2UV,
                                            in Vector3D d3U,
                                            in Vector3D d3V,
                                            in Vector3D d3UUV,
                                            in Vector3D d3UVV)
        where TBasis : struct, IBasisSurface
    {
        double uStart = u;
        double vStart = v;
        basisSurf.Bounds(out double uMin, out double uMax, out double vMin, out double vMax);
        bool isUPer = basisSurf.IsUPeriodic();
        bool isVPer = basisSurf.IsVPeriodic();

        // Use pre-computed D3 for first iteration
        Vector3D aD3U = d3U;
        Vector3D aD3V = d3V;
        Vector3D aD3UUV = d3UUV;
        Vector3D aD3UVV = d3UVV;
        bool isFirstIteration = true;

        for (;;)
        {
            // For subsequent iterations, recompute D3 at shifted point
            if (!isFirstIteration)
            {
                Surface.ResD3 d3Result = basisSurf.EvalD3(u, v);
                value = d3Result.Point;
                d1U = d3Result.D1U;
                d1V = d3Result.D1V;
                d2U = d3Result.D2U;
                d2V = d3Result.D2V;
                d2UV = d3Result.D2UV;
                aD3U = d3Result.D3U;
                aD3V = d3Result.D3V;
                aD3UUV = d3Result.D3UUV;
                aD3UVV = d3Result.D3UVV;
            }
            isFirstIteration = false;

            if (IsInfiniteCoord(d1U) || IsInfiniteCoord(d1V))
            {
                return false;
            }

            // Check if singular using CSLib::Normal
            Direction3D normal = new Direction3D();
            global::Kerf.Engine.Geometry.CSLib.Normal(d1U, d1V, THE_D1_MAGNITUDE_TOL, out NormalStatus nStatus, ref normal);

            // MaxOrder = 0 for non-singular, 3 for singular
            int maxOrder = (nStatus == Defined) ? 0 : 3;

            // Get osculating surface info
            OsculatingInfo oscInfo = default;
            BSplineSurface? oscSurf = null;
            if (nStatus != Defined)
            {
                QueryOsculating(oscQuery, u, v, ref oscInfo, ref oscSurf);
            }

            // Setup derivative arrays
            NCollection_Array2<Vector3D> derNUV = new(0, maxOrder + 2, 0, maxOrder + 2);
            NCollection_Array2<Vector3D> derSurf = new(0, maxOrder + 3, 0, maxOrder + 3);

            derSurf.SetValue(1, 0, d1U);
            derSurf.SetValue(0, 1, d1V);
            derSurf.SetValue(1, 1, d2UV);
            derSurf.SetValue(2, 0, d2U);
            derSurf.SetValue(0, 2, d2V);
            derSurf.SetValue(3, 0, aD3U);
            derSurf.SetValue(2, 1, aD3UUV);
            derSurf.SetValue(1, 2, aD3UVV);
            derSurf.SetValue(0, 3, aD3V);

            // Use ComputeDerivatives to populate DerNUV
            if (oscInfo.HasOsculating && oscSurf is not null)
            {
                if (!ComputeDerivatives(maxOrder, 3, u, v, basisSurf, 2, 2, oscInfo.AlongU,
                                        oscInfo.AlongV, oscSurf, derNUV, derSurf))
                {
                    return false;
                }
            }
            else
            {
                if (!ComputeDerivatives(maxOrder, 3, u, v, basisSurf, 2, 2, false, false, null, derNUV,
                                        derSurf))
                {
                    return false;
                }
            }

            // Compute normal using CSLib (second call with MaxOrder)
            global::Kerf.Engine.Geometry.CSLib.Normal(maxOrder,
                         derNUV,
                         THE_D1_MAGNITUDE_TOL,
                         u,
                         v,
                         uMin,
                         uMax,
                         vMin,
                         vMax,
                         out nStatus,
                         ref normal,
                         out int orderU,
                         out int orderV);

            if (nStatus == Defined)
            {
                double sign = offset * oscInfo.Sign();

                // Compute offset point
                value = value.WithXYZ(value.XYZ() + sign * normal.XYZ());

                // Compute D1 using CSLib::DNNormal
                d1U = derSurf[1, 0].Added(global::Kerf.Engine.Geometry.CSLib.DNNormal(1, 0, derNUV, orderU, orderV).Multiplied(sign));
                d1V = derSurf[0, 1].Added(global::Kerf.Engine.Geometry.CSLib.DNNormal(0, 1, derNUV, orderU, orderV).Multiplied(sign));

                // For D2, re-fetch from basis surface
                Vector3D dn20 = basisSurf.EvalDN(u, v, 2, 0);
                Vector3D dn02 = basisSurf.EvalDN(u, v, 0, 2);
                Vector3D dn11 = basisSurf.EvalDN(u, v, 1, 1);
                d2U = dn20;
                d2V = dn02;
                d2UV = dn11;

                // Add offset corrections using CSLib::DNNormal
                d2U = d2U.Added(global::Kerf.Engine.Geometry.CSLib.DNNormal(2, 0, derNUV, orderU, orderV).Multiplied(sign));
                d2V = d2V.Added(global::Kerf.Engine.Geometry.CSLib.DNNormal(0, 2, derNUV, orderU, orderV).Multiplied(sign));
                d2UV = d2UV.Added(global::Kerf.Engine.Geometry.CSLib.DNNormal(1, 1, derNUV, orderU, orderV).Multiplied(sign));
                return true;
            }

            // Try shifting point towards center - returns false when center is overpassed
            if (!ShiftPoint(uStart, vStart, ref u, ref v, uMin, uMax, vMin, vMax, isUPer, isVPer, d1U,
                            d1V))
            {
                return false;
            }
        }
    }

    /// <summary>
    /// The offset point and derivatives up to the second order at (theU, theV); false if it fails at a singular
    /// point.
    /// </summary>
    internal static bool EvaluateD2<TBasis>(double u,
                                            double v,
                                            in TBasis basisSurf,
                                            double offset,
                                            OsculatingSurface? oscQuery,
                                            out Point3D value,
                                            out Vector3D d1U,
                                            out Vector3D d1V,
                                            out Vector3D d2U,
                                            out Vector3D d2V,
                                            out Vector3D d2UV)
        where TBasis : struct, IBasisSurface
    {
        Surface.ResD3 basisD3 = basisSurf.EvalD3(u, v);
        value = basisD3.Point;
        d1U = basisD3.D1U;
        d1V = basisD3.D1V;
        d2U = basisD3.D2U;
        d2V = basisD3.D2V;
        d2UV = basisD3.D2UV;
        return EvaluateD2(u,
                          v,
                          basisSurf,
                          offset,
                          oscQuery,
                          ref value,
                          ref d1U,
                          ref d1V,
                          ref d2U,
                          ref d2V,
                          ref d2UV,
                          basisD3.D3U,
                          basisD3.D3V,
                          basisD3.D3UUV,
                          basisD3.D3UVV);
    }

    /// <summary>
    /// The offset point and derivatives up to the third order at (theU, theV); at singular points, retries closer
    /// to the center. False if it fails.
    /// </summary>
    internal static bool EvaluateD3<TBasis>(double u,
                                            double v,
                                            in TBasis basisSurf,
                                            double offset,
                                            OsculatingSurface? oscQuery,
                                            out Point3D value,
                                            out Vector3D d1U,
                                            out Vector3D d1V,
                                            out Vector3D d2U,
                                            out Vector3D d2V,
                                            out Vector3D d2UV,
                                            out Vector3D d3U,
                                            out Vector3D d3V,
                                            out Vector3D d3UUV,
                                            out Vector3D d3UVV)
        where TBasis : struct, IBasisSurface
    {
        double uStart = u;
        double vStart = v;
        basisSurf.Bounds(out double uMin, out double uMax, out double vMin, out double vMax);
        bool isUPer = basisSurf.IsUPeriodic();
        bool isVPer = basisSurf.IsVPeriodic();

        for (;;)
        {
            Surface.ResD3 d3Result = basisSurf.EvalD3(u, v);
            value = d3Result.Point;
            d1U = d3Result.D1U;
            d1V = d3Result.D1V;
            d2U = d3Result.D2U;
            d2V = d3Result.D2V;
            d2UV = d3Result.D2UV;
            d3U = d3Result.D3U;
            d3V = d3Result.D3V;
            d3UUV = d3Result.D3UUV;
            d3UVV = d3Result.D3UVV;

            if (IsInfiniteCoord(d1U) || IsInfiniteCoord(d1V))
            {
                return false;
            }

            // Check if singular using CSLib::Normal
            Direction3D normal = new Direction3D();
            global::Kerf.Engine.Geometry.CSLib.Normal(d1U, d1V, THE_D1_MAGNITUDE_TOL, out NormalStatus nStatus, ref normal);

            // MaxOrder = 0 for non-singular, 3 for singular
            int maxOrder = (nStatus == Defined) ? 0 : 3;

            // Get osculating surface info
            OsculatingInfo oscInfo = default;
            BSplineSurface? oscSurf = null;
            if (nStatus != Defined)
            {
                QueryOsculating(oscQuery, u, v, ref oscInfo, ref oscSurf);
            }

            // Setup derivative arrays
            // For D3: DerNUV needs (aMaxOrder + 4), DerSurf needs (aMaxOrder + 5)
            NCollection_Array2<Vector3D> derNUV = new(0, maxOrder + 3, 0, maxOrder + 3);
            NCollection_Array2<Vector3D> derSurf = new(0, maxOrder + 4, 0, maxOrder + 4);

            derSurf.SetValue(1, 0, d1U);
            derSurf.SetValue(0, 1, d1V);
            derSurf.SetValue(1, 1, d2UV);
            derSurf.SetValue(2, 0, d2U);
            derSurf.SetValue(0, 2, d2V);
            derSurf.SetValue(3, 0, d3U);
            derSurf.SetValue(2, 1, d3UUV);
            derSurf.SetValue(1, 2, d3UVV);
            derSurf.SetValue(0, 3, d3V);

            // Use ComputeDerivatives to populate DerNUV
            if (oscInfo.HasOsculating && oscSurf is not null)
            {
                if (!ComputeDerivatives(maxOrder, 3, u, v, basisSurf, 3, 3, oscInfo.AlongU,
                                        oscInfo.AlongV, oscSurf, derNUV, derSurf))
                {
                    return false;
                }
            }
            else
            {
                if (!ComputeDerivatives(maxOrder, 3, u, v, basisSurf, 3, 3, false, false, null, derNUV,
                                        derSurf))
                {
                    return false;
                }
            }

            // Compute normal using CSLib (second call with MaxOrder)
            global::Kerf.Engine.Geometry.CSLib.Normal(maxOrder,
                         derNUV,
                         THE_D1_MAGNITUDE_TOL,
                         u,
                         v,
                         uMin,
                         uMax,
                         vMin,
                         vMax,
                         out nStatus,
                         ref normal,
                         out int orderU,
                         out int orderV);

            if (nStatus == Defined)
            {
                double sign = offset * oscInfo.Sign();

                // Compute offset point
                value = value.WithXYZ(value.XYZ() + sign * normal.XYZ());

                // Compute D1 using CSLib::DNNormal
                d1U = derSurf[1, 0].Added(global::Kerf.Engine.Geometry.CSLib.DNNormal(1, 0, derNUV, orderU, orderV).Multiplied(sign));
                d1V = derSurf[0, 1].Added(global::Kerf.Engine.Geometry.CSLib.DNNormal(0, 1, derNUV, orderU, orderV).Multiplied(sign));

                // For D2 and D3, re-fetch from basis surface
                Vector3D dn20 = basisSurf.EvalDN(u, v, 2, 0);
                Vector3D dn02 = basisSurf.EvalDN(u, v, 0, 2);
                Vector3D dn11 = basisSurf.EvalDN(u, v, 1, 1);
                Vector3D dn30 = basisSurf.EvalDN(u, v, 3, 0);
                Vector3D dn03 = basisSurf.EvalDN(u, v, 0, 3);
                Vector3D dn21 = basisSurf.EvalDN(u, v, 2, 1);
                Vector3D dn12 = basisSurf.EvalDN(u, v, 1, 2);
                d2U = dn20;
                d2V = dn02;
                d2UV = dn11;
                d3U = dn30;
                d3V = dn03;
                d3UUV = dn21;
                d3UVV = dn12;

                // Add offset corrections using CSLib::DNNormal
                d2U = d2U.Added(global::Kerf.Engine.Geometry.CSLib.DNNormal(2, 0, derNUV, orderU, orderV).Multiplied(sign));
                d2V = d2V.Added(global::Kerf.Engine.Geometry.CSLib.DNNormal(0, 2, derNUV, orderU, orderV).Multiplied(sign));
                d2UV = d2UV.Added(global::Kerf.Engine.Geometry.CSLib.DNNormal(1, 1, derNUV, orderU, orderV).Multiplied(sign));
                d3U = d3U.Added(global::Kerf.Engine.Geometry.CSLib.DNNormal(3, 0, derNUV, orderU, orderV).Multiplied(sign));
                d3V = d3V.Added(global::Kerf.Engine.Geometry.CSLib.DNNormal(0, 3, derNUV, orderU, orderV).Multiplied(sign));
                d3UUV = d3UUV.Added(global::Kerf.Engine.Geometry.CSLib.DNNormal(2, 1, derNUV, orderU, orderV).Multiplied(sign));
                d3UVV = d3UVV.Added(global::Kerf.Engine.Geometry.CSLib.DNNormal(1, 2, derNUV, orderU, orderV).Multiplied(sign));
                return true;
            }

            // Try shifting point towards center - returns false when center is overpassed
            if (!ShiftPoint(uStart, vStart, ref u, ref v, uMin, uMax, vMin, vMax, isUPer, isVPer, d1U,
                            d1V))
            {
                return false;
            }
        }
    }

    /// <summary>
    /// The derivative of order (theNu, theNv) from the basis first derivatives at (theU, theV); at singular points,
    /// retries closer to the center. False if it fails.
    /// </summary>
    internal static bool EvaluateDN<TBasis>(double u,
                                            double v,
                                            int nu,
                                            int nv,
                                            in TBasis basisSurf,
                                            double offset,
                                            OsculatingSurface? oscQuery,
                                            out Vector3D result,
                                            in Vector3D d1U,
                                            in Vector3D d1V)
        where TBasis : struct, IBasisSurface
    {
        result = new Vector3D();
        double uStart = u;
        double vStart = v;
        basisSurf.Bounds(out double uMin, out double uMax, out double vMin, out double vMax);
        bool isUPer = basisSurf.IsUPeriodic();
        bool isVPer = basisSurf.IsVPeriodic();

        // Use pre-computed D1 for first iteration
        Vector3D aD1U = d1U;
        Vector3D aD1V = d1V;
        bool isFirstIteration = true;

        for (;;)
        {
            // For subsequent iterations, recompute D1 at shifted point
            if (!isFirstIteration)
            {
                Surface.ResD1 d1Result = basisSurf.EvalD1(u, v);
                aD1U = d1Result.D1U;
                aD1V = d1Result.D1V;
            }
            isFirstIteration = false;

            if (IsInfiniteCoord(aD1U) || IsInfiniteCoord(aD1V))
            {
                return false;
            }

            // Check if singular to determine MaxOrder
            Direction3D normal = new Direction3D();
            global::Kerf.Engine.Geometry.CSLib.Normal(aD1U, aD1V, THE_D1_MAGNITUDE_TOL, out NormalStatus nStatus, ref normal);

            int maxOrder = (nStatus == Defined) ? 0 : 3;

            NCollection_Array2<Vector3D> derNUV = new(0, maxOrder + nu, 0, maxOrder + nv);
            NCollection_Array2<Vector3D> derSurf = new(0, maxOrder + nu + 1, 0, maxOrder + nv + 1);

            derSurf.SetValue(1, 0, aD1U);
            derSurf.SetValue(0, 1, aD1V);

            // Check osculating surface only in singular case
            OsculatingInfo oscInfo = default;
            BSplineSurface? oscSurf = null;
            if (nStatus != Defined)
            {
                QueryOsculating(oscQuery, u, v, ref oscInfo, ref oscSurf);
            }

            // Use ComputeDerivatives
            if (oscInfo.HasOsculating && oscSurf is not null)
            {
                if (!ComputeDerivatives(maxOrder, 1, u, v, basisSurf, nu, nv, oscInfo.AlongU,
                                        oscInfo.AlongV, oscSurf, derNUV, derSurf))
                {
                    return false;
                }
            }
            else
            {
                if (!ComputeDerivatives(maxOrder, 1, u, v, basisSurf, nu, nv, false, false, null,
                                        derNUV, derSurf))
                {
                    return false;
                }
            }

            // Compute normal with CSLib
            global::Kerf.Engine.Geometry.CSLib.Normal(maxOrder,
                         derNUV,
                         THE_D1_MAGNITUDE_TOL,
                         u,
                         v,
                         uMin,
                         uMax,
                         vMin,
                         vMax,
                         out nStatus,
                         ref normal,
                         out int orderU,
                         out int orderV);

            if (nStatus == Defined)
            {
                double sign = offset * oscInfo.Sign();

                // Compute DN result: basis DN + offset * DNNormal
                Vector3D basisDN = basisSurf.EvalDN(u, v, nu, nv);
                result = basisDN;
                result = result.Added(global::Kerf.Engine.Geometry.CSLib.DNNormal(nu, nv, derNUV, orderU, orderV).Multiplied(sign));
                return true;
            }

            // Try shifting point towards center - returns false when center is overpassed
            if (!ShiftPoint(uStart, vStart, ref u, ref v, uMin, uMax, vMin, vMax, isUPer, isVPer, aD1U,
                            aD1V))
            {
                return false;
            }
        }
    }

    /// <summary>The derivative of order (theNu, theNv) at (theU, theV); false if it fails at a singular point.</summary>
    internal static bool EvaluateDN<TBasis>(double u,
                                            double v,
                                            int nu,
                                            int nv,
                                            in TBasis basisSurf,
                                            double offset,
                                            OsculatingSurface? oscQuery,
                                            out Vector3D result)
        where TBasis : struct, IBasisSurface
    {
        Surface.ResD1 basisD1 = basisSurf.EvalD1(u, v);
        return EvaluateDN(u,
                          v,
                          nu,
                          nv,
                          basisSurf,
                          offset,
                          oscQuery,
                          out result,
                          basisD1.D1U,
                          basisD1.D1V);
    }
}
