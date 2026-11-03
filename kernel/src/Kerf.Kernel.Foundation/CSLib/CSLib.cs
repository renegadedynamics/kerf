// Port of: src/FoundationClasses/TKMath/CSLib/CSLib.hxx
// Port of: src/FoundationClasses/TKMath/CSLib/CSLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.DerivativeStatus;
using static Kerf.Engine.Geometry.NormalStatus;

namespace Kerf.Engine.Geometry;

/// <summary>
/// Computes the normal of a surface from its derivatives, including at singular points, where the normal is
/// found from higher-order derivatives; and the derivatives of the normal itself.
/// </summary>
/// <remarks>
/// <c>theNormal</c> is a <c>ref</c> parameter: as in OCCT, it is left unchanged when the status says that no
/// normal was computed.
/// </remarks>
public static class CSLib
{
    // Angular tolerance for parallel/opposite vector detection (radians).
    private const double THE_PARALLEL_ANGULAR_TOL = 1e-6;

    // Maximum number of iterations for root finding algorithm.
    private const int THE_MAX_ROOT_ITERATIONS = 200;

    // Tolerance for root finding algorithm.
    private const double THE_ROOT_FINDING_TOL = 1e-5;

    /// <summary>
    /// The normal D1U ^ D1V, normalized. <paramref name="status"/> is <c>CSLib_Done</c> when it is computed;
    /// otherwise a derivative is null or D1U and D1V are parallel (the sine of their angle is below
    /// <paramref name="sinTol"/>).
    /// </summary>
    public static void Normal(in Vector3D d1U, in Vector3D d1V, double sinTol,
                              out DerivativeStatus status, ref Direction3D normal)
    {
        double d1UMag  = d1U.SquareMagnitude;
        double d1VMag  = d1V.SquareMagnitude;
        Vector3D d1UxD1V = d1U.Crossed(d1V);

        if (d1UMag <= Constants.Resolution() && d1VMag <= Constants.Resolution())
        {
            status = D1IsNull;
        }
        else if (d1UMag <= Constants.Resolution())
        {
            status = D1uIsNull;
        }
        else if (d1VMag <= Constants.Resolution())
        {
            status = D1vIsNull;
        }
        else
        {
            double sin2 = d1UxD1V.SquareMagnitude / (d1UMag * d1VMag);
            if (sin2 < sinTol * sinTol)
            {
                status = D1uIsParallelD1v;
            }
            else
            {
                normal = new Direction3D(d1UxD1V);
                status = Done;
            }
        }
    }

    /// <summary>
    /// The normal at a point where D1U ^ D1V is null, approximated from the second derivatives (Taylor expansion
    /// of the normal). <paramref name="done"/> is true when <paramref name="normal"/> is computed.
    /// </summary>
    public static void Normal(in Vector3D d1U, in Vector3D d1V, in Vector3D d2U, in Vector3D d2V,
                              in Vector3D d2UV, double sinTol, out bool done,
                              out NormalStatus status, ref Direction3D normal)
    {
        // Compute approximate normal using Taylor expansion:
        // N(u0+du, v0+dv) = N0 + dN/du * du + dN/dv * dv + O(du^2, dv^2)
        // where N = D1U ^ D1V, so dN/du = D2U ^ D1V + D1U ^ D2UV
        // and dN/dv = D2UV ^ D1V + D1U ^ D2V

        Vector3D d1Nu = d2U.Crossed(d1V);
        d1Nu = d1Nu.Added(d1U.Crossed(d2UV));

        Vector3D d1Nv = d2UV.Crossed(d1V);
        d1Nv = d1Nv.Added(d1U.Crossed(d2V));

        double ld1Nu = d1Nu.SquareMagnitude;
        double ld1Nv = d1Nv.SquareMagnitude;

        if (ld1Nu <= RealEpsilon() && ld1Nv <= RealEpsilon())
        {
            status = D1NIsNull;
            done   = false;
        }
        else if (ld1Nu < RealEpsilon())
        {
            status = D1NuIsNull;
            done   = true;
            normal = new Direction3D(d1Nv);
        }
        else if (ld1Nv < RealEpsilon())
        {
            status = D1NvIsNull;
            done   = true;
            normal = new Direction3D(d1Nu);
        }
        else if ((ld1Nv / ld1Nu) <= RealEpsilon())
        {
            status = D1NvNuRatioIsNull;
            done   = false;
        }
        else if ((ld1Nu / ld1Nv) <= RealEpsilon())
        {
            status = D1NuNvRatioIsNull;
            done   = false;
        }
        else
        {
            Vector3D d1NCross = d1Nu.Crossed(d1Nv);
            double sin2     = d1NCross.SquareMagnitude / (ld1Nu * ld1Nv);

            if (sin2 < sinTol * sinTol)
            {
                status = D1NuIsParallelD1Nv;
                done   = true;
                normal = new Direction3D(d1Nu);
            }
            else
            {
                status = InfinityOfSolutions;
                done   = false;
            }
        }
    }

    /// <summary>
    /// The normal from the normalized first derivatives. <paramref name="status"/> is <c>CSLib_Singular</c>
    /// when a derivative or their cross product is shorter than <paramref name="magTol"/>.
    /// </summary>
    public static void Normal(in Vector3D d1U, in Vector3D d1V, double magTol,
                              out NormalStatus status, ref Direction3D normal)
    {
        double d1UMag  = d1U.Magnitude;
        double d1VMag  = d1V.Magnitude;
        Vector3D d1UxD1V = d1U.Crossed(d1V);
        double nMag    = d1UxD1V.Magnitude;

        if (nMag <= magTol || d1UMag <= magTol || d1VMag <= magTol)
        {
            status = Singular;
        }
        else
        {
            // Normalize tangent vectors first for better numerical stability.
            Direction3D d1UDir = new(d1U);
            Direction3D d1VDir = new(d1V);
            normal = d1UDir.Crossed(d1VDir);
            status = Defined;
        }
    }

    /// <summary>
    /// The normal at a singular point, from the derivatives of N = dS/du ^ dS/dv up to order
    /// <paramref name="maxOrder"/>: <paramref name="derNUV"/>(i, j) is the derivative of N of order i in U
    /// and j in V. The parameter bounds tell whether the point is inside the domain, on an edge or at a corner.
    /// <paramref name="orderU"/> and <paramref name="orderV"/> are the orders of the first non-null
    /// derivative.
    /// </summary>
    public static void Normal(int maxOrder, NCollection_Array2<Vector3D> derNUV, double sinTol,
                              double u, double v, double umin, double umax, double vmin,
                              double vmax, out NormalStatus status, ref Direction3D normal,
                              out int orderU, out int orderV)
    {
        int    order    = -1;
        int    foundUIdx = 0;
        bool   found     = false;
        double norme;
        Vector3D d = new();

        // Find k0 such that all derivatives N = dS/du ^ dS/dv are null till order k0-1.
        while (!found && order < maxOrder)
        {
            ++order;
            foundUIdx = order;
            while (foundUIdx >= 0 && !found)
            {
                int vIdx = order - foundUIdx;
                d        = derNUV.Value(foundUIdx, vIdx);
                norme    = d.Magnitude;
                found    = (norme >= sinTol);
                --foundUIdx;
            }
        }

        orderU = foundUIdx + 1;
        orderV = order - orderU;

        // Vk0 is the first non-null derivative of N: the reference vector.
        if (!found)
        {
            status = Singular;
            return;
        }

        if (order == 0)
        {
            status = Defined;
            normal = new Direction3D(d.Normalized());
            return;
        }

        Vector3D                     vk0   = derNUV.Value(orderU, orderV);
        NCollection_Array1<double> ratio = new(0, order);

        // Calculate lambda_i ratios for each derivative at this order.
        int  ratioIdx = 0;
        bool isDefined = false;
        while (ratioIdx <= order && !isDefined)
        {
            Vector3D derVec = derNUV.Value(ratioIdx, order - ratioIdx);
            if (derVec.Magnitude <= sinTol)
            {
                ratio[ratioIdx] = 0.0;
            }
            else if (derVec.IsParallel(vk0, THE_PARALLEL_ANGULAR_TOL))
            {
                double magnitudeRatio = derVec.Magnitude / vk0.Magnitude;
                if (derVec.IsOpposite(vk0, THE_PARALLEL_ANGULAR_TOL))
                {
                    magnitudeRatio = -magnitudeRatio;
                }
                ratio[ratioIdx] = magnitudeRatio;
            }
            else
            {
                isDefined = true;
            }
            ++ratioIdx;
        }

        if (isDefined)
        {
            status = Defined;
            normal = new Direction3D(d.Normalized());
            return;
        }

        // All lambda_i exist - analyze the polynomial sign.
        double inf = -M_PI;
        double sup = M_PI;

        // Determine domain based on position (interior, edge, corner).
        bool isFU = (Math.Abs(u - umin) < global::Kerf.Engine.Foundation.Precision.PConfusion());
        bool isLU = (Math.Abs(u - umax) < global::Kerf.Engine.Foundation.Precision.PConfusion());
        bool isFV = (Math.Abs(v - vmin) < global::Kerf.Engine.Foundation.Precision.PConfusion());
        bool isLV = (Math.Abs(v - vmax) < global::Kerf.Engine.Foundation.Precision.PConfusion());

        if (isLU)
        {
            inf = M_PI / 2.0;
            sup = 3.0 * M_PI / 2.0;
            if (isLV)
            {
                inf = M_PI;
            }
            if (isFV)
            {
                sup = M_PI;
            }
        }
        else if (isFU)
        {
            sup = M_PI / 2.0;
            inf = -M_PI / 2.0;
            if (isLV)
            {
                sup = 0.0;
            }
            if (isFV)
            {
                inf = 0.0;
            }
        }
        else if (isLV)
        {
            inf = -M_PI;
            sup = 0.0;
        }
        else if (isFV)
        {
            inf = 0.0;
            sup = M_PI;
        }

        bool   changesSign = false;
        double vprec       = 0.0;
        double vsuiv       = 0.0;

        // Create polynomial and find its roots.
        NormalPolyDef poly      = new(order, ratio);
        FunctionRoots  findRoots = new(poly,
                                             inf,
                                             sup,
                                             THE_MAX_ROOT_ITERATIONS,
                                             THE_ROOT_FINDING_TOL,
                                             global::Kerf.Engine.Foundation.Precision.Confusion(),
                                             global::Kerf.Engine.Foundation.Precision.Confusion());

        if (findRoots.IsDone && findRoots.SolutionCount > 0)
        {
            // Sort roots in ascending order.
            int                        nbSol = findRoots.SolutionCount;
            NCollection_Array1<double> sol   = new(0, nbSol + 1);

            for (int rootIdx = 1; rootIdx <= nbSol; ++rootIdx)
            {
                sol[rootIdx] = findRoots.Value(rootIdx);
            }
            sol.AsSpan(1)[..nbSol].Sort();

            // Add domain limits.
            sol[0]          = inf;
            sol[nbSol + 1] = sup;

            // Check for sign changes between consecutive roots.
            int first = 0;
            for (int intervalIdx = 0; intervalIdx <= nbSol; ++intervalIdx)
            {
                if (Math.Abs(sol[intervalIdx + 1] - sol[intervalIdx]) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    poly.Value((sol[intervalIdx] + sol[intervalIdx + 1]) / 2.0, out vsuiv);
                    // OCCT: aFirst is set to the interval index, which is 0 for the first interval, so the
                    // "first interval" branch runs again for the next one and the sign change between the first
                    // two intervals is never tested.
                    if (first == 0)
                    {
                        first       = intervalIdx;
                        changesSign = false;
                        vprec       = vsuiv;
                    }
                    else
                    {
                        changesSign = changesSign || (vprec * vsuiv) < 0.0;
                        vprec       = vsuiv;
                    }
                }
            }
        }
        else
        {
            // No roots found, polynomial doesn't change sign.
            changesSign = false;
            poly.Value(inf, out vsuiv);
        }

        // Determine status based on polynomial sign.
        if (changesSign)
        {
            status = InfinityOfSolutions;
        }
        else
        {
            status = Defined;
            int sign = (vsuiv > 0.0) ? 1 : -1;
            normal = new Direction3D(sign * vk0.Normalized());
        }
    }

    /// <summary>
    /// The derivative of order (<paramref name="nu"/>, <paramref name="nv"/>) of the non-normalized normal
    /// dS/du ^ dS/dv; <paramref name="derSurf"/>(i, j) holds the derivatives of the surface.
    /// </summary>
    public static Vector3D DNNUV(int nu, int nv, NCollection_Array2<Vector3D> derSurf)
    {
        Vector3D result = new(0.0, 0.0, 0.0);

        for (int i = 0; i <= nu; ++i)
        {
            for (int j = 0; j <= nv; ++j)
            {
                Vector3D vg      = derSurf.Value(i + 1, j);
                Vector3D vd      = derSurf.Value(nu - i, nv + 1 - j);
                Vector3D cross   = vg.Crossed(vd);
                double binCoef = global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(nu, i) * global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(nv, j);
                result = result.Added(binCoef * cross);
            }
        }

        return result;
    }

    /// <summary>
    /// The derivative of order (<paramref name="nu"/>, <paramref name="nv"/>) of dS1/du ^ dS2/dv, for two
    /// surfaces with the derivatives <paramref name="derSurf1"/> and <paramref name="derSurf2"/>.
    /// </summary>
    public static Vector3D DNNUV(int nu, int nv, NCollection_Array2<Vector3D> derSurf1,
                               NCollection_Array2<Vector3D> derSurf2)
    {
        Vector3D result = new(0.0, 0.0, 0.0);

        for (int i = 0; i <= nu; ++i)
        {
            for (int j = 0; j <= nv; ++j)
            {
                Vector3D vg      = derSurf1.Value(i + 1, j);
                Vector3D vd      = derSurf2.Value(nu - i, nv + 1 - j);
                Vector3D cross   = vg.Crossed(vd);
                double binCoef = global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(nu, i) * global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(nv, j);
                result = result.Added(binCoef * cross);
            }
        }

        return result;
    }

    /// <summary>
    /// The derivative of order (<paramref name="nu"/>, <paramref name="nv"/>) of the normalized normal,
    /// from the derivatives <paramref name="derNUV"/> of the non-normalized normal; at a singular point,
    /// (<paramref name="iduref"/>, <paramref name="idvref"/>) are the orders of its first non-null
    /// derivative.
    /// </summary>
    public static Vector3D DNNormal(int nu, int nv, NCollection_Array2<Vector3D> derNUV,
                                  int iduref = 0, int idvref = 0)
    {
        int kderiv = nu + nv;

        NCollection_Array2<Vector3D> derVecNor = new(0, kderiv, 0, kderiv);
        NCollection_Array2<double> tabScal   = new(0, kderiv, 0, kderiv);
        NCollection_Array2<double> tabNorm   = new(0, kderiv, 0, kderiv);

        Vector3D derNor = derNUV.Value(iduref, idvref).Normalized();
        derVecNor.SetValue(0, 0, derNor);

        double dnorm0 = derNUV.Value(iduref, idvref).Dot(derVecNor.Value(0, 0));
        tabNorm.SetValue(0, 0, dnorm0);
        tabScal.SetValue(0, 0, 0.0);

        for (int mderiv = 1; mderiv <= kderiv; ++mderiv)
        {
            for (int pderiv = 0; pderiv <= mderiv; ++pderiv)
            {
                int qderiv = mderiv - pderiv;
                if (pderiv > nu || qderiv > nv)
                {
                    continue;
                }

                // Compute n . derivative(p,q) of n
                double scal = 0.0;
                if (pderiv > qderiv)
                {
                    for (int jderiv = 1; jderiv <= qderiv; ++jderiv)
                    {
                        scal -= global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(qderiv, jderiv)
                                 * derVecNor.Value(0, jderiv).Dot(derVecNor.Value(pderiv, qderiv - jderiv));
                    }

                    for (int jderiv = 0; jderiv < qderiv; ++jderiv)
                    {
                        scal -= global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(qderiv, jderiv)
                                 * derVecNor.Value(pderiv, jderiv).Dot(derVecNor.Value(0, qderiv - jderiv));
                    }

                    for (int ideriv = 1; ideriv < pderiv; ++ideriv)
                    {
                        for (int jderiv = 0; jderiv <= qderiv; ++jderiv)
                        {
                            scal -= global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(pderiv, ideriv) * global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(qderiv, jderiv)
                                     * derVecNor.Value(ideriv, jderiv)
                                         .Dot(derVecNor.Value(pderiv - ideriv, qderiv - jderiv));
                        }
                    }
                }
                else
                {
                    for (int ideriv = 1; ideriv <= pderiv; ++ideriv)
                    {
                        scal -= global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(pderiv, ideriv)
                                 * derVecNor.Value(ideriv, 0).Dot(derVecNor.Value(pderiv - ideriv, qderiv));
                    }

                    for (int ideriv = 0; ideriv < pderiv; ++ideriv)
                    {
                        scal -= global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(pderiv, ideriv)
                                 * derVecNor.Value(ideriv, qderiv).Dot(derVecNor.Value(pderiv - ideriv, 0));
                    }

                    for (int ideriv = 0; ideriv <= pderiv; ++ideriv)
                    {
                        for (int jderiv = 1; jderiv < qderiv; ++jderiv)
                        {
                            scal -= global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(pderiv, ideriv) * global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(qderiv, jderiv)
                                     * derVecNor.Value(ideriv, jderiv)
                                         .Dot(derVecNor.Value(pderiv - ideriv, qderiv - jderiv));
                        }
                    }
                }
                tabScal.SetValue(pderiv, qderiv, scal / 2.0);

                // Compute the derivative (n,p) of NUV Length.
                double dnorm =
                  derNUV.Value(pderiv + iduref, qderiv + idvref).Dot(derVecNor.Value(0, 0));

                for (int jderiv = 0; jderiv < qderiv; ++jderiv)
                {
                    dnorm -= global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(qderiv + idvref, jderiv + idvref)
                              * tabNorm.Value(pderiv, jderiv) * tabScal.Value(0, qderiv - jderiv);
                }

                for (int ideriv = 0; ideriv < pderiv; ++ideriv)
                {
                    for (int jderiv = 0; jderiv <= qderiv; ++jderiv)
                    {
                        dnorm -= global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(pderiv + iduref, ideriv + iduref)
                                  * global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(qderiv + idvref, jderiv + idvref)
                                  * tabNorm.Value(ideriv, jderiv)
                                  * tabScal.Value(pderiv - ideriv, qderiv - jderiv);
                    }
                }
                tabNorm.SetValue(pderiv, qderiv, dnorm);

                // Compute derivative (p,q) of n.
                derNor = derNUV.Value(pderiv + iduref, qderiv + idvref);

                for (int jderiv = 1; jderiv <= qderiv; ++jderiv)
                {
                    derNor = derNor.Subtracted(global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(pderiv + iduref, iduref)
                                     * global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(qderiv + idvref, jderiv + idvref)
                                     * tabNorm.Value(0, jderiv)
                                     * derVecNor.Value(pderiv, qderiv - jderiv));
                }

                for (int ideriv = 1; ideriv <= pderiv; ++ideriv)
                {
                    for (int jderiv = 0; jderiv <= qderiv; ++jderiv)
                    {
                        derNor = derNor.Subtracted(global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(pderiv + iduref, ideriv + iduref)
                                         * global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(qderiv + idvref, jderiv + idvref)
                                         * tabNorm.Value(ideriv, jderiv)
                                         * derVecNor.Value(pderiv - ideriv, qderiv - jderiv));
                    }
                }

                derNor = derNor.Divided(global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(pderiv + iduref, iduref)
                               * global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(qderiv + idvref, idvref) * tabNorm.Value(0, 0));
                derVecNor.SetValue(pderiv, qderiv, derNor);
            }
        }

        return derVecNor.Value(nu, nv);
    }
}
