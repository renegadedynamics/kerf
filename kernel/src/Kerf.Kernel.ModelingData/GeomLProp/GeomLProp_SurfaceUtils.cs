// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_SurfaceUtils.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.DerivativeStatus;
using static Kerf.Engine.Geometry.Properties.Status;

namespace Kerf.Engine.Geometry.Properties;

/// <summary>
/// The computations shared by the surface local-property classes (OCCT's namespace <c>LProp_SurfaceUtils</c>, in
/// GeomLProp_SurfaceUtils.hxx): the order of the first significant derivative, the normal, and the principal
/// curvatures and directions from the fundamental forms.
/// </summary>
/// <remarks>
/// OCCT's templates over the surface type are ported per instantiation (rules, section 9): the helpers that don't
/// depend on the surface are here; the access policies, derivative evaluation, bounds and the wrappers that take
/// the properties object are written out in <see cref="SLProps"/> and <see cref="SLProps3d"/>.
/// </remarks>
internal static class SurfaceUtils
{
    /// <summary>
    /// The order (1 or 2) of the first derivative in one direction longer than the tolerance; the status is
    /// LProp_Defined if there is one, LProp_Undefined otherwise.
    /// </summary>
    internal static bool FindSurfTangentOrder(in Vector3D d1,
                                              in Vector3D d2,
                                              int cn,
                                              double tolSq,
                                              out int order,
                                              out Status status)
    {
        order = 0;

        while (order < 3)
        {
            order++;
            if (cn >= order)
            {
                // OCCT's array of pointers {&theD1, &theD2}
                if (order <= 2 && (order == 1 ? d1 : d2).SquareMagnitude > tolSq)
                {
                    status = Defined;
                    return true;
                }
            }
            else
            {
                status = Undefined;
                return false;
            }
        }

        status = Undefined;
        return false;
    }

    /// <summary>The normal by CSLib::Normal; true if it is defined.</summary>
    internal static bool ComputeSurfNormal(in Vector3D d1u, in Vector3D d1v, double linTol, ref Direction3D normal)
    {
        DerivativeStatus status;
        global::Kerf.Engine.Geometry.CSLib.Normal(d1u, d1v, linTol, out status, ref normal);
        return status == Done;
    }

    /// <summary>
    /// The principal curvatures and directions, the mean and the Gaussian curvatures, from the first and second
    /// fundamental forms; false if they can't be computed (the outputs may then be partly written, as in OCCT).
    /// </summary>
    internal static bool ComputeSurfCurvatures(in Vector3D d1u,
                                               in Vector3D d1v,
                                               in Vector3D d2u,
                                               in Vector3D d2v,
                                               in Vector3D duv,
                                               in Direction3D normal,
                                               ref double minCurv,
                                               ref double maxCurv,
                                               ref Direction3D dirMin,
                                               ref Direction3D dirMax,
                                               ref double meanCurv,
                                               ref double gausCurv)
    {
        Vector3D norm = new(normal);

        double e = d1u.SquareMagnitude;
        double f = d1u.Dot(d1v);
        double g = d1v.SquareMagnitude;

        double l = norm.Dot(d2u);
        double m = norm.Dot(duv);
        double n = norm.Dot(d2v);

        double a0 = e * m - f * l;
        double b0 = e * n - g * l;
        double c0 = f * n - g * m;

        double maxABC = Max(Max(Math.Abs(a0), Math.Abs(b0)), Math.Abs(c0));
        if (maxABC < RealEpsilon())
        {
            // Umbilic point
            if (g < RealEpsilon())
            {
                return false;
            }

            minCurv = n / g;
            maxCurv = minCurv;
            dirMin = new Direction3D(d1u);
            dirMax = new Direction3D(d1u.Crossed(norm));
            meanCurv = minCurv;
            gausCurv = minCurv * minCurv;
            return true;
        }

        double a = a0 / maxABC;
        double b = b0 / maxABC;
        double c = c0 / maxABC;

        double curv1, curv2;
        Vector3D vectCurv1, vectCurv2;

        if (Math.Abs(a) > RealEpsilon())
        {
            var root = new DirectPolynomialRoots(a, b, c);
            if (root.SolutionCount != 2)
            {
                return false;
            }

            double root1 = root.Value(1);
            double root2 = root.Value(2);
            curv1 = ((l * root1 + 2.0 * m) * root1 + n) / ((e * root1 + 2.0 * f) * root1 + g);
            curv2 = ((l * root2 + 2.0 * m) * root2 + n) / ((e * root2 + 2.0 * f) * root2 + g);
            vectCurv1 = root1 * d1u + d1v;
            vectCurv2 = root2 * d1u + d1v;
        }
        else if (Math.Abs(c) > RealEpsilon())
        {
            var root = new DirectPolynomialRoots(c, b, a);
            if (root.SolutionCount != 2)
            {
                return false;
            }

            double root1 = root.Value(1);
            double root2 = root.Value(2);
            curv1 = ((n * root1 + 2.0 * m) * root1 + l) / ((g * root1 + 2.0 * f) * root1 + e);
            curv2 = ((n * root2 + 2.0 * m) * root2 + l) / ((g * root2 + 2.0 * f) * root2 + e);
            vectCurv1 = d1u + root1 * d1v;
            vectCurv2 = d1u + root2 * d1v;
        }
        else
        {
            curv1 = l / e;
            curv2 = n / g;
            vectCurv1 = d1u;
            vectCurv2 = d1v;
        }

        if (curv1 < curv2)
        {
            minCurv = curv1;
            maxCurv = curv2;
            dirMin = new Direction3D(vectCurv1);
            dirMax = new Direction3D(vectCurv2);
        }
        else
        {
            minCurv = curv2;
            maxCurv = curv1;
            dirMin = new Direction3D(vectCurv2);
            dirMax = new Direction3D(vectCurv1);
        }

        double eg_FF = (e * g) - (f * f);
        meanCurv = ((n * e) - (2.0 * m * f) + (l * g)) / (2.0 * eg_FF);
        gausCurv = ((l * n) - (m * m)) / eg_FF;
        return true;
    }

    /// <summary>The normal status, computing the normal if it is undecided.</summary>
    internal static bool IsNormalDefined(in Vector3D d1u,
                                         in Vector3D d1v,
                                         double linTol,
                                         ref Direction3D normal,
                                         ref Status normStatus)
    {
        if (normStatus == Undefined)
        {
            return false;
        }

        if (normStatus >= Defined)
        {
            return true;
        }

        if (ComputeSurfNormal(d1u, d1v, linTol, ref normal))
        {
            normStatus = Computed;
            return true;
        }

        normStatus = Undefined;
        return false;
    }
}
