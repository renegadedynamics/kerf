// Port of: src/ModelingData/TKG3d/Geom/Geom_OffsetCurveUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// The point and derivatives of a 3D offset curve from those of its basis curve:
/// P(u) = p(u) + Offset * N / ||N||, where N = p'(u) ^ Direction is the local normal. An internal C++ namespace
/// in OCCT, used by Geom_OffsetCurve, GeomAdaptor_Curve and GeomGridEval.
/// </summary>
/// <remarks>
/// OCCT's function templates over the curve type are ported for the instantiations OCCT uses:
/// <see cref="AdjustDerivative(Curve3D, int, double, ref Vector3D, ref Vector3D, ref Vector3D, ref Vector3D, out bool)"/>
/// over <see cref="Curve3D"/> and <see cref="CurveAdaptorBase"/>, and the <c>Evaluate*</c> functions over
/// <see cref="CurveAdaptorBase"/> (GeomAdaptor_Curve's basis adaptor). Both curve types have the same evaluation
/// members, so the bodies are identical.
/// </remarks>
internal static class OffsetCurveUtils3D
{
    /// <summary>
    /// The offset point: theValue is the basis point on input and the offset point on output. False if the
    /// normal p' ^ Dir has a magnitude at most theTolerance.
    /// </summary>
    internal static bool CalculateD0(ref Point3D value,
                                     in Vector3D d1,
                                     in Xyz dirXYZ,
                                     double offset,
                                     double tolerance = gp_Resolution)
    {
        Xyz ndir = (d1.XYZ()).Crossed(dirXYZ);
        double r = ndir.Modulus();
        if (r <= tolerance)
        {
            return false;
        }
        ndir.Multiply(offset / r);
        value = new Point3D(value.XYZ().Added(ndir));
        return true;
    }

    /// <summary>The offset point and first derivative (theValue and theD1 are the basis's on input).</summary>
    internal static bool CalculateD1(ref Point3D value,
                                     ref Vector3D d1,
                                     in Vector3D d2,
                                     in Xyz dirXYZ,
                                     double offset,
                                     double tolerance = gp_Resolution)
    {
        // P(u) = p(u) + Offset * Ndir / R
        // with R = || p' ^ V|| and Ndir = P' ^ direction (local normal direction)

        // P'(u) = p'(u) + (Offset / R**2) * (DNdir/DU * R -  Ndir * (DR/R))

        Xyz ndir = (d1.XYZ()).Crossed(dirXYZ);
        Xyz dNdir = (d2.XYZ()).Crossed(dirXYZ);
        double r2 = ndir.SquareModulus();
        double r = Math.Sqrt(r2);
        double r3 = r * r2;
        double dr = ndir.Dot(dNdir);
        if (r3 <= tolerance)
        {
            if (r2 <= tolerance)
            {
                return false;
            }
            // We try another computation but the stability is not very good.
            dNdir.Multiply(r);
            dNdir.Subtract(ndir.Multiplied(dr / r));
            dNdir.Multiply(offset / r2);
        }
        else
        {
            // Same computation as IICURV in EUCLID-IS because the stability is better
            dNdir.Multiply(offset / r);
            dNdir.Subtract(ndir.Multiplied(offset * dr / r3));
        }

        ndir.Multiply(offset / r);
        // P(u)
        value = new Point3D(value.XYZ().Added(ndir));
        // P'(u)
        d1 = d1.Added(new Vector3D(dNdir));
        return true;
    }

    /// <summary>
    /// The offset point and first two derivatives (the basis's on input). theIsDirChange: the tangent was
    /// reversed at a singular point (see <c>AdjustDerivative</c>).
    /// </summary>
    internal static bool CalculateD2(ref Point3D value,
                                     ref Vector3D d1,
                                     ref Vector3D d2,
                                     in Vector3D d3,
                                     in Xyz dirXYZ,
                                     double offset,
                                     bool isDirChange,
                                     double tolerance = gp_Resolution)
    {
        // P(u) = p(u) + Offset * Ndir / R
        // with R = || p' ^ V|| and Ndir = P' ^ direction (local normal direction)

        // P'(u) = p'(u) + (Offset / R**2) * (DNdir/DU * R -  Ndir * (DR/R))

        // P"(u) = p"(u) + (Offset / R) * (D2Ndir/DU - DNdir * (2.0 * Dr/ R**2) +
        //         Ndir * ( (3.0 * Dr**2 / R**4) - (D2r / R**2)))

        Xyz ndir = (d1.XYZ()).Crossed(dirXYZ);
        Xyz dNdir = (d2.XYZ()).Crossed(dirXYZ);
        Xyz d2Ndir = (d3.XYZ()).Crossed(dirXYZ);
        double r2 = ndir.SquareModulus();
        double r = Math.Sqrt(r2);
        double r3 = r2 * r;
        double r4 = r2 * r2;
        double r5 = r3 * r2;
        double dr = ndir.Dot(dNdir);
        double d2r = ndir.Dot(d2Ndir) + dNdir.Dot(dNdir);

        if (r5 <= tolerance)
        {
            if (r4 <= tolerance)
            {
                return false;
            }
            // We try another computation but the stability is not very good
            // dixit ISG.
            //  V2 = P" (U) :
            r4 = r2 * r2;
            d2Ndir.Subtract(dNdir.Multiplied(2.0 * dr / r2));
            d2Ndir.Add(ndir.Multiplied(((3.0 * dr * dr) / r4) - (d2r / r2)));
            d2Ndir.Multiply(offset / r);

            // V1 = P' (U) :
            dNdir.Multiply(r);
            dNdir.Subtract(ndir.Multiplied(dr / r));
            dNdir.Multiply(offset / r2);
        }
        else
        {
            // Same computation as IICURV in EUCLID-IS because the stability is better.
            // V2 = P" (U) :
            d2Ndir.Multiply(offset / r);
            d2Ndir.Subtract(dNdir.Multiplied(2.0 * offset * dr / r3));
            d2Ndir.Add(ndir.Multiplied(offset * (((3.0 * dr * dr) / r5) - (d2r / r3))));

            // V1 = P' (U) :
            dNdir.Multiply(offset / r);
            dNdir.Subtract(ndir.Multiplied(offset * dr / r3));
        }

        ndir.Multiply(offset / r);
        // P(u)
        value = new Point3D(value.XYZ().Added(ndir));
        // P'(u) :
        d1 = d1.Added(new Vector3D(dNdir));
        // P"(u) :
        if (isDirChange)
        {
            d2 = d2.Reversed();
        }
        d2 = d2.Added(new Vector3D(d2Ndir));
        return true;
    }

    /// <summary>The offset point and first three derivatives (the basis's on input, plus its D4).</summary>
    internal static bool CalculateD3(ref Point3D value,
                                     ref Vector3D d1,
                                     ref Vector3D d2,
                                     ref Vector3D d3,
                                     in Vector3D d4,
                                     in Xyz dirXYZ,
                                     double offset,
                                     bool isDirChange,
                                     double tolerance = gp_Resolution)
    {
        // P(u) = p(u) + Offset * Ndir / R
        // with R = || p' ^ V|| and Ndir = P' ^ direction (local normal direction)

        // P'(u) = p'(u) + (Offset / R**2) * (DNdir/DU * R -  Ndir * (DR/R))

        // P"(u) = p"(u) + (Offset / R) * (D2Ndir/DU - DNdir * (2.0 * Dr/ R**2) +
        //         Ndir * ( (3.0 * Dr**2 / R**4) - (D2r / R**2)))

        // P"'(u) = p"'(u) + (Offset / R) * (D3Ndir - (3.0 * Dr/R**2) * D2Ndir -
        //          (3.0 * D2r / R2) * DNdir + (9.0 * Dr * Dr / R4) * DNdir -
        //          (D3r/R2) * Ndir + (9.0 * Dr * D2r / R4) * Ndir - (15.0 * Dr* Dr* Dr /R6) * Ndir)

        Xyz ndir = (d1.XYZ()).Crossed(dirXYZ);
        Xyz dNdir = (d2.XYZ()).Crossed(dirXYZ);
        Xyz d2Ndir = (d3.XYZ()).Crossed(dirXYZ);
        Xyz d3Ndir = (d4.XYZ()).Crossed(dirXYZ);
        double r2 = ndir.SquareModulus();
        double r = Math.Sqrt(r2);
        double r3 = r2 * r;
        double r4 = r2 * r2;
        double r5 = r3 * r2;
        double r6 = r3 * r3;
        double r7 = r5 * r2;
        double dr = ndir.Dot(dNdir);
        double d2r = ndir.Dot(d2Ndir) + dNdir.Dot(dNdir);
        double d3r = ndir.Dot(d3Ndir) + 3.0 * dNdir.Dot(d2Ndir);

        if (r7 <= tolerance)
        {
            if (r6 <= tolerance)
            {
                return false;
            }
            // V3 is the derivative of V2 (OCCT's DNdir and Ndir coefficients weren't, and its V2 subtracted the Ndir
            // term; Kerf decision 1487)
            // V3 = P"' (U) :
            d3Ndir.Subtract(d2Ndir.Multiplied(3.0 * dr / r2));
            d3Ndir.Subtract(dNdir.Multiplied(3.0 * ((d2r / r2) - (3.0 * dr * dr / r4))));
            d3Ndir.Add(
              ndir.Multiplied(9.0 * dr * d2r / r4 - 15.0 * dr * dr * dr / r6 - d3r / r2));
            d3Ndir.Multiply(offset / r);
            // V2 = P" (U) :
            r4 = r2 * r2;
            d2Ndir.Subtract(dNdir.Multiplied(2.0 * dr / r2));
            d2Ndir.Add(ndir.Multiplied((3.0 * dr * dr / r4) - (d2r / r2)));
            d2Ndir.Multiply(offset / r);
            // V1 = P' (U) :
            dNdir.Multiply(r);
            dNdir.Subtract(ndir.Multiplied(dr / r));
            dNdir.Multiply(offset / r2);
        }
        else
        {
            // V3 is the derivative of V2 (OCCT's DNdir and Ndir coefficients weren't, and its V2 subtracted the Ndir
            // term; Kerf decision 1487)
            // V3 = P"' (U) :
            d3Ndir.Divide(r);
            d3Ndir.Subtract(d2Ndir.Multiplied(3.0 * dr / r3));
            d3Ndir.Subtract(dNdir.Multiplied((3.0 * ((d2r / r3) - (3.0 * dr * dr) / r5))));
            d3Ndir.Add(
              ndir.Multiplied(9.0 * dr * d2r / r5 - 15.0 * dr * dr * dr / r7 - d3r / r3));
            d3Ndir.Multiply(offset);
            // V2 = P" (U) :
            d2Ndir.Divide(r);
            d2Ndir.Subtract(dNdir.Multiplied(2.0 * dr / r3));
            d2Ndir.Add(ndir.Multiplied((3.0 * dr * dr / r5) - (d2r / r3)));
            d2Ndir.Multiply(offset);
            // V1 = P' (U) :
            dNdir.Multiply(offset / r);
            dNdir.Subtract(ndir.Multiplied(offset * dr / r3));
        }

        ndir.Multiply(offset / r);
        // P(u)
        value = new Point3D(value.XYZ().Added(ndir));
        // P'(u) :
        d1 = d1.Added(new Vector3D(dNdir));
        // P"(u)
        d2 = d2.Added(new Vector3D(d2Ndir));
        // P"'(u)
        if (isDirChange)
        {
            d3 = d3.Reversed();
        }
        d3 = d3.Added(new Vector3D(d3Ndir));
        return true;
    }

    // gp::Resolution() as a constant, for the default arguments.
    private const double gp_Resolution = RealSmallValue; // gp::Resolution()

    /// <summary>
    /// Replaces the derivatives at a singular point (first derivative nearly zero) by the first non-null higher
    /// derivatives (Taylor series), oriented by the chord around theU. theMaxDerivative is 3 or 4.
    /// </summary>
    internal static bool AdjustDerivative(Curve3D curve,
                                          int maxDerivative,
                                          double theU,
                                          ref Vector3D d1,
                                          ref Vector3D d2,
                                          ref Vector3D d3,
                                          ref Vector3D d4,
                                          out bool isDirectionChange)
    {
        double tol = Constants.Resolution();
        const double minStep = 1e-7;
        const int maxDerivOrder = 3;

        isDirectionChange = false;
        double uinfium = curve.FirstParameter();
        double usupremum = curve.LastParameter();

        const double divisionFactor = 1.0e-3;
        double du;
        if ((usupremum >= RealLast()) || (uinfium <= RealFirst()))
        {
            du = 0.0;
        }
        else
        {
            du = usupremum - uinfium;
        }

        double delta = Max(du * divisionFactor, minStep);

        // Derivative is approximated by Taylor-series
        int index = 1; // Derivative order
        Vector3D v;

        do
        {
            v = curve.EvalDN(theU, ++index);
        } while ((v.SquareMagnitude() <= tol) && index < maxDerivOrder);

        double u;

        if (theU - uinfium < delta)
        {
            u = theU + delta;
        }
        else
        {
            u = theU - delta;
        }

        Point3D p1;
        Point3D p2;
        p1 = curve.EvalD0(Min(theU, u));
        p2 = curve.EvalD0(Max(theU, u));

        Vector3D v1 = new Vector3D(p1, p2);
        isDirectionChange = v.Dot(v1) < 0.0;
        double sign = isDirectionChange ? -1.0 : 1.0;

        d1 = v * sign;
        for (int i = 1; i < maxDerivative; i++)
        {
            Vector3D dn = curve.EvalDN(theU, index + i);
            // C++: *(aDeriv[i - 1]) = aDN * aSign, with aDeriv = {&theD2, &theD3, &theD4}
            switch (i)
            {
                case 1:
                    d2 = dn * sign;
                    break;
                case 2:
                    d3 = dn * sign;
                    break;
                default:
                    d4 = dn * sign;
                    break;
            }
        }

        return true;
    }

    /// <inheritdoc cref="AdjustDerivative(Curve3D, int, double, ref Vector3D, ref Vector3D, ref Vector3D, ref Vector3D, out bool)"/>
    internal static bool AdjustDerivative(CurveAdaptorBase curve,
                                          int maxDerivative,
                                          double theU,
                                          ref Vector3D d1,
                                          ref Vector3D d2,
                                          ref Vector3D d3,
                                          ref Vector3D d4,
                                          out bool isDirectionChange)
    {
        double tol = Constants.Resolution();
        const double minStep = 1e-7;
        const int maxDerivOrder = 3;

        isDirectionChange = false;
        double uinfium = curve.FirstParameter();
        double usupremum = curve.LastParameter();

        const double divisionFactor = 1.0e-3;
        double du;
        if ((usupremum >= RealLast()) || (uinfium <= RealFirst()))
        {
            du = 0.0;
        }
        else
        {
            du = usupremum - uinfium;
        }

        double delta = Max(du * divisionFactor, minStep);

        // Derivative is approximated by Taylor-series
        int index = 1; // Derivative order
        Vector3D v;

        do
        {
            v = curve.EvalDN(theU, ++index);
        } while ((v.SquareMagnitude() <= tol) && index < maxDerivOrder);

        double u;

        if (theU - uinfium < delta)
        {
            u = theU + delta;
        }
        else
        {
            u = theU - delta;
        }

        Point3D p1;
        Point3D p2;
        p1 = curve.EvalD0(Min(theU, u));
        p2 = curve.EvalD0(Max(theU, u));

        Vector3D v1 = new Vector3D(p1, p2);
        isDirectionChange = v.Dot(v1) < 0.0;
        double sign = isDirectionChange ? -1.0 : 1.0;

        d1 = v * sign;
        for (int i = 1; i < maxDerivative; i++)
        {
            Vector3D dn = curve.EvalDN(theU, index + i);
            switch (i)
            {
                case 1:
                    d2 = dn * sign;
                    break;
                case 2:
                    d3 = dn * sign;
                    break;
                default:
                    d4 = dn * sign;
                    break;
            }
        }

        return true;
    }

    /// <summary>The offset point of the basis curve at theU; false if the normal is undefined.</summary>
    internal static bool EvaluateD0(double u,
                                    CurveAdaptorBase basisCurve,
                                    in Direction3D dir,
                                    double offset,
                                    out Point3D value)
    {
        Curve3D.ResD1 d1 = basisCurve.EvalD1(u);
        value = d1.Point;
        return CalculateD0(ref value, d1.D1, dir.XYZ(), offset);
    }

    /// <summary>The offset point and first derivative at theU.</summary>
    internal static bool EvaluateD1(double u,
                                    CurveAdaptorBase basisCurve,
                                    in Direction3D dir,
                                    double offset,
                                    out Point3D value,
                                    out Vector3D d1)
    {
        Curve3D.ResD2 d2 = basisCurve.EvalD2(u);
        value = d2.Point;
        d1 = d2.D1;
        return CalculateD1(ref value, ref d1, d2.D2, dir.XYZ(), offset);
    }

    /// <summary>The offset point and first two derivatives at theU (singular points handled).</summary>
    internal static bool EvaluateD2(double u,
                                    CurveAdaptorBase basisCurve,
                                    in Direction3D dir,
                                    double offset,
                                    out Point3D value,
                                    out Vector3D d1,
                                    out Vector3D d2)
    {
        Curve3D.ResD3 d3Result = basisCurve.EvalD3(u);
        value = d3Result.Point;
        d1 = d3Result.D1;
        d2 = d3Result.D2;
        Vector3D d3 = d3Result.D3;

        bool isDirectionChange = false;
        if (d1.SquareMagnitude() <= Constants.Resolution())
        {
            Vector3D dummyD4 = new Vector3D();
            if (!AdjustDerivative(basisCurve, 3, u, ref d1, ref d2, ref d3, ref dummyD4, out isDirectionChange))
            {
                return false;
            }
        }

        return CalculateD2(ref value, ref d1, ref d2, d3, dir.XYZ(), offset, isDirectionChange);
    }

    /// <summary>The offset point and first three derivatives at theU (singular points handled).</summary>
    internal static bool EvaluateD3(double u,
                                    CurveAdaptorBase basisCurve,
                                    in Direction3D dir,
                                    double offset,
                                    out Point3D value,
                                    out Vector3D d1,
                                    out Vector3D d2,
                                    out Vector3D d3)
    {
        Curve3D.ResD3 d3Result = basisCurve.EvalD3(u);
        value = d3Result.Point;
        d1 = d3Result.D1;
        d2 = d3Result.D2;
        d3 = d3Result.D3;
        Vector3D d4 = basisCurve.EvalDN(u, 4);

        bool isDirectionChange = false;
        if (d1.SquareMagnitude() <= Constants.Resolution())
        {
            if (!AdjustDerivative(basisCurve, 4, u, ref d1, ref d2, ref d3, ref d4, out isDirectionChange))
            {
                return false;
            }
        }

        return CalculateD3(ref value,
                           ref d1,
                           ref d2,
                           ref d3,
                           d4,
                           dir.XYZ(),
                           offset,
                           isDirectionChange);
    }

    /// <summary>
    /// The derivative of order theN (at least 1) at theU; above 3, the basis curve's (the offset contribution is
    /// neglected, as in OCCT).
    /// </summary>
    internal static bool EvaluateDN(double u,
                                    CurveAdaptorBase basisCurve,
                                    in Direction3D dir,
                                    double offset,
                                    int n,
                                    out Vector3D dn)
    {
        Point3D pnt;
        Vector3D dummy;
        switch (n)
        {
            case 1:
                return EvaluateD1(u, basisCurve, dir, offset, out pnt, out dn);
            case 2:
                return EvaluateD2(u, basisCurve, dir, offset, out pnt, out dummy, out dn);
            case 3:
                return EvaluateD3(u, basisCurve, dir, offset, out pnt, out dummy, out Vector3D dummy2, out dn);
            default:
            {
                // For derivatives > 3, return basis curve derivative (no offset contribution)
                dn = basisCurve.EvalDN(u, n);
                return true;
            }
        }
    }
}
