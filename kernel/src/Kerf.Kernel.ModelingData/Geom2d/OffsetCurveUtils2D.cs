// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_OffsetCurveUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// Point and derivatives of a 2D offset curve, P(u) = p(u) + Offset * N / ||N|| with N = (p'(u).Y, -p'(u).X),
/// from the derivatives of the basis curve. Used by Geom2d_OffsetCurve, Adaptor2d_OffsetCurve and (later)
/// Geom2dAdaptor_Curve and Geom2dGridEval. An internal C++ namespace in OCCT.
/// </summary>
/// <remarks>
/// OCCT's templates over the curve type (<c>AdjustDerivative</c>, <c>EvaluateD0</c> ... <c>EvaluateDN</c>) are
/// ported once per type OCCT instantiates them with: Geom2d_Curve and Adaptor2d_Curve2d.
/// </remarks>
internal static class OffsetCurveUtils2D
{
    /// <summary>
    /// The offset point: theValue is the basis point on input and the offset point on output. False if theD1 is
    /// null (theValue then unchanged).
    /// </summary>
    internal static bool CalculateD0(ref Point2D value, in Vector2D d1, double offset)
    {
        if (d1.SquareMagnitude() <= Constants.Resolution())
        {
            return false;
        }

        Direction2D normal = new Direction2D(d1.Y(), -d1.X());
        value.ChangeCoord().Add(normal.XY() * offset);
        return true;
    }

    /// <summary>The offset point and first derivative, from the basis point, D1 and D2 (updated in place).</summary>
    internal static bool CalculateD1(ref Point2D value,
                                     ref Vector2D d1,
                                     in Vector2D d2,
                                     double offset)
    {
        // P(u) = p(u) + Offset * Ndir / R
        // with R = || p' ^ Z|| and Ndir = P' ^ Z

        // P'(u) = p'(u) + (Offset / R**2) * (DNdir/DU * R -  Ndir * (DR/R))

        Xy ndir = new Xy(d1.Y(), -d1.X());
        Xy dNdir = new Xy(d2.Y(), -d2.X());
        double r2 = ndir.SquareModulus();
        double r = Math.Sqrt(r2);
        double r3 = r * r2;
        double dr = ndir.Dot(dNdir);
        if (r3 <= Constants.Resolution())
        {
            if (r2 <= Constants.Resolution())
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
        value.ChangeCoord().Add(ndir);
        // P'(u)
        d1.Add(new Vector2D(dNdir));
        return true;
    }

    /// <summary>
    /// The offset point and first two derivatives, from the basis point, D1, D2 (updated in place) and D3.
    /// theIsDirChange reverses D2 (set by <see cref="AdjustDerivative(Curve2D, int, double, ref Vector2D, ref Vector2D, ref Vector2D, ref Vector2D, out bool)"/>).
    /// </summary>
    internal static bool CalculateD2(ref Point2D value,
                                     ref Vector2D d1,
                                     ref Vector2D d2,
                                     in Vector2D d3,
                                     bool isDirChange,
                                     double offset)
    {
        // P(u) = p(u) + Offset * Ndir / R
        // with R = || p' ^ Z|| and Ndir = P' ^ Z

        // P'(u) = p'(u) + (Offset / R**2) * (DNdir/DU * R -  Ndir * (DR/R))

        // P"(u) = p"(u) + (Offset / R) * (D2Ndir/DU - DNdir * (2.0 * Dr/ R**2) +
        //         Ndir * ( (3.0 * Dr**2 / R**4) - (D2r / R**2)))

        Xy ndir = new Xy(d1.Y(), -d1.X());
        Xy dNdir = new Xy(d2.Y(), -d2.X());
        Xy d2Ndir = new Xy(d3.Y(), -d3.X());
        double r2 = ndir.SquareModulus();
        double r = Math.Sqrt(r2);
        double r3 = r2 * r;
        double r4 = r2 * r2;
        double r5 = r3 * r2;
        double dr = ndir.Dot(dNdir);
        double d2r = ndir.Dot(d2Ndir) + dNdir.Dot(dNdir);
        if (r5 <= Constants.Resolution())
        {
            if (r4 <= Constants.Resolution())
            {
                return false;
            }
            // We try another computation but the stability is not very good dixit ISG.
            //  V2 = P" (U) :
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

            // V1 = P' (U)
            dNdir.Multiply(offset / r);
            dNdir.Subtract(ndir.Multiplied(offset * dr / r3));
        }

        ndir.Multiply(offset / r);
        // P(u)
        value.ChangeCoord().Add(ndir);
        // P'(u) :
        d1.Add(new Vector2D(dNdir));
        // P"(u) :
        if (isDirChange)
        {
            d2.Reverse();
        }
        d2.Add(new Vector2D(d2Ndir));
        return true;
    }

    /// <summary>
    /// The offset point and first three derivatives, from the basis point, D1, D2, D3 (updated in place) and D4.
    /// theIsDirChange reverses D3.
    /// </summary>
    internal static bool CalculateD3(ref Point2D value,
                                     ref Vector2D d1,
                                     ref Vector2D d2,
                                     ref Vector2D d3,
                                     in Vector2D d4,
                                     bool isDirChange,
                                     double offset)
    {
        // P(u) = p(u) + Offset * Ndir / R
        // with R = || p' ^ Z|| and Ndir = P' ^ Z

        // P'(u)  = p'(u) + (Offset / R**2) * (DNdir/DU * R -  Ndir * (DR/R))

        // P"(u)  = p"(u) + (Offset / R) * (D2Ndir/DU - DNdir * (2.0 * Dr/ R**2) +
        //          Ndir * ( (3.0 * Dr**2 / R**4) - (D2r / R**2)))

        // P"'(u) = p"'(u) + (Offset / R) * (D3Ndir - (3.0 * Dr/R**2 ) * D2Ndir -
        //          (3.0 * D2r / R2) * DNdir) + (3.0 * Dr * Dr / R4) * DNdir -
        //          (D3r/R2) * Ndir + (6.0 * Dr * Dr / R4) * Ndir +
        //          (6.0 * Dr * D2r / R4) * Ndir - (15.0 * Dr* Dr* Dr /R6) * Ndir

        Xy ndir = new Xy(d1.Y(), -d1.X());
        Xy dNdir = new Xy(d2.Y(), -d2.X());
        Xy d2Ndir = new Xy(d3.Y(), -d3.X());
        Xy d3Ndir = new Xy(d4.Y(), -d4.X());
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

        if (r7 <= Constants.Resolution())
        {
            if (r6 <= Constants.Resolution())
            {
                return false;
            }
            // We try another computation but the stability is not very good dixit ISG.
            // OCCT: the DNdir and Ndir coefficients of V3 below don't match the derivative of V2 (see
            // occt-issues.md); V3 is right only where |p'| is constant (Dr = D2r = D3r = 0, e.g. a circle).
            // V3 = P"' (U) :
            d3Ndir.Subtract(d2Ndir.Multiplied(3.0 * dr / r2));
            d3Ndir.Subtract(dNdir.Multiplied(3.0 * ((d2r / r2) + (dr * dr / r4))));
            d3Ndir.Add(
                ndir.Multiplied(6.0 * dr * dr / r4 + 6.0 * dr * d2r / r4 - 15.0 * dr * dr * dr / r6 - d3r));
            d3Ndir.Multiply(offset / r);
            // V2 = P" (U) :
            r4 = r2 * r2;
            // OCCT: subtracts the Ndir term that CalculateD2 adds (see occt-issues.md).
            d2Ndir.Subtract(dNdir.Multiplied(2.0 * dr / r2));
            d2Ndir.Subtract(ndir.Multiplied((3.0 * dr * dr / r4) - (d2r / r2)));
            d2Ndir.Multiply(offset / r);
            // V1 = P' (U) :
            dNdir.Multiply(r);
            dNdir.Subtract(ndir.Multiplied(dr / r));
            dNdir.Multiply(offset / r2);
        }
        else
        {
            // Same computation as IICURV in EUCLID-IS because the stability is better.
            // OCCT: the DNdir and Ndir coefficients of V3 below don't match the derivative of V2 (see
            // occt-issues.md); V3 is right only where |p'| is constant (Dr = D2r = D3r = 0, e.g. a circle).
            // V3 = P"' (U) :
            d3Ndir.Multiply(offset / r);
            d3Ndir.Subtract(d2Ndir.Multiplied(3.0 * offset * dr / r3));
            d3Ndir.Subtract(dNdir.Multiplied(((3.0 * offset) * ((d2r / r3) + (dr * dr) / r5))));
            d3Ndir.Add(ndir.Multiplied(
                (offset * (6.0 * dr * dr / r5 + 6.0 * dr * d2r / r5 - 15.0 * dr * dr * dr / r7 - d3r))));
            // V2 = P" (U) :
            d2Ndir.Multiply(offset / r);
            d2Ndir.Subtract(dNdir.Multiplied(2.0 * offset * dr / r3));
            // OCCT: subtracts the Ndir term that CalculateD2 adds (see occt-issues.md).
            d2Ndir.Subtract(ndir.Multiplied(offset * (((3.0 * dr * dr) / r5) - (d2r / r3))));
            // V1 = P' (U) :
            dNdir.Multiply(offset / r);
            dNdir.Subtract(ndir.Multiplied(offset * dr / r3));
        }

        ndir.Multiply(offset / r);
        // P(u)
        value.ChangeCoord().Add(ndir);
        // P'(u) :
        d1.Add(new Vector2D(dNdir));
        // P"(u)
        d2.Add(new Vector2D(d2Ndir));
        // P"'(u)
        if (isDirChange)
        {
            d3.Reverse();
        }
        d3.Add(new Vector2D(d3Ndir));
        return true;
    }

    /// <summary>
    /// Replaces null first derivatives at a singular point by the first non-null higher derivative (Taylor
    /// series), oriented along the curve; theIsDirectionChange tells whether it had to be reversed. Computes
    /// theMaxDerivative (3 or 4) derivatives.
    /// </summary>
    internal static bool AdjustDerivative(Curve2D curve,
                                          int maxDerivative,
                                          double theU,
                                          ref Vector2D d1,
                                          ref Vector2D d2,
                                          ref Vector2D d3,
                                          ref Vector2D d4,
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
        Vector2D v;

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

        Point2D p1;
        Point2D p2;
        p1 = curve.EvalD0(Min(theU, u));
        p2 = curve.EvalD0(Max(theU, u));

        Vector2D v1 = new Vector2D(p1, p2);
        isDirectionChange = v.Dot(v1) < 0.0;
        double sign = isDirectionChange ? -1.0 : 1.0;

        d1 = v * sign;
        for (int i = 1; i < maxDerivative; i++)
        {
            Vector2D dn = curve.EvalDN(theU, index + i);
            switch (i - 1)
            {
                case 0: d2 = dn * sign; break;
                case 1: d3 = dn * sign; break;
                default: d4 = dn * sign; break;
            }
        }

        return true;
    }

    /// <summary>The adaptor instantiation of <see cref="AdjustDerivative(Curve2D, int, double, ref Vector2D, ref Vector2D, ref Vector2D, ref Vector2D, out bool)"/>.</summary>
    internal static bool AdjustDerivative(Curve2DAdaptorBase curve,
                                          int maxDerivative,
                                          double theU,
                                          ref Vector2D d1,
                                          ref Vector2D d2,
                                          ref Vector2D d3,
                                          ref Vector2D d4,
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
        Vector2D v;

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

        Point2D p1;
        Point2D p2;
        p1 = curve.EvalD0(Min(theU, u));
        p2 = curve.EvalD0(Max(theU, u));

        Vector2D v1 = new Vector2D(p1, p2);
        isDirectionChange = v.Dot(v1) < 0.0;
        double sign = isDirectionChange ? -1.0 : 1.0;

        d1 = v * sign;
        for (int i = 1; i < maxDerivative; i++)
        {
            Vector2D dn = curve.EvalDN(theU, index + i);
            switch (i - 1)
            {
                case 0: d2 = dn * sign; break;
                case 1: d3 = dn * sign; break;
                default: d4 = dn * sign; break;
            }
        }

        return true;
    }

    // EvaluateD0 ... EvaluateDN: the Geom2d_Curve instantiation.

    /// <summary>The offset point at theU from the basis curve's D1; false if the normal is undefined.</summary>
    internal static bool EvaluateD0(double u, Curve2D basisCurve, double offset, out Point2D value)
    {
        Curve2D.ResD1 d1 = basisCurve.EvalD1(u);
        value = d1.Point;
        return CalculateD0(ref value, d1.D1, offset);
    }

    /// <summary>The offset point and D1 at theU from the basis curve's D2; false if the computation failed.</summary>
    internal static bool EvaluateD1(double u,
                                    Curve2D basisCurve,
                                    double offset,
                                    out Point2D value,
                                    out Vector2D d1)
    {
        Curve2D.ResD2 d2 = basisCurve.EvalD2(u);
        value = d2.Point;
        d1 = d2.D1;
        return CalculateD1(ref value, ref d1, d2.D2, offset);
    }

    /// <summary>The offset point, D1 and D2 at theU (handling a singular basis point); false if the computation failed.</summary>
    internal static bool EvaluateD2(double u,
                                    Curve2D basisCurve,
                                    double offset,
                                    out Point2D value,
                                    out Vector2D d1,
                                    out Vector2D d2)
    {
        Curve2D.ResD3 d3Result = basisCurve.EvalD3(u);
        value = d3Result.Point;
        d1 = d3Result.D1;
        d2 = d3Result.D2;
        Vector2D d3 = d3Result.D3;

        bool isDirectionChange = false;
        if (d1.SquareMagnitude() <= Constants.Resolution())
        {
            Vector2D dummyD4 = new Vector2D();
            if (!AdjustDerivative(basisCurve, 3, u, ref d1, ref d2, ref d3, ref dummyD4, out isDirectionChange))
            {
                return false;
            }
        }

        return CalculateD2(ref value, ref d1, ref d2, d3, isDirectionChange, offset);
    }

    /// <summary>The offset point and D1 to D3 at theU (handling a singular basis point); false if the computation failed.</summary>
    internal static bool EvaluateD3(double u,
                                    Curve2D basisCurve,
                                    double offset,
                                    out Point2D value,
                                    out Vector2D d1,
                                    out Vector2D d2,
                                    out Vector2D d3)
    {
        Curve2D.ResD3 d3Result = basisCurve.EvalD3(u);
        value = d3Result.Point;
        d1 = d3Result.D1;
        d2 = d3Result.D2;
        d3 = d3Result.D3;
        Vector2D d4 = basisCurve.EvalDN(u, 4);

        bool isDirectionChange = false;
        if (d1.SquareMagnitude() <= Constants.Resolution())
        {
            if (!AdjustDerivative(basisCurve, 4, u, ref d1, ref d2, ref d3, ref d4, out isDirectionChange))
            {
                return false;
            }
        }

        return CalculateD3(ref value, ref d1, ref d2, ref d3, d4, isDirectionChange, offset);
    }

    /// <summary>
    /// The Nth derivative of the offset curve at theU: through EvaluateD1 to D3 up to N = 3, the basis curve's
    /// derivative above (OCCT neglects the offset's contribution there).
    /// </summary>
    internal static bool EvaluateDN(double u, Curve2D basisCurve, double offset, int n, out Vector2D dn)
    {
        Point2D pnt;
        Vector2D dummy;
        switch (n)
        {
            case 1:
                return EvaluateD1(u, basisCurve, offset, out pnt, out dn);
            case 2:
                return EvaluateD2(u, basisCurve, offset, out pnt, out dummy, out dn);
            case 3:
                // OCCT: one variable receives both D1 and D2, which EvaluateD3 uses while computing D3
                return EvaluateD3(u, basisCurve, offset, out pnt, out dummy, out dummy, out dn);
            default:
            {
                // For derivatives > 3, return basis curve derivative (no offset contribution)
                dn = basisCurve.EvalDN(u, n);
                return true;
            }
        }
    }

    // EvaluateD0 ... EvaluateDN: the Adaptor2d_Curve2d instantiation.

    /// <summary>The offset point at theU from the basis adaptor's D1; false if the normal is undefined.</summary>
    internal static bool EvaluateD0(double u, Curve2DAdaptorBase basisCurve, double offset, out Point2D value)
    {
        Curve2D.ResD1 d1 = basisCurve.EvalD1(u);
        value = d1.Point;
        return CalculateD0(ref value, d1.D1, offset);
    }

    /// <summary>The offset point and D1 at theU from the basis adaptor's D2.</summary>
    internal static bool EvaluateD1(double u,
                                    Curve2DAdaptorBase basisCurve,
                                    double offset,
                                    out Point2D value,
                                    out Vector2D d1)
    {
        Curve2D.ResD2 d2 = basisCurve.EvalD2(u);
        value = d2.Point;
        d1 = d2.D1;
        return CalculateD1(ref value, ref d1, d2.D2, offset);
    }

    /// <summary>The offset point, D1 and D2 at theU from the basis adaptor (handling a singular basis point).</summary>
    internal static bool EvaluateD2(double u,
                                    Curve2DAdaptorBase basisCurve,
                                    double offset,
                                    out Point2D value,
                                    out Vector2D d1,
                                    out Vector2D d2)
    {
        Curve2D.ResD3 d3Result = basisCurve.EvalD3(u);
        value = d3Result.Point;
        d1 = d3Result.D1;
        d2 = d3Result.D2;
        Vector2D d3 = d3Result.D3;

        bool isDirectionChange = false;
        if (d1.SquareMagnitude() <= Constants.Resolution())
        {
            Vector2D dummyD4 = new Vector2D();
            if (!AdjustDerivative(basisCurve, 3, u, ref d1, ref d2, ref d3, ref dummyD4, out isDirectionChange))
            {
                return false;
            }
        }

        return CalculateD2(ref value, ref d1, ref d2, d3, isDirectionChange, offset);
    }

    /// <summary>The offset point and D1 to D3 at theU from the basis adaptor (handling a singular basis point).</summary>
    internal static bool EvaluateD3(double u,
                                    Curve2DAdaptorBase basisCurve,
                                    double offset,
                                    out Point2D value,
                                    out Vector2D d1,
                                    out Vector2D d2,
                                    out Vector2D d3)
    {
        Curve2D.ResD3 d3Result = basisCurve.EvalD3(u);
        value = d3Result.Point;
        d1 = d3Result.D1;
        d2 = d3Result.D2;
        d3 = d3Result.D3;
        Vector2D d4 = basisCurve.EvalDN(u, 4);

        bool isDirectionChange = false;
        if (d1.SquareMagnitude() <= Constants.Resolution())
        {
            if (!AdjustDerivative(basisCurve, 4, u, ref d1, ref d2, ref d3, ref d4, out isDirectionChange))
            {
                return false;
            }
        }

        return CalculateD3(ref value, ref d1, ref d2, ref d3, d4, isDirectionChange, offset);
    }

    /// <summary>The Nth derivative of the offset of the basis adaptor at theU (see the Geom2d_Curve overload).</summary>
    internal static bool EvaluateDN(double u, Curve2DAdaptorBase basisCurve, double offset, int n, out Vector2D dn)
    {
        Point2D pnt;
        Vector2D dummy;
        switch (n)
        {
            case 1:
                return EvaluateD1(u, basisCurve, offset, out pnt, out dn);
            case 2:
                return EvaluateD2(u, basisCurve, offset, out pnt, out dummy, out dn);
            case 3:
                // OCCT: one variable receives both D1 and D2, which EvaluateD3 uses while computing D3
                return EvaluateD3(u, basisCurve, offset, out pnt, out dummy, out dummy, out dn);
            default:
            {
                // For derivatives > 3, return basis curve derivative (no offset contribution)
                dn = basisCurve.EvalDN(u, n);
                return true;
            }
        }
    }
}
