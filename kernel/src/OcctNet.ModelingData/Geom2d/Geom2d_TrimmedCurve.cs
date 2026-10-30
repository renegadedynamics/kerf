// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_TrimmedCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_TrimmedCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A portion of a curve between two parameters U1 and U2 of a basis curve, with the basis curve's
/// parametrization. The trimmed curve is oriented as the basis curve when <c>Sense</c> is true, and opposite
/// otherwise (the basis curve is then reversed). For a periodic basis curve the parameters are first adjusted to
/// its period.
/// </summary>
/// <remarks>
/// The trimmed curve keeps its own copy of the basis curve (OCCT copies it in the constructor; a trimmed basis
/// curve is replaced by its own basis). <see cref="BasisCurve"/> returns that copy frozen, as OCCT returns its
/// handle; this curve's internal mutators then copy the basis curve again before changing it (decision 132).
/// </remarks>
public class Geom2d_TrimmedCurve : Geom2d_BoundedCurve
{
    private Geom2d_Curve basisCurve;
    private double uTrim1;
    private double uTrim2;

    /// <summary>A new, unfrozen trimmed curve equal to this one (with its own copy of the basis curve).</summary>
    public sealed override Geom2d_Geometry Copy()
    {
        Geom2d_TrimmedCurve Tc;
        Tc = new Geom2d_TrimmedCurve(basisCurve, uTrim1, uTrim2);
        return Tc;
    }

    /// <summary>
    /// The portion of C between U1 and U2. If C is periodic and theAdjustPeriodic is true, U1 and U2 are brought
    /// into C's period (U2 within one period after U1); Sense false gives the curve from U1 to U2 the other way.
    /// If C is not periodic, the curve goes from min(U1, U2) to max(U1, U2), in C's sense if Sense is true
    /// and U1 &lt; U2.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">
    /// C is null, U1 == U2, or (C not periodic) U1 or U2 is outside C's bounds.
    /// </exception>
    public Geom2d_TrimmedCurve(Geom2d_Curve C, double U1, double U2, bool Sense = true, bool theAdjustPeriodic = true)
    {
        uTrim1 = U1;
        uTrim2 = U2;
        if (C is null)
        {
            throw new Standard_ConstructionError("Geom2d_TrimmedCurve:: C is null");
        }
        // kill trimmed basis curves
        Geom2d_TrimmedCurve? T = C as Geom2d_TrimmedCurve;
        if (T is not null)
        {
            basisCurve = (Geom2d_Curve)T.basisCurve.Copy(); // OCCT.NET: the field, so that T's basis curve isn't frozen
        }
        else
        {
            basisCurve = (Geom2d_Curve)C.Copy();
        }
        SetTrim(U1, U2, Sense, theAdjustPeriodic);
    }

    // OCCT.NET: the basis curve may have been handed out (and frozen) by BasisCurve(); copy it before changing it.
    private void ownBasisCurve()
    {
        if (basisCurve.IsFrozen())
        {
            basisCurve = (Geom2d_Curve)basisCurve.Copy();
        }
    }

    /// <summary>Reverses the trimmed curve: the basis curve is reversed and the trim parameters changed accordingly.</summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        ownBasisCurve();
        double U1 = basisCurve.ReversedParameter(uTrim2);
        double U2 = basisCurve.ReversedParameter(uTrim1);
        basisCurve.Reverse();
        SetTrim(U1, U2, true, false);
    }

    /// <summary>The basis curve's reversed parameter of U.</summary>
    public sealed override double ReversedParameter(double U) => basisCurve.ReversedParameter(U);

    /// <summary>Changes the trim parameters, as the constructor sets them.</summary>
    /// <exception cref="Standard_ConstructionError">
    /// U1 == U2, or (basis curve not periodic) U1 or U2 is outside its bounds.
    /// </exception>
    internal void SetTrim(double U1, double U2, bool Sense = true, bool theAdjustPeriodic = true)
    {
        CheckMutable();
        bool sameSense = true;
        if (U1 == U2)
        {
            throw new Standard_ConstructionError("Geom2d_TrimmedCurve::U1 == U2");
        }

        double Udeb = basisCurve.FirstParameter();
        double Ufin = basisCurve.LastParameter();

        if (basisCurve.IsPeriodic())
        {
            sameSense = Sense;

            // set uTrim1 in the range Udeb , Ufin
            // set uTrim2 in the range uTrim1 , uTrim1 + Period()
            uTrim1 = U1;
            uTrim2 = U2;
            if (theAdjustPeriodic)
            {
                ElCLib.AdjustPeriodic(Udeb,
                                      Ufin,
                                      Min(Math.Abs(uTrim2 - uTrim1) / 2, Precision.PConfusion()),
                                      ref uTrim1,
                                      ref uTrim2);
            }
        }
        else
        {
            if (U1 < U2)
            {
                sameSense = Sense;
                uTrim1 = U1;
                uTrim2 = U2;
            }
            else
            {
                sameSense = !Sense;
                uTrim1 = U2;
                uTrim2 = U1;
            }

            if ((Udeb - uTrim1 > Precision.PConfusion()) || (uTrim2 - Ufin > Precision.PConfusion()))
            {
                // OCCT: the message names Geom_TrimmedCurve (cosmetic).
                throw new Standard_ConstructionError("Geom_TrimmedCurve::parameters out of range");
            }
        }
        if (!sameSense)
        {
            Reverse();
        }
    }

    /// <summary>A copy with the trim parameters U1 and U2 (see the constructor).</summary>
    public Geom2d_TrimmedCurve WithTrim(double U1, double U2, bool Sense = true, bool theAdjustPeriodic = true)
    {
        Geom2d_TrimmedCurve aCopy = (Geom2d_TrimmedCurve)Copy();
        aCopy.SetTrim(U1, U2, Sense, theAdjustPeriodic);
        return aCopy;
    }

    /// <summary>The basis curve, frozen (it is shared with this trimmed curve).</summary>
    public Geom2d_Curve BasisCurve()
    {
        basisCurve.Freeze();
        return basisCurve;
    }

    /// <summary>
    /// The basis curve, not frozen: for ported code that changes it in place through the trimmed curve, as OCCT does
    /// (MAT2d's <c>Bisector_BisecAna::SetTrim</c> on a Bisector_Bisec's curve), and for reads that must not freeze it.
    /// </summary>
    internal Geom2d_Curve ChangeBasisCurve() => basisCurve;

    /// <summary>The basis curve's continuity.</summary>
    public sealed override GeomAbs_Shape Continuity() => basisCurve.Continuity();

    /// <summary>The basis curve's IsCN.</summary>
    /// <exception cref="Standard_RangeError">N &lt; 0.</exception>
    public sealed override bool IsCN(int N)
    {
        Standard_RangeError_Raise_if(N < 0, " ");
        return basisCurve.IsCN(N);
    }

    /// <summary>The point of the basis curve at the last trim parameter.</summary>
    public sealed override gp_Pnt2d EndPoint() => basisCurve.Value(uTrim2);

    /// <summary>The first trim parameter.</summary>
    public sealed override double FirstParameter() => uTrim1;

    /// <summary>
    /// True if the trimmed range of a periodic basis curve is a whole number of periods, or if the end points are
    /// within Precision::Computational() (squared distance).
    /// </summary>
    public sealed override bool IsClosed()
    {
        if (basisCurve.IsPeriodic())
        {
            double aPeriod = basisCurve.Period();
            double aLength = LastParameter() - FirstParameter();
            if (aLength > Precision.PConfusion()
                && Math.Abs(Math.IEEERemainder(aLength, aPeriod)) <= Precision.PConfusion())
            {
                return true;
            }
        }
        return Value(FirstParameter()).SquareDistance(Value(LastParameter()))
               <= Precision.Computational();
    }

    /// <summary>True if the basis curve is periodic and the trimmed range is a whole number of periods.</summary>
    public sealed override bool IsPeriodic()
    {
        if (basisCurve.IsPeriodic())
        {
            double aPeriod = basisCurve.Period();
            double aLength = LastParameter() - FirstParameter();
            if (aLength > Precision.PConfusion()
                && Math.Abs(Math.IEEERemainder(aLength, aPeriod)) <= Precision.PConfusion())
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>The basis curve's period.</summary>
    /// <exception cref="Standard_NoSuchObject">The basis curve is not periodic.</exception>
    public sealed override double Period() => basisCurve.Period();

    /// <summary>The last trim parameter.</summary>
    public sealed override double LastParameter() => uTrim2;

    /// <summary>The point of the basis curve at the first trim parameter.</summary>
    public sealed override gp_Pnt2d StartPoint()
    {
        gp_Pnt2d P;
        P = basisCurve.Value(uTrim1);
        return P;
    }

    public sealed override gp_Pnt2d EvalD0(double U) => basisCurve.EvalD0(U);

    public sealed override ResD1 EvalD1(double U) => basisCurve.EvalD1(U);

    public sealed override ResD2 EvalD2(double U) => basisCurve.EvalD2(U);

    public sealed override ResD3 EvalD3(double U) => basisCurve.EvalD3(U);

    public sealed override gp_Vec2d EvalDN(double U, int N) => basisCurve.EvalDN(U, N);

    /// <summary>Transforms the basis curve and the trim parameters.</summary>
    protected internal sealed override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        ownBasisCurve();
        basisCurve.Transform(T);
        double U1 = basisCurve.TransformedParameter(uTrim1, T);
        double U2 = basisCurve.TransformedParameter(uTrim2, T);
        SetTrim(U1, U2, true, false);
    }

    /// <summary>The basis curve's transformed parameter.</summary>
    public sealed override double TransformedParameter(double U, in gp_Trsf2d T) =>
        basisCurve.TransformedParameter(U, T);

    /// <summary>The basis curve's parametric transformation.</summary>
    public sealed override double ParametricTransformation(in gp_Trsf2d T) =>
        basisCurve.ParametricTransformation(T);
}
