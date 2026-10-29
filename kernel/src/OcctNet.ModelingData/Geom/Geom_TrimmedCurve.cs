// Port of: src/ModelingData/TKG3d/Geom/Geom_TrimmedCurve.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_TrimmedCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A portion of a basis curve limited by two parameter values inside its parametric domain. The trimmed curve
/// has the parametrization of its basis curve, between its first and last parameters; it keeps a copy of the
/// basis curve (a trimmed basis is replaced by its own basis). The derivatives have the orientation of the
/// basis curve.
/// </summary>
public class Geom_TrimmedCurve : Geom_BoundedCurve
{
    private Geom_Curve basisCurve;
    private double uTrim1;
    private double uTrim2;

    /// <summary>
    /// The part of a copy of C between U1 and U2, oriented from U1 to U2. On a periodic basis, Sense chooses
    /// between the two arcs (false: opposite to the basis orientation), and theAdjustPeriodic brings the
    /// parameters into the first period (they may then differ from U1 and U2 by a period).
    /// </summary>
    /// <exception cref="Standard_ConstructionError">U1 == U2, or C is not periodic and U1 or U2 is out of its bounds.</exception>
    public Geom_TrimmedCurve(Geom_Curve C, double U1, double U2, bool Sense = true, bool theAdjustPeriodic = true)
    {
        uTrim1 = U1;
        uTrim2 = U2;
        // kill trimmed basis curves
        Geom_TrimmedCurve? T = C as Geom_TrimmedCurve;
        if (T is not null)
        {
            basisCurve = (Geom_Curve)T.BasisCurve().Copy();
        }
        else
        {
            basisCurve = (Geom_Curve)C.Copy();
        }
        SetTrim(U1, U2, Sense, theAdjustPeriodic);
    }

    /// <summary>A new, unfrozen trimmed curve equal to this one (on a copy of the basis curve).</summary>
    public sealed override Geom_Geometry Copy()
    {
        Geom_TrimmedCurve Tc = new Geom_TrimmedCurve(basisCurve, uTrim1, uTrim2);
        return Tc;
    }

    /// <summary>
    /// Reverses the orientation: the basis curve is reversed and the trim parameters are recomputed
    /// (1 - U2 and 1 - U1 for a basis on [0, 1]).
    /// </summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        double U1 = basisCurve.ReversedParameter(uTrim2);
        double U2 = basisCurve.ReversedParameter(uTrim1);
        basisCurve.Reverse();
        SetTrim(U1, U2, true, false);
    }

    /// <summary>The parameter on the reversed curve (the basis curve's).</summary>
    public sealed override double ReversedParameter(double U) => basisCurve.ReversedParameter(U);

    /// <summary>Changes the trim parameters (see the constructor).</summary>
    /// <exception cref="Standard_ConstructionError">U1 == U2, or the basis is not periodic and U1 or U2 is out of its bounds.</exception>
    internal void SetTrim(double U1, double U2, bool Sense = true, bool theAdjustPeriodic = true)
    {
        CheckMutable();
        bool sameSense = true;
        if (U1 == U2)
        {
            throw new Standard_ConstructionError("Geom_TrimmedCurve::U1 == U2");
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
                throw new Standard_ConstructionError("Geom_TrimmedCurve::parameters out of range");
            }
        }
        if (!sameSense)
        {
            Reverse();
        }
    }

    /// <summary>
    /// True if the basis is periodic and the trim spans a whole number of periods, or if the start and end
    /// points are within Precision::Computational() (squared distance).
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
        return StartPoint().SquareDistance(EndPoint()) <= Precision.Computational();
    }

    /// <summary>True if the basis is periodic and the trim spans a whole number of periods.</summary>
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

    /// <summary>The period of the basis curve.</summary>
    /// <exception cref="Standard_NoSuchObject">The basis curve is not periodic.</exception>
    public sealed override double Period() => basisCurve.Period();

    public sealed override GeomAbs_Shape Continuity() => basisCurve.Continuity();

    /// <summary>
    /// The basis curve, not a copy, as in OCCT. Callers can't change it (its mutators are internal); code inside
    /// OCCT.NET that changes it changes this trimmed curve.
    /// </summary>
    public Geom_Curve BasisCurve() => basisCurve;

    public sealed override gp_Pnt EvalD0(double U) => basisCurve.EvalD0(U);

    public sealed override ResD1 EvalD1(double U) => basisCurve.EvalD1(U);

    public sealed override ResD2 EvalD2(double U) => basisCurve.EvalD2(U);

    public sealed override ResD3 EvalD3(double U) => basisCurve.EvalD3(U);

    public sealed override gp_Vec EvalDN(double U, int N) => basisCurve.EvalDN(U, N);

    public sealed override gp_Pnt EndPoint() => basisCurve.Value(uTrim2);

    public sealed override double FirstParameter() => uTrim1;

    public sealed override double LastParameter() => uTrim2;

    public sealed override gp_Pnt StartPoint() => basisCurve.Value(uTrim1);

    /// <exception cref="Standard_RangeError">N &lt; 0.</exception>
    public sealed override bool IsCN(int N)
    {
        Standard_RangeError_Raise_if(N < 0, " ");
        return basisCurve.IsCN(N);
    }

    /// <summary>Applies T to this trimmed curve and its basis curve.</summary>
    protected internal sealed override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        basisCurve.Transform(T);
        double U1 = basisCurve.TransformedParameter(uTrim1, T);
        double U2 = basisCurve.TransformedParameter(uTrim2, T);
        SetTrim(U1, U2, true, false);
    }

    /// <summary>The basis curve's transformed parameter.</summary>
    public sealed override double TransformedParameter(double U, in gp_Trsf T) => basisCurve.TransformedParameter(U, T);

    /// <summary>The basis curve's parametric transformation.</summary>
    public sealed override double ParametricTransformation(in gp_Trsf T) => basisCurve.ParametricTransformation(T);

    // Non-mutating twin of SetTrim (OCCT.NET)

    /// <summary>A copy trimmed between U1 and U2 of the same basis curve (see the constructor).</summary>
    public Geom_TrimmedCurve WithTrim(double U1, double U2, bool Sense = true, bool theAdjustPeriodic = true)
    {
        Geom_TrimmedCurve aCopy = (Geom_TrimmedCurve)Copy();
        aCopy.SetTrim(U1, U2, Sense, theAdjustPeriodic);
        return aCopy;
    }
}
