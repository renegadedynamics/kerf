// Port of: src/ModelingData/TKG3d/Geom/Geom_RectangularTrimmedSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_RectangularTrimmedSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A portion of a basis surface limited by two values of U, two values of V, or both (a rectangle of its
/// parametric domain). The trimmed surface has the parametrization of its basis surface, and keeps a copy of it
/// (a trimmed basis is replaced by its own basis).
/// </summary>
/// <remarks>
/// <see cref="BasisSurface"/> returns the stored copy frozen, as OCCT returns its handle; this surface's internal
/// mutators then copy the basis surface again before changing it (decision 159).
/// </remarks>
public class Geom_RectangularTrimmedSurface : Geom_BoundedSurface
{
    private Geom_Surface basisSurf;
    private double utrim1;
    private double vtrim1;
    private double utrim2;
    private double vtrim2;
    private bool isutrimmed;
    private bool isvtrimmed;

    /// <summary>
    /// The part of a copy of S between U1 and U2 and between V1 and V2. USense and VSense choose, for a periodic
    /// direction, between the two portions (false: opposite to the basis orientation); for a non-periodic one, they
    /// say whether the trimmed surface keeps the basis orientation (U1 &lt; U2 with USense true, or U1 &gt; U2 with
    /// USense false), otherwise it is reversed.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">
    /// U1 == U2 or V1 == V2, or a non-periodic direction is trimmed outside the bounds of S.
    /// </exception>
    public Geom_RectangularTrimmedSurface(Geom_Surface S,
                                          double U1,
                                          double U2,
                                          double V1,
                                          double V2,
                                          bool USense = true,
                                          bool VSense = true)
    {
        utrim1 = U1;
        vtrim1 = V1;
        utrim2 = U2;
        vtrim2 = V2;
        isutrimmed = true;
        isvtrimmed = true;

        // kill trimmed basis surfaces
        Geom_RectangularTrimmedSurface? T = S as Geom_RectangularTrimmedSurface;
        if (T is not null)
        {
            basisSurf = (Geom_Surface)T.basisSurf.Copy(); // OCCT.NET: the field, so that T's basis surface isn't frozen
        }
        else
        {
            basisSurf = (Geom_Surface)S.Copy();
        }

        Geom_OffsetSurface? O = basisSurf as Geom_OffsetSurface;
        if (O is not null)
        {
            Geom_RectangularTrimmedSurface S2 =
                new Geom_RectangularTrimmedSurface(O.BasisSurface(), U1, U2, V1, V2, USense, VSense);
            basisSurf = new Geom_OffsetSurface(S2, O.Offset(), true);
        }

        SetTrim(U1, U2, V1, V2, USense, VSense);
    }

    /// <summary>
    /// The part of a copy of S between Param1 and Param2 in U (UTrim true) or in V (UTrim false); the other
    /// direction is not trimmed, unless S is itself trimmed in it. Sense as in the other constructor.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">
    /// Param1 == Param2, or the direction is not periodic and is trimmed outside the bounds of S.
    /// </exception>
    public Geom_RectangularTrimmedSurface(Geom_Surface S,
                                          double Param1,
                                          double Param2,
                                          bool UTrim,
                                          bool Sense = true)
    {
        // kill trimmed basis surfaces
        Geom_RectangularTrimmedSurface? T = S as Geom_RectangularTrimmedSurface;
        if (T is not null)
        {
            basisSurf = (Geom_Surface)T.basisSurf.Copy(); // OCCT.NET: the field, so that T's basis surface isn't frozen
        }
        else
        {
            basisSurf = (Geom_Surface)S.Copy();
        }

        Geom_OffsetSurface? O = basisSurf as Geom_OffsetSurface;
        if (O is not null)
        {
            Geom_RectangularTrimmedSurface S2 =
                new Geom_RectangularTrimmedSurface(O.BasisSurface(), Param1, Param2, UTrim, Sense);
            basisSurf = new Geom_OffsetSurface(S2, O.Offset(), true);
        }

        if (T is not null)
        {
            if (UTrim && T.isvtrimmed)
            {
                SetTrim(Param1, Param2, T.vtrim1, T.vtrim2, Sense, true);
                return;
            }
            else if (!UTrim && T.isutrimmed)
            {
                SetTrim(T.utrim1, T.utrim2, Param1, Param2, true, Sense);
                return;
            }
        }

        // OCCT reads isutrimmed/isvtrimmed (and utrim*/vtrim*) here before this constructor initialises them. OCCT's
        // allocator clears memory by default (MMGT_CLEAR), so they are false/0 there as in C#.
        SetTrim(Param1, Param2, UTrim, Sense);
    }

    /// <summary>Changes the trim values in both directions (see the first constructor).</summary>
    /// <exception cref="Standard_ConstructionError">As the first constructor.</exception>
    internal void SetTrim(double U1,
                          double U2,
                          double V1,
                          double V2,
                          bool USense = true,
                          bool VSense = true)
    {
        CheckMutable();
        SetTrim(U1, U2, V1, V2, true, true, USense, VSense);
    }

    /// <summary>Changes the trim values in one direction (see the second constructor).</summary>
    /// <exception cref="Standard_ConstructionError">As the second constructor.</exception>
    internal void SetTrim(double Param1, double Param2, bool UTrim, bool Sense = true)
    {
        CheckMutable();
        // dummy arguments to call general SetTrim
        double dummy_a = 0.0;
        double dummy_b = 0.0;
        bool dummy_Sense = true;

        if (UTrim)
        {
            if (isvtrimmed)
            {
                SetTrim(Param1, Param2, vtrim1, vtrim2, true, true, Sense, dummy_Sense);
            }
            else
            {
                SetTrim(Param1, Param2, dummy_a, dummy_b, true, false, Sense, dummy_Sense);
            }
        }
        else
        {
            if (isutrimmed)
            {
                SetTrim(utrim1, utrim2, Param1, Param2, true, true, dummy_Sense, Sense);
            }
            else
            {
                SetTrim(dummy_a, dummy_b, Param1, Param2, false, true, dummy_Sense, Sense);
            }
        }
    }

    private void SetTrim(double U1,
                         double U2,
                         double V1,
                         double V2,
                         bool UTrim,
                         bool VTrim,
                         bool USense,
                         bool VSense)
    {
        bool UsameSense = true;
        bool VsameSense = true;
        double Udeb, Ufin, Vdeb, Vfin;

        basisSurf.Bounds(out Udeb, out Ufin, out Vdeb, out Vfin);

        // Trimming the U-Direction
        isutrimmed = UTrim;
        if (!UTrim)
        {
            utrim1 = Udeb;
            utrim2 = Ufin;
        }
        else
        {
            if (U1 == U2)
            {
                throw new Standard_ConstructionError("Geom_RectangularTrimmedSurface::U1==U2");
            }

            if (basisSurf.IsUPeriodic())
            {
                UsameSense = USense;

                // set uTrim1 in the range Udeb , Ufin
                // set uTrim2 in the range uTrim1 , uTrim1 + Period()
                utrim1 = U1;
                utrim2 = U2;
                ElCLib.AdjustPeriodic(Udeb,
                                      Ufin,
                                      Min(Math.Abs(utrim2 - utrim1) / 2, Precision.PConfusion()),
                                      ref utrim1,
                                      ref utrim2);
            }
            else
            {
                if (U1 < U2)
                {
                    UsameSense = USense;
                    utrim1 = U1;
                    utrim2 = U2;
                }
                else
                {
                    UsameSense = !USense;
                    utrim1 = U2;
                    utrim2 = U1;
                }

                if ((Udeb - utrim1 > Precision.PConfusion()) || (utrim2 - Ufin > Precision.PConfusion()))
                {
                    throw new Standard_ConstructionError(
                        "Geom_RectangularTrimmedSurface::U parameters out of range");
                }
            }
        }

        // Trimming the V-Direction
        isvtrimmed = VTrim;
        if (!VTrim)
        {
            vtrim1 = Vdeb;
            vtrim2 = Vfin;
        }
        else
        {
            if (V1 == V2)
            {
                throw new Standard_ConstructionError("Geom_RectangularTrimmedSurface::V1==V2");
            }

            if (basisSurf.IsVPeriodic())
            {
                VsameSense = VSense;

                // set vTrim1 in the range Vdeb , Vfin
                // set vTrim2 in the range vTrim1 , vTrim1 + Period()
                vtrim1 = V1;
                vtrim2 = V2;
                ElCLib.AdjustPeriodic(Vdeb,
                                      Vfin,
                                      Min(Math.Abs(vtrim2 - vtrim1) / 2, Precision.PConfusion()),
                                      ref vtrim1,
                                      ref vtrim2);
            }
            else
            {
                if (V1 < V2)
                {
                    VsameSense = VSense;
                    vtrim1 = V1;
                    vtrim2 = V2;
                }
                else
                {
                    VsameSense = !VSense;
                    vtrim1 = V2;
                    vtrim2 = V1;
                }

                if ((Vdeb - vtrim1 > Precision.PConfusion()) || (vtrim2 - Vfin > Precision.PConfusion()))
                {
                    throw new Standard_ConstructionError(
                        "Geom_RectangularTrimmedSurface::V parameters out of range");
                }
            }
        }

        if (!UsameSense)
        {
            UReverse();
        }
        if (!VsameSense)
        {
            VReverse();
        }
    }

    /// <summary>A copy trimmed in both directions (see the first constructor).</summary>
    /// <exception cref="Standard_ConstructionError">As the first constructor.</exception>
    public Geom_RectangularTrimmedSurface WithTrim(double U1,
                                                   double U2,
                                                   double V1,
                                                   double V2,
                                                   bool USense = true,
                                                   bool VSense = true)
    {
        Geom_RectangularTrimmedSurface aCopy = (Geom_RectangularTrimmedSurface)Copy();
        aCopy.SetTrim(U1, U2, V1, V2, USense, VSense);
        return aCopy;
    }

    /// <summary>A copy trimmed in one direction (see the second constructor).</summary>
    /// <exception cref="Standard_ConstructionError">As the second constructor.</exception>
    public Geom_RectangularTrimmedSurface WithTrim(double Param1, double Param2, bool UTrim, bool Sense = true)
    {
        Geom_RectangularTrimmedSurface aCopy = (Geom_RectangularTrimmedSurface)Copy();
        aCopy.SetTrim(Param1, Param2, UTrim, Sense);
        return aCopy;
    }

    /// <summary>The basis surface (the trimmed surface's own copy, frozen; shared, not copied).</summary>
    public Geom_Surface BasisSurface()
    {
        basisSurf.Freeze();
        return basisSurf;
    }

    // OCCT.NET: the basis surface may have been handed out (and frozen) by BasisSurface(); copy it before changing it.
    private void ownBasisSurface()
    {
        if (basisSurf.IsFrozen())
        {
            basisSurf = (Geom_Surface)basisSurf.Copy();
        }
    }

    /// <summary>Reverses the U direction: the basis surface is reversed and the U trim values recomputed.</summary>
    protected internal sealed override void UReverse()
    {
        CheckMutable();
        ownBasisSurface();
        double U1 = basisSurf.UReversedParameter(utrim2);
        double U2 = basisSurf.UReversedParameter(utrim1);
        basisSurf.UReverse();
        SetTrim(U1, U2, vtrim1, vtrim2, isutrimmed, isvtrimmed, true, true);
    }

    /// <summary>The reversed U parameter of the basis surface.</summary>
    public sealed override double UReversedParameter(double U) => basisSurf.UReversedParameter(U);

    /// <summary>Reverses the V direction: the basis surface is reversed and the V trim values recomputed.</summary>
    protected internal sealed override void VReverse()
    {
        CheckMutable();
        ownBasisSurface();
        double V1 = basisSurf.VReversedParameter(vtrim2);
        double V2 = basisSurf.VReversedParameter(vtrim1);
        basisSurf.VReverse();
        SetTrim(utrim1, utrim2, V1, V2, isutrimmed, isvtrimmed, true, true);
    }

    /// <summary>The reversed V parameter of the basis surface.</summary>
    public sealed override double VReversedParameter(double V) => basisSurf.VReversedParameter(V);

    /// <summary>The trim values (the bounds of the basis surface in a direction that is not trimmed).</summary>
    public sealed override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = utrim1;
        U2 = utrim2;
        V1 = vtrim1;
        V2 = vtrim2;
    }

    /// <summary>The continuity of the basis surface.</summary>
    public sealed override GeomAbs_Shape Continuity() => basisSurf.Continuity();

    /// <summary>
    /// The basis surface's closure if U is not trimmed; otherwise true only if the basis is U-periodic and the
    /// trimmed range is a non-zero multiple of the period (within Precision::PConfusion()).
    /// </summary>
    public sealed override bool IsUClosed()
    {
        if (!isutrimmed)
        {
            return basisSurf.IsUClosed();
        }

        if (basisSurf.IsUPeriodic())
        {
            double aPeriod = basisSurf.UPeriod();
            double aLength = utrim2 - utrim1;
            if (aLength > Precision.PConfusion()
                && Math.Abs(Math.IEEERemainder(aLength, aPeriod)) <= Precision.PConfusion())
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>As <see cref="IsUClosed"/>, in V.</summary>
    public sealed override bool IsVClosed()
    {
        if (!isvtrimmed)
        {
            return basisSurf.IsVClosed();
        }

        if (basisSurf.IsVPeriodic())
        {
            double aPeriod = basisSurf.VPeriod();
            double aLength = vtrim2 - vtrim1;
            if (aLength > Precision.PConfusion()
                && Math.Abs(Math.IEEERemainder(aLength, aPeriod)) <= Precision.PConfusion())
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>True if the basis surface is CN in U.</summary>
    /// <exception cref="Standard_RangeError">N &lt; 0.</exception>
    public sealed override bool IsCNu(int N)
    {
        Standard_RangeError_Raise_if(N < 0, " ");
        return basisSurf.IsCNu(N);
    }

    /// <summary>True if the basis surface is CN in V.</summary>
    /// <exception cref="Standard_RangeError">N &lt; 0.</exception>
    public sealed override bool IsCNv(int N)
    {
        Standard_RangeError_Raise_if(N < 0, " ");
        return basisSurf.IsCNv(N);
    }

    /// <summary>
    /// True if the basis surface is U-periodic and U is not trimmed, or trimmed over a non-zero multiple of the
    /// period (within Precision::PConfusion()).
    /// </summary>
    public sealed override bool IsUPeriodic()
    {
        if (!basisSurf.IsUPeriodic())
        {
            return false;
        }
        if (!isutrimmed)
        {
            return true;
        }
        double aPeriod = basisSurf.UPeriod();
        double aLength = utrim2 - utrim1;
        return aLength > Precision.PConfusion()
               && Math.Abs(Math.IEEERemainder(aLength, aPeriod)) <= Precision.PConfusion();
    }

    /// <summary>The U period of the basis surface.</summary>
    /// <exception cref="Standard_NoSuchObject">The basis surface is not U-periodic.</exception>
    public sealed override double UPeriod() => basisSurf.UPeriod();

    /// <summary>As <see cref="IsUPeriodic"/>, in V.</summary>
    public sealed override bool IsVPeriodic()
    {
        if (!basisSurf.IsVPeriodic())
        {
            return false;
        }
        if (!isvtrimmed)
        {
            return true;
        }
        double aPeriod = basisSurf.VPeriod();
        double aLength = vtrim2 - vtrim1;
        return aLength > Precision.PConfusion()
               && Math.Abs(Math.IEEERemainder(aLength, aPeriod)) <= Precision.PConfusion();
    }

    /// <summary>The V period of the basis surface.</summary>
    /// <exception cref="Standard_NoSuchObject">The basis surface is not V-periodic.</exception>
    public sealed override double VPeriod() => basisSurf.VPeriod();

    /// <summary>The U iso-curve of the basis surface, trimmed in V if V is trimmed.</summary>
    public sealed override Geom_Curve UIso(double U)
    {
        Geom_Curve C = basisSurf.UIso(U);

        if (isvtrimmed)
        {
            Geom_TrimmedCurve Ct;
            Ct = new Geom_TrimmedCurve(C, vtrim1, vtrim2, true);
            return Ct;
        }
        else
        {
            return C;
        }
    }

    /// <summary>The V iso-curve of the basis surface, trimmed in U if U is trimmed.</summary>
    public sealed override Geom_Curve VIso(double V)
    {
        Geom_Curve C = basisSurf.VIso(V);

        if (isutrimmed)
        {
            Geom_TrimmedCurve Ct;
            Ct = new Geom_TrimmedCurve(C, utrim1, utrim2, true);
            return Ct;
        }
        else
        {
            return C;
        }
    }

    public sealed override gp_Pnt EvalD0(double U, double V) => basisSurf.EvalD0(U, V);

    public sealed override ResD1 EvalD1(double U, double V) => basisSurf.EvalD1(U, V);

    public sealed override ResD2 EvalD2(double U, double V) => basisSurf.EvalD2(U, V);

    public sealed override ResD3 EvalD3(double U, double V) => basisSurf.EvalD3(U, V);

    /// <exception cref="Geom_UndefinedDerivative">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    public sealed override gp_Vec EvalDN(double U, double V, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            throw new Geom_UndefinedDerivative();
        }
        return basisSurf.EvalDN(U, V, Nu, Nv);
    }

    /// <summary>Transforms the basis surface and the trim values (through its TransformParameters).</summary>
    protected internal sealed override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        ownBasisSurface();
        basisSurf.Transform(T);
        basisSurf.TransformParameters(ref utrim1, ref vtrim1, T);
        basisSurf.TransformParameters(ref utrim2, ref vtrim2, T);
    }

    /// <summary>The basis surface's TransformParameters.</summary>
    public sealed override void TransformParameters(ref double U, ref double V, in gp_Trsf T)
    {
        basisSurf.TransformParameters(ref U, ref V, T);
    }

    /// <summary>The basis surface's ParametricTransformation.</summary>
    public sealed override gp_GTrsf2d ParametricTransformation(in gp_Trsf T) => basisSurf.ParametricTransformation(T);

    /// <summary>A new, unfrozen trimmed surface on a copy of the basis surface.</summary>
    public sealed override Geom_Geometry Copy()
    {
        Geom_RectangularTrimmedSurface S;

        if (isutrimmed && isvtrimmed)
        {
            S = new Geom_RectangularTrimmedSurface(basisSurf, utrim1, utrim2, vtrim1, vtrim2, true, true);
        }
        else if (isutrimmed)
        {
            S = new Geom_RectangularTrimmedSurface(basisSurf, utrim1, utrim2, true, true);
        }
        else if (isvtrimmed)
        {
            S = new Geom_RectangularTrimmedSurface(basisSurf, vtrim1, vtrim2, false, true);
        }
        else
        {
            // OCCT returns a null handle here; no constructor leaves both directions untrimmed (the one-direction
            // SetTrim can only add a trimmed direction), so this is unreachable.
            throw new Standard_ProgramError("Geom_RectangularTrimmedSurface::Copy: not trimmed");
        }

        return S;
    }
}
