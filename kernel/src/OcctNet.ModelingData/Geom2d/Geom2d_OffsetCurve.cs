// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_OffsetCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_OffsetCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// The offset of a basis curve in the plane: P(u) = p(u) + Offset * N(u), where N is the unit normal obtained by
/// rotating the tangent by -90 degrees (to the right of the curve for a positive offset). The basis curve must be
/// at least C1 (or G1 for a B-spline); the offset curve has one degree of continuity less.
/// </summary>
/// <remarks>
/// As in OCCT the offset curve shares its basis curve with the caller (a trimmed or offset basis is replaced by
/// its own basis, and a trimmed one rebuilt). The shared basis curve is frozen; this curve's internal mutators
/// copy it before changing it (decision 132).
/// </remarks>
public class Geom2d_OffsetCurve : Geom2d_Curve
{
    private static readonly double MyAngularToleranceForG1 = Precision.Angular();

    private Geom2d_Curve basisCurve = null!;
    private Geom2dEval_RepCurveDesc.Base? myEvalRep;
    private double offsetValue;
    private GeomAbs_Shape myBasisCurveContinuity;

    /// <summary>True if an evaluation representation is attached.</summary>
    public bool HasEvalRepresentation() => myEvalRep is not null;

    /// <summary>The attached evaluation representation, or null. Internal: descriptors are mutable (decision 507).</summary>
    internal Geom2dEval_RepCurveDesc.Base? EvalRepresentation() => myEvalRep;

    /// <summary>Attaches an evaluation representation (null detaches it).</summary>
    /// <exception cref="Standard_ProgramError">The descriptor is invalid.</exception>
    internal void SetEvalRepresentation(Geom2dEval_RepCurveDesc.Base? theDesc)
    {
        CheckMutable();
        Geom2dEval_RepUtils.ValidateCurveDesc(theDesc, this);
        myEvalRep = theDesc;
    }

    /// <summary>Detaches the evaluation representation.</summary>
    internal void ClearEvalRepresentation()
    {
        CheckMutable();
        myEvalRep = null;
    }

    /// <summary>A new, unfrozen offset curve equal to this one, with its own copy of the basis curve.</summary>
    public sealed override Geom2d_Geometry Copy() => new Geom2d_OffsetCurve(this);

    /// <summary>
    /// The offset of C by theOffset. A trimmed or offset C is replaced by its basis (the offsets add up). Unless
    /// isTheNotCheckC0, a C0 B-spline basis must be G1.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">The basis is a C0 B-spline that is not G1.</exception>
    public Geom2d_OffsetCurve(Geom2d_Curve theCurve, double theOffset, bool isTheNotCheckC0 = false)
    {
        offsetValue = theOffset;
        SetBasisCurve(theCurve, isTheNotCheckC0);
    }

    /// <summary>A copy of theOther with its own copy of the basis curve (and of the evaluation representation).</summary>
    public Geom2d_OffsetCurve(Geom2d_OffsetCurve theOther)
    {
        basisCurve = (Geom2d_Curve)theOther.basisCurve.Copy();
        myEvalRep = Geom2dEval_RepUtils.CloneCurveDesc(theOther.myEvalRep);
        offsetValue = theOther.offsetValue;
        myBasisCurveContinuity = theOther.myBasisCurveContinuity;
        // Deep copy basis curve without validation
    }

    // OCCT.NET: the basis curve is shared (frozen); copy it before changing it.
    private void ownBasisCurve()
    {
        if (basisCurve.IsFrozen())
        {
            basisCurve = (Geom2d_Curve)basisCurve.Copy();
        }
    }

    /// <summary>Reverses the basis curve and the sign of the offset.</summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        ownBasisCurve();
        basisCurve.Reverse();
        offsetValue = -offsetValue;
    }

    /// <summary>The basis curve's reversed parameter.</summary>
    public sealed override double ReversedParameter(double U) => basisCurve.ReversedParameter(U);

    /// <summary>Changes the basis curve (see the constructor).</summary>
    /// <exception cref="Standard_ConstructionError">The basis is a C0 B-spline that is not G1.</exception>
    internal void SetBasisCurve(Geom2d_Curve C, bool isNotCheckC0 = false)
    {
        CheckMutable();
        ClearEvalRepresentation();
        double aUf = C.FirstParameter(), aUl = C.LastParameter();
        Geom2d_Curve aCheckingCurve = C;
        bool isTrimmed = false;

        while (aCheckingCurve is Geom2d_TrimmedCurve || aCheckingCurve is Geom2d_OffsetCurve)
        {
            if (aCheckingCurve is Geom2d_TrimmedCurve aTrimC)
            {
                aCheckingCurve = aTrimC.BasisCurve();
                isTrimmed = true;
            }

            if (aCheckingCurve is Geom2d_OffsetCurve aOC)
            {
                aCheckingCurve = aOC.BasisCurve();
                offsetValue += aOC.Offset();
            }
        }

        myBasisCurveContinuity = aCheckingCurve.Continuity();

        bool isC0 = !isNotCheckC0 && (myBasisCurveContinuity == GeomAbs_C0);

        // Basis curve must be at least C1
        if (isC0 && aCheckingCurve is Geom2d_BSplineCurve aBC)
        {
            if (aBC.IsG1(aUf, aUl, MyAngularToleranceForG1))
            {
                // Checking if basis curve has more smooth (C1, G2 and above) is not done.
                // It can be done in case of need.
                myBasisCurveContinuity = GeomAbs_G1;
                isC0 = false;
            }

            // Raise exception if still C0
            if (isC0)
            {
                throw new Standard_ConstructionError("Offset on C0 curve");
            }
        }
        //
        if (isTrimmed)
        {
            basisCurve = new Geom2d_TrimmedCurve(aCheckingCurve, aUf, aUl);
        }
        else
        {
            // OCCT.NET: shared with the caller, so frozen (decision 132).
            aCheckingCurve.Freeze();
            basisCurve = aCheckingCurve;
        }
    }

    /// <summary>Changes the offset value.</summary>
    internal void SetOffsetValue(double D)
    {
        CheckMutable();
        ClearEvalRepresentation();
        offsetValue = D;
    }

    /// <summary>The basis curve, frozen (it may be shared).</summary>
    public Geom2d_Curve BasisCurve()
    {
        basisCurve.Freeze();
        return basisCurve;
    }

    /// <summary>The continuity of the basis curve lowered by one (C0 for a C1 basis, G1 for a G1 B-spline basis).</summary>
    public sealed override GeomAbs_Shape Continuity()
    {
        GeomAbs_Shape OffsetShape = GeomAbs_C0;
        switch (myBasisCurveContinuity)
        {
            case GeomAbs_C0:
                OffsetShape = GeomAbs_C0;
                break;
            case GeomAbs_C1:
                OffsetShape = GeomAbs_C0;
                break;
            case GeomAbs_C2:
                OffsetShape = GeomAbs_C1;
                break;
            case GeomAbs_C3:
                OffsetShape = GeomAbs_C2;
                break;
            case GeomAbs_CN:
                OffsetShape = GeomAbs_CN;
                break;
            case GeomAbs_G1:
                OffsetShape = GeomAbs_G1;
                break;
            case GeomAbs_G2:
                OffsetShape = GeomAbs_G2;
                break;
        }
        return OffsetShape;
    }

    /// <exception cref="Geom2d_UndefinedValue">The basis curve's first derivative is null.</exception>
    public sealed override gp_Pnt2d EvalD0(double theU)
    {
        gp_Pnt2d aEvalRepResult;
        if (Geom2dEval_RepUtils.TryEvalCurveD0(myEvalRep, theU, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        Geom2d_Curve.ResD1 aBasisD1 = basisCurve.EvalD1(theU);
        gp_Pnt2d aValue = aBasisD1.Point;
        if (!Geom2d_OffsetCurveUtils.CalculateD0(ref aValue, aBasisD1.D1, offsetValue))
        {
            throw new Geom2d_UndefinedValue("Geom2d_OffsetCurve::EvalD0");
        }
        return aValue;
    }

    /// <exception cref="Geom2d_UndefinedDerivative">The derivative can't be computed.</exception>
    public sealed override ResD1 EvalD1(double theU)
    {
        ResD1 aEvalRepResult;
        if (Geom2dEval_RepUtils.TryEvalCurveD1(myEvalRep, theU, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        Geom2d_Curve.ResD2 aBasisD2 = basisCurve.EvalD2(theU);
        gp_Pnt2d aValue = aBasisD2.Point;
        gp_Vec2d aD1 = aBasisD2.D1;
        if (!Geom2d_OffsetCurveUtils.CalculateD1(ref aValue, ref aD1, aBasisD2.D2, offsetValue))
        {
            throw new Geom2d_UndefinedDerivative("Geom2d_OffsetCurve::EvalD1");
        }
        return new ResD1 { Point = aValue, D1 = aD1 };
    }

    /// <exception cref="Geom2d_UndefinedDerivative">The derivatives can't be computed.</exception>
    public sealed override ResD2 EvalD2(double theU)
    {
        ResD2 aEvalRepResult;
        if (Geom2dEval_RepUtils.TryEvalCurveD2(myEvalRep, theU, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        Geom2d_Curve.ResD3 aBasisD3 = basisCurve.EvalD3(theU);
        gp_Pnt2d aValue = aBasisD3.Point;
        gp_Vec2d aD1 = aBasisD3.D1, aD2 = aBasisD3.D2, aD3 = aBasisD3.D3;
        bool isDirectionChange = false;
        if (aD1.SquareMagnitude() <= gp.Resolution())
        {
            gp_Vec2d aDummyD4 = new gp_Vec2d();
            if (!Geom2d_OffsetCurveUtils.AdjustDerivative(basisCurve,
                                                          3,
                                                          theU,
                                                          ref aD1,
                                                          ref aD2,
                                                          ref aD3,
                                                          ref aDummyD4,
                                                          out isDirectionChange))
            {
                throw new Geom2d_UndefinedDerivative("Geom2d_OffsetCurve::EvalD2");
            }
        }
        if (!Geom2d_OffsetCurveUtils.CalculateD2(ref aValue, ref aD1, ref aD2, aD3, isDirectionChange, offsetValue))
        {
            throw new Geom2d_UndefinedDerivative("Geom2d_OffsetCurve::EvalD2");
        }
        return new ResD2 { Point = aValue, D1 = aD1, D2 = aD2 };
    }

    /// <exception cref="Geom2d_UndefinedDerivative">The derivatives can't be computed.</exception>
    /// <remarks>OCCT's D2 and D3 here are wrong where the basis curve's speed varies (occt-issues.md).</remarks>
    public sealed override ResD3 EvalD3(double theU)
    {
        ResD3 aEvalRepResult;
        if (Geom2dEval_RepUtils.TryEvalCurveD3(myEvalRep, theU, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        Geom2d_Curve.ResD3 aBasisD3 = basisCurve.EvalD3(theU);
        gp_Vec2d aD4Basis = basisCurve.EvalDN(theU, 4);
        gp_Pnt2d aValue = aBasisD3.Point;
        gp_Vec2d aD1 = aBasisD3.D1, aD2 = aBasisD3.D2, aD3 = aBasisD3.D3;
        gp_Vec2d aD4 = aD4Basis;
        bool isDirectionChange = false;
        if (aD1.SquareMagnitude() <= gp.Resolution())
        {
            if (!Geom2d_OffsetCurveUtils.AdjustDerivative(basisCurve,
                                                          4,
                                                          theU,
                                                          ref aD1,
                                                          ref aD2,
                                                          ref aD3,
                                                          ref aD4,
                                                          out isDirectionChange))
            {
                throw new Geom2d_UndefinedDerivative("Geom2d_OffsetCurve::EvalD3");
            }
        }
        if (!Geom2d_OffsetCurveUtils.CalculateD3(ref aValue,
                                                 ref aD1,
                                                 ref aD2,
                                                 ref aD3,
                                                 aD4,
                                                 isDirectionChange,
                                                 offsetValue))
        {
            throw new Geom2d_UndefinedDerivative("Geom2d_OffsetCurve::EvalD3");
        }
        return new ResD3 { Point = aValue, D1 = aD1, D2 = aD2, D3 = aD3 };
    }

    /// <summary>The Nth derivative: through EvalD1 to EvalD3 up to N = 3, the basis curve's derivative above.</summary>
    /// <exception cref="Geom2d_UndefinedDerivative">N &lt; 1, or the derivative can't be computed.</exception>
    public sealed override gp_Vec2d EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom2d_UndefinedDerivative("Geom2d_OffsetCurve::EvalDN");
        }

        gp_Vec2d aEvalRepResult;
        if (Geom2dEval_RepUtils.TryEvalCurveDN(myEvalRep, U, N, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        switch (N)
        {
            case 1:
            {
                return EvalD1(U).D1;
            }
            case 2:
            {
                return EvalD2(U).D2;
            }
            case 3:
            {
                return EvalD3(U).D3;
            }
            default:
                return basisCurve.EvalDN(U, N);
        }
    }

    /// <summary>The basis curve's first parameter.</summary>
    public sealed override double FirstParameter() => basisCurve.FirstParameter();

    /// <summary>The basis curve's last parameter.</summary>
    public sealed override double LastParameter() => basisCurve.LastParameter();

    /// <summary>The offset value.</summary>
    public double Offset() => offsetValue;

    /// <summary>True if the points at the first and last parameters are within Precision::Computational() (squared distance).</summary>
    public sealed override bool IsClosed()
    {
        gp_Pnt2d PF, PL;
        D0(FirstParameter(), out PF);
        D0(LastParameter(), out PL);
        return PF.SquareDistance(PL) <= Precision.Computational();
    }

    /// <summary>True if the basis curve is C(N + 1).</summary>
    /// <exception cref="Standard_RangeError">N &lt; 0.</exception>
    public sealed override bool IsCN(int N)
    {
        Standard_RangeError_Raise_if(N < 0, " ");
        return basisCurve.IsCN(N + 1);
    }

    /// <summary>True if the basis curve is periodic.</summary>
    public sealed override bool IsPeriodic() => basisCurve.IsPeriodic();

    /// <summary>The basis curve's period.</summary>
    public sealed override double Period() => basisCurve.Period();

    /// <summary>Transforms the basis curve and scales the offset by |T.ScaleFactor()|.</summary>
    protected internal sealed override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        ClearEvalRepresentation();
        ownBasisCurve();
        basisCurve.Transform(T);
        offsetValue *= Math.Abs(T.ScaleFactor());
    }

    /// <summary>The basis curve's transformed parameter.</summary>
    public sealed override double TransformedParameter(double U, in gp_Trsf2d T) =>
        basisCurve.TransformedParameter(U, T);

    /// <summary>The basis curve's parametric transformation.</summary>
    public sealed override double ParametricTransformation(in gp_Trsf2d T) =>
        basisCurve.ParametricTransformation(T);

    /// <summary>The continuity of the basis curve (G1 for a C0 B-spline accepted as G1).</summary>
    public GeomAbs_Shape GetBasisCurveContinuity() => myBasisCurveContinuity;

    // Non-mutating twins of the mutators (OCCT.NET)

    /// <summary>A copy on the basis curve C (see the constructor; the offsets of an offset C add up).</summary>
    public Geom2d_OffsetCurve WithBasisCurve(Geom2d_Curve C, bool isNotCheckC0 = false)
    {
        Geom2d_OffsetCurve aCopy = new Geom2d_OffsetCurve(this);
        aCopy.SetBasisCurve(C, isNotCheckC0);
        return aCopy;
    }

    /// <summary>A copy with the offset D.</summary>
    public Geom2d_OffsetCurve WithOffsetValue(double D)
    {
        Geom2d_OffsetCurve aCopy = new Geom2d_OffsetCurve(this);
        aCopy.SetOffsetValue(D);
        return aCopy;
    }

    /// <summary>A copy evaluated through theDesc, which is copied (so later changes to theDesc don't affect it).</summary>
    /// <exception cref="Standard_ProgramError">The descriptor is invalid.</exception>
    public Geom2d_OffsetCurve WithEvalRepresentation(Geom2dEval_RepCurveDesc.Base? theDesc)
    {
        Geom2d_OffsetCurve aCopy = new Geom2d_OffsetCurve(this);
        Geom2dEval_RepUtils.ValidateCurveDesc(theDesc, this);
        aCopy.SetEvalRepresentation(Geom2dEval_RepUtils.CloneCurveDesc(theDesc));
        return aCopy;
    }

    /// <summary>A copy without an evaluation representation.</summary>
    public Geom2d_OffsetCurve WithoutEvalRepresentation()
    {
        Geom2d_OffsetCurve aCopy = new Geom2d_OffsetCurve(this);
        aCopy.ClearEvalRepresentation();
        return aCopy;
    }
}
