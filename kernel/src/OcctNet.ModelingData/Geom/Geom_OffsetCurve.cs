// Port of: src/ModelingData/TKG3d/Geom/Geom_OffsetCurve.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_OffsetCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// A curve offset from a basis curve: P(u) = p(u) + Offset * (p'(u) ^ V) / ||p'(u) ^ V||, where V is the
/// reference direction (normal to the plane of a planar basis curve). The basis curve must be at least G1
/// (C1 is checked unless isNotCheckC0); it is copied, and nested offsets and trimmed curves are flattened.
/// Evaluation throws where the normal is undefined (p' parallel to V or null).
/// </summary>
/// <remarks>
/// The evaluation representation (<see cref="HasEvalRepresentation"/>) is set by <see cref="WithEvalRepresentation"/>,
/// which stores a copy of the descriptor. <see cref="BasisCurve"/> returns the stored basis curve frozen, as OCCT
/// returns its handle; this curve's internal mutators then copy the basis curve before changing it (decision 127).
/// </remarks>
public class Geom_OffsetCurve : Geom_Curve
{
    private static readonly double MyAngularToleranceForG1 = global::Kerf.Engine.Foundation.Precision.Angular();

    private Geom_Curve basisCurve = null!;
    private GeomEval_RepCurveDesc.Base? myEvalRep;
    private Direction3D direction;
    private double offsetValue;
    private Continuity myBasisCurveContinuity;

    /// <summary>True if an evaluation representation is attached.</summary>
    public bool HasEvalRepresentation() => myEvalRep is not null;

    /// <summary>The evaluation representation descriptor (may be null). Stored, not copied: don't modify.</summary>
    internal GeomEval_RepCurveDesc.Base? EvalRepresentation() => myEvalRep;

    /// <summary>Sets a new evaluation representation, after checking it (no circular reference).</summary>
    internal void SetEvalRepresentation(GeomEval_RepCurveDesc.Base? theDesc)
    {
        CheckMutable();
        GeomEval_RepUtils.ValidateCurveDesc(theDesc, this);
        myEvalRep = theDesc;
    }

    /// <summary>Removes the evaluation representation.</summary>
    internal void ClearEvalRepresentation()
    {
        CheckMutable();
        myEvalRep = null;
    }

    /// <summary>A new, unfrozen curve equal to this one (on a copy of the basis curve).</summary>
    public sealed override Geom_Geometry Copy() => new Geom_OffsetCurve(this);

    /// <summary>A copy of theOther (deep: the basis curve is copied), without validation.</summary>
    public Geom_OffsetCurve(Geom_OffsetCurve theOther)
    {
        basisCurve = (Geom_Curve)theOther.basisCurve.Copy();
        myEvalRep = GeomEval_RepUtils.CloneCurveDesc(theOther.myEvalRep);
        direction = theOther.direction;
        offsetValue = theOther.offsetValue;
        myBasisCurveContinuity = theOther.myBasisCurveContinuity;
        // Deep copy without validation - source curve is already validated
    }

    /// <summary>
    /// The curve at distance theOffset from theCurve, in the direction theCurve' ^ theDir.
    /// </summary>
    /// <exception cref="ConstructionErrorException">
    /// The basis curve is a C0 B-spline that is not G1 (unless isTheNotCheckC0).
    /// </exception>
    public Geom_OffsetCurve(Geom_Curve theCurve, double theOffset, in Direction3D theDir, bool isTheNotCheckC0 = false)
    {
        direction = theDir;
        offsetValue = theOffset;
        SetBasisCurve(theCurve, isTheNotCheckC0);
    }

    // OCCT.NET: the basis curve may have been handed out (and frozen) by BasisCurve(); copy it before changing it.
    private void ownBasisCurve()
    {
        if (basisCurve.IsFrozen)
        {
            basisCurve = (Geom_Curve)basisCurve.Copy();
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

    /// <summary>The reference direction.</summary>
    public Direction3D Direction() => direction;

    internal void SetDirection(in Direction3D V)
    {
        CheckMutable();
        ClearEvalRepresentation();
        direction = V;
    }

    internal void SetOffsetValue(double D)
    {
        CheckMutable();
        ClearEvalRepresentation();
        offsetValue = D;
    }

    public sealed override bool IsPeriodic() => basisCurve.IsPeriodic();

    /// <summary>The period of the basis curve.</summary>
    public sealed override double Period() => basisCurve.Period();

    /// <summary>
    /// Changes the basis curve (a copy of C, with nested offsets folded into this one's offset and direction).
    /// </summary>
    /// <exception cref="ConstructionErrorException">C is a C0 B-spline that is not G1 (unless isNotCheckC0).</exception>
    internal void SetBasisCurve(Geom_Curve C, bool isNotCheckC0 = false)
    {
        CheckMutable();
        ClearEvalRepresentation();
        double aUf = C.FirstParameter(), aUl = C.LastParameter();
        Geom_Curve aCheckingCurve = (Geom_Curve)C.Copy();
        bool isTrimmed = false;

        while (aCheckingCurve is Geom_TrimmedCurve || aCheckingCurve is Geom_OffsetCurve)
        {
            if (aCheckingCurve is Geom_TrimmedCurve aTrimC)
            {
                aCheckingCurve = aTrimC.BasisCurve();
                isTrimmed = true;
            }

            if (aCheckingCurve is Geom_OffsetCurve aOC)
            {
                aCheckingCurve = aOC.BasisCurve();
                double PrevOff = aOC.Offset();
                Vector3D V1 = new Vector3D(aOC.Direction());
                Vector3D V2 = new Vector3D(direction);
                Vector3D Vdir = PrevOff * V1 + offsetValue * V2;

                if (offsetValue >= 0.0)
                {
                    offsetValue = Vdir.Magnitude();
                    direction.SetXYZ(Vdir.XYZ());
                }
                else
                {
                    offsetValue = -Vdir.Magnitude();
                    direction.SetXYZ((-Vdir).XYZ());
                }
            }
        }

        myBasisCurveContinuity = aCheckingCurve.Continuity();

        bool isC0 = !isNotCheckC0 && (myBasisCurveContinuity == C0);

        // Basis curve must be at least C1
        if (isC0 && aCheckingCurve is Geom_BSplineCurve aBC)
        {
            if (aBC.IsG1(aUf, aUl, MyAngularToleranceForG1))
            {
                // Checking if basis curve has more smooth (C1, G2 and above) is not done.
                // It can be done in case of need.
                myBasisCurveContinuity = G1;
                isC0 = false;
            }

            // Raise exception if still C0
            if (isC0)
            {
                throw new ConstructionErrorException("Offset on C0 curve");
            }
        }
        //
        if (isTrimmed)
        {
            basisCurve = new Geom_TrimmedCurve(aCheckingCurve, aUf, aUl);
        }
        else
        {
            basisCurve = aCheckingCurve;
        }
    }

    /// <summary>The basis curve, frozen (it is shared with this offset curve; decision 127).</summary>
    public Geom_Curve BasisCurve()
    {
        basisCurve.Freeze();
        return basisCurve;
    }

    /// <summary>The continuity of the basis curve lowered by one (C1 gives C0, G1 and G2 are kept).</summary>
    public sealed override Continuity Continuity()
    {
        Continuity OffsetShape = C0;
        switch (myBasisCurveContinuity)
        {
            case C0:
                OffsetShape = C0;
                break;
            case C1:
                OffsetShape = C0;
                break;
            case C2:
                OffsetShape = C1;
                break;
            case C3:
                OffsetShape = C2;
                break;
            case CN:
                OffsetShape = CN;
                break;
            case G1:
                OffsetShape = G1;
                break;
            case G2:
                OffsetShape = G2;
                break;
        }
        return OffsetShape;
    }

    /// <exception cref="Geom_UndefinedValue">The normal is undefined at theU.</exception>
    public sealed override Point3D EvalD0(double theU)
    {
        if (GeomEval_RepUtils.TryEvalCurveD0(myEvalRep, theU, out Point3D aEvalRepResult))
        {
            return aEvalRepResult;
        }
        ResD1 aBasisD1 = basisCurve.EvalD1(theU);
        Point3D aValue = aBasisD1.Point;
        if (!Geom_OffsetCurveUtils.CalculateD0(ref aValue, aBasisD1.D1, direction.XYZ(), offsetValue))
        {
            throw new Geom_UndefinedValue("Geom_OffsetCurve::EvalD0");
        }
        return aValue;
    }

    /// <exception cref="Geom_UndefinedDerivative">The derivative is undefined at theU.</exception>
    public sealed override ResD1 EvalD1(double theU)
    {
        if (GeomEval_RepUtils.TryEvalCurveD1(myEvalRep, theU, out ResD1 aEvalRepResult))
        {
            return aEvalRepResult;
        }
        ResD2 aBasisD2 = basisCurve.EvalD2(theU);
        Point3D aValue = aBasisD2.Point;
        Vector3D aD1 = aBasisD2.D1;
        if (!Geom_OffsetCurveUtils.CalculateD1(ref aValue, ref aD1, aBasisD2.D2, direction.XYZ(), offsetValue))
        {
            throw new Geom_UndefinedDerivative("Geom_OffsetCurve::EvalD1");
        }
        return new ResD1 { Point = aValue, D1 = aD1 };
    }

    /// <exception cref="Geom_UndefinedDerivative">The derivatives are undefined at theU.</exception>
    public sealed override ResD2 EvalD2(double theU)
    {
        if (GeomEval_RepUtils.TryEvalCurveD2(myEvalRep, theU, out ResD2 aEvalRepResult))
        {
            return aEvalRepResult;
        }
        ResD3 aBasisD3 = basisCurve.EvalD3(theU);
        Point3D aValue = aBasisD3.Point;
        Vector3D aD1 = aBasisD3.D1, aD2 = aBasisD3.D2, aD3 = aBasisD3.D3;

        bool isDirectionChange = false;
        if (aD1.SquareMagnitude() <= Constants.Resolution())
        {
            Vector3D aDummyD4 = new Vector3D();
            if (!Geom_OffsetCurveUtils.AdjustDerivative(basisCurve,
                                                        3,
                                                        theU,
                                                        ref aD1,
                                                        ref aD2,
                                                        ref aD3,
                                                        ref aDummyD4,
                                                        out isDirectionChange))
            {
                throw new Geom_UndefinedDerivative("Geom_OffsetCurve::EvalD2");
            }
        }

        if (!Geom_OffsetCurveUtils.CalculateD2(ref aValue,
                                               ref aD1,
                                               ref aD2,
                                               aD3,
                                               direction.XYZ(),
                                               offsetValue,
                                               isDirectionChange))
        {
            throw new Geom_UndefinedDerivative("Geom_OffsetCurve::EvalD2");
        }
        return new ResD2 { Point = aValue, D1 = aD1, D2 = aD2 };
    }

    /// <exception cref="Geom_UndefinedDerivative">The derivatives are undefined at theU.</exception>
    public sealed override ResD3 EvalD3(double theU)
    {
        if (GeomEval_RepUtils.TryEvalCurveD3(myEvalRep, theU, out ResD3 aEvalRepResult))
        {
            return aEvalRepResult;
        }
        ResD3 aBasisD3 = basisCurve.EvalD3(theU);
        Vector3D aD4Basis = basisCurve.EvalDN(theU, 4);
        Point3D aValue = aBasisD3.Point;
        Vector3D aD1 = aBasisD3.D1, aD2 = aBasisD3.D2, aD3 = aBasisD3.D3;
        Vector3D aD4 = aD4Basis;

        bool isDirectionChange = false;
        if (aD1.SquareMagnitude() <= Constants.Resolution())
        {
            if (!Geom_OffsetCurveUtils.AdjustDerivative(basisCurve,
                                                        4,
                                                        theU,
                                                        ref aD1,
                                                        ref aD2,
                                                        ref aD3,
                                                        ref aD4,
                                                        out isDirectionChange))
            {
                throw new Geom_UndefinedDerivative("Geom_OffsetCurve::EvalD3");
            }
        }

        if (!Geom_OffsetCurveUtils.CalculateD3(ref aValue,
                                               ref aD1,
                                               ref aD2,
                                               ref aD3,
                                               aD4,
                                               direction.XYZ(),
                                               offsetValue,
                                               isDirectionChange))
        {
            throw new Geom_UndefinedDerivative("Geom_OffsetCurve::EvalD3");
        }
        return new ResD3 { Point = aValue, D1 = aD1, D2 = aD2, D3 = aD3 };
    }

    /// <summary>The derivative of order N; above 3, the basis curve's (as in OCCT).</summary>
    /// <exception cref="Geom_UndefinedDerivative">N &lt; 1, or the derivative is undefined at U.</exception>
    public sealed override Vector3D EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom_UndefinedDerivative("Geom_OffsetCurve::EvalDN");
        }

        if (GeomEval_RepUtils.TryEvalCurveDN(myEvalRep, U, N, out Vector3D aEvalRepResult))
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

    public sealed override double FirstParameter() => basisCurve.FirstParameter();

    public sealed override double LastParameter() => basisCurve.LastParameter();

    /// <summary>The offset value.</summary>
    public double Offset() => offsetValue;

    /// <summary>True if the end points are within Precision::Computational() (squared distance).</summary>
    public sealed override bool IsClosed()
    {
        Point3D PF, PL;
        D0(FirstParameter(), out PF);
        D0(LastParameter(), out PL);
        return PF.SquareDistance(PL) <= global::Kerf.Engine.Foundation.Precision.Computational();
    }

    /// <summary>True if the basis curve is C(N + 1).</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    public sealed override bool IsCN(int N)
    {
        Standard_RangeError_Raise_if(N < 0, " ");
        return basisCurve.IsCN(N + 1);
    }

    /// <summary>Applies T to the basis curve and the direction; the offset is scaled by T.ScaleFactor().</summary>
    protected internal sealed override void Transform(in Transform3D T)
    {
        CheckMutable();
        ClearEvalRepresentation();
        ownBasisCurve();
        basisCurve.Transform(T);
        direction.Transform(T);
        offsetValue *= T.ScaleFactor();
    }

    /// <summary>The basis curve's transformed parameter.</summary>
    public sealed override double TransformedParameter(double U, in Transform3D T) => basisCurve.TransformedParameter(U, T);

    /// <summary>The basis curve's parametric transformation.</summary>
    public sealed override double ParametricTransformation(in Transform3D T) => basisCurve.ParametricTransformation(T);

    /// <summary>The continuity of the basis curve (G1 for a G1 C0 B-spline).</summary>
    public Continuity GetBasisCurveContinuity() => myBasisCurveContinuity;

    // Non-mutating twins (OCCT.NET)

    public Geom_OffsetCurve WithDirection(in Direction3D V)
    {
        Geom_OffsetCurve aCopy = (Geom_OffsetCurve)Copy();
        aCopy.SetDirection(V);
        return aCopy;
    }

    public Geom_OffsetCurve WithOffsetValue(double D)
    {
        Geom_OffsetCurve aCopy = (Geom_OffsetCurve)Copy();
        aCopy.SetOffsetValue(D);
        return aCopy;
    }

    /// <summary>A copy on the basis curve C (see <see cref="SetBasisCurve"/>).</summary>
    public Geom_OffsetCurve WithBasisCurve(Geom_Curve C, bool isNotCheckC0 = false)
    {
        Geom_OffsetCurve aCopy = (Geom_OffsetCurve)Copy();
        aCopy.SetBasisCurve(C, isNotCheckC0);
        return aCopy;
    }

    /// <summary>A copy evaluated through a copy of theDesc (null: without a representation).</summary>
    public Geom_OffsetCurve WithEvalRepresentation(GeomEval_RepCurveDesc.Base? theDesc)
    {
        GeomEval_RepUtils.ValidateCurveDesc(theDesc, this);
        Geom_OffsetCurve aCopy = (Geom_OffsetCurve)Copy();
        aCopy.SetEvalRepresentation(GeomEval_RepUtils.CloneCurveDesc(theDesc));
        return aCopy;
    }

    /// <summary>A copy without an evaluation representation (the twin of ClearEvalRepresentation).</summary>
    public Geom_OffsetCurve WithoutEvalRepresentation()
    {
        Geom_OffsetCurve aCopy = (Geom_OffsetCurve)Copy();
        aCopy.ClearEvalRepresentation();
        return aCopy;
    }
}
