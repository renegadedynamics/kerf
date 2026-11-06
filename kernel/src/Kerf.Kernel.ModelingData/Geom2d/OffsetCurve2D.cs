// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_OffsetCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_OffsetCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Evaluation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;

namespace Kerf.Geometry;

/// <summary>
/// The offset of a basis curve in the plane: P(u) = p(u) + Offset * N(u), where N is the unit normal obtained by
/// rotating the tangent by -90 degrees (to the right of the curve for a positive offset). The basis curve must be
/// at least C1 (or G1 for a B-spline); the offset curve has one degree of continuity less.
/// </summary>
/// <remarks>
/// As in OCCT the offset curve shares its basis curve with the caller (a trimmed or offset basis is replaced by
/// its own basis, and a trimmed one rebuilt). The shared basis curve is frozen; this curve's internal mutators
/// copy it before changing it.
/// </remarks>
public sealed class OffsetCurve2D : Curve2D
{
    private static readonly double MyAngularToleranceForG1 = global::Kerf.Engine.Foundation.Precision.Angular();

    private Curve2D _basisCurve = null!;
    private RepCurveDesc2D.Base? _evalRep;
    private double _offsetValue;
    private Continuity _basisCurveContinuity;

    /// <summary>True if an evaluation representation is attached.</summary>
    internal bool HasEvalRepresentation => _evalRep is not null;

    /// <summary>The attached evaluation representation, or null. Internal: descriptors are mutable.</summary>
    internal RepCurveDesc2D.Base? EvalRepresentation() => _evalRep;

    /// <summary>Attaches an evaluation representation (null detaches it).</summary>
    /// <exception cref="ProgramErrorException">The descriptor is invalid.</exception>
    internal void SetEvalRepresentation(RepCurveDesc2D.Base? desc)
    {
        CheckMutable();
        RepUtils2D.ValidateCurveDesc(desc, this);
        _evalRep = desc;
    }

    /// <summary>Detaches the evaluation representation.</summary>
    internal void ClearEvalRepresentation()
    {
        CheckMutable();
        _evalRep = null;
    }

    /// <summary>A new, unfrozen offset curve equal to this one, with its own copy of the basis curve.</summary>
    public sealed override Geometry2D Copy() => new OffsetCurve2D(this);

    /// <summary>
    /// The offset of C by theOffset. A trimmed or offset C is replaced by its basis (the offsets add up). Unless
    /// isTheNotCheckC0, a C0 B-spline basis must be G1.
    /// </summary>
    /// <exception cref="ConstructionErrorException">The basis is a C0 B-spline that is not G1.</exception>
    public OffsetCurve2D(Curve2D curve, double offset, bool skipContinuityCheck = false)
    {
        _offsetValue = offset;
        SetBasisCurve(curve, skipContinuityCheck);
    }

    /// <summary>A copy of theOther with its own copy of the basis curve (and of the evaluation representation).</summary>
    public OffsetCurve2D(OffsetCurve2D other)
    {
        _basisCurve = (Curve2D)other._basisCurve.Copy();
        _evalRep = RepUtils2D.CloneCurveDesc(other._evalRep);
        _offsetValue = other._offsetValue;
        _basisCurveContinuity = other._basisCurveContinuity;
        // Deep copy basis curve without validation
    }

    // Kerf: the basis curve is shared (frozen); copy it before changing it.
    private void OwnBasisCurve()
    {
        if (_basisCurve.IsFrozen)
        {
            _basisCurve = (Curve2D)_basisCurve.Copy();
        }
    }

    /// <summary>Reverses the basis curve and the sign of the offset.</summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        OwnBasisCurve();
        _basisCurve.Reverse();
        _offsetValue = -_offsetValue;
    }

    /// <summary>The basis curve's reversed parameter.</summary>
    public sealed override double ReversedParameter(double u) => _basisCurve.ReversedParameter(u);

    /// <summary>Changes the basis curve (see the constructor).</summary>
    /// <exception cref="ConstructionErrorException">The basis is a C0 B-spline that is not G1.</exception>
    internal void SetBasisCurve(Curve2D c, bool isNotCheckC0 = false)
    {
        CheckMutable();
        ClearEvalRepresentation();
        double uf = c.FirstParameter, ul = c.LastParameter;
        Curve2D checkingCurve = c;
        bool isTrimmed = false;

        while (checkingCurve is TrimmedCurve2D || checkingCurve is OffsetCurve2D)
        {
            if (checkingCurve is TrimmedCurve2D trimC)
            {
                checkingCurve = trimC.BasisCurve;
                isTrimmed = true;
            }

            if (checkingCurve is OffsetCurve2D oc)
            {
                checkingCurve = oc.BasisCurve;
                _offsetValue += oc.Offset;
            }
        }

        _basisCurveContinuity = checkingCurve.Continuity;

        bool isC0 = !isNotCheckC0 && (_basisCurveContinuity == C0);

        // Basis curve must be at least C1
        if (isC0 && checkingCurve is BSplineCurve2D bc)
        {
            if (bc.IsG1(uf, ul, MyAngularToleranceForG1))
            {
                // Checking if basis curve has more smooth (C1, G2 and above) is not done.
                // It can be done in case of need.
                _basisCurveContinuity = G1;
                isC0 = false;
            }

            // Raise exception if still C0
            if (isC0)
            {
                throw Refusal.Argument(new ConstructionErrorException("Offset on C0 curve"));
            }
        }
        //
        if (isTrimmed)
        {
            _basisCurve = new TrimmedCurve2D(checkingCurve, uf, ul);
        }
        else
        {
            // Kerf: shared with the caller, so frozen (decision 132).
            checkingCurve.Freeze();
            _basisCurve = checkingCurve;
        }
    }

    /// <summary>Changes the offset value.</summary>
    internal void SetOffsetValue(double d)
    {
        CheckMutable();
        ClearEvalRepresentation();
        _offsetValue = d;
    }

    /// <summary>The basis curve, frozen (it may be shared).</summary>
    public Curve2D BasisCurve
    {
        get
        {
            _basisCurve.Freeze();
            return _basisCurve;
        }
    }

    /// <summary>The continuity of the basis curve lowered by one (C0 for a C1 basis, G1 for a G1 B-spline basis).</summary>
    public sealed override Continuity Continuity
    {
        get
        {
            Continuity offsetShape = C0;
            switch (_basisCurveContinuity)
            {
                case C0:
                    offsetShape = C0;
                    break;
                case C1:
                    offsetShape = C0;
                    break;
                case C2:
                    offsetShape = C1;
                    break;
                case C3:
                    offsetShape = C2;
                    break;
                case CN:
                    offsetShape = CN;
                    break;
                case G1:
                    offsetShape = G1;
                    break;
                case G2:
                    offsetShape = G2;
                    break;
            }
            return offsetShape;
        }
    }

    /// <exception cref="UndefinedValue2DException">The basis curve's first derivative is null.</exception>
    internal sealed override Point2D EvalD0(double u)
    {
        Point2D evalRepResult;
        if (RepUtils2D.TryEvalCurveD0(_evalRep, u, out evalRepResult))
        {
            return evalRepResult;
        }

        Curve2D.ResD1 basisD1 = _basisCurve.EvalD1(u);
        Point2D value = basisD1.Point;
        if (!OffsetCurveUtils2D.CalculateD0(ref value, basisD1.D1, _offsetValue))
        {
            throw Refusal.State(new UndefinedValue2DException("Geom2d_OffsetCurve::EvalD0"));
        }
        return value;
    }

    /// <exception cref="UndefinedDerivative2DException">The derivative can't be computed.</exception>
    internal sealed override ResD1 EvalD1(double u)
    {
        ResD1 evalRepResult;
        if (RepUtils2D.TryEvalCurveD1(_evalRep, u, out evalRepResult))
        {
            return evalRepResult;
        }

        Curve2D.ResD2 basisD2 = _basisCurve.EvalD2(u);
        Point2D value = basisD2.Point;
        Vector2D d1 = basisD2.D1;
        if (!OffsetCurveUtils2D.CalculateD1(ref value, ref d1, basisD2.D2, _offsetValue))
        {
            throw Refusal.State(new UndefinedDerivative2DException("Geom2d_OffsetCurve::EvalD1"));
        }
        return new ResD1 { Point = value, D1 = d1 };
    }

    /// <exception cref="UndefinedDerivative2DException">The derivatives can't be computed.</exception>
    internal sealed override ResD2 EvalD2(double u)
    {
        ResD2 evalRepResult;
        if (RepUtils2D.TryEvalCurveD2(_evalRep, u, out evalRepResult))
        {
            return evalRepResult;
        }

        Curve2D.ResD3 basisD3 = _basisCurve.EvalD3(u);
        Point2D value = basisD3.Point;
        Vector2D d1 = basisD3.D1, d2 = basisD3.D2, d3 = basisD3.D3;
        bool isDirectionChange = false;
        if (d1.SquareMagnitude <= Constants.Resolution())
        {
            Vector2D dummyD4 = new Vector2D();
            if (!OffsetCurveUtils2D.AdjustDerivative(_basisCurve,
                                                          3,
                                                          u,
                                                          ref d1,
                                                          ref d2,
                                                          ref d3,
                                                          ref dummyD4,
                                                          out isDirectionChange))
            {
                throw Refusal.State(new UndefinedDerivative2DException("Geom2d_OffsetCurve::EvalD2"));
            }
        }
        if (!OffsetCurveUtils2D.CalculateD2(ref value, ref d1, ref d2, d3, isDirectionChange, _offsetValue))
        {
            throw Refusal.State(new UndefinedDerivative2DException("Geom2d_OffsetCurve::EvalD2"));
        }
        return new ResD2 { Point = value, D1 = d1, D2 = d2 };
    }

    /// <exception cref="UndefinedDerivative2DException">The derivatives can't be computed.</exception>
    /// <remarks>OCCT's D2 and D3 here are wrong where the basis curve's speed varies (occt-issues.md).</remarks>
    internal sealed override ResD3 EvalD3(double u)
    {
        ResD3 evalRepResult;
        if (RepUtils2D.TryEvalCurveD3(_evalRep, u, out evalRepResult))
        {
            return evalRepResult;
        }

        Curve2D.ResD3 basisD3 = _basisCurve.EvalD3(u);
        Vector2D d4Basis = _basisCurve.EvalDN(u, 4);
        Point2D value = basisD3.Point;
        Vector2D d1 = basisD3.D1, d2 = basisD3.D2, d3 = basisD3.D3;
        Vector2D d4 = d4Basis;
        bool isDirectionChange = false;
        if (d1.SquareMagnitude <= Constants.Resolution())
        {
            if (!OffsetCurveUtils2D.AdjustDerivative(_basisCurve,
                                                          4,
                                                          u,
                                                          ref d1,
                                                          ref d2,
                                                          ref d3,
                                                          ref d4,
                                                          out isDirectionChange))
            {
                throw Refusal.State(new UndefinedDerivative2DException("Geom2d_OffsetCurve::EvalD3"));
            }
        }
        if (!OffsetCurveUtils2D.CalculateD3(ref value,
                                                 ref d1,
                                                 ref d2,
                                                 ref d3,
                                                 d4,
                                                 isDirectionChange,
                                                 _offsetValue))
        {
            throw Refusal.State(new UndefinedDerivative2DException("Geom2d_OffsetCurve::EvalD3"));
        }
        return new ResD3 { Point = value, D1 = d1, D2 = d2, D3 = d3 };
    }

    /// <summary>The Nth derivative: through EvalD1 to EvalD3 up to N = 3, the basis curve's derivative above.</summary>
    /// <exception cref="UndefinedDerivative2DException">N &lt; 1, or the derivative can't be computed.</exception>
    internal sealed override Vector2D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw Refusal.State(new UndefinedDerivative2DException("Geom2d_OffsetCurve::EvalDN"));
        }

        Vector2D evalRepResult;
        if (RepUtils2D.TryEvalCurveDN(_evalRep, u, n, out evalRepResult))
        {
            return evalRepResult;
        }

        switch (n)
        {
            case 1:
            {
                return EvalD1(u).D1;
            }
            case 2:
            {
                return EvalD2(u).D2;
            }
            case 3:
            {
                return EvalD3(u).D3;
            }
            default:
                return _basisCurve.EvalDN(u, n);
        }
    }

    /// <summary>The basis curve's first parameter.</summary>
    public sealed override double FirstParameter => _basisCurve.FirstParameter;

    /// <summary>The basis curve's last parameter.</summary>
    public sealed override double LastParameter => _basisCurve.LastParameter;

    /// <summary>The offset value.</summary>
    public double Offset => _offsetValue;

    /// <summary>True if the points at the first and last parameters are within Precision::Computational() (squared distance).</summary>
    public sealed override bool IsClosed
    {
        get
        {
            Point2D pf, pl;
            D0(FirstParameter, out pf);
            D0(LastParameter, out pl);
            return pf.SquareDistanceTo(pl) <= global::Kerf.Engine.Foundation.Precision.Computational();
        }
    }

    /// <summary>True if the basis curve is C(N + 1).</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    internal sealed override bool IsCN(int n)
    {
        Refusal.RangeIf(n < 0, FailureKind.RangeError, " ");
        return _basisCurve.IsCN(n + 1);
    }

    /// <summary>True if the basis curve is periodic.</summary>
    public sealed override bool IsPeriodic => _basisCurve.IsPeriodic;

    /// <summary>The basis curve's period.</summary>
    public sealed override double Period => _basisCurve.Period;

    /// <summary>Transforms the basis curve and scales the offset by |T.ScaleFactor()|.</summary>
    protected internal sealed override void Transform(in Transform2D transform)
    {
        CheckMutable();
        ClearEvalRepresentation();
        OwnBasisCurve();
        _basisCurve.Transform(transform);
        _offsetValue *= Math.Abs(transform.ScaleFactor);
    }

    /// <summary>The basis curve's transformed parameter.</summary>
    internal sealed override double TransformedParameter(double u, in Transform2D transform) =>
        _basisCurve.TransformedParameter(u, transform);

    /// <summary>The basis curve's parametric transformation.</summary>
    internal sealed override double ParametricTransformation(in Transform2D transform) =>
        _basisCurve.ParametricTransformation(transform);

    /// <summary>The continuity of the basis curve (G1 for a C0 B-spline accepted as G1).</summary>
    internal Continuity GetBasisCurveContinuity() => _basisCurveContinuity;

    // Non-mutating twins of the mutators (Kerf)

    /// <summary>A copy on the basis curve C (see the constructor; the offsets of an offset C add up).</summary>
    public OffsetCurve2D WithBasisCurve(Curve2D curve, bool skipContinuityCheck = false)
    {
        OffsetCurve2D copy = new OffsetCurve2D(this);
        copy.SetBasisCurve(curve, skipContinuityCheck);
        return copy;
    }

    /// <summary>A copy with the offset D.</summary>
    public OffsetCurve2D WithOffsetValue(double d)
    {
        OffsetCurve2D copy = new OffsetCurve2D(this);
        copy.SetOffsetValue(d);
        return copy;
    }

    /// <summary>A copy evaluated through theDesc, which is copied (so later changes to theDesc don't affect it).</summary>
    /// <exception cref="ProgramErrorException">The descriptor is invalid.</exception>
    internal OffsetCurve2D WithEvalRepresentation(RepCurveDesc2D.Base? desc)
    {
        OffsetCurve2D copy = new OffsetCurve2D(this);
        RepUtils2D.ValidateCurveDesc(desc, this);
        copy.SetEvalRepresentation(RepUtils2D.CloneCurveDesc(desc));
        return copy;
    }

    /// <summary>A copy without an evaluation representation.</summary>
    internal OffsetCurve2D WithoutEvalRepresentation()
    {
        OffsetCurve2D copy = new OffsetCurve2D(this);
        copy.ClearEvalRepresentation();
        return copy;
    }
}
