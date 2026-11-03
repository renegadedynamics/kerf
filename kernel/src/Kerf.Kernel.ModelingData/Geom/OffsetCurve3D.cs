// Port of: src/ModelingData/TKG3d/Geom/Geom_OffsetCurve.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_OffsetCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Evaluation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Geometry;

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
public sealed class OffsetCurve3D : Curve3D
{
    private static readonly double MyAngularToleranceForG1 = global::Kerf.Engine.Foundation.Precision.Angular();

    private Curve3D _basisCurve = null!;
    private RepCurveDesc.Base? _evalRep;
    private Direction3D _direction;
    private double _offsetValue;
    private Continuity _basisCurveContinuity;

    /// <summary>True if an evaluation representation is attached.</summary>
    internal bool HasEvalRepresentation => _evalRep is not null;

    /// <summary>The evaluation representation descriptor (may be null). Stored, not copied: don't modify.</summary>
    internal RepCurveDesc.Base? EvalRepresentation() => _evalRep;

    /// <summary>Sets a new evaluation representation, after checking it (no circular reference).</summary>
    internal void SetEvalRepresentation(RepCurveDesc.Base? desc)
    {
        CheckMutable();
        RepUtils.ValidateCurveDesc(desc, this);
        _evalRep = desc;
    }

    /// <summary>Removes the evaluation representation.</summary>
    internal void ClearEvalRepresentation()
    {
        CheckMutable();
        _evalRep = null;
    }

    /// <summary>A new, unfrozen curve equal to this one (on a copy of the basis curve).</summary>
    public sealed override Geometry3D Copy() => new OffsetCurve3D(this);

    /// <summary>A copy of theOther (deep: the basis curve is copied), without validation.</summary>
    public OffsetCurve3D(OffsetCurve3D other)
    {
        _basisCurve = (Curve3D)other._basisCurve.Copy();
        _evalRep = RepUtils.CloneCurveDesc(other._evalRep);
        _direction = other._direction;
        _offsetValue = other._offsetValue;
        _basisCurveContinuity = other._basisCurveContinuity;
        // Deep copy without validation - source curve is already validated
    }

    /// <summary>
    /// The curve at distance theOffset from theCurve, in the direction theCurve' ^ theDir.
    /// </summary>
    /// <exception cref="ConstructionErrorException">
    /// The basis curve is a C0 B-spline that is not G1 (unless isTheNotCheckC0).
    /// </exception>
    public OffsetCurve3D(Curve3D curve, double offset, in Direction3D dir, bool isTheNotCheckC0 = false)
    {
        _direction = dir;
        _offsetValue = offset;
        SetBasisCurve(curve, isTheNotCheckC0);
    }

    // OCCT.NET: the basis curve may have been handed out (and frozen) by BasisCurve(); copy it before changing it.
    private void OwnBasisCurve()
    {
        if (_basisCurve.IsFrozen)
        {
            _basisCurve = (Curve3D)_basisCurve.Copy();
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

    /// <summary>The reference direction.</summary>
    public Direction3D Direction => _direction;

    internal void SetDirection(in Direction3D v)
    {
        CheckMutable();
        ClearEvalRepresentation();
        _direction = v;
    }

    internal void SetOffsetValue(double d)
    {
        CheckMutable();
        ClearEvalRepresentation();
        _offsetValue = d;
    }

    public sealed override bool IsPeriodic => _basisCurve.IsPeriodic;

    /// <summary>The period of the basis curve.</summary>
    public sealed override double Period => _basisCurve.Period;

    /// <summary>
    /// Changes the basis curve (a copy of C, with nested offsets folded into this one's offset and direction).
    /// </summary>
    /// <exception cref="ConstructionErrorException">C is a C0 B-spline that is not G1 (unless isNotCheckC0).</exception>
    internal void SetBasisCurve(Curve3D c, bool isNotCheckC0 = false)
    {
        CheckMutable();
        ClearEvalRepresentation();
        double uf = c.FirstParameter, ul = c.LastParameter;
        Curve3D checkingCurve = (Curve3D)c.Copy();
        bool isTrimmed = false;

        while (checkingCurve is TrimmedCurve3D || checkingCurve is OffsetCurve3D)
        {
            if (checkingCurve is TrimmedCurve3D trimC)
            {
                checkingCurve = trimC.BasisCurve;
                isTrimmed = true;
            }

            if (checkingCurve is OffsetCurve3D oc)
            {
                checkingCurve = oc.BasisCurve;
                double prevOff = oc.Offset;
                Vector3D v1 = new Vector3D(oc.Direction);
                Vector3D v2 = new Vector3D(_direction);
                Vector3D vdir = prevOff * v1 + _offsetValue * v2;

                if (_offsetValue >= 0.0)
                {
                    _offsetValue = vdir.Magnitude;
                    _direction = _direction.WithXYZ(vdir.XYZ());
                }
                else
                {
                    _offsetValue = -vdir.Magnitude;
                    _direction = _direction.WithXYZ((-vdir).XYZ());
                }
            }
        }

        _basisCurveContinuity = checkingCurve.Continuity;

        bool isC0 = !isNotCheckC0 && (_basisCurveContinuity == C0);

        // Basis curve must be at least C1
        if (isC0 && checkingCurve is BSplineCurve3D bc)
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
                throw new ConstructionErrorException("Offset on C0 curve");
            }
        }
        //
        if (isTrimmed)
        {
            _basisCurve = new TrimmedCurve3D(checkingCurve, uf, ul);
        }
        else
        {
            _basisCurve = checkingCurve;
        }
    }

    /// <summary>The basis curve, frozen (it is shared with this offset curve; decision 127).</summary>
    public Curve3D BasisCurve
    {
        get
        {
            _basisCurve.Freeze();
            return _basisCurve;
        }
    }

    /// <summary>The continuity of the basis curve lowered by one (C1 gives C0, G1 and G2 are kept).</summary>
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

    /// <exception cref="UndefinedValue3DException">The normal is undefined at theU.</exception>
    internal sealed override Point3D EvalD0(double u)
    {
        if (RepUtils.TryEvalCurveD0(_evalRep, u, out Point3D evalRepResult))
        {
            return evalRepResult;
        }
        ResD1 basisD1 = _basisCurve.EvalD1(u);
        Point3D value = basisD1.Point;
        if (!OffsetCurveUtils3D.CalculateD0(ref value, basisD1.D1, _direction.XYZ(), _offsetValue))
        {
            throw new UndefinedValue3DException("Geom_OffsetCurve::EvalD0");
        }
        return value;
    }

    /// <exception cref="UndefinedDerivative3DException">The derivative is undefined at theU.</exception>
    internal sealed override ResD1 EvalD1(double u)
    {
        if (RepUtils.TryEvalCurveD1(_evalRep, u, out ResD1 evalRepResult))
        {
            return evalRepResult;
        }
        ResD2 basisD2 = _basisCurve.EvalD2(u);
        Point3D value = basisD2.Point;
        Vector3D d1 = basisD2.D1;
        if (!OffsetCurveUtils3D.CalculateD1(ref value, ref d1, basisD2.D2, _direction.XYZ(), _offsetValue))
        {
            throw new UndefinedDerivative3DException("Geom_OffsetCurve::EvalD1");
        }
        return new ResD1 { Point = value, D1 = d1 };
    }

    /// <exception cref="UndefinedDerivative3DException">The derivatives are undefined at theU.</exception>
    internal sealed override ResD2 EvalD2(double u)
    {
        if (RepUtils.TryEvalCurveD2(_evalRep, u, out ResD2 evalRepResult))
        {
            return evalRepResult;
        }
        ResD3 basisD3 = _basisCurve.EvalD3(u);
        Point3D value = basisD3.Point;
        Vector3D d1 = basisD3.D1, d2 = basisD3.D2, d3 = basisD3.D3;

        bool isDirectionChange = false;
        if (d1.SquareMagnitude <= Constants.Resolution())
        {
            Vector3D dummyD4 = new Vector3D();
            if (!OffsetCurveUtils3D.AdjustDerivative(_basisCurve,
                                                        3,
                                                        u,
                                                        ref d1,
                                                        ref d2,
                                                        ref d3,
                                                        ref dummyD4,
                                                        out isDirectionChange))
            {
                throw new UndefinedDerivative3DException("Geom_OffsetCurve::EvalD2");
            }
        }

        if (!OffsetCurveUtils3D.CalculateD2(ref value,
                                               ref d1,
                                               ref d2,
                                               d3,
                                               _direction.XYZ(),
                                               _offsetValue,
                                               isDirectionChange))
        {
            throw new UndefinedDerivative3DException("Geom_OffsetCurve::EvalD2");
        }
        return new ResD2 { Point = value, D1 = d1, D2 = d2 };
    }

    /// <exception cref="UndefinedDerivative3DException">The derivatives are undefined at theU.</exception>
    internal sealed override ResD3 EvalD3(double u)
    {
        if (RepUtils.TryEvalCurveD3(_evalRep, u, out ResD3 evalRepResult))
        {
            return evalRepResult;
        }
        ResD3 basisD3 = _basisCurve.EvalD3(u);
        Vector3D d4Basis = _basisCurve.EvalDN(u, 4);
        Point3D value = basisD3.Point;
        Vector3D d1 = basisD3.D1, d2 = basisD3.D2, d3 = basisD3.D3;
        Vector3D d4 = d4Basis;

        bool isDirectionChange = false;
        if (d1.SquareMagnitude <= Constants.Resolution())
        {
            if (!OffsetCurveUtils3D.AdjustDerivative(_basisCurve,
                                                        4,
                                                        u,
                                                        ref d1,
                                                        ref d2,
                                                        ref d3,
                                                        ref d4,
                                                        out isDirectionChange))
            {
                throw new UndefinedDerivative3DException("Geom_OffsetCurve::EvalD3");
            }
        }

        if (!OffsetCurveUtils3D.CalculateD3(ref value,
                                               ref d1,
                                               ref d2,
                                               ref d3,
                                               d4,
                                               _direction.XYZ(),
                                               _offsetValue,
                                               isDirectionChange))
        {
            throw new UndefinedDerivative3DException("Geom_OffsetCurve::EvalD3");
        }
        return new ResD3 { Point = value, D1 = d1, D2 = d2, D3 = d3 };
    }

    /// <summary>The derivative of order N; above 3, the basis curve's (as in OCCT).</summary>
    /// <exception cref="UndefinedDerivative3DException">N &lt; 1, or the derivative is undefined at U.</exception>
    internal sealed override Vector3D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new UndefinedDerivative3DException("Geom_OffsetCurve::EvalDN");
        }

        if (RepUtils.TryEvalCurveDN(_evalRep, u, n, out Vector3D evalRepResult))
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

    public sealed override double FirstParameter => _basisCurve.FirstParameter;

    public sealed override double LastParameter => _basisCurve.LastParameter;

    /// <summary>The offset value.</summary>
    public double Offset => _offsetValue;

    /// <summary>True if the end points are within Precision::Computational() (squared distance).</summary>
    public sealed override bool IsClosed
    {
        get
        {
            Point3D pf, pl;
            D0(FirstParameter, out pf);
            D0(LastParameter, out pl);
            return pf.SquareDistanceTo(pl) <= global::Kerf.Engine.Foundation.Precision.Computational();
        }
    }

    /// <summary>True if the basis curve is C(N + 1).</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    internal sealed override bool IsCN(int n)
    {
        Standard_RangeError_Raise_if(n < 0, " ");
        return _basisCurve.IsCN(n + 1);
    }

    /// <summary>Applies T to the basis curve and the direction; the offset is scaled by T.ScaleFactor().</summary>
    protected internal sealed override void Transform(in Transform3D t)
    {
        CheckMutable();
        ClearEvalRepresentation();
        OwnBasisCurve();
        _basisCurve.Transform(t);
        _direction = _direction.Transformed(t);
        _offsetValue *= t.ScaleFactor;
    }

    /// <summary>The basis curve's transformed parameter.</summary>
    internal sealed override double TransformedParameter(double u, in Transform3D t) => _basisCurve.TransformedParameter(u, t);

    /// <summary>The basis curve's parametric transformation.</summary>
    internal sealed override double ParametricTransformation(in Transform3D t) => _basisCurve.ParametricTransformation(t);

    /// <summary>The continuity of the basis curve (G1 for a G1 C0 B-spline).</summary>
    internal Continuity GetBasisCurveContinuity() => _basisCurveContinuity;

    // Non-mutating twins (OCCT.NET)

    public OffsetCurve3D WithDirection(in Direction3D v)
    {
        OffsetCurve3D copy = (OffsetCurve3D)Copy();
        copy.SetDirection(v);
        return copy;
    }

    public OffsetCurve3D WithOffsetValue(double d)
    {
        OffsetCurve3D copy = (OffsetCurve3D)Copy();
        copy.SetOffsetValue(d);
        return copy;
    }

    /// <summary>A copy on the basis curve C (see <see cref="SetBasisCurve"/>).</summary>
    public OffsetCurve3D WithBasisCurve(Curve3D c, bool isNotCheckC0 = false)
    {
        OffsetCurve3D copy = (OffsetCurve3D)Copy();
        copy.SetBasisCurve(c, isNotCheckC0);
        return copy;
    }

    /// <summary>A copy evaluated through a copy of theDesc (null: without a representation).</summary>
    internal OffsetCurve3D WithEvalRepresentation(RepCurveDesc.Base? desc)
    {
        RepUtils.ValidateCurveDesc(desc, this);
        OffsetCurve3D copy = (OffsetCurve3D)Copy();
        copy.SetEvalRepresentation(RepUtils.CloneCurveDesc(desc));
        return copy;
    }

    /// <summary>A copy without an evaluation representation (the twin of ClearEvalRepresentation).</summary>
    internal OffsetCurve3D WithoutEvalRepresentation()
    {
        OffsetCurve3D copy = (OffsetCurve3D)Copy();
        copy.ClearEvalRepresentation();
        return copy;
    }
}
