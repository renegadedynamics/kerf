// Port of: src/ModelingData/TKG3d/Geom/Geom_BezierCurve.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_BezierCurve.cxx
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
/// A rational or non-rational Bezier curve: a table of poles (control points), indexed from 1, with weights when
/// rational. The degree is the number of poles minus 1, from 1 to <see cref="MaxDegree"/> (25). The parameter
/// range is [0, 1]; the first and last poles are the start and end points. A weight is zero when at most
/// gp::Resolution(), and two weights are equal when they differ by at most gp::Resolution(); a curve whose
/// weights are all equal is non-rational. A closed Bezier curve is never periodic.
/// </summary>
/// <remarks>
/// <para>
/// Arrays: <see cref="Poles()"/>, <see cref="Weights()"/>, <see cref="WeightsArray"/>, <see cref="Knots"/>,
/// <see cref="Multiplicities"/> and <see cref="KnotSequence"/> return immutable copies (OCCT returns references to
/// the curve's arrays); their index 0 is OCCT's index 1. Code inside OCCT.NET that passes the arrays on to
/// BSplCLib and the like uses the internal <c>Internal*</c> accessors, which return the stored arrays (shared:
/// don't modify).
/// </para>
/// <para>
/// The evaluation representation (<see cref="HasEvalRepresentation"/>) is set by <see cref="WithEvalRepresentation"/>,
/// which stores a copy of the descriptor.
/// </para>
/// </remarks>
public class BezierCurve3D : BoundedCurve3D
{
    private NCollection_Array1<Point3D> _poles = new NCollection_Array1<Point3D>();
    private NCollection_Array1<double> _weights = new NCollection_Array1<double>();
    private RepCurveDesc.Base? _evalRep;
    private bool _rational = false;
    private bool _closed = false;
    private double _maxDerivInv = 0.0;
    private bool _maxDerivInvOk = false;

    /// <summary>True if an evaluation representation is attached.</summary>
    public bool HasEvalRepresentation => _evalRep is not null;

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

    private static bool Rational(NCollection_Array1<double> w)
    {
        int i, n = w.Length();
        bool rat = false;
        for (i = 1; i < n; i++)
        {
            rat = Math.Abs(w[i] - w[i + 1]) > Constants.Resolution();
            if (rat)
            {
                break;
            }
        }
        return rat;
    }

    /// <summary>A copy of theOther, without validation (OCCT's copy constructor).</summary>
    public BezierCurve3D(BezierCurve3D other)
    {
        _poles = new NCollection_Array1<Point3D>(other._poles);
        _weights = other._rational ? new NCollection_Array1<double>(other._weights)
                                        : global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(other._poles.Length());
        _evalRep = RepUtils.CloneCurveDesc(other._evalRep);
        _rational = other._rational;
        _closed = other._closed;
        _maxDerivInvOk = false;
    }

    /// <summary>A non-rational Bezier curve with the given poles.</summary>
    /// <exception cref="ConstructionErrorException">Fewer than 2 or more than MaxDegree() + 1 poles.</exception>
    public BezierCurve3D(NCollection_Array1<Point3D> poles)
    {
        int nbpoles = poles.Length();
        if (nbpoles < 2 || nbpoles > (BezierCurve3D.MaxDegree() + 1))
        {
            throw new ConstructionErrorException();
        }
        // init non rational
        Init(poles, null);
    }

    /// <summary>A rational Bezier curve (non-rational if all the weights are equal).</summary>
    /// <exception cref="ConstructionErrorException">
    /// Fewer than 2 or more than MaxDegree() + 1 poles, as many weights as poles, or a weight at most gp::Resolution().
    /// </exception>
    public BezierCurve3D(NCollection_Array1<Point3D> poles, NCollection_Array1<double> weights)
    {
        // copy the poles
        int nbpoles = poles.Length();
        if (nbpoles < 2 || nbpoles > (BezierCurve3D.MaxDegree() + 1))
        {
            throw new ConstructionErrorException();
        }

        // check the weights
        if (weights.Length() != nbpoles)
        {
            throw new ConstructionErrorException();
        }

        for (int i = 1; i <= nbpoles; i++)
        {
            // OCCT: indexes Weights from 1 whatever its lower bound, as Rational does (out of range otherwise)
            if (weights[i] <= Constants.Resolution())
            {
                throw new ConstructionErrorException();
            }
        }

        // check really rational
        if (Rational(weights))
        {
            Init(poles, weights);
        }
        else
        {
            Init(poles, null);
        }
    }

    /// <summary>Raises the degree to Deg.</summary>
    /// <exception cref="ConstructionErrorException">Deg is less than the degree or more than MaxDegree().</exception>
    internal void Increase(int deg)
    {
        CheckMutable();
        if (deg == Degree())
        {
            return;
        }

        if (deg < Degree() || deg > BezierCurve3D.MaxDegree())
        {
            throw new ConstructionErrorException("Geom_BezierCurve::Increase");
        }

        int aDeg = _poles.Length() - 1;
        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, deg + 1);
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, 2);

        if (IsRational)
        {
            NCollection_Array1<double> nweights = new NCollection_Array1<double>(1, deg + 1);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(aDeg,
                                    deg,
                                    false,
                                    _poles,
                                    _weights,
                                    InternalKnots(),
                                    InternalMultiplicities(),
                                    npoles,
                                    nweights,
                                    nknots,
                                    nmults);
            Init(npoles, nweights);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(aDeg,
                                    deg,
                                    false,
                                    _poles,
                                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                    InternalKnots(),
                                    InternalMultiplicities(),
                                    npoles,
                                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                    nknots,
                                    nmults);
            Init(npoles, null);
        }
    }

    /// <summary>The maximum degree of a Bezier curve: 25.</summary>
    public static int MaxDegree() => global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree();

    /// <summary>Inserts P after the pole Index (weight 1 if the curve is rational).</summary>
    internal void InsertPoleAfter(int index, in Point3D p)
    {
        CheckMutable();
        InsertPoleAfter(index, p, 1.0);
    }

    /// <summary>Inserts P with its weight after the pole Index (0 inserts first).</summary>
    /// <exception cref="ConstructionErrorException">Too many poles, or Weight at most gp::Resolution().</exception>
    /// <exception cref="OutOfRangeException">Index is not in [0, NbPoles].</exception>
    internal void InsertPoleAfter(int index, in Point3D p, double weight)
    {
        CheckMutable();
        int nbpoles = PoleCount;

        // OCCT: refuses a 25-pole curve, though the constructors accept MaxDegree() + 1 = 26 poles
        if (nbpoles >= BezierCurve3D.MaxDegree() || weight <= Constants.Resolution())
        {
            throw new ConstructionErrorException("Geom_BezierCurve::InsertPoleAfter");
        }

        if (index < 0 || index > nbpoles)
        {
            throw new OutOfRangeException("Geom_BezierCurve::InsertPoleAfter");
        }

        int i;

        // Insert the pole
        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, nbpoles + 1);

        for (i = 1; i <= index; i++)
        {
            npoles[i] = _poles[i];
        }

        npoles[index + 1] = p;

        for (i = index + 1; i <= nbpoles; i++)
        {
            npoles[i + 1] = _poles[i];
        }

        // Insert the weight
        bool rat = IsRational || Math.Abs(weight - 1.0) > Constants.Resolution();

        if (rat)
        {
            NCollection_Array1<double> nweights = new NCollection_Array1<double>(1, nbpoles + 1);

            for (i = 1; i <= index; i++)
            {
                if (IsRational)
                {
                    nweights[i] = _weights[i];
                }
                else
                {
                    nweights[i] = 1.0;
                }
            }

            nweights[index + 1] = weight;

            for (i = index + 1; i <= nbpoles; i++)
            {
                if (IsRational)
                {
                    nweights[i + 1] = _weights[i];
                }
                else
                {
                    nweights[i + 1] = 1.0;
                }
            }

            Init(npoles, nweights);
        }
        else
        {
            Init(npoles, null);
        }
    }

    /// <summary>Inserts P before the pole Index (weight 1 if the curve is rational).</summary>
    internal void InsertPoleBefore(int index, in Point3D p)
    {
        CheckMutable();
        InsertPoleAfter(index - 1, p);
    }

    /// <summary>Inserts P with its weight before the pole Index.</summary>
    internal void InsertPoleBefore(int index, in Point3D p, double weight)
    {
        CheckMutable();
        InsertPoleAfter(index - 1, p, weight);
    }

    /// <summary>Removes the pole Index.</summary>
    /// <exception cref="ConstructionErrorException">The curve has only 2 poles.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    internal void RemovePole(int index)
    {
        CheckMutable();
        int nbpoles = PoleCount;

        if (nbpoles <= 2)
        {
            throw new ConstructionErrorException("Geom_BezierCurve::RemovePole");
        }

        if (index < 1 || index > nbpoles)
        {
            throw new OutOfRangeException("Geom_BezierCurve::RemovePole");
        }

        int i;

        // Remove the pole
        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, nbpoles - 1);

        for (i = 1; i < index; i++)
        {
            npoles[i] = _poles[i];
        }

        for (i = index + 1; i <= nbpoles; i++)
        {
            npoles[i - 1] = _poles[i];
        }

        // Remove the weight
        if (IsRational)
        {
            NCollection_Array1<double> nweights = new NCollection_Array1<double>(1, nbpoles - 1);

            for (i = 1; i < index; i++)
            {
                nweights[i] = _weights[i];
            }

            for (i = index + 1; i <= nbpoles; i++)
            {
                nweights[i - 1] = _weights[i];
            }

            Init(npoles, nweights);
        }
        else
        {
            Init(npoles, null);
        }
    }

    /// <summary>Reverses the parametrization: Value(NewU) = Value(1 - OldU).</summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        Point3D p;
        int i, nbpoles = PoleCount;

        // reverse poles
        for (i = 1; i <= nbpoles / 2; i++)
        {
            p = _poles[i];
            _poles[i] = _poles[nbpoles - i + 1];
            _poles[nbpoles - i + 1] = p;
        }

        // reverse weights
        if (IsRational)
        {
            double w;
            for (i = 1; i <= nbpoles / 2; i++)
            {
                w = _weights[i];
                _weights[i] = _weights[nbpoles - i + 1];
                _weights[nbpoles - i + 1] = w;
            }
        }

        _maxDerivInvOk = false;
    }

    /// <summary>1 - U.</summary>
    public sealed override double ReversedParameter(double u) => (1.0 - u);

    /// <summary>
    /// Keeps the part between U1 and U2 (which may be outside [0, 1]), oriented from U1 to U2, reparametrized on
    /// [0, 1].
    /// </summary>
    internal void Segment(double u1, double u2)
    {
        CheckMutable();
        _closed = (Math.Abs(Value(u1).Distance(Value(u2))) <= global::Kerf.Engine.Foundation.Precision.Confusion());

        int deg = _poles.Length() - 1;
        NCollection_Array1<Point3D> coeffs = new NCollection_Array1<Point3D>(1, _poles.Length());
        if (IsRational)
        {
            NCollection_Array1<double> wcoeffs = new NCollection_Array1<double>(1, _poles.Length());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildCache(0.0,
                                1.0,
                                false,
                                deg,
                                InternalKnotSequence(),
                                _poles,
                                _weights,
                                coeffs,
                                wcoeffs);
            global::Kerf.Engine.Numerics.Polynomials.PLib.Trimming(u1, u2, coeffs, wcoeffs);
            global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(coeffs, wcoeffs, _poles, _weights);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildCache(0.0,
                                1.0,
                                false,
                                deg,
                                InternalKnotSequence(),
                                _poles,
                                global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                coeffs,
                                global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
            global::Kerf.Engine.Numerics.Polynomials.PLib.Trimming(u1, u2, coeffs, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
            global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(coeffs, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), _poles, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
        }
        _maxDerivInvOk = false;
    }

    /// <summary>Replaces the pole Index (its weight is kept).</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    internal void SetPole(int index, in Point3D p)
    {
        CheckMutable();
        if (index < 1 || index > PoleCount)
        {
            throw new OutOfRangeException("Geom_BezierCurve::SetPole");
        }

        _poles[index] = p;

        if (index == 1 || index == _poles.Length())
        {
            _closed = (_poles[1].Distance(_poles[PoleCount]) <= global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        _maxDerivInvOk = false;
    }

    /// <summary>Replaces the pole Index and its weight.</summary>
    internal void SetPole(int index, in Point3D p, double weight)
    {
        CheckMutable();
        SetPole(index, p);
        SetWeight(index, weight);
    }

    /// <summary>Changes the weight of the pole Index; the curve may become rational or non-rational.</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    /// <exception cref="ConstructionErrorException">Weight is at most gp::Resolution().</exception>
    internal void SetWeight(int index, double weight)
    {
        CheckMutable();
        int nbpoles = PoleCount;

        if (index < 1 || index > nbpoles)
        {
            throw new OutOfRangeException("Geom_BezierCurve::SetWeight");
        }
        if (weight <= Constants.Resolution())
        {
            throw new ConstructionErrorException("Geom_BezierCurve::SetWeight");
        }

        // compute new rationality
        bool wasrat = IsRational;
        if (!wasrat)
        {
            // a weight of 1. does not turn to rational
            if (Math.Abs(weight - 1.0) <= Constants.Resolution())
            {
                return;
            }
            // Becoming rational: copy non-owning view to owned array.
            _weights = new NCollection_Array1<double>(_weights);
        }

        _weights[index] = weight;

        // is it turning into non rational
        if (wasrat && !Rational(_weights))
        {
            _rational = false;
            _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(nbpoles);
        }
        else
        {
            _rational = true;
        }
        _maxDerivInvOk = false;
    }

    /// <summary>True if the first and last poles are within Precision::Confusion().</summary>
    public sealed override bool IsClosed => _closed;

    /// <summary>True: a Bezier curve is CN.</summary>
    public sealed override bool IsCN(int n) => true;

    /// <summary>False.</summary>
    public sealed override bool IsPeriodic => false;

    /// <summary>False if all the weights are equal (within gp::Resolution()).</summary>
    public bool IsRational => _rational;

    /// <summary>CN.</summary>
    public sealed override Continuity Continuity() => CN;

    /// <summary>The degree: the number of poles minus 1.</summary>
    public int Degree() => _poles.Length() - 1;

    /// <summary>The point at U (U may be outside [0, 1]).</summary>
    public sealed override Point3D EvalD0(double u)
    {
        if (RepUtils.TryEvalCurveD0(_evalRep, u, out Point3D evalRepResult))
        {
            return evalRepResult;
        }
        Point3D p;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, _poles, InternalWeights(), out p);
        return p;
    }

    public sealed override ResD1 EvalD1(double u)
    {
        if (RepUtils.TryEvalCurveD1(_evalRep, u, out ResD1 evalRepResult))
        {
            return evalRepResult;
        }
        ResD1 result = new ResD1();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, _poles, InternalWeights(), out result.Point, out result.D1);
        return result;
    }

    public sealed override ResD2 EvalD2(double u)
    {
        if (RepUtils.TryEvalCurveD2(_evalRep, u, out ResD2 evalRepResult))
        {
            return evalRepResult;
        }
        ResD2 result = new ResD2();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u, _poles, InternalWeights(), out result.Point, out result.D1, out result.D2);
        return result;
    }

    public sealed override ResD3 EvalD3(double u)
    {
        if (RepUtils.TryEvalCurveD3(_evalRep, u, out ResD3 evalRepResult))
        {
            return evalRepResult;
        }
        ResD3 result = new ResD3();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(u, _poles, InternalWeights(), out result.Point, out result.D1, out result.D2, out result.D3);
        return result;
    }

    /// <exception cref="UndefinedDerivative3DException">N &lt; 1.</exception>
    public sealed override Vector3D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new UndefinedDerivative3DException();
        }
        if (RepUtils.TryEvalCurveDN(_evalRep, u, n, out Vector3D evalRepResult))
        {
            return evalRepResult;
        }
        Vector3D v;
        int deg = _poles.Length() - 1;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(u, n, 0, deg, false, _poles, InternalWeights(), InternalKnots(), InternalMultiplicities(), out v);
        return v;
    }

    /// <summary>The first pole.</summary>
    public sealed override Point3D StartPoint() => _poles[1];

    /// <summary>The last pole.</summary>
    public sealed override Point3D EndPoint() => _poles[_poles.Upper()];

    /// <summary>0.</summary>
    public sealed override double FirstParameter() => 0.0;

    /// <summary>1.</summary>
    public sealed override double LastParameter() => 1.0;

    public int PoleCount => _poles.Length();

    /// <summary>The pole Index.</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    public Point3D Pole(int index)
    {
        if (index < 1 || index > _poles.Length())
        {
            throw new OutOfRangeException("Geom_BezierCurve::Pole");
        }
        return _poles[index];
    }

    /// <summary>Copies the poles into P (OCCT's deprecated overload).</summary>
    /// <exception cref="DimensionErrorException">P has not NbPoles elements.</exception>
    public void Poles(NCollection_Array1<Point3D> p)
    {
        if (p.Length() != _poles.Length())
        {
            throw new DimensionErrorException("Geom_BezierCurve::Poles");
        }
        p.Assign(_poles);
    }

    /// <summary>The poles (a copy; index 0 is pole 1).</summary>
    public PackedList<Point3D> Poles() => _poles.ToPackedList();

    /// <summary>The weight of the pole Index (1 for a non-rational curve).</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    public double Weight(int index)
    {
        if (index < 1 || index > _poles.Length())
        {
            throw new OutOfRangeException("Geom_BezierCurve::Weight");
        }
        if (IsRational)
        {
            return _weights[index];
        }
        else
        {
            return 1.0;
        }
    }

    /// <summary>Copies the weights into W (OCCT's deprecated overload; 1s for a non-rational curve).</summary>
    /// <exception cref="DimensionErrorException">W has not NbPoles elements.</exception>
    public void Weights(NCollection_Array1<double> w)
    {
        int nbpoles = PoleCount;
        if (w.Length() != nbpoles)
        {
            throw new DimensionErrorException("Geom_BezierCurve::Weights");
        }
        if (IsRational)
        {
            w.Assign(_weights);
        }
        else
        {
            for (int i = 1; i <= nbpoles; i++)
            {
                w[i] = 1.0;
            }
        }
    }

    /// <summary>The weights (a copy), or null for a non-rational curve (OCCT returns a null pointer).</summary>
    public PackedList<double>? Weights() => _rational ? _weights.ToPackedList() : null;

    /// <summary>The weights, always NbPoles of them (1s for a non-rational curve); a copy.</summary>
    public PackedList<double> WeightsArray() => _weights.ToPackedList();

    /// <summary>Applies T to the poles.</summary>
    protected internal sealed override void Transform(in Transform3D t)
    {
        CheckMutable();
        int nbpoles = PoleCount;

        for (int i = 1; i <= nbpoles; i++)
        {
            _poles[i].Transform(t);
        }
        ClearEvalRepresentation();
        _maxDerivInvOk = false;
    }

    /// <summary>
    /// The parametric tolerance UTolerance such that |t1 - t0| &lt; UTolerance implies
    /// |f(t1) - f(t0)| &lt; Tolerance3D.
    /// </summary>
    /// <remarks>Non-const in OCCT, because it caches a value; not a mutator, so it works on a frozen curve.</remarks>
    public void Resolution(double tolerance3D, out double uTolerance)
    {
        // OCCT.NET: a cache filled on a possibly shared curve; the flag is published after the value.
        if (!Volatile.Read(ref _maxDerivInvOk))
        {
            double maxDerivInv;
            int deg = _poles.Length() - 1;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Resolution(_poles,
                                InternalWeights(),
                                _poles.Length(),
                                InternalKnotSequence(),
                                deg,
                                1.0,
                                out maxDerivInv);
            _maxDerivInv = maxDerivInv;
            Volatile.Write(ref _maxDerivInvOk, true);
        }
        uTolerance = tolerance3D * _maxDerivInv;
    }

    /// <summary>A new, unfrozen curve equal to this one.</summary>
    public sealed override Geometry3D Copy() => new BezierCurve3D(this);

    /// <summary>Sets poles and weights (null: non-rational); updates the rational and closed flags.</summary>
    protected void Init(NCollection_Array1<Point3D> poles, NCollection_Array1<double>? weights)
    {
        int nbpoles = poles.Length();
        // closed ?
        _closed = poles[poles.Lower()].Distance(poles[poles.Upper()]) <= global::Kerf.Engine.Foundation.Precision.Confusion();

        // set fields
        _poles.Resize(1, nbpoles, false);
        _poles.CopyValues(poles);

        if (weights is not null)
        {
            _weights.Resize(1, nbpoles, false);
            _weights.CopyValues(weights);
            _rational = Rational(_weights);
            if (!_rational)
            {
                _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(nbpoles);
            }
        }
        else
        {
            _rational = false;
            _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(nbpoles);
        }

        _maxDerivInv = 0.0;
        _maxDerivInvOk = false;
        ClearEvalRepresentation();
    }

    // The Bezier knots and multiplicities, shared per degree as OCCT's static arrays.

    private static readonly NCollection_Array1<double> THE_KNOTS = NCollection_Array1<double>.From([0.0, 1.0]);

    private static readonly NCollection_Array1<int>[] THE_MULTS = CreateMults();

    private static readonly NCollection_Array1<double>?[] THE_FKNOTS = CreateFlatKnots();

    private static NCollection_Array1<int>[] CreateMults()
    {
        int THE_MAX_SIZE = global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree() + 1;
        NCollection_Array1<int>[] arr = new NCollection_Array1<int>[THE_MAX_SIZE];
        for (int i = 0; i < THE_MAX_SIZE; ++i)
        {
            arr[i] = NCollection_Array1<int>.From([i + 1, i + 1]);
        }
        return arr;
    }

    private static NCollection_Array1<double>?[] CreateFlatKnots()
    {
        int THE_MAX_SIZE = global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree() + 1;
        NCollection_Array1<double>?[] arr = new NCollection_Array1<double>?[THE_MAX_SIZE];
        for (int i = 1; i <= global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree(); ++i)
        {
            arr[i] = NCollection_Array1<double>.From(global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatBezierKnots(i));
        }
        return arr;
    }

    /// <summary>The Bezier knots {0, 1}: shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalKnots() => THE_KNOTS;

    /// <summary>The Bezier multiplicities {Degree + 1, Degree + 1}: shared, don't modify.</summary>
    internal NCollection_Array1<int> InternalMultiplicities()
    {
        Standard_ProgramError_Raise_if(_poles.IsEmpty(), "Geom_BezierCurve: empty poles");
        return THE_MULTS[_poles.Length() - 1];
    }

    /// <summary>The Bezier flat knots (Degree + 1 zeros, Degree + 1 ones): shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalKnotSequence()
    {
        Standard_ProgramError_Raise_if(_poles.IsEmpty(), "Geom_BezierCurve: empty poles");
        return THE_FKNOTS[_poles.Length() - 1]!;
    }

    /// <summary>The stored poles: shared, don't modify (OCCT's <c>Poles()</c> const reference).</summary>
    internal NCollection_Array1<Point3D> InternalPoles() => _poles;

    /// <summary>The stored weights, or null for a non-rational curve (OCCT's <c>Weights()</c> pointer). Don't modify.</summary>
    internal NCollection_Array1<double>? InternalWeights() => _rational ? _weights : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights();

    /// <summary>The weights, always NbPoles of them (OCCT's <c>WeightsArray()</c>): shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalWeightsArray() => _weights;

    /// <summary>The Bezier knots {0, 1} (a copy).</summary>
    public PackedList<double> Knots() => THE_KNOTS.ToPackedList();

    /// <summary>The Bezier multiplicities {Degree + 1, Degree + 1} (a copy).</summary>
    public PackedList<int> Multiplicities() => InternalMultiplicities().ToPackedList();

    /// <summary>The Bezier flat knots (a copy).</summary>
    public PackedList<double> KnotSequence() => InternalKnotSequence().ToPackedList();

    // Non-mutating twins (OCCT.NET)

    /// <summary>A copy with the degree raised to Deg.</summary>
    public BezierCurve3D Increased(int deg)
    {
        BezierCurve3D copy = (BezierCurve3D)Copy();
        copy.Increase(deg);
        return copy;
    }

    /// <summary>A copy with P inserted after the pole Index.</summary>
    public BezierCurve3D PoleInsertedAfter(int index, in Point3D p)
    {
        BezierCurve3D copy = (BezierCurve3D)Copy();
        copy.InsertPoleAfter(index, p);
        return copy;
    }

    /// <summary>A copy with P and its weight inserted after the pole Index.</summary>
    public BezierCurve3D PoleInsertedAfter(int index, in Point3D p, double weight)
    {
        BezierCurve3D copy = (BezierCurve3D)Copy();
        copy.InsertPoleAfter(index, p, weight);
        return copy;
    }

    /// <summary>A copy with P inserted before the pole Index.</summary>
    public BezierCurve3D PoleInsertedBefore(int index, in Point3D p)
    {
        BezierCurve3D copy = (BezierCurve3D)Copy();
        copy.InsertPoleBefore(index, p);
        return copy;
    }

    /// <summary>A copy with P and its weight inserted before the pole Index.</summary>
    public BezierCurve3D PoleInsertedBefore(int index, in Point3D p, double weight)
    {
        BezierCurve3D copy = (BezierCurve3D)Copy();
        copy.InsertPoleBefore(index, p, weight);
        return copy;
    }

    /// <summary>A copy without the pole Index.</summary>
    public BezierCurve3D PoleRemoved(int index)
    {
        BezierCurve3D copy = (BezierCurve3D)Copy();
        copy.RemovePole(index);
        return copy;
    }

    /// <summary>A copy segmented between U1 and U2 (see <see cref="Segment"/>).</summary>
    public BezierCurve3D Segmented(double u1, double u2)
    {
        BezierCurve3D copy = (BezierCurve3D)Copy();
        copy.Segment(u1, u2);
        return copy;
    }

    public BezierCurve3D WithPole(int index, in Point3D p)
    {
        BezierCurve3D copy = (BezierCurve3D)Copy();
        copy.SetPole(index, p);
        return copy;
    }

    public BezierCurve3D WithPole(int index, in Point3D p, double weight)
    {
        BezierCurve3D copy = (BezierCurve3D)Copy();
        copy.SetPole(index, p, weight);
        return copy;
    }

    public BezierCurve3D WithWeight(int index, double weight)
    {
        BezierCurve3D copy = (BezierCurve3D)Copy();
        copy.SetWeight(index, weight);
        return copy;
    }

    /// <summary>A copy evaluated through a copy of theDesc (null: without a representation).</summary>
    public BezierCurve3D WithEvalRepresentation(RepCurveDesc.Base? desc)
    {
        RepUtils.ValidateCurveDesc(desc, this);
        BezierCurve3D copy = (BezierCurve3D)Copy();
        copy.SetEvalRepresentation(RepUtils.CloneCurveDesc(desc));
        return copy;
    }

    /// <summary>A copy without an evaluation representation (the twin of ClearEvalRepresentation).</summary>
    public BezierCurve3D WithoutEvalRepresentation()
    {
        BezierCurve3D copy = (BezierCurve3D)Copy();
        copy.ClearEvalRepresentation();
        return copy;
    }
}
