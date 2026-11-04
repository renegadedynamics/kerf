// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_BezierCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_BezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
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
/// A rational or non-rational Bezier curve in the plane, defined by 2 to MaxDegree() + 1 poles and, if
/// rational, one positive weight per pole. The degree is the number of poles minus 1; the parameter range is
/// [0, 1]; the curve passes through its first and last poles, and is infinitely smooth.
/// </summary>
/// <remarks>
/// <para>
/// OCCT returns its pole, weight and knot arrays by const reference. The port returns them to callers as
/// <see cref="PackedList{T}"/> copies (<see cref="Poles"/>, <see cref="WeightsOrNull()"/>, ...), and gives ported
/// code the arrays themselves through internal <c>Internal...()</c> accessors, which must not be modified
/// (decision 131).
/// </para>
/// <para>
/// <see cref="Resolution"/> caches a value in the curve, as in OCCT; on a shared (frozen) curve the cache is
/// filled at most once per value and is safe to read from several threads.
/// </para>
/// </remarks>
public sealed partial class BezierCurve2D : BoundedCurve2D
{
    private NCollection_Array1<Point2D> _poles = new NCollection_Array1<Point2D>();
    private NCollection_Array1<double> _weights = new NCollection_Array1<double>();
    private RepCurveDesc2D.Base? _evalRep;
    private bool _rational = false;
    private bool _closed = false;
    private double _maxDerivInv = 0.0;
    private bool _maxDerivInvOk = false;

    /// <summary>True if an evaluation representation is attached.</summary>
    internal bool HasEvalRepresentation => _evalRep is not null;

    /// <summary>The attached evaluation representation, or null. Internal: descriptors are mutable (decision 507).</summary>
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

    /// <summary>The non-rational Bezier curve of poles CurvePoles.</summary>
    /// <exception cref="ConstructionErrorException">Fewer than 2 or more than MaxDegree() + 1 poles.</exception>
    internal BezierCurve2D(NCollection_Array1<Point2D> curvePoles)
    {
        int nbpoles = curvePoles.Length();
        if (nbpoles < 2 || nbpoles > (BezierCurve2D.MaxDegree + 1))
        {
            throw new ConstructionErrorException();
        }
        // init non rational
        Init(curvePoles, null);
    }

    /// <summary>
    /// The rational Bezier curve of poles CurvePoles and weights PoleWeights (non-rational if all the weights
    /// are equal).
    /// </summary>
    /// <exception cref="ConstructionErrorException">
    /// Fewer than 2 or more than MaxDegree() + 1 poles, a weight count different from the pole count, or a
    /// weight not above gp::Resolution().
    /// </exception>
    internal BezierCurve2D(NCollection_Array1<Point2D> curvePoles, NCollection_Array1<double> poleWeights)
    {
        int nbpoles = curvePoles.Length();
        if (nbpoles < 2 || nbpoles > (BezierCurve2D.MaxDegree + 1))
        {
            throw new ConstructionErrorException();
        }

        if (poleWeights.Length() != nbpoles)
        {
            throw new ConstructionErrorException();
        }

        for (int i = 1; i <= nbpoles; i++)
        {
            // OCCT: indexes the weights from 1 whatever their lower bound (occt-issues.md).
            if (poleWeights[i] <= Constants.Resolution())
            {
                throw new ConstructionErrorException();
            }
        }

        // check really rational
        if (Rational(poleWeights))
        {
            Init(curvePoles, poleWeights);
        }
        else
        {
            Init(curvePoles, null);
        }
    }

    /// <summary>A copy of theOther (its evaluation representation deep-copied), not frozen.</summary>
    public BezierCurve2D(BezierCurve2D other)
    {
        _poles = new NCollection_Array1<Point2D>(other._poles);
        _weights = other._rational ? new NCollection_Array1<double>(other._weights)
                                        : global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(other._poles.Length());
        _evalRep = RepUtils2D.CloneCurveDesc(other._evalRep);
        _rational = other._rational;
        _closed = other._closed;
        _maxDerivInvOk = false;
    }

    /// <summary>Raises the degree to Degree, keeping the shape of the curve.</summary>
    /// <exception cref="ConstructionErrorException">Degree is below the current degree or above MaxDegree().</exception>
    internal void Increase(int deg)
    {
        CheckMutable();
        if (deg == Degree)
        {
            return;
        }

        Standard_ConstructionError_Raise_if(deg < Degree || deg > BezierCurve2D.MaxDegree,
                                            "Geom2d_BezierCurve::Increase");

        NCollection_Array1<Point2D> npoles = new NCollection_Array1<Point2D>(1, deg + 1);
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, 2);

        if (IsRational)
        {
            NCollection_Array1<double> nweights = new NCollection_Array1<double>(1, deg + 1);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(Degree,
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
            global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(Degree,
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
    public static int MaxDegree => global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree();

    /// <summary>Inserts the pole P (with the weight Weight) after the pole of index Index (0 inserts first).</summary>
    /// <exception cref="ConstructionErrorException">
    /// The curve already has MaxDegree() poles, or Weight is not above gp::Resolution().
    /// </exception>
    /// <exception cref="OutOfRangeException">Index is outside [0, NbPoles()].</exception>
    internal void InsertPoleAfter(int index, in Point2D p, double weight = 1.0)
    {
        CheckMutable();
        int nbpoles = PoleCount;

        Standard_ConstructionError_Raise_if(nbpoles >= BezierCurve2D.MaxDegree
                                              || weight <= Constants.Resolution(),
                                            "Geom2d_BezierCurve::InsertPoleAfter");

        Standard_OutOfRange_Raise_if(index < 0 || index > nbpoles, "Geom2d_BezierCurve::InsertPoleAfter");

        int i;

        // Insert the pole
        NCollection_Array1<Point2D> npoles = new NCollection_Array1<Point2D>(1, nbpoles + 1);

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

    /// <summary>Inserts the pole P (with the weight Weight) before the pole of index Index.</summary>
    /// <exception cref="ConstructionErrorException">
    /// The curve already has MaxDegree() poles, or Weight is not above gp::Resolution().
    /// </exception>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbPoles() + 1].</exception>
    internal void InsertPoleBefore(int index, in Point2D p, double weight = 1.0)
    {
        CheckMutable();
        InsertPoleAfter(index - 1, p, weight);
    }

    /// <summary>Removes the pole of index Index.</summary>
    /// <exception cref="ConstructionErrorException">The curve has only 2 poles.</exception>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbPoles()].</exception>
    internal void RemovePole(int index)
    {
        CheckMutable();
        int nbpoles = PoleCount;

        Standard_ConstructionError_Raise_if(nbpoles <= 2, "Geom2d_BezierCurve::RemovePole");

        Standard_OutOfRange_Raise_if(index < 1 || index > nbpoles, "Geom2d_BezierCurve::RemovePole");

        int i;

        // Remove the pole
        NCollection_Array1<Point2D> npoles = new NCollection_Array1<Point2D>(1, nbpoles - 1);

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

    /// <summary>Reverses the order of the poles (and weights).</summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        Point2D p;
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
        // OCCT: keeps the evaluation representation, as SetPole, SetWeight and Segment do; only init and Transform
        // drop it (occt-issues.md).
    }

    /// <summary>1 - U.</summary>
    public sealed override double ReversedParameter(double u) => (1.0 - u);

    /// <summary>
    /// Reparametrizes the portion of the curve between U1 and U2 onto [0, 1]. U1 &gt; U2 reverses the curve;
    /// U1 and U2 may lie outside [0, 1] (extrapolation).
    /// </summary>
    internal void Segment(double u1, double u2)
    {
        CheckMutable();
        _closed = (Math.Abs(Value(u1).DistanceTo(Value(u2))) <= Constants.Resolution());

        NCollection_Array1<Point2D> coeffs = new NCollection_Array1<Point2D>(1, _poles.Length());
        if (IsRational)
        {
            NCollection_Array1<double> wcoeffs = new NCollection_Array1<double>(1, _poles.Length());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildCache(0.0,
                                1.0,
                                false,
                                Degree,
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
                                Degree,
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

    /// <summary>Replaces the pole of index Index by P.</summary>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbPoles()].</exception>
    internal void SetPole(int index, in Point2D p)
    {
        CheckMutable();
        Standard_OutOfRange_Raise_if(index < 1 || index > PoleCount, "Geom2d_BezierCurve::SetPole");
        _poles[index] = p;

        if (index == 1 || index == _poles.Length())
        {
            _closed = (_poles[1].DistanceTo(_poles[PoleCount]) <= Constants.Resolution());
        }
        _maxDerivInvOk = false;
    }

    /// <summary>Replaces the pole of index Index by P, and its weight by Weight.</summary>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbPoles()].</exception>
    /// <exception cref="ConstructionErrorException">Weight is not above gp::Resolution().</exception>
    internal void SetPole(int index, in Point2D p, double weight)
    {
        CheckMutable();
        SetPole(index, p);
        SetWeight(index, weight);
    }

    /// <summary>Changes the weight of the pole of index Index; the curve may become rational or non-rational.</summary>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbPoles()].</exception>
    /// <exception cref="ConstructionErrorException">Weight is not above gp::Resolution().</exception>
    internal void SetWeight(int index, double weight)
    {
        CheckMutable();
        int nbpoles = PoleCount;

        Standard_OutOfRange_Raise_if(index < 1 || index > nbpoles, "Geom2d_BezierCurve::SetWeight");
        Standard_ConstructionError_Raise_if(weight <= Constants.Resolution(), "Geom2d_BezierCurve::SetWeight");

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

    /// <summary>True if the first and last poles are within gp::Resolution().</summary>
    public sealed override bool IsClosed => _closed;

    /// <summary>True: a Bezier curve is CN.</summary>
    internal sealed override bool IsCN(int n) => true;

    /// <summary>False.</summary>
    public sealed override bool IsPeriodic => false;

    /// <summary>True if the weights are not all equal.</summary>
    public bool IsRational => _rational;

    /// <summary>CN.</summary>
    public sealed override Continuity Continuity => CN;

    /// <summary>The degree: NbPoles() - 1.</summary>
    public int Degree => _poles.Length() - 1;

    internal sealed override Point2D EvalD0(double u)
    {
        Point2D evalRepResult;
        if (RepUtils2D.TryEvalCurveD0(_evalRep, u, out evalRepResult))
        {
            return evalRepResult;
        }
        Point2D p;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, _poles, InternalWeights(), out p);
        return p;
    }

    internal sealed override ResD1 EvalD1(double u)
    {
        ResD1 evalRepResult;
        if (RepUtils2D.TryEvalCurveD1(_evalRep, u, out evalRepResult))
        {
            return evalRepResult;
        }
        ResD1 result;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, _poles, InternalWeights(), out result.Point, out result.D1);
        return result;
    }

    internal sealed override ResD2 EvalD2(double u)
    {
        ResD2 evalRepResult;
        if (RepUtils2D.TryEvalCurveD2(_evalRep, u, out evalRepResult))
        {
            return evalRepResult;
        }
        ResD2 result;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u, _poles, InternalWeights(), out result.Point, out result.D1, out result.D2);
        return result;
    }

    internal sealed override ResD3 EvalD3(double u)
    {
        ResD3 evalRepResult;
        if (RepUtils2D.TryEvalCurveD3(_evalRep, u, out evalRepResult))
        {
            return evalRepResult;
        }
        ResD3 result;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(u, _poles, InternalWeights(), out result.Point, out result.D1, out result.D2, out result.D3);
        return result;
    }

    /// <exception cref="UndefinedDerivative2DException">N &lt; 1.</exception>
    internal sealed override Vector2D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new UndefinedDerivative2DException();
        }
        Vector2D evalRepResult;
        if (RepUtils2D.TryEvalCurveDN(_evalRep, u, n, out evalRepResult))
        {
            return evalRepResult;
        }
        Vector2D v;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(u, n, 0, Degree, false, _poles, InternalWeights(), InternalKnots(), InternalMultiplicities(), out v);
        return v;
    }

    /// <summary>The last pole.</summary>
    public sealed override Point2D EndPoint => _poles[_poles.Upper()];

    /// <summary>0.</summary>
    public sealed override double FirstParameter => 0.0;

    /// <summary>1.</summary>
    public sealed override double LastParameter => 1.0;

    /// <summary>The number of poles.</summary>
    public int PoleCount => _poles.Length();

    /// <summary>The pole of index Index.</summary>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbPoles()].</exception>
    internal Point2D Pole(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > _poles.Length(), "Geom2d_BezierCurve::Pole");
        return _poles[index];
    }

    /// <summary>Copies the poles into P (indexed as P is).</summary>
    /// <exception cref="DimensionErrorException">P's length is not NbPoles().</exception>
    internal void CopyPolesTo(NCollection_Array1<Point2D> p)
    {
        Standard_DimensionError_Raise_if(p.Length() != _poles.Length(), "Geom2d_BezierCurve::Poles");
        p.Assign(_poles);
    }

    /// <summary>The poles (a copy; OCCT returns a reference to its array).</summary>
    public IReadOnlyList<Point2D> Poles => _poles.ToPackedList();

    /// <summary>The pole array itself, indexed from 1, for ported code: don't modify it (decision 131).</summary>
    internal NCollection_Array1<Point2D> InternalPoles() => _poles;

    /// <summary>The first pole.</summary>
    public sealed override Point2D StartPoint => _poles[1];

    /// <summary>The weight of the pole of index Index (1 if the curve is not rational).</summary>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbPoles()].</exception>
    internal double Weight(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > _poles.Length(), "Geom2d_BezierCurve::Weight");
        if (IsRational)
        {
            return _weights[index];
        }
        else
        {
            return 1.0;
        }
    }

    /// <summary>Copies the weights into W (ones if the curve is not rational).</summary>
    /// <exception cref="DimensionErrorException">W's length is not NbPoles().</exception>
    internal void CopyWeightsTo(NCollection_Array1<double> w)
    {
        int nbpoles = PoleCount;
        Standard_DimensionError_Raise_if(w.Length() != nbpoles, "Geom2d_BezierCurve::Weights");
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

    /// <summary>The weights (a copy), or null if the curve is not rational (OCCT's <c>BSplCLib::NoWeights()</c>).</summary>
    internal IReadOnlyList<double>? WeightsOrNull() => _rational ? _weights.ToPackedList() : null;

    /// <summary>The weight array for ported code, or null if the curve is not rational: don't modify it.</summary>
    internal NCollection_Array1<double>? InternalWeights() => _rational ? _weights : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights();

    /// <summary>The weights (a copy), ones if the curve is not rational.</summary>
    public IReadOnlyList<double> Weights => _weights.ToPackedList();

    /// <summary>The weight array (shared unit weights if not rational) for ported code: don't modify it.</summary>
    internal NCollection_Array1<double> InternalWeightsArray() => _weights;

    /// <summary>Applies T to the poles; the evaluation representation is dropped.</summary>
    protected internal sealed override void Transform(in Transform2D t)
    {
        CheckMutable();
        int nbpoles = PoleCount;

        for (int i = 1; i <= nbpoles; i++)
        {
            _poles[i] = _poles[i].Transformed(t);
        }

        ClearEvalRepresentation();
        _maxDerivInvOk = false;
    }

    /// <summary>
    /// The parametric tolerance UTolerance such that |t1 - t0| &lt; UTolerance implies
    /// |C(t1) - C(t0)| &lt; ToleranceUV.
    /// </summary>
    internal void Resolution(double toleranceUV, out double uTolerance)
    {
        // OCCT.NET: a cache filled on a possibly shared curve; the flag is published after the value.
        if (!Volatile.Read(ref _maxDerivInvOk))
        {
            double maxDerivInv;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Resolution(_poles,
                                InternalWeights(),
                                _poles.Length(),
                                InternalKnotSequence(),
                                Degree,
                                1.0,
                                out maxDerivInv);
            _maxDerivInv = maxDerivInv;
            Volatile.Write(ref _maxDerivInvOk, true);
        }
        uTolerance = toleranceUV * _maxDerivInv;
    }

    /// <summary>A new, unfrozen curve equal to this one.</summary>
    public sealed override Geometry2D Copy() => new BezierCurve2D(this);

    /// <summary>The knots {0, 1} of the curve seen as a B-spline (a copy).</summary>
    public IReadOnlyList<double> Knots => InternalKnots().ToPackedList();

    /// <summary>The multiplicities {Degree() + 1, Degree() + 1} of the curve seen as a B-spline (a copy).</summary>
    public IReadOnlyList<int> Multiplicities => InternalMultiplicities().ToPackedList();

    /// <summary>The flat knots of the curve seen as a B-spline: Degree() + 1 zeros then Degree() + 1 ones (a copy).</summary>
    public IReadOnlyList<double> KnotSequence => InternalKnotSequence().ToPackedList();

    private static readonly NCollection_Array1<double> THE_KNOTS = NCollection_Array1<double>.From([0.0, 1.0]);

    private static readonly NCollection_Array1<int>[] THE_MULTS = BuildMults();

    private static readonly NCollection_Array1<double>[] THE_FKNOTS = BuildFlatKnots();

    private static NCollection_Array1<int>[] BuildMults()
    {
        int THE_MAX_SIZE = global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree() + 1;
        NCollection_Array1<int>[] arr = new NCollection_Array1<int>[THE_MAX_SIZE];
        for (int i = 0; i < THE_MAX_SIZE; ++i)
        {
            arr[i] = NCollection_Array1<int>.From([i + 1, i + 1]);
        }
        return arr;
    }

    private static NCollection_Array1<double>[] BuildFlatKnots()
    {
        int THE_MAX_SIZE = global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree() + 1;
        NCollection_Array1<double>[] arr = new NCollection_Array1<double>[THE_MAX_SIZE];
        arr[0] = new NCollection_Array1<double>();
        for (int i = 1; i <= global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree(); ++i)
        {
            arr[i] = NCollection_Array1<double>.From(global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatBezierKnots(i));
        }
        return arr;
    }

    /// <summary>The shared knot array {0, 1}, for ported code: don't modify it.</summary>
    internal NCollection_Array1<double> InternalKnots() => THE_KNOTS;

    /// <summary>The shared multiplicity array of this degree, for ported code: don't modify it.</summary>
    internal NCollection_Array1<int> InternalMultiplicities()
    {
        Standard_ProgramError_Raise_if(_poles.IsEmpty(), "Geom2d_BezierCurve: empty poles");
        return THE_MULTS[_poles.Length() - 1];
    }

    /// <summary>The shared flat knot array of this degree, for ported code: don't modify it.</summary>
    internal NCollection_Array1<double> InternalKnotSequence()
    {
        Standard_ProgramError_Raise_if(_poles.IsEmpty(), "Geom2d_BezierCurve: empty poles");
        return THE_FKNOTS[_poles.Length() - 1];
    }

    /// <summary>Sets the poles and weights (null: non-rational), resetting the caches and the evaluation representation.</summary>
    private void Init(NCollection_Array1<Point2D> poles, NCollection_Array1<double>? weights)
    {
        int nbpoles = poles.Length();
        // closed ?
        _closed = poles[poles.Lower()].DistanceTo(poles[poles.Upper()]) <= Constants.Resolution();

        // set fields
        _poles.Resize(1, nbpoles, false);
        _poles.CopyValues(poles);

        if (weights is not null)
        {
            // OCCT.NET: never write into the shared unit weights (a resize to the same size keeps the view).
            if (!_weights.IsDeletable())
            {
                _weights = new NCollection_Array1<double>();
            }
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

    // Non-mutating twins of the mutators (OCCT.NET)

    /// <summary>A copy with its degree raised to Deg.</summary>
    public BezierCurve2D Increased(int deg)
    {
        BezierCurve2D copy = new BezierCurve2D(this);
        copy.Increase(deg);
        return copy;
    }

    /// <summary>A copy with the pole P (weight Weight) inserted after the pole of index Index.</summary>
    internal BezierCurve2D PoleInsertedAfter(int index, in Point2D p, double weight = 1.0)
    {
        BezierCurve2D copy = new BezierCurve2D(this);
        copy.InsertPoleAfter(index, p, weight);
        return copy;
    }

    /// <summary>A copy with the pole P (weight Weight) inserted before the pole of index Index.</summary>
    internal BezierCurve2D PoleInsertedBefore(int index, in Point2D p, double weight = 1.0)
    {
        BezierCurve2D copy = new BezierCurve2D(this);
        copy.InsertPoleBefore(index, p, weight);
        return copy;
    }

    /// <summary>A copy without the pole of index Index.</summary>
    internal BezierCurve2D PoleRemoved(int index)
    {
        BezierCurve2D copy = new BezierCurve2D(this);
        copy.RemovePole(index);
        return copy;
    }

    /// <summary>A copy reparametrized between U1 and U2 (see Segment).</summary>
    public BezierCurve2D Segmented(double u1, double u2)
    {
        BezierCurve2D copy = new BezierCurve2D(this);
        copy.Segment(u1, u2);
        return copy;
    }

    /// <summary>A copy with the pole of index Index replaced by P.</summary>
    internal BezierCurve2D WithPoleOneBased(int index, in Point2D p)
    {
        BezierCurve2D copy = new BezierCurve2D(this);
        copy.SetPole(index, p);
        return copy;
    }

    /// <summary>A copy with the pole of index Index replaced by P, of weight Weight.</summary>
    internal BezierCurve2D WithPoleOneBased(int index, in Point2D p, double weight)
    {
        BezierCurve2D copy = new BezierCurve2D(this);
        copy.SetPole(index, p, weight);
        return copy;
    }

    /// <summary>A copy with the weight of the pole of index Index changed to Weight.</summary>
    internal BezierCurve2D WithWeightOneBased(int index, double weight)
    {
        BezierCurve2D copy = new BezierCurve2D(this);
        copy.SetWeight(index, weight);
        return copy;
    }

    /// <summary>A copy evaluated through theDesc, which is copied (so later changes to theDesc don't affect it).</summary>
    /// <exception cref="ProgramErrorException">The descriptor is invalid.</exception>
    internal BezierCurve2D WithEvalRepresentation(RepCurveDesc2D.Base? desc)
    {
        BezierCurve2D copy = new BezierCurve2D(this);
        RepUtils2D.ValidateCurveDesc(desc, this);
        copy.SetEvalRepresentation(RepUtils2D.CloneCurveDesc(desc));
        return copy;
    }

    /// <summary>A copy without an evaluation representation.</summary>
    internal BezierCurve2D WithoutEvalRepresentation()
    {
        BezierCurve2D copy = new BezierCurve2D(this);
        copy.ClearEvalRepresentation();
        return copy;
    }
}
