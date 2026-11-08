// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_BSplineCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_BSplineCurve.cxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_BSplineCurve_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Evaluation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine;
using static Kerf.Engine.Geometry.BSplKnotDistribution;
using static Kerf.Geometry.Continuity;

namespace Kerf.Geometry;

/// <summary>
/// A B-spline curve in the plane: rational or not, periodic or not, of degree 1 to MaxDegree(), defined by its
/// poles, (weights,) knots and their multiplicities. For a non-periodic curve the number of poles is the sum of
/// the multiplicities minus Degree - 1; for a periodic one, the sum of the multiplicities except the last.
/// </summary>
/// <remarks>
/// <para>
/// OCCT returns its pole, weight and knot arrays by const reference. The port returns them to callers as
/// <see cref="IReadOnlyList{T}"/> copies (<see cref="Poles"/>, <see cref="Knots"/>, ...), and gives ported code
/// the arrays themselves through internal <c>Internal...()</c> accessors, which must not be modified.
/// The arrays are indexed from 1.
/// </para>
/// <para>
/// OCCT's Geom2d_BSplineCurve keeps an attached evaluation representation through every modification, where
/// Geom_BSplineCurve drops it (occt-issues.md); the port does the same as Geom2d.
/// </para>
/// </remarks>
public sealed partial class BSplineCurve2D : BoundedCurve2D
{
    private NCollection_Array1<Point2D> _poles = new NCollection_Array1<Point2D>();
    private NCollection_Array1<double> _weights = new NCollection_Array1<double>();
    private NCollection_Array1<double> _knots = new NCollection_Array1<double>();
    private NCollection_Array1<double> _flatKnots = new NCollection_Array1<double>();
    private NCollection_Array1<int> _mults = new NCollection_Array1<int>();
    private RepCurveDesc2D.Base? _evalRep;
    private int _deg = 0;
    private bool _periodic = false;
    private bool _rational = false;
    private BSplKnotDistribution _knotSet = NonUniform;
    private Continuity _smooth = C0;
    private double _maxDerivInv = 0.0;
    private bool _maxDerivInvOk = false;
    private ArcBox? _arcBoxes;

    /// <summary>
    /// The box of the poles of this curve segmented to [First, Last], before it is enlarged, and the boxes kept before
    /// it.
    /// </summary>
    internal sealed record ArcBox(double First, double Last, Box2d Box, ArcBox? Older, int Count);

    /// <summary>The most arcs whose boxes a frozen curve keeps (<see cref="KeepArcBox"/>).</summary>
    private const int MaxArcBoxes = 64;

    /// <summary>
    /// The box kept for the arc [<paramref name="first"/>, <paramref name="last"/>] of this frozen curve, or null.
    /// </summary>
    /// <remarks>
    /// Kerf: bounding an arc of a B-spline segments a copy of it (<c>BSplineCurve2dBounds.Box</c>), and a Boolean of
    /// many faces bounds the same pcurve's arc again and again: 72,314 segmentations of 6,864 arcs in bug25232_9. A
    /// frozen curve can't change, so the poles a segmentation gives, and their box, are always the same: the box is
    /// kept with the curve. The kept boxes are a list whose nodes never change, so a thread reading it sees whole ones;
    /// two threads keeping a box at once may lose one, which is only a box made again.
    /// </remarks>
    internal Box2d? KeptArcBox(double first, double last)
    {
        for (ArcBox? a = Volatile.Read(ref _arcBoxes); a is not null; a = a.Older)
        {
            // the very bits, so that -0.0 isn't taken for 0.0
            if (BitConverter.DoubleToInt64Bits(a.First) == BitConverter.DoubleToInt64Bits(first)
                && BitConverter.DoubleToInt64Bits(a.Last) == BitConverter.DoubleToInt64Bits(last))
            {
                return a.Box;
            }
        }
        return null;
    }

    /// <summary>Keeps the box of an arc of this frozen curve, up to <see cref="MaxArcBoxes"/> of them.</summary>
    internal void KeepArcBox(double first, double last, in Box2d box)
    {
        if (!IsFrozen)
        {
            return;
        }
        ArcBox? kept = Volatile.Read(ref _arcBoxes);
        int count = kept is null ? 0 : kept.Count;
        if (count < MaxArcBoxes)
        {
            Volatile.Write(ref _arcBoxes, new ArcBox(first, last, box, kept, count + 1));
        }
    }

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

    private static void CheckCurveData(NCollection_Array1<Point2D> cPoles,
                                       NCollection_Array1<double> cKnots,
                                       NCollection_Array1<int> cMults,
                                       int degree,
                                       bool periodic)
    {
        if (degree < 1 || degree > BSplineCurve2D.MaxDegree)
        {
            throw Refusal.Argument(new ConstructionErrorException("BSpline curve: invalid degree"));
        }

        if (cPoles.Length() < 2)
        {
            throw Refusal.Argument(new ConstructionErrorException("BSpline curve: at least 2 poles required"));
        }

        if (cKnots.Length() != cMults.Length())
        {
            throw Refusal.Argument(new ConstructionErrorException("BSpline curve: Knot and Mult array size mismatch"));
        }

        for (int i = cKnots.Lower(); i < cKnots.Upper(); i++)
        {
            if (cKnots[i + 1] - cKnots[i] <= Epsilon(Math.Abs(cKnots[i])))
            {
                throw Refusal.Argument(new ConstructionErrorException("BSpline curve: Knots interval values too close"));
            }
        }

        if (cPoles.Length() != global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(degree, periodic, cMults))
        {
            throw Refusal.Argument(new ConstructionErrorException("BSpline curve: # Poles and degree mismatch"));
        }
    }

    // Check rationality of an array of weights
    // Frozen, its knots and multiplicities can't change, and most splines have the same ones (SharedArrays)
    private protected override void OnFreezing()
    {
        _weights = SharedArrays<double>.Shared(_weights, isFixed: !_rational);
        _knots = SharedArrays<double>.Shared(_knots);
        _flatKnots = SharedArrays<double>.Shared(_flatKnots);
        _mults = SharedArrays<int>.Shared(_mults);
    }

    private static bool Rational(NCollection_Array1<double> weights)
    {
        for (int i = weights.Lower(); i < weights.Upper(); i++)
        {
            if (Math.Abs(weights[i] - weights[i + 1]) > Constants.Resolution())
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>A new, unfrozen curve equal to this one.</summary>
    public sealed override Geometry2D Copy() => new BSplineCurve2D(this);

    /// <summary>A copy of theOther (its evaluation representation deep-copied), not frozen.</summary>
    public BSplineCurve2D(BSplineCurve2D other)
    {
        _poles = new NCollection_Array1<Point2D>(other._poles);
        _weights = other._rational ? new NCollection_Array1<double>(other._weights)
                                        : global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(other._poles.Length());
        _knots = new NCollection_Array1<double>(other._knots);
        _flatKnots = new NCollection_Array1<double>(other._flatKnots);
        _mults = new NCollection_Array1<int>(other._mults);
        _evalRep = RepUtils2D.CloneCurveDesc(other._evalRep);
        _deg = other._deg;
        _periodic = other._periodic;
        _rational = other._rational;
        _knotSet = other._knotSet;
        _smooth = other._smooth;
        _maxDerivInv = other._maxDerivInv;
        _maxDerivInvOk = false;
    }

    /// <summary>A non-rational B-spline curve.</summary>
    /// <exception cref="ConstructionErrorException">
    /// Degree outside [1, MaxDegree()], fewer than 2 poles, knots and multiplicities of different lengths, knots
    /// not increasing (by more than Epsilon), or a pole count inconsistent with the degree and multiplicities.
    /// </exception>
    internal BSplineCurve2D(NCollection_Array1<Point2D> poles,
                               NCollection_Array1<double> knots,
                               NCollection_Array1<int> multiplicities,
                               int degree,
                               bool periodic = false)
    {
        _rational = false;
        _maxDerivInv = 0.0;
        _maxDerivInvOk = false;
        _deg = degree;
        _periodic = periodic;

        // check
        CheckCurveData(poles, knots, multiplicities, degree, periodic);

        // copy arrays
        _poles.Resize(1, poles.Length(), false);
        _poles.CopyValues(poles);
        _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(poles.Length());

        _knots.Resize(1, knots.Length(), false);
        _knots.CopyValues(knots);

        _mults.Resize(1, multiplicities.Length(), false);
        _mults.CopyValues(multiplicities);

        UpdateKnots();
    }

    /// <summary>A rational B-spline curve (non-rational if all the weights are equal).</summary>
    /// <exception cref="ConstructionErrorException">
    /// As the non-rational constructor, or weights of a different length than the poles, or a weight not above
    /// gp::Resolution().
    /// </exception>
    internal BSplineCurve2D(NCollection_Array1<Point2D> poles,
                               NCollection_Array1<double> weights,
                               NCollection_Array1<double> knots,
                               NCollection_Array1<int> multiplicities,
                               int degree,
                               bool periodic = false)
    {
        _rational = true;
        _maxDerivInv = 0.0;
        _maxDerivInvOk = false;
        _deg = degree;
        _periodic = periodic;

        // check
        CheckCurveData(poles, knots, multiplicities, degree, periodic);

        if (weights.Length() != poles.Length())
        {
            throw Refusal.Argument(new ConstructionErrorException("Geom2d_BSplineCurve: Weights and Poles array size mismatch"));
        }

        int i;
        for (i = weights.Lower(); i <= weights.Upper(); i++)
        {
            if (weights[i] <= Constants.Resolution())
            {
                throw Refusal.Argument(new ConstructionErrorException("Geom2d_BSplineCurve: Weights values too small"));
            }
        }

        // check really rational
        _rational = Rational(weights);

        // copy arrays
        _poles.Resize(1, poles.Length(), false);
        _poles.CopyValues(poles);
        if (_rational)
        {
            _weights.Resize(1, weights.Length(), false);
            _weights.CopyValues(weights);
        }
        else
        {
            _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(poles.Length());
        }

        _knots.Resize(1, knots.Length(), false);
        _knots.CopyValues(knots);

        _mults.Resize(1, multiplicities.Length(), false);
        _mults.CopyValues(multiplicities);

        UpdateKnots();
    }

    /// <summary>The maximum degree of a B-spline curve: 25.</summary>
    public static int MaxDegree => global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree();

    /// <summary>Raises the degree to Degree, keeping the shape of the curve.</summary>
    /// <exception cref="ConstructionErrorException">Degree is below the current degree or above MaxDegree().</exception>
    internal void IncreaseDegree(int degree)
    {
        CheckMutable();
        if (degree == _deg)
        {
            return;
        }

        if (degree < _deg || degree > BSplineCurve2D.MaxDegree)
        {
            throw Refusal.Argument(new ConstructionErrorException("BSpline curve: IncreaseDegree: bad degree value"));
        }

        int fromK1 = FirstUKnotIndex();
        int toK2 = LastUKnotIndex();

        int step = degree - _deg;

        // OCCT: counts only the knots of the parameter range; on a curve not clamped at its ends the new pole count
        // doesn't match the new multiplicities, or BSplCLib::IncreaseDegree writes past it (occt-issues.md).
        NCollection_Array1<Point2D> npoles = new NCollection_Array1<Point2D>(1, _poles.Length() + step * (toK2 - fromK1));

        int nbknots = global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegreeCountKnots(_deg, degree, _periodic, _mults);

        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);

        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);

        NCollection_Array1<double> nweights = new NCollection_Array1<double>();
        if (IsRational)
        {
            nweights.Resize(1, npoles.Upper(), false);
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(_deg,
                                degree,
                                _periodic,
                                _poles,
                                InternalWeights(),
                                _knots,
                                _mults,
                                npoles,
                                _rational ? nweights : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                nknots,
                                nmults);

        _deg = degree;
        _poles = npoles;
        if (IsRational)
        {
            _weights = nweights;
        }
        else
        {
            _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(_poles.Length());
        }
        _knots = nknots;
        _mults = nmults;
        UpdateKnots();
    }

    /// <summary>Raises the multiplicity of the knot of index Index to M (does nothing if it is already M or more).</summary>
    /// <exception cref="OutOfRangeException">Index is outside the knot range.</exception>
    internal void IncreaseMultiplicity(int index, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = _knots.Value(index);
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M - _mults.Value(index);
        InsertKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>Raises the multiplicities of the knots of indexes I1 to I2 to M.</summary>
    internal void IncreaseMultiplicity(int i1, int i2, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(_knots, i1, i1, i2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(i1, i2);
        int i;
        for (i = i1; i <= i2; i++)
        {
            m[i] = M - _mults.Value(i);
        }
        InsertKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>Increments the multiplicities of the knots of indexes I1 to I2 by M.</summary>
    internal void IncrementMultiplicity(int i1, int i2, int step)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(_knots, i1, i1, i2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(i1, i2);
        m.Init(step);
        InsertKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>
    /// Inserts the knot U with the multiplicity M, or raises the multiplicity of an existing knot within
    /// ParametricTolerance of U to M.
    /// </summary>
    internal void InsertKnot(double u, int M = 1, double parametricTolerance = 0.0)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = u;
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M;
        InsertKnots(k, m, parametricTolerance);
    }

    /// <summary>
    /// Inserts the knots Knots with the multiplicities Mults. A knot within ParametricTolerance of an existing
    /// knot raises its multiplicity: to the given one if Add is false, by the given one if Add is true.
    /// </summary>
    /// <exception cref="ConstructionErrorException">The insertion is not possible (BSplCLib::PrepareInsertKnots).</exception>
    internal void InsertKnots(NCollection_Array1<double> knots,
                              NCollection_Array1<int> mults,
                              double parametricTolerance = 0.0,
                              bool add = false)
    {
        CheckMutable();
        // Check and compute new sizes
        int nbpoles, nbknots;

        if (!global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareInsertKnots(_deg,
                                         _periodic,
                                         _knots,
                                         _mults,
                                         knots,
                                         mults,
                                         out nbpoles,
                                         out nbknots,
                                         parametricTolerance,
                                         add))
        {
            throw Refusal.Argument(new ConstructionErrorException("Geom2d_BSplineCurve::InsertKnots"));
        }

        if (nbpoles == _poles.Length())
        {
            return;
        }

        NCollection_Array1<Point2D> npoles = new NCollection_Array1<Point2D>(1, nbpoles);
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);
        NCollection_Array1<double> nweights = new NCollection_Array1<double>();

        if (_rational)
        {
            nweights.Resize(1, nbpoles, false);
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(_deg,
                             _periodic,
                             _poles,
                             InternalWeights(),
                             _knots,
                             _mults,
                             knots,
                             mults,
                             npoles,
                             _rational ? nweights : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                             nknots,
                             nmults,
                             parametricTolerance,
                             add);

        if (_rational)
        {
            _weights = nweights;
        }
        else
        {
            _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(npoles.Length());
        }
        _poles = npoles;
        _knots = nknots;
        _mults = nmults;
        UpdateKnots();
    }

    /// <summary>
    /// Lowers the multiplicity of the knot of index Index to M (0 removes it) if the curve stays within
    /// Tolerance of itself; returns false, leaving the curve unchanged, if not.
    /// </summary>
    /// <exception cref="OutOfRangeException">Index is outside [FirstUKnotIndex(), LastUKnotIndex()].</exception>
    internal bool RemoveKnot(int index, int m, double tolerance)
    {
        CheckMutable();
        if (m < 0)
        {
            return true;
        }

        int i1 = FirstUKnotIndex();
        int i2 = LastUKnotIndex();

        if (index < i1 || index > i2)
        {
            throw Refusal.Range(new OutOfRangeException("BSpline curve: RemoveKnot: index out of range"));
        }

        NCollection_Array1<Point2D> oldpoles = _poles;

        int step = _mults.Value(index) - m;
        if (step <= 0)
        {
            return true;
        }

        NCollection_Array1<Point2D> npoles = new NCollection_Array1<Point2D>(1, oldpoles.Length() - step);

        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, _knots.Length() - (m == 0 ? 1 : 0));
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, _mults.Length() - (m == 0 ? 1 : 0));

        NCollection_Array1<double> nweights = new NCollection_Array1<double>();
        if (IsRational)
        {
            nweights.Resize(1, npoles.Length(), false);
        }

        if (!global::Kerf.Engine.Geometry.Splines.BSplCLib.RemoveKnot(index,
                                 m,
                                 _deg,
                                 _periodic,
                                 _poles,
                                 InternalWeights(),
                                 _knots,
                                 _mults,
                                 npoles,
                                 _rational ? nweights : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                 nknots,
                                 nmults,
                                 tolerance))
        {
            return false;
        }

        if (IsRational)
        {
            _weights = nweights;
        }
        else
        {
            _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(npoles.Length());
        }
        _poles = npoles;
        _knots = nknots;
        _mults = nmults;

        UpdateKnots();
        _maxDerivInvOk = false;
        return true;
    }

    /// <summary>
    /// Inserts the pole P (weight Weight) after the pole of index Index, adding a knot at the end of a uniform
    /// or quasi-uniform knot sequence.
    /// </summary>
    /// <exception cref="OutOfRangeException">Index is outside [0, NbPoles()].</exception>
    /// <exception cref="ConstructionErrorException">
    /// Weight is not above gp::Resolution(), or the knots are non-uniform or piecewise Bezier.
    /// </exception>
    internal void InsertPoleAfter(int index, in Point2D p, double weight = 1.0)
    {
        CheckMutable();
        if (index < 0 || index > _poles.Length())
        {
            throw Refusal.Range(new OutOfRangeException("BSpline curve: InsertPoleAfter: Index and #pole mismatch"));
        }

        if (weight <= Constants.Resolution())
        {
            throw Refusal.Argument(new ConstructionErrorException("BSpline curve: InsertPoleAfter: Weight too small"));
        }

        if (_knotSet == NonUniform || _knotSet == PiecewiseBezier)
        {
            throw Refusal.Argument(new ConstructionErrorException("BSpline curve: InsertPoleAfter: bad knotSet type"));
        }

        NCollection_Array1<double> cknots = _knots;
        int nbknots = cknots.Length();

        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots + 1);

        int i;
        for (i = 1; i <= nbknots; i++)
        {
            nknots[i] = cknots[i];
        }

        nknots[nbknots + 1] = 2 * nknots[nbknots] - nknots[nbknots - 1];

        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots + 1);
        NCollection_Array1<int> cmults = _mults;

        for (i = 2; i <= nbknots; i++)
        {
            nmults[i] = 1;
        }
        nmults[1] = cmults[1];
        nmults[nbknots + 1] = cmults[nbknots];

        NCollection_Array1<Point2D> cpoles = _poles;
        int nbpoles = cpoles.Length();
        NCollection_Array1<Point2D> npoles = new NCollection_Array1<Point2D>(1, nbpoles + 1);

        // insert the pole

        for (i = 1; i <= index; i++)
        {
            npoles[i] = cpoles[i];
        }

        npoles[index + 1] = p;

        for (i = index + 1; i <= nbpoles; i++)
        {
            npoles[i + 1] = cpoles[i];
        }

        // Insert the weight

        NCollection_Array1<double> nweights = new NCollection_Array1<double>();
        bool rat = IsRational || Math.Abs(weight - 1.0) > Constants.Resolution();

        if (rat)
        {
            nweights.Resize(1, nbpoles + 1, false);

            for (i = 1; i <= index; i++)
            {
                if (IsRational)
                {
                    nweights[i] = _weights.Value(i);
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
                    nweights[i + 1] = _weights.Value(i);
                }
                else
                {
                    nweights[i + 1] = 1.0;
                }
            }
        }

        _poles = npoles;
        if (rat)
        {
            _weights = nweights;
        }
        else
        {
            _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(_poles.Length());
        }
        _rational = rat;
        _knots = nknots;
        _mults = nmults;
        _maxDerivInvOk = false;
        UpdateKnots();
    }

    /// <summary>Inserts the pole P (weight Weight) before the pole of index Index.</summary>
    internal void InsertPoleBefore(int index, in Point2D p, double weight = 1.0)
    {
        CheckMutable();
        InsertPoleAfter(index - 1, p, weight);
    }

    /// <summary>Removes the pole of index Index, and the last knot (uniform or quasi-uniform knots only).</summary>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbPoles()].</exception>
    /// <exception cref="ConstructionErrorException">
    /// The curve has only 2 poles, or the knots are non-uniform or piecewise Bezier.
    /// </exception>
    internal void RemovePole(int index)
    {
        CheckMutable();
        if (index < 1 || index > _poles.Length())
        {
            throw Refusal.Range(new OutOfRangeException("BSpline curve: RemovePole: Index and #pole mismatch"));
        }

        if (_poles.Length() <= 2)
        {
            throw Refusal.Argument(new ConstructionErrorException("BSpline curve: RemovePole: #pole is already minimum"));
        }

        if (_knotSet == NonUniform || _knotSet == PiecewiseBezier)
        {
            throw Refusal.Argument(new ConstructionErrorException("BSpline curve: RemovePole: bad knotSet type"));
        }

        int i;
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, _knots.Length() - 1);

        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, _mults.Length() - 1);

        for (i = 1; i < nknots.Length(); i++)
        {
            nknots[i] = _knots.Value(i);
            nmults[i] = 1;
        }
        nmults[1] = _mults.Value(1);
        nknots[nknots.Upper()] = _knots.Value(_knots.Upper());
        nmults[nmults.Upper()] = _mults.Value(_mults.Upper());

        NCollection_Array1<Point2D> npoles = new NCollection_Array1<Point2D>(1, _poles.Upper() - 1);

        for (i = 1; i < index; i++)
        {
            npoles[i] = _poles.Value(i);
        }
        for (i = index; i <= npoles.Length(); i++)
        {
            npoles[i] = _poles.Value(i + 1);
        }

        NCollection_Array1<double> nweights = new NCollection_Array1<double>();
        if (IsRational)
        {
            nweights.Resize(1, npoles.Length(), false);
            for (i = 1; i < index; i++)
            {
                nweights[i] = _weights.Value(i);
            }
            for (i = index; i <= nweights.Length(); i++)
            {
                nweights[i] = _weights.Value(i + 1);
            }
        }

        _poles = npoles;
        if (IsRational)
        {
            _weights = nweights;
            _rational = Rational(_weights);
            if (!_rational)
            {
                _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(_poles.Length());
            }
        }
        else
        {
            _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(_poles.Length());
        }
        _knots = nknots;
        _mults = nmults;
        UpdateKnots();
    }

    /// <summary>Reverses the curve: knots, multiplicities, poles and weights; the parameter range is kept.</summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(_knots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(_mults);
        int last;
        if (_periodic)
        {
            last = _flatKnots.Upper() - _deg - 1;
        }
        else
        {
            last = _poles.Upper();
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(_poles, last);
        if (_rational)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(_weights, last);
        }
        UpdateKnots();
    }

    /// <summary>FirstParameter() + LastParameter() - U.</summary>
    public sealed override double ReversedParameter(double u) => (FirstParameter + LastParameter - u);

    // Precision::PConfusion() as a constant, for the default argument of Segment.
    private const double Precision_PConfusion = 1.0e-7 * 0.01;

    /// <summary>
    /// Keeps the part of the curve between U1 and U2 (knots within theTolerance are merged). A periodic curve
    /// becomes non-periodic; U2 - U1 may not exceed its period.
    /// </summary>
    /// <exception cref="DomainErrorException">U2 &lt; U1, or U2 - U1 exceeds the period of a periodic curve.</exception>
    internal void Segment(double aU1, double aU2, double tolerance = Precision_PConfusion)
    {
        CheckMutable();
        if (aU2 < aU1)
        {
            throw Refusal.Argument(new DomainErrorException("Geom2d_BSplineCurve::Segment"));
        }

        double newU1, newU2;
        double u, du = 0, ddu = 0;
        int i, k, index;
        bool wasPeriodic = _periodic;
        double u1 = aU1, u2 = aU2;

        NCollection_Array1<double> knots = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int> mults = new NCollection_Array1<int>(1, 2);

        // define param distance to keep (eap, Apr 18 2002, occ311)
        if (_periodic)
        {
            double period = LastParameter - FirstParameter;
            du = u2 - u1;
            if (du - period > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                throw Refusal.Argument(new DomainErrorException("Geom2d_BSplineCurve::Segment"));
            }
            if (du > period)
            {
                du = period;
            }
            ddu = du;
        }

        index = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg,
                                 _knots,
                                 _mults,
                                 u1,
                                 _periodic,
                                 _knots.Lower(),
                                 _knots.Upper(),
                                 out index,
                                 out newU1);
        index = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg,
                                 _knots,
                                 _mults,
                                 u2,
                                 _periodic,
                                 _knots.Lower(),
                                 _knots.Upper(),
                                 out index,
                                 out newU2);

        //-- DBB
        double nu2 = newU2;
        //-- DBB

        double absUMax = Max(Math.Abs(newU1), Math.Abs(newU2));
        absUMax = Max(absUMax, Max(Math.Abs(FirstParameter), Math.Abs(LastParameter)));

        double eps = Max(Epsilon(absUMax), tolerance);

        knots[1] = Min(newU1, newU2);
        knots[2] = Max(newU1, newU2);
        mults[1] = mults[2] = _deg;
        InsertKnots(knots, mults, eps, false);

        if (_periodic)
        { // set the origine at NewU1
            index = 0;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg,
                                     _knots,
                                     _mults,
                                     u1,
                                     _periodic,
                                     _knots.Lower(),
                                     _knots.Upper(),
                                     out index,
                                     out u);
            if (Math.Abs(_knots.Value(index + 1) - u) <= eps)
            {
                index++;
            }
            SetOrigin(index);
            SetNotPeriodic();
            newU2 = newU1 + du;
        }

        // compute index1 and index2 to set the new knots and mults
        int index1 = 0, index2 = 0;
        int fromU1 = _knots.Lower();
        int toU2 = _knots.Upper();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _knots, _mults, newU1, _periodic, fromU1, toU2, out index1, out u);
        if (Math.Abs(_knots.Value(index1 + 1) - u) <= eps)
        {
            index1++;
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _knots, _mults, newU2, _periodic, fromU1, toU2, out index2, out u);
        if (Math.Abs(_knots.Value(index2 + 1) - u) <= eps || index2 == index1)
        {
            index2++;
        }

        int nbknots = index2 - index1 + 1;

        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);

        // to restore changed U1
        if (du > 0)
        { // if was periodic
            du = newU1 - u1;
        }
        //
        k = 1;
        //
        for (i = index1; i <= index2; i++)
        {
            nknots.SetValue(k, _knots.Value(i) - du);
            nmults.SetValue(k, _mults.Value(i));
            k++;
        }
        nmults.SetValue(1, _deg + 1);
        nmults.SetValue(nbknots, _deg + 1);

        // compute index1 and index2 to set the new poles and weights
        int pindex1 = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(_deg, index1, _periodic, _mults);
        int pindex2 = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(_deg, index2, _periodic, _mults);

        pindex1++;
        pindex2 = Min(pindex2 + 1, _poles.Length());

        int nbpoles = pindex2 - pindex1 + 1;

        NCollection_Array1<double> nweights = new NCollection_Array1<double>();
        NCollection_Array1<Point2D> npoles = new NCollection_Array1<Point2D>(1, nbpoles);

        k = 1;
        if (_rational)
        {
            nweights.Resize(1, nbpoles, false);
            for (i = pindex1; i <= pindex2; i++)
            {
                npoles.SetValue(k, _poles.Value(i));
                nweights.SetValue(k, _weights.Value(i));
                k++;
            }
        }
        else
        {
            for (i = pindex1; i <= pindex2; i++)
            {
                npoles.SetValue(k, _poles.Value(i));
                k++;
            }
        }

        //-- DBB
        if (wasPeriodic)
        {
            nknots.ChangeValue(nknots.Lower()) = u1;
            if (nu2 < u2)
            {
                nknots.ChangeValue(nknots.Upper()) = u1 + ddu;
            }
        }
        //-- DBB

        _knots = nknots;
        _mults = nmults;
        _poles = npoles;
        if (_rational)
        {
            _weights = nweights;
        }
        else
        {
            _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(_poles.Length());
        }

        _maxDerivInvOk = false;
        UpdateKnots();
    }

    /// <summary>Changes the value of the knot of index Index; the knots must stay increasing.</summary>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbKnots()].</exception>
    /// <exception cref="ConstructionErrorException">K is not strictly between the neighbouring knots.</exception>
    internal void SetKnot(int index, double k)
    {
        CheckMutable();
        if (index < 1 || index > _knots.Length())
        {
            throw Refusal.Range(new OutOfRangeException("BSpline curve: SetKnot: Index and #knots mismatch"));
        }
        double dk = Math.Abs(Epsilon(k));
        if (index == 1)
        {
            if (k >= _knots.Value(2) - dk)
            {
                throw Refusal.Argument(new ConstructionErrorException("BSpline curve: SetKnot: K out of range"));
            }
        }
        else if (index == _knots.Length())
        {
            if (k <= _knots.Value(_knots.Length() - 1) + dk)
            {
                throw Refusal.Argument(new ConstructionErrorException("BSpline curve: SetKnot: K out of range"));
            }
        }
        else
        {
            if (k <= _knots.Value(index - 1) + dk || k >= _knots.Value(index + 1) - dk)
            {
                throw Refusal.Argument(new ConstructionErrorException("BSpline curve: SetKnot: K out of range"));
            }
        }
        if (k != _knots.Value(index))
        {
            _knots.SetValue(index, k);
            _maxDerivInvOk = false;
            UpdateKnots();
        }
    }

    /// <summary>Changes all the knot values.</summary>
    /// <exception cref="ConstructionErrorException">The knots are not increasing or don't match the multiplicities.</exception>
    internal void SetKnots(NCollection_Array1<double> k)
    {
        CheckMutable();
        CheckCurveData(_poles, k, _mults, _deg, _periodic);
        _knots.CopyValues(k);
        _maxDerivInvOk = false;
        UpdateKnots();
    }

    /// <summary>Raises the multiplicity of the knot of index Index to M, then changes its value to K.</summary>
    internal void SetKnot(int index, double k, int m)
    {
        CheckMutable();
        IncreaseMultiplicity(index, m);
        SetKnot(index, k);
    }

    /// <summary>
    /// Brings the parameter of a periodic curve into [FirstParameter(), LastParameter()] by adding or
    /// subtracting periods; does nothing on a non-periodic curve.
    /// </summary>
    internal void PeriodicNormalization(ref double parameter)
    {
        double period;

        if (_periodic)
        {
            period = _flatKnots.Value(_flatKnots.Upper() - _deg) - _flatKnots.Value(_deg + 1);
            for (int aStep = 0; parameter > _flatKnots.Value(_flatKnots.Upper() - _deg); StepLimit.Next(ref aStep))
            {
                parameter -= period;
            }
            for (int aStep = 0; parameter < _flatKnots.Value((_deg + 1)); StepLimit.Next(ref aStep))
            {
                parameter += period;
            }
        }
    }

    /// <summary>Makes the curve periodic, keeping the knots between FirstUKnotIndex() and LastUKnotIndex().</summary>
    internal void SetPeriodic()
    {
        CheckMutable();
        int first = FirstUKnotIndex();
        int last = LastUKnotIndex();

        NCollection_Array1<double> cknots = new NCollection_Array1<double>(1, last - first + 1);
        for (int k = first; k <= last; k++)
        {
            cknots[k - first + 1] = _knots[k];
        }
        _knots = cknots;

        NCollection_Array1<int> cmults = new NCollection_Array1<int>(1, last - first + 1);
        for (int k = first; k <= last; k++)
        {
            cmults[k - first + 1] = _mults[k];
        }
        cmults[1] = cmults[cmults.Upper()] = Min(_deg, Max(cmults[1], cmults[cmults.Upper()]));
        _mults = cmults;

        // compute new number of poles;
        int nbp = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(_deg, true, _mults);

        _poles.Resize(1, nbp, true);
        if (_rational)
        {
            _weights.Resize(1, nbp, true);
        }
        else
        {
            _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(nbp);
        }

        _periodic = true;

        _maxDerivInvOk = false;
        UpdateKnots();
    }

    /// <summary>Sets the origin of a periodic curve at the knot of index Index.</summary>
    /// <exception cref="NoSuchObjectException">The curve is not periodic.</exception>
    /// <exception cref="DomainErrorException">Index is outside [FirstUKnotIndex(), LastUKnotIndex()].</exception>
    internal void SetOrigin(int Index)
    {
        CheckMutable();
        if (!_periodic)
        {
            throw Refusal.State(new NoSuchObjectException("Geom2d_BSplineCurve::SetOrigin"));
        }

        int i, k;
        int first = FirstUKnotIndex();
        int last = LastUKnotIndex();

        if ((Index < first) || (Index > last))
        {
            throw Refusal.Argument(new DomainErrorException("Geom2d_BSplineCurve::SetOrigin"));
        }

        int nbknots = _knots.Length();
        int nbpoles = _poles.Length();

        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);

        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);

        // set the knots and mults
        double period = _knots.Value(last) - _knots.Value(first);
        k = 1;
        for (i = Index; i <= last; i++)
        {
            nknots[k] = _knots.Value(i);
            nmults[k] = _mults.Value(i);
            k++;
        }
        for (i = first + 1; i <= Index; i++)
        {
            nknots[k] = _knots.Value(i) + period;
            nmults[k] = _mults.Value(i);
            k++;
        }

        int index = 1;
        for (i = first + 1; i <= Index; i++)
        {
            index += _mults.Value(i);
        }

        // set the poles and weights
        NCollection_Array1<Point2D> npoles = new NCollection_Array1<Point2D>(1, nbpoles);
        NCollection_Array1<double> nweights = new NCollection_Array1<double>();
        first = _poles.Lower();
        last = _poles.Upper();
        if (_rational)
        {
            nweights.Resize(1, nbpoles, false);
            k = 1;
            for (i = index; i <= last; i++)
            {
                npoles[k] = _poles.Value(i);
                nweights[k] = _weights.Value(i);
                k++;
            }
            for (i = first; i < index; i++)
            {
                npoles[k] = _poles.Value(i);
                nweights[k] = _weights.Value(i);
                k++;
            }
        }
        else
        {
            k = 1;
            for (i = index; i <= last; i++)
            {
                npoles[k] = _poles.Value(i);
                k++;
            }
            for (i = first; i < index; i++)
            {
                npoles[k] = _poles.Value(i);
                k++;
            }
        }

        _poles = npoles;
        _knots = nknots;
        _mults = nmults;
        if (_rational)
        {
            _weights = nweights;
        }
        else
        {
            _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(nbpoles);
        }
        _maxDerivInvOk = false;
        UpdateKnots();
    }

    /// <summary>Makes a periodic curve non-periodic (with the same shape); does nothing on a non-periodic curve.</summary>
    internal void SetNotPeriodic()
    {
        CheckMutable();
        if (_periodic)
        {
            int nbKnots, nbPoles;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareUnperiodize(_deg, _mults, out nbKnots, out nbPoles);

            NCollection_Array1<Point2D> npoles = new NCollection_Array1<Point2D>(1, nbPoles);

            NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbKnots);

            NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbKnots);

            NCollection_Array1<double> nweights = new NCollection_Array1<double>();

            if (IsRational)
            {
                nweights.Resize(1, nbPoles, false);
            }

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Unperiodize(_deg,
                                 _mults,
                                 _knots,
                                 _poles,
                                 InternalWeights(),
                                 nmults,
                                 nknots,
                                 npoles,
                                 _rational ? nweights : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());

            _poles = npoles;
            if (IsRational)
            {
                _weights = nweights;
            }
            else
            {
                _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(_poles.Length());
            }
            _mults = nmults;
            _knots = nknots;
            _periodic = false;
            _maxDerivInvOk = false;

            UpdateKnots();
        }
    }

    /// <summary>Replaces the pole of index Index by P.</summary>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbPoles()].</exception>
    internal void SetPole(int index, in Point2D p)
    {
        CheckMutable();
        if (index < 1 || index > _poles.Length())
        {
            throw Refusal.Range(new OutOfRangeException("BSpline curve: SetPole: index and #pole mismatch"));
        }
        _poles.SetValue(index, p);
        _maxDerivInvOk = false;
    }

    /// <summary>Replaces the pole of index Index by P, and its weight by W.</summary>
    internal void SetPole(int index, in Point2D p, double w)
    {
        CheckMutable();
        SetPole(index, p);
        SetWeight(index, w);
    }

    /// <summary>Changes the weight of the pole of index Index; the curve may become rational or non-rational.</summary>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbPoles()].</exception>
    /// <exception cref="ConstructionErrorException">W is not above gp::Resolution().</exception>
    internal void SetWeight(int index, double w)
    {
        CheckMutable();
        if (index < 1 || index > _poles.Length())
        {
            throw Refusal.Range(new OutOfRangeException("BSpline curve: SetWeight: Index and #pole mismatch"));
        }

        if (w <= Constants.Resolution())
        {
            throw Refusal.Argument(new ConstructionErrorException("BSpline curve: SetWeight: Weight too small"));
        }

        bool rat = IsRational || (Math.Abs(w - 1.0) > Constants.Resolution());

        if (rat)
        {
            // Becoming rational from non-rational: copy non-owning view to owned array.
            if (!IsRational)
            {
                _weights = new NCollection_Array1<double>(_weights);
            }

            _weights.SetValue(index, w);

            if (IsRational)
            {
                rat = Rational(_weights);
                if (!rat)
                {
                    _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(_poles.Length());
                }
            }

            _rational = rat;
        }

        _maxDerivInvOk = false;
    }

    /// <summary>
    /// Moves the point of parameter U to P by changing the poles of indexes Index1 to Index2.
    /// FirstModifiedPole and LastModifiedPole are the poles actually changed; FirstModifiedPole is 0 if the
    /// point couldn't be moved.
    /// </summary>
    /// <exception cref="OutOfRangeException">Index1 or Index2 is outside [1, NbPoles()], or Index1 &gt; Index2.</exception>
    internal void MovePoint(double u,
                            in Point2D p,
                            int index1,
                            int index2,
                            out int firstModifiedPole,
                            out int lastmodifiedPole)
    {
        CheckMutable();
        if (index1 < 1 || index1 > _poles.Length() || index2 < 1 || index2 > _poles.Length()
            || index1 > index2)
        {
            throw Refusal.Range(new OutOfRangeException("BSpline curve: MovePoint: Index and #pole mismatch"));
        }
        NCollection_Array1<Point2D> npoles = new NCollection_Array1<Point2D>(1, _poles.Length());
        Point2D p0;
        D0(u, out p0);
        Vector2D displ = new Vector2D(p0, p);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.MovePoint(u,
                           displ,
                           index1,
                           index2,
                           _deg,
                           _poles,
                           InternalWeights(),
                           _flatKnots,
                           out firstModifiedPole,
                           out lastmodifiedPole,
                           npoles);
        if (firstModifiedPole != 0)
        {
            _poles = npoles;
            _maxDerivInvOk = false;
        }
    }

    /// <summary>
    /// Moves the point of parameter U to P and its tangent to Tangent (a periodic curve becomes non-periodic).
    /// ErrorStatus is 0 on success; the curve is unchanged otherwise.
    /// </summary>
    internal void MovePointAndTangent(double u,
                                      in Point2D p,
                                      in Vector2D tangent,
                                      double tolerance,
                                      int startingCondition,
                                      int endingCondition,
                                      out int errorStatus)
    {
        CheckMutable();
        int ii;
        if (IsPeriodic)
        {
            //
            // for the time being do not deal with periodic curves
            //
            SetNotPeriodic();
        }
        NCollection_Array1<Point2D> new_poles = new NCollection_Array1<Point2D>(1, _poles.Length());
        Point2D p0;

        Vector2D delta_derivative;
        D1(u, out p0, out delta_derivative);
        Vector2D delta = new Vector2D(p0, p);
        for (ii = 1; ii <= 2; ii++)
        {
            delta_derivative = delta_derivative.WithCoord(ii, tangent.Coord(ii) - delta_derivative.Coord(ii));
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.MovePointAndTangent(u,
                                     delta,
                                     delta_derivative,
                                     tolerance,
                                     _deg,
                                     startingCondition,
                                     endingCondition,
                                     _poles,
                                     InternalWeights(),
                                     _flatKnots,
                                     new_poles,
                                     out errorStatus);
        if (errorStatus == 0)
        {
            _poles = new_poles;
            _maxDerivInvOk = false;
        }
    }

    /// <summary>Recomputes the knot distribution, flat knots and continuity after a change of knots.</summary>
    private void UpdateKnots()
    {
        _maxDerivInvOk = false;
        int maxKnotMult = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotAnalysis(_deg, _periodic, _knots, _mults, out _knotSet, out maxKnotMult);

        if (_knotSet == Uniform && !_periodic)
        {
            _flatKnots.Resize(_knots.Lower(), _knots.Upper(), false);
            _flatKnots.Assign(_knots);
        }
        else
        {
            _flatKnots.Resize(1, global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequenceLength(_mults, _deg, _periodic), false);

            global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(_knots, _mults, _deg, _periodic, _flatKnots);
        }

        if (maxKnotMult == 0)
        {
            _smooth = CN;
        }
        else
        {
            switch (_deg - maxKnotMult)
            {
                case 0:
                    _smooth = C0;
                    break;
                case 1:
                    _smooth = C1;
                    break;
                case 2:
                    _smooth = C2;
                    break;
                case 3:
                    _smooth = C3;
                    break;
                default:
                    _smooth = C3;
                    break;
            }
        }
    }

    // Geom2d_BSplineCurve_1.cxx

    /// <summary>True if the curve is CN (from the multiplicities of its interior knots).</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    internal sealed override bool IsCN(int n)
    {
        Refusal.RangeIf(n < 0, FailureKind.RangeError, "Geom2d_BSplineCurve::IsCN");

        switch (_smooth)
        {
            case CN:
                return true;
            case C0:
                return n <= 0;
            case G1:
                return n <= 0;
            case C1:
                return n <= 1;
            case G2:
                return n <= 1;
            case C2:
                return n <= 2;
            case C3:
                return n <= 3
                         ? true
                         : n <= _deg
                                  - global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxKnotMult(_mults, _mults.Lower() + 1, _mults.Upper() - 1);
            default:
                return false;
        }
    }

    /// <summary>
    /// True if the curve is G1 on [theTf, theTl]: at every interior knot of multiplicity Degree or more, the
    /// left and right tangents are non-null and within theAngTol radians (and, for a periodic curve whose range
    /// meets the bounds, at the start/end point too).
    /// </summary>
    internal bool IsG1(double tf, double tl, double angTol)
    {
        if (IsCN(1))
        {
            return true;
        }

        int start = FirstUKnotIndex() + 1, finish = LastUKnotIndex() - 1;
        int deg = Degree;
        for (int nKnot = start; nKnot <= finish; nKnot++)
        {
            double tpar = Knot(nKnot);

            if (tpar < tf)
            {
                continue;
            }
            if (tpar > tl)
            {
                break;
            }

            int mult = Multiplicity(nKnot);
            if (mult < deg)
            {
                continue;
            }

            Point2D p1, p2;
            Vector2D v1, v2;
            LocalD1(tpar, nKnot - 1, nKnot, out p1, out v1);
            LocalD1(tpar, nKnot, nKnot + 1, out p2, out v2);

            if ((v1.SquareMagnitude <= Constants.Resolution()) || v2.SquareMagnitude <= Constants.Resolution())
            {
                return false;
            }

            if (Math.Abs(v1.AngleTo(v2)) > angTol)
            {
                return false;
            }
        }

        if (!IsPeriodic)
        {
            return true;
        }

        double firstParam = FirstParameter, lastParam = LastParameter;

        if (((firstParam - tf) * (tl - firstParam) < 0.0)
            && ((lastParam - tf) * (tl - lastParam) < 0.0))
        {
            // Range [theTf, theTl] does not intersect curve boundaries
            return true;
        }

        // Curve is closed or periodic and range [theTf, theTl]
        // intersect curve boundary. Therefore, it is necessary to
        // check if curve is smooth in its first and last point.

        Point2D p;
        Vector2D v1b, v2b;
        D1(Knot(FirstUKnotIndex()), out p, out v1b);
        D1(Knot(LastUKnotIndex()), out p, out v2b);

        if ((v1b.SquareMagnitude <= Constants.Resolution()) || v2b.SquareMagnitude <= Constants.Resolution())
        {
            return false;
        }

        if (Math.Abs(v1b.AngleTo(v2b)) > angTol)
        {
            return false;
        }

        return true;
    }

    /// <summary>True if the start and end points are within Precision::Computational() (squared distance).</summary>
    public sealed override bool IsClosed => StartPoint.SquareDistanceTo(EndPoint) <= global::Kerf.Engine.Foundation.Precision.Computational();

    /// <summary>True if the curve is periodic.</summary>
    public sealed override bool IsPeriodic => _periodic;

    /// <summary>The global continuity, from the highest interior knot multiplicity (C3 at most, or CN).</summary>
    public sealed override Continuity Continuity => _smooth;

    /// <summary>The degree.</summary>
    public int Degree => _deg;

    internal sealed override Point2D EvalD0(double u)
    {
        Point2D evalRepResult;
        if (RepUtils2D.TryEvalCurveD0(_evalRep, u, out evalRepResult))
        {
            return evalRepResult;
        }
        Point2D p;
        int spanIndex = 0;
        double newU = u;
        PeriodicNormalization(ref newU);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _knots, _mults, u, _periodic, ref spanIndex, out newU);
        if (newU < _knots.Value(spanIndex))
        {
            spanIndex--;
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(newU, spanIndex, _deg, _periodic, _poles, InternalWeights(), _knots, _mults, out p);
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
        int spanIndex = 0;
        double newU = u;
        PeriodicNormalization(ref newU);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _knots, _mults, u, _periodic, ref spanIndex, out newU);
        if (newU < _knots.Value(spanIndex))
        {
            spanIndex--;
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(newU,
                    spanIndex,
                    _deg,
                    _periodic,
                    _poles,
                    InternalWeights(),
                    _knots,
                    _mults,
                    out result.Point,
                    out result.D1);
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
        int spanIndex = 0;
        double newU = u;
        PeriodicNormalization(ref newU);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _knots, _mults, u, _periodic, ref spanIndex, out newU);
        if (newU < _knots.Value(spanIndex))
        {
            spanIndex--;
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(newU,
                    spanIndex,
                    _deg,
                    _periodic,
                    _poles,
                    InternalWeights(),
                    _knots,
                    _mults,
                    out result.Point,
                    out result.D1,
                    out result.D2);
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
        int spanIndex = 0;
        double newU = u;
        PeriodicNormalization(ref newU);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _knots, _mults, u, _periodic, ref spanIndex, out newU);
        if (newU < _knots.Value(spanIndex))
        {
            spanIndex--;
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(newU,
                    spanIndex,
                    _deg,
                    _periodic,
                    _poles,
                    InternalWeights(),
                    _knots,
                    _mults,
                    out result.Point,
                    out result.D1,
                    out result.D2,
                    out result.D3);
        return result;
    }

    /// <exception cref="UndefinedDerivative2DException">N &lt; 1.</exception>
    internal sealed override Vector2D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw Refusal.State(new UndefinedDerivative2DException());
        }
        Vector2D evalRepResult;
        if (RepUtils2D.TryEvalCurveDN(_evalRep, u, n, out evalRepResult))
        {
            return evalRepResult;
        }
        Vector2D v;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(u, n, 0, _deg, _periodic, _poles, InternalWeights(), _flatKnots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out v);
        return v;
    }

    /// <summary>The last pole if the last knot has multiplicity Degree() + 1, the point at LastParameter() otherwise.</summary>
    public sealed override Point2D EndPoint
    {
        get
        {
            if (_mults.Value(_knots.Upper()) == _deg + 1)
            {
                return _poles.Value(_poles.Upper());
            }
            else
            {
                return Value(LastParameter);
            }
        }
    }

    /// <summary>The index of the knot giving FirstParameter() (1 for a periodic curve).</summary>
    internal int FirstUKnotIndex()
    {
        if (_periodic)
        {
            return 1;
        }
        else
        {
            return global::Kerf.Engine.Geometry.Splines.BSplCLib.FirstUKnotIndex(_deg, _mults);
        }
    }

    /// <summary>The flat knot of index Degree() + 1.</summary>
    public sealed override double FirstParameter => _flatKnots.Value(_deg + 1);

    /// <summary>The knot of index Index.</summary>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbKnots()].</exception>
    internal double Knot(int index)
    {
        Refusal.RangeIf(index < 1 || index > _knots.Length(), FailureKind.OutOfRange, "Geom2d_BSplineCurve::Knot");
        return _knots.Value(index);
    }

    /// <summary>The knot distribution: NonUniform, Uniform, QuasiUniform or PiecewiseBezier.</summary>
    internal BSplKnotDistribution KnotDistribution => _knotSet;

    /// <summary>Copies the knots of K's index range into K.</summary>
    /// <exception cref="DomainErrorException">K's range is outside [1, NbKnots()].</exception>
    internal void CopyKnotsTo(NCollection_Array1<double> k)
    {
        Refusal.ArgumentIf(k.Lower() < _knots.Lower() || k.Upper() > _knots.Upper(), FailureKind.DomainError, "Geom2d_BSplineCurve::Knots");
        for (int idx = k.Lower(); idx <= k.Upper(); idx++)
        {
            k[idx] = _knots.Value(idx);
        }
    }

    /// <summary>The knots (a copy; OCCT returns a reference to its array).</summary>
    public IReadOnlyList<double> Knots => _knots.ToImmutableArray();

    /// <summary>The knot array itself, indexed from 1, for ported code: don't modify it.</summary>
    internal NCollection_Array1<double> InternalKnots() => _knots;

    /// <summary>Copies the flat knots of K's index range into K.</summary>
    /// <exception cref="DomainErrorException">K's range is outside the flat knots' range.</exception>
    internal void CopyKnotSequenceTo(NCollection_Array1<double> k)
    {
        Refusal.ArgumentIf(k.Lower() < _flatKnots.Lower() || k.Upper() > _flatKnots.Upper(), FailureKind.DomainError, "Geom2d_BSplineCurve::KnotSequence");
        for (int idx = k.Lower(); idx <= k.Upper(); idx++)
        {
            k[idx] = _flatKnots.Value(idx);
        }
    }

    /// <summary>The flat knots, each knot repeated by its multiplicity (a copy).</summary>
    public IReadOnlyList<double> KnotSequence => _flatKnots.ToImmutableArray();

    /// <summary>The flat knot array itself, for ported code: don't modify it.</summary>
    internal NCollection_Array1<double> InternalKnotSequence() => _flatKnots;

    /// <summary>The index of the knot giving LastParameter() (NbKnots() for a periodic curve).</summary>
    internal int LastUKnotIndex()
    {
        if (_periodic)
        {
            return _knots.Length();
        }
        else
        {
            return global::Kerf.Engine.Geometry.Splines.BSplCLib.LastUKnotIndex(_deg, _mults);
        }
    }

    /// <summary>The flat knot of index NbFlatKnots - Degree().</summary>
    public sealed override double LastParameter => _flatKnots.Value(_flatKnots.Upper() - _deg);

    /// <summary>The point of parameter U, computed on the span between the knots FromK1 and ToK2.</summary>
    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    internal Point2D LocalValue(double u, int fromK1, int toK2)
    {
        Point2D p;
        LocalD0(u, fromK1, toK2, out p);
        return p;
    }

    /// <summary>The point of parameter U, computed on the span between the knots FromK1 and ToK2.</summary>
    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    internal void LocalD0(double U, int fromK1, int toK2, out Point2D p)
    {
        Refusal.ArgumentIf(fromK1 == toK2, FailureKind.DomainError, "Geom2d_BSplineCurve::LocalValue");

        double u = U;
        int index = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _flatKnots, U, _periodic, fromK1, toK2, out index, out u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_deg, index, _mults, _periodic);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u,
                    index,
                    _deg,
                    _periodic,
                    _poles,
                    InternalWeights(),
                    _flatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    out p);
    }

    /// <summary>The point and first derivative at U, computed on the span between the knots FromK1 and ToK2.</summary>
    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    internal void LocalD1(double U, int fromK1, int toK2, out Point2D p, out Vector2D v1)
    {
        Refusal.ArgumentIf(fromK1 == toK2, FailureKind.DomainError, "Geom2d_BSplineCurve::LocalD1");

        double u = U;
        int index = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _flatKnots, U, _periodic, fromK1, toK2, out index, out u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_deg, index, _mults, _periodic);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u,
                    index,
                    _deg,
                    _periodic,
                    _poles,
                    InternalWeights(),
                    _flatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    out p,
                    out v1);
    }

    /// <summary>The point and first two derivatives at U, computed on the span between the knots FromK1 and ToK2.</summary>
    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    internal void LocalD2(double U, int fromK1, int toK2, out Point2D p, out Vector2D v1, out Vector2D v2)
    {
        Refusal.ArgumentIf(fromK1 == toK2, FailureKind.DomainError, "Geom2d_BSplineCurve::LocalD2");

        double u = U;
        int index = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _flatKnots, U, _periodic, fromK1, toK2, out index, out u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_deg, index, _mults, _periodic);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u,
                    index,
                    _deg,
                    _periodic,
                    _poles,
                    InternalWeights(),
                    _flatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    out p,
                    out v1,
                    out v2);
    }

    /// <summary>The point and first three derivatives at U, computed on the span between the knots FromK1 and ToK2.</summary>
    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    internal void LocalD3(double U,
                        int fromK1,
                        int toK2,
                        out Point2D p,
                        out Vector2D v1,
                        out Vector2D v2,
                        out Vector2D v3)
    {
        Refusal.ArgumentIf(fromK1 == toK2, FailureKind.DomainError, "Geom2d_BSplineCurve::LocalD3");

        double u = U;
        int index = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _flatKnots, U, _periodic, fromK1, toK2, out index, out u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_deg, index, _mults, _periodic);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(u,
                    index,
                    _deg,
                    _periodic,
                    _poles,
                    InternalWeights(),
                    _flatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    out p,
                    out v1,
                    out v2,
                    out v3);
    }

    /// <summary>The Nth derivative at U, computed on the span between the knots FromK1 and ToK2.</summary>
    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    internal Vector2D LocalDN(double U, int fromK1, int toK2, int n)
    {
        // OCCT: the message says LocalD3 (cosmetic).
        Refusal.ArgumentIf(fromK1 == toK2, FailureKind.DomainError, "Geom2d_BSplineCurve::LocalD3");

        double u = U;
        int index = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _flatKnots, U, _periodic, fromK1, toK2, out index, out u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_deg, index, _mults, _periodic);

        Vector2D v;

        global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(u,
                    n,
                    index,
                    _deg,
                    _periodic,
                    _poles,
                    InternalWeights(),
                    _flatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    out v);
        return v;
    }

    /// <summary>The multiplicity of the knot of index Index.</summary>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbKnots()].</exception>
    internal int Multiplicity(int index)
    {
        Refusal.RangeIf(index < 1 || index > _mults.Length(), FailureKind.OutOfRange, "Geom2d_BSplineCurve::Multiplicity");
        return _mults.Value(index);
    }

    /// <summary>Copies the multiplicities into M.</summary>
    /// <exception cref="DimensionErrorException">M's length is not NbKnots().</exception>
    internal void CopyMultiplicitiesTo(NCollection_Array1<int> m)
    {
        Refusal.ArgumentIf(m.Length() != _mults.Length(), FailureKind.DimensionError, "Geom2d_BSplineCurve::Multiplicities");
        m.Assign(_mults);
    }

    /// <summary>The multiplicities (a copy).</summary>
    public IReadOnlyList<int> Multiplicities => _mults.ToImmutableArray();

    /// <summary>The multiplicity array itself, for ported code: don't modify it.</summary>
    internal NCollection_Array1<int> InternalMultiplicities() => _mults;

    /// <summary>The number of knots.</summary>
    public int KnotCount => _knots.Length();

    /// <summary>The number of poles.</summary>
    public int PoleCount => _poles.Length();

    /// <summary>The pole of index Index.</summary>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbPoles()].</exception>
    internal Point2D Pole(int index)
    {
        Refusal.RangeIf(index < 1 || index > _poles.Length(), FailureKind.OutOfRange, "Geom2d_BSplineCurve::Pole");
        return _poles.Value(index);
    }

    /// <summary>Copies the poles into P.</summary>
    /// <exception cref="DimensionErrorException">P's length is not NbPoles().</exception>
    internal void CopyPolesTo(NCollection_Array1<Point2D> p)
    {
        Refusal.ArgumentIf(p.Length() != _poles.Length(), FailureKind.DimensionError, "Geom2d_BSplineCurve::Poles");
        p.Assign(_poles);
    }

    /// <summary>The poles (a copy; OCCT returns a reference to its array).</summary>
    public IReadOnlyList<Point2D> Poles => _poles.ToImmutableArray();

    /// <summary>The pole array itself, indexed from 1, for ported code: don't modify it.</summary>
    internal NCollection_Array1<Point2D> InternalPoles() => _poles;

    /// <summary>The first pole if the first knot has multiplicity Degree() + 1, the point at FirstParameter() otherwise.</summary>
    public sealed override Point2D StartPoint
    {
        get
        {
            if (_mults.Value(1) == _deg + 1)
            {
                return _poles.Value(1);
            }
            else
            {
                return Value(FirstParameter);
            }
        }
    }

    /// <summary>The weight of the pole of index Index (1 if the curve is not rational).</summary>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbPoles()].</exception>
    internal double Weight(int index)
    {
        Refusal.RangeIf(index < 1 || index > _poles.Length(), FailureKind.OutOfRange, "Geom2d_BSplineCurve::Weight");
        if (IsRational)
        {
            return _weights.Value(index);
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
        Refusal.ArgumentIf(w.Length() != _poles.Length(), FailureKind.DimensionError, "Geom2d_BSplineCurve::Weights");
        if (IsRational)
        {
            w.Assign(_weights);
        }
        else
        {
            int i;

            for (i = w.Lower(); i <= w.Upper(); i++)
            {
                w[i] = 1.0;
            }
        }
    }

    /// <summary>The weights (a copy), or null if the curve is not rational (OCCT's <c>BSplCLib::NoWeights()</c>).</summary>
    internal IReadOnlyList<double>? WeightsOrNull() => IsRational ? _weights.ToImmutableArray() : null;

    /// <summary>The weight array for ported code, or null if the curve is not rational: don't modify it.</summary>
    internal NCollection_Array1<double>? InternalWeights()
    {
        if (IsRational)
        {
            return _weights;
        }
        return global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights();
    }

    /// <summary>The weights (a copy), ones if the curve is not rational.</summary>
    public IReadOnlyList<double> Weights => _weights.ToImmutableArray();

    /// <summary>The weight array (shared unit weights if not rational) for ported code: don't modify it.</summary>
    internal NCollection_Array1<double> InternalWeightsArray() => _weights;

    /// <summary>True if the weights are not all equal.</summary>
    public bool IsRational => _rational;

    /// <summary>Applies T to the poles.</summary>
    protected internal sealed override void Transform(in Transform2D transform)
    {
        CheckMutable();
        for (int i = 1; i <= _poles.Length(); i++)
        {
            _poles[i] = _poles[i].Transformed(transform);
        }
        _maxDerivInvOk = false;
        // OCCT: the evaluation representation is kept (Geom_BSplineCurve clears it), so the transformed curve still
        // evaluates through the untransformed representation (occt-issues.md). The same holds for every mutator here.
    }

    /// <summary>
    /// Locates U among the knots (the flat knots if WithKnotRepetition): I1 = I2 = the index of a knot within
    /// ParametricTolerance of U, or the indexes of the knots around U (0 and 1, or NbKnots and NbKnots + 1,
    /// outside the range).
    /// </summary>
    internal void LocateU(double u,
                        double parametricTolerance,
                        out int i1,
                        out int i2,
                        bool withKnotRepetition = false)
    {
        double newU = u;
        NCollection_Array1<double> cKnots = withKnotRepetition ? _flatKnots : _knots;

        PeriodicNormalization(ref newU); // Attention a la periode

        double uFirst = cKnots[1];
        double uLast = cKnots[cKnots.Length()];
        double pParametricTolerance = Math.Abs(parametricTolerance);
        if (Math.Abs(newU - uFirst) <= pParametricTolerance)
        {
            i1 = i2 = 1;
        }
        else if (Math.Abs(newU - uLast) <= pParametricTolerance)
        {
            i1 = i2 = cKnots.Length();
        }
        else if (newU < uFirst)
        {
            i2 = 1;
            i1 = 0;
        }
        else if (newU > uLast)
        {
            i1 = cKnots.Length();
            i2 = i1 + 1;
        }
        else
        {
            i1 = 1;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(cKnots, newU, out i1);
            i1 = Max(Min(i1, cKnots.Upper()), cKnots.Lower());
            while (i1 + 1 <= cKnots.Upper() && Math.Abs(cKnots[i1 + 1] - newU) <= pParametricTolerance)
            {
                i1++;
            }
            if (Math.Abs(cKnots[i1] - newU) <= pParametricTolerance)
            {
                i2 = i1;
            }
            else
            {
                i2 = i1 + 1;
            }
        }
    }

    /// <summary>
    /// The parametric tolerance UTolerance such that |t1 - t0| &lt; UTolerance implies
    /// |C(t1) - C(t0)| &lt; ToleranceUV.
    /// </summary>
    internal void Resolution(double toleranceUV, out double uTolerance)
    {
        // Kerf: a cache filled on a possibly shared curve; the flag is published after the value.
        if (!Volatile.Read(ref _maxDerivInvOk))
        {
            double maxDerivInv;
            if (_periodic)
            {
                int nbKnots, nbPoles;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareUnperiodize(_deg, _mults, out nbKnots, out nbPoles);
                NCollection_Array1<Point2D> new_poles = new NCollection_Array1<Point2D>(1, nbPoles);
                NCollection_Array1<double> new_weights = new NCollection_Array1<double>(1, nbPoles);
                for (int ii = 1; ii <= nbPoles; ii++)
                {
                    new_poles[ii] = _poles[((ii - 1) % _poles.Length()) + 1];
                }
                if (_rational)
                {
                    for (int ii = 1; ii <= nbPoles; ii++)
                    {
                        new_weights[ii] = _weights[((ii - 1) % _poles.Length()) + 1];
                    }
                }
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Resolution(new_poles,
                                    _rational ? new_weights : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                    new_poles.Length(),
                                    _flatKnots,
                                    _deg,
                                    1.0,
                                    out maxDerivInv);
            }
            else
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Resolution(_poles,
                                    InternalWeights(),
                                    _poles.Length(),
                                    _flatKnots,
                                    _deg,
                                    1.0,
                                    out maxDerivInv);
            }
            _maxDerivInv = maxDerivInv;
            Volatile.Write(ref _maxDerivInvOk, true);
        }
        uTolerance = toleranceUV * _maxDerivInv;
    }

    // Non-mutating twins of the mutators (Kerf)

    /// <summary>A copy with its degree raised to Degree.</summary>
    public BSplineCurve2D DegreeIncreased(int degree)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.IncreaseDegree(degree);
        return copy;
    }

    /// <summary>A copy with the multiplicity of the knot of index Index raised to M.</summary>
    internal BSplineCurve2D MultiplicityIncreased(int index, int m)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.IncreaseMultiplicity(index, m);
        return copy;
    }

    /// <summary>A copy with the multiplicities of the knots of indexes I1 to I2 raised to M.</summary>
    internal BSplineCurve2D MultiplicityIncreased(int i1, int i2, int m)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.IncreaseMultiplicity(i1, i2, m);
        return copy;
    }

    /// <summary>A copy with the multiplicities of the knots of indexes I1 to I2 incremented by Step.</summary>
    internal BSplineCurve2D MultiplicityIncremented(int i1, int i2, int step)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.IncrementMultiplicity(i1, i2, step);
        return copy;
    }

    /// <summary>A copy with the knot U inserted (see InsertKnot).</summary>
    internal BSplineCurve2D KnotInserted(double u, int m = 1, double parametricTolerance = 0.0)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.InsertKnot(u, m, parametricTolerance);
        return copy;
    }

    /// <summary>A copy with the knots Knots inserted (see InsertKnots; Add defaults to false, as in OCCT).</summary>
    internal BSplineCurve2D KnotsInserted(NCollection_Array1<double> knots,
                                             NCollection_Array1<int> mults,
                                             double parametricTolerance = 0.0,
                                             bool add = false)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.InsertKnots(knots, mults, parametricTolerance, add);
        return copy;
    }

    /// <summary>A copy with the multiplicity of the knot of index Index lowered to M, or null if that changes the curve by more than Tolerance.</summary>
    internal BSplineCurve2D? KnotRemoved(int index, int m, double tolerance)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        return copy.RemoveKnot(index, m, tolerance) ? copy : null;
    }

    /// <summary>A copy with the pole P (weight Weight) inserted after the pole of index Index.</summary>
    internal BSplineCurve2D PoleInsertedAfter(int index, in Point2D p, double weight = 1.0)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.InsertPoleAfter(index, p, weight);
        return copy;
    }

    /// <summary>A copy with the pole P (weight Weight) inserted before the pole of index Index.</summary>
    internal BSplineCurve2D PoleInsertedBefore(int index, in Point2D p, double weight = 1.0)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.InsertPoleBefore(index, p, weight);
        return copy;
    }

    /// <summary>A copy without the pole of index Index.</summary>
    internal BSplineCurve2D PoleRemoved(int index)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.RemovePole(index);
        return copy;
    }

    /// <summary>A copy limited to [U1, U2] (see Segment).</summary>
    public BSplineCurve2D Segmented(double u1, double u2, double tolerance = Precision_PConfusion)
    {
        try
        {
            BSplineCurve2D copy = new BSplineCurve2D(this);
            copy.Segment(u1, u2, tolerance);
            return copy;
        }
        catch (FailureException failure)
        {
            throw Refusal.Argument(failure);
        }
    }

    /// <summary>A copy with the knot of index Index changed to K.</summary>
    internal BSplineCurve2D WithKnotOneBased(int index, double k)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.SetKnot(index, k);
        return copy;
    }

    /// <summary>A copy with the knots K.</summary>
    internal BSplineCurve2D WithKnots(NCollection_Array1<double> k)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.SetKnots(k);
        return copy;
    }

    /// <summary>A copy with the knot of index Index raised to multiplicity M and changed to K.</summary>
    internal BSplineCurve2D WithKnotOneBased(int index, double k, int m)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.SetKnot(index, k, m);
        return copy;
    }

    /// <summary>A periodic copy (see SetPeriodic).</summary>
    public BSplineCurve2D WithPeriodic()
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.SetPeriodic();
        return copy;
    }

    /// <summary>A copy of this periodic curve with its origin at the knot of index Index.</summary>
    internal BSplineCurve2D WithOrigin(int index)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.SetOrigin(index);
        return copy;
    }

    /// <summary>A non-periodic copy (see SetNotPeriodic).</summary>
    public BSplineCurve2D WithNotPeriodic()
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.SetNotPeriodic();
        return copy;
    }

    /// <summary>A copy with the pole of index Index replaced by P.</summary>
    internal BSplineCurve2D WithPoleOneBased(int index, in Point2D p)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.SetPole(index, p);
        return copy;
    }

    /// <summary>A copy with the pole of index Index replaced by P, of weight W.</summary>
    internal BSplineCurve2D WithPoleOneBased(int index, in Point2D p, double w)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.SetPole(index, p, w);
        return copy;
    }

    /// <summary>A copy with the weight of the pole of index Index changed to W.</summary>
    internal BSplineCurve2D WithWeightOneBased(int index, double w)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.SetWeight(index, w);
        return copy;
    }

    /// <summary>A copy whose point of parameter U is moved to P (see MovePoint).</summary>
    internal BSplineCurve2D PointMoved(double u,
                                          in Point2D p,
                                          int index1,
                                          int index2,
                                          out int firstModifiedPole,
                                          out int lastModifiedPole)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.MovePoint(u, p, index1, index2, out firstModifiedPole, out lastModifiedPole);
        return copy;
    }

    /// <summary>A copy whose point and tangent at U are moved (see MovePointAndTangent).</summary>
    internal BSplineCurve2D PointAndTangentMoved(double u,
                                                    in Point2D p,
                                                    in Vector2D tangent,
                                                    double tolerance,
                                                    int startingCondition,
                                                    int endingCondition,
                                                    out int errorStatus)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.MovePointAndTangent(u, p, tangent, tolerance, startingCondition, endingCondition, out errorStatus);
        return copy;
    }

    /// <summary>A copy evaluated through theDesc, which is copied (so later changes to theDesc don't affect it).</summary>
    /// <exception cref="ProgramErrorException">The descriptor is invalid.</exception>
    internal BSplineCurve2D WithEvalRepresentation(RepCurveDesc2D.Base? desc)
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        RepUtils2D.ValidateCurveDesc(desc, this);
        copy.SetEvalRepresentation(RepUtils2D.CloneCurveDesc(desc));
        return copy;
    }

    /// <summary>A copy without an evaluation representation.</summary>
    internal BSplineCurve2D WithoutEvalRepresentation()
    {
        BSplineCurve2D copy = new BSplineCurve2D(this);
        copy.ClearEvalRepresentation();
        return copy;
    }
}
