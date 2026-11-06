// Port of: src/ModelingData/TKG3d/Geom/Geom_BSplineCurve.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_BSplineCurve.cxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_BSplineCurve_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Evaluation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine;
using static Kerf.Engine.Geometry.BSplKnotDistribution;
using static Kerf.Geometry.Continuity;

namespace Kerf.Geometry;

/// <summary>
/// A B-spline curve: uniform or non-uniform, rational or non-rational, periodic or non-periodic. It is defined by
/// its degree (1 to <see cref="MaxDegree"/>, 25), its poles (with weights when rational) and its knots with their
/// multiplicities, all indexed from 1. The knots are strictly increasing; the continuity at an inner knot is
/// Degree - Mult. A non-periodic curve has NbPoles = Sum(Mults) - Degree - 1 poles, a periodic one
/// Sum(Mults) - Mult(last) poles (the first and last multiplicities being equal).
/// </summary>
/// <remarks>
/// <para>
/// Arrays: <see cref="Poles"/>, <see cref="WeightsOrNull()"/>, <see cref="Weights"/>, <see cref="Knots"/>,
/// <see cref="Multiplicities"/> and <see cref="KnotSequence"/> return immutable copies (OCCT returns
/// references to the curve's arrays); their index 0 is OCCT's index 1. Code inside Kerf that passes the
/// arrays on to BSplCLib and the like uses the internal <c>Internal*</c> accessors, which return the stored
/// arrays (shared: don't modify).
/// </para>
/// <para>
/// The evaluation representation (<see cref="HasEvalRepresentation"/>) is set by <see cref="WithEvalRepresentation"/>,
/// which stores a copy of the descriptor. Modifications of the curve drop it, as in OCCT.
/// </para>
/// </remarks>
public sealed partial class BSplineCurve3D : BoundedCurve3D
{
    private NCollection_Array1<Point3D> _poles = new NCollection_Array1<Point3D>();
    private NCollection_Array1<double> _weights = new NCollection_Array1<double>();
    private NCollection_Array1<double> _knots = new NCollection_Array1<double>();
    private NCollection_Array1<double> _flatKnots = new NCollection_Array1<double>();
    private NCollection_Array1<int> _mults = new NCollection_Array1<int>();
    private RepCurveDesc.Base? _evalRep;
    private int _deg = 0;
    private bool _periodic = false;
    private bool _rational = false;
    private BSplKnotDistribution _knotSet = NonUniform;
    private Continuity _smooth = C0;
    private double _maxDerivInv = 0.0;
    private bool _maxDerivInvOk = false;

    //=============================================================================================
    // Geom_BSplineCurve.cxx
    //=============================================================================================

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

    private static void CheckCurveData(NCollection_Array1<Point3D> cPoles,
                                       NCollection_Array1<double> cKnots,
                                       NCollection_Array1<int> cMults,
                                       int degree,
                                       bool periodic)
    {
        if (degree < 1 || degree > BSplineCurve3D.MaxDegree)
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

    /// <summary>Check rationality of an array of weights.</summary>
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
    public sealed override Geometry3D Copy() => new BSplineCurve3D(this);

    /// <summary>A copy of theOther, without validation (OCCT's copy constructor).</summary>
    public BSplineCurve3D(BSplineCurve3D other)
    {
        _poles = new NCollection_Array1<Point3D>(other._poles);
        _weights = other._rational ? new NCollection_Array1<double>(other._weights)
                                        : global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(other._poles.Length());
        _knots = new NCollection_Array1<double>(other._knots);
        _flatKnots = new NCollection_Array1<double>(other._flatKnots);
        _mults = new NCollection_Array1<int>(other._mults);
        _evalRep = RepUtils.CloneCurveDesc(other._evalRep);
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
    /// Degree not in [1, MaxDegree()], fewer than 2 poles, knots and multiplicities of different lengths, knots
    /// not increasing (by more than Epsilon), or a number of poles that doesn't match the multiplicities.
    /// </exception>
    internal BSplineCurve3D(NCollection_Array1<Point3D> poles,
                             NCollection_Array1<double> knots,
                             NCollection_Array1<int> mults,
                             int degree,
                             bool periodic = false)
    {
        _rational = false;
        _maxDerivInv = 0.0;
        _maxDerivInvOk = false;
        _deg = degree;
        _periodic = periodic;

        // check
        CheckCurveData(poles, knots, mults, degree, periodic);

        // copy arrays
        _poles.Resize(1, poles.Length(), false);
        _poles.CopyValues(poles);
        _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(poles.Length());

        _knots.Resize(1, knots.Length(), false);
        _knots.CopyValues(knots);

        _mults.Resize(1, mults.Length(), false);
        _mults.CopyValues(mults);

        UpdateKnots();
    }

    /// <summary>
    /// A rational B-spline curve; with CheckRational, it is non-rational if all the weights are equal.
    /// </summary>
    /// <exception cref="ConstructionErrorException">
    /// As the non-rational constructor, or the weights are not as many as the poles, or a weight is at most
    /// gp::Resolution().
    /// </exception>
    internal BSplineCurve3D(NCollection_Array1<Point3D> poles,
                             NCollection_Array1<double> weights,
                             NCollection_Array1<double> knots,
                             NCollection_Array1<int> mults,
                             int degree,
                             bool periodic = false,
                             bool checkRational = true)
    {
        _rational = true;
        _maxDerivInv = 0.0;
        _maxDerivInvOk = false;
        _deg = degree;
        _periodic = periodic;

        // check
        CheckCurveData(poles, knots, mults, degree, periodic);

        if (weights.Length() != poles.Length())
        {
            throw Refusal.Argument(new ConstructionErrorException("Geom_BSplineCurve: Weights and Poles array size mismatch"));
        }

        int i;
        for (i = weights.Lower(); i <= weights.Upper(); i++)
        {
            if (weights[i] <= Constants.Resolution())
            {
                throw Refusal.Argument(new ConstructionErrorException("Geom_BSplineCurve: Weights values too small"));
            }
        }

        // check really rational
        if (checkRational)
        {
            _rational = Rational(weights);
        }

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

        _mults.Resize(1, mults.Length(), false);
        _mults.CopyValues(mults);

        UpdateKnots();
    }

    /// <summary>The maximum degree of a B-spline curve: 25.</summary>
    public static int MaxDegree => global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree();

    /// <summary>Raises the degree to Degree, keeping the curve (knot multiplicities grow by the same step).</summary>
    /// <exception cref="ConstructionErrorException">Degree is less than the degree or more than MaxDegree().</exception>
    internal void IncreaseDegree(int degree)
    {
        CheckMutable();
        if (degree == _deg)
        {
            return;
        }

        ClearEvalRepresentation();

        if (degree < _deg || degree > BSplineCurve3D.MaxDegree)
        {
            throw Refusal.Argument(new ConstructionErrorException("BSpline curve: IncreaseDegree: bad degree value"));
        }

        int fromK1 = FirstUKnotIndex();
        int toK2 = LastUKnotIndex();

        int step = degree - _deg;

        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, _poles.Length() + step * (toK2 - fromK1));

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

    /// <summary>Raises the multiplicity of the knot Index to M (unchanged if already at least M).</summary>
    internal void IncreaseMultiplicity(int index, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = _knots.Value(index);
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M - _mults.Value(index);
        InsertKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>Raises the multiplicities of the knots I1 to I2 to M.</summary>
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

    /// <summary>Raises the multiplicities of the knots I1 to I2 by Step.</summary>
    internal void IncrementMultiplicity(int i1, int i2, int step)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(_knots, i1, i1, i2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(i1, i2);
        m.Init(step);
        InsertKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>
    /// Inserts the knot U with multiplicity M. A knot within ParametricTolerance of an existing one raises its
    /// multiplicity: by M if Add, to M otherwise.
    /// </summary>
    internal void InsertKnot(double u, int M = 1, double parametricTolerance = 0.0, bool add = true)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = u;
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M;
        InsertKnots(k, m, parametricTolerance, add);
    }

    /// <summary>
    /// Inserts the knots with their multiplicities (see <see cref="InsertKnot"/>). Note that Add defaults to false
    /// here, and to true in InsertKnot and BSplCLib.InsertKnots.
    /// </summary>
    /// <exception cref="ConstructionErrorException">The knots can't be inserted (BSplCLib::PrepareInsertKnots).</exception>
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
            throw Refusal.Argument(new ConstructionErrorException("Geom_BSplineCurve::InsertKnots"));
        }

        if (nbpoles == _poles.Length())
        {
            return;
        }

        ClearEvalRepresentation();

        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, nbpoles);
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
    /// Lowers the multiplicity of the knot Index to M (0 removes it) if the curve stays within Tolerance;
    /// returns false, leaving the curve unchanged, otherwise.
    /// </summary>
    /// <exception cref="OutOfRangeException">Index is not an inner knot (non-periodic) or not in the knot range (periodic).</exception>
    internal bool RemoveKnot(int index, int m, double tolerance)
    {
        CheckMutable();
        if (m < 0)
        {
            return true;
        }

        int i1 = FirstUKnotIndex();
        int i2 = LastUKnotIndex();

        if (!_periodic && (index <= i1 || index >= i2))
        {
            throw Refusal.Range(new OutOfRangeException("BSpline curve: RemoveKnot: index out of range"));
        }
        else if (_periodic && (index < i1 || index > i2))
        {
            throw Refusal.Range(new OutOfRangeException("BSpline curve: RemoveKnot: index out of range"));
        }

        NCollection_Array1<Point3D> oldpoles = _poles;

        int step = _mults.Value(index) - m;
        if (step <= 0)
        {
            return true;
        }

        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, oldpoles.Length() - step);

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

        ClearEvalRepresentation();

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
    /// Reverses the parametrization; the knots K1, ..., Kn become -Kn, ..., -K1 shifted to keep the bounds.
    /// </summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        ClearEvalRepresentation();

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

    /// <summary>
    /// Keeps the part between U1 and U2 (U1 &lt;= U2); a periodic curve becomes non-periodic. theTolerance is the
    /// minimum distance at which a new knot is inserted.
    /// </summary>
    /// <exception cref="DomainErrorException">U2 &lt; U1, or U2 - U1 exceeds the period of a periodic curve.</exception>
    internal void Segment(double u1, double u2, double tolerance = Precision_PConfusion)
    {
        CheckMutable();
        if (u2 < u1)
        {
            throw Refusal.Argument(new DomainErrorException("Geom_BSplineCurve::Segment"));
        }

        ClearEvalRepresentation();

        double newU1, newU2;
        double u, du = 0, ddu = 0;
        int index;
        bool wasPeriodic = _periodic;

        NCollection_Array1<double> knots = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int> mults = new NCollection_Array1<int>(1, 2);

        // define param distance to keep (eap, Apr 18 2002, occ311)
        if (_periodic)
        {
            double period = LastParameter - FirstParameter;
            du = u2 - u1;
            if (du - period > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                throw Refusal.Argument(new DomainErrorException("Geom_BSplineCurve::Segment"));
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

        knots[1] = Min(newU1, newU2);
        knots[2] = Max(newU1, newU2);
        mults[1] = mults[2] = _deg;

        double absUMax = Max(Math.Abs(newU1), Math.Abs(newU2));

        //  Modified by Sergey KHROMOV - Fri Apr 11 12:15:40 2003 Begin
        absUMax = Max(absUMax, Max(Math.Abs(FirstParameter), Math.Abs(LastParameter)));
        //  Modified by Sergey KHROMOV - Fri Apr 11 12:15:40 2003 End

        double eps = Max(Epsilon(absUMax), tolerance);

        InsertKnots(knots, mults, eps);

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
            // Test if the insertion is OK, shift otherwise.
            if (Math.Abs(_knots.Value(index + 1) - u) <= eps)
            { // <= to be consistent with InsertKnots
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

        int i, k = 1;
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
        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, nbpoles);

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

    // Precision::PConfusion() as a constant, for the default argument of Segment.
    private const double Precision_PConfusion = 1.0e-7 * 0.01;

    /// <summary>Changes the value of the knot Index (it must stay strictly between its neighbours).</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbKnots].</exception>
    /// <exception cref="ConstructionErrorException">K is not between the neighbouring knots.</exception>
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
            ClearEvalRepresentation();
            _knots.SetValue(index, k);
            _maxDerivInvOk = false;
            UpdateKnots();
        }
    }

    /// <summary>Changes all the knots.</summary>
    /// <exception cref="ConstructionErrorException">The knots are invalid for the curve.</exception>
    internal void SetKnots(NCollection_Array1<double> k)
    {
        CheckMutable();
        CheckCurveData(_poles, k, _mults, _deg, _periodic);
        ClearEvalRepresentation();
        _knots.CopyValues(k);
        _maxDerivInvOk = false;
        UpdateKnots();
    }

    /// <summary>Raises the multiplicity of the knot Index to M, then changes its value.</summary>
    internal void SetKnot(int index, double k, int m)
    {
        CheckMutable();
        IncreaseMultiplicity(index, m);
        SetKnot(index, k);
    }

    /// <summary>
    /// Makes the curve periodic on its parameter range: the knots outside [FirstUKnotIndex, LastUKnotIndex] are
    /// dropped, the end multiplicities set to Min(Degree, Max(first, last)) and the poles truncated.
    /// </summary>
    internal void SetPeriodic()
    {
        CheckMutable();
        ClearEvalRepresentation();

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

    /// <summary>Makes the knot Index the origin of a periodic curve.</summary>
    /// <exception cref="NoSuchObjectException">The curve is not periodic.</exception>
    /// <exception cref="DomainErrorException">Index is not in [FirstUKnotIndex, LastUKnotIndex].</exception>
    internal void SetOrigin(int Index)
    {
        CheckMutable();
        if (!_periodic)
        {
            throw Refusal.State(new NoSuchObjectException("Geom_BSplineCurve::SetOrigin"));
        }

        ClearEvalRepresentation();

        int i, k;
        int first = FirstUKnotIndex();
        int last = LastUKnotIndex();

        if ((Index < first) || (Index > last))
        {
            throw Refusal.Argument(new DomainErrorException("Geom_BSplineCurve::SetOrigin"));
        }

        int nbknots = _knots.Length();
        int nbpoles = _poles.Length();

        NCollection_Array1<double> newknots = new NCollection_Array1<double>(1, nbknots);

        NCollection_Array1<int> newmults = new NCollection_Array1<int>(1, nbknots);

        // set the knots and mults
        double period = _knots.Value(last) - _knots.Value(first);
        k = 1;
        for (i = Index; i <= last; i++)
        {
            newknots[k] = _knots.Value(i);
            newmults[k] = _mults.Value(i);
            k++;
        }
        for (i = first + 1; i <= Index; i++)
        {
            newknots[k] = _knots.Value(i) + period;
            newmults[k] = _mults.Value(i);
            k++;
        }

        int index = 1;
        for (i = first + 1; i <= Index; i++)
        {
            index += _mults.Value(i);
        }

        // set the poles and weights
        NCollection_Array1<Point3D> newpoles = new NCollection_Array1<Point3D>(1, nbpoles);
        first = _poles.Lower();
        last = _poles.Upper();
        if (_rational)
        {
            NCollection_Array1<double> newweights = new NCollection_Array1<double>(1, nbpoles);
            k = 1;
            for (i = index; i <= last; i++)
            {
                newpoles[k] = _poles.Value(i);
                newweights[k] = _weights.Value(i);
                k++;
            }
            for (i = first; i < index; i++)
            {
                newpoles[k] = _poles.Value(i);
                newweights[k] = _weights.Value(i);
                k++;
            }
            _weights = newweights;
        }
        else
        {
            k = 1;
            for (i = index; i <= last; i++)
            {
                newpoles[k] = _poles.Value(i);
                k++;
            }
            for (i = first; i < index; i++)
            {
                newpoles[k] = _poles.Value(i);
                k++;
            }
            _weights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(nbpoles);
        }

        _poles = newpoles;
        _knots = newknots;
        _mults = newmults;
        _maxDerivInvOk = false;
        UpdateKnots();
    }

    /// <summary>
    /// Makes the parameter U the origin of a periodic curve: the knots are shifted by whole periods so that U is
    /// in range, a knot is inserted at U unless one is within Tol, and that knot becomes the origin.
    /// </summary>
    /// <exception cref="NoSuchObjectException">The curve is not periodic.</exception>
    internal void SetOrigin(double U, double tol)
    {
        CheckMutable();
        if (!_periodic)
        {
            throw Refusal.State(new NoSuchObjectException("Geom_BSplineCurve::SetOrigin"));
        }
        // Is U within the period?
        double uf = FirstParameter, ul = LastParameter;
        double u = U, period = ul - uf;
        while (tol < (uf - u))
        {
            u += period;
        }
        while (tol > (ul - u))
        {
            u -= period;
        }
        if (Math.Abs(U - u) > tol)
        { // Reparametrize the curve
            ClearEvalRepresentation();
            double delta = U - u;
            uf += delta;
            ul += delta;
            int fk = _knots.Lower(), lk = _knots.Upper();
            for (int i = fk; i <= lk; i++)
            {
                _knots.ChangeValue(i) += delta;
            }
            UpdateKnots();
        }
        {
            // For periodic curve, uf and ul represent the same point
            if (Math.Abs(U - uf) < tol || Math.Abs(U - ul) < tol)
            {
                return;
            }

            int fk = _knots.Lower(), lk = _knots.Upper(), ik = 0;
            double delta = RealLast();
            for (int i = fk; i <= lk; i++)
            {
                double dki = _knots.Value(i) - U;
                if (Math.Abs(dki) < Math.Abs(delta))
                {
                    ik = i;
                    delta = dki;
                }
            }
            if (Math.Abs(delta) > tol)
            {
                InsertKnot(U);
                if (delta < 0.0)
                {
                    ik++;
                }
            }

            SetOrigin(ik);
        }
    }

    /// <summary>Makes a periodic curve non-periodic (the same curve, with clamped ends).</summary>
    internal void SetNotPeriodic()
    {
        CheckMutable();
        if (_periodic)
        {
            ClearEvalRepresentation();
            int nbKnots, nbPoles;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareUnperiodize(_deg, _mults, out nbKnots, out nbPoles);

            NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, nbPoles);

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

    /// <summary>Replaces the pole Index (its weight is kept).</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    internal void SetPole(int index, in Point3D p)
    {
        CheckMutable();
        if (index < 1 || index > _poles.Length())
        {
            throw Refusal.Range(new OutOfRangeException("BSpline curve: SetPole: index and #pole mismatch"));
        }
        ClearEvalRepresentation();
        _poles.SetValue(index, p);
        _maxDerivInvOk = false;
    }

    /// <summary>Replaces the pole Index and its weight.</summary>
    internal void SetPole(int index, in Point3D p, double w)
    {
        CheckMutable();
        SetPole(index, p);
        SetWeight(index, w);
    }

    /// <summary>Changes the weight of the pole Index; the curve may become rational or non-rational.</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    /// <exception cref="ConstructionErrorException">W is at most gp::Resolution().</exception>
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

        ClearEvalRepresentation();

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
    /// Moves the point of parameter U to P by changing the poles Index1 to Index2 (as few as possible);
    /// FirstModifiedPole and LastModifiedPole are 0 if nothing could be done.
    /// </summary>
    /// <exception cref="OutOfRangeException">Index1 or Index2 is not in [1, NbPoles], or Index1 &gt; Index2.</exception>
    internal void MovePoint(double u,
                            in Point3D p,
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
        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, _poles.Length());
        Point3D p0;
        D0(u, out p0);
        Vector3D displ = new Vector3D(p0, p);

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
            ClearEvalRepresentation();
            _poles = npoles;
            _maxDerivInvOk = false;
        }
    }

    /// <summary>
    /// Moves the point of parameter U to P and its tangent to Tangent. A periodic curve is made non-periodic
    /// first. ErrorStatus is 0 on success, and the curve unchanged otherwise.
    /// </summary>
    internal void MovePointAndTangent(double u,
                                      in Point3D p,
                                      in Vector3D tangent,
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
        NCollection_Array1<Point3D> new_poles = new NCollection_Array1<Point3D>(1, _poles.Length());
        Point3D p0;

        Vector3D delta_derivative;
        D1(u, out p0, out delta_derivative);
        Vector3D delta = new Vector3D(p0, p);
        for (ii = 1; ii <= 3; ii++)
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
            ClearEvalRepresentation();
            _poles = new_poles;
            _maxDerivInvOk = false;
        }
    }

    /// <summary>Recomputes the flat knots, the knot distribution and the continuity.</summary>
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

    /// <summary>
    /// Brings theParameter into [FirstParameter, LastParameter] by whole periods when the curve is periodic
    /// (LastParameter itself is left unchanged); does nothing otherwise.
    /// </summary>
    internal void PeriodicNormalization(ref double parameter)
    {
        if (!_periodic)
        {
            return;
        }

        double first = _flatKnots.Value(_deg + 1);
        double last = _flatKnots.Value(_flatKnots.Upper() - _deg);

        // If the parameter is already within the valid range, no adjustment is needed.
        // Note: Strictly speaking, the parameter should be within [aFirst, aLast), since for
        // periodic curves first and last parameters represent the same point.
        // However, we allow the case theParameter == aLast to be valid as well just because
        // there is already a lot of code that relies on this particular behavior.
        if (parameter >= first && parameter <= last)
        {
            return;
        }

        double period = last - first;

        // Shift by the exact number of full periods in one step - O(1) regardless of distance.
        double nbPeriods = Math.Floor((parameter - first) / period);
        if (nbPeriods != 0.0)
        {
            parameter -= nbPeriods * period;
        }

        // One-step correction for floating-point residual overshoot after the bulk shift.
        if (parameter > last)
        {
            double newParam = parameter - period;
            parameter = newParam == parameter ? NextAfter(newParam, first) : newParam;
        }
        else if (parameter < first)
        {
            double newParam = parameter + period;
            parameter = newParam == parameter ? NextAfter(newParam, last) : newParam;
        }
    }

    //=============================================================================================
    // Geom_BSplineCurve_1.cxx
    //=============================================================================================

    /// <summary>True if the curve is at least CN.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    internal sealed override bool IsCN(int n)
    {
        Refusal.RangeIf(n < 0, FailureKind.RangeError, "Geom_BSplineCurve::IsCN");

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
    /// True if the curve is G1 on [theTf, theTl]: at every inner knot of multiplicity at least the degree in the
    /// range, the tangents on both sides make an angle at most theAngTol (and, for a periodic curve whose range
    /// includes its ends, also at the ends).
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

            Point3D p1, p2;
            Vector3D v1, v2;
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

        Point3D p;
        Vector3D v1b, v2b;
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

    /// <inheritdoc/>
    public sealed override bool IsPeriodic => _periodic;

    /// <summary>The global continuity: C0 to C3, or CN without inner knots.</summary>
    public sealed override Continuity Continuity => _smooth;

    /// <summary>The polynomial degree of its spans.</summary>
    public int Degree => _deg;

    internal sealed override Point3D EvalD0(double u)
    {
        if (RepUtils.TryEvalCurveD0(_evalRep, u, out Point3D evalRepResult))
        {
            return evalRepResult;
        }

        Point3D p;
        int spanIndex = 0;
        double newU = u;
        // OCCT: the normalized aNewU is overwritten by LocateParameter, which is given U (dead code, harmless)
        PeriodicNormalization(ref newU);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _knots, _mults, u, _periodic, ref spanIndex, out newU);
        if (newU < _knots[spanIndex])
        {
            spanIndex--;
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(newU, spanIndex, _deg, _periodic, _poles, InternalWeights(), _knots, _mults, out p);
        return p;
    }

    internal sealed override ResD1 EvalD1(double u)
    {
        if (RepUtils.TryEvalCurveD1(_evalRep, u, out ResD1 evalRepResult))
        {
            return evalRepResult;
        }

        ResD1 result = new ResD1();
        int spanIndex = 0;
        double newU = u;
        PeriodicNormalization(ref newU);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _knots, _mults, u, _periodic, ref spanIndex, out newU);
        if (newU < _knots[spanIndex])
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
        if (RepUtils.TryEvalCurveD2(_evalRep, u, out ResD2 evalRepResult))
        {
            return evalRepResult;
        }

        ResD2 result = new ResD2();
        int spanIndex = 0;
        double newU = u;
        PeriodicNormalization(ref newU);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _knots, _mults, u, _periodic, ref spanIndex, out newU);
        if (newU < _knots[spanIndex])
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
        if (RepUtils.TryEvalCurveD3(_evalRep, u, out ResD3 evalRepResult))
        {
            return evalRepResult;
        }

        ResD3 result = new ResD3();
        int spanIndex = 0;
        double newU = u;
        PeriodicNormalization(ref newU);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _knots, _mults, u, _periodic, ref spanIndex, out newU);
        if (newU < _knots[spanIndex])
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

    /// <exception cref="UndefinedDerivative3DException">N &lt; 1.</exception>
    internal sealed override Vector3D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw Refusal.State(new UndefinedDerivative3DException());
        }

        if (RepUtils.TryEvalCurveDN(_evalRep, u, n, out Vector3D evalRepResult))
        {
            return evalRepResult;
        }

        Vector3D v;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(u, n, 0, _deg, _periodic, _poles, InternalWeights(), _flatKnots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out v);
        return v;
    }

    /// <summary>The end point: the last pole when the last multiplicity is Degree + 1.</summary>
    public sealed override Point3D EndPoint
    {
        get
        {
            if (_mults[_knots.Upper()] == _deg + 1)
            {
                return _poles[_poles.Upper()];
            }
            else
            {
                return Value(LastParameter);
            }
        }
    }

    /// <summary>The index of the knot giving the first parameter (1 for a periodic curve).</summary>
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

    /// <summary>The parameter of the start point: the first knot of the span the curve begins on.</summary>
    public sealed override double FirstParameter => _flatKnots[_deg + 1];

    /// <summary>The knot Index.</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbKnots].</exception>
    internal double Knot(int index)
    {
        Refusal.RangeIf(index < 1 || index > _knots.Length(), FailureKind.OutOfRange, "Geom_BSplineCurve::Knot");
        return _knots[index];
    }

    /// <summary>NonUniform, Uniform, QuasiUniform or PiecewiseBezier.</summary>
    internal BSplKnotDistribution KnotDistribution => _knotSet;

    /// <summary>Copies the knots K.Lower() to K.Upper() into K (OCCT's deprecated overload).</summary>
    /// <exception cref="DomainErrorException">K's bounds are outside [1, NbKnots].</exception>
    internal void CopyKnotsTo(NCollection_Array1<double> k)
    {
        Refusal.ArgumentIf(k.Lower() < _knots.Lower() || k.Upper() > _knots.Upper(), FailureKind.DomainError, "Geom_BSplineCurve::Knots");
        for (int idx = k.Lower(); idx <= k.Upper(); idx++)
        {
            k[idx] = _knots[idx];
        }
    }

    /// <summary>The knots (a copy; index 0 is knot 1).</summary>
    public IReadOnlyList<double> Knots => _knots.ToImmutableArray();

    /// <summary>Copies the flat knots K.Lower() to K.Upper() into K (OCCT's deprecated overload).</summary>
    /// <exception cref="DomainErrorException">K's bounds are outside the knot sequence's.</exception>
    internal void CopyKnotSequenceTo(NCollection_Array1<double> k)
    {
        Refusal.ArgumentIf(k.Lower() < _flatKnots.Lower() || k.Upper() > _flatKnots.Upper(), FailureKind.DomainError, "Geom_BSplineCurve::KnotSequence");
        for (int idx = k.Lower(); idx <= k.Upper(); idx++)
        {
            k[idx] = _flatKnots[idx];
        }
    }

    /// <summary>The knots repeated by their multiplicities (a copy).</summary>
    public IReadOnlyList<double> KnotSequence => _flatKnots.ToImmutableArray();

    /// <summary>The index of the knot giving the last parameter (NbKnots for a periodic curve).</summary>
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

    /// <summary>The parameter of the end point: the last knot of the span the curve ends on.</summary>
    public sealed override double LastParameter => _flatKnots[_flatKnots.Upper() - _deg];

    /// <summary>The point at U, computed on the span between the knots FromK1 and ToK2.</summary>
    internal Point3D LocalValue(double u, int fromK1, int toK2)
    {
        Point3D p;
        LocalD0(u, fromK1, toK2, out p);
        return p;
    }

    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    internal void LocalD0(double U, int fromK1, int toK2, out Point3D p)
    {
        Refusal.ArgumentIf(fromK1 == toK2, FailureKind.DomainError, "Geom_BSplineCurve::LocalValue");

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

    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    internal void LocalD1(double U, int fromK1, int toK2, out Point3D p, out Vector3D v1)
    {
        Refusal.ArgumentIf(fromK1 == toK2, FailureKind.DomainError, "Geom_BSplineCurve::LocalD1");

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

    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    internal void LocalD2(double U, int fromK1, int toK2, out Point3D p, out Vector3D v1, out Vector3D v2)
    {
        Refusal.ArgumentIf(fromK1 == toK2, FailureKind.DomainError, "Geom_BSplineCurve::LocalD2");

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

    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    internal void LocalD3(double U, int fromK1, int toK2, out Point3D p, out Vector3D v1, out Vector3D v2, out Vector3D v3)
    {
        Refusal.ArgumentIf(fromK1 == toK2, FailureKind.DomainError, "Geom_BSplineCurve::LocalD3");

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

    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    internal Vector3D LocalDN(double U, int fromK1, int toK2, int n)
    {
        Refusal.ArgumentIf(fromK1 == toK2, FailureKind.DomainError, "Geom_BSplineCurve::LocalD3"); // OCCT: says LocalD3 in LocalDN

        double u = U;
        int index = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _flatKnots, U, _periodic, fromK1, toK2, out index, out u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_deg, index, _mults, _periodic);

        Vector3D v;
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

    /// <summary>The multiplicity of the knot Index.</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbKnots].</exception>
    internal int Multiplicity(int index)
    {
        Refusal.RangeIf(index < 1 || index > _mults.Length(), FailureKind.OutOfRange, "Geom_BSplineCurve::Multiplicity");
        return _mults[index];
    }

    /// <summary>Copies the multiplicities into M (OCCT's deprecated overload).</summary>
    /// <exception cref="DimensionErrorException">M has not NbKnots elements.</exception>
    internal void CopyMultiplicitiesTo(NCollection_Array1<int> m)
    {
        Refusal.ArgumentIf(m.Length() != _mults.Length(), FailureKind.DimensionError, "Geom_BSplineCurve::Multiplicities");
        m.Assign(_mults);
    }

    /// <summary>The multiplicities (a copy; index 0 is knot 1's).</summary>
    public IReadOnlyList<int> Multiplicities => _mults.ToImmutableArray();

    /// <summary>The number of distinct knots: their multiplicities aren't counted.</summary>
    public int KnotCount => _knots.Length();

    /// <summary>The number of poles.</summary>
    public int PoleCount => _poles.Length();

    /// <summary>The pole Index.</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    internal Point3D Pole(int index)
    {
        Refusal.RangeIf(index < 1 || index > _poles.Length(), FailureKind.OutOfRange, "Geom_BSplineCurve::Pole");
        return _poles[index];
    }

    /// <summary>Copies the poles into P (OCCT's deprecated overload).</summary>
    /// <exception cref="DimensionErrorException">P has not NbPoles elements.</exception>
    internal void CopyPolesTo(NCollection_Array1<Point3D> p)
    {
        Refusal.ArgumentIf(p.Length() != _poles.Length(), FailureKind.DimensionError, "Geom_BSplineCurve::Poles");
        p.Assign(_poles);
    }

    /// <summary>The poles (a copy; index 0 is pole 1).</summary>
    public IReadOnlyList<Point3D> Poles => _poles.ToImmutableArray();

    /// <summary>The start point: the first pole when the first multiplicity is Degree + 1.</summary>
    public sealed override Point3D StartPoint
    {
        get
        {
            if (_mults[1] == _deg + 1)
            {
                return _poles[1];
            }
            else
            {
                return Value(FirstParameter);
            }
        }
    }

    /// <summary>The weight of the pole Index (1 for a non-rational curve).</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    internal double Weight(int index)
    {
        Refusal.RangeIf(index < 1 || index > _poles.Length(), FailureKind.OutOfRange, "Geom_BSplineCurve::Weight");
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
    internal void CopyWeightsTo(NCollection_Array1<double> w)
    {
        Refusal.ArgumentIf(w.Length() != _poles.Length(), FailureKind.DimensionError, "Geom_BSplineCurve::Weights");
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

    /// <summary>The weights (a copy), or null for a non-rational curve (OCCT returns a null pointer).</summary>
    internal IReadOnlyList<double>? WeightsOrNull() => IsRational ? _weights.ToImmutableArray() : null;

    /// <summary>The weights, always NbPoles of them (1s for a non-rational curve); a copy.</summary>
    public IReadOnlyList<double> Weights => _weights.ToImmutableArray();

    /// <summary>True when the poles have weights that aren't all equal.</summary>
    public bool IsRational => _rational;

    /// <summary>Applies T to the poles.</summary>
    protected internal sealed override void Transform(in Transform3D transform)
    {
        CheckMutable();
        ClearEvalRepresentation();
        for (int i = 1; i <= _poles.Length(); i++)
        {
            _poles.ChangeValue(i) = _poles.ChangeValue(i).Transformed(transform);
        }
        _maxDerivInvOk = false;
    }

    /// <summary>
    /// The knots around U within ParametricTolerance: I1 = I2 = the knot's index when U is on a knot, otherwise
    /// the knots I1 &lt; U &lt; I2 (I1 = 0 or I2 = NbKnots + 1 outside the range). With WithKnotRepetition the
    /// indexes are in the knot sequence.
    /// </summary>
    internal void LocateU(double u,
                        double parametricTolerance,
                        out int i1,
                        out int i2,
                        bool withKnotRepetition = false)
    {
        double newU = u;
        NCollection_Array1<double> cKnots = withKnotRepetition ? _flatKnots : _knots;

        PeriodicNormalization(ref newU); // Handle periodicity

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
    /// |f(t1) - f(t0)| &lt; Tolerance3D.
    /// </summary>
    /// <remarks>Non-const in OCCT, because it caches a value; not a mutator, so it works on a frozen curve.</remarks>
    internal void Resolution(double tolerance3D, out double uTolerance)
    {
        // Kerf: a cache filled on a possibly shared curve; the flag is published after the value.
        if (!Volatile.Read(ref _maxDerivInvOk))
        {
            double maxDerivInv;
            if (_periodic)
            {
                int nbKnots, nbPoles;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareUnperiodize(_deg, _mults, out nbKnots, out nbPoles);
                NCollection_Array1<Point3D> new_poles = new NCollection_Array1<Point3D>(1, nbPoles);
                NCollection_Array1<double> new_weights = new NCollection_Array1<double>(1, nbPoles);
                for (int ii = 1; ii <= nbPoles; ii++)
                {
                    new_poles[ii] = _poles[(ii - 1) % _poles.Length() + 1];
                }
                if (_rational)
                {
                    for (int ii = 1; ii <= nbPoles; ii++)
                    {
                        new_weights[ii] = _weights[(ii - 1) % _poles.Length() + 1];
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
        uTolerance = tolerance3D * _maxDerivInv;
    }

    /// <summary>
    /// True if theOther has the same degree, rationality, knots (within Precision::Parametric(thePreci)),
    /// multiplicities, poles (each coordinate within thePreci) and weights (within Epsilon).
    /// </summary>
    public bool IsEqual(BSplineCurve3D other, double tolerance)
    {
        if (_knots.IsEmpty() || _poles.IsEmpty() || _mults.IsEmpty() || _deg != other.Degree
            || _rational != other.IsRational || _knots.Length() != other.KnotCount
            || _poles.Length() != other.PoleCount)
        {
            return false;
        }

        for (int poleIter = 1; poleIter <= _poles.Length(); ++poleIter)
        {
            Point3D pole1 = _poles[poleIter];
            Point3D pole2 = other.Pole(poleIter);
            if (Math.Abs(pole1.X - pole2.X) > tolerance || Math.Abs(pole1.Y - pole2.Y) > tolerance
                || Math.Abs(pole1.Z - pole2.Z) > tolerance)
            {
                return false;
            }
        }

        for (int knotIter = 1; knotIter <= _knots.Length(); ++knotIter)
        {
            if (Math.Abs(_knots[knotIter] - other.Knot(knotIter)) > global::Kerf.Engine.Foundation.Precision.Parametric(tolerance))
            {
                return false;
            }
        }

        for (int multIter = 1; multIter <= _mults.Length(); ++multIter)
        {
            if (_mults[multIter] != other.Multiplicity(multIter))
            {
                return false;
            }
        }

        if (!_rational)
        {
            return true;
        }

        for (int weightIter = 1; weightIter <= _weights.Length(); ++weightIter)
        {
            if (Math.Abs((double)(_weights[weightIter] - other.Weight(weightIter)))
                > Epsilon(_weights[weightIter]))
            {
                return false;
            }
        }

        return true;
    }

    //=============================================================================================
    // Internal access to the stored arrays (Kerf)
    //=============================================================================================

    /// <summary>The stored poles: shared, don't modify (OCCT's <c>Poles()</c> const reference).</summary>
    internal NCollection_Array1<Point3D> InternalPoles() => _poles;

    /// <summary>The stored weights, or null for a non-rational curve (OCCT's <c>Weights()</c> pointer). Don't modify.</summary>
    internal NCollection_Array1<double>? InternalWeights() => IsRational ? _weights : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights();

    /// <summary>The weights, always NbPoles of them (OCCT's <c>WeightsArray()</c>): shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalWeightsArray() => _weights;

    /// <summary>The stored knots: shared, don't modify (OCCT's <c>Knots()</c> const reference).</summary>
    internal NCollection_Array1<double> InternalKnots() => _knots;

    /// <summary>The stored multiplicities: shared, don't modify (OCCT's <c>Multiplicities()</c> const reference).</summary>
    internal NCollection_Array1<int> InternalMultiplicities() => _mults;

    /// <summary>The stored knot sequence: shared, don't modify (OCCT's <c>KnotSequence()</c> const reference).</summary>
    internal NCollection_Array1<double> InternalKnotSequence() => _flatKnots;

    //=============================================================================================
    // Non-mutating twins (Kerf)
    //=============================================================================================

    private BSplineCurve3D CopyBSpline() => (BSplineCurve3D)Copy();

    /// <summary>A copy with the degree raised to Degree.</summary>
    public BSplineCurve3D DegreeIncreased(int degree)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.IncreaseDegree(degree);
        return copy;
    }

    /// <summary>A copy with the multiplicity of the knot Index raised to M.</summary>
    internal BSplineCurve3D MultiplicityIncreased(int index, int m)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.IncreaseMultiplicity(index, m);
        return copy;
    }

    /// <summary>A copy with the multiplicities of the knots I1 to I2 raised to M.</summary>
    internal BSplineCurve3D MultiplicityIncreased(int i1, int i2, int m)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.IncreaseMultiplicity(i1, i2, m);
        return copy;
    }

    /// <summary>A copy with the multiplicities of the knots I1 to I2 raised by Step.</summary>
    internal BSplineCurve3D MultiplicityIncremented(int i1, int i2, int step)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.IncrementMultiplicity(i1, i2, step);
        return copy;
    }

    /// <summary>A copy with the knot U inserted (see <see cref="InsertKnot"/>).</summary>
    internal BSplineCurve3D KnotInserted(double u, int m = 1, double parametricTolerance = 0.0, bool add = true)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.InsertKnot(u, m, parametricTolerance, add);
        return copy;
    }

    /// <summary>A copy with the knots inserted (see <see cref="InsertKnots"/>; Add defaults to false).</summary>
    internal BSplineCurve3D KnotsInserted(NCollection_Array1<double> knots,
                                           NCollection_Array1<int> mults,
                                           double parametricTolerance = 0.0,
                                           bool add = false)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.InsertKnots(knots, mults, parametricTolerance, add);
        return copy;
    }

    /// <summary>
    /// A copy with the multiplicity of the knot Index lowered to M (see <see cref="RemoveKnot"/>), or null if
    /// the curve would move by more than Tolerance.
    /// </summary>
    internal BSplineCurve3D? KnotRemoved(int index, int m, double tolerance)
    {
        BSplineCurve3D copy = CopyBSpline();
        return copy.RemoveKnot(index, m, tolerance) ? copy : null;
    }

    /// <summary>A copy segmented between U1 and U2 (see <see cref="Segment"/>).</summary>
    public BSplineCurve3D Segmented(double u1, double u2, double tolerance = Precision_PConfusion)
    {
        BSplineCurve3D copy = CopyBSpline();
        try
        {
            copy.Segment(u1, u2, tolerance);
        }
        catch (FailureException failure)
        {
            throw Refusal.Argument(failure);
        }
        return copy;
    }

    internal BSplineCurve3D WithKnotOneBased(int index, double k)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.SetKnot(index, k);
        return copy;
    }

    internal BSplineCurve3D WithKnots(NCollection_Array1<double> k)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.SetKnots(k);
        return copy;
    }

    internal BSplineCurve3D WithKnotOneBased(int index, double k, int m)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.SetKnot(index, k, m);
        return copy;
    }

    /// <summary>A periodic copy (see <see cref="SetPeriodic"/>).</summary>
    public BSplineCurve3D WithPeriodic()
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.SetPeriodic();
        return copy;
    }

    internal BSplineCurve3D WithOrigin(int index)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.SetOrigin(index);
        return copy;
    }

    internal BSplineCurve3D WithOrigin(double u, double tol)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.SetOrigin(u, tol);
        return copy;
    }

    /// <summary>A non-periodic copy (see <see cref="SetNotPeriodic"/>).</summary>
    public BSplineCurve3D WithNotPeriodic()
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.SetNotPeriodic();
        return copy;
    }

    internal BSplineCurve3D WithPoleOneBased(int index, in Point3D p)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.SetPole(index, p);
        return copy;
    }

    internal BSplineCurve3D WithPoleOneBased(int index, in Point3D p, double w)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.SetPole(index, p, w);
        return copy;
    }

    internal BSplineCurve3D WithWeightOneBased(int index, double w)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.SetWeight(index, w);
        return copy;
    }

    /// <summary>A copy whose point of parameter U is moved to P (see <see cref="MovePoint"/>).</summary>
    internal BSplineCurve3D PointMoved(double u,
                                        in Point3D p,
                                        int index1,
                                        int index2,
                                        out int firstModifiedPole,
                                        out int lastModifiedPole)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.MovePoint(u, p, index1, index2, out firstModifiedPole, out lastModifiedPole);
        return copy;
    }

    /// <summary>A copy whose point and tangent at U are moved (see <see cref="MovePointAndTangent"/>).</summary>
    internal BSplineCurve3D PointAndTangentMoved(double u,
                                                  in Point3D p,
                                                  in Vector3D tangent,
                                                  double tolerance,
                                                  int startingCondition,
                                                  int endingCondition,
                                                  out int errorStatus)
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.MovePointAndTangent(u, p, tangent, tolerance, startingCondition, endingCondition, out errorStatus);
        return copy;
    }

    /// <summary>A copy evaluated through a copy of theDesc (null: without a representation).</summary>
    internal BSplineCurve3D WithEvalRepresentation(RepCurveDesc.Base? desc)
    {
        RepUtils.ValidateCurveDesc(desc, this);
        BSplineCurve3D copy = CopyBSpline();
        copy.SetEvalRepresentation(RepUtils.CloneCurveDesc(desc));
        return copy;
    }

    /// <summary>A copy without an evaluation representation (the twin of ClearEvalRepresentation).</summary>
    internal BSplineCurve3D WithoutEvalRepresentation()
    {
        BSplineCurve3D copy = CopyBSpline();
        copy.ClearEvalRepresentation();
        return copy;
    }
}
