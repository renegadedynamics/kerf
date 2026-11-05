// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_BSpline.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_BSpline.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Splines;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.BSplKnotDistribution;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Geometry.Splines.KnotDistribution;
using static Kerf.Engine.Geometry.Splines.MultDistribution;

namespace Kerf.Engine.Construction.Laws;

/// <summary>
/// A one-dimensional B-spline curve (a real function of one parameter), rational or not, periodic or not: the
/// basis of the B-spline evolution laws. Its definition and its knot operations are those of Geom2d_BSplineCurve
/// with real poles.
/// </summary>
/// <remarks>
/// <para>
/// Geometry (rules, section 8): the mutators are internal and check that the curve is not frozen; callers use
/// the twins (<see cref="DegreeIncreased"/>, <see cref="KnotInserted"/>, <see cref="WithPole(int, double)"/>,
/// ...), which change a copy. <see cref="Copy"/> returns an unfrozen curve. <see cref="BSpFunc"/> freezes the
/// curve it keeps.
/// </para>
/// <para>
/// The arrays are returned to callers as <see cref="IReadOnlyList{T}"/> copies (<see cref="Poles()"/>,
/// <see cref="Knots()"/>, ...), and to ported code through the internal <c>Internal...()</c> accessors, which must
/// not be modified (decision 131). They are indexed from 1, whatever the bounds of the arrays given (OCCT 8 would
/// copy those bounds, occt-issues.md).
/// </para>
/// </remarks>
internal sealed class BSpline : Transient
{
    private bool _rational;
    private bool _periodic;
    private BSplKnotDistribution _knotSet;
    private Continuity _smooth;
    private int _deg;
    private NCollection_HArray1<double> _poles;
    private NCollection_HArray1<double>? _weights;
    private NCollection_HArray1<double> _flatknots;
    private NCollection_HArray1<double> _knots;
    private NCollection_HArray1<int> _mults;

    private static void SetPoles(NCollection_Array1<double> poles, NCollection_Array1<double> weights,
                                 NCollection_Array1<double> fp)
    {
        int i, j = fp.Lower();
        for (i = poles.Lower(); i <= poles.Upper(); i++)
        {
            double w = weights[i];
            fp[j] = poles[i] * w;
            j++;
            fp[j] = w;
            j++;
        }
    }

    private static void GetPoles(NCollection_Array1<double> fp, NCollection_Array1<double> poles,
                                 NCollection_Array1<double> weights)
    {
        int i, j = fp.Lower();
        for (i = poles.Lower(); i <= poles.Upper(); i++)
        {
            double w = fp[j + 1];
            weights[i] = w;
            poles[i] = fp[j] / w;
            j += 2;
        }
    }

    private static void CheckCurveData(NCollection_Array1<double> cPoles, NCollection_Array1<double> cKnots,
                                       NCollection_Array1<int> cMults, int degree, bool periodic)
    {
        if (degree < 1 || degree > BSpline.MaxDegree())
        {
            throw new ConstructionErrorException();
        }

        if (cPoles.Length() < 2)
        {
            throw new ConstructionErrorException();
        }
        if (cKnots.Length() != cMults.Length())
        {
            throw new ConstructionErrorException();
        }

        for (int i = cKnots.Lower(); i < cKnots.Upper(); i++)
        {
            if (cKnots[i + 1] - cKnots[i] <= Epsilon(Math.Abs(cKnots[i])))
            {
                throw new ConstructionErrorException();
            }
        }

        if (cPoles.Length() != global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(degree, periodic, cMults))
        {
            throw new ConstructionErrorException();
        }
    }

    private static void KnotAnalysis(int degree, bool periodic, NCollection_Array1<double> cKnots,
                                     NCollection_Array1<int> cMults, out BSplKnotDistribution knotForm,
                                     out int maxKnotMult)
    {
        knotForm = BSplKnotDistribution.NonUniform;

        KnotDistribution kSet = global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotForm(cKnots, 1, cKnots.Length());

        if (kSet == Kerf.Engine.Geometry.Splines.KnotDistribution.Uniform)
        {
            MultDistribution mSet = global::Kerf.Engine.Geometry.Splines.BSplCLib.MultForm(cMults, 1, cMults.Length());
            switch (mSet)
            {
                case NonConstant:
                    break;
                case MultDistribution.Constant:
                    if (cKnots.Length() == 2)
                    {
                        knotForm = PiecewiseBezier;
                    }
                    else
                    {
                        if (cMults[1] == 1)
                        {
                            knotForm = BSplKnotDistribution.Uniform;
                        }
                    }
                    break;
                case QuasiConstant:
                    if (cMults[1] == degree + 1)
                    {
                        double m = cMults[2];
                        if (m == degree)
                        {
                            knotForm = PiecewiseBezier;
                        }
                        else if (m == 1)
                        {
                            knotForm = QuasiUniform;
                        }
                    }
                    break;
            }
        }

        int firstKM = periodic ? cKnots.Lower() : global::Kerf.Engine.Geometry.Splines.BSplCLib.FirstUKnotIndex(degree, cMults);
        int lastKM = periodic ? cKnots.Upper() : global::Kerf.Engine.Geometry.Splines.BSplCLib.LastUKnotIndex(degree, cMults);
        maxKnotMult = 0;
        if (lastKM - firstKM != 1)
        {
            int multi;
            for (int i = firstKM + 1; i < lastKM; i++)
            {
                multi = cMults[i];
                maxKnotMult = Max(maxKnotMult, multi);
            }
        }
    }

    /// <summary>Whether an array of weights is really rational.</summary>
    private static bool Rational(NCollection_Array1<double> w)
    {
        // OCCT reads W from index 1 whatever its lower bound
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

    /// <summary>An unfrozen copy of the curve.</summary>
    public BSpline Copy()
    {
        BSpline c;
        if (IsRational)
        {
            c = new BSpline(_poles.Array1(), _weights!.Array1(), _knots.Array1(), _mults.Array1(), _deg, _periodic);
        }
        else
        {
            c = new BSpline(_poles.Array1(), _knots.Array1(), _mults.Array1(), _deg, _periodic);
        }
        return c;
    }

    /// <summary>A non-rational B-spline of the given poles, knots, multiplicities and degree.</summary>
    /// <exception cref="ConstructionErrorException">The data don't define a B-spline (see Geom2d_BSplineCurve).</exception>
    public BSpline(NCollection_Array1<double> poles, NCollection_Array1<double> knots, NCollection_Array1<int> mults,
                       int degree, bool periodic = false)
    {
        _rational = false;
        _periodic = periodic;
        _deg = degree;

        // check

        CheckCurveData(poles, knots, mults, degree, periodic);

        // copy arrays (OCCT 8's array assignment would copy the arguments' bounds; the port keeps the arrays indexed
        // from 1, as OCCT allocates them and as before OCCT 8)

        _poles = new NCollection_HArray1<double>(1, poles.Length());
        _poles.ChangeArray1().CopyValues(poles);

        _knots = new NCollection_HArray1<double>(1, knots.Length());
        _knots.ChangeArray1().CopyValues(knots);

        _mults = new NCollection_HArray1<int>(1, mults.Length());
        _mults.ChangeArray1().CopyValues(mults);

        _flatknots = _knots;
        UpdateKnots();
    }

    /// <summary>A rational B-spline (non-rational if all the weights are equal).</summary>
    /// <exception cref="ConstructionErrorException">The data don't define a B-spline, or a weight is not positive.</exception>
    public BSpline(NCollection_Array1<double> poles, NCollection_Array1<double> weights,
                       NCollection_Array1<double> knots, NCollection_Array1<int> mults, int degree,
                       bool periodic = false)
    {
        _rational = true;
        _periodic = periodic;
        _deg = degree;

        // check

        CheckCurveData(poles, knots, mults, degree, periodic);

        if (weights.Length() != poles.Length())
        {
            throw new ConstructionErrorException("Law_BSpline");
        }

        int i;
        for (i = weights.Lower(); i <= weights.Upper(); i++)
        {
            if (weights[i] <= Constants.Resolution())
            {
                throw new ConstructionErrorException("Law_BSpline");
            }
        }

        // check really rational
        _rational = Rational(weights);

        // copy arrays (indexed from 1, as above)

        _poles = new NCollection_HArray1<double>(1, poles.Length());
        _poles.ChangeArray1().CopyValues(poles);
        if (_rational)
        {
            _weights = new NCollection_HArray1<double>(1, weights.Length());
            _weights.ChangeArray1().CopyValues(weights);
        }

        _knots = new NCollection_HArray1<double>(1, knots.Length());
        _knots.ChangeArray1().CopyValues(knots);

        _mults = new NCollection_HArray1<int>(1, mults.Length());
        _mults.ChangeArray1().CopyValues(mults);

        _flatknots = _knots;
        UpdateKnots();
    }

    public static int MaxDegree() => global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree();

    /// <summary>Raises the degree to Degree (no change if it is the current degree).</summary>
    internal void IncreaseDegree(int degree)
    {
        CheckMutable();
        if (degree == _deg)
        {
            return;
        }

        if (degree < _deg || degree > BSpline.MaxDegree())
        {
            throw new ConstructionErrorException();
        }

        int fromK1 = FirstUKnotIndex();
        int toK2 = LastUKnotIndex();

        int step = degree - _deg;

        NCollection_HArray1<double> npoles = new NCollection_HArray1<double>(1, _poles.Length() + step * (toK2 - fromK1));

        int nbknots = global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegreeCountKnots(_deg, degree, _periodic, _mults.Array1());

        NCollection_HArray1<double> nknots = new NCollection_HArray1<double>(1, nbknots);

        NCollection_HArray1<int> nmults = new NCollection_HArray1<int>(1, nbknots);

        NCollection_HArray1<double>? nweights = null;

        if (IsRational)
        {
            nweights = new NCollection_HArray1<double>(1, npoles.Upper());
            NCollection_Array1<double> adimpol = new NCollection_Array1<double>(1, 2 * _poles.Upper());
            SetPoles(_poles.Array1(), _weights!.Array1(), adimpol);
            NCollection_Array1<double> adimnpol = new NCollection_Array1<double>(1, 2 * npoles.Upper());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(_deg, degree, _periodic, 2, adimpol, _knots.Array1(), _mults.Array1(), adimnpol,
                                    nknots.ChangeArray1(), nmults.ChangeArray1());
            GetPoles(adimnpol, npoles.ChangeArray1(), nweights.ChangeArray1());
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(_deg, degree, _periodic, 1, _poles.Array1(), _knots.Array1(), _mults.Array1(),
                                    npoles.ChangeArray1(), nknots.ChangeArray1(), nmults.ChangeArray1());
        }

        _deg = degree;
        _poles = npoles;
        _weights = nweights;
        _knots = nknots;
        _mults = nmults;
        UpdateKnots();
    }

    /// <summary>Raises the multiplicity of the knot Index to M.</summary>
    internal void IncreaseMultiplicity(int index, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = _knots.Value(index);
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M - _mults.Value(index);
        // OCCT inserts with Add = false (Geom's IncreaseMultiplicity passes true): the multiplicity becomes
        // Max(old, M - old), not M
        InsertKnots(k, m, Epsilon(1.0));
    }

    /// <summary>Raises the multiplicities of the knots I1 to I2 to M.</summary>
    internal void IncreaseMultiplicity(int i1, int i2, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(_knots.Array1(), i1, i1, i2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(i1, i2);
        int i;
        for (i = i1; i <= i2; i++)
        {
            m[i] = M - _mults.Value(i);
        }
        // OCCT inserts with Add = false (Geom passes true): the multiplicities become Max(old, M - old), not M
        InsertKnots(k, m, Epsilon(1.0));
    }

    /// <summary>Increments the multiplicities of the knots I1 to I2 by Step.</summary>
    internal void IncrementMultiplicity(int i1, int i2, int step)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(_knots.Array1(), i1, i1, i2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(i1, i2);
        m.Init(step);
        // OCCT inserts with Add = false (Geom passes true): the multiplicities become Max(old, Step), not old + Step
        InsertKnots(k, m, Epsilon(1.0));
    }

    /// <summary>Inserts the knot U with multiplicity M (adding to an existing knot's multiplicity if Add).</summary>
    internal void InsertKnot(double u, int M = 1, double parametricTolerance = 0.0, bool add = true)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = u;
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M;
        InsertKnots(k, m, parametricTolerance, add);
    }

    /// <summary>Inserts the knots Knots with the multiplicities Mults.</summary>
    /// <exception cref="ConstructionErrorException">The knots can't be inserted.</exception>
    internal void InsertKnots(NCollection_Array1<double> knots, NCollection_Array1<int> mults, double epsilon = 0.0,
                              bool add = false)
    {
        CheckMutable();
        // Check and compute new sizes
        int nbpoles, nbknots;

        if (!global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareInsertKnots(_deg, _periodic, _knots.Array1(), _mults.Array1(), knots, mults, out nbpoles,
                                         out nbknots, epsilon, add))
        {
            throw new ConstructionErrorException("Law_BSpline::InsertKnots");
        }

        if (nbpoles == _poles.Length())
        {
            return;
        }

        NCollection_HArray1<double> npoles = new NCollection_HArray1<double>(1, nbpoles);
        NCollection_HArray1<double> nknots = _knots;
        NCollection_HArray1<int> nmults = _mults;

        if (nbknots != _knots.Length())
        {
            nknots = new NCollection_HArray1<double>(1, nbknots);
            nmults = new NCollection_HArray1<int>(1, nbknots);
        }

        if (_rational)
        {
            NCollection_HArray1<double> nweights = new NCollection_HArray1<double>(1, nbpoles);
            NCollection_Array1<double> adimpol = new NCollection_Array1<double>(1, 2 * _poles.Upper());
            SetPoles(_poles.Array1(), _weights!.Array1(), adimpol);
            NCollection_Array1<double> adimnpol = new NCollection_Array1<double>(1, 2 * npoles.Upper());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(_deg, _periodic, 2, adimpol, _knots.Array1(), _mults.Array1(), knots, mults, adimnpol,
                                 nknots.ChangeArray1(), nmults.ChangeArray1(), epsilon, add);
            GetPoles(adimnpol, npoles.ChangeArray1(), nweights.ChangeArray1());
            _weights = nweights;
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(_deg, _periodic, 1, _poles.Array1(), _knots.Array1(), _mults.Array1(), knots, mults,
                                 npoles.ChangeArray1(), nknots.ChangeArray1(), nmults.ChangeArray1(), epsilon, add);
        }

        _poles = npoles;
        _knots = nknots;
        _mults = nmults;
        UpdateKnots();
    }

    /// <summary>
    /// Lowers the multiplicity of the knot Index to M (0 removes it) if the curve stays within Tolerance;
    /// returns false otherwise.
    /// </summary>
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
            throw new OutOfRangeException();
        }
        else if (_periodic && (index < i1 || index > i2))
        {
            throw new OutOfRangeException();
        }

        NCollection_Array1<double> oldpoles = _poles.Array1();
        int step = _mults.Value(index) - m;
        if (step <= 0)
        {
            return true;
        }

        NCollection_HArray1<double> npoles = new NCollection_HArray1<double>(1, oldpoles.Length() - step);

        NCollection_HArray1<double> nknots = _knots;
        NCollection_HArray1<int> nmults = _mults;

        if (m == 0)
        {
            nknots = new NCollection_HArray1<double>(1, _knots.Length() - 1);
            nmults = new NCollection_HArray1<int>(1, _knots.Length() - 1);
        }

        if (IsRational)
        {
            NCollection_HArray1<double> nweights = new NCollection_HArray1<double>(1, npoles.Length());
            NCollection_Array1<double> adimpol = new NCollection_Array1<double>(1, 2 * _poles.Upper());
            SetPoles(_poles.Array1(), _weights!.Array1(), adimpol);
            NCollection_Array1<double> adimnpol = new NCollection_Array1<double>(1, 2 * npoles.Upper());
            if (!global::Kerf.Engine.Geometry.Splines.BSplCLib.RemoveKnot(index, m, _deg, _periodic, 2, adimpol, _knots.Array1(), _mults.Array1(), adimnpol,
                                     nknots.ChangeArray1(), nmults.ChangeArray1(), tolerance))
            {
                return false;
            }
            GetPoles(adimnpol, npoles.ChangeArray1(), nweights.ChangeArray1());
            _weights = nweights;
        }
        else
        {
            if (!global::Kerf.Engine.Geometry.Splines.BSplCLib.RemoveKnot(index, m, _deg, _periodic, 1, _poles.Array1(), _knots.Array1(), _mults.Array1(),
                                     npoles.ChangeArray1(), nknots.ChangeArray1(), nmults.ChangeArray1(), tolerance))
            {
                return false;
            }
        }

        _poles = npoles;
        _knots = nknots;
        _mults = nmults;

        UpdateKnots();
        return true;
    }

    /// <summary>Reverses the parametrization (the knots, multiplicities, poles and weights).</summary>
    internal void Reverse()
    {
        CheckMutable();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(_knots.ChangeArray1());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(_mults.ChangeArray1());
        int last;
        if (_periodic)
        {
            last = _flatknots.Upper() - _deg - 1;
        }
        else
        {
            last = _poles.Upper();
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(_poles.ChangeArray1(), last);
        if (_rational)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(_weights!.ChangeArray1(), last);
        }
        UpdateKnots();
    }

    /// <summary>The parameter on the reversed curve of the point of parameter U.</summary>
    public double ReversedParameter(double u) => (FirstParameter() + LastParameter() - u);

    /// <summary>Limits the curve to [U1, U2] (a periodic curve becomes non-periodic).</summary>
    internal void Segment(double u1, double u2)
    {
        CheckMutable();
        Standard_DomainError_Raise_if(u2 < u1, "Law_BSpline::Segment");
        double eps = Epsilon(Max(Math.Abs(u1), Math.Abs(u2)));
        double delta = u2 - u1;

        double newU1, newU2;
        double u;
        int index;

        NCollection_Array1<double> knots = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int> mults = new NCollection_Array1<int>(1, 2);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _knots.Array1(), _mults.Array1(), u1, _periodic, _knots.Lower(), _knots.Upper(),
                                 out index, out newU1);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _knots.Array1(), _mults.Array1(), u2, _periodic, _knots.Lower(), _knots.Upper(),
                                 out index, out newU2);
        knots[1] = Min(newU1, newU2);
        knots[2] = Max(newU1, newU2);
        mults[1] = mults[2] = _deg;
        InsertKnots(knots, mults, eps);

        if (_periodic)
        { // set the origine at NewU1
            int index0;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _knots.Array1(), _mults.Array1(), u1, _periodic, _knots.Lower(), _knots.Upper(),
                                     out index0, out u);
            if (Math.Abs(_knots.Value(index0 + 1) - u) < eps)
            {
                index0++;
            }
            SetOrigin(index0);
            SetNotPeriodic();
        }

        // compute index1 and index2 to set the new knots and mults
        int index1, index2;
        int fromU1 = _knots.Lower();
        int toU2 = _knots.Upper();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _knots.Array1(), _mults.Array1(), newU1, _periodic, fromU1, toU2, out index1, out u);
        // OCCT: NewU1 + delta can round one ulp below the knot inserted at U2, and the strict test against Eps
        // (Epsilon(max(|U1|, |U2|))) then misses it, so the segment ends at the previous knot
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _knots.Array1(), _mults.Array1(), newU1 + delta, _periodic, fromU1, toU2,
                                 out index2, out u);
        if (Math.Abs(_knots.Value(index2 + 1) - u) < eps)
        {
            index2++;
        }

        int nbknots = index2 - index1 + 1;

        NCollection_HArray1<double> nknots = new NCollection_HArray1<double>(1, nbknots);
        NCollection_HArray1<int> nmults = new NCollection_HArray1<int>(1, nbknots);

        int i, k = 1;
        for (i = index1; i <= index2; i++)
        {
            nknots.SetValue(k, _knots.Value(i));
            nmults.SetValue(k, _mults.Value(i));
            k++;
        }
        nmults.SetValue(1, _deg + 1);
        nmults.SetValue(nbknots, _deg + 1);

        // compute index1 and index2 to set the new poles and weights
        int pindex1 = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(_deg, index1, _periodic, _mults.Array1());
        int pindex2 = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(_deg, index2, _periodic, _mults.Array1());

        pindex1++;
        pindex2 = Min(pindex2 + 1, _poles.Length());

        int nbpoles = pindex2 - pindex1 + 1;

        NCollection_HArray1<double> nweights = new NCollection_HArray1<double>(1, nbpoles);
        NCollection_HArray1<double> npoles = new NCollection_HArray1<double>(1, nbpoles);

        k = 1;
        if (_rational)
        {
            nweights = new NCollection_HArray1<double>(1, nbpoles);
            for (i = pindex1; i <= pindex2; i++)
            {
                npoles.SetValue(k, _poles.Value(i));
                nweights.SetValue(k, _weights!.Value(i));
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

        _knots = nknots;
        _mults = nmults;
        _poles = npoles;
        if (_rational)
        {
            _weights = nweights;
        }

        UpdateKnots();
    }

    /// <summary>Changes the value of the knot Index to K.</summary>
    /// <exception cref="ConstructionErrorException">K is not between the neighbouring knots.</exception>
    internal void SetKnot(int index, double k)
    {
        CheckMutable();
        if (index < 1 || index > _knots.Length())
        {
            throw new OutOfRangeException();
        }
        double dk = Math.Abs(Epsilon(k));
        if (index == 1)
        {
            if (k >= _knots.Value(2) - dk)
            {
                throw new ConstructionErrorException();
            }
        }
        else if (index == _knots.Length())
        {
            if (k <= _knots.Value(_knots.Length() - 1) + dk)
            {
                throw new ConstructionErrorException();
            }
        }
        else
        {
            if (k <= _knots.Value(index - 1) + dk || k >= _knots.Value(index + 1) - dk)
            {
                throw new ConstructionErrorException();
            }
        }
        if (k != _knots.Value(index))
        {
            _knots.SetValue(index, k);
            UpdateKnots();
        }
    }

    /// <summary>Changes all the knots.</summary>
    internal void SetKnots(NCollection_Array1<double> k)
    {
        CheckMutable();
        CheckCurveData(_poles.Array1(), k, _mults.Array1(), _deg, _periodic);
        _knots.ChangeArray1().CopyValues(k);
        UpdateKnots();
    }

    /// <summary>Raises the multiplicity of the knot Index to M and changes its value to K.</summary>
    internal void SetKnot(int index, double k, int m)
    {
        CheckMutable();
        IncreaseMultiplicity(index, m);
        SetKnot(index, k);
    }

    /// <summary>Brings U into the period of a periodic curve.</summary>
    public void PeriodicNormalization(ref double parameter)
    {
        double period;

        if (_periodic)
        {
            period = _flatknots.Value(_flatknots.Upper() - _deg) - _flatknots.Value(_deg + 1);
            while (parameter > _flatknots.Value(_flatknots.Upper() - _deg))
            {
                parameter -= period;
            }
            while (parameter < _flatknots.Value((_deg + 1)))
            {
                parameter += period;
            }
        }
    }

    /// <summary>Makes the curve periodic, keeping the knots from FirstUKnotIndex to LastUKnotIndex.</summary>
    internal void SetPeriodic()
    {
        CheckMutable();
        int first = FirstUKnotIndex();
        int last = LastUKnotIndex();

        // OCCT 8's array assignment copies the bounds, so OCCT's new knots and multiplicities are indexed from
        // first, which the 1-based reads that follow (KnotForm) overrun when first > 1; the port keeps them
        // indexed from 1 (CopyValues), as before OCCT 8
        NCollection_Array1<double> cknots = new NCollection_Array1<double>(_knots.Array1(), first, first, last);
        _knots = new NCollection_HArray1<double>(1, cknots.Length());
        _knots.ChangeArray1().CopyValues(cknots);

        NCollection_Array1<int> cmults = new NCollection_Array1<int>(_mults.Array1(), first, first, last);
        cmults[first] = cmults[last] = Max(cmults[first], cmults[last]);
        _mults = new NCollection_HArray1<int>(1, cmults.Length());
        _mults.ChangeArray1().CopyValues(cmults);

        // compute new number of poles;
        int nbp = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(_deg, true, cmults);

        NCollection_Array1<double> cpoles = new NCollection_Array1<double>(_poles.Array1(), 1, 1, nbp);
        _poles = new NCollection_HArray1<double>(1, nbp);
        _poles.ChangeArray1().CopyValues(cpoles);

        if (_rational)
        {
            NCollection_Array1<double> cweights = new NCollection_Array1<double>(_weights!.Array1(), 1, 1, nbp);
            _weights = new NCollection_HArray1<double>(1, nbp);
            _weights.ChangeArray1().CopyValues(cweights);
        }

        _periodic = true;

        UpdateKnots();
    }

    /// <summary>Sets the origin of a periodic curve at the knot Index.</summary>
    internal void SetOrigin(int Index)
    {
        CheckMutable();
        Standard_NoSuchObject_Raise_if(!_periodic, "Law_BSpline::SetOrigin");
        int i, k;
        int first = FirstUKnotIndex();
        int last = LastUKnotIndex();

        Standard_DomainError_Raise_if((Index < first) || (Index > last), "Law_BSpline::SetOrigine");

        int nbknots = _knots.Length();
        int nbpoles = _poles.Length();

        NCollection_HArray1<double> nknots = new NCollection_HArray1<double>(1, nbknots);
        NCollection_Array1<double> newknots = nknots.ChangeArray1();

        NCollection_HArray1<int> nmults = new NCollection_HArray1<int>(1, nbknots);
        NCollection_Array1<int> newmults = nmults.ChangeArray1();

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
        NCollection_HArray1<double> npoles = new NCollection_HArray1<double>(1, nbpoles);
        NCollection_HArray1<double> nweights = new NCollection_HArray1<double>(1, nbpoles);
        NCollection_Array1<double> newpoles = npoles.ChangeArray1();
        NCollection_Array1<double> newweights = nweights.ChangeArray1();
        first = _poles.Lower();
        last = _poles.Upper();
        if (_rational)
        {
            k = 1;
            for (i = index; i <= last; i++)
            {
                newpoles[k] = _poles.Value(i);
                newweights[k] = _weights!.Value(i);
                k++;
            }
            for (i = first; i < index; i++)
            {
                newpoles[k] = _poles.Value(i);
                newweights[k] = _weights!.Value(i);
                k++;
            }
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
        }

        _poles = npoles;
        _knots = nknots;
        _mults = nmults;
        if (_rational)
        {
            _weights = nweights;
        }
        UpdateKnots();
    }

    /// <summary>Makes a periodic curve non-periodic (the same curve, with clamped ends).</summary>
    internal void SetNotPeriodic()
    {
        CheckMutable();
        if (_periodic)
        {
            int nbKnots, nbPoles;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareUnperiodize(_deg, _mults.Array1(), out nbKnots, out nbPoles);

            NCollection_HArray1<double> npoles = new NCollection_HArray1<double>(1, nbPoles);

            NCollection_HArray1<double> nknots = new NCollection_HArray1<double>(1, nbKnots);

            NCollection_HArray1<int> nmults = new NCollection_HArray1<int>(1, nbKnots);

            NCollection_HArray1<double>? nweights = null;

            if (IsRational)
            {
                nweights = new NCollection_HArray1<double>(1, nbPoles);

                NCollection_Array1<double> adimpol = new NCollection_Array1<double>(1, 2 * _poles.Upper());
                SetPoles(_poles.Array1(), _weights!.Array1(), adimpol);
                NCollection_Array1<double> adimnpol = new NCollection_Array1<double>(1, 2 * npoles.Upper());
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Unperiodize(_deg, 1, _mults.Array1(), _knots.Array1(), adimpol, nmults.ChangeArray1(),
                                     nknots.ChangeArray1(), adimnpol);
                GetPoles(adimnpol, npoles.ChangeArray1(), nweights.ChangeArray1());
            }
            else
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Unperiodize(_deg, 1, _mults.Array1(), _knots.Array1(), _poles.Array1(), nmults.ChangeArray1(),
                                     nknots.ChangeArray1(), npoles.ChangeArray1());
            }
            _poles = npoles;
            _weights = nweights;
            _mults = nmults;
            _knots = nknots;
            _periodic = false;

            UpdateKnots();
        }
    }

    /// <summary>Changes the pole Index to P.</summary>
    internal void SetPole(int index, double p)
    {
        CheckMutable();
        if (index < 1 || index > _poles.Length())
        {
            throw new OutOfRangeException();
        }
        _poles.SetValue(index, p);
    }

    /// <summary>Changes the pole Index to P and its weight to W.</summary>
    internal void SetPole(int index, double p, double w)
    {
        CheckMutable();
        SetPole(index, p);
        SetWeight(index, w);
    }

    /// <summary>Changes the weight of the pole Index to W.</summary>
    internal void SetWeight(int index, double w)
    {
        CheckMutable();
        if (index < 1 || index > _poles.Length())
        {
            throw new OutOfRangeException();
        }

        if (w <= Constants.Resolution())
        {
            throw new ConstructionErrorException();
        }

        bool rat = IsRational || (Math.Abs(w - 1.0) > Constants.Resolution());

        if (rat)
        {
            if (rat && !IsRational)
            {
                _weights = new NCollection_HArray1<double>(1, _poles.Length(), 1.0);
            }

            _weights!.SetValue(index, w);

            if (IsRational)
            {
                rat = Rational(_weights.Array1());
                if (!rat)
                {
                    _weights = null;
                }
            }

            _rational = _weights is not null;
        }
    }

    private void UpdateKnots()
    {

        _rational = _weights is not null;

        KnotAnalysis(_deg, _periodic, _knots.Array1(), _mults.Array1(), out _knotSet, out int maxKnotMult);

        if (_knotSet == BSplKnotDistribution.Uniform && !_periodic)
        {
            _flatknots = _knots;
        }
        else
        {
            _flatknots = new NCollection_HArray1<double>(1, global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequenceLength(_mults.Array1(), _deg, _periodic));

            global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(_knots.Array1(), _mults.Array1(), _deg, _periodic, _flatknots.ChangeArray1());
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

    /// <summary>Whether the curve is at least CN.</summary>
    public bool IsCN(int n)
    {
        Standard_RangeError_Raise_if(n < 0, "Law_BSpline::IsCN");

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
                return n <= 3 ? true : n <= _deg - global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxKnotMult(_mults.Array1(), _mults.Lower() + 1, _mults.Upper() - 1);
            default:
                return false;
        }
    }

    public bool IsClosed => (Math.Abs(StartPoint() - EndPoint())) <= Constants.Resolution();

    public bool IsPeriodic => _periodic;

    public Continuity Continuity() => _smooth;

    public int Degree() => _deg;

    public double Value(double u)
    {
        D0(u, out double p);
        return p;
    }

    public void D0(double u, out double p)
    {
        double newU = u;
        PeriodicNormalization(ref newU);
        if (_rational)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(newU, 0, _deg, _periodic, _poles, _weights!.Array1(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(newU, 0, _deg, _periodic, _poles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p);
        }
    }

    public void D1(double u, out double p, out double v1)
    {
        double newU = u;
        PeriodicNormalization(ref newU);
        if (_rational)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(newU, 0, _deg, _periodic, _poles, _weights!.Array1(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p, out v1);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(newU, 0, _deg, _periodic, _poles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p,
                        out v1);
        }
    }

    public void D2(double u, out double p, out double v1, out double v2)
    {
        double newU = u;
        PeriodicNormalization(ref newU);
        if (_rational)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(newU, 0, _deg, _periodic, _poles, _weights!.Array1(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p, out v1,
                        out v2);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(newU, 0, _deg, _periodic, _poles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p,
                        out v1, out v2);
        }
    }

    public void D3(double u, out double p, out double v1, out double v2, out double v3)
    {
        double newU = u;
        PeriodicNormalization(ref newU);
        if (_rational)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(newU, 0, _deg, _periodic, _poles, _weights!.Array1(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p, out v1,
                        out v2, out v3);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(newU, 0, _deg, _periodic, _poles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p,
                        out v1, out v2, out v3);
        }
    }

    /// <summary>The derivative of order N at U.</summary>
    public double DN(double u, int n)
    {
        double v;
        if (_rational)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(u, n, 0, _deg, _periodic, _poles, _weights!.Array1(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out v);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(u, n, 0, _deg, _periodic, _poles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out v);
        }
        return v;
    }

    public double EndPoint()
    {
        if (_mults.Value(_knots.Upper()) == _deg + 1)
        {
            return _poles.Value(_poles.Upper());
        }
        else
        {
            return Value(LastParameter());
        }
    }

    public int FirstUKnotIndex()
    {
        if (_periodic)
        {
            return 1;
        }
        else
        {
            return global::Kerf.Engine.Geometry.Splines.BSplCLib.FirstUKnotIndex(_deg, _mults.Array1());
        }
    }

    public double FirstParameter() => _flatknots.Value(_deg + 1);

    public double Knot(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > _knots.Length(), "Law_BSpline::Knot");
        return _knots.Value(index);
    }

    public BSplKnotDistribution KnotDistribution() => _knotSet;

    /// <summary>Copies the knots into K (OCCT's array assignment: K must have NbKnots() elements).</summary>
    public void Knots(NCollection_Array1<double> k)
    {
        Standard_DimensionError_Raise_if(k.Length() != _knots.Length(), "Law_BSpline::Knots");
        k.Assign(_knots.Array1());
    }

    /// <summary>The knots (a copy).</summary>
    public IReadOnlyList<double> Knots() => _knots.ToImmutableArray();

    /// <summary>The knot array itself, for ported code: don't modify it (decision 131).</summary>
    internal NCollection_Array1<double> InternalKnots() => _knots;

    /// <summary>Copies the flat knots into K (K must have as many elements).</summary>
    public void KnotSequence(NCollection_Array1<double> k)
    {
        Standard_DimensionError_Raise_if(k.Length() != _flatknots.Length(), "Law_BSpline::KnotSequence");
        k.Assign(_flatknots.Array1());
    }

    /// <summary>The flat knots, each knot repeated by its multiplicity (a copy).</summary>
    public IReadOnlyList<double> KnotSequence() => _flatknots.ToImmutableArray();

    /// <summary>The flat knot array itself, for ported code: don't modify it (decision 131).</summary>
    internal NCollection_Array1<double> InternalKnotSequence() => _flatknots;

    public int LastUKnotIndex()
    {
        if (_periodic)
        {
            return _knots.Length();
        }
        else
        {
            return global::Kerf.Engine.Geometry.Splines.BSplCLib.LastUKnotIndex(_deg, _mults.Array1());
        }
    }

    public double LastParameter() => _flatknots.Value(_flatknots.Upper() - _deg);

    /// <summary>The value at U computed on the span between the knots FromK1 and ToK2.</summary>
    public double LocalValue(double u, int fromK1, int toK2)
    {
        LocalD0(u, fromK1, toK2, out double p);
        return p;
    }

    public void LocalD0(double U, int fromK1, int toK2, out double p)
    {
        Standard_DomainError_Raise_if(fromK1 == toK2, "Law_BSpline::LocalValue");
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _flatknots, U, _periodic, fromK1, toK2, out int index, out double u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_deg, index, _mults.Array1(), _periodic);
        if (_rational)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, index, _deg, _periodic, _poles, _weights!.Array1(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, index, _deg, _periodic, _poles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p);
        }
    }

    public void LocalD1(double U, int fromK1, int toK2, out double p, out double v1)
    {
        Standard_DomainError_Raise_if(fromK1 == toK2, "Law_BSpline::LocalD1");
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _flatknots, U, _periodic, fromK1, toK2, out int index, out double u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_deg, index, _mults.Array1(), _periodic);
        if (_rational)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, index, _deg, _periodic, _poles, _weights!.Array1(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p, out v1);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, index, _deg, _periodic, _poles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p,
                        out v1);
        }
    }

    public void LocalD2(double U, int fromK1, int toK2, out double p, out double v1, out double v2)
    {
        Standard_DomainError_Raise_if(fromK1 == toK2, "Law_BSpline::LocalD2");
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _flatknots, U, _periodic, fromK1, toK2, out int index, out double u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_deg, index, _mults.Array1(), _periodic);
        if (_rational)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u, index, _deg, _periodic, _poles, _weights!.Array1(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p, out v1,
                        out v2);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u, index, _deg, _periodic, _poles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p,
                        out v1, out v2);
        }
    }

    public void LocalD3(double U, int fromK1, int toK2, out double p, out double v1, out double v2, out double v3)
    {
        Standard_DomainError_Raise_if(fromK1 == toK2, "Law_BSpline::LocalD3");
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _flatknots, U, _periodic, fromK1, toK2, out int index, out double u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_deg, index, _mults.Array1(), _periodic);
        if (_rational)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(u, index, _deg, _periodic, _poles, _weights!.Array1(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p, out v1,
                        out v2, out v3);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(u, index, _deg, _periodic, _poles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out p,
                        out v1, out v2, out v3);
        }
    }

    public double LocalDN(double U, int fromK1, int toK2, int n)
    {
        Standard_DomainError_Raise_if(fromK1 == toK2, "Law_BSpline::LocalD3");
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(_deg, _flatknots, U, _periodic, fromK1, toK2, out int index, out double u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(_deg, index, _mults.Array1(), _periodic);

        double v;
        if (_rational)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(u, n, index, _deg, _periodic, _poles, _weights!.Array1(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out v);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(u, n, index, _deg, _periodic, _poles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _flatknots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out v);
        }
        return v;
    }

    public int Multiplicity(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > _mults.Length(), "Law_BSpline::Multiplicity");
        return _mults.Value(index);
    }

    /// <summary>Copies the multiplicities into M (M must have NbKnots() elements).</summary>
    public void Multiplicities(NCollection_Array1<int> m)
    {
        Standard_DimensionError_Raise_if(m.Length() != _mults.Length(), "Law_BSpline::Multiplicities");
        m.Assign(_mults.Array1());
    }

    /// <summary>The multiplicities (a copy).</summary>
    public IReadOnlyList<int> Multiplicities() => _mults.ToImmutableArray();

    /// <summary>The multiplicity array itself, for ported code: don't modify it (decision 131).</summary>
    internal NCollection_Array1<int> InternalMultiplicities() => _mults;

    public int KnotCount => _knots.Length();

    public int PoleCount => _poles.Length();

    public double Pole(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > _poles.Length(), "Law_BSpline::Pole");
        return _poles.Value(index);
    }

    /// <summary>Copies the poles into P (P must have NbPoles() elements).</summary>
    public void Poles(NCollection_Array1<double> p)
    {
        Standard_DimensionError_Raise_if(p.Length() != _poles.Length(), "Law_BSpline::Poles");
        p.Assign(_poles.Array1());
    }

    /// <summary>The poles (a copy).</summary>
    public IReadOnlyList<double> Poles() => _poles.ToImmutableArray();

    /// <summary>The pole array itself, for ported code: don't modify it (decision 131).</summary>
    internal NCollection_Array1<double> InternalPoles() => _poles;

    public double StartPoint()
    {
        if (_mults.Value(1) == _deg + 1)
        {
            return _poles.Value(1);
        }
        else
        {
            return Value(FirstParameter());
        }
    }

    public double Weight(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > _poles.Length(), "Law_BSpline::Weight");
        if (IsRational)
        {
            return _weights!.Value(index);
        }
        else
        {
            return 1.0;
        }
    }

    /// <summary>Copies the weights into W (all 1 for a non-rational curve; W must have NbPoles() elements).</summary>
    public void Weights(NCollection_Array1<double> w)
    {
        Standard_DimensionError_Raise_if(w.Length() != _poles.Length(), "Law_BSpline::Weights");
        if (IsRational)
        {
            w.Assign(_weights!.Array1());
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

    /// <summary>The weights (a copy), or null for a non-rational curve.</summary>
    public IReadOnlyList<double>? Weights() => IsRational ? _weights!.ToImmutableArray() : null;

    /// <summary>The weight array itself (null if non-rational), for ported code: don't modify it (decision 131).</summary>
    internal NCollection_Array1<double>? InternalWeights() => _weights;

    public bool IsRational => _weights is not null;

    /// <summary>
    /// Locates U in the knots (the flat knots if WithKnotRepetition): Knots(I1) &lt;= U &lt;= Knots(I2), with
    /// I1 = I2 when U is on a knot within ParametricTolerance.
    /// </summary>
    public void LocateU(double u, double parametricTolerance, out int i1, out int i2, bool withKnotRepetition = false)
    {
        double newU = u;
        NCollection_HArray1<double> theKnots;
        if (withKnotRepetition)
        {
            theKnots = _flatknots;
        }
        else
        {
            theKnots = _knots;
        }

        PeriodicNormalization(ref newU); // Attention a la periode

        NCollection_Array1<double> cKnots = theKnots.Array1();
        double uFirst = cKnots[1];
        double uLast = cKnots[cKnots.Length()];
        if (Math.Abs(u - uFirst) <= Math.Abs(parametricTolerance))
        {
            i1 = i2 = 1;
        }
        else if (Math.Abs(u - uLast) <= Math.Abs(parametricTolerance))
        {
            i1 = i2 = cKnots.Length();
        }
        else if (newU < uFirst - Math.Abs(parametricTolerance))
        {
            i2 = 1;
            i1 = 0;
        }
        else if (newU > uLast + Math.Abs(parametricTolerance))
        {
            i1 = cKnots.Length();
            i2 = i1 + 1;
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(cKnots, newU, out i1);
            i1 = Max(Min(i1, cKnots.Upper()), cKnots.Lower());
            while (i1 + 1 <= cKnots.Upper() && Math.Abs(cKnots[i1 + 1] - newU) <= Math.Abs(parametricTolerance))
            {
                i1++;
            }
            if (Math.Abs(cKnots[i1] - newU) <= Math.Abs(parametricTolerance))
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
    /// Moves the value at U to NewValue and the derivative to Derivative, by changing the poles (see
    /// Geom2d_BSplineCurve.MovePointAndTangent); ErrorStatus is 0 on success, and the curve is unchanged otherwise.
    /// </summary>
    internal void MovePointAndTangent(double u, double p, double tangent, double tolerance, int startingCondition,
                                     int endingCondition, out int errorStatus)
    {
        CheckMutable();
        NCollection_Array1<double> new_poles = new NCollection_Array1<double>(1, _poles.Length());
        double delta, delta_derivative;
        const int dimension = 1;
        D1(u, out delta, out delta_derivative);
        delta = p - delta;

        delta_derivative = tangent - delta_derivative;
        ReadOnlySpan<double> aDelta = [delta];
        ReadOnlySpan<double> deltaDerivative = [delta_derivative];
        global::Kerf.Engine.Geometry.Splines.BSplCLib.MovePointAndTangent(u, dimension, aDelta, deltaDerivative, tolerance, _deg, startingCondition,
                                     endingCondition, _poles.AsSpan(), _rational ? _weights!.Array1() : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                     _flatknots.Array1(), new_poles.AsSpan(), out errorStatus);
        if (errorStatus == 0)
        {
            _poles.ChangeArray1().CopyValues(new_poles);
        }
    }

    /// <summary>The parametric tolerance UTolerance such that |t1 - t0| &lt; UTolerance implies |C(t1) - C(t0)| &lt; Tolerance3D.</summary>
    public void Resolution(double tolerance3D, out double uTolerance)
    {
        if (_rational)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Resolution(_poles.AsSpan(), 1, _poles.Length(), _weights!.Array1(), _flatknots, _deg, tolerance3D,
                                out uTolerance);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Resolution(_poles.AsSpan(), 1, _poles.Length(), global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _flatknots, _deg, tolerance3D,
                                out uTolerance);
        }
    }

    // Non-mutating twins of the mutators (OCCT.NET)

    /// <summary>A copy with its degree raised to Degree.</summary>
    public BSpline DegreeIncreased(int degree)
    {
        BSpline copy = Copy();
        copy.IncreaseDegree(degree);
        return copy;
    }

    /// <summary>A copy with the multiplicity of the knot Index raised to M.</summary>
    public BSpline MultiplicityIncreased(int index, int m)
    {
        BSpline copy = Copy();
        copy.IncreaseMultiplicity(index, m);
        return copy;
    }

    /// <summary>A copy with the multiplicities of the knots I1 to I2 raised to M.</summary>
    public BSpline MultiplicityIncreased(int i1, int i2, int m)
    {
        BSpline copy = Copy();
        copy.IncreaseMultiplicity(i1, i2, m);
        return copy;
    }

    /// <summary>A copy with the multiplicities of the knots I1 to I2 incremented by Step.</summary>
    public BSpline MultiplicityIncremented(int i1, int i2, int step)
    {
        BSpline copy = Copy();
        copy.IncrementMultiplicity(i1, i2, step);
        return copy;
    }

    /// <summary>A copy with the knot U inserted (see InsertKnot; Add defaults to true, as in OCCT).</summary>
    public BSpline KnotInserted(double u, int m = 1, double parametricTolerance = 0.0, bool add = true)
    {
        BSpline copy = Copy();
        copy.InsertKnot(u, m, parametricTolerance, add);
        return copy;
    }

    /// <summary>A copy with the knots Knots inserted (Add defaults to false, as in OCCT).</summary>
    public BSpline KnotsInserted(NCollection_Array1<double> knots, NCollection_Array1<int> mults,
                                     double parametricTolerance = 0.0, bool add = false)
    {
        BSpline copy = Copy();
        copy.InsertKnots(knots, mults, parametricTolerance, add);
        return copy;
    }

    /// <summary>A copy with the multiplicity of the knot Index lowered to M, or null if that changes the curve by more than Tolerance.</summary>
    public BSpline? KnotRemoved(int index, int m, double tolerance)
    {
        BSpline copy = Copy();
        return copy.RemoveKnot(index, m, tolerance) ? copy : null;
    }

    /// <summary>A copy with the parametrization reversed.</summary>
    public BSpline Reversed()
    {
        BSpline copy = Copy();
        copy.Reverse();
        return copy;
    }

    /// <summary>A copy limited to [U1, U2] (see Segment).</summary>
    public BSpline Segmented(double u1, double u2)
    {
        BSpline copy = Copy();
        copy.Segment(u1, u2);
        return copy;
    }

    /// <summary>A copy with the knot Index changed to K.</summary>
    public BSpline WithKnot(int index, double k)
    {
        BSpline copy = Copy();
        copy.SetKnot(index, k);
        return copy;
    }

    /// <summary>A copy with the knots K.</summary>
    public BSpline WithKnots(NCollection_Array1<double> k)
    {
        BSpline copy = Copy();
        copy.SetKnots(k);
        return copy;
    }

    /// <summary>A copy with the knot Index raised to multiplicity M and changed to K.</summary>
    public BSpline WithKnot(int index, double k, int m)
    {
        BSpline copy = Copy();
        copy.SetKnot(index, k, m);
        return copy;
    }

    /// <summary>A periodic copy (see SetPeriodic).</summary>
    public BSpline WithPeriodic()
    {
        BSpline copy = Copy();
        copy.SetPeriodic();
        return copy;
    }

    /// <summary>A copy of this periodic curve with its origin at the knot Index.</summary>
    public BSpline WithOrigin(int index)
    {
        BSpline copy = Copy();
        copy.SetOrigin(index);
        return copy;
    }

    /// <summary>A non-periodic copy (see SetNotPeriodic).</summary>
    public BSpline WithNotPeriodic()
    {
        BSpline copy = Copy();
        copy.SetNotPeriodic();
        return copy;
    }

    /// <summary>A copy with the pole Index changed to P.</summary>
    public BSpline WithPole(int index, double p)
    {
        BSpline copy = Copy();
        copy.SetPole(index, p);
        return copy;
    }

    /// <summary>A copy with the pole Index changed to P, of weight W.</summary>
    public BSpline WithPole(int index, double p, double w)
    {
        BSpline copy = Copy();
        copy.SetPole(index, p, w);
        return copy;
    }

    /// <summary>A copy with the weight of the pole Index changed to W.</summary>
    public BSpline WithWeight(int index, double w)
    {
        BSpline copy = Copy();
        copy.SetWeight(index, w);
        return copy;
    }

    /// <summary>A copy whose value and derivative at U are moved (see MovePointAndTangent).</summary>
    public BSpline PointAndTangentMoved(double u, double p, double tangent, double tolerance, int startingCondition,
                                           int endingCondition, out int errorStatus)
    {
        BSpline copy = Copy();
        copy.MovePointAndTangent(u, p, tangent, tolerance, startingCondition, endingCondition, out errorStatus);
        return copy;
    }
}
