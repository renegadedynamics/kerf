// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Profiler.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Profiler.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Profiler.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The common B-spline profile of a group of curves: <see cref="Perform"/> converts every curve to a B-spline and
/// gives them all the same degree and the same knot vector, so the same number of poles.
/// </summary>
/// <remarks>
/// A working object (decision 21). The profiler converts each curve it is given into a B-spline of its own (a copy:
/// the caller's curve is never changed) and changes those in <see cref="Perform"/>. <see cref="Curve"/> hands one out
/// frozen; a later <see cref="Perform"/> copies a frozen curve before changing it (decision 844).
/// </remarks>
internal class Profiler
{
    protected NCollection_Sequence<Curve3D> _sequence = new();
    protected bool _isDone;
    protected bool _isPeriodic;

    private static void UnifyByInsertingAllKnots(NCollection_Sequence<Curve3D> curves, double pTol)
    {
        // inserting in the first curve the knot-vector of all the others.
        BSplineCurve3D c = (BSplineCurve3D)curves[1];

        int i;
        for (i = 2; i <= curves.Length(); i++)
        {
            BSplineCurve3D ci = (BSplineCurve3D)curves[i];
            NCollection_Array1<double> ki = ci.InternalKnots();
            NCollection_Array1<int> mi = ci.InternalMultiplicities();

            c.InsertKnots(ki, mi, pTol, false);
        }

        NCollection_Array1<double> newKnots = c.InternalKnots();
        NCollection_Array1<int> newMults = c.InternalMultiplicities();
        for (i = 2; i <= curves.Length(); i++)
        {
            BSplineCurve3D ci = (BSplineCurve3D)curves[i];
            ci.InsertKnots(newKnots, newMults, pTol, false);
        }

        // essai : tentative mise des poids sur chaque section a une moyenne 1
        for (i = 1; i <= curves.Length(); i++)
        {
            BSplineCurve3D ci = (BSplineCurve3D)curves[i];
            if (ci.IsRational)
            {
                int np = ci.PoleCount;
                double sigma = 0.0;
                int j;
                for (j = 1; j <= np; j++)
                {
                    sigma += ci.Weight(j);
                }
                sigma /= np;
                for (j = 1; j <= np; j++)
                {
                    ci.SetWeight(j, ci.Weight(j) / sigma);
                }
            }
        }
        // fin de l essai
    }

    private static void UnifyBySettingMiddleKnots(NCollection_Sequence<Curve3D> curves)
    {
        int i, j;

        BSplineCurve3D c = (BSplineCurve3D)curves[1];

        int nbKnots = c.KnotCount;
        double uLast = c.Knot(c.LastUKnotIndex());
        double uFirst = c.Knot(c.FirstUKnotIndex());

        // Set middle values of knots
        NCollection_Array1<double> newKnots = new(1, nbKnots);
        newKnots[1] = uFirst;
        newKnots[nbKnots] = uLast;
        for (j = 2; j < nbKnots; j++)
        {
            double midKnot = 0.0;
            for (i = 1; i <= curves.Length(); i++)
            {
                BSplineCurve3D ctemp = (BSplineCurve3D)curves[i];
                midKnot += ctemp.Knot(j);
            }
            midKnot /= curves.Length();
            newKnots[j] = midKnot;
        }

        for (i = 1; i <= curves.Length(); i++)
        {
            BSplineCurve3D cres = (BSplineCurve3D)curves[i];
            cres.SetKnots(newKnots);
        }
    }

    public Profiler()
    {
        _isDone = false;
        _isPeriodic = true;
    }

    /// <summary>Adds a curve: conics are approximated (C1, tolerance Precision::Confusion), others converted.</summary>
    public void AddCurve(Curve3D curve)
    {
        Curve3D? c = null;
        //// modified by jgv, 19.01.05 for OCC7354 ////
        Curve3D theCurve = curve;
        if (theCurve.GetType() == typeof(TrimmedCurve3D))
        {
            theCurve = ((TrimmedCurve3D)theCurve).BasisCurve;
        }
        if (theCurve is ConicCurve3D)
        {
            ApproxCurve appr = new(curve, global::Kerf.Engine.Foundation.Precision.Confusion(), C1, 16, 14);
            if (appr.HasResult)
            {
                c = appr.InternalCurve();
            }
        }
        if (c is null)
        {
            c = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(curve);
        }
        ///////////////////////////////////////////////

        _sequence.Append(c);

        if (_isPeriodic && !c.IsPeriodic)
        {
            _isPeriodic = false;
        }
    }

    /// <summary>
    /// Converts all curves to B-spline curves and sets them to the common profile. <paramref name="pTol"/> is used
    /// to compare two knots.
    /// </summary>
    public virtual void Perform(double pTol)
    {
        int i;
        int myDegree = 0;
        BSplineCurve3D c;
        double u1, u2, uFirst = 0, uLast = 0;
        double ecartMax = 0.0;

        // The curves handed out by Curve() are frozen: change copies (decision 844).
        for (i = 1; i <= _sequence.Length(); i++)
        {
            if (_sequence[i].IsFrozen)
            {
                _sequence[i] = (Curve3D)_sequence[i].Copy();
            }
        }

        for (i = 1; i <= _sequence.Length(); i++)
        {
            c = (BSplineCurve3D)_sequence[i];

            // si non periodique, il faut deperiodiser toutes les courbes
            // on les segmente ensuite pour assurer K(1) et K(n) de multiplicite
            // degre + 1

            u2 = c.Knot(c.LastUKnotIndex());
            u1 = c.Knot(c.FirstUKnotIndex());

            if (!_isPeriodic && c.IsPeriodic)
            {
                c.SetNotPeriodic();
                c.Segment(u1, u2);
            }

            // evaluate the max degree
            myDegree = Max(myDegree, c.Degree);

            // Calcul de Max ( Ufin - Udeb) sur l ensemble des courbes.
            if ((u2 - u1) > ecartMax)
            {
                ecartMax = u2 - u1;
                uFirst = u1;
                uLast = u2;
            }
        }

        // increase the degree of the curves to my degree
        // reparametrize them in the range U1, U2.
        for (i = 1; i <= _sequence.Length(); i++)
        {
            c = (BSplineCurve3D)_sequence[i];

            c.IncreaseDegree(myDegree);

            NCollection_Array1<double> knots = new(c.InternalKnots());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(uFirst, uLast, knots);
            c.SetKnots(knots);
        }

        NCollection_Sequence<Curve3D> theCurves = new();
        for (i = 1; i <= _sequence.Length(); i++)
        {
            theCurves.Append((Curve3D)_sequence[i].Copy());
        }

        bool unified = true;
        try
        {
            UnifyByInsertingAllKnots(theCurves, pTol);
        }
        catch (Exception caught) when (Refusal.Of(caught) is RangeErrorException && (_isPeriodic))
        {
            // BSplCLib::PrepareInsertKnots mispredicts the pole count of periodic curves (decision 1439)
            unified = false;
        }

        int theNbKnots = ((BSplineCurve3D)theCurves[1]).KnotCount;
        int theNbPoles = ((BSplineCurve3D)theCurves[1]).PoleCount;
        for (i = 2; i <= theCurves.Length() && unified; i++)
        {
            // the pole counts too, which differ with the knots' multiplicities (decision 1439)
            if (((BSplineCurve3D)theCurves[i]).KnotCount != theNbKnots
                || (_isPeriodic && ((BSplineCurve3D)theCurves[i]).PoleCount != theNbPoles))
            {
                unified = false;
            }
        }

        if (unified)
        {
            _sequence = theCurves;
        }
        else if (_isPeriodic)
        {
            // Periodic sections whose knots couldn't be unified (OCCT wrote past its arrays, natively, or kept curves
            // with different numbers of poles): the profile is made of their non-periodic forms, which unify
            // (decision 1439).
            _isPeriodic = false;
            Perform(pTol);
            return;
        }
        else
        {
            UnifyBySettingMiddleKnots(_sequence);
        }

        _isDone = true;
    }

    /// <summary>The common degree. Throws StdFail_NotDone before <see cref="Perform"/>.</summary>
    public int Degree()
    {
        if (!_isDone)
        {
            throw new NotDoneException("GeomFill_Profiler::Degree");
        }

        BSplineCurve3D c = (BSplineCurve3D)_sequence[1];
        return c.Degree;
    }

    public bool IsPeriodic => _isPeriodic;

    /// <summary>The common number of poles. Throws StdFail_NotDone before <see cref="Perform"/>.</summary>
    public int NbPoles()
    {
        if (!_isDone)
        {
            throw new NotDoneException("GeomFill_Profiler::Degree");
        }

        BSplineCurve3D c = (BSplineCurve3D)_sequence[1];
        return c.PoleCount;
    }

    /// <summary>
    /// Copies the poles of curve <paramref name="index"/> (from 1) into <paramref name="poles"/>, which must have
    /// <see cref="NbPoles"/> elements.
    /// </summary>
    public void Poles(int index, NCollection_Array1<Point3D> poles)
    {
        if (!_isDone)
        {
            throw new NotDoneException("GeomFill_Profiler::Degree");
        }

        Standard_DomainError_Raise_if(poles.Length() != NbPoles(), "GeomFill_Profiler::Poles");
        Standard_DomainError_Raise_if(index < 1 || index > _sequence.Length(), "GeomFill_Profiler::Poles");

        BSplineCurve3D c = (BSplineCurve3D)_sequence[index];

        poles.Assign(c.InternalPoles());
    }

    /// <summary>
    /// Copies the weights of curve <paramref name="index"/> (from 1) into <paramref name="weights"/>, which must
    /// have <see cref="NbPoles"/> elements.
    /// </summary>
    public void Weights(int index, NCollection_Array1<double> weights)
    {
        if (!_isDone)
        {
            throw new NotDoneException("GeomFill_Profiler::Degree");
        }

        Standard_DomainError_Raise_if(weights.Length() != NbPoles(), "GeomFill_Profiler::Weights");
        Standard_DomainError_Raise_if(index < 1 || index > _sequence.Length(), "GeomFill_Profiler::Weights");

        BSplineCurve3D c = (BSplineCurve3D)_sequence[index];

        weights.Assign(c.InternalWeightsArray());
    }

    /// <summary>The common number of knots. Throws StdFail_NotDone before <see cref="Perform"/>.</summary>
    public int NbKnots()
    {
        if (!_isDone)
        {
            throw new NotDoneException("GeomFill_Profiler::Degree");
        }

        BSplineCurve3D c = (BSplineCurve3D)_sequence[1];

        return c.KnotCount;
    }

    /// <summary>Copies the common knots and multiplicities; both arrays must have <see cref="NbKnots"/> elements.</summary>
    public void KnotsAndMults(NCollection_Array1<double> knots, NCollection_Array1<int> mults)
    {
        if (!_isDone)
        {
            throw new NotDoneException("GeomFill_Profiler::Degree");
        }

        int n = NbKnots();
        Standard_DomainError_Raise_if(knots.Length() != n || mults.Length() != n, "GeomFill_Profiler::KnotsAndMults");

        BSplineCurve3D c = (BSplineCurve3D)_sequence[1];

        knots.Assign(c.InternalKnots());
        mults.Assign(c.InternalMultiplicities());
    }

    /// <summary>
    /// The curve <paramref name="index"/> (from 1): the profiler's own B-spline, set to the common profile after
    /// <see cref="Perform"/>; frozen (decision 844).
    /// </summary>
    public Curve3D Curve(int index)
    {
        Curve3D c = _sequence[index];
        c.Freeze();
        return c;
    }

    /// <summary>The curve <paramref name="index"/>, not frozen, for ported callers.</summary>
    internal Curve3D InternalCurve(int index) => _sequence[index];
}
