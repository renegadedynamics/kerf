// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Profiler.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Profiler.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Profiler.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// The common B-spline profile of a group of curves: <see cref="Perform"/> converts every curve to a B-spline and
/// gives them all the same degree and the same knot vector, so the same number of poles.
/// </summary>
/// <remarks>
/// A working object (decision 21). The profiler converts each curve it is given into a B-spline of its own (a copy:
/// the caller's curve is never changed) and changes those in <see cref="Perform"/>. <see cref="Curve"/> hands one out
/// frozen; a later <see cref="Perform"/> copies a frozen curve before changing it (decision 5921).
/// </remarks>
public class GeomFill_Profiler
{
    protected NCollection_Sequence<Geom_Curve> mySequence = new();
    protected bool myIsDone;
    protected bool myIsPeriodic;

    private static void UnifyByInsertingAllKnots(NCollection_Sequence<Geom_Curve> theCurves, double PTol)
    {
        // inserting in the first curve the knot-vector of all the others.
        Geom_BSplineCurve C = (Geom_BSplineCurve)theCurves[1];

        int i;
        for (i = 2; i <= theCurves.Length(); i++)
        {
            Geom_BSplineCurve Ci = (Geom_BSplineCurve)theCurves[i];
            NCollection_Array1<double> Ki = Ci.InternalKnots();
            NCollection_Array1<int> Mi = Ci.InternalMultiplicities();

            C.InsertKnots(Ki, Mi, PTol, false);
        }

        NCollection_Array1<double> NewKnots = C.InternalKnots();
        NCollection_Array1<int> NewMults = C.InternalMultiplicities();
        for (i = 2; i <= theCurves.Length(); i++)
        {
            Geom_BSplineCurve Ci = (Geom_BSplineCurve)theCurves[i];
            Ci.InsertKnots(NewKnots, NewMults, PTol, false);
        }

        // essai : tentative mise des poids sur chaque section a une moyenne 1
        for (i = 1; i <= theCurves.Length(); i++)
        {
            Geom_BSplineCurve Ci = (Geom_BSplineCurve)theCurves[i];
            if (Ci.IsRational())
            {
                int np = Ci.NbPoles();
                double sigma = 0.0;
                int j;
                for (j = 1; j <= np; j++)
                {
                    sigma += Ci.Weight(j);
                }
                sigma /= np;
                for (j = 1; j <= np; j++)
                {
                    Ci.SetWeight(j, Ci.Weight(j) / sigma);
                }
            }
        }
        // fin de l essai
    }

    private static void UnifyBySettingMiddleKnots(NCollection_Sequence<Geom_Curve> theCurves)
    {
        int i, j;

        Geom_BSplineCurve C = (Geom_BSplineCurve)theCurves[1];

        int NbKnots = C.NbKnots();
        double ULast = C.Knot(C.LastUKnotIndex());
        double UFirst = C.Knot(C.FirstUKnotIndex());

        // Set middle values of knots
        NCollection_Array1<double> NewKnots = new(1, NbKnots);
        NewKnots[1] = UFirst;
        NewKnots[NbKnots] = ULast;
        for (j = 2; j < NbKnots; j++)
        {
            double aMidKnot = 0.0;
            for (i = 1; i <= theCurves.Length(); i++)
            {
                Geom_BSplineCurve Ctemp = (Geom_BSplineCurve)theCurves[i];
                aMidKnot += Ctemp.Knot(j);
            }
            aMidKnot /= theCurves.Length();
            NewKnots[j] = aMidKnot;
        }

        for (i = 1; i <= theCurves.Length(); i++)
        {
            Geom_BSplineCurve Cres = (Geom_BSplineCurve)theCurves[i];
            Cres.SetKnots(NewKnots);
        }
    }

    public GeomFill_Profiler()
    {
        myIsDone = false;
        myIsPeriodic = true;
    }

    /// <summary>Adds a curve: conics are approximated (C1, tolerance Precision::Confusion), others converted.</summary>
    public void AddCurve(Geom_Curve Curve)
    {
        Geom_Curve? C = null;
        //// modified by jgv, 19.01.05 for OCC7354 ////
        Geom_Curve theCurve = Curve;
        if (theCurve.GetType() == typeof(Geom_TrimmedCurve))
        {
            theCurve = ((Geom_TrimmedCurve)theCurve).BasisCurve();
        }
        if (theCurve is Geom_Conic)
        {
            GeomConvert_ApproxCurve appr = new(Curve, Precision.Confusion(), GeomAbs_C1, 16, 14);
            if (appr.HasResult())
            {
                C = appr.InternalCurve();
            }
        }
        if (C is null)
        {
            C = GeomConvert.CurveToBSplineCurve(Curve);
        }
        ///////////////////////////////////////////////

        mySequence.Append(C);

        if (myIsPeriodic && !C.IsPeriodic())
        {
            myIsPeriodic = false;
        }
    }

    /// <summary>
    /// Converts all curves to B-spline curves and sets them to the common profile. <paramref name="PTol"/> is used
    /// to compare two knots.
    /// </summary>
    public virtual void Perform(double PTol)
    {
        int i;
        int myDegree = 0;
        Geom_BSplineCurve C;
        double U1, U2, UFirst = 0, ULast = 0;
        double EcartMax = 0.0;

        // The curves handed out by Curve() are frozen: change copies (decision 5921).
        for (i = 1; i <= mySequence.Length(); i++)
        {
            if (mySequence[i].IsFrozen())
            {
                mySequence[i] = (Geom_Curve)mySequence[i].Copy();
            }
        }

        for (i = 1; i <= mySequence.Length(); i++)
        {
            C = (Geom_BSplineCurve)mySequence[i];

            // si non periodique, il faut deperiodiser toutes les courbes
            // on les segmente ensuite pour assurer K(1) et K(n) de multiplicite
            // degre + 1

            U2 = C.Knot(C.LastUKnotIndex());
            U1 = C.Knot(C.FirstUKnotIndex());

            if (!myIsPeriodic && C.IsPeriodic())
            {
                C.SetNotPeriodic();
                C.Segment(U1, U2);
            }

            // evaluate the max degree
            myDegree = Max(myDegree, C.Degree());

            // Calcul de Max ( Ufin - Udeb) sur l ensemble des courbes.
            if ((U2 - U1) > EcartMax)
            {
                EcartMax = U2 - U1;
                UFirst = U1;
                ULast = U2;
            }
        }

        // increase the degree of the curves to my degree
        // reparametrize them in the range U1, U2.
        for (i = 1; i <= mySequence.Length(); i++)
        {
            C = (Geom_BSplineCurve)mySequence[i];

            C.IncreaseDegree(myDegree);

            NCollection_Array1<double> Knots = new(C.InternalKnots());
            BSplCLib.Reparametrize(UFirst, ULast, Knots);
            C.SetKnots(Knots);
        }

        NCollection_Sequence<Geom_Curve> theCurves = new();
        for (i = 1; i <= mySequence.Length(); i++)
        {
            theCurves.Append((Geom_Curve)mySequence[i].Copy());
        }

        UnifyByInsertingAllKnots(theCurves, PTol);

        bool Unified = true;
        int theNbKnots = ((Geom_BSplineCurve)theCurves[1]).NbKnots();
        for (i = 2; i <= theCurves.Length(); i++)
        {
            if (((Geom_BSplineCurve)theCurves[i]).NbKnots() != theNbKnots)
            {
                Unified = false;
                break;
            }
        }

        if (Unified)
        {
            mySequence = theCurves;
        }
        else
        {
            UnifyBySettingMiddleKnots(mySequence);
        }

        myIsDone = true;
    }

    /// <summary>The common degree. Throws StdFail_NotDone before <see cref="Perform"/>.</summary>
    public int Degree()
    {
        if (!myIsDone)
        {
            throw new StdFail_NotDone("GeomFill_Profiler::Degree");
        }

        Geom_BSplineCurve C = (Geom_BSplineCurve)mySequence[1];
        return C.Degree();
    }

    public bool IsPeriodic() => myIsPeriodic;

    /// <summary>The common number of poles. Throws StdFail_NotDone before <see cref="Perform"/>.</summary>
    public int NbPoles()
    {
        if (!myIsDone)
        {
            throw new StdFail_NotDone("GeomFill_Profiler::Degree");
        }

        Geom_BSplineCurve C = (Geom_BSplineCurve)mySequence[1];
        return C.NbPoles();
    }

    /// <summary>
    /// Copies the poles of curve <paramref name="Index"/> (from 1) into <paramref name="Poles"/>, which must have
    /// <see cref="NbPoles"/> elements.
    /// </summary>
    public void Poles(int Index, NCollection_Array1<gp_Pnt> Poles)
    {
        if (!myIsDone)
        {
            throw new StdFail_NotDone("GeomFill_Profiler::Degree");
        }

        Standard_DomainError_Raise_if(Poles.Length() != NbPoles(), "GeomFill_Profiler::Poles");
        Standard_DomainError_Raise_if(Index < 1 || Index > mySequence.Length(), "GeomFill_Profiler::Poles");

        Geom_BSplineCurve C = (Geom_BSplineCurve)mySequence[Index];

        Poles.Assign(C.InternalPoles());
    }

    /// <summary>
    /// Copies the weights of curve <paramref name="Index"/> (from 1) into <paramref name="Weights"/>, which must
    /// have <see cref="NbPoles"/> elements.
    /// </summary>
    public void Weights(int Index, NCollection_Array1<double> Weights)
    {
        if (!myIsDone)
        {
            throw new StdFail_NotDone("GeomFill_Profiler::Degree");
        }

        Standard_DomainError_Raise_if(Weights.Length() != NbPoles(), "GeomFill_Profiler::Weights");
        Standard_DomainError_Raise_if(Index < 1 || Index > mySequence.Length(), "GeomFill_Profiler::Weights");

        Geom_BSplineCurve C = (Geom_BSplineCurve)mySequence[Index];

        Weights.Assign(C.InternalWeightsArray());
    }

    /// <summary>The common number of knots. Throws StdFail_NotDone before <see cref="Perform"/>.</summary>
    public int NbKnots()
    {
        if (!myIsDone)
        {
            throw new StdFail_NotDone("GeomFill_Profiler::Degree");
        }

        Geom_BSplineCurve C = (Geom_BSplineCurve)mySequence[1];

        return C.NbKnots();
    }

    /// <summary>Copies the common knots and multiplicities; both arrays must have <see cref="NbKnots"/> elements.</summary>
    public void KnotsAndMults(NCollection_Array1<double> Knots, NCollection_Array1<int> Mults)
    {
        if (!myIsDone)
        {
            throw new StdFail_NotDone("GeomFill_Profiler::Degree");
        }

        int n = NbKnots();
        Standard_DomainError_Raise_if(Knots.Length() != n || Mults.Length() != n, "GeomFill_Profiler::KnotsAndMults");

        Geom_BSplineCurve C = (Geom_BSplineCurve)mySequence[1];

        Knots.Assign(C.InternalKnots());
        Mults.Assign(C.InternalMultiplicities());
    }

    /// <summary>
    /// The curve <paramref name="Index"/> (from 1): the profiler's own B-spline, set to the common profile after
    /// <see cref="Perform"/>; frozen (decision 5921).
    /// </summary>
    public Geom_Curve Curve(int Index)
    {
        Geom_Curve C = mySequence[Index];
        C.Freeze();
        return C;
    }

    /// <summary>The curve <paramref name="Index"/>, not frozen, for ported callers.</summary>
    internal Geom_Curve InternalCurve(int Index) => mySequence[Index];
}
