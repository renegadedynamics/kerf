// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Profiler.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Profiler.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Profiler.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// The common B-spline profile of a group of curves: <see cref="Perform"/> converts every curve to a B-spline and
/// gives them all the same degree and the same knot vector, so the same number of poles.
/// </summary>
/// <remarks>
/// A working object (decision 21). The profiler converts each curve it is given into a B-spline of its own (a copy:
/// the caller's curve is never changed) and changes those in <see cref="Perform"/>. <see cref="Curve"/> hands one out
/// frozen; a later <see cref="Perform"/> copies a frozen curve before changing it (decision 844).
/// </remarks>
public class GeomFill_Profiler
{
    protected NCollection_Sequence<Curve3D> mySequence = new();
    protected bool myIsDone;
    protected bool myIsPeriodic;

    private static void UnifyByInsertingAllKnots(NCollection_Sequence<Curve3D> theCurves, double PTol)
    {
        // inserting in the first curve the knot-vector of all the others.
        BSplineCurve3D C = (BSplineCurve3D)theCurves[1];

        int i;
        for (i = 2; i <= theCurves.Length(); i++)
        {
            BSplineCurve3D Ci = (BSplineCurve3D)theCurves[i];
            NCollection_Array1<double> Ki = Ci.InternalKnots();
            NCollection_Array1<int> Mi = Ci.InternalMultiplicities();

            C.InsertKnots(Ki, Mi, PTol, false);
        }

        NCollection_Array1<double> NewKnots = C.InternalKnots();
        NCollection_Array1<int> NewMults = C.InternalMultiplicities();
        for (i = 2; i <= theCurves.Length(); i++)
        {
            BSplineCurve3D Ci = (BSplineCurve3D)theCurves[i];
            Ci.InsertKnots(NewKnots, NewMults, PTol, false);
        }

        // essai : tentative mise des poids sur chaque section a une moyenne 1
        for (i = 1; i <= theCurves.Length(); i++)
        {
            BSplineCurve3D Ci = (BSplineCurve3D)theCurves[i];
            if (Ci.IsRational)
            {
                int np = Ci.PoleCount;
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

    private static void UnifyBySettingMiddleKnots(NCollection_Sequence<Curve3D> theCurves)
    {
        int i, j;

        BSplineCurve3D C = (BSplineCurve3D)theCurves[1];

        int NbKnots = C.KnotCount;
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
                BSplineCurve3D Ctemp = (BSplineCurve3D)theCurves[i];
                aMidKnot += Ctemp.Knot(j);
            }
            aMidKnot /= theCurves.Length();
            NewKnots[j] = aMidKnot;
        }

        for (i = 1; i <= theCurves.Length(); i++)
        {
            BSplineCurve3D Cres = (BSplineCurve3D)theCurves[i];
            Cres.SetKnots(NewKnots);
        }
    }

    public GeomFill_Profiler()
    {
        myIsDone = false;
        myIsPeriodic = true;
    }

    /// <summary>Adds a curve: conics are approximated (C1, tolerance Precision::Confusion), others converted.</summary>
    public void AddCurve(Curve3D Curve)
    {
        Curve3D? C = null;
        //// modified by jgv, 19.01.05 for OCC7354 ////
        Curve3D theCurve = Curve;
        if (theCurve.GetType() == typeof(TrimmedCurve3D))
        {
            theCurve = ((TrimmedCurve3D)theCurve).BasisCurve();
        }
        if (theCurve is ConicCurve3D)
        {
            ApproxCurve appr = new(Curve, global::Kerf.Engine.Foundation.Precision.Confusion(), C1, 16, 14);
            if (appr.HasResult)
            {
                C = appr.InternalCurve();
            }
        }
        if (C is null)
        {
            C = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(Curve);
        }
        ///////////////////////////////////////////////

        mySequence.Append(C);

        if (myIsPeriodic && !C.IsPeriodic)
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
        BSplineCurve3D C;
        double U1, U2, UFirst = 0, ULast = 0;
        double EcartMax = 0.0;

        // The curves handed out by Curve() are frozen: change copies (decision 844).
        for (i = 1; i <= mySequence.Length(); i++)
        {
            if (mySequence[i].IsFrozen)
            {
                mySequence[i] = (Curve3D)mySequence[i].Copy();
            }
        }

        for (i = 1; i <= mySequence.Length(); i++)
        {
            C = (BSplineCurve3D)mySequence[i];

            // si non periodique, il faut deperiodiser toutes les courbes
            // on les segmente ensuite pour assurer K(1) et K(n) de multiplicite
            // degre + 1

            U2 = C.Knot(C.LastUKnotIndex());
            U1 = C.Knot(C.FirstUKnotIndex());

            if (!myIsPeriodic && C.IsPeriodic)
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
            C = (BSplineCurve3D)mySequence[i];

            C.IncreaseDegree(myDegree);

            NCollection_Array1<double> Knots = new(C.InternalKnots());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(UFirst, ULast, Knots);
            C.SetKnots(Knots);
        }

        NCollection_Sequence<Curve3D> theCurves = new();
        for (i = 1; i <= mySequence.Length(); i++)
        {
            theCurves.Append((Curve3D)mySequence[i].Copy());
        }

        UnifyByInsertingAllKnots(theCurves, PTol);

        bool Unified = true;
        int theNbKnots = ((BSplineCurve3D)theCurves[1]).KnotCount;
        for (i = 2; i <= theCurves.Length(); i++)
        {
            if (((BSplineCurve3D)theCurves[i]).KnotCount != theNbKnots)
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
            throw new NotDoneException("GeomFill_Profiler::Degree");
        }

        BSplineCurve3D C = (BSplineCurve3D)mySequence[1];
        return C.Degree();
    }

    public bool IsPeriodic() => myIsPeriodic;

    /// <summary>The common number of poles. Throws StdFail_NotDone before <see cref="Perform"/>.</summary>
    public int NbPoles()
    {
        if (!myIsDone)
        {
            throw new NotDoneException("GeomFill_Profiler::Degree");
        }

        BSplineCurve3D C = (BSplineCurve3D)mySequence[1];
        return C.PoleCount;
    }

    /// <summary>
    /// Copies the poles of curve <paramref name="Index"/> (from 1) into <paramref name="Poles"/>, which must have
    /// <see cref="NbPoles"/> elements.
    /// </summary>
    public void Poles(int Index, NCollection_Array1<Point3D> Poles)
    {
        if (!myIsDone)
        {
            throw new NotDoneException("GeomFill_Profiler::Degree");
        }

        Standard_DomainError_Raise_if(Poles.Length() != NbPoles(), "GeomFill_Profiler::Poles");
        Standard_DomainError_Raise_if(Index < 1 || Index > mySequence.Length(), "GeomFill_Profiler::Poles");

        BSplineCurve3D C = (BSplineCurve3D)mySequence[Index];

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
            throw new NotDoneException("GeomFill_Profiler::Degree");
        }

        Standard_DomainError_Raise_if(Weights.Length() != NbPoles(), "GeomFill_Profiler::Weights");
        Standard_DomainError_Raise_if(Index < 1 || Index > mySequence.Length(), "GeomFill_Profiler::Weights");

        BSplineCurve3D C = (BSplineCurve3D)mySequence[Index];

        Weights.Assign(C.InternalWeightsArray());
    }

    /// <summary>The common number of knots. Throws StdFail_NotDone before <see cref="Perform"/>.</summary>
    public int NbKnots()
    {
        if (!myIsDone)
        {
            throw new NotDoneException("GeomFill_Profiler::Degree");
        }

        BSplineCurve3D C = (BSplineCurve3D)mySequence[1];

        return C.KnotCount;
    }

    /// <summary>Copies the common knots and multiplicities; both arrays must have <see cref="NbKnots"/> elements.</summary>
    public void KnotsAndMults(NCollection_Array1<double> Knots, NCollection_Array1<int> Mults)
    {
        if (!myIsDone)
        {
            throw new NotDoneException("GeomFill_Profiler::Degree");
        }

        int n = NbKnots();
        Standard_DomainError_Raise_if(Knots.Length() != n || Mults.Length() != n, "GeomFill_Profiler::KnotsAndMults");

        BSplineCurve3D C = (BSplineCurve3D)mySequence[1];

        Knots.Assign(C.InternalKnots());
        Mults.Assign(C.InternalMultiplicities());
    }

    /// <summary>
    /// The curve <paramref name="Index"/> (from 1): the profiler's own B-spline, set to the common profile after
    /// <see cref="Perform"/>; frozen (decision 844).
    /// </summary>
    public Curve3D Curve(int Index)
    {
        Curve3D C = mySequence[Index];
        C.Freeze();
        return C;
    }

    /// <summary>The curve <paramref name="Index"/>, not frozen, for ported callers.</summary>
    internal Curve3D InternalCurve(int Index) => mySequence[Index];
}
