// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_BSpFunc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_BSpFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>A law based on a 1D B-spline curve (<see cref="Law_BSpline"/>) between two parameters.</summary>
/// <remarks>
/// A working object (decision 21): <see cref="SetCurve"/> is public and changes the law. The curve the law keeps
/// is frozen (decision 132's pattern): the constructor and <see cref="SetCurve"/> freeze the caller's curve,
/// which they keep as OCCT keeps the handle, and <see cref="Curve"/> returns that same curve. Laws made by
/// <see cref="Trim"/> share it.
/// </remarks>
public class Law_BSpFunc : Law_Function
{
    private Law_BSpline? curv;
    private double first;
    private double last;

    private static double PosTol => global::Kerf.Engine.Foundation.Precision.PConfusion() / 2;

    public Law_BSpFunc()
    {
        first = 0.0;
        last = 0.0;
    }

    /// <summary>The law of the curve C between First and Last; C is kept and frozen.</summary>
    public Law_BSpFunc(Law_BSpline C, double First, double Last)
    {
        C.Freeze();
        curv = C;
        first = First;
        last = Last;
    }

    public override Continuity Continuity() => curv!.Continuity();

    public override int NbIntervals(Continuity S)
    {
        int myNbIntervals = 1;
        if (S > Continuity())
        {
            int Cont;
            switch (S)
            {
                case G1:
                case G2:
                    throw new DomainErrorException("Law_BSpFunc::NbIntervals");
                case C0:
                    myNbIntervals = 1;
                    break;
                case C1:
                case C2:
                case C3:
                case CN:
                {
                    if (S == C1)
                    {
                        Cont = 1;
                    }
                    else if (S == C2)
                    {
                        Cont = 2;
                    }
                    else if (S == C3)
                    {
                        Cont = 3;
                    }
                    else
                    {
                        Cont = curv!.Degree();
                    }
                    Law_BSplineKnotSplitting Convector = new Law_BSplineKnotSplitting(curv!, Cont);
                    int NbInt = Convector.NbSplits() - 1;
                    NCollection_Array1<int> Inter = new NCollection_Array1<int>(1, NbInt + 1);
                    Convector.Splitting(Inter);

                    int Nb = curv!.NbKnots();
                    int Index1;
                    int Index2;
                    double newFirst, newLast;
                    NCollection_Array1<double> TK = new NCollection_Array1<double>(1, Nb);
                    NCollection_Array1<int> TM = new NCollection_Array1<int>(1, Nb);
                    curv.Knots(TK);
                    curv.Multiplicities(TM);
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(curv.Degree(), TK, TM, first, curv.IsPeriodic(), 1, Nb, out Index1,
                                             out newFirst);
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(curv.Degree(), TK, TM, last, curv.IsPeriodic(), 1, Nb, out Index2,
                                             out newLast);
                    if (Math.Abs(newFirst - TK[Index1 + 1]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        Index1++;
                    }
                    if (newLast - TK[Index2] > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        Index2++;
                    }

                    myNbIntervals = 1;
                    for (int i = 1; i <= NbInt; i++)
                    {
                        if (Inter[i] > Index1 && Inter[i] < Index2)
                        {
                            myNbIntervals++;
                        }
                    }
                }
                break;
            }
        }

        return myNbIntervals;
    }

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        int myNbIntervals = 1;
        if (S > Continuity())
        {
            int Cont;
            switch (S)
            {
                case G1:
                case G2:
                    throw new DomainErrorException("Law_BSpFunc_Curve::Intervals");
                case C0:
                    myNbIntervals = 1;
                    break;
                case C1:
                case C2:
                case C3:
                case CN:
                {
                    if (S == C1)
                    {
                        Cont = 1;
                    }
                    else if (S == C2)
                    {
                        Cont = 2;
                    }
                    else if (S == C3)
                    {
                        Cont = 3;
                    }
                    else
                    {
                        Cont = curv!.Degree();
                    }
                    Law_BSplineKnotSplitting Convector = new Law_BSplineKnotSplitting(curv!, Cont);
                    int NbInt = Convector.NbSplits() - 1;
                    NCollection_Array1<int> Inter = new NCollection_Array1<int>(1, NbInt + 1);
                    Convector.Splitting(Inter);

                    int Nb = curv!.NbKnots();
                    int Index1;
                    int Index2;
                    double newFirst, newLast;
                    NCollection_Array1<double> TK = new NCollection_Array1<double>(1, Nb);
                    NCollection_Array1<int> TM = new NCollection_Array1<int>(1, Nb);
                    curv.Knots(TK);
                    curv.Multiplicities(TM);
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(curv.Degree(), TK, TM, first, curv.IsPeriodic(), 1, Nb, out Index1,
                                             out newFirst);
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(curv.Degree(), TK, TM, last, curv.IsPeriodic(), 1, Nb, out Index2,
                                             out newLast);
                    if (Math.Abs(newFirst - TK[Index1 + 1]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        Index1++;
                    }
                    if (newLast - TK[Index2] > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        Index2++;
                    }

                    Inter[1] = Index1;
                    myNbIntervals = 1;
                    for (int i = 1; i <= NbInt; i++)
                    {
                        if (Inter[i] > Index1 && Inter[i] < Index2)
                        {
                            myNbIntervals++;
                            Inter[myNbIntervals] = Inter[i];
                        }
                    }
                    Inter[myNbIntervals + 1] = Index2;

                    for (int I = 1; I <= myNbIntervals + 1; I++)
                    {
                        T[I] = TK[Inter[I]];
                    }
                }
                break;
            }
        }
        T[T.Lower()] = first;
        T[T.Lower() + myNbIntervals] = last;
    }

    /// <summary>The knot span to evaluate X on when X is a bound of the law (the span inside the bounds).</summary>
    private void LocateBound(double X, out int Ideb, out int Ifin)
    {
        Ideb = 0;
        Ifin = 0;
        if (X == first)
        {
            curv!.LocateU(first, PosTol, out Ideb, out Ifin);
            if (Ideb < 1)
            {
                Ideb = 1;
            }
            if (Ideb >= Ifin)
            {
                Ifin = Ideb + 1;
            }
        }
        if (X == last)
        {
            curv!.LocateU(last, PosTol, out Ideb, out Ifin);
            if (Ifin > curv.NbKnots())
            {
                Ifin = curv.NbKnots();
            }
            if (Ideb >= Ifin)
            {
                Ideb = Ifin - 1;
            }
        }
    }

    public override double Value(double X)
    {
        if ((X == first) || (X == last))
        {
            LocateBound(X, out int Ideb, out int Ifin);
            return curv!.LocalValue(X, Ideb, Ifin);
        }
        else
        {
            return curv!.Value(X);
        }
    }

    public override void D1(double X, out double F, out double D)
    {
        if ((X == first) || (X == last))
        {
            LocateBound(X, out int Ideb, out int Ifin);
            curv!.LocalD1(X, Ideb, Ifin, out F, out D);
        }
        else
        {
            curv!.D1(X, out F, out D);
        }
    }

    public override void D2(double X, out double F, out double D, out double D2)
    {
        if ((X == first) || (X == last))
        {
            LocateBound(X, out int Ideb, out int Ifin);
            curv!.LocalD2(X, Ideb, Ifin, out F, out D, out D2);
        }
        else
        {
            curv!.D2(X, out F, out D, out D2);
        }
    }

    /// <summary>A law of the same curve between PFirst and PLast.</summary>
    public override Law_Function Trim(double PFirst, double PLast, double Tol)
    {
        Law_BSpFunc l = new Law_BSpFunc(curv!, PFirst, PLast);
        return l;
    }

    public override void Bounds(out double PFirst, out double PLast)
    {
        PFirst = first;
        PLast = last;
    }

    /// <summary>The curve of the law (frozen), or null if none was set.</summary>
    public Law_BSpline? Curve() => curv;

    /// <summary>Sets the curve of the law (kept and frozen) and its bounds as the law's.</summary>
    public void SetCurve(Law_BSpline C)
    {
        C.Freeze();
        curv = C;
        first = C.FirstParameter();
        last = C.LastParameter();
    }
}
