// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GGenExtCC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ECC2d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Extrema_GGenExtCC;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// The minimal distances between two 2D curves, found by global optimisation (math_GlobOptMin on
/// <see cref="Extrema_GlobOptFuncCCC2"/>) over the pairs of continuity intervals of the curves. When the minimum is
/// reached along a whole range (parallel curves), one solution is kept and <see cref="IsParallel"/> is true.
/// </summary>
/// <remarks>
/// The 2D instantiation of OCCT's template Extrema_GGenExtCC (Extrema_ECC2d). An algorithm object (decision 117): set
/// the curves, then <c>Perform</c>. Keeps references to the curves, as OCCT does. As in OCCT, <c>Perform</c> does not
/// clear the solutions of a previous <c>Perform</c> (occt-issues.md).
/// </remarks>
public class Extrema_ECC2d
{
    private bool myIsFindSingleSolution;
    private bool myParallel;
    private double myCurveMinTol;
    private readonly math_Vector myLowBorder;
    private readonly math_Vector myUppBorder;
    private readonly NCollection_Sequence<double> myPoints1 = new();
    private readonly NCollection_Sequence<double> myPoints2 = new();
    private readonly Adaptor2d_Curve2d?[] myC = new Adaptor2d_Curve2d?[2];
    private bool myDone;

    public Extrema_ECC2d()
    {
        myIsFindSingleSolution = false;
        myParallel = false;
        myCurveMinTol = Precision.PConfusion();
        myLowBorder = new math_Vector(1, 2);
        myUppBorder = new math_Vector(1, 2);
        myDone = false;
        myC[0] = myC[1] = null;
    }

    /// <summary>The algorithm for the curves theC1 and theC2 on their whole ranges.</summary>
    public Extrema_ECC2d(Adaptor2d_Curve2d theC1, Adaptor2d_Curve2d theC2)
    {
        myIsFindSingleSolution = false;
        myParallel = false;
        myCurveMinTol = Precision.PConfusion();
        myLowBorder = new math_Vector(1, 2);
        myUppBorder = new math_Vector(1, 2);
        myDone = false;
        myC[0] = theC1;
        myC[1] = theC2;
        myLowBorder[1] = theC1.FirstParameter();
        myLowBorder[2] = theC2.FirstParameter();
        myUppBorder[1] = theC1.LastParameter();
        myUppBorder[2] = theC2.LastParameter();
    }

    /// <summary>The algorithm for theC1 on [theUinf, theUsup] and theC2 on [theVinf, theVsup].</summary>
    public Extrema_ECC2d(Adaptor2d_Curve2d theC1, Adaptor2d_Curve2d theC2, double theUinf, double theUsup, double theVinf,
        double theVsup)
    {
        myIsFindSingleSolution = false;
        myParallel = false;
        myCurveMinTol = Precision.PConfusion();
        myLowBorder = new math_Vector(1, 2);
        myUppBorder = new math_Vector(1, 2);
        myDone = false;
        myC[0] = theC1;
        myC[1] = theC2;
        myLowBorder[1] = theUinf;
        myLowBorder[2] = theVinf;
        myUppBorder[1] = theUsup;
        myUppBorder[2] = theVsup;
    }

    /// <summary>Sets the curves and their ranges.</summary>
    public void SetParams(Adaptor2d_Curve2d theC1, Adaptor2d_Curve2d theC2, double theUinf, double theUsup,
        double theVinf, double theVsup)
    {
        myC[0] = theC1;
        myC[1] = theC2;
        myLowBorder[1] = theUinf;
        myLowBorder[2] = theVinf;
        myUppBorder[1] = theUsup;
        myUppBorder[2] = theVsup;
    }

    /// <summary>Sets the parametric tolerance under which two solutions are the same.</summary>
    public void SetTolerance(double theTol) => myCurveMinTol = theTol;

    /// <summary>Asks for one solution only (faster).</summary>
    public void SetSingleSolutionFlag(bool theFlag) => myIsFindSingleSolution = theFlag;

    public bool GetSingleSolutionFlag() => myIsFindSingleSolution;

    /// <summary>Computes the minimal distances.</summary>
    public void Perform()
    {
        // OCCT doesn't clear myPoints1 and myPoints2: a second Perform appends to the first one's solutions
        myDone = false;
        myParallel = false;

        Adaptor2d_Curve2d C1 = myC[0]!;
        Adaptor2d_Curve2d C2 = myC[1]!;

        Span<int> aNbInter = stackalloc int[2];
        GeomAbs_Shape aContinuity = GeomAbs_C2;
        aNbInter[0] = C1.NbIntervals(aContinuity);
        aNbInter[1] = C2.NbIntervals(aContinuity);

        if (aNbInter[0] * aNbInter[1] > 100)
        {
            aContinuity = GeomAbs_C1;
            aNbInter[0] = C1.NbIntervals(aContinuity);
            aNbInter[1] = C2.NbIntervals(aContinuity);
        }

        Span<double> anL = stackalloc double[2];
        int indmax = -1, indmin = -1;
        const double mult = 20.0;
        if (!(Precision.IsInfinite(C1.FirstParameter()) || Precision.IsInfinite(C1.LastParameter())
              || Precision.IsInfinite(C2.FirstParameter()) || Precision.IsInfinite(C2.LastParameter())))
        {
            anL[0] = GCPnts_AbscissaPoint.Length(C1);
            anL[1] = GCPnts_AbscissaPoint.Length(C2);
            if (anL[0] / aNbInter[0] > mult * anL[1] / aNbInter[1])
            {
                indmax = 0;
                indmin = 1;
            }
            else if (anL[1] / aNbInter[1] > mult * anL[0] / aNbInter[0])
            {
                indmax = 1;
                indmin = 0;
            }
        }
        int aNbIntOpt = 0;
        if (indmax >= 0)
        {
            aNbIntOpt = RealToInt(anL[indmax] * aNbInter[indmin] / anL[indmin] / (mult / 4.0)) + 1;
            if (aNbIntOpt > 100 || aNbIntOpt < aNbInter[indmax])
            {
                indmax = -1;
            }
            else
            {
                if (aNbIntOpt * aNbInter[indmin] > 100)
                {
                    aNbIntOpt = 100 / aNbInter[indmin];
                    if (aNbIntOpt < aNbInter[indmax])
                    {
                        indmax = -1;
                    }
                }
            }
        }

        NCollection_HArray1<double> anIntervals1 = new(1, aNbInter[0] + 1);
        NCollection_HArray1<double> anIntervals2 = new(1, aNbInter[1] + 1);
        C1.Intervals(anIntervals1.ChangeArray1(), aContinuity);
        C2.Intervals(anIntervals2.ChangeArray1(), aContinuity);
        if (indmax >= 0)
        {
            if (indmax == 0)
            {
                Extrema_GGenExtCC_ChangeIntervals(ref anIntervals1, aNbIntOpt);
                aNbInter[0] = anIntervals1.Length() - 1;
            }
            else
            {
                Extrema_GGenExtCC_ChangeIntervals(ref anIntervals2, aNbIntOpt);
                aNbInter[1] = anIntervals2.Length() - 1;
            }
        }
        if (C1.IsClosed() && aNbInter[0] == 1)
        {
            Extrema_GGenExtCC_ChangeIntervals(ref anIntervals1, 3);
            aNbInter[0] = anIntervals1.Length() - 1;
        }
        if (C2.IsClosed() && aNbInter[1] == 1)
        {
            Extrema_GGenExtCC_ChangeIntervals(ref anIntervals2, 3);
            aNbInter[1] = anIntervals2.Length() - 1;
        }

        const double aMaxLC = 10000.0;
        double aLC = 100.0;
        double aMaxDer1 = 1.0 / C1.Resolution(1.0);
        double aMaxDer2 = 1.0 / C2.Resolution(1.0);
        double aMaxDer = Max(aMaxDer1, aMaxDer2) * M_SQRT2;
        if (aLC > aMaxDer)
        {
            aLC = aMaxDer;
        }

        bool isConstLockedFlag = false;
        const double aCR = 0.001;
        if (aMaxDer1 / aMaxDer < aCR || aMaxDer2 / aMaxDer < aCR)
        {
            isConstLockedFlag = true;
        }
        if (aMaxDer > aMaxLC)
        {
            aLC = aMaxLC;
            isConstLockedFlag = true;
        }
        if (C1.GetType() == GeomAbs_Line)
        {
            aMaxDer = 1.0 / C2.Resolution(1.0);
            if (aLC > aMaxDer)
            {
                isConstLockedFlag = true;
                aLC = aMaxDer;
            }
        }
        if (C2.GetType() == GeomAbs_Line)
        {
            aMaxDer = 1.0 / C1.Resolution(1.0);
            if (aLC > aMaxDer)
            {
                isConstLockedFlag = true;
                aLC = aMaxDer;
            }
        }

        Extrema_GlobOptFuncCCC2 aFunc = new(C1, C2);
        if (aLC < aMaxLC || aMaxDer > aMaxLC)
        {
            math_Vector aT = new(1, 2), aG = new(1, 2);
            double aF0, aMaxG = 0.0;
            double t1, t2, dt1, dt2;
            int n1 = 21, n2 = 21, i1, i2;
            dt1 = (C1.LastParameter() - C1.FirstParameter()) / (n1 - 1);
            dt2 = (C2.LastParameter() - C2.FirstParameter()) / (n2 - 1);
            for (i1 = 1, t1 = C1.FirstParameter(); i1 <= n1; ++i1, t1 += dt1)
            {
                aT[1] = t1;
                for (i2 = 1, t2 = C2.FirstParameter(); i2 <= n2; ++i2, t2 += dt2)
                {
                    aT[2] = t2;
                    aFunc.Values(aT, out aF0, aG);
                    double aMod = aG[1] * aG[1] + aG[2] * aG[2];
                    aMaxG = Max(aMaxG, aMod);
                }
            }
            aMaxG = Math.Sqrt(aMaxG);
            if (aMaxG > aMaxDer)
            {
                aLC = Min(aMaxG, aMaxLC);
                isConstLockedFlag = true;
            }
            if (aMaxG > 100.0 * aMaxLC)
            {
                aLC = 100.0 * aMaxLC;
                isConstLockedFlag = true;
            }
            else if (aMaxG < 0.1 * aMaxDer)
            {
                isConstLockedFlag = true;
            }
        }
        math_GlobOptMin aFinder = new(aFunc, myLowBorder, myUppBorder, aLC);
        aFinder.SetLipConstState(isConstLockedFlag);
        aFinder.SetContinuity(aContinuity == GeomAbs_C2 ? 2 : 1);
        double aDiscTol = 1.0e-2;
        double aValueTol = 1.0e-2;
        double aSameTol = myCurveMinTol / (aDiscTol);
        aFinder.SetTol(aDiscTol, aSameTol);
        aFinder.SetFunctionalMinimalValue(0.0);

        double aCellSize = Max(Max(anIntervals1.Last() - anIntervals1.First(),
                                   anIntervals2.Last() - anIntervals2.First())
                                 * Precision.PConfusion() / (2.0 * M_SQRT2),
                               Precision.PConfusion());
        Extrema_GGenExtCC_PointsInspector anInspector = new(aCellSize);
        NCollection_CellFilter<Extrema_GGenExtCC_PointsInspector, gp_XY, gp_XY> aFilter = new(aCellSize);
        NCollection_DynamicArray<gp_XY> aPnts = new();

        int i, j, k;
        math_Vector aFirstBorderInterval = new(1, 2);
        math_Vector aSecondBorderInterval = new(1, 2);
        double aF = RealLast();
        double aCurrF = RealLast();
        for (i = 1; i <= aNbInter[0]; i++)
        {
            for (j = 1; j <= aNbInter[1]; j++)
            {
                aFirstBorderInterval[1] = anIntervals1.Value(i);
                aFirstBorderInterval[2] = anIntervals2.Value(j);
                aSecondBorderInterval[1] = anIntervals1.Value(i + 1);
                aSecondBorderInterval[2] = anIntervals2.Value(j + 1);

                aFinder.SetLocalParams(aFirstBorderInterval, aSecondBorderInterval);
                aFinder.Perform(GetSingleSolutionFlag());

                aCurrF = aFinder.GetF();
                if (aCurrF >= aF + aSameTol * aValueTol)
                {
                    continue;
                }

                if (aCurrF > aF - aSameTol * aValueTol)
                {
                    if (aCurrF < aF)
                    {
                        aF = aCurrF;
                    }
                }
                else
                {
                    aF = aCurrF;
                    aFilter.Reset(aCellSize);
                    aPnts.Clear();
                }

                math_Vector sol = new(1, 2);
                for (k = 1; k <= aFinder.NbExtrema(); k++)
                {
                    aFinder.Points(k, sol);
                    gp_XY aPnt2d = new(sol[1], sol[2]);

                    gp_XY aXYmin = Extrema_GGenExtCC_PointsInspector.Shift(aPnt2d, -aCellSize);
                    gp_XY aXYmax = Extrema_GGenExtCC_PointsInspector.Shift(aPnt2d, aCellSize);

                    anInspector.ClearFind();
                    anInspector.SetCurrent(aPnt2d);
                    aFilter.Inspect(aXYmin, aXYmax, ref anInspector);
                    if (!anInspector.isFind())
                    {
                        aFilter.Add(aPnt2d, aPnt2d);
                        aPnts.Append(new gp_XY(sol[1], sol[2]));
                    }
                }
            }
        }

        int aNbSol = aPnts.Length();
        if (aNbSol == 0)
        {
            myDone = false;
            return;
        }

        myDone = true;

        if (aNbSol == 1)
        {
            gp_XY aSol = aPnts.First();
            myPoints1.Append(aSol.X());
            myPoints2.Append(aSol.Y());
            return;
        }

        aPnts.AsSpan().Sort(Extrema_GGenExtCC_Compare);

        NCollection_List<int> aSolutions = new();

        bool bSaveSolution = true;
        bool bDirsCoinside = true;
        bool bDifferentSolutions = false;

        bool isParallel = true;
        double aVal = 0.0;
        math_Vector aVec = new(1, 2, 0.0);

        for (int anIdx = 0; anIdx < aNbSol - 1; anIdx++)
        {
            gp_XY aCurrent = aPnts[anIdx];
            gp_XY aNext = aPnts[anIdx + 1];

            aVec[1] = (aCurrent.X() + aNext.X()) * 0.5;
            aVec[2] = (aCurrent.Y() + aNext.Y()) * 0.5;

            aFunc.Value(aVec, out aVal);
            if (Math.Abs(aVal - aF) < Precision.Confusion())
            {
                if (bSaveSolution)
                {
                    aSolutions.Append(anIdx);
                    bSaveSolution = false;
                }
            }
            else
            {
                isParallel = false;
                aSolutions.Append(anIdx);
                bSaveSolution = true;
            }

            if (!bDifferentSolutions)
            {
                if (aNext.X() > aCurrent.X())
                {
                    if (aNext.Y() > aCurrent.Y())
                    {
                        bDifferentSolutions = true;
                        bDirsCoinside = true;
                    }
                    else if (aNext.Y() < aCurrent.Y())
                    {
                        bDifferentSolutions = true;
                        bDirsCoinside = false;
                    }
                }
            }
        }
        aSolutions.Append(aNbSol - 1);

        if (!bDifferentSolutions)
        {
            isParallel = false;
        }

        if (isParallel)
        {
            ReadOnlySpan<double> aT1 = [myLowBorder[1], myUppBorder[1]];
            ReadOnlySpan<double> aT2 = [bDirsCoinside ? myLowBorder[2] : myUppBorder[2],
                                        bDirsCoinside ? myUppBorder[2] : myLowBorder[2]];

            Extrema_ExtPC2d anExtPC1 = new(), anExtPC2 = new();
            anExtPC1.Initialize(C1, myLowBorder[1], myUppBorder[1]);
            anExtPC2.Initialize(C2, myLowBorder[2], myUppBorder[2]);

            for (int iT = 0; isParallel && (iT < 2); ++iT)
            {
                double aDist1 = Extrema_GGenExtCC_ProjPOnC(C1.Value(aT1[iT]), anExtPC2);
                double aDist2 = Extrema_GGenExtCC_ProjPOnC(C2.Value(aT2[iT]), anExtPC1);
                isParallel = (Math.Abs(Min(aDist1, aDist2) - aF * aF) < Precision.Confusion());
            }
        }

        if (isParallel)
        {
            gp_XY aSol = aPnts.First();
            myPoints1.Append(aSol.X());
            myPoints2.Append(aSol.Y());
            myParallel = true;
        }
        else
        {
            NCollection_List<int>.Iterator aItSol = new(aSolutions);
            for (; aItSol.More(); aItSol.Next())
            {
                gp_XY aSol = aPnts[aItSol.Value()];
                myPoints1.Append(aSol.X());
                myPoints2.Append(aSol.Y());
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myDone;

    /// <summary>True if the curves are parallel (the minimum is reached along a range; one solution is kept).</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public bool IsParallel()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        return myParallel;
    }

    /// <summary>The number of extremum distances.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public int NbExt()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        return myPoints1.Length();
    }

    /// <summary>The Nth square distance.</summary>
    /// <exception cref="Standard_OutOfRange">theN is not in 1..NbExt().</exception>
    public double SquareDistance(int theN = 1)
    {
        if (theN < 1 || theN > NbExt())
        {
            throw new Standard_OutOfRange();
        }

        return Extrema_Curve2dTool.Value(myC[0]!, myPoints1[theN])
            .SquareDistance(Extrema_Curve2dTool.Value(myC[1]!, myPoints2[theN]));
    }

    /// <summary>The points of the Nth extremum, on the first and on the second curve.</summary>
    /// <exception cref="Standard_OutOfRange">theN is not in 1..NbExt().</exception>
    public void Points(int theN, out Extrema_POnCurv2d theP1, out Extrema_POnCurv2d theP2)
    {
        if (theN < 1 || theN > NbExt())
        {
            throw new Standard_OutOfRange();
        }

        theP1 = new Extrema_POnCurv2d(myPoints1[theN], Extrema_Curve2dTool.Value(myC[0]!, myPoints1[theN]));
        theP2 = new Extrema_POnCurv2d(myPoints2[theN], Extrema_Curve2dTool.Value(myC[1]!, myPoints2[theN]));
    }
}
