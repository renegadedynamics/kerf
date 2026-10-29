// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtCC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtCC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Modified by MPS (june 96) : fix for the trap in the case line/Bezier
// Modified by MPS (may 97) : PRO 7598
//                            sort solutions to avoid returning the same
//                            solution multiple times

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// The extremum distances between two 3D curves on parameter ranges. A line against a line or conic, and two
/// circles, are solved analytically (<see cref="Extrema_ExtElC"/>); other pairs by global optimisation
/// (<see cref="Extrema_ECC"/>). Parallel curves (lines, circles) report <see cref="IsParallel"/> with one distance.
/// </summary>
/// <remarks>
/// An algorithm object (decision 117): construct it with the curves, or <c>Initialize</c> (or <c>SetCurve</c>,
/// <c>SetRange</c>, <c>SetTolerance</c>) and <c>Perform</c>. Keeps references to the curves, as OCCT does. OCCT
/// disallows copies.
/// </remarks>
public class Extrema_ExtCC
{
    private bool myIsFindSingleSolution; // Default value is false.
    private readonly Extrema_ECC myECC;
    private bool myDone;
    private bool myIsParallel;
    private readonly NCollection_Sequence<Extrema_POnCurv> mypoints = new();
    private readonly NCollection_Sequence<double> mySqDist = new();
    private readonly Adaptor3d_Curve?[] myC = new Adaptor3d_Curve?[2];
    private readonly double[] myInf = new double[2];
    private readonly double[] mySup = new double[2];
    private readonly double[] myTol = new double[2];
    private gp_Pnt myP1f = new();
    private gp_Pnt myP1l = new();
    private gp_Pnt myP2f = new();
    private gp_Pnt myP2l = new();
    private double mydist11;
    private double mydist12;
    private double mydist21;
    private double mydist22;

    /// <summary>An empty algorithm with the tolerances of the two curves.</summary>
    public Extrema_ExtCC(double TolC1 = 1.0e-10, double TolC2 = 1.0e-10)
    {
        myIsFindSingleSolution = false;
        myECC = new Extrema_ECC();
        myDone = false;
        myIsParallel = false;
        myC[0] = null;
        myC[1] = null;
        myInf[0] = myInf[1] = -Precision.Infinite();
        mySup[0] = mySup[1] = Precision.Infinite();
        myTol[0] = TolC1;
        myTol[1] = TolC2;
        mydist11 = mydist12 = mydist21 = mydist22 = RealFirst();
    }

    /// <summary>The extremum distances between C1 on [U1, U2] and C2 on [V1, V2].</summary>
    public Extrema_ExtCC(Adaptor3d_Curve C1, Adaptor3d_Curve C2, double U1, double U2, double V1, double V2,
        double TolC1 = 1.0e-10, double TolC2 = 1.0e-10)
    {
        myIsFindSingleSolution = false;
        myECC = new Extrema_ECC(C1, C2, U1, U2, V1, V2);
        myDone = false;
        SetCurve(1, C1, U1, U2);
        SetCurve(2, C2, V1, V2);
        SetTolerance(1, TolC1);
        SetTolerance(2, TolC2);
        mydist11 = mydist12 = mydist21 = mydist22 = RealFirst();
        Perform();
    }

    /// <summary>The extremum distances between C1 and C2 on their whole ranges.</summary>
    public Extrema_ExtCC(Adaptor3d_Curve C1, Adaptor3d_Curve C2, double TolC1 = 1.0e-10, double TolC2 = 1.0e-10)
    {
        myIsFindSingleSolution = false;
        myECC = new Extrema_ECC(C1, C2);
        myDone = false;
        SetCurve(1, C1, C1.FirstParameter(), C1.LastParameter());
        SetCurve(2, C2, C2.FirstParameter(), C2.LastParameter());
        SetTolerance(1, TolC1);
        SetTolerance(2, TolC2);
        mydist11 = mydist12 = mydist21 = mydist22 = RealFirst();
        Perform();
    }

    /// <summary>Sets the curves on their whole ranges and the tolerances; <c>Perform</c> then computes.</summary>
    public void Initialize(Adaptor3d_Curve C1, Adaptor3d_Curve C2, double TolC1 = 1.0e-10, double TolC2 = 1.0e-10)
    {
        // myECC will be re-initialized by Perform()
        myDone = false;
        SetCurve(1, C1, C1.FirstParameter(), C1.LastParameter());
        SetCurve(2, C2, C2.FirstParameter(), C2.LastParameter());
        SetTolerance(1, TolC1);
        SetTolerance(2, TolC2);
        mydist11 = mydist12 = mydist21 = mydist22 = RealFirst();
    }

    /// <summary>Sets C1 on [U1, U2], C2 on [V1, V2] and the tolerances; <c>Perform</c> then computes.</summary>
    public void Initialize(Adaptor3d_Curve C1, Adaptor3d_Curve C2, double U1, double U2, double V1, double V2,
        double TolC1 = 1.0e-10, double TolC2 = 1.0e-10)
    {
        // myECC will be re-initialized by Perform()
        myDone = false;
        SetCurve(1, C1, U1, U2);
        SetCurve(2, C2, V1, V2);
        SetTolerance(1, TolC1);
        SetTolerance(2, TolC2);
        mydist11 = mydist12 = mydist21 = mydist22 = RealFirst();
    }

    /// <summary>Sets the curve of rank theRank (1 or 2), keeping its range.</summary>
    /// <exception cref="Standard_OutOfRange">theRank is not 1 or 2.</exception>
    public void SetCurve(int theRank, Adaptor3d_Curve C)
    {
        Standard_OutOfRange_Raise_if(theRank < 1 || theRank > 2, "Extrema_ExtCC::SetCurve()");
        int anInd = theRank - 1;
        myC[anInd] = C;
    }

    /// <summary>Sets the curve of rank theRank (1 or 2) on [Uinf, Usup].</summary>
    /// <exception cref="Standard_OutOfRange">theRank is not 1 or 2.</exception>
    public void SetCurve(int theRank, Adaptor3d_Curve C, double Uinf, double Usup)
    {
        SetCurve(theRank, C);
        SetRange(theRank, Uinf, Usup);
    }

    /// <summary>Sets the range of the curve of rank theRank (1 or 2).</summary>
    /// <exception cref="Standard_OutOfRange">theRank is not 1 or 2.</exception>
    public void SetRange(int theRank, double Uinf, double Usup)
    {
        Standard_OutOfRange_Raise_if(theRank < 1 || theRank > 2, "Extrema_ExtCC::SetRange()");
        int anInd = theRank - 1;
        myInf[anInd] = Uinf;
        mySup[anInd] = Usup;
    }

    /// <summary>Sets the tolerance of the curve of rank theRank (1 or 2).</summary>
    /// <exception cref="Standard_OutOfRange">theRank is not 1 or 2.</exception>
    public void SetTolerance(int theRank, double theTol)
    {
        Standard_OutOfRange_Raise_if(theRank < 1 || theRank > 2, "Extrema_ExtCC::SetTolerance()");
        int anInd = theRank - 1;
        myTol[anInd] = theTol;
    }

    /// <summary>Computes the extremum distances.</summary>
    /// <exception cref="Standard_NullObject">A curve is not set.</exception>
    public void Perform()
    {
        Standard_NullObject_Raise_if(myC[0] is null || myC[1] is null, "Extrema_ExtCC::Perform()");
        myECC.SetParams(myC[0]!, myC[1]!, myInf[0], mySup[0], myInf[1], mySup[1]);
        myECC.SetTolerance(Min(myTol[0], myTol[1]));
        myECC.SetSingleSolutionFlag(GetSingleSolutionFlag());
        myDone = false;
        mypoints.Clear();
        mySqDist.Clear();
        myIsParallel = false;

        GeomAbs_CurveType type1 = myC[0]!.GetType();
        GeomAbs_CurveType type2 = myC[1]!.GetType();
        double U11, U12, U21, U22, Tol = Min(myTol[0], myTol[1]);

        U11 = myInf[0];
        U12 = mySup[0];
        U21 = myInf[1];
        U22 = mySup[1];

        if (!Precision.IsInfinite(U11))
        {
            myP1f = Extrema_CurveTool.Value(myC[0]!, U11);
        }
        if (!Precision.IsInfinite(U12))
        {
            myP1l = Extrema_CurveTool.Value(myC[0]!, U12);
        }
        if (!Precision.IsInfinite(U21))
        {
            myP2f = Extrema_CurveTool.Value(myC[1]!, U21);
        }
        if (!Precision.IsInfinite(U22))
        {
            myP2l = Extrema_CurveTool.Value(myC[1]!, U22);
        }

        if (Precision.IsInfinite(U11) || Precision.IsInfinite(U21))
        {
            mydist11 = RealLast();
        }
        else
        {
            mydist11 = myP1f.SquareDistance(myP2f);
        }
        if (Precision.IsInfinite(U11) || Precision.IsInfinite(U22))
        {
            mydist12 = RealLast();
        }
        else
        {
            mydist12 = myP1f.SquareDistance(myP2l);
        }
        if (Precision.IsInfinite(U12) || Precision.IsInfinite(U21))
        {
            mydist21 = RealLast();
        }
        else
        {
            mydist21 = myP1l.SquareDistance(myP2f);
        }
        if (Precision.IsInfinite(U12) || Precision.IsInfinite(U22))
        {
            mydist22 = RealLast();
        }
        else
        {
            mydist22 = myP1l.SquareDistance(myP2l);
        }

        // Depending on the types of curves, the algorithm is chosen:
        //- _ExtElC, when one of the curves is a line and the other is elementary,
        //    or there are two circles;
        //- _GenExtCC, in all other cases
        if ((type1 == GeomAbs_Line && type2 <= GeomAbs_Parabola)
            || (type2 == GeomAbs_Line && type1 <= GeomAbs_Parabola))
        {
            // analytical case - one curve is always a line
            int anInd1 = 0, anInd2 = 1;
            GeomAbs_CurveType aType2 = type2;
            bool isInverse = (type1 > type2);
            if (isInverse)
            {
                // algorithm uses inverse order of arguments
                anInd1 = 1;
                anInd2 = 0;
                aType2 = type1;
            }
            switch (aType2)
            {
                case GeomAbs_Line:
                {
                    Extrema_ExtElC Xtrem = new(myC[anInd1]!.Line(), myC[anInd2]!.Line(), Tol);
                    PrepareResults(Xtrem, isInverse, U11, U12, U21, U22);
                    break;
                }
                case GeomAbs_Circle:
                {
                    Extrema_ExtElC Xtrem = new(myC[anInd1]!.Line(), myC[anInd2]!.Circle(), Tol);
                    PrepareResults(Xtrem, isInverse, U11, U12, U21, U22);
                    break;
                }
                case GeomAbs_Ellipse:
                {
                    Extrema_ExtElC Xtrem = new(myC[anInd1]!.Line(), myC[anInd2]!.Ellipse());
                    PrepareResults(Xtrem, isInverse, U11, U12, U21, U22);
                    break;
                }
                case GeomAbs_Hyperbola:
                {
                    Extrema_ExtElC Xtrem = new(myC[anInd1]!.Line(), myC[anInd2]!.Hyperbola());
                    PrepareResults(Xtrem, isInverse, U11, U12, U21, U22);
                    break;
                }
                case GeomAbs_Parabola:
                {
                    Extrema_ExtElC Xtrem = new(myC[anInd1]!.Line(), myC[anInd2]!.Parabola());
                    PrepareResults(Xtrem, isInverse, U11, U12, U21, U22);
                    break;
                }
                default:
                    break;
            }
        }
        else if (type1 == GeomAbs_Circle && type2 == GeomAbs_Circle)
        {
            // analytical case - two circles
            bool bIsDone;
            Extrema_ExtElC CCXtrem = new(myC[0]!.Circle(), myC[1]!.Circle());
            bIsDone = CCXtrem.IsDone();
            if (bIsDone)
            {
                PrepareResults(CCXtrem, false, U11, U12, U21, U22);
            }
            else
            {
                myECC.Perform();
                PrepareResults(myECC, U11, U12, U21, U22);
            }
        }
        else
        {
            myECC.Perform();
            PrepareResults(myECC, U11, U12, U21, U22);
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myDone;

    /// <summary>True if the curves are parallel (lines, circles); then there is one distance and no points, unless
    /// the trimmed ranges made the algorithm choose points.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public bool IsParallel()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }

        return myIsParallel;
    }

    /// <summary>The Nth square distance.</summary>
    /// <exception cref="Standard_OutOfRange">N is not in 1..NbExt().</exception>
    public double SquareDistance(int N = 1)
    {
        if ((N < 1) || (N > NbExt()))
        {
            throw new Standard_OutOfRange();
        }
        return mySqDist.Value(N);
    }

    /// <summary>The number of extremum distances.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public int NbExt()
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        return mySqDist.Length();
    }

    /// <summary>The points of the Nth extremum, on the first and on the second curve.</summary>
    /// <exception cref="Standard_OutOfRange">N is not in 1..NbExt().</exception>
    public void Points(int N, out Extrema_POnCurv P1, out Extrema_POnCurv P2)
    {
        if (N < 1 || N > NbExt())
        {
            throw new Standard_OutOfRange();
        }

        P1 = mypoints.Value(2 * N - 1);
        P2 = mypoints.Value(2 * N);
    }

    /// <summary>The square distances between the ends of the ranges and the end points (RealLast for an infinite
    /// end).</summary>
    public void TrimmedSquareDistances(out double dist11, out double dist12, out double dist21, out double dist22,
        out gp_Pnt P11, out gp_Pnt P12, out gp_Pnt P21, out gp_Pnt P22)
    {
        dist11 = mydist11;
        dist12 = mydist12;
        dist21 = mydist21;
        dist22 = mydist22;
        P11 = myP1f;
        P12 = myP1l;
        P21 = myP2f;
        P22 = myP2l;
    }

    /// <summary>Asks for one solution only (faster, for the global optimisation).</summary>
    public void SetSingleSolutionFlag(bool theFlag) => myIsFindSingleSolution = theFlag;

    public bool GetSingleSolutionFlag() => myIsFindSingleSolution;

    // Clears all found extremas.
    // This method does not change any flags (e.g. Done or IsParallel)
    protected void ClearSolutions()
    {
        mySqDist.Clear();
        mypoints.Clear();
    }

    protected void PrepareParallelResult(double theUt11, double theUt12, double theUt21, double theUt22,
        double theSqDist)
    {
        if (!myIsParallel)
        {
            return;
        }

        GeomAbs_CurveType aType1 = Extrema_CurveTool.GetType(myC[0]!);
        GeomAbs_CurveType aType2 = Extrema_CurveTool.GetType(myC[1]!);
        if (((aType1 != GeomAbs_Line) && (aType1 != GeomAbs_Circle))
            || ((aType2 != GeomAbs_Line) && (aType2 != GeomAbs_Circle)))
        {
            mySqDist.Append(theSqDist);
            myDone = true;
            myIsParallel = true;
            return;
        }

        // Parallel case is only for line-line, circle-circle and circle-line!!!
        // But really for trimmed curves extremas can not exist!
        if (aType1 != aType2)
        {
            // The projection of the circle's location to the trimmed line must exist.
            bool isReversed = (aType1 != GeomAbs_Circle);
            gp_Pnt aPonC = !isReversed ? Extrema_CurveTool.Value(myC[0]!, theUt11)
                                       : Extrema_CurveTool.Value(myC[1]!, theUt21);

            gp_Lin aL = myC[!isReversed ? 1 : 0]!.Line();
            Extrema_ExtPElC ExtPLin = new(aPonC,
                                          aL,
                                          Precision.Confusion(),
                                          !isReversed ? theUt21 : theUt11,
                                          !isReversed ? theUt22 : theUt12);

            if (ExtPLin.IsDone())
            {
                mySqDist.Append(theSqDist);
            }
            else
            {
                myIsParallel = false;
            }

            return;
        }

        if (aType1 == GeomAbs_Line)
        {
            // Line - Line

            // OCCT declares these two flags as double
            bool isFirstInfinite = (Precision.IsInfinite(theUt11) && Precision.IsInfinite(theUt12));
            bool isLastInfinite = (Precision.IsInfinite(theUt21) && Precision.IsInfinite(theUt22));

            if (isFirstInfinite || isLastInfinite)
            {
                // Infinite number of solution

                mySqDist.Append(theSqDist);
            }
            else
            {
                // The range created by projection of both ends of the 1st line
                // to the 2nd one must intersect the (native) trimmed range of
                // the 2nd line.

                myIsParallel = false;

                gp_Lin aLin1 = myC[0]!.Line();
                gp_Lin aLin2 = myC[1]!.Line();
                bool isOpposite = (aLin1.Direction().Dot(aLin2.Direction()) < 0.0);

                Bnd_Range aRange2 = new(theUt21, theUt22);
                Bnd_Range aProjRng12 = new();

                if (Precision.IsInfinite(theUt11))
                {
                    if (isOpposite)
                    {
                        aProjRng12.Add(Precision.Infinite());
                    }
                    else
                    {
                        aProjRng12.Add(-Precision.Infinite());
                    }
                }
                else
                {
                    gp_Pnt aPonC1 = ElCLib.Value(theUt11, aLin1);
                    double aPar = ElCLib.Parameter(aLin2, aPonC1);
                    aProjRng12.Add(aPar);
                }

                if (Precision.IsInfinite(theUt12))
                {
                    if (isOpposite)
                    {
                        aProjRng12.Add(-Precision.Infinite());
                    }
                    else
                    {
                        aProjRng12.Add(Precision.Infinite());
                    }
                }
                else
                {
                    gp_Pnt aPonC1 = ElCLib.Value(theUt12, aLin1);
                    double aPar = ElCLib.Parameter(aLin2, aPonC1);
                    aProjRng12.Add(aPar);
                }

                aRange2.Common(aProjRng12);
                if (aRange2.Delta() > Precision.Confusion())
                {
                    ClearSolutions();
                    mySqDist.Append(theSqDist);
                    myIsParallel = true;
                }
                else if (!aRange2.IsVoid())
                {
                    // Case like this:

                    //  **************     aLin1
                    //               o
                    //               o
                    //               ***************  aLin2

                    ClearSolutions();
                    double aPar1 = 0.0, aPar2 = 0.0;
                    aRange2.GetBounds(ref aPar1, ref aPar2);
                    aPar2 = 0.5 * (aPar1 + aPar2);
                    gp_Pnt aP = ElCLib.Value(aPar2, aLin2);
                    Extrema_POnCurv aP2 = new(aPar2, aP);
                    aPar1 = ElCLib.Parameter(aLin1, aP);
                    aP = ElCLib.Value(aPar1, aLin1);
                    Extrema_POnCurv aP1 = new(aPar1, aP);
                    mypoints.Append(aP1);
                    mypoints.Append(aP2);
                    mySqDist.Append(theSqDist);
                }
                else
                {
                    // Case like this:

                    //  **************     aLin1
                    //                 o
                    //                  o
                    //                   ***********  aLin2
                    //
                    // Take minimal trimmed distance
                    double aDmin;
                    ReadOnlySpan<double> aDists = [mydist11, mydist12, mydist21, mydist22];
                    Extrema_POnCurv aP1 = new(), aP2 = new();
                    aDmin = aDists[0];
                    int i, imin = 0;
                    for (i = 1; i < 4; ++i)
                    {
                        if (aDmin > aDists[i])
                        {
                            aDmin = aDists[i];
                            imin = i;
                        }
                    }
                    if (imin == 0)
                    {
                        aP1.SetValues(myInf[0], myP1f);
                        aP2.SetValues(myInf[1], myP2f);
                    }
                    else if (imin == 1)
                    {
                        aP1.SetValues(myInf[0], myP1f);
                        aP2.SetValues(mySup[1], myP2l);
                    }
                    else if (imin == 2)
                    {
                        aP1.SetValues(mySup[0], myP1l);
                        aP2.SetValues(myInf[1], myP2f);
                    }
                    else
                    {
                        aP1.SetValues(mySup[0], myP1l);
                        aP2.SetValues(mySup[1], myP2l);
                    }
                    ClearSolutions();
                    mypoints.Append(aP1);
                    mypoints.Append(aP2);
                    mySqDist.Append(aDmin);
                }
            }
        }
        else
        {
            // Circle - Circle
            myIsParallel = false;

            // Two arcs with ranges [U1, U2] and [V1, V2] correspondingly are
            // considered to be parallel in the following case:
            //   The range created by projection both points U1 and U2 of the
            //   1st circle to the 2nd one intersects either the range [V1, V2] or
            //   the range [V1-PI, V2-PI]. All ranges must be adjusted to correspond
            //   periodic range before checking of intersection.

            gp_Circ aWorkCirc = myC[1]!.Circle();
            double aPeriod = M_PI + M_PI;
            gp_Vec aVTg1;
            gp_Pnt aP11;
            gp_Pnt aP12 = Extrema_CurveTool.Value(myC[0]!, theUt12);
            Extrema_CurveTool.D1(myC[0]!, theUt11, out aP11, out aVTg1);

            Bnd_Range aRange = new(theUt21, theUt22);
            Bnd_Range aProjRng1 = new();

            // Precision of the calculation depends on circles radii
            double aPrecision = Max(Epsilon(myC[0]!.Circle().Radius()), Epsilon(myC[1]!.Circle().Radius()));

            // Project arc of the 1st circle between points theUt11 and theUt12 to the
            // 2nd circle. It is necessary to chose correct arc from two possible ones.

            double aPar1 = ElCLib.InPeriod(ElCLib.Parameter(aWorkCirc, aP11), theUt21, theUt21 + aPeriod);
            gp_Vec aVTg2 = Extrema_CurveTool.DN(myC[1]!, aPar1, 1);

            // Check if circles have same/opposite directions
            bool isOpposite = (aVTg1.Dot(aVTg2) < 0.0);

            double aPar2 = ElCLib.InPeriod(ElCLib.Parameter(aWorkCirc, aP12), theUt21, theUt21 + aPeriod);

            if (isOpposite)
            {
                // Must be aPar2 < aPar1
                if ((aRange.Delta() > Precision.Angular()) && ((aPar1 - aPar2) < Precision.Angular()))
                {
                    aPar2 -= aPeriod;
                }
            }
            else
            {
                // Must be aPar2 > aPar1
                if ((aRange.Delta() > Precision.Angular()) && ((aPar2 - aPar1) < Precision.Angular()))
                {
                    aPar2 += aPeriod;
                }
            }

            // Now the projection result is the range [aPar1, aPar2]
            // if aPar1 < aPar2 or the range [aPar2, aPar1], otherwise.

            double aMinSquareDist = RealLast();

            aProjRng1.Add(aPar1 - aPeriod);
            aProjRng1.Add(aPar2 - aPeriod);
            for (int i = 0; i < 3; i++)
            {
                // Repeat computation three times, shifting the range to PI on each step,
                // to be able to find if the concentric arcs ranges are intersected in just one parameter
                // (lower or upper boundary).

                Bnd_Range aRng = aProjRng1;
                aRng.Common(aRange);

                // Cases are possible and processed below:
                // 1. Extrema does not exist. In this case all common ranges are VOID.
                // 2. Arcs are parallel and distance between them is equal to sqrt(theSqDist).
                //     In this case myIsParallel = TRUE definitely.
                // 3. Arcs are parallel and distance between them is equal to (sqrt(theSqDist) + R),
                //     where R is the least radius of the both circles. In this case myIsParallel flag
                //     will temporary be set to TRUE but check will be continued until less
                //     distance will be found. At that, region with the least distance can be
                //     either a local point or continuous range. In 1st case myIsParallel = FALSE and
                //     several (or single) extremas will be returned. In the 2nd one
                //     myIsParallel = TRUE and only the least distance will be returned.
                // 4. Arcs are not parallel. Then several (or single) extremas will be returned.

                if (aRng.Delta() > Precision.Confusion())
                {
                    double aPar = 0.0;
                    aRng.GetIntermediatePoint(0.5, ref aPar);
                    gp_Pnt aPCirc2 = ElCLib.Value(aPar, aWorkCirc);
                    Extrema_ExtPElC ExtPCir = new(aPCirc2,
                                                  Extrema_CurveTool.Circle(myC[0]!),
                                                  Precision.Confusion(),
                                                  theUt11,
                                                  theUt12);
                    if (ExtPCir.NbExt() < 1)
                    {
                        // OCCT: this skips the shift of the projected range at the end of the loop, so the next
                        // two iterations test the same range again
                        continue;
                    }
                    double aMinSqD = ExtPCir.SquareDistance(1);
                    for (int anExtID = 2; anExtID <= ExtPCir.NbExt(); anExtID++)
                    {
                        aMinSqD = Min(aMinSqD, ExtPCir.SquareDistance(anExtID));
                    }

                    if (aMinSqD <= aMinSquareDist + (1.0 + aMinSqD) * aPrecision)
                    {
                        ClearSolutions();
                        mySqDist.Append(aMinSqD);
                        myIsParallel = true;

                        double aDeltaSqDist = aMinSqD - theSqDist;
                        double aSqD = Max(aMinSqD, theSqDist);

                        //  0 <= Dist1-Dist2 <= Eps
                        //  0 <= Dist1^2 - Dist2^2 < Eps*(Dist1+Dist2)

                        // If Dist1 ~= Dist2 ==> Dist1+Dist2 ~= 2*Dist2.
                        // Consequently,
                        //   0 <= Dist1^2 - Dist2^2 <= 2*Dist2*Eps

                        // Or
                        //   (Dist1^2 - Dist2^2)^2 <= 4*Dist2^2*Eps^2

                        if (aDeltaSqDist * aDeltaSqDist < 4.0 * aSqD * Precision.SquareConfusion())
                        {
                            // New solution is found
                            break;
                        }
                    }
                    // Nearer solution can be found
                }
                else if (!aRng.IsVoid())
                {
                    // Check cases like this:

                    //  **************     aCirc1
                    //               o
                    //               o
                    //               ***************  aCirc2

                    double aPar = 0.0;
                    aRng.GetIntermediatePoint(0.5, ref aPar);
                    gp_Pnt aPCirc2 = ElCLib.Value(aPar, aWorkCirc);
                    Extrema_POnCurv aP2 = new(aPar, aPCirc2);

                    Extrema_ExtPElC ExtPCir = new(aPCirc2,
                                                  Extrema_CurveTool.Circle(myC[0]!),
                                                  Precision.Confusion(),
                                                  theUt11,
                                                  theUt12);

                    bool isFound = !myIsParallel;

                    if (!isFound)
                    {
                        // If the flag myIsParallel was set earlier then it does not mean that
                        // we have found the minimal distance. Here we check it. If there is
                        // a pair of points, which are in less distance then myIsParallel flag
                        // was unset and the algorithm will return these nearest points.

                        for (int anExtID = 1; anExtID <= ExtPCir.NbExt(); anExtID++)
                        {
                            if (ExtPCir.SquareDistance(anExtID) < aMinSquareDist)
                            {
                                isFound = true;
                                break;
                            }
                        }
                    }

                    if (isFound)
                    {
                        ClearSolutions();
                        myIsParallel = false;
                        for (int anExtID = 1; anExtID <= ExtPCir.NbExt(); anExtID++)
                        {
                            mypoints.Append(ExtPCir.Point(anExtID));
                            mypoints.Append(aP2);
                            mySqDist.Append(ExtPCir.SquareDistance(anExtID));
                            aMinSquareDist = Min(aMinSquareDist, ExtPCir.SquareDistance(anExtID));
                        }
                    }
                }
                else
                {
                    // Case like this:

                    //  **************     Cir1
                    //                 o
                    //                  o
                    //                   ***********  Cir2
                    //
                    // Take minimal trimmed distance
                    myIsParallel = false;
                    double aDmin;
                    ReadOnlySpan<double> aDists = [mydist11, mydist12, mydist21, mydist22];
                    Extrema_POnCurv aP1 = new(), aP2 = new();
                    aDmin = aDists[0];
                    int k, imin = 0;
                    for (k = 1; k < 4; ++k)
                    {
                        if (aDmin > aDists[k])
                        {
                            aDmin = aDists[k];
                            imin = k;
                        }
                    }
                    if (aDmin <= aMinSquareDist + (1.0 + aDmin) * aPrecision)
                    {
                        if (imin == 0)
                        {
                            aP1.SetValues(myInf[0], myP1f);
                            aP2.SetValues(myInf[1], myP2f);
                        }
                        else if (imin == 1)
                        {
                            aP1.SetValues(myInf[0], myP1f);
                            aP2.SetValues(mySup[1], myP2l);
                        }
                        else if (imin == 2)
                        {
                            aP1.SetValues(mySup[0], myP1l);
                            aP2.SetValues(myInf[1], myP2f);
                        }
                        else
                        {
                            aP1.SetValues(mySup[0], myP1l);
                            aP2.SetValues(mySup[1], myP2l);
                        }
                        ClearSolutions();
                        mypoints.Append(aP1);
                        mypoints.Append(aP2);
                        mySqDist.Append(aDmin);
                        aMinSquareDist = Min(aMinSquareDist, aDmin);
                    }
                }
                aProjRng1.Shift(M_PI);
            }
        }
    }

    protected void PrepareResults(Extrema_ExtElC AlgExt, bool theIsInverse, double Ut11, double Ut12, double Ut21,
        double Ut22)
    {
        int i, NbExt;
        double Val, U, U2;
        Extrema_POnCurv P1, P2;

        myDone = AlgExt.IsDone();
        if (myDone)
        {
            myIsParallel = AlgExt.IsParallel();
            if (myIsParallel)
            {
                PrepareParallelResult(Ut11, Ut12, Ut21, Ut22, AlgExt.SquareDistance());
            }
            else
            {
                NbExt = AlgExt.NbExt();
                for (i = 1; i <= NbExt; i++)
                {
                    // Verification of the validity of parameters
                    AlgExt.Points(i, out P1, out P2);
                    if (!theIsInverse)
                    {
                        U = P1.Parameter();
                        U2 = P2.Parameter();
                    }
                    else
                    {
                        U2 = P1.Parameter();
                        U = P2.Parameter();
                    }

                    if (Extrema_CurveTool.IsPeriodic(myC[0]!))
                    {
                        U = ElCLib.InPeriod(U, Ut11, Ut11 + Extrema_CurveTool.Period(myC[0]!));
                    }
                    if (Extrema_CurveTool.IsPeriodic(myC[1]!))
                    {
                        U2 = ElCLib.InPeriod(U2, Ut21, Ut21 + Extrema_CurveTool.Period(myC[1]!));
                    }

                    if ((U >= Ut11 - RealEpsilon()) && (U <= Ut12 + RealEpsilon())
                        && (U2 >= Ut21 - RealEpsilon()) && (U2 <= Ut22 + RealEpsilon()))
                    {
                        Val = AlgExt.SquareDistance(i);
                        mySqDist.Append(Val);
                        if (!theIsInverse)
                        {
                            P1.SetValues(U, P1.Value());
                            P2.SetValues(U2, P2.Value());
                            mypoints.Append(P1);
                            mypoints.Append(P2);
                        }
                        else
                        {
                            P1.SetValues(U2, P1.Value());
                            P2.SetValues(U, P2.Value());
                            mypoints.Append(P2);
                            mypoints.Append(P1);
                        }
                    }
                }
            }
        }
    }

    protected void PrepareResults(Extrema_ECC AlgExt, double Ut11, double Ut12, double Ut21, double Ut22)
    {
        int i, NbExt;
        double Val, U, U2;
        Extrema_POnCurv P1, P2;

        myDone = AlgExt.IsDone();
        if (myDone)
        {
            myIsParallel = AlgExt.IsParallel();
            if (myIsParallel)
            {
                PrepareParallelResult(Ut11, Ut12, Ut21, Ut22, AlgExt.SquareDistance());
            }
            else
            {
                NbExt = AlgExt.NbExt();
                for (i = 1; i <= NbExt; i++)
                {
                    AlgExt.Points(i, out P1, out P2);
                    U = P1.Parameter();
                    U2 = P2.Parameter();

                    // Check points to be into param space.
                    if (Extrema_CurveTool.IsPeriodic(myC[0]!))
                    {
                        U = ElCLib.InPeriod(U, Ut11, Ut11 + Extrema_CurveTool.Period(myC[0]!));
                    }
                    if (Extrema_CurveTool.IsPeriodic(myC[1]!))
                    {
                        U2 = ElCLib.InPeriod(U2, Ut21, Ut21 + Extrema_CurveTool.Period(myC[1]!));
                    }

                    if ((U >= Ut11 - RealEpsilon()) && (U <= Ut12 + RealEpsilon())
                        && (U2 >= Ut21 - RealEpsilon()) && (U2 <= Ut22 + RealEpsilon()))
                    {
                        Val = AlgExt.SquareDistance(i);
                        mySqDist.Append(Val);
                        P1.SetValues(U, P1.Value());
                        P2.SetValues(U2, P2.Value());
                        mypoints.Append(P1);
                        mypoints.Append(P2);
                    }
                }
            }
        }
    }
}
