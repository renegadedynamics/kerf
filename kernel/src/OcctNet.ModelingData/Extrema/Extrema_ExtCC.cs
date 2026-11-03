// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtCC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtCC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Modified by MPS (june 96) : fix for the trap in the case line/Bezier
// Modified by MPS (may 97) : PRO 7598
//                            sort solutions to avoid returning the same
//                            solution multiple times

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

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
    private Point3D myP1f = new();
    private Point3D myP1l = new();
    private Point3D myP2f = new();
    private Point3D myP2l = new();
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
        myInf[0] = myInf[1] = -global::Kerf.Engine.Foundation.Precision.Infinite();
        mySup[0] = mySup[1] = global::Kerf.Engine.Foundation.Precision.Infinite();
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
    /// <exception cref="OutOfRangeException">theRank is not 1 or 2.</exception>
    public void SetCurve(int theRank, Adaptor3d_Curve C)
    {
        Standard_OutOfRange_Raise_if(theRank < 1 || theRank > 2, "Extrema_ExtCC::SetCurve()");
        int anInd = theRank - 1;
        myC[anInd] = C;
    }

    /// <summary>Sets the curve of rank theRank (1 or 2) on [Uinf, Usup].</summary>
    /// <exception cref="OutOfRangeException">theRank is not 1 or 2.</exception>
    public void SetCurve(int theRank, Adaptor3d_Curve C, double Uinf, double Usup)
    {
        SetCurve(theRank, C);
        SetRange(theRank, Uinf, Usup);
    }

    /// <summary>Sets the range of the curve of rank theRank (1 or 2).</summary>
    /// <exception cref="OutOfRangeException">theRank is not 1 or 2.</exception>
    public void SetRange(int theRank, double Uinf, double Usup)
    {
        Standard_OutOfRange_Raise_if(theRank < 1 || theRank > 2, "Extrema_ExtCC::SetRange()");
        int anInd = theRank - 1;
        myInf[anInd] = Uinf;
        mySup[anInd] = Usup;
    }

    /// <summary>Sets the tolerance of the curve of rank theRank (1 or 2).</summary>
    /// <exception cref="OutOfRangeException">theRank is not 1 or 2.</exception>
    public void SetTolerance(int theRank, double theTol)
    {
        Standard_OutOfRange_Raise_if(theRank < 1 || theRank > 2, "Extrema_ExtCC::SetTolerance()");
        int anInd = theRank - 1;
        myTol[anInd] = theTol;
    }

    /// <summary>Computes the extremum distances.</summary>
    /// <exception cref="NullObjectException">A curve is not set.</exception>
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

        CurveKind type1 = myC[0]!.GetType();
        CurveKind type2 = myC[1]!.GetType();
        double U11, U12, U21, U22, Tol = Min(myTol[0], myTol[1]);

        U11 = myInf[0];
        U12 = mySup[0];
        U21 = myInf[1];
        U22 = mySup[1];

        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(U11))
        {
            myP1f = Extrema_CurveTool.Value(myC[0]!, U11);
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(U12))
        {
            myP1l = Extrema_CurveTool.Value(myC[0]!, U12);
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(U21))
        {
            myP2f = Extrema_CurveTool.Value(myC[1]!, U21);
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(U22))
        {
            myP2l = Extrema_CurveTool.Value(myC[1]!, U22);
        }

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(U11) || global::Kerf.Engine.Foundation.Precision.IsInfinite(U21))
        {
            mydist11 = RealLast();
        }
        else
        {
            mydist11 = myP1f.SquareDistance(myP2f);
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(U11) || global::Kerf.Engine.Foundation.Precision.IsInfinite(U22))
        {
            mydist12 = RealLast();
        }
        else
        {
            mydist12 = myP1f.SquareDistance(myP2l);
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(U12) || global::Kerf.Engine.Foundation.Precision.IsInfinite(U21))
        {
            mydist21 = RealLast();
        }
        else
        {
            mydist21 = myP1l.SquareDistance(myP2f);
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(U12) || global::Kerf.Engine.Foundation.Precision.IsInfinite(U22))
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
        if ((type1 == Line && type2 <= Parabola)
            || (type2 == Line && type1 <= Parabola))
        {
            // analytical case - one curve is always a line
            int anInd1 = 0, anInd2 = 1;
            CurveKind aType2 = type2;
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
                case Line:
                {
                    Extrema_ExtElC Xtrem = new(myC[anInd1]!.Line(), myC[anInd2]!.Line(), Tol);
                    PrepareResults(Xtrem, isInverse, U11, U12, U21, U22);
                    break;
                }
                case Circle:
                {
                    Extrema_ExtElC Xtrem = new(myC[anInd1]!.Line(), myC[anInd2]!.Circle(), Tol);
                    PrepareResults(Xtrem, isInverse, U11, U12, U21, U22);
                    break;
                }
                case Ellipse:
                {
                    Extrema_ExtElC Xtrem = new(myC[anInd1]!.Line(), myC[anInd2]!.Ellipse());
                    PrepareResults(Xtrem, isInverse, U11, U12, U21, U22);
                    break;
                }
                case Hyperbola:
                {
                    Extrema_ExtElC Xtrem = new(myC[anInd1]!.Line(), myC[anInd2]!.Hyperbola());
                    PrepareResults(Xtrem, isInverse, U11, U12, U21, U22);
                    break;
                }
                case Parabola:
                {
                    Extrema_ExtElC Xtrem = new(myC[anInd1]!.Line(), myC[anInd2]!.Parabola());
                    PrepareResults(Xtrem, isInverse, U11, U12, U21, U22);
                    break;
                }
                default:
                    break;
            }
        }
        else if (type1 == Circle && type2 == Circle)
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
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public bool IsParallel()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }

        return myIsParallel;
    }

    /// <summary>The Nth square distance.</summary>
    /// <exception cref="OutOfRangeException">N is not in 1..NbExt().</exception>
    public double SquareDistance(int N = 1)
    {
        if ((N < 1) || (N > NbExt()))
        {
            throw new OutOfRangeException();
        }
        return mySqDist.Value(N);
    }

    /// <summary>The number of extremum distances.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public int NbExt()
    {
        if (!myDone)
        {
            throw new NotDoneException();
        }
        return mySqDist.Length();
    }

    /// <summary>The points of the Nth extremum, on the first and on the second curve.</summary>
    /// <exception cref="OutOfRangeException">N is not in 1..NbExt().</exception>
    public void Points(int N, out Extrema_POnCurv P1, out Extrema_POnCurv P2)
    {
        if (N < 1 || N > NbExt())
        {
            throw new OutOfRangeException();
        }

        P1 = mypoints.Value(2 * N - 1);
        P2 = mypoints.Value(2 * N);
    }

    /// <summary>The square distances between the ends of the ranges and the end points (RealLast for an infinite
    /// end).</summary>
    public void TrimmedSquareDistances(out double dist11, out double dist12, out double dist21, out double dist22,
        out Point3D P11, out Point3D P12, out Point3D P21, out Point3D P22)
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

        CurveKind aType1 = Extrema_CurveTool.GetType(myC[0]!);
        CurveKind aType2 = Extrema_CurveTool.GetType(myC[1]!);
        if (((aType1 != Line) && (aType1 != Circle))
            || ((aType2 != Line) && (aType2 != Circle)))
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
            bool isReversed = (aType1 != Circle);
            Point3D aPonC = !isReversed ? Extrema_CurveTool.Value(myC[0]!, theUt11)
                                       : Extrema_CurveTool.Value(myC[1]!, theUt21);

            Line3D aL = myC[!isReversed ? 1 : 0]!.Line();
            Extrema_ExtPElC ExtPLin = new(aPonC,
                                          aL,
                                          global::Kerf.Engine.Foundation.Precision.Confusion(),
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

        if (aType1 == Line)
        {
            // Line - Line

            // OCCT declares these two flags as double
            bool isFirstInfinite = (global::Kerf.Engine.Foundation.Precision.IsInfinite(theUt11) && global::Kerf.Engine.Foundation.Precision.IsInfinite(theUt12));
            bool isLastInfinite = (global::Kerf.Engine.Foundation.Precision.IsInfinite(theUt21) && global::Kerf.Engine.Foundation.Precision.IsInfinite(theUt22));

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

                Line3D aLin1 = myC[0]!.Line();
                Line3D aLin2 = myC[1]!.Line();
                bool isOpposite = (aLin1.Direction().Dot(aLin2.Direction()) < 0.0);

                BoundRange aRange2 = new(theUt21, theUt22);
                BoundRange aProjRng12 = new();

                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(theUt11))
                {
                    if (isOpposite)
                    {
                        aProjRng12.Add(global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                    else
                    {
                        aProjRng12.Add(-global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                }
                else
                {
                    Point3D aPonC1 = global::Kerf.Engine.Geometry.ElCLib.Value(theUt11, aLin1);
                    double aPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(aLin2, aPonC1);
                    aProjRng12.Add(aPar);
                }

                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(theUt12))
                {
                    if (isOpposite)
                    {
                        aProjRng12.Add(-global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                    else
                    {
                        aProjRng12.Add(global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                }
                else
                {
                    Point3D aPonC1 = global::Kerf.Engine.Geometry.ElCLib.Value(theUt12, aLin1);
                    double aPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(aLin2, aPonC1);
                    aProjRng12.Add(aPar);
                }

                aRange2.Common(aProjRng12);
                if (aRange2.Delta() > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    ClearSolutions();
                    mySqDist.Append(theSqDist);
                    myIsParallel = true;
                }
                else if (!aRange2.IsVoid)
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
                    Point3D aP = global::Kerf.Engine.Geometry.ElCLib.Value(aPar2, aLin2);
                    Extrema_POnCurv aP2 = new(aPar2, aP);
                    aPar1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(aLin1, aP);
                    aP = global::Kerf.Engine.Geometry.ElCLib.Value(aPar1, aLin1);
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

            Circle3D aWorkCirc = myC[1]!.Circle();
            double aPeriod = M_PI + M_PI;
            Vector3D aVTg1;
            Point3D aP11;
            Point3D aP12 = Extrema_CurveTool.Value(myC[0]!, theUt12);
            Extrema_CurveTool.D1(myC[0]!, theUt11, out aP11, out aVTg1);

            BoundRange aRange = new(theUt21, theUt22);
            BoundRange aProjRng1 = new();

            // Precision of the calculation depends on circles radii
            double aPrecision = Max(Epsilon(myC[0]!.Circle().Radius()), Epsilon(myC[1]!.Circle().Radius()));

            // Project arc of the 1st circle between points theUt11 and theUt12 to the
            // 2nd circle. It is necessary to chose correct arc from two possible ones.

            double aPar1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(global::Kerf.Engine.Geometry.ElCLib.Parameter(aWorkCirc, aP11), theUt21, theUt21 + aPeriod);
            Vector3D aVTg2 = Extrema_CurveTool.DN(myC[1]!, aPar1, 1);

            // Check if circles have same/opposite directions
            bool isOpposite = (aVTg1.Dot(aVTg2) < 0.0);

            double aPar2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(global::Kerf.Engine.Geometry.ElCLib.Parameter(aWorkCirc, aP12), theUt21, theUt21 + aPeriod);

            if (isOpposite)
            {
                // Must be aPar2 < aPar1
                if ((aRange.Delta() > global::Kerf.Engine.Foundation.Precision.Angular()) && ((aPar1 - aPar2) < global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    aPar2 -= aPeriod;
                }
            }
            else
            {
                // Must be aPar2 > aPar1
                if ((aRange.Delta() > global::Kerf.Engine.Foundation.Precision.Angular()) && ((aPar2 - aPar1) < global::Kerf.Engine.Foundation.Precision.Angular()))
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

                BoundRange aRng = aProjRng1;
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

                if (aRng.Delta() > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    double aPar = 0.0;
                    aRng.GetIntermediatePoint(0.5, ref aPar);
                    Point3D aPCirc2 = global::Kerf.Engine.Geometry.ElCLib.Value(aPar, aWorkCirc);
                    Extrema_ExtPElC ExtPCir = new(aPCirc2,
                                                  Extrema_CurveTool.Circle(myC[0]!),
                                                  global::Kerf.Engine.Foundation.Precision.Confusion(),
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

                        if (aDeltaSqDist * aDeltaSqDist < 4.0 * aSqD * global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                        {
                            // New solution is found
                            break;
                        }
                    }
                    // Nearer solution can be found
                }
                else if (!aRng.IsVoid)
                {
                    // Check cases like this:

                    //  **************     aCirc1
                    //               o
                    //               o
                    //               ***************  aCirc2

                    double aPar = 0.0;
                    aRng.GetIntermediatePoint(0.5, ref aPar);
                    Point3D aPCirc2 = global::Kerf.Engine.Geometry.ElCLib.Value(aPar, aWorkCirc);
                    Extrema_POnCurv aP2 = new(aPar, aPCirc2);

                    Extrema_ExtPElC ExtPCir = new(aPCirc2,
                                                  Extrema_CurveTool.Circle(myC[0]!),
                                                  global::Kerf.Engine.Foundation.Precision.Confusion(),
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
                        U = global::Kerf.Engine.Geometry.ElCLib.InPeriod(U, Ut11, Ut11 + Extrema_CurveTool.Period(myC[0]!));
                    }
                    if (Extrema_CurveTool.IsPeriodic(myC[1]!))
                    {
                        U2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(U2, Ut21, Ut21 + Extrema_CurveTool.Period(myC[1]!));
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
                        U = global::Kerf.Engine.Geometry.ElCLib.InPeriod(U, Ut11, Ut11 + Extrema_CurveTool.Period(myC[0]!));
                    }
                    if (Extrema_CurveTool.IsPeriodic(myC[1]!))
                    {
                        U2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(U2, Ut21, Ut21 + Extrema_CurveTool.Period(myC[1]!));
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
