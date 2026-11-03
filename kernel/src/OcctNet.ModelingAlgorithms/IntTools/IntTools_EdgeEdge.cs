// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_EdgeEdge.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_EdgeEdge.lxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_EdgeEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The edge/edge intersection algorithm, based on the intersection of the edges' bounding boxes: it finds the common
/// parts (vertices and edges) of two edges on their ranges.
/// </summary>
/// <remarks>
/// A working object (plan section 6), derived from by the pave filler's <c>BOPAlgo_EdgeEdge</c> (unit 6E), which
/// reads and restores its protected members. The common parts are the algorithm's own
/// <see cref="IntTools_CommonPrt"/> objects (a class, decision 620): callers copy one with
/// <c>new IntTools_CommonPrt(x)</c> to keep it.
/// </remarks>
public class IntTools_EdgeEdge
{
    protected Edge myEdge1 = new();
    protected Edge myEdge2 = new();
    protected Curve3D? myGeom1;
    protected Curve3D? myGeom2;
    protected EdgeCurveAdaptor myCurve1 = new();
    protected EdgeCurveAdaptor myCurve2 = new();
    protected double myTol1;
    protected double myTol2;
    protected double myTol;
    protected double myFuzzyValue;
    protected double myRes1;
    protected double myRes2;
    protected double myResCoeff1;
    protected double myResCoeff2;
    protected double myPTol1;
    protected double myPTol2;
    protected IntTools_Range myRange1;
    protected IntTools_Range myRange2;
    protected bool mySwap;
    protected int myErrorStatus;
    protected readonly NCollection_Sequence<IntTools_CommonPrt> myCommonParts = new();

    /// <summary>
    /// Allows avoiding the use of the edge/edge intersection algorithm (speeding up the Boolean algorithm) if the edges
    /// really coincide. If it is not evident, the flag should not be set (the Boolean algorithm would be slower).
    /// </summary>
    protected bool myQuickCoincidenceCheck;

    /// <summary>An empty algorithm.</summary>
    public IntTools_EdgeEdge()
    {
        myTol1 = 0.0;
        myTol2 = 0.0;
        myTol = 0.0;
        myFuzzyValue = global::Kerf.Engine.Foundation.Precision.Confusion();
        myRes1 = 0.0;
        myRes2 = 0.0;
        myResCoeff1 = 0.0;
        myResCoeff2 = 0.0;
        myPTol1 = 0.0;
        myPTol2 = 0.0;
        myRange1 = new IntTools_Range(0.0, 0.0);
        myRange2 = new IntTools_Range(0.0, 0.0);
        mySwap = false;
        myErrorStatus = 0;
        myQuickCoincidenceCheck = false;
    }

    /// <summary>The intersection of the edges on their whole ranges.</summary>
    public IntTools_EdgeEdge(in Edge theEdge1, in Edge theEdge2)
        : this()
    {
        myEdge1 = theEdge1;
        myEdge2 = theEdge2;
    }

    /// <summary>The intersection of the edges on the given ranges.</summary>
    public IntTools_EdgeEdge(in Edge theEdge1, double aT11, double aT12, in Edge theEdge2, double aT21,
                             double aT22)
        : this()
    {
        myEdge1 = theEdge1;
        myEdge2 = theEdge2;
        myRange1 = new IntTools_Range(aT11, aT12);
        myRange2 = new IntTools_Range(aT21, aT22);
    }

    /// <summary>Sets the first edge.</summary>
    public void SetEdge1(in Edge theEdge) => myEdge1 = theEdge;

    /// <summary>Sets the first edge and its range.</summary>
    public void SetEdge1(in Edge theEdge, double aT1, double aT2)
    {
        SetEdge1(theEdge);
        SetRange1(aT1, aT2);
    }

    /// <summary>Sets the range of the first edge.</summary>
    public void SetRange1(in IntTools_Range theRange) => myRange1 = theRange;

    /// <summary>Sets the range of the first edge.</summary>
    public void SetRange1(double aT1, double aT2)
    {
        myRange1.SetFirst(aT1);
        myRange1.SetLast(aT2);
    }

    /// <summary>Sets the second edge.</summary>
    public void SetEdge2(in Edge theEdge) => myEdge2 = theEdge;

    /// <summary>Sets the second edge and its range.</summary>
    public void SetEdge2(in Edge theEdge, double aT1, double aT2)
    {
        SetEdge2(theEdge);
        SetRange2(aT1, aT2);
    }

    /// <summary>Sets the range of the second edge.</summary>
    public void SetRange2(in IntTools_Range theRange) => myRange2 = theRange;

    /// <summary>Sets the range of the second edge.</summary>
    public void SetRange2(double aT1, double aT2)
    {
        myRange2.SetFirst(aT1);
        myRange2.SetLast(aT2);
    }

    /// <summary>Sets the fuzzy value (at least <see cref="Precision.Confusion"/>).</summary>
    public void SetFuzzyValue(double theFuzz) => myFuzzyValue = Max(theFuzz, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <summary>True if the computation succeeded (common parts may still be none).</summary>
    public bool IsDone() => myErrorStatus == 0;

    /// <summary>The fuzzy value.</summary>
    public double FuzzyValue() => myFuzzyValue;

    /// <summary>The common parts (the algorithm's own sequence; ported code only reads it).</summary>
    public NCollection_Sequence<IntTools_CommonPrt> CommonParts() => myCommonParts;

    /// <summary>Sets the flag of the quick coincidence check.</summary>
    public void UseQuickCoincidenceCheck(bool bFlag) => myQuickCoincidenceCheck = bFlag;

    /// <summary>The flag of the quick coincidence check.</summary>
    public bool IsCoincidenceCheckedQuickly() => myQuickCoincidenceCheck;

    /// <summary>Checks the data.</summary>
    protected void CheckData()
    {
        if (myEdge1.IsNull || myEdge2.IsNull)
        {
            myErrorStatus = 1;
            return;
        }
        //
        if (Tool.Degenerated(myEdge1) || Tool.Degenerated(myEdge2))
        {
            myErrorStatus = 2;
            return;
        }
        //
        if (!Tool.IsGeometric(myEdge1) || !Tool.IsGeometric(myEdge2))
        {
            myErrorStatus = 3;
            return;
        }
    }

    /// <summary>Prepares the data.</summary>
    protected void Prepare()
    {
        CurveKind aCT1, aCT2;
        int iCT1, iCT2;
        //
        myCurve1.Initialize(myEdge1);
        myCurve2.Initialize(myEdge2);
        //
        if (myRange1.First() == 0.0 && myRange1.Last() == 0.0)
        {
            myRange1.SetFirst(myCurve1.FirstParameter());
            myRange1.SetLast(myCurve1.LastParameter());
        }
        //
        if (myRange2.First() == 0.0 && myRange2.Last() == 0.0)
        {
            myRange2.SetFirst(myCurve2.FirstParameter());
            myRange2.SetLast(myCurve2.LastParameter());
        }
        //
        aCT1 = myCurve1.GetType();
        aCT2 = myCurve2.GetType();
        //
        iCT1 = TypeToInteger(aCT1);
        iCT2 = TypeToInteger(aCT2);
        //
        if (iCT1 == iCT2)
        {
            if (iCT1 != 0)
            {
                // compute deflection
                double aC1, aC2;
                //
                aC2 = CurveDeflection(myCurve2, myRange2);
                aC1 = (aC2 > global::Kerf.Engine.Foundation.Precision.Confusion()) ? CurveDeflection(myCurve1, myRange1) : 1.0;
                //
                if (aC1 < aC2)
                {
                    --iCT1;
                }
            }
        }
        //
        if (iCT1 < iCT2)
        {
            Edge tmpE = myEdge1;
            myEdge1 = myEdge2;
            myEdge2 = tmpE;
            //
            EdgeCurveAdaptor tmpC = myCurve1;
            myCurve1 = myCurve2;
            myCurve2 = tmpC;
            //
            IntTools_Range tmpR = myRange1;
            myRange1 = myRange2;
            myRange2 = tmpR;
            //
            mySwap = true;
        }
        //
        double aTolAdd = myFuzzyValue / 2.0;
        myTol1 = myCurve1.Tolerance() + aTolAdd;
        myTol2 = myCurve2.Tolerance() + aTolAdd;
        myTol = myTol1 + myTol2;
        //
        if (iCT1 != 0 || iCT2 != 0)
        {
            double aTM;
            //
            myGeom1 = Tool.Curve(myEdge1, out _, out _);
            myGeom2 = Tool.Curve(myEdge2, out _, out _);
            //
            myResCoeff1 = ResolutionCoeff(myCurve1, myRange1);
            myResCoeff2 = ResolutionCoeff(myCurve2, myRange2);
            //
            myRes1 = Resolution(myCurve1.Curve().Curve()!, myCurve1.GetType(), myResCoeff1, myTol1);
            myRes2 = Resolution(myCurve2.Curve().Curve()!, myCurve2.GetType(), myResCoeff2, myTol2);
            //
            myPTol1 = 5.0e-13;
            aTM = Max(Math.Abs(myRange1.First()), Math.Abs(myRange1.Last()));
            if (aTM > 999.0)
            {
                myPTol1 = 5.0e-16 * aTM;
            }
            //
            myPTol2 = 5.0e-13;
            aTM = Max(Math.Abs(myRange2.First()), Math.Abs(myRange2.Last()));
            if (aTM > 999.0)
            {
                myPTol2 = 5.0e-16 * aTM;
            }
        }
    }

    /// <summary>Performs the intersection of the edges.</summary>
    public void Perform()
    {
        // 1. Check data
        CheckData();
        if (myErrorStatus != 0)
        {
            return;
        }
        //
        // 2. Prepare Data
        Prepare();
        //
        // 3.1. Check Line/Line case
        if (myCurve1.GetType() == Line && myCurve2.GetType() == Line)
        {
            ComputeLineLine();
            return;
        }
        //
        if (myQuickCoincidenceCheck)
        {
            if (IsCoincident())
            {
                double aT11, aT12, aT21, aT22;
                //
                myRange1.Range(out aT11, out aT12);
                myRange2.Range(out aT21, out aT22);
                AddSolution(aT11, aT12, aT21, aT22, ShapeType.Edge);
                return;
            }
        }
        //
        if ((myCurve1.GetType() <= Parabola && myCurve2.GetType() <= Parabola)
            && (myCurve1.GetType() == Line || myCurve2.GetType() == Line))
        {
            // Improvement of performance for cases of searching common parts between line
            // and analytical curve. This code allows to define that edges have no
            // common parts more fast, then regular algorithm (FindSolution(...))
            // Check minimal distance between edges
            BRepExtrema_DistShapeShape aMinDist = new(myEdge1, myEdge2, ExtFlag_MIN);
            if (aMinDist.IsDone())
            {
                double d = aMinDist.Value();
                if (d > 1.1 * myTol)
                {
                    return;
                }
            }
        }

        NCollection_Sequence<IntTools_Range> aRanges1 = new(), aRanges2 = new();
        //
        // 3.2. Find ranges containing solutions
        bool bSplit2;
        FindSolutions(aRanges1, aRanges2, out bSplit2);
        //
        // 4. Merge solutions and save common parts
        MergeSolutions(aRanges1, aRanges2, bSplit2);
    }

    /// <summary>Checks if the edges really coincide.</summary>
    protected bool IsCoincident()
    {
        int i, iCnt, aNbSeg, aNbP2;
        double dT, aT1, aCoeff, aTresh, aD;
        double aT11, aT12, aT21, aT22;
        GeomAPI_ProjectPointOnCurve aProjPC = new();
        Point3D aP1;
        //
        aTresh = 0.5;
        aNbSeg = 23;
        myRange1.Range(out aT11, out aT12);
        myRange2.Range(out aT21, out aT22);
        //
        aProjPC.Init(myGeom2!, aT21, aT22);
        //
        dT = (aT12 - aT11) / aNbSeg;
        //
        iCnt = 0;
        for (i = 0; i <= aNbSeg; ++i)
        {
            aT1 = aT11 + i * dT;
            myGeom1!.D0(aT1, out aP1);
            //
            aProjPC.Perform(aP1);
            aNbP2 = aProjPC.NbPoints();
            if (aNbP2 == 0)
            {
                continue;
            }
            //
            aD = aProjPC.LowerDistance();
            if (aD < myTol)
            {
                ++iCnt;
            }
        }
        //
        aCoeff = (double)iCnt / ((double)aNbSeg + 1);
        return aCoeff > aTresh;
    }

    /// <summary>Intermediate function: finds the ranges containing solutions.</summary>
    protected void FindSolutions(NCollection_Sequence<IntTools_Range> theRanges1,
                                 NCollection_Sequence<IntTools_Range> theRanges2,
                                 out bool bSplit2)
    {
        bool bIsClosed2;
        double aT11, aT12, aT21, aT22;
        Box aB1 = new(), aB2 = new();
        //
        bSplit2 = false;
        myRange1.Range(out aT11, out aT12);
        myRange2.Range(out aT21, out aT22);
        //
        bIsClosed2 = IsClosed(myGeom2!, aT21, aT22, myTol2, myRes2);
        //
        if (bIsClosed2)
        {
            BndBuildBox(myCurve1, aT11, aT12, myTol1, out aB1);
            //
            Point3D aP = myGeom2!.Value(aT21);
            bIsClosed2 = !aB1.IsOut(aP);
        }
        //
        if (!bIsClosed2)
        {
            BndBuildBox(myCurve1, aT11, aT12, myTol1, out aB1);
            BndBuildBox(myCurve2, aT21, aT22, myTol2, out aB2);
            FindSolutions(myRange1, aB1, myRange2, aB2, theRanges1, theRanges2);
            return;
        }
        //
        if (CheckCoincidence(aT11, aT12, aT21, aT22, myTol, myRes1) == 0)
        {
            theRanges1.Append(myRange1);
            theRanges2.Append(myRange2);
            return;
        }
        //
        int i, j, aNb1, aNb2;
        NCollection_Sequence<IntTools_Range> aSegments1 = new(), aSegments2 = new();
        //
        aNb1 = IsClosed(myGeom1!, aT11, aT12, myTol1, myRes1) ? 2 : 1;
        aNb2 = 2;
        //
        aNb1 = SplitRangeOnSegments(aT11, aT12, myRes1, aNb1, aSegments1);
        aNb2 = SplitRangeOnSegments(aT21, aT22, myRes2, aNb2, aSegments2);
        //
        for (i = 1; i <= aNb1; ++i)
        {
            IntTools_Range aR1 = aSegments1.Value(i);
            BndBuildBox(myCurve1, aR1.First(), aR1.Last(), myTol1, out aB1);
            for (j = 1; j <= aNb2; ++j)
            {
                IntTools_Range aR2 = aSegments2.Value(j);
                BndBuildBox(myCurve2, aR2.First(), aR2.Last(), myTol2, out aB2);
                FindSolutions(aR1, aB1, aR2, aB2, theRanges1, theRanges2);
            }
        }
        //
        bSplit2 = aNb2 > 1;
    }

    /// <summary>Looks for the exact intersection ranges.</summary>
    protected void FindSolutions(in IntTools_Range theR1,
                                 in Box theBox1,
                                 in IntTools_Range theR2,
                                 in Box theBox2,
                                 NCollection_Sequence<IntTools_Range> theRanges1,
                                 NCollection_Sequence<IntTools_Range> theRanges2)
    {
        bool bOut, bStop, bThin;
        double aT11, aT12, aT21, aT22;
        double aTB11, aTB12, aTB21, aTB22;
        double aSmallStep1, aSmallStep2;
        int iCom;
        Box aB1, aB2;
        //
        theR1.Range(out aT11, out aT12);
        theR2.Range(out aT21, out aT22);
        //
        aB1 = theBox1;
        aB2 = theBox2;
        //
        bThin = false;
        bStop = false;
        iCom = 1;
        //
        do
        {
            aTB11 = aT11;
            aTB12 = aT12;
            aTB21 = aT21;
            aTB22 = aT22;
            //
            // 1. Find parameters of the second edge in the box of first one
            bOut = aB1.IsOut(aB2);
            if (bOut)
            {
                break;
            }
            //
            bThin = ((aT12 - aT11) < myRes1) || (aB1.IsXThin(myTol) && aB1.IsYThin(myTol) && aB1.IsZThin(myTol));
            //
            bOut = !FindParameters(myCurve2, aTB21, aTB22, myTol2, myRes2, myPTol2, myResCoeff2, aB1, ref aT21, ref aT22);
            if (bOut || bThin)
            {
                break;
            }
            //
            // 2. Build box for second edge and find parameters
            //   of the first one in that box
            BndBuildBox(myCurve2, aT21, aT22, myTol2, out aB2);
            bOut = aB1.IsOut(aB2);
            if (bOut)
            {
                break;
            }
            //
            bThin = ((aT22 - aT21) < myRes2) || (aB2.IsXThin(myTol) && aB2.IsYThin(myTol) && aB2.IsZThin(myTol));
            //
            bOut = !FindParameters(myCurve1, aTB11, aTB12, myTol1, myRes1, myPTol1, myResCoeff1, aB2, ref aT11, ref aT12);
            //
            if (bOut || bThin)
            {
                break;
            }
            //
            // 3. Check if it makes sense to continue
            aSmallStep1 = (aTB12 - aTB11) / 250.0;
            aSmallStep2 = (aTB22 - aTB21) / 250.0;
            //
            if (aSmallStep1 < myRes1)
            {
                aSmallStep1 = myRes1;
            }
            if (aSmallStep2 < myRes2)
            {
                aSmallStep2 = myRes2;
            }
            //
            if (((aT11 - aTB11) < aSmallStep1) && ((aTB12 - aT12) < aSmallStep1)
                && ((aT21 - aTB21) < aSmallStep2) && ((aTB22 - aT22) < aSmallStep2))
            {
                bStop = true;
            }
            else
            {
                BndBuildBox(myCurve1, aT11, aT12, myTol1, out aB1);
            }
        } while (!bStop);
        //
        if (bOut)
        {
            // no intersection;
            return;
        }
        //
        if (!bThin)
        {
            // check curves for coincidence on the ranges
            iCom = CheckCoincidence(aT11, aT12, aT21, aT22, myTol, myRes1);
            if (iCom == 0)
            {
                bThin = true;
            }
        }
        //
        if (bThin)
        {
            if (iCom != 0)
            {
                // check intermediate points
                bool bSol;
                double aT1;
                Point3D aP1;
                GeomAPI_ProjectPointOnCurve aProjPC = new();
                //
                aT1 = (aT11 + aT12) * 0.5;
                myGeom1!.D0(aT1, out aP1);
                //
                aProjPC.Init(myGeom2!, aT21, aT22);
                aProjPC.Perform(aP1);
                //
                if (aProjPC.NbPoints() != 0)
                {
                    bSol = aProjPC.LowerDistance() <= myTol;
                }
                else
                {
                    double aT2;
                    Point3D aP2;
                    //
                    aT2 = (aT21 + aT22) * 0.5;
                    myGeom2!.D0(aT2, out aP2);
                    //
                    bSol = aP1.IsEqual(aP2, myTol);
                }
                //
                if (!bSol)
                {
                    return;
                }
            }
            // add common part
            IntTools_Range aR1 = new(aT11, aT12), aR2 = new(aT21, aT22);
            //
            theRanges1.Append(aR1);
            theRanges2.Append(aR2);
            return;
        }
        //
        if (!IsIntersection(aT11, aT12, aT21, aT22))
        {
            return;
        }
        //
        // split ranges on segments and repeat
        int i, aNb1;
        NCollection_Sequence<IntTools_Range> aSegments1 = new();
        //
        // Build box for first curve to compare
        // the boxes of the splits with this one
        BndBuildBox(myCurve1, aT11, aT12, myTol1, out aB1);
        double aB1SqExtent = aB1.SquareExtent();
        //
        IntTools_Range aR2r = new(aT21, aT22);
        BndBuildBox(myCurve2, aT21, aT22, myTol2, out aB2);
        //
        aNb1 = SplitRangeOnSegments(aT11, aT12, myRes1, 3, aSegments1);
        for (i = 1; i <= aNb1; ++i)
        {
            IntTools_Range aR1 = aSegments1.Value(i);
            BndBuildBox(myCurve1, aR1.First(), aR1.Last(), myTol1, out aB1);
            if (!aB1.IsOut(aB2) && (aNb1 == 1 || aB1.SquareExtent() < aB1SqExtent))
            {
                FindSolutions(aR1, aB1, aR2r, aB2, theRanges1, theRanges2);
            }
        }
    }

    /// <summary>Looks for the range of the edge which is in the box.</summary>
    protected static bool FindParameters(EdgeCurveAdaptor theBAC,
                                         double aT1,
                                         double aT2,
                                         double theTol,
                                         double theRes,
                                         double thePTol,
                                         double theResCoeff,
                                         in Box theCBox,
                                         ref double aTB1,
                                         ref double aTB2)
    {
        bool bRet;
        int aC, i;
        double aCf, aDiff, aDt, aT, aTB, aTOut, aTIn;
        double aDist, aDistP;
        Point3D aP;
        Box aCBx;
        //
        bRet = false;
        aCf = 0.6180339887498948482045868343656; // =0.5*(1.+sqrt(5.))/2.;
        aCBx = theCBox;
        aCBx.SetGap(aCBx.GetGap() + theTol);
        //
        Curve3D aCurve = theBAC.Curve().Curve()!;
        CurveKind aCurveType = theBAC.GetType();
        double aMaxDt = (aT2 - aT1) * 0.01;
        //
        for (i = 0; i < 2; ++i)
        {
            aTB = i == 0 ? aT1 : aT2;
            aT = i == 0 ? aT2 : aTB1;
            aC = i == 0 ? 1 : -1;
            aDt = theRes;
            aDistP = 0.0;
            bRet = false;
            double k = 1;
            // looking for the point on the edge which is in the box;
            while (aC * (aT - aTB) >= 0)
            {
                theBAC.D0(aTB, out aP);
                aDist = PointBoxDistance(theCBox, aP);
                if (aDist > theTol)
                {
                    if (aDistP > 0.0)
                    {
                        bool toGrow = false;
                        if (Math.Abs(aDistP - aDist) / aDistP < 0.1)
                        {
                            aDt = Resolution(aCurve, aCurveType, theResCoeff, k * aDist);
                            if (aDt < aMaxDt)
                            {
                                toGrow = true;
                                k *= 2;
                            }
                        }
                        if (!toGrow)
                        {
                            k = 1;
                            aDt = Resolution(aCurve, aCurveType, theResCoeff, aDist);
                        }
                    }
                    aTB += aC * aDt;
                }
                else
                {
                    bRet = true;
                    break;
                }
                aDistP = aDist;
            }
            //
            if (!bRet)
            {
                if (i == 0)
                {
                    // edge is out of the box;
                    return bRet;
                }
                else
                {
                    bRet = !bRet;
                    aTB = aTB1;
                    aDt = aT2 - aTB1;
                }
            }
            //
            aT = i == 0 ? aT1 : aT2;
            if (aTB != aT)
            {
                // one point IN, one point OUT; looking for the bounding point;
                aTIn = aTB;
                aTOut = aTB - aC * aDt;
                aDiff = aTIn - aTOut;
                while (Math.Abs(aDiff) > thePTol)
                {
                    aTB = aTOut + aDiff * aCf;
                    theBAC.D0(aTB, out aP);
                    if (aCBx.IsOut(aP))
                    {
                        aTOut = aTB;
                    }
                    else
                    {
                        aTIn = aTB;
                    }
                    aDiff = aTIn - aTOut;
                }
            }
            if (i == 0)
            {
                aTB1 = aTB;
            }
            else
            {
                aTB2 = aTB;
            }
        }
        return bRet;
    }

    /// <summary>Merges the found solutions into common parts.</summary>
    protected void MergeSolutions(NCollection_Sequence<IntTools_Range> theRanges1,
                                  NCollection_Sequence<IntTools_Range> theRanges2,
                                  bool bSplit2)
    {
        int aNbCP = theRanges1.Length();
        if (aNbCP == 0)
        {
            return;
        }
        //
        IntTools_Range aRi1, aRi2, aRj1, aRj2;
        bool bCond;
        int i, j;
        ShapeType aType;
        double aT11, aT12, aT21, aT22;
        double aTi11, aTi12, aTi21, aTi22;
        double aTj11, aTj12, aTj21, aTj22;
        double aRes1, aRes2, dTR1, dTR2;
        NCollection_Map<int> aMI = new();
        //
        aRes1 = Resolution(myCurve1.Curve().Curve()!, myCurve1.GetType(), myResCoeff1, myTol);
        aRes2 = Resolution(myCurve2.Curve().Curve()!, myCurve2.GetType(), myResCoeff2, myTol);
        //
        myRange1.Range(out aT11, out aT12);
        myRange2.Range(out aT21, out aT22);
        dTR1 = 20 * aRes1;
        dTR2 = 20 * aRes2;
        aType = ShapeType.Vertex;
        //
        for (i = 1; i <= aNbCP;)
        {
            if (aMI.Contains(i))
            {
                ++i;
                continue;
            }
            //
            aRi1 = theRanges1.Value(i);
            aRi2 = theRanges2.Value(i);
            //
            aRi1.Range(out aTi11, out aTi12);
            aRi2.Range(out aTi21, out aTi22);
            //
            aMI.Add(i);
            //
            for (j = i + 1; j <= aNbCP; ++j)
            {
                if (aMI.Contains(j))
                {
                    continue;
                }
                //
                aRj1 = theRanges1.Value(j);
                aRj2 = theRanges2.Value(j);
                //
                aRj1.Range(out aTj11, out aTj12);
                aRj2.Range(out aTj21, out aTj22);
                //
                bCond = (Math.Abs(aTi12 - aTj11) < dTR1) || (aTj11 > aTi11 && aTj11 < aTi12)
                        || (aTi11 > aTj11 && aTi11 < aTj12) || (bSplit2 && (Math.Abs(aTj12 - aTi11) < dTR1));
                if (bCond && bSplit2)
                {
                    bCond = (Math.Abs((Max(aTi22, aTj22) - Min(aTi21, aTj21)) - ((aTi22 - aTi21) + (aTj22 - aTj21)))
                             < dTR2)
                            || (aTj21 > aTi21 && aTj21 < aTi22) || (aTi21 > aTj21 && aTi21 < aTj22);
                }
                //
                if (bCond)
                {
                    aTi11 = Min(aTi11, aTj11);
                    aTi12 = Max(aTi12, aTj12);
                    aTi21 = Min(aTi21, aTj21);
                    aTi22 = Max(aTi22, aTj22);
                    aMI.Add(j);
                }
                else if (!bSplit2)
                {
                    i = j;
                    break;
                }
            }
            //
            if (((Math.Abs(aT11 - aTi11) < myRes1) && (Math.Abs(aT12 - aTi12) < myRes1))
                || ((Math.Abs(aT21 - aTi21) < myRes2) && (Math.Abs(aT22 - aTi22) < myRes2)))
            {
                aType = ShapeType.Edge;
                myCommonParts.Clear();
            }
            //
            AddSolution(aTi11, aTi12, aTi21, aTi22, aType);
            if (aType == ShapeType.Edge)
            {
                break;
            }
            //
            if (bSplit2)
            {
                ++i;
            }
        }
    }

    /// <summary>Adds a common part of the given type.</summary>
    protected void AddSolution(double aT11, double aT12, double aT21, double aT22, ShapeType theType)
    {
        IntTools_CommonPrt aCPart = new();
        //
        aCPart.SetType(theType);
        if (!mySwap)
        {
            aCPart.SetEdge1(myEdge1);
            aCPart.SetEdge2(myEdge2);
            aCPart.SetRange1(aT11, aT12);
            aCPart.AppendRange2(aT21, aT22);
        }
        else
        {
            aCPart.SetEdge1(myEdge2);
            aCPart.SetEdge2(myEdge1);
            aCPart.SetRange1(aT21, aT22);
            aCPart.AppendRange2(aT11, aT12);
        }
        //
        if (theType == ShapeType.Vertex)
        {
            double aT1, aT2;
            //
            FindBestSolution(aT11, aT12, aT21, aT22, out aT1, out aT2);
            //
            if (!mySwap)
            {
                aCPart.SetVertexParameter1(aT1);
                aCPart.SetVertexParameter2(aT2);
            }
            else
            {
                aCPart.SetVertexParameter1(aT2);
                aCPart.SetVertexParameter2(aT1);
            }
        }
        myCommonParts.Append(aCPart);
    }

    /// <summary>Looks for the minimal distance between the edges on the ranges.</summary>
    protected void FindBestSolution(double aT11, double aT12, double aT21, double aT22, out double aT1, out double aT2)
    {
        int i, aNbS, iErr;
        double aDMin, aD = 0.0, aRes1, aSolCriteria, aTouchCriteria;
        double aT1A, aT1B, aT1Min = 0.0, aT2Min = 0.0;
        GeomAPI_ProjectPointOnCurve aProjPC = new();
        NCollection_Sequence<IntTools_Range> aRanges = new();
        //
        // Both are written on every path below; C++ leaves them unset until then.
        aT1 = 0.0;
        aT2 = 0.0;
        aDMin = global::Kerf.Engine.Foundation.Precision.Infinite();
        aSolCriteria = 5.0e-16;
        aTouchCriteria = 5.0e-13;
        bool bTouch = false;
        bool bTouchConfirm = false;
        //
        aRes1 = Resolution(myCurve1.Curve().Curve()!, myCurve1.GetType(), myResCoeff1, myTol);
        aNbS = 10;
        aNbS = SplitRangeOnSegments(aT11, aT12, 3 * aRes1, aNbS, aRanges);
        //
        aProjPC.Init(myGeom2!, aT21, aT22);
        //
        double aT11Touch = aT11, aT12Touch = aT12;
        double aT21Touch = aT21, aT22Touch = aT22;
        bool isSolFound = false;
        for (i = 1; i <= aNbS; ++i)
        {
            IntTools_Range aR1 = aRanges.Value(i);
            aR1.Range(out aT1A, out aT1B);
            //
            aD = myTol;
            iErr = FindDistPC(aT1A, aT1B, myGeom1!, aSolCriteria, myPTol1, aProjPC, ref aD, ref aT1Min, ref aT2Min, false);
            if (iErr != 1)
            {
                if (aD < aDMin)
                {
                    aT1 = aT1Min;
                    aT2 = aT2Min;
                    aDMin = aD;
                    isSolFound = true;
                }
                //
                if (aD < aTouchCriteria)
                {
                    if (bTouch)
                    {
                        aT12Touch = aT1Min;
                        aT22Touch = aT2Min;
                        bTouchConfirm = true;
                    }
                    else
                    {
                        aT11Touch = aT1Min;
                        aT21Touch = aT2Min;
                        bTouch = true;
                    }
                }
            }
        }
        if (!isSolFound || bTouchConfirm)
        {
            aT1 = (aT11Touch + aT12Touch) * 0.5;
            iErr = DistPC(aT1, myGeom1!, aSolCriteria, aProjPC, ref aD, ref aT2, -1);
            if (iErr == 1)
            {
                aT2 = (aT21Touch + aT22Touch) * 0.5;
            }
        }
    }

    /// <summary>Computes the Line/Line intersection.</summary>
    protected void ComputeLineLine()
    {
        double aTol = myTol * myTol;

        Line3D aL1 = myCurve1.Line();
        Line3D aL2 = myCurve2.Line();

        Direction3D aD1 = aL1.Direction();
        Direction3D aD2 = aL2.Direction();

        double anAngle = aD1.Angle(aD2);
        bool IsCoincide = anAngle < global::Kerf.Engine.Foundation.Precision.Angular();
        if (IsCoincide)
        {
            if (aL1.SquareDistance(aL2.Location()) > aTol)
            {
                return;
            }
        }

        double aT11, aT12, aT21, aT22;
        myRange1.Range(out aT11, out aT12);
        myRange2.Range(out aT21, out aT22);

        Point3D aP11 = global::Kerf.Engine.Geometry.ElCLib.Value(aT11, aL1);
        Point3D aP12 = global::Kerf.Engine.Geometry.ElCLib.Value(aT12, aL1);

        if (!IsCoincide)
        {
            Point3D O2 = aL2.Location();
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(aT21) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(aT22))
            {
                O2 = global::Kerf.Engine.Geometry.ElCLib.Value((aT21 + aT22) / 2.0, aL2);
            }

            Vector3D aVec1 = new Vector3D(O2, aP11).Crossed(new Vector3D(aD2));
            Vector3D aVec2 = new Vector3D(O2, aP12).Crossed(new Vector3D(aD2));

            double aSqDist1 = aVec1.SquareMagnitude();
            double aSqDist2 = aVec2.SquareMagnitude();

            IsCoincide = (aSqDist1 <= aTol && aSqDist2 <= aTol);

            if (!IsCoincide && aVec1.Dot(aVec2) > 0)
            {
                // the lines do not intersect
                return;
            }
        }

        IntTools_CommonPrt aCommonPrt = new();
        aCommonPrt.SetEdge1(myEdge1);
        aCommonPrt.SetEdge2(myEdge2);

        if (IsCoincide)
        {
            double t21 = global::Kerf.Engine.Geometry.ElCLib.Parameter(aL2, aP11);
            double t22 = global::Kerf.Engine.Geometry.ElCLib.Parameter(aL2, aP12);

            if ((t21 > aT22 && t22 > aT22) || (t21 < aT21 && t22 < aT21))
            {
                // projections are out of range
                return;
            }

            if (t21 > t22)
            {
                (t21, t22) = (t22, t21);
            }

            if (t21 >= aT21)
            {
                if (t22 <= aT22)
                {
                    aCommonPrt.SetRange1(aT11, aT12);
                    aCommonPrt.SetAllNullFlag(true);
                    aCommonPrt.AppendRange2(t21, t22);
                }
                else
                {
                    aCommonPrt.SetRange1(aT11, aT12 - (t22 - aT22));
                    aCommonPrt.AppendRange2(t21, aT22);
                }
            }
            else
            {
                aCommonPrt.SetRange1(aT11 + (aT21 - t21), aT12);
                aCommonPrt.AppendRange2(aT21, t22);
            }
            aCommonPrt.SetType(ShapeType.Edge);
            myCommonParts.Append(aCommonPrt);
            return;
        }

        Vector3D O1O2 = new(aL1.Location(), aL2.Location());
        Xyz aCross = aD1.XYZ().Crossed(aD2.XYZ());
        double aDistLL = O1O2.Dot(new Vector3D(aCross.Normalized()));
        if (Math.Abs(aDistLL) > myTol)
        {
            return;
        }

        {
            // Fast check that no intersection needs to be added
            for (Iterator it1 = new(myEdge1); it1.More(); it1.Next())
            {
                for (Iterator it2 = new(myEdge2); it2.More(); it2.Next())
                {
                    if (it1.Value().IsSame(it2.Value()))
                    {
                        return;
                    }
                }
            }
        }

        double aSqSin = aCross.SquareModulus();
        double aT2 = (aD1.XYZ() * (O1O2.Dot(new Vector3D(aD1))) - (O1O2.XYZ())).Dot(aD2.XYZ());
        aT2 /= aSqSin;

        if (aT2 < aT21 || aT2 > aT22)
        {
            // out of range
            return;
        }

        Point3D aP2 = global::Kerf.Engine.Geometry.ElCLib.Value(aT2, aL2);
        double aT1 = new Vector3D(aL1.Location(), aP2).Dot(new Vector3D(aD1));

        if (aT1 < aT11 || aT1 > aT12)
        {
            // out of range
            return;
        }

        Point3D aP1 = global::Kerf.Engine.Geometry.ElCLib.Value(aT1, aL1);
        double aDist = aP1.SquareDistance(aP2);

        if (aDist > aTol)
        {
            // no intersection
            return;
        }

        // compute correct range on the edges
        double aDt1 = IntTools_Tools.ComputeIntRange(myTol1, myTol2, anAngle);
        double aDt2 = IntTools_Tools.ComputeIntRange(myTol2, myTol1, anAngle);

        aCommonPrt.SetRange1(aT1 - aDt1, aT1 + aDt1);
        aCommonPrt.AppendRange2(aT2 - aDt2, aT2 + aDt2);
        aCommonPrt.SetType(ShapeType.Vertex);
        aCommonPrt.SetVertexParameter1(aT1);
        aCommonPrt.SetVertexParameter2(aT2);
        myCommonParts.Append(aCommonPrt);
    }

    /// <summary>Checks if there is an intersection between the edges on the ranges (for nearly coincident edges).</summary>
    protected bool IsIntersection(double aT11, double aT12, double aT21, double aT22)
    {
        bool bRet;
        Point3D aP11, aP12, aP21, aP22;
        Vector3D aV11, aV12, aV21, aV22;
        double aD11_21, aD11_22, aD12_21, aD12_22, aCriteria, aCoef;
        bool bSmall_11_21, bSmall_11_22, bSmall_12_21, bSmall_12_22;
        //
        bRet = true;
        aCoef = 1.0e+5;
        if (((aT12 - aT11) > aCoef * myRes1) && ((aT22 - aT21) > aCoef * myRes2))
        {
            aCoef = 5000;
        }
        else
        {
            double aTRMin = Min((aT12 - aT11) / myRes1, (aT22 - aT21) / myRes2);
            aCoef = aTRMin / 100.0;
            if (aCoef < 1.0)
            {
                aCoef = 1.0;
            }
        }
        aCriteria = aCoef * myTol;
        aCriteria *= aCriteria;
        //
        myGeom1!.D1(aT11, out aP11, out aV11);
        myGeom1.D1(aT12, out aP12, out aV12);
        myGeom2!.D1(aT21, out aP21, out aV21);
        myGeom2.D1(aT22, out aP22, out aV22);
        //
        aD11_21 = aP11.SquareDistance(aP21);
        aD11_22 = aP11.SquareDistance(aP22);
        aD12_21 = aP12.SquareDistance(aP21);
        aD12_22 = aP12.SquareDistance(aP22);
        //
        bSmall_11_21 = aD11_21 < aCriteria;
        bSmall_11_22 = aD11_22 < aCriteria;
        bSmall_12_21 = aD12_21 < aCriteria;
        bSmall_12_22 = aD12_22 < aCriteria;
        //
        if ((bSmall_11_21 && bSmall_12_22) || (bSmall_11_22 && bSmall_12_21))
        {
            if (aCoef == 1.0)
            {
                return bRet;
            }
            //
            double anAngleCriteria;
            double anAngle1 = 0.0, anAngle2 = 0.0;
            //
            anAngleCriteria = 5.0e-3;
            if (aV11.SquareMagnitude() > global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                && aV12.SquareMagnitude() > global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                && aV21.SquareMagnitude() > global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                && aV22.SquareMagnitude() > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                if (bSmall_11_21 && bSmall_12_22)
                {
                    anAngle1 = aV11.Angle(aV21);
                    anAngle2 = aV12.Angle(aV22);
                }
                else
                {
                    anAngle1 = aV11.Angle(aV22);
                    anAngle2 = aV12.Angle(aV21);
                }
            }
            //
            if (((anAngle1 < anAngleCriteria) || ((M_PI - anAngle1) < anAngleCriteria))
                || ((anAngle2 < anAngleCriteria) || ((M_PI - anAngle2) < anAngleCriteria)))
            {
                GeomAPI_ProjectPointOnCurve aProjPC = new();
                int iErr;
                double aD, aT1Min = 0.0, aT2Min = 0.0;
                //
                aD = global::Kerf.Engine.Foundation.Precision.Infinite();
                aProjPC.Init(myGeom2, aT21, aT22);
                iErr = FindDistPC(aT11, aT12, myGeom1, myTol, myRes1, aProjPC, ref aD, ref aT1Min, ref aT2Min, false);
                bRet = (iErr == 2);
            }
        }
        return bRet;
    }

    /// <summary>
    /// Checks if the edges coincide on the ranges: 0 if they do, 1 if a point of the first curve can't be projected
    /// on the second, 2 if the distance is too big.
    /// </summary>
    protected int CheckCoincidence(double aT11, double aT12, double aT21, double aT22, double theCriteria,
                                   double theCurveRes1)
    {
        int iErr, aNb, aNb1, i;
        double aT1A, aT1B, aT1max = 0.0, aT2max = 0.0, aDmax;
        GeomAPI_ProjectPointOnCurve aProjPC = new();
        NCollection_Sequence<IntTools_Range> aRanges = new();
        //
        iErr = 0;
        aDmax = -1.0;
        aProjPC.Init(myGeom2!, aT21, aT22);
        //
        // 1. Express evaluation
        aNb = 10; // Number of intervals on the curve #1
        aNb1 = SplitRangeOnSegments(aT11, aT12, theCurveRes1, aNb, aRanges);
        for (i = 1; i < aNb1; ++i)
        {
            IntTools_Range aR1 = aRanges.Value(i);
            aR1.Range(out aT1A, out aT1B);
            //
            iErr = DistPC(aT1B, myGeom1!, theCriteria, aProjPC, ref aDmax, ref aT2max);
            if (iErr != 0)
            {
                return iErr;
            }
        }
        //
        // if the ranges in aRanges are less than theCurveRes1,
        // there is no need to do step 2 (deep evaluation)
        if (aNb1 < aNb)
        {
            return iErr;
        }
        //
        // 2. Deep evaluation
        for (i = 2; i < aNb1; ++i)
        {
            IntTools_Range aR1 = aRanges.Value(i);
            aR1.Range(out aT1A, out aT1B);
            //
            iErr = FindDistPC(aT1A, aT1B, myGeom1!, theCriteria, theCurveRes1, aProjPC, ref aDmax, ref aT1max, ref aT2max);
            if (iErr != 0)
            {
                return iErr;
            }
        }
        // Possible values:
        // iErr == 0 - the patches are coincided
        // iErr == 1 - a point from aC1 can not be projected on aC2
        // iErr == 2 - the distance is too big
        return iErr;
    }

    // ---- file-static functions of IntTools_EdgeEdge.cxx

    private static int FindDistPC(double aT1A,
                                  double aT1B,
                                  Curve3D theC1,
                                  double theCriteria,
                                  double theEps,
                                  GeomAPI_ProjectPointOnCurve theProjPC,
                                  ref double aDmax,
                                  ref double aT1max,
                                  ref double aT2max,
                                  bool bMaxDist = true)
    {
        int iErr, iC;
        double aGS, aXP, aA, aB, aXL, aYP = 0.0, aYL = 0.0, aT2P = 0.0, aT2L = 0.0;
        //
        iC = bMaxDist ? 1 : -1;
        iErr = 0;
        aT1max = aT2max = 0.0; // silence GCC warning
        //
        aGS = 0.6180339887498948482045868343656; // =0.5*(1.+sqrt(5.))-1.;
        aA = aT1A;
        aB = aT1B;
        //
        // check bounds
        iErr = DistPC(aA, theC1, theCriteria, theProjPC, ref aYP, ref aT2P, ref aDmax, ref aT1max, ref aT2max, iC);
        if (iErr == 2)
        {
            return iErr;
        }
        //
        iErr = DistPC(aB, theC1, theCriteria, theProjPC, ref aYL, ref aT2L, ref aDmax, ref aT1max, ref aT2max, iC);
        if (iErr == 2)
        {
            return iErr;
        }
        //
        aXP = aA + (aB - aA) * aGS;
        aXL = aB - (aB - aA) * aGS;
        //
        iErr = DistPC(aXP, theC1, theCriteria, theProjPC, ref aYP, ref aT2P, ref aDmax, ref aT1max, ref aT2max, iC);
        if (iErr != 0)
        {
            return iErr;
        }
        //
        iErr = DistPC(aXL, theC1, theCriteria, theProjPC, ref aYL, ref aT2L, ref aDmax, ref aT1max, ref aT2max, iC);
        if (iErr != 0)
        {
            return iErr;
        }
        //
        double anEps = Max(theEps, Epsilon(Max(Math.Abs(aA), Math.Abs(aB))) * 10.0);
        for (;;)
        {
            if (iC * (aYP - aYL) > 0)
            {
                aA = aXL;
                aXL = aXP;
                aYL = aYP;
                aXP = aA + (aB - aA) * aGS;
                iErr = DistPC(aXP, theC1, theCriteria, theProjPC, ref aYP, ref aT2P, ref aDmax, ref aT1max, ref aT2max, iC);
            }
            else
            {
                aB = aXP;
                aXP = aXL;
                aYP = aYL;
                aXL = aB - (aB - aA) * aGS;
                iErr = DistPC(aXL, theC1, theCriteria, theProjPC, ref aYL, ref aT2L, ref aDmax, ref aT1max, ref aT2max, iC);
            }
            //
            if (iErr != 0)
            {
                if ((iErr == 2) && !bMaxDist)
                {
                    aXP = (aA + aB) * 0.5;
                    DistPC(aXP, theC1, theCriteria, theProjPC, ref aYP, ref aT2P, ref aDmax, ref aT1max, ref aT2max, iC);
                }
                return iErr;
            }
            //
            if ((aB - aA) < anEps)
            {
                break;
            }
        } // for (;;) {
        //
        return iErr;
    }

    private static int DistPC(double aT1,
                              Curve3D theC1,
                              double theCriteria,
                              GeomAPI_ProjectPointOnCurve theProjPC,
                              ref double aD,
                              ref double aT2,
                              ref double aDmax,
                              ref double aT1max,
                              ref double aT2max,
                              int iC)
    {
        int iErr;
        //
        iErr = DistPC(aT1, theC1, theCriteria, theProjPC, ref aD, ref aT2, iC);
        if (iErr == 1)
        {
            return iErr;
        }
        //
        if (iC * (aD - aDmax) > 0)
        {
            aDmax = aD;
            aT1max = aT1;
            aT2max = aT2;
        }
        //
        return iErr;
    }

    private static int DistPC(double aT1,
                              Curve3D theC1,
                              double theCriteria,
                              GeomAPI_ProjectPointOnCurve theProjPC,
                              ref double aD,
                              ref double aT2,
                              int iC = 1)
    {
        int iErr, aNbP2;
        Point3D aP1;
        //
        iErr = 0;
        theC1.D0(aT1, out aP1);
        //
        theProjPC.Perform(aP1);
        aNbP2 = theProjPC.NbPoints();
        if (aNbP2 == 0)
        {
            iErr = 1; // the point from aC1 can not be projected on aC2
            return iErr;
        }
        //
        aD = theProjPC.LowerDistance();
        aT2 = theProjPC.LowerDistanceParameter();
        if (iC * (aD - theCriteria) > 0)
        {
            iErr = 2; // the distance is too big or small
        }
        //
        return iErr;
    }

    private static int SplitRangeOnSegments(double aT1, double aT2, double theResolution, int theNbSeg,
                                            NCollection_Sequence<IntTools_Range> theSegments)
    {
        double aDiff = aT2 - aT1;
        if (aDiff < theResolution || theNbSeg == 1)
        {
            theSegments.Append(new IntTools_Range(aT1, aT2));
            return 1;
        }
        //
        double aDt, aT1x, aT2x, aSeg;
        int aNbSegments, i;
        //
        aNbSegments = theNbSeg;
        aDt = aDiff / aNbSegments;
        if (aDt < theResolution)
        {
            aSeg = aDiff / theResolution;
            aNbSegments = (int)aSeg + 1;
            aDt = aDiff / aNbSegments;
        }
        //
        aT1x = aT1;
        for (i = 1; i < aNbSegments; ++i)
        {
            aT2x = aT1x + aDt;
            //
            IntTools_Range aR = new(aT1x, aT2x);
            theSegments.Append(aR);
            //
            aT1x = aT2x;
        }
        //
        IntTools_Range aRLast = new(aT1x, aT2);
        theSegments.Append(aRLast);
        //
        return aNbSegments;
    }

    private static void BndBuildBox(EdgeCurveAdaptor theBAC, double aT1, double aT2, double theTol, out Box theBox)
    {
        Box aB = new();
        Add3dCurve.Add(theBAC, aT1, aT2, theTol, ref aB);
        theBox = aB;
    }

    private static double PointBoxDistance(in Box aB, in Point3D aP)
    {
        Span<double> aPCoord = stackalloc double[3];
        Span<double> aBMinCoord = stackalloc double[3];
        Span<double> aBMaxCoord = stackalloc double[3];
        double aDist, aR1, aR2;
        int i;
        //
        aP.Coord(out aPCoord[0], out aPCoord[1], out aPCoord[2]);
        aB.Get(out aBMinCoord[0], out aBMinCoord[1], out aBMinCoord[2], out aBMaxCoord[0], out aBMaxCoord[1],
               out aBMaxCoord[2]);
        //
        aDist = 0.0;
        for (i = 0; i < 3; ++i)
        {
            aR1 = aBMinCoord[i] - aPCoord[i];
            if (aR1 > 0.0)
            {
                aDist += aR1 * aR1;
                continue;
            }
            //
            aR2 = aPCoord[i] - aBMaxCoord[i];
            if (aR2 > 0.0)
            {
                aDist += aR2 * aR2;
            }
        }
        //
        aDist = Math.Sqrt(aDist);
        return aDist;
    }

    private static int TypeToInteger(CurveKind theCType)
    {
        int iRet;
        //
        switch (theCType)
        {
            case Line:
                iRet = 0;
                break;
            case Hyperbola:
            case Parabola:
                iRet = 1;
                break;
            case Circle:
            case Ellipse:
                iRet = 2;
                break;
            case BezierCurve:
            case BSplineCurve:
                iRet = 3;
                break;
            default:
                iRet = 4;
                break;
        }
        return iRet;
    }

    private static double ResolutionCoeff(EdgeCurveAdaptor theBAC, in IntTools_Range theRange)
    {
        double aResCoeff = 0.0;
        //
        Curve3D aCurve = theBAC.Curve().Curve()!;
        CurveKind aCurveType = theBAC.GetType();
        //
        // OCCT's switch with the offset case falling through to the sampled one
        bool aSample = false;
        switch (aCurveType)
        {
            case Circle:
                aResCoeff = 1.0 / (2 * ((CircleCurve3D)aCurve).Circ().Radius());
                break;
            case Ellipse:
                aResCoeff = 1.0 / ((EllipseCurve3D)aCurve).MajorRadius();
                break;
            case OffsetCurve:
                {
                    OffsetCurve3D anOffsetCurve = (OffsetCurve3D)aCurve;
                    Curve3D aBasisCurve = anOffsetCurve.BasisCurve();
                    CurveAdaptor aGBasisCurve = new(aBasisCurve);
                    CurveKind aBCType = aGBasisCurve.GetType();
                    if (aBCType == Line)
                    {
                        break;
                    }
                    else if (aBCType == Circle)
                    {
                        aResCoeff = 1.0 / (2 * (anOffsetCurve.Offset() + aGBasisCurve.Circle().Radius()));
                        break;
                    }
                    else if (aBCType == Ellipse)
                    {
                        aResCoeff = 1.0 / (anOffsetCurve.Offset() + aGBasisCurve.Ellipse().MajorRadius());
                        break;
                    }
                    aSample = true;
                    break;
                }
            case Hyperbola:
            case Parabola:
            case OtherCurve:
                aSample = true;
                break;
            default:
                break;
        }
        if (aSample)
        {
            double k, kMin, aDist, aDt, aT1, aT2, aT;
            int aNbP, i;
            Point3D aP1, aP2;
            //
            aNbP = 30;
            theRange.Range(out aT1, out aT2);
            aDt = (aT2 - aT1) / aNbP;
            aT = aT1;
            kMin = 10.0;
            //
            theBAC.D0(aT1, out aP1);
            for (i = 1; i <= aNbP; ++i)
            {
                aT += aDt;
                theBAC.D0(aT, out aP2);
                aDist = aP1.Distance(aP2);
                k = aDt / aDist;
                if (k < kMin)
                {
                    kMin = k;
                }
                aP1 = aP2;
            }
            //
            aResCoeff = kMin;
        }
        //
        return aResCoeff;
    }

    private static double Resolution(Curve3D theCurve, CurveKind theCurveType, double theResCoeff, double theR3D)
    {
        double aRes;
        //
        switch (theCurveType)
        {
            case Line:
                aRes = theR3D;
                break;
            case Circle:
                {
                    double aDt = theResCoeff * theR3D;
                    aRes = (aDt <= 1.0) ? 2 * Math.Asin(aDt) : 2 * M_PI;
                    break;
                }
            case BezierCurve:
                ((BezierCurve3D)theCurve).Resolution(theR3D, out aRes);
                break;
            case BSplineCurve:
                ((BSplineCurve3D)theCurve).Resolution(theR3D, out aRes);
                break;
            case OffsetCurve:
                {
                    Curve3D aBasisCurve = ((OffsetCurve3D)theCurve).BasisCurve();
                    CurveKind aBCType = new CurveAdaptor(aBasisCurve).GetType();
                    if (aBCType == Line)
                    {
                        aRes = theR3D;
                        break;
                    }
                    else if (aBCType == Circle)
                    {
                        double aDt = theResCoeff * theR3D;
                        aRes = (aDt <= 1.0) ? 2 * Math.Asin(aDt) : 2 * M_PI;
                        break;
                    }
                    // falls through to the default case, as OCCT's [[fallthrough]]
                    aRes = theResCoeff * theR3D;
                    break;
                }
            default:
                aRes = theResCoeff * theR3D;
                break;
        }
        //
        return aRes;
    }

    private static double CurveDeflection(EdgeCurveAdaptor theBAC, in IntTools_Range theRange)
    {
        double aDt, aT, aT1, aT2, aDefl;
        int i, aNbP;
        Vector3D aV1, aV2;
        //
        aDefl = 0;
        aNbP = 10;
        theRange.Range(out aT1, out aT2);
        aDt = (aT2 - aT1) / aNbP;
        aT = aT1;
        //
        theBAC.D1(aT1, out _, out aV1);
        for (i = 1; i <= aNbP; ++i)
        {
            aT += aDt;
            theBAC.D1(aT, out _, out aV2);
            if (aV1.Magnitude() > Constants.Resolution() && aV2.Magnitude() > Constants.Resolution())
            {
                Direction3D aD1 = new(aV1), aD2 = new(aV2);
                aDefl += aD1.Angle(aD2);
            }
            aV1 = aV2;
        }
        //
        return aDefl;
    }

    private static bool IsClosed(Curve3D theCurve, double aT1, double aT2, double theTol, double theRes)
    {
        if (Math.Abs(aT1 - aT2) < theRes)
        {
            return false;
        }

        Point3D aP1, aP2;
        theCurve.D0(aT1, out aP1);
        theCurve.D0(aT2, out aP2);
        //
        double aD = aP1.Distance(aP2);
        return aD < theTol;
    }
}
