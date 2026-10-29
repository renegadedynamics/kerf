// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GGExtPC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ELPCOfLocateExtPC.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// All the extremum distances (minima and maxima) between a point and a 3D curve on a parameter range. Elementary
/// curves are solved analytically (<see cref="Extrema_ExtPElC"/>); Bezier and B-spline curves are searched span by
/// span, other curves interval by interval of C2 continuity (or of nearly constant deflection).
/// </summary>
/// <remarks>
/// The 3D instantiation of OCCT's template Extrema_GGExtPC, which OCCT names Extrema_ExtPC and
/// Extrema_ELPCOfLocateExtPC (one type). An algorithm object (decision 117): construct it with the point and the
/// curve, or <c>Initialize</c> and <c>Perform</c>. Keeps a reference to the curve, as OCCT does.
/// </remarks>
public class Extrema_ExtPC
{
    private Adaptor3d_Curve? myC;
    private gp_Pnt Pf = new();
    private gp_Pnt Pl = new();
    private readonly Extrema_ExtPElC myExtPElC = new();
    private readonly NCollection_Sequence<Extrema_POnCurv> mypoint = new();
    private bool mydone;
    private double mydist1;
    private double mydist2;
    private readonly Extrema_EPCOfExtPC myExtPC = new();
    private double mytolu;
    private double mytolf;
    private int mysample;
    private double myintuinf;
    private double myintusup;
    private double myuinf;
    private double myusup;
    private GeomAbs_CurveType type;
    private readonly NCollection_Sequence<bool> myismin = new();
    private readonly NCollection_Sequence<double> mySqDist = new();

    public Extrema_ExtPC()
    {
        myC = null;
        mydone = false;
        mydist1 = RealLast();
        mydist2 = RealLast();
        mytolu = 0.0;
        mytolf = 0.0;
        mysample = 17;
        myintuinf = Precision.Infinite();
        myintusup = Precision.Infinite();
        myuinf = Precision.Infinite();
        myusup = Precision.Infinite();
        type = GeomAbs_OtherCurve;
    }

    /// <summary>All the extremum distances between theP and theC on [theUinf, theUsup];
    /// theTolF is the tolerance on the function value of the root search.</summary>
    public Extrema_ExtPC(in gp_Pnt theP, Adaptor3d_Curve theC, double theUinf, double theUsup, double theTolF = 1.0e-10)
    {
        Initialize(theC, theUinf, theUsup, theTolF);
        Perform(theP);
    }

    /// <summary>All the extremum distances between theP and theC over the curve's whole range;
    /// theTolF is the tolerance on the function value of the root search.</summary>
    public Extrema_ExtPC(in gp_Pnt theP, Adaptor3d_Curve theC, double theTolF = 1.0e-10)
    {
        Initialize(theC, Extrema_CurveTool.FirstParameter(theC), Extrema_CurveTool.LastParameter(theC), theTolF);
        Perform(theP);
    }

    /// <summary>Sets the curve, the range and the tolerance; <c>Perform</c> then computes.</summary>
    public void Initialize(Adaptor3d_Curve theC, double theUinf, double theUsup, double theTolF = 1.0e-10)
    {
        myC = theC;
        myintuinf = myuinf = theUinf;
        myintusup = myusup = theUsup;
        mytolf = theTolF;
        mytolu = Extrema_CurveTool.Resolution(myC, Precision.Confusion());
        type = Extrema_CurveTool.GetType(theC);
        mydone = false;
        mydist1 = RealLast();
        mydist2 = RealLast();
        mysample = 17;
    }

    /// <summary>Computes the extremum distances for the point theP.</summary>
    public void Perform(in gp_Pnt theP)
    {
        mySqDist.Clear();
        mypoint.Clear();
        myismin.Clear();
        int i, NbExt, n;
        double U;
        mysample = 17;
        double t3d = Precision.Confusion();

        if (Precision.IsInfinite(myuinf))
        {
            mydist1 = RealLast();
        }
        else
        {
            Pf = Extrema_CurveTool.Value(myC!, myuinf);
            mydist1 = theP.SquareDistance(Pf);
        }

        if (Precision.IsInfinite(myusup))
        {
            mydist2 = RealLast();
        }
        else
        {
            Pl = Extrema_CurveTool.Value(myC!, myusup);
            mydist2 = theP.SquareDistance(Pl);
        }

        Adaptor3d_Curve aCurve = myC!;

        switch (type)
        {
            case GeomAbs_Circle:
            {
                myExtPElC.Perform(theP, Extrema_CurveTool.Circle(aCurve), t3d, myuinf, myusup);
                break;
            }
            case GeomAbs_Ellipse:
            {
                myExtPElC.Perform(theP, Extrema_CurveTool.Ellipse(aCurve), t3d, myuinf, myusup);
                break;
            }
            case GeomAbs_Parabola:
            {
                myExtPElC.Perform(theP, Extrema_CurveTool.Parabola(aCurve), t3d, myuinf, myusup);
                break;
            }
            case GeomAbs_Hyperbola:
            {
                myExtPElC.Perform(theP, Extrema_CurveTool.Hyperbola(aCurve), t3d, myuinf, myusup);
                break;
            }
            case GeomAbs_Line:
            {
                myExtPElC.Perform(theP, Extrema_CurveTool.Line(aCurve), t3d, myuinf, myusup);
                break;
            }
            case GeomAbs_BezierCurve:
            {
                myintuinf = myuinf;
                myintusup = myusup;
                mysample = (Extrema_CurveTool.Bezier(aCurve)).NbPoles() * 2;
                myExtPC.Initialize(aCurve);
                IntervalPerform(theP);
                return;
            }
            case GeomAbs_BSplineCurve:
            {
                Geom_BSplineCurve aBSpline = Extrema_CurveTool.BSpline(aCurve);
                int aFirstIdx = aBSpline.FirstUKnotIndex();
                int aLastIdx = aBSpline.LastUKnotIndex();
                NCollection_Array1<double> aKnots = aBSpline.InternalKnots();

                double aPeriodJump = 0.0;
                double aTolCoeff = (myusup - myuinf) * Precision.PConfusion();
                if (Extrema_CurveTool.IsPeriodic(aCurve))
                {
                    int aPeriodShift = (int)((myuinf - aKnots[aFirstIdx]) / Extrema_CurveTool.Period(aCurve));
                    if (myuinf < aKnots[aFirstIdx] - aTolCoeff)
                    {
                        aPeriodShift--;
                    }
                    aPeriodJump = Extrema_CurveTool.Period(aCurve) * aPeriodShift;
                }

                int anIdx;
                int aFirstUsedKnot = aFirstIdx, aLastUsedKnot = aLastIdx;
                for (anIdx = aFirstIdx; anIdx <= aLastIdx; anIdx++)
                {
                    double aKnot = aKnots[anIdx] + aPeriodJump;
                    if (myuinf >= aKnot - aTolCoeff)
                    {
                        aFirstUsedKnot = anIdx;
                    }
                    else
                    {
                        break;
                    }
                }
                for (anIdx = aLastIdx; anIdx >= aFirstIdx; anIdx--)
                {
                    double aKnot = aKnots[anIdx] + aPeriodJump;
                    if (myusup <= aKnot + aTolCoeff)
                    {
                        aLastUsedKnot = anIdx;
                    }
                    else
                    {
                        break;
                    }
                }

                if (aFirstUsedKnot == aLastUsedKnot)
                {
                    aFirstUsedKnot = aFirstIdx;
                    aLastUsedKnot = aFirstIdx + 1;
                }

                mysample = aBSpline.Degree() + 1;

                if (mysample == 2)
                {
                    gp_Pnt aPmin = new();
                    double tmin = 0.0, distmin = RealLast();
                    double aMin1 = 0.0, aMin2 = 0.0;
                    myExtPC.Initialize(aCurve);
                    for (anIdx = aFirstUsedKnot; anIdx < aLastUsedKnot; anIdx++)
                    {
                        double aF = aKnots[anIdx] + aPeriodJump, aL = aKnots[anIdx + 1] + aPeriodJump;

                        if (anIdx == aFirstUsedKnot)
                        {
                            aF = myuinf;
                        }
                        else if (anIdx == aLastUsedKnot - 1)
                        {
                            aL = myusup;
                        }

                        Extrema_CurveTool.D0(aCurve, aF, out gp_Pnt aP1);
                        Extrema_CurveTool.D0(aCurve, aL, out gp_Pnt aP2);
                        gp_Vec aBase1 = new(theP, aP1), aBase2 = new(theP, aP2);
                        gp_Vec aV = new(aP2, aP1);
                        double aVal1 = aV.Dot(aBase1);
                        double aVal2 = aV.Dot(aBase2);
                        if (anIdx == aFirstUsedKnot)
                        {
                            aMin1 = theP.SquareDistance(aP1);
                        }
                        else
                        {
                            aMin1 = aMin2;
                            if (distmin > aMin1)
                            {
                                distmin = aMin1;
                                tmin = aF;
                                aPmin = aP1;
                            }
                        }
                        aMin2 = theP.SquareDistance(aP2);
                        double aMinSqDist = Min(aMin1, aMin2);
                        double aMinDer = Min(Math.Abs(aVal1), Math.Abs(aVal2));
                        if (!(Precision.IsInfinite(aVal1) || Precision.IsInfinite(aVal2)))
                        {
                            if (aVal1 * aVal2 <= 0.0 || aMinSqDist < 100.0 * Precision.SquareConfusion()
                                || 2.0 * aMinDer < Precision.Confusion())
                            {
                                myintuinf = aF;
                                myintusup = aL;
                                IntervalPerform(theP);
                            }
                        }
                    }
                    if (!Precision.IsInfinite(distmin))
                    {
                        bool isToAdd = true;
                        NbExt = mypoint.Length();
                        for (i = 1; i <= NbExt && isToAdd; i++)
                        {
                            double t = mypoint.Value(i).Parameter();
                            isToAdd = (distmin < mySqDist[i]) && (Math.Abs(t - tmin) > mytolu);
                        }
                        if (isToAdd)
                        {
                            Extrema_POnCurv PC = new(tmin, aPmin);
                            mySqDist.Append(distmin);
                            myismin.Append(true);
                            mypoint.Append(PC);
                        }
                    }
                }
                else
                {
                    int aValIdx = 1;
                    NCollection_Array1<double> aVal = new(1, (mysample) * (aLastUsedKnot - aFirstUsedKnot) + 1);
                    NCollection_Array1<double> aParam = new(1, (mysample) * (aLastUsedKnot - aFirstUsedKnot) + 1);
                    for (anIdx = aFirstUsedKnot; anIdx < aLastUsedKnot; anIdx++)
                    {
                        double aF = aKnots[anIdx] + aPeriodJump, aL = aKnots[anIdx + 1] + aPeriodJump;

                        if (anIdx == aFirstUsedKnot)
                        {
                            aF = myuinf;
                        }
                        if (anIdx == aLastUsedKnot - 1)
                        {
                            aL = myusup;
                        }

                        double aStep = (aL - aF) / mysample;
                        for (int aPntIdx = 0; aPntIdx < mysample; aPntIdx++)
                        {
                            double aCurrentParam = aF + aStep * aPntIdx;
                            aVal[aValIdx] = Extrema_CurveTool.Value(aCurve, aCurrentParam).SquareDistance(theP);
                            aParam[aValIdx] = aCurrentParam;
                            aValIdx++;
                        }
                    }
                    aVal[aValIdx] = Extrema_CurveTool.Value(aCurve, myusup).SquareDistance(theP);
                    aParam[aValIdx] = myusup;

                    myExtPC.Initialize(aCurve);

                    for (anIdx = aVal.Lower() + 1; anIdx < aVal.Upper(); anIdx++)
                    {
                        if (aVal[anIdx] <= Precision.SquareConfusion())
                        {
                            mySqDist.Append(aVal[anIdx]);
                            myismin.Append(true);
                            mypoint.Append(new Extrema_POnCurv(aParam[anIdx], Extrema_CurveTool.Value(aCurve, aParam[anIdx])));
                        }
                        if ((aVal[anIdx] >= aVal[anIdx + 1] && aVal[anIdx] >= aVal[anIdx - 1])
                            || (aVal[anIdx] <= aVal[anIdx + 1] && aVal[anIdx] <= aVal[anIdx - 1]))
                        {
                            myintuinf = aParam[anIdx - 1];
                            myintusup = aParam[anIdx + 1];
                            IntervalPerform(theP);
                        }
                    }

                    if (mydist1 > Precision.SquareConfusion() && !Precision.IsPositiveInfinite(mydist1))
                    {
                        Extrema_CurveTool.D1(aCurve, aParam.Value(aParam.Lower()), out gp_Pnt aP1, out gp_Vec aV1);
                        Extrema_CurveTool.D1(aCurve, aParam.Value(aParam.Lower() + 1), out gp_Pnt aP2, out gp_Vec aV2);
                        gp_Vec aBase1 = new(theP, aP1), aBase2 = new(theP, aP2);
                        double aVal1 = aV1.Dot(aBase1);
                        double aVal2 = aV2.Dot(aBase2);
                        if (!(Precision.IsInfinite(aVal1) || Precision.IsInfinite(aVal2)))
                        {
                            if (aVal1 * aVal2 <= 0.0 || aBase1.Dot(aBase2) <= 0.0
                                || 2.0 * Math.Abs(aVal1) < Precision.Confusion())
                            {
                                myintuinf = aParam[aVal.Lower()];
                                myintusup = aParam[aVal.Lower() + 1];
                                IntervalPerform(theP);
                            }
                        }
                    }

                    if (mydist2 > Precision.SquareConfusion() && !Precision.IsPositiveInfinite(mydist2))
                    {
                        Extrema_CurveTool.D1(aCurve, aParam.Value(aParam.Upper() - 1), out gp_Pnt aP1, out gp_Vec aV1);
                        Extrema_CurveTool.D1(aCurve, aParam.Value(aParam.Upper()), out gp_Pnt aP2, out gp_Vec aV2);
                        gp_Vec aBase1 = new(theP, aP1), aBase2 = new(theP, aP2);
                        double aVal1 = aV1.Dot(aBase1);
                        double aVal2 = aV2.Dot(aBase2);

                        if (!(Precision.IsInfinite(aVal1) || Precision.IsInfinite(aVal2)))
                        {
                            if (aVal1 * aVal2 <= 0.0 || aBase1.Dot(aBase2) <= 0.0
                                || 2.0 * Math.Abs(aVal2) < Precision.Confusion())
                            {
                                myintuinf = aParam[aVal.Upper() - 1];
                                myintusup = aParam[aVal.Upper()];
                                IntervalPerform(theP);
                            }
                        }
                    }
                }
                mydone = true;
                break;
            }
            default:
            {
                const int aMaxSample = 17;
                bool IntExtIsDone = false;
                bool IntIsNotValid;
                NCollection_HArray1<double> theHInter;
                n = Extrema_CurveTool.NbIntervals(aCurve, GeomAbs_C2);
                if (n > 1)
                {
                    theHInter = new NCollection_HArray1<double>(1, n + 1);
                    Extrema_CurveTool.Intervals(aCurve, theHInter.ChangeArray1(), GeomAbs_C2);
                }
                else
                {
                    theHInter = Extrema_CurveTool.DeflCurvIntervals(aCurve);
                    n = theHInter.Length() - 1;
                }
                mysample = Max(mysample / n, aMaxSample);
                double maxint = 0.0;
                for (i = 1; i <= n; ++i)
                {
                    double dt = theHInter.Value(i + 1) - theHInter.Value(i);
                    if (maxint < dt)
                    {
                        maxint = dt;
                    }
                }
                bool isPeriodic = Extrema_CurveTool.IsPeriodic(aCurve);
                gp_Vec V1;
                gp_Pnt PP;
                double s1 = 0.0;
                double s2 = 0.0;
                myExtPC.Initialize(aCurve);
                for (i = 1; i <= n; i++)
                {
                    myintuinf = theHInter.Value(i);
                    myintusup = theHInter.Value(i + 1);
                    mysample = Max(RealToInt(aMaxSample * (myintusup - myintuinf) / maxint), 3);

                    double anInfToCheck = myintuinf;
                    double aSupToCheck = myintusup;

                    if (isPeriodic)
                    {
                        double aPeriod = Extrema_CurveTool.Period(aCurve);
                        anInfToCheck = ElCLib.InPeriod(myintuinf, myuinf, myuinf + aPeriod);
                        aSupToCheck = myintusup + (anInfToCheck - myintuinf);
                    }
                    IntIsNotValid = (myuinf > aSupToCheck) || (myusup < anInfToCheck);

                    if (IntIsNotValid)
                    {
                        continue;
                    }

                    if (myuinf >= anInfToCheck)
                    {
                        anInfToCheck = myuinf;
                    }
                    if (myusup <= aSupToCheck)
                    {
                        aSupToCheck = myusup;
                    }
                    if ((aSupToCheck - anInfToCheck) <= mytolu)
                    {
                        continue;
                    }

                    if (i != 1)
                    {
                        Extrema_CurveTool.D1(aCurve, myintuinf, out PP, out V1);
                        s1 = (new gp_Vec(theP, PP)) * V1;
                        if (s1 * s2 < 0.0)
                        {
                            mySqDist.Append(PP.SquareDistance(theP));
                            myismin.Append((s1 < 0.0));
                            mypoint.Append(new Extrema_POnCurv(myintuinf, PP));
                        }
                    }
                    if (i != n)
                    {
                        Extrema_CurveTool.D1(aCurve, myintusup, out PP, out V1);
                        s2 = (new gp_Vec(theP, PP)) * V1;
                    }

                    IntervalPerform(theP);
                    IntExtIsDone = IntExtIsDone || mydone;
                }

                mydone = IntExtIsDone;
                break;
            }
        }

        // Postprocessing
        if (type == GeomAbs_BSplineCurve || type == GeomAbs_OffsetCurve || type == GeomAbs_OtherCurve)
        {
            if (mydist1 < Precision.SquareConfusion() || mydist2 < Precision.SquareConfusion())
            {
                bool isFirstAdded = false;
                bool isLastAdded = false;
                int aNbPoints = mypoint.Length();
                for (i = 1; i <= aNbPoints; i++)
                {
                    U = mypoint.Value(i).Parameter();
                    if (Math.Abs(U - myuinf) < mytolu)
                    {
                        isFirstAdded = true;
                    }
                    else if (Math.Abs(myusup - U) < mytolu)
                    {
                        isLastAdded = true;
                    }
                }
                if (!isFirstAdded && mydist1 < Precision.SquareConfusion())
                {
                    mySqDist.Prepend(mydist1);
                    myismin.Prepend(true);
                    mypoint.Prepend(new Extrema_POnCurv(myuinf, Pf));
                }
                if (!isLastAdded && mydist2 < Precision.SquareConfusion())
                {
                    mySqDist.Append(mydist2);
                    myismin.Append(true);
                    mypoint.Append(new Extrema_POnCurv(myusup, Pl));
                }
                mydone = true;
            }
        }
        else
        {
            mydone = myExtPElC.IsDone();
            if (mydone)
            {
                NbExt = myExtPElC.NbExt();
                for (i = 1; i <= NbExt; i++)
                {
                    Extrema_POnCurv PC = myExtPElC.Point(i);
                    U = PC.Parameter();
                    if (Extrema_CurveTool.IsPeriodic(aCurve))
                    {
                        U = ElCLib.InPeriod(U, myuinf, myuinf + Extrema_CurveTool.Period(aCurve));
                    }
                    if ((U >= myuinf - mytolu) && (U <= myusup + mytolu))
                    {
                        PC.SetValues(U, myExtPElC.Point(i).Value());
                        mySqDist.Append(myExtPElC.SquareDistance(i));
                        myismin.Append(myExtPElC.IsMin(i));
                        mypoint.Append(PC);
                    }
                }
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => mydone;

    /// <summary>The Nth extremum square distance.</summary>
    /// <exception cref="Standard_OutOfRange">theN is not in 1..NbExt().</exception>
    public double SquareDistance(int theN)
    {
        if ((theN < 1) || (theN > NbExt()))
        {
            throw new Standard_OutOfRange();
        }
        return mySqDist.Value(theN);
    }

    /// <summary>The number of extremum distances.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public int NbExt()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        return mySqDist.Length();
    }

    /// <summary>True if the Nth extremum distance is a minimum.</summary>
    /// <exception cref="Standard_OutOfRange">theN is not in 1..NbExt().</exception>
    public bool IsMin(int theN)
    {
        if ((theN < 1) || (theN > NbExt()))
        {
            throw new Standard_OutOfRange();
        }
        return myismin.Value(theN);
    }

    /// <summary>The point of the Nth extremum distance.</summary>
    /// <exception cref="Standard_OutOfRange">theN is not in 1..NbExt().</exception>
    public Extrema_POnCurv Point(int theN)
    {
        if ((theN < 1) || (theN > NbExt()))
        {
            throw new Standard_OutOfRange();
        }
        return mypoint.Value(theN);
    }

    /// <summary>The square distances to the ends of the range and the end points (RealLast for an infinite end, whose
    /// point is then the one of a previous computation, or the origin).</summary>
    public void TrimmedSquareDistances(out double theDist1, out double theDist2, out gp_Pnt theP1, out gp_Pnt theP2)
    {
        theDist1 = mydist1;
        theDist2 = mydist2;
        theP1 = Pf;
        theP2 = Pl;
    }

    /// <summary>Searches the extrema on the current interval [myintuinf, myintusup].</summary>
    protected void IntervalPerform(in gp_Pnt theP)
    {
        int i;
        double U;
        myExtPC.Initialize(mysample, myintuinf, myintusup, mytolu, mytolf);
        myExtPC.Perform(theP);
        mydone = myExtPC.IsDone();
        if (mydone)
        {
            int NbExt = myExtPC.NbExt();
            for (i = 1; i <= NbExt; i++)
            {
                Extrema_POnCurv PC = myExtPC.Point(i);
                U = PC.Parameter();
                if (Extrema_CurveTool.IsPeriodic(myC!))
                {
                    U = ElCLib.InPeriod(U, myuinf, myuinf + Extrema_CurveTool.Period(myC!));
                }
                if ((U >= myuinf - mytolu) && (U <= myusup + mytolu))
                {
                    AddSol(U, PC.Value(), myExtPC.SquareDistance(i), myExtPC.IsMin(i));
                }
            }
        }
    }

    /// <summary>Adds a solution unless one with the same parameter (within the parametric tolerance) is there.</summary>
    protected void AddSol(double theU, in gp_Pnt theP, double theSqDist, bool isMin)
    {
        int i, NbExt = mypoint.Length();
        for (i = 1; i <= NbExt; i++)
        {
            double t = mypoint.Value(i).Parameter();
            if (Math.Abs(t - theU) <= mytolu)
            {
                return;
            }
        }
        Extrema_POnCurv PC = new(theU, theP);
        mySqDist.Append(theSqDist);
        myismin.Append(isMin);
        mypoint.Append(PC);
    }
}
