// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_ApproxArcsSegments.hxx
// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_ApproxArcsSegments.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// Approximation of a free-form 2D curve by a sequence of arcs of circle and line segments, within a distance
/// tolerance and an angular tolerance at the junctions. The approximation is made by the constructor.
/// </summary>
public sealed class Geom2dConvert_ApproxArcsSegments
{
    /// <summary>The state of the approximation.</summary>
    public enum Status
    {
        StatusOK = 0,
        StatusNotDone,
        StatusError,
    }

    private const int MAXPOINTS = 100;
    private const double MyCurvatureTolerance = 0.0001;

    private readonly Adaptor2d_Curve2d myCurve;
    private readonly Geom2dConvert_PPoint[] myExt = [new(), new()];

    private readonly double myTolerance;
    private readonly double myAngleTolerance;

    private readonly NCollection_Sequence<Geom2dConvert_PPoint> mySeqParams = new();
    private readonly NCollection_Sequence<Geom2d_Curve> mySeqCurves = new();
    private Status myStatus;

    /// <summary>
    /// Approximates theCurve with the distance tolerance theTolerance and the angular tolerance theAngleTol at the
    /// junctions. The adaptor is kept, as OCCT keeps a reference to it, but only during the construction.
    /// </summary>
    public Geom2dConvert_ApproxArcsSegments(Adaptor2d_Curve2d theCurve, double theTolerance, double theAngleTol)
    {
        myCurve = theCurve;
        myTolerance = theTolerance;
        myAngleTolerance = theAngleTol;
        myStatus = Status.StatusNotDone;

        myExt[0] = new Geom2dConvert_PPoint(myCurve.FirstParameter(), myCurve);
        myExt[1] = new Geom2dConvert_PPoint(myCurve.LastParameter(), myCurve);

        switch (myCurve.GetType())
        {
            case GeomAbs_Line:
            {
                // Create a single line segment.
                double aDist = myExt[0].Dist(myExt[1]);
                if (aDist > Precision.Confusion())
                {
                    gp_Ax2d anAx2d = new(new gp_Pnt2d(myExt[0].Point()),
                                         new gp_Dir2d(new gp_Vec2d(new gp_Pnt2d(myExt[0].Point()), new gp_Pnt2d(myExt[1].Point()))));
                    Geom2d_Line aLine = new(anAx2d);
                    mySeqCurves.Append(new Geom2d_TrimmedCurve(aLine, 0.0, aDist));
                    myStatus = Status.StatusOK;
                }
            }
            break;
            case GeomAbs_Circle:
            {
                // Create a couple of arcs of equal size.
                Geom2dConvert_PPoint aPP = new(.5 * (myExt[0].Parameter() + myExt[1].Parameter()), myCurve);
                Geom2d_Curve? aCurve = makeCircle(myExt[0], aPP);
                if (aCurve is not null)
                {
                    mySeqCurves.Append(aCurve);
                    aCurve = makeCircle(aPP, myExt[1]);
                    if (aCurve is not null)
                    {
                        mySeqCurves.Append(aCurve);
                    }
                }
            }
            break;
            default:
                makeFreeform();
                break;
        }

        // Check status of the calculation
        if (myStatus == Status.StatusNotDone)
        {
            if (!mySeqCurves.IsEmpty())
            {
                myStatus = Status.StatusOK;
            }
            else
            {
                myStatus = Status.StatusError;
            }
        }
    }

    /// <summary>
    /// The curves of the approximation (lines and arcs of circle, as trimmed curves), in order along the curve; a
    /// new list each call, holding the same curves, frozen.
    /// </summary>
    public IBentoList<Geom2d_Curve> GetResult()
    {
        var aCurves = new Geom2d_Curve[mySeqCurves.Length()];
        for (int i = 1; i <= mySeqCurves.Length(); i++)
        {
            Geom2d_Curve aCurve = mySeqCurves.Value(i);
            aCurve.Freeze();
            aCurves[i - 1] = aCurve;
        }

        return PackedList.Own(aCurves);
    }

    // Create arc of circle by three points (knowing that myCurve is circle).
    private Geom2d_Curve? makeCircle(in Geom2dConvert_PPoint theFirst, in Geom2dConvert_PPoint theLast)
    {
        Geom2d_Curve? aResult = null;
        double aParaM = (theFirst.Parameter() + theLast.Parameter()) * .5;
        myCurve.D0(aParaM, out gp_Pnt2d aPointM);
        GC_MakeArcOfCircle2d aMakeArc1 = new(new gp_Pnt2d(theFirst.Point()), aPointM, new gp_Pnt2d(theLast.Point()));

        if (aMakeArc1.IsDone())
        {
            aResult = aMakeArc1.Value();
        }

        return aResult;
    }

    // Create an arc of circle using 2 points and a derivative in the first point.
    private bool makeArc(in Geom2dConvert_PPoint theParam1,
                         ref Geom2dConvert_PPoint theParam2,
                         bool isFirst,
                         out Geom2d_TrimmedCurve? theCurve)
    {
        theCurve = null;
        gp_XY aP1 = theParam1.Point();
        gp_XY aP2 = theParam2.Point();
        gp_XY aVec = isFirst ? theParam1.D1() : -theParam1.D1();

        // Detect the sense (CCW means positive)
        gp_XY aDelta = aP2 - aP1;
        double aSense = aVec ^ aDelta;
        if (aSense > Precision.Angular())
        {
            aSense = 1.0;
        }
        else if (aSense < -Precision.Angular())
        {
            aSense = -1.0;
        }
        else
        {
            return false;
        }

        // Find the centre of the circle
        gp_XY aMiddle = (aP2 + aP1) * 0.5;
        double prodP1V = aP1 * aVec;
        double prodDeM = aDelta * aMiddle;
        double vprodVD = aVec ^ aDelta;
        double aResolution = gp.Resolution();

        if (vprodVD < -aResolution || vprodVD > aResolution)
        {
            gp_Pnt2d aCenter = new((prodP1V * aDelta.Y() - prodDeM * aVec.Y()) / vprodVD,
                                   (prodDeM * aVec.X() - prodP1V * aDelta.X()) / vprodVD);
            double aRad = (aCenter.Distance(new gp_Pnt2d(aP1)) + aCenter.Distance(new gp_Pnt2d(aP2))) * 0.5;
            gp_Ax22d ax22d = new(aCenter, new gp_Dir2d(gp_Dir2d.D.X), new gp_Dir2d(gp_Dir2d.D.Y));
            gp_Circ2d aCir = new(ax22d, aRad);
            Geom2d_Circle Circ = new(aCir);

            // calculation parameters first and last points of arc.
            double anAlpha1, anAlpha2;
            if (isFirst)
            {
                anAlpha1 = ElCLib.Parameter(aCir, new gp_Pnt2d(aP1));
                anAlpha2 = ElCLib.Parameter(aCir, new gp_Pnt2d(aP2));
            }
            else
            {
                anAlpha1 = ElCLib.Parameter(aCir, new gp_Pnt2d(aP2));
                anAlpha2 = ElCLib.Parameter(aCir, new gp_Pnt2d(aP1));
                aSense = -aSense;
            }

            if (Math.Abs(anAlpha1 - anAlpha2) < 1e-100)
            {
                // very small value, just to avoid exact match
                return false;
            }

            // Reverse the circle if the sense is negative
            if (aSense < 0.0)
            {
                anAlpha1 = Circ.ReversedParameter(anAlpha1);
                anAlpha2 = Circ.ReversedParameter(anAlpha2);
                Circ.Reverse();
            }

            theCurve = new Geom2d_TrimmedCurve(Circ, anAlpha1, anAlpha2);
            // Correct the direction in the opposite point
            gp_XY aRadV = theParam2.Point() - aCenter.XY();
            theParam2.SetD1(new gp_XY(-aRadV.Y() * aSense, aRadV.X() * aSense));
            return true;
        }

        // Algorithm failed, possibly because aVec is normal to the chorde
        return false;
    }

    // Make a line from myCurve in the limits by parameter from theFirst to theLast
    private Geom2d_TrimmedCurve? makeLine(ref Geom2dConvert_PPoint theFirst,
                                          ref Geom2dConvert_PPoint theLast,
                                          bool isCheck)
    {
        Geom2d_TrimmedCurve? aResult = null;

        gp_XY aSlope = theLast.Point() - theFirst.Point();
        if (Math.Abs(aSlope.SquareModulus()) < gp.Resolution())
        {
            return aResult;
        }

        gp_Dir2d aDirLine = new(aSlope);

        if (isCheck)
        {
            if (theFirst.D1().SquareModulus() < gp.Resolution() || theLast.D1().SquareModulus() < gp.Resolution())
            {
                return aResult;
            }

            // Angular continuity (G1) is only checked when the end of the line is not
            // on the extremity of the curve
            Span<double> absAngle = [0.0, 0.0];
            if (theFirst.IsNotEqual(myExt[0]))
            {
                double anAng = aDirLine.Angle(new gp_Dir2d(theFirst.D1()));
                absAngle[0] = (anAng > 0.0 ? anAng : -anAng);
            }

            if (theLast.IsNotEqual(myExt[1]))
            {
                double anAng = aDirLine.Angle(new gp_Dir2d(theLast.D1()));
                absAngle[1] = (anAng > 0.0 ? anAng : -anAng);
            }

            // if the derivatives in the end points differ from the derivative line
            // more than value of the specified continuity tolerance
            // then a biarc should be built instead of a line.
            double aContTolerance = Max(myAngleTolerance, 0.01);
            if (absAngle[0] > aContTolerance || absAngle[1] > aContTolerance)
            {
                return aResult;
            }
        } // end if (isCheck)

        // building segment of line
        GC_MakeSegment2d aMakeSeg = new(new gp_Pnt2d(theFirst.Point()), new gp_Pnt2d(theLast.Point()));
        if (aMakeSeg.IsDone())
        {
            Geom2d_TrimmedCurve aCurve = aMakeSeg.Value();
            if (checkCurve(aCurve, theFirst.Parameter(), theLast.Parameter()))
            {
                aResult = aCurve;
                // correct the derivatives fields in both arguments
                gp_XY aNewD1 = theLast.Point() - theFirst.Point();
                theFirst.SetD1(aNewD1);
                theLast.SetD1(aNewD1);
            }
        }

        return aResult;
    }

    // Create a sequence of elementary curves from a free-form adaptor curve.
    private bool makeFreeform()
    {
        NCollection_Sequence<Geom2dConvert_PPoint> seqParamPoints = new();
        ref Geom2dConvert_PPoint aPrevParam = ref myExt[0];

        // calculation of the inflection points.
        getLinearParts(seqParamPoints);
        bool isNoInfPoints = seqParamPoints.IsEmpty();

        NCollection_Sequence<Geom2d_Curve?> aSeqLinearParts = new();
        bool isDone = true;
        int i;
        for (i = 1; i < seqParamPoints.Length(); i += 2)
        {
            Geom2d_Curve? aLineCurve = null;
            ref Geom2dConvert_PPoint aParam0 = ref seqParamPoints.ChangeValue(i);
            ref Geom2dConvert_PPoint aParam1 = ref seqParamPoints.ChangeValue(i + 1);
            if (aParam0.IsNotEqual(aParam1))
            {
                // linear part of the curve lies between odd and even values of i.
                // parameters from parameter's sequence.
                aLineCurve = makeLine(ref aParam0, ref aParam1, false);
            }

            aSeqLinearParts.Append(aLineCurve);
        }

        for (i = 1; i < seqParamPoints.Length(); i += 2)
        {
            // approximation for non-linear part preceding the linear part
            if (seqParamPoints[i].IsNotEqual(aPrevParam))
            {
                int aLastInd = mySeqCurves.Length();
                isDone = makeApproximation(ref aPrevParam, ref seqParamPoints[i]);
                if (isDone && aLastInd != 0 && mySeqCurves.Length() > aLastInd)
                {
                    isDone = checkContinuity(mySeqCurves.Value(aLastInd), mySeqCurves.Value(aLastInd + 1), myAngleTolerance);
                }

                if (!isDone)
                {
                    myStatus = Status.StatusError;
                    break;
                }
            }

            Geom2d_Curve? aCurve = aSeqLinearParts.Value((i + 1) / 2);
            if (aCurve is not null)
            {
                mySeqCurves.Append(aCurve);
            }
            else
            {
                ref Geom2dConvert_PPoint aParam0 = ref seqParamPoints.ChangeValue(i);
                ref Geom2dConvert_PPoint aParam1 = ref seqParamPoints.ChangeValue(i + 1);
                int aLastInd = mySeqCurves.Length();
                isDone = makeApproximation(ref aParam0, ref aParam1);
                if (isDone && aLastInd != 0 && mySeqCurves.Length() > aLastInd)
                {
                    isDone = checkContinuity(mySeqCurves.Value(aLastInd), mySeqCurves.Value(aLastInd + 1), myAngleTolerance);
                }

                if (!isDone)
                {
                    myStatus = Status.StatusError;
                    break;
                }
            }

            aPrevParam = ref seqParamPoints[i + 1];
        }

        // approximation for non-linear part following the last linear part
        if (isDone && aPrevParam.IsNotEqual(myExt[1]))
        {
            // Case of a closed edge like an ellipse
            if (isNoInfPoints && (myExt[0].Point() - myExt[1].Point()).Modulus() < myTolerance)
            {
                Geom2dConvert_PPoint aPPoint = new(0.5 * (myExt[0].Parameter() + myExt[1].Parameter()), myCurve);
                isDone = makeApproximation(ref myExt[0], ref aPPoint);
                if (isDone)
                {
                    isDone = makeApproximation(ref aPPoint, ref myExt[1]);
                }
            }
            else
            {
                isDone = makeApproximation(ref aPrevParam, ref myExt[1]);
            }

            if (!isDone)
            {
                myStatus = Status.StatusError;
            }
        }

        return (mySeqCurves.Length() != 0 && myStatus != Status.StatusError);
    }

    // Obtain the linear intervals on the curve using as criteria
    // curvature tolerance (indicating either linear part or inflection)
    private void getLinearParts(NCollection_Sequence<Geom2dConvert_PPoint> theSeqPar)
    {
        int i;
        // Fill the sequences with values along the curve
        mySeqParams.Clear();
        GCPnts_QuasiUniformDeflection aQUDefAlgo = new(myCurve, myTolerance * 0.5);
        bool isUniformDone = aQUDefAlgo.IsDone();

        gp_XY aLastPnt = myExt[0].Point();
        if (isUniformDone)
        {
            for (i = 1; i <= aQUDefAlgo.NbPoints(); i++)
            {
                Geom2dConvert_PPoint aPP = new(aQUDefAlgo.Parameter(i), myCurve);
                mySeqParams.Append(aPP);
                aLastPnt = aPP.Point();
            }
        }
        else
        {
            double aParamStep = (myExt[1].Parameter() - myExt[0].Parameter()) / MAXPOINTS;
            for (i = 1; i <= MAXPOINTS; i++)
            {
                double aParam = myExt[0].Parameter() + aParamStep * i;
                Geom2dConvert_PPoint aPP = new(aParam, myCurve);
                mySeqParams.Append(aPP);
                aLastPnt = aPP.Point();
            }
        }

        // check if the curve may be linearised
        gp_XY aDir = myExt[1].Point() - myExt[0].Point();
        double aMod2 = aDir.SquareModulus();
        if (aMod2 > Precision.Confusion())
        {
            bool isLinear = true;
            aDir /= Math.Sqrt(aMod2);
            for (i = 1; i <= mySeqParams.Length(); i++)
            {
                // Distance from point "i" to the segment between two extremities
                double aDist = aDir ^ (mySeqParams[i].Point() - myExt[0].Point());
                if (aDist > myTolerance * 0.5 || aDist < -myTolerance * 0.5)
                {
                    isLinear = false;
                    break;
                }
            }

            if (isLinear)
            {
                theSeqPar.Append(myExt[0]);
                theSeqPar.Append(myExt[1]);
                return;
            }
        }

        // check if point for First Parameter is inflection point.
        int indStartLinear = 0;
        Geom2dConvert_PPoint aLastInflParam = myExt[0];
        Geom2dConvert_PPoint aFirstInflParam = myExt[0];

        // Getting further inflection points with step by parameter.
        // The point with index 1 is the same as myExt[0]
        for (i = 1; i <= mySeqParams.Length(); i++)
        {
            Geom2dConvert_PPoint aCurParam = mySeqParams[i];
            if (indStartLinear != 0)
            {
                bool isStillInflectionFirst =
                    isInflectionPoint(aFirstInflParam.Parameter(), aCurParam, myCurve, myAngleTolerance);
                if (isInflectionPoint(aCurParam.Parameter(), aFirstInflParam, myCurve, myAngleTolerance))
                {
                    aLastInflParam = mySeqParams[i];
                    while (!isStillInflectionFirst)
                    {
                        if (++indStartLinear >= i)
                        {
                            indStartLinear = 0;
                            break;
                        }

                        aFirstInflParam = mySeqParams[indStartLinear];
                        isStillInflectionFirst =
                            isInflectionPoint(aFirstInflParam.Parameter(), aCurParam, myCurve, myAngleTolerance);
                    }
                }
                else
                {
                    // Add the interval in the output sequence
                    // The interval is added only if it is more than 10 times the tolerance
                    aLastInflParam = findInflection(aLastInflParam, aCurParam);
                    if (!isInflectionPoint(aFirstInflParam.Parameter(), aLastInflParam, myCurve, myAngleTolerance))
                    {
                        aFirstInflParam = findInflection(aLastInflParam, aFirstInflParam);
                    }

                    double aDist = (aFirstInflParam.Point() - aLastInflParam.Point()).Modulus();
                    if (aFirstInflParam.Parameter() < aLastInflParam.Parameter() && aDist > 10 * myTolerance)
                    {
                        theSeqPar.Append(aFirstInflParam);
                        theSeqPar.Append(aLastInflParam);
                    }

                    indStartLinear = 0;
                }
            }
            else if (isInflectionPoint(aCurParam.Parameter(), myCurve))
            {
                aLastInflParam = aCurParam;
                if (i > 1)
                {
                    aFirstInflParam = findInflection(aCurParam, mySeqParams[i - 1]);
                }

                indStartLinear = i;
            }
        }

        double aDist2 = (aFirstInflParam.Point() - myExt[1].Point()).Modulus();
        if (indStartLinear != 0 && aDist2 > 10 * myTolerance)
        {
            theSeqPar.Append(aFirstInflParam);
            theSeqPar.Append(myExt[1]);
        }
    }

    // Dichotomic search of the boundary of inflection interval, between
    // two parameters on the Curve
    private Geom2dConvert_PPoint findInflection(in Geom2dConvert_PPoint theParamIsInfl,
                                                in Geom2dConvert_PPoint theParamNoInfl)
    {
        double aLower = theParamIsInfl.Parameter();
        double anUpper = theParamNoInfl.Parameter();
        double aTest = 0.0;
        for (int i = 0; i < 3; i++)
        { // 3 iterations
            aTest = (aLower + anUpper) * 0.5;
            if (isInflectionPoint(aTest, theParamIsInfl, myCurve, myAngleTolerance))
            {
                aLower = aTest;
            }
            else
            {
                anUpper = aTest;
            }
        }

        return new Geom2dConvert_PPoint(aTest, myCurve);
    }

    // Make approximation non-linear part of the other curve.
    private bool makeApproximation(ref Geom2dConvert_PPoint theFirstParam, ref Geom2dConvert_PPoint theLastParam)
    {
        // if difference between parameters is less than Precision::PConfusion
        // approximation was not made.
        bool isDone = false;
        if (theLastParam.IsNotEqual(theFirstParam))
        {
            double aDistance = (theFirstParam.Point() - theLastParam.Point()).Modulus();
            if (aDistance < myTolerance)
            {
                Geom2d_Curve? aCurve = makeLine(ref theFirstParam, ref theLastParam, true);
                isDone = aCurve is not null;
                if (isDone && mySeqCurves.Length() != 0)
                {
                    isDone = checkContinuity(mySeqCurves.Last(), aCurve!, myAngleTolerance);
                }

                if (isDone || aDistance < Precision.Confusion())
                {
                    // OCCT appends the curve even when makeLine returned a null handle (aDistance below
                    // Confusion); the port's sequence holds non-null curves, so a null one is left out.
                    // OCCT GetResult() can then contain null curves (occt-issues.md).
                    if (aCurve is not null)
                    {
                        mySeqCurves.Append(aCurve);
                    }

                    return isDone;
                }
            }

            // calculate biarc
            isDone = calculateBiArcs(ref theFirstParam, ref theLastParam);

            // if biarc was not calculated calculation is repeated on half the interval.
            if (!isDone)
            {
                Geom2dConvert_PPoint aParaM = new(theFirstParam.Parameter()
                                                    + (theLastParam.Parameter() - theFirstParam.Parameter()) * 0.55,
                                                  myCurve);
                isDone = makeApproximation(ref theFirstParam, ref aParaM);
                if (isDone)
                {
                    isDone = makeApproximation(ref aParaM, ref theLastParam);
                }
            }
        }

        return isDone;
    }

    // Method for calculation of a biarc.
    private bool calculateBiArcs(ref Geom2dConvert_PPoint theFirstParam, ref Geom2dConvert_PPoint theLastParam)
    {
        double aResolution = gp.Resolution();

        if (theFirstParam.D1().SquareModulus() < aResolution || theLastParam.D1().SquareModulus() < aResolution)
        {
            return false;
        }

        gp_XY aPnt0 = theFirstParam.Point(), aPnt1 = theLastParam.Point();
        gp_Dir2d aDir0 = new(theFirstParam.D1()), aDir1 = new(theLastParam.D1());

        // Try to approximate the curve by a single arc. The criterion for that is
        // more rigid if the curve is the entire input curve
        // (possible pb. connecting with other boundaries)
        gp_Vec2d aDelta = new(aPnt1 - aPnt0);
        double anAngle1 = aDelta.Angle(new gp_Vec2d(aDir0));
        if (anAngle1 < 0.0)
        {
            anAngle1 = -anAngle1;
        }

        double anAngle2 = aDelta.Angle(new gp_Vec2d(aDir1));
        if (anAngle2 < 0.0)
        {
            anAngle2 = -anAngle2;
        }

        // in the case when two angles are equal one arc can be built.
        double anAngleThreshold = Precision.Angular() * 10.0;
        if (theFirstParam.IsNotEqual(myExt[0]) || theLastParam.IsNotEqual(myExt[1]))
        {
            anAngleThreshold = myAngleTolerance * 0.1;
        }

        if (Math.Abs(anAngle1 - anAngle2) < anAngleThreshold)
        {
            // protect the theLastParam from modification of D1, when
            // the created arc is rejected.
            Geom2dConvert_PPoint aLastParam = theLastParam;
            if (!makeArc(theFirstParam, ref aLastParam, true, out Geom2d_TrimmedCurve? aCurve))
            {
                return false;
            }

            if (checkCurve(aCurve, theFirstParam.Parameter(), aLastParam.Parameter()))
            {
                theLastParam = aLastParam;
                mySeqCurves.Append(aCurve!);
                return true;
            }
        }

        // if one arc was not built or for other cases biarc will be built
        // method for building biarc was taken from article Ahmad H. Nasri et al.
        // "A Recursive Subdivision Algorithm for Piecewise Circular Spline",
        // Computer Graphics Forum, 2001.

        // definition of point of intersection two tangent directions in the points
        // corresponding FirstParameter and LastParameter.
        aDir1.Reverse();

        // Direct calculation of intersection point, replaces a class call below
        double aProd0 = aPnt0 ^ aDir0.XY(), aProd1 = aPnt1 ^ aDir1.XY(), aProd2 = aDir1 ^ new gp_Dir2d(aDir0.XY()); // C++ converts the gp_XY to a gp_Dir2d implicitly
        gp_XY aIntPoint = new((aProd0 * aDir1.X() - aProd1 * aDir0.X()) / aProd2,
                              (aProd0 * aDir1.Y() - aProd1 * aDir0.Y()) / aProd2);
        gp_XY aDiff0 = aIntPoint - aPnt0, aDiff1 = aIntPoint - aPnt1;
        if (aDiff0 * aDir0.XY() < 0.0 || aDiff1 * aDir1.XY() < 0.0)
        {
            return false;
        }

        // calculation middle point for building biarc.
        double ad1 = aDiff0.Modulus();
        double ad2 = aDiff1.Modulus();
        double ad12 = aDelta.Magnitude();

        double aB1 = ad1 / (ad1 + ad2);
        if (Math.Abs(aB1 - 0.5) < 0.0001)
        {
            return false;
        }

        gp_XY aXY0 = aPnt0 + aDir0.XY() * ad12 * ad1 / (ad12 + ad1 + ad2);
        gp_XY aXY1 = aPnt1 + aDir1.XY() * ad12 * ad2 / (ad12 + ad1 + ad2);

        gp_XY aXYmidArc = aXY0 + aB1 * (aXY1 - aXY0);
        Geom2dConvert_PPoint aParamMidArc =
            getParameter(aXYmidArc, theFirstParam.Parameter(), theLastParam.Parameter(), myCurve);

        // building first arc from biarc.
        if (!makeArc(theFirstParam, ref aParamMidArc, true, out Geom2d_TrimmedCurve? aCurve1))
        {
            return false;
        }

        if (!checkCurve(aCurve1, theFirstParam.Parameter(), aParamMidArc.Parameter()))
        {
            return false;
        }

        // building second arc from biarc.
        if (makeArc(theLastParam, ref aParamMidArc, false, out Geom2d_TrimmedCurve? aCurve2))
        {
            if (checkCurve(aCurve2, aParamMidArc.Parameter(), theLastParam.Parameter()))
            {
                mySeqCurves.Append(aCurve1!);
                mySeqCurves.Append(aCurve2!);
                return true;
            }
        }

        return false;
    }

    // Method for calculation of a linear interpolation. (Unused by OCCT, ported for completeness.)
    private bool calculateLines(ref Geom2dConvert_PPoint theFirstParam, in Geom2dConvert_PPoint theLastParam)
    {
        ref Geom2dConvert_PPoint aPrevParam = ref theFirstParam;
        for (int i = 1; i <= mySeqParams.Length(); i++)
        {
            ref Geom2dConvert_PPoint aCurParam = ref mySeqParams.ChangeValue(i);
            if (aCurParam.Parameter() < aPrevParam.Parameter())
            {
                continue;
            }

            if (aCurParam.Parameter() > theLastParam.Parameter())
            {
                break;
            }

            // build line segment
            if (aCurParam.IsNotEqual(aPrevParam))
            {
                double aDistance = (aCurParam.Point() - aPrevParam.Point()).Modulus();
                if (aDistance > myTolerance)
                {
                    Geom2d_Curve? aCurve = makeLine(ref aPrevParam, ref aCurParam, false);
                    if (aCurve is null)
                    {
                        return false;
                    }

                    mySeqCurves.Append(aCurve);
                    aPrevParam = ref mySeqParams[i];
                }
            }
        }

        return true;
    }

    // Checking max deflection Geom curve from Adaptor Curve
    private bool checkCurve(Geom2d_Curve? aCurve, double theFirstParam, double theLastParam)
    {
        if (aCurve is null)
        {
            return false; // check fails on empty input
        }

        bool isUniformDone = !mySeqParams.IsEmpty();
        // calculation sequence of the parameters or step by parameter.
        int aNbPnts = (isUniformDone ? mySeqParams.Length() : MAXPOINTS);
        double aParamStep = (theLastParam - theFirstParam) / MAXPOINTS;

        Geom2d_Curve aCurve1 = aCurve;
        if (aCurve is Geom2d_TrimmedCurve aTrCurve)
        {
            aCurve1 = aTrCurve.BasisCurve();
        }

        gp_Lin2d aLin2d = new();
        gp_Circ2d aCirc2d = new();
        bool isLine = aCurve1 is Geom2d_Line;
        bool isCircle = !isLine;
        if (aCurve1 is Geom2d_Line aGeomLine)
        {
            aLin2d = aGeomLine.Lin2d();
        }
        else
        {
            isCircle = aCurve1 is Geom2d_Circle;
            if (aCurve1 is Geom2d_Circle aGeomCircle)
            {
                aCirc2d = aGeomCircle.Circ2d();
            }
            else
            {
                return false;
            }
        }

        // calculation of the max deflection points from CurveAdaptor from Geom curve.
        bool isLess = true;
        int i = 1;
        for (; i <= aNbPnts && isLess; i++)
        {
            double aParam = (isUniformDone ? mySeqParams.Value(i).Parameter() : (theFirstParam + i * aParamStep));
            if (aParam < (theFirstParam - Precision.PConfusion()) || aParam > (theLastParam + Precision.PConfusion()))
            {
                continue;
            }

            // getting point from adaptor curve by specified parameter.
            gp_Pnt2d aProjPoint = new(0.0, 0.0);
            myCurve.D0(aParam, out gp_Pnt2d aPointAdaptor);
            double aParameterCurve;

            // getting point from geom curve by specified parameter.
            if (isLine)
            {
                aParameterCurve = ElCLib.Parameter(aLin2d, aPointAdaptor);
                aProjPoint = ElCLib.Value(aParameterCurve, aLin2d);
            }
            else if (isCircle)
            {
                aParameterCurve = ElCLib.Parameter(aCirc2d, aPointAdaptor);
                aProjPoint = ElCLib.Value(aParameterCurve, aCirc2d);
            }

            isLess = (aProjPoint.Distance(aPointAdaptor) < myTolerance + Precision.PConfusion());
        }

        return isLess;
    }

    private static bool checkContinuity(Geom2d_Curve theCurve1, Geom2d_Curve theCurve2, double theAngleTol)
    {
        theCurve1.D1(theCurve1.LastParameter(), out _, out gp_Vec2d v11);
        theCurve2.D1(theCurve2.FirstParameter(), out _, out gp_Vec2d v21);

        // check continuity with the specified tolerance.
        return (v11.IsParallel(v21, theAngleTol));
    }

    private static Geom2dConvert_PPoint getParameter(in gp_XY theXY1,
                                                     double theFirstParam,
                                                     double theLastParam,
                                                     Adaptor2d_Curve2d theCurve)
    {
        Geom2dConvert_PPoint aResult;
        double prevParam = theFirstParam;
        double af1 = theFirstParam;
        double af2 = theLastParam;

        // for finding nearest point use method half division.
        double aMinDist = RealLast();
        int i = 1;
        for (; i <= MAXPOINTS; i++)
        {
            aResult = new Geom2dConvert_PPoint(af1, theCurve);
            double adist1 = (theXY1 - aResult.Point()).Modulus();
            if (adist1 < Precision.Confusion())
            {
                return aResult;
            }

            aResult = new Geom2dConvert_PPoint(af2, theCurve);
            double adist2 = (theXY1 - aResult.Point()).Modulus();
            if (adist2 < Precision.Confusion())
            {
                return aResult;
            }

            if (aMinDist <= adist2 - Precision.Confusion() && aMinDist <= adist1 - Precision.Confusion())
            {
                break;
            }

            if (adist1 < adist2 - Precision.Confusion())
            {
                prevParam = af1;
                aMinDist = adist1;
                af2 = (af1 + af2) * 0.5;
            }
            else
            {
                prevParam = af2;
                aMinDist = adist2;
                af1 = (af1 + af2) * 0.5;
            }
        }

        aResult = new Geom2dConvert_PPoint(prevParam, theCurve);
        return aResult;
    }

    private static bool isInflectionPoint(double theParam, Adaptor2d_Curve2d theCurve)
    {
        theCurve.D2(theParam, out _, out gp_Vec2d aD1, out gp_Vec2d aD2);
        double aSqMod = aD1.XY().SquareModulus();
        double aCurvature = Math.Abs(aD1.XY() ^ aD2.XY()) / (aSqMod * Math.Sqrt(aSqMod));
        return (aCurvature < MyCurvatureTolerance);
    }

    private static bool isInflectionPoint(double theParam,
                                          in Geom2dConvert_PPoint theFirstInfl,
                                          Adaptor2d_Curve2d theCurve,
                                          double theAngleTol)
    {
        theCurve.D2(theParam, out gp_Pnt2d aP1, out gp_Vec2d aD1, out gp_Vec2d aD2);
        double aSqMod = aD1.XY().SquareModulus();
        double aCurvature = Math.Abs(aD1.XY() ^ aD2.XY()) / (aSqMod * Math.Sqrt(aSqMod));
        double aContAngle = Math.Abs(new gp_Vec2d(aP1.XY() - theFirstInfl.Point()).Angle(aD1));
        aContAngle = Min(aContAngle, Math.Abs(M_PI - aContAngle));
        return (aCurvature < MyCurvatureTolerance && aContAngle < theAngleTol);
    }
}
