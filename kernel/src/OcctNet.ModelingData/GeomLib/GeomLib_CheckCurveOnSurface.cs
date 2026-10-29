// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_CheckCurveOnSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_CheckCurveOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.math_Status;

namespace Occt;

/// <summary>
/// Computes the maximal distance between a 3D curve and a 2D curve on a surface (an Adaptor3d_CurveOnSurface)
/// over the 3D curve's range. A working object (decision 21); the adaptors given must outlive it.
/// </summary>
public sealed class GeomLib_CheckCurveOnSurface
{
    private Adaptor3d_Curve? myCurve;
    private int myErrorStatus;
    private double myMaxDistance;
    private double myMaxParameter;
    private double myTolRange;
    private bool myIsParallel;

    /// <summary>An empty check: <see cref="Init(Adaptor3d_Curve?, double)"/> sets the curve.</summary>
    public GeomLib_CheckCurveOnSurface()
    {
        myErrorStatus = 0;
        myMaxDistance = RealLast();
        myMaxParameter = 0.0;
        myTolRange = Precision.PConfusion();
        myIsParallel = false;
    }

    /// <summary>A check of theCurve, whose range may exceed the curve on surface's by theTolRange.</summary>
    public GeomLib_CheckCurveOnSurface(Adaptor3d_Curve? theCurve, double theTolRange = 1.0e-7 * 0.01)
    {
        myCurve = theCurve;
        myErrorStatus = 0;
        myMaxDistance = RealLast();
        myMaxParameter = 0.0;
        myTolRange = theTolRange;
        myIsParallel = false;
    }

    /// <summary>Sets the data for the algorithm.</summary>
    public void Init(Adaptor3d_Curve? theCurve, double theTolRange = 1.0e-7 * 0.01)
    {
        myCurve = theCurve;
        myErrorStatus = 0;
        myMaxDistance = RealLast();
        myMaxParameter = 0.0;
        myTolRange = theTolRange;
    }

    /// <summary>Resets all members to their default values.</summary>
    public void Init()
    {
        myCurve = null;
        myErrorStatus = 0;
        myMaxDistance = RealLast();
        myMaxParameter = 0.0;
        myTolRange = Precision.PConfusion();
    }

    /// <summary>
    /// Computes the maximal distance between the 3D curve and theCurveOnSurface, in parallel if
    /// <see cref="SetParallel"/> was given true.
    /// </summary>
    public void Perform(Adaptor3d_CurveOnSurface? theCurveOnSurface)
    {
        if (myCurve is null || theCurveOnSurface is null)
        {
            myErrorStatus = 1;
            return;
        }

        if ((myCurve.FirstParameter() - theCurveOnSurface.FirstParameter() > myTolRange)
            || (myCurve.LastParameter() - theCurveOnSurface.LastParameter() < -myTolRange))
        {
            myErrorStatus = 2;
            return;
        }

        const double anEpsilonRange = 1.0e-3;

        int aNbParticles = 3;

        // Polynomial function with degree n has not more than n-1 maxima and
        // minima (degree of 1st derivative is equal to n-1 => 1st derivative has
        // no greater than n-1 roots). Consequently, this function has
        // maximum n monotonicity intervals. That is a good idea to try to put
        // at least one particle in every monotonicity interval. Therefore,
        // number of particles should be equal to n.

        int aNbSubIntervals = FillSubIntervals(myCurve,
                                               theCurveOnSurface.GetCurve()!,
                                               myCurve.FirstParameter(),
                                               myCurve.LastParameter(),
                                               ref aNbParticles);

        if (aNbSubIntervals == 0)
        {
            myErrorStatus = 3;
            return;
        }

        try
        {
            NCollection_Array1<double> anIntervals = new(1, aNbSubIntervals + 1);
            FillSubIntervals(myCurve,
                             theCurveOnSurface.GetCurve()!,
                             myCurve.FirstParameter(),
                             myCurve.LastParameter(),
                             ref aNbParticles,
                             anIntervals);

            // OSD_ThreadPool's default pool launches one thread per logical processor.
            int aNbThreads = myIsParallel ? Min(anIntervals.Length(), Environment.ProcessorCount) : 1;
            NCollection_Array1<Adaptor3d_Curve> aCurveArray = new(0, aNbThreads - 1);
            NCollection_Array1<Adaptor3d_Curve> aCurveOnSurfaceArray = new(0, aNbThreads - 1);
            for (int anI = 0; anI < aNbThreads; ++anI)
            {
                aCurveArray.SetValue(anI, aNbThreads > 1 ? myCurve.ShallowCopy() : myCurve);
                aCurveOnSurfaceArray.SetValue(anI,
                                              aNbThreads > 1 ? theCurveOnSurface.ShallowCopy() : theCurveOnSurface);
            }
            GeomLib_CheckCurveOnSurface_Local aComp = new(aCurveArray,
                                                          aCurveOnSurfaceArray,
                                                          anIntervals,
                                                          anEpsilonRange,
                                                          aNbParticles);
            if (aNbThreads > 1)
            {
                // OSD_ThreadPool::Launcher: each thread takes the next element of [Lower, Upper); every element is
                // computed on its own, so the distribution doesn't change the results.
                int aLower = anIntervals.Lower(), anUpper = anIntervals.Upper();
                try
                {
                    Parallel.For(0, aNbThreads, theThreadIndex =>
                    {
                        for (int anI = aLower + theThreadIndex; anI < anUpper; anI += aNbThreads)
                        {
                            aComp.Run(theThreadIndex, anI);
                        }
                    });
                }
                catch (AggregateException anExc) when (anExc.InnerException is Standard_Failure)
                {
                    throw anExc.InnerException;
                }
            }
            else
            {
                for (int anI = anIntervals.Lower(); anI < anIntervals.Upper(); ++anI)
                {
                    aComp.Run(0, anI);
                }
            }
            aComp.OptimalValues(out myMaxDistance, out myMaxParameter);

            myMaxDistance = Math.Sqrt(Math.Abs(myMaxDistance));
        }
        catch (Standard_Failure)
        {
            myErrorStatus = 3;
        }
    }

    /// <summary>Sets whether <see cref="Perform"/> runs in parallel.</summary>
    public void SetParallel(bool theIsParallel) => myIsParallel = theIsParallel;

    /// <summary>Whether the parallel flag is set.</summary>
    public bool IsParallel() => myIsParallel;

    /// <summary>Whether the maximal distance has been found.</summary>
    public bool IsDone() => myErrorStatus == 0;

    /// <summary>
    /// The error status: 0 OK; 1 null curve, surface or 2D curve; 2 invalid parametric range; 3 error in
    /// calculations.
    /// </summary>
    public int ErrorStatus() => myErrorStatus;

    /// <summary>The maximal distance.</summary>
    public double MaxDistance() => myMaxDistance;

    /// <summary>The parameter at which the distance is maximal.</summary>
    public double MaxParameter() => myMaxParameter;

    private sealed class GeomLib_CheckCurveOnSurface_TargetFunc : math_MultipleVarFunctionWithHessian
    {
        private readonly Adaptor3d_Curve myCurve1;
        private readonly Adaptor3d_Curve myCurve2;
        private readonly double myFirst;
        private readonly double myLast;

        public GeomLib_CheckCurveOnSurface_TargetFunc(Adaptor3d_Curve theC3D, Adaptor3d_Curve theCurveOnSurface,
                                                      double theFirst, double theLast)
        {
            myCurve1 = theC3D;
            myCurve2 = theCurveOnSurface;
            myFirst = theFirst;
            myLast = theLast;
        }

        // returns the number of parameters of the function
        // (the function is one-dimension).
        public override int NbVariables() => 1;

        // returns value of the function when parameters are equal to theX
        public override bool Value(math_Vector theX, out double theFVal) => Value(theX[1], out theFVal);

        // returns value of the one-dimension-function when parameter
        // is equal to theX (0 on failure, where OCCT leaves theFVal unchanged)
        public bool Value(double theX, out double theFVal)
        {
            theFVal = 0.0;
            try
            {
                if (!CheckParameter(theX))
                {
                    return false;
                }

                gp_Pnt aP1 = myCurve1.Value(theX), aP2 = myCurve2.Value(theX);

                theFVal = -1.0 * aP1.SquareDistance(aP2);
            }
            catch (Standard_Failure)
            {
                return false;
            }
            return true;
        }

        // see analogical method for abstract owner class math_MultipleVarFunction
        public override int GetStateNumber() => 0;

        // returns the gradient of the function when parameters are
        // equal to theX
        public override bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            if (!Derive(theX[1], out double aDeriv1, false, out _))
            {
                return false;
            }
            theGrad[1] = aDeriv1;
            return true;
        }

        // returns 1st derivative of the one-dimension-function when
        // parameter is equal to theX, and the 2nd one if theWithDeriv2 (OCCT's optional pointer)
        private bool Derive(double theX, out double theDeriv1, bool theWithDeriv2, out double theDeriv2)
        {
            theDeriv1 = 0.0;
            theDeriv2 = 0.0;
            try
            {
                if (!CheckParameter(theX))
                {
                    return false;
                }
                //
                gp_Pnt aP1, aP2;
                gp_Vec aDC1, aDC2, aDCC1 = new(), aDCC2 = new();
                //
                if (!theWithDeriv2)
                {
                    myCurve1.D1(theX, out aP1, out aDC1);
                    myCurve2.D1(theX, out aP2, out aDC2);
                }
                else
                {
                    myCurve1.D2(theX, out aP1, out aDC1, out aDCC1);
                    myCurve2.D2(theX, out aP2, out aDC2, out aDCC2);
                }

                gp_Vec aVec1 = new(aP1, aP2), aVec2 = aDC2 - aDC1;
                //
                theDeriv1 = -2.0 * aVec1.Dot(aVec2);

                if (theWithDeriv2)
                {
                    gp_Vec aVec3 = aDCC2 - aDCC1;
                    theDeriv2 = -2.0 * (aVec2.SquareMagnitude() + aVec1.Dot(aVec3));
                }
            }
            catch (Standard_Failure)
            {
                return false;
            }

            return true;
        }

        // returns value and gradient
        public override bool Values(math_Vector theX, out double theVal, math_Vector theGrad)
        {
            if (!Value(theX, out theVal))
            {
                return false;
            }
            //
            if (!Gradient(theX, theGrad))
            {
                return false;
            }
            //
            return true;
        }

        // returns value, gradient and hessian
        public override bool Values(math_Vector theX, out double theVal, math_Vector theGrad, math_Matrix theHessian)
        {
            if (!Value(theX, out theVal))
            {
                return false;
            }
            //
            if (!Derive(theX[1], out double aDeriv1, true, out double aDeriv2))
            {
                return false;
            }
            theGrad[1] = aDeriv1;
            theHessian[1, 1] = aDeriv2;
            //
            return true;
        }

        //
        public double FirstParameter() => myFirst;

        //
        public double LastParameter() => myLast;

        private struct Coefficients
        {
            public gp_XYZ Origin;
            public gp_XYZ Cos;
            public gp_XYZ Sin;
        }

        private static Coefficients getCoefficients(Adaptor3d_Curve theCurve, GeomAbs_CurveType theType)
        {
            if (theType == GeomAbs_Circle)
            {
                gp_Circ aCircle = theCurve.Circle();
                return new Coefficients
                {
                    Origin = aCircle.Location().XYZ(),
                    Cos = aCircle.XAxis().Direction().XYZ() * aCircle.Radius(),
                    Sin = aCircle.YAxis().Direction().XYZ() * aCircle.Radius(),
                };
            }

            gp_Elips anEllipse = theCurve.Ellipse();
            return new Coefficients
            {
                Origin = anEllipse.Location().XYZ(),
                Cos = anEllipse.XAxis().Direction().XYZ() * anEllipse.MajorRadius(),
                Sin = anEllipse.YAxis().Direction().XYZ() * anEllipse.MinorRadius(),
            };
        }

        // Computes the exact maximum for elementary compositions supported by the adaptors.
        public bool ElementaryMaximum(ref double theBestValue, ref double theBestParameter)
        {
            GeomAbs_CurveType aType1 = myCurve1.GetType();
            GeomAbs_CurveType aType2 = myCurve2.GetType();
            if (aType1 == GeomAbs_Line && aType2 == GeomAbs_Line)
            {
                double aFirstValue, aLastValue;
                if (!Value(myFirst, out aFirstValue) || !Value(myLast, out aLastValue))
                {
                    return false;
                }

                if (aFirstValue <= aLastValue)
                {
                    theBestValue = aFirstValue;
                    theBestParameter = myFirst;
                }
                else
                {
                    theBestValue = aLastValue;
                    theBestParameter = myLast;
                }
                return true;
            }

            bool isTrigType1 = aType1 == GeomAbs_Circle || aType1 == GeomAbs_Ellipse;
            bool isTrigType2 = aType2 == GeomAbs_Circle || aType2 == GeomAbs_Ellipse;
            if (!isTrigType1 || !isTrigType2)
            {
                return false;
            }

            Coefficients aCoeffs1 = getCoefficients(myCurve1, aType1);
            Coefficients aCoeffs2 = getCoefficients(myCurve2, aType2);

            gp_XYZ anOffset = aCoeffs1.Origin - aCoeffs2.Origin;
            gp_XYZ aCosCoeff = aCoeffs1.Cos - aCoeffs2.Cos;
            gp_XYZ aSinCoeff = aCoeffs1.Sin - aCoeffs2.Sin;

            // Coefficients of the derivative of the squared distance in trigonometric form.
            double aCosCos = 2.0 * aCosCoeff.Dot(aSinCoeff);
            double aCosSin = 0.5 * (aSinCoeff.SquareModulus() - aCosCoeff.SquareModulus());
            double aCos = anOffset.Dot(aSinCoeff);
            double aSin = -anOffset.Dot(aCosCoeff);
            double aConstant = -aCosCoeff.Dot(aSinCoeff);

            // Exact zero is required here: a tolerance could classify a varying distance as constant.
            if (aCosCos == 0.0 && aCosSin == 0.0 && aCos == 0.0 && aSin == 0.0 && aConstant == 0.0)
            {
                theBestParameter = myFirst;
                bool isOk = Value(myFirst, out double aValue);
                if (isOk)
                {
                    theBestValue = aValue;
                }
                return isOk;
            }

            double aPeriod = 2.0 * M_PI;
            double aFirst = ElCLib.InPeriod(myFirst, 0.0, aPeriod);
            double aShift = aFirst - myFirst;
            double aLast = aFirst + Min(myLast - myFirst, aPeriod);
            // std::max of an initializer list
            double aScale = Math.Abs(aCosCos);
            aScale = Max(aScale, Math.Abs(aCosSin));
            aScale = Max(aScale, Math.Abs(aCos));
            aScale = Max(aScale, Math.Abs(aSin));
            aScale = Max(aScale, Math.Abs(aConstant));
            math_TrigonometricFunctionRoots aRoots = new(aCosCos / aScale,
                                                         aCosSin / aScale,
                                                         aCos / aScale,
                                                         aSin / aScale,
                                                         aConstant / aScale,
                                                         aFirst,
                                                         aLast);
            if (!aRoots.IsDone() || aRoots.InfiniteRoots()
                || (myLast - myFirst >= aPeriod && aRoots.NbSolutions() == 0))
            {
                return false;
            }

            theBestValue = RealLast();
            theBestParameter = myFirst;

            if (!updateBest(myFirst, ref theBestValue, ref theBestParameter)
                || !updateBest(myLast, ref theBestValue, ref theBestParameter))
            {
                return false;
            }
            for (int anIndex = 1; anIndex <= aRoots.NbSolutions(); ++anIndex)
            {
                if (!updateBest(aRoots.Value(anIndex) - aShift, ref theBestValue, ref theBestParameter))
                {
                    return false;
                }
            }
            return true;
        }

        // OCCT's lambda updateBest
        private bool updateBest(double theParameter, ref double theBestValue, ref double theBestParameter)
        {
            if (!Value(theParameter, out double aValue))
            {
                return false;
            }
            if (aValue < theBestValue)
            {
                theBestValue = aValue;
                theBestParameter = theParameter;
            }
            return true;
        }

        // checks if the function can be computed when its parameter is
        // equal to theParam
        private bool CheckParameter(double theParam)
        {
            return ((myFirst <= theParam) && (theParam <= myLast));
        }
    }

    private sealed class GeomLib_CheckCurveOnSurface_Local
    {
        private readonly NCollection_Array1<Adaptor3d_Curve> myCurveArray;
        private readonly NCollection_Array1<Adaptor3d_Curve> myCurveOnSurfaceArray;

        private readonly NCollection_Array1<double> mySubIntervals;
        private readonly double myEpsilonRange;
        private readonly int myNbParticles;
        private readonly NCollection_Array1<double> myArrOfDist;
        private readonly NCollection_Array1<double> myArrOfParam;

        public GeomLib_CheckCurveOnSurface_Local(NCollection_Array1<Adaptor3d_Curve> theCurveArray,
                                                 NCollection_Array1<Adaptor3d_Curve> theCurveOnSurfaceArray,
                                                 NCollection_Array1<double> theIntervalsArr,
                                                 double theEpsilonRange,
                                                 int theNbParticles)
        {
            myCurveArray = theCurveArray;
            myCurveOnSurfaceArray = theCurveOnSurfaceArray;
            mySubIntervals = theIntervalsArr;
            myEpsilonRange = theEpsilonRange;
            myNbParticles = theNbParticles;
            myArrOfDist = new(theIntervalsArr.Lower(), theIntervalsArr.Upper() - 1);
            myArrOfParam = new(theIntervalsArr.Lower(), theIntervalsArr.Upper() - 1);
        }

        // OCCT's operator()(theThreadIndex, theElemIndex)
        public void Run(int theThreadIndex, int theElemIndex)
        {
            // For every sub-interval (which is set by mySubIntervals array) this method
            // computes optimal value of GeomLib_CheckCurveOnSurface_TargetFunc function.
            // This optimal value will be put in corresponding (depending on theIndex - the
            // identificator of the current interval in mySubIntervals array) cell of
            // myArrOfDist and myArrOfParam arrays.
            GeomLib_CheckCurveOnSurface_TargetFunc aFunc = new(myCurveArray.Value(theThreadIndex),
                                                               myCurveOnSurfaceArray.Value(theThreadIndex),
                                                               mySubIntervals.Value(theElemIndex),
                                                               mySubIntervals.Value(theElemIndex + 1));

            double aMinDist = RealLast(), aPar = 0.0;
            if (!MinComputing(aFunc, myEpsilonRange, myNbParticles, ref aMinDist, ref aPar))
            {
                myArrOfDist[theElemIndex] = RealLast();
                myArrOfParam[theElemIndex] = aFunc.FirstParameter();
                return;
            }

            myArrOfDist[theElemIndex] = aMinDist;
            myArrOfParam[theElemIndex] = aPar;
        }

        // Returns optimal value (inverse of square of maximal distance)
        public void OptimalValues(out double theMinimalValue, out double theParameter)
        {
            // This method looks for the minimal value of myArrOfDist.

            int aStartInd = myArrOfDist.Lower();
            theMinimalValue = myArrOfDist[aStartInd];
            theParameter = myArrOfParam[aStartInd];
            for (int i = aStartInd + 1; i <= myArrOfDist.Upper(); i++)
            {
                if (myArrOfDist[i] < theMinimalValue)
                {
                    theMinimalValue = myArrOfDist[i];
                    theParameter = myArrOfParam[i];
                }
            }
        }
    }

    // theSubIntervals is OCCT's optional pointer (null: only count)
    private static int FillSubIntervals(Adaptor3d_Curve theCurve3d,
                                        Adaptor2d_Curve2d theCurve2d,
                                        double theFirst,
                                        double theLast,
                                        ref int theNbParticles,
                                        NCollection_Array1<double>? theSubIntervals = null)
    {
        const int aMaxKnots = 101;
        NCollection_Array1<double> anArrTemp = new(1, 2);
        anArrTemp[1] = theFirst;
        anArrTemp[2] = theLast;

        theNbParticles = 3;
        Geom2d_BSplineCurve? aBS2DCurv = null;
        Geom_BSplineCurve? aBS3DCurv = null;
        bool isTrimmed3D = false, isTrimmed2D = false;

        //
        if (theCurve3d.GetType() == GeomAbs_BSplineCurve)
        {
            aBS3DCurv = theCurve3d.BSpline();
        }
        if (theCurve2d.GetType() == GeomAbs_BSplineCurve)
        {
            aBS2DCurv = theCurve2d.BSpline();
        }

        NCollection_HArray1<double> anArrKnots3D, anArrKnots2D;

        if (aBS3DCurv is not null)
        {
            if (aBS3DCurv.NbKnots() <= aMaxKnots)
            {
                anArrKnots3D = new NCollection_HArray1<double>(aBS3DCurv.InternalKnots());
            }
            else
            {
                int KnotCount;
                if (isTrimmed3D)
                {
                    int i;
                    KnotCount = 0;
                    NCollection_Array1<double> aKnots = aBS3DCurv.InternalKnots();
                    for (i = aBS3DCurv.FirstUKnotIndex(); i <= aBS3DCurv.LastUKnotIndex(); ++i)
                    {
                        if (aKnots[i] > theFirst && aKnots[i] < theLast)
                        {
                            ++KnotCount;
                        }
                    }
                    KnotCount += 2;
                }
                else
                {
                    KnotCount = aBS3DCurv.LastUKnotIndex() - aBS3DCurv.FirstUKnotIndex() + 1;
                }
                if (KnotCount <= aMaxKnots)
                {
                    anArrKnots3D = new NCollection_HArray1<double>(aBS3DCurv.InternalKnots());
                }
                else
                {
                    anArrKnots3D = new NCollection_HArray1<double>(1, aMaxKnots);
                    anArrKnots3D.SetValue(1, theFirst);
                    anArrKnots3D.SetValue(aMaxKnots, theLast);
                    int i;
                    double dt = (theLast - theFirst) / (aMaxKnots - 1);
                    double t = theFirst + dt;
                    for (i = 2; i < aMaxKnots; ++i, t += dt)
                    {
                        anArrKnots3D.SetValue(i, t);
                    }
                }
            }
        }
        else
        {
            anArrKnots3D = new NCollection_HArray1<double>(anArrTemp);
        }
        if (aBS2DCurv is not null)
        {
            if (aBS2DCurv.NbKnots() <= aMaxKnots)
            {
                anArrKnots2D = new NCollection_HArray1<double>(aBS2DCurv.InternalKnots());
            }
            else
            {
                int KnotCount;
                if (isTrimmed2D)
                {
                    int i;
                    KnotCount = 0;
                    NCollection_Array1<double> aKnots = aBS2DCurv.InternalKnots();
                    for (i = aBS2DCurv.FirstUKnotIndex(); i <= aBS2DCurv.LastUKnotIndex(); ++i)
                    {
                        if (aKnots[i] > theFirst && aKnots[i] < theLast)
                        {
                            ++KnotCount;
                        }
                    }
                    KnotCount += 2;
                }
                else
                {
                    KnotCount = aBS2DCurv.LastUKnotIndex() - aBS2DCurv.FirstUKnotIndex() + 1;
                }
                if (KnotCount <= aMaxKnots)
                {
                    anArrKnots2D = new NCollection_HArray1<double>(aBS2DCurv.InternalKnots());
                }
                else
                {
                    anArrKnots2D = new NCollection_HArray1<double>(1, aMaxKnots);
                    anArrKnots2D.SetValue(1, theFirst);
                    anArrKnots2D.SetValue(aMaxKnots, theLast);
                    int i;
                    double dt = (theLast - theFirst) / (aMaxKnots - 1);
                    double t = theFirst + dt;
                    for (i = 2; i < aMaxKnots; ++i, t += dt)
                    {
                        anArrKnots2D.SetValue(i, t);
                    }
                }
            }
        }
        else
        {
            anArrKnots2D = new NCollection_HArray1<double>(anArrTemp);
        }

        int aNbSubIntervals = 1;

        try
        {
            int anIndMax3D = anArrKnots3D.Upper(), anIndMax2D = anArrKnots2D.Upper();

            int anIndex3D = anArrKnots3D.Lower(), anIndex2D = anArrKnots2D.Lower();

            if (theSubIntervals is not null)
            {
                theSubIntervals[aNbSubIntervals] = theFirst;
            }

            while ((anIndex3D <= anIndMax3D) && (anIndex2D <= anIndMax2D))
            {
                double aVal3D = anArrKnots3D.Value(anIndex3D), aVal2D = anArrKnots2D.Value(anIndex2D);
                double aDelta = aVal3D - aVal2D;

                if (aDelta < Precision.PConfusion())
                { // aVal3D <= aVal2D
                    if ((aVal3D > theFirst) && (aVal3D < theLast))
                    {
                        aNbSubIntervals++;

                        if (theSubIntervals is not null)
                        {
                            theSubIntervals[aNbSubIntervals] = aVal3D;
                        }
                    }

                    anIndex3D++;

                    if (-aDelta < Precision.PConfusion())
                    { // aVal3D == aVal2D
                        anIndex2D++;
                    }
                }
                else
                { // aVal2D < aVal3D
                    if ((aVal2D > theFirst) && (aVal2D < theLast))
                    {
                        aNbSubIntervals++;

                        if (theSubIntervals is not null)
                        {
                            theSubIntervals[aNbSubIntervals] = aVal2D;
                        }
                    }

                    anIndex2D++;
                }
            }

            if (theSubIntervals is not null)
            {
                theSubIntervals[aNbSubIntervals + 1] = theLast;
            }

            if (aBS3DCurv is not null)
            {
                theNbParticles = Max(theNbParticles, aBS3DCurv.Degree());
            }

            if (aBS2DCurv is not null)
            {
                theNbParticles = Max(theNbParticles, aBS2DCurv.Degree());
            }
        }
        catch (Standard_Failure)
        {
            aNbSubIntervals = 0;
        }

        return aNbSubIntervals;
    }

    private static bool PSO_Perform(GeomLib_CheckCurveOnSurface_TargetFunc theFunction,
                                    math_Vector theParInf,
                                    math_Vector theParSup,
                                    double theEpsilon,
                                    int theNbParticles,
                                    out double theBestValue,
                                    math_Vector theOutputParam)
    {
        theBestValue = 0.0;
        double aDeltaParam = theParSup[1] - theParInf[1];
        if (aDeltaParam < Precision.PConfusion())
        {
            return false;
        }

        math_Vector aStepPar = new(1, 1);
        aStepPar[1] = theEpsilon * aDeltaParam;

        math_PSOParticlesPool aParticles = new(theNbParticles, 1);

        // They are used for finding a position of theNbParticles worst places
        int aNbControlPoints = 3 * theNbParticles;

        double aStep = aDeltaParam / (aNbControlPoints - 1);
        int aCount = 1;
        for (double aPrm = theParInf[1]; aCount <= aNbControlPoints;
             aCount++, aPrm = (aCount == aNbControlPoints) ? theParSup[1] : aPrm + aStep)
        {
            double aVal;
            if (!theFunction.Value(aPrm, out aVal))
            {
                continue;
            }

            PSO_Particle aParticle = aParticles.GetWorstParticle();

            if (aVal > aParticle.BestDistance)
            {
                continue;
            }

            aParticle.Position[aParticle.Position.Lower()] = aPrm;
            aParticle.BestPosition[aParticle.BestPosition.Lower()] = aPrm;
            aParticle.Distance = aVal;
            aParticle.BestDistance = aVal;
        }

        math_PSO aPSO = new(theFunction, theParInf, theParSup, aStepPar);
        aPSO.Perform(aParticles, theNbParticles, out theBestValue, theOutputParam);

        return true;
    }

    private static bool MinComputing(GeomLib_CheckCurveOnSurface_TargetFunc theFunction,
                                     double theEpsilon, // 1.0e-3
                                     int theNbParticles,
                                     ref double theBestValue,
                                     ref double theBestParameter)
    {
        try
        {
            //
            math_Vector aParInf = new(1, 1), aParSup = new(1, 1), anOutputParam = new(1, 1);
            aParInf[1] = theFunction.FirstParameter();
            aParSup[1] = theFunction.LastParameter();
            theBestParameter = aParInf[1];
            theBestValue = RealLast();

            if (theFunction.ElementaryMaximum(ref theBestValue, ref theBestParameter))
            {
                return true;
            }

            if (!PSO_Perform(theFunction,
                             aParInf,
                             aParSup,
                             theEpsilon,
                             theNbParticles,
                             out theBestValue,
                             anOutputParam))
            {
                return false;
            }

            theBestParameter = anOutputParam[1];

            // Here, anOutputParam contains parameter, which is near to optimal.
            // It needs to be more precise. Precision is made by math_NewtonMinimum.
            math_NewtonMinimum aMinSol = new(theFunction);
            aMinSol.Perform(theFunction, anOutputParam);

            if (aMinSol.IsDone() && (aMinSol.GetStatus() == math_OK))
            { // math_NewtonMinimum has precised the value. We take it.
                aMinSol.Location(anOutputParam);
                theBestParameter = anOutputParam[1];
                theBestValue = aMinSol.Minimum();
            }
            else
            { // Use math_PSO again but on smaller range.
                double aStep = theEpsilon * (aParSup[1] - aParInf[1]);
                aParInf[1] = theBestParameter - 0.5 * aStep;
                aParSup[1] = theBestParameter + 0.5 * aStep;

                double aValue;
                if (PSO_Perform(theFunction,
                                aParInf,
                                aParSup,
                                theEpsilon,
                                theNbParticles,
                                out aValue,
                                anOutputParam))
                {
                    if (aValue < theBestValue)
                    {
                        theBestValue = aValue;
                        theBestParameter = anOutputParam[1];
                    }
                }
            }
        }
        catch (Standard_Failure)
        {
            return false;
        }

        return true;
    }
}
