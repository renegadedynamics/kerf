// Port of: src/FoundationClasses/TKMath/math/math_GlobOptMin.hxx
// Port of: src/FoundationClasses/TKMath/math/math_GlobOptMin.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Evtushenko's algorithm of global optimization on a non-uniform mesh (Yu. Evtushenko, Numerical methods for
/// finding global extreme (case of a non-uniform mesh), U.S.S.R. Comput. Maths. Math. Phys., Vol. 11, N 6, pp. 38-54).
/// </summary>
/// <remarks>
/// The search space is a box in R^n; by default all the minima in it are found (maxima are not supported). The box can
/// be split by discontinuity criteria (<see cref="SetGlobalParams"/>, <see cref="SetLocalParams"/>). The continuity of
/// the local boxes (<see cref="SetContinuity"/>) selects the local method: Newton (C2, when the function has a
/// hessian), BFGS (C1, with a gradient) or Powell. The Lipschitz constant can be locked
/// (<see cref="SetLipConstState"/>); a single solution can be searched (<see cref="Perform"/>), which works well
/// with a known minimal value (<see cref="SetFunctionalMinimalValue"/>).
/// </remarks>
public class math_GlobOptMin
{
    // Class for duplicate fast search. For internal usage only.
    private sealed class NCollection_CellFilter_Inspector : INCollection_CellFilter_Inspector<math_Vector, math_Vector>
    {
        public NCollection_CellFilter_Inspector(int theDim, double theTol)
        {
            myCurrent = new math_Vector(1, theDim);
            myTol = theTol * theTol;
            myIsFind = false;
            Dimension = theDim;
        }

        // Access to coordinate.
        public static double Coord(int i, math_Vector thePnt) => thePnt[i + 1];

        // Auxiliary method to shift point by each coordinate on given value;
        // useful for preparing a points range for Inspect with tolerance.
        public void Shift(math_Vector thePnt, NCollection_Array1<double> theTol, math_Vector theLowPnt, math_Vector theUppPnt)
        {
            for (int anIdx = 1; anIdx <= Dimension; anIdx++)
            {
                theLowPnt[anIdx] = thePnt[anIdx] - theTol[anIdx - 1];
                theUppPnt[anIdx] = thePnt[anIdx] + theTol[anIdx - 1];
            }
        }

        public void ClearFind() => myIsFind = false;

        public bool isFind() => myIsFind;

        // Set current point to search for coincidence
        public void SetCurrent(math_Vector theCurPnt) => myCurrent.Initialized(theCurPnt);

        // Implementation of inspection method
        public NCollection_CellFilter_Action Inspect(math_Vector theObject)
        {
            double aSqDist = (myCurrent - theObject).Norm2();

            if (aSqDist < myTol)
            {
                myIsFind = true;
            }

            return NCollection_CellFilter_Action.CellFilter_Keep;
        }

        private double myTol;
        private math_Vector myCurrent;
        private bool myIsFind;
        private int Dimension;
    }

    // Input.
    private math_MultipleVarFunction myFunc;
    private int myN;
    private math_Vector myA; // Left border on current C2 interval.
    private math_Vector myB; // Right border on current C2 interval.
    private math_Vector myGlobA; // Global left border.
    private math_Vector myGlobB; // Global right border.
    private double myTol; // Discretization tolerance, default 1.0e-2.
    private double mySameTol; // points with ||p1 - p2|| < mySameTol is equal,
                              // function values |val1 - val2| * 0.01 < mySameTol is equal,
                              // default value is 1.0e-7.
    private double myC; // Lipchitz constant, default 9
    private double myInitC; // Lipchitz constant initial value.
    private bool myIsFindSingleSolution; // Default value is false.
    private double myFunctionalMinimalValue; // Default value is -Precision::Infinite
    private bool myIsConstLocked; // Is constant locked for modifications.

    // Output.
    private bool myDone;
    private NCollection_Sequence<double> myY = new(); // Current solutions.
    private int mySolCount; // Count of solutions.

    // Algorithm data.
    private double myZ;
    private double myE1; // Border coefficient.
    private double myE2; // Minimum step size.
    private double myE3; // Local extrema starting parameter.

    private math_Vector myX; // Current modified solution.
    private math_Vector myTmp; // Current modified solution.
    private math_Vector myV; // Steps array.
    private math_Vector myMaxV; // Max Steps array.
    private double myLastStep; // Last step.

    private NCollection_Array1<double> myCellSize;
    private int myMinCellFilterSol;
    private bool isFirstCellFilterInvoke;
    private NCollection_CellFilter<NCollection_CellFilter_Inspector, math_Vector, math_Vector> myFilter;

    // Continuity of local borders.
    private int myCont;

    private double myF; // Current value of Global optimum.

    private static double DistanceToBorder(math_Vector theX, math_Vector theMin, math_Vector theMax)
    {
        double aDist = RealLast();

        for (int anIdx = theMin.Lower(); anIdx <= theMin.Upper(); ++anIdx)
        {
            double aDist1 = Math.Abs(theX[anIdx] - theMin[anIdx]);
            double aDist2 = Math.Abs(theX[anIdx] - theMax[anIdx]);
            aDist = Min(aDist, Min(aDist1, aDist2));
        }

        return aDist;
    }

    /// <summary>
    /// Prepares the algorithm for the function <paramref name="theFunc"/> on the box [theA, theB], with the Lipschitz
    /// constant theC, the parameter space discretization tolerance and the functional value indifference tolerance.
    /// <see cref="Perform"/> does the job.
    /// </summary>
    public math_GlobOptMin(math_MultipleVarFunction theFunc, math_Vector theA, math_Vector theB, double theC = 9,
                           double theDiscretizationTol = 1.0e-2, double theSameTol = 1.0e-7)
    {
        myN = theFunc.NbVariables();
        myA = new math_Vector(1, myN);
        myB = new math_Vector(1, myN);
        myGlobA = new math_Vector(1, myN);
        myGlobB = new math_Vector(1, myN);
        myIsConstLocked = false;
        myX = new math_Vector(1, myN);
        myTmp = new math_Vector(1, myN);
        myV = new math_Vector(1, myN);
        myMaxV = new math_Vector(1, myN);
        myCellSize = new NCollection_Array1<double>(0, myN - 1);
        myFilter = new NCollection_CellFilter<NCollection_CellFilter_Inspector, math_Vector, math_Vector>(theFunc.NbVariables());
        myCont = 2;
        myF = Precision.Infinite();

        int i;

        myFunc = theFunc;
        myC = theC;
        myInitC = theC;
        myIsFindSingleSolution = false;
        myFunctionalMinimalValue = -Precision.Infinite();
        myZ = -1;
        mySolCount = 0;

        for (i = 1; i <= myN; i++)
        {
            myGlobA[i] = theA[i];
            myGlobB[i] = theB[i];

            myA[i] = theA[i];
            myB[i] = theB[i];
        }

        for (i = 1; i <= myN; i++)
        {
            myMaxV[i] = (myB[i] - myA[i]) / 3.0;
        }

        myTol = theDiscretizationTol;
        mySameTol = theSameTol;

        const int aMaxSquareSearchSol = 200;
        int aSolNb = (int)Math.Pow(3.0, (double)myN);
        myMinCellFilterSol = Max(2 * aSolNb, aMaxSquareSearchSol);
        initCellSize();
        ComputeInitSol();

        myDone = false;
    }

    /// <summary>Sets the parameters again, without allocating memory.</summary>
    public void SetGlobalParams(math_MultipleVarFunction theFunc, math_Vector theA, math_Vector theB, double theC = 9,
                                double theDiscretizationTol = 1.0e-2, double theSameTol = 1.0e-7)
    {
        int i;

        myFunc = theFunc;
        myC = theC;
        myInitC = theC;
        myZ = -1;
        mySolCount = 0;

        for (i = 1; i <= myN; i++)
        {
            myGlobA[i] = theA[i];
            myGlobB[i] = theB[i];

            myA[i] = theA[i];
            myB[i] = theB[i];
        }

        for (i = 1; i <= myN; i++)
        {
            myMaxV[i] = (myB[i] - myA[i]) / 3.0;
        }

        myTol = theDiscretizationTol;
        mySameTol = theSameTol;

        initCellSize();
        ComputeInitSol();

        myDone = false;
    }

    /// <summary>Reduces the search box to [theLocalA, theLocalB]; <see cref="Perform"/> uses it.</summary>
    public void SetLocalParams(math_Vector theLocalA, math_Vector theLocalB)
    {
        int i;

        myZ = -1;
        for (i = 1; i <= myN; i++)
        {
            myA[i] = theLocalA[i];
            myB[i] = theLocalB[i];
        }

        for (i = 1; i <= myN; i++)
        {
            myMaxV[i] = (myB[i] - myA[i]) / 3.0;
        }

        myDone = false;
    }

    /// <summary>Sets the parameter space discretization tolerance and the functional value indifference tolerance.</summary>
    public void SetTol(double theDiscretizationTol, double theSameTol)
    {
        myTol = theDiscretizationTol;
        mySameTol = theSameTol;
    }

    /// <summary>The parameter space discretization tolerance and the functional value indifference tolerance.</summary>
    public void GetTol(out double theDiscretizationTol, out double theSameTol)
    {
        theDiscretizationTol = myTol;
        theSameTol = mySameTol;
    }

    /// <summary>Searches the minima; a single one if <paramref name="isFindSingleSolution"/>.</summary>
    public void Perform(bool isFindSingleSolution = false)
    {
        // In this algo indexes started from 1, not from 0.
        myDone = false;

        // Compute parameters range
        double minLength = RealLast();
        double maxLength = RealFirst();
        for (int i = 1; i <= myN; i++)
        {
            double currentLength = myB[i] - myA[i];
            if (currentLength < minLength)
            {
                minLength = currentLength;
            }

            if (currentLength > maxLength)
            {
                maxLength = currentLength;
            }

            myV[i] = 0.0;
        }

        if (minLength < Precision.PConfusion())
        {
            return;
        }

        if (!myIsConstLocked)
        {
            // Compute initial value for myC.
            computeInitialValues();
        }

        myE1 = minLength * myTol;
        myE2 = maxLength * myTol;

        myIsFindSingleSolution = isFindSingleSolution;
        if (isFindSingleSolution)
        {
            // Run local optimization if current value better than optimal.
            myE3 = 0.0;
        }
        else
        {
            if (myC > 1.0)
            {
                myE3 = -maxLength * myTol / 4.0;
            }
            else
            {
                myE3 = -maxLength * myTol * myC / 4.0;
            }
        }

        // Search single solution and current solution in its neighborhood.
        if (CheckFunctionalStopCriteria())
        {
            myDone = true;
            return;
        }

        myLastStep = 0.0;
        isFirstCellFilterInvoke = true;
        computeGlobalExtremum(myN);

        myDone = true;
    }

    // theVal is ref: OCCT leaves it unchanged when no extremum is found.
    private bool computeLocalExtremum(math_Vector thePnt, ref double theVal, math_Vector theOutPnt)
    {
        int i;

        // OCCT: ComputeInitSol passes the same vector as thePnt and theOutPnt, so a local method that fails after
        // writing theOutPnt changes the starting point of the next method.

        // Newton method
        if (myCont >= 2 && myFunc is math_MultipleVarFunctionWithHessian)
        {
            math_MultipleVarFunctionWithHessian aTmp = (math_MultipleVarFunctionWithHessian)myFunc;
            math_NewtonMinimum newtonMinimum = new(aTmp);
            newtonMinimum.SetBoundary(myGlobA, myGlobB);
            newtonMinimum.Perform(aTmp, thePnt);

            if (newtonMinimum.IsDone())
            {
                newtonMinimum.Location(theOutPnt);
                theVal = newtonMinimum.Minimum();

                if (isInside(theOutPnt))
                {
                    return true;
                }
            }
        }

        // BFGS method used.
        if (myCont >= 1 && myFunc is math_MultipleVarFunctionWithGradient)
        {
            math_MultipleVarFunctionWithGradient aTmp = (math_MultipleVarFunctionWithGradient)myFunc;
            math_BFGS bfgs = new(aTmp.NbVariables());
            bfgs.SetBoundary(myGlobA, myGlobB);
            bfgs.Perform(aTmp, thePnt);

            if (bfgs.IsDone())
            {
                bfgs.Location(theOutPnt);
                theVal = bfgs.Minimum();

                if (isInside(theOutPnt))
                {
                    return true;
                }
            }
        }

        // Powell method used.
        if (myFunc is not null)
        {
            math_Matrix m = new(1, myN, 1, myN, 0.0);
            for (i = 1; i <= myN; i++)
            {
                m[i, i] = 1.0;
            }

            math_Powell powell = new(myFunc, 1e-10);
            powell.Perform(myFunc, thePnt, m);

            if (powell.IsDone())
            {
                powell.Location(theOutPnt);
                theVal = powell.Minimum();

                if (isInside(theOutPnt))
                {
                    return true;
                }
            }
        }

        return false;
    }

    // Computes the starting values: myF (initial best value), myY (initial best point) and myC (approximation of
    // the Lipschitz constant), to improve convergence speed.
    private void computeInitialValues()
    {
        const double aMinLC = 0.01;
        const double aMaxLC = 1000.0;
        const double aMinEps = 0.1;
        const double aMaxEps = 100.0;
        int i;
        math_Vector aCurrPnt = new(1, myN);
        math_Vector aBestPnt = new(1, myN);
        math_Vector aParamStep = new(1, myN);
        double aCurrVal = RealLast();

        // Lipchitz const approximation.
        double aLipConst = 0.0, aPrevValDiag, aPrevValProj;
        int aPntNb = 13;
        myFunc.Value(myA, out aPrevValDiag);
        aPrevValProj = aPrevValDiag;
        double aStep = (myB - myA).Norm() / aPntNb;
        aParamStep.Initialized((myB - myA) / aPntNb);
        for (i = 1; i <= aPntNb; i++)
        {
            aCurrPnt.Initialized(myA + aParamStep * i);

            // Walk over diagonal.
            myFunc.Value(aCurrPnt, out aCurrVal);
            aLipConst = Max(Math.Abs(aCurrVal - aPrevValDiag), aLipConst);
            aPrevValDiag = aCurrVal;

            // Walk over diag in projected space aPnt(1) = myA(1) = const.
            aCurrPnt[1] = myA[1];
            myFunc.Value(aCurrPnt, out aCurrVal);
            aLipConst = Max(Math.Abs(aCurrVal - aPrevValProj), aLipConst);
            aPrevValProj = aCurrVal;
        }

        myC = myInitC;
        aLipConst *= Math.Sqrt(myN) / aStep;
        if (aLipConst < myC * aMinEps)
        {
            myC = Max(aLipConst * aMinEps, aMinLC);
        }
        else if (aLipConst > myC * aMaxEps)
        {
            myC = Min(myC * aMaxEps, aMaxLC);
        }
    }

    private void computeGlobalExtremum(int j)
    {
        int i;
        double d = RealLast(), aPrevVal; // Functional in original and moved points.
        double val = RealLast(); // Local extrema computed in moved point.
        double aStepBestValue = RealLast();
        math_Vector aStepBestPoint = new(1, myN);
        bool isInside = false, isReached = false;

        double r1, r2, r;

        for (myX[j] = myA[j] + myE1; !isReached; myX[j] += myV[j])
        {
            if (myX[j] > myB[j])
            {
                myX[j] = myB[j];
                isReached = true;
            }

            if (CheckFunctionalStopCriteria())
            {
                return; // Best possible value is obtained.
            }

            if (j == 1)
            {
                isInside = false;
                aPrevVal = d;
                myFunc.Value(myX, out d);
                r1 = (d + myZ * myC * myLastStep - myF) * myZ; // Evtushenko estimation.
                r2 = ((d + aPrevVal - myC * myLastStep) * 0.5 - myF) * myZ; // Shubert / Piyavsky estimation.
                r = Min(r1, r2);
                if (r > myE3)
                {
                    double aSaveParam = myX[1];

                    // Piyavsky midpoint estimation.
                    double aParam = (2 * myX[1] - myV[1]) * 0.5 + (aPrevVal - d) * 0.5 / myC;
                    if (Precision.IsInfinite(aPrevVal))
                    {
                        aParam = myX[1] - myV[1] * 0.5; // Protection from upper dimension step.
                    }

                    myX[1] = aParam;
                    double aVal = 0;
                    myFunc.Value(myX, out aVal);
                    myX[1] = aSaveParam;

                    if ((aVal < d && aVal < aPrevVal)
                        || DistanceToBorder(myX, myA, myB) < myE1) // Condition optimization case near the border.
                    {
                        isInside = computeLocalExtremum(myX, ref val, myTmp);
                    }
                }

                aStepBestValue = (isInside && (val < d)) ? val : d;
                aStepBestPoint.Initialized((isInside && (val < d)) ? myTmp : myX);

                // Check point and value on the current step to be optimal.
                checkAddCandidate(aStepBestPoint, aStepBestValue);

                if (CheckFunctionalStopCriteria())
                {
                    return; // Best possible value is obtained.
                }

                myV[1] = Min(myE2 + Math.Abs(myF - d) / myC, myMaxV[1]);
                myLastStep = myV[1];
            }
            else
            {
                myV[j] = RealLast() / 2.0;
                computeGlobalExtremum(j - 1);

                // Nullify steps on lower dimensions.
                for (i = 1; i < j; i++)
                {
                    myV[i] = 0.0;
                }
            }

            if (j < myN)
            {
                double aUpperDimStep = Max(myV[j], myE2);
                if (myV[j + 1] > aUpperDimStep)
                {
                    if (aUpperDimStep > myMaxV[j + 1])
                    { // Case of too big step.
                        myV[j + 1] = myMaxV[j + 1];
                    }
                    else
                    {
                        myV[j + 1] = aUpperDimStep;
                    }
                }
            }
        }
    }

    // Checks that myGlobA <= thePnt <= myGlobB.
    private bool isInside(math_Vector thePnt)
    {
        int i;

        for (i = 1; i <= myN; i++)
        {
            if (thePnt[i] < myGlobA[i] || thePnt[i] > myGlobB[i])
            {
                return false;
            }
        }

        return true;
    }

    // Checks presence of thePnt in the solutions.
    private bool isStored(math_Vector thePnt)
    {
        int i, j;
        bool isSame = true;
        math_Vector aTol = new(1, myN);
        aTol.Initialized((myB - myA) * mySameTol);

        // C1 * n^2 = C2 * 3^dim * n
        if (mySolCount < myMinCellFilterSol)
        {
            for (i = 0; i < mySolCount; i++)
            {
                isSame = true;
                for (j = 1; j <= myN; j++)
                {
                    if ((Math.Abs(thePnt[j] - myY[i * myN + j])) > aTol[j])
                    {
                        isSame = false;
                        break;
                    }
                }

                if (isSame)
                {
                    return true;
                }
            }
        }
        else
        {
            NCollection_CellFilter_Inspector anInspector = new(myN, Precision.PConfusion());
            if (isFirstCellFilterInvoke)
            {
                myFilter.Reset(myCellSize);

                // Copy initial data into cell filter.
                for (int aSolIdx = 0; aSolIdx < mySolCount; aSolIdx++)
                {
                    math_Vector aVec = new(1, myN);
                    for (int aSolDim = 1; aSolDim <= myN; aSolDim++)
                    {
                        aVec[aSolDim] = myY[aSolIdx * myN + aSolDim];
                    }

                    myFilter.Add(aVec, aVec);
                }
            }

            isFirstCellFilterInvoke = false;

            math_Vector aLow = new(1, myN), anUp = new(1, myN);
            anInspector.Shift(thePnt, myCellSize, aLow, anUp);

            anInspector.ClearFind();
            anInspector.SetCurrent(thePnt);
            myFilter.Inspect(aLow, anUp, ref anInspector);
            if (!anInspector.isFind())
            {
                // Point is out of close cells, add new one.
                // The C++ filter stores a copy of the target; callers reuse their vector, so copy it here.
                math_Vector aStored = new(thePnt);
                myFilter.Add(aStored, aStored);
            }
        }

        return false;
    }

    /// <summary>Copies the solution number <paramref name="theIndex"/>, 1 &lt;= theIndex &lt;= NbExtrema, into <paramref name="theSol"/>.</summary>
    public void Points(int theIndex, math_Vector theSol)
    {
        int j;

        for (j = 1; j <= myN; j++)
        {
            theSol[j] = myY[(theIndex - 1) * myN + j];
        }
    }

    private void initCellSize()
    {
        for (int anIdx = 1; anIdx <= myN; anIdx++)
        {
            myCellSize[anIdx - 1] = (myGlobB[anIdx] - myGlobA[anIdx]) * Precision.PConfusion() / (2.0 * M_SQRT2);
        }
    }

    // Checks the possibility to stop computations: a single solution is searched and the best possible one is found.
    private bool CheckFunctionalStopCriteria()
    {
        // Search single solution and current solution in its neighborhood.
        return myIsFindSingleSolution && Math.Abs(myF - myFunctionalMinimalValue) < mySameTol * 0.01;
    }

    private void ComputeInitSol()
    {
        double aVal;
        math_Vector aPnt = new(1, myN);

        // Check functional value in midpoint. It is necessary since local optimization
        // algorithm may fail and return nothing. This is a protection from uninitialized
        // variables.
        aPnt.Initialized((myGlobA + myGlobB) * 0.5);
        myFunc.Value(aPnt, out aVal);
        checkAddCandidate(aPnt, aVal);

        // Run local optimization from lower corner, midpoint, and upper corner.
        for (int i = 1; i <= 3; i++)
        {
            aPnt.Initialized(myA + (myB - myA) * (i - 1) / 2.0);

            if (computeLocalExtremum(aPnt, ref aVal, aPnt))
            {
                checkAddCandidate(aPnt, aVal);
            }
        }
    }

    // Checks and adds a candidate point into the solutions.
    private void checkAddCandidate(math_Vector thePnt, double theValue)
    {
        if (Math.Abs(theValue - myF) < mySameTol * 0.01 && // Value in point is close to optimal value.
            !myIsFindSingleSolution) // Several optimal solutions are allowed.
        {
            if (!isStored(thePnt))
            {
                if ((theValue - myF) * myZ > 0.0)
                {
                    myF = theValue;
                }

                for (int j = 1; j <= myN; j++)
                {
                    myY.Append(thePnt[j]);
                }

                mySolCount++;
            }
        }

        // New best solution:
        // new point is out of (mySameTol * 0.01) surrounding or
        // new point is better than old and single point search.
        double aDelta = (theValue - myF) * myZ;
        if (aDelta > mySameTol * 0.01 || (aDelta > 0.0 && myIsFindSingleSolution))
        {
            myF = theValue;
            myY.Clear();
            for (int j = 1; j <= myN; j++)
            {
                myY.Append(thePnt[j]);
            }

            mySolCount = 1;

            isFirstCellFilterInvoke = true;
        }
    }

    /// <summary>Sets the continuity of the local borders splits (0 ~ C0, 1 ~ C1, 2 ~ C2).</summary>
    public void SetContinuity(int theCont) => myCont = theCont;

    /// <summary>The continuity of the local borders splits (0 ~ C0, 1 ~ C1, 2 ~ C2).</summary>
    public int GetContinuity() => myCont;

    /// <summary>Sets the functional minimal value.</summary>
    public void SetFunctionalMinimalValue(double theMinimalValue) => myFunctionalMinimalValue = theMinimalValue;

    public double GetFunctionalMinimalValue() => myFunctionalMinimalValue;

    /// <summary>Locks (true) or unlocks the Lipschitz constant against modification.</summary>
    public void SetLipConstState(bool theFlag) => myIsConstLocked = theFlag;

    public bool GetLipConstState() => myIsConstLocked;

    /// <summary>The computation state of the algorithm.</summary>
    public bool isDone() => myDone;

    /// <summary>The best functional value.</summary>
    public double GetF() => myF;

    /// <summary>The number of global extrema.</summary>
    public int NbExtrema() => mySolCount;
}
