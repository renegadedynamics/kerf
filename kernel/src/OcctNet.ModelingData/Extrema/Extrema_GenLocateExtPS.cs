// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtPS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtPS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// The extremum of the distance between a point and a surface near a starting point (u0, v0): by the normal
/// projection criteria (math_FunctionSetRoot, then Newton) or the distance criteria (math_BFGS, then math_FRPR).
/// </summary>
/// <remarks>The surface adaptor is kept, as OCCT keeps a reference: it must outlive the algorithm.</remarks>
public class Extrema_GenLocateExtPS
{
    // Input.
    private readonly Adaptor3d_Surface mySurf;
    private readonly double myTolU, myTolV;

    // State.
    private bool myDone;

    // Result.
    private double mySqDist;
    private Extrema_POnSurf myPoint = new();

    private static void CorrectTol(double theU0, double theV0, math_Vector theTol)
    {
        // Correct tolerance for large values of UV parameters
        double aTolRef = global::Kerf.Engine.Foundation.Precision.PConfusion();
        double anEpsRef = Epsilon(1.0);
        double epsu = Epsilon(theU0);
        const double tolog10 = 0.43429;
        if (epsu > anEpsRef)
        {
            int n = RealToInt(tolog10 * Math.Log(epsu / anEpsRef) + 1) + 1;
            int i;
            double tol = aTolRef;
            for (i = 1; i <= n; ++i)
            {
                tol *= 10.0;
            }
            theTol[1] = Max(theTol[1], tol);
        }
        double epsv = Epsilon(theV0);
        if (epsv > anEpsRef)
        {
            int n = RealToInt(tolog10 * Math.Log(epsv / anEpsRef) + 1) + 1;
            int i;
            double tol = aTolRef;
            for (i = 1; i <= n; ++i)
            {
                tol *= 10.0;
            }
            theTol[2] = Max(theTol[2], tol);
        }
    }

    /// <summary>
    /// True if (theU0, theV0) is a local minimum of the square distance between theP and theS(U, V) over a small area
    /// around it.
    /// </summary>
    public static bool IsMinDist(in Point3D theP, Adaptor3d_Surface theS, double theU0, double theV0)
    {
        double du = Max(theS.UResolution(10.0 * global::Kerf.Engine.Foundation.Precision.Confusion()), 10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion());
        double dv = Max(theS.VResolution(10.0 * global::Kerf.Engine.Foundation.Precision.Confusion()), 10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion());
        double u, v;
        Point3D aP0 = theS.Value(theU0, theV0);
        double d0 = theP.SquareDistance(aP0);
        int iu, iv;
        for (iu = -1; iu <= 1; ++iu)
        {
            u = theU0 + iu * du;
            if (!theS.IsUPeriodic())
            {
                u = Max(u, theS.FirstUParameter());
                u = Min(u, theS.LastUParameter());
            }
            for (iv = -1; iv <= 1; ++iv)
            {
                if (iu == 0 && iv == 0)
                {
                    continue;
                }

                v = theV0 + iv * dv;
                if (!theS.IsVPeriodic())
                {
                    v = Max(v, theS.FirstVParameter());
                    v = Min(v, theS.LastVParameter());
                }
                double d = theP.SquareDistance(theS.Value(u, v));
                if (d < d0)
                {
                    return false;
                }
            }
        }
        return true;
    }

    public Extrema_GenLocateExtPS(Adaptor3d_Surface theS, double theTolU = 1e-7 * 0.01,
        double theTolV = 1e-7 * 0.01) // Precision::PConfusion() (decision 129)
    {
        mySurf = theS;
        myTolU = theTolU;
        myTolV = theTolV;
        myDone = false;
        mySqDist = -1.0;
    }

    /// <summary>
    /// Computes the extremum near (theU0, theV0): normal projection criteria if <paramref name="isDistanceCriteria"/>
    /// is false, distance criteria otherwise.
    /// </summary>
    public void Perform(in Point3D theP, double theU0, double theV0, bool isDistanceCriteria = false)
    {
        myDone = false;

        // Prepare initial data structures.
        math_Vector aTol = new(1, 2), aStart = new(1, 2), aBoundInf = new(1, 2), aBoundSup = new(1, 2);

        // Tolerance.
        aTol[1] = myTolU;
        aTol[2] = myTolV;

        // Initial solution approximation.
        aStart[1] = theU0;
        aStart[2] = theV0;

        // Borders.
        aBoundInf[1] = mySurf.FirstUParameter();
        aBoundInf[2] = mySurf.FirstVParameter();
        aBoundSup[1] = mySurf.LastUParameter();
        aBoundSup[2] = mySurf.LastVParameter();

        if (isDistanceCriteria)
        {
            // Distance criteria.
            double aRelTol = 1.0e-8;
            math_Vector aResPnt = new(1, 2);

            Extrema_FuncPSDist F = new(mySurf, theP);

            Bfgs aSolver = new(2, aRelTol);
            aSolver.Perform(F, aStart);

            if (!aSolver.IsDone)
            {
                // Try another method
                Frpr aSolver1 = new(F, aRelTol);
                aSolver1.Perform(F, aStart);
                if (!aSolver1.IsDone)
                {
                    return;
                }
                aSolver1.Location(aResPnt);
                mySqDist = aSolver1.Minimum();
            }
            else
            {
                aSolver.Location(aResPnt);
                mySqDist = aSolver.Minimum();
            }

            myPoint.SetParameters(aResPnt[1], aResPnt[2], mySurf.Value(aResPnt[1], aResPnt[2]));
            myDone = true;
        }
        else
        {
            // Normal projection criteria.
            Extrema_FuncPSNorm F = new(theP, mySurf);

            if (mySurf.GetType() == BSplineSurface)
            {
                aTol[1] = myTolU;
                aTol[2] = myTolV;
                CorrectTol(theU0, theV0, aTol);
            }

            bool isCorrectTol = (Math.Abs(aTol[1] - myTolU) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                                 || Math.Abs(aTol[2] - myTolV) > global::Kerf.Engine.Foundation.Precision.PConfusion());

            FunctionSetRoot aSR = new(F, aTol);
            aSR.Perform(F, aStart, aBoundInf, aBoundSup);

            if (!aSR.IsDone || isCorrectTol)
            {
                if (isCorrectTol)
                {
                    aTol[1] = myTolU;
                    aTol[2] = myTolV;
                }
                NewtonFunctionSetRoot aNSR = new(F, aTol, global::Kerf.Engine.Foundation.Precision.Confusion());
                aNSR.Perform(F, aStart, aBoundInf, aBoundSup);
                if (!aSR.IsDone && !aNSR.IsDone)
                {
                    return;
                }
            }

            double aNbExt = F.NbExt();
            mySqDist = F.SquareDistance(1);
            myPoint = F.Point(1);
            int i;
            for (i = 2; i <= aNbExt; ++i)
            {
                if (F.SquareDistance(i) < mySqDist)
                {
                    mySqDist = F.SquareDistance(i);
                    myPoint = F.Point(i);
                }
            }
            myDone = true;
        }
    }

    /// <summary>True if the distance is found.</summary>
    public bool IsDone() => myDone;

    /// <summary>The square distance of the extremum.</summary>
    public double SquareDistance()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return mySqDist;
    }

    /// <summary>The point of the extremum.</summary>
    public Extrema_POnSurf Point()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return myPoint;
    }
}
