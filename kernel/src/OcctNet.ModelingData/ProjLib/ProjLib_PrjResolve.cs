// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_PrjResolve.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_PrjResolve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Created on: 1997-11-06
// Created by: Roman BORISOV

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Finds the orthogonal projection of a curve point on a surface near a starting point, one of the variables
/// (t, u, v) fixed (<see cref="ProjLib_PrjFunc"/>): Newton's method, then math_FunctionSetRoot if it fails.
/// </summary>
/// <remarks>
/// A working object (decision 21). It keeps the caller's adaptors, as OCCT keeps its pointers: they must outlive
/// the solver.
/// </remarks>
public class ProjLib_PrjResolve
{
    private readonly Adaptor3d_Curve myCurve;
    private readonly Adaptor3d_Surface mySurface;
    private bool myDone;
    private Point2D mySolution;
    private readonly int myFix;

    /// <exception cref="ConstructionErrorException"><paramref name="Fix"/> is not 1, 2 or 3.</exception>
    public ProjLib_PrjResolve(Adaptor3d_Curve C, Adaptor3d_Surface S, int Fix)
    {
        myDone = false;
        myFix = Fix;
        if (myFix > 3 || myFix < 1)
        {
            throw new ConstructionErrorException();
        }
        mySolution = new Point2D(0.0, 0.0);
        myCurve = C;
        mySurface = S;
    }

    // void ProjLib_PrjResolve::Perform(const double t, const double U, const
    // double  V, const gp_Pnt2d& Tol2d, const gp_Pnt2d& Inf, const gp_Pnt2d& Sup, const
    // double FuncTol, const bool StrictInside)

    /// <summary>
    /// Calculates the orthogonal projection from C(t) to S near the point (U, V): the zero of the gradient of
    /// <c>distance(S(u, v), C(t))</c> in the free variables, within [Inf, Sup] widened by the tolerances.
    /// <paramref name="StrictInside"/> is not used (as in OCCT).
    /// </summary>
    public void Perform(double t,
                        double U,
                        double V,
                        in Point2D Tol2d,
                        in Point2D Inf,
                        in Point2D Sup,
                        double FuncTol = -1,
                        bool StrictInside = false)
    {
        _ = StrictInside;
        myDone = false;
        double FixVal = 0.0;
        Point2D ExtInf = new(0.0, 0.0), ExtSup = new(0.0, 0.0);
        double ExtU = 10 * Tol2d.X(), ExtV = 10 * Tol2d.Y();
        math_Vector Tol = new(1, 2), Start = new(1, 2), BInf = new(1, 2), BSup = new(1, 2);

        ExtInf.SetCoord(Inf.X() - ExtU, Inf.Y() - ExtV);
        ExtSup.SetCoord(Sup.X() + ExtU, Sup.Y() + ExtV);
        BInf[1] = ExtInf.X();
        BInf[2] = ExtInf.Y();
        BSup[1] = ExtSup.X();
        BSup[2] = ExtSup.Y();
        Tol[1] = Tol2d.X();
        Tol[2] = Tol2d.Y();

        switch (myFix)
        {
            case 1:
                Start[1] = U;
                Start[2] = V;
                FixVal = t;
                break;
            case 2:
                Start[1] = t;
                Start[2] = V;
                FixVal = U;
                break;
            case 3:
                Start[1] = t;
                Start[2] = U;
                FixVal = V;
                break;
        }

        ProjLib_PrjFunc F = new(myCurve, FixVal, mySurface, myFix);

        //  int option = 1;//2;
        //  if (option == 1) {
        //    math_FunctionSetRoot S1 (F, Start,Tol, BInf, BSup);
        //    if (!S1.IsDone()) { return; }
        //  }
        //  else {
        NewtonFunctionSetRoot SR = new(F, Tol, FuncTol);
        SR.Perform(F, Start, BInf, BSup);
        //    if (!SR.IsDone()) { return; }
        if (!SR.IsDone)
        {
            FunctionSetRoot S1 = new(F, Tol);
            S1.Perform(F, Start, BInf, BSup);

            if (!S1.IsDone)
            {
                return;
            }
        }

        mySolution.SetXY(F.Solution().XY());

        // computation of myDone
        myDone = true;

        double ExtraU, ExtraV;
        //  if(!StrictInside) {
        ExtraU = 2.0 * Tol2d.X();
        ExtraV = 2.0 * Tol2d.Y();
        //  }
        if (mySolution.X() > Inf.X() - Tol2d.X() && mySolution.X() < Inf.X())
        {
            mySolution.SetX(Inf.X());
        }
        if (mySolution.X() > Sup.X() && mySolution.X() < Sup.X() + Tol2d.X())
        {
            mySolution.SetX(Sup.X());
        }
        if (mySolution.Y() > Inf.Y() - Tol2d.Y() && mySolution.Y() < Inf.Y())
        {
            mySolution.SetY(Inf.Y());
        }
        if (mySolution.Y() > Sup.Y() && mySolution.Y() < Sup.Y() + Tol2d.Y())
        {
            mySolution.SetY(Sup.Y());
        }
        if (mySolution.X() < Inf.X() - ExtraU || mySolution.X() > Sup.X() + ExtraU
            || mySolution.Y() < Inf.Y() - ExtraV || mySolution.Y() > Sup.Y() + ExtraV)
        {
            myDone = false;
        }
        else if (FuncTol > 0)
        {
            math_Vector X = new(1, 2, 0.0), FVal = new(1, 2, 0.0);
            X[1] = mySolution.X();
            X[2] = mySolution.Y();

            F.Value(X, FVal);

            if (!SR.IsDone)
            {
                if ((FVal[1] * FVal[1] + FVal[2] * FVal[2]) > FuncTol)
                {
                    myDone = false;
                }
            }
        }
    }

    /// <summary>True if the distance is found.</summary>
    public bool IsDone() => myDone;

    /// <summary>The point of the extremum distance in the free variables.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public Point2D Solution()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return mySolution;
    }
}
