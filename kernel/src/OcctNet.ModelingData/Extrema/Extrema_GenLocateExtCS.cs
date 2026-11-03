// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtCS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtCS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

namespace Occt;

/// <summary>
/// The extremum of the distance between a curve and a surface near starting parameters T on the curve and (U, V)
/// on the surface: a zero of the gradient of F(t, u, v) = distance(C(t), S(u, v)), found by math_FunctionSetRoot.
/// </summary>
public class Extrema_GenLocateExtCS
{
    private bool myDone;
    private double mySqDist;
    private Extrema_POnCurv myPoint1 = new();
    private Extrema_POnSurf myPoint2 = new();

    public Extrema_GenLocateExtCS()
    {
        myDone = false;
        mySqDist = RealLast();
    }

    /// <summary>Computes the extremum near T on C and (U, V) on S (see <see cref="Perform"/>).</summary>
    public Extrema_GenLocateExtCS(Adaptor3d_Curve C, Adaptor3d_Surface S, double T, double U, double V, double Tol1,
        double Tol2)
    {
        Perform(C, S, T, U, V, Tol1, Tol2);
    }

    /// <summary>Computes the extremum near T on C and (U, V) on S, with tolerances Tol1 on t and Tol2 on u and v.</summary>
    public void Perform(Adaptor3d_Curve C, Adaptor3d_Surface S, double T, double U, double V, double Tol1, double Tol2)
    {
        myDone = false;

        double Tinf, Tsup;
        Tinf = C.FirstParameter();
        Tsup = C.LastParameter();

        double Uinf, Usup, Vinf, Vsup;
        Uinf = S.FirstUParameter();
        Usup = S.LastUParameter();
        Vinf = S.FirstVParameter();
        Vsup = S.LastVParameter();

        Extrema_FuncExtCS F = new(C, S);
        math_Vector Tol = new(1, 3), Start = new(1, 3), BInf = new(1, 3), BSup = new(1, 3);
        Tol[1] = Tol1;
        Tol[2] = Tol2;
        Tol[3] = Tol2;

        Start[1] = T;
        Start[2] = U;
        Start[3] = V;

        BInf[1] = Tinf;
        BInf[2] = Uinf;
        BInf[3] = Vinf;

        BSup[1] = Tsup;
        BSup[2] = Usup;
        BSup[3] = Vsup;

        FunctionSetRoot SR = new(F, Tol);
        SR.Perform(F, Start, BInf, BSup);
        if (!SR.IsDone)
        {
            return;
        }

        mySqDist = F.SquareDistance(1);
        myPoint1 = F.PointOnCurve(1);
        myPoint2 = F.PointOnSurface(1);
        myDone = true;
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

    /// <summary>The point of the extremum on the curve.</summary>
    public Extrema_POnCurv PointOnCurve()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return myPoint1;
    }

    /// <summary>The point of the extremum on the surface.</summary>
    public Extrema_POnSurf PointOnSurface()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return myPoint2;
    }
}
