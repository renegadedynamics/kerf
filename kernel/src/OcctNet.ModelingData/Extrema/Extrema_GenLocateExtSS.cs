// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtSS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtSS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

namespace Occt;

/// <summary>
/// The extremum of the distance between two surfaces near starting parameters (U1, V1) on S1 and (U2, V2) on S2: a
/// zero of the gradient of F = distance(S1(u1, v1), S2(u2, v2)), found by math_FunctionSetRoot.
/// </summary>
public class Extrema_GenLocateExtSS
{
    private bool myDone;
    private double mySqDist;
    private Extrema_POnSurf myPoint1 = new();
    private Extrema_POnSurf myPoint2 = new();

    public Extrema_GenLocateExtSS()
    {
        myDone = false;
        mySqDist = RealLast();
    }

    /// <summary>Computes the extremum near (U1, V1) on S1 and (U2, V2) on S2 (see <see cref="Perform"/>).</summary>
    public Extrema_GenLocateExtSS(Adaptor3d_Surface S1, Adaptor3d_Surface S2, double U1, double V1, double U2,
        double V2, double Tol1, double Tol2)
    {
        Perform(S1, S2, U1, V1, U2, V2, Tol1, Tol2);
    }

    /// <summary>
    /// Computes the extremum near (U1, V1) on S1 and (U2, V2) on S2, with the tolerances Tol1 on S1's parameters and
    /// Tol2 on S2's.
    /// </summary>
    public void Perform(Adaptor3d_Surface S1, Adaptor3d_Surface S2, double U1, double V1, double U2, double V2,
        double Tol1, double Tol2)
    {
        myDone = false;

        double Uinf1, Usup1, Vinf1, Vsup1;
        Uinf1 = S1.FirstUParameter();
        Usup1 = S1.LastUParameter();
        Vinf1 = S1.FirstVParameter();
        Vsup1 = S1.LastVParameter();

        double Uinf2, Usup2, Vinf2, Vsup2;
        Uinf2 = S2.FirstUParameter();
        Usup2 = S2.LastUParameter();
        Vinf2 = S2.FirstVParameter();
        Vsup2 = S2.LastVParameter();

        Extrema_FuncExtSS F = new(S1, S2);
        math_Vector Tol = new(1, 4), Start = new(1, 4), BInf = new(1, 4), BSup = new(1, 4);

        Tol[1] = Tol1;
        Tol[2] = Tol1;
        Tol[3] = Tol2;
        Tol[4] = Tol2;

        Start[1] = U1;
        Start[2] = V1;
        Start[3] = U2;
        Start[4] = V2;

        BInf[1] = Uinf1;
        BInf[2] = Vinf1;
        BInf[3] = Uinf2;
        BInf[4] = Vinf2;
        BSup[1] = Usup1;
        BSup[2] = Vsup1;
        BSup[3] = Usup2;
        BSup[4] = Vsup2;

        FunctionSetRoot SR = new(F, Tol);
        SR.Perform(F, Start, BInf, BSup);
        if (!SR.IsDone)
        {
            return;
        }

        mySqDist = F.SquareDistance(1);
        myPoint1 = F.PointOnS1(1);
        myPoint2 = F.PointOnS2(1);
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

    /// <summary>The point of the extremum on S1.</summary>
    public Extrema_POnSurf PointOnS1()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return myPoint1;
    }

    /// <summary>The point of the extremum on S2.</summary>
    public Extrema_POnSurf PointOnS2()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return myPoint2;
    }
}
