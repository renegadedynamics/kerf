// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtCC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_LocECC2d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The extremum distance between two 2D curves nearest the starting parameters (U0, V0): the root of
/// <see cref="Extrema_CCLocFOfLocECC2d"/> found by math_FunctionSetRoot, kept if both functions are below 1e-10 there.
/// </summary>
/// <remarks>The 2D instantiation of OCCT's template Extrema_GenLocateExtCC (Extrema_LocECC2d), computed by its
/// constructor.</remarks>
public class Extrema_LocECC2d
{
    private readonly bool myDone;
    private readonly double mySqDist;
    private readonly Extrema_POnCurv2d myPoint1 = new();
    private readonly Extrema_POnCurv2d myPoint2 = new();

    /// <summary>The extremum between theC1 and theC2 nearest (theU0, theV0), with the parametric tolerances theTolU and
    /// theTolV.</summary>
    public Extrema_LocECC2d(Adaptor2d_Curve2d theC1, Adaptor2d_Curve2d theC2, double theU0, double theV0, double theTolU,
        double theTolV)
    {
        myDone = false;
        mySqDist = RealLast();

        double Uinf = Extrema_Curve2dTool.FirstParameter(theC1);
        double Usup = Extrema_Curve2dTool.LastParameter(theC1);
        double Uu;
        if (Uinf > Usup)
        {
            Uu = Uinf;
            Uinf = Usup;
            Usup = Uu;
        }

        double Vinf = Extrema_Curve2dTool.FirstParameter(theC2);
        double Vsup = Extrema_Curve2dTool.LastParameter(theC2);
        if (Vinf > Vsup)
        {
            Uu = Vinf;
            Vinf = Vsup;
            Vsup = Uu;
        }

        Extrema_CCLocFOfLocECC2d F = new(theC1, theC2);
        math_Vector Tol = new(1, 2);
        Tol[1] = theTolU;
        Tol[2] = theTolV;
        double Tolf = 1.0e-10;

        math_Vector Start = new(1, 2);
        math_Vector Uuinf = new(1, 2);
        math_Vector Uusup = new(1, 2);

        Start[1] = theU0;
        Start[2] = theV0;

        Uuinf[1] = Uinf;
        Uuinf[2] = Vinf;
        Uusup[1] = Usup;
        Uusup[2] = Vsup;

        math_FunctionSetRoot S = new(F, Tol);
        S.Perform(F, Start, Uuinf, Uusup);

        if (S.IsDone() && F.NbExt() > 0)
        {
            mySqDist = F.SquareDistance(1);
            F.Points(1, out myPoint1, out myPoint2);
            Start[1] = myPoint1.Parameter();
            Start[2] = myPoint2.Parameter();
            math_Vector Ff = new(1, 2);
            F.Value(Start, Ff);
            // OCCT compares the signed values, not their absolute values: any negative residual passes
            if ((Ff[1] < Tolf) && (Ff[2] < Tolf))
            {
                myDone = true;
            }
        }
    }

    /// <summary>True if the distance is found.</summary>
    public bool IsDone() => myDone;

    /// <summary>The square distance of the extremum.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public double SquareDistance()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        return mySqDist;
    }

    /// <summary>The points of the extremum, on the first and on the second curve.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public void Point(out Extrema_POnCurv2d theP1, out Extrema_POnCurv2d theP2)
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        theP1 = myPoint1;
        theP2 = myPoint2;
    }
}
