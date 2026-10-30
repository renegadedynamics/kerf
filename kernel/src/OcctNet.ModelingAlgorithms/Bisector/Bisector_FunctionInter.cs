// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_FunctionInter.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_FunctionInter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// F(u) = ||PC(u) - PBis1(u)|| - ||PC(u) - PBis2(u)||, used by Bisector_Inter to intersect two neighbouring bisectors
/// parameterised by the same curve.
/// </summary>
public sealed class Bisector_FunctionInter : math_FunctionWithDerivative
{
    private Geom2d_Curve curve = null!;
    private Bisector_Curve bisector1 = null!;
    private Bisector_Curve bisector2 = null!;

    /// <summary>An empty function: Perform must be called before it is evaluated.</summary>
    public Bisector_FunctionInter()
    {
    }

    /// <summary>The function of the curve C and the bisectors B1 and B2.</summary>
    public Bisector_FunctionInter(Geom2d_Curve C, Bisector_Curve B1, Bisector_Curve B2)
    {
        curve = C;
        bisector1 = B1;
        bisector2 = B2;
    }

    /// <summary>Sets the curve C and the bisectors B1 and B2.</summary>
    public void Perform(Geom2d_Curve C, Bisector_Curve B1, Bisector_Curve B2)
    {
        curve = C;
        bisector1 = B1;
        bisector2 = B2;
    }

    /// <inheritdoc/>
    public override bool Value(double X, out double F)
    {
        gp_Pnt2d PC = curve.Value(X);
        gp_Pnt2d PB1 = bisector1.Value(X);
        gp_Pnt2d PB2 = bisector2.Value(X);

        F = PC.Distance(PB1) - PC.Distance(PB2);

        return true;
    }

    /// <inheritdoc/>
    public override bool Derivative(double X, out double D)
    {
        return Values(X, out double _, out D);
    }

    /// <inheritdoc/>
    public override bool Values(double X, out double F, out double D)
    {
        double F1, F2, DF1, DF2;

        curve.D1(X, out gp_Pnt2d PC, out gp_Vec2d TC);
        bisector1.D1(X, out gp_Pnt2d PB1, out gp_Vec2d TB1);
        bisector2.D1(X, out gp_Pnt2d PB2, out gp_Vec2d TB2);

        F1 = PC.Distance(PB1);
        F2 = PC.Distance(PB2);

        F = F1 - F2;

        if (Math.Abs(F1) < gp.Resolution())
        {
            DF1 = Precision.Infinite();
        }
        else
        {
            DF1 = ((PC.X() - PB1.X()) * (TC.X() - TB1.X()) + (PC.Y() - PB1.Y()) * (TC.Y() - TB1.Y())) / F1;
        }
        if (Math.Abs(F2) < gp.Resolution())
        {
            DF2 = Precision.Infinite();
        }
        else
        {
            DF2 = ((PC.X() - PB2.X()) * (TC.X() - TB2.X()) + (PC.Y() - PB2.Y()) * (TC.Y() - TB2.Y())) / F2;
        }
        D = DF1 - DF2;

        return true;
    }
}
