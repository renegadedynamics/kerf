// Port of: src/FoundationClasses/TKMath/math/math_TrigonometricEquationFunction.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The function a*cos(x)*cos(x) + 2*b*cos(x)*sin(x) + c*cos(x) + d*sin(x) + e of the trigonometric equation solved by
/// math_TrigonometricFunctionRoots.
/// </summary>
public class math_TrigonometricEquationFunction : math_FunctionWithDerivative
{
    private double myAA;
    private double myBB;
    private double myCC;
    private double myDD;
    private double myEE;

    public math_TrigonometricEquationFunction(double A, double B, double C, double D, double E)
    {
        myAA = A;
        myBB = B;
        myCC = C;
        myDD = D;
        myEE = E;
    }

    public override bool Value(double X, out double F)
    {
        double CN = Math.Cos(X), SN = Math.Sin(X);
        //-- F= AA*CN*CN+2*BB*CN*SN+CC*CN+DD*SN+EE;
        F = CN * (myAA * CN + (myBB + myBB) * SN + myCC) + myDD * SN + myEE;
        return true;
    }

    public override bool Derivative(double X, out double D)
    {
        double CN = Math.Cos(X), SN = Math.Sin(X);
        //-- D = -2*AA*CN*SN+2*BB*(CN*CN-SN*SN)-CC*SN+DD*CN;
        D = -myAA * CN * SN + myBB * (CN * CN - SN * SN);
        D += D;
        D += -myCC * SN + myDD * CN;
        return true;
    }

    public override bool Values(double X, out double F, out double D)
    {
        double CN = Math.Cos(X), SN = Math.Sin(X);
        //-- F= AA*CN*CN+2*BB*CN*SN+CC*CN+DD*SN+EE;
        //-- D = -2*AA*CN*SN+2*BB*(CN*CN-SN*SN)-CC*SN+DD*CN;
        double AACN = myAA * CN;
        double BBSN = myBB * SN;
        F = AACN * CN + BBSN * (CN + CN) + myCC * CN + myDD * SN + myEE;
        D = -AACN * SN + myBB * (CN * CN - SN * SN);
        D += D;
        D += -myCC * SN + myDD * CN;
        return true;
    }
}
