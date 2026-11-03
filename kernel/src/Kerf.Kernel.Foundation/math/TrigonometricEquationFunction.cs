// Port of: src/FoundationClasses/TKMath/math/math_TrigonometricEquationFunction.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The function a*cos(x)*cos(x) + 2*b*cos(x)*sin(x) + c*cos(x) + d*sin(x) + e of the trigonometric equation solved by
/// math_TrigonometricFunctionRoots.
/// </summary>
public class TrigonometricEquationFunction : FunctionWithDerivative
{
    private double _aa;
    private double _bb;
    private double _cc;
    private double _dd;
    private double _ee;

    public TrigonometricEquationFunction(double a, double b, double c, double d, double e)
    {
        _aa = a;
        _bb = b;
        _cc = c;
        _dd = d;
        _ee = e;
    }

    public override bool Value(double x, out double f)
    {
        double cn = Math.Cos(x), sn = Math.Sin(x);
        //-- F= AA*CN*CN+2*BB*CN*SN+CC*CN+DD*SN+EE;
        f = cn * (_aa * cn + (_bb + _bb) * sn + _cc) + _dd * sn + _ee;
        return true;
    }

    public override bool Derivative(double x, out double d)
    {
        double cn = Math.Cos(x), sn = Math.Sin(x);
        //-- D = -2*AA*CN*SN+2*BB*(CN*CN-SN*SN)-CC*SN+DD*CN;
        d = -_aa * cn * sn + _bb * (cn * cn - sn * sn);
        d += d;
        d += -_cc * sn + _dd * cn;
        return true;
    }

    public override bool Values(double x, out double f, out double d)
    {
        double cn = Math.Cos(x), sn = Math.Sin(x);
        //-- F= AA*CN*CN+2*BB*CN*SN+CC*CN+DD*SN+EE;
        //-- D = -2*AA*CN*SN+2*BB*(CN*CN-SN*SN)-CC*SN+DD*CN;
        double aacn = _aa * cn;
        double bbsn = _bb * sn;
        f = aacn * cn + bbsn * (cn + cn) + _cc * cn + _dd * sn + _ee;
        d = -aacn * sn + _bb * (cn * cn - sn * sn);
        d += d;
        d += -_cc * sn + _dd * cn;
        return true;
    }
}
