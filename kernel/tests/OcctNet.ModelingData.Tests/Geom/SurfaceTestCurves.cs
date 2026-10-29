// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// A circle written as OCCT's Geom_Circle (ElCLib's circle evaluators, Geom_Conic's reversal), standing in for
/// Geom_Circle in the tests of the swept surfaces until unit 1A's curves are merged.
/// </summary>
internal sealed class TestCircle : Geom_Conic
{
    private double radius;

    public TestCircle(in gp_Ax2 A2, double R)
    {
        radius = R;
        pos = A2;
    }

    public TestCircle(in gp_Circ C) : this(C.Position(), C.Radius()) { }

    public double Radius() => radius;

    public override Geom_Geometry Copy() => new TestCircle(pos, radius);

    public override bool IsClosed() => true;

    public override bool IsPeriodic() => true;

    public override double ReversedParameter(double U) => (2.0 * M_PI - U);

    public override double Eccentricity() => 0.0;

    public override double FirstParameter() => 0.0;

    public override double LastParameter() => 2.0 * M_PI;

    public override gp_Pnt EvalD0(double U) => ElCLib.CircleValue(U, pos, radius);

    public override ResD1 EvalD1(double U)
    {
        ResD1 aResult = new ResD1();
        ElCLib.CircleD1(U, pos, radius, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public override ResD2 EvalD2(double U)
    {
        ResD2 aResult = new ResD2();
        ElCLib.CircleD2(U, pos, radius, out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    public override ResD3 EvalD3(double U)
    {
        ResD3 aResult = new ResD3();
        ElCLib.CircleD3(U, pos, radius, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
        return aResult;
    }

    public override gp_Vec EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom_UndefinedDerivative();
        }
        return ElCLib.CircleDN(U, pos, radius, N);
    }

    protected internal override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        radius = radius * Math.Abs(T.ScaleFactor());
        pos.Transform(T);
    }
}
