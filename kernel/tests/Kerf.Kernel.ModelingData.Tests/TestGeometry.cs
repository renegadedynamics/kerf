// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// A minimal concrete curve, written outside Kerf.Kernel.ModelingData as a user would: the line P(u) = O + u * D.
/// It exercises the abstract layer until Geom_Line is ported, and shows how another assembly overrides the
/// protected internal mutators: <c>protected internal override</c> here, because this assembly sees
/// ModelingData's internals; <c>protected override</c> in an assembly that doesn't.
/// </summary>
internal sealed class TestLine : Curve3D
{
    private Point3D myOrigin;
    private Vector3D myDir;

    public TestLine(in Point3D theOrigin, in Vector3D theDir)
    {
        myOrigin = theOrigin;
        myDir = theDir;
    }

    public Point3D Origin() => myOrigin;

    public Vector3D Dir() => myDir;

    protected internal override void Transform(in Transform3D T)
    {
        CheckMutable();
        myOrigin = myOrigin.Transformed(T);
        myDir = myDir.Transformed(T);
    }

    public override Geometry3D Copy() => new TestLine(myOrigin, myDir);

    protected internal override void Reverse()
    {
        CheckMutable();
        myDir = myDir.Reversed();
    }

    public override double ReversedParameter(double U) => -U;

    public override double FirstParameter => -global::Kerf.Engine.Foundation.Precision.Infinite();

    public override double LastParameter => global::Kerf.Engine.Foundation.Precision.Infinite();

    public override bool IsClosed => false;

    public override bool IsPeriodic => false;

    public override Continuity Continuity => CN;

    internal override bool IsCN(int N) => true;

    internal override Point3D EvalD0(double U) => myOrigin.Translated(myDir * U);

    internal override ResD1 EvalD1(double U) => new ResD1 { Point = EvalD0(U), D1 = myDir };

    internal override ResD2 EvalD2(double U) => new ResD2 { Point = EvalD0(U), D1 = myDir, D2 = new Vector3D(0, 0, 0) };

    internal override ResD3 EvalD3(double U) =>
        new ResD3 { Point = EvalD0(U), D1 = myDir, D2 = new Vector3D(0, 0, 0), D3 = new Vector3D(0, 0, 0) };

    internal override Vector3D EvalDN(double U, int N) => N == 1 ? myDir : new Vector3D(0, 0, 0);
}

/// <summary>
/// A minimal concrete surface: S(u, v) = (u, 2v, u * v), with nontrivial mixed derivatives so that parameter
/// maps (scale and UV swap) show in every derivative.
/// </summary>
internal sealed class TestSaddle : Surface
{
    protected internal override void Transform(in Transform3D T) => throw new NotSupportedException();

    public override Geometry3D Copy() => new TestSaddle();

    protected internal override void UReverse() => throw new NotSupportedException();

    protected internal override void VReverse() => throw new NotSupportedException();

    public override double UReversedParameter(double U) => -U;

    public override double VReversedParameter(double V) => -V;

    internal override void ParameterBounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = -10;
        U2 = 10;
        V1 = -10;
        V2 = 10;
    }

    public override bool IsUClosed => false;

    public override bool IsVClosed => false;

    public override bool IsUPeriodic => false;

    public override bool IsVPeriodic => false;

    public override Curve3D UIso(double U) => throw new NotSupportedException();

    public override Curve3D VIso(double V) => throw new NotSupportedException();

    public override Continuity Continuity => CN;

    internal override bool IsCNu(int N) => true;

    internal override bool IsCNv(int N) => true;

    internal override Point3D EvalD0(double U, double V) => new Point3D(U, 2 * V, U * V);

    internal override ResD1 EvalD1(double U, double V) =>
        new ResD1 { Point = EvalD0(U, V), D1U = new Vector3D(1, 0, V), D1V = new Vector3D(0, 2, U) };

    internal override ResD2 EvalD2(double U, double V)
    {
        ResD1 a = EvalD1(U, V);
        return new ResD2
        {
            Point = a.Point, D1U = a.D1U, D1V = a.D1V,
            D2U = new Vector3D(0, 0, 0), D2V = new Vector3D(0, 0, 0), D2UV = new Vector3D(0, 0, 1),
        };
    }

    internal override ResD3 EvalD3(double U, double V)
    {
        ResD2 a = EvalD2(U, V);
        Vector3D z = new Vector3D(0, 0, 0);
        return new ResD3
        {
            Point = a.Point, D1U = a.D1U, D1V = a.D1V, D2U = a.D2U, D2V = a.D2V, D2UV = a.D2UV,
            D3U = z, D3V = z, D3UUV = z, D3UVV = z,
        };
    }

    internal override Vector3D EvalDN(double U, double V, int Nu, int Nv) => (Nu, Nv) switch
    {
        (1, 0) => new Vector3D(1, 0, V),
        (0, 1) => new Vector3D(0, 2, U),
        (1, 1) => new Vector3D(0, 0, 1),
        _ => new Vector3D(0, 0, 0),
    };
}
