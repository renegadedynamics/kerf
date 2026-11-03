// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Construct;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeConstruct::ConvertSurfaceToBSpline when every approximation fails (occt-issues.md, unit 6H1's ShapeConstruct
/// row; Kerf unit 10K1, decision 1432): OCCT retried C0 for as long as GeomConvert_ApproxSurface threw, without end.
/// </summary>
public class ShapeConstructHangTests
{
    /// <summary>A surface whose evaluation always fails, as a surface with an undefined normal can.</summary>
    private sealed class FailingSurface : Surface
    {
        protected internal override void Transform(in Transform3D T) => throw new NotSupportedException();

        public override Geometry3D Copy() => new FailingSurface();

        protected internal override void UReverse() => throw new NotSupportedException();

        protected internal override void VReverse() => throw new NotSupportedException();

        public override double UReversedParameter(double U) => -U;

        public override double VReversedParameter(double V) => -V;

        public override void Bounds(out double U1, out double U2, out double V1, out double V2)
        {
            U1 = 0;
            U2 = 1;
            V1 = 0;
            V2 = 1;
        }

        public override bool IsUClosed => false;

        public override bool IsVClosed => false;

        public override bool IsUPeriodic => false;

        public override bool IsVPeriodic => false;

        public override Curve3D UIso(double U) => throw new NotSupportedException();

        public override Curve3D VIso(double V) => throw new NotSupportedException();

        public override Continuity Continuity() => CN;

        public override bool IsCNu(int N) => true;

        public override bool IsCNv(int N) => true;

        public override Point3D EvalD0(double U, double V) => throw new DomainErrorException("FailingSurface");

        public override ResD1 EvalD1(double U, double V) => throw new DomainErrorException("FailingSurface");

        public override ResD2 EvalD2(double U, double V) => throw new DomainErrorException("FailingSurface");

        public override ResD3 EvalD3(double U, double V) => throw new DomainErrorException("FailingSurface");

        public override Vector3D EvalDN(double U, double V, int Nu, int Nv) => throw new DomainErrorException("FailingSurface");
    }

    [Fact]
    public void ConvertSurfaceToBSpline_GivesUpWhenEveryApproximationFails()
    {
        BSplineSurface? aResult = HangGuard.Within(TimeSpan.FromSeconds(30),
            () => ShapeConstruct.ConvertSurfaceToBSpline(new FailingSurface(), 0, 1, 0, 1, 1e-3, C2, 10, 9));
        Assert.Null(aResult);
    }
}
