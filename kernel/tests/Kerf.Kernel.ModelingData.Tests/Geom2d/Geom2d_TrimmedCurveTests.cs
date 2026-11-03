// Port of: src/ModelingData/TKG2d/GTests/Geom2d_TrimmedCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom2d_TrimmedCurveTest
{
    // Line through origin along X
    private readonly LineCurve2D myLine = new LineCurve2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));

    // Circle centered at origin, radius 5
    private readonly CircleCurve2D myCircle =
        new CircleCurve2D(new Circle2D(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 5.0));

    [Fact]
    public void ConstructFromLine()
    {
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myLine, 0.0, 10.0);
        Assert.Equal(0.0, aTrim.FirstParameter());
        Assert.Equal(10.0, aTrim.LastParameter());
    }

    [Fact]
    public void ConstructFromCircle()
    {
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myCircle, 0.0, M_PI);
        Assert.Equal(0.0, aTrim.FirstParameter());
        Assert.Equal(M_PI, aTrim.LastParameter());
    }

    [Fact]
    public void StartAndEndPoints()
    {
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myLine, 2.0, 8.0);
        Point2D aStart = aTrim.StartPoint();
        Point2D anEnd = aTrim.EndPoint();
        Assert.Equal(2.0, aStart.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(8.0, anEnd.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void BasisCurve()
    {
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myCircle, 0.0, M_PI);
        Curve2D aBasis = aTrim.BasisCurve();
        Assert.NotNull(aBasis);
    }

    [Fact]
    public void EvalD0_MatchesBasisCurve()
    {
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myCircle, 0.0, M_PI);
        for (double u = 0.0; u <= M_PI; u += M_PI / 4.0)
        {
            Point2D aTrimPnt = aTrim.EvalD0(u);
            Point2D aBasePnt = myCircle.EvalD0(u);
            Assert.True(aTrimPnt.IsEqual(aBasePnt, global::Kerf.Engine.Foundation.Precision.Confusion()));
        }
    }

    [Fact]
    public void EvalD1_MatchesBasisCurve()
    {
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myLine, 0.0, 10.0);
        Curve2D.ResD1 aT = aTrim.EvalD1(5.0);
        Curve2D.ResD1 aB = myLine.EvalD1(5.0);
        Assert.True(aT.Point.IsEqual(aB.Point, global::Kerf.Engine.Foundation.Precision.Confusion()));
        Assert.Equal(aB.D1.X(), aT.D1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aB.D1.Y(), aT.D1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Continuity()
    {
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myLine, 0.0, 10.0);
        Assert.Equal(CN, aTrim.Continuity());
    }

    [Fact]
    public void IsCN()
    {
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myLine, 0.0, 10.0);
        Assert.True(aTrim.IsCN(10));
    }

    [Fact]
    public void IsClosed_OpenArc()
    {
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myCircle, 0.0, M_PI);
        Assert.False(aTrim.IsClosed);
    }

    [Fact]
    public void IsClosed_FullCircle()
    {
        // Full-period trim on a periodic curve should be reported as closed
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myCircle, 0.0, 2.0 * M_PI);
        Assert.True(aTrim.IsClosed);
    }

    [Fact]
    public void IsPeriodic_PartialArc()
    {
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myCircle, 0.0, M_PI);
        Assert.False(aTrim.IsPeriodic);
    }

    [Fact]
    public void IsPeriodic_FullPeriod()
    {
        // Full-period trim on a periodic curve should be reported as periodic
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myCircle, 0.0, 2.0 * M_PI);
        Assert.True(aTrim.IsPeriodic);
    }

    [Fact]
    public void IsPeriodic_Line()
    {
        // Line is not periodic, so trimmed line should not be periodic
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myLine, 0.0, 10.0);
        Assert.False(aTrim.IsPeriodic);
    }

    [Fact]
    public void SetTrim()
    {
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myLine, 0.0, 10.0);
        aTrim.SetTrim(3.0, 7.0);
        Assert.Equal(3.0, aTrim.FirstParameter());
        Assert.Equal(7.0, aTrim.LastParameter());
    }

    [Fact]
    public void Copy()
    {
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myCircle, 0.0, M_PI);
        Geometry2D aCopyGeom = aTrim.Copy();
        TrimmedCurve2D? aCopy = aCopyGeom as TrimmedCurve2D;
        Assert.NotNull(aCopy);
        Assert.Equal(0.0, aCopy.FirstParameter());
        Assert.Equal(M_PI, aCopy.LastParameter());
    }

    [Fact]
    public void Transform_Translation()
    {
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myLine, 0.0, 10.0);
        Transform2D aTrsf = new Transform2D().WithTranslation(new Vector2D(5.0, 5.0));
        aTrim.Transform(aTrsf);
        Point2D aStart = aTrim.StartPoint();
        Assert.Equal(5.0, aStart.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aStart.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void CircleArc_StartEndPoints()
    {
        // Quarter circle arc
        TrimmedCurve2D aTrim = new TrimmedCurve2D(myCircle, 0.0, M_PI / 2.0);
        Point2D aStart = aTrim.StartPoint();
        Point2D anEnd = aTrim.EndPoint();
        Assert.Equal(5.0, aStart.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aStart.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, anEnd.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, anEnd.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
