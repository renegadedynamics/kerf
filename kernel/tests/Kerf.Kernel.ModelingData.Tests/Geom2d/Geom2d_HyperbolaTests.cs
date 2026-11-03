// Port of: src/ModelingData/TKG2d/GTests/Geom2d_Hyperbola_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom2d_HyperbolaTest
{
    // Hyperbola centered at origin, a=5, b=3
    private readonly HyperbolaCurve2D myHyperbola =
        new HyperbolaCurve2D(new Hyperbola2D(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 5.0, 3.0));

    [Fact]
    public void Radii()
    {
        Assert.Equal(5.0, myHyperbola.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, myHyperbola.MinorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void IsNotClosedNotPeriodic()
    {
        Assert.False(myHyperbola.IsClosed);
        Assert.False(myHyperbola.IsPeriodic);
    }

    [Fact]
    public void Eccentricity()
    {
        // e = c/a where c = sqrt(a^2 + b^2) = sqrt(25+9) = sqrt(34)
        double anExpected = Math.Sqrt(34.0) / 5.0;
        Assert.Equal(anExpected, myHyperbola.Eccentricity(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Focal()
    {
        // 2c = 2*sqrt(a^2 + b^2)
        double anExpected = 2.0 * Math.Sqrt(25.0 + 9.0);
        Assert.Equal(anExpected, myHyperbola.Focal(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Foci()
    {
        double c = Math.Sqrt(25.0 + 9.0);
        Point2D aF1 = myHyperbola.Focus1();
        Point2D aF2 = myHyperbola.Focus2();
        Assert.Equal(c, aF1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-c, aF2.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Parameter()
    {
        // Semi-latus rectum = b^2/a = 9/5 = 1.8
        Assert.Equal(1.8, myHyperbola.Parameter(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD0_AtZero()
    {
        // At U=0: vertex = (a, 0) = (5, 0) since x = a*cosh(0) = a, y = b*sinh(0) = 0
        Point2D aPnt = myHyperbola.EvalD0(0.0);
        Assert.Equal(5.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD1_AtZero()
    {
        Curve2D.ResD1 aR = myHyperbola.EvalD1(0.0);
        // At U=0: dx/du = a*sinh(0) = 0, dy/du = b*cosh(0) = b = 3
        Assert.Equal(0.0, aR.D1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aR.D1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void PointOnHyperbola_FociDistanceDifference()
    {
        // For any point on hyperbola: |d(P,F1) - d(P,F2)| = 2a
        Point2D aF1 = myHyperbola.Focus1();
        Point2D aF2 = myHyperbola.Focus2();
        for (double u = -2.0; u <= 2.0; u += 0.5)
        {
            Point2D aPnt = myHyperbola.EvalD0(u);
            double aDiff = Math.Abs(aPnt.Distance(aF1) - aPnt.Distance(aF2));
            Assert.Equal(2.0 * 5.0, aDiff, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    [Fact]
    public void SetMajorRadius()
    {
        myHyperbola.SetMajorRadius(8.0);
        Assert.Equal(8.0, myHyperbola.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SetMinorRadius()
    {
        myHyperbola.SetMinorRadius(4.0);
        Assert.Equal(4.0, myHyperbola.MinorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geometry2D aCopyGeom = myHyperbola.Copy();
        HyperbolaCurve2D? aCopy = aCopyGeom as HyperbolaCurve2D;
        Assert.NotNull(aCopy);
        Assert.Equal(5.0, aCopy.MajorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aCopy.MinorRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform_Translation()
    {
        Transform2D aTrsf = new Transform2D();
        aTrsf = aTrsf.WithTranslation(new Vector2D(10.0, 20.0));
        myHyperbola.Transform(aTrsf);
        Assert.Equal(10.0, myHyperbola.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(20.0, myHyperbola.Location().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Asymptotes()
    {
        Axis2D anAsym1 = myHyperbola.Asymptote1();
        Axis2D anAsym2 = myHyperbola.Asymptote2();
        // Asymptotes pass through center
        Assert.Equal(0.0, anAsym1.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, anAsym2.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
