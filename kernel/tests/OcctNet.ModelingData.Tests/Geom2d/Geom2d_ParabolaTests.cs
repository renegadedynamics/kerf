// Port of: src/ModelingData/TKG2d/GTests/Geom2d_Parabola_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

public class Geom2d_ParabolaTest
{
    // Parabola centered at origin with focal distance 4
    private readonly ParabolaCurve2D myParabola =
        new ParabolaCurve2D(new Parabola2D(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 4.0));

    [Fact]
    public void Focal()
    {
        Assert.Equal(4.0, myParabola.Focal(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void IsNotClosedNotPeriodic()
    {
        Assert.False(myParabola.IsClosed);
        Assert.False(myParabola.IsPeriodic);
    }

    [Fact]
    public void Eccentricity()
    {
        Assert.Equal(1.0, myParabola.Eccentricity(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Focus()
    {
        Point2D aFocus = myParabola.Focus();
        Assert.Equal(4.0, aFocus.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aFocus.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Parameter()
    {
        // p = 2*Focal = 8
        Assert.Equal(8.0, myParabola.Parameter(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD0_AtZero()
    {
        // At U=0: vertex
        Point2D aPnt = myParabola.EvalD0(0.0);
        Assert.Equal(0.0, aPnt.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD0_Symmetric()
    {
        // Parabola is symmetric: P(U) and P(-U) should have same X, opposite Y
        double aU = 3.0;
        Point2D aPntPos = myParabola.EvalD0(aU);
        Point2D aPntNeg = myParabola.EvalD0(-aU);
        Assert.Equal(aPntNeg.X(), aPntPos.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-aPntNeg.Y(), aPntPos.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD1_AtZero()
    {
        Curve2D.ResD1 aR = myParabola.EvalD1(0.0);
        // At vertex: tangent is along Y axis
        Assert.Equal(0.0, aR.D1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aR.D1.Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD2_AtZero()
    {
        Curve2D.ResD2 aR = myParabola.EvalD2(0.0);
        // Second derivative at vertex should be along X (opening direction)
        Assert.True(aR.D2.X() > 0.0);
    }

    [Fact]
    public void PointOnParabola_FocusDirectrixProperty()
    {
        // For any point P on parabola: distance(P, Focus) = distance(P, Directrix)
        Point2D aFocus = myParabola.Focus();
        Axis2D aDirectrix = myParabola.Directrix();
        Line2D aDirLine = new Line2D(aDirectrix);
        for (double u = -5.0; u <= 5.0; u += 1.0)
        {
            Point2D aPnt = myParabola.EvalD0(u);
            double aDistFocus = aPnt.Distance(aFocus);
            double aDistDir = aDirLine.Distance(aPnt);
            Assert.Equal(aDistDir, aDistFocus, 1e-6);
        }
    }

    [Fact]
    public void SetFocal()
    {
        myParabola.SetFocal(10.0);
        Assert.Equal(10.0, myParabola.Focal(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geometry2D aCopyGeom = myParabola.Copy();
        ParabolaCurve2D? aCopy = aCopyGeom as ParabolaCurve2D;
        Assert.NotNull(aCopy);
        Assert.Equal(4.0, aCopy.Focal(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform_Translation()
    {
        Transform2D aTrsf = new Transform2D();
        aTrsf.SetTranslation(new Vector2D(5.0, 10.0));
        myParabola.Transform(aTrsf);
        Assert.Equal(5.0, myParabola.Location().X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(10.0, myParabola.Location().Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
