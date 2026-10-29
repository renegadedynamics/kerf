// Port of: src/ModelingData/TKG2d/GTests/Geom2d_Ellipse_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Geom2d_EllipseTest
{
    // Ellipse centered at origin, major radius 10, minor radius 5
    private readonly Geom2d_Ellipse myEllipse =
        new Geom2d_Ellipse(new gp_Elips2d(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 10.0, 5.0));

    [Fact]
    public void Radii()
    {
        Assert.Equal(10.0, myEllipse.MajorRadius(), Precision.Confusion());
        Assert.Equal(5.0, myEllipse.MinorRadius(), Precision.Confusion());
    }

    [Fact]
    public void ParameterBounds()
    {
        Assert.Equal(0.0, myEllipse.FirstParameter());
        Assert.Equal(2.0 * M_PI, myEllipse.LastParameter());
    }

    [Fact]
    public void IsClosedAndPeriodic()
    {
        Assert.True(myEllipse.IsClosed());
        Assert.True(myEllipse.IsPeriodic());
    }

    [Fact]
    public void Eccentricity()
    {
        // e = sqrt(1 - (b/a)^2) = sqrt(1 - 0.25) = sqrt(0.75)
        double anExpected = Math.Sqrt(1.0 - (5.0 * 5.0) / (10.0 * 10.0));
        Assert.Equal(anExpected, myEllipse.Eccentricity(), Precision.Confusion());
    }

    [Fact]
    public void Focal()
    {
        // Focal distance = sqrt(a^2 - b^2) * 2
        double anExpected = 2.0 * Math.Sqrt(10.0 * 10.0 - 5.0 * 5.0);
        Assert.Equal(anExpected, myEllipse.Focal(), Precision.Confusion());
    }

    [Fact]
    public void Foci()
    {
        gp_Pnt2d aFocus1 = myEllipse.Focus1();
        gp_Pnt2d aFocus2 = myEllipse.Focus2();
        double c = Math.Sqrt(100.0 - 25.0); // sqrt(a^2 - b^2)
        Assert.Equal(c, aFocus1.X(), Precision.Confusion());
        Assert.Equal(0.0, aFocus1.Y(), Precision.Confusion());
        Assert.Equal(-c, aFocus2.X(), Precision.Confusion());
        Assert.Equal(0.0, aFocus2.Y(), Precision.Confusion());
    }

    [Fact]
    public void Parameter()
    {
        // Semi-latus rectum = b^2/a = 25/10 = 2.5
        Assert.Equal(2.5, myEllipse.Parameter(), Precision.Confusion());
    }

    [Fact]
    public void EvalD0_MajorVertex()
    {
        // At U=0: point on major axis = (a, 0) = (10, 0)
        gp_Pnt2d aPnt = myEllipse.EvalD0(0.0);
        Assert.Equal(10.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void EvalD0_MinorVertex()
    {
        // At U=PI/2: point on minor axis = (0, b) = (0, 5)
        gp_Pnt2d aPnt = myEllipse.EvalD0(M_PI / 2.0);
        Assert.Equal(0.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(5.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void EvalD1_AtZero()
    {
        Geom2d_Curve.ResD1 aR = myEllipse.EvalD1(0.0);
        // At U=0: tangent is (0, b) = (0, 5)
        Assert.Equal(0.0, aR.D1.X(), Precision.Confusion());
        Assert.Equal(5.0, aR.D1.Y(), Precision.Confusion());
    }

    [Fact]
    public void EvalD2_AtZero()
    {
        Geom2d_Curve.ResD2 aR = myEllipse.EvalD2(0.0);
        // At U=0: second derivative is (-a, 0) = (-10, 0)
        Assert.Equal(-10.0, aR.D2.X(), Precision.Confusion());
        Assert.Equal(0.0, aR.D2.Y(), Precision.Confusion());
    }

    [Fact]
    public void SetMajorRadius()
    {
        myEllipse.SetMajorRadius(20.0);
        Assert.Equal(20.0, myEllipse.MajorRadius(), Precision.Confusion());
    }

    [Fact]
    public void SetMinorRadius()
    {
        myEllipse.SetMinorRadius(3.0);
        Assert.Equal(3.0, myEllipse.MinorRadius(), Precision.Confusion());
    }

    [Fact]
    public void PointOnEllipse_FociDistanceSum()
    {
        // For any point on ellipse: d(P, F1) + d(P, F2) = 2a
        gp_Pnt2d aF1 = myEllipse.Focus1();
        gp_Pnt2d aF2 = myEllipse.Focus2();
        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 6.0)
        {
            gp_Pnt2d aPnt = myEllipse.EvalD0(u);
            double aSum = aPnt.Distance(aF1) + aPnt.Distance(aF2);
            Assert.Equal(2.0 * 10.0, aSum, Precision.Confusion());
        }
    }

    [Fact]
    public void Copy()
    {
        Geom2d_Geometry aCopyGeom = myEllipse.Copy();
        Geom2d_Ellipse? aCopy = aCopyGeom as Geom2d_Ellipse;
        Assert.NotNull(aCopy);
        Assert.Equal(10.0, aCopy.MajorRadius(), Precision.Confusion());
        Assert.Equal(5.0, aCopy.MinorRadius(), Precision.Confusion());
    }

    [Fact]
    public void Transform_Translation()
    {
        gp_Trsf2d aTrsf = new gp_Trsf2d();
        aTrsf.SetTranslation(new gp_Vec2d(5.0, 10.0));
        myEllipse.Transform(aTrsf);
        Assert.Equal(5.0, myEllipse.Location().X(), Precision.Confusion());
        Assert.Equal(10.0, myEllipse.Location().Y(), Precision.Confusion());
        Assert.Equal(10.0, myEllipse.MajorRadius(), Precision.Confusion());
    }

    [Fact]
    public void ReversedParameter()
    {
        double aU = M_PI / 4.0;
        Assert.Equal(2.0 * M_PI - aU, myEllipse.ReversedParameter(aU), Precision.Confusion());
    }
}
