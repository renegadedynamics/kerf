// Port of: src/ModelingData/TKG2d/GTests/Adaptor2d_Line2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace OcctNet.ModelingData.Tests;

public class Adaptor2d_Line2dTest
{
    private readonly Adaptor2d_Line2d myLine;
    private readonly Adaptor2d_Line2d myDiagLine;

    public Adaptor2d_Line2dTest()
    {
        // Line through origin along direction (1, 0) with bounds [0, 10]
        myLine = new Adaptor2d_Line2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0), 0.0, 10.0);

        // Line through (1, 2) along direction (3, 4)/5 with bounds [-5, 5]
        myDiagLine =
          new Adaptor2d_Line2d(new gp_Pnt2d(1.0, 2.0), new gp_Dir2d(3.0 / 5.0, 4.0 / 5.0), -5.0, 5.0);
    }

    [Fact]
    public void DefaultConstructor()
    {
        Adaptor2d_Line2d aLine = new Adaptor2d_Line2d();
        Assert.Equal(GeomAbs_Line, aLine.GetType());
    }

    [Fact]
    public void ParameterBounds()
    {
        Assert.Equal(0.0, myLine.FirstParameter());
        Assert.Equal(10.0, myLine.LastParameter());
        Assert.Equal(-5.0, myDiagLine.FirstParameter());
        Assert.Equal(5.0, myDiagLine.LastParameter());
    }

    [Fact]
    public void Continuity()
    {
        Assert.Equal(GeomAbs_CN, myLine.Continuity());
    }

    [Fact]
    public void NbIntervals()
    {
        Assert.Equal(1, myLine.NbIntervals(GeomAbs_C0));
        Assert.Equal(1, myLine.NbIntervals(GeomAbs_CN));
    }

    [Fact]
    public void Intervals()
    {
        NCollection_Array1<double> anIntervals = new(1, 2);
        myLine.Intervals(anIntervals, GeomAbs_CN);
        Assert.Equal(0.0, anIntervals[1]);
        Assert.Equal(10.0, anIntervals[2]);
    }

    [Fact]
    public void IsNotClosed()
    {
        Assert.False(myLine.IsClosed());
    }

    [Fact]
    public void IsNotPeriodic()
    {
        Assert.False(myLine.IsPeriodic());
    }

    [Fact]
    public void GetType_() // GetType in OCCT; renamed, as it would hide object.GetType
    {
        Assert.Equal(GeomAbs_Line, myLine.GetType());
    }

    [Fact]
    public void Value_AtOrigin()
    {
        gp_Pnt2d aPnt = myLine.Value(0.0);
        Assert.Equal(0.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void Value_AtParameter()
    {
        gp_Pnt2d aPnt = myLine.Value(5.0);
        Assert.Equal(5.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void Value_DiagonalLine()
    {
        // At U=0, point should be origin (1, 2)
        gp_Pnt2d aPnt0 = myDiagLine.Value(0.0);
        Assert.Equal(1.0, aPnt0.X(), Precision.Confusion());
        Assert.Equal(2.0, aPnt0.Y(), Precision.Confusion());

        // At U=5, point should be (1 + 3, 2 + 4) = (4, 6)
        gp_Pnt2d aPnt5 = myDiagLine.Value(5.0);
        Assert.Equal(4.0, aPnt5.X(), Precision.Confusion());
        Assert.Equal(6.0, aPnt5.Y(), Precision.Confusion());
    }

    [Fact]
    public void D0()
    {
        gp_Pnt2d aPnt = new();
        myLine.D0(3.0, out aPnt);
        Assert.Equal(3.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void D1_FirstDerivativeIsDirection()
    {
        gp_Pnt2d aPnt = new();
        gp_Vec2d aVec = new();
        myLine.D1(5.0, out aPnt, out aVec);

        Assert.Equal(5.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(1.0, aVec.X(), Precision.Confusion());
        Assert.Equal(0.0, aVec.Y(), Precision.Confusion());
    }

    [Fact]
    public void D2_SecondDerivativeIsZero()
    {
        gp_Pnt2d aPnt = new();
        gp_Vec2d aV1, aV2;
        myLine.D2(5.0, out aPnt, out aV1, out aV2);

        Assert.Equal(0.0, aV2.Magnitude(), Precision.Confusion());
    }

    [Fact]
    public void D3_ThirdDerivativeIsZero()
    {
        gp_Pnt2d aPnt = new();
        gp_Vec2d aV1, aV2, aV3;
        myLine.D3(5.0, out aPnt, out aV1, out aV2, out aV3);

        Assert.Equal(0.0, aV3.Magnitude(), Precision.Confusion());
    }

    [Fact]
    public void DN_FirstOrderMatchesD1()
    {
        gp_Vec2d aDN1 = myLine.DN(5.0, 1);
        Assert.Equal(1.0, aDN1.X(), Precision.Confusion());
        Assert.Equal(0.0, aDN1.Y(), Precision.Confusion());
    }

    [Fact]
    public void DN_HigherOrderIsZero()
    {
        gp_Vec2d aDN2 = myLine.DN(5.0, 2);
        Assert.Equal(0.0, aDN2.Magnitude(), Precision.Confusion());

        gp_Vec2d aDN3 = myLine.DN(5.0, 3);
        Assert.Equal(0.0, aDN3.Magnitude(), Precision.Confusion());
    }

    [Fact]
    public void Resolution()
    {
        // For a unit-speed line, resolution should equal the input tolerance
        double aResolution = myLine.Resolution(0.001);
        Assert.Equal(0.001, aResolution, Precision.Confusion());
    }

    [Fact]
    public void LineGeometry()
    {
        gp_Lin2d aLin = myLine.Line();
        Assert.Equal(1.0, aLin.Direction().X(), Precision.Confusion());
        Assert.Equal(0.0, aLin.Direction().Y(), Precision.Confusion());
        Assert.Equal(0.0, aLin.Location().X(), Precision.Confusion());
        Assert.Equal(0.0, aLin.Location().Y(), Precision.Confusion());
    }

    [Fact]
    public void Load_WithLine()
    {
        Adaptor2d_Line2d aLine = new Adaptor2d_Line2d();
        gp_Lin2d         aLin = new(new gp_Pnt2d(1.0, 1.0), new gp_Dir2d(0.0, 1.0));
        aLine.Load(aLin);

        gp_Pnt2d aPnt = aLine.Value(3.0);
        Assert.Equal(1.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(4.0, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void Load_WithBounds()
    {
        Adaptor2d_Line2d aLine = new Adaptor2d_Line2d();
        gp_Lin2d         aLin = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0));
        aLine.Load(aLin, 2.0, 8.0);

        Assert.Equal(2.0, aLine.FirstParameter());
        Assert.Equal(8.0, aLine.LastParameter());
    }

    [Fact]
    public void Trim()
    {
        Adaptor2d_Curve2d aTrimmed = myLine.Trim(2.0, 7.0, Precision.Confusion());
        Assert.Equal(2.0, aTrimmed.FirstParameter());
        Assert.Equal(7.0, aTrimmed.LastParameter());

        gp_Pnt2d aPnt = aTrimmed.Value(2.0);
        Assert.Equal(2.0, aPnt.X(), Precision.Confusion());
    }

    [Fact]
    public void ShallowCopy()
    {
        Adaptor2d_Curve2d aCopy = myLine.ShallowCopy();
        Assert.Equal(0.0, aCopy.FirstParameter());
        Assert.Equal(10.0, aCopy.LastParameter());
        Assert.Equal(GeomAbs_Line, aCopy.GetType());

        gp_Pnt2d aPnt = aCopy.Value(5.0);
        Assert.Equal(5.0, aPnt.X(), Precision.Confusion());
    }
}
