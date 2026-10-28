// Port of: src/FoundationClasses/TKMath/GTests/Bnd_Box2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class Bnd_Box2dTest
{
    [Fact]
    public void DefaultConstructor()
    {
        Bnd_Box2d aBox = new();
        Assert.True(aBox.IsVoid());
        Assert.False(aBox.IsWhole());
        GTest.ExpectDoubleEq(aBox.GetGap(), 0.0);
    }

    [Fact]
    public void SetVoid()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(0.0, 0.0, 10.0, 10.0);
        Assert.False(aBox.IsVoid());
        aBox.SetVoid();
        Assert.True(aBox.IsVoid());
    }

    [Fact]
    public void SetWhole()
    {
        Bnd_Box2d aBox = new();
        aBox.SetWhole();
        Assert.True(aBox.IsWhole());
        Assert.False(aBox.IsVoid());
    }

    [Fact]
    public void Update_Bounds()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(1.0, 2.0, 10.0, 20.0);
        double aXmin = 0.0, aYmin = 0.0, aXmax = 0.0, aYmax = 0.0;
        aBox.Get(out aXmin, out aYmin, out aXmax, out aYmax);
        GTest.ExpectDoubleEq(aXmin, 1.0);
        GTest.ExpectDoubleEq(aYmin, 2.0);
        GTest.ExpectDoubleEq(aXmax, 10.0);
        GTest.ExpectDoubleEq(aYmax, 20.0);
    }

    [Fact]
    public void Update_SinglePoint()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(5.0, 7.0);
        double aXmin = 0.0, aYmin = 0.0, aXmax = 0.0, aYmax = 0.0;
        aBox.Get(out aXmin, out aYmin, out aXmax, out aYmax);
        GTest.ExpectDoubleEq(aXmin, 5.0);
        GTest.ExpectDoubleEq(aYmin, 7.0);
        GTest.ExpectDoubleEq(aXmax, 5.0);
        GTest.ExpectDoubleEq(aYmax, 7.0);
    }

    [Fact]
    public void Update_Expansion()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(1.0, 2.0, 10.0, 20.0);
        aBox.Update(0.0, 0.0, 15.0, 25.0);
        double aXmin = 0.0, aYmin = 0.0, aXmax = 0.0, aYmax = 0.0;
        aBox.Get(out aXmin, out aYmin, out aXmax, out aYmax);
        GTest.ExpectDoubleEq(aXmin, 0.0);
        GTest.ExpectDoubleEq(aYmin, 0.0);
        GTest.ExpectDoubleEq(aXmax, 15.0);
        GTest.ExpectDoubleEq(aYmax, 25.0);
    }

    [Fact]
    public void GapOperations()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(0.0, 0.0, 10.0, 10.0);
        GTest.ExpectDoubleEq(aBox.GetGap(), 0.0);
        aBox.SetGap(2.0);
        GTest.ExpectDoubleEq(aBox.GetGap(), 2.0);
        aBox.SetGap(-3.0);
        GTest.ExpectDoubleEq(aBox.GetGap(), 3.0);
    }

    [Fact]
    public void Enlarge()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(0.0, 0.0, 10.0, 10.0);
        aBox.Enlarge(5.0);
        GTest.ExpectDoubleEq(aBox.GetGap(), 5.0);
        aBox.Enlarge(3.0);
        GTest.ExpectDoubleEq(aBox.GetGap(), 5.0); // max(5, 3) = 5
        aBox.Enlarge(7.0);
        GTest.ExpectDoubleEq(aBox.GetGap(), 7.0); // max(5, 7) = 7
    }

    [Fact]
    public void Get_WithGap()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(1.0, 2.0, 10.0, 20.0);
        aBox.SetGap(0.5);
        double aXmin = 0.0, aYmin = 0.0, aXmax = 0.0, aYmax = 0.0;
        aBox.Get(out aXmin, out aYmin, out aXmax, out aYmax);
        GTest.ExpectDoubleEq(aXmin, 0.5);
        GTest.ExpectDoubleEq(aYmin, 1.5);
        GTest.ExpectDoubleEq(aXmax, 10.5);
        GTest.ExpectDoubleEq(aYmax, 20.5);
    }

    [Fact]
    public void Get_StructuredBindings()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(1.0, 2.0, 10.0, 20.0);
        var (aXmin, aXmax, aYmin, aYmax) = aBox.Get();
        GTest.ExpectDoubleEq(aXmin, 1.0);
        GTest.ExpectDoubleEq(aXmax, 10.0);
        GTest.ExpectDoubleEq(aYmin, 2.0);
        GTest.ExpectDoubleEq(aYmax, 20.0);
    }

    [Fact]
    public void Set_Point()
    {
        Bnd_Box2d aBox = new();
        aBox.Set(new gp_Pnt2d(5.0, 7.0));
        double aXmin = 0.0, aYmin = 0.0, aXmax = 0.0, aYmax = 0.0;
        aBox.Get(out aXmin, out aYmin, out aXmax, out aYmax);
        GTest.ExpectDoubleEq(aXmin, 5.0);
        GTest.ExpectDoubleEq(aYmin, 7.0);
        GTest.ExpectDoubleEq(aXmax, 5.0);
        GTest.ExpectDoubleEq(aYmax, 7.0);
    }

    [Fact]
    public void Add_Point()
    {
        Bnd_Box2d aBox = new();
        aBox.Add(new gp_Pnt2d(1.0, 2.0));
        aBox.Add(new gp_Pnt2d(10.0, 20.0));
        double aXmin = 0.0, aYmin = 0.0, aXmax = 0.0, aYmax = 0.0;
        aBox.Get(out aXmin, out aYmin, out aXmax, out aYmax);
        GTest.ExpectDoubleEq(aXmin, 1.0);
        GTest.ExpectDoubleEq(aYmin, 2.0);
        GTest.ExpectDoubleEq(aXmax, 10.0);
        GTest.ExpectDoubleEq(aYmax, 20.0);
    }

    [Fact]
    public void Add_Box()
    {
        Bnd_Box2d aBox1 = new();
        aBox1.Update(0.0, 0.0, 5.0, 5.0);
        Bnd_Box2d aBox2 = new();
        aBox2.Update(3.0, 3.0, 10.0, 10.0);
        aBox1.Add(aBox2);
        double aXmin = 0.0, aYmin = 0.0, aXmax = 0.0, aYmax = 0.0;
        aBox1.Get(out aXmin, out aYmin, out aXmax, out aYmax);
        GTest.ExpectDoubleEq(aXmin, 0.0);
        GTest.ExpectDoubleEq(aYmin, 0.0);
        GTest.ExpectDoubleEq(aXmax, 10.0);
        GTest.ExpectDoubleEq(aYmax, 10.0);
    }

    [Fact]
    public void Add_VoidBox()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(1.0, 2.0, 5.0, 6.0);
        Bnd_Box2d aVoidBox = new();
        aBox.Add(aVoidBox);
        double aXmin = 0.0, aYmin = 0.0, aXmax = 0.0, aYmax = 0.0;
        aBox.Get(out aXmin, out aYmin, out aXmax, out aYmax);
        GTest.ExpectDoubleEq(aXmin, 1.0);
        GTest.ExpectDoubleEq(aYmin, 2.0);
    }

    [Fact]
    public void Add_ToVoidBox()
    {
        Bnd_Box2d aVoidBox = new();
        Bnd_Box2d aBox = new();
        aBox.Update(1.0, 2.0, 5.0, 6.0);
        aVoidBox.Add(aBox);
        double aXmin = 0.0, aYmin = 0.0, aXmax = 0.0, aYmax = 0.0;
        aVoidBox.Get(out aXmin, out aYmin, out aXmax, out aYmax);
        GTest.ExpectDoubleEq(aXmin, 1.0);
        GTest.ExpectDoubleEq(aYmin, 2.0);
        GTest.ExpectDoubleEq(aXmax, 5.0);
        GTest.ExpectDoubleEq(aYmax, 6.0);
    }

    [Fact]
    public void OpenDirections()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(0.0, 0.0, 10.0, 10.0);
        Assert.False(aBox.IsOpenXmin());
        Assert.False(aBox.IsOpenXmax());
        Assert.False(aBox.IsOpenYmin());
        Assert.False(aBox.IsOpenYmax());

        aBox.OpenXmin();
        Assert.True(aBox.IsOpenXmin());
        aBox.OpenYmax();
        Assert.True(aBox.IsOpenYmax());
        Assert.False(aBox.IsWhole());
    }

    [Fact]
    public void IsOut_Point()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(0.0, 0.0, 10.0, 10.0);
        Assert.False(aBox.IsOut(new gp_Pnt2d(5.0, 5.0)));
        Assert.True(aBox.IsOut(new gp_Pnt2d(15.0, 5.0)));
        Assert.True(aBox.IsOut(new gp_Pnt2d(-1.0, 5.0)));
    }

    [Fact]
    public void IsOut_Box_Overlapping()
    {
        Bnd_Box2d aBox1 = new();
        aBox1.Update(0.0, 0.0, 10.0, 10.0);
        Bnd_Box2d aBox2 = new();
        aBox2.Update(5.0, 5.0, 15.0, 15.0);
        Assert.False(aBox1.IsOut(aBox2));
    }

    [Fact]
    public void IsOut_Box_Separated()
    {
        Bnd_Box2d aBox1 = new();
        aBox1.Update(0.0, 0.0, 5.0, 5.0);
        Bnd_Box2d aBox2 = new();
        aBox2.Update(10.0, 10.0, 15.0, 15.0);
        Assert.True(aBox1.IsOut(aBox2));
    }

    [Fact]
    public void IsOut_Box_FastPath()
    {
        // Both non-void, non-whole, non-open -> fast path
        Bnd_Box2d aBox1 = new();
        aBox1.Update(0.0, 0.0, 10.0, 10.0);
        Bnd_Box2d aBox2 = new();
        aBox2.Update(5.0, 5.0, 15.0, 15.0);
        Assert.False(aBox1.IsOut(aBox2));

        Bnd_Box2d aBox3 = new();
        aBox3.Update(20.0, 20.0, 30.0, 30.0);
        Assert.True(aBox1.IsOut(aBox3));
    }

    [Fact]
    public void IsOut_Box_FastPath_WithGap()
    {
        // Boxes separated by 1.0, but gap bridges it
        Bnd_Box2d aBox1 = new();
        aBox1.Update(0.0, 0.0, 5.0, 5.0);
        aBox1.SetGap(1.0);
        Bnd_Box2d aBox2 = new();
        aBox2.Update(6.0, 0.0, 10.0, 5.0);
        // Gap from box1 = 1.0, gap from box2 = 0.0, total = 1.0
        // Xmin1 - Xmax2 = 0 - 10 = -10 (not > 1.0) -> not out in X
        // Xmin2 - Xmax1 = 6 - 5 = 1 (not > 1.0, equal) -> not out
        Assert.False(aBox1.IsOut(aBox2));

        // Now increase separation
        Bnd_Box2d aBox3 = new();
        aBox3.Update(7.0, 0.0, 10.0, 5.0);
        // Xmin3 - Xmax1 = 7 - 5 = 2 > 1.0 -> out
        Assert.True(aBox1.IsOut(aBox3));
    }

    [Fact]
    public void IsOut_VoidBox()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(0.0, 0.0, 10.0, 10.0);
        Bnd_Box2d aVoid = new();
        Assert.True(aBox.IsOut(aVoid));
        Assert.True(aVoid.IsOut(aBox));
    }

    [Fact]
    public void IsOut_WholeBox()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(0.0, 0.0, 10.0, 10.0);
        Bnd_Box2d aWhole = new();
        aWhole.SetWhole();
        Assert.False(aBox.IsOut(aWhole));
        Assert.False(aWhole.IsOut(aBox));
    }

    [Fact]
    public void IsOut_OpenBox()
    {
        Bnd_Box2d aBox1 = new();
        aBox1.Update(0.0, 0.0, 10.0, 10.0);
        aBox1.OpenXmin(); // extends to -infinity in X

        Bnd_Box2d aBox2 = new();
        aBox2.Update(-100.0, 0.0, -50.0, 10.0);

        // aBox1 is open in Xmin, so extends to -infinity: should overlap
        Assert.False(aBox1.IsOut(aBox2));
    }

    [Fact]
    public void IsOut_Line()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(0.0, 0.0, 10.0, 10.0);

        // Line that intersects
        gp_Lin2d aLinInside = new(new gp_Pnt2d(5.0, -5.0), new gp_Dir2d(0.0, 1.0));
        Assert.False(aBox.IsOut(aLinInside));

        // Line that doesn't intersect
        gp_Lin2d aLinOutside = new(new gp_Pnt2d(20.0, -5.0), new gp_Dir2d(0.0, 1.0));
        Assert.True(aBox.IsOut(aLinOutside));
    }

    [Fact]
    public void IsOut_Segment()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(0.0, 0.0, 10.0, 10.0);

        // Segment that crosses
        Assert.False(aBox.IsOut(new gp_Pnt2d(5.0, -5.0), new gp_Pnt2d(5.0, 15.0)));

        // Segment fully outside
        Assert.True(aBox.IsOut(new gp_Pnt2d(20.0, 0.0), new gp_Pnt2d(20.0, 10.0)));
    }

    [Fact]
    public void Transformed()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(0.0, 0.0, 10.0, 10.0);

        gp_Trsf2d aTrsf = new();
        aTrsf.SetTranslation(new gp_Pnt2d(0.0, 0.0), new gp_Pnt2d(5.0, 5.0));
        Bnd_Box2d aTransformed = aBox.Transformed(aTrsf);

        double aXmin = 0.0, aYmin = 0.0, aXmax = 0.0, aYmax = 0.0;
        aTransformed.Get(out aXmin, out aYmin, out aXmax, out aYmax);
        GTest.ExpectDoubleEq(aXmin, 5.0);
        GTest.ExpectDoubleEq(aYmin, 5.0);
        GTest.ExpectDoubleEq(aXmax, 15.0);
        GTest.ExpectDoubleEq(aYmax, 15.0);
    }

    [Fact]
    public void SquareExtent()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(0.0, 0.0, 3.0, 4.0);
        // Diagonal^2 = 3^2 + 4^2 = 25
        GTest.ExpectDoubleEq(aBox.SquareExtent(), 25.0);
    }

    [Fact]
    public void SquareExtent_Void()
    {
        Bnd_Box2d aBox = new();
        GTest.ExpectDoubleEq(aBox.SquareExtent(), 0.0);
    }

    [Fact]
    public void Add_Direction()
    {
        Bnd_Box2d aBox = new();
        aBox.Set(new gp_Pnt2d(5.0, 5.0));
        aBox.Add(new gp_Dir2d(1.0, 0.0));
        Assert.True(aBox.IsOpenXmax());
        Assert.False(aBox.IsOpenXmin());
    }

    [Fact]
    public void Contains_Point()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(0.0, 0.0, 10.0, 10.0);
        Assert.True(aBox.Contains(new gp_Pnt2d(5.0, 5.0)));
        Assert.True(aBox.Contains(new gp_Pnt2d(0.0, 0.0)));
        Assert.False(aBox.Contains(new gp_Pnt2d(-1.0, 5.0)));
        Assert.False(aBox.Contains(new gp_Pnt2d(5.0, 11.0)));
    }

    [Fact]
    public void Intersects_Box()
    {
        Bnd_Box2d aBox1 = new();
        aBox1.Update(0.0, 0.0, 10.0, 10.0);
        Bnd_Box2d aBox2 = new();
        aBox2.Update(5.0, 5.0, 15.0, 15.0);
        Bnd_Box2d aBox3 = new();
        aBox3.Update(20.0, 20.0, 30.0, 30.0);
        Assert.True(aBox1.Intersects(aBox2));
        Assert.False(aBox1.Intersects(aBox3));
    }

    [Fact]
    public void Distance_Separated()
    {
        Bnd_Box2d aBox1 = new();
        aBox1.Update(0.0, 0.0, 1.0, 1.0);
        Bnd_Box2d aBox2 = new();
        aBox2.Update(4.0, 0.0, 5.0, 1.0);
        Assert.Equal(3.0, aBox1.Distance(aBox2), 1e-10);
    }

    [Fact]
    public void Distance_Overlapping()
    {
        Bnd_Box2d aBox1 = new();
        aBox1.Update(0.0, 0.0, 10.0, 10.0);
        Bnd_Box2d aBox2 = new();
        aBox2.Update(5.0, 5.0, 15.0, 15.0);
        GTest.ExpectDoubleEq(aBox1.Distance(aBox2), 0.0);
    }

    [Fact]
    public void Distance_Diagonal()
    {
        Bnd_Box2d aBox1 = new();
        aBox1.Update(0.0, 0.0, 1.0, 1.0);
        Bnd_Box2d aBox2 = new();
        aBox2.Update(4.0, 4.0, 5.0, 5.0);
        Assert.Equal(Math.Sqrt(18.0), aBox1.Distance(aBox2), 1e-10);
    }

    [Fact]
    public void Distance_Void()
    {
        Bnd_Box2d aBox1 = new();
        Bnd_Box2d aBox2 = new();
        aBox2.Update(0.0, 0.0, 1.0, 1.0);
        GTest.ExpectDoubleEq(aBox1.Distance(aBox2), 0.0);
    }

    [Fact]
    public void Center()
    {
        Bnd_Box2d aBox = new();
        aBox.Update(0.0, 0.0, 10.0, 20.0);
        var aCenter = aBox.Center();
        Assert.True(aCenter.HasValue);
        GTest.ExpectDoubleEq(aCenter.Value.X(), 5.0);
        GTest.ExpectDoubleEq(aCenter.Value.Y(), 10.0);
    }

    [Fact]
    public void Center_VoidNullopt()
    {
        Bnd_Box2d aVoid = new();
        Assert.False(aVoid.Center().HasValue);
    }
}
