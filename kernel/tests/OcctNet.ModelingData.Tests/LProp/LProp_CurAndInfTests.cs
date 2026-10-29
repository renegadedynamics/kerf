// Port of: src/ModelingData/TKGeomBase/GTests/LProp_CurAndInf_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.LProp_CIType;

namespace OcctNet.ModelingData.Tests;

public class LProp_CurAndInfTest
{
    [Fact]
    public void DefaultConstructor_IsEmpty()
    {
        var aResult = new LProp_CurAndInf();
        Assert.True(aResult.IsEmpty());
        Assert.Equal(0, aResult.NbPoints());
    }

    [Fact]
    public void AddInflection()
    {
        var aResult = new LProp_CurAndInf();
        aResult.AddInflection(1.5);

        Assert.False(aResult.IsEmpty());
        Assert.Equal(1, aResult.NbPoints());
        Assert.Equal(1.5, aResult.Parameter(1));
        Assert.Equal(LProp_Inflection, aResult.Type(1));
    }

    [Fact]
    public void AddExtCur_Minimum()
    {
        var aResult = new LProp_CurAndInf();
        aResult.AddExtCur(2.0, true);

        Assert.Equal(1, aResult.NbPoints());
        Assert.Equal(2.0, aResult.Parameter(1));
        Assert.Equal(LProp_MinCur, aResult.Type(1));
    }

    [Fact]
    public void AddExtCur_Maximum()
    {
        var aResult = new LProp_CurAndInf();
        aResult.AddExtCur(3.0, false);

        Assert.Equal(1, aResult.NbPoints());
        Assert.Equal(3.0, aResult.Parameter(1));
        Assert.Equal(LProp_MaxCur, aResult.Type(1));
    }

    [Fact]
    public void MultiplePoints_SortedByParameter()
    {
        var aResult = new LProp_CurAndInf();
        // Add in non-sorted order
        aResult.AddInflection(5.0);
        aResult.AddExtCur(1.0, true);
        aResult.AddExtCur(3.0, false);
        aResult.AddInflection(7.0);

        Assert.Equal(4, aResult.NbPoints());

        // Verify sorted order
        for (int i = 1; i < aResult.NbPoints(); ++i)
        {
            Assert.True(aResult.Parameter(i) <= aResult.Parameter(i + 1));
        }
    }

    [Fact]
    public void MultiplePoints_CorrectTypes()
    {
        var aResult = new LProp_CurAndInf();
        aResult.AddExtCur(1.0, true);  // MinCur
        aResult.AddInflection(3.0);    // Inflection
        aResult.AddExtCur(5.0, false); // MaxCur

        Assert.Equal(3, aResult.NbPoints());
        Assert.Equal(LProp_MinCur, aResult.Type(1));
        Assert.Equal(LProp_Inflection, aResult.Type(2));
        Assert.Equal(LProp_MaxCur, aResult.Type(3));
    }

    [Fact]
    public void Clear()
    {
        var aResult = new LProp_CurAndInf();
        aResult.AddInflection(1.0);
        aResult.AddExtCur(2.0, true);

        Assert.Equal(2, aResult.NbPoints());

        aResult.Clear();
        Assert.True(aResult.IsEmpty());
        Assert.Equal(0, aResult.NbPoints());
    }

    [Fact]
    public void Clear_ThenRefill()
    {
        var aResult = new LProp_CurAndInf();
        aResult.AddInflection(1.0);
        aResult.Clear();

        aResult.AddExtCur(5.0, false);
        Assert.Equal(1, aResult.NbPoints());
        Assert.Equal(5.0, aResult.Parameter(1));
        Assert.Equal(LProp_MaxCur, aResult.Type(1));
    }

    [Fact]
    public void Parameter_OutOfRange_Throws()
    {
        var aResult = new LProp_CurAndInf();
        aResult.AddInflection(1.0);

        Assert.ThrowsAny<Standard_OutOfRange>(() => aResult.Parameter(0));
        Assert.ThrowsAny<Standard_OutOfRange>(() => aResult.Parameter(2));
    }

    [Fact]
    public void Type_OutOfRange_Throws()
    {
        var aResult = new LProp_CurAndInf();
        aResult.AddInflection(1.0);

        Assert.ThrowsAny<Standard_OutOfRange>(() => aResult.Type(0));
        Assert.ThrowsAny<Standard_OutOfRange>(() => aResult.Type(2));
    }
}
