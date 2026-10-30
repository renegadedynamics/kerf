// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>GccEnt: the qualifiers' names and the qualified lines and circles.</summary>
public class GccEntTests
{
    private static readonly gp_Dir2d DX = new(gp_Dir2d.D.X);

    [Theory]
    [InlineData(GccEnt_unqualified, "UNQUALIFIED")]
    [InlineData(GccEnt_enclosing, "ENCLOSING")]
    [InlineData(GccEnt_enclosed, "ENCLOSED")]
    [InlineData(GccEnt_outside, "OUTSIDE")]
    [InlineData(GccEnt_noqualifier, "NOQUALIFIER")]
    public void PositionNames_RoundTrip(GccEnt_Position thePosition, string theName)
    {
        Assert.Equal(theName, GccEnt.PositionToString(thePosition));
        Assert.Equal(thePosition, GccEnt.PositionFromString(theName.ToLowerInvariant()));
        GccEnt_Position aPos = GccEnt_outside;
        Assert.True(GccEnt.PositionFromString(theName, ref aPos));
        Assert.Equal(thePosition, aPos);
        StringWriter aWriter = new();
        GccEnt.Print(thePosition, aWriter);
        Assert.Equal(theName, aWriter.ToString());
    }

    [Fact]
    public void PositionFromString_Unknown()
    {
        GccEnt_Position aPos = GccEnt_outside;
        Assert.False(GccEnt.PositionFromString("inside", ref aPos));
        Assert.Equal(GccEnt_outside, aPos);
        Assert.Equal(GccEnt_unqualified, GccEnt.PositionFromString("inside"));
    }

    [Fact]
    public void QualifiedCirc()
    {
        gp_Circ2d aC = new(new gp_Ax2d(new gp_Pnt2d(1, 2), DX), 3);
        Assert.True(GccEnt.Unqualified(aC).IsUnqualified());
        Assert.True(GccEnt.Enclosing(aC).IsEnclosing());
        Assert.True(GccEnt.Enclosed(aC).IsEnclosed());
        Assert.True(GccEnt.Outside(aC).IsOutside());
        Assert.False(GccEnt.Outside(aC).IsEnclosed());
        GccEnt_QualifiedCirc aQ = GccEnt.Enclosed(aC);
        Assert.Equal(aC, aQ.Qualified());
        Assert.Equal(GccEnt_enclosed, aQ.Qualifier());
        Assert.Equal(new GccEnt_QualifiedCirc(aC, GccEnt_enclosed), aQ);
        Assert.NotEqual(GccEnt.Outside(aC), aQ);
        Assert.Equal(aQ.GetHashCode(), new GccEnt_QualifiedCirc(aC, GccEnt_enclosed).GetHashCode());
        Assert.Contains("ENCLOSED", aQ.ToString());
    }

    [Fact]
    public void QualifiedLin()
    {
        gp_Lin2d aL = new(new gp_Pnt2d(1, 2), new gp_Dir2d(1, 1));
        Assert.True(GccEnt.Unqualified(aL).IsUnqualified());
        Assert.True(GccEnt.Enclosed(aL).IsEnclosed());
        Assert.True(GccEnt.Outside(aL).IsOutside());
        GccEnt_QualifiedLin aQ = GccEnt.Outside(aL);
        Assert.Equal(aL, aQ.Qualified());
        Assert.Equal(GccEnt_outside, aQ.Qualifier());
        Assert.True(aQ == new GccEnt_QualifiedLin(aL, GccEnt_outside));
        Assert.True(aQ != GccEnt.Enclosed(aL));
        Assert.Contains("OUTSIDE", aQ.ToString());
    }
}
