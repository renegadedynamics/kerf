// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>GccEnt: the qualifiers' names and the qualified lines and circles.</summary>
public class GccEntTests
{
    private static readonly Direction2D DX = new(Direction2D.D.X);

    [Theory]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, "UNQUALIFIED")]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, "ENCLOSING")]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, "ENCLOSED")]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, "OUTSIDE")]
    [InlineData(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, "NOQUALIFIER")]
    public void PositionNames_RoundTrip(global::Kerf.Engine.Construction.Constraints.Qualifier thePosition, string theName)
    {
        Assert.Equal(theName, global::Kerf.Engine.Construction.Constraints.GccEnt.PositionToString(thePosition));
        Assert.Equal(thePosition, global::Kerf.Engine.Construction.Constraints.GccEnt.PositionFromString(theName.ToLowerInvariant()));
        global::Kerf.Engine.Construction.Constraints.Qualifier aPos = global::Kerf.Engine.Construction.Constraints.Qualifier.Outside;
        Assert.True(global::Kerf.Engine.Construction.Constraints.GccEnt.PositionFromString(theName, ref aPos));
        Assert.Equal(thePosition, aPos);
        StringWriter aWriter = new();
        global::Kerf.Engine.Construction.Constraints.GccEnt.Print(thePosition, aWriter);
        Assert.Equal(theName, aWriter.ToString());
    }

    [Fact]
    public void PositionFromString_Unknown()
    {
        global::Kerf.Engine.Construction.Constraints.Qualifier aPos = global::Kerf.Engine.Construction.Constraints.Qualifier.Outside;
        Assert.False(global::Kerf.Engine.Construction.Constraints.GccEnt.PositionFromString("inside", ref aPos));
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, aPos);
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, global::Kerf.Engine.Construction.Constraints.GccEnt.PositionFromString("inside"));
    }

    [Fact]
    public void QualifiedCirc()
    {
        Circle2D aC = new(new Axis2D(new Point2D(1, 2), DX), 3);
        Assert.True(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC).IsUnqualified);
        Assert.True(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(aC).IsEnclosing);
        Assert.True(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(aC).IsEnclosed);
        Assert.True(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(aC).IsOutside);
        Assert.False(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(aC).IsEnclosed);
        global::Kerf.Engine.Construction.Constraints.QualifiedCirc aQ = global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(aC);
        Assert.Equal(aC, aQ.Qualified());
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, aQ.Qualifier());
        Assert.Equal(new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(aC, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed), aQ);
        Assert.NotEqual(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(aC), aQ);
        Assert.Equal(aQ.GetHashCode(), new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(aC, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed).GetHashCode());
        Assert.Contains("ENCLOSED", aQ.ToString());
    }

    [Fact]
    public void QualifiedLin()
    {
        Line2D aL = new(new Point2D(1, 2), new Direction2D(1, 1));
        Assert.True(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aL).IsUnqualified);
        Assert.True(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(aL).IsEnclosed);
        Assert.True(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(aL).IsOutside);
        global::Kerf.Engine.Construction.Constraints.QualifiedLin aQ = global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(aL);
        Assert.Equal(aL, aQ.Qualified());
        Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, aQ.Qualifier());
        Assert.True(aQ == new global::Kerf.Engine.Construction.Constraints.QualifiedLin(aL, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside));
        Assert.True(aQ != global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(aL));
        Assert.Contains("OUTSIDE", aQ.ToString());
    }
}
