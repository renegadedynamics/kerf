// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingData.Tests;

public class TopAbsTests
{
    [Fact]
    public void Compose_Reverse_Complement()
    {
        Assert.Equal(Forward, global::Kerf.Engine.Topology.TopAbs.Compose(Forward, Forward));
        Assert.Equal(Reversed, global::Kerf.Engine.Topology.TopAbs.Compose(Forward, Reversed));
        Assert.Equal(Forward, global::Kerf.Engine.Topology.TopAbs.Compose(Reversed, Reversed));
        Assert.Equal(Internal, global::Kerf.Engine.Topology.TopAbs.Compose(Reversed, Internal));
        Assert.Equal(External, global::Kerf.Engine.Topology.TopAbs.Compose(External, Reversed));
        Assert.Equal(Internal, global::Kerf.Engine.Topology.TopAbs.Compose(Internal, Forward));

        Assert.Equal(Reversed, global::Kerf.Engine.Topology.TopAbs.Reverse(Forward));
        Assert.Equal(Internal, global::Kerf.Engine.Topology.TopAbs.Reverse(Internal));
        Assert.Equal(External, global::Kerf.Engine.Topology.TopAbs.Complement(Internal));
        Assert.Equal(Forward, global::Kerf.Engine.Topology.TopAbs.Complement(Reversed));
    }

    [Fact]
    public void Names_round_trip()
    {
        foreach (ShapeType aType in Enum.GetValues<ShapeType>())
        {
            Assert.Equal(aType, global::Kerf.Engine.Topology.TopAbs.ShapeTypeFromString(global::Kerf.Engine.Topology.TopAbs.ShapeTypeToString(aType).ToLowerInvariant()));
        }
        foreach (Orientation anOri in Enum.GetValues<Orientation>())
        {
            Assert.Equal(anOri, global::Kerf.Engine.Topology.TopAbs.ShapeOrientationFromString(global::Kerf.Engine.Topology.TopAbs.ShapeOrientationToString(anOri)));
        }
        Assert.Equal("COMPSOLID", global::Kerf.Engine.Topology.TopAbs.ShapeTypeToString(ShapeType.CompSolid));

        ShapeType aRes = ShapeType.Edge;
        Assert.False(global::Kerf.Engine.Topology.TopAbs.ShapeTypeFromString("nonsense", ref aRes));
        Assert.Equal(ShapeType.Edge, aRes);
        Assert.Equal(ShapeType.Shape, global::Kerf.Engine.Topology.TopAbs.ShapeTypeFromString("nonsense"));
        Assert.Equal(Forward, global::Kerf.Engine.Topology.TopAbs.ShapeOrientationFromString("nonsense"));
    }

    [Fact]
    public void Print_state_uses_OCCTs_mismatched_table()
    {
        // OCCT: the table is ON, IN, OUT, UNKNOWN while the enum is IN, OUT, ON, UNKNOWN (docs/occt-issues.md)
        string Print(Containment s)
        {
            StringWriter w = new();
            global::Kerf.Engine.Topology.TopAbs.Print(s, w);
            return w.ToString();
        }
        Assert.Equal("ON", Print(In));
        Assert.Equal("IN", Print(Out));
        Assert.Equal("OUT", Print(On));
        Assert.Equal("UNKNOWN", Print(Unknown));

        StringWriter aW = new();
        global::Kerf.Engine.Topology.TopAbs.Print(ShapeType.Face, aW);
        global::Kerf.Engine.Topology.TopAbs.Print(Internal, aW);
        Assert.Equal("FACEINTERNAL", aW.ToString());
    }
}
