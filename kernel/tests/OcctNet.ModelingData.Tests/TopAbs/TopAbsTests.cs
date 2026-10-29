// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingData.Tests;

public class TopAbsTests
{
    [Fact]
    public void Compose_Reverse_Complement()
    {
        Assert.Equal(TopAbs_FORWARD, TopAbs.Compose(TopAbs_FORWARD, TopAbs_FORWARD));
        Assert.Equal(TopAbs_REVERSED, TopAbs.Compose(TopAbs_FORWARD, TopAbs_REVERSED));
        Assert.Equal(TopAbs_FORWARD, TopAbs.Compose(TopAbs_REVERSED, TopAbs_REVERSED));
        Assert.Equal(TopAbs_INTERNAL, TopAbs.Compose(TopAbs_REVERSED, TopAbs_INTERNAL));
        Assert.Equal(TopAbs_EXTERNAL, TopAbs.Compose(TopAbs_EXTERNAL, TopAbs_REVERSED));
        Assert.Equal(TopAbs_INTERNAL, TopAbs.Compose(TopAbs_INTERNAL, TopAbs_FORWARD));

        Assert.Equal(TopAbs_REVERSED, TopAbs.Reverse(TopAbs_FORWARD));
        Assert.Equal(TopAbs_INTERNAL, TopAbs.Reverse(TopAbs_INTERNAL));
        Assert.Equal(TopAbs_EXTERNAL, TopAbs.Complement(TopAbs_INTERNAL));
        Assert.Equal(TopAbs_FORWARD, TopAbs.Complement(TopAbs_REVERSED));
    }

    [Fact]
    public void Names_round_trip()
    {
        foreach (TopAbs_ShapeEnum aType in Enum.GetValues<TopAbs_ShapeEnum>())
        {
            Assert.Equal(aType, TopAbs.ShapeTypeFromString(TopAbs.ShapeTypeToString(aType).ToLowerInvariant()));
        }
        foreach (TopAbs_Orientation anOri in Enum.GetValues<TopAbs_Orientation>())
        {
            Assert.Equal(anOri, TopAbs.ShapeOrientationFromString(TopAbs.ShapeOrientationToString(anOri)));
        }
        Assert.Equal("COMPSOLID", TopAbs.ShapeTypeToString(TopAbs_COMPSOLID));

        TopAbs_ShapeEnum aRes = TopAbs_EDGE;
        Assert.False(TopAbs.ShapeTypeFromString("nonsense", ref aRes));
        Assert.Equal(TopAbs_EDGE, aRes);
        Assert.Equal(TopAbs_SHAPE, TopAbs.ShapeTypeFromString("nonsense"));
        Assert.Equal(TopAbs_FORWARD, TopAbs.ShapeOrientationFromString("nonsense"));
    }

    [Fact]
    public void Print_state_uses_OCCTs_mismatched_table()
    {
        // OCCT: the table is ON, IN, OUT, UNKNOWN while the enum is IN, OUT, ON, UNKNOWN (docs/occt-issues.md)
        string Print(TopAbs_State s)
        {
            StringWriter w = new();
            TopAbs.Print(s, w);
            return w.ToString();
        }
        Assert.Equal("ON", Print(TopAbs_IN));
        Assert.Equal("IN", Print(TopAbs_OUT));
        Assert.Equal("OUT", Print(TopAbs_ON));
        Assert.Equal("UNKNOWN", Print(TopAbs_UNKNOWN));

        StringWriter aW = new();
        TopAbs.Print(TopAbs_FACE, aW);
        TopAbs.Print(TopAbs_INTERNAL, aW);
        Assert.Equal("FACEINTERNAL", aW.ToString());
    }
}
