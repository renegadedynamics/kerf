// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// SHMessage (unit 7A): ShapeExtend.Init loads Shape Healing's compiled-in messages (OCCT's SHAPE.us) through
/// Message_MsgFile.LoadFromString, OCCT's parser of message files (decision 975).
/// </summary>
public class SHMessageTests
{
    [Fact]
    public void Init_LoadsShapeHealingsMessages()
    {
        ShapeExtend.Init();
        ShapeExtend.Init(); // once
        Assert.True(MsgFile.HasMsg("ShapeFix.FixSmallSolid.MSG0"));
        // As OCCT's, Init loads only when its probe keyword is missing, and Msg stores an unknown keyword's error text
        // under that keyword: a test that ran ShapeFix_FixSmallSolid before Init leaves the table without the texts
        // for the rest of the process. Load them again, as Init would have.
        MsgFile.LoadFromString(SHMessage_SHAPE_us.Text);
        Assert.Equal("Face converted to BSpline", MsgFile.Msg("BSplineRestriction.NewSurface.MSG0"));
        Assert.Equal("Info: Shape Processing: Operator %d/%d: %s", MsgFile.Msg("SP.Sequence.Info.Operator"));
        Assert.Equal("Edge %d was small, removed", new Msg("FixAdvWire.FixSmall.MSG5").Original());
        // keywords without a message (followed by a comment and another keyword) have none
        Assert.False(MsgFile.HasMsg("Name.Shape.MSG0"));
    }

    [Fact]
    public void LoadFromString_ParsesKeywordsMessagesCommentsAndIndentation()
    {
        string aText = "! a comment\r\n"
                       + "  .SHMessageTests.One   \r\n"
                       + "First line\r\n"
                       + "    indented by four\r\n"
                       + "\tby a tab\r\n"
                       + "! a comment inside\r\n"
                       + "last\r\n"
                       + ".SHMessageTests.Empty\n"
                       + ".SHMessageTests.Two\n"
                       + "  starts indented\n"
                       + "  same level";
        Assert.True(MsgFile.LoadFromString(aText));
        Assert.Equal("First line\n    indented by four\n        by a tab\nlast", MsgFile.Msg("SHMessageTests.One"));
        Assert.False(MsgFile.HasMsg("SHMessageTests.Empty"));
        Assert.Equal("starts indented\nsame level", MsgFile.Msg("SHMessageTests.Two"));
        Assert.False(MsgFile.LoadFromString(""));
    }
}
