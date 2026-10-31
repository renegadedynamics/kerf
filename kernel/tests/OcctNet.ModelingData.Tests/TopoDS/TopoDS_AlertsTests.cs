// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Tests of TopoDS_AlertWithShape and TopoDS_AlertAttribute (no GTests in OCCT).

using static Occt.Message_Gravity;

namespace OcctNet.ModelingData.Tests;

internal sealed class TestShapeAlert(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape);

internal sealed class NullPrinter : Message_Printer
{
    protected override void send(string theString, Message_Gravity theGravity)
    {
    }
}

public class TopoDS_AlertsTests
{
    [Fact]
    public void AlertWithShape_NeverMerges()
    {
        TopoDS_Edge anEdge = TestShapes.Edge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0));
        Message_Report aReport = new();
        aReport.AddAlert(Message_Warning, new TestShapeAlert(anEdge));
        aReport.AddAlert(Message_Warning, new TestShapeAlert(anEdge));
        Assert.Equal(2, aReport.GetAlerts(Message_Warning).Extent());
        Assert.True(aReport.HasAlert(typeof(TestShapeAlert), Message_Warning));
        Assert.False(new TestShapeAlert(anEdge).SupportsMerge());
    }

    [Fact]
    public void GetShape_Publishes()
    {
        TopoDS_Edge anEdge = TestShapes.Edge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0));
        Assert.False(anEdge.TShape()!.IsFrozen());
        TestShapeAlert anAlert = new(anEdge);
        Assert.False(anAlert.InternalGetShape().TShape()!.IsFrozen());
        TopoDS_Shape aShape = anAlert.GetShape();
        Assert.True(aShape.IsSame(anEdge));
        Assert.True(anEdge.TShape()!.IsFrozen());
        Assert.True(TopoDS.Edge(aShape).IsEqual(anEdge));
    }

    [Fact]
    public void AlertAttribute_SendsToReportPrinters()
    {
        TopoDS_Edge anEdge = TestShapes.Edge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0));
        TopoDS_AlertAttribute anAttribute = new(anEdge, "edge");
        Assert.Equal("edge", anAttribute.GetMessageKey());
        Assert.Equal(anEdge.ToString(), anAttribute.Stream());
        Assert.True(anAttribute.GetShape().IsSame(anEdge));

        Message_Report aReport = new();
        Message_Messenger aMessenger = new(new NullPrinter());
        TopoDS_AlertAttribute.Send(aMessenger, anEdge); // no printer to a report: nothing
        Assert.True(aReport.GetAlerts(Message_Info).IsEmpty());
        aReport.ActivateInMessenger(true, aMessenger);
        TopoDS_AlertAttribute.Send(aMessenger, anEdge);
        var anAlert = Assert.IsType<Message_AlertExtended>(aReport.GetAlerts(Message_Info).First());
        var aShapeAttribute = Assert.IsType<TopoDS_AlertAttribute>(anAlert.Attribute());
        Assert.True(aShapeAttribute.GetShape().IsSame(anEdge));
    }
}
