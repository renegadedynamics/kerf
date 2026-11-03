// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Tests of TopoDS_AlertWithShape and TopoDS_AlertAttribute (no GTests in OCCT).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Foundation.Gravity;

namespace OcctNet.ModelingData.Tests;

internal sealed class TestShapeAlert(in Shape theShape) : AlertWithShape(theShape);

internal sealed class NullPrinter : Printer
{
    protected override void SendCore(string theString, Gravity theGravity)
    {
    }
}

public class TopoDS_AlertsTests
{
    [Fact]
    public void AlertWithShape_NeverMerges()
    {
        Edge anEdge = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        Report aReport = new();
        aReport.AddAlert(Warning, new TestShapeAlert(anEdge));
        aReport.AddAlert(Warning, new TestShapeAlert(anEdge));
        Assert.Equal(2, aReport.GetAlerts(Warning).Extent());
        Assert.True(aReport.HasAlert(typeof(TestShapeAlert), Warning));
        Assert.False(new TestShapeAlert(anEdge).SupportsMerge());
    }

    [Fact]
    public void GetShape_Publishes()
    {
        Edge anEdge = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        Assert.False(anEdge.TShape()!.IsFrozen);
        TestShapeAlert anAlert = new(anEdge);
        Assert.False(anAlert.InternalGetShape().TShape()!.IsFrozen);
        Shape aShape = anAlert.GetShape();
        Assert.True(aShape.IsSame(anEdge));
        Assert.True(anEdge.TShape()!.IsFrozen);
        Assert.True(ShapeCasts.Edge(aShape).IsEqual(anEdge));
    }

    [Fact]
    public void AlertAttribute_SendsToReportPrinters()
    {
        Edge anEdge = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        AlertAttribute anAttribute = new(anEdge, "edge");
        Assert.Equal("edge", anAttribute.GetMessageKey());
        Assert.Equal(anEdge.ToString(), anAttribute.Stream());
        Assert.True(anAttribute.GetShape().IsSame(anEdge));

        Report aReport = new();
        Messenger aMessenger = new(new NullPrinter());
        AlertAttribute.Send(aMessenger, anEdge); // no printer to a report: nothing
        Assert.True(aReport.GetAlerts(Info).IsEmpty());
        aReport.ActivateInMessenger(true, aMessenger);
        AlertAttribute.Send(aMessenger, anEdge);
        var anAlert = Assert.IsType<AlertExtended>(aReport.GetAlerts(Info).First());
        var aShapeAttribute = Assert.IsType<AlertAttribute>(anAlert.Attribute());
        Assert.True(aShapeAttribute.GetShape().IsSame(anEdge));
    }
}
