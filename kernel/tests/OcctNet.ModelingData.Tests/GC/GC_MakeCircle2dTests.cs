// Port of: src/ModelingData/TKGeomBase/GTests/GC_MakeCircle2d_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace OcctNet.ModelingData.Tests;

public class GC_MakeCircle2dTest
{
    [Fact]
    public void FromCircAndPoint_Done()
    {
        Circle2D aBase = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 5.0);
        Point2D aPoint = new(0.0, 10.0);
        MakeCircle2d aMaker = new(aBase, aPoint);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
        Assert.Equal(10.0, aMaker.Value().Circ2d().Radius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FromCircAndDist_Done()
    {
        Circle2D aBase = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 5.0);
        MakeCircle2d aMaker = new(aBase, -2.0);
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
        Assert.Equal(3.0, aMaker.Value().Circ2d().Radius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FromCircAndPoint_AtCenter_DoneZeroRadius()
    {
        Circle2D aBase = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 5.0);
        MakeCircle2d aMaker = new(aBase, new Point2D(0.0, 0.0));
        Assert.True(aMaker.IsDone);
        Assert.Equal(Done, aMaker.Status());
        Assert.NotNull(aMaker.Value());
        Assert.Equal(0.0, aMaker.Value().Circ2d().Radius(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
