// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BOPAlgo_BOP's <c>BuildSolid</c>, the fuse of solids (occt-issues.md, unit 6F's first row; Kerf unit 10K1).
/// </summary>
public class BOPAlgo_BuildSolidTests
{
    private static int Count(in Shape theShape, ShapeType theType)
    {
        IndexedSet<Shape> aMap = new(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    private static double Volume(in Shape theShape)
    {
        Properties aProps = new();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static Shape ShellOf(in Shape theSolid)
    {
        Explorer anExp = new(theSolid, ShapeType.Shell);
        return anExp.Current();
    }

    /// <summary>
    /// #1613: a reversed solid whose shell is reversed too is the ordinary solid. In a compound, OCCT turned its splits
    /// inside out, and a fuse dropped it with no error (decision 1832).
    /// </summary>
    [Fact]
    public void A_reversed_solid_in_a_compound_fuses_as_it_does_alone()
    {
        Shape aBox = new MakeBox(new Point3D(5, 5, 5), 10, 10, 10).Shape();
        Shape aReversed = Builder.MakeSolid([ShellOf(aBox).Reversed()]).Reversed();
        Assert.Equal(1000.0, Volume(aReversed), 1e-6);
        Shape anOther = new MakeBox(new Point3D(30, 30, 30), 1, 1, 1).Shape();
        var aBuilder = new BrepBuilder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, aReversed);
        aBuilder.Add(aCompound, anOther);

        Shape aBase = new MakeBox(10, 10, 10).Shape();
        Assert.Equal(1875.0, Volume(new Fuse(aBase, aReversed).Shape()), 1e-6);
        Assert.Equal(1876.0, Volume(new Fuse(aBase, aCompound).Shape()), 1e-6);
        Assert.Equal(1876.0, Volume(new Fuse(aCompound, aBase).Shape()), 1e-6);
    }

    /// <summary>
    /// A ball in a hollow ball, sharing the inner sphere: the hollow ball's faces are the outer sphere then the inner
    /// one, and the ball adds no face of its own, so the shared face is the last of BuildSolid's face-to-solids map.
    /// OCCT's scan stopped before it, so both solids were taken as untouched and kept apart; they share a face, so the
    /// fuse merges them into one ball (the shared face, met with both orientations, goes). The box is far away.
    /// </summary>
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Fuse_SolidsSharingOnlyTheLastFace_AreMerged(bool theCompSolid)
    {
        Shape anInner = new MakeSphere(1.0).Shape();
        Shape anOuterBall = new MakeSphere(2.0).Shape();
        Shape aHollow = Builder.MakeSolid([ShellOf(anOuterBall), ShellOf(anInner).Reversed()]);
        Shape anArgument = theCompSolid ? Builder.MakeCompSolid([aHollow, anInner]) : Builder.MakeCompound(aHollow, anInner);
        Shape aBox = new MakeBox(new Point3D(10.0, 10.0, 10.0), 1.0, 1.0, 1.0).Shape();

        Fuse aFuse = new();
        aFuse.SetArguments(new NCollection_List<Shape>([anArgument]));
        aFuse.SetTools(new NCollection_List<Shape>([aBox]));
        aFuse.Build();

        Assert.True(aFuse.IsDone);
        Shape aResult = aFuse.Shape();
        Assert.Equal(2, Count(aResult, ShapeType.Solid)); // the ball of radius 2 and the box
        Assert.Equal(7, Count(aResult, ShapeType.Face)); // the outer sphere's one face and the box's six
        Assert.Equal(4.0 / 3.0 * Math.PI * 8.0 + 1.0, Volume(aResult), 1e-3);
    }
}
