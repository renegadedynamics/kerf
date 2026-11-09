// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The volume check keeps a frozen shape's volume with its TShape (unit 12Y): what it gives back must be the bits a fresh
/// integration gives, for each placement and orientation of the shape.
/// </summary>
public class BooleanVolumeKeptTests
{
    private static readonly MassOptions Measured = new() { Tolerance = 1e-6 };

    private static Shape Moved(Shape shape, double x, double y, double z) =>
        shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(x, y, z))));

    private static void Same(Shape shape)
    {
        double fresh = shape.IntegrateVolume(Measured);
        // The first measures and keeps it, the second reads it back
        Assert.Equal(BitConverter.DoubleToInt64Bits(fresh), BitConverter.DoubleToInt64Bits(BooleanVolumes.VolumeOf(shape)));
        Assert.Equal(BitConverter.DoubleToInt64Bits(fresh), BitConverter.DoubleToInt64Bits(BooleanVolumes.VolumeOf(shape)));
    }

    [Fact]
    public void A_chain_of_cuts_reads_back_the_bits_it_measured()
    {
        Shape plate = Solid.Box(60, 40, 6).Value.Shape;
        Same(plate);
        for (int i = 0; i < 6; i++)
        {
            Shape hole = Moved(Solid.Cylinder(2.5 + i * 0.25, 10).Value.Shape, 6 + 9 * i, 12 + 2 * i, -2);
            Shape cut = plate.Cut(hole).Value.Shape;
            // The check measured the cut, and the next one measures it again as its object
            Same(cut);
            Same(hole);
            plate = cut;
        }
    }

    [Fact]
    public void Each_placement_and_orientation_has_its_own()
    {
        Shape cylinder = Solid.Cylinder(3, 7).Value.Shape;
        // More placements than are kept a TShape, each measured twice; and a reversed solid's volume is negative
        for (int i = 0; i < 7; i++)
        {
            Same(Moved(cylinder, 1e3 * i + 0.1, -i, 7.5 * i));
            Same(Moved(cylinder, 1e3 * i + 0.1, -i, 7.5 * i).Reversed());
        }
        Assert.True(BooleanVolumes.VolumeOf(cylinder.Reversed()) < 0);
        Same(cylinder);
        // The placements' volumes differ in their last bits, so a key without the placement would fail above
        Assert.True(Enumerable.Range(0, 7).Select(i => Moved(cylinder, 1e3 * i + 0.1, -i, 7.5 * i).IntegrateVolume(Measured)).Distinct().Count() > 1);
    }
}
