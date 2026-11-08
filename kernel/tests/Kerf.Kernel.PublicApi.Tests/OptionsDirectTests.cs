// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Exchange;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Offsets;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// Members no test named until unit 12R (direct-members.md): the STEP and BREP options and read limits, as documented
/// and by what they do to what's written or read; a box's equality; and the offset options that change nothing for a
/// box, which must then give the box's own offset.
/// </summary>
public class OptionsDirectTests
{
    // Reports in the order they come, on the caller's thread (Progress<T> posts them elsewhere)
    private sealed class Recorder : IProgress<double>
    {
        public List<double> Values { get; } = [];

        public void Report(double value) => Values.Add(value);
    }

    private static string StepText(Shape theShape, StepWriteOptions? theOptions = null)
    {
        var aStream = new MemoryStream();
        Assert.True(Step.Write(theShape, aStream, theOptions).IsSuccess);
        return Encoding.ASCII.GetString(aStream.ToArray());
    }

    private static MemoryStream Stream(string theText) => new(Encoding.ASCII.GetBytes(theText));

    [Fact]
    public void Read_limits_default_as_documented_and_refuse_what_passes_them()
    {
        ReadLimits d = ReadLimits.Default;
        Assert.Equal(1L << 30, d.MaxBytes);
        Assert.Equal(20_000_000, d.MaxEntities);
        Assert.Equal(1_000, d.MaxDepth);
        Assert.Equal(50_000_000, d.MaxTriangles);
        Assert.Equal(1_000_000, d.MaxNodes);
        var aTight = new ReadLimits { MaxEntities = 10, MaxDepth = 4, MaxNodes = 2, MaxTriangles = 12 };
        Assert.Equal((10, 4, 2, 12L), (aTight.MaxEntities, aTight.MaxDepth, aTight.MaxNodes, aTight.MaxTriangles));

        // a box's STEP has hundreds of entities
        string aText = StepText(Fixtures.Box());
        Result<ModelNode> aRefused = Step.Read(Stream(aText), new StepReadOptions { Limits = new ReadLimits { MaxEntities = 10 } });
        Assert.False(aRefused.IsSuccess);
        Assert.Equal("exchange.limit-exceeded", aRefused.Error!.Code.Value);
        Assert.True(Step.Read(Stream(aText), new StepReadOptions { Limits = new ReadLimits { MaxEntities = 10_000 } }).IsSuccess);
    }

    [Fact]
    public void Step_read_options_default_as_documented()
    {
        StepReadOptions d = StepReadOptions.Default;
        Assert.True(d.Heal);
        Assert.True(d.ProductNames);
        Assert.False(d.ExternalReferences);
        Assert.Equal(ReadLimits.Default, d.Limits);
        var aSet = new StepReadOptions { ExternalReferences = true, Limits = new ReadLimits { MaxDepth = 7 } };
        Assert.True(aSet.ExternalReferences);
        Assert.Equal(7, aSet.Limits.MaxDepth);
    }

    [Fact]
    public void A_file_reads_with_its_progress_reported_and_its_references_followed_or_not()
    {
        string aPath = Path.Combine(Path.GetTempPath(), $"kerf-12r-{Guid.NewGuid():N}.stp");
        try
        {
            File.WriteAllText(aPath, StepText(Fixtures.Box()), Encoding.ASCII);
            foreach (bool aReferences in new[] { false, true })
            {
                var aRecorder = new Recorder();
                Result<ModelNode> aRead = Step.Read(aPath, new StepReadOptions { ExternalReferences = aReferences }, aRecorder, CancellationToken.None);
                Assert.True(aRead.IsSuccess, aRead.Error?.Message);
                // a file with no references reads the same either way
                Assert.Equal(6000, aRead.Value.Shape!.Value.IntegrateVolume(), 1e-6);
                Assert.NotEmpty(aRecorder.Values);
                Assert.All(aRecorder.Values, v => Assert.InRange(v, 0, 1));
                Assert.Equal(1, aRecorder.Values[^1], 9);
            }
            using var aCancelled = new CancellationTokenSource();
            aCancelled.Cancel();
            Assert.ThrowsAny<OperationCanceledException>(() => Step.Read(aPath, null, null, aCancelled.Token));
        }
        finally
        {
            File.Delete(aPath);
        }
    }

    [Fact]
    public void Step_write_options_default_as_documented_and_pcurves_can_be_left_out()
    {
        StepWriteOptions d = StepWriteOptions.Default;
        Assert.Equal(LengthUnit.Millimetre, d.Unit);
        Assert.True(d.WritePcurves);
        Assert.Equal(StepPrecision.Average, d.Precision);
        Assert.Equal(StepSchema.Ap214, d.Schema);

        // a box's edges have their curves on both faces, which are written unless asked not to be
        string aWith = StepText(Fixtures.Box()), aWithout = StepText(Fixtures.Box(), new StepWriteOptions { WritePcurves = false });
        Assert.False(new StepWriteOptions { WritePcurves = false }.WritePcurves);
        Assert.Contains("PCURVE", aWith);
        Assert.DoesNotContain("PCURVE", aWithout);
        Assert.True(aWithout.Length < aWith.Length);
        Result<ModelNode> aRead = Step.Read(Stream(aWithout));
        Assert.True(aRead.IsSuccess);
        Assert.Equal(6000, aRead.Value.Shape!.Value.IntegrateVolume(), 1e-6);
    }

    [Fact]
    public void Brep_write_options_default_as_documented()
    {
        BrepWriteOptions d = BrepWriteOptions.Default;
        Assert.False(d.Binary);
        Assert.True(d.WithTriangles);
        Assert.False(d.WithNormals);
        var aSet = new BrepWriteOptions { Binary = true, WithTriangles = false, WithNormals = true };
        Assert.Equal((true, false, true), (aSet.Binary, aSet.WithTriangles, aSet.WithNormals));
    }

    [Fact]
    public void Boxes_are_equal_by_their_corners()
    {
        BoundingBox a = BoundingBox.Of(new Point3D(0, 0, 0), new Point3D(1, 2, 3));
        BoundingBox b = new(new Point3D(0, 0, 0), new Point3D(1, 2, 3));
        BoundingBox c = a.Including(new Point3D(4, 0, 0));
        Assert.True(a.Equals(b));
        Assert.True(a == b);
        Assert.False(a != b);
        Assert.False(a.Equals(c));
        Assert.True(a != c);
        Assert.False(a == c);
        // empty boxes are equal to each other and to nothing else
        Assert.True(BoundingBox.Empty == default);
        Assert.True(BoundingBox.Empty != a);
        // a solid's box, asked twice, is the same box
        Solid aBox = Fixtures.Box();
        Assert.True(aBox.BoundingBox() == aBox.BoundingBox());
    }

    [Fact]
    public void Offset_options_that_change_nothing_for_a_box_give_its_own_offset()
    {
        // a box of 4 grown by 1 with arcs: the box, its slabs, quarter cylinders on its edges and spheres at its corners
        Solid aBox = Solid.Box(4, 4, 4).Value;
        double aRounded = 64 + (6 * 16) + (12 * 4 * Math.PI / 4) + (4.0 / 3 * Math.PI);
        foreach (OffsetOptions anOptions in new[]
        {
            new OffsetOptions { Tolerance = 1e-5 },
            new OffsetOptions { Mode = OffsetMode.Skin },
            new OffsetOptions { Intersection = true },
            new OffsetOptions { SelfIntersection = true },
            new OffsetOptions { RemoveInternalEdges = true },
        })
        {
            Result<Built<Shape>> aGrown = aBox.Offset(1, anOptions);
            Assert.True(aGrown.IsSuccess, aGrown.Error?.Message);
            Assert.Equal(aRounded, aGrown.Value.Shape.IntegrateVolume(), 5);
        }
        Assert.Equal(1e-5, new OffsetOptions { Tolerance = 1e-5 }.Tolerance);
        Assert.True(new OffsetOptions { Intersection = true, SelfIntersection = true, RemoveInternalEdges = true } is { Intersection: true, SelfIntersection: true, RemoveInternalEdges: true });
    }

    [Theory]
    [InlineData(LengthUnit.Millimetre, ".MILLI.")]
    [InlineData(LengthUnit.Micrometre, ".MICRO.")]
    [InlineData(LengthUnit.Centimetre, ".CENTI.")]
    [InlineData(LengthUnit.Metre, "SI_UNIT($,.METRE.)")]
    [InlineData(LengthUnit.Inch, "INCH")]
    [InlineData(LengthUnit.Foot, "FOOT")]
    public void A_shape_written_in_a_unit_reads_back_in_millimetres_by_its_factor(LengthUnit theUnit, string theMarker)
    {
        // the unit is the shape's: the box's 10 by 20 by 30 are written as so many of it, and read back in millimetres,
        // so its volume grows by the cube of the unit's millimetres
        string aText = StepText(Fixtures.Box(), new StepWriteOptions { Unit = theUnit });
        Assert.Equal(theUnit, new StepWriteOptions { Unit = theUnit }.Unit);
        Assert.Contains(theMarker, aText);
        Result<ModelNode> aRead = Step.Read(Stream(aText));
        Assert.True(aRead.IsSuccess, aRead.Error?.Message);
        double k = theUnit.Millimetres(), aVolume = 6000 * k * k * k;
        Assert.Equal(aVolume, aRead.Value.Shape!.Value.IntegrateVolume(), aVolume * 1e-9);
    }

    public static TheoryData<string> Solids() => ["cone", "sphere", "torus", "half cone", "ring", "lofted", "holed"];

    private static (Shape Shape, double Volume) SolidOf(string theName)
    {
        switch (theName)
        {
            case "cone":
                return (Solid.Cone(5, 0, 12).Value.Shape, Math.PI * 25 * 12 / 3);
            case "sphere":
                return (Solid.Sphere(3).Value.Shape, 4.0 / 3 * Math.PI * 27);
            case "torus":
                return (Solid.Torus(10, 2).Value.Shape, 2 * Math.PI * Math.PI * 10 * 4);
            case "half cone":
                return (Solid.Cone(5, 2, 10, Math.PI).Value.Shape, Math.PI * 10 / 3 * (25 + 10 + 4) / 2);
            case "ring":
            {
                // a square section 1 by 1 at radius 5 to 6, turned all the way round: Pappus at 5.5
                Face aSection = Face.FromWire(Wire.Polygon([new(5, 0, 0), new(6, 0, 0), new(6, 0, 1), new(5, 0, 1)], closed: true).Value).Value;
                return (aSection.Revolve(Axis3D.ZAxis).Value.Shape, 2 * Math.PI * 5.5);
            }
            case "lofted":
            {
                // squares of 4 and 2, 6 apart: a frustum of a pyramid
                Wire aBottom = Wire.Polygon([new(0, 0, 0), new(4, 0, 0), new(4, 4, 0), new(0, 4, 0)], closed: true).Value;
                Wire aTop = Wire.Polygon([new(1, 1, 6), new(3, 1, 6), new(3, 3, 6), new(1, 3, 6)], closed: true).Value;
                return (Sweeps.Loft([aBottom, aTop], new LoftOptions { MakeSolid = true, Ruled = true }).Value.Shape, 6.0 / 3 * (16 + 8 + 4));
            }
            default:
            {
                Solid aBlock = Solid.Box(10, 10, 10).Value;
                Solid aHole = Solid.Cylinder(2, 20, new Frame3D(new Point3D(5, 5, -5), Direction3D.UnitZ)).Value;
                return (aBlock.Cut(aHole).Value.Shape, 1000 - (Math.PI * 4 * 10));
            }
        }
    }

    [Theory]
    [MemberData(nameof(Solids))]
    public void A_solid_written_and_read_keeps_its_volume(string theName)
    {
        // apexes, poles, seams and holes go through STEP's degenerate edges, vertex loops and inner loops
        (Shape aShape, double aVolume) = SolidOf(theName);
        Assert.Equal(aVolume, aShape.IntegrateVolume(), aVolume * 1e-6);
        Result<ModelNode> aRead = Step.Read(Stream(StepText(aShape)));
        Assert.True(aRead.IsSuccess, aRead.Error?.Message);
        Assert.Equal(aVolume, aRead.Value.Shape!.Value.IntegrateVolume(), aVolume * 1e-6);
    }
}
