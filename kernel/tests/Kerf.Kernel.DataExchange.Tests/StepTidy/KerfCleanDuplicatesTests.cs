// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Where StepTidy differs from OCCT's on purpose, and how it's reached from the public options (unit 11N).

using System.Text;
using Kerf.Construction;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Tidy;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Exchange;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.DataExchange.Tests.StepTidy;

public class KerfCleanDuplicatesTests : BaseTestFixture
{
    private static string Written(Shape shape, StepWriteOptions options)
    {
        var stream = new MemoryStream();
        Assert.True(Step.Write(shape, stream, options).IsSuccess);
        return Encoding.Latin1.GetString(stream.ToArray());
    }

    private static int Count(string text, string entity)
    {
        int count = 0;
        for (int at = text.IndexOf(entity, StringComparison.Ordinal); at >= 0; at = text.IndexOf(entity, at + 1, StringComparison.Ordinal))
        {
            count++;
        }
        return count;
    }

    [Fact]
    public void Only_exactly_equal_values_merge()
    {
        // Merging within 1e-12 changed what bug32596_abc_0_13.step read back as: decision 1665
        StepCartesianPoint point1 = addCartesianPoint(null, new Xyz(1.0, 2.0, 3.0));
        StepCartesianPoint point2 = addCartesianPoint(null, new Xyz(1.0 + 1e-13, 2.0, 3.0));
        StepCartesianPoint point3 = addCartesianPoint(null, new Xyz(1.0, 2.0, 3.0));
        Assert.False(CartesianPointHasher.Instance.Equals(point1, point2));
        Assert.True(CartesianPointHasher.Instance.Equals(point1, point3));
        Assert.Equal(CartesianPointHasher.Instance.GetHashCode(point1), CartesianPointHasher.Instance.GetHashCode(point3));
    }

    [Fact]
    public void A_zero_keeps_its_hash_whatever_its_sign()
    {
        StepCartesianPoint point1 = addCartesianPoint(null, new Xyz(0.0, 2.0, 3.0));
        StepCartesianPoint point2 = addCartesianPoint(null, new Xyz(-0.0, 2.0, 3.0));
        Assert.True(CartesianPointHasher.Instance.Equals(point1, point2));
        Assert.Equal(CartesianPointHasher.Instance.GetHashCode(point1), CartesianPointHasher.Instance.GetHashCode(point2));
    }

    [Fact]
    public void A_2D_point_or_direction_is_not_the_3D_one_with_a_zero_z()
    {
        StepCartesianPoint point3d = addCartesianPoint(null, new Xyz(1.0, 2.0, 0.0));
        var point2d = new StepCartesianPoint();
        point2d.Init2D(new TCollection_HAsciiString(), 1.0, 2.0);
        Assert.False(CartesianPointHasher.Instance.Equals(point3d, point2d));

        StepDirection direction3d = addDirection(null, new Xyz(1.0, 0.0, 0.0));
        var direction2d = new StepDirection();
        var ratios = new NCollection_HArray1<double>(1, 2);
        ratios.SetValue(1, 1.0);
        ratios.SetValue(2, 0.0);
        direction2d.Init(new TCollection_HAsciiString(), ratios);
        Assert.False(DirectionHasher.Instance.Equals(direction3d, direction2d));
    }

    [Fact]
    public void A_conic_takes_the_placement_kept()
    {
        // OCCT's sets the value on a copy of the position, so its conic keeps the duplicate
        StepCircle circle1 = addCircle(null, new Xyz(0.0, 0.0, 0.0), null, null, 1.0);
        StepCircle circle2 = addCircle(null, new Xyz(0.0, 0.0, 0.0), null, null, 2.0);
        StepAxis2Placement3d first = circle1.Position().Axis2Placement3d();
        Assert.NotSame(first, circle2.Position().Axis2Placement3d());
        new DuplicateCleaner(myWS).Perform();
        Assert.Same(first, circle2.Position().Axis2Placement3d());
    }

    [Fact]
    public void A_placement_with_one_direction_for_both_loses_the_duplicate()
    {
        // OCCT's replaces the axis and returns, leaving the same duplicate as the reference direction
        StepDirection direction1 = addDirection(null, new Xyz(1.0, 0.0, 0.0));
        StepDirection direction2 = addDirection(null, new Xyz(1.0, 0.0, 0.0));
        StepCartesianPoint location = addCartesianPoint();
        var placement1 = new StepAxis2Placement3d();
        placement1.Init(new TCollection_HAsciiString(), location, true, direction1, false, null!);
        addToModel(placement1);
        var placement2 = new StepAxis2Placement3d();
        placement2.Init(new TCollection_HAsciiString(), location, true, direction2, true, direction2);
        addToModel(placement2);
        new DuplicateCleaner(myWS).Perform();
        Assert.Same(direction1, placement2.Axis());
        Assert.Same(direction1, placement2.RefDirection());
    }

    [Fact]
    public void It_is_off_by_default_and_on_when_asked()
    {
        Assert.False(StepWriteOptions.Default.CleanDuplicates);
        Shape box = Primitives.Box(1, 2, 3).Value.Shape;
        string off = Written(box, StepWriteOptions.Default);
        string on = Written(box, new StepWriteOptions { CleanDuplicates = true });
        Assert.True(on.Length < off.Length);
        Assert.True(Count(on, "CARTESIAN_POINT") < Count(off, "CARTESIAN_POINT"));
        Assert.True(Count(on, "DIRECTION(") < Count(off, "DIRECTION("));
    }

    [Theory]
    [InlineData(StepSchema.Ap203)]
    [InlineData(StepSchema.Ap214)]
    [InlineData(StepSchema.Ap242)]
    public void The_file_keeps_its_schema_header_and_shape(StepSchema schema)
    {
        // OCCT's cleaned model forgets its parameters, so its file names the default schema
        Shape box = Primitives.Box(1, 2, 3).Value.Shape;
        string off = Written(box, new StepWriteOptions { Schema = schema });
        string on = Written(box, new StepWriteOptions { Schema = schema, CleanDuplicates = true });
        Assert.Equal(Record(off, "FILE_SCHEMA"), Record(on, "FILE_SCHEMA"));
        Assert.Equal(Record(off, "FILE_DESCRIPTION"), Record(on, "FILE_DESCRIPTION"));
        Assert.Contains("'Kerf'", on);

        var reader = new StepReader();
        reader.ReadStream("box", new MemoryStream(Encoding.Latin1.GetBytes(on)));
        reader.TransferRoots();
        Shape read = reader.OneShape();
        Assert.Equal(6.0, read.MassProperties(Dimension.Volume).Mass, 9);
        Assert.Equal(6, read.AllFaces().Length);
    }

    private static string Record(string text, string name)
    {
        int start = text.IndexOf(name, StringComparison.Ordinal);
        return text[start..text.IndexOf(';', start)];
    }
}
