// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Offsets;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepGProp;

/// <summary>
/// A measure that couldn't be confirmed to its precision (decision 34806): the public forms that return a number throw an
/// <see cref="ArithmeticException"/> whose diagnostic is <c>measurement.unconfirmed</c>, <c>Measure</c> fails with it, and no
/// check or proof of the kernel's passes it. Every confirmation is made to fail on the test's thread, as where nothing
/// confirms a face.
/// </summary>
public class UnconfirmedMeasureTests
{
    private static Solid Box(double size = 10) => Primitives.Box(size, size, size).Value.Shape;

    private static Solid Cylinder() => Primitives.Cylinder(3, 20, new Frame3D(new Point3D(5, 5, -5), Direction3D.UnitZ)).Value.Shape;

    private static T Unconfirmable<T>(Func<T> measure)
    {
        MeasureOperations.s_confirmNothing = true;
        try
        {
            return measure();
        }
        finally
        {
            MeasureOperations.s_confirmNothing = false;
        }
    }

    [Fact]
    public void A_volume_that_cant_be_confirmed_throws_saying_which_face_and_why()
    {
        Solid box = Box();
        var e = Assert.ThrowsAny<ArithmeticException>(() => Unconfirmable(() => box.IntegrateVolume()));
        Diagnostic diagnostic = Diagnostic.FromException(e);
        Assert.Equal(MeasureOperations.Unconfirmed, diagnostic.Code);
        Assert.True(diagnostic.IsError);
        Assert.Contains("volume couldn't be confirmed to 1E-06: face 1", diagnostic.Message, StringComparison.Ordinal);
        Assert.True(Diagnostic.IsKernelFailure(e));
        Assert.ThrowsAny<ArithmeticException>(() => Unconfirmable(() => box.IntegrateArea()));
        Assert.ThrowsAny<ArithmeticException>(() => Unconfirmable(() => box.MassProperties(Dimension.Volume)));
        Assert.ThrowsAny<ArithmeticException>(() => Unconfirmable(() => box.CentreOfArea()));
        Assert.True(double.IsNaN(Unconfirmable(() => MeasureOperations.VolumeOrNaN(box, MassOptions.Default))));
    }

    [Fact]
    public void Measure_fails_with_the_code_and_is_the_mass_properties_where_confirmed()
    {
        Solid box = Box();
        Result<MassProperties> failed = Unconfirmable(() => box.Measure(Dimension.Volume));
        Assert.False(failed.IsSuccess);
        Assert.Equal(MeasureOperations.Unconfirmed, failed.Error!.Code);
        Result<MassProperties> measured = box.Measure(Dimension.Volume);
        Assert.True(measured.IsSuccess);
        Assert.Equal(1000, measured.Value.Mass, 1e-9);
        Assert.Equal(box.MassProperties(Dimension.Volume), measured.Value);
    }

    [Fact]
    public void A_Boolean_s_check_of_volume_fails_on_a_volume_it_cant_confirm()
    {
        Solid box = Box(), cylinder = Cylinder();
        // Before anything is measured: a frozen shape's volume, once confirmed, is kept (BooleanVolumes.VolumeOf)
        Result<Built<Shape>> checkedCut = Unconfirmable(() => box.Cut(cylinder));
        Assert.False(checkedCut.IsSuccess);
        Assert.Equal(MeasureOperations.Unconfirmed, checkedCut.Error!.Code);

        Solid other = Box(11);
        Shape cut = other.Cut(cylinder, new BooleanOptions { CheckResult = false }).Value.Shape;
        Diagnostic? wrong = Unconfirmable(() => BooleanVolumes.Check(BooleanVolumeRule.Cut, [other], [cylinder], cut));
        Assert.Equal(MeasureOperations.Unconfirmed, wrong?.Code);
        Assert.Null(BooleanVolumes.Check(BooleanVolumeRule.Cut, [other], [cylinder], cut));
    }

    [Fact]
    public void An_offset_s_check_of_volume_fails_on_a_volume_it_cant_confirm()
    {
        Solid box = Box();
        Shape grown = box.Offset(1).Value.Shape;
        Assert.Null(OffsetVolumes.Check(box, 1, JoinKind.Arc, grown, "the offset"));
        Assert.Equal(MeasureOperations.Unconfirmed, Unconfirmable(() => OffsetVolumes.Check(box, 1, JoinKind.Arc, grown, "the offset"))?.Code);
    }
}
