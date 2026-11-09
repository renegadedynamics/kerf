// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Kerf.Healing (docs/api/healing.md): checking, fixing, sewing, unifying, and the wire tools.</summary>
public class HealingTests
{
    [Fact]
    public void A_valid_shape_has_an_empty_report()
    {
        ValidationReport report = Fixtures.Box().Validate();
        Assert.True(report.IsValid);
        Assert.Empty(report.Problems);
        Assert.True(HealingOperations.Validate(Fixtures.Cylinder(), new ValidationOptions { Parallel = false, Exact = true }).IsValid);
    }

    [Fact]
    public void A_solid_of_an_open_shell_is_reported()
    {
        Solid box = Fixtures.Box();
        Shell open = Shell.Create(box.AllFaces().Take(5));
        ValidationReport report = Solid.Create([open]).Validate();
        Assert.False(report.IsValid);
        Assert.Contains(report.Problems, p => p.Code.Value == "check.not-closed");
        Assert.All(report.Problems, p => Assert.False(p.Shape.IsNull));
        Assert.All(report.Problems, p => Assert.EndsWith(".", p.Message));
    }

    [Fact]
    public void Fixing_a_valid_shape_keeps_it()
    {
        Solid box = Fixtures.Box();
        Result<Built<Shape>> fixedBox = box.Fix();
        Assert.True(fixedBox.IsSuccess);
        Assert.Equal(6000, fixedBox.Value.Shape.IntegrateVolume(), 1e-6);
        Assert.True(fixedBox.Value.Shape.Validate().IsValid);
        Assert.True(HealingOperations.Fix(box, new FixOptions { MaxTolerance = 1e-3, FixSolids = true }).IsSuccess);
    }

    [Fact]
    public void A_shape_left_invalid_by_fixing_fails_and_is_carried()
    {
        Shell open = Shell.Create(Fixtures.Box().AllFaces().Take(5));
        Result<Built<Shape>> fixedSolid = Solid.Create([open]).Fix(new FixOptions { FixSolids = false, FixShells = false });
        Assert.True(fixedSolid.IsFailure);
        Assert.Equal(DiagnosticCode.InvalidResult, fixedSolid.Error!.Code);
        Assert.NotEmpty(fixedSolid.Error.Shapes);
        Assert.True(fixedSolid.TryGetInvalid(out Built<Shape>? left));
        Assert.False(left.Shape.Validate().IsValid);
        Assert.True(Fixtures.Box().Fix().IsSuccess);
    }

    [Fact]
    public void Unifying_merges_the_faces_on_one_plane()
    {
        Shape fused = Fixtures.TwoBoxesFused();
        Assert.Equal(10, fused.AllFaces().Length);
        Result<Built<Shape>> unified = fused.Unify();
        Assert.True(unified.IsSuccess);
        Shape result = unified.Value;
        Assert.Equal(6, result.AllFaces().Length);
        Assert.Equal(2000, result.IntegrateVolume(), 1e-6);
        Assert.Contains(fused.AllFaces(), f => unified.Value.History.Modified(f).Count > 0);
        Assert.Equal(10, fused.AllFaces().Length);   // the input is unchanged
    }

    [Fact]
    public void Faces_sew_into_a_shell_that_makes_a_solid()
    {
        Solid box = Fixtures.Box();
        Result<Built<Shape>> sewn = HealingOperations.Sew(box.AllFaces().Select(f => (Shape)f), new SewOptions { Tolerance = 1e-4 });
        Assert.True(sewn.IsSuccess);
        Assert.True(sewn.Value.Shape.TryAs(out Shell shell));
        Result<Built<Solid>> solid = shell.MakeSolid();
        Assert.True(solid.IsSuccess);
        Assert.Equal(6000, Math.Abs(solid.Value.Shape.IntegrateVolume()), 1e-6);
    }

    [Fact]
    public void An_open_shell_makes_no_solid()
    {
        Shell open = Shell.Create(Fixtures.Box().AllFaces().Take(5));
        Result<Built<Solid>> solid = open.MakeSolid();
        Assert.False(solid.IsSuccess);
        Assert.Equal(HealingOperations.NotAClosedShell, solid.Error!.Code);
    }

    [Fact]
    public void Edges_connect_into_wires_and_free_boundaries_are_found()
    {
        Solid box = Fixtures.Box();
        Face face = box.AllFaces()[0];
        var wires = HealingOperations.ConnectEdges(face.AllEdges(), 1e-6);
        Assert.Single(wires);
        Assert.Equal(4, wires[0].AllEdges().Length);

        Shell open = Shell.Create(box.AllFaces().Take(5));
        var bounds = open.FreeBoundaries();
        Assert.Single(bounds);
        Assert.Equal(4, bounds[0].AllEdges().Length);
        Assert.Empty(box.AllShells()[0].FreeBoundaries());
    }

    [Fact]
    public void Surfaces_convert_to_B_splines_and_wireframes_are_fixed()
    {
        Solid cylinder = Fixtures.Cylinder();
        Result<Built<Shape>> converted = cylinder.ConvertToBSpline(new BSplineConversionOptions { Planes = true });
        Assert.True(converted.IsSuccess);
        Assert.Equal(cylinder.IntegrateVolume(), converted.Value.Shape.IntegrateVolume(), 1e-3);

        Result<Built<Shape>> wireframe = Fixtures.Box().FixWireframe(1e-4);
        Assert.True(wireframe.IsSuccess);
        Assert.Equal(12, wireframe.Value.Shape.AllEdges().Length);
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => Fixtures.Box().FixWireframe(0));
    }

    [Fact]
    public void A_null_shape_is_misuse()
    {
        KernelAssert.Throws<ArgumentException>(() => default(Shape).Validate());
        KernelAssert.Throws<ArgumentException>(() => default(Shape).Fix());
        KernelAssert.Throws<ArgumentException>(() => HealingOperations.Sew([default(Shape)]));
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => new SewOptions { Tolerance = 0 });
    }
}
