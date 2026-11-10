// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Volumes integrated on several threads as a caller asks (decision 2092): by <see cref="MassOptions.ParallelVolumes"/> and
/// <see cref="BooleanOptions.ParallelVolumes"/>, or by the scope <see cref="ParallelVolumes"/> for code that passes neither. On or
/// off, the same bits; off unless asked for; the setting an operation is given overrides the scope.
/// </summary>
public class ParallelVolumesTests
{
    private static int Integrations => MassPropertiesBuilder.ParallelIntegrationsOnThisThread;

    // 24 spheres of radii 1 to 3.3, apart, in one solid: 24 faces to integrate
    private static Shape Spheres()
    {
        var aB = new BrepBuilder();
        Shell aShell = new();
        aB.MakeShell(ref aShell);
        for (int i = 0; i < 24; i++)
        {
            Face aF = BRepGPropTestShapes.NaturalFace(new SphericalSurface(
                new CoordinateSystem(new Point3D(10 * i, (i % 3) * 7.0, (i % 5) * 3.0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)),
                1 + 0.1 * i));
            aB.Add(aShell, aF);
        }

        return TestShapes.Frozen(BRepGPropTestShapes.SolidOf(aShell));
    }

    private static Shape Moved(Shape shape, double x, double y, double z) =>
        shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(x, y, z))));

    // A plate and 20 cylinders through it: the cut has 20 cylindrical faces to integrate
    private static (Shape Plate, Shape[] Holes) PlateAndHoles()
    {
        Shape plate = Solid.Box(100, 40, 6).Value.Shape;
        Shape[] holes = [.. Enumerable.Range(0, 20).Select(i => Moved(Solid.Cylinder(1.5 + 0.05 * i, 10).Value.Shape, 5 + 9 * (i % 10), 10 + 20 * (i / 10), -2))];
        return (plate, holes);
    }

    private static void SameBits(MassProperties expected, MassProperties actual)
    {
        Assert.Equal(BitConverter.DoubleToInt64Bits(expected.Mass), BitConverter.DoubleToInt64Bits(actual.Mass));
        Assert.Equal(expected.CentreOfMass, actual.CentreOfMass);
        Assert.Equal(expected.Inertia, actual.Inertia);
    }

    [Fact]
    public void The_scope_is_off_outside_and_on_inside_and_nests()
    {
        Assert.False(ParallelVolumes.Current);
        using (ParallelVolumes.Use())
        {
            Assert.True(ParallelVolumes.Current);
            using (ParallelVolumes.Use(false))
            {
                Assert.False(ParallelVolumes.Current);
            }

            Assert.True(ParallelVolumes.Current);
        }

        Assert.False(ParallelVolumes.Current);
    }

    [Fact]
    public async Task The_scope_flows_into_tasks_and_loops_and_ends_where_it_is_disposed()
    {
        bool inTask, inLoop = true;
        ParallelVolumes.Scope scope = ParallelVolumes.Use();
        inTask = await Task.Run(() => ParallelVolumes.Current);
        Parallel.For(0, 8, _ =>
        {
            if (!ParallelVolumes.Current)
            {
                inLoop = false;
            }
        });
        Assert.True(inTask);
        Assert.True(inLoop);

        // A task that outlives the scope goes back to what was outside it, once the scope is disposed in another task
        var release = new TaskCompletionSource();
        Task<bool> outliving = Task.Run(async () =>
        {
            await release.Task;
            return ParallelVolumes.Current;
        });
        await Task.Run(() => scope.Dispose());
        Assert.False(ParallelVolumes.Current);
        release.SetResult();
        Assert.False(await outliving);
    }

    [Fact]
    public void Mass_properties_are_the_same_bits_on_and_off_and_parallel_only_when_asked()
    {
        Shape spheres = Spheres();
        foreach (double tolerance in new[] { 1e-6, 1e-9, 0 })
        {
            var off = new MassOptions { Tolerance = tolerance, ParallelVolumes = false };
            var on = off with { ParallelVolumes = true };
            var unset = off with { ParallelVolumes = null };

            int before = Integrations;
            MassProperties serial = spheres.MassProperties(Dimension.Volume, off);
            double serialVolume = spheres.IntegrateVolume(off);
            MassProperties byDefault = spheres.MassProperties(Dimension.Volume, unset);
            Assert.Equal(before, Integrations);

            MassProperties parallel = spheres.MassProperties(Dimension.Volume, on);
            double parallelVolume = spheres.IntegrateVolume(on);
            Assert.Equal(before + 2, Integrations);
            SameBits(serial, parallel);
            SameBits(serial, byDefault);
            Assert.Equal(BitConverter.DoubleToInt64Bits(serialVolume), BitConverter.DoubleToInt64Bits(parallelVolume));

            // The scope for calls with no setting; a setting overrides it either way
            using (ParallelVolumes.Use())
            {
                SameBits(serial, spheres.MassProperties(Dimension.Volume, unset));
                Assert.Equal(before + 3, Integrations);
                SameBits(serial, spheres.MassProperties(Dimension.Volume, off));
                Assert.Equal(before + 3, Integrations);
            }

            Assert.False(ParallelVolumes.Current);
        }
    }

    [Fact]
    public void A_Booleans_check_measures_in_parallel_when_asked_and_gives_the_same_result()
    {
        (Shape plate, Shape[] holes) = PlateAndHoles();
        var off = new BooleanOptions { ParallelVolumes = false };
        var on = new BooleanOptions { ParallelVolumes = true };
        var measured = new MassOptions { ParallelVolumes = false };

        int before = Integrations;
        Result<Built<Shape>> serial = BooleanOperations.Cut([plate], holes, off);
        Assert.Equal(before, Integrations);
        Result<Built<Shape>> parallel = BooleanOperations.Cut([plate], holes, on);
        Assert.True(Integrations > before);

        Assert.True(serial.IsSuccess);
        Assert.True(parallel.IsSuccess);
        Shape a = serial.Value.Shape, b = parallel.Value.Shape;
        Assert.Equal(a.Children.Count(), b.Children.Count());
        SameBits(a.MassProperties(Dimension.Volume, measured), b.MassProperties(Dimension.Volume, measured));

        // The scope, for a Boolean given no setting; the setting overrides it
        before = Integrations;
        using (ParallelVolumes.Use())
        {
            Assert.True(BooleanOperations.Cut([plate], holes).IsSuccess);
            int inScope = Integrations;
            Assert.True(inScope > before);
            Assert.True(BooleanOperations.Cut([plate], holes, off).IsSuccess);
            Assert.Equal(inScope, Integrations);
        }
    }
}
