// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// <see cref="Placement.ProductTransformation"/> gives the very transformation the product's chain has, bit for bit,
/// without building it: over chains that share datums, cancel at the join, invert, and run past the recursion limit.
/// </summary>
public class PlacementProductTests
{
    private static Datum3D[] Datums()
    {
        var random = new Random(1701);
        var datums = new Datum3D[6];
        for (int i = 0; i < datums.Length; i++)
        {
            var axis = new Axis3D(new Point3D(random.NextDouble(), random.NextDouble(), random.NextDouble()),
                new Direction3D(random.NextDouble() - 0.5, random.NextDouble() - 0.5, random.NextDouble() + 0.1));
            Transform3D t = Transform3D.Rotation(axis, random.NextDouble() * 3)
                .Multiplied(Transform3D.Translation(new Vector3D(random.NextDouble() * 100, -random.NextDouble() * 50, 7.25 * i)));
            datums[i] = new Datum3D(t);
        }

        return datums;
    }

    private static Placement Chain(Random random, Datum3D[] datums, int length)
    {
        var result = new Placement();
        for (int i = 0; i < length; i++)
        {
            var item = new Placement(datums[random.Next(datums.Length)]);
            result = random.Next(4) switch
            {
                0 => result * item.Inverted(),
                1 => item * result,
                2 => result * item.Powered(random.Next(1, 3)),
                _ => result * item,
            };
        }

        return result;
    }

    [Fact]
    public void The_product_transformation_is_the_products_own()
    {
        Datum3D[] datums = Datums();
        var random = new Random(26701);
        int cancelled = 0;
        for (int round = 0; round < 20_000; round++)
        {
            Placement left = Chain(random, datums, random.Next(0, 6));
            Placement right = random.Next(5) == 0 ? left.Inverted() : Chain(random, datums, random.Next(0, 6));
            if (random.Next(4) == 0)
            {
                right = left.NextLocation().Inverted() * right;
            }

            Placement product = left * right;
            Assert.True(product.Transformation == Placement.ProductTransformation(left, right), $"round {round}");
            Assert.True(left.Predivided(right).Transformation == Placement.ProductTransformation(right.Inverted(), left), $"round {round}, predivided");
            if (product.IsIdentity && !left.IsIdentity)
            {
                cancelled++;
            }
        }

        Assert.True(cancelled > 100, $"only {cancelled} products cancelled to the identity");
    }

    [Fact]
    public void A_chain_at_the_recursion_limit_takes_the_product()
    {
        Datum3D[] datums = Datums();
        var random = new Random(3);
        foreach (int length in new[] { 62, 63, 64, 65, 200 })
        {
            Placement left = Chain(random, datums, 10);
            var right = new Placement();
            for (int i = 0; i < length; i++)
            {
                // Alternate two datums so that nothing cancels and the chain is as long as asked
                right *= new Placement(datums[i % 2]);
            }

            Assert.True((left * right).Transformation == Placement.ProductTransformation(left, right), $"length {length}");
        }
    }
}
