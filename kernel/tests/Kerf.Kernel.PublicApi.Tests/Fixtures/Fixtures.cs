// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Exchange;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// Shapes for the tests of measurement, healing, meshing and exchange (unit 10L3), read with the public <see cref="Brep"/>
/// reader from BREP files embedded beside: a box of 10 × 20 × 30 at the origin, a cylinder of radius 5 and height 10 on
/// the Z axis, and two boxes of 10 × 10 × 10 side by side, fused without their faces unified (10 faces).
/// </summary>
internal static class Fixtures
{
    public static Solid Box() => Load("box").As<Solid>();

    public static Solid Cylinder() => Load("cylinder").As<Solid>();

    public static Shape TwoBoxesFused() => Load("two-boxes-fused");

    /// <summary>The shape of the fixture <paramref name="name"/>.brep.</summary>
    public static Shape Named(string name) => Load(name);

    public static byte[] Bytes(string name)
    {
        using Stream stream = typeof(Fixtures).Assembly.GetManifestResourceStream($"Fixtures.{name}.brep")
                              ?? throw new InvalidOperationException($"No fixture {name}");
        using var memory = new MemoryStream();
        stream.CopyTo(memory);
        return memory.ToArray();
    }

    /// <summary>An embedded file of Fixtures (the golden pictures, unit 10L4).</summary>
    public static byte[] Resource(string fileName)
    {
        using Stream stream = typeof(Fixtures).Assembly.GetManifestResourceStream($"Fixtures.{fileName}")
                              ?? throw new InvalidOperationException($"No fixture {fileName}");
        using var memory = new MemoryStream();
        stream.CopyTo(memory);
        return memory.ToArray();
    }

    private static Shape Load(string name) => Brep.Read(new MemoryStream(Bytes(name))).Value;
}
