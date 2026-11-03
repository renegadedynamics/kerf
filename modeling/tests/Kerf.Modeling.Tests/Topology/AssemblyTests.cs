// Ported from build123d tests/test_direct_api/test_assembly.py. The assembly is built with Compound.Assembly, its
// children placed by WithOrientation/WithPosition, in place of setting .parent.
// Changed:
// - attributes: ported without the .parent checks; children hold no parent link.
using System.Text.RegularExpressions;

namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class AssemblyTests
{
    static Compound CreateTestAssembly()
    {
        var box = Solid.MakeBox(1, 1, 1).WithOrientation((45, 45, 0)).WithLabel("box");
        var sphere = Solid.MakeSphere(1).WithLabel("sphere").WithPosition((1, 2, 3));
        return Compound.Assembly([box, sphere], label: "assembly");
    }

    /// <summary>Each line matches apart from the address, as Python's assertTopoEqual.</summary>
    static void AssertTopoEqual(string actualTopo, string[] expectedTopoLines)
    {
        var actualTopoLines = actualTopo.Split('\n', StringSplitOptions.RemoveEmptyEntries);
        Assert.Equal(expectedTopoLines.Length, actualTopoLines.Length);
        foreach (var (actualLine, expectedLine) in actualTopoLines.Zip(expectedTopoLines))
        {
            var parts = Regex.Split(expectedLine, "at 0x[0-9a-f]+,", RegexOptions.IgnoreCase);
            Assert.Equal(2, parts.Length);
            Assert.StartsWith(parts[0], actualLine);
            Assert.EndsWith(parts[1], actualLine);
        }
    }

    [Fact]
    public void ShowTopologyCompound()
    {
        var assembly = CreateTestAssembly();
        string[] expected =
        [
            "assembly   Compound at 0x7fced0fd1b50, Location((0, 0, 0), (0, 0, 0))",
            "├── box    Solid    at 0x7fced102d3a0, Location((0, 0, 0), (45, 45, 0))",
            "└── sphere Solid    at 0x7fced0fd1f10, Location((1, 2, 3), (0, 0, 0))",
        ];
        AssertTopoEqual(assembly.ShowTopology(ShapeType.Solid), expected);
    }

    [Fact]
    public void ShowTopologyShapeLocation()
    {
        var assembly = CreateTestAssembly();
        string[] expected =
        [
            "Solid        at 0x7f3754501530, Position(1, 2, 3)",
            "└── Shell    at 0x7f3754501a70, Position(1, 2, 3)",
            "    └── Face at 0x7f3754501030, Position(1, 2, 3)",
        ];
        AssertTopoEqual(assembly.AssemblyChildren[1].ShowTopology(ShapeType.Face, showCenter: false), expected);
    }

    [Fact]
    public void ShowTopologyShape()
    {
        var assembly = CreateTestAssembly();
        string[] expected =
        [
            "Solid        at 0x7f6279043ab0, Center(1, 2, 3)",
            "└── Shell    at 0x7f62790438f0, Center(1, 2, 3)",
            "    └── Face at 0x7f62790439f0, Center(1, 2, 3)",
        ];
        AssertTopoEqual(assembly.AssemblyChildren[1].ShowTopology(ShapeType.Face), expected);
    }

    [Fact]
    public void Attributes()
    {
        var box = Solid.MakeBox(1, 1, 1).WithLabel("box");
        var sphere = Solid.MakeSphere(1).WithLabel("sphere");
        var assembly = Compound.Assembly([box, sphere], label: "assembly");

        Assert.Equal("box", box.Label);
        Assert.Equal(2, assembly.AssemblyChildren.Count);
        Assert.Same(box, assembly.AssemblyChildren[0]);
        Assert.Same(sphere, assembly.AssemblyChildren[1]);
    }

    [Fact]
    public void RemoveChild()
    {
        var assembly = CreateTestAssembly();
        Assert.Equal(2, assembly.AssemblyChildren.Count);
        assembly = assembly.WithAssemblyChildren(assembly.AssemblyChildren.Skip(1));
        Assert.Single(assembly.AssemblyChildren);
    }

    [Fact]
    public void DoChildrenIntersect()
    {
        var (overlap, _, _) = CreateTestAssembly().DoChildrenIntersect();
        Assert.False(overlap);
        var box = Solid.MakeBox(1, 1, 1).WithOrientation((45, 45, 0)).WithLabel("box");
        var sphere = Solid.MakeSphere(1).WithLabel("sphere").WithPosition((0, 0, 0));
        var assembly = Compound.Assembly([box, sphere], label: "assembly");
        (overlap, _, _) = assembly.DoChildrenIntersect();
        Assert.True(overlap);
    }
}
