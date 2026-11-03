// Differential tests: results from Python build123d (tests/reference/geometry_reference.py) recomputed
// by the port. These hold the port to build123d's numbers far more tightly than the ported unit tests.
using System.Text.Json;
using Xunit.Sdk;

namespace Build123d.Tests.Geometry;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class ReferenceTests
{
    static readonly JsonElement Data = JsonDocument.Parse(
        File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "Reference", "geometry.json"))).RootElement;

    static IEnumerable<JsonElement> Cases(string name) => Data.GetProperty(name).EnumerateArray();

    static Vector V(JsonElement e) => new(e[0].GetDouble(), e[1].GetDouble(), e[2].GetDouble());

    static RotationOrder Order(JsonElement c) =>
        c.GetProperty("kind").GetString() == "Intrinsic"
            ? Enum.Parse<Intrinsic>(c.GetProperty("order").GetString()!)
            : Enum.Parse<Extrinsic>(c.GetProperty("order").GetString()!);

    static void Close(Vector expected, Vector actual, double tolerance, string what)
    {
        for (int i = 0; i < 3; i++)
            if (Math.Abs(expected[i] - actual[i]) > tolerance * Math.Max(1, Math.Abs(expected[i])))
                throw new XunitException($"{what}: expected {expected.ToString("R")} got {actual.ToString("R")}");
    }

    // Euler angles can legitimately differ at a gimbal-lock singularity; compare the rotation instead
    static void SameRotation(Vector expectedAngles, Vector actualAngles, string what)
    {
        var e = new Plane(new Location((0, 0, 0), expectedAngles));
        var a = new Plane(new Location((0, 0, 0), actualAngles));
        Close(e.XDir, a.XDir, 1e-9, what + " x");
        Close(e.ZDir, a.ZDir, 1e-9, what + " z");
    }

    static void SamePlane(JsonElement expected, Plane actual, double tolerance, string what)
    {
        Close(V(expected.GetProperty("origin")), actual.Origin, tolerance, what + " origin");
        Close(V(expected.GetProperty("x")), actual.XDir, tolerance, what + " x");
        Close(V(expected.GetProperty("y")), actual.YDir, tolerance, what + " y");
        Close(V(expected.GetProperty("z")), actual.ZDir, tolerance, what + " z");
    }

    [Fact]
    public void LocationsFromEulerAngles()
    {
        foreach (var c in Cases("location_euler"))
        {
            var loc = new Location(V(c.GetProperty("pos")), V(c.GetProperty("angles")), Order(c));
            string what = $"{Order(c)} {V(c.GetProperty("angles"))}";
            Close(V(c.GetProperty("position")), loc.Position, 1e-12, what);
            Close(V(c.GetProperty("orientation")), loc.Orientation, 1e-9, what);
            SamePlane(c.GetProperty("plane"), new Plane(loc), 1e-12, what);
        }
    }

    [Fact]
    public void LocationOperations()
    {
        foreach (var c in Cases("location_ops"))
        {
            var a = new Location(V(c.GetProperty("a_pos")), V(c.GetProperty("a_orient")));
            var b = new Location(V(c.GetProperty("b_pos")), V(c.GetProperty("b_orient")));
            int n = c.GetProperty("n").GetInt32();
            void Same(string name, Location actual)
            {
                var expected = c.GetProperty(name);
                Close(V(expected[0]), actual.Position, 1e-9, name);
                SameRotation(V(expected[1]), actual.Orientation, name);
            }
            Same("product", a * b);
            Same("inverse", a.Inverse());
            Same("power", a.Pow(n));
            Same("negated", -a);
        }
    }

    [Fact]
    public void MirroredLocations()
    {
        foreach (var c in Cases("location_mirror"))
        {
            var loc = new Location(V(c.GetProperty("pos")), V(c.GetProperty("orient")));
            var mirrored = loc.Mirror(new Plane(V(c.GetProperty("plane_origin")), zDir: V(c.GetProperty("plane_z"))));
            Close(V(c.GetProperty("position")), mirrored.Position, 1e-9, "mirror position");
            SameRotation(V(c.GetProperty("orientation")), mirrored.Orientation, "mirror orientation");
        }
    }

    [Fact]
    public void PlanesFromMirroringTransformations()
    {
        foreach (var c in Cases("plane_from_mirror"))
        {
            var p = V(c.GetProperty("point")).ToGpPnt();
            var d = V(c.GetProperty("dir")).ToGpDir();
            var ax2 = new Kerf.Geometry.Frame3D(p, d);
            var trsf = new Kerf.Geometry.Transform3D();
            trsf = trsf.WithMirror(ax2);
            SamePlane(c.GetProperty("plane"), new Plane(new Location(trsf)), 1e-12, "mirror plane");
        }
    }

    [Fact]
    public void PlanesFromDirections()
    {
        foreach (var c in Cases("plane_dirs"))
        {
            var o = V(c.GetProperty("origin"));
            var z = V(c.GetProperty("z"));
            SamePlane(c.GetProperty("plane"), new Plane(o, V(c.GetProperty("x")), z), 1e-12, "plane");
            SamePlane(c.GetProperty("no_x"), new Plane(o, zDir: z), 1e-12, "plane without x");
        }
    }

    [Fact]
    public void RotatedPlanes()
    {
        foreach (var c in Cases("plane_rotated"))
        {
            var p = new Plane(V(c.GetProperty("origin")), V(c.GetProperty("x")), V(c.GetProperty("z")))
                .Rotated(V(c.GetProperty("angles")), Order(c));
            SamePlane(c.GetProperty("plane"), p, 1e-12, $"rotated {Order(c)}");
        }
    }

    [Fact]
    public void VectorAngles()
    {
        foreach (var c in Cases("vector_angles"))
        {
            var a = V(c.GetProperty("a"));
            var b = V(c.GetProperty("b"));
            Assert.Equal(c.GetProperty("angle").GetDouble(), a.GetAngle(b), 12);
            Assert.Equal(c.GetProperty("signed").GetDouble(), a.GetSignedAngle(b), 12);
            Assert.Equal(c.GetProperty("signed_n").GetDouble(), a.GetSignedAngle(b, V(c.GetProperty("n"))), 12);
        }
    }

    [Fact]
    public void AxisIntersections()
    {
        foreach (var c in Cases("axis_intersect"))
        {
            var hit = new Axis(V(c.GetProperty("p1")), V(c.GetProperty("d1"))) & new Axis(V(c.GetProperty("p2")), V(c.GetProperty("d2")));
            Close(V(c.GetProperty("point")), Assert.IsType<Vector>(hit), 1e-9, "axis intersection");
        }
    }

    static double[] Doubles(JsonElement e) => e.EnumerateArray().Select(x => x.GetDouble()).ToArray();

    [Fact]
    public void ColorsFromComponents()
    {
        foreach (var c in Cases("colors_rgb"))
        {
            var input = Doubles(c.GetProperty("in"));
            var color = new Color(input[0], input[1], input[2], input[3]);
            Assert.Equal(Doubles(c.GetProperty("out")), Check.Of(color));
            Assert.Equal(c.GetProperty("str").GetString(), color.ToString());
        }
    }

    [Fact]
    public void ColorsFromNames()
    {
        foreach (var c in Cases("colors_named"))
        {
            var color = new Color(c.GetProperty("name").GetString()!);
            Assert.Equal(Doubles(c.GetProperty("out")), Check.Of(color));
            Assert.Equal(c.GetProperty("str").GetString(), color.ToString());
        }
    }

    [Fact]
    public void CategoricalSets()
    {
        foreach (var c in Cases("colors_categorical"))
        {
            var colors = Color.CategoricalSet(c.GetProperty("count").GetInt32(), c.GetProperty("hue").GetDouble());
            var expected = c.GetProperty("out").EnumerateArray().Select(Doubles).ToArray();
            Assert.Equal(expected.Length, colors.Count);
            for (int i = 0; i < expected.Length; i++)
                Assert.Equal(expected[i], Check.Of(colors[i]));
        }
    }
}
