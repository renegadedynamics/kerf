// Ported from build123d tests/test_direct_api/test_mass_properties.py. Nothing omitted; Sphere, Box and Cylinder
// (centred) are built with the direct API.
namespace Build123d.Tests.Topology;

public class MassPropertiesTests
{
    [Fact]
    public void Sphere()
    {
        double r = 2;   // Sphere radius
        var sphere = CompoundObjects.Sphere(r);

        // Expected mass properties
        var volume = 4.0 / 3 * Math.PI * Math.Pow(r, 3);
        var expectedInertia = 2.0 / 5 * volume * r * r;   // Ixx = Iyy = Izz

        // Test static moments (should be zero if centered at origin)
        var (mx, my, mz) = sphere.StaticMoments;
        Check.AlmostEqual(0, mx, 5);
        Check.AlmostEqual(0, my, 5);
        Check.AlmostEqual(0, mz, 5);

        // Test matrix of inertia (diagonal and equal for a sphere)
        var inertiaMatrix = sphere.MatrixOfInertia;
        Check.AlmostEqual(expectedInertia, inertiaMatrix[0][0], 5);
        Check.AlmostEqual(expectedInertia, inertiaMatrix[1][1], 5);
        Check.AlmostEqual(expectedInertia, inertiaMatrix[2][2], 5);

        // Test principal properties (should match matrix of inertia)
        var principalMoments = sphere.PrincipalProperties.Select(p => p.Moment).ToList();
        Check.AlmostEqual(expectedInertia, principalMoments[0], 5);
        Check.AlmostEqual(expectedInertia, principalMoments[1], 5);
        Check.AlmostEqual(expectedInertia, principalMoments[2], 5);

        // Test radius of gyration (should be sqrt(2/5) * r)
        var expectedRadiusOfGyration = Math.Sqrt(2.0 / 5) * r;
        Check.AlmostEqual(expectedRadiusOfGyration, sphere.RadiusOfGyration(Axis.X), 5);
    }

    [Fact]
    public void Cube()
    {
        double side = 2;
        var cube = CompoundObjects.Box(side, side, side);

        // Expected values
        var volume = Math.Pow(side, 3);
        var expectedInertia = 1.0 / 6 * volume * side * side;   // Ixx = Iyy = Izz

        // Test inertia matrix (should be diagonal)
        var inertiaMatrix = cube.MatrixOfInertia;
        Check.AlmostEqual(expectedInertia, inertiaMatrix[0][0], 5);
        Check.AlmostEqual(expectedInertia, inertiaMatrix[1][1], 5);
        Check.AlmostEqual(expectedInertia, inertiaMatrix[2][2], 5);

        // Test principal moments (should be equal)
        var principalMoments = cube.PrincipalProperties.Select(p => p.Moment).ToList();
        Check.AlmostEqual(expectedInertia, principalMoments[0], 5);
        Check.AlmostEqual(expectedInertia, principalMoments[1], 5);
        Check.AlmostEqual(expectedInertia, principalMoments[2], 5);

        // Test radius of gyration (should be sqrt(1/6) * side)
        var expectedRadiusOfGyration = Math.Sqrt(1.0 / 6) * side;
        Check.AlmostEqual(expectedRadiusOfGyration, cube.RadiusOfGyration(Axis.X), 5);
    }

    [Fact]
    public void Cylinder()
    {
        double r = 2, h = 5;
        var cylinder = CompoundObjects.Cylinder(r, h);

        // Expected values
        var volume = Math.PI * r * r * h;
        var expectedInertiaXX = 1.0 / 12 * volume * (3 * r * r + h * h);   // Ixx = Iyy
        var expectedInertiaZZ = 1.0 / 2 * volume * r * r;   // Iz about Z-axis

        // Test principal moments (should align with Z)
        var principalMoments = cylinder.PrincipalProperties.Select(p => p.Moment).ToList();
        Check.AlmostEqual(expectedInertiaXX, principalMoments[0], 5);
        Check.AlmostEqual(expectedInertiaXX, principalMoments[1], 5);
        Check.AlmostEqual(expectedInertiaZZ, principalMoments[2], 5);

        // Test radius of gyration (should be sqrt(I/m))
        var expectedRadiusX = Math.Sqrt(expectedInertiaXX / volume);
        var expectedRadiusZ = Math.Sqrt(expectedInertiaZZ / volume);
        Check.AlmostEqual(expectedRadiusX, cylinder.RadiusOfGyration(Axis.X), 5);
        Check.AlmostEqual(expectedRadiusZ, cylinder.RadiusOfGyration(Axis.Z), 5);
    }
}

/// <summary>An empty shape has zero of every amount, but no place or direction.</summary>
public class EmptyShapeMassPropertiesTests
{
    [Fact]
    public void MatrixOfInertia()
    {
        var matrix = new Solid().MatrixOfInertia;
        Assert.Equal(3, matrix.Length);
        Assert.All(matrix, row => Assert.Equal([0.0, 0.0, 0.0], row));
    }

    [Fact]
    public void PrincipalProperties()
    {
        var error = Assert.Throws<InvalidOperationException>(() => new Solid().PrincipalProperties);
        Assert.Contains("empty Solid has no principal axes", error.Message);
    }

    [Fact]
    public void StaticMoments() => Assert.Equal((0.0, 0.0, 0.0), new Solid().StaticMoments);

    [Fact]
    public void RadiusOfGyration()
    {
        var error = Assert.Throws<InvalidOperationException>(() => new Solid().RadiusOfGyration(Axis.Z));
        Assert.Contains("empty Solid has no radius", error.Message);
    }

    [Fact]
    public void Orientation()
    {
        var error = Assert.Throws<InvalidOperationException>(() => new Solid().Orientation);
        Assert.Contains("empty Solid has no orientation", error.Message);
    }
}
