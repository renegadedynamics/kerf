// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The point a walking line ends at on the border of its domain (decision 2429): made from the function evaluated there,
/// not from whatever the function last evaluated, which OCCT takes and which the solver can leave elsewhere.
/// </summary>
public class IWalkingBorderPointTests
{
    // OCCT #1496's tubes, swept along a corner filleted at 24 and at 30: the first's straight run, a cylinder of radius 12
    // along X for 36, and the second's bend, a torus of 30 by 12 starting at X = 30, tangent to the cylinder along its circle
    private static Face Cylinder()
    {
        CylindricalSurface surface = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), new Direction3D(0, 1, 0)), 12);
        return new MakeFace(surface, 0, 2 * Math.PI, 0, 36, 1e-7).Face();
    }

    private static Face Torus()
    {
        ToroidalSurface surface = new(new CoordinateSystem(new Point3D(30, 0, 30), new Direction3D(0, -1, 0), new Direction3D(0, 0, -1)), 30, 12);
        return new MakeFace(surface, 0, Math.PI / 2, 1.5 * Math.PI, 3.5 * Math.PI, 1e-7).Face();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void A_line_ending_on_a_tangent_border_ends_on_both_surfaces(bool torusFirst)
    {
        Face cylinder = Cylinder(), torus = Torus();
        FaceFace intersection = new();
        intersection.SetParameters(true, true, true, 1.0e-7);
        intersection.Perform(torusFirst ? torus : cylinder, torusFirst ? cylinder : torus);
        Assert.True(intersection.IsDone);
        Assert.True(intersection.Lines().Length() > 0);
        // Before, one branch's last point was a quarter turn round the tangent circle from where it reached the border, and
        // its curve was approximated 16.97 off both surfaces
        for (int i = 1; i <= intersection.Lines().Length(); i++)
        {
            Assert.True(intersection.Lines().Value(i).Tolerance() < 1e-5, $"curve {i}: {intersection.Lines().Value(i).Tolerance()}");
        }
    }
}
