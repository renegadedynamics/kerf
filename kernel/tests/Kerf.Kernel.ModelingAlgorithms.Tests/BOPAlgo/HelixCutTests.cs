// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// #1543: a cut by a tube swept along a helix, leaving the cylinder through its end. Where the tube leaves relative to the
/// cylinder's seam decided whether anything was removed: the walking line's last piece lost the vertex on the face's bound
/// when it was split at the seam, and went out whole because its middle is past the bound (decision 1834).
/// </summary>
public class HelixCutTests(ITestOutputHelper output)
{
    private const double Pitch = 2, Height = 15, Radius = 8, TubeRadius = 0.6;

    private static double Volume(in Shape theShape)
    {
        Properties aProps = new();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static Shape Tube(double theDz)
    {
        var aCylinder = new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), Radius);
        double aSlope = Pitch / (2 * Math.PI);
        double aLength = Height / Pitch * 2 * Math.PI * Math.Sqrt(1 + aSlope * aSlope);
        var aLine = new LineCurve2D(new Point2D(0, 0), new Direction2D(1, aSlope));
        Wire aSpine = new MakeWire(new MakeEdge(aLine, aCylinder, 0, aLength).InternalEdge()).InternalWire();
        Assert.True(BrepLibrary.BuildCurves3d(aSpine, 1e-6, Continuity.C1, 14, 2000));

        var aCurve = new WireCurveAdaptor(aSpine);
        aCurve.D1(aCurve.FirstParameter(), out Point3D aStart, out Vector3D aTangent);
        var aT = new Direction3D(aTangent);
        var anX = new Direction3D(new Vector3D(aT.X, aT.Y, aT.Z).Crossed(new Vector3D(0, 0, 1)));
        Wire aProfile = new MakeWire(new MakeEdge(new Circle3D(new Frame3D(aStart, aT, anX), TubeRadius)).Edge()).Wire();

        var aPipe = new MakePipeShell(aSpine);
        aPipe.SetMode(new Direction3D(0, 0, 1));
        aPipe.SetTransitionMode(Kerf.Construction.TransitionMode.RoundCorner);
        aPipe.Add(aProfile);
        aPipe.Build();
        Assert.True(aPipe.IsDone);
        aPipe.MakeSolid();

        Transform3D aMove = new Transform3D().WithTranslation(new Vector3D(0, 0, theDz));
        return new Transform(aPipe.Shape(), aMove, true).Shape();
    }

    // The volumes the issue gives for the three that worked, and the one it expects for 16.2, within what it says the fix moves
    [Theory]
    [InlineData(15.4, 5830.86)]
    [InlineData(15.5, 5832.39)]
    [InlineData(16.0, 5839.06)]
    [InlineData(16.2, 5841.80)]
    [InlineData(16.4, double.NaN)]
    [InlineData(16.6, double.NaN)]
    public void The_cut_removes_the_groove_wherever_the_tube_leaves(double theDz, double theExpected)
    {
        Shape aCylinder = new ApiMakeCylinder(Radius, 30).Shape();
        double aWhole = Volume(aCylinder);
        Shape aTube = Tube(theDz);
        double aCut = Volume(new Cut(aCylinder, aTube).Shape());
        double aCommon = Volume(new Common(aCylinder, aTube).Shape());
        output.WriteLine($"dz {theDz}: tube {Volume(aTube):F3}, cut {aCut:F3}, common {aCommon:F3}, whole {aWhole:F3}");

        Assert.True(aWhole - aCut > 150, $"removed only {aWhole - aCut}");
        if (!double.IsNaN(theExpected))
        {
            Assert.Equal(theExpected, aCut, 0.1);
        }

        Assert.Equal(aWhole, aCut + aCommon, 1.0);
    }
}
