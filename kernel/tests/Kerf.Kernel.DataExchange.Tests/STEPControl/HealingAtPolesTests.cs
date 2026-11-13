// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Kernel.DataExchange.Tests.StepGen;
using Kerf.Topology;

namespace Kerf.Kernel.DataExchange.Tests.STEPControl;

/// <summary>
/// The healing at a sphere's pole on a file of OCCT's corpus (decision 34704): <c>trj3_s1-db-214.stp</c> has two faces
/// on spheres of radius 6, mirror images of each other across y = 0. OCCT 8.0.1's line through the pole gave the second
/// a pcurve that went round the other way, so the face was the rest of the sphere: 344.5015, which with its twin's
/// 107.888 is the whole sphere, 4 pi 36 = 452.389. Now it is its twin's mirror image.
/// </summary>
public class HealingAtPolesTests
{
    private static (double Area, Point3D Centre) Measure(in Shape theFace)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(theFace, aProps);
        return (aProps.Mass(), aProps.CentreOfMass());
    }

    [Fact]
    public void A_face_on_a_sphere_through_its_pole_is_its_twins_mirror_image_not_the_rest_of_the_sphere()
    {
        string? aPath = StepRoundTripTests.CorpusFiles().SingleOrDefault(f => Path.GetFileName(f) == "trj3_s1-db-214.stp");
        if (aPath is null)
        {
            return; // OCCT's dataset not fetched
        }
        var aReader = new StepReader();
        Assert.Equal(ReturnStatus.RetDone, aReader.ReadFile(aPath));
        aReader.TransferRoots();
        var aOnSpheres = new List<(double Area, Point3D Centre)>();
        for (Explorer aFaces = new(aReader.OneShape(), ShapeType.Face); aFaces.More(); aFaces.Next())
        {
            Face aFace = ShapeCasts.Face(aFaces.Current());
            // the faces on spheres of radius 6 about (15, 0, 0): the twins and, in 8.0.1, the rest of the second's sphere
            if (Tool.Surface(aFace) is SphericalSurface { Radius: 6.0 } && Measure(aFace) is var aMeasure
                && Math.Abs(aMeasure.Centre.X - 16.0) < 4.0 && Math.Abs(aMeasure.Centre.Y) < 4.0 && Math.Abs(aMeasure.Centre.Z) < 1.0)
            {
                aOnSpheres.Add(aMeasure);
            }
        }

        Assert.DoesNotContain(aOnSpheres, f => f.Area > 2.0 * Math.PI * 36.0); // more than half the sphere
        (double Area, Point3D Centre) anUpper = Assert.Single(aOnSpheres, f => Math.Abs(f.Centre.X - 15.04) < 0.5 && Math.Abs(f.Centre.Y - 3.11) < 0.5);
        (double Area, Point3D Centre) aLower = Assert.Single(aOnSpheres, f => Math.Abs(f.Centre.X - 15.04) < 0.5 && Math.Abs(f.Centre.Y + 3.11) < 0.5);
        Assert.Equal(107.888, anUpper.Area, 1e-3);
        Assert.Equal(anUpper.Area, aLower.Area, 2e-3);
        Assert.True(aLower.Centre.IsEqual(new Point3D(anUpper.Centre.X, -anUpper.Centre.Y, anUpper.Centre.Z), 1e-4),
                    $"{anUpper.Centre} and {aLower.Centre}");
    }
}
