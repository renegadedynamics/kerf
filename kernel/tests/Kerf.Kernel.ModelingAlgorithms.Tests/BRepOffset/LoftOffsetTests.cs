// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Offsets.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.JoinKind;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// A loft from a square to a circle offset outwards with round joins (unit 15O). At each top corner the two side faces are
/// tangent, though their edge isn't, so the tubes meet there with no gap; OCCT 8.0.1 still makes a sphere there, a face of
/// no area whose wire the check finds the wrong way round, and the result was invalid at every distance. The spheres are
/// left out and the tubes sewn.
/// </summary>
public class LoftOffsetTests
{
    private static Shape Loft()
    {
        var aSquare = new MakePolygon(new Point3D(-5, -5, 0), new Point3D(5, -5, 0), new Point3D(5, 5, 0), new Point3D(-5, 5, 0), true);
        var aCircle = new MakeWire();
        aCircle.Add(new MakeEdge(new Circle3D(new Frame3D(new Point3D(0, 0, 10), new Direction3D(0, 0, 1)), 4)).Edge());
        var aLoft = new ThruSections(true, false);
        aLoft.AddWire(aSquare.Wire());
        aLoft.AddWire(aCircle.Wire());
        aLoft.Build();
        return aLoft.Shape();
    }

    private static double Volume(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps, 1.0e-9);
        return aProps.Mass();
    }

    private static double Area(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(theShape, aProps, 1.0e-9);
        return aProps.Mass();
    }

    private static Shape Offset(Shape theLoft, double theOffset) => Made(theLoft, theOffset).Shape();

    private static MakeThickSolid Made(Shape theLoft, double theOffset)
    {
        var anAlgo = new MakeThickSolid();
        anAlgo.MakeThickSolidByJoin(theLoft, new NCollection_List<Shape>(), theOffset, 1.0e-4, global::Kerf.Offsets.OffsetMode.Skin,
                                    true, false, Arc, true);
        Assert.True(anAlgo.IsDone, anAlgo.MakeOffset().Error().ToString());
        return anAlgo;
    }

    [Theory]
    [InlineData(0.02)]
    [InlineData(0.5)]
    [InlineData(5.0)]
    public void Outwards_it_is_valid_closed_and_has_no_face_of_no_area(double theOffset)
    {
        Shape aLoft = Loft();
        MakeThickSolid anAlgo = Made(aLoft, theOffset);
        Shape aResult = anAlgo.Shape();

        Assert.True(new Analyzer(aResult).IsValid());
        Assert.True(BRepOffset_MakeOffsetTests.SubShapes(aResult, ShapeType.Shell).All(s => Tool.IsClosed(s)));
        foreach (Shape aFace in BRepOffset_MakeOffsetTests.SubShapes(aResult, ShapeType.Face))
        {
            Assert.True(Area(aFace) > 1.0e-6);
        }
        // four tubes end in a point where their faces are tangent
        Assert.Equal(4, BRepOffset_MakeOffsetTests.SubShapes(aResult, ShapeType.Edge).Count(e => Tool.Degenerated(ShapeCasts.Edge(e))));
        Assert.Empty(EdgeCrossings.All(aLoft, aResult));
        // the history follows the sewing: each of the loft's faces generated one face of the result, and each of its
        // edges but the one along which its faces are tangent (of 15) a tube
        List<Shape> aResultFaces = BRepOffset_MakeOffsetTests.SubShapes(aResult, ShapeType.Face);
        foreach (Shape aFace in BRepOffset_MakeOffsetTests.SubShapes(aLoft, ShapeType.Face))
        {
            Shape aGenerated = Assert.Single(anAlgo.Generated(aFace));
            Assert.Contains(aResultFaces, f => f.IsSame(aGenerated));
        }
        List<Shape> aLoftEdges = BRepOffset_MakeOffsetTests.SubShapes(aLoft, ShapeType.Edge);
        Assert.Equal(15, aLoftEdges.Count);
        List<Shape> aTubes = [.. aLoftEdges.SelectMany(e => anAlgo.Generated(e))];
        Assert.Equal(14, aTubes.Count);
        Assert.All(aTubes, t => Assert.Contains(aResultFaces, f => f.IsSame(t)));
    }

    [Fact]
    public void Outwards_its_volume_follows_Steiners_formula()
    {
        // The loft is convex, so its offset by d has the volume V + A d + M d^2 + 4/3 pi d^3, M fixed: M from each distance
        // must agree, and with the mean curvature a fine mesh of the loft gives (84.4535, at deflections 0.01 to 0.0005)
        Shape aLoft = Loft();
        double aV = Volume(aLoft), anA = Area(aLoft);
        foreach (double aD in new[] { 0.02, 0.5, 1.0, 5.0 })
        {
            double aVolume = Math.Abs(Volume(Offset(aLoft, aD)));
            double aM = (aVolume - aV - anA * aD - 4.0 / 3.0 * Math.PI * aD * aD * aD) / (aD * aD);
            Assert.Equal(84.4535, aM, aD < 0.1 ? 0.01 : 1.0e-4);
        }
    }

    [Fact]
    public void Inwards_the_result_is_as_before()
    {
        Shape aLoft = Loft();
        Shape aResult = Offset(aLoft, -0.5);
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.Equal(0, BRepOffset_MakeOffsetTests.SubShapes(aResult, ShapeType.Face).Count(f => new FaceSurfaceAdaptor(ShapeCasts.Face(f)).GetType() == SurfaceKind.Sphere));
    }
}
