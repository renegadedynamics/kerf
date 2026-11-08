// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text;

namespace Kerf.Kernel.DataExchange.Tests.STEPControl;

/// <summary>
/// STEP's write modes by what comes back (unit 12Q, decision 1753): each mode written and read again gives the shape's
/// volume, area or length, with its shells, faces and edges where the mode keeps them, and a mode that can't hold the
/// shape refuses it.
/// </summary>
public class StepWriteModesLawsTests
{
    private static (ReturnStatus Status, string Text) Write(in Shape theShape, ModelType theMode, string theSchema = "AP214IS")
    {
        var aWriter = new Writer();
        aWriter.WS()!.Statics().SetCVal("write.step.schema", theSchema);
        ReturnStatus aStatus = aWriter.Transfer(theShape, theMode);
        if (aStatus != ReturnStatus.RetDone)
        {
            return (aStatus, "");
        }
        using var aStream = new MemoryStream();
        Assert.Equal(ReturnStatus.RetDone, aWriter.WriteStream(aStream));
        return (aStatus, Encoding.UTF8.GetString(aStream.ToArray()));
    }

    private static Shape Read(string theText)
    {
        var aReader = new StepReader();
        Assert.Equal(ReturnStatus.RetDone, aReader.ReadStream("mode.stp", new MemoryStream(Encoding.UTF8.GetBytes(theText))));
        Assert.True(aReader.TransferRoots() > 0);
        return aReader.OneShape();
    }

    private static double Volume(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static double Area(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static double Length(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.LinearProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    // A box of 10 with a cube of 2 hollowed out of its middle: one solid of two shells
    private static Shape HollowBox()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Shape aCavity = new MakeBox(new Point3D(4, 4, 4), 2, 2, 2).Shape();
        Shape aHollow = new Cut(aBox, aCavity).Shape();
        Assert.Equal(1, Count(aHollow, ShapeType.Solid));
        Assert.Equal(2, Count(aHollow, ShapeType.Shell));
        return aHollow;
    }

    [Theory]
    [InlineData((int)ModelType.BrepWithVoids, "BREP_WITH_VOIDS")]
    [InlineData((int)ModelType.FacetedBrepAndBrepWithVoids, "( BREP_WITH_VOIDS(")]
    [InlineData((int)ModelType.AsIs, "BREP_WITH_VOIDS")]
    public void A_hollow_box_comes_back_with_its_void(int theMode, string theEntity)
    {
        (ReturnStatus aStatus, string aText) = Write(HollowBox(), (ModelType)theMode);
        Assert.Equal(ReturnStatus.RetDone, aStatus);
        Assert.Contains(theEntity, aText);
        Shape aRead = Read(aText);
        Assert.Equal(2, Count(aRead, ShapeType.Shell));
        Assert.Equal(12, Count(aRead, ShapeType.Face));
        Assert.Equal(1000 - 8, Volume(aRead), 6);
        Assert.Equal(600 + 24, Area(aRead), 6);
    }

    [Fact]
    public void A_box_written_faceted_comes_back_whole()
    {
        (ReturnStatus aStatus, string aText) = Write(new MakeBox(10, 20, 30).Shape(), ModelType.FacetedBrep);
        Assert.Equal(ReturnStatus.RetDone, aStatus);
        Assert.Contains("FACETED_BREP", aText);
        // a faceted B-rep has no curves on its edges: its loops are polygons of points
        Assert.Contains("POLY_LOOP", aText);
        Assert.DoesNotContain("EDGE_CURVE", aText);
        Shape aRead = Read(aText);
        Assert.Equal(6, Count(aRead, ShapeType.Face));
        Assert.Equal(6000, Volume(aRead), 6);
    }

    [Fact]
    public void A_hollow_box_written_faceted_keeps_its_void()
    {
        // STEP's faceted_brep_and_brep_with_voids is a complex instance of the two
        (ReturnStatus aStatus, string aText) = Write(HollowBox(), ModelType.FacetedBrepAndBrepWithVoids);
        Assert.Equal(ReturnStatus.RetDone, aStatus);
        Assert.Contains("FACETED_BREP()", aText);
        Assert.Equal(1000 - 8, Volume(Read(aText)), 6);
    }

    [Fact]
    public void A_hollow_box_is_no_faceted_brep()
    {
        // a faceted B-rep is one closed shell: OCCT wrote the outer one alone, losing the void (decision 23103)
        (ReturnStatus aStatus, _) = Write(HollowBox(), ModelType.FacetedBrep);
        Assert.NotEqual(ReturnStatus.RetDone, aStatus);
    }

    [Theory]
    [InlineData((int)ModelType.FacetedBrep)]
    [InlineData((int)ModelType.FacetedBrepAndBrepWithVoids)]
    public void A_cylinder_cannot_be_written_faceted(int theMode)
    {
        // its side is no plane and its edges are no lines
        (ReturnStatus aStatus, _) = Write(new ApiMakeCylinder(5, 10).Shape(), (ModelType)theMode);
        Assert.NotEqual(ReturnStatus.RetDone, aStatus);
    }

    [Fact]
    public void An_open_box_written_as_a_surface_model_comes_back_as_its_shell()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        var aSew = new Sewing(1.0e-6);
        int aKept = 0;
        for (var anExp = new Explorer(aBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            if (aKept++ < 5)
            {
                aSew.Add(anExp.Current());
            }
        }
        aSew.Perform();
        Shape anOpen = aSew.SewedShape();
        (ReturnStatus aStatus, string aText) = Write(anOpen, ModelType.ShellBasedSurfaceModel);
        Assert.Equal(ReturnStatus.RetDone, aStatus);
        Assert.Contains("SHELL_BASED_SURFACE_MODEL", aText);
        Assert.Contains("OPEN_SHELL", aText);
        Shape aRead = Read(aText);
        Assert.Equal(5, Count(aRead, ShapeType.Face));
        Assert.Equal(1, Count(aRead, ShapeType.Shell));
        Assert.Equal(500, Area(aRead), 6);
    }

    [Fact]
    public void A_solid_written_as_a_surface_model_keeps_its_area()
    {
        Shape aCylinder = new ApiMakeCylinder(5, 10).Shape();
        (ReturnStatus aStatus, string aText) = Write(aCylinder, ModelType.ShellBasedSurfaceModel);
        Assert.Equal(ReturnStatus.RetDone, aStatus);
        Assert.Contains("CLOSED_SHELL", aText);
        Assert.Equal((2 * Math.PI * 25) + (2 * Math.PI * 5 * 10), Area(Read(aText)), 6);
    }

    private const string Twice = "OCCT's wireframe lists each edge once for every face it bounds, so the set holds every shared edge twice (occt-issues.md, unit 12Q)";

    [Theory(Skip = Twice)]
    [InlineData("AP214IS")]
    [InlineData("AP203")]
    [InlineData("AP242DIS")]
    public void A_box_written_as_curves_comes_back_as_its_edges(string theSchema)
    {
        (ReturnStatus aStatus, string aText) = Write(new MakeBox(10, 20, 30).Shape(), ModelType.GeometricCurveSet, theSchema);
        Assert.Equal(ReturnStatus.RetDone, aStatus);
        Assert.Contains("GEOMETRIC_CURVE_SET", aText);
        Shape aRead = Read(aText);
        Assert.Equal(0, Count(aRead, ShapeType.Face));
        // the twelve edges, each once
        Assert.Equal(4 * (10 + 20 + 30), Length(aRead), 6);
    }

    [Fact(Skip = Twice)]
    public void A_cylinder_written_as_curves_keeps_its_circles()
    {
        (ReturnStatus aStatus, string aText) = Write(new ApiMakeCylinder(5, 10).Shape(), ModelType.GeometricCurveSet);
        Assert.Equal(ReturnStatus.RetDone, aStatus);
        Assert.Contains("CIRCLE", aText);
        // two circles and the seam
        Assert.Equal((2 * 2 * Math.PI * 5) + 10, Length(Read(aText)), 6);
    }
}
