// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Reflection;
using Xunit.Abstractions;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepAlgo::ConcatenateWireC0 on OCCT's bugs/modalg_7/bug29573 wires (unit 7W): two-edge wires whose concatenation
/// goes through GeomConvert_CompCurveToBSplineCurve and is then reversed. The maker's curve was taken through its
/// public, freezing accessor, so the reversal threw Standard_ImmutableObject.
/// </summary>
public class BRepAlgo_ConcatenateWireC0Tests(ITestOutputHelper output)
{
    private static string Metadata(string theKey) =>
        typeof(BRepAlgo_ConcatenateWireC0Tests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static double Length(in Shape theShape)
    {
        var aProps = new Properties();
        BRepGProp.LinearProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void The_two_edge_wires_of_bug29573_concatenate_into_one_edge()
    {
        string aFile = Path.Combine(Metadata("OcctTestDataDir"), "brep", "bug29573.brep");
        if (!File.Exists(aFile))
        {
            output.WriteLine($"{aFile} isn't there: nothing to do");
            return;
        }
        Assert.True(BrepTools.Read(out Shape aShape, aFile));
        aShape.TShape()!.FreezeGraph(); // a caller's shape
        string aBefore = BRepTextOf(aShape);

        int aWires = 0;
        for (var anIt = new Iterator(aShape); anIt.More(); anIt.Next())
        {
            Wire aWire = ShapeCasts.Wire(anIt.Value());
            Edge anEdge = BRepAlgo.ConcatenateWireC0(aWire);
            Assert.False(anEdge.IsNull);
            // checknbshapes r -vertex 2 -edge 1
            var aVertices = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(anEdge, ShapeType.Vertex, aVertices);
            Assert.Equal(2, aVertices.Extent());
            // the same path: its length, and its ends at the wire's ends
            Assert.Equal(Length(aWire), Length(anEdge), Length(aWire) * 1e-6);
            Exploration.Vertices(aWire, out Vertex aV1, out Vertex aV2);
            Exploration.Vertices(anEdge, out Vertex aE1, out Vertex aE2);
            Assert.True(Tool.Pnt(aV1).Distance(Tool.Pnt(aE1)) < 1e-6 || Tool.Pnt(aV1).Distance(Tool.Pnt(aE2)) < 1e-6);
            Assert.True(Tool.Pnt(aV2).Distance(Tool.Pnt(aE1)) < 1e-6 || Tool.Pnt(aV2).Distance(Tool.Pnt(aE2)) < 1e-6);
            aWires++;
        }
        Assert.Equal(2, aWires);
        Assert.Equal(aBefore, BRepTextOf(aShape));
    }

    private static string BRepTextOf(in Shape theShape)
    {
        var aWriter = new StringWriter();
        BrepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }
}
