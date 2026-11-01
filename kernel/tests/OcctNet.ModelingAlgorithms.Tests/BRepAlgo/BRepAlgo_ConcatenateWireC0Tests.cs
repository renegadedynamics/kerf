// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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

    private static double Length(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
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
        Assert.True(BRepTools.Read(out TopoDS_Shape aShape, aFile));
        aShape.TShape()!.FreezeGraph(); // a caller's shape
        string aBefore = BRepTextOf(aShape);

        int aWires = 0;
        for (var anIt = new TopoDS_Iterator(aShape); anIt.More(); anIt.Next())
        {
            TopoDS_Wire aWire = TopoDS.Wire(anIt.Value());
            TopoDS_Edge anEdge = BRepAlgo.ConcatenateWireC0(aWire);
            Assert.False(anEdge.IsNull());
            // checknbshapes r -vertex 2 -edge 1
            var aVertices = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(anEdge, TopAbs_ShapeEnum.TopAbs_VERTEX, aVertices);
            Assert.Equal(2, aVertices.Extent());
            // the same path: its length, and its ends at the wire's ends
            Assert.Equal(Length(aWire), Length(anEdge), Length(aWire) * 1e-6);
            TopExp.Vertices(aWire, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
            TopExp.Vertices(anEdge, out TopoDS_Vertex aE1, out TopoDS_Vertex aE2);
            Assert.True(BRep_Tool.Pnt(aV1).Distance(BRep_Tool.Pnt(aE1)) < 1e-6 || BRep_Tool.Pnt(aV1).Distance(BRep_Tool.Pnt(aE2)) < 1e-6);
            Assert.True(BRep_Tool.Pnt(aV2).Distance(BRep_Tool.Pnt(aE1)) < 1e-6 || BRep_Tool.Pnt(aV2).Distance(BRep_Tool.Pnt(aE2)) < 1e-6);
            aWires++;
        }
        Assert.Equal(2, aWires);
        Assert.Equal(aBefore, BRepTextOf(aShape));
    }

    private static string BRepTextOf(in TopoDS_Shape theShape)
    {
        var aWriter = new StringWriter();
        BRepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }
}
