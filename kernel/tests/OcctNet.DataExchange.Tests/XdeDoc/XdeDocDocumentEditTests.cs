// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static OcctNet.DataExchange.Tests.MeshTestKit;

namespace OcctNet.DataExchange.Tests;

/// <summary>
/// Unit 8G1's additions to XdeDoc_Document: XCAF's auto-naming, colour labels, the removal of shapes, components,
/// colours and layers, and the compound added as an assembly keeping its shape.
/// </summary>
public class XdeDocDocumentEditTests
{
    private static TopoDS_Compound Compound(params TopoDS_Shape[] theShapes)
    {
        var aBuilder = new BRep_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        foreach (TopoDS_Shape aShape in theShapes)
        {
            aBuilder.Add(aCompound, aShape);
        }
        return aCompound;
    }

    private static int CountChildren(in TopoDS_Shape theShape)
    {
        int aCount = 0;
        for (var anIt = new TopoDS_Iterator(theShape); anIt.More(); anIt.Next())
        {
            aCount++;
        }
        return aCount;
    }

    [Fact]
    public void ACompoundAddedAsAnAssemblyIsFoundByItsShape()
    {
        TopoDS_Shape aBox1 = Box(1, 1, 1), aBox2 = Box(2, 2, 2);
        TopoDS_Compound aCompound = Compound(aBox1, aBox2);
        var aDoc = new XdeDoc_Document();
        XdeDoc_Label aLabel = aDoc.AddShape(aCompound);
        Assert.Equal("0:1:1:1", aLabel.Entry());
        Assert.Equal(aLabel, aDoc.FindShape(aCompound));
        Assert.Same(aCompound.TShape(), aDoc.GetShape(aLabel).TShape());
    }

    [Fact]
    public void AutoNamingNamesAsXcaf()
    {
        TopoDS_Compound aCompound = Compound(Box(1, 1, 1), Box(2, 2, 2));
        var aPlain = new XdeDoc_Document();
        aPlain.AddShape(aCompound);
        Assert.False(aPlain.GetName(aPlain.FindLabel("0:1:1:1"), out _));

        var aDoc = new XdeDoc_Document { AutoNaming = true };
        aDoc.AddShape(aCompound);
        Assert.True(aDoc.GetName(aDoc.FindLabel("0:1:1:1"), out string? aName));
        Assert.Equal("ASSEMBLY", aName);
        aDoc.GetName(aDoc.FindLabel("0:1:1:1:2"), out aName);
        Assert.Equal("=>[0:1:1:3]", aName);
        aDoc.GetName(aDoc.FindLabel("0:1:1:2"), out aName);
        Assert.Equal("SOLID", aName);

        // a tree opened as a document isn't auto-named
        XdeDoc_Document aFromTree = XdeDoc_Document.FromTree(aPlain.ToTree());
        Assert.False(aFromTree.GetName(aFromTree.FindLabel("0:1:1:1"), out _));
    }

    [Fact]
    public void ColoursHaveLabels()
    {
        var aDoc = new XdeDoc_Document();
        XdeDoc_Label aBox = aDoc.AddShape(Box(1, 1, 1));
        var aRed = new ColorWithAlpha(new Color(NameOfColor.NocRed));
        var aBlue = new ColorWithAlpha(new Color(NameOfColor.NocBlue));
        XdeDoc_Label aRedLabel = aDoc.AddColor(aRed);
        Assert.Equal("0:1:2:1", aRedLabel.Entry());
        Assert.Equal(aRedLabel, aDoc.AddColor(aRed));
        Assert.Equal(aRedLabel, aDoc.FindColor(aRed));
        Assert.True(aDoc.FindColor(aBlue).IsNull());
        Assert.True(aDoc.GetName(aRedLabel, out string? aName));
        Assert.Equal("RED (#FF0000FF)", aName);

        aDoc.SetColor(aBox, aBlue, XdeDoc_ColorType.XdeDoc_ColorSurf);
        aDoc.SetColor(aBox, aRed, XdeDoc_ColorType.XdeDoc_ColorCurv);
        var aColors = new NCollection_Sequence<XdeDoc_Label>();
        aDoc.GetColors(aColors);
        Assert.Equal(2, aColors.Length());
        Assert.True(aDoc.GetColor(aBox, XdeDoc_ColorType.XdeDoc_ColorSurf, out XdeDoc_Label aBlueLabel));
        Assert.Equal("0:1:2:2", aBlueLabel.Entry());

        // a removed colour leaves its shapes; its label stays, empty
        aDoc.RemoveColor(aRedLabel);
        Assert.False(aDoc.IsSet(aBox, XdeDoc_ColorType.XdeDoc_ColorCurv));
        Assert.True(aDoc.IsSet(aBox, XdeDoc_ColorType.XdeDoc_ColorSurf));
        aDoc.GetColors(aColors);
        Assert.Equal(1, aColors.Length());
        Assert.False(aDoc.IsColor(aRedLabel));
        Assert.Equal("0:1:2:3", aDoc.AddColor(aRed).Entry());

        // a tree's colours get labels in the document opened from it
        XdeDoc_Document aFromTree = XdeDoc_Document.FromTree(aDoc.ToTree());
        aFromTree.GetColors(aColors);
        Assert.Equal(1, aColors.Length());
    }

    [Fact]
    public void ShapesComponentsAndLayersAreRemoved()
    {
        TopoDS_Shape aBox1 = Box(1, 1, 1), aBox2 = Box(2, 2, 2);
        var aDoc = new XdeDoc_Document();
        XdeDoc_Label anAssembly = aDoc.AddShape(Compound(aBox1, aBox2));
        XdeDoc_Label aComp2 = aDoc.FindLabel("0:1:1:1:2");
        XdeDoc_Label aPart2 = aDoc.FindLabel("0:1:1:3");
        Assert.False(aDoc.RemoveShape(aPart2));   // used
        aDoc.RemoveComponent(aComp2);
        Assert.True(aDoc.IsFree(aPart2));
        Assert.Equal(1, aDoc.NbComponents(anAssembly));
        Assert.Equal(1, CountChildren(aDoc.GetShape(anAssembly)));
        Assert.True(aDoc.RemoveShape(aPart2));
        Assert.True(aDoc.GetShape(aPart2).IsNull());
        Assert.True(aDoc.FindShape(aBox2).IsNull());
        var aFree = new NCollection_Sequence<XdeDoc_Label>();
        aDoc.GetFreeShapes(aFree);
        Assert.Equal(1, aFree.Length());

        XdeDoc_Label aPart1 = aDoc.FindLabel("0:1:1:2");
        aDoc.SetLayer(aPart1, "L1");
        aDoc.SetLayer(aPart1, "L2");
        Assert.True(aDoc.UnSetOneLayer(aPart1, "L1"));
        var aNames = new NCollection_Sequence<string>();
        aDoc.GetLayers(aPart1, aNames);
        Assert.Equal(["L2"], aNames);
        aDoc.RemoveLayer(aDoc.FindLayer("L2"));
        Assert.False(aDoc.GetLayers(aPart1, aNames));
        var aLayers = new NCollection_Sequence<XdeDoc_Label>();
        aDoc.GetLayerLabels(aLayers);
        Assert.Equal(1, aLayers.Length());
        Assert.Single(aDoc.ToTree().Layers);
    }
}
