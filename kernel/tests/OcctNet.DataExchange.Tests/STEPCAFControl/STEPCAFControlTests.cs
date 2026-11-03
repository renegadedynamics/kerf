// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text;
using OcctNet.DataExchange.Tests.StepGen;

namespace OcctNet.DataExchange.Tests.STEPCAFControl;

/// <summary>STEPCAFControl's reader and writer over XdeDoc_Document, and STEPConstruct's tools they use (unit 8G2).</summary>
public sealed class STEPCAFControlTests
{
    private static Shape Box(double theSize)
    {
        var aMaker = new BRepPrimAPI_MakeBox(theSize, theSize, theSize);
        return aMaker.Shape();
    }

    private static byte[] Write(XdeDoc_Document theDoc, Action<STEPCAFControl_Writer>? theSetup = null)
    {
        var aWriter = new STEPCAFControl_Writer();
        theSetup?.Invoke(aWriter);
        Assert.True(aWriter.Transfer(theDoc));
        var aStream = new MemoryStream();
        Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone, aWriter.WriteStream(aStream));
        return aStream.ToArray();
    }

    private static XdeDoc_Document Read(byte[] theText, Action<STEPCAFControl_Reader>? theSetup = null)
    {
        var aReader = new STEPCAFControl_Reader();
        theSetup?.Invoke(aReader);
        Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone, aReader.ReadStream("doc.stp", new MemoryStream(theText)));
        var aDoc = new XdeDoc_Document();
        Assert.True(aReader.Transfer(aDoc));
        return aDoc;
    }

    [Fact]
    public void RoundTripsNamesColoursLayersAndProperties()
    {
        var aDoc = new XdeDoc_Document();
        XdeDoc_Label aPart = aDoc.AddShape(Box(10.0), false);
        aDoc.SetName(aPart, "box");
        aDoc.SetColor(aPart, new Color(0.25, 0.5, 0.75, ColorSpace.TocRgb), XdeDoc_ColorType.XdeDoc_ColorSurf);
        aDoc.SetLayer(aPart, "parts");
        aDoc.SetVolume(aPart, 1000.0);
        string aBefore = string.Join("\n", XdeDocDump.Dump(aDoc));

        XdeDoc_Document aRead = Read(Write(aDoc));
        Assert.Equal(aBefore, string.Join("\n", XdeDocDump.Dump(aDoc))); // the writer never changes the document

        var aFree = new NCollection_Sequence<XdeDoc_Label>();
        aRead.GetFreeShapes(aFree);
        Assert.Equal(1, aFree.Length());
        XdeDoc_Label aLabel = aFree.Value(1);
        Assert.True(aRead.GetName(aLabel, out string? aName));
        Assert.Equal("box", aName);
        Assert.True(aRead.GetColor(aLabel, XdeDoc_ColorType.XdeDoc_ColorSurf, out Color aColor));
        Assert.True(aColor.IsEqual(new Color(0.25, 0.5, 0.75, ColorSpace.TocRgb)));
        var aLayers = new NCollection_Sequence<string>();
        Assert.True(aRead.GetLayers(aLabel, aLayers));
        Assert.Equal("parts", aLayers.Value(1));
        Assert.Equal(1000.0, aRead.GetValidationProps(aLabel)!.Volume!.Value, 6);
        Assert.True(aRead.GetLengthUnit(out double aUnit));
        Assert.Equal(0.001, aUnit, 12);
    }

    [Fact]
    public void RoundTripsAssembliesAndInstanceColours()
    {
        XdeDoc_Document aDoc = XdeDocDump.MakeAssemblyDoc(Box(5.0));
        XdeDoc_Document aRead = Read(Write(aDoc));
        aRead.AutoNaming = true;
        XdeDoc_Tree aTree = aRead.ToTree();
        Assert.Single(aTree.Roots);
        XdeDoc_Product aTop = aTree.Roots[0].Product;
        Assert.Equal("top", aTop.Attributes.Name);
        Assert.Equal(2, aTop.Components.Length);
        Assert.Equal("upper", aTop.Components[1].Attributes.Name);
        XdeDoc_Product aSub = aTop.Components[0].Product;
        Assert.Same(aSub, aTop.Components[1].Product);
        Assert.Equal("sub", aSub.Attributes.Name);
        Assert.Equal("second", aSub.Components[1].Attributes.Name);
        // the instance's colour, written as a NAUO's style, read back on the component
        Assert.NotNull(aSub.Components[1].Attributes.SurfaceColor);
        Assert.Equal("part", aSub.Components[0].Product.Attributes.Name);
        Assert.NotNull(aSub.Components[0].Product.Attributes.SurfaceColor);
    }

    [Fact]
    public void ReadsSubShapeNamesWhenAsked()
    {
        var aDoc = new XdeDoc_Document();
        Shape aBox = Box(10.0);
        XdeDoc_Label aPart = aDoc.AddShape(aBox, false);
        XdeDoc_Label aFace = aDoc.AddSubShape(aPart, new Explorer(aBox, ShapeType.Face).Current());
        aDoc.SetName(aFace, "top face");
        byte[] aText = Write(aDoc, w => w.Writer().WS()!.Statics().SetIVal("write.stepcaf.subshapes.name", 1));
        Assert.Contains("'top face'", Encoding.Latin1.GetString(aText), StringComparison.Ordinal);

        XdeDoc_Document aWithout = Read(aText);
        Assert.False(aWithout.HasChild(aWithout.FindLabel("0:1:1:1")));
        XdeDoc_Document aWith = Read(aText, r => r.Reader().WS()!.Statics().SetIVal("read.stepcaf.subshapes.name", 1));
        XdeDoc_Label aRoot = aWith.FindLabel("0:1:1:1");
        Assert.Contains(aWith.Children(aRoot), l => aWith.GetName(l, out string? n) && n == "top face");
    }

    [Fact]
    public void FindsComponentsAndSetsInstanceColours()
    {
        var aDoc = new XdeDoc_Document();
        Shape aBox = Box(1.0);
        XdeDoc_Label aPart = aDoc.AddShape(aBox, false);
        XdeDoc_Label anAssembly = aDoc.NewShape();
        var aTrsf = new Transform3D();
        aTrsf.SetTranslation(new Vector3D(5.0, 0.0, 0.0));
        var aLoc = new Placement(aTrsf);
        XdeDoc_Label aComp = aDoc.AddComponent(anAssembly, aPart, aLoc);
        var aLabels = new NCollection_Sequence<XdeDoc_Label>();
        Assert.True(aDoc.FindComponent(aBox.Located(aLoc), aLabels));
        Assert.Equal(aComp, aLabels.Value(1));
        Assert.True(aDoc.SetInstanceColor(aBox.Located(aLoc), XdeDoc_ColorType.XdeDoc_ColorSurf,
                                          new Color(NameOfColor.NocRed)));
        Assert.True(aDoc.IsSet(aComp, XdeDoc_ColorType.XdeDoc_ColorSurf));
        Assert.False(aDoc.FindComponent(aBox.Located(new Placement(new Transform3D())), aLabels));
    }

    [Fact]
    public void EncodesAndDecodesColours()
    {
        StepVisual_Colour aRed = STEPConstruct_Styles.EncodeColor(new Color(NameOfColor.NocRed));
        Assert.IsType<StepVisual_DraughtingPreDefinedColour>(aRed);
        var aColor = new Color();
        Assert.True(STEPConstruct_Styles.DecodeColor(aRed, ref aColor));
        Assert.True(aColor.IsEqual(new Color(NameOfColor.NocRed)));
        var aRgb = new Color(0.25, 0.5, 0.75, ColorSpace.TOC_sRGB);
        StepVisual_Colour anEncoded = STEPConstruct_Styles.EncodeColor(aRgb);
        Assert.IsType<StepVisual_ColourRgb>(anEncoded);
        Assert.True(STEPConstruct_Styles.DecodeColor(anEncoded, ref aColor));
        Assert.True(aColor.IsEqual(aRgb));

        var aProps = new STEPConstruct_RenderingProperties(new ColorWithAlpha(aRgb, 0.25f));
        Assert.True(aProps.IsDefined());
        Assert.Equal(0.75, aProps.Transparency(), 6);
        var aBack = new STEPConstruct_RenderingProperties(aProps.CreateRenderingProperties());
        Assert.Equal(0.25f, aBack.GetRGBAColor().Alpha(), 5);
    }

    [Fact]
    public void ReadsACorpusFileThroughTheTree()
    {
        string aFile = Path.Combine(StepRoundTripTests.Metadata("OcctTestDataDir"), "step", "bug21802_as1-oc-214.stp");
        if (!File.Exists(aFile))
        {
            return;
        }
        var aReader = new STEPCAFControl_Reader();
        Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone, aReader.ReadFile(aFile));
        var aDoc = new XdeDoc_Document();
        Assert.True(aReader.Transfer(aDoc));
        XdeDoc_Tree aTree = aDoc.ToTree();
        Assert.NotEmpty(aTree.Roots);
        Assert.True(aTree.Roots[0].Product.IsAssembly);
        Assert.NotEmpty(aTree.Flatten());
        // a tree opened again writes as the document read
        byte[] aText = Write(XdeDoc_Document.FromTree(aTree));
        Assert.NotEmpty(aText);
    }

    [Fact]
    public void ReadsBackTheAssemblyItWrote()
    {
        // the assembly's compound shares its parts with their product definitions' results (the regularity encoded
        // in place around the parts published before: XSControl_TransferReader.ShapeResult, unit 8G2)
        string aFile = Path.Combine(StepRoundTripTests.Metadata("OcctTestDataDir"), "step", "trj4_s1-ai-214.stp");
        if (!File.Exists(aFile))
        {
            return;
        }
        var aReader = new STEPCAFControl_Reader();
        Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone, aReader.ReadFile(aFile));
        var aDoc = new XdeDoc_Document();
        Assert.True(aReader.Transfer(aDoc));
        XdeDoc_Document aRead = Read(Write(aDoc));
        XdeDoc_Tree aTree = aRead.ToTree();
        Assert.Single(aTree.Roots);
        Assert.True(aTree.Roots[0].Product.IsAssembly);
        Assert.Equal(8, aTree.Roots[0].Product.Components.Length);
    }
}
