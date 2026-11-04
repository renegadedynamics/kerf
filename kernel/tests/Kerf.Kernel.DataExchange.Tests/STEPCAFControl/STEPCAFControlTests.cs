// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step.Caf;
using Kerf.Engine.Exchange.Step.Construct;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text;
using Kerf.Kernel.DataExchange.Tests.StepGen;

namespace Kerf.Kernel.DataExchange.Tests.STEPCAFControl;

/// <summary>STEPCAFControl's reader and writer over XdeDoc_Document, and STEPConstruct's tools they use (unit 8G2).</summary>
public sealed class STEPCAFControlTests
{
    private static Shape Box(double theSize)
    {
        var aMaker = new MakeBox(theSize, theSize, theSize);
        return aMaker.Shape();
    }

    private static byte[] Write(ExchangeDocument theDoc, Action<CafWriter>? theSetup = null)
    {
        var aWriter = new CafWriter();
        theSetup?.Invoke(aWriter);
        Assert.True(aWriter.Transfer(theDoc));
        var aStream = new MemoryStream();
        Assert.Equal(ReturnStatus.RetDone, aWriter.WriteStream(aStream));
        return aStream.ToArray();
    }

    private static ExchangeDocument Read(byte[] theText, Action<CafReader>? theSetup = null)
    {
        var aReader = new CafReader();
        theSetup?.Invoke(aReader);
        Assert.Equal(ReturnStatus.RetDone, aReader.ReadStream("doc.stp", new MemoryStream(theText)));
        var aDoc = new ExchangeDocument();
        Assert.True(aReader.Transfer(aDoc));
        return aDoc;
    }

    [Fact]
    public void RoundTripsNamesColoursLayersAndProperties()
    {
        var aDoc = new ExchangeDocument();
        Label aPart = aDoc.AddShape(Box(10.0), false);
        aDoc.SetName(aPart, "box");
        aDoc.SetColor(aPart, new LinearColor(0.25, 0.5, 0.75, ColorSpace.TocRgb), ColorType.ColorSurf);
        aDoc.SetLayer(aPart, "parts");
        aDoc.SetVolume(aPart, 1000.0);
        string aBefore = string.Join("\n", XdeDocDump.Dump(aDoc));

        ExchangeDocument aRead = Read(Write(aDoc));
        Assert.Equal(aBefore, string.Join("\n", XdeDocDump.Dump(aDoc))); // the writer never changes the document

        var aFree = new NCollection_Sequence<Label>();
        aRead.GetFreeShapes(aFree);
        Assert.Equal(1, aFree.Length());
        Label aLabel = aFree.Value(1);
        Assert.True(aRead.GetName(aLabel, out string? aName));
        Assert.Equal("box", aName);
        Assert.True(aRead.GetColor(aLabel, ColorType.ColorSurf, out LinearColor aColor));
        Assert.True(aColor.IsEqual(new LinearColor(0.25, 0.5, 0.75, ColorSpace.TocRgb)));
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
        ExchangeDocument aDoc = XdeDocDump.MakeAssemblyDoc(Box(5.0));
        ExchangeDocument aRead = Read(Write(aDoc));
        aRead.AutoNaming = true;
        Tree aTree = aRead.ToTree();
        Assert.Single(aTree.Roots);
        DocumentProduct aTop = aTree.Roots[0].Product;
        Assert.Equal("top", aTop.Attributes.Name);
        Assert.Equal(2, aTop.Components.Length);
        Assert.Equal("upper", aTop.Components[1].Attributes.Name);
        DocumentProduct aSub = aTop.Components[0].Product;
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
        var aDoc = new ExchangeDocument();
        Shape aBox = Box(10.0);
        Label aPart = aDoc.AddShape(aBox, false);
        Label aFace = aDoc.AddSubShape(aPart, new Explorer(aBox, ShapeType.Face).Current());
        aDoc.SetName(aFace, "top face");
        byte[] aText = Write(aDoc, w => w.Writer().WS()!.Statics().SetIVal("write.stepcaf.subshapes.name", 1));
        Assert.Contains("'top face'", Encoding.Latin1.GetString(aText), StringComparison.Ordinal);

        ExchangeDocument aWithout = Read(aText);
        Assert.False(aWithout.HasChild(aWithout.FindLabel("0:1:1:1")));
        ExchangeDocument aWith = Read(aText, r => r.Reader().WS()!.Statics().SetIVal("read.stepcaf.subshapes.name", 1));
        Label aRoot = aWith.FindLabel("0:1:1:1");
        Assert.Contains(aWith.Children(aRoot), l => aWith.GetName(l, out string? n) && n == "top face");
    }

    [Fact]
    public void FindsComponentsAndSetsInstanceColours()
    {
        var aDoc = new ExchangeDocument();
        Shape aBox = Box(1.0);
        Label aPart = aDoc.AddShape(aBox, false);
        Label anAssembly = aDoc.NewShape();
        var aTrsf = new Transform3D();
        aTrsf = aTrsf.WithTranslation(new Vector3D(5.0, 0.0, 0.0));
        var aLoc = new Placement(aTrsf);
        Label aComp = aDoc.AddComponent(anAssembly, aPart, aLoc);
        var aLabels = new NCollection_Sequence<Label>();
        Assert.True(aDoc.FindComponent(aBox.Located(aLoc), aLabels));
        Assert.Equal(aComp, aLabels.Value(1));
        Assert.True(aDoc.SetInstanceColor(aBox.Located(aLoc), ColorType.ColorSurf,
                                          new LinearColor(NameOfColor.NocRed)));
        Assert.True(aDoc.IsSet(aComp, ColorType.ColorSurf));
        Assert.False(aDoc.FindComponent(aBox.Located(new Placement(new Transform3D())), aLabels));
    }

    [Fact]
    public void EncodesAndDecodesColours()
    {
        Colour aRed = Styles.EncodeColor(new LinearColor(NameOfColor.NocRed));
        Assert.IsType<DraughtingPreDefinedColour>(aRed);
        var aColor = new LinearColor();
        Assert.True(Styles.DecodeColor(aRed, ref aColor));
        Assert.True(aColor.IsEqual(new LinearColor(NameOfColor.NocRed)));
        var aRgb = new LinearColor(0.25, 0.5, 0.75, ColorSpace.TOC_sRGB);
        Colour anEncoded = Styles.EncodeColor(aRgb);
        Assert.IsType<ColourRgb>(anEncoded);
        Assert.True(Styles.DecodeColor(anEncoded, ref aColor));
        Assert.True(aColor.IsEqual(aRgb));

        var aProps = new RenderingProperties(new ColorWithAlpha(aRgb, 0.25f));
        Assert.True(aProps.IsDefined);
        Assert.Equal(0.75, aProps.Transparency(), 6);
        var aBack = new RenderingProperties(aProps.CreateRenderingProperties());
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
        var aReader = new CafReader();
        Assert.Equal(ReturnStatus.RetDone, aReader.ReadFile(aFile));
        var aDoc = new ExchangeDocument();
        Assert.True(aReader.Transfer(aDoc));
        Tree aTree = aDoc.ToTree();
        Assert.NotEmpty(aTree.Roots);
        Assert.True(aTree.Roots[0].Product.IsAssembly);
        Assert.NotEmpty(aTree.Flatten());
        // a tree opened again writes as the document read
        byte[] aText = Write(ExchangeDocument.FromTree(aTree));
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
        var aReader = new CafReader();
        Assert.Equal(ReturnStatus.RetDone, aReader.ReadFile(aFile));
        var aDoc = new ExchangeDocument();
        Assert.True(aReader.Transfer(aDoc));
        ExchangeDocument aRead = Read(Write(aDoc));
        Tree aTree = aRead.ToTree();
        Assert.Single(aTree.Roots);
        Assert.True(aTree.Roots[0].Product.IsAssembly);
        Assert.Equal(8, aTree.Roots[0].Product.Components.Length);
    }
}
