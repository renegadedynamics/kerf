// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Kernel.DataExchange.Tests.MeshTestKit;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>
/// The immutable assembly tree (XdeDoc_Tree, XdeDoc_Product, XdeDoc_Instance) and its label interface
/// (XdeDoc_Document), unit 8T: sharing, flattening, style resolution, meshing, and XCAF's shape, colour and layer tool
/// behaviour as STEPCAFControl uses it.
/// </summary>
public class XdeDocTests
{
    [Fact]
    public void PartsAreFrozenAndShared()
    {
        Shape aBox = Box(1, 2, 3);
        DocumentProduct aPart = DocumentProduct.Part(aBox, "B");
        Assert.True(aPart.Shape.TShape()!.IsFrozen);
        Assert.Throws<ArgumentException>(() => DocumentProduct.Part(new Shape()));

        DocumentProduct anAssembly = DocumentProduct.Assembly("A", aPart.Placed(Translation(1, 0, 0)), aPart.Placed(Translation(2, 0, 0)));
        var aTree = new Tree(anAssembly.Placed(new Placement()));
        Assert.Equal(2, aTree.Products().Count());
        Assert.Same(anAssembly.Components[0].Product, anAssembly.Components[1].Product);

        // the assembly's shape: the compound of its components, made once, frozen
        Shape aCompound = anAssembly.Shape;
        Assert.Equal(ShapeType.Compound, aCompound.Type);
        Assert.True(aCompound.TShape()!.IsFrozen);
        Assert.True(aCompound.IsEqual(anAssembly.Shape));
        Assert.Equal(2, new Iterator(aCompound).More() ? CountChildren(aCompound) : 0);

        // With... makes new objects; the old ones keep their values
        DocumentProduct aRenamed = aPart.WithName("C");
        Assert.Equal("B", aPart.Name);
        Assert.Equal("C", aRenamed.Name);
        Assert.True(aRenamed.Shape.IsEqual(aPart.Shape));
        Tree aTree2 = aTree.WithRoot(aRenamed.Placed(new Placement()));
        Assert.Single(aTree.Roots);
        Assert.Equal(2, aTree2.Roots.Length);
        Assert.Same(aTree.Roots[0], aTree2.Roots[0]);
    }

    private static int CountChildren(in Shape theShape)
    {
        int aCount = 0;
        for (var anIt = new Iterator(theShape); anIt.More(); anIt.Next())
        {
            aCount++;
        }
        return aCount;
    }

    [Fact]
    public void FlattenComposesLocationsAndStyles()
    {
        Tree aTree = SampleTree(false);
        IReadOnlyList<FlatItem> anItems = aTree.Flatten();
        Assert.Equal(3, anItems.Count);
        Assert.Equal(new string?[] { "Asm", "Box" }, anItems[0].NamePath);
        Assert.Equal(new string?[] { "Asm", "Box2" }, anItems[1].NamePath);
        Assert.Equal(new string?[] { "Asm", "Plate" }, anItems[2].NamePath);

        // the root's (1, 2, 3) composed with each component's
        Xyz aT0 = anItems[0].Location.Transformation.TranslationXyz();
        Assert.Equal(1.0, aT0.X(), 12);
        Assert.Equal(2.0, aT0.Y(), 12);
        Assert.Equal(3.0, aT0.Z(), 12);
        Xyz aT2 = anItems[2].Location.Transformation.TranslationXyz();
        Assert.Equal(42.0, aT2.Y(), 12);

        // the product's colour, overridden by the instance's
        Assert.Equal(Red, anItems[0].Style.SurfaceColor);
        Assert.Equal(Green, anItems[1].Style.SurfaceColor);
        Assert.Null(anItems[2].Style.SurfaceColor);

        // a hidden layer hides the plate
        Tree aHidden = aTree.WithLayers([new Layer("L1", false)]);
        Assert.Equal(2, aHidden.Flatten().Count);
        Assert.Equal(3, aHidden.Flatten(true).Count);
        Assert.False(aHidden.Flatten(true)[2].Style.IsVisible);
        Assert.Single(aTree.Layers);
    }

    [Fact]
    public void StylesFollowXcafsMerge()
    {
        // a generic colour sets both; an assembly's colour is inherited by parts without their own
        Style aStyle = Style.Default.Merged(new Attributes { GenericColor = Red });
        Assert.Equal(Red, aStyle.SurfaceColor);
        Assert.Equal(Red, aStyle.CurveColor);
        aStyle = aStyle.Merged(new Attributes { SurfaceColor = Green, IsVisible = false });
        Assert.Equal(Green, aStyle.SurfaceColor);
        Assert.Equal(Red, aStyle.CurveColor);
        Assert.False(aStyle.IsVisible);

        DocumentProduct aPart = DocumentProduct.Part(Box(1, 1, 1), "P");
        DocumentProduct anAssembly = DocumentProduct.Assembly([aPart.Placed(new Placement())], new Attributes { SurfaceColor = Blue });
        FlatItem anItem = Assert.Single(new Tree(anAssembly.Placed(new Placement())).Flatten());
        Assert.Equal(Blue, anItem.Style.SurfaceColor);
    }

    [Fact]
    public void AttributesCompareByValue()
    {
        var a = new Attributes { Name = "x", Layers = ["a", "b"] };
        var b = new Attributes { Name = "x", Layers = ["a", "b"] };
        Assert.Equal(a, b);
        Assert.Equal(a.GetHashCode(), b.GetHashCode());
        Assert.NotEqual(a, b.WithLayer("c"));
        Assert.True(Attributes.Empty.IsEmpty);
        Assert.False(a.IsEmpty);
        Assert.Same(a, a.WithLayer("a"));
    }

    [Fact]
    public void MeshedSharesWhatMeshingLeaves()
    {
        DocumentProduct aMeshed = DocumentProduct.Part(MeshedBox(1, 1, 1), "M");
        Shape aBox = Box(2, 2, 2);
        Shape aFace = new Explorer(aBox, ShapeType.Face).Current();
        DocumentProduct aRaw = DocumentProduct.Part(aBox, "R").WithSubShape(aFace, new Attributes { SurfaceColor = Red });
        var aTree = new Tree([aMeshed.Placed(new Placement()), aRaw.Placed(Translation(5, 0, 0)), aRaw.Placed(Translation(9, 0, 0))]);

        Tree? aResult = aTree.Meshed(MeshParameters);
        Assert.NotNull(aResult);
        Assert.Same(aMeshed, aResult.Roots[0].Product);
        Assert.NotSame(aRaw, aResult.Roots[1].Product);
        Assert.Same(aResult.Roots[1].Product, aResult.Roots[2].Product); // still shared
        DocumentProduct aNew = aResult.Roots[1].Product;
        Assert.Equal("R", aNew.Name);
        SubShape aSub = Assert.Single(aNew.SubShapes);
        Assert.Equal(Red, aSub.Attributes.SurfaceColor);
        Assert.NotNull(Tool.Triangulation(ShapeCasts.Face(aSub.Shape), out _));
        Assert.True(CountShapes(aNew.Shape, ShapeType.Face) == 6);
        Assert.Null(Tool.Triangulation(ShapeCasts.Face(aFace), out _)); // the input isn't changed
        Assert.Equal(12 * 3, WorldTriangles(aResult).Count);

        using var aCancel = new CancellationTokenSource();
        aCancel.Cancel();
        Assert.Null(aTree.Meshed(MeshParameters, new ProgressIndicator(aCancel.Token).Start()));
    }

    // ---------------------------------------------------------------------------------------------- the document

    // What STEPCAFControl_Reader::AddShape does for an assembly: NewShape, then a component per located sub-shape
    [Fact]
    public void DocumentBuildsAssembliesAsTheStepReaderDoes()
    {
        Shape aBox = MeshedBox(10, 20, 30);
        var aDoc = new ExchangeDocument();
        Label anAssembly = aDoc.NewShape();
        Assert.Equal("0:1:1:1", anAssembly.Entry());
        Assert.True(aDoc.IsSimpleShape(anAssembly)); // an empty compound until it gets components
        Label aPart = aDoc.AddShape(aBox, false);
        Assert.Equal("0:1:1:2", aPart.Entry());
        Label aComp1 = aDoc.AddComponent(anAssembly, aPart, Translation(100, 0, 0));
        Label aComp2 = aDoc.AddComponent(anAssembly, aPart, QuarterTurn(0, 100, 0));
        Assert.Equal("0:1:1:1:1", aComp1.Entry());
        Assert.Equal("0:1:1:1:2", aComp2.Entry());
        Assert.True(aDoc.IsAssembly(anAssembly));
        Assert.True(aDoc.IsComponent(aComp1));
        Assert.True(aDoc.IsReference(aComp1));
        Assert.True(aDoc.GetReferredShape(aComp2, out Label aReferred));
        Assert.Equal(aPart, aReferred);
        Assert.False(aDoc.IsFree(aPart));
        Assert.True(aDoc.IsFree(anAssembly));
        var aUsers = new NCollection_Sequence<Label>();
        Assert.Equal(2, aDoc.GetUsers(aPart, aUsers));
        Assert.Equal(2, aDoc.NbComponents(anAssembly));
        var aFree = new NCollection_Sequence<Label>();
        aDoc.GetFreeShapes(aFree);
        Assert.Equal(anAssembly, Assert.Single(aFree));

        // the assembly's shape is the compound of its components
        aDoc.UpdateAssemblies();
        Shape aCompound = aDoc.GetShape(anAssembly);
        Assert.Equal(2, CountChildren(aCompound));
        Assert.True(aDoc.GetShape(aComp1).IsPartner(aBox));

        // Search: a located shape finds its component; the unlocated one its part; a face a new sub-shape label
        Assert.True(aDoc.Search(aDoc.GetShape(aComp2), out Label aFound));
        Assert.Equal(aComp2, aFound);
        Assert.True(aDoc.Search(aBox, out aFound));
        Assert.Equal(aPart, aFound);
        Shape aFace = new Explorer(aBox, ShapeType.Face).Current();
        Assert.False(aDoc.Search(aFace, out aFound, true, true, false));
        Assert.True(aDoc.Search(aFace, out Label aSub));
        Assert.Equal("0:1:1:2:1", aSub.Entry());
        Assert.True(aDoc.IsSubShape(aSub));
        Assert.True(aDoc.FindSubShape(aPart, aFace, out aFound));
        Assert.Equal(aSub, aFound);
        Assert.Equal(aSub, aDoc.AddSubShape(aPart, aFace)); // existing
        Assert.True(aDoc.SearchUsingMap(aFace, out aFound, false, true));
        Assert.Equal(aSub, aFound);
        Assert.True(aDoc.SearchUsingMap(aBox, out aFound, false, false));
        Assert.Equal(aPart, aFound);

        // names, colours, visibility, layers, validation properties
        aDoc.SetName(anAssembly, "Asm");
        aDoc.SetName(aPart, "Box");
        aDoc.SetName(aComp2, "Box2");
        aDoc.SetColor(aPart, new LinearColor(NameOfColor.NocRed), ColorType.ColorSurf);
        aDoc.SetColor(aComp2, Green, ColorType.ColorGen);
        aDoc.SetColor(aSub, Blue, ColorType.ColorSurf);
        Assert.True(aDoc.GetColor(aSub, ColorType.ColorSurf, out ColorWithAlpha aBlue));
        Assert.Equal(Blue, aBlue);
        Assert.False(aDoc.IsSet(aSub, ColorType.ColorCurv));
        Assert.True(aDoc.GetColor(aFace, ColorType.ColorSurf, out aBlue));
        Label aLayer = aDoc.AddLayer("L1");
        Assert.Equal("0:1:3:1", aLayer.Entry());
        Assert.Equal(aLayer, aDoc.AddLayer("L1"));
        aDoc.SetLayer(aComp1, aLayer);
        aDoc.SetLayer(aComp1, "L2");
        var aNames = new NCollection_Sequence<string>();
        Assert.True(aDoc.GetLayers(aComp1, aNames));
        Assert.Equal(new[] { "L1", "L2" }, aNames);
        var aOfLayer = new NCollection_Sequence<Label>();
        aDoc.GetShapesOfLayer(aLayer, aOfLayer);
        Assert.Equal(aComp1, Assert.Single(aOfLayer));
        aDoc.SetVisibility(aDoc.FindLayer("L2"), false);
        aDoc.SetVolume(aPart, 6000.0);
        aDoc.SetCentroid(aPart, new Point3D(5, 10, 15));
        aDoc.SetLengthUnit(0.001, "MM");
        Assert.Equal(aComp2, aDoc.FindLabel("0:1:1:1:2"));
        Assert.True(aDoc.FindLabel("0:1:9").IsNull);

        // frozen into a tree: one root, the part shared by both components
        Tree aTree = aDoc.ToTree();
        Instance aRoot = Assert.Single(aTree.Roots);
        DocumentProduct anAsmProduct = aRoot.Product;
        Assert.True(anAsmProduct.IsAssembly);
        Assert.Equal("Asm", anAsmProduct.Name);
        Assert.Equal(2, anAsmProduct.Components.Length);
        Assert.Same(anAsmProduct.Components[0].Product, anAsmProduct.Components[1].Product);
        DocumentProduct aBoxProduct = anAsmProduct.Components[0].Product;
        Assert.Equal("Box", aBoxProduct.Name);
        Assert.Equal(6000.0, aBoxProduct.Attributes.ValidationProps!.Volume);
        Assert.Equal(Blue, Assert.Single(aBoxProduct.SubShapes).Attributes.SurfaceColor);
        Assert.Equal(new[] { "L1", "L2" }, anAsmProduct.Components[0].Attributes.Layers);
        Assert.Equal("Box2", anAsmProduct.Components[1].Name);
        Assert.Equal(0.001, aTree.LengthUnit!.UnitValue);
        Assert.False(aTree.FindLayer("L2")!.IsVisible);
        Assert.Single(aTree.Flatten()); // the first component is in a hidden layer
        IReadOnlyList<FlatItem> anAll = aTree.Flatten(true);
        Assert.Equal(2, anAll.Count);
        Assert.Equal(Green, anAll[1].Style.SurfaceColor);

        // and opened again: the same labels
        ExchangeDocument aDoc2 = ExchangeDocument.FromTree(aTree);
        var aFree2 = new NCollection_Sequence<Label>();
        aDoc2.GetFreeShapes(aFree2);
        Label aRoot2 = Assert.Single(aFree2);
        Assert.Equal("0:1:1:1", aRoot2.Entry());
        Assert.True(aDoc2.IsAssembly(aRoot2));
        var aComps = new NCollection_Sequence<Label>();
        aDoc2.GetComponents(aRoot2, aComps);
        Assert.Equal(2, aComps.Length());
        Assert.True(aDoc2.GetReferredShape(aComps.Value(1), out Label aPart2));
        Assert.Equal("0:1:1:2", aPart2.Entry());
        Assert.True(aDoc2.GetName(aPart2, out string? aName2));
        Assert.Equal("Box", aName2);
        Assert.True(aDoc2.GetColor(aPart2, ColorType.ColorSurf, out LinearColor aRed));
        Assert.True(aRed.IsEqual(new LinearColor(NameOfColor.NocRed)));
        var aSubs = new NCollection_Sequence<Label>();
        Assert.True(aDoc2.GetSubShapes(aPart2, aSubs));
        Assert.Equal("0:1:1:2:1", aSubs.Value(1).Entry());
        Assert.True(aDoc2.FindShape(aBox, out Label aFoundBox));
        Assert.Equal(aPart2, aFoundBox);
        Assert.True(aDoc2.GetShape(aRoot2).IsSame(anAsmProduct.Shape));
        var aLayers2 = new NCollection_Sequence<Label>();
        aDoc2.GetLayerLabels(aLayers2);
        Assert.Equal(2, aLayers2.Length());
        Assert.False(aDoc2.IsVisible(aLayers2.Value(2)));
    }

    // XCAFDoc_ShapeTool::AddShape: a compound becomes an assembly of its sub-shapes, a located shape a reference
    [Fact]
    public void AddShapeMakesAssembliesAndReferencesAsXcaf()
    {
        Shape aBox = Box(1, 1, 1);
        var aBuilder = new BrepBuilder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, aBox.Moved(Translation(5, 0, 0)));
        aBuilder.Add(aCompound, aBox.Moved(Translation(10, 0, 0)));

        var aDoc = new ExchangeDocument();
        Label aL = aDoc.AddShape(aCompound);
        Assert.Equal("0:1:1:1", aL.Entry());
        Assert.True(aDoc.IsAssembly(aL));
        var aComps = new NCollection_Sequence<Label>();
        aDoc.GetComponents(aL, aComps);
        Assert.Equal(new[] { "0:1:1:1:1", "0:1:1:1:2" }, aComps.Select(l => l.Entry()));
        Assert.True(aDoc.GetReferredShape(aComps.Value(1), out Label aPart));
        Assert.Equal("0:1:1:2", aPart.Entry());
        Assert.True(aDoc.GetReferredShape(aComps.Value(2), out Label aPart2));
        Assert.Equal(aPart, aPart2);
        Assert.Equal(aL, aDoc.AddShape(aCompound)); // found again

        // without makeAssembly: a simple shape
        Label aFlat = new ExchangeDocument().AddShape(aCompound, false);
        Assert.True(aFlat.Document!.IsSimpleShape(aFlat));

        // a located shape: a top-level reference to its unlocated shape, found with its location
        var aDoc3 = new ExchangeDocument();
        Shape aMoved = aBox.Moved(Translation(0, 0, 7));
        Label aRef = aDoc3.AddShape(aMoved);
        Assert.Equal("0:1:1:1", aRef.Entry());
        Assert.True(aDoc3.IsReference(aRef));
        Assert.False(aDoc3.IsComponent(aRef));
        Assert.True(aDoc3.FindShape(aMoved, out Label aFound, true));
        Assert.Equal(aRef, aFound);
        Assert.True(aDoc3.FindShape(aMoved, out aFound));
        Assert.Equal("0:1:1:2", aFound.Entry());
        Tree aTree = aDoc3.ToTree();
        Instance aRoot = Assert.Single(aTree.Roots);
        Assert.Equal(7.0, aRoot.Location.Transformation.TranslationXyz().Z(), 12);
        Assert.False(aRoot.Product.IsAssembly);
    }

    [Fact]
    public void DocumentRefusesCycles()
    {
        var aDoc = new ExchangeDocument();
        Label anA = aDoc.NewShape();
        Label aB = aDoc.NewShape();
        Assert.False(aDoc.AddComponent(anA, aB, new Placement()).IsNull);
        Assert.True(aDoc.AddComponent(aB, anA, new Placement()).IsNull);
        Assert.True(aDoc.AddComponent(anA, anA, new Placement()).IsNull);
        Instance aRoot = Assert.Single(aDoc.ToTree().Roots);
        Assert.Single(aRoot.Product.Components);
        Label aForeign = new ExchangeDocument().NewShape();
        Assert.Throws<ArgumentException>(() => aDoc.SetName(aForeign, "x"));
        Assert.Throws<ArgumentException>(() => aDoc.SetName(default, "x"));
    }
}
