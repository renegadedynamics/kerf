// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>Unit tests of BRepAlgo_Image and BRepAlgo_AsDes, the offset algorithms' records of shapes (unit 5O1).</summary>
public class BRepAlgo_ImageAsDesTests
{
    private static Shape V(double x) => new MakeVertex(new Point3D(x, 0, 0)).Vertex();

    private static List<Shape> Items(NCollection_List<Shape> theList)
    {
        List<Shape> anItems = [];
        for (NCollection_List<Shape>.Iterator it = new(theList); it.More(); it.Next())
        {
            anItems.Add(it.Value());
        }
        return anItems;
    }

    [Fact]
    public void Image_BindAddLastImageAndRoot()
    {
        Shape a = V(0), b = V(1), c = V(2), d = V(3);
        ShapeImage anImage = new();
        anImage.SetRoot(a);
        anImage.Bind(a, b);
        anImage.Add(a, c);
        anImage.Bind(b, d);

        Assert.True(anImage.HasImage(a));
        Assert.True(anImage.IsImage(d));
        Assert.True(anImage.ImageFrom(d).IsSame(b));
        Assert.True(anImage.Root(d).IsSame(a));
        Assert.Equal([b, c], Items(anImage.Image(a)));

        NCollection_List<Shape> aLast = new();
        anImage.LastImage(a, aLast);
        Assert.Equal([d, c], Items(aLast));

        anImage.Compact();
        Assert.Equal([d, c], Items(anImage.Image(a)));
        Assert.False(anImage.HasImage(b));

        EngineAssert.Throws<ConstructionErrorException>(() => anImage.Bind(a, b));
        EngineAssert.Throws<ConstructionErrorException>(() => anImage.Add(b, c));
    }

    [Fact]
    public void Image_RemoveAndReplaceRoot()
    {
        Shape a = V(0), b = V(1), c = V(2), r = V(4);
        ShapeImage anImage = new();
        anImage.SetRoot(a);
        anImage.Bind(a, b);
        anImage.Add(a, c);
        anImage.Remove(b);
        Assert.Equal([c], Items(anImage.Image(a)));

        anImage.ReplaceRoot(a, r);
        Assert.Equal([r], Items(anImage.Roots()));
        Assert.Equal([c], Items(anImage.Image(r)));
        Assert.False(anImage.HasImage(a));
    }

    [Fact]
    public void Image_OfAShapeWithoutImages_KeepsEveryShapeAskedFor()
    {
        // OCCT: a function-level static list (occt-issues.md); per thread in the port
        Thread aThread = new(() =>
        {
            ShapeImage anImage = new();
            Shape a = V(0), b = V(1);
            Assert.Equal([a], Items(anImage.Image(a)));
            Assert.Equal([a, b], Items(anImage.Image(b)));
        });
        aThread.Start();
        aThread.Join();
    }

    [Fact]
    public void AsDes_AddReplaceRemoveAndCommonDescendants()
    {
        Shape f1 = V(0), f2 = V(1), e1 = V(2), e2 = V(3), e3 = V(4);
        AsDes anAsDes = new();
        anAsDes.Add(f1, e1);
        anAsDes.Add(f1, e2);
        anAsDes.Add(f2, e2);
        anAsDes.Add(f2, e3);

        NCollection_List<Shape> aCommon = new();
        Assert.True(anAsDes.HasCommonDescendant(f1, f2, aCommon));
        Assert.Equal([e2], Items(aCommon));
        Assert.Equal([f1, f2], Items(anAsDes.Ascendant(e2)));

        Shape e4 = V(5);
        anAsDes.Replace(e2, e4);
        Assert.False(anAsDes.HasAscendant(e2));
        Assert.Equal([e1, e4], Items(anAsDes.Descendant(f1)));
        Assert.Equal([e4, e3], Items(anAsDes.Descendant(f2)));

        anAsDes.Remove(e4);
        Assert.Equal([e1], Items(anAsDes.Descendant(f1)));
        EngineAssert.Throws<ConstructionErrorException>(() => anAsDes.Remove(f1));
        Assert.Empty(Items(anAsDes.Descendant(e1)));
    }
}
