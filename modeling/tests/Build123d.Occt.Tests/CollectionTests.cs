namespace Build123d.Occt.Tests;

/// <summary>NCollection instantiations bound by OcctGen's collection support.</summary>
public class CollectionTests
{
    static TopoDS_Shape Box()
    {
        using var box = new BRepPrimAPI_MakeBox(1, 2, 3);
        return box.Shape();
    }

    [Fact]
    public void TopExpFillsAnIndexedMap()
    {
        using var box = Box();
        using var faces = new TopTools_IndexedMapOfShape();
        TopExp.MapShapes(box, TopAbs_ShapeEnum.TopAbs_FACE, faces);
        Assert.Equal(6, faces.Extent());
        var all = faces.ToArray();
        Assert.Equal(6, all.Length);
        Assert.All(all, f => Assert.Equal(TopAbs_ShapeEnum.TopAbs_FACE, f.ShapeType()));
        Assert.Equal(3, faces.FindIndex(all[2]));
    }

    [Fact]
    public void AncestorMapsHoldListsOfShapes()
    {
        using var box = Box();
        using var map = new TopTools_IndexedDataMapOfShapeListOfShape();
        TopExp.MapShapesAndAncestors(box, TopAbs_ShapeEnum.TopAbs_EDGE, TopAbs_ShapeEnum.TopAbs_FACE, map);
        Assert.Equal(12, map.Extent());
        using var faces = map.FindFromIndex(1);
        Assert.Equal(2, faces.Extent());   // every edge of a box borders two faces
    }

    [Fact]
    public void ListsKeepTheirOrder()
    {
        using var box = Box();
        using var list = new TopTools_ListOfShape();
        using var faces = new TopExp_Explorer(box, TopAbs_ShapeEnum.TopAbs_FACE);
        var expected = new List<TopoDS_Shape>();
        for (; faces.More(); faces.Next())
        {
            expected.Add(faces.Current());
            list.Append(faces.Current());
        }
        Assert.Equal(6, list.Extent());
        var items = list.ToArray();
        for (int i = 0; i < 6; i++)
            Assert.True(items[i].IsSame(expected[i]));
    }

    [Fact]
    public void ArraysOfValuesAndTransientArraysOfPrimitives()
    {
        using var points = new TColgp_Array1OfPnt(1, 3);
        for (int i = 1; i <= 3; i++)
        {
            using var p = new gp_Pnt(i, 0, 0);
            points.SetValue(i, p);
        }
        Assert.Equal([1.0, 2.0, 3.0], points.Select(p => p.X()));

        using var flags = new TColStd_HArray1OfBoolean(0, 1);
        flags.SetValue(0, false);   // NCollection arrays aren't initialised
        flags.SetValue(1, true);
        Assert.Equal([false, true], flags.ToArray());
    }
}
