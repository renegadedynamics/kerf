using System.Runtime.CompilerServices;

namespace Build123d.Occt.Tests;

/// <summary>Wrappers that keep alive what OCCT needs alive.</summary>
public class LifetimeTests
{
    /// <summary>An iterator over a box's shell, whose wrapper (and the solid's) only the iterator references.</summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    static TopoDS_Iterator IterateABoxsShell()
    {
        using var box = new BRepPrimAPI_MakeBox(1, 2, 3);
        using var shells = new TopoDS_Iterator(box.Shape());
        return new TopoDS_Iterator(shells.Value());
    }

    [Fact]
    public void AnIteratorKeepsItsShapeAlive()
    {
        // OCCT 8's TopoDS_Iterator holds no reference to the shape it iterates, so finalizing the shape's wrapper could
        // free its TShape mid-iteration (OCCT.NET's decision 1109)
        using var faces = IterateABoxsShell();
        GC.Collect();
        GC.WaitForPendingFinalizers();
        GC.Collect();
        int n = 0;
        for (; faces.More(); faces.Next())
        {
            using var face = faces.Value();
            Assert.Equal(TopAbs_ShapeEnum.TopAbs_FACE, face.ShapeType());
            n++;
        }
        Assert.Equal(6, n);
    }

    [Fact]
    public void AnIteratorHoldsTheShapeItWasLastGiven()
    {
        using var iterator = new TopoDS_Iterator();
        var shape = Hold(iterator);
        GC.Collect();
        GC.WaitForPendingFinalizers();
        Assert.True(shape.IsAlive);
        GC.KeepAlive(iterator);

        [MethodImpl(MethodImplOptions.NoInlining)]
        static WeakReference Hold(TopoDS_Iterator iterator)
        {
            using var box = new BRepPrimAPI_MakeBox(1, 2, 3);
            var solid = box.Shape();
            iterator.Initialize(solid);
            return new WeakReference(solid);
        }
    }
}
