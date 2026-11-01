using System.Collections;
using O = global::Occt;

namespace Build123d.Occt;

// OCCT's collection typedefs, as OCCT.NET's NCollection templates. Their constructors and enumeration are written
// here; the rest is generated. A collection OCCT returns by reference is copied (__Wrap), as the native binding copies
// it, so a later call can't change it. Maps of shapes compare with IsSame, as TopTools_ShapeMapHasher does.

file static class Shapes
{
    public static TopoDS_Shape[] Wrap(IEnumerable<O.TopoDS_Shape> shapes) => shapes.Select(s => new TopoDS_Shape(s)).ToArray();

    public static O.NCollection_List<O.TopoDS_Shape> List(IEnumerable<O.TopoDS_Shape> shapes)
    {
        var list = new O.NCollection_List<O.TopoDS_Shape>();
        foreach (var s in shapes)
            list.Append(s);
        return list;
    }
}

public sealed partial class TopTools_ListOfShape
{
    public TopTools_ListOfShape() : this(new O.NCollection_List<O.TopoDS_Shape>()) { }

    public TopTools_ListOfShape(TopTools_ListOfShape other) : this(Shapes.List((other ?? throw new ArgumentNullException(nameof(other))).Impl)) { }

    internal static TopTools_ListOfShape __Wrap(IEnumerable<O.TopoDS_Shape> source) => new(Shapes.List(source));

    public TopoDS_Shape[] ToArray() => Shapes.Wrap(Impl);

    public IEnumerator<TopoDS_Shape> GetEnumerator() => ((IEnumerable<TopoDS_Shape>)ToArray()).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

public sealed partial class TopTools_SequenceOfShape
{
    public TopTools_SequenceOfShape() : this(new O.NCollection_Sequence<O.TopoDS_Shape>()) { }

    public TopTools_SequenceOfShape(TopTools_SequenceOfShape other) : this(Copy((other ?? throw new ArgumentNullException(nameof(other))).Impl)) { }

    private static O.NCollection_Sequence<O.TopoDS_Shape> Copy(IEnumerable<O.TopoDS_Shape> source)
    {
        var s = new O.NCollection_Sequence<O.TopoDS_Shape>();
        foreach (var x in source)
            s.Append(x);
        return s;
    }

    internal static TopTools_SequenceOfShape __Wrap(IEnumerable<O.TopoDS_Shape> source) => new(Copy(source));

    public TopoDS_Shape[] ToArray() => Shapes.Wrap(Impl);

    public IEnumerator<TopoDS_Shape> GetEnumerator() => ((IEnumerable<TopoDS_Shape>)ToArray()).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

public sealed partial class TopTools_HSequenceOfShape
{
    public TopTools_HSequenceOfShape() : this(new O.NCollection_HSequence<O.TopoDS_Shape>()) { }

    // A handle: shared, as OCCT shares it
    internal static TopTools_HSequenceOfShape __Wrap(O.NCollection_HSequence<O.TopoDS_Shape> source) => new(source);

    public TopoDS_Shape[] ToArray() => Shapes.Wrap(Impl);

    public IEnumerator<TopoDS_Shape> GetEnumerator() => ((IEnumerable<TopoDS_Shape>)ToArray()).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

public sealed partial class TopTools_MapOfShape
{
    public TopTools_MapOfShape() : this(new O.NCollection_Map<O.TopoDS_Shape>(O.TopTools_ShapeMapHasher.Instance)) { }

    public TopTools_MapOfShape(TopTools_MapOfShape other) : this(new O.NCollection_Map<O.TopoDS_Shape>((other ?? throw new ArgumentNullException(nameof(other))).Impl)) { }

    internal static TopTools_MapOfShape __Wrap(O.NCollection_Map<O.TopoDS_Shape> source) => new(new O.NCollection_Map<O.TopoDS_Shape>(source));

    public TopoDS_Shape[] ToArray() => Shapes.Wrap(Impl);

    public IEnumerator<TopoDS_Shape> GetEnumerator() => ((IEnumerable<TopoDS_Shape>)ToArray()).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

public sealed partial class TopTools_IndexedMapOfShape
{
    public TopTools_IndexedMapOfShape() : this(new O.NCollection_IndexedMap<O.TopoDS_Shape>(O.TopTools_ShapeMapHasher.Instance)) { }

    public TopTools_IndexedMapOfShape(TopTools_IndexedMapOfShape other) : this(Copy((other ?? throw new ArgumentNullException(nameof(other))).Impl)) { }

    private static O.NCollection_IndexedMap<O.TopoDS_Shape> Copy(IEnumerable<O.TopoDS_Shape> source)
    {
        var map = new O.NCollection_IndexedMap<O.TopoDS_Shape>(O.TopTools_ShapeMapHasher.Instance);
        foreach (var s in source)
            map.Add(s);
        return map;
    }

    internal static TopTools_IndexedMapOfShape __Wrap(O.NCollection_IndexedMap<O.TopoDS_Shape> source) => new(Copy(source));

    public TopoDS_Shape[] ToArray() => Shapes.Wrap(Impl);

    public IEnumerator<TopoDS_Shape> GetEnumerator() => ((IEnumerable<TopoDS_Shape>)ToArray()).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

public sealed partial class TopTools_DataMapOfShapeShape
{
    public TopTools_DataMapOfShapeShape() : this(new O.NCollection_DataMap<O.TopoDS_Shape, O.TopoDS_Shape>(O.TopTools_ShapeMapHasher.Instance)) { }

    public TopTools_DataMapOfShapeShape(TopTools_DataMapOfShapeShape other) : this(Copy((other ?? throw new ArgumentNullException(nameof(other))).Impl)) { }

    private static O.NCollection_DataMap<O.TopoDS_Shape, O.TopoDS_Shape> Copy(O.NCollection_DataMap<O.TopoDS_Shape, O.TopoDS_Shape> source)
    {
        var map = new O.NCollection_DataMap<O.TopoDS_Shape, O.TopoDS_Shape>(O.TopTools_ShapeMapHasher.Instance);
        foreach (var (k, v) in source)
            map.Bind(k, v);
        return map;
    }

    internal static TopTools_DataMapOfShapeShape __Wrap(O.NCollection_DataMap<O.TopoDS_Shape, O.TopoDS_Shape> source) => new(Copy(source));

    /// <summary>The keys, as the native binding enumerates them.</summary>
    public TopoDS_Shape[] ToArray() => Shapes.Wrap(Impl.Select(p => p.Key));

    public IEnumerator<TopoDS_Shape> GetEnumerator() => ((IEnumerable<TopoDS_Shape>)ToArray()).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

public sealed partial class TopTools_IndexedDataMapOfShapeListOfShape
{
    public TopTools_IndexedDataMapOfShapeListOfShape()
        : this(new O.NCollection_IndexedDataMap<O.TopoDS_Shape, O.NCollection_List<O.TopoDS_Shape>>(O.TopTools_ShapeMapHasher.Instance)) { }

    public TopTools_IndexedDataMapOfShapeListOfShape(TopTools_IndexedDataMapOfShapeListOfShape other)
        : this(Copy((other ?? throw new ArgumentNullException(nameof(other))).Impl)) { }

    private static O.NCollection_IndexedDataMap<O.TopoDS_Shape, O.NCollection_List<O.TopoDS_Shape>> Copy(
        O.NCollection_IndexedDataMap<O.TopoDS_Shape, O.NCollection_List<O.TopoDS_Shape>> source)
    {
        var map = new O.NCollection_IndexedDataMap<O.TopoDS_Shape, O.NCollection_List<O.TopoDS_Shape>>(O.TopTools_ShapeMapHasher.Instance);
        for (int i = 1; i <= source.Extent(); i++)
            map.Add(source.FindKey(i), Shapes.List(source.FindFromIndex(i)));
        return map;
    }

    internal static TopTools_IndexedDataMapOfShapeListOfShape __Wrap(O.NCollection_IndexedDataMap<O.TopoDS_Shape, O.NCollection_List<O.TopoDS_Shape>> source) =>
        new(Copy(source));

    /// <summary>The keys, in index order.</summary>
    public TopoDS_Shape[] ToArray()
    {
        var map = Impl;
        var keys = new TopoDS_Shape[map.Extent()];
        for (int i = 0; i < keys.Length; i++)
            keys[i] = new TopoDS_Shape(map.FindKey(i + 1));
        return keys;
    }

    public IEnumerator<TopoDS_Shape> GetEnumerator() => ((IEnumerable<TopoDS_Shape>)ToArray()).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

// Arrays

file static class Arrays
{
    public static O.NCollection_Array1<T> Copy<T>(O.NCollection_Array1<T> source)
    {
        var a = new O.NCollection_Array1<T>(source.Lower(), source.Upper());
        for (int i = source.Lower(); i <= source.Upper(); i++)
            a.SetValue(i, source.Value(i));
        return a;
    }

    public static O.NCollection_HArray1<T> CopyH<T>(O.NCollection_Array1<T> source)
    {
        var a = new O.NCollection_HArray1<T>(source.Lower(), source.Upper());
        for (int i = source.Lower(); i <= source.Upper(); i++)
            a.SetValue(i, source.Value(i));
        return a;
    }

    public static O.NCollection_Array2<T> Copy<T>(O.NCollection_Array2<T> source)
    {
        var a = new O.NCollection_Array2<T>(source.LowerRow(), source.UpperRow(), source.LowerCol(), source.UpperCol());
        for (int r = source.LowerRow(); r <= source.UpperRow(); r++)
            for (int c = source.LowerCol(); c <= source.UpperCol(); c++)
                a.SetValue(r, c, source.Value(r, c));
        return a;
    }

    /// <summary>An array from 1 of OCCT.NET's packed lists, which it returns where OCCT returns an array.</summary>
    public static O.NCollection_Array1<T> From<T>(IEnumerable<T> source)
    {
        var values = source.ToArray();
        var a = new O.NCollection_Array1<T>(1, Math.Max(values.Length, 1));
        for (int i = 0; i < values.Length; i++)
            a.SetValue(i + 1, values[i]);
        return a;
    }

    public static T[] Values<T>(O.NCollection_Array1<T> a)
    {
        var values = new T[a.Length()];
        for (int i = 0; i < values.Length; i++)
            values[i] = a.Value(a.Lower() + i);
        return values;
    }
}

public sealed partial class TColgp_Array1OfPnt
{
    public TColgp_Array1OfPnt(int lower, int upper) : this(New(lower, upper)) { }

    public TColgp_Array1OfPnt(TColgp_Array1OfPnt other) : this(Arrays.Copy((other ?? throw new ArgumentNullException(nameof(other))).Impl)) { }

    private static O.NCollection_Array1<O.gp_Pnt> New(int lower, int upper)
    {
        try { return new O.NCollection_Array1<O.gp_Pnt>(lower, upper); }
        catch (O.Standard_Failure e) { throw OcctException.From(e); }
    }

    internal static TColgp_Array1OfPnt __Wrap(O.NCollection_Array1<O.gp_Pnt> source) => new(Arrays.Copy(source));

    internal static TColgp_Array1OfPnt __Wrap(IEnumerable<O.gp_Pnt> source) => new(Arrays.From(source));

    public gp_Pnt[] ToArray() => Arrays.Values(Impl).Select(p => new gp_Pnt(p)).ToArray();

    public IEnumerator<gp_Pnt> GetEnumerator() => ((IEnumerable<gp_Pnt>)ToArray()).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

public sealed partial class TColgp_Array1OfPnt2d
{
    public TColgp_Array1OfPnt2d(int lower, int upper) : this(New(lower, upper)) { }

    public TColgp_Array1OfPnt2d(TColgp_Array1OfPnt2d other) : this(Arrays.Copy((other ?? throw new ArgumentNullException(nameof(other))).Impl)) { }

    private static O.NCollection_Array1<O.gp_Pnt2d> New(int lower, int upper)
    {
        try { return new O.NCollection_Array1<O.gp_Pnt2d>(lower, upper); }
        catch (O.Standard_Failure e) { throw OcctException.From(e); }
    }

    internal static TColgp_Array1OfPnt2d __Wrap(O.NCollection_Array1<O.gp_Pnt2d> source) => new(Arrays.Copy(source));

    internal static TColgp_Array1OfPnt2d __Wrap(IEnumerable<O.gp_Pnt2d> source) => new(Arrays.From(source));

    public gp_Pnt2d[] ToArray() => Arrays.Values(Impl).Select(p => new gp_Pnt2d(p)).ToArray();

    public IEnumerator<gp_Pnt2d> GetEnumerator() => ((IEnumerable<gp_Pnt2d>)ToArray()).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

public sealed partial class TColgp_Array1OfVec
{
    public TColgp_Array1OfVec(int lower, int upper) : this(New(lower, upper)) { }

    public TColgp_Array1OfVec(TColgp_Array1OfVec other) : this(Arrays.Copy((other ?? throw new ArgumentNullException(nameof(other))).Impl)) { }

    private static O.NCollection_Array1<O.gp_Vec> New(int lower, int upper)
    {
        try { return new O.NCollection_Array1<O.gp_Vec>(lower, upper); }
        catch (O.Standard_Failure e) { throw OcctException.From(e); }
    }

    internal static TColgp_Array1OfVec __Wrap(O.NCollection_Array1<O.gp_Vec> source) => new(Arrays.Copy(source));

    internal static TColgp_Array1OfVec __Wrap(IEnumerable<O.gp_Vec> source) => new(Arrays.From(source));

    public gp_Vec[] ToArray() => Arrays.Values(Impl).Select(p => new gp_Vec(p)).ToArray();

    public IEnumerator<gp_Vec> GetEnumerator() => ((IEnumerable<gp_Vec>)ToArray()).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

public sealed partial class TColgp_HArray1OfPnt
{
    public TColgp_HArray1OfPnt(int lower, int upper) : this(New(lower, upper)) { }

    private static O.NCollection_HArray1<O.gp_Pnt> New(int lower, int upper)
    {
        try { return new O.NCollection_HArray1<O.gp_Pnt>(lower, upper); }
        catch (O.Standard_Failure e) { throw OcctException.From(e); }
    }

    // A handle: shared, as OCCT shares it
    internal static TColgp_HArray1OfPnt __Wrap(O.NCollection_HArray1<O.gp_Pnt> source) => new(source);

    public gp_Pnt[] ToArray() => Arrays.Values(Impl).Select(p => new gp_Pnt(p)).ToArray();

    public IEnumerator<gp_Pnt> GetEnumerator() => ((IEnumerable<gp_Pnt>)ToArray()).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

public sealed partial class TColStd_Array1OfInteger
{
    public TColStd_Array1OfInteger(int lower, int upper) : this(New(lower, upper)) { }

    public TColStd_Array1OfInteger(TColStd_Array1OfInteger other) : this(Arrays.Copy((other ?? throw new ArgumentNullException(nameof(other))).Impl)) { }

    private static O.NCollection_Array1<int> New(int lower, int upper)
    {
        try { return new O.NCollection_Array1<int>(lower, upper); }
        catch (O.Standard_Failure e) { throw OcctException.From(e); }
    }

    internal static TColStd_Array1OfInteger __Wrap(O.NCollection_Array1<int> source) => new(Arrays.Copy(source));

    internal static TColStd_Array1OfInteger __Wrap(IEnumerable<int> source) => new(Arrays.From(source));

    public int[] ToArray() => Arrays.Values(Impl);

    public IEnumerator<int> GetEnumerator() => ((IEnumerable<int>)ToArray()).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

public sealed partial class TColStd_Array1OfReal
{
    public TColStd_Array1OfReal(int lower, int upper) : this(New(lower, upper)) { }

    public TColStd_Array1OfReal(TColStd_Array1OfReal other) : this(Arrays.Copy((other ?? throw new ArgumentNullException(nameof(other))).Impl)) { }

    private static O.NCollection_Array1<double> New(int lower, int upper)
    {
        try { return new O.NCollection_Array1<double>(lower, upper); }
        catch (O.Standard_Failure e) { throw OcctException.From(e); }
    }

    internal static TColStd_Array1OfReal __Wrap(O.NCollection_Array1<double> source) => new(Arrays.Copy(source));

    internal static TColStd_Array1OfReal __Wrap(IEnumerable<double> source) => new(Arrays.From(source));

    public double[] ToArray() => Arrays.Values(Impl);

    public IEnumerator<double> GetEnumerator() => ((IEnumerable<double>)ToArray()).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

public sealed partial class TColStd_HArray1OfBoolean
{
    public TColStd_HArray1OfBoolean(int lower, int upper) : this(New(lower, upper)) { }

    private static O.NCollection_HArray1<bool> New(int lower, int upper)
    {
        try { return new O.NCollection_HArray1<bool>(lower, upper); }
        catch (O.Standard_Failure e) { throw OcctException.From(e); }
    }

    internal static TColStd_HArray1OfBoolean __Wrap(O.NCollection_HArray1<bool> source) => new(source);

    public bool[] ToArray() => Arrays.Values(Impl);

    public IEnumerator<bool> GetEnumerator() => ((IEnumerable<bool>)ToArray()).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

public sealed partial class TColStd_HArray1OfReal
{
    public TColStd_HArray1OfReal(int lower, int upper) : this(New(lower, upper)) { }

    private static O.NCollection_HArray1<double> New(int lower, int upper)
    {
        try { return new O.NCollection_HArray1<double>(lower, upper); }
        catch (O.Standard_Failure e) { throw OcctException.From(e); }
    }

    internal static TColStd_HArray1OfReal __Wrap(O.NCollection_HArray1<double> source) => new(source);

    public double[] ToArray() => Arrays.Values(Impl);

    public IEnumerator<double> GetEnumerator() => ((IEnumerable<double>)ToArray()).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

public sealed partial class TColgp_Array2OfPnt
{
    public TColgp_Array2OfPnt(int rowLower, int rowUpper, int colLower, int colUpper) : this(New(rowLower, rowUpper, colLower, colUpper)) { }

    public TColgp_Array2OfPnt(TColgp_Array2OfPnt other) : this(Arrays.Copy((other ?? throw new ArgumentNullException(nameof(other))).Impl)) { }

    private static O.NCollection_Array2<O.gp_Pnt> New(int rl, int ru, int cl, int cu)
    {
        try { return new O.NCollection_Array2<O.gp_Pnt>(rl, ru, cl, cu); }
        catch (O.Standard_Failure e) { throw OcctException.From(e); }
    }

    internal static TColgp_Array2OfPnt __Wrap(O.NCollection_Array2<O.gp_Pnt> source) => new(Arrays.Copy(source));
}

public sealed partial class TColgp_HArray2OfPnt
{
    public TColgp_HArray2OfPnt(int rowLower, int rowUpper, int colLower, int colUpper) : this(New(rowLower, rowUpper, colLower, colUpper)) { }

    private static O.NCollection_HArray2<O.gp_Pnt> New(int rl, int ru, int cl, int cu)
    {
        try { return new O.NCollection_HArray2<O.gp_Pnt>(rl, ru, cl, cu); }
        catch (O.Standard_Failure e) { throw OcctException.From(e); }
    }

    internal static TColgp_HArray2OfPnt __Wrap(O.NCollection_HArray2<O.gp_Pnt> source) => new(source);
}

public sealed partial class TColStd_Array2OfReal
{
    public TColStd_Array2OfReal(int rowLower, int rowUpper, int colLower, int colUpper) : this(New(rowLower, rowUpper, colLower, colUpper)) { }

    public TColStd_Array2OfReal(TColStd_Array2OfReal other) : this(Arrays.Copy((other ?? throw new ArgumentNullException(nameof(other))).Impl)) { }

    private static O.NCollection_Array2<double> New(int rl, int ru, int cl, int cu)
    {
        try { return new O.NCollection_Array2<double>(rl, ru, cl, cu); }
        catch (O.Standard_Failure e) { throw OcctException.From(e); }
    }

    internal static TColStd_Array2OfReal __Wrap(O.NCollection_Array2<double> source) => new(Arrays.Copy(source));
}

public sealed partial class TColStd_HArray2OfReal
{
    public TColStd_HArray2OfReal(int rowLower, int rowUpper, int colLower, int colUpper) : this(New(rowLower, rowUpper, colLower, colUpper)) { }

    private static O.NCollection_HArray2<double> New(int rl, int ru, int cl, int cu)
    {
        try { return new O.NCollection_HArray2<double>(rl, ru, cl, cu); }
        catch (O.Standard_Failure e) { throw OcctException.From(e); }
    }

    internal static TColStd_HArray2OfReal __Wrap(O.NCollection_HArray2<double> source) => new(source);
}
