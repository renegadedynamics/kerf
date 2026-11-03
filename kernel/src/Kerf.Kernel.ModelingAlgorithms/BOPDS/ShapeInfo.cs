// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_ShapeInfo.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_ShapeInfo.lxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_ShapeInfo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Data;

/// <summary>
/// What the data structure knows about one shape: the shape, its type, its bounding box, the indices of its
/// sub-shapes, the index of its pave blocks or face information (the reference) and a flag.
/// </summary>
/// <remarks>
/// A class, a working object of the data structure (decision 631): OCCT's by-value class is held in the data
/// structure's vector and changed in place through <c>ChangeShapeInfo</c>. OCCT's by-value copies are the copy
/// constructor, which copies the list of sub-shapes.
/// </remarks>
public class ShapeInfo
{
    protected Shape _shape = new();
    protected ShapeType _type;
    protected Box _box = new();
    protected NCollection_List<int> _subShapes;
    protected int _reference;
    protected int _flag;

    /// <summary>Empty information: type <see cref="Shape"/>, no reference, no flag.</summary>
    public ShapeInfo()
    {
        _type = global::Kerf.Topology.ShapeType.Shape;
        _subShapes = new NCollection_List<int>();
        _reference = -1;
        _flag = -1;
    }

    /// <summary>A copy of <paramref name="other"/> (OCCT's implicit copy constructor).</summary>
    public ShapeInfo(ShapeInfo other)
    {
        _shape = other._shape;
        _type = other._type;
        _box = other._box;
        _subShapes = new NCollection_List<int>(other._subShapes);
        _reference = other._reference;
        _flag = other._flag;
    }

    internal void SetShape(in Shape s) => _shape = s;

    public Shape Shape() => _shape;

    internal void SetShapeType(ShapeType type) => _type = type;

    public ShapeType ShapeType() => _type;

    internal void SetBox(in Box box) => _box = box;

    public Box Box() => _box;

    internal ref Box ChangeBox() => ref _box;

    /// <summary>The indices of the sub-shapes (the data structure's own list; read only for callers).</summary>
    public NCollection_List<int> SubShapes() => _subShapes;

    internal NCollection_List<int> ChangeSubShapes() => _subShapes;

    /// <summary>True if the shape has the sub-shape with the index.</summary>
    public bool HasSubShape(int i) => _subShapes.Contains(i);

    public bool HasReference => _reference >= 0;

    internal void SetReference(int i) => _reference = i;

    public int Reference() => _reference;

    /// <summary>True if the shape has a boundary representation.</summary>
    public bool HasBRep => DataTools.HasBRep(_type);

    /// <summary>True if the shape can take part in an interference.</summary>
    public bool IsInterfering => HasBRep || _type == global::Kerf.Topology.ShapeType.Solid;

    public bool HasFlag() => _flag >= 0;

    public bool HasFlag(out int flag)
    {
        flag = _flag;
        return _flag >= 0;
    }

    internal void SetFlag(int flag) => _flag = flag;

    public int Flag() => _flag;

    /// <summary>Writes the type and the sub-shape indices as OCCT's <c>Dump</c> prints them.</summary>
    public void Dump(TextWriter os)
    {
        ShapeType ts = ShapeType();
        os.Write(" " + global::Kerf.Engine.Topology.TopAbs.ShapeTypeToString(ts));
        os.Write(" {");
        for (NCollection_List<int>.Iterator it = new(_subShapes); it.More(); it.Next())
        {
            int n = it.Value();
            os.Write(" " + n);
        }
        os.Write(" }");
    }
}
