// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Interf.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Tools;

using Occt;

namespace Kerf.Engine.Booleans.Data;

/// <summary>
/// An interference between two shapes: their indices in the data structure and the index of the new shape it made.
/// The root of the interference classes.
/// </summary>
/// <remarks>
/// The interference classes are classes, working objects held in the data structure's vectors and changed in place
/// (decision 631): <c>NCollection_DynamicArray&lt;T&gt;.Appended()</c> appends null for a class, so ported code
/// writes <c>aVVs.Append(new BOPDS_InterfVV())</c> where OCCT writes <c>aVVs.Appended()</c>. OCCT's
/// <c>std::optional&lt;int&gt; GetIndexNew()</c> is <c>int?</c>.
/// </remarks>
internal abstract class Interf
{
    protected int _index1;
    protected int _index2;
    protected int _indexNew;

    protected Interf()
    {
        _index1 = -1;
        _index2 = -1;
        _indexNew = -1;
    }

    /// <summary>Sets the indices of the interfering shapes.</summary>
    internal void SetIndices(int index1, int index2)
    {
        _index1 = index1;
        _index2 = index2;
    }

    /// <summary>The indices of the interfering shapes.</summary>
    public void Indices(out int index1, out int index2)
    {
        index1 = _index1;
        index2 = _index2;
    }

    internal void SetIndex1(int index) => _index1 = index;

    internal void SetIndex2(int index) => _index2 = index;

    public int Index1() => _index1;

    public int Index2() => _index2;

    /// <summary>The index of the other shape than <paramref name="i"/>, or -1 if neither is <paramref name="i"/>.</summary>
    public int OppositeIndex(int i)
    {
        if (i == _index1)
        {
            return _index2;
        }
        else if (i == _index2)
        {
            return _index1;
        }
        else
        {
            return -1;
        }
    }

    /// <summary>True if the interference involves the shape.</summary>
    public bool Contains(int index) => _index1 == index || _index2 == index;

    internal void SetIndexNew(int index) => _indexNew = index;

    /// <summary>The index of the new shape.</summary>
    public int IndexNew() => _indexNew;

    /// <summary>True if the interference has a new shape; its index then.</summary>
    public bool HasIndexNew(out int index)
    {
        index = _indexNew;
        return _indexNew >= 0;
    }

    /// <summary>True if the interference has a new shape (an index other than -1).</summary>
    public bool HasIndexNew() => _indexNew != -1;

    /// <summary>The index of the new shape, or null (OCCT's <c>std::optional</c>).</summary>
    public int? GetIndexNew() => _indexNew != -1 ? _indexNew : null;
}

/// <summary>A vertex/vertex interference.</summary>
internal class InterfVV : Interf
{
}

/// <summary>A vertex/edge interference, with the vertex's parameter on the edge's curve.</summary>
internal class InterfVE : Interf
{
    protected double _parameter;

    public InterfVE()
    {
        _parameter = 0.0;
    }

    internal void SetParameter(double t) => _parameter = t;

    /// <summary>The parameter of the vertex's point on the edge's curve.</summary>
    public double Parameter() => _parameter;
}

/// <summary>A vertex/face interference, with the vertex's parameters on the face's surface.</summary>
internal class InterfVF : Interf
{
    protected double _u;
    protected double _v;

    public InterfVF()
    {
        _u = 0.0;
        _v = 0.0;
    }

    internal void SetUV(double u, double v)
    {
        _u = u;
        _v = v;
    }

    /// <summary>The parameters of the vertex's point on the face's surface.</summary>
    public void UV(out double u, out double v)
    {
        u = _u;
        v = _v;
    }
}

/// <summary>An edge/edge interference, with its common part.</summary>
internal class InterfEE : Interf
{
    protected CommonPrt _commonPart = new();

    /// <summary>Sets the common part (OCCT's assignment: see <see cref="CommonPrt"/>).</summary>
    internal void SetCommonPart(CommonPrt cp) => _commonPart.Assign(cp);

    /// <summary>The common part (the interference's own object; read only for callers).</summary>
    public CommonPrt CommonPart() => _commonPart;
}

/// <summary>An edge/face interference, with its common part.</summary>
internal class InterfEF : Interf
{
    protected CommonPrt _commonPart = new();

    /// <summary>Sets the common part (OCCT's assignment: see <see cref="CommonPrt"/>).</summary>
    internal void SetCommonPart(CommonPrt cp) => _commonPart.Assign(cp);

    /// <summary>The common part (the interference's own object; read only for callers).</summary>
    public CommonPrt CommonPart() => _commonPart;
}

/// <summary>A face/face interference: the intersection curves and points, and whether the faces are tangent.</summary>
internal class InterfFF : Interf
{
    protected bool _tangentFaces;
    protected List<SectionCurve> _curves = new();
    protected List<DataPoint> _points = new();

    public InterfFF()
    {
        _tangentFaces = false;
    }

    /// <summary>
    /// Sets the expected numbers of curves and points. OCCT sets the vectors' allocation increments; the port's
    /// vectors grow by themselves, so this does nothing.
    /// </summary>
    internal void Init(int nbCurves, int nbPoints)
    {
    }

    internal void SetTangentFaces(bool flag) => _tangentFaces = flag;

    /// <summary>True if the faces are tangent.</summary>
    public bool TangentFaces() => _tangentFaces;

    /// <summary>The intersection curves (the interference's own vector; read only for callers).</summary>
    public List<SectionCurve> Curves() => _curves;

    internal List<SectionCurve> ChangeCurves() => _curves;

    /// <summary>The intersection points (the interference's own vector; read only for callers).</summary>
    public List<DataPoint> Points() => _points;

    internal List<DataPoint> ChangePoints() => _points;
}

/// <summary>A vertex/solid interference.</summary>
internal class InterfVZ : Interf
{
}

/// <summary>An edge/solid interference.</summary>
internal class InterfEZ : Interf
{
}

/// <summary>A face/solid interference.</summary>
internal class InterfFZ : Interf
{
}

/// <summary>A solid/solid interference.</summary>
internal class InterfZZ : Interf
{
}
