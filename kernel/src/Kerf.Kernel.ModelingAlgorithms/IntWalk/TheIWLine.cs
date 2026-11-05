// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheIWLineOfTheIWalking.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheIWLineOfTheIWalking_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>
/// A line computed by IntWalk_IWalking: its points (an <see cref="LineOn2S"/>), whether it is closed, its
/// first and last start points, the passing points it goes through, and a tangent vector.
/// </summary>
/// <remarks>
/// OCCT has no generic file for it: IntPatch_TheIWLineOfTheIWalking and Contap_TheIWLineOfTheIWalking are two copies
/// of the same class, named by the macro <c>IntWalk_TheIWLine</c> in IntWalk_IWalking.gxx. The port writes the class
/// once under the macro's name, and each instantiation is a sealed subclass with OCCT's name (decision 686), which
/// <see cref="IWalking{TheIWFunction, TheIWLine}"/> builds. A working object (decision 609): its mutators are
/// public, as OCCT's, and <see cref="Line"/> is the line itself, not a copy. <see cref="Value"/>,
/// <see cref="FirstPoint"/> and <see cref="LastPoint"/> return copies (OCCT's <c>const&amp;</c>).
/// </remarks>
internal class TheIWLine : Transient
{
    private readonly LineOn2S _line;
    private readonly NCollection_Sequence<Couple> _couple = new();
    private bool _closed;
    private bool _hasFirst;
    private bool _hasLast;
    private int _firstIndex;
    private int _lastIndex;
    private PathPoint _firstPoint;
    private PathPoint _lastPoint;
    private int _indextg;
    private Vector3D _vcttg;
    private bool _istgtbeg;
    private bool _istgtend;

    /// <summary>An empty line (OCCT's allocator argument is dropped).</summary>
    public TheIWLine()
    {
        _line = new LineOn2S();
        _closed = false;
        _hasFirst = false;
        _hasLast = false;
        _firstIndex = -1;
        _lastIndex = -1;
        _firstPoint = new PathPoint();
        _lastPoint = new PathPoint();
        _indextg = -1;
        _vcttg = new Vector3D();
        _istgtbeg = false;
        _istgtend = false;
    }

    /// <summary>Reverses the points and renumbers the passing points.</summary>
    public void Reverse()
    {
        _line.Reverse();
        int n = _line.PointCount;
        int nbCouple = _couple.Length();
        for (int i = 1; i <= nbCouple; i++)
        {
            _couple.SetValue(i, new Couple(n - _couple.Value(i).First() + 1, _couple.Value(i).Second()));
        }
    }

    /// <summary>Removes the points from <paramref name="index"/> on.</summary>
    public void Cut(int index)
    {
        _ = _line.Split(index);
    }

    public void AddPoint(in PntOn2S p) => _line.Add(p);

    public void AddStatusFirst(bool closed, bool hasFirst)
    {
        _closed = closed;
        _hasFirst = hasFirst;
    }

    public void AddStatusFirst(bool closed, bool hasFirst, int index, in PathPoint p)
    {
        _closed = closed;
        _hasFirst = hasFirst;
        _firstIndex = index;
        _firstPoint = p;
    }

    public void AddStatusFirstLast(bool closed, bool hasFirst, bool hasLast)
    {
        _closed = closed;
        _hasFirst = hasFirst;
        _hasLast = hasLast;
    }

    public void AddStatusLast(bool hasLast) => _hasLast = hasLast;

    public void AddStatusLast(bool hasLast, int index, in PathPoint p)
    {
        _hasLast = hasLast;
        _lastIndex = index;
        _lastPoint = p;
    }

    /// <summary>Records that the next point added is the passing point <paramref name="index"/>.</summary>
    public void AddIndexPassing(int index) => _couple.Append(new Couple(_line.PointCount + 1, index));

    public void SetTangentVector(in Vector3D v, int index)
    {
        _indextg = index;
        _vcttg = v;
    }

    public void SetTangencyAtBegining(bool isTangent) => _istgtbeg = isTangent;

    public void SetTangencyAtEnd(bool isTangent) => _istgtend = isTangent;

    public int PointCount => _line.PointCount;

    public PntOn2S Value(int index) => _line.Value(index);

    /// <summary>The line's points (the line itself).</summary>
    public LineOn2S Line() => _line;

    public bool IsClosed => _closed;

    public bool HasFirstPoint => _hasFirst;

    public bool HasLastPoint => _hasLast;

    /// <summary>The first start point. Throws Standard_DomainError if there is none.</summary>
    public PathPoint FirstPoint()
    {
        if (!_hasFirst)
        {
            throw new DomainErrorException();
        }
        return _firstPoint;
    }

    public int FirstPointIndex()
    {
        if (!_hasFirst)
        {
            throw new DomainErrorException();
        }
        return _firstIndex;
    }

    /// <summary>The last start point. Throws Standard_DomainError if there is none.</summary>
    public PathPoint LastPoint()
    {
        if (!_hasLast)
        {
            throw new DomainErrorException();
        }
        return _lastPoint;
    }

    public int LastPointIndex()
    {
        if (!_hasLast)
        {
            throw new DomainErrorException();
        }
        return _lastIndex;
    }

    public int PassingPointCount => _couple.Length();

    public void PassingPoint(int index, out int indexLine, out int indexPnts)
    {
        indexLine = _couple.Value(index).First();
        indexPnts = _couple.Value(index).Second();
    }

    /// <summary>The tangent vector and, in <paramref name="index"/>, the index of the point it is at.</summary>
    public Vector3D TangentVector(out int index)
    {
        index = _indextg;
        return _vcttg;
    }

    public bool IsTangentAtBegining => _istgtbeg;

    public bool IsTangentAtEnd => _istgtend;
}
