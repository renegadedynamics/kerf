// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Point.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Point.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Booleans.Data;

/// <summary>An intersection point of two faces: the 3D point, its 2D points on each face and the vertex index.</summary>
/// <remarks>The default point is (99, 99, 99) with 2D points (99, 99) and the index -1, as OCCT's.</remarks>
internal partial struct DataPoint : IEquatable<DataPoint>
{
    private Point3D _pnt;
    private Point2D _pnt2D1;
    private Point2D _pnt2D2;
    private int _index;

    public DataPoint()
    {
        _pnt = new Point3D(99.0, 99.0, 99.0);
        _pnt2D1 = new Point2D(99.0, 99.0);
        _pnt2D2 = _pnt2D1;
        _index = -1;
    }

    internal void SetIndex(int index) => _index = index;

    public readonly int Index() => _index;

    internal void SetPnt(in Point3D pnt) => _pnt = pnt;

    public readonly Point3D Pnt() => _pnt;

    internal void SetPnt2D1(in Point2D pnt) => _pnt2D1 = pnt;

    public readonly Point2D Pnt2D1() => _pnt2D1;

    internal void SetPnt2D2(in Point2D pnt) => _pnt2D2 = pnt;

    public readonly Point2D Pnt2D2() => _pnt2D2;

    // ---- OCCT.NET additions

    public readonly DataPoint WithIndex(int index)
    {
        DataPoint copy = this;
        copy.SetIndex(index);
        return copy;
    }

    public readonly DataPoint WithPnt(in Point3D pnt)
    {
        DataPoint copy = this;
        copy.SetPnt(pnt);
        return copy;
    }

    public readonly bool Equals(DataPoint other) =>
        _pnt.Equals(other._pnt) && _pnt2D1.Equals(other._pnt2D1) && _pnt2D2.Equals(other._pnt2D2)
        && _index == other._index;

    public override readonly bool Equals(object? obj) => obj is DataPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pnt, _pnt2D1, _pnt2D2, _index);

    public static bool operator ==(DataPoint left, DataPoint right) => left.Equals(right);

    public static bool operator !=(DataPoint left, DataPoint right) => !left.Equals(right);

    public override readonly string ToString() =>
        Constants.Format("BOPDS_Point", _pnt.X, _pnt.Y, _pnt.Z, _pnt2D1.X, _pnt2D1.Y, _pnt2D2.X, _pnt2D2.Y, _index);
}
