// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_PntOnFace.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_PntOnFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>A face, a 3D point, its UV parameters on the face and a validity flag.</summary>
/// <remarks>
/// A struct, as OCCT's by-value class; the setters are internal. OCCT declares <c>IsValid()</c> without defining it
/// (only <see cref="Valid"/> exists), so it is left out.
/// </remarks>
public partial struct PntOnFace : IEquatable<PntOnFace>
{
    private bool _isValid;
    private Point3D _pnt;
    private double _u;
    private double _v;
    private Face _face;

    /// <summary>An invalid point with the parameters (99, 99).</summary>
    public PntOnFace()
    {
        _isValid = false;
        _pnt = new Point3D();
        _u = 99.0;
        _v = 99.0;
        _face = new Face();
    }

    /// <summary>Sets the face, the point and its parameters (not the validity flag).</summary>
    internal void Init(in Face aF, in Point3D aP, double anU, double aV)
    {
        _face = aF;
        _pnt = aP;
        _u = anU;
        _v = aV;
    }

    internal void SetFace(in Face aF) => _face = aF;

    internal void SetPnt(in Point3D aP) => _pnt = aP;

    internal void SetParameters(double anU, double aV)
    {
        _u = anU;
        _v = aV;
    }

    internal void SetValid(bool bF) => _isValid = bF;

    public readonly bool Valid() => _isValid;

    public readonly Face Face() => _face;

    public readonly Point3D Pnt() => _pnt;

    public readonly void Parameters(out double anU, out double aV)
    {
        anU = _u;
        aV = _v;
    }

    // ---- OCCT.NET additions

    /// <summary>A copy initialised as <c>Init</c> does.</summary>
    public readonly PntOnFace Inited(in Face aF, in Point3D aP, double anU, double aV)
    {
        PntOnFace copy = this;
        copy.Init(aF, aP, anU, aV);
        return copy;
    }

    public readonly PntOnFace WithFace(in Face aF)
    {
        PntOnFace copy = this;
        copy.SetFace(aF);
        return copy;
    }

    public readonly PntOnFace WithPnt(in Point3D aP)
    {
        PntOnFace copy = this;
        copy.SetPnt(aP);
        return copy;
    }

    public readonly PntOnFace WithParameters(double anU, double aV)
    {
        PntOnFace copy = this;
        copy.SetParameters(anU, aV);
        return copy;
    }

    public readonly PntOnFace WithValid(bool bF)
    {
        PntOnFace copy = this;
        copy.SetValid(bF);
        return copy;
    }

    /// <summary>Field by field: the face by <see cref="Shape.IsEqual"/>, the numbers exactly.</summary>
    public readonly bool Equals(PntOnFace other) =>
        _isValid == other._isValid && _pnt.Equals(other._pnt) && _u.Equals(other._u)
        && _v.Equals(other._v) && _face.Equals(other._face);

    public override readonly bool Equals(object? obj) => obj is PntOnFace other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_isValid, _pnt, _u, _v, _face);

    public static bool operator ==(PntOnFace left, PntOnFace right) => left.Equals(right);

    public static bool operator !=(PntOnFace left, PntOnFace right) => !left.Equals(right);

    public override readonly string ToString() =>
        $"IntTools_PntOnFace({_pnt}, {Constants.Format("UV", _u, _v)}, valid {_isValid})";
}
