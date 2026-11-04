// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Extremity.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Extremity.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Extremity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// An extremity of a walking line on one of its supports: the point, its parameters, the guide's parameter, the
/// tolerance, the tangent if known, the vertex if it is one, and the arcs of restriction it lies on.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct, copied by value as OCCT copies it (decision 1059): its sequence of points on
/// arcs is copied on write (<c>AddArc</c> appends to a new sequence, <c>SetValue</c> drops it), so two copies never
/// share one. <c>new BRepBlend_Extremity()</c> is OCCT's default extremity; <c>default</c> is the same.
/// The mutators are internal, with public <c>With*</c> twins.
/// </remarks>
internal partial struct Extremity : IEquatable<Extremity>
{
    private HVertex? _vtx;
    private NCollection_Sequence<PointOnRst>? _seqpt;
    private Point3D _pt;
    private Vector3D _tang;
    private double _param;
    private double _u;
    private double _v;
    private double _tol;
    private bool _isvtx;
    private bool _hastang;

    public Extremity()
    {
        _pt = new Point3D(0, 0, 0);
        _tang = new Vector3D(0, 0, 0);
        _param = 0.0;
        _u = 0.0;
        _v = 0.0;
        _tol = 0.0;
        _isvtx = false;
        _hastang = false;
    }

    /// <summary>An extremity at <paramref name="p"/>, (<paramref name="u"/>, <paramref name="v"/>) on a surface.</summary>
    public Extremity(in Point3D p, double u, double v, double param, double tol)
    {
        _pt = p;
        _tang = new Vector3D(0, 0, 0);
        _param = param;
        _u = u;
        _v = v;
        _tol = tol;
        _isvtx = false;
        _hastang = false;
    }

    /// <summary>An extremity on the vertex <paramref name="vtx"/>.</summary>
    public Extremity(in Point3D p, double u, double v, double param, double tol, HVertex vtx)
    {
        _vtx = vtx;
        _pt = p;
        _tang = new Vector3D(0, 0, 0);
        _param = param;
        _u = u;
        _v = v;
        _tol = tol;
        _isvtx = true;
        _hastang = false;
    }

    /// <summary>An extremity at parameter <paramref name="w"/> on a curve.</summary>
    public Extremity(in Point3D p, double w, double param, double tol)
    {
        _pt = p;
        _tang = new Vector3D(0, 0, 0);
        _param = param;
        _u = w;
        _v = 0.0;
        _tol = tol;
        _isvtx = false;
        _hastang = false;
    }

    internal void SetValue(in Point3D p, double u, double v, double param, double tol)
    {
        _pt = p;
        _u = u;
        _v = v;
        _param = param;
        _tol = tol;
        _isvtx = false;
        _seqpt = null; // seqpt.Clear()
    }

    internal void SetValue(in Point3D p, double u, double v, double param, double tol, HVertex vtx)
    {
        _pt = p;
        _u = u;
        _v = v;
        _param = param;
        _tol = tol;
        _isvtx = true;
        _vtx = vtx;
        _seqpt = null; // seqpt.Clear()
    }

    internal void SetValue(in Point3D p, double w, double param, double tol)
    {
        _pt = p;
        _u = w;
        _param = param;
        _tol = tol;
        _isvtx = false;
        _seqpt = null; // seqpt.Clear()
    }

    public readonly Point3D Value() => _pt;

    internal void SetTangent(in Vector3D tangent)
    {
        _hastang = true;
        _tang = tangent;
    }

    public readonly bool HasTangent => _hastang;

    public readonly Vector3D Tangent()
    {
        if (!_hastang)
        {
            throw new DomainErrorException();
        }

        return _tang;
    }

    public readonly double Tolerance() => _tol;

    internal void SetVertex(HVertex v)
    {
        _isvtx = true;
        _vtx = v;
    }

    /// <summary>Adds the arc <paramref name="a"/> through the extremity, at parameter <paramref name="param"/>.</summary>
    internal void AddArc(Curve2DAdaptorBase a, double param, in WalkingTransition tLine, in WalkingTransition tArc)
    {
        NCollection_Sequence<PointOnRst> seq = _seqpt is null ? new() : new(_seqpt);
        seq.Append(new PointOnRst(a, param, tLine, tArc));
        _seqpt = seq;
    }

    public readonly void Parameters(out double u, out double v)
    {
        u = _u;
        v = _v;
    }

    public readonly bool IsVertex => _isvtx;

    public readonly HVertex Vertex()
    {
        if (!_isvtx)
        {
            throw new DomainErrorException();
        }

        return _vtx!;
    }

    public readonly int PointOnRstCount => _seqpt is null ? 0 : _seqpt.Length();

    public readonly PointOnRst PointOnRst(int index)
    {
        if (_seqpt is null)
        {
            throw new OutOfRangeException("NCollection_Sequence::Value");
        }

        return _seqpt.Value(index);
    }

    public readonly double Parameter() => _u;

    public readonly double ParameterOnGuide() => _param;

    public readonly Extremity WithValue(in Point3D p, double u, double v, double param, double tol)
    {
        Extremity copy = this;
        copy.SetValue(p, u, v, param, tol);
        return copy;
    }

    public readonly Extremity WithValue(in Point3D p, double u, double v, double param, double tol,
                                                  HVertex vtx)
    {
        Extremity copy = this;
        copy.SetValue(p, u, v, param, tol, vtx);
        return copy;
    }

    public readonly Extremity WithValue(in Point3D p, double w, double param, double tol)
    {
        Extremity copy = this;
        copy.SetValue(p, w, param, tol);
        return copy;
    }

    public readonly Extremity WithTangent(in Vector3D tangent)
    {
        Extremity copy = this;
        copy.SetTangent(tangent);
        return copy;
    }

    public readonly Extremity WithVertex(HVertex v)
    {
        Extremity copy = this;
        copy.SetVertex(v);
        return copy;
    }

    public readonly Extremity WithArc(Curve2DAdaptorBase a, double param, in WalkingTransition tLine,
                                                in WalkingTransition tArc)
    {
        Extremity copy = this;
        copy.AddArc(a, param, tLine, tArc);
        return copy;
    }

    public readonly bool Equals(Extremity other)
    {
        if (!(ReferenceEquals(_vtx, other._vtx) && _pt.Equals(other._pt) && _tang.Equals(other._tang)
              && _param.Equals(other._param) && _u.Equals(other._u) && _v.Equals(other._v) && _tol.Equals(other._tol)
              && _isvtx == other._isvtx && _hastang == other._hastang && PointOnRstCount == other.PointOnRstCount))
        {
            return false;
        }

        for (int i = 1; i <= PointOnRstCount; i++)
        {
            if (!PointOnRst(i).Equals(other.PointOnRst(i)))
            {
                return false;
            }
        }

        return true;
    }

    public override readonly bool Equals(object? obj) => obj is Extremity other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pt, _param, _u, _v, _tol, _isvtx, PointOnRstCount);

    public static bool operator ==(in Extremity left, in Extremity right)
        => left.Equals(right);

    public static bool operator !=(in Extremity left, in Extremity right)
        => !left.Equals(right);

    public override readonly string ToString()
        => Constants.Format("BRepBlend_Extremity", _pt.X, _pt.Y, _pt.Z, _u, _v, _param, _tol);
}
