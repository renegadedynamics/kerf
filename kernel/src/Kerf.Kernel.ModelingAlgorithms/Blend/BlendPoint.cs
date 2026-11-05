// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_Point.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_Point.lxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_Point.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// A point of a blend's walking line: the section's extremities on the two supports (surfaces, curves or
/// restrictions), their parameters, the guide's parameter and, unless it is a tangency point, the tangents.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. <c>new Blend_Point()</c> is a tangency point, as OCCT's default constructor makes
/// it; <c>default</c> is not (its tangency flag is false). OCCT leaves the fields a constructor doesn't set
/// uninitialised; here they are zero.
/// </remarks>
internal partial struct BlendPoint : IEquatable<BlendPoint>
{
    private Point3D _pt1;
    private Point3D _pt2;
    private Vector3D _tg1;
    private Vector3D _tg2;
    private double _prm;
    private double _u1;
    private double _v1;
    private double _u2;
    private double _v2;
    private double _pc1;
    private double _pc2;
    private double _utg12d;
    private double _vtg12d;
    private double _utg22d;
    private double _vtg22d;
    private bool _hass1;
    private bool _hass2;
    private bool _hasc1;
    private bool _hasc2;
    private bool _istgt;

    /// <summary>An empty tangency point.</summary>
    public BlendPoint()
    {
        _istgt = true;
    }

    /// <summary>A point on two surfaces with its tangents.</summary>
    public BlendPoint(in Point3D p1, in Point3D p2, double param, double u1, double v1, double u2, double v2,
                       in Vector3D tg1, in Vector3D tg2, in Vector2D tg12d, in Vector2D tg22d)
    {
        _pt1 = p1;
        _pt2 = p2;
        _tg1 = tg1;
        _tg2 = tg2;
        _prm = param;
        _u1 = u1;
        _v1 = v1;
        _u2 = u2;
        _v2 = v2;
        _utg12d = tg12d.X;
        _vtg12d = tg12d.Y;
        _utg22d = tg22d.X;
        _vtg22d = tg22d.Y;
        _hass1 = true;
        _hass2 = true;
        _hasc1 = false;
        _hasc2 = false;
        _istgt = false;
    }

    /// <summary>A tangency point on two surfaces.</summary>
    public BlendPoint(in Point3D p1, in Point3D p2, double param, double u1, double v1, double u2, double v2)
    {
        _pt1 = p1;
        _pt2 = p2;
        _prm = param;
        _u1 = u1;
        _v1 = v1;
        _u2 = u2;
        _v2 = v2;
        _hass1 = true;
        _hass2 = true;
        _hasc1 = false;
        _hasc2 = false;
        _istgt = true;
    }

    /// <summary>A point on a surface and a curve with its tangents.</summary>
    public BlendPoint(in Point3D ps, in Point3D pc, double param, double u, double v, double w, in Vector3D tgs,
                       in Vector3D tgc, in Vector2D tg2d)
    {
        _pt1 = ps;
        _pt2 = pc;
        _tg1 = tgs;
        _tg2 = tgc;
        _prm = param;
        _u1 = u;
        _v1 = v;
        _pc2 = w;
        _utg12d = tg2d.X;
        _vtg12d = tg2d.Y;
        _hass1 = true;
        _hass2 = false;
        _hasc1 = false;
        _hasc2 = true;
        _istgt = false;
    }

    /// <summary>A tangency point on a surface and a curve.</summary>
    public BlendPoint(in Point3D ps, in Point3D pc, double param, double u, double v, double w)
    {
        _pt1 = ps;
        _pt2 = pc;
        _prm = param;
        _u1 = u;
        _v1 = v;
        _pc2 = w;
        _hass1 = true;
        _hass2 = false;
        _hasc1 = false;
        _hasc2 = true;
        _istgt = true;
    }

    /// <summary>A point on a surface and a restriction with its tangents.</summary>
    public BlendPoint(in Point3D p1, in Point3D p2, double param, double u1, double v1, double u2, double v2,
                       double pc, in Vector3D tg1, in Vector3D tg2, in Vector2D tg12d, in Vector2D tg22d)
    {
        _pt1 = p1;
        _pt2 = p2;
        _tg1 = tg1;
        _tg2 = tg2;
        _prm = param;
        _u1 = u1;
        _v1 = v1;
        _u2 = u2;
        _v2 = v2;
        _pc2 = pc;
        _utg12d = tg12d.X;
        _vtg12d = tg12d.Y;
        _utg22d = tg22d.X;
        _vtg22d = tg22d.Y;
        _hass1 = true;
        _hass2 = true;
        _hasc1 = false;
        _hasc2 = true;
        _istgt = false;
    }

    /// <summary>A tangency point on a surface and a restriction.</summary>
    public BlendPoint(in Point3D p1, in Point3D p2, double param, double u1, double v1, double u2, double v2,
                       double pc)
    {
        _pt1 = p1;
        _pt2 = p2;
        _prm = param;
        _u1 = u1;
        _v1 = v1;
        _u2 = u2;
        _v2 = v2;
        _pc2 = pc;
        _hass1 = true;
        _hass2 = true;
        _hasc1 = false;
        _hasc2 = true;
        _istgt = true;
    }

    /// <summary>A point on two restrictions with its tangents.</summary>
    public BlendPoint(in Point3D p1, in Point3D p2, double param, double u1, double v1, double u2, double v2,
                       double pc1, double pc2, in Vector3D tg1, in Vector3D tg2, in Vector2D tg12d, in Vector2D tg22d)
    {
        _pt1 = p1;
        _pt2 = p2;
        _tg1 = tg1;
        _tg2 = tg2;
        _prm = param;
        _u1 = u1;
        _v1 = v1;
        _u2 = u2;
        _v2 = v2;
        _pc1 = pc1;
        _pc2 = pc2;
        _utg12d = tg12d.X;
        _vtg12d = tg12d.Y;
        _utg22d = tg22d.X;
        _vtg22d = tg22d.Y;
        _hass1 = true;
        _hass2 = true;
        _hasc1 = true;
        _hasc2 = true;
        _istgt = false;
    }

    /// <summary>A tangency point on two restrictions.</summary>
    public BlendPoint(in Point3D p1, in Point3D p2, double param, double u1, double v1, double u2, double v2,
                       double pc1, double pc2)
    {
        _pt1 = p1;
        _pt2 = p2;
        _prm = param;
        _u1 = u1;
        _v1 = v1;
        _u2 = u2;
        _v2 = v2;
        _pc1 = pc1;
        _pc2 = pc2;
        _hass1 = true;
        _hass2 = true;
        _hasc1 = true;
        _hasc2 = true;
        _istgt = true;
    }

    internal void SetValue(in Point3D p1, in Point3D p2, double param, double u1, double v1, double u2, double v2,
                           in Vector3D tg1, in Vector3D tg2, in Vector2D tg12d, in Vector2D tg22d)
    {
        _pt1 = p1;
        _pt2 = p2;
        _prm = param;
        _u1 = u1;
        _v1 = v1;
        _hass1 = true;
        _u2 = u2;
        _v2 = v2;
        _hass2 = true;
        _hasc1 = false;
        _hasc2 = false;
        _istgt = false;
        _tg1 = tg1;
        _tg2 = tg2;
        _utg12d = tg12d.X;
        _vtg12d = tg12d.Y;
        _utg22d = tg22d.X;
        _vtg22d = tg22d.Y;
    }

    internal void SetValue(in Point3D p1, in Point3D p2, double param, double u1, double v1, double u2, double v2)
    {
        _pt1 = p1;
        _pt2 = p2;
        _prm = param;
        _u1 = u1;
        _v1 = v1;
        _hass1 = true;
        _u2 = u2;
        _v2 = v2;
        _hass2 = true;
        _hasc1 = false;
        _hasc2 = false;
        _istgt = true;
    }

    internal void SetValue(in Point3D ps, in Point3D pc, double param, double u, double v, double w, in Vector3D tgs,
                           in Vector3D tgc, in Vector2D tg2d)
    {
        _pt1 = ps;
        _pt2 = pc;
        _prm = param;
        _u1 = u;
        _v1 = v;
        _hass1 = true;
        _hass2 = false;
        _hasc1 = false;
        _pc2 = w;
        _hasc2 = true;
        _istgt = false;
        _tg1 = tgs;
        _tg2 = tgc;
        _utg12d = tg2d.X;
        _vtg12d = tg2d.Y;
    }

    internal void SetValue(in Point3D ps, in Point3D pc, double param, double u, double v, double w)
    {
        _pt1 = ps;
        _pt2 = pc;
        _prm = param;
        _u1 = u;
        _v1 = v;
        _hass1 = true;
        _hass2 = false;
        _hasc1 = false;
        _pc2 = w;
        _hasc2 = true;
        _istgt = true;
    }

    internal void SetValue(in Point3D p1, in Point3D p2, double param, double u1, double v1, double u2, double v2,
                           double pc, in Vector3D tg1, in Vector3D tg2, in Vector2D tg12d, in Vector2D tg22d)
    {
        _pt1 = p1;
        _pt2 = p2;
        _prm = param;
        _u1 = u1;
        _v1 = v1;
        _hass1 = true;
        _u2 = u2;
        _v2 = v2;
        _hass2 = true;
        _hasc1 = false;
        _pc2 = pc;
        _hasc2 = true;
        _istgt = false;
        _tg1 = tg1;
        _tg2 = tg2;
        _utg12d = tg12d.X;
        _vtg12d = tg12d.Y;
        _utg22d = tg22d.X;
        _vtg22d = tg22d.Y;
    }

    internal void SetValue(in Point3D p1, in Point3D p2, double param, double u1, double v1, double u2, double v2,
                           double pc)
    {
        _pt1 = p1;
        _pt2 = p2;
        _prm = param;
        _u1 = u1;
        _v1 = v1;
        _hass1 = true;
        _u2 = u2;
        _v2 = v2;
        _hass2 = true;
        _hasc1 = false;
        _pc2 = pc;
        _hasc2 = true;
        _istgt = true;
    }

    internal void SetValue(in Point3D p1, in Point3D p2, double param, double u1, double v1, double u2, double v2,
                           double pc1, double pc2, in Vector3D tg1, in Vector3D tg2, in Vector2D tg12d,
                           in Vector2D tg22d)
    {
        _pt1 = p1;
        _pt2 = p2;
        _prm = param;
        _u1 = u1;
        _v1 = v1;
        _hass1 = true;
        _u2 = u2;
        _v2 = v2;
        _hass2 = true;
        _pc1 = pc1;
        _hasc1 = true;
        _pc2 = pc2;
        _hasc2 = true;
        _istgt = false;
        _tg1 = tg1;
        _tg2 = tg2;
        _utg12d = tg12d.X;
        _vtg12d = tg12d.Y;
        _utg22d = tg22d.X;
        _vtg22d = tg22d.Y;
    }

    internal void SetValue(in Point3D p1, in Point3D p2, double param, double u1, double v1, double u2, double v2,
                           double pc1, double pc2)
    {
        _pt1 = p1;
        _pt2 = p2;
        _prm = param;
        _u1 = u1;
        _v1 = v1;
        _hass1 = true;
        _u2 = u2;
        _v2 = v2;
        _hass2 = true;
        _pc1 = pc1;
        _hasc1 = true;
        _pc2 = pc2;
        _hasc2 = true;
        _istgt = true;
    }

    internal void SetValue(in Point3D p1, in Point3D p2, double param, double pc1, double pc2)
    {
        _pt1 = p1;
        _pt2 = p2;
        _prm = param;
        _hass1 = false;
        _hass2 = false;
        _pc1 = pc1;
        _hasc1 = true;
        _pc2 = pc2;
        _hasc2 = true;
        _istgt = true;
    }

    internal void SetParameter(double param)
    {
        _prm = param;
    }

    /// <summary>
    /// A copy that is a tangency point on two curves only (the twin of the five-argument <c>SetValue</c>; the
    /// constructors are the twins of the others).
    /// </summary>
    public readonly BlendPoint WithValue(in Point3D p1, in Point3D p2, double param, double pc1, double pc2)
    {
        BlendPoint copy = this;
        copy.SetValue(p1, p2, param, pc1, pc2);
        return copy;
    }

    /// <summary>A copy with the guide's parameter <paramref name="param"/>.</summary>
    public readonly BlendPoint WithParameter(double param)
    {
        BlendPoint copy = this;
        copy.SetParameter(param);
        return copy;
    }

    /// <summary>The parameter on the guide.</summary>
    public readonly double Parameter()
    {
        return _prm;
    }

    /// <summary>True when the point carries no tangents.</summary>
    public readonly bool IsTangencyPoint
    {
        get
        {
            return _istgt;
        }
    }

    public readonly Point3D PointOnS1()
    {
        return _pt1;
    }

    public readonly Point3D PointOnS2()
    {
        return _pt2;
    }

    /// <summary>The parameters on the first surface; throws <see cref="DomainErrorException"/> if there are none.</summary>
    public readonly void ParametersOnS1(out double u1, out double v1)
    {
        if (!_hass1)
        {
            throw new DomainErrorException();
        }

        u1 = _u1;
        v1 = _v1;
    }

    /// <summary>The parameters on the second surface; throws <see cref="DomainErrorException"/> if there are none.</summary>
    public readonly void ParametersOnS2(out double u2, out double v2)
    {
        if (!_hass2)
        {
            throw new DomainErrorException();
        }

        u2 = _u2;
        v2 = _v2;
    }

    public readonly Vector3D TangentOnS1()
    {
        if (_istgt)
        {
            throw new DomainErrorException();
        }

        return _tg1;
    }

    public readonly Vector3D TangentOnS2()
    {
        if (_istgt)
        {
            throw new DomainErrorException();
        }

        return _tg2;
    }

    public readonly Vector2D Tangent2dOnS1()
    {
        if (_istgt || !_hass1)
        {
            throw new DomainErrorException();
        }

        return new Vector2D(_utg12d, _vtg12d);
    }

    public readonly Vector2D Tangent2dOnS2()
    {
        if (_istgt || !_hass2)
        {
            throw new DomainErrorException();
        }

        return new Vector2D(_utg22d, _vtg22d);
    }

    public readonly Point3D PointOnS()
    {
        return _pt1;
    }

    public readonly Point3D PointOnC()
    {
        return _pt2;
    }

    public readonly void ParametersOnS(out double u1, out double v1)
    {
        if (!_hass1)
        {
            throw new DomainErrorException();
        }

        u1 = _u1;
        v1 = _v1;
    }

    public readonly double ParameterOnC()
    {
        if (!_hasc2)
        {
            throw new DomainErrorException();
        }

        return _pc2;
    }

    public readonly Vector3D TangentOnS()
    {
        if (_istgt || !_hass1)
        {
            throw new DomainErrorException();
        }

        return _tg1;
    }

    public readonly Vector3D TangentOnC()
    {
        if (_istgt)
        {
            throw new DomainErrorException();
        }

        return _tg2;
    }

    public readonly Vector2D Tangent2d()
    {
        if (_istgt || !_hass1)
        {
            throw new DomainErrorException();
        }

        return new Vector2D(_utg12d, _vtg12d);
    }

    public readonly Point3D PointOnC1()
    {
        return _pt1;
    }

    public readonly Point3D PointOnC2()
    {
        return _pt2;
    }

    public readonly double ParameterOnC1()
    {
        if (!_hasc1)
        {
            throw new DomainErrorException();
        }

        return _pc1;
    }

    public readonly double ParameterOnC2()
    {
        if (!_hasc2)
        {
            throw new DomainErrorException();
        }

        return _pc2;
    }

    public readonly Vector3D TangentOnC1()
    {
        if (_istgt || !_hass1)
        {
            throw new DomainErrorException();
        }

        return _tg1;
    }

    public readonly Vector3D TangentOnC2()
    {
        if (_istgt)
        {
            throw new DomainErrorException();
        }

        return _tg2;
    }

    public readonly bool Equals(BlendPoint other)
        => _pt1.Equals(other._pt1) && _pt2.Equals(other._pt2) && _tg1.Equals(other._tg1) && _tg2.Equals(other._tg2)
           && _prm.Equals(other._prm) && _u1.Equals(other._u1) && _v1.Equals(other._v1) && _u2.Equals(other._u2)
           && _v2.Equals(other._v2) && _pc1.Equals(other._pc1) && _pc2.Equals(other._pc2) && _utg12d.Equals(other._utg12d)
           && _vtg12d.Equals(other._vtg12d) && _utg22d.Equals(other._utg22d) && _vtg22d.Equals(other._vtg22d)
           && _hass1 == other._hass1 && _hass2 == other._hass2 && _hasc1 == other._hasc1 && _hasc2 == other._hasc2
           && _istgt == other._istgt;

    public override readonly bool Equals(object? obj) => obj is BlendPoint other && Equals(other);

    public override readonly int GetHashCode()
    {
        HashCode h = new();
        h.Add(_pt1);
        h.Add(_pt2);
        h.Add(_tg1);
        h.Add(_tg2);
        h.Add(_prm);
        h.Add(_u1);
        h.Add(_v1);
        h.Add(_u2);
        h.Add(_v2);
        h.Add(_pc1);
        h.Add(_pc2);
        h.Add(_istgt);
        return h.ToHashCode();
    }

    public static bool operator ==(in BlendPoint left, in BlendPoint right) => left.Equals(right);

    public static bool operator !=(in BlendPoint left, in BlendPoint right) => !left.Equals(right);

    public override readonly string ToString()
        => Constants.Format("Blend_Point", _prm, _pt1.X, _pt1.Y, _pt1.Z, _pt2.X, _pt2.Y, _pt2.Z, _u1, _v1, _u2, _v2, _pc1, _pc2);
}
