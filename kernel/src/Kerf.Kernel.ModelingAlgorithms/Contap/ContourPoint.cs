// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_Point.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_Point.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_Point.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.HiddenLines.Contours;

/// <summary>
/// A vertex of a contour line: most of the time the meeting of the contour and a restriction of the surface (then
/// <see cref="IsOnArc"/>), with its point, its parameters on the surface and on the line, and its logical
/// information (vertex of the domain, multiple, internal).
/// </summary>
/// <remarks>
/// A C++ value class, so a struct (decision 609): lines keep copies, and <see cref="ContourLine.Vertex"/>
/// returns a reference to the stored one. <c>new()</c> and <c>default</c> differ only in the transitions
/// (<c>new()</c> has OCCT's undecided ones), so ported code writes <c>new()</c>. The mutators are internal. The arcs
/// and vertices are adaptors, held and compared by reference, as OCCT's handles.
/// </remarks>
internal partial struct ContourPoint : IEquatable<ContourPoint>
{
    private Point3D _pt;
    private double _uparam;
    private double _vparam;
    private double _paraline;
    private bool _onarc;
    private Curve2DAdaptorBase? _arc;
    private WalkingTransition _traline;
    private WalkingTransition _traarc;
    private double _prmarc;
    private bool _isvtx;
    private HVertex? _vtx;
    private bool _ismult;
    private bool _internal;

    /// <summary>Empty constructor.</summary>
    public ContourPoint()
    {
        _pt = new Point3D();
        _uparam = 0.0;
        _vparam = 0.0;
        _paraline = 0.0;
        _onarc = false;
        _arc = null;
        _traline = new WalkingTransition();
        _traarc = new WalkingTransition();
        _prmarc = 0.0;
        _isvtx = false;
        _vtx = null;
        _ismult = false;
        _internal = false;
    }

    /// <summary>A point of the surface at (U, V).</summary>
    public ContourPoint(in Point3D pt, double u, double v)
        : this()
    {
        _pt = pt;
        _uparam = u;
        _vparam = v;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the point and its parameters on the surface (and clears the logical information but the arc's).</summary>
    internal void SetValue(in Point3D pt, double u, double v)
    {
        _pt = pt;
        _uparam = u;
        _vparam = v;
        _onarc = false;
        _isvtx = false;
        _ismult = false;
        _internal = false;
    }

    /// <summary>Sets the parameter on the contour line.</summary>
    internal void SetParameter(double para)
    {
        _paraline = para;
    }

    /// <summary>Makes the point a vertex of the domain.</summary>
    internal void SetVertex(HVertex v)
    {
        _isvtx = true;
        _vtx = v;
    }

    /// <summary>Sets the arc, the parameter on it and the transitions of the line and the arc.</summary>
    internal void SetArc(Curve2DAdaptorBase a, double param, in WalkingTransition tLine, in WalkingTransition tArc)
    {
        _onarc = true;
        _arc = a;
        _prmarc = param;
        _traline = tLine;
        _traarc = tArc;
    }

    internal void SetMultiple()
    {
        _ismult = true;
    }

    internal void SetInternal()
    {
        _internal = true;
    }

    // Queries

    /// <summary>The point.</summary>
    public readonly Point3D Value() => _pt;

    /// <summary>The parameter on the contour line (meaningless when the point is on none).</summary>
    public readonly double ParameterOnLine() => _paraline;

    /// <summary>The parameters on the surface.</summary>
    public readonly void Parameters(out double u1, out double v1)
    {
        u1 = _uparam;
        v1 = _vparam;
    }

    /// <summary>True when the point is the meeting of the contour and a restriction.</summary>
    public readonly bool IsOnArc => _onarc;

    /// <summary>The arc of restriction; throws Standard_DomainError if the point is on none.</summary>
    public readonly Curve2DAdaptorBase Arc()
    {
        if (!_onarc)
        {
            throw new DomainErrorException();
        }
        return _arc!;
    }

    /// <summary>The parameter on <see cref="Arc"/>; throws Standard_DomainError if the point is on no arc.</summary>
    public readonly double ParameterOnArc()
    {
        if (!_onarc)
        {
            throw new DomainErrorException();
        }
        return _prmarc;
    }

    /// <summary>The transition of the point on the contour; throws Standard_DomainError if the point is on no arc.</summary>
    public readonly WalkingTransition TransitionOnLine()
    {
        if (!_onarc)
        {
            throw new DomainErrorException();
        }
        return _traline;
    }

    /// <summary>The transition of the point on the arc; throws Standard_DomainError if the point is on no arc.</summary>
    public readonly WalkingTransition TransitionOnArc()
    {
        if (!_onarc)
        {
            throw new DomainErrorException();
        }
        return _traarc;
    }

    /// <summary>True if the point is a vertex of the domain.</summary>
    public readonly bool IsVertex => _isvtx;

    /// <summary>The vertex; throws Standard_DomainError if the point is not one.</summary>
    public readonly HVertex Vertex()
    {
        if (!_isvtx)
        {
            throw new DomainErrorException();
        }
        return _vtx!;
    }

    /// <summary>True if the point belongs to several lines.</summary>
    public readonly bool IsMultiple => _ismult;

    /// <summary>True if the tangent to the line at the point and the eye direction are parallel.</summary>
    public readonly bool IsInternal => _internal;

    public readonly bool Equals(ContourPoint other)
        => _pt.Equals(other._pt) && _uparam.Equals(other._uparam) && _vparam.Equals(other._vparam)
           && _paraline.Equals(other._paraline) && _onarc == other._onarc && ReferenceEquals(_arc, other._arc)
           && _traline.Equals(other._traline) && _traarc.Equals(other._traarc) && _prmarc.Equals(other._prmarc)
           && _isvtx == other._isvtx && ReferenceEquals(_vtx, other._vtx) && _ismult == other._ismult
           && _internal == other._internal;

    public override readonly bool Equals(object? obj) => obj is ContourPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pt, _uparam, _vparam, _paraline, _onarc, _prmarc, _isvtx);

    public static bool operator ==(in ContourPoint left, in ContourPoint right) => left.Equals(right);

    public static bool operator !=(in ContourPoint left, in ContourPoint right) => !left.Equals(right);

    public override readonly string ToString()
        => $"{Constants.Format("Contap_Point", _pt.X, _pt.Y, _pt.Z, _uparam, _vparam, _paraline)} onarc {_onarc} vertex {_isvtx}";
}
