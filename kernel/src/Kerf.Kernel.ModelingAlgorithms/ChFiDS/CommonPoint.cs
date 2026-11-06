// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_CommonPoint.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_CommonPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets.Data;

/// <summary>
/// The start or end point of a fillet, common to two adjacent fillets and to an edge of one of the two faces the
/// fillet is built on.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. <see cref="SurfData"/> holds four of them and the fillet builders change
/// them in place through its <c>Change*</c> accessors.
/// </remarks>
internal partial struct CommonPoint : IEquatable<CommonPoint>
{
    private Edge _arc;
    private Vertex _vtx;
    private Point3D _point;
    private Vector3D _vector;
    private double _tol;
    private double _prmarc;
    private double _prmtg;
    private Orientation _traarc;
    private bool _isonarc;
    private bool _isvtx;
    private bool _hasvector;

    /// <summary>Empty constructor.</summary>
    public CommonPoint()
    {
        _arc = new Edge();
        _vtx = new Vertex();
        _point = new Point3D();
        _vector = new Vector3D();
        _tol = 0.0;
        _prmarc = 0.0;
        _prmtg = 0.0;
        _traarc = Forward;
        _isonarc = false;
        _isvtx = false;
        _hasvector = false;
    }

    /// <summary>Default value for all fields.</summary>
    internal void Reset()
    {
        _tol = 0.0;
        _isvtx = false;
        _isonarc = false;
        _hasvector = false;
    }

    /// <summary>The point is the vertex <paramref name="vertex"/> of the restriction of one of the surfaces.</summary>
    internal void SetVertex(in Vertex vertex)
    {
        _isvtx = true;
        _vtx = vertex;
    }

    /// <summary>The point is on the arc <paramref name="a"/>, at parameter <paramref name="param"/>.</summary>
    internal void SetArc(double tol, in Edge a, double param, Orientation tArc)
    {
        _isonarc = true;
        if (tol > _tol)
        {
            _tol = tol;
        }

        _arc = a;
        _prmarc = param;
        _traarc = tArc;
    }

    /// <summary>Sets the parameter on the spine.</summary>
    internal void SetParameter(double param)
    {
        _prmtg = param;
    }

    /// <summary>Sets the 3d point, for a point that is neither a vertex nor on an arc.</summary>
    internal void SetPoint(in Point3D point)
    {
        _point = point;
    }

    /// <summary>Sets the output 3d vector.</summary>
    internal void SetVector(in Vector3D vector)
    {
        _hasvector = true;
        _vector = vector;
    }

    /// <summary>Raises the fuzziness on the point to <paramref name="tol"/>.</summary>
    internal void SetTolerance(double tol)
    {
        if (tol > _tol)
        {
            _tol = tol;
        }
    }

    /// <summary>A copy reset to the default values.</summary>
    public readonly CommonPoint Reseted()
    {
        CommonPoint copy = this;
        copy.Reset();
        return copy;
    }

    public readonly CommonPoint WithVertex(in Vertex vertex)
    {
        CommonPoint copy = this;
        copy.SetVertex(vertex);
        return copy;
    }

    public readonly CommonPoint WithArc(double tol, in Edge a, double param, Orientation tArc)
    {
        CommonPoint copy = this;
        copy.SetArc(tol, a, param, tArc);
        return copy;
    }

    public readonly CommonPoint WithParameter(double param)
    {
        CommonPoint copy = this;
        copy.SetParameter(param);
        return copy;
    }

    public readonly CommonPoint WithPoint(in Point3D point)
    {
        CommonPoint copy = this;
        copy.SetPoint(point);
        return copy;
    }

    public readonly CommonPoint WithVector(in Vector3D vector)
    {
        CommonPoint copy = this;
        copy.SetVector(vector);
        return copy;
    }

    public readonly CommonPoint WithTolerance(double tol)
    {
        CommonPoint copy = this;
        copy.SetTolerance(tol);
        return copy;
    }

    /// <summary>The fuzziness on the point.</summary>
    public readonly double Tolerance()
    {
        return _tol;
    }

    /// <summary>True if the point is a vertex of the restriction of the surface.</summary>
    public readonly bool IsVertex
    {
        get
        {
            return _isvtx;
        }
    }

    /// <summary>The vertex; throws <see cref="DomainErrorException"/> if the point is not a vertex.</summary>
    public readonly Vertex Vertex()
    {
        if (!_isvtx)
        {
            throw new DomainErrorException();
        }

        return _vtx;
    }

    /// <summary>True if the point is on an edge of the restriction of the surface.</summary>
    public readonly bool IsOnArc
    {
        get
        {
            return _isonarc;
        }
    }

    /// <summary>The arc of restriction containing the point.</summary>
    public readonly Edge Arc()
    {
        if (!_isonarc)
        {
            throw new DomainErrorException("CommonPoint not on Arc");
        }

        return _arc;
    }

    /// <summary>The transition of the point on the arc.</summary>
    public readonly Orientation TransitionOnArc()
    {
        if (!_isonarc)
        {
            throw new DomainErrorException("CommonPoint not on Arc");
        }

        return _traarc;
    }

    /// <summary>The parameter of the point on the arc.</summary>
    public readonly double ParameterOnArc()
    {
        if (!_isonarc)
        {
            throw new DomainErrorException("CommonPoint not on Arc");
        }

        return _prmarc;
    }

    /// <summary>The parameter on the spine.</summary>
    public readonly double Parameter()
    {
        return _prmtg;
    }

    /// <summary>The 3d point.</summary>
    public readonly Point3D Point()
    {
        return _point;
    }

    /// <summary>True if the output vector is stored.</summary>
    public readonly bool HasVector
    {
        get
        {
            return _hasvector;
        }
    }

    /// <summary>The output 3d vector.</summary>
    public readonly Vector3D Vector()
    {
        if (!_hasvector)
        {
            throw new DomainErrorException("ChFiDS_CommonPoint::Vector");
        }

        return _vector;
    }

    public readonly bool Equals(CommonPoint other)
        => _arc.Equals(other._arc) && _vtx.Equals(other._vtx) && _point.Equals(other._point) && _vector.Equals(other._vector)
           && _tol.Equals(other._tol) && _prmarc.Equals(other._prmarc) && _prmtg.Equals(other._prmtg)
           && _traarc == other._traarc && _isonarc == other._isonarc && _isvtx == other._isvtx
           && _hasvector == other._hasvector;

    public override readonly bool Equals(object? obj) => obj is CommonPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_arc, _vtx, _point, _tol, _prmarc, _prmtg, _isonarc, _isvtx);

    public static bool operator ==(in CommonPoint left, in CommonPoint right) => left.Equals(right);

    public static bool operator !=(in CommonPoint left, in CommonPoint right) => !left.Equals(right);

    public override readonly string ToString()
        => Constants.Format("ChFiDS_CommonPoint", _point.X, _point.Y, _point.Z, _tol, _prmarc, _prmtg);
}
