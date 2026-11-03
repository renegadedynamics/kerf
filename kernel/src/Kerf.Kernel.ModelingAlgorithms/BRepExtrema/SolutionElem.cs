// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_SolutionElem.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Measurement.Distance.SupportType;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>
/// One solution of a distance computation on one shape: the distance, the point, and its support (a vertex, or an
/// edge with a parameter, or a face with two parameters).
/// </summary>
/// <remarks>
/// OCCT's class is used by value and has no mutators, so it is a readonly struct. <c>default</c> is OCCT's empty
/// constructor (zeros, a vertex support with null shapes).
/// </remarks>
public readonly struct SolutionElem : IEquatable<SolutionElem>
{
    private readonly double _dist;
    private readonly Point3D _point;
    private readonly SupportType _supType;
    private readonly Vertex _vertex;
    private readonly Edge _edge;
    private readonly Face _face;
    private readonly double _par1;
    private readonly double _par2;

    /// <summary>The empty solution.</summary>
    public SolutionElem()
    {
        _dist = 0.0;
        _point = new Point3D(0.0, 0.0, 0.0);
        _supType = SupportType.IsVertex;
        _par1 = 0.0;
        _par2 = 0.0;
    }

    /// <summary>A solution at a vertex.</summary>
    public SolutionElem(double dist, in Point3D point, SupportType solType, in Vertex vertex)
    {
        _dist = dist;
        _point = point;
        _supType = solType;
        _vertex = vertex;
        _par1 = 0.0;
        _par2 = 0.0;
    }

    /// <summary>A solution on an edge at the parameter <paramref name="param"/>.</summary>
    public SolutionElem(double dist, in Point3D point, SupportType solType, in Edge edge, double param)
    {
        _dist = dist;
        _point = point;
        _supType = solType;
        _edge = edge;
        _par1 = param;
        _par2 = 0.0;
    }

    /// <summary>A solution in a face at the parameters (<paramref name="u"/>, <paramref name="v"/>).</summary>
    public SolutionElem(double dist, in Point3D point, SupportType solType, in Face face, double u, double v)
    {
        _dist = dist;
        _point = point;
        _supType = solType;
        _face = face;
        _par1 = u;
        _par2 = v;
    }

    /// <summary>The distance.</summary>
    public double Dist() => _dist;

    /// <summary>The solution point.</summary>
    public Point3D Point() => _point;

    /// <summary>The kind of support: a vertex, on an edge or inside a face.</summary>
    public SupportType SupportKind() => _supType;

    /// <summary>The vertex, if the solution is a vertex.</summary>
    public Vertex Vertex() => _vertex;

    /// <summary>The edge, if the solution is on an edge.</summary>
    public Edge Edge() => _edge;

    /// <summary>The face, if the solution is in a face.</summary>
    public Face Face() => _face;

    /// <summary>The parameter on the edge.</summary>
    public void EdgeParameter(out double param) => param = _par1;

    /// <summary>The parameters on the face.</summary>
    public void FaceParameter(out double u, out double v)
    {
        u = _par1;
        v = _par2;
    }

    public bool Equals(SolutionElem other) =>
        _dist.Equals(other._dist) && _point.Equals(other._point) && _supType == other._supType
        && _vertex.Equals(other._vertex) && _edge.Equals(other._edge) && _face.Equals(other._face)
        && _par1.Equals(other._par1) && _par2.Equals(other._par2);

    public override bool Equals(object? obj) => obj is SolutionElem other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(_dist, _point, _supType, _vertex, _edge, _face, _par1, _par2);

    public static bool operator ==(in SolutionElem left, in SolutionElem right) => left.Equals(right);

    public static bool operator !=(in SolutionElem left, in SolutionElem right) => !left.Equals(right);

    public override string ToString() =>
        $"BRepExtrema_SolutionElem({_supType}, {_point}, "
        + Constants.Format("", _dist, _par1, _par2) + ")";
}
