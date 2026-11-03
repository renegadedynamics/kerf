// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PathPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PathPoint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PathPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>
/// A starting point of a walking line on the boundary of a parametric surface (IntWalk_IWalking): the 3D point, its
/// parameters (several when the point is multiple), the tangent directions in 3D and in the parametric space, and
/// the tangency and passing flags.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. As in OCCT, the parameters are a handle to a sequence that copies share:
/// <see cref="AddUV"/> on a copy is seen by the original, and <see cref="SetValue"/> gives the point a new sequence.
/// <c>new()</c> is OCCT's empty constructor (passing, tangent, no parameters); <c>default</c> has the flags false,
/// so ported code writes <c>new()</c>. The mutators are internal (translation rules, section 3); the constructor is
/// <see cref="SetValue"/>'s twin.
/// </remarks>
public partial struct PathPoint : IEquatable<PathPoint>
{
    private Point3D _pt;
    private bool _ispass;
    private bool _istgt;
    private Vector3D _vectg;
    private Direction2D _dirtg;
    private NCollection_HSequence<Xy>? _sequv;

    public PathPoint()
    {
        _pt = new Point3D();
        _ispass = true;
        _istgt = true;
        _vectg = new Vector3D();
        _dirtg = new Direction2D();
        _sequv = null;
    }

    public PathPoint(in Point3D p, double u, double v)
    {
        _pt = p;
        _ispass = true;
        _istgt = true;
        _vectg = new Vector3D();
        _dirtg = new Direction2D();
        _sequv = new NCollection_HSequence<Xy>();
        _sequv.Append(new Xy(u, v));
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValue(in Point3D p, double u, double v)
    {
        _pt = p;
        _sequv = new NCollection_HSequence<Xy>();
        _sequv.Append(new Xy(u, v));
    }

    /// <summary>Adds a pair of parameters (the point is then multiple); shared with the copies of the point.</summary>
    internal readonly void AddUV(double u, double v) => _sequv!.Append(new Xy(u, v));

    internal void SetDirections(in Vector3D v, in Direction2D d)
    {
        _istgt = false;
        _vectg = v;
        _dirtg = d;
    }

    internal void SetTangency(bool tang) => _istgt = tang;

    internal void SetPassing(bool pass) => _ispass = pass;

    // Queries

    public readonly Point3D Value() => _pt;

    public readonly void Value2d(out double u, out double v)
    {
        Xy uv = _sequv!.Sequence().First();
        u = uv.X();
        v = uv.Y();
    }

    public readonly bool IsPassingPnt => _ispass;

    public readonly bool IsTangent => _istgt;

    /// <summary>The tangent in 3D. Throws StdFail_UndefinedDerivative at a tangent point.</summary>
    public readonly Vector3D Direction3d()
    {
        if (_istgt)
        {
            throw new UndefinedDerivativeException();
        }
        return _vectg;
    }

    /// <summary>The tangent in the parametric space. Throws StdFail_UndefinedDerivative at a tangent point.</summary>
    public readonly Direction2D Direction2d()
    {
        if (_istgt)
        {
            throw new UndefinedDerivativeException();
        }
        return _dirtg;
    }

    /// <summary>The number of parameter pairs after the first.</summary>
    public readonly int Multiplicity() => (_sequv!.Length() - 1);

    /// <summary>The parameters of rank <paramref name="index"/> (0 is the first pair).</summary>
    public readonly void Parameters(int index, out double u, out double v)
    {
        Xy uv = _sequv!.Value(index + 1);
        u = uv.X();
        v = uv.Y();
    }

    /// <summary>Field equality, the parameter sequences compared by reference (OCCT's handle).</summary>
    public readonly bool Equals(PathPoint other)
        => _pt.Equals(other._pt) && _ispass == other._ispass && _istgt == other._istgt && _vectg.Equals(other._vectg)
           && _dirtg.Equals(other._dirtg) && ReferenceEquals(_sequv, other._sequv);

    public override readonly bool Equals(object? obj) => obj is PathPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pt, _ispass, _istgt, _vectg, _dirtg, _sequv);

    public static bool operator ==(in PathPoint left, in PathPoint right) => left.Equals(right);

    public static bool operator !=(in PathPoint left, in PathPoint right) => !left.Equals(right);

    public override readonly string ToString()
        => $"{Constants.Format("IntSurf_PathPoint", _pt.X(), _pt.Y(), _pt.Z())} passing {_ispass} tangent {_istgt} uv {_sequv?.Length() ?? 0}";
}
