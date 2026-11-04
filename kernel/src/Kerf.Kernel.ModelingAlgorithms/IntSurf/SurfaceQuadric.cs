// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Quadric.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Quadric.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Quadric.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>
/// An elementary surface (plane, cylinder, sphere, cone or torus) with its implicit "distance" function, its
/// gradient and its parametric evaluation, as the intersection algorithms use it.
/// </summary>
/// <remarks>
/// A value type (OCCT copies it). <c>new IntSurf_Quadric()</c> is OCCT's empty quadric (type
/// GeomAbs_OtherSurface); <c>default</c> is all zeros (type GeomAbs_Plane with a zero equation). The <c>SetValue</c>
/// methods are internal mutators; the constructors are their twins.
/// </remarks>
internal partial struct SurfaceQuadric : IEquatable<SurfaceQuadric>
{
    private CoordinateSystem _ax3;
    private Line3D _lin;
    private SurfaceKind _typ;
    private double _prm1;
    private double _prm2;
    private double _prm3;
    private double _prm4;
    private bool _ax3direc;

    public SurfaceQuadric()
    {
        _ax3 = new CoordinateSystem();
        _lin = new Line3D();
        _typ = OtherSurface;
        _prm1 = 0.0;
        _prm2 = 0.0;
        _prm3 = 0.0;
        _prm4 = 0.0;
        _ax3direc = false;
    }

    public SurfaceQuadric(in Plane3D p)
    {
        _ax3 = p.Position;
        _lin = new Line3D();
        _typ = global::Kerf.Geometry.SurfaceKind.Plane;
        _ax3direc = _ax3.IsDirect;
        p.Coefficients(out _prm1, out _prm2, out _prm3, out _prm4);
    }

    public SurfaceQuadric(in Cylinder c)
    {
        _ax3 = c.Position;
        _lin = new Line3D(_ax3.Axis);
        _typ = global::Kerf.Geometry.SurfaceKind.Cylinder;
        _prm2 = _prm3 = _prm4 = 0.0;
        _ax3direc = _ax3.IsDirect;
        _prm1 = c.Radius;
    }

    public SurfaceQuadric(in Sphere s)
    {
        _ax3 = s.Position;
        _lin = new Line3D(_ax3.Axis);
        _typ = global::Kerf.Geometry.SurfaceKind.Sphere;
        _prm2 = _prm3 = _prm4 = 0.0;
        _ax3direc = _ax3.IsDirect;
        _prm1 = s.Radius;
    }

    public SurfaceQuadric(in Cone c)
    {
        _ax3 = c.Position;
        _lin = new Line3D();
        _typ = global::Kerf.Geometry.SurfaceKind.Cone;
        _ax3direc = _ax3.IsDirect;
        _lin = _lin.WithPosition(_ax3.Axis);
        _prm1 = c.RefRadius;
        _prm2 = c.SemiAngle;
        _prm3 = Math.Cos(_prm2);
        _prm4 = 0.0;
    }

    public SurfaceQuadric(in Torus t)
    {
        _ax3 = t.Position;
        _lin = new Line3D();
        _typ = global::Kerf.Geometry.SurfaceKind.Torus;
        _ax3direc = _ax3.IsDirect;
        _lin = _lin.WithPosition(_ax3.Axis);
        _prm1 = t.MajorRadius;
        _prm2 = t.MinorRadius;
        _prm3 = 0.0;
        _prm4 = 0.0;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValue(in Plane3D p)
    {
        _typ = global::Kerf.Geometry.SurfaceKind.Plane;
        _ax3 = p.Position;
        _ax3direc = _ax3.IsDirect;
        p.Coefficients(out _prm1, out _prm2, out _prm3, out _prm4);
    }

    internal void SetValue(in Cylinder c)
    {
        _typ = global::Kerf.Geometry.SurfaceKind.Cylinder;
        _ax3 = c.Position;
        _ax3direc = _ax3.IsDirect;
        _lin = _lin.WithPosition(_ax3.Axis);
        _prm1 = c.Radius;
        _prm2 = _prm3 = _prm4 = 0.0;
    }

    internal void SetValue(in Sphere s)
    {
        _typ = global::Kerf.Geometry.SurfaceKind.Sphere;
        _ax3 = s.Position;
        _ax3direc = _ax3.IsDirect;
        _lin = _lin.WithPosition(_ax3.Axis);
        _prm1 = s.Radius;
        _prm2 = _prm3 = _prm4 = 0.0;
    }

    internal void SetValue(in Cone c)
    {
        _typ = global::Kerf.Geometry.SurfaceKind.Cone;
        _ax3 = c.Position;
        _ax3direc = _ax3.IsDirect;
        _lin = _lin.WithPosition(_ax3.Axis);
        _prm1 = c.RefRadius;
        _prm2 = c.SemiAngle;
        _prm3 = Math.Cos(_prm2);
        _prm4 = 0.0;
    }

    internal void SetValue(in Torus t)
    {
        _typ = global::Kerf.Geometry.SurfaceKind.Torus;
        _ax3 = t.Position;
        _ax3direc = _ax3.IsDirect;
        _lin = _lin.WithPosition(_ax3.Axis);
        _prm1 = t.MajorRadius;
        _prm2 = t.MinorRadius;
        _prm3 = 0.0;
        _prm4 = 0.0;
    }

    // Queries

    /// <summary>
    /// The signed "distance" of <paramref name="p"/> to the quadric (the plane equation for a plane), zero on the
    /// surface.
    /// </summary>
    public readonly double Distance(in Point3D p)
    {
        switch (_typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane: // plan
                return _prm1 * p.X + _prm2 * p.Y + _prm3 * p.Z + _prm4;
            case global::Kerf.Geometry.SurfaceKind.Cylinder: // cylindre
                return (_lin.DistanceTo(p) - _prm1);
            case global::Kerf.Geometry.SurfaceKind.Sphere: // sphere
                return (_lin.Location.DistanceTo(p) - _prm1);
            case global::Kerf.Geometry.SurfaceKind.Cone: // cone
            {
                double dist = _lin.DistanceTo(p);
                global::Kerf.Engine.Geometry.ElSLib.ConeParameters(_ax3, _prm1, _prm2, p, out double u, out double v);
                Point3D pp = global::Kerf.Engine.Geometry.ElSLib.ConeValue(u, v, _ax3, _prm1, _prm2);
                double distp = _lin.DistanceTo(pp);
                dist = (dist - distp) / _prm3;
                return (dist);
            }
            case global::Kerf.Geometry.SurfaceKind.Torus: // torus
            {
                TorusPoint(p, out Point3D pt, out _);
                double dist = p.DistanceTo(pt) - _prm2;
                return dist;
            }
            default:
            {
            }
            break;
        }
        return (0.0);
    }

    /// <summary>The gradient of <see cref="Distance"/> at <paramref name="p"/> (a zero vector where it is undefined).</summary>
    public readonly Vector3D Gradient(in Point3D p)
    {
        Vector3D grad = new();
        switch (_typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane: // plan
                grad = grad.WithCoord(_prm1, _prm2, _prm3);
                break;
            case global::Kerf.Geometry.SurfaceKind.Cylinder: // cylindre
            {
                Xyz PP = _lin.Location.XYZ();
                PP.Add(global::Kerf.Engine.Geometry.ElCLib.Parameter(_lin, p) * _lin.Direction.XYZ());
                grad = grad.WithXYZ(p.XYZ() - PP);
                double n = grad.Magnitude;
                if (n > 1e-14)
                {
                    grad = grad.Divided(n);
                }
                else
                {
                    grad = grad.WithCoord(0.0, 0.0, 0.0);
                }
            }
            break;
            case global::Kerf.Geometry.SurfaceKind.Sphere: // sphere
            {
                Xyz PP = p.XYZ();
                grad = grad.WithXYZ((PP - _lin.Location.XYZ()));
                double n = grad.Magnitude;
                if (n > 1e-14)
                {
                    grad = grad.Divided(n);
                }
                else
                {
                    grad = grad.WithCoord(0.0, 0.0, 0.0);
                }
            }
            break;
            case global::Kerf.Geometry.SurfaceKind.Cone: // cone
            {
                global::Kerf.Engine.Geometry.ElSLib.ConeParameters(_ax3, _prm1, _prm2, p, out double u, out double v);
                Point3D pp = global::Kerf.Engine.Geometry.ElSLib.ConeValue(u, v, _ax3, _prm1, _prm2);
                global::Kerf.Engine.Geometry.ElSLib.ConeD1(u, v, _ax3, _prm1, _prm2, out pp, out Vector3D d1u, out Vector3D d1v);
                grad = d1u.Crossed(d1v);
                if (!_ax3direc)
                {
                    grad = grad.Reversed();
                }
                if (HasMagnitudeForNormalization(grad))
                {
                    grad = grad.Normalized();
                }
                else
                {
                    grad = grad.WithCoord(0.0, 0.0, 0.0);
                }
            }
            break;
            case global::Kerf.Geometry.SurfaceKind.Torus: // torus
            {
                TorusPoint(p, out Point3D pt, out _);
                grad = grad.WithXYZ(p.XYZ() - pt.XYZ());
                double n = grad.Magnitude;
                if (n > 1e-14)
                {
                    grad = grad.Divided(n);
                }
                else
                {
                    grad = grad.WithCoord(0.0, 0.0, 0.0);
                }
            }
            break;
            default:
            {
            }
            break;
        }
        return grad;
    }

    /// <summary>
    /// <see cref="Distance"/> and <see cref="Gradient"/> together. As in OCCT, an empty quadric leaves both
    /// arguments unchanged.
    /// </summary>
    public readonly void ValAndGrad(in Point3D p, ref double Dist, ref Vector3D grad)
    {
        switch (_typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
            {
                Dist = _prm1 * p.X + _prm2 * p.Y + _prm3 * p.Z + _prm4;
                grad = grad.WithCoord(_prm1, _prm2, _prm3);
            }
            break;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            {
                Dist = _lin.DistanceTo(p) - _prm1;
                Xyz PP = _lin.Location.XYZ();
                PP.Add(global::Kerf.Engine.Geometry.ElCLib.Parameter(_lin, p) * _lin.Direction.XYZ());
                grad = grad.WithXYZ((p.XYZ() - PP));
                double n = grad.Magnitude;
                if (n > 1e-14)
                {
                    grad = grad.Divided(n);
                }
                else
                {
                    grad = grad.WithCoord(0.0, 0.0, 0.0);
                }
            }
            break;
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            {
                Dist = _lin.Location.DistanceTo(p) - _prm1;
                Xyz PP = p.XYZ();
                grad = grad.WithXYZ((PP - _lin.Location.XYZ()));
                double n = grad.Magnitude;
                if (n > 1e-14)
                {
                    grad = grad.Divided(n);
                }
                else
                {
                    grad = grad.WithCoord(0.0, 0.0, 0.0);
                }
            }
            break;
            case global::Kerf.Geometry.SurfaceKind.Cone:
            {
                double dist = _lin.DistanceTo(p);
                global::Kerf.Engine.Geometry.ElSLib.ConeParameters(_ax3, _prm1, _prm2, p, out double u, out double v);
                global::Kerf.Engine.Geometry.ElSLib.ConeD1(u, v, _ax3, _prm1, _prm2, out Point3D pp, out Vector3D d1u, out Vector3D d1v);
                double distp = _lin.DistanceTo(pp);
                dist = (dist - distp) / _prm3;
                Dist = dist;
                grad = d1u.Crossed(d1v);
                if (!_ax3direc)
                {
                    grad = grad.Reversed();
                }
                //-- lbr le 7 mars 96
                //-- If the gradient is null, we are on the axis
                //-- and in this case dist is 0,
                //-- so any value can be returned.
                if (HasMagnitudeForNormalization(grad))
                {
                    grad = grad.Normalized();
                }
                else
                {
                    grad = grad.WithCoord(0.0, 0.0, 0.0);
                }
            }
            break;
            case global::Kerf.Geometry.SurfaceKind.Torus:
            {
                TorusPoint(p, out Point3D pt, out _);
                Dist = p.DistanceTo(pt) - _prm2;
                grad = grad.WithXYZ(p.XYZ() - pt.XYZ());
                double n = grad.Magnitude;
                if (n > 1e-14)
                {
                    grad = grad.Divided(n);
                }
                else
                {
                    grad = grad.WithCoord(0.0, 0.0, 0.0);
                }
            }
            break;
            default:
            {
            }
            break;
        }
    }

    public readonly SurfaceKind TypeQuadric() => _typ;

    public readonly Plane3D Plane() => new(_ax3);

    public readonly Sphere Sphere() => new(_ax3, _prm1);

    public readonly Cylinder Cylinder() => new(_ax3, _prm1);

    public readonly Cone Cone() => new(_ax3, _prm2, _prm1);

    public readonly Torus Torus() => new(_ax3, _prm1, _prm2);

    public readonly Point3D Value(double u, double v)
    {
        switch (_typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                return global::Kerf.Engine.Geometry.ElSLib.PlaneValue(u, v, _ax3);
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                return global::Kerf.Engine.Geometry.ElSLib.CylinderValue(u, v, _ax3, _prm1);
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                return global::Kerf.Engine.Geometry.ElSLib.SphereValue(u, v, _ax3, _prm1);
            case global::Kerf.Geometry.SurfaceKind.Cone:
                return global::Kerf.Engine.Geometry.ElSLib.ConeValue(u, v, _ax3, _prm1, _prm2);
            case global::Kerf.Geometry.SurfaceKind.Torus:
                return global::Kerf.Engine.Geometry.ElSLib.TorusValue(u, v, _ax3, _prm1, _prm2);
            default:
            {
                Point3D p = new(0, 0, 0);
                return (p);
            }
        }
    }

    /// <summary>The point and first derivatives at (U, V). As in OCCT, an empty quadric leaves the arguments unchanged.</summary>
    public readonly void D1(double u, double v, ref Point3D p, ref Vector3D d1U, ref Vector3D d1V)
    {
        switch (_typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                global::Kerf.Engine.Geometry.ElSLib.PlaneD1(u, v, _ax3, out p, out d1U, out d1V);
                break;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.CylinderD1(u, v, _ax3, _prm1, out p, out d1U, out d1V);
                break;
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.SphereD1(u, v, _ax3, _prm1, out p, out d1U, out d1V);
                break;
            case global::Kerf.Geometry.SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.ConeD1(u, v, _ax3, _prm1, _prm2, out p, out d1U, out d1V);
                break;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.TorusD1(u, v, _ax3, _prm1, _prm2, out p, out d1U, out d1V);
                break;
            default:
            {
            }
            break;
        }
    }

    public readonly Vector3D DN(double u, double V, int nu, int nv)
    {
        switch (_typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                return global::Kerf.Engine.Geometry.ElSLib.PlaneDN(u, V, _ax3, nu, nv);
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                return global::Kerf.Engine.Geometry.ElSLib.CylinderDN(u, V, _ax3, _prm1, nu, nv);
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                return global::Kerf.Engine.Geometry.ElSLib.SphereDN(u, V, _ax3, _prm1, nu, nv);
            case global::Kerf.Geometry.SurfaceKind.Cone:
                return global::Kerf.Engine.Geometry.ElSLib.ConeDN(u, V, _ax3, _prm1, _prm2, nu, nv);
            case global::Kerf.Geometry.SurfaceKind.Torus:
                return global::Kerf.Engine.Geometry.ElSLib.TorusDN(u, V, _ax3, _prm1, _prm2, nu, nv);
            default:
            {
                Vector3D v = new(0, 0, 0);
                return (v);
            }
        }
    }

    /// <summary>The normal at (U, V), oriented by the direct sense of the quadric's axes.</summary>
    public readonly Vector3D Normale(double u, double V)
    {
        switch (_typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                if (_ax3direc)
                {
                    return new Vector3D(_ax3.Direction);
                }
                else
                {
                    return new Vector3D(_ax3.Direction.Reversed());
                }
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                return Normale(Value(u, V));
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                return Normale(Value(u, V));
            case global::Kerf.Geometry.SurfaceKind.Cone:
            {
                global::Kerf.Engine.Geometry.ElSLib.ConeD1(u, V, _ax3, _prm1, _prm2, out _, out Vector3D d1u, out Vector3D d1v);
                if (d1u.Magnitude < 0.0000001)
                {
                    Vector3D vn = new(0.0, 0.0, 0.0);
                    return (vn);
                }
                return (d1u.Crossed(d1v));
            }
            case global::Kerf.Geometry.SurfaceKind.Torus:
                return Normale(Value(u, V));
            default:
            {
                Vector3D v = new(0, 0, 0);
                return (v);
            }
        }
    }

    /// <summary>The parameters of the point of the quadric nearest to <paramref name="p"/> (ElSLib's projection).</summary>
    /// <remarks>As in OCCT, an empty quadric leaves <paramref name="u"/> and <paramref name="v"/> unchanged.</remarks>
    public readonly void Parameters(in Point3D p, ref double u, ref double v)
    {
        switch (_typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(_ax3, p, out u, out v);
                break;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.CylinderParameters(_ax3, _prm1, p, out u, out v);
                break;
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.SphereParameters(_ax3, _prm1, p, out u, out v);
                break;
            case global::Kerf.Geometry.SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.ConeParameters(_ax3, _prm1, _prm2, p, out u, out v);
                break;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.TorusParameters(_ax3, _prm1, _prm2, p, out u, out v);
                break;
            default:
                break;
        }
    }

    /// <summary>The normal at the point <paramref name="p"/> of the quadric.</summary>
    public readonly Vector3D Normale(in Point3D p)
    {
        switch (_typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                if (_ax3direc)
                {
                    return new Vector3D(_ax3.Direction);
                }
                else
                {
                    return new Vector3D(_ax3.Direction.Reversed());
                }
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            {
                if (_ax3direc)
                {
                    return new Vector3D(_lin.Normal(p).Direction);
                }
                else
                {
                    Direction3D D = _lin.Normal(p).Direction;
                    D = D.Reversed();
                    return new Vector3D(D);
                }
            }
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            {
                if (_ax3direc)
                {
                    Vector3D ax3P = new(_ax3.Location, p);
                    return new Vector3D(new Direction3D(ax3P));
                }
                else
                {
                    Vector3D pax3 = new(p, _ax3.Location);
                    return new Vector3D(new Direction3D(pax3));
                }
            }
            case global::Kerf.Geometry.SurfaceKind.Cone:
            {
                global::Kerf.Engine.Geometry.ElSLib.ConeParameters(_ax3, _prm1, _prm2, p, out double u, out double V);
                return Normale(u, V);
            }
            case global::Kerf.Geometry.SurfaceKind.Torus:
            {
                TorusPoint(p, out Point3D pt, out Vector3D oz);
                if (pt.SquareDistanceTo(p) < 1e-14)
                {
                    return new Vector3D(new Direction3D(oz));
                }
                Direction3D d = new(_ax3direc ? new Vector3D(pt, p) : new Vector3D(p, pt));
                return new Vector3D(d);
            }
            default:
            {
                Vector3D v = new(0, 0, 0);
                return (v);
            }
        }
    }

    /// <summary>
    /// The point of the torus's directing circle nearest to <paramref name="p"/>, as OCCT computes it in each torus
    /// branch (written there inline), and the torus's axis as a vector.
    /// </summary>
    private readonly void TorusPoint(in Point3D p, out Point3D pt, out Vector3D oz)
    {
        Point3D o, pp;
        pt = new Point3D();
        //
        o = _ax3.Location;
        oz = new Vector3D(_ax3.Direction);
        pp = p.Translated(oz.Multiplied(-(new Vector3D(o, p).Dot(new Vector3D(_ax3.Direction)))));
        //
        Direction3D doPp = (o.SquareDistanceTo(pp) < 1e-14) ? _ax3.XDirection : new Direction3D(new Vector3D(o, pp));
        pt = pt.WithXYZ(o.XYZ() + doPp.XYZ() * _prm1);
    }

    private static bool HasMagnitudeForNormalization(in Vector3D vector)
    {
        double resolution = Constants.Resolution();
        return vector.SquareMagnitude > resolution * resolution;
    }

    public readonly bool Equals(SurfaceQuadric other)
        => _ax3.Equals(other._ax3) && _lin.Equals(other._lin) && _typ == other._typ && _prm1.Equals(other._prm1)
           && _prm2.Equals(other._prm2) && _prm3.Equals(other._prm3) && _prm4.Equals(other._prm4)
           && _ax3direc == other._ax3direc;

    public override readonly bool Equals(object? obj) => obj is SurfaceQuadric other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_ax3, _lin, _typ, _prm1, _prm2, _prm3, _prm4, _ax3direc);

    public static bool operator ==(in SurfaceQuadric left, in SurfaceQuadric right) => left.Equals(right);

    public static bool operator !=(in SurfaceQuadric left, in SurfaceQuadric right) => !left.Equals(right);

    public override readonly string ToString() => $"{_typ} {Constants.Format("IntSurf_Quadric", _prm1, _prm2, _prm3, _prm4)} {_ax3}";
}
