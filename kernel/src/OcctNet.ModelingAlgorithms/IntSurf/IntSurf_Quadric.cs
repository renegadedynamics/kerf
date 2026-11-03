// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Quadric.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Quadric.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Quadric.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// An elementary surface (plane, cylinder, sphere, cone or torus) with its implicit "distance" function, its
/// gradient and its parametric evaluation, as the intersection algorithms use it.
/// </summary>
/// <remarks>
/// A value type (OCCT copies it). <c>new IntSurf_Quadric()</c> is OCCT's empty quadric (type
/// GeomAbs_OtherSurface); <c>default</c> is all zeros (type GeomAbs_Plane with a zero equation). The <c>SetValue</c>
/// methods are internal mutators; the constructors are their twins.
/// </remarks>
public partial struct IntSurf_Quadric : IEquatable<IntSurf_Quadric>
{
    private CoordinateSystem ax3;
    private Line3D lin;
    private SurfaceKind typ;
    private double prm1;
    private double prm2;
    private double prm3;
    private double prm4;
    private bool ax3direc;

    public IntSurf_Quadric()
    {
        ax3 = new CoordinateSystem();
        lin = new Line3D();
        typ = OtherSurface;
        prm1 = 0.0;
        prm2 = 0.0;
        prm3 = 0.0;
        prm4 = 0.0;
        ax3direc = false;
    }

    public IntSurf_Quadric(in Plane3D P)
    {
        ax3 = P.Position();
        lin = new Line3D();
        typ = global::Kerf.Geometry.SurfaceKind.Plane;
        ax3direc = ax3.Direct();
        P.Coefficients(out prm1, out prm2, out prm3, out prm4);
    }

    public IntSurf_Quadric(in Cylinder C)
    {
        ax3 = C.Position();
        lin = new Line3D(ax3.Axis());
        typ = global::Kerf.Geometry.SurfaceKind.Cylinder;
        prm2 = prm3 = prm4 = 0.0;
        ax3direc = ax3.Direct();
        prm1 = C.Radius();
    }

    public IntSurf_Quadric(in Sphere S)
    {
        ax3 = S.Position();
        lin = new Line3D(ax3.Axis());
        typ = global::Kerf.Geometry.SurfaceKind.Sphere;
        prm2 = prm3 = prm4 = 0.0;
        ax3direc = ax3.Direct();
        prm1 = S.Radius();
    }

    public IntSurf_Quadric(in Cone C)
    {
        ax3 = C.Position();
        lin = new Line3D();
        typ = global::Kerf.Geometry.SurfaceKind.Cone;
        ax3direc = ax3.Direct();
        lin.SetPosition(ax3.Axis());
        prm1 = C.RefRadius();
        prm2 = C.SemiAngle();
        prm3 = Math.Cos(prm2);
        prm4 = 0.0;
    }

    public IntSurf_Quadric(in Torus T)
    {
        ax3 = T.Position();
        lin = new Line3D();
        typ = global::Kerf.Geometry.SurfaceKind.Torus;
        ax3direc = ax3.Direct();
        lin.SetPosition(ax3.Axis());
        prm1 = T.MajorRadius();
        prm2 = T.MinorRadius();
        prm3 = 0.0;
        prm4 = 0.0;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValue(in Plane3D P)
    {
        typ = global::Kerf.Geometry.SurfaceKind.Plane;
        ax3 = P.Position();
        ax3direc = ax3.Direct();
        P.Coefficients(out prm1, out prm2, out prm3, out prm4);
    }

    internal void SetValue(in Cylinder C)
    {
        typ = global::Kerf.Geometry.SurfaceKind.Cylinder;
        ax3 = C.Position();
        ax3direc = ax3.Direct();
        lin.SetPosition(ax3.Axis());
        prm1 = C.Radius();
        prm2 = prm3 = prm4 = 0.0;
    }

    internal void SetValue(in Sphere S)
    {
        typ = global::Kerf.Geometry.SurfaceKind.Sphere;
        ax3 = S.Position();
        ax3direc = ax3.Direct();
        lin.SetPosition(ax3.Axis());
        prm1 = S.Radius();
        prm2 = prm3 = prm4 = 0.0;
    }

    internal void SetValue(in Cone C)
    {
        typ = global::Kerf.Geometry.SurfaceKind.Cone;
        ax3 = C.Position();
        ax3direc = ax3.Direct();
        lin.SetPosition(ax3.Axis());
        prm1 = C.RefRadius();
        prm2 = C.SemiAngle();
        prm3 = Math.Cos(prm2);
        prm4 = 0.0;
    }

    internal void SetValue(in Torus T)
    {
        typ = global::Kerf.Geometry.SurfaceKind.Torus;
        ax3 = T.Position();
        ax3direc = ax3.Direct();
        lin.SetPosition(ax3.Axis());
        prm1 = T.MajorRadius();
        prm2 = T.MinorRadius();
        prm3 = 0.0;
        prm4 = 0.0;
    }

    // Queries

    /// <summary>
    /// The signed "distance" of <paramref name="P"/> to the quadric (the plane equation for a plane), zero on the
    /// surface.
    /// </summary>
    public readonly double Distance(in Point3D P)
    {
        switch (typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane: // plan
                return prm1 * P.X() + prm2 * P.Y() + prm3 * P.Z() + prm4;
            case global::Kerf.Geometry.SurfaceKind.Cylinder: // cylindre
                return (lin.Distance(P) - prm1);
            case global::Kerf.Geometry.SurfaceKind.Sphere: // sphere
                return (lin.Location().Distance(P) - prm1);
            case global::Kerf.Geometry.SurfaceKind.Cone: // cone
            {
                double dist = lin.Distance(P);
                global::Kerf.Engine.Geometry.ElSLib.ConeParameters(ax3, prm1, prm2, P, out double U, out double V);
                Point3D Pp = global::Kerf.Engine.Geometry.ElSLib.ConeValue(U, V, ax3, prm1, prm2);
                double distp = lin.Distance(Pp);
                dist = (dist - distp) / prm3;
                return (dist);
            }
            case global::Kerf.Geometry.SurfaceKind.Torus: // torus
            {
                TorusPoint(P, out Point3D PT, out _);
                double dist = P.Distance(PT) - prm2;
                return dist;
            }
            default:
            {
            }
            break;
        }
        return (0.0);
    }

    /// <summary>The gradient of <see cref="Distance"/> at <paramref name="P"/> (a zero vector where it is undefined).</summary>
    public readonly Vector3D Gradient(in Point3D P)
    {
        Vector3D grad = new();
        switch (typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane: // plan
                grad.SetCoord(prm1, prm2, prm3);
                break;
            case global::Kerf.Geometry.SurfaceKind.Cylinder: // cylindre
            {
                Xyz PP = lin.Location().XYZ();
                PP.Add(global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, P) * lin.Direction().XYZ());
                grad.SetXYZ(P.XYZ() - PP);
                double N = grad.Magnitude();
                if (N > 1e-14)
                {
                    grad.Divide(N);
                }
                else
                {
                    grad.SetCoord(0.0, 0.0, 0.0);
                }
            }
            break;
            case global::Kerf.Geometry.SurfaceKind.Sphere: // sphere
            {
                Xyz PP = P.XYZ();
                grad.SetXYZ((PP - lin.Location().XYZ()));
                double N = grad.Magnitude();
                if (N > 1e-14)
                {
                    grad.Divide(N);
                }
                else
                {
                    grad.SetCoord(0.0, 0.0, 0.0);
                }
            }
            break;
            case global::Kerf.Geometry.SurfaceKind.Cone: // cone
            {
                global::Kerf.Engine.Geometry.ElSLib.ConeParameters(ax3, prm1, prm2, P, out double U, out double V);
                Point3D Pp = global::Kerf.Engine.Geometry.ElSLib.ConeValue(U, V, ax3, prm1, prm2);
                global::Kerf.Engine.Geometry.ElSLib.ConeD1(U, V, ax3, prm1, prm2, out Pp, out Vector3D D1u, out Vector3D D1v);
                grad = D1u.Crossed(D1v);
                if (!ax3direc)
                {
                    grad.Reverse();
                }
                if (hasMagnitudeForNormalization(grad))
                {
                    grad.Normalize();
                }
                else
                {
                    grad.SetCoord(0.0, 0.0, 0.0);
                }
            }
            break;
            case global::Kerf.Geometry.SurfaceKind.Torus: // torus
            {
                TorusPoint(P, out Point3D PT, out _);
                grad.SetXYZ(P.XYZ() - PT.XYZ());
                double N = grad.Magnitude();
                if (N > 1e-14)
                {
                    grad.Divide(N);
                }
                else
                {
                    grad.SetCoord(0.0, 0.0, 0.0);
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
    public readonly void ValAndGrad(in Point3D P, ref double Dist, ref Vector3D Grad)
    {
        switch (typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
            {
                Dist = prm1 * P.X() + prm2 * P.Y() + prm3 * P.Z() + prm4;
                Grad.SetCoord(prm1, prm2, prm3);
            }
            break;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            {
                Dist = lin.Distance(P) - prm1;
                Xyz PP = lin.Location().XYZ();
                PP.Add(global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, P) * lin.Direction().XYZ());
                Grad.SetXYZ((P.XYZ() - PP));
                double N = Grad.Magnitude();
                if (N > 1e-14)
                {
                    Grad.Divide(N);
                }
                else
                {
                    Grad.SetCoord(0.0, 0.0, 0.0);
                }
            }
            break;
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            {
                Dist = lin.Location().Distance(P) - prm1;
                Xyz PP = P.XYZ();
                Grad.SetXYZ((PP - lin.Location().XYZ()));
                double N = Grad.Magnitude();
                if (N > 1e-14)
                {
                    Grad.Divide(N);
                }
                else
                {
                    Grad.SetCoord(0.0, 0.0, 0.0);
                }
            }
            break;
            case global::Kerf.Geometry.SurfaceKind.Cone:
            {
                double dist = lin.Distance(P);
                global::Kerf.Engine.Geometry.ElSLib.ConeParameters(ax3, prm1, prm2, P, out double U, out double V);
                global::Kerf.Engine.Geometry.ElSLib.ConeD1(U, V, ax3, prm1, prm2, out Point3D Pp, out Vector3D D1u, out Vector3D D1v);
                double distp = lin.Distance(Pp);
                dist = (dist - distp) / prm3;
                Dist = dist;
                Grad = D1u.Crossed(D1v);
                if (!ax3direc)
                {
                    Grad.Reverse();
                }
                //-- lbr le 7 mars 96
                //-- If the gradient is null, we are on the axis
                //-- and in this case dist is 0,
                //-- so any value can be returned.
                if (hasMagnitudeForNormalization(Grad))
                {
                    Grad.Normalize();
                }
                else
                {
                    Grad.SetCoord(0.0, 0.0, 0.0);
                }
            }
            break;
            case global::Kerf.Geometry.SurfaceKind.Torus:
            {
                TorusPoint(P, out Point3D PT, out _);
                Dist = P.Distance(PT) - prm2;
                Grad.SetXYZ(P.XYZ() - PT.XYZ());
                double N = Grad.Magnitude();
                if (N > 1e-14)
                {
                    Grad.Divide(N);
                }
                else
                {
                    Grad.SetCoord(0.0, 0.0, 0.0);
                }
            }
            break;
            default:
            {
            }
            break;
        }
    }

    public readonly SurfaceKind TypeQuadric() => typ;

    public readonly Plane3D Plane() => new(ax3);

    public readonly Sphere Sphere() => new(ax3, prm1);

    public readonly Cylinder Cylinder() => new(ax3, prm1);

    public readonly Cone Cone() => new(ax3, prm2, prm1);

    public readonly Torus Torus() => new(ax3, prm1, prm2);

    public readonly Point3D Value(double U, double V)
    {
        switch (typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                return global::Kerf.Engine.Geometry.ElSLib.PlaneValue(U, V, ax3);
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                return global::Kerf.Engine.Geometry.ElSLib.CylinderValue(U, V, ax3, prm1);
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                return global::Kerf.Engine.Geometry.ElSLib.SphereValue(U, V, ax3, prm1);
            case global::Kerf.Geometry.SurfaceKind.Cone:
                return global::Kerf.Engine.Geometry.ElSLib.ConeValue(U, V, ax3, prm1, prm2);
            case global::Kerf.Geometry.SurfaceKind.Torus:
                return global::Kerf.Engine.Geometry.ElSLib.TorusValue(U, V, ax3, prm1, prm2);
            default:
            {
                Point3D p = new(0, 0, 0);
                return (p);
            }
        }
    }

    /// <summary>The point and first derivatives at (U, V). As in OCCT, an empty quadric leaves the arguments unchanged.</summary>
    public readonly void D1(double U, double V, ref Point3D P, ref Vector3D D1U, ref Vector3D D1V)
    {
        switch (typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                global::Kerf.Engine.Geometry.ElSLib.PlaneD1(U, V, ax3, out P, out D1U, out D1V);
                break;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.CylinderD1(U, V, ax3, prm1, out P, out D1U, out D1V);
                break;
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.SphereD1(U, V, ax3, prm1, out P, out D1U, out D1V);
                break;
            case global::Kerf.Geometry.SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.ConeD1(U, V, ax3, prm1, prm2, out P, out D1U, out D1V);
                break;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.TorusD1(U, V, ax3, prm1, prm2, out P, out D1U, out D1V);
                break;
            default:
            {
            }
            break;
        }
    }

    public readonly Vector3D DN(double U, double V, int Nu, int Nv)
    {
        switch (typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                return global::Kerf.Engine.Geometry.ElSLib.PlaneDN(U, V, ax3, Nu, Nv);
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                return global::Kerf.Engine.Geometry.ElSLib.CylinderDN(U, V, ax3, prm1, Nu, Nv);
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                return global::Kerf.Engine.Geometry.ElSLib.SphereDN(U, V, ax3, prm1, Nu, Nv);
            case global::Kerf.Geometry.SurfaceKind.Cone:
                return global::Kerf.Engine.Geometry.ElSLib.ConeDN(U, V, ax3, prm1, prm2, Nu, Nv);
            case global::Kerf.Geometry.SurfaceKind.Torus:
                return global::Kerf.Engine.Geometry.ElSLib.TorusDN(U, V, ax3, prm1, prm2, Nu, Nv);
            default:
            {
                Vector3D v = new(0, 0, 0);
                return (v);
            }
        }
    }

    /// <summary>The normal at (U, V), oriented by the direct sense of the quadric's axes.</summary>
    public readonly Vector3D Normale(double U, double V)
    {
        switch (typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                if (ax3direc)
                {
                    return new Vector3D(ax3.Direction());
                }
                else
                {
                    return new Vector3D(ax3.Direction().Reversed());
                }
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                return Normale(Value(U, V));
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                return Normale(Value(U, V));
            case global::Kerf.Geometry.SurfaceKind.Cone:
            {
                global::Kerf.Engine.Geometry.ElSLib.ConeD1(U, V, ax3, prm1, prm2, out _, out Vector3D D1u, out Vector3D D1v);
                if (D1u.Magnitude() < 0.0000001)
                {
                    Vector3D Vn = new(0.0, 0.0, 0.0);
                    return (Vn);
                }
                return (D1u.Crossed(D1v));
            }
            case global::Kerf.Geometry.SurfaceKind.Torus:
                return Normale(Value(U, V));
            default:
            {
                Vector3D v = new(0, 0, 0);
                return (v);
            }
        }
    }

    /// <summary>The parameters of the point of the quadric nearest to <paramref name="P"/> (ElSLib's projection).</summary>
    /// <remarks>As in OCCT, an empty quadric leaves <paramref name="U"/> and <paramref name="V"/> unchanged.</remarks>
    public readonly void Parameters(in Point3D P, ref double U, ref double V)
    {
        switch (typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(ax3, P, out U, out V);
                break;
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.CylinderParameters(ax3, prm1, P, out U, out V);
                break;
            case global::Kerf.Geometry.SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.SphereParameters(ax3, prm1, P, out U, out V);
                break;
            case global::Kerf.Geometry.SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.ConeParameters(ax3, prm1, prm2, P, out U, out V);
                break;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.TorusParameters(ax3, prm1, prm2, P, out U, out V);
                break;
            default:
                break;
        }
    }

    /// <summary>The normal at the point <paramref name="P"/> of the quadric.</summary>
    public readonly Vector3D Normale(in Point3D P)
    {
        switch (typ)
        {
            case global::Kerf.Geometry.SurfaceKind.Plane:
                if (ax3direc)
                {
                    return new Vector3D(ax3.Direction());
                }
                else
                {
                    return new Vector3D(ax3.Direction().Reversed());
                }
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            {
                if (ax3direc)
                {
                    return new Vector3D(lin.Normal(P).Direction());
                }
                else
                {
                    Direction3D D = lin.Normal(P).Direction();
                    D.Reverse();
                    return new Vector3D(D);
                }
            }
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            {
                if (ax3direc)
                {
                    Vector3D ax3P = new(ax3.Location(), P);
                    return new Vector3D(new Direction3D(ax3P));
                }
                else
                {
                    Vector3D Pax3 = new(P, ax3.Location());
                    return new Vector3D(new Direction3D(Pax3));
                }
            }
            case global::Kerf.Geometry.SurfaceKind.Cone:
            {
                global::Kerf.Engine.Geometry.ElSLib.ConeParameters(ax3, prm1, prm2, P, out double U, out double V);
                return Normale(U, V);
            }
            case global::Kerf.Geometry.SurfaceKind.Torus:
            {
                TorusPoint(P, out Point3D PT, out Vector3D OZ);
                if (PT.SquareDistance(P) < 1e-14)
                {
                    return new Vector3D(new Direction3D(OZ));
                }
                Direction3D aD = new(ax3direc ? new Vector3D(PT, P) : new Vector3D(P, PT));
                return new Vector3D(aD);
            }
            default:
            {
                Vector3D v = new(0, 0, 0);
                return (v);
            }
        }
    }

    /// <summary>
    /// The point of the torus's directing circle nearest to <paramref name="P"/>, as OCCT computes it in each torus
    /// branch (written there inline), and the torus's axis as a vector.
    /// </summary>
    private readonly void TorusPoint(in Point3D P, out Point3D PT, out Vector3D OZ)
    {
        Point3D O, Pp;
        PT = new Point3D();
        //
        O = ax3.Location();
        OZ = new Vector3D(ax3.Direction());
        Pp = P.Translated(OZ.Multiplied(-(new Vector3D(O, P).Dot(new Vector3D(ax3.Direction())))));
        //
        Direction3D DOPp = (O.SquareDistance(Pp) < 1e-14) ? ax3.XDirection() : new Direction3D(new Vector3D(O, Pp));
        PT.SetXYZ(O.XYZ() + DOPp.XYZ() * prm1);
    }

    private static bool hasMagnitudeForNormalization(in Vector3D theVector)
    {
        double aResolution = Constants.Resolution();
        return theVector.SquareMagnitude() > aResolution * aResolution;
    }

    public readonly bool Equals(IntSurf_Quadric other)
        => ax3.Equals(other.ax3) && lin.Equals(other.lin) && typ == other.typ && prm1.Equals(other.prm1)
           && prm2.Equals(other.prm2) && prm3.Equals(other.prm3) && prm4.Equals(other.prm4)
           && ax3direc == other.ax3direc;

    public override readonly bool Equals(object? obj) => obj is IntSurf_Quadric other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(ax3, lin, typ, prm1, prm2, prm3, prm4, ax3direc);

    public static bool operator ==(in IntSurf_Quadric theLeft, in IntSurf_Quadric theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntSurf_Quadric theLeft, in IntSurf_Quadric theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"{typ} {Constants.Format("IntSurf_Quadric", prm1, prm2, prm3, prm4)} {ax3}";
}
