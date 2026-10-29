// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Quadric.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Quadric.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Quadric.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;

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
    private gp_Ax3 ax3;
    private gp_Lin lin;
    private GeomAbs_SurfaceType typ;
    private double prm1;
    private double prm2;
    private double prm3;
    private double prm4;
    private bool ax3direc;

    public IntSurf_Quadric()
    {
        ax3 = new gp_Ax3();
        lin = new gp_Lin();
        typ = GeomAbs_OtherSurface;
        prm1 = 0.0;
        prm2 = 0.0;
        prm3 = 0.0;
        prm4 = 0.0;
        ax3direc = false;
    }

    public IntSurf_Quadric(in gp_Pln P)
    {
        ax3 = P.Position();
        lin = new gp_Lin();
        typ = GeomAbs_Plane;
        ax3direc = ax3.Direct();
        P.Coefficients(out prm1, out prm2, out prm3, out prm4);
    }

    public IntSurf_Quadric(in gp_Cylinder C)
    {
        ax3 = C.Position();
        lin = new gp_Lin(ax3.Axis());
        typ = GeomAbs_Cylinder;
        prm2 = prm3 = prm4 = 0.0;
        ax3direc = ax3.Direct();
        prm1 = C.Radius();
    }

    public IntSurf_Quadric(in gp_Sphere S)
    {
        ax3 = S.Position();
        lin = new gp_Lin(ax3.Axis());
        typ = GeomAbs_Sphere;
        prm2 = prm3 = prm4 = 0.0;
        ax3direc = ax3.Direct();
        prm1 = S.Radius();
    }

    public IntSurf_Quadric(in gp_Cone C)
    {
        ax3 = C.Position();
        lin = new gp_Lin();
        typ = GeomAbs_Cone;
        ax3direc = ax3.Direct();
        lin.SetPosition(ax3.Axis());
        prm1 = C.RefRadius();
        prm2 = C.SemiAngle();
        prm3 = Math.Cos(prm2);
        prm4 = 0.0;
    }

    public IntSurf_Quadric(in gp_Torus T)
    {
        ax3 = T.Position();
        lin = new gp_Lin();
        typ = GeomAbs_Torus;
        ax3direc = ax3.Direct();
        lin.SetPosition(ax3.Axis());
        prm1 = T.MajorRadius();
        prm2 = T.MinorRadius();
        prm3 = 0.0;
        prm4 = 0.0;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValue(in gp_Pln P)
    {
        typ = GeomAbs_Plane;
        ax3 = P.Position();
        ax3direc = ax3.Direct();
        P.Coefficients(out prm1, out prm2, out prm3, out prm4);
    }

    internal void SetValue(in gp_Cylinder C)
    {
        typ = GeomAbs_Cylinder;
        ax3 = C.Position();
        ax3direc = ax3.Direct();
        lin.SetPosition(ax3.Axis());
        prm1 = C.Radius();
        prm2 = prm3 = prm4 = 0.0;
    }

    internal void SetValue(in gp_Sphere S)
    {
        typ = GeomAbs_Sphere;
        ax3 = S.Position();
        ax3direc = ax3.Direct();
        lin.SetPosition(ax3.Axis());
        prm1 = S.Radius();
        prm2 = prm3 = prm4 = 0.0;
    }

    internal void SetValue(in gp_Cone C)
    {
        typ = GeomAbs_Cone;
        ax3 = C.Position();
        ax3direc = ax3.Direct();
        lin.SetPosition(ax3.Axis());
        prm1 = C.RefRadius();
        prm2 = C.SemiAngle();
        prm3 = Math.Cos(prm2);
        prm4 = 0.0;
    }

    internal void SetValue(in gp_Torus T)
    {
        typ = GeomAbs_Torus;
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
    public readonly double Distance(in gp_Pnt P)
    {
        switch (typ)
        {
            case GeomAbs_Plane: // plan
                return prm1 * P.X() + prm2 * P.Y() + prm3 * P.Z() + prm4;
            case GeomAbs_Cylinder: // cylindre
                return (lin.Distance(P) - prm1);
            case GeomAbs_Sphere: // sphere
                return (lin.Location().Distance(P) - prm1);
            case GeomAbs_Cone: // cone
            {
                double dist = lin.Distance(P);
                ElSLib.ConeParameters(ax3, prm1, prm2, P, out double U, out double V);
                gp_Pnt Pp = ElSLib.ConeValue(U, V, ax3, prm1, prm2);
                double distp = lin.Distance(Pp);
                dist = (dist - distp) / prm3;
                return (dist);
            }
            case GeomAbs_Torus: // torus
            {
                TorusPoint(P, out gp_Pnt PT, out _);
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
    public readonly gp_Vec Gradient(in gp_Pnt P)
    {
        gp_Vec grad = new();
        switch (typ)
        {
            case GeomAbs_Plane: // plan
                grad.SetCoord(prm1, prm2, prm3);
                break;
            case GeomAbs_Cylinder: // cylindre
            {
                gp_XYZ PP = lin.Location().XYZ();
                PP.Add(ElCLib.Parameter(lin, P) * lin.Direction().XYZ());
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
            case GeomAbs_Sphere: // sphere
            {
                gp_XYZ PP = P.XYZ();
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
            case GeomAbs_Cone: // cone
            {
                ElSLib.ConeParameters(ax3, prm1, prm2, P, out double U, out double V);
                gp_Pnt Pp = ElSLib.ConeValue(U, V, ax3, prm1, prm2);
                ElSLib.ConeD1(U, V, ax3, prm1, prm2, out Pp, out gp_Vec D1u, out gp_Vec D1v);
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
            case GeomAbs_Torus: // torus
            {
                TorusPoint(P, out gp_Pnt PT, out _);
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
    public readonly void ValAndGrad(in gp_Pnt P, ref double Dist, ref gp_Vec Grad)
    {
        switch (typ)
        {
            case GeomAbs_Plane:
            {
                Dist = prm1 * P.X() + prm2 * P.Y() + prm3 * P.Z() + prm4;
                Grad.SetCoord(prm1, prm2, prm3);
            }
            break;
            case GeomAbs_Cylinder:
            {
                Dist = lin.Distance(P) - prm1;
                gp_XYZ PP = lin.Location().XYZ();
                PP.Add(ElCLib.Parameter(lin, P) * lin.Direction().XYZ());
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
            case GeomAbs_Sphere:
            {
                Dist = lin.Location().Distance(P) - prm1;
                gp_XYZ PP = P.XYZ();
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
            case GeomAbs_Cone:
            {
                double dist = lin.Distance(P);
                ElSLib.ConeParameters(ax3, prm1, prm2, P, out double U, out double V);
                ElSLib.ConeD1(U, V, ax3, prm1, prm2, out gp_Pnt Pp, out gp_Vec D1u, out gp_Vec D1v);
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
            case GeomAbs_Torus:
            {
                TorusPoint(P, out gp_Pnt PT, out _);
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

    public readonly GeomAbs_SurfaceType TypeQuadric() => typ;

    public readonly gp_Pln Plane() => new(ax3);

    public readonly gp_Sphere Sphere() => new(ax3, prm1);

    public readonly gp_Cylinder Cylinder() => new(ax3, prm1);

    public readonly gp_Cone Cone() => new(ax3, prm2, prm1);

    public readonly gp_Torus Torus() => new(ax3, prm1, prm2);

    public readonly gp_Pnt Value(double U, double V)
    {
        switch (typ)
        {
            case GeomAbs_Plane:
                return ElSLib.PlaneValue(U, V, ax3);
            case GeomAbs_Cylinder:
                return ElSLib.CylinderValue(U, V, ax3, prm1);
            case GeomAbs_Sphere:
                return ElSLib.SphereValue(U, V, ax3, prm1);
            case GeomAbs_Cone:
                return ElSLib.ConeValue(U, V, ax3, prm1, prm2);
            case GeomAbs_Torus:
                return ElSLib.TorusValue(U, V, ax3, prm1, prm2);
            default:
            {
                gp_Pnt p = new(0, 0, 0);
                return (p);
            }
        }
    }

    /// <summary>The point and first derivatives at (U, V). As in OCCT, an empty quadric leaves the arguments unchanged.</summary>
    public readonly void D1(double U, double V, ref gp_Pnt P, ref gp_Vec D1U, ref gp_Vec D1V)
    {
        switch (typ)
        {
            case GeomAbs_Plane:
                ElSLib.PlaneD1(U, V, ax3, out P, out D1U, out D1V);
                break;
            case GeomAbs_Cylinder:
                ElSLib.CylinderD1(U, V, ax3, prm1, out P, out D1U, out D1V);
                break;
            case GeomAbs_Sphere:
                ElSLib.SphereD1(U, V, ax3, prm1, out P, out D1U, out D1V);
                break;
            case GeomAbs_Cone:
                ElSLib.ConeD1(U, V, ax3, prm1, prm2, out P, out D1U, out D1V);
                break;
            case GeomAbs_Torus:
                ElSLib.TorusD1(U, V, ax3, prm1, prm2, out P, out D1U, out D1V);
                break;
            default:
            {
            }
            break;
        }
    }

    public readonly gp_Vec DN(double U, double V, int Nu, int Nv)
    {
        switch (typ)
        {
            case GeomAbs_Plane:
                return ElSLib.PlaneDN(U, V, ax3, Nu, Nv);
            case GeomAbs_Cylinder:
                return ElSLib.CylinderDN(U, V, ax3, prm1, Nu, Nv);
            case GeomAbs_Sphere:
                return ElSLib.SphereDN(U, V, ax3, prm1, Nu, Nv);
            case GeomAbs_Cone:
                return ElSLib.ConeDN(U, V, ax3, prm1, prm2, Nu, Nv);
            case GeomAbs_Torus:
                return ElSLib.TorusDN(U, V, ax3, prm1, prm2, Nu, Nv);
            default:
            {
                gp_Vec v = new(0, 0, 0);
                return (v);
            }
        }
    }

    /// <summary>The normal at (U, V), oriented by the direct sense of the quadric's axes.</summary>
    public readonly gp_Vec Normale(double U, double V)
    {
        switch (typ)
        {
            case GeomAbs_Plane:
                if (ax3direc)
                {
                    return new gp_Vec(ax3.Direction());
                }
                else
                {
                    return new gp_Vec(ax3.Direction().Reversed());
                }
            case GeomAbs_Cylinder:
                return Normale(Value(U, V));
            case GeomAbs_Sphere:
                return Normale(Value(U, V));
            case GeomAbs_Cone:
            {
                ElSLib.ConeD1(U, V, ax3, prm1, prm2, out _, out gp_Vec D1u, out gp_Vec D1v);
                if (D1u.Magnitude() < 0.0000001)
                {
                    gp_Vec Vn = new(0.0, 0.0, 0.0);
                    return (Vn);
                }
                return (D1u.Crossed(D1v));
            }
            case GeomAbs_Torus:
                return Normale(Value(U, V));
            default:
            {
                gp_Vec v = new(0, 0, 0);
                return (v);
            }
        }
    }

    /// <summary>The parameters of the point of the quadric nearest to <paramref name="P"/> (ElSLib's projection).</summary>
    /// <remarks>As in OCCT, an empty quadric leaves <paramref name="U"/> and <paramref name="V"/> unchanged.</remarks>
    public readonly void Parameters(in gp_Pnt P, ref double U, ref double V)
    {
        switch (typ)
        {
            case GeomAbs_Plane:
                ElSLib.PlaneParameters(ax3, P, out U, out V);
                break;
            case GeomAbs_Cylinder:
                ElSLib.CylinderParameters(ax3, prm1, P, out U, out V);
                break;
            case GeomAbs_Sphere:
                ElSLib.SphereParameters(ax3, prm1, P, out U, out V);
                break;
            case GeomAbs_Cone:
                ElSLib.ConeParameters(ax3, prm1, prm2, P, out U, out V);
                break;
            case GeomAbs_Torus:
                ElSLib.TorusParameters(ax3, prm1, prm2, P, out U, out V);
                break;
            default:
                break;
        }
    }

    /// <summary>The normal at the point <paramref name="P"/> of the quadric.</summary>
    public readonly gp_Vec Normale(in gp_Pnt P)
    {
        switch (typ)
        {
            case GeomAbs_Plane:
                if (ax3direc)
                {
                    return new gp_Vec(ax3.Direction());
                }
                else
                {
                    return new gp_Vec(ax3.Direction().Reversed());
                }
            case GeomAbs_Cylinder:
            {
                if (ax3direc)
                {
                    return new gp_Vec(lin.Normal(P).Direction());
                }
                else
                {
                    gp_Dir D = lin.Normal(P).Direction();
                    D.Reverse();
                    return new gp_Vec(D);
                }
            }
            case GeomAbs_Sphere:
            {
                if (ax3direc)
                {
                    gp_Vec ax3P = new(ax3.Location(), P);
                    return new gp_Vec(new gp_Dir(ax3P));
                }
                else
                {
                    gp_Vec Pax3 = new(P, ax3.Location());
                    return new gp_Vec(new gp_Dir(Pax3));
                }
            }
            case GeomAbs_Cone:
            {
                ElSLib.ConeParameters(ax3, prm1, prm2, P, out double U, out double V);
                return Normale(U, V);
            }
            case GeomAbs_Torus:
            {
                TorusPoint(P, out gp_Pnt PT, out gp_Vec OZ);
                if (PT.SquareDistance(P) < 1e-14)
                {
                    return new gp_Vec(new gp_Dir(OZ));
                }
                gp_Dir aD = new(ax3direc ? new gp_Vec(PT, P) : new gp_Vec(P, PT));
                return new gp_Vec(aD);
            }
            default:
            {
                gp_Vec v = new(0, 0, 0);
                return (v);
            }
        }
    }

    /// <summary>
    /// The point of the torus's directing circle nearest to <paramref name="P"/>, as OCCT computes it in each torus
    /// branch (written there inline), and the torus's axis as a vector.
    /// </summary>
    private readonly void TorusPoint(in gp_Pnt P, out gp_Pnt PT, out gp_Vec OZ)
    {
        gp_Pnt O, Pp;
        PT = new gp_Pnt();
        //
        O = ax3.Location();
        OZ = new gp_Vec(ax3.Direction());
        Pp = P.Translated(OZ.Multiplied(-(new gp_Vec(O, P).Dot(new gp_Vec(ax3.Direction())))));
        //
        gp_Dir DOPp = (O.SquareDistance(Pp) < 1e-14) ? ax3.XDirection() : new gp_Dir(new gp_Vec(O, Pp));
        PT.SetXYZ(O.XYZ() + DOPp.XYZ() * prm1);
    }

    private static bool hasMagnitudeForNormalization(in gp_Vec theVector)
    {
        double aResolution = gp.Resolution();
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

    public override readonly string ToString() => $"{typ} {gp.Format("IntSurf_Quadric", prm1, prm2, prm3, prm4)} {ax3}";
}
