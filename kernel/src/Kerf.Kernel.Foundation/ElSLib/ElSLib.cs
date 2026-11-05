// Port of: src/FoundationClasses/TKMath/ElSLib/ElSLib.hxx
// Port of: src/FoundationClasses/TKMath/ElSLib/ElSLib.lxx
// Port of: src/FoundationClasses/TKMath/ElSLib/ElSLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry;

/// <summary>
/// Elementary Surfaces Library: points and derivatives of planes, cones, cylinders, spheres and tori at given
/// parameters, the parameters of a point on such a surface, and their isoparametric curves. Each function exists
/// for a gp surface and for the surface given by its position and dimensions.
/// </summary>
/// <remarks>
/// Parametrisations, in the local coordinate system of the surface:
/// plane <c>P(U, V) = O + U * XDir + V * YDir</c>;
/// cylinder <c>P(U, V) = O + V * ZDir + Radius * (cos(U) * XDir + sin(U) * YDir)</c>;
/// cone <c>P(U, V) = O + V * cos(SAngle) * ZDir + (Radius + V * sin(SAngle)) * (cos(U) * XDir + sin(U) * YDir)</c>;
/// sphere <c>P(U, V) = O + Radius * cos(V) * (cos(U) * XDir + sin(U) * YDir) + Radius * sin(V) * ZDir</c>;
/// torus <c>P(U, V) = O + (MajorRadius + MinorRadius * cos(V)) * (cos(U) * XDir + sin(U) * YDir) + MinorRadius * sin(V) * ZDir</c>.
/// </remarks>
internal static class ElSLib
{
    // ElSLib.lxx: the gp surface overloads

    /// <summary>The point of parameters (U, V) on an elementary surface.</summary>
    public static Point3D Value(double u, double v, in Plane3D pl) => PlaneValue(u, v, pl.Position);

    public static Point3D Value(double u, double v, in Cone3D c)
        => ConeValue(u, v, c.Position, c.RefRadius, c.SemiAngle);

    public static Point3D Value(double u, double v, in Cylinder3D c) => CylinderValue(u, v, c.Position, c.Radius);

    public static Point3D Value(double u, double v, in Sphere3D s) => SphereValue(u, v, s.Position, s.Radius);

    public static Point3D Value(double u, double v, in Torus3D t)
        => TorusValue(u, v, t.Position, t.MajorRadius, t.MinorRadius);

    /// <summary>The derivative of order Nu in U and Nv in V at the point of parameters (U, V).</summary>
    public static Vector3D DN(double u, double v, in Plane3D pl, int nu, int nv) => PlaneDN(u, v, pl.Position, nu, nv);

    public static Vector3D DN(double u, double v, in Cone3D c, int nu, int nv)
        => ConeDN(u, v, c.Position, c.RefRadius, c.SemiAngle, nu, nv);

    public static Vector3D DN(double u, double v, in Cylinder3D c, int nu, int nv)
        => CylinderDN(u, v, c.Position, c.Radius, nu, nv);

    public static Vector3D DN(double u, double v, in Sphere3D s, int nu, int nv)
        => SphereDN(u, v, s.Position, s.Radius, nu, nv);

    public static Vector3D DN(double u, double v, in Torus3D t, int nu, int nv)
        => TorusDN(u, v, t.Position, t.MajorRadius, t.MinorRadius, nu, nv);

    /// <summary>The point P of parameters (U, V).</summary>
    public static void D0(double u, double v, in Plane3D pl, out Point3D p) => PlaneD0(u, v, pl.Position, out p);

    public static void D0(double u, double v, in Cone3D c, out Point3D p)
        => ConeD0(u, v, c.Position, c.RefRadius, c.SemiAngle, out p);

    public static void D0(double u, double v, in Cylinder3D c, out Point3D p)
        => CylinderD0(u, v, c.Position, c.Radius, out p);

    public static void D0(double u, double v, in Sphere3D s, out Point3D p)
        => SphereD0(u, v, s.Position, s.Radius, out p);

    public static void D0(double u, double v, in Torus3D t, out Point3D p)
        => TorusD0(u, v, t.Position, t.MajorRadius, t.MinorRadius, out p);

    /// <summary>The point P of parameters (U, V) and the first derivatives Vu and Vv.</summary>
    public static void D1(double u, double v, in Plane3D pl, out Point3D p, out Vector3D vu, out Vector3D vv)
        => PlaneD1(u, v, pl.Position, out p, out vu, out vv);

    public static void D1(double u, double v, in Cone3D c, out Point3D p, out Vector3D vu, out Vector3D vv)
        => ConeD1(u, v, c.Position, c.RefRadius, c.SemiAngle, out p, out vu, out vv);

    public static void D1(double u, double v, in Cylinder3D c, out Point3D p, out Vector3D vu, out Vector3D vv)
        => CylinderD1(u, v, c.Position, c.Radius, out p, out vu, out vv);

    public static void D1(double u, double v, in Sphere3D s, out Point3D p, out Vector3D vu, out Vector3D vv)
        => SphereD1(u, v, s.Position, s.Radius, out p, out vu, out vv);

    public static void D1(double u, double v, in Torus3D t, out Point3D p, out Vector3D vu, out Vector3D vv)
        => TorusD1(u, v, t.Position, t.MajorRadius, t.MinorRadius, out p, out vu, out vv);

    /// <summary>The point, the first derivatives and the second derivatives Vuu, Vvv and Vuv.</summary>
    public static void D2(double u, double v, in Cone3D c, out Point3D p, out Vector3D vu, out Vector3D vv,
                          out Vector3D vuu, out Vector3D vvv, out Vector3D vuv)
        => ConeD2(u, v, c.Position, c.RefRadius, c.SemiAngle, out p, out vu, out vv, out vuu, out vvv, out vuv);

    public static void D2(double u, double v, in Cylinder3D c, out Point3D p, out Vector3D vu, out Vector3D vv,
                          out Vector3D vuu, out Vector3D vvv, out Vector3D vuv)
        => CylinderD2(u, v, c.Position, c.Radius, out p, out vu, out vv, out vuu, out vvv, out vuv);

    public static void D2(double u, double v, in Sphere3D s, out Point3D p, out Vector3D vu, out Vector3D vv,
                          out Vector3D vuu, out Vector3D vvv, out Vector3D vuv)
        => SphereD2(u, v, s.Position, s.Radius, out p, out vu, out vv, out vuu, out vvv, out vuv);

    public static void D2(double u, double v, in Torus3D t, out Point3D p, out Vector3D vu, out Vector3D vv,
                          out Vector3D vuu, out Vector3D vvv, out Vector3D vuv)
        => TorusD2(u, v, t.Position, t.MajorRadius, t.MinorRadius, out p, out vu, out vv, out vuu, out vvv, out vuv);

    /// <summary>
    /// The point, the first and second derivatives, and the third derivatives Vuuu, Vvvv, Vuuv and Vuvv.
    /// </summary>
    public static void D3(double u, double v, in Cone3D c, out Point3D p, out Vector3D vu, out Vector3D vv,
                          out Vector3D vuu, out Vector3D vvv, out Vector3D vuv,
                          out Vector3D vuuu, out Vector3D vvvv, out Vector3D vuuv, out Vector3D vuvv)
        => ConeD3(u, v, c.Position, c.RefRadius, c.SemiAngle, out p, out vu, out vv, out vuu, out vvv, out vuv,
                  out vuuu, out vvvv, out vuuv, out vuvv);

    public static void D3(double u, double v, in Cylinder3D c, out Point3D p, out Vector3D vu, out Vector3D vv,
                          out Vector3D vuu, out Vector3D vvv, out Vector3D vuv,
                          out Vector3D vuuu, out Vector3D vvvv, out Vector3D vuuv, out Vector3D vuvv)
        => CylinderD3(u, v, c.Position, c.Radius, out p, out vu, out vv, out vuu, out vvv, out vuv,
                      out vuuu, out vvvv, out vuuv, out vuvv);

    public static void D3(double u, double v, in Sphere3D s, out Point3D p, out Vector3D vu, out Vector3D vv,
                          out Vector3D vuu, out Vector3D vvv, out Vector3D vuv,
                          out Vector3D vuuu, out Vector3D vvvv, out Vector3D vuuv, out Vector3D vuvv)
        => SphereD3(u, v, s.Position, s.Radius, out p, out vu, out vv, out vuu, out vvv, out vuv,
                    out vuuu, out vvvv, out vuuv, out vuvv);

    public static void D3(double u, double v, in Torus3D t, out Point3D p, out Vector3D vu, out Vector3D vv,
                          out Vector3D vuu, out Vector3D vvv, out Vector3D vuv,
                          out Vector3D vuuu, out Vector3D vvvv, out Vector3D vuuv, out Vector3D vuvv)
        => TorusD3(u, v, t.Position, t.MajorRadius, t.MinorRadius, out p, out vu, out vv, out vuu, out vvv, out vuv,
                   out vuuu, out vvvv, out vuuv, out vuvv);

    /// <summary>The parameters (U, V) of a point on an elementary surface.</summary>
    public static void Parameters(in Plane3D pl, in Point3D p, out double u, out double v)
        => PlaneParameters(pl.Position, p, out u, out v);

    public static void Parameters(in Cylinder3D c, in Point3D p, out double u, out double v)
        => CylinderParameters(c.Position, c.Radius, p, out u, out v);

    public static void Parameters(in Cone3D c, in Point3D p, out double u, out double v)
        => ConeParameters(c.Position, c.RefRadius, c.SemiAngle, p, out u, out v);

    public static void Parameters(in Sphere3D s, in Point3D p, out double u, out double v)
        => SphereParameters(s.Position, s.Radius, p, out u, out v);

    public static void Parameters(in Torus3D t, in Point3D p, out double u, out double v)
        => TorusParameters(t.Position, t.MajorRadius, t.MinorRadius, p, out u, out v);

    // ElSLib.cxx

    private const double Pipi = M_PI + M_PI;

    // Threshold for angle normalization to avoid discontinuity near zero
    private static readonly double NEGATIVE_RESOLUTION = -global::Kerf.Engine.Foundation.Precision.Computational();

    // Normalize angle to [0, 2*PI] range, with special handling
    // for values very close to zero to avoid discontinuity.
    // Preserves values at exactly 2*PI for proper seam handling.
    private static void NormalizeAngle(ref double angle)
    {
        while (angle < NEGATIVE_RESOLUTION)
        {
            angle += Pipi;
        }

        // Only normalize angles strictly greater than 2*PI (with small tolerance)
        // to preserve the closing seam value of exactly 2*PI
        while (angle > Pipi * (1.0 + Constants.Resolution()))
        {
            angle -= Pipi;
        }

        if (angle < 0.0)
        {
            angle = 0.0;
        }
    }

    public static Point3D PlaneValue(double u, double v, in CoordinateSystem pos)
    {
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        return new Point3D(u * xDir.X() + v * yDir.X() + pLoc.X(),
                          u * xDir.Y() + v * yDir.Y() + pLoc.Y(),
                          u * xDir.Z() + v * yDir.Z() + pLoc.Z());
    }

    public static Point3D ConeValue(double u, double v, in CoordinateSystem pos, double radius, double sAngle)
    {
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double r = radius + v * Math.Sin(sAngle);
        double a3 = v * Math.Cos(sAngle);
        double a1 = r * Math.Cos(u);
        double a2 = r * Math.Sin(u);
        return new Point3D(a1 * xDir.X() + a2 * yDir.X() + a3 * zDir.X() + pLoc.X(),
                          a1 * xDir.Y() + a2 * yDir.Y() + a3 * zDir.Y() + pLoc.Y(),
                          a1 * xDir.Z() + a2 * yDir.Z() + a3 * zDir.Z() + pLoc.Z());
    }

    public static Point3D CylinderValue(double u, double v, in CoordinateSystem pos, double radius)
    {
        // M(u,v) = C + Radius * ( Xdir * cos(u) + Ydir * sin(u)) + V * Zdir
        // where C is the location point of the Axis2placement
        // Xdir, Ydir ,Zdir are the directions of the local coordinates system
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double a1 = radius * Math.Cos(u);
        double a2 = radius * Math.Sin(u);
        return new Point3D(a1 * xDir.X() + a2 * yDir.X() + v * zDir.X() + pLoc.X(),
                          a1 * xDir.Y() + a2 * yDir.Y() + v * zDir.Y() + pLoc.Y(),
                          a1 * xDir.Z() + a2 * yDir.Z() + v * zDir.Z() + pLoc.Z());
    }

    public static Point3D SphereValue(double u, double v, in CoordinateSystem pos, double radius)
    {
        // M(U,V) = Location +
        //          R * CosV (CosU * XDirection + SinU * YDirection) +
        //          R * SinV * Direction
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double r = radius * Math.Cos(v);
        double a3 = radius * Math.Sin(v);
        double a1 = r * Math.Cos(u);
        double a2 = r * Math.Sin(u);
        return new Point3D(a1 * xDir.X() + a2 * yDir.X() + a3 * zDir.X() + pLoc.X(),
                          a1 * xDir.Y() + a2 * yDir.Y() + a3 * zDir.Y() + pLoc.Y(),
                          a1 * xDir.Z() + a2 * yDir.Z() + a3 * zDir.Z() + pLoc.Z());
    }

    public static Point3D TorusValue(double u, double v, in CoordinateSystem pos, double majorRadius, double minorRadius)
    {
        // M(U,V) =
        //   Location +
        //   (MajRadius+MinRadius*cos(V)) * (cos(U)*XDirection + sin(U)*YDirection) +
        //   MinorRadius * sin(V) * Direction
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double r = majorRadius + minorRadius * Math.Cos(v);
        double a3 = minorRadius * Math.Sin(v);
        double a1 = r * Math.Cos(u);
        double a2 = r * Math.Sin(u);
        // OCC620: components below the rounding noise are zeroed
        double eps = 10.0 * (minorRadius + majorRadius) * RealEpsilon();
        if (Math.Abs(a1) <= eps)
        {
            a1 = 0.0;
        }

        if (Math.Abs(a2) <= eps)
        {
            a2 = 0.0;
        }

        if (Math.Abs(a3) <= eps)
        {
            a3 = 0.0;
        }

        return new Point3D(a1 * xDir.X() + a2 * yDir.X() + a3 * zDir.X() + pLoc.X(),
                          a1 * xDir.Y() + a2 * yDir.Y() + a3 * zDir.Y() + pLoc.Y(),
                          a1 * xDir.Z() + a2 * yDir.Z() + a3 * zDir.Z() + pLoc.Z());
    }

    public static Vector3D PlaneDN(double u, double v, in CoordinateSystem pos, int nu, int nv)
    {
        if (nu == 0 && nv == 1)
        {
            return new Vector3D(pos.YDirection);
        }
        else if (nu == 1 && nv == 0)
        {
            return new Vector3D(pos.XDirection);
        }

        return new Vector3D(0.0, 0.0, 0.0);
    }

    public static Vector3D ConeDN(double u, double v, in CoordinateSystem pos, double radius, double sAngle, int nu, int nv)
    {
        Xyz xdir = pos.XDirection.XYZ();
        Xyz ydir = pos.YDirection.XYZ();
        double um = u + nu * M_PI_2; // M_PI * 0.5
        xdir.Multiply(Math.Cos(um));
        ydir.Multiply(Math.Sin(um));
        xdir.Add(ydir);
        if (nv == 0)
        {
            xdir.Multiply(radius + v * Math.Sin(sAngle));
            if (nu == 0)
            {
                xdir.Add(pos.Location.XYZ());
            }

            return new Vector3D(xdir);
        }
        else if (nv == 1)
        {
            xdir.Multiply(Math.Sin(sAngle));
            if (nu == 0)
            {
                xdir.Add(pos.Direction.XYZ() * Math.Cos(sAngle));
            }

            return new Vector3D(xdir);
        }

        return new Vector3D(0.0, 0.0, 0.0);
    }

    public static Vector3D CylinderDN(double u, double v, in CoordinateSystem pos, double radius, int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            return new Vector3D();
        }

        if (nv == 0)
        {
            double rCosU = radius * Math.Cos(u);
            double rSinU = radius * Math.Sin(u);
            Xyz xdir = pos.XDirection.XYZ();
            Xyz ydir = pos.YDirection.XYZ();
            if ((nu + 6) % 4 == 0)
            {
                xdir.Multiply(-rCosU);
                ydir.Multiply(-rSinU);
            }
            else if ((nu + 5) % 4 == 0)
            {
                xdir.Multiply(rSinU);
                ydir.Multiply(-rCosU);
            }
            else if ((nu + 3) % 4 == 0)
            {
                xdir.Multiply(-rSinU);
                ydir.Multiply(rCosU);
            }
            else if (nu % 4 == 0)
            {
                xdir.Multiply(rCosU);
                ydir.Multiply(rSinU);
            }

            xdir.Add(ydir);
            return new Vector3D(xdir);
        }
        else if (nv == 1 && nu == 0)
        {
            return new Vector3D(pos.Direction);
        }
        else
        {
            return new Vector3D(0.0, 0.0, 0.0);
        }
    }

    public static Vector3D SphereDN(double u, double v, in CoordinateSystem pos, double radius, int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            return new Vector3D();
        }

        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double rCosV = radius * Math.Cos(v);
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        double a1, a2, a3, x, y, z;
        if (nu == 0)
        {
            double rSinV = radius * Math.Sin(v);
            if (IsOdd(nv))
            {
                a1 = -rSinV * cosU;
                a2 = -rSinV * sinU;
                a3 = rCosV;
            }
            else
            {
                a1 = -rCosV * cosU;
                a2 = -rCosV * sinU;
                a3 = -rSinV;
            }

            x = a1 * xDir.X() + a2 * yDir.X() + a3 * zDir.X();
            y = a1 * xDir.Y() + a2 * yDir.Y() + a3 * zDir.Y();
            z = a1 * xDir.Z() + a2 * yDir.Z() + a3 * zDir.Z();
            if ((nv + 2) % 4 != 0 && (nv + 3) % 4 != 0)
            {
                x = -x;
                y = -y;
                z = -z;
            }
        }
        else if (nv == 0)
        {
            if (IsOdd(nu))
            {
                a1 = -rCosV * sinU;
                a2 = rCosV * cosU;
            }
            else
            {
                a1 = rCosV * cosU;
                a2 = rCosV * sinU;
            }

            x = a1 * xDir.X() + a2 * yDir.X();
            y = a1 * xDir.Y() + a2 * yDir.Y();
            z = a1 * xDir.Z() + a2 * yDir.Z();
            if ((nu + 2) % 4 == 0 || (nu + 1) % 4 == 0)
            {
                x = -x;
                y = -y;
                z = -z;
            }
        }
        else
        {
            double rSinV = radius * Math.Sin(v);
            if (IsOdd(nu))
            {
                a1 = -sinU;
                a2 = cosU;
            }
            else
            {
                a1 = -cosU;
                a2 = -sinU;
            }

            if (IsOdd(nv))
            {
                a3 = -rSinV;
            }
            else
            {
                a3 = -rCosV;
            }

            x = (a1 * xDir.X() + a2 * yDir.X()) * a3;
            y = (a1 * xDir.Y() + a2 * yDir.Y()) * a3;
            z = (a1 * xDir.Z() + a2 * yDir.Z()) * a3;
            if (((nu + 2) % 4 != 0 && (nu + 3) % 4 != 0 && ((nv + 2) % 4 == 0 || (nv + 3) % 4 == 0))
                || (((nu + 2) % 4 == 0 || (nu + 3) % 4 == 0) && (nv + 2) % 4 != 0 && (nv + 3) % 4 != 0))
            {
                x = -x;
                y = -y;
                z = -z;
            }
        }

        return new Vector3D(x, y, z);
    }

    public static Vector3D TorusDN(double u, double v, in CoordinateSystem pos, double majorRadius, double minorRadius,
                                 int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            return new Vector3D();
        }

        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        double a1, a2, a3, x = 0, y = 0, z = 0;
        // OCC620: components below the rounding noise are zeroed
        double eps = 10.0 * (minorRadius + majorRadius) * RealEpsilon();
        if (nv == 0)
        {
            double r = majorRadius + minorRadius * Math.Cos(v);
            if (IsOdd(nu))
            {
                a1 = -r * sinU;
                a2 = r * cosU;
            }
            else
            {
                a1 = -r * cosU;
                a2 = -r * sinU;
            }

            if (Math.Abs(a1) <= eps)
            {
                a1 = 0.0;
            }

            if (Math.Abs(a2) <= eps)
            {
                a2 = 0.0;
            }

            x = a1 * xDir.X() + a2 * yDir.X();
            y = a1 * xDir.Y() + a2 * yDir.Y();
            z = a1 * xDir.Z() + a2 * yDir.Z();
            if ((nu + 2) % 4 != 0 && (nu + 3) % 4 != 0)
            {
                x = -x;
                y = -y;
                z = -z;
            }
        }
        else if (nu == 0)
        {
            double rCosV = minorRadius * Math.Cos(v);
            double rSinV = minorRadius * Math.Sin(v);
            if (IsOdd(nv))
            {
                a1 = -rSinV * cosU;
                a2 = -rSinV * sinU;
                a3 = rCosV;
            }
            else
            {
                a1 = -rCosV * cosU;
                a2 = -rCosV * sinU;
                a3 = -rSinV;
            }

            if (Math.Abs(a1) <= eps)
            {
                a1 = 0.0;
            }

            if (Math.Abs(a2) <= eps)
            {
                a2 = 0.0;
            }

            if (Math.Abs(a3) <= eps)
            {
                a3 = 0.0;
            }

            x = a1 * xDir.X() + a2 * yDir.X() + a3 * zDir.X();
            y = a1 * xDir.Y() + a2 * yDir.Y() + a3 * zDir.Y();
            z = a1 * xDir.Z() + a2 * yDir.Z() + a3 * zDir.Z();
            if ((nv + 2) % 4 != 0 && (nv + 3) % 4 != 0)
            {
                x = -x;
                y = -y;
                z = -z;
            }
        }
        else
        {
            // the odd/odd and even/even branches change sign with (Nu + Nv) mod 4, as the others do (OCCT didn't, so
            // (1, 3) and (2, 4) came out reversed; Kerf decision 1486)
            if (IsOdd(nu) && IsOdd(nv))
            {
                double rSinV = minorRadius * Math.Sin(v);
                a1 = rSinV * sinU;
                a2 = -rSinV * cosU;
                if (Math.Abs(a1) <= eps)
                {
                    a1 = 0.0;
                }

                if (Math.Abs(a2) <= eps)
                {
                    a2 = 0.0;
                }

                x = a1 * xDir.X() + a2 * yDir.X();
                y = a1 * xDir.Y() + a2 * yDir.Y();
                z = a1 * xDir.Z() + a2 * yDir.Z();
                if ((nu + nv) % 4 == 0)
                {
                    x = -x;
                    y = -y;
                    z = -z;
                }
            }
            else if (IsEven(nu) && IsEven(nv))
            {
                double rCosV = minorRadius * Math.Cos(v);
                a1 = rCosV * cosU;
                a2 = rCosV * sinU;
                if (Math.Abs(a1) <= eps)
                {
                    a1 = 0.0;
                }

                if (Math.Abs(a2) <= eps)
                {
                    a2 = 0.0;
                }

                x = a1 * xDir.X() + a2 * yDir.X();
                y = a1 * xDir.Y() + a2 * yDir.Y();
                z = a1 * xDir.Z() + a2 * yDir.Z();
                if ((nu + nv) % 4 == 2)
                {
                    x = -x;
                    y = -y;
                    z = -z;
                }
            }
            else if (IsEven(nv) && IsOdd(nu))
            {
                double rCosV = minorRadius * Math.Cos(v);
                a1 = rCosV * sinU;
                a2 = -rCosV * cosU;
                if (Math.Abs(a1) <= eps)
                {
                    a1 = 0.0;
                }

                if (Math.Abs(a2) <= eps)
                {
                    a2 = 0.0;
                }

                x = a1 * xDir.X() + a2 * yDir.X();
                y = a1 * xDir.Y() + a2 * yDir.Y();
                z = a1 * xDir.Z() + a2 * yDir.Z();
                if (((nv + nu + 3) % 4) == 0)
                {
                    x = -x;
                    y = -y;
                    z = -z;
                }
            }
            else if (IsOdd(nv) && IsEven(nu))
            {
                double rSinV = minorRadius * Math.Sin(v);
                a1 = rSinV * cosU;
                a2 = rSinV * sinU;
                if (Math.Abs(a1) <= eps)
                {
                    a1 = 0.0;
                }

                if (Math.Abs(a2) <= eps)
                {
                    a2 = 0.0;
                }

                x = a1 * xDir.X() + a2 * yDir.X();
                y = a1 * xDir.Y() + a2 * yDir.Y();
                z = a1 * xDir.Z() + a2 * yDir.Z();
                if (((nu + nv + 3) % 4) == 0)
                {
                    x = -x;
                    y = -y;
                    z = -z;
                }
            }
        }

        return new Vector3D(x, y, z);
    }

    // The D0..D3 functions below assign whole points and vectors where OCCT calls SetX, SetY and SetZ on its
    // output references; the coordinates, and the order they are computed in, are the same.

    public static void PlaneD0(double u, double v, in CoordinateSystem pos, out Point3D p)
    {
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        p = new Point3D(u * xDir.X() + v * yDir.X() + pLoc.X(),
                       u * xDir.Y() + v * yDir.Y() + pLoc.Y(),
                       u * xDir.Z() + v * yDir.Z() + pLoc.Z());
    }

    public static void ConeD0(double u, double v, in CoordinateSystem pos, double radius, double sAngle, out Point3D p)
    {
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double r = radius + v * Math.Sin(sAngle);
        double a3 = v * Math.Cos(sAngle);
        double a1 = r * Math.Cos(u);
        double a2 = r * Math.Sin(u);
        p = new Point3D(a1 * xDir.X() + a2 * yDir.X() + a3 * zDir.X() + pLoc.X(),
                       a1 * xDir.Y() + a2 * yDir.Y() + a3 * zDir.Y() + pLoc.Y(),
                       a1 * xDir.Z() + a2 * yDir.Z() + a3 * zDir.Z() + pLoc.Z());
    }

    public static void CylinderD0(double u, double v, in CoordinateSystem pos, double radius, out Point3D p)
    {
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double a1 = radius * Math.Cos(u);
        double a2 = radius * Math.Sin(u);
        p = new Point3D(a1 * xDir.X() + a2 * yDir.X() + v * zDir.X() + pLoc.X(),
                       a1 * xDir.Y() + a2 * yDir.Y() + v * zDir.Y() + pLoc.Y(),
                       a1 * xDir.Z() + a2 * yDir.Z() + v * zDir.Z() + pLoc.Z());
    }

    public static void SphereD0(double u, double v, in CoordinateSystem pos, double radius, out Point3D p)
    {
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double r = radius * Math.Cos(v);
        double a3 = radius * Math.Sin(v);
        double a1 = r * Math.Cos(u);
        double a2 = r * Math.Sin(u);
        p = new Point3D(a1 * xDir.X() + a2 * yDir.X() + a3 * zDir.X() + pLoc.X(),
                       a1 * xDir.Y() + a2 * yDir.Y() + a3 * zDir.Y() + pLoc.Y(),
                       a1 * xDir.Z() + a2 * yDir.Z() + a3 * zDir.Z() + pLoc.Z());
    }

    public static void TorusD0(double u, double v, in CoordinateSystem pos, double majorRadius, double minorRadius, out Point3D p)
    {
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double r = majorRadius + minorRadius * Math.Cos(v);
        double a3 = minorRadius * Math.Sin(v);
        double a1 = r * Math.Cos(u);
        double a2 = r * Math.Sin(u);
        // OCC620: components below the rounding noise are zeroed
        double eps = 10.0 * (minorRadius + majorRadius) * RealEpsilon();
        if (Math.Abs(a1) <= eps)
        {
            a1 = 0.0;
        }

        if (Math.Abs(a2) <= eps)
        {
            a2 = 0.0;
        }

        if (Math.Abs(a3) <= eps)
        {
            a3 = 0.0;
        }

        p = new Point3D(a1 * xDir.X() + a2 * yDir.X() + a3 * zDir.X() + pLoc.X(),
                       a1 * xDir.Y() + a2 * yDir.Y() + a3 * zDir.Y() + pLoc.Y(),
                       a1 * xDir.Z() + a2 * yDir.Z() + a3 * zDir.Z() + pLoc.Z());
    }

    public static void PlaneD1(double u, double v, in CoordinateSystem pos, out Point3D p, out Vector3D vu, out Vector3D vv)
    {
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        p = new Point3D(u * xDir.X() + v * yDir.X() + pLoc.X(),
                       u * xDir.Y() + v * yDir.Y() + pLoc.Y(),
                       u * xDir.Z() + v * yDir.Z() + pLoc.Z());
        vu = new Vector3D(xDir.X(), xDir.Y(), xDir.Z());
        vv = new Vector3D(yDir.X(), yDir.Y(), yDir.Z());
    }

    public static void ConeD1(double u, double v, in CoordinateSystem pos, double radius, double sAngle,
                              out Point3D p, out Vector3D vu, out Vector3D vv)
    {
        // Z = V * cos(SAngle)
        // M(U,V) = Location() + V * cos(SAngle) * ZDirection() +
        // (Radius + V*Sin(SAng)) * (cos(U) * XDirection() + sin(U) * YDirection())
        // D1U =
        // (Radius + V*Sin(SAng)) * (-sin(U) * XDirection() + cos(U) * YDirection())
        // D1V =
        // Direction() *cos(SAngle) + sin(SAng) * (cos(U) * XDirection() + sin(U) * YDirection())
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double cosA = Math.Cos(sAngle);
        double sinA = Math.Sin(sAngle);
        double r = radius + v * sinA;
        double a3 = v * cosA;
        double a1 = r * cosU;
        double a2 = r * sinU;
        double r1 = sinA * cosU;
        double r2 = sinA * sinU;
        p = new Point3D(a1 * xDir.X() + a2 * yDir.X() + a3 * zDir.X() + pLoc.X(),
                       a1 * xDir.Y() + a2 * yDir.Y() + a3 * zDir.Y() + pLoc.Y(),
                       a1 * xDir.Z() + a2 * yDir.Z() + a3 * zDir.Z() + pLoc.Z());
        vu = new Vector3D(-a2 * xDir.X() + a1 * yDir.X(),
                        -a2 * xDir.Y() + a1 * yDir.Y(),
                        -a2 * xDir.Z() + a1 * yDir.Z());
        vv = new Vector3D(r1 * xDir.X() + r2 * yDir.X() + cosA * zDir.X(),
                        r1 * xDir.Y() + r2 * yDir.Y() + cosA * zDir.Y(),
                        r1 * xDir.Z() + r2 * yDir.Z() + cosA * zDir.Z());
    }

    public static void CylinderD1(double u, double v, in CoordinateSystem pos, double radius,
                                  out Point3D p, out Vector3D vu, out Vector3D vv)
    {
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double a1 = radius * Math.Cos(u);
        double a2 = radius * Math.Sin(u);
        p = new Point3D(a1 * xDir.X() + a2 * yDir.X() + v * zDir.X() + pLoc.X(),
                       a1 * xDir.Y() + a2 * yDir.Y() + v * zDir.Y() + pLoc.Y(),
                       a1 * xDir.Z() + a2 * yDir.Z() + v * zDir.Z() + pLoc.Z());
        vu = new Vector3D(-a2 * xDir.X() + a1 * yDir.X(),
                        -a2 * xDir.Y() + a1 * yDir.Y(),
                        -a2 * xDir.Z() + a1 * yDir.Z());
        vv = new Vector3D(zDir.X(), zDir.Y(), zDir.Z());
    }

    public static void SphereD1(double u, double v, in CoordinateSystem pos, double radius,
                                out Point3D p, out Vector3D vu, out Vector3D vv)
    {
        // Vxy = CosU * XDirection + SinU * YDirection
        // DVxy = -SinU * XDirection + CosU * YDirection
        // P(U,V) = Location +  R * CosV * Vxy  +   R * SinV * Direction
        // Vu = R * CosV * DVxy
        // Vv = -R * SinV * Vxy + R * CosV * Direction
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double r1 = radius * Math.Cos(v);
        double r2 = radius * Math.Sin(v);
        double a1 = r1 * cosU;
        double a2 = r1 * sinU;
        double a3 = r2 * cosU;
        double a4 = r2 * sinU;
        p = new Point3D(a1 * xDir.X() + a2 * yDir.X() + r2 * zDir.X() + pLoc.X(),
                       a1 * xDir.Y() + a2 * yDir.Y() + r2 * zDir.Y() + pLoc.Y(),
                       a1 * xDir.Z() + a2 * yDir.Z() + r2 * zDir.Z() + pLoc.Z());
        vu = new Vector3D(-a2 * xDir.X() + a1 * yDir.X(),
                        -a2 * xDir.Y() + a1 * yDir.Y(),
                        -a2 * xDir.Z() + a1 * yDir.Z());
        vv = new Vector3D(-a3 * xDir.X() - a4 * yDir.X() + r1 * zDir.X(),
                        -a3 * xDir.Y() - a4 * yDir.Y() + r1 * zDir.Y(),
                        -a3 * xDir.Z() - a4 * yDir.Z() + r1 * zDir.Z());
    }

    public static void TorusD1(double u, double v, in CoordinateSystem pos, double majorRadius, double minorRadius,
                               out Point3D p, out Vector3D vu, out Vector3D vv)
    {
        // P(U,V) =
        //   Location +
        //   (MajorRadius+MinorRadius*cos(V)) * (cos(U)*XDirection + sin(U)*YDirection) +
        //   MinorRadius * sin(V) * Direction
        // Vv = -MinorRadius * sin(V) * (cos(U)*XDirection + sin(U)*YDirection) +
        //       MinorRadius * cos(V) * Direction
        // Vu = (MajorRadius+MinorRadius*cos(V)) * (-sin(U)*XDirection + cos(U)*YDirection)
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double r1 = minorRadius * Math.Cos(v);
        double r2 = minorRadius * Math.Sin(v);
        double r = majorRadius + r1;
        double a1 = r * cosU;
        double a2 = r * sinU;
        double a3 = r2 * cosU;
        double a4 = r2 * sinU;
        // OCC620: components below the rounding noise are zeroed
        double eps = 10.0 * (minorRadius + majorRadius) * RealEpsilon();
        if (Math.Abs(a1) <= eps)
        {
            a1 = 0.0;
        }

        if (Math.Abs(a2) <= eps)
        {
            a2 = 0.0;
        }

        if (Math.Abs(a3) <= eps)
        {
            a3 = 0.0;
        }

        if (Math.Abs(a4) <= eps)
        {
            a4 = 0.0;
        }

        p = new Point3D(a1 * xDir.X() + a2 * yDir.X() + r2 * zDir.X() + pLoc.X(),
                       a1 * xDir.Y() + a2 * yDir.Y() + r2 * zDir.Y() + pLoc.Y(),
                       a1 * xDir.Z() + a2 * yDir.Z() + r2 * zDir.Z() + pLoc.Z());
        vu = new Vector3D(-a2 * xDir.X() + a1 * yDir.X(),
                        -a2 * xDir.Y() + a1 * yDir.Y(),
                        -a2 * xDir.Z() + a1 * yDir.Z());
        vv = new Vector3D(-a3 * xDir.X() - a4 * yDir.X() + r1 * zDir.X(),
                        -a3 * xDir.Y() - a4 * yDir.Y() + r1 * zDir.Y(),
                        -a3 * xDir.Z() - a4 * yDir.Z() + r1 * zDir.Z());
    }

    public static void ConeD2(double u, double v, in CoordinateSystem pos, double radius, double sAngle,
                              out Point3D p, out Vector3D vu, out Vector3D vv,
                              out Vector3D vuu, out Vector3D vvv, out Vector3D vuv)
    {
        // D2U = (Radius + V*Sin(SAng)) * (-cos(U) * XDirection() - sin(U) * YDirection())
        // D2V = 0.0
        // DUV = sin(SAng) * (-sin(U) * XDirection() + cos(U) * YDirection())
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double cosA = Math.Cos(sAngle);
        double sinA = Math.Sin(sAngle);
        double r = radius + v * sinA;
        double a3 = v * cosA;
        double a1 = r * cosU;
        double a2 = r * sinU;
        double r1 = sinA * cosU;
        double r2 = sinA * sinU;
        double som1X = a1 * xDir.X() + a2 * yDir.X();
        double som1Y = a1 * xDir.Y() + a2 * yDir.Y();
        double som1Z = a1 * xDir.Z() + a2 * yDir.Z();
        p = new Point3D(som1X + a3 * zDir.X() + pLoc.X(),
                       som1Y + a3 * zDir.Y() + pLoc.Y(),
                       som1Z + a3 * zDir.Z() + pLoc.Z());
        vu = new Vector3D(-a2 * xDir.X() + a1 * yDir.X(),
                        -a2 * xDir.Y() + a1 * yDir.Y(),
                        -a2 * xDir.Z() + a1 * yDir.Z());
        vv = new Vector3D(r1 * xDir.X() + r2 * yDir.X() + cosA * zDir.X(),
                        r1 * xDir.Y() + r2 * yDir.Y() + cosA * zDir.Y(),
                        r1 * xDir.Z() + r2 * yDir.Z() + cosA * zDir.Z());
        vuu = new Vector3D(-som1X, -som1Y, -som1Z);
        vvv = new Vector3D(0.0, 0.0, 0.0);
        vuv = new Vector3D(-r2 * xDir.X() + r1 * yDir.X(),
                         -r2 * xDir.Y() + r1 * yDir.Y(),
                         -r2 * xDir.Z() + r1 * yDir.Z());
    }

    public static void CylinderD2(double u, double v, in CoordinateSystem pos, double radius,
                                  out Point3D p, out Vector3D vu, out Vector3D vv,
                                  out Vector3D vuu, out Vector3D vvv, out Vector3D vuv)
    {
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double a1 = radius * Math.Cos(u);
        double a2 = radius * Math.Sin(u);
        double som1X = a1 * xDir.X() + a2 * yDir.X();
        double som1Y = a1 * xDir.Y() + a2 * yDir.Y();
        double som1Z = a1 * xDir.Z() + a2 * yDir.Z();
        p = new Point3D(som1X + v * zDir.X() + pLoc.X(),
                       som1Y + v * zDir.Y() + pLoc.Y(),
                       som1Z + v * zDir.Z() + pLoc.Z());
        vu = new Vector3D(-a2 * xDir.X() + a1 * yDir.X(),
                        -a2 * xDir.Y() + a1 * yDir.Y(),
                        -a2 * xDir.Z() + a1 * yDir.Z());
        vv = new Vector3D(zDir.X(), zDir.Y(), zDir.Z());
        vuu = new Vector3D(-som1X, -som1Y, -som1Z);
        vvv = new Vector3D(0.0, 0.0, 0.0);
        vuv = new Vector3D(0.0, 0.0, 0.0);
    }

    public static void SphereD2(double u, double v, in CoordinateSystem pos, double radius,
                                out Point3D p, out Vector3D vu, out Vector3D vv,
                                out Vector3D vuu, out Vector3D vvv, out Vector3D vuv)
    {
        // Vxy = CosU * XDirection + SinU * YDirection
        // DVxy = -SinU * XDirection + CosU * YDirection
        // P(U,V) = Location +  R * CosV * Vxy  +   R * SinV * Direction
        // Vu = R * CosV * DVxy
        // Vuu = - R * CosV * Vxy
        // Vv = -R * SinV * Vxy + R * CosV * Direction
        // Vvv = -R * CosV * Vxy - R * SinV * Direction
        // Vuv = - R * SinV * DVxy
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double r1 = radius * Math.Cos(v);
        double r2 = radius * Math.Sin(v);
        double a1 = r1 * cosU;
        double a2 = r1 * sinU;
        double a3 = r2 * cosU;
        double a4 = r2 * sinU;
        double som1X = a1 * xDir.X() + a2 * yDir.X();
        double som1Y = a1 * xDir.Y() + a2 * yDir.Y();
        double som1Z = a1 * xDir.Z() + a2 * yDir.Z();
        double r2ZX = r2 * zDir.X();
        double r2ZY = r2 * zDir.Y();
        double r2ZZ = r2 * zDir.Z();
        p = new Point3D(som1X + r2ZX + pLoc.X(),
                       som1Y + r2ZY + pLoc.Y(),
                       som1Z + r2ZZ + pLoc.Z());
        vu = new Vector3D(-a2 * xDir.X() + a1 * yDir.X(),
                        -a2 * xDir.Y() + a1 * yDir.Y(),
                        -a2 * xDir.Z() + a1 * yDir.Z());
        vv = new Vector3D(-a3 * xDir.X() - a4 * yDir.X() + r1 * zDir.X(),
                        -a3 * xDir.Y() - a4 * yDir.Y() + r1 * zDir.Y(),
                        -a3 * xDir.Z() - a4 * yDir.Z() + r1 * zDir.Z());
        vuu = new Vector3D(-som1X, -som1Y, -som1Z);
        vvv = new Vector3D(-som1X - r2ZX, -som1Y - r2ZY, -som1Z - r2ZZ);
        vuv = new Vector3D(a4 * xDir.X() - a3 * yDir.X(),
                         a4 * xDir.Y() - a3 * yDir.Y(),
                         a4 * xDir.Z() - a3 * yDir.Z());
    }

    public static void TorusD2(double u, double v, in CoordinateSystem pos, double majorRadius, double minorRadius,
                               out Point3D p, out Vector3D vu, out Vector3D vv,
                               out Vector3D vuu, out Vector3D vvv, out Vector3D vuv)
    {
        // Vvv = -MinorRadius * cos(V) * (cos(U)*XDirection + sin(U)*YDirection)
        //       -MinorRadius * sin(V) * Direction
        // Vuu = -(MajorRadius+MinorRadius*cos(V)) * (cos(U)*XDirection + sin(U)*YDirection)
        // Vuv = MinorRadius * sin(V) * (sin(U)*XDirection - cos(U)*YDirection)
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double r1 = minorRadius * Math.Cos(v);
        double r2 = minorRadius * Math.Sin(v);
        double r = majorRadius + r1;
        double a1 = r * cosU;
        double a2 = r * sinU;
        double a3 = r2 * cosU;
        double a4 = r2 * sinU;
        double a5 = r1 * cosU;
        double a6 = r1 * sinU;
        // OCC620: components below the rounding noise are zeroed
        double eps = 10.0 * (minorRadius + majorRadius) * RealEpsilon();
        if (Math.Abs(a1) <= eps)
        {
            a1 = 0.0;
        }

        if (Math.Abs(a2) <= eps)
        {
            a2 = 0.0;
        }

        if (Math.Abs(a3) <= eps)
        {
            a3 = 0.0;
        }

        if (Math.Abs(a4) <= eps)
        {
            a4 = 0.0;
        }

        if (Math.Abs(a5) <= eps)
        {
            a5 = 0.0;
        }

        if (Math.Abs(a6) <= eps)
        {
            a6 = 0.0;
        }

        double som1X = a1 * xDir.X() + a2 * yDir.X();
        double som1Y = a1 * xDir.Y() + a2 * yDir.Y();
        double som1Z = a1 * xDir.Z() + a2 * yDir.Z();
        double r2ZX = r2 * zDir.X();
        double r2ZY = r2 * zDir.Y();
        double r2ZZ = r2 * zDir.Z();
        p = new Point3D(som1X + r2ZX + pLoc.X(),
                       som1Y + r2ZY + pLoc.Y(),
                       som1Z + r2ZZ + pLoc.Z());
        vu = new Vector3D(-a2 * xDir.X() + a1 * yDir.X(),
                        -a2 * xDir.Y() + a1 * yDir.Y(),
                        -a2 * xDir.Z() + a1 * yDir.Z());
        vv = new Vector3D(-a3 * xDir.X() - a4 * yDir.X() + r1 * zDir.X(),
                        -a3 * xDir.Y() - a4 * yDir.Y() + r1 * zDir.Y(),
                        -a3 * xDir.Z() - a4 * yDir.Z() + r1 * zDir.Z());
        vuu = new Vector3D(-som1X, -som1Y, -som1Z);
        vvv = new Vector3D(-a5 * xDir.X() - a6 * yDir.X() - r2ZX,
                         -a5 * xDir.Y() - a6 * yDir.Y() - r2ZY,
                         -a5 * xDir.Z() - a6 * yDir.Z() - r2ZZ);
        vuv = new Vector3D(a4 * xDir.X() - a3 * yDir.X(),
                         a4 * xDir.Y() - a3 * yDir.Y(),
                         a4 * xDir.Z() - a3 * yDir.Z());
    }

    public static void ConeD3(double u, double v, in CoordinateSystem pos, double radius, double sAngle,
                              out Point3D p, out Vector3D vu, out Vector3D vv,
                              out Vector3D vuu, out Vector3D vvv, out Vector3D vuv,
                              out Vector3D vuuu, out Vector3D vvvv, out Vector3D vuuv, out Vector3D vuvv)
    {
        // D3U = (Radius + V*Sin(SAng)) * (sin(U) * XDirection() - cos(U) * YDirection())
        // DUVV = 0.0
        // D3V = 0.0
        // DUUV = sin(SAng) * (-cos(U)*XDirection()-sin(U) * YDirection())
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double cosA = Math.Cos(sAngle);
        double sinA = Math.Sin(sAngle);
        double r = radius + v * sinA;
        double a3 = v * cosA;
        double a1 = r * cosU;
        double a2 = r * sinU;
        double r1 = sinA * cosU;
        double r2 = sinA * sinU;
        double som1X = a1 * xDir.X() + a2 * yDir.X();
        double som1Y = a1 * xDir.Y() + a2 * yDir.Y();
        double som1Z = a1 * xDir.Z() + a2 * yDir.Z();
        double som2X = r1 * xDir.X() + r2 * yDir.X();
        double som2Y = r1 * xDir.Y() + r2 * yDir.Y();
        double som2Z = r1 * xDir.Z() + r2 * yDir.Z();
        double dif1X = a2 * xDir.X() - a1 * yDir.X();
        double dif1Y = a2 * xDir.Y() - a1 * yDir.Y();
        double dif1Z = a2 * xDir.Z() - a1 * yDir.Z();
        p = new Point3D(som1X + a3 * zDir.X() + pLoc.X(),
                       som1Y + a3 * zDir.Y() + pLoc.Y(),
                       som1Z + a3 * zDir.Z() + pLoc.Z());
        vu = new Vector3D(-dif1X, -dif1Y, -dif1Z);
        vv = new Vector3D(som2X + cosA * zDir.X(),
                        som2Y + cosA * zDir.Y(),
                        som2Z + cosA * zDir.Z());
        vuu = new Vector3D(-som1X, -som1Y, -som1Z);
        vvv = new Vector3D(0.0, 0.0, 0.0);
        vuv = new Vector3D(-r2 * xDir.X() + r1 * yDir.X(),
                         -r2 * xDir.Y() + r1 * yDir.Y(),
                         -r2 * xDir.Z() + r1 * yDir.Z());
        vuuu = new Vector3D(dif1X, dif1Y, dif1Z);
        vvvv = new Vector3D(0.0, 0.0, 0.0);
        vuvv = new Vector3D(0.0, 0.0, 0.0);
        vuuv = new Vector3D(-som2X, -som2Y, -som2Z);
    }

    public static void CylinderD3(double u, double v, in CoordinateSystem pos, double radius,
                                  out Point3D p, out Vector3D vu, out Vector3D vv,
                                  out Vector3D vuu, out Vector3D vvv, out Vector3D vuv,
                                  out Vector3D vuuu, out Vector3D vvvv, out Vector3D vuuv, out Vector3D vuvv)
    {
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double a1 = radius * Math.Cos(u);
        double a2 = radius * Math.Sin(u);
        double som1X = a1 * xDir.X() + a2 * yDir.X();
        double som1Y = a1 * xDir.Y() + a2 * yDir.Y();
        double som1Z = a1 * xDir.Z() + a2 * yDir.Z();
        double dif1X = a2 * xDir.X() - a1 * yDir.X();
        double dif1Y = a2 * xDir.Y() - a1 * yDir.Y();
        double dif1Z = a2 * xDir.Z() - a1 * yDir.Z();
        p = new Point3D(som1X + v * zDir.X() + pLoc.X(),
                       som1Y + v * zDir.Y() + pLoc.Y(),
                       som1Z + v * zDir.Z() + pLoc.Z());
        vu = new Vector3D(-dif1X, -dif1Y, -dif1Z);
        vv = new Vector3D(zDir.X(), zDir.Y(), zDir.Z());
        vuu = new Vector3D(-som1X, -som1Y, -som1Z);
        vvv = new Vector3D(0.0, 0.0, 0.0);
        vuv = new Vector3D(0.0, 0.0, 0.0);
        vuuu = new Vector3D(dif1X, dif1Y, dif1Z);
        vvvv = new Vector3D(0.0, 0.0, 0.0);
        vuvv = new Vector3D(0.0, 0.0, 0.0);
        vuuv = new Vector3D(0.0, 0.0, 0.0);
    }

    public static void SphereD3(double u, double v, in CoordinateSystem pos, double radius,
                                out Point3D p, out Vector3D vu, out Vector3D vv,
                                out Vector3D vuu, out Vector3D vvv, out Vector3D vuv,
                                out Vector3D vuuu, out Vector3D vvvv, out Vector3D vuuv, out Vector3D vuvv)
    {
        // Vxy = CosU * XDirection + SinU * YDirection
        // DVxy = -SinU * XDirection + CosU * YDirection
        // P(U,V) = Location +  R * CosV * Vxy  +   R * SinV * Direction
        // Vu = R * CosV * DVxy
        // Vuu = - R * CosV * Vxy
        // Vuuu = - Vu
        // Vv = -R * SinV * Vxy + R * CosV * Direction
        // Vvv = -R * CosV * Vxy - R * SinV * Direction
        // Vvvv = -Vv
        // Vuv = - R * SinV * DVxy
        // Vuuv = R * SinV * Vxy
        // Vuvv = - R * CosV * DVxy = Vuuu = -Vu
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double r1 = radius * Math.Cos(v);
        double r2 = radius * Math.Sin(v);
        double a1 = r1 * cosU;
        double a2 = r1 * sinU;
        double a3 = r2 * cosU;
        double a4 = r2 * sinU;
        double som1X = a1 * xDir.X() + a2 * yDir.X();
        double som1Y = a1 * xDir.Y() + a2 * yDir.Y();
        double som1Z = a1 * xDir.Z() + a2 * yDir.Z();
        double som3X = a3 * xDir.X() + a4 * yDir.X();
        double som3Y = a3 * xDir.Y() + a4 * yDir.Y();
        double som3Z = a3 * xDir.Z() + a4 * yDir.Z();
        double dif1X = a2 * xDir.X() - a1 * yDir.X();
        double dif1Y = a2 * xDir.Y() - a1 * yDir.Y();
        double dif1Z = a2 * xDir.Z() - a1 * yDir.Z();
        double r1ZX = r1 * zDir.X();
        double r1ZY = r1 * zDir.Y();
        double r1ZZ = r1 * zDir.Z();
        double r2ZX = r2 * zDir.X();
        double r2ZY = r2 * zDir.Y();
        double r2ZZ = r2 * zDir.Z();
        p = new Point3D(som1X + r2ZX + pLoc.X(),
                       som1Y + r2ZY + pLoc.Y(),
                       som1Z + r2ZZ + pLoc.Z());
        vu = new Vector3D(-dif1X, -dif1Y, -dif1Z);
        vv = new Vector3D(-som3X + r1ZX, -som3Y + r1ZY, -som3Z + r1ZZ);
        vuu = new Vector3D(-som1X, -som1Y, -som1Z);
        vvv = new Vector3D(-som1X - r2ZX, -som1Y - r2ZY, -som1Z - r2ZZ);
        vuv = new Vector3D(a4 * xDir.X() - a3 * yDir.X(),
                         a4 * xDir.Y() - a3 * yDir.Y(),
                         a4 * xDir.Z() - a3 * yDir.Z());
        vuuu = new Vector3D(dif1X, dif1Y, dif1Z);
        vvvv = new Vector3D(som3X - r1ZX, som3Y - r1ZY, som3Z - r1ZZ);
        vuvv = new Vector3D(dif1X, dif1Y, dif1Z);
        vuuv = new Vector3D(som3X, som3Y, som3Z);
    }

    public static void TorusD3(double u, double v, in CoordinateSystem pos, double majorRadius, double minorRadius,
                               out Point3D p, out Vector3D vu, out Vector3D vv,
                               out Vector3D vuu, out Vector3D vvv, out Vector3D vuv,
                               out Vector3D vuuu, out Vector3D vvvv, out Vector3D vuuv, out Vector3D vuvv)
    {
        // Vvvv = - Vv
        // Vuuu = -Vu
        // Vuvv = MinorRadius * cos(V) * (sin(U)*XDirection - cos(U)*YDirection)
        // Vuuv = MinorRadius * sin(V) * (cos(U)*XDirection + sin(U)*YDirection)
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double r1 = minorRadius * Math.Cos(v);
        double r2 = minorRadius * Math.Sin(v);
        double r = majorRadius + r1;
        double a1 = r * cosU;
        double a2 = r * sinU;
        double a3 = r2 * cosU;
        double a4 = r2 * sinU;
        double a5 = r1 * cosU;
        double a6 = r1 * sinU;
        // OCC620: components below the rounding noise are zeroed
        double eps = 10.0 * (minorRadius + majorRadius) * RealEpsilon();
        if (Math.Abs(a1) <= eps)
        {
            a1 = 0.0;
        }

        if (Math.Abs(a2) <= eps)
        {
            a2 = 0.0;
        }

        if (Math.Abs(a3) <= eps)
        {
            a3 = 0.0;
        }

        if (Math.Abs(a4) <= eps)
        {
            a4 = 0.0;
        }

        if (Math.Abs(a5) <= eps)
        {
            a5 = 0.0;
        }

        if (Math.Abs(a6) <= eps)
        {
            a6 = 0.0;
        }

        double som1X = a1 * xDir.X() + a2 * yDir.X();
        double som1Y = a1 * xDir.Y() + a2 * yDir.Y();
        double som1Z = a1 * xDir.Z() + a2 * yDir.Z();
        double som3X = a3 * xDir.X() + a4 * yDir.X();
        double som3Y = a3 * xDir.Y() + a4 * yDir.Y();
        double som3Z = a3 * xDir.Z() + a4 * yDir.Z();
        double dif1X = a2 * xDir.X() - a1 * yDir.X();
        double dif1Y = a2 * xDir.Y() - a1 * yDir.Y();
        double dif1Z = a2 * xDir.Z() - a1 * yDir.Z();
        double r1ZX = r1 * zDir.X();
        double r1ZY = r1 * zDir.Y();
        double r1ZZ = r1 * zDir.Z();
        double r2ZX = r2 * zDir.X();
        double r2ZY = r2 * zDir.Y();
        double r2ZZ = r2 * zDir.Z();
        p = new Point3D(som1X + r2ZX + pLoc.X(),
                       som1Y + r2ZY + pLoc.Y(),
                       som1Z + r2ZZ + pLoc.Z());
        vu = new Vector3D(-dif1X, -dif1Y, -dif1Z);
        vv = new Vector3D(-som3X + r1ZX, -som3Y + r1ZY, -som3Z + r1ZZ);
        vuu = new Vector3D(-som1X, -som1Y, -som1Z);
        vvv = new Vector3D(-a5 * xDir.X() - a6 * yDir.X() - r2ZX,
                         -a5 * xDir.Y() - a6 * yDir.Y() - r2ZY,
                         -a5 * xDir.Z() - a6 * yDir.Z() - r2ZZ);
        vuv = new Vector3D(a4 * xDir.X() - a3 * yDir.X(),
                         a4 * xDir.Y() - a3 * yDir.Y(),
                         a4 * xDir.Z() - a3 * yDir.Z());
        vuuu = new Vector3D(dif1X, dif1Y, dif1Z);
        vvvv = new Vector3D(som3X - r1ZX, som3Y - r1ZY, som3Z - r1ZZ);
        vuuv = new Vector3D(som3X, som3Y, som3Z);
        vuvv = new Vector3D(a6 * xDir.X() - a5 * yDir.X(),
                          a6 * xDir.Y() - a5 * yDir.Y(),
                          a6 * xDir.Z() - a5 * yDir.Z());
    }

    /// <summary>The parameters (U, V) of a point on the plane: its coordinates in the plane's system.</summary>
    public static void PlaneParameters(in CoordinateSystem pos, in Point3D p, out double u, out double v)
    {
        Transform3D t = new Transform3D().WithTransformation(pos);
        Point3D ploc = p.Transformed(t);
        u = ploc.X;
        v = ploc.Y;
    }

    /// <summary>The parameters (U, V) of a point on the cylinder, U in [0, 2*PI].</summary>
    public static void CylinderParameters(in CoordinateSystem pos, double radius, in Point3D p, out double u, out double v)
    {
        Transform3D t = new Transform3D().WithTransformation(pos);
        Point3D ploc = p.Transformed(t);
        u = Math.Atan2(ploc.Y, ploc.X);
        NormalizeAngle(ref u);
        v = ploc.Z;
    }

    /// <summary>
    /// The parameters (U, V) of a point on the cone, U in [0, 2*PI]. For a point off the cone, V is that of its
    /// projection on the generating line of parameter U.
    /// </summary>
    public static void ConeParameters(in CoordinateSystem pos, double radius, double sAngle, in Point3D p,
                                      out double u, out double v)
    {
        Transform3D t = new Transform3D().WithTransformation(pos);
        Point3D ploc = p.Transformed(t);

        // Check if point is at the apex
        if (Math.Abs(ploc.X) < Constants.Resolution() && Math.Abs(ploc.Y) < Constants.Resolution())
        {
            u = 0.0;
        }
        else if (-radius > ploc.Z * Math.Tan(sAngle))
        {
            // the point is at the wrong side of the apex
            u = Math.Atan2(-ploc.Y, -ploc.X);
        }
        else
        {
            u = Math.Atan2(ploc.Y, ploc.X);
        }

        NormalizeAngle(ref u);

        // Evaluate V as follows :
        // P0 = Cone.Value(U,0)
        // P1 = Cone.Value(U,1)
        // V = P0 P1 . P0 Ploc
        // After simplification obtain:
        // V = sin(Sang) * ( x cosU + y SinU - R) + z * cos(Sang)
        // Method that permits to find V of the projected point if the point
        // is not actually on the cone.
        v = Math.Sin(sAngle) * (ploc.X * Math.Cos(u) + ploc.Y * Math.Sin(u) - radius) + Math.Cos(sAngle) * ploc.Z;
    }

    /// <summary>The parameters (U, V) of a point on the sphere, U in [0, 2*PI] and V in [-PI/2, PI/2].</summary>
    public static void SphereParameters(in CoordinateSystem pos, double radius, in Point3D p, out double u, out double v)
    {
        Transform3D t = new Transform3D().WithTransformation(pos);
        Point3D ploc = p.Transformed(t);
        ploc.Coord(out double x, out double y, out double z);
        double l = Math.Sqrt(x * x + y * y);
        if (l < Constants.Resolution())
        {
            // point on axis Z of the sphere
            if (z > 0.0)
            {
                v = M_PI_2; // PI * 0.5
            }
            else
            {
                v = -M_PI_2; // PI * 0.5
            }

            u = 0.0;
        }
        else
        {
            v = Math.Atan(z / l);
            u = Math.Atan2(y, x);
            NormalizeAngle(ref u);
        }
    }

    /// <summary>The parameters (U, V) of a point on the torus, both in [0, 2*PI].</summary>
    public static void TorusParameters(in CoordinateSystem pos, double majorRadius, double minorRadius, in Point3D p,
                                       out double u, out double v)
    {
        Transform3D tref = new Transform3D().WithTransformation(pos);
        Point3D ploc = p.Transformed(tref);
        ploc.Coord(out double x, out double y, out double z);

        // all that to process case of  Major < Minor.
        u = Math.Atan2(y, x);
        if (majorRadius < minorRadius)
        {
            double cosu0 = Math.Cos(u);
            double sinu0 = Math.Sin(u);
            double z2 = z * z;
            double minR2 = minorRadius * minorRadius;
            double rCosU = majorRadius * cosu0;
            double rSinU = majorRadius * sinu0;
            double xm = x - rCosU;
            double ym = y - rSinU;
            double xp = x + rCosU;
            double yp = y + rSinU;
            double d1 = xm * xm + ym * ym + z2 - minR2;
            double d2 = xp * xp + yp * yp + z2 - minR2;
            double ad1 = Math.Abs(d1);
            double ad2 = Math.Abs(d2);
            if (ad2 < ad1)
            {
                u += M_PI;
            }
        }

        NormalizeAngle(ref u);
        double cosu = Math.Cos(u);
        double sinu = Math.Sin(u);
        Direction3D dx = new Direction3D(cosu, sinu, 0.0);
        Xyz dPV = new Xyz(x - majorRadius * cosu, y - majorRadius * sinu, z);
        double mag = dPV.Modulus();
        if (mag <= Constants.Resolution())
        {
            v = 0.0;
        }
        else
        {
            Direction3D dP = new Direction3D(dPV);
            v = dx.AngleWithRef(dP, dx ^ Constants.DZ());
        }

        NormalizeAngle(ref v);
    }

    /// <summary>The U isoparametric line of the plane.</summary>
    public static Line3D PlaneUIso(in CoordinateSystem pos, double u)
    {
        Line3D l = new Line3D(pos.Location, pos.YDirection);
        Vector3D ve = new Vector3D(pos.XDirection);
        ve *= u;
        l = l.Translated(ve);
        return l;
    }

    /// <summary>The U isoparametric line of the cylinder.</summary>
    public static Line3D CylinderUIso(in CoordinateSystem pos, double radius, double u)
    {
        CylinderD1(u, 0.0, pos, radius, out Point3D p, out Vector3D du, out Vector3D dv);
        Line3D l = new Line3D(p, new Direction3D(dv));
        return l;
    }

    /// <summary>The U isoparametric line of the cone.</summary>
    public static Line3D ConeUIso(in CoordinateSystem pos, double radius, double sAngle, double u)
    {
        ConeD1(u, 0, pos, radius, sAngle, out Point3D p, out Vector3D du, out Vector3D dv);
        Line3D l = new Line3D(p, new Direction3D(dv));
        return l;
    }

    /// <summary>The U isoparametric circle of the sphere (a whole meridian circle, not trimmed).</summary>
    public static Circle3D SphereUIso(in CoordinateSystem pos, double radius, double u)
    {
        Vector3D dx = new Vector3D(pos.XDirection);
        Vector3D dy = new Vector3D(pos.YDirection);
        Direction3D dz = pos.Direction;
        Direction3D cx = new Direction3D(Math.Cos(u) * dx + Math.Sin(u) * dy);
        Frame3D axes = new Frame3D(pos.Location, cx.Crossed(dz), cx);
        Circle3D circ = new Circle3D(axes, radius);
        return circ;
    }

    /// <summary>The U isoparametric circle of the torus.</summary>
    public static Circle3D TorusUIso(in CoordinateSystem pos, double majorRadius, double minorRadius, double u)
    {
        Vector3D dx = new Vector3D(pos.XDirection);
        Vector3D dy = new Vector3D(pos.YDirection);
        Direction3D dz = pos.Direction;
        Direction3D cx = new Direction3D(Math.Cos(u) * dx + Math.Sin(u) * dy);
        Frame3D axes = new Frame3D(pos.Location, cx.Crossed(dz), cx);
        Vector3D ve = new Vector3D(cx);
        ve *= majorRadius;
        axes = axes.Translated(ve);
        Circle3D circ = new Circle3D(axes, minorRadius);
        return circ;
    }

    /// <summary>The V isoparametric line of the plane.</summary>
    public static Line3D PlaneVIso(in CoordinateSystem pos, double v)
    {
        Line3D l = new Line3D(pos.Location, pos.XDirection);
        Vector3D ve = new Vector3D(pos.YDirection);
        ve *= v;
        l = l.Translated(ve);
        return l;
    }

    /// <summary>The V isoparametric circle of the cylinder.</summary>
    public static Circle3D CylinderVIso(in CoordinateSystem pos, double radius, double v)
    {
        Frame3D axes = pos.Frame;
        Vector3D ve = new Vector3D(pos.Direction);
        ve = ve.Multiplied(v);
        axes = axes.Translated(ve);
        Circle3D c = new Circle3D(axes, radius);
        return c;
    }

    /// <summary>The V isoparametric circle of the cone.</summary>
    public static Circle3D ConeVIso(in CoordinateSystem pos, double radius, double sAngle, double v)
    {
        CoordinateSystem axes = pos;
        Vector3D ve = new Vector3D(pos.Direction);
        ve = ve.Multiplied(v * Math.Cos(sAngle));
        axes = axes.Translated(ve);
        double r = radius + v * Math.Sin(sAngle);
        if (r < 0)
        {
            axes = axes.XReversed();
            axes = axes.YReversed();
            r = -r;
        }

        Circle3D c = new Circle3D(axes.Frame, r);
        return c;
    }

    /// <summary>The V isoparametric circle (a parallel) of the sphere.</summary>
    public static Circle3D SphereVIso(in CoordinateSystem pos, double Radius, double v)
    {
        Frame3D axes = pos.Frame;
        Vector3D ve = new Vector3D(pos.Direction);
        ve = ve.Multiplied(Radius * Math.Sin(v));
        axes = axes.Translated(ve);
        double radius = Radius * Math.Cos(v);
        // #23170: if V is even slightly (e.g. by double epsilon) greater than PI/2,
        // radius will become negative and constructor of gp_Circ will raise exception.
        // Lets try to create correct isoline even on analytical continuation for |V| > PI/2...
        if (radius < 0.0)
        {
            axes = axes.WithDirection(-axes.Direction);
            radius = -radius;
        }

        Circle3D circ = new Circle3D(axes, radius);
        return circ;
    }

    /// <summary>The V isoparametric circle of the torus.</summary>
    public static Circle3D TorusVIso(in CoordinateSystem pos, double majorRadius, double minorRadius, double v)
    {
        CoordinateSystem axes = new CoordinateSystem(pos.Frame);
        Vector3D ve = new Vector3D(pos.Direction);
        ve = ve.Multiplied(minorRadius * Math.Sin(v));
        axes = axes.Translated(ve);
        double r = majorRadius + minorRadius * Math.Cos(v);
        if (r < 0)
        {
            axes = axes.XReversed();
            axes = axes.YReversed();
            r = -r;
        }

        Circle3D circ = new Circle3D(axes.Frame, r);
        return circ;
    }
}
