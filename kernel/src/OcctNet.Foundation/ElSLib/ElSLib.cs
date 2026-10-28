// Port of: src/FoundationClasses/TKMath/ElSLib/ElSLib.hxx
// Port of: src/FoundationClasses/TKMath/ElSLib/ElSLib.lxx
// Port of: src/FoundationClasses/TKMath/ElSLib/ElSLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
public static class ElSLib
{
    // ElSLib.lxx: the gp surface overloads

    /// <summary>The point of parameters (U, V) on an elementary surface.</summary>
    public static gp_Pnt Value(double U, double V, in gp_Pln Pl) => PlaneValue(U, V, Pl.Position());

    public static gp_Pnt Value(double U, double V, in gp_Cone C)
        => ConeValue(U, V, C.Position(), C.RefRadius(), C.SemiAngle());

    public static gp_Pnt Value(double U, double V, in gp_Cylinder C) => CylinderValue(U, V, C.Position(), C.Radius());

    public static gp_Pnt Value(double U, double V, in gp_Sphere S) => SphereValue(U, V, S.Position(), S.Radius());

    public static gp_Pnt Value(double U, double V, in gp_Torus T)
        => TorusValue(U, V, T.Position(), T.MajorRadius(), T.MinorRadius());

    /// <summary>The derivative of order Nu in U and Nv in V at the point of parameters (U, V).</summary>
    public static gp_Vec DN(double U, double V, in gp_Pln Pl, int Nu, int Nv) => PlaneDN(U, V, Pl.Position(), Nu, Nv);

    public static gp_Vec DN(double U, double V, in gp_Cone C, int Nu, int Nv)
        => ConeDN(U, V, C.Position(), C.RefRadius(), C.SemiAngle(), Nu, Nv);

    public static gp_Vec DN(double U, double V, in gp_Cylinder C, int Nu, int Nv)
        => CylinderDN(U, V, C.Position(), C.Radius(), Nu, Nv);

    public static gp_Vec DN(double U, double V, in gp_Sphere S, int Nu, int Nv)
        => SphereDN(U, V, S.Position(), S.Radius(), Nu, Nv);

    public static gp_Vec DN(double U, double V, in gp_Torus T, int Nu, int Nv)
        => TorusDN(U, V, T.Position(), T.MajorRadius(), T.MinorRadius(), Nu, Nv);

    /// <summary>The point P of parameters (U, V).</summary>
    public static void D0(double U, double V, in gp_Pln Pl, out gp_Pnt P) => PlaneD0(U, V, Pl.Position(), out P);

    public static void D0(double U, double V, in gp_Cone C, out gp_Pnt P)
        => ConeD0(U, V, C.Position(), C.RefRadius(), C.SemiAngle(), out P);

    public static void D0(double U, double V, in gp_Cylinder C, out gp_Pnt P)
        => CylinderD0(U, V, C.Position(), C.Radius(), out P);

    public static void D0(double U, double V, in gp_Sphere S, out gp_Pnt P)
        => SphereD0(U, V, S.Position(), S.Radius(), out P);

    public static void D0(double U, double V, in gp_Torus T, out gp_Pnt P)
        => TorusD0(U, V, T.Position(), T.MajorRadius(), T.MinorRadius(), out P);

    /// <summary>The point P of parameters (U, V) and the first derivatives Vu and Vv.</summary>
    public static void D1(double U, double V, in gp_Pln Pl, out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv)
        => PlaneD1(U, V, Pl.Position(), out P, out Vu, out Vv);

    public static void D1(double U, double V, in gp_Cone C, out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv)
        => ConeD1(U, V, C.Position(), C.RefRadius(), C.SemiAngle(), out P, out Vu, out Vv);

    public static void D1(double U, double V, in gp_Cylinder C, out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv)
        => CylinderD1(U, V, C.Position(), C.Radius(), out P, out Vu, out Vv);

    public static void D1(double U, double V, in gp_Sphere S, out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv)
        => SphereD1(U, V, S.Position(), S.Radius(), out P, out Vu, out Vv);

    public static void D1(double U, double V, in gp_Torus T, out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv)
        => TorusD1(U, V, T.Position(), T.MajorRadius(), T.MinorRadius(), out P, out Vu, out Vv);

    /// <summary>The point, the first derivatives and the second derivatives Vuu, Vvv and Vuv.</summary>
    public static void D2(double U, double V, in gp_Cone C, out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                          out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv)
        => ConeD2(U, V, C.Position(), C.RefRadius(), C.SemiAngle(), out P, out Vu, out Vv, out Vuu, out Vvv, out Vuv);

    public static void D2(double U, double V, in gp_Cylinder C, out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                          out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv)
        => CylinderD2(U, V, C.Position(), C.Radius(), out P, out Vu, out Vv, out Vuu, out Vvv, out Vuv);

    public static void D2(double U, double V, in gp_Sphere S, out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                          out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv)
        => SphereD2(U, V, S.Position(), S.Radius(), out P, out Vu, out Vv, out Vuu, out Vvv, out Vuv);

    public static void D2(double U, double V, in gp_Torus T, out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                          out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv)
        => TorusD2(U, V, T.Position(), T.MajorRadius(), T.MinorRadius(), out P, out Vu, out Vv, out Vuu, out Vvv, out Vuv);

    /// <summary>
    /// The point, the first and second derivatives, and the third derivatives Vuuu, Vvvv, Vuuv and Vuvv.
    /// </summary>
    public static void D3(double U, double V, in gp_Cone C, out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                          out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv,
                          out gp_Vec Vuuu, out gp_Vec Vvvv, out gp_Vec Vuuv, out gp_Vec Vuvv)
        => ConeD3(U, V, C.Position(), C.RefRadius(), C.SemiAngle(), out P, out Vu, out Vv, out Vuu, out Vvv, out Vuv,
                  out Vuuu, out Vvvv, out Vuuv, out Vuvv);

    public static void D3(double U, double V, in gp_Cylinder C, out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                          out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv,
                          out gp_Vec Vuuu, out gp_Vec Vvvv, out gp_Vec Vuuv, out gp_Vec Vuvv)
        => CylinderD3(U, V, C.Position(), C.Radius(), out P, out Vu, out Vv, out Vuu, out Vvv, out Vuv,
                      out Vuuu, out Vvvv, out Vuuv, out Vuvv);

    public static void D3(double U, double V, in gp_Sphere S, out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                          out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv,
                          out gp_Vec Vuuu, out gp_Vec Vvvv, out gp_Vec Vuuv, out gp_Vec Vuvv)
        => SphereD3(U, V, S.Position(), S.Radius(), out P, out Vu, out Vv, out Vuu, out Vvv, out Vuv,
                    out Vuuu, out Vvvv, out Vuuv, out Vuvv);

    public static void D3(double U, double V, in gp_Torus T, out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                          out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv,
                          out gp_Vec Vuuu, out gp_Vec Vvvv, out gp_Vec Vuuv, out gp_Vec Vuvv)
        => TorusD3(U, V, T.Position(), T.MajorRadius(), T.MinorRadius(), out P, out Vu, out Vv, out Vuu, out Vvv, out Vuv,
                   out Vuuu, out Vvvv, out Vuuv, out Vuvv);

    /// <summary>The parameters (U, V) of a point on an elementary surface.</summary>
    public static void Parameters(in gp_Pln Pl, in gp_Pnt P, out double U, out double V)
        => PlaneParameters(Pl.Position(), P, out U, out V);

    public static void Parameters(in gp_Cylinder C, in gp_Pnt P, out double U, out double V)
        => CylinderParameters(C.Position(), C.Radius(), P, out U, out V);

    public static void Parameters(in gp_Cone C, in gp_Pnt P, out double U, out double V)
        => ConeParameters(C.Position(), C.RefRadius(), C.SemiAngle(), P, out U, out V);

    public static void Parameters(in gp_Sphere S, in gp_Pnt P, out double U, out double V)
        => SphereParameters(S.Position(), S.Radius(), P, out U, out V);

    public static void Parameters(in gp_Torus T, in gp_Pnt P, out double U, out double V)
        => TorusParameters(T.Position(), T.MajorRadius(), T.MinorRadius(), P, out U, out V);

    // ElSLib.cxx

    private const double PIPI = M_PI + M_PI;

    // Threshold for angle normalization to avoid discontinuity near zero
    private static readonly double NEGATIVE_RESOLUTION = -Precision.Computational();

    // Normalize angle to [0, 2*PI] range, with special handling
    // for values very close to zero to avoid discontinuity.
    // Preserves values at exactly 2*PI for proper seam handling.
    private static void normalizeAngle(ref double theAngle)
    {
        while (theAngle < NEGATIVE_RESOLUTION)
        {
            theAngle += PIPI;
        }

        // Only normalize angles strictly greater than 2*PI (with small tolerance)
        // to preserve the closing seam value of exactly 2*PI
        while (theAngle > PIPI * (1.0 + gp.Resolution()))
        {
            theAngle -= PIPI;
        }

        if (theAngle < 0.0)
        {
            theAngle = 0.0;
        }
    }

    public static gp_Pnt PlaneValue(double U, double V, in gp_Ax3 Pos)
    {
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        return new gp_Pnt(U * XDir.X() + V * YDir.X() + PLoc.X(),
                          U * XDir.Y() + V * YDir.Y() + PLoc.Y(),
                          U * XDir.Z() + V * YDir.Z() + PLoc.Z());
    }

    public static gp_Pnt ConeValue(double U, double V, in gp_Ax3 Pos, double Radius, double SAngle)
    {
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double R = Radius + V * Math.Sin(SAngle);
        double A3 = V * Math.Cos(SAngle);
        double A1 = R * Math.Cos(U);
        double A2 = R * Math.Sin(U);
        return new gp_Pnt(A1 * XDir.X() + A2 * YDir.X() + A3 * ZDir.X() + PLoc.X(),
                          A1 * XDir.Y() + A2 * YDir.Y() + A3 * ZDir.Y() + PLoc.Y(),
                          A1 * XDir.Z() + A2 * YDir.Z() + A3 * ZDir.Z() + PLoc.Z());
    }

    public static gp_Pnt CylinderValue(double U, double V, in gp_Ax3 Pos, double Radius)
    {
        // M(u,v) = C + Radius * ( Xdir * cos(u) + Ydir * sin(u)) + V * Zdir
        // where C is the location point of the Axis2placement
        // Xdir, Ydir ,Zdir are the directions of the local coordinates system
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double A1 = Radius * Math.Cos(U);
        double A2 = Radius * Math.Sin(U);
        return new gp_Pnt(A1 * XDir.X() + A2 * YDir.X() + V * ZDir.X() + PLoc.X(),
                          A1 * XDir.Y() + A2 * YDir.Y() + V * ZDir.Y() + PLoc.Y(),
                          A1 * XDir.Z() + A2 * YDir.Z() + V * ZDir.Z() + PLoc.Z());
    }

    public static gp_Pnt SphereValue(double U, double V, in gp_Ax3 Pos, double Radius)
    {
        // M(U,V) = Location +
        //          R * CosV (CosU * XDirection + SinU * YDirection) +
        //          R * SinV * Direction
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double R = Radius * Math.Cos(V);
        double A3 = Radius * Math.Sin(V);
        double A1 = R * Math.Cos(U);
        double A2 = R * Math.Sin(U);
        return new gp_Pnt(A1 * XDir.X() + A2 * YDir.X() + A3 * ZDir.X() + PLoc.X(),
                          A1 * XDir.Y() + A2 * YDir.Y() + A3 * ZDir.Y() + PLoc.Y(),
                          A1 * XDir.Z() + A2 * YDir.Z() + A3 * ZDir.Z() + PLoc.Z());
    }

    public static gp_Pnt TorusValue(double U, double V, in gp_Ax3 Pos, double MajorRadius, double MinorRadius)
    {
        // M(U,V) =
        //   Location +
        //   (MajRadius+MinRadius*cos(V)) * (cos(U)*XDirection + sin(U)*YDirection) +
        //   MinorRadius * sin(V) * Direction
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double R = MajorRadius + MinorRadius * Math.Cos(V);
        double A3 = MinorRadius * Math.Sin(V);
        double A1 = R * Math.Cos(U);
        double A2 = R * Math.Sin(U);
        // OCC620: components below the rounding noise are zeroed
        double eps = 10.0 * (MinorRadius + MajorRadius) * RealEpsilon();
        if (Math.Abs(A1) <= eps)
        {
            A1 = 0.0;
        }

        if (Math.Abs(A2) <= eps)
        {
            A2 = 0.0;
        }

        if (Math.Abs(A3) <= eps)
        {
            A3 = 0.0;
        }

        return new gp_Pnt(A1 * XDir.X() + A2 * YDir.X() + A3 * ZDir.X() + PLoc.X(),
                          A1 * XDir.Y() + A2 * YDir.Y() + A3 * ZDir.Y() + PLoc.Y(),
                          A1 * XDir.Z() + A2 * YDir.Z() + A3 * ZDir.Z() + PLoc.Z());
    }

    public static gp_Vec PlaneDN(double U, double V, in gp_Ax3 Pos, int Nu, int Nv)
    {
        if (Nu == 0 && Nv == 1)
        {
            return new gp_Vec(Pos.YDirection());
        }
        else if (Nu == 1 && Nv == 0)
        {
            return new gp_Vec(Pos.XDirection());
        }

        return new gp_Vec(0.0, 0.0, 0.0);
    }

    public static gp_Vec ConeDN(double U, double V, in gp_Ax3 Pos, double Radius, double SAngle, int Nu, int Nv)
    {
        gp_XYZ Xdir = Pos.XDirection().XYZ();
        gp_XYZ Ydir = Pos.YDirection().XYZ();
        double Um = U + Nu * M_PI_2; // M_PI * 0.5
        Xdir.Multiply(Math.Cos(Um));
        Ydir.Multiply(Math.Sin(Um));
        Xdir.Add(Ydir);
        if (Nv == 0)
        {
            Xdir.Multiply(Radius + V * Math.Sin(SAngle));
            if (Nu == 0)
            {
                Xdir.Add(Pos.Location().XYZ());
            }

            return new gp_Vec(Xdir);
        }
        else if (Nv == 1)
        {
            Xdir.Multiply(Math.Sin(SAngle));
            if (Nu == 0)
            {
                Xdir.Add(Pos.Direction().XYZ() * Math.Cos(SAngle));
            }

            return new gp_Vec(Xdir);
        }

        return new gp_Vec(0.0, 0.0, 0.0);
    }

    public static gp_Vec CylinderDN(double U, double V, in gp_Ax3 Pos, double Radius, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            return new gp_Vec();
        }

        if (Nv == 0)
        {
            double RCosU = Radius * Math.Cos(U);
            double RSinU = Radius * Math.Sin(U);
            gp_XYZ Xdir = Pos.XDirection().XYZ();
            gp_XYZ Ydir = Pos.YDirection().XYZ();
            if ((Nu + 6) % 4 == 0)
            {
                Xdir.Multiply(-RCosU);
                Ydir.Multiply(-RSinU);
            }
            else if ((Nu + 5) % 4 == 0)
            {
                Xdir.Multiply(RSinU);
                Ydir.Multiply(-RCosU);
            }
            else if ((Nu + 3) % 4 == 0)
            {
                Xdir.Multiply(-RSinU);
                Ydir.Multiply(RCosU);
            }
            else if (Nu % 4 == 0)
            {
                Xdir.Multiply(RCosU);
                Ydir.Multiply(RSinU);
            }

            Xdir.Add(Ydir);
            return new gp_Vec(Xdir);
        }
        else if (Nv == 1 && Nu == 0)
        {
            return new gp_Vec(Pos.Direction());
        }
        else
        {
            return new gp_Vec(0.0, 0.0, 0.0);
        }
    }

    public static gp_Vec SphereDN(double U, double V, in gp_Ax3 Pos, double Radius, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            return new gp_Vec();
        }

        double CosU = Math.Cos(U);
        double SinU = Math.Sin(U);
        double RCosV = Radius * Math.Cos(V);
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        double A1, A2, A3, X, Y, Z;
        if (Nu == 0)
        {
            double RSinV = Radius * Math.Sin(V);
            if (IsOdd(Nv))
            {
                A1 = -RSinV * CosU;
                A2 = -RSinV * SinU;
                A3 = RCosV;
            }
            else
            {
                A1 = -RCosV * CosU;
                A2 = -RCosV * SinU;
                A3 = -RSinV;
            }

            X = A1 * XDir.X() + A2 * YDir.X() + A3 * ZDir.X();
            Y = A1 * XDir.Y() + A2 * YDir.Y() + A3 * ZDir.Y();
            Z = A1 * XDir.Z() + A2 * YDir.Z() + A3 * ZDir.Z();
            if ((Nv + 2) % 4 != 0 && (Nv + 3) % 4 != 0)
            {
                X = -X;
                Y = -Y;
                Z = -Z;
            }
        }
        else if (Nv == 0)
        {
            if (IsOdd(Nu))
            {
                A1 = -RCosV * SinU;
                A2 = RCosV * CosU;
            }
            else
            {
                A1 = RCosV * CosU;
                A2 = RCosV * SinU;
            }

            X = A1 * XDir.X() + A2 * YDir.X();
            Y = A1 * XDir.Y() + A2 * YDir.Y();
            Z = A1 * XDir.Z() + A2 * YDir.Z();
            if ((Nu + 2) % 4 == 0 || (Nu + 1) % 4 == 0)
            {
                X = -X;
                Y = -Y;
                Z = -Z;
            }
        }
        else
        {
            double RSinV = Radius * Math.Sin(V);
            if (IsOdd(Nu))
            {
                A1 = -SinU;
                A2 = CosU;
            }
            else
            {
                A1 = -CosU;
                A2 = -SinU;
            }

            if (IsOdd(Nv))
            {
                A3 = -RSinV;
            }
            else
            {
                A3 = -RCosV;
            }

            X = (A1 * XDir.X() + A2 * YDir.X()) * A3;
            Y = (A1 * XDir.Y() + A2 * YDir.Y()) * A3;
            Z = (A1 * XDir.Z() + A2 * YDir.Z()) * A3;
            if (((Nu + 2) % 4 != 0 && (Nu + 3) % 4 != 0 && ((Nv + 2) % 4 == 0 || (Nv + 3) % 4 == 0))
                || (((Nu + 2) % 4 == 0 || (Nu + 3) % 4 == 0) && (Nv + 2) % 4 != 0 && (Nv + 3) % 4 != 0))
            {
                X = -X;
                Y = -Y;
                Z = -Z;
            }
        }

        return new gp_Vec(X, Y, Z);
    }

    public static gp_Vec TorusDN(double U, double V, in gp_Ax3 Pos, double MajorRadius, double MinorRadius,
                                 int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            return new gp_Vec();
        }

        double CosU = Math.Cos(U);
        double SinU = Math.Sin(U);
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        double A1, A2, A3, X = 0, Y = 0, Z = 0;
        // OCC620: components below the rounding noise are zeroed
        double eps = 10.0 * (MinorRadius + MajorRadius) * RealEpsilon();
        if (Nv == 0)
        {
            double R = MajorRadius + MinorRadius * Math.Cos(V);
            if (IsOdd(Nu))
            {
                A1 = -R * SinU;
                A2 = R * CosU;
            }
            else
            {
                A1 = -R * CosU;
                A2 = -R * SinU;
            }

            if (Math.Abs(A1) <= eps)
            {
                A1 = 0.0;
            }

            if (Math.Abs(A2) <= eps)
            {
                A2 = 0.0;
            }

            X = A1 * XDir.X() + A2 * YDir.X();
            Y = A1 * XDir.Y() + A2 * YDir.Y();
            Z = A1 * XDir.Z() + A2 * YDir.Z();
            if ((Nu + 2) % 4 != 0 && (Nu + 3) % 4 != 0)
            {
                X = -X;
                Y = -Y;
                Z = -Z;
            }
        }
        else if (Nu == 0)
        {
            double RCosV = MinorRadius * Math.Cos(V);
            double RSinV = MinorRadius * Math.Sin(V);
            if (IsOdd(Nv))
            {
                A1 = -RSinV * CosU;
                A2 = -RSinV * SinU;
                A3 = RCosV;
            }
            else
            {
                A1 = -RCosV * CosU;
                A2 = -RCosV * SinU;
                A3 = -RSinV;
            }

            if (Math.Abs(A1) <= eps)
            {
                A1 = 0.0;
            }

            if (Math.Abs(A2) <= eps)
            {
                A2 = 0.0;
            }

            if (Math.Abs(A3) <= eps)
            {
                A3 = 0.0;
            }

            X = A1 * XDir.X() + A2 * YDir.X() + A3 * ZDir.X();
            Y = A1 * XDir.Y() + A2 * YDir.Y() + A3 * ZDir.Y();
            Z = A1 * XDir.Z() + A2 * YDir.Z() + A3 * ZDir.Z();
            if ((Nv + 2) % 4 != 0 && (Nv + 3) % 4 != 0)
            {
                X = -X;
                Y = -Y;
                Z = -Z;
            }
        }
        else
        {
            // OCCT: the odd/odd and even/even branches below never change sign, so the mixed derivatives with
            // (Nu + Nv) % 4 == 0 (odd/odd, e.g. Nu = 1, Nv = 3) or (Nu + Nv) % 4 == 2 (even/even, e.g. Nu = 2,
            // Nv = 4) come out reversed. The other orders, (1, 1) and (2, 2) among them, are right.
            if (IsOdd(Nu) && IsOdd(Nv))
            {
                double RSinV = MinorRadius * Math.Sin(V);
                A1 = RSinV * SinU;
                A2 = -RSinV * CosU;
                if (Math.Abs(A1) <= eps)
                {
                    A1 = 0.0;
                }

                if (Math.Abs(A2) <= eps)
                {
                    A2 = 0.0;
                }

                X = A1 * XDir.X() + A2 * YDir.X();
                Y = A1 * XDir.Y() + A2 * YDir.Y();
                Z = A1 * XDir.Z() + A2 * YDir.Z();
            }
            else if (IsEven(Nu) && IsEven(Nv))
            {
                double RCosV = MinorRadius * Math.Cos(V);
                A1 = RCosV * CosU;
                A2 = RCosV * SinU;
                if (Math.Abs(A1) <= eps)
                {
                    A1 = 0.0;
                }

                if (Math.Abs(A2) <= eps)
                {
                    A2 = 0.0;
                }

                X = A1 * XDir.X() + A2 * YDir.X();
                Y = A1 * XDir.Y() + A2 * YDir.Y();
                Z = A1 * XDir.Z() + A2 * YDir.Z();
            }
            else if (IsEven(Nv) && IsOdd(Nu))
            {
                double RCosV = MinorRadius * Math.Cos(V);
                A1 = RCosV * SinU;
                A2 = -RCosV * CosU;
                if (Math.Abs(A1) <= eps)
                {
                    A1 = 0.0;
                }

                if (Math.Abs(A2) <= eps)
                {
                    A2 = 0.0;
                }

                X = A1 * XDir.X() + A2 * YDir.X();
                Y = A1 * XDir.Y() + A2 * YDir.Y();
                Z = A1 * XDir.Z() + A2 * YDir.Z();
                if (((Nv + Nu + 3) % 4) == 0)
                {
                    X = -X;
                    Y = -Y;
                    Z = -Z;
                }
            }
            else if (IsOdd(Nv) && IsEven(Nu))
            {
                double RSinV = MinorRadius * Math.Sin(V);
                A1 = RSinV * CosU;
                A2 = RSinV * SinU;
                if (Math.Abs(A1) <= eps)
                {
                    A1 = 0.0;
                }

                if (Math.Abs(A2) <= eps)
                {
                    A2 = 0.0;
                }

                X = A1 * XDir.X() + A2 * YDir.X();
                Y = A1 * XDir.Y() + A2 * YDir.Y();
                Z = A1 * XDir.Z() + A2 * YDir.Z();
                if (((Nu + Nv + 3) % 4) == 0)
                {
                    X = -X;
                    Y = -Y;
                    Z = -Z;
                }
            }
        }

        return new gp_Vec(X, Y, Z);
    }

    // The D0..D3 functions below assign whole points and vectors where OCCT calls SetX, SetY and SetZ on its
    // output references; the coordinates, and the order they are computed in, are the same.

    public static void PlaneD0(double U, double V, in gp_Ax3 Pos, out gp_Pnt P)
    {
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        P = new gp_Pnt(U * XDir.X() + V * YDir.X() + PLoc.X(),
                       U * XDir.Y() + V * YDir.Y() + PLoc.Y(),
                       U * XDir.Z() + V * YDir.Z() + PLoc.Z());
    }

    public static void ConeD0(double U, double V, in gp_Ax3 Pos, double Radius, double SAngle, out gp_Pnt P)
    {
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double R = Radius + V * Math.Sin(SAngle);
        double A3 = V * Math.Cos(SAngle);
        double A1 = R * Math.Cos(U);
        double A2 = R * Math.Sin(U);
        P = new gp_Pnt(A1 * XDir.X() + A2 * YDir.X() + A3 * ZDir.X() + PLoc.X(),
                       A1 * XDir.Y() + A2 * YDir.Y() + A3 * ZDir.Y() + PLoc.Y(),
                       A1 * XDir.Z() + A2 * YDir.Z() + A3 * ZDir.Z() + PLoc.Z());
    }

    public static void CylinderD0(double U, double V, in gp_Ax3 Pos, double Radius, out gp_Pnt P)
    {
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double A1 = Radius * Math.Cos(U);
        double A2 = Radius * Math.Sin(U);
        P = new gp_Pnt(A1 * XDir.X() + A2 * YDir.X() + V * ZDir.X() + PLoc.X(),
                       A1 * XDir.Y() + A2 * YDir.Y() + V * ZDir.Y() + PLoc.Y(),
                       A1 * XDir.Z() + A2 * YDir.Z() + V * ZDir.Z() + PLoc.Z());
    }

    public static void SphereD0(double U, double V, in gp_Ax3 Pos, double Radius, out gp_Pnt P)
    {
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double R = Radius * Math.Cos(V);
        double A3 = Radius * Math.Sin(V);
        double A1 = R * Math.Cos(U);
        double A2 = R * Math.Sin(U);
        P = new gp_Pnt(A1 * XDir.X() + A2 * YDir.X() + A3 * ZDir.X() + PLoc.X(),
                       A1 * XDir.Y() + A2 * YDir.Y() + A3 * ZDir.Y() + PLoc.Y(),
                       A1 * XDir.Z() + A2 * YDir.Z() + A3 * ZDir.Z() + PLoc.Z());
    }

    public static void TorusD0(double U, double V, in gp_Ax3 Pos, double MajorRadius, double MinorRadius, out gp_Pnt P)
    {
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double R = MajorRadius + MinorRadius * Math.Cos(V);
        double A3 = MinorRadius * Math.Sin(V);
        double A1 = R * Math.Cos(U);
        double A2 = R * Math.Sin(U);
        // OCC620: components below the rounding noise are zeroed
        double eps = 10.0 * (MinorRadius + MajorRadius) * RealEpsilon();
        if (Math.Abs(A1) <= eps)
        {
            A1 = 0.0;
        }

        if (Math.Abs(A2) <= eps)
        {
            A2 = 0.0;
        }

        if (Math.Abs(A3) <= eps)
        {
            A3 = 0.0;
        }

        P = new gp_Pnt(A1 * XDir.X() + A2 * YDir.X() + A3 * ZDir.X() + PLoc.X(),
                       A1 * XDir.Y() + A2 * YDir.Y() + A3 * ZDir.Y() + PLoc.Y(),
                       A1 * XDir.Z() + A2 * YDir.Z() + A3 * ZDir.Z() + PLoc.Z());
    }

    public static void PlaneD1(double U, double V, in gp_Ax3 Pos, out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv)
    {
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        P = new gp_Pnt(U * XDir.X() + V * YDir.X() + PLoc.X(),
                       U * XDir.Y() + V * YDir.Y() + PLoc.Y(),
                       U * XDir.Z() + V * YDir.Z() + PLoc.Z());
        Vu = new gp_Vec(XDir.X(), XDir.Y(), XDir.Z());
        Vv = new gp_Vec(YDir.X(), YDir.Y(), YDir.Z());
    }

    public static void ConeD1(double U, double V, in gp_Ax3 Pos, double Radius, double SAngle,
                              out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv)
    {
        // Z = V * cos(SAngle)
        // M(U,V) = Location() + V * cos(SAngle) * ZDirection() +
        // (Radius + V*Sin(SAng)) * (cos(U) * XDirection() + sin(U) * YDirection())
        // D1U =
        // (Radius + V*Sin(SAng)) * (-sin(U) * XDirection() + cos(U) * YDirection())
        // D1V =
        // Direction() *cos(SAngle) + sin(SAng) * (cos(U) * XDirection() + sin(U) * YDirection())
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double CosU = Math.Cos(U);
        double SinU = Math.Sin(U);
        double CosA = Math.Cos(SAngle);
        double SinA = Math.Sin(SAngle);
        double R = Radius + V * SinA;
        double A3 = V * CosA;
        double A1 = R * CosU;
        double A2 = R * SinU;
        double R1 = SinA * CosU;
        double R2 = SinA * SinU;
        P = new gp_Pnt(A1 * XDir.X() + A2 * YDir.X() + A3 * ZDir.X() + PLoc.X(),
                       A1 * XDir.Y() + A2 * YDir.Y() + A3 * ZDir.Y() + PLoc.Y(),
                       A1 * XDir.Z() + A2 * YDir.Z() + A3 * ZDir.Z() + PLoc.Z());
        Vu = new gp_Vec(-A2 * XDir.X() + A1 * YDir.X(),
                        -A2 * XDir.Y() + A1 * YDir.Y(),
                        -A2 * XDir.Z() + A1 * YDir.Z());
        Vv = new gp_Vec(R1 * XDir.X() + R2 * YDir.X() + CosA * ZDir.X(),
                        R1 * XDir.Y() + R2 * YDir.Y() + CosA * ZDir.Y(),
                        R1 * XDir.Z() + R2 * YDir.Z() + CosA * ZDir.Z());
    }

    public static void CylinderD1(double U, double V, in gp_Ax3 Pos, double Radius,
                                  out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv)
    {
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double A1 = Radius * Math.Cos(U);
        double A2 = Radius * Math.Sin(U);
        P = new gp_Pnt(A1 * XDir.X() + A2 * YDir.X() + V * ZDir.X() + PLoc.X(),
                       A1 * XDir.Y() + A2 * YDir.Y() + V * ZDir.Y() + PLoc.Y(),
                       A1 * XDir.Z() + A2 * YDir.Z() + V * ZDir.Z() + PLoc.Z());
        Vu = new gp_Vec(-A2 * XDir.X() + A1 * YDir.X(),
                        -A2 * XDir.Y() + A1 * YDir.Y(),
                        -A2 * XDir.Z() + A1 * YDir.Z());
        Vv = new gp_Vec(ZDir.X(), ZDir.Y(), ZDir.Z());
    }

    public static void SphereD1(double U, double V, in gp_Ax3 Pos, double Radius,
                                out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv)
    {
        // Vxy = CosU * XDirection + SinU * YDirection
        // DVxy = -SinU * XDirection + CosU * YDirection
        // P(U,V) = Location +  R * CosV * Vxy  +   R * SinV * Direction
        // Vu = R * CosV * DVxy
        // Vv = -R * SinV * Vxy + R * CosV * Direction
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double CosU = Math.Cos(U);
        double SinU = Math.Sin(U);
        double R1 = Radius * Math.Cos(V);
        double R2 = Radius * Math.Sin(V);
        double A1 = R1 * CosU;
        double A2 = R1 * SinU;
        double A3 = R2 * CosU;
        double A4 = R2 * SinU;
        P = new gp_Pnt(A1 * XDir.X() + A2 * YDir.X() + R2 * ZDir.X() + PLoc.X(),
                       A1 * XDir.Y() + A2 * YDir.Y() + R2 * ZDir.Y() + PLoc.Y(),
                       A1 * XDir.Z() + A2 * YDir.Z() + R2 * ZDir.Z() + PLoc.Z());
        Vu = new gp_Vec(-A2 * XDir.X() + A1 * YDir.X(),
                        -A2 * XDir.Y() + A1 * YDir.Y(),
                        -A2 * XDir.Z() + A1 * YDir.Z());
        Vv = new gp_Vec(-A3 * XDir.X() - A4 * YDir.X() + R1 * ZDir.X(),
                        -A3 * XDir.Y() - A4 * YDir.Y() + R1 * ZDir.Y(),
                        -A3 * XDir.Z() - A4 * YDir.Z() + R1 * ZDir.Z());
    }

    public static void TorusD1(double U, double V, in gp_Ax3 Pos, double MajorRadius, double MinorRadius,
                               out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv)
    {
        // P(U,V) =
        //   Location +
        //   (MajorRadius+MinorRadius*cos(V)) * (cos(U)*XDirection + sin(U)*YDirection) +
        //   MinorRadius * sin(V) * Direction
        // Vv = -MinorRadius * sin(V) * (cos(U)*XDirection + sin(U)*YDirection) +
        //       MinorRadius * cos(V) * Direction
        // Vu = (MajorRadius+MinorRadius*cos(V)) * (-sin(U)*XDirection + cos(U)*YDirection)
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double CosU = Math.Cos(U);
        double SinU = Math.Sin(U);
        double R1 = MinorRadius * Math.Cos(V);
        double R2 = MinorRadius * Math.Sin(V);
        double R = MajorRadius + R1;
        double A1 = R * CosU;
        double A2 = R * SinU;
        double A3 = R2 * CosU;
        double A4 = R2 * SinU;
        // OCC620: components below the rounding noise are zeroed
        double eps = 10.0 * (MinorRadius + MajorRadius) * RealEpsilon();
        if (Math.Abs(A1) <= eps)
        {
            A1 = 0.0;
        }

        if (Math.Abs(A2) <= eps)
        {
            A2 = 0.0;
        }

        if (Math.Abs(A3) <= eps)
        {
            A3 = 0.0;
        }

        if (Math.Abs(A4) <= eps)
        {
            A4 = 0.0;
        }

        P = new gp_Pnt(A1 * XDir.X() + A2 * YDir.X() + R2 * ZDir.X() + PLoc.X(),
                       A1 * XDir.Y() + A2 * YDir.Y() + R2 * ZDir.Y() + PLoc.Y(),
                       A1 * XDir.Z() + A2 * YDir.Z() + R2 * ZDir.Z() + PLoc.Z());
        Vu = new gp_Vec(-A2 * XDir.X() + A1 * YDir.X(),
                        -A2 * XDir.Y() + A1 * YDir.Y(),
                        -A2 * XDir.Z() + A1 * YDir.Z());
        Vv = new gp_Vec(-A3 * XDir.X() - A4 * YDir.X() + R1 * ZDir.X(),
                        -A3 * XDir.Y() - A4 * YDir.Y() + R1 * ZDir.Y(),
                        -A3 * XDir.Z() - A4 * YDir.Z() + R1 * ZDir.Z());
    }

    public static void ConeD2(double U, double V, in gp_Ax3 Pos, double Radius, double SAngle,
                              out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                              out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv)
    {
        // D2U = (Radius + V*Sin(SAng)) * (-cos(U) * XDirection() - sin(U) * YDirection())
        // D2V = 0.0
        // DUV = sin(SAng) * (-sin(U) * XDirection() + cos(U) * YDirection())
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double CosU = Math.Cos(U);
        double SinU = Math.Sin(U);
        double CosA = Math.Cos(SAngle);
        double SinA = Math.Sin(SAngle);
        double R = Radius + V * SinA;
        double A3 = V * CosA;
        double A1 = R * CosU;
        double A2 = R * SinU;
        double R1 = SinA * CosU;
        double R2 = SinA * SinU;
        double Som1X = A1 * XDir.X() + A2 * YDir.X();
        double Som1Y = A1 * XDir.Y() + A2 * YDir.Y();
        double Som1Z = A1 * XDir.Z() + A2 * YDir.Z();
        P = new gp_Pnt(Som1X + A3 * ZDir.X() + PLoc.X(),
                       Som1Y + A3 * ZDir.Y() + PLoc.Y(),
                       Som1Z + A3 * ZDir.Z() + PLoc.Z());
        Vu = new gp_Vec(-A2 * XDir.X() + A1 * YDir.X(),
                        -A2 * XDir.Y() + A1 * YDir.Y(),
                        -A2 * XDir.Z() + A1 * YDir.Z());
        Vv = new gp_Vec(R1 * XDir.X() + R2 * YDir.X() + CosA * ZDir.X(),
                        R1 * XDir.Y() + R2 * YDir.Y() + CosA * ZDir.Y(),
                        R1 * XDir.Z() + R2 * YDir.Z() + CosA * ZDir.Z());
        Vuu = new gp_Vec(-Som1X, -Som1Y, -Som1Z);
        Vvv = new gp_Vec(0.0, 0.0, 0.0);
        Vuv = new gp_Vec(-R2 * XDir.X() + R1 * YDir.X(),
                         -R2 * XDir.Y() + R1 * YDir.Y(),
                         -R2 * XDir.Z() + R1 * YDir.Z());
    }

    public static void CylinderD2(double U, double V, in gp_Ax3 Pos, double Radius,
                                  out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                                  out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv)
    {
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double A1 = Radius * Math.Cos(U);
        double A2 = Radius * Math.Sin(U);
        double Som1X = A1 * XDir.X() + A2 * YDir.X();
        double Som1Y = A1 * XDir.Y() + A2 * YDir.Y();
        double Som1Z = A1 * XDir.Z() + A2 * YDir.Z();
        P = new gp_Pnt(Som1X + V * ZDir.X() + PLoc.X(),
                       Som1Y + V * ZDir.Y() + PLoc.Y(),
                       Som1Z + V * ZDir.Z() + PLoc.Z());
        Vu = new gp_Vec(-A2 * XDir.X() + A1 * YDir.X(),
                        -A2 * XDir.Y() + A1 * YDir.Y(),
                        -A2 * XDir.Z() + A1 * YDir.Z());
        Vv = new gp_Vec(ZDir.X(), ZDir.Y(), ZDir.Z());
        Vuu = new gp_Vec(-Som1X, -Som1Y, -Som1Z);
        Vvv = new gp_Vec(0.0, 0.0, 0.0);
        Vuv = new gp_Vec(0.0, 0.0, 0.0);
    }

    public static void SphereD2(double U, double V, in gp_Ax3 Pos, double Radius,
                                out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                                out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv)
    {
        // Vxy = CosU * XDirection + SinU * YDirection
        // DVxy = -SinU * XDirection + CosU * YDirection
        // P(U,V) = Location +  R * CosV * Vxy  +   R * SinV * Direction
        // Vu = R * CosV * DVxy
        // Vuu = - R * CosV * Vxy
        // Vv = -R * SinV * Vxy + R * CosV * Direction
        // Vvv = -R * CosV * Vxy - R * SinV * Direction
        // Vuv = - R * SinV * DVxy
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double CosU = Math.Cos(U);
        double SinU = Math.Sin(U);
        double R1 = Radius * Math.Cos(V);
        double R2 = Radius * Math.Sin(V);
        double A1 = R1 * CosU;
        double A2 = R1 * SinU;
        double A3 = R2 * CosU;
        double A4 = R2 * SinU;
        double Som1X = A1 * XDir.X() + A2 * YDir.X();
        double Som1Y = A1 * XDir.Y() + A2 * YDir.Y();
        double Som1Z = A1 * XDir.Z() + A2 * YDir.Z();
        double R2ZX = R2 * ZDir.X();
        double R2ZY = R2 * ZDir.Y();
        double R2ZZ = R2 * ZDir.Z();
        P = new gp_Pnt(Som1X + R2ZX + PLoc.X(),
                       Som1Y + R2ZY + PLoc.Y(),
                       Som1Z + R2ZZ + PLoc.Z());
        Vu = new gp_Vec(-A2 * XDir.X() + A1 * YDir.X(),
                        -A2 * XDir.Y() + A1 * YDir.Y(),
                        -A2 * XDir.Z() + A1 * YDir.Z());
        Vv = new gp_Vec(-A3 * XDir.X() - A4 * YDir.X() + R1 * ZDir.X(),
                        -A3 * XDir.Y() - A4 * YDir.Y() + R1 * ZDir.Y(),
                        -A3 * XDir.Z() - A4 * YDir.Z() + R1 * ZDir.Z());
        Vuu = new gp_Vec(-Som1X, -Som1Y, -Som1Z);
        Vvv = new gp_Vec(-Som1X - R2ZX, -Som1Y - R2ZY, -Som1Z - R2ZZ);
        Vuv = new gp_Vec(A4 * XDir.X() - A3 * YDir.X(),
                         A4 * XDir.Y() - A3 * YDir.Y(),
                         A4 * XDir.Z() - A3 * YDir.Z());
    }

    public static void TorusD2(double U, double V, in gp_Ax3 Pos, double MajorRadius, double MinorRadius,
                               out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                               out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv)
    {
        // Vvv = -MinorRadius * cos(V) * (cos(U)*XDirection + sin(U)*YDirection)
        //       -MinorRadius * sin(V) * Direction
        // Vuu = -(MajorRadius+MinorRadius*cos(V)) * (cos(U)*XDirection + sin(U)*YDirection)
        // Vuv = MinorRadius * sin(V) * (sin(U)*XDirection - cos(U)*YDirection)
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double CosU = Math.Cos(U);
        double SinU = Math.Sin(U);
        double R1 = MinorRadius * Math.Cos(V);
        double R2 = MinorRadius * Math.Sin(V);
        double R = MajorRadius + R1;
        double A1 = R * CosU;
        double A2 = R * SinU;
        double A3 = R2 * CosU;
        double A4 = R2 * SinU;
        double A5 = R1 * CosU;
        double A6 = R1 * SinU;
        // OCC620: components below the rounding noise are zeroed
        double eps = 10.0 * (MinorRadius + MajorRadius) * RealEpsilon();
        if (Math.Abs(A1) <= eps)
        {
            A1 = 0.0;
        }

        if (Math.Abs(A2) <= eps)
        {
            A2 = 0.0;
        }

        if (Math.Abs(A3) <= eps)
        {
            A3 = 0.0;
        }

        if (Math.Abs(A4) <= eps)
        {
            A4 = 0.0;
        }

        if (Math.Abs(A5) <= eps)
        {
            A5 = 0.0;
        }

        if (Math.Abs(A6) <= eps)
        {
            A6 = 0.0;
        }

        double Som1X = A1 * XDir.X() + A2 * YDir.X();
        double Som1Y = A1 * XDir.Y() + A2 * YDir.Y();
        double Som1Z = A1 * XDir.Z() + A2 * YDir.Z();
        double R2ZX = R2 * ZDir.X();
        double R2ZY = R2 * ZDir.Y();
        double R2ZZ = R2 * ZDir.Z();
        P = new gp_Pnt(Som1X + R2ZX + PLoc.X(),
                       Som1Y + R2ZY + PLoc.Y(),
                       Som1Z + R2ZZ + PLoc.Z());
        Vu = new gp_Vec(-A2 * XDir.X() + A1 * YDir.X(),
                        -A2 * XDir.Y() + A1 * YDir.Y(),
                        -A2 * XDir.Z() + A1 * YDir.Z());
        Vv = new gp_Vec(-A3 * XDir.X() - A4 * YDir.X() + R1 * ZDir.X(),
                        -A3 * XDir.Y() - A4 * YDir.Y() + R1 * ZDir.Y(),
                        -A3 * XDir.Z() - A4 * YDir.Z() + R1 * ZDir.Z());
        Vuu = new gp_Vec(-Som1X, -Som1Y, -Som1Z);
        Vvv = new gp_Vec(-A5 * XDir.X() - A6 * YDir.X() - R2ZX,
                         -A5 * XDir.Y() - A6 * YDir.Y() - R2ZY,
                         -A5 * XDir.Z() - A6 * YDir.Z() - R2ZZ);
        Vuv = new gp_Vec(A4 * XDir.X() - A3 * YDir.X(),
                         A4 * XDir.Y() - A3 * YDir.Y(),
                         A4 * XDir.Z() - A3 * YDir.Z());
    }

    public static void ConeD3(double U, double V, in gp_Ax3 Pos, double Radius, double SAngle,
                              out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                              out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv,
                              out gp_Vec Vuuu, out gp_Vec Vvvv, out gp_Vec Vuuv, out gp_Vec Vuvv)
    {
        // D3U = (Radius + V*Sin(SAng)) * (sin(U) * XDirection() - cos(U) * YDirection())
        // DUVV = 0.0
        // D3V = 0.0
        // DUUV = sin(SAng) * (-cos(U)*XDirection()-sin(U) * YDirection())
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double CosU = Math.Cos(U);
        double SinU = Math.Sin(U);
        double CosA = Math.Cos(SAngle);
        double SinA = Math.Sin(SAngle);
        double R = Radius + V * SinA;
        double A3 = V * CosA;
        double A1 = R * CosU;
        double A2 = R * SinU;
        double R1 = SinA * CosU;
        double R2 = SinA * SinU;
        double Som1X = A1 * XDir.X() + A2 * YDir.X();
        double Som1Y = A1 * XDir.Y() + A2 * YDir.Y();
        double Som1Z = A1 * XDir.Z() + A2 * YDir.Z();
        double Som2X = R1 * XDir.X() + R2 * YDir.X();
        double Som2Y = R1 * XDir.Y() + R2 * YDir.Y();
        double Som2Z = R1 * XDir.Z() + R2 * YDir.Z();
        double Dif1X = A2 * XDir.X() - A1 * YDir.X();
        double Dif1Y = A2 * XDir.Y() - A1 * YDir.Y();
        double Dif1Z = A2 * XDir.Z() - A1 * YDir.Z();
        P = new gp_Pnt(Som1X + A3 * ZDir.X() + PLoc.X(),
                       Som1Y + A3 * ZDir.Y() + PLoc.Y(),
                       Som1Z + A3 * ZDir.Z() + PLoc.Z());
        Vu = new gp_Vec(-Dif1X, -Dif1Y, -Dif1Z);
        Vv = new gp_Vec(Som2X + CosA * ZDir.X(),
                        Som2Y + CosA * ZDir.Y(),
                        Som2Z + CosA * ZDir.Z());
        Vuu = new gp_Vec(-Som1X, -Som1Y, -Som1Z);
        Vvv = new gp_Vec(0.0, 0.0, 0.0);
        Vuv = new gp_Vec(-R2 * XDir.X() + R1 * YDir.X(),
                         -R2 * XDir.Y() + R1 * YDir.Y(),
                         -R2 * XDir.Z() + R1 * YDir.Z());
        Vuuu = new gp_Vec(Dif1X, Dif1Y, Dif1Z);
        Vvvv = new gp_Vec(0.0, 0.0, 0.0);
        Vuvv = new gp_Vec(0.0, 0.0, 0.0);
        Vuuv = new gp_Vec(-Som2X, -Som2Y, -Som2Z);
    }

    public static void CylinderD3(double U, double V, in gp_Ax3 Pos, double Radius,
                                  out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                                  out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv,
                                  out gp_Vec Vuuu, out gp_Vec Vvvv, out gp_Vec Vuuv, out gp_Vec Vuvv)
    {
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double A1 = Radius * Math.Cos(U);
        double A2 = Radius * Math.Sin(U);
        double Som1X = A1 * XDir.X() + A2 * YDir.X();
        double Som1Y = A1 * XDir.Y() + A2 * YDir.Y();
        double Som1Z = A1 * XDir.Z() + A2 * YDir.Z();
        double Dif1X = A2 * XDir.X() - A1 * YDir.X();
        double Dif1Y = A2 * XDir.Y() - A1 * YDir.Y();
        double Dif1Z = A2 * XDir.Z() - A1 * YDir.Z();
        P = new gp_Pnt(Som1X + V * ZDir.X() + PLoc.X(),
                       Som1Y + V * ZDir.Y() + PLoc.Y(),
                       Som1Z + V * ZDir.Z() + PLoc.Z());
        Vu = new gp_Vec(-Dif1X, -Dif1Y, -Dif1Z);
        Vv = new gp_Vec(ZDir.X(), ZDir.Y(), ZDir.Z());
        Vuu = new gp_Vec(-Som1X, -Som1Y, -Som1Z);
        Vvv = new gp_Vec(0.0, 0.0, 0.0);
        Vuv = new gp_Vec(0.0, 0.0, 0.0);
        Vuuu = new gp_Vec(Dif1X, Dif1Y, Dif1Z);
        Vvvv = new gp_Vec(0.0, 0.0, 0.0);
        Vuvv = new gp_Vec(0.0, 0.0, 0.0);
        Vuuv = new gp_Vec(0.0, 0.0, 0.0);
    }

    public static void SphereD3(double U, double V, in gp_Ax3 Pos, double Radius,
                                out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                                out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv,
                                out gp_Vec Vuuu, out gp_Vec Vvvv, out gp_Vec Vuuv, out gp_Vec Vuvv)
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
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double CosU = Math.Cos(U);
        double SinU = Math.Sin(U);
        double R1 = Radius * Math.Cos(V);
        double R2 = Radius * Math.Sin(V);
        double A1 = R1 * CosU;
        double A2 = R1 * SinU;
        double A3 = R2 * CosU;
        double A4 = R2 * SinU;
        double Som1X = A1 * XDir.X() + A2 * YDir.X();
        double Som1Y = A1 * XDir.Y() + A2 * YDir.Y();
        double Som1Z = A1 * XDir.Z() + A2 * YDir.Z();
        double Som3X = A3 * XDir.X() + A4 * YDir.X();
        double Som3Y = A3 * XDir.Y() + A4 * YDir.Y();
        double Som3Z = A3 * XDir.Z() + A4 * YDir.Z();
        double Dif1X = A2 * XDir.X() - A1 * YDir.X();
        double Dif1Y = A2 * XDir.Y() - A1 * YDir.Y();
        double Dif1Z = A2 * XDir.Z() - A1 * YDir.Z();
        double R1ZX = R1 * ZDir.X();
        double R1ZY = R1 * ZDir.Y();
        double R1ZZ = R1 * ZDir.Z();
        double R2ZX = R2 * ZDir.X();
        double R2ZY = R2 * ZDir.Y();
        double R2ZZ = R2 * ZDir.Z();
        P = new gp_Pnt(Som1X + R2ZX + PLoc.X(),
                       Som1Y + R2ZY + PLoc.Y(),
                       Som1Z + R2ZZ + PLoc.Z());
        Vu = new gp_Vec(-Dif1X, -Dif1Y, -Dif1Z);
        Vv = new gp_Vec(-Som3X + R1ZX, -Som3Y + R1ZY, -Som3Z + R1ZZ);
        Vuu = new gp_Vec(-Som1X, -Som1Y, -Som1Z);
        Vvv = new gp_Vec(-Som1X - R2ZX, -Som1Y - R2ZY, -Som1Z - R2ZZ);
        Vuv = new gp_Vec(A4 * XDir.X() - A3 * YDir.X(),
                         A4 * XDir.Y() - A3 * YDir.Y(),
                         A4 * XDir.Z() - A3 * YDir.Z());
        Vuuu = new gp_Vec(Dif1X, Dif1Y, Dif1Z);
        Vvvv = new gp_Vec(Som3X - R1ZX, Som3Y - R1ZY, Som3Z - R1ZZ);
        Vuvv = new gp_Vec(Dif1X, Dif1Y, Dif1Z);
        Vuuv = new gp_Vec(Som3X, Som3Y, Som3Z);
    }

    public static void TorusD3(double U, double V, in gp_Ax3 Pos, double MajorRadius, double MinorRadius,
                               out gp_Pnt P, out gp_Vec Vu, out gp_Vec Vv,
                               out gp_Vec Vuu, out gp_Vec Vvv, out gp_Vec Vuv,
                               out gp_Vec Vuuu, out gp_Vec Vvvv, out gp_Vec Vuuv, out gp_Vec Vuvv)
    {
        // Vvvv = - Vv
        // Vuuu = -Vu
        // Vuvv = MinorRadius * cos(V) * (sin(U)*XDirection - cos(U)*YDirection)
        // Vuuv = MinorRadius * sin(V) * (cos(U)*XDirection + sin(U)*YDirection)
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double CosU = Math.Cos(U);
        double SinU = Math.Sin(U);
        double R1 = MinorRadius * Math.Cos(V);
        double R2 = MinorRadius * Math.Sin(V);
        double R = MajorRadius + R1;
        double A1 = R * CosU;
        double A2 = R * SinU;
        double A3 = R2 * CosU;
        double A4 = R2 * SinU;
        double A5 = R1 * CosU;
        double A6 = R1 * SinU;
        // OCC620: components below the rounding noise are zeroed
        double eps = 10.0 * (MinorRadius + MajorRadius) * RealEpsilon();
        if (Math.Abs(A1) <= eps)
        {
            A1 = 0.0;
        }

        if (Math.Abs(A2) <= eps)
        {
            A2 = 0.0;
        }

        if (Math.Abs(A3) <= eps)
        {
            A3 = 0.0;
        }

        if (Math.Abs(A4) <= eps)
        {
            A4 = 0.0;
        }

        if (Math.Abs(A5) <= eps)
        {
            A5 = 0.0;
        }

        if (Math.Abs(A6) <= eps)
        {
            A6 = 0.0;
        }

        double Som1X = A1 * XDir.X() + A2 * YDir.X();
        double Som1Y = A1 * XDir.Y() + A2 * YDir.Y();
        double Som1Z = A1 * XDir.Z() + A2 * YDir.Z();
        double Som3X = A3 * XDir.X() + A4 * YDir.X();
        double Som3Y = A3 * XDir.Y() + A4 * YDir.Y();
        double Som3Z = A3 * XDir.Z() + A4 * YDir.Z();
        double Dif1X = A2 * XDir.X() - A1 * YDir.X();
        double Dif1Y = A2 * XDir.Y() - A1 * YDir.Y();
        double Dif1Z = A2 * XDir.Z() - A1 * YDir.Z();
        double R1ZX = R1 * ZDir.X();
        double R1ZY = R1 * ZDir.Y();
        double R1ZZ = R1 * ZDir.Z();
        double R2ZX = R2 * ZDir.X();
        double R2ZY = R2 * ZDir.Y();
        double R2ZZ = R2 * ZDir.Z();
        P = new gp_Pnt(Som1X + R2ZX + PLoc.X(),
                       Som1Y + R2ZY + PLoc.Y(),
                       Som1Z + R2ZZ + PLoc.Z());
        Vu = new gp_Vec(-Dif1X, -Dif1Y, -Dif1Z);
        Vv = new gp_Vec(-Som3X + R1ZX, -Som3Y + R1ZY, -Som3Z + R1ZZ);
        Vuu = new gp_Vec(-Som1X, -Som1Y, -Som1Z);
        Vvv = new gp_Vec(-A5 * XDir.X() - A6 * YDir.X() - R2ZX,
                         -A5 * XDir.Y() - A6 * YDir.Y() - R2ZY,
                         -A5 * XDir.Z() - A6 * YDir.Z() - R2ZZ);
        Vuv = new gp_Vec(A4 * XDir.X() - A3 * YDir.X(),
                         A4 * XDir.Y() - A3 * YDir.Y(),
                         A4 * XDir.Z() - A3 * YDir.Z());
        Vuuu = new gp_Vec(Dif1X, Dif1Y, Dif1Z);
        Vvvv = new gp_Vec(Som3X - R1ZX, Som3Y - R1ZY, Som3Z - R1ZZ);
        Vuuv = new gp_Vec(Som3X, Som3Y, Som3Z);
        Vuvv = new gp_Vec(A6 * XDir.X() - A5 * YDir.X(),
                          A6 * XDir.Y() - A5 * YDir.Y(),
                          A6 * XDir.Z() - A5 * YDir.Z());
    }

    /// <summary>The parameters (U, V) of a point on the plane: its coordinates in the plane's system.</summary>
    public static void PlaneParameters(in gp_Ax3 Pos, in gp_Pnt P, out double U, out double V)
    {
        gp_Trsf T = new gp_Trsf();
        T.SetTransformation(Pos);
        gp_Pnt Ploc = P.Transformed(T);
        U = Ploc.X();
        V = Ploc.Y();
    }

    /// <summary>The parameters (U, V) of a point on the cylinder, U in [0, 2*PI].</summary>
    public static void CylinderParameters(in gp_Ax3 Pos, double Radius, in gp_Pnt P, out double U, out double V)
    {
        gp_Trsf T = new gp_Trsf();
        T.SetTransformation(Pos);
        gp_Pnt Ploc = P.Transformed(T);
        U = Math.Atan2(Ploc.Y(), Ploc.X());
        normalizeAngle(ref U);
        V = Ploc.Z();
    }

    /// <summary>
    /// The parameters (U, V) of a point on the cone, U in [0, 2*PI]. For a point off the cone, V is that of its
    /// projection on the generating line of parameter U.
    /// </summary>
    public static void ConeParameters(in gp_Ax3 Pos, double Radius, double SAngle, in gp_Pnt P,
                                      out double U, out double V)
    {
        gp_Trsf T = new gp_Trsf();
        T.SetTransformation(Pos);
        gp_Pnt Ploc = P.Transformed(T);

        // Check if point is at the apex
        if (Math.Abs(Ploc.X()) < gp.Resolution() && Math.Abs(Ploc.Y()) < gp.Resolution())
        {
            U = 0.0;
        }
        else if (-Radius > Ploc.Z() * Math.Tan(SAngle))
        {
            // the point is at the wrong side of the apex
            U = Math.Atan2(-Ploc.Y(), -Ploc.X());
        }
        else
        {
            U = Math.Atan2(Ploc.Y(), Ploc.X());
        }

        normalizeAngle(ref U);

        // Evaluate V as follows :
        // P0 = Cone.Value(U,0)
        // P1 = Cone.Value(U,1)
        // V = P0 P1 . P0 Ploc
        // After simplification obtain:
        // V = sin(Sang) * ( x cosU + y SinU - R) + z * cos(Sang)
        // Method that permits to find V of the projected point if the point
        // is not actually on the cone.
        V = Math.Sin(SAngle) * (Ploc.X() * Math.Cos(U) + Ploc.Y() * Math.Sin(U) - Radius) + Math.Cos(SAngle) * Ploc.Z();
    }

    /// <summary>The parameters (U, V) of a point on the sphere, U in [0, 2*PI] and V in [-PI/2, PI/2].</summary>
    public static void SphereParameters(in gp_Ax3 Pos, double Radius, in gp_Pnt P, out double U, out double V)
    {
        gp_Trsf T = new gp_Trsf();
        T.SetTransformation(Pos);
        gp_Pnt Ploc = P.Transformed(T);
        Ploc.Coord(out double x, out double y, out double z);
        double l = Math.Sqrt(x * x + y * y);
        if (l < gp.Resolution())
        {
            // point on axis Z of the sphere
            if (z > 0.0)
            {
                V = M_PI_2; // PI * 0.5
            }
            else
            {
                V = -M_PI_2; // PI * 0.5
            }

            U = 0.0;
        }
        else
        {
            V = Math.Atan(z / l);
            U = Math.Atan2(y, x);
            normalizeAngle(ref U);
        }
    }

    /// <summary>The parameters (U, V) of a point on the torus, both in [0, 2*PI].</summary>
    public static void TorusParameters(in gp_Ax3 Pos, double MajorRadius, double MinorRadius, in gp_Pnt P,
                                       out double U, out double V)
    {
        gp_Trsf Tref = new gp_Trsf();
        Tref.SetTransformation(Pos);
        gp_Pnt Ploc = P.Transformed(Tref);
        Ploc.Coord(out double x, out double y, out double z);

        // all that to process case of  Major < Minor.
        U = Math.Atan2(y, x);
        if (MajorRadius < MinorRadius)
        {
            double cosu0 = Math.Cos(U);
            double sinu0 = Math.Sin(U);
            double z2 = z * z;
            double MinR2 = MinorRadius * MinorRadius;
            double RCosU = MajorRadius * cosu0;
            double RSinU = MajorRadius * sinu0;
            double xm = x - RCosU;
            double ym = y - RSinU;
            double xp = x + RCosU;
            double yp = y + RSinU;
            double D1 = xm * xm + ym * ym + z2 - MinR2;
            double D2 = xp * xp + yp * yp + z2 - MinR2;
            double AD1 = Math.Abs(D1);
            double AD2 = Math.Abs(D2);
            if (AD2 < AD1)
            {
                U += M_PI;
            }
        }

        normalizeAngle(ref U);
        double cosu = Math.Cos(U);
        double sinu = Math.Sin(U);
        gp_Dir dx = new gp_Dir(cosu, sinu, 0.0);
        gp_XYZ dPV = new gp_XYZ(x - MajorRadius * cosu, y - MajorRadius * sinu, z);
        double aMag = dPV.Modulus();
        if (aMag <= gp.Resolution())
        {
            V = 0.0;
        }
        else
        {
            gp_Dir dP = new gp_Dir(dPV);
            V = dx.AngleWithRef(dP, dx ^ gp.DZ());
        }

        normalizeAngle(ref V);
    }

    /// <summary>The U isoparametric line of the plane.</summary>
    public static gp_Lin PlaneUIso(in gp_Ax3 Pos, double U)
    {
        gp_Lin L = new gp_Lin(Pos.Location(), Pos.YDirection());
        gp_Vec Ve = new gp_Vec(Pos.XDirection());
        Ve *= U;
        L.Translate(Ve);
        return L;
    }

    /// <summary>The U isoparametric line of the cylinder.</summary>
    public static gp_Lin CylinderUIso(in gp_Ax3 Pos, double Radius, double U)
    {
        CylinderD1(U, 0.0, Pos, Radius, out gp_Pnt P, out gp_Vec DU, out gp_Vec DV);
        gp_Lin L = new gp_Lin(P, new gp_Dir(DV));
        return L;
    }

    /// <summary>The U isoparametric line of the cone.</summary>
    public static gp_Lin ConeUIso(in gp_Ax3 Pos, double Radius, double SAngle, double U)
    {
        ConeD1(U, 0, Pos, Radius, SAngle, out gp_Pnt P, out gp_Vec DU, out gp_Vec DV);
        gp_Lin L = new gp_Lin(P, new gp_Dir(DV));
        return L;
    }

    /// <summary>The U isoparametric circle of the sphere (a whole meridian circle, not trimmed).</summary>
    public static gp_Circ SphereUIso(in gp_Ax3 Pos, double Radius, double U)
    {
        gp_Vec dx = new gp_Vec(Pos.XDirection());
        gp_Vec dy = new gp_Vec(Pos.YDirection());
        gp_Dir dz = Pos.Direction();
        gp_Dir cx = new gp_Dir(Math.Cos(U) * dx + Math.Sin(U) * dy);
        gp_Ax2 axes = new gp_Ax2(Pos.Location(), cx.Crossed(dz), cx);
        gp_Circ Circ = new gp_Circ(axes, Radius);
        return Circ;
    }

    /// <summary>The U isoparametric circle of the torus.</summary>
    public static gp_Circ TorusUIso(in gp_Ax3 Pos, double MajorRadius, double MinorRadius, double U)
    {
        gp_Vec dx = new gp_Vec(Pos.XDirection());
        gp_Vec dy = new gp_Vec(Pos.YDirection());
        gp_Dir dz = Pos.Direction();
        gp_Dir cx = new gp_Dir(Math.Cos(U) * dx + Math.Sin(U) * dy);
        gp_Ax2 axes = new gp_Ax2(Pos.Location(), cx.Crossed(dz), cx);
        gp_Vec Ve = new gp_Vec(cx);
        Ve *= MajorRadius;
        axes.Translate(Ve);
        gp_Circ Circ = new gp_Circ(axes, MinorRadius);
        return Circ;
    }

    /// <summary>The V isoparametric line of the plane.</summary>
    public static gp_Lin PlaneVIso(in gp_Ax3 Pos, double V)
    {
        gp_Lin L = new gp_Lin(Pos.Location(), Pos.XDirection());
        gp_Vec Ve = new gp_Vec(Pos.YDirection());
        Ve *= V;
        L.Translate(Ve);
        return L;
    }

    /// <summary>The V isoparametric circle of the cylinder.</summary>
    public static gp_Circ CylinderVIso(in gp_Ax3 Pos, double Radius, double V)
    {
        gp_Ax2 axes = Pos.Ax2();
        gp_Vec Ve = new gp_Vec(Pos.Direction());
        Ve.Multiply(V);
        axes.Translate(Ve);
        gp_Circ C = new gp_Circ(axes, Radius);
        return C;
    }

    /// <summary>The V isoparametric circle of the cone.</summary>
    public static gp_Circ ConeVIso(in gp_Ax3 Pos, double Radius, double SAngle, double V)
    {
        gp_Ax3 axes = Pos;
        gp_Vec Ve = new gp_Vec(Pos.Direction());
        Ve.Multiply(V * Math.Cos(SAngle));
        axes.Translate(Ve);
        double R = Radius + V * Math.Sin(SAngle);
        if (R < 0)
        {
            axes.XReverse();
            axes.YReverse();
            R = -R;
        }

        gp_Circ C = new gp_Circ(axes.Ax2(), R);
        return C;
    }

    /// <summary>The V isoparametric circle (a parallel) of the sphere.</summary>
    public static gp_Circ SphereVIso(in gp_Ax3 Pos, double Radius, double V)
    {
        gp_Ax2 axes = Pos.Ax2();
        gp_Vec Ve = new gp_Vec(Pos.Direction());
        Ve.Multiply(Radius * Math.Sin(V));
        axes.Translate(Ve);
        double radius = Radius * Math.Cos(V);
        // #23170: if V is even slightly (e.g. by double epsilon) greater than PI/2,
        // radius will become negative and constructor of gp_Circ will raise exception.
        // Lets try to create correct isoline even on analytical continuation for |V| > PI/2...
        if (radius < 0.0)
        {
            axes.SetDirection(-axes.Direction());
            radius = -radius;
        }

        gp_Circ Circ = new gp_Circ(axes, radius);
        return Circ;
    }

    /// <summary>The V isoparametric circle of the torus.</summary>
    public static gp_Circ TorusVIso(in gp_Ax3 Pos, double MajorRadius, double MinorRadius, double V)
    {
        gp_Ax3 axes = new gp_Ax3(Pos.Ax2());
        gp_Vec Ve = new gp_Vec(Pos.Direction());
        Ve.Multiply(MinorRadius * Math.Sin(V));
        axes.Translate(Ve);
        double R = MajorRadius + MinorRadius * Math.Cos(V);
        if (R < 0)
        {
            axes.XReverse();
            axes.YReverse();
            R = -R;
        }

        gp_Circ Circ = new gp_Circ(axes.Ax2(), R);
        return Circ;
    }
}
