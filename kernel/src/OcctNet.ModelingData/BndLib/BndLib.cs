// Port of: src/ModelingData/TKGeomBase/BndLib/BndLib.hxx
// Port of: src/ModelingData/TKGeomBase/BndLib/BndLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Adds elementary geometric primitives of gp (lines, conics, elementary surfaces), optionally limited by parameter
/// values, to a bounding box. The gp objects are parameterised as the equivalent Geom and Geom2d objects.
/// </summary>
public static class BndLib
{
    // Cosine of M_PI/8 (22.5 degrees) - used for 8-point polygon approximation.
    private const double THE_COS_PI8 = 0.92387953251128674;

    // Cosine (and sine) of M_PI/4 (45 degrees) - used for diagonal points.
    private const double THE_COS_PI4 = 0.70710678118654746;

    // X and Y multipliers for 8 polygon points at 45-degree intervals (0, 45, 90, ..., 315 degrees).
    // Point i corresponds to angle i * 45 degrees. Also the torus cross-section multipliers
    // (aRadiusMult and aZMult), which have the same values.
    private static ReadOnlySpan<double> XMult => [1.0, THE_COS_PI4, 0.0, -THE_COS_PI4, -1.0, -THE_COS_PI4, 0.0, THE_COS_PI4];

    private static ReadOnlySpan<double> YMult => [0.0, THE_COS_PI4, 1.0, THE_COS_PI4, 0.0, -THE_COS_PI4, -1.0, -THE_COS_PI4];

    // The anonymous-namespace template Compute<PointType, BndBoxType>, ported per instantiation (2D and 3D).

    private static void Compute(double theP1, double theP2, double theRa, double theRb,
                                in Point2D theXd, in Point2D theYd, in Point2D theO, ref Box2d theB)
    {
        double aTeta1;
        double aTeta2;
        if (theP2 < theP1)
        {
            aTeta1 = theP2;
            aTeta2 = theP1;
        }
        else
        {
            aTeta1 = theP1;
            aTeta2 = theP2;
        }

        double aDelta = Math.Abs(aTeta2 - aTeta1);
        if (aDelta > 2.0 * M_PI)
        {
            aTeta1 = 0.0;
            aTeta2 = 2.0 * M_PI;
        }
        else
        {
            // Normalize aTeta1 to [0, 2*PI) range
            aTeta1 = aTeta1 % (2.0 * M_PI);
            if (aTeta1 < 0.0)
            {
                aTeta1 += 2.0 * M_PI;
            }
            aTeta2 = aTeta1 + aDelta;
        }

        // One places already both ends
        double aCn1, aSn1, aCn2, aSn2;
        aCn1 = Math.Cos(aTeta1);
        aSn1 = Math.Sin(aTeta1);
        aCn2 = Math.Cos(aTeta2);
        aSn2 = Math.Sin(aTeta2);
        theB.Add(new Point2D(theO.Coord() + theRa * aCn1 * theXd.Coord() + theRb * aSn1 * theYd.Coord()));
        theB.Add(new Point2D(theO.Coord() + theRa * aCn2 * theXd.Coord() + theRb * aSn2 * theYd.Coord()));

        double aRam, aRbm;
        if (aDelta > M_PI / 8.0)
        {
            // Main radiuses to take into account only 8 points (/cos(Pi/8.))
            aRam = theRa / THE_COS_PI8;
            aRbm = theRb / THE_COS_PI8;
        }
        else
        {
            // Main radiuses to take into account the arrow
            double aTc = Math.Cos(aDelta / 2);
            aRam = theRa / aTc;
            aRbm = theRb / aTc;
        }
        theB.Add(new Point2D(theO.Coord() + aRam * aCn1 * theXd.Coord() + aRbm * aSn1 * theYd.Coord()));
        theB.Add(new Point2D(theO.Coord() + aRam * aCn2 * theXd.Coord() + aRbm * aSn2 * theYd.Coord()));

        int aDeb = (int)(aTeta1 / (M_PI / 4.0));
        int aFin = (int)(aTeta2 / (M_PI / 4.0));
        aDeb++;

        if (aDeb > aFin)
        {
            return;
        }

        // Add polygon points from aDeb to aFin, wrapping around using modulo 8.
        for (int i = aDeb; i <= aFin; ++i)
        {
            int anIdx = i % 8;
            theB.Add(new Point2D(theO.Coord() + aRam * XMult[anIdx] * theXd.Coord()
                                  + aRbm * YMult[anIdx] * theYd.Coord()));
        }
    }

    private static void Compute(double theP1, double theP2, double theRa, double theRb,
                                in Point3D theXd, in Point3D theYd, in Point3D theO, ref Box theB)
    {
        double aTeta1;
        double aTeta2;
        if (theP2 < theP1)
        {
            aTeta1 = theP2;
            aTeta2 = theP1;
        }
        else
        {
            aTeta1 = theP1;
            aTeta2 = theP2;
        }

        double aDelta = Math.Abs(aTeta2 - aTeta1);
        if (aDelta > 2.0 * M_PI)
        {
            aTeta1 = 0.0;
            aTeta2 = 2.0 * M_PI;
        }
        else
        {
            // Normalize aTeta1 to [0, 2*PI) range
            aTeta1 = aTeta1 % (2.0 * M_PI);
            if (aTeta1 < 0.0)
            {
                aTeta1 += 2.0 * M_PI;
            }
            aTeta2 = aTeta1 + aDelta;
        }

        // One places already both ends
        double aCn1, aSn1, aCn2, aSn2;
        aCn1 = Math.Cos(aTeta1);
        aSn1 = Math.Sin(aTeta1);
        aCn2 = Math.Cos(aTeta2);
        aSn2 = Math.Sin(aTeta2);
        theB.Add(new Point3D(theO.Coord() + theRa * aCn1 * theXd.Coord() + theRb * aSn1 * theYd.Coord()));
        theB.Add(new Point3D(theO.Coord() + theRa * aCn2 * theXd.Coord() + theRb * aSn2 * theYd.Coord()));

        double aRam, aRbm;
        if (aDelta > M_PI / 8.0)
        {
            // Main radiuses to take into account only 8 points (/cos(Pi/8.))
            aRam = theRa / THE_COS_PI8;
            aRbm = theRb / THE_COS_PI8;
        }
        else
        {
            // Main radiuses to take into account the arrow
            double aTc = Math.Cos(aDelta / 2);
            aRam = theRa / aTc;
            aRbm = theRb / aTc;
        }
        theB.Add(new Point3D(theO.Coord() + aRam * aCn1 * theXd.Coord() + aRbm * aSn1 * theYd.Coord()));
        theB.Add(new Point3D(theO.Coord() + aRam * aCn2 * theXd.Coord() + aRbm * aSn2 * theYd.Coord()));

        int aDeb = (int)(aTeta1 / (M_PI / 4.0));
        int aFin = (int)(aTeta2 / (M_PI / 4.0));
        aDeb++;

        if (aDeb > aFin)
        {
            return;
        }

        // Add polygon points from aDeb to aFin, wrapping around using modulo 8.
        for (int i = aDeb; i <= aFin; ++i)
        {
            int anIdx = i % 8;
            theB.Add(new Point3D(theO.Coord() + aRam * XMult[anIdx] * theXd.Coord()
                                + aRbm * YMult[anIdx] * theYd.Coord()));
        }
    }

    private static void OpenMin(in Direction3D V, ref Box B)
    {
        // OpenMin opens the box in the direction of decreasing parameter.
        // For a line P(t) = Origin + t*V, as t -> -Inf:
        //   - If V.X() > 0, x-coordinate -> -Inf, so open Xmin
        //   - If V.X() < 0, x-coordinate -> +Inf, so open Xmax
        if (V.IsParallel(Constants.DX(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (V.X() > 0.0)
            {
                B.OpenXmin();
            }
            else
            {
                B.OpenXmax();
            }
        }
        else if (V.IsParallel(Constants.DY(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (V.Y() > 0.0)
            {
                B.OpenYmin();
            }
            else
            {
                B.OpenYmax();
            }
        }
        else if (V.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (V.Z() > 0.0)
            {
                B.OpenZmin();
            }
            else
            {
                B.OpenZmax();
            }
        }
        else
        {
            B.OpenXmin();
            B.OpenYmin();
            B.OpenZmin();
        }
    }

    private static void OpenMax(in Direction3D V, ref Box B)
    {
        // OpenMax opens the box in the direction of increasing parameter.
        // For a line P(t) = Origin + t*V, as t -> +Inf:
        //   - If V.X() > 0, x-coordinate -> +Inf, so open Xmax
        //   - If V.X() < 0, x-coordinate -> -Inf, so open Xmin
        if (V.IsParallel(Constants.DX(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (V.X() > 0.0)
            {
                B.OpenXmax();
            }
            else
            {
                B.OpenXmin();
            }
        }
        else if (V.IsParallel(Constants.DY(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (V.Y() > 0.0)
            {
                B.OpenYmax();
            }
            else
            {
                B.OpenYmin();
            }
        }
        else if (V.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (V.Z() > 0.0)
            {
                B.OpenZmax();
            }
            else
            {
                B.OpenZmin();
            }
        }
        else
        {
            B.OpenXmax();
            B.OpenYmax();
            B.OpenZmax();
        }
    }

    private static void OpenMinMax(in Direction3D V, ref Box B)
    {
        if (V.IsParallel(Constants.DX(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            B.OpenXmax();
            B.OpenXmin();
        }
        else if (V.IsParallel(Constants.DY(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            B.OpenYmax();
            B.OpenYmin();
        }
        else if (V.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            B.OpenZmax();
            B.OpenZmin();
        }
        else
        {
            B.OpenXmin();
            B.OpenYmin();
            B.OpenZmin();
            B.OpenXmax();
            B.OpenYmax();
            B.OpenZmax();
        }
    }

    private static void OpenMin(in Direction2D V, ref Box2d B)
    {
        // OpenMin opens the box in the direction of decreasing parameter.
        // For a line P(t) = Origin + t*V, as t -> -Inf:
        //   - If V.X() > 0, x-coordinate -> -Inf, so open Xmin
        //   - If V.X() < 0, x-coordinate -> +Inf, so open Xmax
        if (V.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (V.X() > 0.0)
            {
                B.OpenXmin();
            }
            else
            {
                B.OpenXmax();
            }
        }
        else if (V.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (V.Y() > 0.0)
            {
                B.OpenYmin();
            }
            else
            {
                B.OpenYmax();
            }
        }
        else
        {
            B.OpenXmin();
            B.OpenYmin();
        }
    }

    private static void OpenMax(in Direction2D V, ref Box2d B)
    {
        // OpenMax opens the box in the direction of increasing parameter.
        // For a line P(t) = Origin + t*V, as t -> +Inf:
        //   - If V.X() > 0, x-coordinate -> +Inf, so open Xmax
        //   - If V.X() < 0, x-coordinate -> -Inf, so open Xmin
        if (V.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (V.X() > 0.0)
            {
                B.OpenXmax();
            }
            else
            {
                B.OpenXmin();
            }
        }
        else if (V.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (V.Y() > 0.0)
            {
                B.OpenYmax();
            }
            else
            {
                B.OpenYmin();
            }
        }
        else
        {
            B.OpenXmax();
            B.OpenYmax();
        }
    }

    private static void OpenMinMax(in Direction2D V, ref Box2d B)
    {
        if (V.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            B.OpenXmax();
            B.OpenXmin();
        }
        else if (V.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            B.OpenYmax();
            B.OpenYmin();
        }
        else
        {
            B.OpenXmin();
            B.OpenYmin();
            B.OpenXmax();
            B.OpenYmax();
        }
    }

    /// <summary>
    /// Adds the segment of the line L limited by the parameter values P1 and P2 to B, then enlarges B by Tol.
    /// P1 and P2 may be infinite.
    /// </summary>
    /// <exception cref="FailureException">P1 and P2 are both negative infinite, or both positive infinite.</exception>
    public static void Add(in Line3D L, double P1, double P2, double Tol, ref Box B)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                OpenMinMax(L.Direction(), ref B);
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, L));
            }
            else
            {
                OpenMin(L.Direction(), ref B);
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, L));
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                OpenMinMax(L.Direction(), ref B);
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, L));
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                OpenMax(L.Direction(), ref B);
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, L));
            }
        }
        else
        {
            B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P1, L));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                OpenMin(L.Direction(), ref B);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                OpenMax(L.Direction(), ref B);
            }
            else
            {
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, L));
            }
        }
        B.Enlarge(Tol);
    }

    /// <summary>The 2D twin of <see cref="Add(in Line3D, double, double, double, ref Box)"/>.</summary>
    public static void Add(in Line2D L, double P1, double P2, double Tol, ref Box2d B)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                OpenMinMax(L.Direction(), ref B);
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, L));
            }
            else
            {
                OpenMin(L.Direction(), ref B);
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, L));
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                OpenMinMax(L.Direction(), ref B);
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, L));
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                OpenMax(L.Direction(), ref B);
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, L));
            }
        }
        else
        {
            B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P1, L));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                OpenMin(L.Direction(), ref B);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                OpenMax(L.Direction(), ref B);
            }
            else
            {
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, L));
            }
        }
        B.Enlarge(Tol);
    }

    /// <summary>Adds the whole circle C to B, then enlarges B by Tol.</summary>
    public static void Add(in Circle3D C, double Tol, ref Box B)
    {
        double U1 = 0.0, U2 = 2.0 * M_PI;
        Add(C, U1, U2, Tol, ref B);
    }

    /// <summary>Adds the arc [U1, U2] of the circle C to B, then enlarges B by Tol. U2 - U1 can be in [0, 2*pi].</summary>
    public static void Add(in Circle3D C, double U1, double U2, double Tol, ref Box B)
    {
        double period = 2.0 * M_PI - Epsilon(2.0 * M_PI);

        double utrim1 = U1, utrim2 = U2;
        if (U2 - U1 > period)
        {
            utrim1 = 0.0;
            utrim2 = 2.0 * M_PI;
        }
        else
        {
            double tol = Epsilon(1.0);
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, tol, ref utrim1, ref utrim2);
        }
        double R = C.Radius();
        Xyz O = C.Location().XYZ();
        Xyz Xd = C.XAxis().Direction().XYZ();
        Xyz Yd = C.YAxis().Direction().XYZ();
        Frame3D pos = C.Position();
        //
        double tt;
        double xmin, xmax, txmin, txmax;
        if (Math.Abs(Xd.X()) > Constants.Resolution())
        {
            txmin = Math.Atan(Yd.X() / Xd.X());
            txmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(txmin, 0.0, 2.0 * M_PI);
        }
        else
        {
            txmin = M_PI / 2.0;
        }
        txmax = txmin <= M_PI ? txmin + M_PI : txmin - M_PI;
        xmin = R * Math.Cos(txmin) * Xd.X() + R * Math.Sin(txmin) * Yd.X() + O.X();
        xmax = R * Math.Cos(txmax) * Xd.X() + R * Math.Sin(txmax) * Yd.X() + O.X();
        if (xmin > xmax)
        {
            tt = xmin;
            xmin = xmax;
            xmax = tt;
            tt = txmin;
            txmin = txmax;
            txmax = tt;
        }
        //
        double ymin, ymax, tymin, tymax;
        if (Math.Abs(Xd.Y()) > Constants.Resolution())
        {
            tymin = Math.Atan(Yd.Y() / Xd.Y());
            tymin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tymin, 0.0, 2.0 * M_PI);
        }
        else
        {
            tymin = M_PI / 2.0;
        }
        tymax = tymin <= M_PI ? tymin + M_PI : tymin - M_PI;
        ymin = R * Math.Cos(tymin) * Xd.Y() + R * Math.Sin(tymin) * Yd.Y() + O.Y();
        ymax = R * Math.Cos(tymax) * Xd.Y() + R * Math.Sin(tymax) * Yd.Y() + O.Y();
        if (ymin > ymax)
        {
            tt = ymin;
            ymin = ymax;
            ymax = tt;
            tt = tymin;
            tymin = tymax;
            tymax = tt;
        }
        //
        double zmin, zmax, tzmin, tzmax;
        if (Math.Abs(Xd.Z()) > Constants.Resolution())
        {
            tzmin = Math.Atan(Yd.Z() / Xd.Z());
            tzmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tzmin, 0.0, 2.0 * M_PI);
        }
        else
        {
            tzmin = M_PI / 2.0;
        }
        tzmax = tzmin <= M_PI ? tzmin + M_PI : tzmin - M_PI;
        zmin = R * Math.Cos(tzmin) * Xd.Z() + R * Math.Sin(tzmin) * Yd.Z() + O.Z();
        zmax = R * Math.Cos(tzmax) * Xd.Z() + R * Math.Sin(tzmax) * Yd.Z() + O.Z();
        if (zmin > zmax)
        {
            tt = zmin;
            zmin = zmax;
            zmax = tt;
            tt = tzmin;
            tzmin = tzmax;
            tzmax = tt;
        }
        //
        if (utrim2 - utrim1 >= period)
        {
            B.Update(xmin, ymin, zmin, xmax, ymax, zmax);
        }
        else
        {
            Point3D P = global::Kerf.Engine.Geometry.ElCLib.CircleValue(utrim1, pos, R);
            B.Add(P);
            P = global::Kerf.Engine.Geometry.ElCLib.CircleValue(utrim2, pos, R);
            B.Add(P);
            double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
            B.FinitePart().Get(out Xmin, out Ymin, out Zmin, out Xmax, out Ymax, out Zmax);
            double gap = B.GetGap();
            Xmin += gap;
            Ymin += gap;
            Zmin += gap;
            Xmax -= gap;
            Ymax -= gap;
            Zmax -= gap;
            //
            txmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(txmin, utrim1, utrim1 + 2.0 * M_PI);
            if (txmin >= utrim1 && txmin <= utrim2)
            {
                Xmin = Min(xmin, Xmin);
            }
            txmax = global::Kerf.Engine.Geometry.ElCLib.InPeriod(txmax, utrim1, utrim1 + 2.0 * M_PI);
            if (txmax >= utrim1 && txmax <= utrim2)
            {
                Xmax = Max(xmax, Xmax);
            }
            //
            tymin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tymin, utrim1, utrim1 + 2.0 * M_PI);
            if (tymin >= utrim1 && tymin <= utrim2)
            {
                Ymin = Min(ymin, Ymin);
            }
            tymax = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tymax, utrim1, utrim1 + 2.0 * M_PI);
            if (tymax >= utrim1 && tymax <= utrim2)
            {
                Ymax = Max(ymax, Ymax);
            }
            //
            tzmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tzmin, utrim1, utrim1 + 2.0 * M_PI);
            if (tzmin >= utrim1 && tzmin <= utrim2)
            {
                Zmin = Min(zmin, Zmin);
            }
            tzmax = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tzmax, utrim1, utrim1 + 2.0 * M_PI);
            if (tzmax >= utrim1 && tzmax <= utrim2)
            {
                Zmax = Max(zmax, Zmax);
            }
            //
            B.Update(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);
        }
        //
        B.Enlarge(Tol);
    }

    /// <summary>Adds the whole 2D circle C to B, then enlarges B by Tol.</summary>
    public static void Add(in Circle2D C, double Tol, ref Box2d B)
    {
        double R = C.Radius();
        Xy O = C.Location().XY();
        Xy Xd = C.XAxis().Direction().XY();
        Xy Yd = C.YAxis().Direction().XY();
        B.Add(new Point2D(O - R * Xd - R * Yd));
        B.Add(new Point2D(O - R * Xd + R * Yd));
        B.Add(new Point2D(O + R * Xd - R * Yd));
        B.Add(new Point2D(O + R * Xd + R * Yd));
        B.Enlarge(Tol);
    }

    /// <summary>Adds the arc [P1, P2] of the 2D circle C to B, then enlarges B by Tol. P2 - P1 can be in [0, 2*pi].</summary>
    public static void Add(in Circle2D C, double P1, double P2, double Tol, ref Box2d B)
    {
        Compute(P1,
                P2,
                C.Radius(),
                C.Radius(),
                new Point2D(C.XAxis().Direction().XY()),
                new Point2D(C.YAxis().Direction().XY()),
                C.Location(),
                ref B);
        B.Enlarge(Tol);
    }

    /// <summary>Adds the whole ellipse C to B, then enlarges B by Tol.</summary>
    public static void Add(in Ellipse3D C, double Tol, ref Box B)
    {
        double U1 = 0.0, U2 = 2.0 * M_PI;
        Add(C, U1, U2, Tol, ref B);
    }

    /// <summary>Adds the arc [U1, U2] of the ellipse C to B, then enlarges B by Tol. U2 - U1 can be in [0, 2*pi].</summary>
    public static void Add(in Ellipse3D C, double U1, double U2, double Tol, ref Box B)
    {
        double period = 2.0 * M_PI - Epsilon(2.0 * M_PI);

        double utrim1 = U1, utrim2 = U2;
        if (U2 - U1 > period)
        {
            utrim1 = 0.0;
            utrim2 = 2.0 * M_PI;
        }
        else
        {
            double tol = Epsilon(1.0);
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, tol, ref utrim1, ref utrim2);
        }
        double MajR = C.MajorRadius();
        double MinR = C.MinorRadius();
        Xyz O = C.Location().XYZ();
        Xyz Xd = C.XAxis().Direction().XYZ();
        Xyz Yd = C.YAxis().Direction().XYZ();
        Frame3D pos = C.Position();
        //
        double tt;
        double xmin, xmax, txmin, txmax;
        if (Math.Abs(Xd.X()) > Constants.Resolution())
        {
            txmin = Math.Atan((MinR * Yd.X()) / (MajR * Xd.X()));
            txmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(txmin, 0.0, 2.0 * M_PI);
        }
        else
        {
            txmin = M_PI / 2.0;
        }
        txmax = txmin <= M_PI ? txmin + M_PI : txmin - M_PI;
        xmin = MajR * Math.Cos(txmin) * Xd.X() + MinR * Math.Sin(txmin) * Yd.X() + O.X();
        xmax = MajR * Math.Cos(txmax) * Xd.X() + MinR * Math.Sin(txmax) * Yd.X() + O.X();
        if (xmin > xmax)
        {
            tt = xmin;
            xmin = xmax;
            xmax = tt;
            tt = txmin;
            txmin = txmax;
            txmax = tt;
        }
        //
        double ymin, ymax, tymin, tymax;
        if (Math.Abs(Xd.Y()) > Constants.Resolution())
        {
            tymin = Math.Atan((MinR * Yd.Y()) / (MajR * Xd.Y()));
            tymin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tymin, 0.0, 2.0 * M_PI);
        }
        else
        {
            tymin = M_PI / 2.0;
        }
        tymax = tymin <= M_PI ? tymin + M_PI : tymin - M_PI;
        ymin = MajR * Math.Cos(tymin) * Xd.Y() + MinR * Math.Sin(tymin) * Yd.Y() + O.Y();
        ymax = MajR * Math.Cos(tymax) * Xd.Y() + MinR * Math.Sin(tymax) * Yd.Y() + O.Y();
        if (ymin > ymax)
        {
            tt = ymin;
            ymin = ymax;
            ymax = tt;
            tt = tymin;
            tymin = tymax;
            tymax = tt;
        }
        //
        double zmin, zmax, tzmin, tzmax;
        if (Math.Abs(Xd.Z()) > Constants.Resolution())
        {
            tzmin = Math.Atan((MinR * Yd.Z()) / (MajR * Xd.Z()));
            tzmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tzmin, 0.0, 2.0 * M_PI);
        }
        else
        {
            tzmin = M_PI / 2.0;
        }
        tzmax = tzmin <= M_PI ? tzmin + M_PI : tzmin - M_PI;
        zmin = MajR * Math.Cos(tzmin) * Xd.Z() + MinR * Math.Sin(tzmin) * Yd.Z() + O.Z();
        zmax = MajR * Math.Cos(tzmax) * Xd.Z() + MinR * Math.Sin(tzmax) * Yd.Z() + O.Z();
        if (zmin > zmax)
        {
            tt = zmin;
            zmin = zmax;
            zmax = tt;
            tt = tzmin;
            tzmin = tzmax;
            tzmax = tt;
        }
        //
        if (utrim2 - utrim1 >= period)
        {
            B.Update(xmin, ymin, zmin, xmax, ymax, zmax);
        }
        else
        {
            Point3D P = global::Kerf.Engine.Geometry.ElCLib.EllipseValue(utrim1, pos, MajR, MinR);
            B.Add(P);
            P = global::Kerf.Engine.Geometry.ElCLib.EllipseValue(utrim2, pos, MajR, MinR);
            B.Add(P);
            double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
            B.FinitePart().Get(out Xmin, out Ymin, out Zmin, out Xmax, out Ymax, out Zmax);
            double gap = B.GetGap();
            Xmin += gap;
            Ymin += gap;
            Zmin += gap;
            Xmax -= gap;
            Ymax -= gap;
            Zmax -= gap;
            //
            txmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(txmin, utrim1, utrim1 + 2.0 * M_PI);
            if (txmin >= utrim1 && txmin <= utrim2)
            {
                Xmin = Min(xmin, Xmin);
            }
            txmax = global::Kerf.Engine.Geometry.ElCLib.InPeriod(txmax, utrim1, utrim1 + 2.0 * M_PI);
            if (txmax >= utrim1 && txmax <= utrim2)
            {
                Xmax = Max(xmax, Xmax);
            }
            //
            tymin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tymin, utrim1, utrim1 + 2.0 * M_PI);
            if (tymin >= utrim1 && tymin <= utrim2)
            {
                Ymin = Min(ymin, Ymin);
            }
            tymax = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tymax, utrim1, utrim1 + 2.0 * M_PI);
            if (tymax >= utrim1 && tymax <= utrim2)
            {
                Ymax = Max(ymax, Ymax);
            }
            //
            tzmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tzmin, utrim1, utrim1 + 2.0 * M_PI);
            if (tzmin >= utrim1 && tzmin <= utrim2)
            {
                Zmin = Min(zmin, Zmin);
            }
            tzmax = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tzmax, utrim1, utrim1 + 2.0 * M_PI);
            if (tzmax >= utrim1 && tzmax <= utrim2)
            {
                Zmax = Max(zmax, Zmax);
            }
            //
            B.Update(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);
        }
        //
        B.Enlarge(Tol);
    }

    /// <summary>Adds the whole 2D ellipse C to B, then enlarges B by Tol.</summary>
    public static void Add(in Ellipse2D C, double Tol, ref Box2d B)
    {
        double Ra = C.MajorRadius();
        double Rb = C.MinorRadius();
        Xy Xd = C.XAxis().Direction().XY();
        Xy Yd = C.YAxis().Direction().XY();
        Xy O = C.Location().XY();
        B.Add(new Point2D(O + Ra * Xd + Rb * Yd));
        B.Add(new Point2D(O - Ra * Xd + Rb * Yd));
        B.Add(new Point2D(O - Ra * Xd - Rb * Yd));
        B.Add(new Point2D(O + Ra * Xd - Rb * Yd));
        B.Enlarge(Tol);
    }

    /// <summary>Adds the arc [P1, P2] of the 2D ellipse C to B, then enlarges B by Tol. P2 - P1 can be in [0, 2*pi].</summary>
    public static void Add(in Ellipse2D C, double P1, double P2, double Tol, ref Box2d B)
    {
        Compute(P1,
                P2,
                C.MajorRadius(),
                C.MinorRadius(),
                new Point2D(C.XAxis().Direction().XY()),
                new Point2D(C.YAxis().Direction().XY()),
                C.Location(),
                ref B);
        B.Enlarge(Tol);
    }

    /// <summary>
    /// Adds the arc [P1, P2] of the parabola P to B, then enlarges B by Tol. P1 and P2 may be infinite.
    /// </summary>
    /// <exception cref="FailureException">P1 and P2 are both negative infinite, or both positive infinite.</exception>
    public static void Add(in Parabola3D P, double P1, double P2, double Tol, ref Box B)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                B.OpenXmax();
                B.OpenYmax();
                B.OpenZmax();
            }
            else
            {
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, P));
            }
            B.OpenXmin();
            B.OpenYmin();
            B.OpenZmin();
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                B.OpenXmin();
                B.OpenYmin();
                B.OpenZmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, P));
            }
            B.OpenXmax();
            B.OpenYmax();
            B.OpenZmax();
        }
        else
        {
            B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P1, P));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                B.OpenXmin();
                B.OpenYmin();
                B.OpenZmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                B.OpenXmax();
                B.OpenYmax();
                B.OpenZmax();
            }
            else
            {
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, P));
                // OCCT: only the ends and the vertex; a turned parabola's coordinate extrema are missed (occt-issues.md)
                if (P1 * P2 < 0)
                {
                    B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, P));
                }
            }
        }
        B.Enlarge(Tol);
    }

    /// <summary>The 2D twin of <see cref="Add(in Parabola3D, double, double, double, ref Box)"/>.</summary>
    public static void Add(in Parabola2D P, double P1, double P2, double Tol, ref Box2d B)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                B.OpenXmax();
                B.OpenYmax();
            }
            else
            {
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, P));
            }
            B.OpenXmin();
            B.OpenYmin();
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                B.OpenXmin();
                B.OpenYmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, P));
            }
            B.OpenXmax();
            B.OpenYmax();
        }
        else
        {
            B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P1, P));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                B.OpenXmin();
                B.OpenYmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                B.OpenXmax();
                B.OpenYmax();
            }
            else
            {
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, P));
                // OCCT: only the ends and the vertex; a turned parabola's coordinate extrema are missed (occt-issues.md)
                if (P1 * P2 < 0)
                {
                    B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, P));
                }
            }
        }
        B.Enlarge(Tol);
    }

    /// <summary>
    /// Adds the arc [P1, P2] of the branch of the hyperbola H to B, then enlarges B by Tol. P1 and P2 may be infinite.
    /// </summary>
    /// <exception cref="FailureException">P1 and P2 are both negative infinite, or both positive infinite.</exception>
    public static void Add(in Hyperbola3D H, double P1, double P2, double Tol, ref Box B)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                B.OpenXmax();
                B.OpenYmax();
                B.OpenZmax();
            }
            else
            {
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, H));
            }
            B.OpenXmin();
            B.OpenYmin();
            B.OpenZmin();
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                B.OpenXmin();
                B.OpenYmin();
                B.OpenZmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, H));
            }
            B.OpenXmax();
            B.OpenYmax();
            B.OpenZmax();
        }
        else
        {
            B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P1, H));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                B.OpenXmin();
                B.OpenYmin();
                B.OpenZmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                B.OpenXmax();
                B.OpenYmax();
                B.OpenZmax();
            }
            else
            {
                ComputeBox(H, P1, P2, ref B);
            }
        }
        B.Enlarge(Tol);
    }

    /// <summary>
    /// Adds the arc [P1, P2] of the branch of the 2D hyperbola H to B, then enlarges B by Tol. P1 and P2 may be
    /// infinite. Unlike the 3D overload, a finite arc is bounded by its ends and its vertex only (an OCCT bug: the box
    /// can miss the arc's coordinate extrema).
    /// </summary>
    /// <exception cref="FailureException">P1 and P2 are both negative infinite, or both positive infinite.</exception>
    public static void Add(in Hyperbola2D H, double P1, double P2, double Tol, ref Box2d B)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                B.OpenXmax();
                B.OpenYmax();
            }
            else
            {
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, H));
            }
            B.OpenXmin();
            B.OpenYmin();
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                B.OpenXmin();
                B.OpenYmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, H));
            }
            B.OpenXmax();
            B.OpenYmax();
        }
        else
        {
            B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P1, H));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(P2))
            {
                B.OpenXmin();
                B.OpenYmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(P2))
            {
                B.OpenXmax();
                B.OpenYmax();
            }
            else
            {
                B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(P2, H));
                // OCCT: only the ends and the vertex, where the 3D overload adds each coordinate's extremum
                // (ComputeBox); a turned hyperbola's extrema are missed (occt-issues.md)
                if (P1 * P2 < 0)
                {
                    B.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, H));
                }
            }
        }
        B.Enlarge(Tol);
    }

    private static void ComputeCyl(in Cylinder Cyl, double UMin, double UMax, double VMin, double VMax, ref Box B)
    {
        Circle3D aC = global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(Cyl.Position(), Cyl.Radius(), VMin);
        Add(aC, UMin, UMax, 0.0, ref B);
        //
        Vector3D aT = (VMax - VMin) * new Vector3D(Cyl.Axis().Direction());
        aC.Translate(aT);
        Add(aC, UMin, UMax, 0.0, ref B);
    }

    /// <summary>
    /// Adds the patch [UMin, UMax] x [VMin, VMax] of the cylinder S to B, then enlarges B by Tol. UMax - UMin can
    /// be in [0, 2*pi]; VMin and VMax may be infinite.
    /// </summary>
    /// <exception cref="FailureException">VMin and VMax are both negative infinite, or both positive infinite.</exception>
    public static void Add(in Cylinder S, double UMin, double UMax, double VMin, double VMax, double Tol, ref Box B)
    {
        // Cache axis direction for infinite cases.
        Direction3D aDir = S.Axis().Direction();

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(VMin))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(VMax))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(VMax))
            {
                OpenMinMax(aDir, ref B);
            }
            else
            {
                ComputeCyl(S, UMin, UMax, 0.0, VMax, ref B);
                OpenMin(aDir, ref B);
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(VMin))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(VMax))
            {
                OpenMinMax(aDir, ref B);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(VMax))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                ComputeCyl(S, UMin, UMax, 0.0, VMax, ref B);
                OpenMax(aDir, ref B);
            }
        }
        else
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(VMax))
            {
                ComputeCyl(S, UMin, UMax, VMin, 0.0, ref B);
                OpenMin(aDir, ref B);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(VMax))
            {
                ComputeCyl(S, UMin, UMax, VMin, 0.0, ref B);
                OpenMax(aDir, ref B);
            }
            else
            {
                ComputeCyl(S, UMin, UMax, VMin, VMax, ref B);
            }
        }

        B.Enlarge(Tol);
    }

    /// <summary>Adds the band [VMin, VMax] of the whole cylinder S to B, then enlarges B by Tol.</summary>
    public static void Add(in Cylinder S, double VMin, double VMax, double Tol, ref Box B)
    {
        Add(S, 0.0, 2.0 * M_PI, VMin, VMax, Tol, ref B);
    }

    private static void ComputeCone(in Cone Cone, double UMin, double UMax, double VMin, double VMax, ref Box B)
    {
        CoordinateSystem aPos = Cone.Position();
        double R = Cone.RefRadius();
        double sang = Cone.SemiAngle();
        Circle3D aC = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(aPos, R, sang, VMin);
        if (aC.Radius() > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Add(aC, UMin, UMax, 0.0, ref B);
        }
        else
        {
            B.Add(aC.Location());
        }
        //
        aC = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(aPos, R, sang, VMax);
        if (aC.Radius() > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Add(aC, UMin, UMax, 0.0, ref B);
        }
        else
        {
            B.Add(aC.Location());
        }
    }

    /// <summary>
    /// Adds the patch [UMin, UMax] x [VMin, VMax] of the cone S to B, then enlarges B by Tol. UMax - UMin can be in
    /// [0, 2*pi]; VMin and VMax may be infinite.
    /// </summary>
    /// <exception cref="FailureException">VMin and VMax are both negative infinite, or both positive infinite.</exception>
    public static void Add(in Cone S, double UMin, double UMax, double VMin, double VMax, double Tol, ref Box B)
    {
        // Cache axis direction for infinite cases.
        Direction3D aD = S.Axis().Direction();

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(VMin))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(VMax))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(VMax))
            {
                OpenMinMax(aD, ref B);
            }
            else
            {
                ComputeCone(S, UMin, UMax, 0.0, VMax, ref B);
                OpenMin(aD, ref B);
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(VMin))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(VMax))
            {
                OpenMinMax(aD, ref B);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(VMax))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                ComputeCone(S, UMin, UMax, 0.0, VMax, ref B);
                OpenMax(aD, ref B);
            }
        }
        else
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(VMax))
            {
                ComputeCone(S, UMin, UMax, VMin, 0.0, ref B);
                OpenMin(aD, ref B);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(VMax))
            {
                ComputeCone(S, UMin, UMax, VMin, 0.0, ref B);
                OpenMax(aD, ref B);
            }
            else
            {
                ComputeCone(S, UMin, UMax, VMin, VMax, ref B);
            }
        }

        B.Enlarge(Tol);
    }

    /// <summary>Adds the band [VMin, VMax] of the whole cone S to B, then enlarges B by Tol.</summary>
    public static void Add(in Cone S, double VMin, double VMax, double Tol, ref Box B)
    {
        Add(S, 0.0, 2.0 * M_PI, VMin, VMax, Tol, ref B);
    }

    private static void ComputeSphere(in Sphere Sphere, double UMin, double UMax, double VMin, double VMax, ref Box B)
    {
        Point3D P = Sphere.Location();
        double R = Sphere.Radius();
        double xmin, ymin, zmin, xmax, ymax, zmax;
        xmin = P.X() - R;
        xmax = P.X() + R;
        ymin = P.Y() - R;
        ymax = P.Y() + R;
        zmin = P.Z() - R;
        zmax = P.Z() + R;

        double uper = 2.0 * M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion();
        double vper = M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (UMax - UMin >= uper && VMax - VMin >= vper)
        {
            // a whole sphere
            B.Update(xmin, ymin, zmin, xmax, ymax, zmax);
        }
        else
        {
            double u, v;
            double umax = UMin + 2.0 * M_PI;
            CoordinateSystem Pos = Sphere.Position();
            Point3D PExt = P;
            PExt.SetX(xmin);
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(Pos, R, PExt, out u, out v);
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, UMin, umax);
            if (u >= UMin && u <= UMax && v >= VMin && v <= VMax)
            {
                B.Add(PExt);
            }
            //
            PExt.SetX(xmax);
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(Pos, R, PExt, out u, out v);
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, UMin, umax);
            if (u >= UMin && u <= UMax && v >= VMin && v <= VMax)
            {
                B.Add(PExt);
            }
            PExt.SetX(P.X());
            //
            PExt.SetY(ymin);
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(Pos, R, PExt, out u, out v);
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, UMin, umax);
            if (u >= UMin && u <= UMax && v >= VMin && v <= VMax)
            {
                B.Add(PExt);
            }
            //
            PExt.SetY(ymax);
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(Pos, R, PExt, out u, out v);
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, UMin, umax);
            if (u >= UMin && u <= UMax && v >= VMin && v <= VMax)
            {
                B.Add(PExt);
            }
            PExt.SetY(P.Y());
            //
            PExt.SetZ(zmin);
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(Pos, R, PExt, out u, out v);
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, UMin, umax);
            if (u >= UMin && u <= UMax && v >= VMin && v <= VMax)
            {
                B.Add(PExt);
            }
            //
            PExt.SetZ(zmax);
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(Pos, R, PExt, out u, out v);
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, UMin, umax);
            if (u >= UMin && u <= UMax && v >= VMin && v <= VMax)
            {
                B.Add(PExt);
            }
            //
            // Add boundaries of patch
            // UMin, UMax
            Circle3D aC = global::Kerf.Engine.Geometry.ElSLib.SphereUIso(Pos, R, UMin);
            Add(aC, VMin, VMax, 0.0, ref B);
            aC = global::Kerf.Engine.Geometry.ElSLib.SphereUIso(Pos, R, UMax);
            Add(aC, VMin, VMax, 0.0, ref B);
            // VMin, VMax
            aC = global::Kerf.Engine.Geometry.ElSLib.SphereVIso(Pos, R, VMin);
            Add(aC, UMin, UMax, 0.0, ref B);
            aC = global::Kerf.Engine.Geometry.ElSLib.SphereVIso(Pos, R, VMax);
            Add(aC, UMin, UMax, 0.0, ref B);
        }
    }

    // Bounding box of a degenerated torus (major radius below the minor one).
    private static void computeDegeneratedTorus(in Torus theTorus, double theUMin, double theUMax,
                                                double theVMin, double theVMax, ref Box theB)
    {
        Point3D aP = theTorus.Location();
        double aRa = theTorus.MajorRadius();
        double aRi = theTorus.MinorRadius();
        double aXmin, anYmin, aZmin, aXmax, anYmax, aZmax;
        aXmin = aP.X() - aRa - aRi;
        aXmax = aP.X() + aRa + aRi;
        anYmin = aP.Y() - aRa - aRi;
        anYmax = aP.Y() + aRa + aRi;
        aZmin = aP.Z() - aRi;
        aZmax = aP.Z() + aRi;

        double aPhi = Math.Acos(-aRa / aRi);

        double anUper = 2.0 * M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion();
        double aVper = 2.0 * aPhi - global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (theUMax - theUMin >= anUper && theVMax - theVMin >= aVper)
        {
            // a whole torus
            theB.Update(aXmin, anYmin, aZmin, aXmax, anYmax, aZmax);
            return;
        }

        double anU, aV;
        double anUmax = theUMin + 2.0 * M_PI;
        CoordinateSystem aPos = theTorus.Position();
        Point3D aPExt = aP;
        aPExt.SetX(aXmin);
        global::Kerf.Engine.Geometry.ElSLib.TorusParameters(aPos, aRa, aRi, aPExt, out anU, out aV);
        anU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theB.Add(aPExt);
        }
        //
        aPExt.SetX(aXmax);
        global::Kerf.Engine.Geometry.ElSLib.TorusParameters(aPos, aRa, aRi, aPExt, out anU, out aV);
        anU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theB.Add(aPExt);
        }
        aPExt.SetX(aP.X());
        //
        aPExt.SetY(anYmin);
        global::Kerf.Engine.Geometry.ElSLib.TorusParameters(aPos, aRa, aRi, aPExt, out anU, out aV);
        anU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theB.Add(aPExt);
        }
        //
        aPExt.SetY(anYmax);
        global::Kerf.Engine.Geometry.ElSLib.TorusParameters(aPos, aRa, aRi, aPExt, out anU, out aV);
        anU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theB.Add(aPExt);
        }
        aPExt.SetY(aP.Y());
        //
        aPExt.SetZ(aZmin);
        global::Kerf.Engine.Geometry.ElSLib.TorusParameters(aPos, aRa, aRi, aPExt, out anU, out aV);
        anU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theB.Add(aPExt);
        }
        //
        aPExt.SetZ(aZmax);
        global::Kerf.Engine.Geometry.ElSLib.TorusParameters(aPos, aRa, aRi, aPExt, out anU, out aV);
        anU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theB.Add(aPExt);
        }
        //
        // Add boundaries of patch
        // UMin, UMax
        Circle3D aC = global::Kerf.Engine.Geometry.ElSLib.TorusUIso(aPos, aRa, aRi, theUMin);
        Add(aC, theVMin, theVMax, 0.0, ref theB);
        aC = global::Kerf.Engine.Geometry.ElSLib.TorusUIso(aPos, aRa, aRi, theUMax);
        Add(aC, theVMin, theVMax, 0.0, ref theB);
        // VMin, VMax
        aC = global::Kerf.Engine.Geometry.ElSLib.TorusVIso(aPos, aRa, aRi, theVMin);
        Add(aC, theUMin, theUMax, 0.0, ref theB);
        aC = global::Kerf.Engine.Geometry.ElSLib.TorusVIso(aPos, aRa, aRi, theVMax);
        Add(aC, theUMin, theUMax, 0.0, ref theB);
    }

    /// <summary>
    /// Adds the patch [UMin, UMax] x [VMin, VMax] of the sphere S to B, then enlarges B by Tol. UMax - UMin can be
    /// in [0, 2*pi], VMin and VMax in [-pi/2, pi/2].
    /// </summary>
    public static void Add(in Sphere S, double UMin, double UMax, double VMin, double VMax, double Tol, ref Box B)
    {
        ComputeSphere(S, UMin, UMax, VMin, VMax, ref B);
        B.Enlarge(Tol);
    }

    /// <summary>Adds the whole sphere S to B, then enlarges B by Tol.</summary>
    public static void Add(in Sphere S, double Tol, ref Box B)
    {
        Point3D P = S.Location();
        double R = S.Radius();
        double xmin, ymin, zmin, xmax, ymax, zmax;
        xmin = P.X() - R;
        xmax = P.X() + R;
        ymin = P.Y() - R;
        ymax = P.Y() + R;
        zmin = P.Z() - R;
        zmax = P.Z() + R;
        B.Update(xmin, ymin, zmin, xmax, ymax, zmax);
        B.Enlarge(Tol);
    }

    /// <summary>
    /// Adds the patch [UMin, UMax] x [VMin, VMax] of the torus S to B, then enlarges B by Tol. UMax - UMin can be
    /// in [0, 2*pi]. The box is built from 8 cross-section circles enlarged to circumscribed octagons.
    /// </summary>
    public static void Add(in Torus S, double UMin, double UMax, double VMin, double VMax, double Tol, ref Box B)
    {
        int Fi1;
        int Fi2;
        if (VMax < VMin)
        {
            Fi1 = (int)Math.Floor(VMax / (M_PI / 4.0));
            Fi2 = (int)Math.Floor(VMin / (M_PI / 4.0));
        }
        else
        {
            Fi1 = (int)Math.Floor(VMin / (M_PI / 4.0));
            Fi2 = (int)Math.Floor(VMax / (M_PI / 4.0));
        }
        Fi2++;

        double Ra = S.MajorRadius();
        double Ri = S.MinorRadius();

        if (Fi2 < Fi1)
        {
            return;
        }

        if (Ra < Ri)
        {
            computeDegeneratedTorus(S, UMin, UMax, VMin, VMax, ref B);
            B.Enlarge(Tol);
            return;
        }

        // Cache direction vectors.
        Xyz aZDir = S.Axis().Direction().XYZ();
        Xyz aLocXYZ = S.Location().XYZ();
        Point3D aXd = new(S.XAxis().Direction().XYZ());
        Point3D aYd = new(S.YAxis().Direction().XYZ());

        // Add torus cross-section points by index (0-7), at 45-degree intervals: radius Ra + Ri * aRadiusMult[i],
        // centre offset Ri * aZMult[i] along the axis (the multipliers are XMult and YMult).
        // Add points from Fi1 to Fi2, handling wrap-around for indices.
        // Use ((i % 8) + 8) % 8 to handle negative indices correctly
        // (C++ modulo can return negative values for negative dividends).
        for (int i = Fi1; i <= Fi2; ++i)
        {
            int anIdx = ((i % 8) + 8) % 8;
            double aRadius = Ra + Ri * XMult[anIdx];
            Point3D aCenter = new(aLocXYZ + (Ri * YMult[anIdx]) * aZDir);
            Compute(UMin, UMax, aRadius, aRadius, aXd, aYd, aCenter, ref B);
        }

        B.Enlarge(Tol);
    }

    /// <summary>Adds the whole torus S to B (the 8 corners of its box), then enlarges B by Tol.</summary>
    public static void Add(in Torus S, double Tol, ref Box B)
    {
        double aRMa = S.MajorRadius();
        double aRmi = S.MinorRadius();
        double aR = aRMa + aRmi;
        Xyz aO = S.Location().XYZ();
        Xyz aXd = S.XAxis().Direction().XYZ();
        Xyz aYd = S.YAxis().Direction().XYZ();
        Xyz aZd = S.Axis().Direction().XYZ();
        // Precompute scaled direction vectors.
        Xyz aRXd = aR * aXd;
        Xyz aRYd = aR * aYd;
        Xyz aRiZd = aRmi * aZd;
        // Add 8 corner points of torus bounding box.
        B.Add(new Point3D(aO - aRXd - aRYd + aRiZd));
        B.Add(new Point3D(aO - aRXd - aRYd - aRiZd));
        B.Add(new Point3D(aO + aRXd - aRYd + aRiZd));
        B.Add(new Point3D(aO + aRXd - aRYd - aRiZd));
        B.Add(new Point3D(aO - aRXd + aRYd + aRiZd));
        B.Add(new Point3D(aO - aRXd + aRYd - aRiZd));
        B.Add(new Point3D(aO + aRXd + aRYd + aRiZd));
        B.Add(new Point3D(aO + aRXd + aRYd - aRiZd));
        B.Enlarge(Tol);
    }

    // Adds the finite arc [aT1, aT2] of a hyperbola: its ends, its vertex when inside, and each coordinate's
    // extremum. Returns 0 if an extremum was found, 1 otherwise (unused by OCCT).
    private static int ComputeBox(in Hyperbola3D aHypr, double aT1, double aT2, ref Box aBox)
    {
        int i, iErr;
        double aRmaj, aRmin, aA, aB, aABP, aBAM, aT3, aCf, aEps;
        Point3D aP1, aP2, aP3, aP0;
        //
        //
        aP1 = global::Kerf.Engine.Geometry.ElCLib.Value(aT1, aHypr);
        aP2 = global::Kerf.Engine.Geometry.ElCLib.Value(aT2, aHypr);
        //
        aBox.Add(aP1);
        aBox.Add(aP2);
        //
        if (aT1 * aT2 < 0.0)
        {
            aP0 = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, aHypr);
            aBox.Add(aP0);
        }
        //
        aEps = Epsilon(1.0);
        iErr = 1;
        //
        Frame3D aPos = aHypr.Position();
        Xyz aXDir = aPos.XDirection().XYZ();
        Xyz aYDir = aPos.YDirection().XYZ();
        aRmaj = aHypr.MajorRadius();
        aRmin = aHypr.MinorRadius();
        //
        // Find extrema for each coordinate (X, Y, Z) independently.
        // Each coordinate can have its extremum at a different parameter value.
        for (i = 1; i <= 3; ++i)
        {
            aA = aRmin * aYDir.Coord(i);
            aB = aRmaj * aXDir.Coord(i);
            //
            aABP = aA + aB;
            aBAM = aB - aA;
            //
            aABP = Math.Abs(aABP);
            aBAM = Math.Abs(aBAM);
            //
            if (aABP < aEps || aBAM < aEps)
            {
                continue;
            }
            //
            aCf = aBAM / aABP;
            aT3 = 0.5 * Math.Log(aCf);
            //
            if (aT3 < aT1 || aT3 > aT2)
            {
                continue;
            }
            // Add extremum point for this coordinate.
            aP3 = global::Kerf.Engine.Geometry.ElCLib.Value(aT3, aHypr);
            aBox.Add(aP3);
            iErr = 0;
        }
        //
        return iErr;
    }
}
