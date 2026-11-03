// Port of: src/ModelingData/TKGeomBase/BndLib/BndLib.hxx
// Port of: src/ModelingData/TKGeomBase/BndLib/BndLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds;

/// <summary>
/// Adds elementary geometric primitives of gp (lines, conics, elementary surfaces), optionally limited by parameter
/// values, to a bounding box. The gp objects are parameterised as the equivalent Geom and Geom2d objects.
/// </summary>
internal static class BndLib
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

    private static void Compute(double p1, double p2, double ra, double rb,
                                in Point2D xd, in Point2D yd, in Point2D o, ref Box2d b)
    {
        double teta1;
        double teta2;
        if (p2 < p1)
        {
            teta1 = p2;
            teta2 = p1;
        }
        else
        {
            teta1 = p1;
            teta2 = p2;
        }

        double delta = Math.Abs(teta2 - teta1);
        if (delta > 2.0 * M_PI)
        {
            teta1 = 0.0;
            teta2 = 2.0 * M_PI;
        }
        else
        {
            // Normalize aTeta1 to [0, 2*PI) range
            teta1 = teta1 % (2.0 * M_PI);
            if (teta1 < 0.0)
            {
                teta1 += 2.0 * M_PI;
            }
            teta2 = teta1 + delta;
        }

        // One places already both ends
        double cn1, sn1, cn2, sn2;
        cn1 = Math.Cos(teta1);
        sn1 = Math.Sin(teta1);
        cn2 = Math.Cos(teta2);
        sn2 = Math.Sin(teta2);
        b.Add(new Point2D(o.Coord() + ra * cn1 * xd.Coord() + rb * sn1 * yd.Coord()));
        b.Add(new Point2D(o.Coord() + ra * cn2 * xd.Coord() + rb * sn2 * yd.Coord()));

        double ram, rbm;
        if (delta > M_PI / 8.0)
        {
            // Main radiuses to take into account only 8 points (/cos(Pi/8.))
            ram = ra / THE_COS_PI8;
            rbm = rb / THE_COS_PI8;
        }
        else
        {
            // Main radiuses to take into account the arrow
            double tc = Math.Cos(delta / 2);
            ram = ra / tc;
            rbm = rb / tc;
        }
        b.Add(new Point2D(o.Coord() + ram * cn1 * xd.Coord() + rbm * sn1 * yd.Coord()));
        b.Add(new Point2D(o.Coord() + ram * cn2 * xd.Coord() + rbm * sn2 * yd.Coord()));

        int deb = (int)(teta1 / (M_PI / 4.0));
        int fin = (int)(teta2 / (M_PI / 4.0));
        deb++;

        if (deb > fin)
        {
            return;
        }

        // Add polygon points from aDeb to aFin, wrapping around using modulo 8.
        for (int i = deb; i <= fin; ++i)
        {
            int idx = i % 8;
            b.Add(new Point2D(o.Coord() + ram * XMult[idx] * xd.Coord()
                                  + rbm * YMult[idx] * yd.Coord()));
        }
    }

    private static void Compute(double p1, double p2, double ra, double rb,
                                in Point3D xd, in Point3D yd, in Point3D o, ref Box b)
    {
        double teta1;
        double teta2;
        if (p2 < p1)
        {
            teta1 = p2;
            teta2 = p1;
        }
        else
        {
            teta1 = p1;
            teta2 = p2;
        }

        double delta = Math.Abs(teta2 - teta1);
        if (delta > 2.0 * M_PI)
        {
            teta1 = 0.0;
            teta2 = 2.0 * M_PI;
        }
        else
        {
            // Normalize aTeta1 to [0, 2*PI) range
            teta1 = teta1 % (2.0 * M_PI);
            if (teta1 < 0.0)
            {
                teta1 += 2.0 * M_PI;
            }
            teta2 = teta1 + delta;
        }

        // One places already both ends
        double cn1, sn1, cn2, sn2;
        cn1 = Math.Cos(teta1);
        sn1 = Math.Sin(teta1);
        cn2 = Math.Cos(teta2);
        sn2 = Math.Sin(teta2);
        b.Add(new Point3D(o.Coord() + ra * cn1 * xd.Coord() + rb * sn1 * yd.Coord()));
        b.Add(new Point3D(o.Coord() + ra * cn2 * xd.Coord() + rb * sn2 * yd.Coord()));

        double ram, rbm;
        if (delta > M_PI / 8.0)
        {
            // Main radiuses to take into account only 8 points (/cos(Pi/8.))
            ram = ra / THE_COS_PI8;
            rbm = rb / THE_COS_PI8;
        }
        else
        {
            // Main radiuses to take into account the arrow
            double tc = Math.Cos(delta / 2);
            ram = ra / tc;
            rbm = rb / tc;
        }
        b.Add(new Point3D(o.Coord() + ram * cn1 * xd.Coord() + rbm * sn1 * yd.Coord()));
        b.Add(new Point3D(o.Coord() + ram * cn2 * xd.Coord() + rbm * sn2 * yd.Coord()));

        int deb = (int)(teta1 / (M_PI / 4.0));
        int fin = (int)(teta2 / (M_PI / 4.0));
        deb++;

        if (deb > fin)
        {
            return;
        }

        // Add polygon points from aDeb to aFin, wrapping around using modulo 8.
        for (int i = deb; i <= fin; ++i)
        {
            int idx = i % 8;
            b.Add(new Point3D(o.Coord() + ram * XMult[idx] * xd.Coord()
                                + rbm * YMult[idx] * yd.Coord()));
        }
    }

    private static void OpenMin(in Direction3D v, ref Box b)
    {
        // OpenMin opens the box in the direction of decreasing parameter.
        // For a line P(t) = Origin + t*V, as t -> -Inf:
        //   - If V.X() > 0, x-coordinate -> -Inf, so open Xmin
        //   - If V.X() < 0, x-coordinate -> +Inf, so open Xmax
        if (v.IsParallel(Constants.DX(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (v.X > 0.0)
            {
                b.OpenXmin();
            }
            else
            {
                b.OpenXmax();
            }
        }
        else if (v.IsParallel(Constants.DY(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (v.Y > 0.0)
            {
                b.OpenYmin();
            }
            else
            {
                b.OpenYmax();
            }
        }
        else if (v.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (v.Z > 0.0)
            {
                b.OpenZmin();
            }
            else
            {
                b.OpenZmax();
            }
        }
        else
        {
            b.OpenXmin();
            b.OpenYmin();
            b.OpenZmin();
        }
    }

    private static void OpenMax(in Direction3D v, ref Box b)
    {
        // OpenMax opens the box in the direction of increasing parameter.
        // For a line P(t) = Origin + t*V, as t -> +Inf:
        //   - If V.X() > 0, x-coordinate -> +Inf, so open Xmax
        //   - If V.X() < 0, x-coordinate -> -Inf, so open Xmin
        if (v.IsParallel(Constants.DX(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (v.X > 0.0)
            {
                b.OpenXmax();
            }
            else
            {
                b.OpenXmin();
            }
        }
        else if (v.IsParallel(Constants.DY(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (v.Y > 0.0)
            {
                b.OpenYmax();
            }
            else
            {
                b.OpenYmin();
            }
        }
        else if (v.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (v.Z > 0.0)
            {
                b.OpenZmax();
            }
            else
            {
                b.OpenZmin();
            }
        }
        else
        {
            b.OpenXmax();
            b.OpenYmax();
            b.OpenZmax();
        }
    }

    private static void OpenMinMax(in Direction3D v, ref Box b)
    {
        if (v.IsParallel(Constants.DX(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            b.OpenXmax();
            b.OpenXmin();
        }
        else if (v.IsParallel(Constants.DY(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            b.OpenYmax();
            b.OpenYmin();
        }
        else if (v.IsParallel(Constants.DZ(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            b.OpenZmax();
            b.OpenZmin();
        }
        else
        {
            b.OpenXmin();
            b.OpenYmin();
            b.OpenZmin();
            b.OpenXmax();
            b.OpenYmax();
            b.OpenZmax();
        }
    }

    private static void OpenMin(in Direction2D v, ref Box2d b)
    {
        // OpenMin opens the box in the direction of decreasing parameter.
        // For a line P(t) = Origin + t*V, as t -> -Inf:
        //   - If V.X() > 0, x-coordinate -> -Inf, so open Xmin
        //   - If V.X() < 0, x-coordinate -> +Inf, so open Xmax
        if (v.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (v.X > 0.0)
            {
                b.OpenXmin();
            }
            else
            {
                b.OpenXmax();
            }
        }
        else if (v.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (v.Y > 0.0)
            {
                b.OpenYmin();
            }
            else
            {
                b.OpenYmax();
            }
        }
        else
        {
            b.OpenXmin();
            b.OpenYmin();
        }
    }

    private static void OpenMax(in Direction2D v, ref Box2d b)
    {
        // OpenMax opens the box in the direction of increasing parameter.
        // For a line P(t) = Origin + t*V, as t -> +Inf:
        //   - If V.X() > 0, x-coordinate -> +Inf, so open Xmax
        //   - If V.X() < 0, x-coordinate -> -Inf, so open Xmin
        if (v.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (v.X > 0.0)
            {
                b.OpenXmax();
            }
            else
            {
                b.OpenXmin();
            }
        }
        else if (v.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            if (v.Y > 0.0)
            {
                b.OpenYmax();
            }
            else
            {
                b.OpenYmin();
            }
        }
        else
        {
            b.OpenXmax();
            b.OpenYmax();
        }
    }

    private static void OpenMinMax(in Direction2D v, ref Box2d b)
    {
        if (v.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            b.OpenXmax();
            b.OpenXmin();
        }
        else if (v.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            b.OpenYmax();
            b.OpenYmin();
        }
        else
        {
            b.OpenXmin();
            b.OpenYmin();
            b.OpenXmax();
            b.OpenYmax();
        }
    }

    /// <summary>
    /// Adds the segment of the line L limited by the parameter values P1 and P2 to B, then enlarges B by Tol.
    /// P1 and P2 may be infinite.
    /// </summary>
    /// <exception cref="FailureException">P1 and P2 are both negative infinite, or both positive infinite.</exception>
    public static void Add(in Line3D l, double p1, double p2, double tol, ref Box b)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                OpenMinMax(l.Direction, ref b);
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, l));
            }
            else
            {
                OpenMin(l.Direction, ref b);
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, l));
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                OpenMinMax(l.Direction, ref b);
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, l));
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                OpenMax(l.Direction, ref b);
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, l));
            }
        }
        else
        {
            b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p1, l));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                OpenMin(l.Direction, ref b);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                OpenMax(l.Direction, ref b);
            }
            else
            {
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, l));
            }
        }
        b.Enlarge(tol);
    }

    /// <summary>The 2D twin of <see cref="Add(in Line3D, double, double, double, ref Box)"/>.</summary>
    public static void Add(in Line2D l, double p1, double p2, double tol, ref Box2d b)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                OpenMinMax(l.Direction, ref b);
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, l));
            }
            else
            {
                OpenMin(l.Direction, ref b);
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, l));
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                OpenMinMax(l.Direction, ref b);
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, l));
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                OpenMax(l.Direction, ref b);
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, l));
            }
        }
        else
        {
            b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p1, l));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                OpenMin(l.Direction, ref b);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                OpenMax(l.Direction, ref b);
            }
            else
            {
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, l));
            }
        }
        b.Enlarge(tol);
    }

    /// <summary>Adds the whole circle C to B, then enlarges B by Tol.</summary>
    public static void Add(in Circle3D c, double tol, ref Box b)
    {
        double u1 = 0.0, u2 = 2.0 * M_PI;
        Add(c, u1, u2, tol, ref b);
    }

    /// <summary>Adds the arc [U1, U2] of the circle C to B, then enlarges B by Tol. U2 - U1 can be in [0, 2*pi].</summary>
    public static void Add(in Circle3D c, double u1, double u2, double Tol, ref Box b)
    {
        double period = 2.0 * M_PI - Epsilon(2.0 * M_PI);

        double utrim1 = u1, utrim2 = u2;
        if (u2 - u1 > period)
        {
            utrim1 = 0.0;
            utrim2 = 2.0 * M_PI;
        }
        else
        {
            double tol = Epsilon(1.0);
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, tol, ref utrim1, ref utrim2);
        }
        double r = c.Radius;
        Xyz o = c.Location.XYZ();
        Xyz xd = c.XAxis.Direction.XYZ();
        Xyz yd = c.YAxis.Direction.XYZ();
        Frame3D pos = c.Position;
        //
        double tt;
        double xmin, xmax, txmin, txmax;
        if (Math.Abs(xd.X()) > Constants.Resolution())
        {
            txmin = Math.Atan(yd.X() / xd.X());
            txmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(txmin, 0.0, 2.0 * M_PI);
        }
        else
        {
            txmin = M_PI / 2.0;
        }
        txmax = txmin <= M_PI ? txmin + M_PI : txmin - M_PI;
        xmin = r * Math.Cos(txmin) * xd.X() + r * Math.Sin(txmin) * yd.X() + o.X();
        xmax = r * Math.Cos(txmax) * xd.X() + r * Math.Sin(txmax) * yd.X() + o.X();
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
        if (Math.Abs(xd.Y()) > Constants.Resolution())
        {
            tymin = Math.Atan(yd.Y() / xd.Y());
            tymin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tymin, 0.0, 2.0 * M_PI);
        }
        else
        {
            tymin = M_PI / 2.0;
        }
        tymax = tymin <= M_PI ? tymin + M_PI : tymin - M_PI;
        ymin = r * Math.Cos(tymin) * xd.Y() + r * Math.Sin(tymin) * yd.Y() + o.Y();
        ymax = r * Math.Cos(tymax) * xd.Y() + r * Math.Sin(tymax) * yd.Y() + o.Y();
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
        if (Math.Abs(xd.Z()) > Constants.Resolution())
        {
            tzmin = Math.Atan(yd.Z() / xd.Z());
            tzmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tzmin, 0.0, 2.0 * M_PI);
        }
        else
        {
            tzmin = M_PI / 2.0;
        }
        tzmax = tzmin <= M_PI ? tzmin + M_PI : tzmin - M_PI;
        zmin = r * Math.Cos(tzmin) * xd.Z() + r * Math.Sin(tzmin) * yd.Z() + o.Z();
        zmax = r * Math.Cos(tzmax) * xd.Z() + r * Math.Sin(tzmax) * yd.Z() + o.Z();
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
            b.Update(xmin, ymin, zmin, xmax, ymax, zmax);
        }
        else
        {
            Point3D p = global::Kerf.Engine.Geometry.ElCLib.CircleValue(utrim1, pos, r);
            b.Add(p);
            p = global::Kerf.Engine.Geometry.ElCLib.CircleValue(utrim2, pos, r);
            b.Add(p);
            double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
            b.FinitePart().Get(out Xmin, out Ymin, out Zmin, out Xmax, out Ymax, out Zmax);
            double gap = b.GetGap();
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
            b.Update(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);
        }
        //
        b.Enlarge(Tol);
    }

    /// <summary>Adds the whole 2D circle C to B, then enlarges B by Tol.</summary>
    public static void Add(in Circle2D c, double tol, ref Box2d b)
    {
        double r = c.Radius;
        Xy o = c.Location.XY();
        Xy xd = c.XAxis.Direction.XY();
        Xy yd = c.YAxis.Direction.XY();
        b.Add(new Point2D(o - r * xd - r * yd));
        b.Add(new Point2D(o - r * xd + r * yd));
        b.Add(new Point2D(o + r * xd - r * yd));
        b.Add(new Point2D(o + r * xd + r * yd));
        b.Enlarge(tol);
    }

    /// <summary>Adds the arc [P1, P2] of the 2D circle C to B, then enlarges B by Tol. P2 - P1 can be in [0, 2*pi].</summary>
    public static void Add(in Circle2D c, double p1, double p2, double tol, ref Box2d b)
    {
        Compute(p1,
                p2,
                c.Radius,
                c.Radius,
                new Point2D(c.XAxis.Direction.XY()),
                new Point2D(c.YAxis.Direction.XY()),
                c.Location,
                ref b);
        b.Enlarge(tol);
    }

    /// <summary>Adds the whole ellipse C to B, then enlarges B by Tol.</summary>
    public static void Add(in Ellipse3D c, double tol, ref Box b)
    {
        double u1 = 0.0, u2 = 2.0 * M_PI;
        Add(c, u1, u2, tol, ref b);
    }

    /// <summary>Adds the arc [U1, U2] of the ellipse C to B, then enlarges B by Tol. U2 - U1 can be in [0, 2*pi].</summary>
    public static void Add(in Ellipse3D c, double u1, double u2, double Tol, ref Box b)
    {
        double period = 2.0 * M_PI - Epsilon(2.0 * M_PI);

        double utrim1 = u1, utrim2 = u2;
        if (u2 - u1 > period)
        {
            utrim1 = 0.0;
            utrim2 = 2.0 * M_PI;
        }
        else
        {
            double tol = Epsilon(1.0);
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, tol, ref utrim1, ref utrim2);
        }
        double majR = c.MajorRadius;
        double minR = c.MinorRadius;
        Xyz o = c.Location.XYZ();
        Xyz xd = c.XAxis.Direction.XYZ();
        Xyz yd = c.YAxis.Direction.XYZ();
        Frame3D pos = c.Position;
        //
        double tt;
        double xmin, xmax, txmin, txmax;
        if (Math.Abs(xd.X()) > Constants.Resolution())
        {
            txmin = Math.Atan((minR * yd.X()) / (majR * xd.X()));
            txmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(txmin, 0.0, 2.0 * M_PI);
        }
        else
        {
            txmin = M_PI / 2.0;
        }
        txmax = txmin <= M_PI ? txmin + M_PI : txmin - M_PI;
        xmin = majR * Math.Cos(txmin) * xd.X() + minR * Math.Sin(txmin) * yd.X() + o.X();
        xmax = majR * Math.Cos(txmax) * xd.X() + minR * Math.Sin(txmax) * yd.X() + o.X();
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
        if (Math.Abs(xd.Y()) > Constants.Resolution())
        {
            tymin = Math.Atan((minR * yd.Y()) / (majR * xd.Y()));
            tymin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tymin, 0.0, 2.0 * M_PI);
        }
        else
        {
            tymin = M_PI / 2.0;
        }
        tymax = tymin <= M_PI ? tymin + M_PI : tymin - M_PI;
        ymin = majR * Math.Cos(tymin) * xd.Y() + minR * Math.Sin(tymin) * yd.Y() + o.Y();
        ymax = majR * Math.Cos(tymax) * xd.Y() + minR * Math.Sin(tymax) * yd.Y() + o.Y();
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
        if (Math.Abs(xd.Z()) > Constants.Resolution())
        {
            tzmin = Math.Atan((minR * yd.Z()) / (majR * xd.Z()));
            tzmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(tzmin, 0.0, 2.0 * M_PI);
        }
        else
        {
            tzmin = M_PI / 2.0;
        }
        tzmax = tzmin <= M_PI ? tzmin + M_PI : tzmin - M_PI;
        zmin = majR * Math.Cos(tzmin) * xd.Z() + minR * Math.Sin(tzmin) * yd.Z() + o.Z();
        zmax = majR * Math.Cos(tzmax) * xd.Z() + minR * Math.Sin(tzmax) * yd.Z() + o.Z();
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
            b.Update(xmin, ymin, zmin, xmax, ymax, zmax);
        }
        else
        {
            Point3D p = global::Kerf.Engine.Geometry.ElCLib.EllipseValue(utrim1, pos, majR, minR);
            b.Add(p);
            p = global::Kerf.Engine.Geometry.ElCLib.EllipseValue(utrim2, pos, majR, minR);
            b.Add(p);
            double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
            b.FinitePart().Get(out Xmin, out Ymin, out Zmin, out Xmax, out Ymax, out Zmax);
            double gap = b.GetGap();
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
            b.Update(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);
        }
        //
        b.Enlarge(Tol);
    }

    /// <summary>Adds the whole 2D ellipse C to B, then enlarges B by Tol.</summary>
    public static void Add(in Ellipse2D c, double tol, ref Box2d b)
    {
        double ra = c.MajorRadius;
        double rb = c.MinorRadius;
        Xy xd = c.XAxis.Direction.XY();
        Xy yd = c.YAxis.Direction.XY();
        Xy o = c.Location.XY();
        b.Add(new Point2D(o + ra * xd + rb * yd));
        b.Add(new Point2D(o - ra * xd + rb * yd));
        b.Add(new Point2D(o - ra * xd - rb * yd));
        b.Add(new Point2D(o + ra * xd - rb * yd));
        b.Enlarge(tol);
    }

    /// <summary>Adds the arc [P1, P2] of the 2D ellipse C to B, then enlarges B by Tol. P2 - P1 can be in [0, 2*pi].</summary>
    public static void Add(in Ellipse2D c, double p1, double p2, double tol, ref Box2d b)
    {
        Compute(p1,
                p2,
                c.MajorRadius,
                c.MinorRadius,
                new Point2D(c.XAxis.Direction.XY()),
                new Point2D(c.YAxis.Direction.XY()),
                c.Location,
                ref b);
        b.Enlarge(tol);
    }

    /// <summary>
    /// Adds the arc [P1, P2] of the parabola P to B, then enlarges B by Tol. P1 and P2 may be infinite.
    /// </summary>
    /// <exception cref="FailureException">P1 and P2 are both negative infinite, or both positive infinite.</exception>
    public static void Add(in Parabola3D p, double p1, double p2, double tol, ref Box b)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                b.OpenXmax();
                b.OpenYmax();
                b.OpenZmax();
            }
            else
            {
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, p));
            }
            b.OpenXmin();
            b.OpenYmin();
            b.OpenZmin();
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                b.OpenXmin();
                b.OpenYmin();
                b.OpenZmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, p));
            }
            b.OpenXmax();
            b.OpenYmax();
            b.OpenZmax();
        }
        else
        {
            b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p1, p));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                b.OpenXmin();
                b.OpenYmin();
                b.OpenZmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                b.OpenXmax();
                b.OpenYmax();
                b.OpenZmax();
            }
            else
            {
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, p));
                // each coordinate's extremum, where OCCT added only the vertex (decision 1429)
                Computation.ParabolaBounds.AddCoordinateExtrema(p, p1, p2, ref b);
            }
        }
        b.Enlarge(tol);
    }

    /// <summary>The 2D twin of <see cref="Add(in Parabola3D, double, double, double, ref Box)"/>.</summary>
    public static void Add(in Parabola2D p, double p1, double p2, double tol, ref Box2d b)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                b.OpenXmax();
                b.OpenYmax();
            }
            else
            {
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, p));
            }
            b.OpenXmin();
            b.OpenYmin();
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                b.OpenXmin();
                b.OpenYmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, p));
            }
            b.OpenXmax();
            b.OpenYmax();
        }
        else
        {
            b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p1, p));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                b.OpenXmin();
                b.OpenYmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                b.OpenXmax();
                b.OpenYmax();
            }
            else
            {
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, p));
                // each coordinate's extremum, where OCCT added only the vertex (decision 1429)
                Computation.Parabola2dBounds.AddCoordinateExtrema(p, p1, p2, ref b);
            }
        }
        b.Enlarge(tol);
    }

    /// <summary>
    /// Adds the arc [P1, P2] of the branch of the hyperbola H to B, then enlarges B by Tol. P1 and P2 may be infinite.
    /// </summary>
    /// <exception cref="FailureException">P1 and P2 are both negative infinite, or both positive infinite.</exception>
    public static void Add(in Hyperbola3D h, double p1, double p2, double tol, ref Box b)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                b.OpenXmax();
                b.OpenYmax();
                b.OpenZmax();
            }
            else
            {
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, h));
            }
            b.OpenXmin();
            b.OpenYmin();
            b.OpenZmin();
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                b.OpenXmin();
                b.OpenYmin();
                b.OpenZmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, h));
            }
            b.OpenXmax();
            b.OpenYmax();
            b.OpenZmax();
        }
        else
        {
            b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p1, h));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                b.OpenXmin();
                b.OpenYmin();
                b.OpenZmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                b.OpenXmax();
                b.OpenYmax();
                b.OpenZmax();
            }
            else
            {
                ComputeBox(h, p1, p2, ref b);
            }
        }
        b.Enlarge(tol);
    }

    /// <summary>
    /// Adds the arc [P1, P2] of the branch of the 2D hyperbola H to B, then enlarges B by Tol. P1 and P2 may be
    /// infinite.
    /// </summary>
    /// <exception cref="FailureException">P1 and P2 are both negative infinite, or both positive infinite.</exception>
    public static void Add(in Hyperbola2D h, double p1, double p2, double tol, ref Box2d b)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                b.OpenXmax();
                b.OpenYmax();
            }
            else
            {
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, h));
            }
            b.OpenXmin();
            b.OpenYmin();
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p1))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                b.OpenXmin();
                b.OpenYmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p2, h));
            }
            b.OpenXmax();
            b.OpenYmax();
        }
        else
        {
            b.Add(global::Kerf.Engine.Geometry.ElCLib.Value(p1, h));
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p2))
            {
                b.OpenXmin();
                b.OpenYmin();
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2))
            {
                b.OpenXmax();
                b.OpenYmax();
            }
            else
            {
                // the ends, the vertex and each coordinate's extremum, as the 3D overload (OCCT added only the ends and
                // the vertex; decision 1429)
                Computation.Hyperbola2dBounds.ComputeHyperbola2dBox(h, p1, p2, ref b);
            }
        }
        b.Enlarge(tol);
    }

    private static void ComputeCyl(in Cylinder cyl, double uMin, double uMax, double vMin, double vMax, ref Box b)
    {
        Circle3D c = global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(cyl.Position, cyl.Radius, vMin);
        Add(c, uMin, uMax, 0.0, ref b);
        //
        Vector3D t = (vMax - vMin) * new Vector3D(cyl.Axis.Direction);
        c = c.Translated(t);
        Add(c, uMin, uMax, 0.0, ref b);
    }

    /// <summary>
    /// Adds the patch [UMin, UMax] x [VMin, VMax] of the cylinder S to B, then enlarges B by Tol. UMax - UMin can
    /// be in [0, 2*pi]; VMin and VMax may be infinite.
    /// </summary>
    /// <exception cref="FailureException">VMin and VMax are both negative infinite, or both positive infinite.</exception>
    public static void Add(in Cylinder s, double uMin, double uMax, double vMin, double vMax, double tol, ref Box b)
    {
        // Cache axis direction for infinite cases.
        Direction3D dir = s.Axis.Direction;

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMin))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMax))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax))
            {
                OpenMinMax(dir, ref b);
            }
            else
            {
                ComputeCyl(s, uMin, uMax, 0.0, vMax, ref b);
                OpenMin(dir, ref b);
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMin))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMax))
            {
                OpenMinMax(dir, ref b);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                ComputeCyl(s, uMin, uMax, 0.0, vMax, ref b);
                OpenMax(dir, ref b);
            }
        }
        else
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMax))
            {
                ComputeCyl(s, uMin, uMax, vMin, 0.0, ref b);
                OpenMin(dir, ref b);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax))
            {
                ComputeCyl(s, uMin, uMax, vMin, 0.0, ref b);
                OpenMax(dir, ref b);
            }
            else
            {
                ComputeCyl(s, uMin, uMax, vMin, vMax, ref b);
            }
        }

        b.Enlarge(tol);
    }

    /// <summary>Adds the band [VMin, VMax] of the whole cylinder S to B, then enlarges B by Tol.</summary>
    public static void Add(in Cylinder s, double vMin, double vMax, double tol, ref Box b)
    {
        Add(s, 0.0, 2.0 * M_PI, vMin, vMax, tol, ref b);
    }

    private static void ComputeCone(in Cone cone, double uMin, double uMax, double vMin, double vMax, ref Box b)
    {
        CoordinateSystem pos = cone.Position;
        double r = cone.RefRadius;
        double sang = cone.SemiAngle;
        Circle3D c = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(pos, r, sang, vMin);
        if (c.Radius > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Add(c, uMin, uMax, 0.0, ref b);
        }
        else
        {
            b.Add(c.Location);
        }
        //
        c = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(pos, r, sang, vMax);
        if (c.Radius > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Add(c, uMin, uMax, 0.0, ref b);
        }
        else
        {
            b.Add(c.Location);
        }
    }

    /// <summary>
    /// Adds the patch [UMin, UMax] x [VMin, VMax] of the cone S to B, then enlarges B by Tol. UMax - UMin can be in
    /// [0, 2*pi]; VMin and VMax may be infinite.
    /// </summary>
    /// <exception cref="FailureException">VMin and VMax are both negative infinite, or both positive infinite.</exception>
    public static void Add(in Cone s, double uMin, double uMax, double vMin, double vMax, double tol, ref Box b)
    {
        // Cache axis direction for infinite cases.
        Direction3D d = s.Axis.Direction;

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMin))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMax))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax))
            {
                OpenMinMax(d, ref b);
            }
            else
            {
                ComputeCone(s, uMin, uMax, 0.0, vMax, ref b);
                OpenMin(d, ref b);
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMin))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMax))
            {
                OpenMinMax(d, ref b);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax))
            {
                throw new FailureException("BndLib::bad parameter");
            }
            else
            {
                ComputeCone(s, uMin, uMax, 0.0, vMax, ref b);
                OpenMax(d, ref b);
            }
        }
        else
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMax))
            {
                ComputeCone(s, uMin, uMax, vMin, 0.0, ref b);
                OpenMin(d, ref b);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax))
            {
                ComputeCone(s, uMin, uMax, vMin, 0.0, ref b);
                OpenMax(d, ref b);
            }
            else
            {
                ComputeCone(s, uMin, uMax, vMin, vMax, ref b);
            }
        }

        b.Enlarge(tol);
    }

    /// <summary>Adds the band [VMin, VMax] of the whole cone S to B, then enlarges B by Tol.</summary>
    public static void Add(in Cone s, double vMin, double vMax, double tol, ref Box b)
    {
        Add(s, 0.0, 2.0 * M_PI, vMin, vMax, tol, ref b);
    }

    private static void ComputeSphere(in Sphere sphere, double uMin, double uMax, double vMin, double vMax, ref Box b)
    {
        Point3D p = sphere.Location;
        double r = sphere.Radius;
        double xmin, ymin, zmin, xmax, ymax, zmax;
        xmin = p.X - r;
        xmax = p.X + r;
        ymin = p.Y - r;
        ymax = p.Y + r;
        zmin = p.Z - r;
        zmax = p.Z + r;

        double uper = 2.0 * M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion();
        double vper = M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (uMax - uMin >= uper && vMax - vMin >= vper)
        {
            // a whole sphere
            b.Update(xmin, ymin, zmin, xmax, ymax, zmax);
        }
        else
        {
            double u, v;
            double umax = uMin + 2.0 * M_PI;
            CoordinateSystem pos = sphere.Position;
            Point3D pExt = p;
            pExt = pExt.WithX(xmin);
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(pos, r, pExt, out u, out v);
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
            if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
            {
                b.Add(pExt);
            }
            //
            pExt = pExt.WithX(xmax);
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(pos, r, pExt, out u, out v);
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
            if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
            {
                b.Add(pExt);
            }
            pExt = pExt.WithX(p.X);
            //
            pExt = pExt.WithY(ymin);
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(pos, r, pExt, out u, out v);
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
            if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
            {
                b.Add(pExt);
            }
            //
            pExt = pExt.WithY(ymax);
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(pos, r, pExt, out u, out v);
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
            if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
            {
                b.Add(pExt);
            }
            pExt = pExt.WithY(p.Y);
            //
            pExt = pExt.WithZ(zmin);
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(pos, r, pExt, out u, out v);
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
            if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
            {
                b.Add(pExt);
            }
            //
            pExt = pExt.WithZ(zmax);
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(pos, r, pExt, out u, out v);
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
            if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
            {
                b.Add(pExt);
            }
            //
            // Add boundaries of patch
            // UMin, UMax
            Circle3D c = global::Kerf.Engine.Geometry.ElSLib.SphereUIso(pos, r, uMin);
            Add(c, vMin, vMax, 0.0, ref b);
            c = global::Kerf.Engine.Geometry.ElSLib.SphereUIso(pos, r, uMax);
            Add(c, vMin, vMax, 0.0, ref b);
            // VMin, VMax
            c = global::Kerf.Engine.Geometry.ElSLib.SphereVIso(pos, r, vMin);
            Add(c, uMin, uMax, 0.0, ref b);
            c = global::Kerf.Engine.Geometry.ElSLib.SphereVIso(pos, r, vMax);
            Add(c, uMin, uMax, 0.0, ref b);
        }
    }

    // Bounding box of a degenerated torus (major radius below the minor one).
    private static void ComputeDegeneratedTorus(in Torus torus, double uMin, double uMax,
                                                double vMin, double vMax, ref Box b)
    {
        Point3D p = torus.Location;
        double ra = torus.MajorRadius;
        double ri = torus.MinorRadius;
        double xmin, ymin, zmin, xmax, ymax, zmax;
        xmin = p.X - ra - ri;
        xmax = p.X + ra + ri;
        ymin = p.Y - ra - ri;
        ymax = p.Y + ra + ri;
        zmin = p.Z - ri;
        zmax = p.Z + ri;

        double phi = Math.Acos(-ra / ri);

        double uper = 2.0 * M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion();
        double vper = 2.0 * phi - global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (uMax - uMin >= uper && vMax - vMin >= vper)
        {
            // a whole torus
            b.Update(xmin, ymin, zmin, xmax, ymax, zmax);
            return;
        }

        double u, v;
        double umax = uMin + 2.0 * M_PI;
        CoordinateSystem pos = torus.Position;
        Point3D pExt = p;
        pExt = pExt.WithX(xmin);
        global::Kerf.Engine.Geometry.ElSLib.TorusParameters(pos, ra, ri, pExt, out u, out v);
        u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
        if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
        {
            b.Add(pExt);
        }
        //
        pExt = pExt.WithX(xmax);
        global::Kerf.Engine.Geometry.ElSLib.TorusParameters(pos, ra, ri, pExt, out u, out v);
        u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
        if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
        {
            b.Add(pExt);
        }
        pExt = pExt.WithX(p.X);
        //
        pExt = pExt.WithY(ymin);
        global::Kerf.Engine.Geometry.ElSLib.TorusParameters(pos, ra, ri, pExt, out u, out v);
        u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
        if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
        {
            b.Add(pExt);
        }
        //
        pExt = pExt.WithY(ymax);
        global::Kerf.Engine.Geometry.ElSLib.TorusParameters(pos, ra, ri, pExt, out u, out v);
        u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
        if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
        {
            b.Add(pExt);
        }
        pExt = pExt.WithY(p.Y);
        //
        pExt = pExt.WithZ(zmin);
        global::Kerf.Engine.Geometry.ElSLib.TorusParameters(pos, ra, ri, pExt, out u, out v);
        u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
        if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
        {
            b.Add(pExt);
        }
        //
        pExt = pExt.WithZ(zmax);
        global::Kerf.Engine.Geometry.ElSLib.TorusParameters(pos, ra, ri, pExt, out u, out v);
        u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
        if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
        {
            b.Add(pExt);
        }
        //
        // Add boundaries of patch
        // UMin, UMax
        Circle3D c = global::Kerf.Engine.Geometry.ElSLib.TorusUIso(pos, ra, ri, uMin);
        Add(c, vMin, vMax, 0.0, ref b);
        c = global::Kerf.Engine.Geometry.ElSLib.TorusUIso(pos, ra, ri, uMax);
        Add(c, vMin, vMax, 0.0, ref b);
        // VMin, VMax
        c = global::Kerf.Engine.Geometry.ElSLib.TorusVIso(pos, ra, ri, vMin);
        Add(c, uMin, uMax, 0.0, ref b);
        c = global::Kerf.Engine.Geometry.ElSLib.TorusVIso(pos, ra, ri, vMax);
        Add(c, uMin, uMax, 0.0, ref b);
    }

    /// <summary>
    /// Adds the patch [UMin, UMax] x [VMin, VMax] of the sphere S to B, then enlarges B by Tol. UMax - UMin can be
    /// in [0, 2*pi], VMin and VMax in [-pi/2, pi/2].
    /// </summary>
    public static void Add(in Sphere s, double uMin, double uMax, double vMin, double vMax, double tol, ref Box b)
    {
        ComputeSphere(s, uMin, uMax, vMin, vMax, ref b);
        b.Enlarge(tol);
    }

    /// <summary>Adds the whole sphere S to B, then enlarges B by Tol.</summary>
    public static void Add(in Sphere s, double tol, ref Box b)
    {
        Point3D p = s.Location;
        double r = s.Radius;
        double xmin, ymin, zmin, xmax, ymax, zmax;
        xmin = p.X - r;
        xmax = p.X + r;
        ymin = p.Y - r;
        ymax = p.Y + r;
        zmin = p.Z - r;
        zmax = p.Z + r;
        b.Update(xmin, ymin, zmin, xmax, ymax, zmax);
        b.Enlarge(tol);
    }

    /// <summary>
    /// Adds the patch [UMin, UMax] x [VMin, VMax] of the torus S to B, then enlarges B by Tol. UMax - UMin can be
    /// in [0, 2*pi]. The box is built from 8 cross-section circles enlarged to circumscribed octagons.
    /// </summary>
    public static void Add(in Torus s, double uMin, double uMax, double vMin, double vMax, double tol, ref Box b)
    {
        int fi1;
        int fi2;
        if (vMax < vMin)
        {
            fi1 = (int)Math.Floor(vMax / (M_PI / 4.0));
            fi2 = (int)Math.Floor(vMin / (M_PI / 4.0));
        }
        else
        {
            fi1 = (int)Math.Floor(vMin / (M_PI / 4.0));
            fi2 = (int)Math.Floor(vMax / (M_PI / 4.0));
        }
        fi2++;

        double ra = s.MajorRadius;
        double ri = s.MinorRadius;

        if (fi2 < fi1)
        {
            return;
        }

        if (ra < ri)
        {
            ComputeDegeneratedTorus(s, uMin, uMax, vMin, vMax, ref b);
            b.Enlarge(tol);
            return;
        }

        // Cache direction vectors.
        Xyz zDir = s.Axis.Direction.XYZ();
        Xyz locXYZ = s.Location.XYZ();
        Point3D xd = new(s.XAxis.Direction.XYZ());
        Point3D yd = new(s.YAxis.Direction.XYZ());

        // Add torus cross-section points by index (0-7), at 45-degree intervals: radius Ra + Ri * aRadiusMult[i],
        // centre offset Ri * aZMult[i] along the axis (the multipliers are XMult and YMult).
        // Add points from Fi1 to Fi2, handling wrap-around for indices.
        // Use ((i % 8) + 8) % 8 to handle negative indices correctly
        // (C++ modulo can return negative values for negative dividends).
        for (int i = fi1; i <= fi2; ++i)
        {
            int idx = ((i % 8) + 8) % 8;
            double radius = ra + ri * XMult[idx];
            Point3D center = new(locXYZ + (ri * YMult[idx]) * zDir);
            Compute(uMin, uMax, radius, radius, xd, yd, center, ref b);
        }

        b.Enlarge(tol);
    }

    /// <summary>Adds the whole torus S to B (the 8 corners of its box), then enlarges B by Tol.</summary>
    public static void Add(in Torus s, double tol, ref Box b)
    {
        double rMa = s.MajorRadius;
        double rmi = s.MinorRadius;
        double r = rMa + rmi;
        Xyz o = s.Location.XYZ();
        Xyz xd = s.XAxis.Direction.XYZ();
        Xyz yd = s.YAxis.Direction.XYZ();
        Xyz zd = s.Axis.Direction.XYZ();
        // Precompute scaled direction vectors.
        Xyz rXd = r * xd;
        Xyz rYd = r * yd;
        Xyz riZd = rmi * zd;
        // Add 8 corner points of torus bounding box.
        b.Add(new Point3D(o - rXd - rYd + riZd));
        b.Add(new Point3D(o - rXd - rYd - riZd));
        b.Add(new Point3D(o + rXd - rYd + riZd));
        b.Add(new Point3D(o + rXd - rYd - riZd));
        b.Add(new Point3D(o - rXd + rYd + riZd));
        b.Add(new Point3D(o - rXd + rYd - riZd));
        b.Add(new Point3D(o + rXd + rYd + riZd));
        b.Add(new Point3D(o + rXd + rYd - riZd));
        b.Enlarge(tol);
    }

    // Adds the finite arc [aT1, aT2] of a hyperbola: its ends, its vertex when inside, and each coordinate's
    // extremum. Returns 0 if an extremum was found, 1 otherwise (unused by OCCT).
    private static int ComputeBox(in Hyperbola3D aHypr, double aT1, double aT2, ref Box aBox)
    {
        int i, iErr;
        double rmaj, rmin, a, b, abp, bam, t3, cf, eps;
        Point3D p1, p2, p3, p0;
        //
        //
        p1 = global::Kerf.Engine.Geometry.ElCLib.Value(aT1, aHypr);
        p2 = global::Kerf.Engine.Geometry.ElCLib.Value(aT2, aHypr);
        //
        aBox.Add(p1);
        aBox.Add(p2);
        //
        if (aT1 * aT2 < 0.0)
        {
            p0 = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, aHypr);
            aBox.Add(p0);
        }
        //
        eps = Epsilon(1.0);
        iErr = 1;
        //
        Frame3D pos = aHypr.Position;
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        rmaj = aHypr.MajorRadius;
        rmin = aHypr.MinorRadius;
        //
        // Find extrema for each coordinate (X, Y, Z) independently.
        // Each coordinate can have its extremum at a different parameter value.
        for (i = 1; i <= 3; ++i)
        {
            a = rmin * yDir.Coord(i);
            b = rmaj * xDir.Coord(i);
            //
            abp = a + b;
            bam = b - a;
            //
            abp = Math.Abs(abp);
            bam = Math.Abs(bam);
            //
            if (abp < eps || bam < eps)
            {
                continue;
            }
            //
            cf = bam / abp;
            t3 = 0.5 * Math.Log(cf);
            //
            if (t3 < aT1 || t3 > aT2)
            {
                continue;
            }
            // Add extremum point for this coordinate.
            p3 = global::Kerf.Engine.Geometry.ElCLib.Value(t3, aHypr);
            aBox.Add(p3);
            iErr = 0;
        }
        //
        return iErr;
    }
}
