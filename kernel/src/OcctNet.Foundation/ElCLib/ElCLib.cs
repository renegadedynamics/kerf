// Port of: src/FoundationClasses/TKMath/ElCLib/ElCLib.hxx
// Port of: src/FoundationClasses/TKMath/ElCLib/ElCLib.lxx
// Port of: src/FoundationClasses/TKMath/ElCLib/ElCLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Elementary Curves Library: points and derivatives of lines and conics (2D and 3D) at a parameter, the parameter
/// of a point on such a curve, placing parameters within the period of a curve, and building 3D curves from 2D
/// ones in a coordinate system. Each function exists for a gp curve and for the curve given by its position and
/// dimensions.
/// </summary>
/// <remarks>
/// Parametrisations, in the local coordinate system of the curve:
/// line <c>P(U) = O + U * Dir</c>;
/// circle <c>X(U) = Radius * cos(U), Y(U) = Radius * sin(U)</c>;
/// ellipse <c>X(U) = MajorRadius * cos(U), Y(U) = MinorRadius * sin(U)</c>;
/// hyperbola <c>X(U) = MajorRadius * ch(U), Y(U) = MinorRadius * sh(U)</c>;
/// parabola <c>X(U) = U * U / (4 * Focal), Y(U) = U</c>.
/// Output points and vectors (OCCT's <c>gp_Pnt&amp;</c>, <c>gp_Vec&amp;</c>) are <c>out</c> parameters.
/// </remarks>
public static class ElCLib
{
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

    /// <summary>
    /// U brought into [UFirst, ULast) by adding or removing the period ULast - UFirst. ULast is expected, but not
    /// checked, to be greater than UFirst.
    /// </summary>
    /// <remarks>
    /// ULast is never returned. Because the period is computed as ULast - UFirst, rounding can make it differ
    /// slightly from the curve's true period; use with care.
    /// </remarks>
    public static double InPeriod(double theU, double theUFirst, double theULast)
    {
        if (Precision.IsInfinite(theU) || Precision.IsInfinite(theUFirst)
            || Precision.IsInfinite(theULast))
        { // In order to avoid FLT_Overflow exception
            return theU;
        }

        double aPeriod = theULast - theUFirst;
        if (aPeriod < Epsilon(theULast))
        {
            return theU;
        }

        return Max(theUFirst, theU + aPeriod * Math.Ceiling((theUFirst - theU) / aPeriod));
    }

    /// <summary>
    /// Adjusts U1 and U2 to the range [UFirst, ULast] of a periodic curve of period ULast - UFirst: U1 is brought
    /// into [UFirst, ULast], then U2 into [U1, U1 + period]. Preci is used to test the equalities.
    /// </summary>
    public static void AdjustPeriodic(double UFirst, double ULast, double Preci, ref double U1, ref double U2)
    {
        if (Precision.IsInfinite(UFirst) || Precision.IsInfinite(ULast))
        {
            U1 = UFirst;
            U2 = ULast;
            return;
        }

        double aPeriod = ULast - UFirst;
        if (aPeriod < Epsilon(ULast))
        {
            // In order to avoid FLT_Overflow exception
            // (test bugs moddata_1 bug22757)
            U1 = UFirst;
            U2 = ULast;
            return;
        }

        U1 -= Math.Floor((U1 - UFirst) / aPeriod) * aPeriod;
        if (ULast - U1 < Preci)
        {
            U1 -= aPeriod;
        }

        U2 -= Math.Floor((U2 - U1) / aPeriod) * aPeriod;
        if (U2 - U1 < Preci)
        {
            U2 += aPeriod;
        }
    }

    // ElCLib.lxx: the gp curve overloads

    /// <summary>The point of parameter U on a line or conic.</summary>
    public static gp_Pnt Value(double U, in gp_Lin L) => LineValue(U, L.Position());

    public static gp_Pnt Value(double U, in gp_Circ C) => CircleValue(U, C.Position(), C.Radius());

    public static gp_Pnt Value(double U, in gp_Elips E) => EllipseValue(U, E.Position(), E.MajorRadius(), E.MinorRadius());

    public static gp_Pnt Value(double U, in gp_Hypr H) => HyperbolaValue(U, H.Position(), H.MajorRadius(), H.MinorRadius());

    public static gp_Pnt Value(double U, in gp_Parab Prb) => ParabolaValue(U, Prb.Position(), Prb.Focal());

    /// <summary>The point P of parameter U and the first derivative V1.</summary>
    public static void D1(double U, in gp_Lin L, out gp_Pnt P, out gp_Vec V1) => LineD1(U, L.Position(), out P, out V1);

    public static void D1(double U, in gp_Circ C, out gp_Pnt P, out gp_Vec V1)
        => CircleD1(U, C.Position(), C.Radius(), out P, out V1);

    public static void D1(double U, in gp_Elips E, out gp_Pnt P, out gp_Vec V1)
        => EllipseD1(U, E.Position(), E.MajorRadius(), E.MinorRadius(), out P, out V1);

    public static void D1(double U, in gp_Hypr H, out gp_Pnt P, out gp_Vec V1)
        => HyperbolaD1(U, H.Position(), H.MajorRadius(), H.MinorRadius(), out P, out V1);

    public static void D1(double U, in gp_Parab Prb, out gp_Pnt P, out gp_Vec V1)
        => ParabolaD1(U, Prb.Position(), Prb.Focal(), out P, out V1);

    /// <summary>The point P of parameter U and the first and second derivatives V1 and V2.</summary>
    public static void D2(double U, in gp_Circ C, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2)
        => CircleD2(U, C.Position(), C.Radius(), out P, out V1, out V2);

    public static void D2(double U, in gp_Elips E, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2)
        => EllipseD2(U, E.Position(), E.MajorRadius(), E.MinorRadius(), out P, out V1, out V2);

    public static void D2(double U, in gp_Hypr H, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2)
        => HyperbolaD2(U, H.Position(), H.MajorRadius(), H.MinorRadius(), out P, out V1, out V2);

    public static void D2(double U, in gp_Parab Prb, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2)
        => ParabolaD2(U, Prb.Position(), Prb.Focal(), out P, out V1, out V2);

    /// <summary>The point P of parameter U and the first, second and third derivatives V1, V2 and V3.</summary>
    public static void D3(double U, in gp_Circ C, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2, out gp_Vec V3)
        => CircleD3(U, C.Position(), C.Radius(), out P, out V1, out V2, out V3);

    public static void D3(double U, in gp_Elips E, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2, out gp_Vec V3)
        => EllipseD3(U, E.Position(), E.MajorRadius(), E.MinorRadius(), out P, out V1, out V2, out V3);

    public static void D3(double U, in gp_Hypr H, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2, out gp_Vec V3)
        => HyperbolaD3(U, H.Position(), H.MajorRadius(), H.MinorRadius(), out P, out V1, out V2, out V3);

    /// <summary>The derivative of order N (greater than 0) at the point of parameter U.</summary>
    public static gp_Vec DN(double U, in gp_Lin L, int N) => LineDN(U, L.Position(), N);

    public static gp_Vec DN(double U, in gp_Circ C, int N) => CircleDN(U, C.Position(), C.Radius(), N);

    public static gp_Vec DN(double U, in gp_Elips E, int N)
        => EllipseDN(U, E.Position(), E.MajorRadius(), E.MinorRadius(), N);

    public static gp_Vec DN(double U, in gp_Hypr H, int N)
        => HyperbolaDN(U, H.Position(), H.MajorRadius(), H.MinorRadius(), N);

    public static gp_Vec DN(double U, in gp_Parab Prb, int N) => ParabolaDN(U, Prb.Position(), Prb.Focal(), N);

    public static gp_Pnt2d Value(double U, in gp_Lin2d L) => LineValue(U, L.Position());

    public static gp_Pnt2d Value(double U, in gp_Circ2d C) => CircleValue(U, C.Axis(), C.Radius());

    public static gp_Pnt2d Value(double U, in gp_Elips2d E) => EllipseValue(U, E.Axis(), E.MajorRadius(), E.MinorRadius());

    public static gp_Pnt2d Value(double U, in gp_Hypr2d H) => HyperbolaValue(U, H.Axis(), H.MajorRadius(), H.MinorRadius());

    public static gp_Pnt2d Value(double U, in gp_Parab2d Prb) => ParabolaValue(U, Prb.Axis(), Prb.Focal());

    public static void D1(double U, in gp_Lin2d L, out gp_Pnt2d P, out gp_Vec2d V1) => LineD1(U, L.Position(), out P, out V1);

    public static void D1(double U, in gp_Circ2d C, out gp_Pnt2d P, out gp_Vec2d V1)
        => CircleD1(U, C.Axis(), C.Radius(), out P, out V1);

    public static void D1(double U, in gp_Elips2d E, out gp_Pnt2d P, out gp_Vec2d V1)
        => EllipseD1(U, E.Axis(), E.MajorRadius(), E.MinorRadius(), out P, out V1);

    public static void D1(double U, in gp_Hypr2d H, out gp_Pnt2d P, out gp_Vec2d V1)
        => HyperbolaD1(U, H.Axis(), H.MajorRadius(), H.MinorRadius(), out P, out V1);

    public static void D1(double U, in gp_Parab2d Prb, out gp_Pnt2d P, out gp_Vec2d V1)
        => ParabolaD1(U, Prb.Axis(), Prb.Focal(), out P, out V1);

    public static void D2(double U, in gp_Circ2d C, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2)
        => CircleD2(U, C.Axis(), C.Radius(), out P, out V1, out V2);

    public static void D2(double U, in gp_Elips2d E, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2)
        => EllipseD2(U, E.Axis(), E.MajorRadius(), E.MinorRadius(), out P, out V1, out V2);

    public static void D2(double U, in gp_Hypr2d H, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2)
        => HyperbolaD2(U, H.Axis(), H.MajorRadius(), H.MinorRadius(), out P, out V1, out V2);

    public static void D2(double U, in gp_Parab2d Prb, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2)
        => ParabolaD2(U, Prb.Axis(), Prb.Focal(), out P, out V1, out V2);

    public static void D3(double U, in gp_Circ2d C, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2, out gp_Vec2d V3)
        => CircleD3(U, C.Axis(), C.Radius(), out P, out V1, out V2, out V3);

    public static void D3(double U, in gp_Elips2d E, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2, out gp_Vec2d V3)
        => EllipseD3(U, E.Axis(), E.MajorRadius(), E.MinorRadius(), out P, out V1, out V2, out V3);

    public static void D3(double U, in gp_Hypr2d H, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2, out gp_Vec2d V3)
        => HyperbolaD3(U, H.Axis(), H.MajorRadius(), H.MinorRadius(), out P, out V1, out V2, out V3);

    public static gp_Vec2d DN(double U, in gp_Lin2d L, int N) => LineDN(U, L.Position(), N);

    public static gp_Vec2d DN(double U, in gp_Circ2d C, int N) => CircleDN(U, C.Axis(), C.Radius(), N);

    public static gp_Vec2d DN(double U, in gp_Elips2d E, int N)
        => EllipseDN(U, E.Axis(), E.MajorRadius(), E.MinorRadius(), N);

    public static gp_Vec2d DN(double U, in gp_Hypr2d H, int N)
        => HyperbolaDN(U, H.Axis(), H.MajorRadius(), H.MinorRadius(), N);

    public static gp_Vec2d DN(double U, in gp_Parab2d Prb, int N) => ParabolaDN(U, Prb.Axis(), Prb.Focal(), N);

    /// <summary>
    /// The parameter of a point on a curve. The point must be on the curve; these functions don't check it, and
    /// may throw if it is not.
    /// </summary>
    public static double Parameter(in gp_Lin L, in gp_Pnt P) => LineParameter(L.Position(), P);

    public static double Parameter(in gp_Lin2d L, in gp_Pnt2d P) => LineParameter(L.Position(), P);

    public static double Parameter(in gp_Circ C, in gp_Pnt P) => CircleParameter(C.Position(), P);

    public static double Parameter(in gp_Circ2d C, in gp_Pnt2d P) => CircleParameter(C.Axis(), P);

    public static double Parameter(in gp_Elips E, in gp_Pnt P)
        => EllipseParameter(E.Position(), E.MajorRadius(), E.MinorRadius(), P);

    public static double Parameter(in gp_Elips2d E, in gp_Pnt2d P)
        => EllipseParameter(E.Axis(), E.MajorRadius(), E.MinorRadius(), P);

    public static double Parameter(in gp_Hypr H, in gp_Pnt P)
        => HyperbolaParameter(H.Position(), H.MajorRadius(), H.MinorRadius(), P);

    public static double Parameter(in gp_Hypr2d H, in gp_Pnt2d P)
        => HyperbolaParameter(H.Axis(), H.MajorRadius(), H.MinorRadius(), P);

    public static double Parameter(in gp_Parab Prb, in gp_Pnt P) => ParabolaParameter(Prb.Position(), P);

    public static double Parameter(in gp_Parab2d Prb, in gp_Pnt2d P) => ParabolaParameter(Prb.Axis(), P);

    // ElCLib.cxx: curves given by their geometric characteristics

    public static gp_Pnt LineValue(double U, in gp_Ax1 Pos)
    {
        gp_XYZ ZDir = Pos.Direction().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        return new gp_Pnt(U * ZDir.X() + PLoc.X(), U * ZDir.Y() + PLoc.Y(), U * ZDir.Z() + PLoc.Z());
    }

    public static gp_Pnt CircleValue(double U, in gp_Ax2 Pos, double Radius)
    {
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double A1 = Radius * Math.Cos(U);
        double A2 = Radius * Math.Sin(U);
        return new gp_Pnt(A1 * XDir.X() + A2 * YDir.X() + PLoc.X(),
                          A1 * XDir.Y() + A2 * YDir.Y() + PLoc.Y(),
                          A1 * XDir.Z() + A2 * YDir.Z() + PLoc.Z());
    }

    public static gp_Pnt EllipseValue(double U, in gp_Ax2 Pos, double MajorRadius, double MinorRadius)
    {
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double A1 = MajorRadius * Math.Cos(U);
        double A2 = MinorRadius * Math.Sin(U);
        return new gp_Pnt(A1 * XDir.X() + A2 * YDir.X() + PLoc.X(),
                          A1 * XDir.Y() + A2 * YDir.Y() + PLoc.Y(),
                          A1 * XDir.Z() + A2 * YDir.Z() + PLoc.Z());
    }

    public static gp_Pnt HyperbolaValue(double U, in gp_Ax2 Pos, double MajorRadius, double MinorRadius)
    {
        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double A1 = MajorRadius * Math.Cosh(U);
        double A2 = MinorRadius * Math.Sinh(U);
        return new gp_Pnt(A1 * XDir.X() + A2 * YDir.X() + PLoc.X(),
                          A1 * XDir.Y() + A2 * YDir.Y() + PLoc.Y(),
                          A1 * XDir.Z() + A2 * YDir.Z() + PLoc.Z());
    }

    /// <summary>The point of parameter U on the parabola; a parabola of focal length below gp::Resolution() is a line.</summary>
    public static gp_Pnt ParabolaValue(double U, in gp_Ax2 Pos, double Focal)
    {
        if (Math.Abs(Focal) <= gp.Resolution())
        {
            gp_XYZ XDir0 = Pos.XDirection().XYZ();
            gp_XYZ PLoc0 = Pos.Location().XYZ();
            return new gp_Pnt(U * XDir0.X() + PLoc0.X(), U * XDir0.Y() + PLoc0.Y(), U * XDir0.Z() + PLoc0.Z());
        }

        gp_XYZ XDir = Pos.XDirection().XYZ();
        gp_XYZ YDir = Pos.YDirection().XYZ();
        gp_XYZ PLoc = Pos.Location().XYZ();
        double A1 = U * U / (4.0 * Focal);
        return new gp_Pnt(A1 * XDir.X() + U * YDir.X() + PLoc.X(),
                          A1 * XDir.Y() + U * YDir.Y() + PLoc.Y(),
                          A1 * XDir.Z() + U * YDir.Z() + PLoc.Z());
    }

    public static void LineD1(double U, in gp_Ax1 Pos, out gp_Pnt P, out gp_Vec V1)
    {
        gp_XYZ Coord = Pos.Direction().XYZ();
        V1 = new gp_Vec(Coord);
        Coord.SetLinearForm(U, Coord, Pos.Location().XYZ());
        P = new gp_Pnt(Coord);
    }

    public static void CircleD1(double U, in gp_Ax2 Pos, double Radius, out gp_Pnt P, out gp_Vec V1)
    {
        double Xc = Radius * Math.Cos(U);
        double Yc = Radius * Math.Sin(U);
        gp_XYZ Coord1 = Pos.XDirection().XYZ();
        gp_XYZ Coord2 = Pos.YDirection().XYZ();
        gp_XYZ Coord0 = new gp_XYZ();
        // Point courant :
        Coord0.SetLinearForm(Xc, Coord1, Yc, Coord2, Pos.Location().XYZ());
        P = new gp_Pnt(Coord0);
        // D1 :
        Coord0.SetLinearForm(-Yc, Coord1, Xc, Coord2);
        V1 = new gp_Vec(Coord0);
    }

    public static void EllipseD1(double U, in gp_Ax2 Pos, double MajorRadius, double MinorRadius,
                                 out gp_Pnt P, out gp_Vec V1)
    {
        double Xc = Math.Cos(U);
        double Yc = Math.Sin(U);
        gp_XYZ Coord1 = Pos.XDirection().XYZ();
        gp_XYZ Coord2 = Pos.YDirection().XYZ();
        gp_XYZ Coord0 = new gp_XYZ();
        // Point courant :
        Coord0.SetLinearForm(Xc * MajorRadius, Coord1, Yc * MinorRadius, Coord2, Pos.Location().XYZ());
        P = new gp_Pnt(Coord0);
        // D1 :
        Coord0.SetLinearForm(-Yc * MajorRadius, Coord1, Xc * MinorRadius, Coord2);
        V1 = new gp_Vec(Coord0);
    }

    public static void HyperbolaD1(double U, in gp_Ax2 Pos, double MajorRadius, double MinorRadius,
                                   out gp_Pnt P, out gp_Vec V1)
    {
        double Xc = Math.Cosh(U);
        double Yc = Math.Sinh(U);
        gp_XYZ Coord1 = Pos.XDirection().XYZ();
        gp_XYZ Coord2 = Pos.YDirection().XYZ();
        gp_XYZ Coord0 = new gp_XYZ();
        // Point courant :
        Coord0.SetLinearForm(Xc * MajorRadius, Coord1, Yc * MinorRadius, Coord2, Pos.Location().XYZ());
        P = new gp_Pnt(Coord0);
        // D1 :
        Coord0.SetLinearForm(Yc * MajorRadius, Coord1, Xc * MinorRadius, Coord2);
        V1 = new gp_Vec(Coord0);
    }

    public static void ParabolaD1(double U, in gp_Ax2 Pos, double Focal, out gp_Pnt P, out gp_Vec V1)
    {
        gp_XYZ Coord1 = Pos.XDirection().XYZ();
        if (Math.Abs(Focal) <= gp.Resolution())
        { // Parabole degenere en une droite
            V1 = new gp_Vec(Coord1);
            Coord1.Multiply(U);
            Coord1.Add(Pos.Location().XYZ());
            P = new gp_Pnt(Coord1);
        }
        else
        {
            gp_XYZ Coord0 = new gp_XYZ();
            gp_XYZ Coord2 = Pos.YDirection().XYZ();
            Coord0.SetLinearForm(U / (2.0 * Focal), Coord1, Coord2);
            V1 = new gp_Vec(Coord0);
            Coord0.SetLinearForm((U * U) / (4.0 * Focal), Coord1, U, Coord2, Pos.Location().XYZ());
            P = new gp_Pnt(Coord0);
        }
    }

    public static void CircleD2(double U, in gp_Ax2 Pos, double Radius, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2)
    {
        double Xc = Radius * Math.Cos(U);
        double Yc = Radius * Math.Sin(U);
        gp_XYZ Coord1 = Pos.XDirection().XYZ();
        gp_XYZ Coord2 = Pos.YDirection().XYZ();
        gp_XYZ Coord0 = new gp_XYZ();
        // Point courant :
        Coord0.SetLinearForm(Xc, Coord1, Yc, Coord2, Pos.Location().XYZ());
        P = new gp_Pnt(Coord0);
        // D1 :
        Coord0.SetLinearForm(-Yc, Coord1, Xc, Coord2);
        V1 = new gp_Vec(Coord0);
        // D2 :
        Coord0.SetLinearForm(-Xc, Coord1, -Yc, Coord2);
        V2 = new gp_Vec(Coord0);
    }

    public static void EllipseD2(double U, in gp_Ax2 Pos, double MajorRadius, double MinorRadius,
                                 out gp_Pnt P, out gp_Vec V1, out gp_Vec V2)
    {
        double Xc = Math.Cos(U);
        double Yc = Math.Sin(U);
        gp_XYZ Coord1 = Pos.XDirection().XYZ();
        gp_XYZ Coord2 = Pos.YDirection().XYZ();
        gp_XYZ Coord0 = new gp_XYZ();
        // Point courant :
        Coord0.SetLinearForm(Xc * MajorRadius, Coord1, Yc * MinorRadius, Coord2, Pos.Location().XYZ());
        P = new gp_Pnt(Coord0);
        // D1 :
        Coord0.SetLinearForm(-Yc * MajorRadius, Coord1, Xc * MinorRadius, Coord2);
        V1 = new gp_Vec(Coord0);
        // D2 :
        Coord0.SetLinearForm(-Xc * MajorRadius, Coord1, -Yc * MinorRadius, Coord2);
        V2 = new gp_Vec(Coord0);
    }

    public static void HyperbolaD2(double U, in gp_Ax2 Pos, double MajorRadius, double MinorRadius,
                                   out gp_Pnt P, out gp_Vec V1, out gp_Vec V2)
    {
        double Xc = Math.Cosh(U);
        double Yc = Math.Sinh(U);
        gp_XYZ Coord1 = Pos.XDirection().XYZ();
        gp_XYZ Coord2 = Pos.YDirection().XYZ();
        gp_XYZ Coord0 = new gp_XYZ();
        // Point courant et D2:
        Coord0.SetLinearForm(Xc * MajorRadius, Coord1, Yc * MinorRadius, Coord2);
        V2 = new gp_Vec(Coord0);
        Coord0.Add(Pos.Location().XYZ());
        P = new gp_Pnt(Coord0);
        // D1 :
        Coord0.SetLinearForm(Yc * MajorRadius, Coord1, Xc * MinorRadius, Coord2);
        V1 = new gp_Vec(Coord0);
    }

    public static void ParabolaD2(double U, in gp_Ax2 Pos, double Focal, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2)
    {
        gp_XYZ Coord1 = Pos.XDirection().XYZ();
        if (Math.Abs(Focal) <= gp.Resolution())
        {
            V2 = new gp_Vec(0.0, 0.0, 0.0);
            V1 = new gp_Vec(Coord1);
            Coord1.Multiply(U);
            Coord1.Add(Pos.Location().XYZ());
            P = new gp_Pnt(Coord1);
        }
        else
        {
            gp_XYZ Coord0 = new gp_XYZ();
            gp_XYZ Coord2 = Pos.YDirection().XYZ();
            Coord0.SetLinearForm((U * U) / (4.0 * Focal), Coord1, U, Coord2, Pos.Location().XYZ());
            P = new gp_Pnt(Coord0);
            Coord0.SetLinearForm(U / (2.0 * Focal), Coord1, Coord2);
            V1 = new gp_Vec(Coord0);
            Coord1.Multiply(1.0 / (2.0 * Focal));
            V2 = new gp_Vec(Coord1);
        }
    }

    public static void CircleD3(double U, in gp_Ax2 Pos, double Radius,
                                out gp_Pnt P, out gp_Vec V1, out gp_Vec V2, out gp_Vec V3)
    {
        double Xc = Radius * Math.Cos(U);
        double Yc = Radius * Math.Sin(U);
        gp_XYZ Coord1 = Pos.XDirection().XYZ();
        gp_XYZ Coord2 = Pos.YDirection().XYZ();
        gp_XYZ Coord0 = new gp_XYZ();
        // Point Courant :
        Coord0.SetLinearForm(Xc, Coord1, Yc, Coord2, Pos.Location().XYZ());
        P = new gp_Pnt(Coord0);
        // D1 :
        Coord0.SetLinearForm(-Yc, Coord1, Xc, Coord2);
        V1 = new gp_Vec(Coord0);
        // D2 :
        Coord0.SetLinearForm(-Xc, Coord1, -Yc, Coord2);
        V2 = new gp_Vec(Coord0);
        // D3 :
        Coord0.SetLinearForm(Yc, Coord1, -Xc, Coord2);
        V3 = new gp_Vec(Coord0);
    }

    public static void EllipseD3(double U, in gp_Ax2 Pos, double MajorRadius, double MinorRadius,
                                 out gp_Pnt P, out gp_Vec V1, out gp_Vec V2, out gp_Vec V3)
    {
        double Xc = Math.Cos(U);
        double Yc = Math.Sin(U);
        gp_XYZ Coord1 = Pos.XDirection().XYZ();
        gp_XYZ Coord2 = Pos.YDirection().XYZ();
        gp_XYZ Coord0 = new gp_XYZ();
        // Point Courant :
        Coord0.SetLinearForm(Xc * MajorRadius, Coord1, Yc * MinorRadius, Coord2, Pos.Location().XYZ());
        P = new gp_Pnt(Coord0);
        // D1 :
        Coord0.SetLinearForm(-Yc * MajorRadius, Coord1, Xc * MinorRadius, Coord2);
        V1 = new gp_Vec(Coord0);
        // D2 :
        Coord0.SetLinearForm(-Xc * MajorRadius, Coord1, -Yc * MinorRadius, Coord2);
        V2 = new gp_Vec(Coord0);
        // D3
        Coord0.SetLinearForm(Yc * MajorRadius, Coord1, -Xc * MinorRadius, Coord2);
        V3 = new gp_Vec(Coord0);
    }

    public static void HyperbolaD3(double U, in gp_Ax2 Pos, double MajorRadius, double MinorRadius,
                                   out gp_Pnt P, out gp_Vec V1, out gp_Vec V2, out gp_Vec V3)
    {
        double Xc = Math.Cosh(U);
        double Yc = Math.Sinh(U);
        gp_XYZ Coord1 = Pos.XDirection().XYZ();
        gp_XYZ Coord2 = Pos.YDirection().XYZ();
        gp_XYZ Coord0 = new gp_XYZ();
        // Point courant et D2 :
        Coord0.SetLinearForm(Xc * MajorRadius, Coord1, Yc * MinorRadius, Coord2);
        V2 = new gp_Vec(Coord0);
        Coord0.Add(Pos.Location().XYZ());
        P = new gp_Pnt(Coord0);
        // D1 et D3 :
        Coord0.SetLinearForm(Yc * MajorRadius, Coord1, Xc * MinorRadius, Coord2);
        V1 = new gp_Vec(Coord0);
        V3 = new gp_Vec(Coord0);
    }

    public static gp_Pnt2d LineValue(double U, in gp_Ax2d Pos)
    {
        gp_XY ZDir = Pos.Direction().XY();
        gp_XY PLoc = Pos.Location().XY();
        return new gp_Pnt2d(U * ZDir.X() + PLoc.X(), U * ZDir.Y() + PLoc.Y());
    }

    public static gp_Pnt2d CircleValue(double U, in gp_Ax22d Pos, double Radius)
    {
        gp_XY XDir = Pos.XDirection().XY();
        gp_XY YDir = Pos.YDirection().XY();
        gp_XY PLoc = Pos.Location().XY();
        double A1 = Radius * Math.Cos(U);
        double A2 = Radius * Math.Sin(U);
        return new gp_Pnt2d(A1 * XDir.X() + A2 * YDir.X() + PLoc.X(),
                            A1 * XDir.Y() + A2 * YDir.Y() + PLoc.Y());
    }

    public static gp_Pnt2d EllipseValue(double U, in gp_Ax22d Pos, double MajorRadius, double MinorRadius)
    {
        gp_XY XDir = Pos.XDirection().XY();
        gp_XY YDir = Pos.YDirection().XY();
        gp_XY PLoc = Pos.Location().XY();
        double A1 = MajorRadius * Math.Cos(U);
        double A2 = MinorRadius * Math.Sin(U);
        return new gp_Pnt2d(A1 * XDir.X() + A2 * YDir.X() + PLoc.X(),
                            A1 * XDir.Y() + A2 * YDir.Y() + PLoc.Y());
    }

    public static gp_Pnt2d HyperbolaValue(double U, in gp_Ax22d Pos, double MajorRadius, double MinorRadius)
    {
        gp_XY XDir = Pos.XDirection().XY();
        gp_XY YDir = Pos.YDirection().XY();
        gp_XY PLoc = Pos.Location().XY();
        double A1 = MajorRadius * Math.Cosh(U);
        double A2 = MinorRadius * Math.Sinh(U);
        return new gp_Pnt2d(A1 * XDir.X() + A2 * YDir.X() + PLoc.X(),
                            A1 * XDir.Y() + A2 * YDir.Y() + PLoc.Y());
    }

    public static gp_Pnt2d ParabolaValue(double U, in gp_Ax22d Pos, double Focal)
    {
        if (Math.Abs(Focal) <= gp.Resolution())
        {
            gp_XY XDir0 = Pos.XDirection().XY();
            gp_XY PLoc0 = Pos.Location().XY();
            return new gp_Pnt2d(U * XDir0.X() + PLoc0.X(), U * XDir0.Y() + PLoc0.Y());
        }

        gp_XY XDir = Pos.XDirection().XY();
        gp_XY YDir = Pos.YDirection().XY();
        gp_XY PLoc = Pos.Location().XY();
        double A1 = U * U / (4.0 * Focal);
        return new gp_Pnt2d(A1 * XDir.X() + U * YDir.X() + PLoc.X(), A1 * XDir.Y() + U * YDir.Y() + PLoc.Y());
    }

    public static void LineD1(double U, in gp_Ax2d Pos, out gp_Pnt2d P, out gp_Vec2d V1)
    {
        gp_XY Coord = Pos.Direction().XY();
        V1 = new gp_Vec2d(Coord);
        Coord.SetLinearForm(U, Coord, Pos.Location().XY());
        P = new gp_Pnt2d(Coord);
    }

    public static void CircleD1(double U, in gp_Ax22d Pos, double Radius, out gp_Pnt2d P, out gp_Vec2d V1)
    {
        double Xc = Radius * Math.Cos(U);
        double Yc = Radius * Math.Sin(U);
        gp_XY Xdir = Pos.XDirection().XY();
        gp_XY Ydir = Pos.YDirection().XY();
        gp_XY Vxy = new gp_XY();
        // Point courant :
        Vxy.SetLinearForm(Xc, Xdir, Yc, Ydir, Pos.Location().XY());
        P = new gp_Pnt2d(Vxy);
        // V1 :
        Vxy.SetLinearForm(-Yc, Xdir, Xc, Ydir);
        V1 = new gp_Vec2d(Vxy);
    }

    public static void EllipseD1(double U, in gp_Ax22d Pos, double MajorRadius, double MinorRadius,
                                 out gp_Pnt2d P, out gp_Vec2d V1)
    {
        double Xc = Math.Cos(U);
        double Yc = Math.Sin(U);
        gp_XY Xdir = Pos.XDirection().XY();
        gp_XY Ydir = Pos.YDirection().XY();
        gp_XY Vxy = new gp_XY();
        // Point courant :
        Vxy.SetLinearForm(Xc * MajorRadius, Xdir, Yc * MinorRadius, Ydir, Pos.Location().XY());
        P = new gp_Pnt2d(Vxy);
        // V1 :
        Vxy.SetLinearForm(-Yc * MajorRadius, Xdir, Xc * MinorRadius, Ydir);
        V1 = new gp_Vec2d(Vxy);
    }

    public static void HyperbolaD1(double U, in gp_Ax22d Pos, double MajorRadius, double MinorRadius,
                                   out gp_Pnt2d P, out gp_Vec2d V1)
    {
        double Xc = Math.Cosh(U);
        double Yc = Math.Sinh(U);
        gp_XY Xdir = Pos.XDirection().XY();
        gp_XY Ydir = Pos.YDirection().XY();
        gp_XY Vxy = new gp_XY();
        // Point courant :
        Vxy.SetLinearForm(Xc * MajorRadius, Xdir, Yc * MinorRadius, Ydir, Pos.Location().XY());
        P = new gp_Pnt2d(Vxy);
        // V1 :
        Vxy.SetLinearForm(Yc * MajorRadius, Xdir, Xc * MinorRadius, Ydir);
        V1 = new gp_Vec2d(Vxy);
    }

    public static void ParabolaD1(double U, in gp_Ax22d Pos, double Focal, out gp_Pnt2d P, out gp_Vec2d V1)
    {
        gp_XY Vxy = new gp_XY();
        gp_XY Xdir = Pos.XDirection().XY();
        if (Math.Abs(Focal) <= gp.Resolution())
        { // Parabole degenere en une droite
            V1 = new gp_Vec2d(Xdir);
            Vxy.SetLinearForm(U, Xdir, Pos.Location().XY());
        }
        else
        {
            gp_XY Ydir = Pos.YDirection().XY();
            Vxy.SetLinearForm(U / (2.0 * Focal), Xdir, Ydir);
            V1 = new gp_Vec2d(Vxy);
            Vxy.SetLinearForm((U * U) / (4.0 * Focal), Xdir, U, Ydir, Pos.Location().XY());
        }

        P = new gp_Pnt2d(Vxy);
    }

    public static void CircleD2(double U, in gp_Ax22d Pos, double Radius,
                                out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2)
    {
        gp_XY Xdir = Pos.XDirection().XY();
        gp_XY Ydir = Pos.YDirection().XY();
        double Xc = Radius * Math.Cos(U);
        double Yc = Radius * Math.Sin(U);
        gp_XY Vxy = new gp_XY();
        // V2 :
        Vxy.SetLinearForm(Xc, Xdir, Yc, Ydir);
        V2 = new gp_Vec2d(Vxy);
        V2.Reverse();
        Vxy.Add(Pos.Location().XY());
        P = new gp_Pnt2d(Vxy);
        // V1 :
        Vxy.SetLinearForm(-Yc, Xdir, Xc, Ydir);
        V1 = new gp_Vec2d(Vxy);
    }

    public static void EllipseD2(double U, in gp_Ax22d Pos, double MajorRadius, double MinorRadius,
                                 out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2)
    {
        gp_XY Xdir = Pos.XDirection().XY();
        gp_XY Ydir = Pos.YDirection().XY();
        double Xc = Math.Cos(U);
        double Yc = Math.Sin(U);
        gp_XY Vxy = new gp_XY();
        // V2 :
        Vxy.SetLinearForm(Xc * MajorRadius, Xdir, Yc * MinorRadius, Ydir);
        V2 = new gp_Vec2d(Vxy);
        V2.Reverse();
        // Point courant :
        Vxy.Add(Pos.Location().XY());
        P = new gp_Pnt2d(Vxy);
        // V1 :
        Vxy.SetLinearForm(-Yc * MajorRadius, Xdir, Xc * MinorRadius, Ydir);
        V1 = new gp_Vec2d(Vxy);
    }

    public static void HyperbolaD2(double U, in gp_Ax22d Pos, double MajorRadius, double MinorRadius,
                                   out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2)
    {
        gp_XY Xdir = Pos.XDirection().XY();
        gp_XY Ydir = Pos.YDirection().XY();
        double Xc = Math.Cosh(U);
        double Yc = Math.Sinh(U);
        gp_XY Vxy = new gp_XY();
        // V2 :
        Vxy.SetLinearForm(Xc * MajorRadius, Xdir, Yc * MinorRadius, Ydir);
        V2 = new gp_Vec2d(Vxy);
        // Point courant :
        Vxy.Add(Pos.Location().XY());
        P = new gp_Pnt2d(Vxy);
        // V1 :
        Vxy.SetLinearForm(Yc * MajorRadius, Xdir, Xc * MinorRadius, Ydir);
        V1 = new gp_Vec2d(Vxy);
    }

    public static void ParabolaD2(double U, in gp_Ax22d Pos, double Focal,
                                  out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2)
    {
        gp_XY Vxy = new gp_XY();
        gp_XY Xdir = Pos.XDirection().XY();
        if (Math.Abs(Focal) <= gp.Resolution())
        {
            V2 = new gp_Vec2d(0.0, 0.0);
            V1 = new gp_Vec2d(Xdir);
            Vxy.SetLinearForm(U, Xdir, Pos.Location().XY());
        }
        else
        {
            gp_XY Ydir = Pos.YDirection().XY();
            Vxy = Xdir.Multiplied(1.0 / (2.0 * Focal));
            V2 = new gp_Vec2d(Vxy);
            Vxy.SetLinearForm(U, Vxy, Ydir);
            V1 = new gp_Vec2d(Vxy);
            Vxy.SetLinearForm(U * U / (4.0 * Focal), Xdir, U, Ydir);
            Vxy.Add(Pos.Location().XY());
        }

        P = new gp_Pnt2d(Vxy);
    }

    public static void CircleD3(double U, in gp_Ax22d Pos, double Radius,
                                out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2, out gp_Vec2d V3)
    {
        gp_XY Vxy = new gp_XY();
        gp_XY Xdir = Pos.XDirection().XY();
        gp_XY Ydir = Pos.YDirection().XY();
        double Xc = Radius * Math.Cos(U);
        double Yc = Radius * Math.Sin(U);
        // V2 :
        Vxy.SetLinearForm(Xc, Xdir, Yc, Ydir);
        V2 = new gp_Vec2d(Vxy);
        V2.Reverse();
        // Point courant :
        Vxy.Add(Pos.Location().XY());
        P = new gp_Pnt2d(Vxy);
        // V1 :
        Vxy.SetLinearForm(-Yc, Xdir, Xc, Ydir);
        V1 = new gp_Vec2d(Vxy);
        // V3 :
        V3 = new gp_Vec2d(Vxy);
        V3.Reverse();
    }

    public static void EllipseD3(double U, in gp_Ax22d Pos, double MajorRadius, double MinorRadius,
                                 out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2, out gp_Vec2d V3)
    {
        gp_XY Xdir = Pos.XDirection().XY();
        gp_XY Ydir = Pos.YDirection().XY();
        double Xc = Math.Cos(U);
        double Yc = Math.Sin(U);
        gp_XY Vxy = new gp_XY();
        // V2 :
        Vxy.SetLinearForm(Xc * MajorRadius, Xdir, Yc * MinorRadius, Ydir);
        V2 = new gp_Vec2d(Vxy);
        V2.Reverse();
        // Point courant :
        Vxy.Add(Pos.Location().XY());
        P = new gp_Pnt2d(Vxy);
        // V1 :
        Vxy.SetLinearForm(-Yc * MajorRadius, Xdir, Xc * MinorRadius, Ydir);
        V1 = new gp_Vec2d(Vxy);
        // V3 :
        V3 = new gp_Vec2d(Vxy);
        V3.Reverse();
    }

    public static void HyperbolaD3(double U, in gp_Ax22d Pos, double MajorRadius, double MinorRadius,
                                   out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2, out gp_Vec2d V3)
    {
        gp_XY Xdir = Pos.XDirection().XY();
        gp_XY Ydir = Pos.YDirection().XY();
        double Xc = Math.Cosh(U);
        double Yc = Math.Sinh(U);
        gp_XY Vxy = new gp_XY();
        // V2 :
        Vxy.SetLinearForm(Xc * MajorRadius, Xdir, Yc * MinorRadius, Ydir);
        V2 = new gp_Vec2d(Vxy);
        // Point courant :
        Vxy.Add(Pos.Location().XY());
        P = new gp_Pnt2d(Vxy);
        // V1 :
        Vxy.SetLinearForm(Yc * MajorRadius, Xdir, Xc * MinorRadius, Ydir);
        V1 = new gp_Vec2d(Vxy);
        // V3 :
        V3 = new gp_Vec2d(Vxy);
    }

    /// <summary>The derivative of order N of the line: its direction for N = 1, zero otherwise.</summary>
    public static gp_Vec LineDN(double U, in gp_Ax1 Pos, int N)
    {
        if (N == 1)
        {
            return new gp_Vec(Pos.Direction());
        }

        return new gp_Vec(0.0, 0.0, 0.0);
    }

    public static gp_Vec CircleDN(double U, in gp_Ax2 Pos, double Radius, int N)
    {
        double Xc = 0, Yc = 0;
        if (N == 1)
        {
            Xc = Radius * -Math.Sin(U);
            Yc = Radius * Math.Cos(U);
        }
        else if ((N + 2) % 4 == 0)
        {
            Xc = Radius * -Math.Cos(U);
            Yc = Radius * -Math.Sin(U);
        }
        else if ((N + 1) % 4 == 0)
        {
            Xc = Radius * Math.Sin(U);
            Yc = Radius * -Math.Cos(U);
        }
        else if (N % 4 == 0)
        {
            Xc = Radius * Math.Cos(U);
            Yc = Radius * Math.Sin(U);
        }
        else if ((N - 1) % 4 == 0)
        {
            Xc = Radius * -Math.Sin(U);
            Yc = Radius * Math.Cos(U);
        }

        gp_XYZ Coord1 = Pos.XDirection().XYZ();
        Coord1.SetLinearForm(Xc, Coord1, Yc, Pos.YDirection().XYZ());
        return new gp_Vec(Coord1);
    }

    public static gp_Vec EllipseDN(double U, in gp_Ax2 Pos, double MajorRadius, double MinorRadius, int N)
    {
        double Xc = 0, Yc = 0;
        if (N == 1)
        {
            Xc = MajorRadius * -Math.Sin(U);
            Yc = MinorRadius * Math.Cos(U);
        }
        else if ((N + 2) % 4 == 0)
        {
            Xc = MajorRadius * -Math.Cos(U);
            Yc = MinorRadius * -Math.Sin(U);
        }
        else if ((N + 1) % 4 == 0)
        {
            Xc = MajorRadius * Math.Sin(U);
            Yc = MinorRadius * -Math.Cos(U);
        }
        else if (N % 4 == 0)
        {
            Xc = MajorRadius * Math.Cos(U);
            Yc = MinorRadius * Math.Sin(U);
        }
        else if ((N - 1) % 4 == 0)
        {
            Xc = MajorRadius * -Math.Sin(U);
            Yc = MinorRadius * Math.Cos(U);
        }

        gp_XYZ Coord1 = Pos.XDirection().XYZ();
        Coord1.SetLinearForm(Xc, Coord1, Yc, Pos.YDirection().XYZ());
        return new gp_Vec(Coord1);
    }

    public static gp_Vec HyperbolaDN(double U, in gp_Ax2 Pos, double MajorRadius, double MinorRadius, int N)
    {
        double Xc = 0, Yc = 0;
        if (IsOdd(N))
        {
            Xc = MajorRadius * Math.Sinh(U);
            Yc = MinorRadius * Math.Cosh(U);
        }
        else if (IsEven(N))
        {
            Xc = MajorRadius * Math.Cosh(U);
            Yc = MinorRadius * Math.Sinh(U);
        }

        gp_XYZ Coord1 = Pos.XDirection().XYZ();
        Coord1.SetLinearForm(Xc, Coord1, Yc, Pos.YDirection().XYZ());
        return new gp_Vec(Coord1);
    }

    public static gp_Vec ParabolaDN(double U, in gp_Ax2 Pos, double Focal, int N)
    {
        if (N > 2 || N <= 0)
        {
            return new gp_Vec(0.0, 0.0, 0.0);
        }

        gp_XYZ Coord1 = Pos.XDirection().XYZ();
        if (N == 1)
        {
            if (Math.Abs(Focal) <= gp.Resolution())
            {
                return new gp_Vec(Coord1);
            }

            Coord1.SetLinearForm(U / (2.0 * Focal), Coord1, Pos.YDirection().XYZ());
            return new gp_Vec(Coord1);
        }

        if (Math.Abs(Focal) <= gp.Resolution())
        {
            return new gp_Vec(0.0, 0.0, 0.0);
        }

        Coord1.Multiply(1.0 / (2.0 * Focal));
        return new gp_Vec(Coord1);
    }

    public static gp_Vec2d LineDN(double U, in gp_Ax2d Pos, int N)
    {
        if (N == 1)
        {
            return new gp_Vec2d(Pos.Direction());
        }

        return new gp_Vec2d(0.0, 0.0);
    }

    public static gp_Vec2d CircleDN(double U, in gp_Ax22d Pos, double Radius, int N)
    {
        double Xc = 0, Yc = 0;
        if (N == 1)
        {
            Xc = Radius * -Math.Sin(U);
            Yc = Radius * Math.Cos(U);
        }
        else if ((N + 2) % 4 == 0)
        {
            Xc = Radius * -Math.Cos(U);
            Yc = Radius * -Math.Sin(U);
        }
        else if ((N + 1) % 4 == 0)
        {
            Xc = Radius * Math.Sin(U);
            Yc = Radius * -Math.Cos(U);
        }
        else if (N % 4 == 0)
        {
            Xc = Radius * Math.Cos(U);
            Yc = Radius * Math.Sin(U);
        }
        else if ((N - 1) % 4 == 0)
        {
            Xc = Radius * -Math.Sin(U);
            Yc = Radius * Math.Cos(U);
        }

        gp_XY Xdir = Pos.XDirection().XY();
        gp_XY Ydir = Pos.YDirection().XY();
        Xdir.SetLinearForm(Xc, Xdir, Yc, Ydir);
        return new gp_Vec2d(Xdir);
    }

    public static gp_Vec2d EllipseDN(double U, in gp_Ax22d Pos, double MajorRadius, double MinorRadius, int N)
    {
        double Xc = 0, Yc = 0;
        if (N == 1)
        {
            Xc = MajorRadius * -Math.Sin(U);
            Yc = MinorRadius * Math.Cos(U);
        }
        else if ((N + 2) % 4 == 0)
        {
            Xc = MajorRadius * -Math.Cos(U);
            Yc = MinorRadius * -Math.Sin(U);
        }
        else if ((N + 1) % 4 == 0)
        {
            Xc = MajorRadius * Math.Sin(U);
            Yc = MinorRadius * -Math.Cos(U);
        }
        else if (N % 4 == 0)
        {
            Xc = MajorRadius * Math.Cos(U);
            Yc = MinorRadius * Math.Sin(U);
        }
        else if ((N - 1) % 4 == 0)
        {
            Xc = MajorRadius * -Math.Sin(U);
            Yc = MinorRadius * Math.Cos(U);
        }

        gp_XY Xdir = Pos.XDirection().XY();
        gp_XY Ydir = Pos.YDirection().XY();
        Xdir.SetLinearForm(Xc, Xdir, Yc, Ydir);
        return new gp_Vec2d(Xdir);
    }

    public static gp_Vec2d HyperbolaDN(double U, in gp_Ax22d Pos, double MajorRadius, double MinorRadius, int N)
    {
        double Xc = 0, Yc = 0;
        if (IsOdd(N))
        {
            Xc = MajorRadius * Math.Sinh(U);
            Yc = MinorRadius * Math.Cosh(U);
        }
        else if (IsEven(N))
        {
            Xc = MajorRadius * Math.Cosh(U);
            Yc = MinorRadius * Math.Sinh(U);
        }

        gp_XY Xdir = Pos.XDirection().XY();
        gp_XY Ydir = Pos.YDirection().XY();
        Xdir.SetLinearForm(Xc, Xdir, Yc, Ydir);
        return new gp_Vec2d(Xdir);
    }

    public static gp_Vec2d ParabolaDN(double U, in gp_Ax22d Pos, double Focal, int N)
    {
        if (N > 2 || N <= 0)
        {
            return new gp_Vec2d(0.0, 0.0);
        }

        gp_XY Xdir = Pos.XDirection().XY();
        if (N == 1)
        {
            if (Math.Abs(Focal) <= gp.Resolution())
            {
                return new gp_Vec2d(Xdir);
            }

            gp_XY Ydir = Pos.YDirection().XY();
            Xdir.SetLinearForm(U / (2.0 * Focal), Xdir, Ydir);
            return new gp_Vec2d(Xdir);
        }

        if (Math.Abs(Focal) <= gp.Resolution())
        {
            return new gp_Vec2d(0.0, 0.0);
        }

        Xdir.Multiply(1.0 / (2.0 * Focal));
        return new gp_Vec2d(Xdir);
    }

    /// <summary>The parameter of P on the line: P(U) = L.Location() + U * L.Direction().</summary>
    public static double LineParameter(in gp_Ax1 L, in gp_Pnt P)
        => (P.XYZ() - L.Location().XYZ()).Dot(L.Direction().XYZ());

    /// <summary>The parameter of P on the circle of axis Pos, in [0, 2*PI]; 0 when P is on the axis.</summary>
    public static double CircleParameter(in gp_Ax2 Pos, in gp_Pnt P)
    {
        gp_Vec aVec = new gp_Vec(Pos.Location(), P);
        if (aVec.SquareMagnitude() < gp.Resolution())
        {
            // coinciding points -> infinite number of parameters
            return 0.0;
        }

        gp_Dir dir = Pos.Direction();
        // Project vector on circle's plane
        gp_XYZ aVProj = dir.XYZ().CrossCrossed(aVec.XYZ(), dir.XYZ());
        if (aVProj.SquareModulus() < gp.Resolution())
        {
            return 0.0;
        }

        // Angle between X direction and projected vector
        double Teta = Pos.XDirection().AngleWithRef(new gp_Dir(aVProj), dir);
        normalizeAngle(ref Teta);
        return Teta;
    }

    public static double EllipseParameter(in gp_Ax2 Pos, double MajorRadius, double MinorRadius, in gp_Pnt P)
    {
        gp_XYZ OP = P.XYZ() - Pos.Location().XYZ();
        gp_XYZ xaxis = Pos.XDirection().XYZ();
        gp_XYZ yaxis = Pos.YDirection().XYZ();
        double NY = OP.Dot(yaxis);
        double NX = OP.Dot(xaxis);
        if ((Math.Abs(NX) <= gp.Resolution()) && (Math.Abs(NY) <= gp.Resolution()))
        {
            //-- The point P is on the Axis of the Ellipse.
            return 0.0;
        }

        yaxis.Multiply(NY * (MajorRadius / MinorRadius));
        gp_XYZ Om = xaxis.Multiplied(NX);
        Om.Add(yaxis);
        double Teta = new gp_Vec(xaxis).AngleWithRef(new gp_Vec(Om), new gp_Vec(Pos.Direction()));
        normalizeAngle(ref Teta);
        return Teta;
    }

    public static double HyperbolaParameter(in gp_Ax2 Pos, double MajorRadius, double MinorRadius, in gp_Pnt P)
    {
        double sht = new gp_Vec(Pos.Location(), P).Dot(new gp_Vec(Pos.YDirection())) / MinorRadius;
        return Math.Asinh(sht);
    }

    public static double ParabolaParameter(in gp_Ax2 Pos, in gp_Pnt P)
        => new gp_Vec(Pos.Location(), P).Dot(new gp_Vec(Pos.YDirection()));

    public static double LineParameter(in gp_Ax2d L, in gp_Pnt2d P)
    {
        gp_XY Coord = P.XY();
        Coord.Subtract(L.Location().XY());
        return Coord.Dot(L.Direction().XY());
    }

    public static double CircleParameter(in gp_Ax22d Pos, in gp_Pnt2d P)
    {
        double Teta = Pos.XDirection().Angle(new gp_Dir2d(new gp_Vec2d(Pos.Location(), P)));
        Teta = ((Pos.XDirection() ^ Pos.YDirection()) >= 0.0) ? Teta : -Teta;
        normalizeAngle(ref Teta);
        return Teta;
    }

    public static double EllipseParameter(in gp_Ax22d Pos, double MajorRadius, double MinorRadius, in gp_Pnt2d P)
    {
        gp_XY OP = P.XY();
        OP.Subtract(Pos.Location().XY());
        gp_XY xaxis = Pos.XDirection().XY();
        gp_XY yaxis = Pos.YDirection().XY();
        gp_XY Om = xaxis.Multiplied(OP.Dot(xaxis));
        yaxis.Multiply((OP.Dot(yaxis)) * (MajorRadius / MinorRadius));
        Om.Add(yaxis);
        double Teta = new gp_Vec2d(xaxis).Angle(new gp_Vec2d(Om));
        Teta = ((Pos.XDirection() ^ Pos.YDirection()) >= 0.0) ? Teta : -Teta;
        normalizeAngle(ref Teta);
        return Teta;
    }

    public static double HyperbolaParameter(in gp_Ax22d Pos, double MajorRadius, double MinorRadius, in gp_Pnt2d P)
    {
        gp_Vec2d V = new gp_Vec2d(Pos.YDirection().XY());
        double sht = new gp_Vec2d(Pos.Location(), P).Dot(V) / MinorRadius;
        return Math.Asinh(sht);
    }

    public static double ParabolaParameter(in gp_Ax22d Pos, in gp_Pnt2d P)
    {
        gp_Vec2d Directrix = new gp_Vec2d(Pos.YDirection().XY());
        return new gp_Vec2d(Pos.Location(), P).Dot(Directrix);
    }

    // Building 3D entities from 2D ones: the X and Y axes of the 2D space are lined up with the X and Y
    // directions of Pos.

    public static gp_Pnt To3d(in gp_Ax2 Pos, in gp_Pnt2d P)
    {
        gp_XYZ Vxy = Pos.XDirection().XYZ();
        Vxy.SetLinearForm(P.X(), Vxy, P.Y(), Pos.YDirection().XYZ(), Pos.Location().XYZ());
        return new gp_Pnt(Vxy);
    }

    public static gp_Dir To3d(in gp_Ax2 Pos, in gp_Dir2d V)
    {
        gp_Vec Vx = new gp_Vec(Pos.XDirection());
        gp_Vec Vy = new gp_Vec(Pos.YDirection());
        Vx.Multiply(V.X());
        Vy.Multiply(V.Y());
        Vx.Add(Vy);
        return new gp_Dir(Vx);
    }

    public static gp_Vec To3d(in gp_Ax2 Pos, in gp_Vec2d V)
    {
        gp_Vec Vx = new gp_Vec(Pos.XDirection());
        gp_Vec Vy = new gp_Vec(Pos.YDirection());
        Vx.Multiply(V.X());
        Vy.Multiply(V.Y());
        Vx.Add(Vy);
        return Vx;
    }

    public static gp_Ax1 To3d(in gp_Ax2 Pos, in gp_Ax2d A)
    {
        gp_Pnt P = To3d(Pos, A.Location());
        gp_Vec V = new gp_Vec(To3d(Pos, A.Direction()));
        return new gp_Ax1(P, new gp_Dir(V));
    }

    public static gp_Ax2 To3d(in gp_Ax2 Pos, in gp_Ax22d A)
    {
        gp_Pnt P = To3d(Pos, A.Location());
        gp_Vec VX = new gp_Vec(To3d(Pos, A.XDirection()));
        gp_Vec VY = new gp_Vec(To3d(Pos, A.YDirection()));
        return new gp_Ax2(P, new gp_Dir(VX.Crossed(VY)), new gp_Dir(VX));
    }

    public static gp_Lin To3d(in gp_Ax2 Pos, in gp_Lin2d L) => new gp_Lin(To3d(Pos, L.Position()));

    public static gp_Circ To3d(in gp_Ax2 Pos, in gp_Circ2d C) => new gp_Circ(To3d(Pos, C.Axis()), C.Radius());

    public static gp_Elips To3d(in gp_Ax2 Pos, in gp_Elips2d E)
        => new gp_Elips(To3d(Pos, E.Axis()), E.MajorRadius(), E.MinorRadius());

    public static gp_Hypr To3d(in gp_Ax2 Pos, in gp_Hypr2d H)
        => new gp_Hypr(To3d(Pos, H.Axis()), H.MajorRadius(), H.MinorRadius());

    public static gp_Parab To3d(in gp_Ax2 Pos, in gp_Parab2d Prb) => new gp_Parab(To3d(Pos, Prb.Axis()), Prb.Focal());
}
