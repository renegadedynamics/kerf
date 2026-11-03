// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_Conic.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_Conic.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A 2D conic given by its implicit equation <c>A.X**2 + B.Y**2 + 2.C.X*Y + 2.D.X + 2.E.Y + F = 0</c>.
/// </summary>
/// <remarks><c>default(IntAna2d_Conic)</c> has all coefficients zero.</remarks>
public partial struct IntAna2d_Conic : IEquatable<IntAna2d_Conic>
{
    private double a;
    private double b;
    private double c;
    private double d;
    private double e;
    private double f;

    public IntAna2d_Conic(in Line2D L)
    {
        a = 0.0;
        b = 0.0;
        c = 0.0;
        L.Coefficients(out d, out e, out f);
        f = 2 * f;
    }

    public IntAna2d_Conic(in Circle2D C) => C.Coefficients(out a, out b, out c, out d, out e, out f);

    public IntAna2d_Conic(in Ellipse2D E) => E.Coefficients(out a, out b, out c, out d, out e, out f);

    public IntAna2d_Conic(in Parabola2D P) => P.Coefficients(out a, out b, out c, out d, out e, out f);

    public IntAna2d_Conic(in Hyperbola2D H) => H.Coefficients(out a, out b, out c, out d, out e, out f);

    /// <summary>
    /// Transforms the coefficients A to F of a polynomial equation <c>A x x + B y y + 2 C x y + 2 D x + 2 E y + F</c>
    /// (written in the natural coordinate system) into the local coordinate system defined by <paramref name="Dir1"/>.
    /// </summary>
    public readonly void NewCoefficients(ref double A, ref double B, ref double C, ref double D, ref double E, ref double F,
                                         in Axis2D Dir1)
    {
        double t11, t12, t13; // x = t11 X + t12 Y + t13
        double t21, t22, t23; // y = t21 X + t22 Y + t23
        double A1, B1, C1, D1, E1, F1;

        //      P0(x,y)=A x x + B y y + ... + F =0  (x,y "absolute" coordinates)
        // and  P1(X(x,y),Y(x,y))=P0(x,y)
        // with P1(X,Y)= A1 X X + B1 Y Y + 2 C1 X Y + 2 D1 X + 2 E1 Y + F1
        //             = A  x x + B  y y + 2 C  x y + 2 D  x + 2 E  y + f

        Dir1.Direction().Coord(out t11, out t21);
        Dir1.Location().Coord(out t13, out t23);

        t22 = t11;
        t12 = -t21;

        A1 = (t11 * (A * t11 + 2 * C * t21) + B * t21 * t21);
        B1 = (t12 * (A * t12 + 2 * C * t22) + B * t22 * t22);
        C1 = (t12 * (A * t11 + C * t21) + t22 * (C * t11 + B * t21));
        D1 = (t11 * (D + A * t13) + t21 * (E + C * t13) + t23 * (C * t11 + B * t21));
        E1 = (t12 * (D + A * t13) + t22 * (E + C * t13) + t23 * (C * t12 + B * t22));
        F1 = F + t13 * (2.0 * D + A * t13) + t23 * (2.0 * E + 2.0 * C * t13 + B * t23);

        A = A1;
        B = B1;
        C = C1;
        D = D1;
        E = E1;
        F = F1;
    }

    /// <summary>The value of the function F at the point X,Y.</summary>
    public readonly double Value(double X, double Y)
    {
        double _a, _b, _c, _d, _e, _f;
        Coefficients(out _a, out _b, out _c, out _d, out _e, out _f);
        return (_a * X * X + _b * Y * Y + 2.0 * _c * X * Y + 2.0 * _d * X + 2.0 * _e * Y + _f);
    }

    /// <summary>The gradient of F at the point X,Y.</summary>
    public readonly Xy Grad(double X, double Y)
    {
        double _a, _b, _c, _d, _e, _f;
        Coefficients(out _a, out _b, out _c, out _d, out _e, out _f);
        return new Xy(2.0 * _a * X + 2.0 * _c * Y + 2.0 * _d, 2.0 * _b * Y + 2.0 * _c * X + 2.0 * _e);
    }

    /// <summary>The value of the function and its gradient at the point X,Y.</summary>
    public readonly void ValAndGrad(double X, double Y, out double Val, out Xy Grd)
    {
        double la, lb, lc, ld, le, lf;
        Coefficients(out la, out lb, out lc, out ld, out le, out lf);
        Grd = new Xy(2.0 * la * X + 2.0 * lc * Y + 2.0 * ld, 2.0 * lb * Y + 2.0 * lc * X + 2.0 * le);
        Val = la * X * X + lb * Y * Y + 2.0 * lc * X * Y + 2.0 * ld * X + 2.0 * le * Y + lf;
    }

    /// <summary>The coefficients of the conic's equation <c>A.X**2 + B.Y**2 + 2.C.X*Y + 2.D.X + 2.E.Y + F = 0</c>.</summary>
    public readonly void Coefficients(out double A, out double B, out double C, out double D, out double E, out double F)
    {
        A = a;
        B = b;
        C = c;
        D = d;
        E = e;
        F = f;
    }

    public readonly bool Equals(IntAna2d_Conic other)
        => a.Equals(other.a) && b.Equals(other.b) && c.Equals(other.c)
           && d.Equals(other.d) && e.Equals(other.e) && f.Equals(other.f);

    public override readonly bool Equals(object? obj) => obj is IntAna2d_Conic other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(a, b, c, d, e, f);

    public static bool operator ==(in IntAna2d_Conic theLeft, in IntAna2d_Conic theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntAna2d_Conic theLeft, in IntAna2d_Conic theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => Constants.Format("IntAna2d_Conic", a, b, c, d, e, f);
}
