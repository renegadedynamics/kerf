// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_Conic.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_Conic.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>
/// A 2D conic given by its implicit equation <c>A.X**2 + B.Y**2 + 2.C.X*Y + 2.D.X + 2.E.Y + F = 0</c>.
/// </summary>
/// <remarks><c>default(IntAna2d_Conic)</c> has all coefficients zero.</remarks>
public partial struct Conic : IEquatable<Conic>
{
    private double _a;
    private double _b;
    private double _c;
    private double _d;
    private double _e;
    private double _f;

    public Conic(in Line2D l)
    {
        _a = 0.0;
        _b = 0.0;
        _c = 0.0;
        l.Coefficients(out _d, out _e, out _f);
        _f = 2 * _f;
    }

    public Conic(in Circle2D c) => c.Coefficients(out _a, out _b, out _c, out _d, out _e, out _f);

    public Conic(in Ellipse2D e) => e.Coefficients(out _a, out _b, out _c, out _d, out _e, out _f);

    public Conic(in Parabola2D p) => p.Coefficients(out _a, out _b, out _c, out _d, out _e, out _f);

    public Conic(in Hyperbola2D h) => h.Coefficients(out _a, out _b, out _c, out _d, out _e, out _f);

    /// <summary>
    /// Transforms the coefficients A to F of a polynomial equation <c>A x x + B y y + 2 C x y + 2 D x + 2 E y + F</c>
    /// (written in the natural coordinate system) into the local coordinate system defined by <paramref name="dir1"/>.
    /// </summary>
    public readonly void NewCoefficients(ref double a, ref double b, ref double c, ref double d, ref double e, ref double f,
                                         in Axis2D dir1)
    {
        double t11, t12, t13; // x = t11 X + t12 Y + t13
        double t21, t22, t23; // y = t21 X + t22 Y + t23
        double a1, b1, c1, d1, e1, f1;

        //      P0(x,y)=A x x + B y y + ... + F =0  (x,y "absolute" coordinates)
        // and  P1(X(x,y),Y(x,y))=P0(x,y)
        // with P1(X,Y)= A1 X X + B1 Y Y + 2 C1 X Y + 2 D1 X + 2 E1 Y + F1
        //             = A  x x + B  y y + 2 C  x y + 2 D  x + 2 E  y + f

        dir1.Direction().Coord(out t11, out t21);
        dir1.Location().Coord(out t13, out t23);

        t22 = t11;
        t12 = -t21;

        a1 = (t11 * (a * t11 + 2 * c * t21) + b * t21 * t21);
        b1 = (t12 * (a * t12 + 2 * c * t22) + b * t22 * t22);
        c1 = (t12 * (a * t11 + c * t21) + t22 * (c * t11 + b * t21));
        d1 = (t11 * (d + a * t13) + t21 * (e + c * t13) + t23 * (c * t11 + b * t21));
        e1 = (t12 * (d + a * t13) + t22 * (e + c * t13) + t23 * (c * t12 + b * t22));
        f1 = f + t13 * (2.0 * d + a * t13) + t23 * (2.0 * e + 2.0 * c * t13 + b * t23);

        a = a1;
        b = b1;
        c = c1;
        d = d1;
        e = e1;
        f = f1;
    }

    /// <summary>The value of the function F at the point X,Y.</summary>
    public readonly double Value(double x, double y)
    {
        double _a, _b, _c, _d, _e, _f;
        Coefficients(out _a, out _b, out _c, out _d, out _e, out _f);
        return (_a * x * x + _b * y * y + 2.0 * _c * x * y + 2.0 * _d * x + 2.0 * _e * y + _f);
    }

    /// <summary>The gradient of F at the point X,Y.</summary>
    public readonly Xy Grad(double x, double y)
    {
        double _a, _b, _c, _d, _e, _f;
        Coefficients(out _a, out _b, out _c, out _d, out _e, out _f);
        return new Xy(2.0 * _a * x + 2.0 * _c * y + 2.0 * _d, 2.0 * _b * y + 2.0 * _c * x + 2.0 * _e);
    }

    /// <summary>The value of the function and its gradient at the point X,Y.</summary>
    public readonly void ValAndGrad(double x, double y, out double val, out Xy grd)
    {
        double la, lb, lc, ld, le, lf;
        Coefficients(out la, out lb, out lc, out ld, out le, out lf);
        grd = new Xy(2.0 * la * x + 2.0 * lc * y + 2.0 * ld, 2.0 * lb * y + 2.0 * lc * x + 2.0 * le);
        val = la * x * x + lb * y * y + 2.0 * lc * x * y + 2.0 * ld * x + 2.0 * le * y + lf;
    }

    /// <summary>The coefficients of the conic's equation <c>A.X**2 + B.Y**2 + 2.C.X*Y + 2.D.X + 2.E.Y + F = 0</c>.</summary>
    public readonly void Coefficients(out double a, out double b, out double c, out double d, out double e, out double f)
    {
        a = _a;
        b = _b;
        c = _c;
        d = _d;
        e = _e;
        f = _f;
    }

    public readonly bool Equals(Conic other)
        => _a.Equals(other._a) && _b.Equals(other._b) && _c.Equals(other._c)
           && _d.Equals(other._d) && _e.Equals(other._e) && _f.Equals(other._f);

    public override readonly bool Equals(object? obj) => obj is Conic other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_a, _b, _c, _d, _e, _f);

    public static bool operator ==(in Conic left, in Conic right) => left.Equals(right);

    public static bool operator !=(in Conic left, in Conic right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("IntAna2d_Conic", _a, _b, _c, _d, _e, _f);
}
