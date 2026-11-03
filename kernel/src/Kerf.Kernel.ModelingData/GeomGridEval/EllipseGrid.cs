// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Ellipse.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Ellipse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for ellipse grid points: P(u) = C + MajR * cos(u) * X + MinR * sin(u) * Y.
/// </summary>
public sealed class EllipseGrid : GeomGridEvalGrid.ICurveEvaluator
{
    private readonly EllipseCurve3D? _geom;

    /// <summary>Constructor with the ellipse to evaluate (null gives empty results).</summary>
    public EllipseGrid(EllipseCurve3D? ellipse)
    {
        _geom = ellipse;
    }

    /// <summary>Returns the geometry.</summary>
    public EllipseCurve3D? Geometry() => _geom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point3D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Point3D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Point3D> result = new(1, nb);

        Ellipse3D elips = _geom.Elips();
        Point3D center = elips.Location();
        Direction3D xDir = elips.XAxis().Direction();
        Direction3D yDir = elips.YAxis().Direction();
        double majR = elips.MajorRadius();
        double minR = elips.MinorRadius();

        double cx = center.X();
        double cy = center.Y();
        double cz = center.Z();
        double xx = xDir.X();
        double xy = xDir.Y();
        double xz = xDir.Z();
        double yx = yDir.X();
        double yy = yDir.Y();
        double yz = yDir.Z();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            result.SetValue(i - theParams.Lower() + 1,
                             new Point3D(cx + majR * cosU * xx + minR * sinU * yx,
                                        cy + majR * cosU * xy + minR * sinU * yy,
                                        cz + majR * cosU * xz + minR * sinU * yz));
        }
        return result;
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Curve3D.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve3D.ResD1>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve3D.ResD1> result = new(1, nb);

        Ellipse3D elips = _geom.Elips();
        Point3D center = elips.Location();
        Direction3D xDir = elips.XAxis().Direction();
        Direction3D yDir = elips.YAxis().Direction();
        double majR = elips.MajorRadius();
        double minR = elips.MinorRadius();

        double cx = center.X();
        double cy = center.Y();
        double cz = center.Z();
        double xx = xDir.X();
        double xy = xDir.Y();
        double xz = xDir.Z();
        double yx = yDir.X();
        double yy = yDir.Y();
        double yz = yDir.Z();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            // D1 = -MajorR * sin(u) * XDir + MinorR * cos(u) * YDir
            result[i - theParams.Lower() + 1] = new Curve3D.ResD1
            {
                Point = new Point3D(cx + majR * cosU * xx + minR * sinU * yx,
                                   cy + majR * cosU * xy + minR * sinU * yy,
                                   cz + majR * cosU * xz + minR * sinU * yz),
                D1 = new Vector3D(-majR * sinU * xx + minR * cosU * yx,
                                -majR * sinU * xy + minR * cosU * yy,
                                -majR * sinU * xz + minR * cosU * yz),
            };
        }
        return result;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Curve3D.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve3D.ResD2>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve3D.ResD2> result = new(1, nb);

        Ellipse3D elips = _geom.Elips();
        Point3D center = elips.Location();
        Direction3D xDir = elips.XAxis().Direction();
        Direction3D yDir = elips.YAxis().Direction();
        double majR = elips.MajorRadius();
        double minR = elips.MinorRadius();

        double cx = center.X();
        double cy = center.Y();
        double cz = center.Z();
        double xx = xDir.X();
        double xy = xDir.Y();
        double xz = xDir.Z();
        double yx = yDir.X();
        double yy = yDir.Y();
        double yz = yDir.Z();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            // D1 = -MajorR * sin(u) * XDir + MinorR * cos(u) * YDir
            // D2 = -MajorR * cos(u) * XDir - MinorR * sin(u) * YDir
            result[i - theParams.Lower() + 1] = new Curve3D.ResD2
            {
                Point = new Point3D(cx + majR * cosU * xx + minR * sinU * yx,
                                   cy + majR * cosU * xy + minR * sinU * yy,
                                   cz + majR * cosU * xz + minR * sinU * yz),
                D1 = new Vector3D(-majR * sinU * xx + minR * cosU * yx,
                                -majR * sinU * xy + minR * cosU * yy,
                                -majR * sinU * xz + minR * cosU * yz),
                D2 = new Vector3D(-majR * cosU * xx - minR * sinU * yx,
                                -majR * cosU * xy - minR * sinU * yy,
                                -majR * cosU * xz - minR * sinU * yz),
            };
        }
        return result;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third.</summary>
    public NCollection_Array1<Curve3D.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve3D.ResD3>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve3D.ResD3> result = new(1, nb);

        Ellipse3D elips = _geom.Elips();
        Point3D center = elips.Location();
        Direction3D xDir = elips.XAxis().Direction();
        Direction3D yDir = elips.YAxis().Direction();
        double majR = elips.MajorRadius();
        double minR = elips.MinorRadius();

        double cx = center.X();
        double cy = center.Y();
        double cz = center.Z();
        double xx = xDir.X();
        double xy = xDir.Y();
        double xz = xDir.Z();
        double yx = yDir.X();
        double yy = yDir.Y();
        double yz = yDir.Z();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            // D1 = -MajorR * sin(u) * XDir + MinorR * cos(u) * YDir
            // D2 = -MajorR * cos(u) * XDir - MinorR * sin(u) * YDir
            // D3 =  MajorR * sin(u) * XDir - MinorR * cos(u) * YDir = -D1
            result[i - theParams.Lower() + 1] = new Curve3D.ResD3
            {
                Point = new Point3D(cx + majR * cosU * xx + minR * sinU * yx,
                                   cy + majR * cosU * xy + minR * sinU * yy,
                                   cz + majR * cosU * xz + minR * sinU * yz),
                D1 = new Vector3D(-majR * sinU * xx + minR * cosU * yx,
                                -majR * sinU * xy + minR * cosU * yy,
                                -majR * sinU * xz + minR * cosU * yz),
                D2 = new Vector3D(-majR * cosU * xx - minR * sinU * yx,
                                -majR * cosU * xy - minR * sinU * yy,
                                -majR * cosU * xz - minR * sinU * yz),
                D3 = new Vector3D(majR * sinU * xx - minR * cosU * yx,
                                majR * sinU * xy - minR * cosU * yy,
                                majR * sinU * xz - minR * cosU * yz),
            };
        }
        return result;
    }

    /// <summary>Evaluates the Nth derivative (N &gt;= 1) at all grid points; the derivatives cycle with period 4.</summary>
    public NCollection_Array1<Vector3D> EvaluateGridDN(NCollection_Array1<double> theParams, int n)
    {
        if (_geom is null || theParams.IsEmpty() || n < 1)
        {
            return new NCollection_Array1<Vector3D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Vector3D> result = new(1, nb);

        Ellipse3D elips = _geom.Elips();
        Direction3D xDir = elips.XAxis().Direction();
        Direction3D yDir = elips.YAxis().Direction();
        double majR = elips.MajorRadius();
        double minR = elips.MinorRadius();

        double xx = xDir.X();
        double xy = xDir.Y();
        double xz = xDir.Z();
        double yx = yDir.X();
        double yy = yDir.Y();
        double yz = yDir.Z();

        // Ellipse derivatives are cyclic with period 4:
        // D1 = -MajR * sin(u) * X + MinR * cos(u) * Y  -> coefficients: (-sin, cos)
        // D2 = -MajR * cos(u) * X - MinR * sin(u) * Y  -> coefficients: (-cos, -sin)
        // D3 =  MajR * sin(u) * X - MinR * cos(u) * Y  -> coefficients: (sin, -cos)
        // D4 =  MajR * cos(u) * X + MinR * sin(u) * Y  -> coefficients: (cos, sin) = D0
        int phase = (n - 1) % 4; // 0->D1, 1->D2, 2->D3, 3->D4

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            double coeffMajR, coeffMinR;
            switch (phase)
            {
                case 0: // D1, D5, D9, ...
                    coeffMajR = -sinU;
                    coeffMinR = cosU;
                    break;
                case 1: // D2, D6, D10, ...
                    coeffMajR = -cosU;
                    coeffMinR = -sinU;
                    break;
                case 2: // D3, D7, D11, ...
                    coeffMajR = sinU;
                    coeffMinR = -cosU;
                    break;
                default: // D4, D8, D12, ... (case 3)
                    coeffMajR = cosU;
                    coeffMinR = sinU;
                    break;
            }

            result.SetValue(i - theParams.Lower() + 1,
                             new Vector3D(majR * coeffMajR * xx + minR * coeffMinR * yx,
                                        majR * coeffMajR * xy + minR * coeffMinR * yy,
                                        majR * coeffMajR * xz + minR * coeffMinR * yz));
        }
        return result;
    }
}
