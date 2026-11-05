// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Ellipse.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Ellipse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for 2D ellipse grid points:
/// P(u) = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir.
/// </summary>
internal sealed class EllipseGrid2D : Geom2dGridEvalGrid2D.ICurveEvaluator
{
    private readonly EllipseCurve2D? _geom;

    /// <summary>Constructor with the ellipse to evaluate (null gives empty results).</summary>
    public EllipseGrid2D(EllipseCurve2D? ellipse)
    {
        _geom = ellipse;
    }

    /// <summary>Returns the geometry.</summary>
    public EllipseCurve2D? Geometry() => _geom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point2D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Point2D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Point2D> result = new(1, nb);

        Ellipse2D elips = _geom.Ellipse;
        Point2D center = elips.Location;
        Direction2D xDir = elips.XAxis.Direction;
        Direction2D yDir = elips.YAxis.Direction;
        double majR = elips.MajorRadius;
        double minR = elips.MinorRadius;

        double cx = center.X;
        double cy = center.Y;
        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            result.SetValue(i - theParams.Lower() + 1,
                             new Point2D(cx + majR * cosU * xx + minR * sinU * yx,
                                          cy + majR * cosU * xy + minR * sinU * yy));
        }
        return result;
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Curve2D.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve2D.ResD1>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve2D.ResD1> result = new(1, nb);

        Ellipse2D elips = _geom.Ellipse;
        Point2D center = elips.Location;
        Direction2D xDir = elips.XAxis.Direction;
        Direction2D yDir = elips.YAxis.Direction;
        double majR = elips.MajorRadius;
        double minR = elips.MinorRadius;

        double cx = center.X;
        double cy = center.Y;
        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            // D1 = -MajorR * sin(u) * XDir + MinorR * cos(u) * YDir
            result[i - theParams.Lower() + 1] = new Curve2D.ResD1
            {
                Point = new Point2D(cx + majR * cosU * xx + minR * sinU * yx,
                                     cy + majR * cosU * xy + minR * sinU * yy),
                D1 = new Vector2D(-majR * sinU * xx + minR * cosU * yx, -majR * sinU * xy + minR * cosU * yy),
            };
        }
        return result;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Curve2D.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve2D.ResD2>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve2D.ResD2> result = new(1, nb);

        Ellipse2D elips = _geom.Ellipse;
        Point2D center = elips.Location;
        Direction2D xDir = elips.XAxis.Direction;
        Direction2D yDir = elips.YAxis.Direction;
        double majR = elips.MajorRadius;
        double minR = elips.MinorRadius;

        double cx = center.X;
        double cy = center.Y;
        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            // D1 = -MajorR * sin(u) * XDir + MinorR * cos(u) * YDir
            // D2 = -MajorR * cos(u) * XDir - MinorR * sin(u) * YDir = -(P - Center)
            result[i - theParams.Lower() + 1] = new Curve2D.ResD2
            {
                Point = new Point2D(cx + majR * cosU * xx + minR * sinU * yx,
                                     cy + majR * cosU * xy + minR * sinU * yy),
                D1 = new Vector2D(-majR * sinU * xx + minR * cosU * yx, -majR * sinU * xy + minR * cosU * yy),
                D2 = new Vector2D(-majR * cosU * xx - minR * sinU * yx, -majR * cosU * xy - minR * sinU * yy),
            };
        }
        return result;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third.</summary>
    public NCollection_Array1<Curve2D.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve2D.ResD3>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve2D.ResD3> result = new(1, nb);

        Ellipse2D elips = _geom.Ellipse;
        Point2D center = elips.Location;
        Direction2D xDir = elips.XAxis.Direction;
        Direction2D yDir = elips.YAxis.Direction;
        double majR = elips.MajorRadius;
        double minR = elips.MinorRadius;

        double cx = center.X;
        double cy = center.Y;
        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            // D1 = -MajorR * sin(u) * XDir + MinorR * cos(u) * YDir
            // D2 = -MajorR * cos(u) * XDir - MinorR * sin(u) * YDir
            // D3 =  MajorR * sin(u) * XDir - MinorR * cos(u) * YDir = -D1
            result[i - theParams.Lower() + 1] = new Curve2D.ResD3
            {
                Point = new Point2D(cx + majR * cosU * xx + minR * sinU * yx,
                                     cy + majR * cosU * xy + minR * sinU * yy),
                D1 = new Vector2D(-majR * sinU * xx + minR * cosU * yx, -majR * sinU * xy + minR * cosU * yy),
                D2 = new Vector2D(-majR * cosU * xx - minR * sinU * yx, -majR * cosU * xy - minR * sinU * yy),
                D3 = new Vector2D(majR * sinU * xx - minR * cosU * yx, majR * sinU * xy - minR * cosU * yy),
            };
        }
        return result;
    }

    /// <summary>Evaluates the Nth derivative at all grid points; derivatives cycle with period 4.</summary>
    public NCollection_Array1<Vector2D> EvaluateGridDN(NCollection_Array1<double> theParams, int n)
    {
        if (_geom is null || theParams.IsEmpty() || n < 1)
        {
            return new NCollection_Array1<Vector2D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Vector2D> result = new(1, nb);

        Ellipse2D elips = _geom.Ellipse;
        Direction2D xDir = elips.XAxis.Direction;
        Direction2D yDir = elips.YAxis.Direction;
        double majR = elips.MajorRadius;
        double minR = elips.MinorRadius;

        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        // Ellipse derivatives are cyclic with period 4:
        // D1 = -MajR * sin(u) * X + MinR * cos(u) * Y  -> coefficients: (-sin, cos)
        // D2 = -MajR * cos(u) * X - MinR * sin(u) * Y  -> coefficients: (-cos, -sin)
        // D3 =  MajR * sin(u) * X - MinR * cos(u) * Y  -> coefficients: (sin, -cos)
        // D4 =  MajR * cos(u) * X + MinR * sin(u) * Y  -> coefficients: (cos, sin) = D0
        int phase = (n - 1) % 4;

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
                             new Vector2D(majR * coeffMajR * xx + minR * coeffMinR * yx,
                                          majR * coeffMajR * xy + minR * coeffMinR * yy));
        }
        return result;
    }
}
