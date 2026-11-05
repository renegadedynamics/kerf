// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Hyperbola.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Hyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for 2D hyperbola grid points:
/// P(u) = Center + MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir.
/// </summary>
internal sealed class HyperbolaGrid2D : Geom2dGridEvalGrid2D.ICurveEvaluator
{
    private readonly HyperbolaCurve2D? _geom;

    /// <summary>Constructor with the hyperbola to evaluate (null gives empty results).</summary>
    public HyperbolaGrid2D(HyperbolaCurve2D? hyperbola)
    {
        _geom = hyperbola;
    }

    /// <summary>Returns the geometry.</summary>
    public HyperbolaCurve2D? Geometry() => _geom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point2D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Point2D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Point2D> result = new(1, nb);

        Hyperbola2D hypr = _geom.Hyperbola;
        Point2D center = hypr.Location;
        Direction2D xDir = hypr.XAxis.Direction;
        Direction2D yDir = hypr.YAxis.Direction;
        double majR = hypr.MajorRadius;
        double minR = hypr.MinorRadius;

        double cx = center.X;
        double cy = center.Y;
        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            result.SetValue(i - theParams.Lower() + 1,
                             new Point2D(cx + majR * coshU * xx + minR * sinhU * yx,
                                          cy + majR * coshU * xy + minR * sinhU * yy));
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

        Hyperbola2D hypr = _geom.Hyperbola;
        Point2D center = hypr.Location;
        Direction2D xDir = hypr.XAxis.Direction;
        Direction2D yDir = hypr.YAxis.Direction;
        double majR = hypr.MajorRadius;
        double minR = hypr.MinorRadius;

        double cx = center.X;
        double cy = center.Y;
        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            // P = Center + MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir
            // D1 = MajorR * sinh(u) * XDir + MinorR * cosh(u) * YDir
            result[i - theParams.Lower() + 1] = new Curve2D.ResD1
            {
                Point = new Point2D(cx + majR * coshU * xx + minR * sinhU * yx,
                                     cy + majR * coshU * xy + minR * sinhU * yy),
                D1 = new Vector2D(majR * sinhU * xx + minR * coshU * yx,
                                  majR * sinhU * xy + minR * coshU * yy),
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

        Hyperbola2D hypr = _geom.Hyperbola;
        Point2D center = hypr.Location;
        Direction2D xDir = hypr.XAxis.Direction;
        Direction2D yDir = hypr.YAxis.Direction;
        double majR = hypr.MajorRadius;
        double minR = hypr.MinorRadius;

        double cx = center.X;
        double cy = center.Y;
        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            // D2 = MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir = (P - Center)
            result[i - theParams.Lower() + 1] = new Curve2D.ResD2
            {
                Point = new Point2D(cx + majR * coshU * xx + minR * sinhU * yx,
                                     cy + majR * coshU * xy + minR * sinhU * yy),
                D1 = new Vector2D(majR * sinhU * xx + minR * coshU * yx,
                                  majR * sinhU * xy + minR * coshU * yy),
                D2 = new Vector2D(majR * coshU * xx + minR * sinhU * yx,
                                  majR * coshU * xy + minR * sinhU * yy),
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

        Hyperbola2D hypr = _geom.Hyperbola;
        Point2D center = hypr.Location;
        Direction2D xDir = hypr.XAxis.Direction;
        Direction2D yDir = hypr.YAxis.Direction;
        double majR = hypr.MajorRadius;
        double minR = hypr.MinorRadius;

        double cx = center.X;
        double cy = center.Y;
        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            // D3 = MajorR * sinh(u) * XDir + MinorR * cosh(u) * YDir = D1
            result[i - theParams.Lower() + 1] = new Curve2D.ResD3
            {
                Point = new Point2D(cx + majR * coshU * xx + minR * sinhU * yx,
                                     cy + majR * coshU * xy + minR * sinhU * yy),
                D1 = new Vector2D(majR * sinhU * xx + minR * coshU * yx,
                                  majR * sinhU * xy + minR * coshU * yy),
                D2 = new Vector2D(majR * coshU * xx + minR * sinhU * yx,
                                  majR * coshU * xy + minR * sinhU * yy),
                D3 = new Vector2D(majR * sinhU * xx + minR * coshU * yx,
                                  majR * sinhU * xy + minR * coshU * yy),
            };
        }
        return result;
    }

    /// <summary>Evaluates the Nth derivative at all grid points; derivatives cycle with period 2.</summary>
    public NCollection_Array1<Vector2D> EvaluateGridDN(NCollection_Array1<double> theParams, int n)
    {
        if (_geom is null || theParams.IsEmpty() || n < 1)
        {
            return new NCollection_Array1<Vector2D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Vector2D> result = new(1, nb);

        Hyperbola2D hypr = _geom.Hyperbola;
        Direction2D xDir = hypr.XAxis.Direction;
        Direction2D yDir = hypr.YAxis.Direction;
        double majR = hypr.MajorRadius;
        double minR = hypr.MinorRadius;

        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        // Hyperbola derivatives are cyclic with period 2:
        // D0 = MajR * cosh(u) * X + MinR * sinh(u) * Y  -> coefficients: (cosh, sinh)
        // D1 = MajR * sinh(u) * X + MinR * cosh(u) * Y  -> coefficients: (sinh, cosh)
        // D2 = D0, D3 = D1, etc.
        bool isOdd = (n % 2) == 1;

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            if (isOdd)
            {
                result.SetValue(i - theParams.Lower() + 1,
                                 new Vector2D(majR * sinhU * xx + minR * coshU * yx,
                                              majR * sinhU * xy + minR * coshU * yy));
            }
            else
            {
                result.SetValue(i - theParams.Lower() + 1,
                                 new Vector2D(majR * coshU * xx + minR * sinhU * yx,
                                              majR * coshU * xy + minR * sinhU * yy));
            }
        }
        return result;
    }
}
