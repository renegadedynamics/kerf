// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Parabola.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Parabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for 2D parabola grid points: P(u) = Center + (u^2 / 4F) * XDir + u * YDir.
/// </summary>
/// <remarks>
/// As in OCCT, YDir is the mirror axis direction turned by +90 degrees, not the parabola's own Y direction: on a
/// parabola with an indirect (clockwise) coordinate system the grid is the mirror image of the curve
/// (occt-issues.md).
/// </remarks>
internal sealed class ParabolaGrid2D : Geom2dGridEvalGrid2D.ICurveEvaluator
{
    private readonly ParabolaCurve2D? _geom;

    /// <summary>Constructor with the parabola to evaluate (null gives empty results).</summary>
    public ParabolaGrid2D(ParabolaCurve2D? parabola)
    {
        _geom = parabola;
    }

    /// <summary>Returns the geometry.</summary>
    public ParabolaCurve2D? Geometry() => _geom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point2D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Point2D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Point2D> result = new(1, nb);

        Parabola2D parab = _geom.Parabola;
        Point2D center = parab.Location;
        Direction2D xDir = parab.MirrorAxis.Direction;
        Direction2D yDir = new(-xDir.Y, xDir.X); // OCCT: ignores the sense of the parabola's axes
        double focal = parab.Focal; // OCCT: no case for a zero focal length (ElCLib: a line)

        double cx = center.X;
        double cy = center.Y;
        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        double coeff = 1.0 / (4.0 * focal);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            // P = Center + (u^2 / 4F) * XDir + u * YDir
            double u2Term = u * u * coeff;
            result.SetValue(i - theParams.Lower() + 1,
                             new Point2D(cx + u2Term * xx + u * yx, cy + u2Term * xy + u * yy));
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

        Parabola2D parab = _geom.Parabola;
        Point2D center = parab.Location;
        Direction2D xDir = parab.MirrorAxis.Direction;
        Direction2D yDir = new(-xDir.Y, xDir.X); // OCCT: ignores the sense of the parabola's axes
        double focal = parab.Focal; // OCCT: no case for a zero focal length (ElCLib: a line)

        double cx = center.X;
        double cy = center.Y;
        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        double coeff = 1.0 / (4.0 * focal);
        double coeff2 = 1.0 / (2.0 * focal);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            // P = Center + (u^2 / 4F) * XDir + u * YDir
            // D1 = (u / 2F) * XDir + YDir
            double u2Term = u * u * coeff;
            double d1Term = u * coeff2;
            result[i - theParams.Lower() + 1] = new Curve2D.ResD1
            {
                Point = new Point2D(cx + u2Term * xx + u * yx, cy + u2Term * xy + u * yy),
                D1 = new Vector2D(d1Term * xx + yx, d1Term * xy + yy),
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

        Parabola2D parab = _geom.Parabola;
        Point2D center = parab.Location;
        Direction2D xDir = parab.MirrorAxis.Direction;
        Direction2D yDir = new(-xDir.Y, xDir.X); // OCCT: ignores the sense of the parabola's axes
        double focal = parab.Focal; // OCCT: no case for a zero focal length (ElCLib: a line)

        double cx = center.X;
        double cy = center.Y;
        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        double coeff = 1.0 / (4.0 * focal);
        double coeff2 = 1.0 / (2.0 * focal);

        // D2 is constant for parabola: (1/2F) * XDir
        Vector2D d2 = new(coeff2 * xx, coeff2 * xy);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double u2Term = u * u * coeff;
            double d1Term = u * coeff2;
            result[i - theParams.Lower() + 1] = new Curve2D.ResD2
            {
                Point = new Point2D(cx + u2Term * xx + u * yx, cy + u2Term * xy + u * yy),
                D1 = new Vector2D(d1Term * xx + yx, d1Term * xy + yy),
                D2 = d2,
            };
        }
        return result;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third (D3 is zero).</summary>
    public NCollection_Array1<Curve2D.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve2D.ResD3>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve2D.ResD3> result = new(1, nb);

        Parabola2D parab = _geom.Parabola;
        Point2D center = parab.Location;
        Direction2D xDir = parab.MirrorAxis.Direction;
        Direction2D yDir = new(-xDir.Y, xDir.X); // OCCT: ignores the sense of the parabola's axes
        double focal = parab.Focal; // OCCT: no case for a zero focal length (ElCLib: a line)

        double cx = center.X;
        double cy = center.Y;
        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        double coeff = 1.0 / (4.0 * focal);
        double coeff2 = 1.0 / (2.0 * focal);

        Vector2D d2 = new(coeff2 * xx, coeff2 * xy);
        Vector2D d3 = new(0.0, 0.0);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double u2Term = u * u * coeff;
            double d1Term = u * coeff2;
            result[i - theParams.Lower() + 1] = new Curve2D.ResD3
            {
                Point = new Point2D(cx + u2Term * xx + u * yx, cy + u2Term * xy + u * yy),
                D1 = new Vector2D(d1Term * xx + yx, d1Term * xy + yy),
                D2 = d2,
                D3 = d3,
            };
        }
        return result;
    }

    /// <summary>Evaluates the Nth derivative at all grid points (zero for N &gt;= 3).</summary>
    public NCollection_Array1<Vector2D> EvaluateGridDN(NCollection_Array1<double> theParams, int n)
    {
        if (_geom is null || theParams.IsEmpty() || n < 1)
        {
            return new NCollection_Array1<Vector2D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Vector2D> result = new(1, nb);

        Parabola2D parab = _geom.Parabola;
        Direction2D xDir = parab.MirrorAxis.Direction;
        Direction2D yDir = new(-xDir.Y, xDir.X); // OCCT: ignores the sense of the parabola's axes
        double focal = parab.Focal; // OCCT: no case for a zero focal length (ElCLib: a line)

        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        double coeff2 = 1.0 / (2.0 * focal);

        if (n == 1)
        {
            // D1 = (u/2F) * X + Y (depends on u)
            for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
            {
                double u = theParams.Value(i);
                double d1Term = u * coeff2;
                result.SetValue(i - theParams.Lower() + 1, new Vector2D(d1Term * xx + yx, d1Term * xy + yy));
            }
        }
        else if (n == 2)
        {
            // D2 = (1/2F) * X (constant)
            Vector2D d2 = new(coeff2 * xx, coeff2 * xy);
            for (int i = 1; i <= nb; ++i)
            {
                result.SetValue(i, d2);
            }
        }
        else
        {
            // DN = 0 for N >= 3
            Vector2D zero = new(0.0, 0.0);
            for (int i = 1; i <= nb; ++i)
            {
                result.SetValue(i, zero);
            }
        }
        return result;
    }
}
