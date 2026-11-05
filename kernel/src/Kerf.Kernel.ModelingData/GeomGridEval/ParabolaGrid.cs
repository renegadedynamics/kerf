// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Parabola.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Parabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for parabola grid points: P(u) = C + (u^2 / 4F) * X + u * Y.
/// </summary>
/// <remarks>
/// As in OCCT, a parabola of focal length 0 (a line, which ElCLib evaluates as Location + U * XDir) gives
/// infinite and NaN values (occt-issues.md).
/// </remarks>
internal sealed class ParabolaGrid : GeomGridEvalGrid.ICurveEvaluator
{
    private readonly ParabolaCurve3D? _geom;

    /// <summary>Constructor with the parabola to evaluate (null gives empty results).</summary>
    public ParabolaGrid(ParabolaCurve3D? parabola)
    {
        _geom = parabola;
    }

    /// <summary>Returns the geometry.</summary>
    public ParabolaCurve3D? Geometry() => _geom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point3D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Point3D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Point3D> result = new(1, nb);

        Parabola3D parab = _geom.Parabola;
        Point3D center = parab.Location;
        Direction3D xDir = parab.XAxis.Direction;
        Direction3D yDir = parab.YAxis.Direction;
        double focal = parab.Focal;

        double cx = center.X;
        double cy = center.Y;
        double cz = center.Z;
        double xx = xDir.X;
        double xy = xDir.Y;
        double xz = xDir.Z;
        double yx = yDir.X;
        double yy = yDir.Y;
        double yz = yDir.Z;

        // OCCT: no case for a zero focal length (ElCLib evaluates that degenerate parabola as a line)
        double coeff = 1.0 / (4.0 * focal);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);

            // P = Center + (u^2 / 4F) * XDir + u * YDir
            double u2Term = u * u * coeff;
            result.SetValue(i - theParams.Lower() + 1,
                             new Point3D(cx + u2Term * xx + u * yx,
                                        cy + u2Term * xy + u * yy,
                                        cz + u2Term * xz + u * yz));
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

        Parabola3D parab = _geom.Parabola;
        Point3D center = parab.Location;
        Direction3D xDir = parab.XAxis.Direction;
        Direction3D yDir = parab.YAxis.Direction;
        double focal = parab.Focal;

        double cx = center.X;
        double cy = center.Y;
        double cz = center.Z;
        double xx = xDir.X;
        double xy = xDir.Y;
        double xz = xDir.Z;
        double yx = yDir.X;
        double yy = yDir.Y;
        double yz = yDir.Z;

        // OCCT: no case for a zero focal length (ElCLib evaluates that degenerate parabola as a line)
        double coeff = 1.0 / (4.0 * focal);
        double coeff2 = 1.0 / (2.0 * focal); // Derivative of u^2/4F is 2u/4F = u/2F

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);

            // P = Center + (u^2 / 4F) * XDir + u * YDir
            // D1 = (u / 2F) * XDir + YDir
            double u2Term = u * u * coeff;
            double d1Term = u * coeff2;
            result[i - theParams.Lower() + 1] = new Curve3D.ResD1
            {
                Point = new Point3D(cx + u2Term * xx + u * yx,
                                   cy + u2Term * xy + u * yy,
                                   cz + u2Term * xz + u * yz),
                D1 = new Vector3D(d1Term * xx + yx,
                                d1Term * xy + yy,
                                d1Term * xz + yz),
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

        Parabola3D parab = _geom.Parabola;
        Point3D center = parab.Location;
        Direction3D xDir = parab.XAxis.Direction;
        Direction3D yDir = parab.YAxis.Direction;
        double focal = parab.Focal;

        double cx = center.X;
        double cy = center.Y;
        double cz = center.Z;
        double xx = xDir.X;
        double xy = xDir.Y;
        double xz = xDir.Z;
        double yx = yDir.X;
        double yy = yDir.Y;
        double yz = yDir.Z;

        // OCCT: no case for a zero focal length (ElCLib evaluates that degenerate parabola as a line)
        double coeff = 1.0 / (4.0 * focal);
        double coeff2 = 1.0 / (2.0 * focal);
        double coeff3 = coeff2; // Derivative of u/2F is 1/2F

        // D2 is constant for parabola: (1/2F) * XDir
        Vector3D d2 = new(coeff3 * xx, coeff3 * xy, coeff3 * xz);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);

            // P = Center + (u^2 / 4F) * XDir + u * YDir
            // D1 = (u / 2F) * XDir + YDir
            // D2 = (1 / 2F) * XDir
            double u2Term = u * u * coeff;
            double d1Term = u * coeff2;
            result[i - theParams.Lower() + 1] = new Curve3D.ResD2
            {
                Point = new Point3D(cx + u2Term * xx + u * yx,
                                   cy + u2Term * xy + u * yy,
                                   cz + u2Term * xz + u * yz),
                D1 = new Vector3D(d1Term * xx + yx,
                                d1Term * xy + yy,
                                d1Term * xz + yz),
                D2 = d2,
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

        Parabola3D parab = _geom.Parabola;
        Point3D center = parab.Location;
        Direction3D xDir = parab.XAxis.Direction;
        Direction3D yDir = parab.YAxis.Direction;
        double focal = parab.Focal;

        double cx = center.X;
        double cy = center.Y;
        double cz = center.Z;
        double xx = xDir.X;
        double xy = xDir.Y;
        double xz = xDir.Z;
        double yx = yDir.X;
        double yy = yDir.Y;
        double yz = yDir.Z;

        // OCCT: no case for a zero focal length (ElCLib evaluates that degenerate parabola as a line)
        double coeff = 1.0 / (4.0 * focal);
        double coeff2 = 1.0 / (2.0 * focal);
        double coeff3 = coeff2;

        Vector3D d2 = new(coeff3 * xx, coeff3 * xy, coeff3 * xz);
        Vector3D d3 = new(0.0, 0.0, 0.0); // D3 is 0

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);

            double u2Term = u * u * coeff;
            double d1Term = u * coeff2;
            result[i - theParams.Lower() + 1] = new Curve3D.ResD3
            {
                Point = new Point3D(cx + u2Term * xx + u * yx,
                                   cy + u2Term * xy + u * yy,
                                   cz + u2Term * xz + u * yz),
                D1 = new Vector3D(d1Term * xx + yx,
                                d1Term * xy + yy,
                                d1Term * xz + yz),
                D2 = d2,
                D3 = d3,
            };
        }
        return result;
    }

    /// <summary>Evaluates the Nth derivative (N &gt;= 1) at all grid points: D1 depends on u, D2 is constant, DN is 0 above.</summary>
    public NCollection_Array1<Vector3D> EvaluateGridDN(NCollection_Array1<double> theParams, int n)
    {
        if (_geom is null || theParams.IsEmpty() || n < 1)
        {
            return new NCollection_Array1<Vector3D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Vector3D> result = new(1, nb);

        Parabola3D parab = _geom.Parabola;
        Direction3D xDir = parab.XAxis.Direction;
        Direction3D yDir = parab.YAxis.Direction;
        double focal = parab.Focal;

        double xx = xDir.X;
        double xy = xDir.Y;
        double xz = xDir.Z;
        double yx = yDir.X;
        double yy = yDir.Y;
        double yz = yDir.Z;

        // OCCT: no case for a zero focal length (ElCLib evaluates that degenerate parabola as a line)
        double coeff2 = 1.0 / (2.0 * focal);

        if (n == 1)
        {
            // D1 = (u/2F) * X + Y (depends on u)
            for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
            {
                double u = theParams.Value(i);
                double d1Term = u * coeff2;
                result.SetValue(i - theParams.Lower() + 1,
                                 new Vector3D(d1Term * xx + yx, d1Term * xy + yy, d1Term * xz + yz));
            }
        }
        else if (n == 2)
        {
            // D2 = (1/2F) * X (constant)
            Vector3D d2 = new(coeff2 * xx, coeff2 * xy, coeff2 * xz);
            for (int i = 1; i <= nb; ++i)
            {
                result.SetValue(i, d2);
            }
        }
        else
        {
            // DN = 0 for N >= 3
            Vector3D zero = new(0.0, 0.0, 0.0);
            for (int i = 1; i <= nb; ++i)
            {
                result.SetValue(i, zero);
            }
        }
        return result;
    }
}
