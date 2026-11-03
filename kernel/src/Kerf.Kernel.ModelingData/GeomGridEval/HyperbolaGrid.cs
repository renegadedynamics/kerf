// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Hyperbola.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Hyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for hyperbola grid points: P(u) = C + MajR * cosh(u) * X + MinR * sinh(u) * Y.
/// </summary>
public sealed class HyperbolaGrid : GeomGridEvalGrid.ICurveEvaluator
{
    private readonly HyperbolaCurve3D? _geom;

    /// <summary>Constructor with the hyperbola to evaluate (null gives empty results).</summary>
    public HyperbolaGrid(HyperbolaCurve3D? hyperbola)
    {
        _geom = hyperbola;
    }

    /// <summary>Returns the geometry.</summary>
    public HyperbolaCurve3D? Geometry() => _geom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point3D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Point3D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Point3D> result = new(1, nb);

        Hyperbola3D hypr = _geom.Hypr();
        Point3D center = hypr.Location();
        Direction3D xDir = hypr.XAxis().Direction();
        Direction3D yDir = hypr.YAxis().Direction();
        double majR = hypr.MajorRadius();
        double minR = hypr.MinorRadius();

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
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            // P = Center + MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir
            result.SetValue(i - theParams.Lower() + 1,
                             new Point3D(cx + majR * coshU * xx + minR * sinhU * yx,
                                        cy + majR * coshU * xy + minR * sinhU * yy,
                                        cz + majR * coshU * xz + minR * sinhU * yz));
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

        Hyperbola3D hypr = _geom.Hypr();
        Point3D center = hypr.Location();
        Direction3D xDir = hypr.XAxis().Direction();
        Direction3D yDir = hypr.YAxis().Direction();
        double majR = hypr.MajorRadius();
        double minR = hypr.MinorRadius();

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
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            // P = Center + MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir
            // D1 = MajorR * sinh(u) * XDir + MinorR * cosh(u) * YDir
            result[i - theParams.Lower() + 1] = new Curve3D.ResD1
            {
                Point = new Point3D(cx + majR * coshU * xx + minR * sinhU * yx,
                                   cy + majR * coshU * xy + minR * sinhU * yy,
                                   cz + majR * coshU * xz + minR * sinhU * yz),
                D1 = new Vector3D(majR * sinhU * xx + minR * coshU * yx,
                                majR * sinhU * xy + minR * coshU * yy,
                                majR * sinhU * xz + minR * coshU * yz),
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

        Hyperbola3D hypr = _geom.Hypr();
        Point3D center = hypr.Location();
        Direction3D xDir = hypr.XAxis().Direction();
        Direction3D yDir = hypr.YAxis().Direction();
        double majR = hypr.MajorRadius();
        double minR = hypr.MinorRadius();

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
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            // P = Center + MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir
            // D1 = MajorR * sinh(u) * XDir + MinorR * cosh(u) * YDir
            // D2 = MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir
            result[i - theParams.Lower() + 1] = new Curve3D.ResD2
            {
                Point = new Point3D(cx + majR * coshU * xx + minR * sinhU * yx,
                                   cy + majR * coshU * xy + minR * sinhU * yy,
                                   cz + majR * coshU * xz + minR * sinhU * yz),
                D1 = new Vector3D(majR * sinhU * xx + minR * coshU * yx,
                                majR * sinhU * xy + minR * coshU * yy,
                                majR * sinhU * xz + minR * coshU * yz),
                D2 = new Vector3D(majR * coshU * xx + minR * sinhU * yx,
                                majR * coshU * xy + minR * sinhU * yy,
                                majR * coshU * xz + minR * sinhU * yz),
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

        Hyperbola3D hypr = _geom.Hypr();
        Point3D center = hypr.Location();
        Direction3D xDir = hypr.XAxis().Direction();
        Direction3D yDir = hypr.YAxis().Direction();
        double majR = hypr.MajorRadius();
        double minR = hypr.MinorRadius();

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
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            // P = Center + MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir
            // D1 = MajorR * sinh(u) * XDir + MinorR * cosh(u) * YDir
            // D2 = MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir
            // D3 = MajorR * sinh(u) * XDir + MinorR * cosh(u) * YDir = D1
            result[i - theParams.Lower() + 1] = new Curve3D.ResD3
            {
                Point = new Point3D(cx + majR * coshU * xx + minR * sinhU * yx,
                                   cy + majR * coshU * xy + minR * sinhU * yy,
                                   cz + majR * coshU * xz + minR * sinhU * yz),
                D1 = new Vector3D(majR * sinhU * xx + minR * coshU * yx,
                                majR * sinhU * xy + minR * coshU * yy,
                                majR * sinhU * xz + minR * coshU * yz),
                D2 = new Vector3D(majR * coshU * xx + minR * sinhU * yx,
                                majR * coshU * xy + minR * sinhU * yy,
                                majR * coshU * xz + minR * sinhU * yz),
                D3 = new Vector3D(majR * sinhU * xx + minR * coshU * yx,
                                majR * sinhU * xy + minR * coshU * yy,
                                majR * sinhU * xz + minR * coshU * yz),
            };
        }
        return result;
    }

    /// <summary>Evaluates the Nth derivative (N &gt;= 1) at all grid points; the derivatives cycle with period 2.</summary>
    public NCollection_Array1<Vector3D> EvaluateGridDN(NCollection_Array1<double> theParams, int n)
    {
        if (_geom is null || theParams.IsEmpty() || n < 1)
        {
            return new NCollection_Array1<Vector3D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Vector3D> result = new(1, nb);

        Hyperbola3D hypr = _geom.Hypr();
        Direction3D xDir = hypr.XAxis().Direction();
        Direction3D yDir = hypr.YAxis().Direction();
        double majR = hypr.MajorRadius();
        double minR = hypr.MinorRadius();

        double xx = xDir.X();
        double xy = xDir.Y();
        double xz = xDir.Z();
        double yx = yDir.X();
        double yy = yDir.Y();
        double yz = yDir.Z();

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
                // Odd derivatives (D1, D3, D5, ...): coefficients (sinh, cosh)
                result.SetValue(i - theParams.Lower() + 1,
                                 new Vector3D(majR * sinhU * xx + minR * coshU * yx,
                                            majR * sinhU * xy + minR * coshU * yy,
                                            majR * sinhU * xz + minR * coshU * yz));
            }
            else
            {
                // Even derivatives (D2, D4, D6, ...): coefficients (cosh, sinh) = D0
                result.SetValue(i - theParams.Lower() + 1,
                                 new Vector3D(majR * coshU * xx + minR * sinhU * yx,
                                            majR * coshU * xy + minR * sinhU * yy,
                                            majR * coshU * xz + minR * sinhU * yz));
            }
        }
        return result;
    }
}
