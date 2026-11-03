// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Parabola.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Parabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Efficient batch evaluator for 2D parabola grid points: P(u) = Center + (u^2 / 4F) * XDir + u * YDir.
/// </summary>
/// <remarks>
/// As in OCCT, YDir is the mirror axis direction turned by +90 degrees, not the parabola's own Y direction: on a
/// parabola with an indirect (clockwise) coordinate system the grid is the mirror image of the curve
/// (occt-issues.md).
/// </remarks>
public sealed class Geom2dGridEval_Parabola : Geom2dGridEval.ICurveEvaluator
{
    private readonly Geom2d_Parabola? myGeom;

    /// <summary>Constructor with the parabola to evaluate (null gives empty results).</summary>
    public Geom2dGridEval_Parabola(Geom2d_Parabola? theParabola)
    {
        myGeom = theParabola;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom2d_Parabola? Geometry() => myGeom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point2D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Point2D>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Point2D> aResult = new(1, aNb);

        Parabola2D aParab = myGeom.Parab2d();
        Point2D aCenter = aParab.Location();
        Direction2D aXDir = aParab.MirrorAxis().Direction();
        Direction2D aYDir = new(-aXDir.Y(), aXDir.X()); // OCCT: ignores the sense of the parabola's axes
        double aFocal = aParab.Focal(); // OCCT: no case for a zero focal length (ElCLib: a line)

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        double aCoeff = 1.0 / (4.0 * aFocal);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            // P = Center + (u^2 / 4F) * XDir + u * YDir
            double u2Term = u * u * aCoeff;
            aResult.SetValue(i - theParams.Lower() + 1,
                             new Point2D(aCX + u2Term * aXX + u * aYX, aCY + u2Term * aXY + u * aYY));
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD1>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD1> aResult = new(1, aNb);

        Parabola2D aParab = myGeom.Parab2d();
        Point2D aCenter = aParab.Location();
        Direction2D aXDir = aParab.MirrorAxis().Direction();
        Direction2D aYDir = new(-aXDir.Y(), aXDir.X()); // OCCT: ignores the sense of the parabola's axes
        double aFocal = aParab.Focal(); // OCCT: no case for a zero focal length (ElCLib: a line)

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        double aCoeff = 1.0 / (4.0 * aFocal);
        double aCoeff2 = 1.0 / (2.0 * aFocal);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            // P = Center + (u^2 / 4F) * XDir + u * YDir
            // D1 = (u / 2F) * XDir + YDir
            double u2Term = u * u * aCoeff;
            double d1Term = u * aCoeff2;
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD1
            {
                Point = new Point2D(aCX + u2Term * aXX + u * aYX, aCY + u2Term * aXY + u * aYY),
                D1 = new Vector2D(d1Term * aXX + aYX, d1Term * aXY + aYY),
            };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD2>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD2> aResult = new(1, aNb);

        Parabola2D aParab = myGeom.Parab2d();
        Point2D aCenter = aParab.Location();
        Direction2D aXDir = aParab.MirrorAxis().Direction();
        Direction2D aYDir = new(-aXDir.Y(), aXDir.X()); // OCCT: ignores the sense of the parabola's axes
        double aFocal = aParab.Focal(); // OCCT: no case for a zero focal length (ElCLib: a line)

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        double aCoeff = 1.0 / (4.0 * aFocal);
        double aCoeff2 = 1.0 / (2.0 * aFocal);

        // D2 is constant for parabola: (1/2F) * XDir
        Vector2D aD2 = new(aCoeff2 * aXX, aCoeff2 * aXY);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double u2Term = u * u * aCoeff;
            double d1Term = u * aCoeff2;
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD2
            {
                Point = new Point2D(aCX + u2Term * aXX + u * aYX, aCY + u2Term * aXY + u * aYY),
                D1 = new Vector2D(d1Term * aXX + aYX, d1Term * aXY + aYY),
                D2 = aD2,
            };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third (D3 is zero).</summary>
    public NCollection_Array1<Geom2d_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD3>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD3> aResult = new(1, aNb);

        Parabola2D aParab = myGeom.Parab2d();
        Point2D aCenter = aParab.Location();
        Direction2D aXDir = aParab.MirrorAxis().Direction();
        Direction2D aYDir = new(-aXDir.Y(), aXDir.X()); // OCCT: ignores the sense of the parabola's axes
        double aFocal = aParab.Focal(); // OCCT: no case for a zero focal length (ElCLib: a line)

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        double aCoeff = 1.0 / (4.0 * aFocal);
        double aCoeff2 = 1.0 / (2.0 * aFocal);

        Vector2D aD2 = new(aCoeff2 * aXX, aCoeff2 * aXY);
        Vector2D aD3 = new(0.0, 0.0);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double u2Term = u * u * aCoeff;
            double d1Term = u * aCoeff2;
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD3
            {
                Point = new Point2D(aCX + u2Term * aXX + u * aYX, aCY + u2Term * aXY + u * aYY),
                D1 = new Vector2D(d1Term * aXX + aYX, d1Term * aXY + aYY),
                D2 = aD2,
                D3 = aD3,
            };
        }
        return aResult;
    }

    /// <summary>Evaluates the Nth derivative at all grid points (zero for N &gt;= 3).</summary>
    public NCollection_Array1<Vector2D> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (myGeom is null || theParams.IsEmpty() || theN < 1)
        {
            return new NCollection_Array1<Vector2D>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Vector2D> aResult = new(1, aNb);

        Parabola2D aParab = myGeom.Parab2d();
        Direction2D aXDir = aParab.MirrorAxis().Direction();
        Direction2D aYDir = new(-aXDir.Y(), aXDir.X()); // OCCT: ignores the sense of the parabola's axes
        double aFocal = aParab.Focal(); // OCCT: no case for a zero focal length (ElCLib: a line)

        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        double aCoeff2 = 1.0 / (2.0 * aFocal);

        if (theN == 1)
        {
            // D1 = (u/2F) * X + Y (depends on u)
            for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
            {
                double u = theParams.Value(i);
                double d1Term = u * aCoeff2;
                aResult.SetValue(i - theParams.Lower() + 1, new Vector2D(d1Term * aXX + aYX, d1Term * aXY + aYY));
            }
        }
        else if (theN == 2)
        {
            // D2 = (1/2F) * X (constant)
            Vector2D aD2 = new(aCoeff2 * aXX, aCoeff2 * aXY);
            for (int i = 1; i <= aNb; ++i)
            {
                aResult.SetValue(i, aD2);
            }
        }
        else
        {
            // DN = 0 for N >= 3
            Vector2D aZero = new(0.0, 0.0);
            for (int i = 1; i <= aNb; ++i)
            {
                aResult.SetValue(i, aZero);
            }
        }
        return aResult;
    }
}
