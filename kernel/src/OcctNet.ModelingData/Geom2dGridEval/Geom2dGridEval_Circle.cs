// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Circle.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Circle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Efficient batch evaluator for 2D circle grid points: P(u) = C + R * (cos(u) * X + sin(u) * Y).
/// </summary>
public sealed class Geom2dGridEval_Circle : Geom2dGridEval.ICurveEvaluator
{
    private readonly Geom2d_Circle? myGeom;

    /// <summary>Constructor with the circle to evaluate (null gives empty results).</summary>
    public Geom2dGridEval_Circle(Geom2d_Circle? theCircle)
    {
        myGeom = theCircle;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom2d_Circle? Geometry() => myGeom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point2D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Point2D>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Point2D> aResult = new(1, aNb);

        // Extract circle data from geometry
        Circle2D aCirc = myGeom.Circ2d();
        Point2D aCenter = aCirc.Location();
        Direction2D aXDir = aCirc.Position().XDirection();
        Direction2D aYDir = aCirc.Position().YDirection();
        double aRadius = aCirc.Radius();

        // Pre-extract coordinates for performance
        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            aResult.SetValue(i - theParams.Lower() + 1,
                             new Point2D(aCX + aRadius * (cosU * aXX + sinU * aYX),
                                          aCY + aRadius * (cosU * aXY + sinU * aYY)));
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

        Circle2D aCirc = myGeom.Circ2d();
        Point2D aCenter = aCirc.Location();
        Direction2D aXDir = aCirc.Position().XDirection();
        Direction2D aYDir = aCirc.Position().YDirection();
        double aRadius = aCirc.Radius();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = C + R * (cos(u) * X + sin(u) * Y)
            // D1 = R * (-sin(u) * X + cos(u) * Y)
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD1
            {
                Point = new Point2D(aCX + aRadius * (cosU * aXX + sinU * aYX),
                                     aCY + aRadius * (cosU * aXY + sinU * aYY)),
                D1 = new Vector2D(aRadius * (-sinU * aXX + cosU * aYX), aRadius * (-sinU * aXY + cosU * aYY)),
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

        Circle2D aCirc = myGeom.Circ2d();
        Point2D aCenter = aCirc.Location();
        Direction2D aXDir = aCirc.Position().XDirection();
        Direction2D aYDir = aCirc.Position().YDirection();
        double aRadius = aCirc.Radius();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = C + R * (cos(u) * X + sin(u) * Y)
            // D1 = R * (-sin(u) * X + cos(u) * Y)
            // D2 = R * (-cos(u) * X - sin(u) * Y)
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD2
            {
                Point = new Point2D(aCX + aRadius * (cosU * aXX + sinU * aYX),
                                     aCY + aRadius * (cosU * aXY + sinU * aYY)),
                D1 = new Vector2D(aRadius * (-sinU * aXX + cosU * aYX), aRadius * (-sinU * aXY + cosU * aYY)),
                D2 = new Vector2D(aRadius * (-cosU * aXX - sinU * aYX), aRadius * (-cosU * aXY - sinU * aYY)),
            };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD3>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD3> aResult = new(1, aNb);

        Circle2D aCirc = myGeom.Circ2d();
        Point2D aCenter = aCirc.Location();
        Direction2D aXDir = aCirc.Position().XDirection();
        Direction2D aYDir = aCirc.Position().YDirection();
        double aRadius = aCirc.Radius();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = C + R * (cos(u) * X + sin(u) * Y)
            // D1 = R * (-sin(u) * X + cos(u) * Y)
            // D2 = R * (-cos(u) * X - sin(u) * Y)
            // D3 = R * (sin(u) * X - cos(u) * Y)
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD3
            {
                Point = new Point2D(aCX + aRadius * (cosU * aXX + sinU * aYX),
                                     aCY + aRadius * (cosU * aXY + sinU * aYY)),
                D1 = new Vector2D(aRadius * (-sinU * aXX + cosU * aYX), aRadius * (-sinU * aXY + cosU * aYY)),
                D2 = new Vector2D(aRadius * (-cosU * aXX - sinU * aYX), aRadius * (-cosU * aXY - sinU * aYY)),
                D3 = new Vector2D(aRadius * (sinU * aXX - cosU * aYX), aRadius * (sinU * aXY - cosU * aYY)),
            };
        }
        return aResult;
    }

    /// <summary>Evaluates the Nth derivative at all grid points; derivatives cycle with period 4.</summary>
    public NCollection_Array1<Vector2D> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (myGeom is null || theParams.IsEmpty() || theN < 1)
        {
            return new NCollection_Array1<Vector2D>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Vector2D> aResult = new(1, aNb);

        Circle2D aCirc = myGeom.Circ2d();
        Direction2D aXDir = aCirc.Position().XDirection();
        Direction2D aYDir = aCirc.Position().YDirection();
        double aRadius = aCirc.Radius();

        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        // Circle derivatives are cyclic with period 4:
        // D1 = R * (-sin(u) * X + cos(u) * Y)  -> coefficients: (-sin, cos)
        // D2 = R * (-cos(u) * X - sin(u) * Y)  -> coefficients: (-cos, -sin)
        // D3 = R * (sin(u) * X - cos(u) * Y)   -> coefficients: (sin, -cos)
        // D4 = R * (cos(u) * X + sin(u) * Y)   -> coefficients: (cos, sin) = D0
        int aPhase = (theN - 1) % 4; // 0->D1, 1->D2, 2->D3, 3->D4

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            double aCoeffX, aCoeffY;
            switch (aPhase)
            {
                case 0: // D1, D5, D9, ...
                    aCoeffX = -sinU;
                    aCoeffY = cosU;
                    break;
                case 1: // D2, D6, D10, ...
                    aCoeffX = -cosU;
                    aCoeffY = -sinU;
                    break;
                case 2: // D3, D7, D11, ...
                    aCoeffX = sinU;
                    aCoeffY = -cosU;
                    break;
                default: // D4, D8, D12, ... (case 3)
                    aCoeffX = cosU;
                    aCoeffY = sinU;
                    break;
            }

            aResult.SetValue(i - theParams.Lower() + 1,
                             new Vector2D(aRadius * (aCoeffX * aXX + aCoeffY * aYX),
                                          aRadius * (aCoeffX * aXY + aCoeffY * aYY)));
        }
        return aResult;
    }
}
