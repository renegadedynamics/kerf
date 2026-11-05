// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Circle.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Circle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for 2D circle grid points: P(u) = C + R * (cos(u) * X + sin(u) * Y).
/// </summary>
internal sealed class CircleGrid2D : Geom2dGridEvalGrid2D.ICurveEvaluator
{
    private readonly CircleCurve2D? _geom;

    /// <summary>Constructor with the circle to evaluate (null gives empty results).</summary>
    public CircleGrid2D(CircleCurve2D? circle)
    {
        _geom = circle;
    }

    /// <summary>Returns the geometry.</summary>
    public CircleCurve2D? Geometry() => _geom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point2D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Point2D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Point2D> result = new(1, nb);

        // Extract circle data from geometry
        Circle2D circ = _geom.Circle;
        Point2D center = circ.Location;
        Direction2D xDir = circ.Position.XDirection;
        Direction2D yDir = circ.Position.YDirection;
        double radius = circ.Radius;

        // Pre-extract coordinates for performance
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

            result.SetValue(i - theParams.Lower() + 1,
                             new Point2D(cx + radius * (cosU * xx + sinU * yx),
                                          cy + radius * (cosU * xy + sinU * yy)));
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

        Circle2D circ = _geom.Circle;
        Point2D center = circ.Location;
        Direction2D xDir = circ.Position.XDirection;
        Direction2D yDir = circ.Position.YDirection;
        double radius = circ.Radius;

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

            // P = C + R * (cos(u) * X + sin(u) * Y)
            // D1 = R * (-sin(u) * X + cos(u) * Y)
            result[i - theParams.Lower() + 1] = new Curve2D.ResD1
            {
                Point = new Point2D(cx + radius * (cosU * xx + sinU * yx),
                                     cy + radius * (cosU * xy + sinU * yy)),
                D1 = new Vector2D(radius * (-sinU * xx + cosU * yx), radius * (-sinU * xy + cosU * yy)),
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

        Circle2D circ = _geom.Circle;
        Point2D center = circ.Location;
        Direction2D xDir = circ.Position.XDirection;
        Direction2D yDir = circ.Position.YDirection;
        double radius = circ.Radius;

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

            // P = C + R * (cos(u) * X + sin(u) * Y)
            // D1 = R * (-sin(u) * X + cos(u) * Y)
            // D2 = R * (-cos(u) * X - sin(u) * Y)
            result[i - theParams.Lower() + 1] = new Curve2D.ResD2
            {
                Point = new Point2D(cx + radius * (cosU * xx + sinU * yx),
                                     cy + radius * (cosU * xy + sinU * yy)),
                D1 = new Vector2D(radius * (-sinU * xx + cosU * yx), radius * (-sinU * xy + cosU * yy)),
                D2 = new Vector2D(radius * (-cosU * xx - sinU * yx), radius * (-cosU * xy - sinU * yy)),
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

        Circle2D circ = _geom.Circle;
        Point2D center = circ.Location;
        Direction2D xDir = circ.Position.XDirection;
        Direction2D yDir = circ.Position.YDirection;
        double radius = circ.Radius;

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

            // P = C + R * (cos(u) * X + sin(u) * Y)
            // D1 = R * (-sin(u) * X + cos(u) * Y)
            // D2 = R * (-cos(u) * X - sin(u) * Y)
            // D3 = R * (sin(u) * X - cos(u) * Y)
            result[i - theParams.Lower() + 1] = new Curve2D.ResD3
            {
                Point = new Point2D(cx + radius * (cosU * xx + sinU * yx),
                                     cy + radius * (cosU * xy + sinU * yy)),
                D1 = new Vector2D(radius * (-sinU * xx + cosU * yx), radius * (-sinU * xy + cosU * yy)),
                D2 = new Vector2D(radius * (-cosU * xx - sinU * yx), radius * (-cosU * xy - sinU * yy)),
                D3 = new Vector2D(radius * (sinU * xx - cosU * yx), radius * (sinU * xy - cosU * yy)),
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

        Circle2D circ = _geom.Circle;
        Direction2D xDir = circ.Position.XDirection;
        Direction2D yDir = circ.Position.YDirection;
        double radius = circ.Radius;

        double xx = xDir.X;
        double xy = xDir.Y;
        double yx = yDir.X;
        double yy = yDir.Y;

        // Circle derivatives are cyclic with period 4:
        // D1 = R * (-sin(u) * X + cos(u) * Y)  -> coefficients: (-sin, cos)
        // D2 = R * (-cos(u) * X - sin(u) * Y)  -> coefficients: (-cos, -sin)
        // D3 = R * (sin(u) * X - cos(u) * Y)   -> coefficients: (sin, -cos)
        // D4 = R * (cos(u) * X + sin(u) * Y)   -> coefficients: (cos, sin) = D0
        int phase = (n - 1) % 4; // 0->D1, 1->D2, 2->D3, 3->D4

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            double coeffX, coeffY;
            switch (phase)
            {
                case 0: // D1, D5, D9, ...
                    coeffX = -sinU;
                    coeffY = cosU;
                    break;
                case 1: // D2, D6, D10, ...
                    coeffX = -cosU;
                    coeffY = -sinU;
                    break;
                case 2: // D3, D7, D11, ...
                    coeffX = sinU;
                    coeffY = -cosU;
                    break;
                default: // D4, D8, D12, ... (case 3)
                    coeffX = cosU;
                    coeffY = sinU;
                    break;
            }

            result.SetValue(i - theParams.Lower() + 1,
                             new Vector2D(radius * (coeffX * xx + coeffY * yx),
                                          radius * (coeffX * xy + coeffY * yy)));
        }
        return result;
    }
}
