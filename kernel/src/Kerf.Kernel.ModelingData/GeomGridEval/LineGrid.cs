// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Line.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for line grid points, by the direct formula P(t) = Location + t * Direction.
/// </summary>
internal sealed class LineGrid : GeomGridEvalGrid.ICurveEvaluator
{
    private readonly LineCurve3D? _geom;

    /// <summary>Constructor with the line to evaluate (null gives empty results).</summary>
    public LineGrid(LineCurve3D? line)
    {
        _geom = line;
    }

    /// <summary>Returns the geometry.</summary>
    public LineCurve3D? Geometry() => _geom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point3D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Point3D>();
        }

        NCollection_Array1<Point3D> result = new(1, theParams.Length());

        // Extract line data
        Line3D lin = _geom.Line;
        Point3D loc = lin.Location;
        Direction3D dir = lin.Direction;

        // Pre-extract coordinates for vectorization potential
        double locX = loc.X;
        double locY = loc.Y;
        double locZ = loc.Z;
        double dirX = dir.X;
        double dirY = dir.Y;
        double dirZ = dir.Z;

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double t = theParams.Value(i);
            result.SetValue(i - theParams.Lower() + 1,
                             new Point3D(locX + t * dirX, locY + t * dirY, locZ + t * dirZ));
        }
        return result;
    }

    /// <summary>Evaluates all grid points with the first derivative, which is constant for a line.</summary>
    public NCollection_Array1<Curve3D.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve3D.ResD1>();
        }

        NCollection_Array1<Curve3D.ResD1> result = new(1, theParams.Length());

        Line3D lin = _geom.Line;
        Point3D loc = lin.Location;
        Direction3D dir = lin.Direction;

        double locX = loc.X;
        double locY = loc.Y;
        double locZ = loc.Z;
        double dirX = dir.X;
        double dirY = dir.Y;
        double dirZ = dir.Z;

        // D1 is constant for a line
        Vector3D d1 = new(dirX, dirY, dirZ);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double t = theParams.Value(i);
            result[i - theParams.Lower() + 1] = new Curve3D.ResD1
            {
                Point = new Point3D(locX + t * dirX, locY + t * dirY, locZ + t * dirZ),
                D1 = d1,
            };
        }
        return result;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives (D2 is zero).</summary>
    public NCollection_Array1<Curve3D.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve3D.ResD2>();
        }

        NCollection_Array1<Curve3D.ResD2> result = new(1, theParams.Length());

        Line3D lin = _geom.Line;
        Point3D loc = lin.Location;
        Direction3D dir = lin.Direction;

        double locX = loc.X;
        double locY = loc.Y;
        double locZ = loc.Z;
        double dirX = dir.X;
        double dirY = dir.Y;
        double dirZ = dir.Z;

        Vector3D d1 = new(dirX, dirY, dirZ);
        Vector3D d2 = new(0, 0, 0); // Second derivative is zero for a line

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double t = theParams.Value(i);
            result[i - theParams.Lower() + 1] = new Curve3D.ResD2
            {
                Point = new Point3D(locX + t * dirX, locY + t * dirY, locZ + t * dirZ),
                D1 = d1,
                D2 = d2,
            };
        }
        return result;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third (D2 and D3 are zero).</summary>
    public NCollection_Array1<Curve3D.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve3D.ResD3>();
        }

        NCollection_Array1<Curve3D.ResD3> result = new(1, theParams.Length());

        Line3D lin = _geom.Line;
        Point3D loc = lin.Location;
        Direction3D dir = lin.Direction;

        double locX = loc.X;
        double locY = loc.Y;
        double locZ = loc.Z;
        double dirX = dir.X;
        double dirY = dir.Y;
        double dirZ = dir.Z;

        Vector3D d1 = new(dirX, dirY, dirZ);
        Vector3D zero = new(0, 0, 0);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double t = theParams.Value(i);
            result[i - theParams.Lower() + 1] = new Curve3D.ResD3
            {
                Point = new Point3D(locX + t * dirX, locY + t * dirY, locZ + t * dirZ),
                D1 = d1,
                D2 = zero,
                D3 = zero,
            };
        }
        return result;
    }

    /// <summary>Evaluates the Nth derivative at all grid points: the direction for N = 1, zero above.</summary>
    public NCollection_Array1<Vector3D> EvaluateGridDN(NCollection_Array1<double> theParams, int n)
    {
        if (_geom is null || theParams.IsEmpty() || n < 1)
        {
            return new NCollection_Array1<Vector3D>();
        }

        NCollection_Array1<Vector3D> result = new(1, theParams.Length());

        if (n == 1)
        {
            // D1 is constant for a line (the direction)
            Direction3D dir = _geom.Line.Direction;
            Vector3D d1 = new(dir.X, dir.Y, dir.Z);
            for (int i = 1; i <= theParams.Length(); ++i)
            {
                result.SetValue(i, d1);
            }
        }
        else
        {
            // All higher derivatives are zero for a line
            Vector3D zero = new(0, 0, 0);
            for (int i = 1; i <= theParams.Length(); ++i)
            {
                result.SetValue(i, zero);
            }
        }
        return result;
    }
}
