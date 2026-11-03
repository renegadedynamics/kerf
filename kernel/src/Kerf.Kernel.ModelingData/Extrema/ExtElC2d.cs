// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElC2d.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElC2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Extrema;

/// <summary>
/// All the extremum distances (minimum or maximum) between two elementary 2D curves: a line or a circle and an
/// elementary curve.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (OCCT gives it no Perform).</remarks>
public class ExtElC2d
{
    [System.Runtime.CompilerServices.InlineArray(MaxExtrema)]
    private struct SquareDistances
    {
        private double _element;
    }

    [System.Runtime.CompilerServices.InlineArray(2 * MaxExtrema)]
    private struct PointPairs
    {
        private POnCurv2d _element;
    }

    private bool _done;
    private bool _isPar;
    private int _nbExt;
    private const int MaxExtrema = 8;

    // Unit 10J: fixed-size fields in place of a double[] and a POnCurv2d[,] per object, as in ExtElC.
    // _point[2 * i + j] is OCCT's myPoint[i][j].
    private SquareDistances _sqDist;

    // Extrema_POnCurv's default is all zeros, so the points need no filling
    private PointPairs _point;

    public ExtElC2d()
    {
        _done = false;
        _isPar = false;
        _nbExt = 0;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }
    }

    /*-----------------------------------------------------------------------------
    Function:
       Find min distance between 2 straight lines.

    Method:
      Let D1 and D2 be 2 directions of straight lines C1 and C2.
      2 cases are considered:
      1- if Angle(D1,D2) < AngTol, the straight lines are parallel.
         The distance is the distance between any point of C1 and straight line C2.
      2- if Angle(D1,D2) > AngTol:
         Let P = C1(u1) and P =C2(u2) the point intersection:

    -----------------------------------------------------------------------------*/

    /// <summary>
    /// The distance between two lines. AngTol is not used: the lines are parallel when their angle is below
    /// <see cref="Precision.Angular"/>.
    /// </summary>
    public ExtElC2d(in Line2D c1, in Line2D c2, double angTol)
    {
        _ = angTol; // OCCT: the tolerance is not used
        _done = false;
        _isPar = false;
        _nbExt = 0;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }

        Vector2D d1 = new(c1.Direction());
        Vector2D d2 = new(c2.Direction());
        if (d1.IsParallel(d2, global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            _isPar = true;
            _sqDist[0] = c2.SquareDistance(c1.Location());
            _nbExt = 1;
        }
        else
        {
            // Vector from P1 to P2 (P2 - P1).
            Vector2D p1P2 = new(c1.Location(), c2.Location());

            // Solve linear system using Cramer's rule:
            // D1.X * t1 + D2.X * (-t2)  = P2.X - P1.X
            // D1.Y * t1 + D2.Y * (-t2)  = P2.Y - P1.Y

            // There is no division by zero since lines are not parallel.
            double delim = 1 / (d1 ^ d2);

            double param1 = (p1P2 ^ d2) * delim;
            double param2 = -(d1 ^ p1P2) * delim; // -1.0 coefficient before t2.

            Point2D p1 = global::Kerf.Engine.Geometry.ElCLib.Value(param1, c1);
            Point2D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(param2, c2);

            _sqDist[_nbExt] = 0.0;
            _point[2 * _nbExt] = new POnCurv2d(param1, p1);
            _point[2 * _nbExt + 1] = new POnCurv2d(param2, p2);
            _nbExt = 1;
        }

        _done = true;
    }

    /*-----------------------------------------------------------------------------
    Function:
      Find extreme distances between straight line C1 and circle C2.

    Method:
      Let P1=C1(u1) and P2=C2(u2) be two solution points
            D the direction of straight line C1
        T the tangent at point P2;
      Then, ( P1P2.D = 0. (1)
             ( P1P2.T = 0. (2)
    -----------------------------------------------------------------------------*/

    /// <summary>The distances between a line and a circle. Tol is not used.</summary>
    public ExtElC2d(in Line2D c1, in Circle2D c2, double tol)
    {
        _ = tol; // OCCT: the tolerance is not used
        _isPar = false;
        _done = false;
        _nbExt = 0;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }

        // Calculate T1 in the reference of the circle ...
        Direction2D d = c1.Direction();
        Direction2D x2, y2;
        x2 = c2.XAxis().Direction();
        y2 = c2.YAxis().Direction();

        double dx = d.Dot(x2);
        double dy = d.Dot(y2);
        double u1;
        Span<double> teta = stackalloc double[2];
        Point2D o1 = c1.Location();
        Point2D p1, p2;

        if (Math.Abs(dy) <= RealEpsilon())
        {
            teta[0] = M_PI / 2.0;
        }
        else
        {
            teta[0] = Math.Atan(-dx / dy);
        }
        teta[1] = teta[0] + M_PI;
        if (teta[0] < 0.0)
        {
            teta[0] = teta[0] + 2.0 * M_PI;
        }

        p2 = global::Kerf.Engine.Geometry.ElCLib.Value(teta[0], c2);
        u1 = (new Vector2D(o1, p2)).Dot(new Vector2D(d));
        p1 = global::Kerf.Engine.Geometry.ElCLib.Value(u1, c1);
        _sqDist[_nbExt] = p1.SquareDistance(p2);
        _point[2 * _nbExt] = new POnCurv2d(u1, p1);
        _point[2 * _nbExt + 1] = new POnCurv2d(teta[0], p2);
        _nbExt++;

        p2 = global::Kerf.Engine.Geometry.ElCLib.Value(teta[1], c2);
        u1 = (new Vector2D(o1, p2)).Dot(new Vector2D(d));
        p1 = global::Kerf.Engine.Geometry.ElCLib.Value(u1, c1);
        _sqDist[_nbExt] = p1.SquareDistance(p2);
        _point[2 * _nbExt] = new POnCurv2d(u1, p1);
        _point[2 * _nbExt + 1] = new POnCurv2d(teta[1], p2);
        _nbExt++;
        _done = true;
    }

    /// <summary>The distances between a line and an ellipse.</summary>
    public ExtElC2d(in Line2D c1, in Ellipse2D c2)
    {
        _done = true;
        _isPar = false;
        _done = false;
        _nbExt = 0;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }

        // Calculate T1 in the reference of the ellipse ...
        Direction2D d = c1.Direction();
        Direction2D x2, y2;
        x2 = c2.XAxis().Direction();
        y2 = c2.YAxis().Direction();

        double dx = d.Dot(x2);
        double dy = d.Dot(y2);
        double u1, r1 = c2.MajorRadius(), r2 = c2.MinorRadius();
        Span<double> teta = stackalloc double[2];
        Point2D o1 = c1.Location(), p1, p2;

        if (Math.Abs(dy) <= RealEpsilon())
        {
            teta[0] = M_PI / 2.0;
        }
        else
        {
            teta[0] = Math.Atan(-dx * r2 / (dy * r1));
        }

        teta[1] = teta[0] + M_PI;
        if (teta[0] < 0.0)
        {
            teta[0] += 2.0 * M_PI;
        }
        p2 = global::Kerf.Engine.Geometry.ElCLib.Value(teta[0], c2);
        u1 = (new Vector2D(o1, p2)).Dot(new Vector2D(d));
        p1 = global::Kerf.Engine.Geometry.ElCLib.Value(u1, c1);
        _sqDist[_nbExt] = p1.SquareDistance(p2);
        _point[2 * _nbExt] = new POnCurv2d(u1, p1);
        _point[2 * _nbExt + 1] = new POnCurv2d(teta[0], p2);
        _nbExt++;

        p2 = global::Kerf.Engine.Geometry.ElCLib.Value(teta[1], c2);
        u1 = (new Vector2D(o1, p2)).Dot(new Vector2D(d));
        p1 = global::Kerf.Engine.Geometry.ElCLib.Value(u1, c1);
        _sqDist[_nbExt] = p1.SquareDistance(p2);
        _point[2 * _nbExt] = new POnCurv2d(u1, p1);
        _point[2 * _nbExt + 1] = new POnCurv2d(teta[1], p2);
        _nbExt++;
        _done = true;
    }

    /// <summary>The distance between a line and a hyperbola.</summary>
    public ExtElC2d(in Line2D c1, in Hyperbola2D c2)
    {
        _isPar = false;
        _done = false;
        _nbExt = 0;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }

        // Calculate T1 in the reference of the parabole ...
        Direction2D d = c1.Direction();
        Direction2D x2, y2;
        x2 = c2.XAxis().Direction();
        y2 = c2.YAxis().Direction();
        double dx = d.Dot(x2);
        double dy = d.Dot(y2);

        double u1, v2, u2 = 0, R = c2.MajorRadius(), r = c2.MinorRadius();
        Point2D p1, p2;
        if (Math.Abs(dy) < RealEpsilon())
        {
            return;
        }
        if (Math.Abs(R - r * dx / dy) < RealEpsilon())
        {
            return;
        }

        v2 = (R + r * dx / dy) / (R - r * dx / dy);
        if (v2 > 0.0)
        {
            u2 = Math.Log(Math.Sqrt(v2));
        }
        p2 = global::Kerf.Engine.Geometry.ElCLib.Value(u2, c2);

        u1 = (new Vector2D(c1.Location(), p2)).Dot(new Vector2D(d));
        p1 = global::Kerf.Engine.Geometry.ElCLib.Value(u1, c1);
        _sqDist[_nbExt] = p1.SquareDistance(p2);
        _point[2 * _nbExt] = new POnCurv2d(u1, p1);
        _point[2 * _nbExt + 1] = new POnCurv2d(u2, p2);
        _nbExt++;
        _done = true;
    }

    /// <summary>The distance between a line and a parabola.</summary>
    public ExtElC2d(in Line2D c1, in Parabola2D c2)
    {
        _isPar = false;
        _done = false;
        _nbExt = 0;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }

        // Calculate  T1 in the reference of the parabole ...
        Direction2D d = c1.Direction();
        Direction2D x2, y2;
        x2 = c2.MirrorAxis().Direction();
        y2 = c2.Axis().YAxis().Direction();
        double dx = d.Dot(x2);
        double dy = d.Dot(y2);

        double u1, u2, p = c2.Parameter();
        Point2D p1, p2;
        if (Math.Abs(dy) < RealEpsilon())
        {
            return;
        }
        u2 = dx * p / dy;
        p2 = global::Kerf.Engine.Geometry.ElCLib.Value(u2, c2);

        u1 = (new Vector2D(c1.Location(), p2)).Dot(new Vector2D(d));
        p1 = global::Kerf.Engine.Geometry.ElCLib.Value(u1, c1);
        _sqDist[_nbExt] = p1.SquareDistance(p2);
        _point[2 * _nbExt] = new POnCurv2d(u1, p1);
        _point[2 * _nbExt + 1] = new POnCurv2d(u2, p2);
        _nbExt++;
        _done = true;
    }

    /// <summary>The distances between two circles. The circles can be parallel or identical.</summary>
    public ExtElC2d(in Circle2D c1, in Circle2D c2)
    {
        _isPar = false;
        _done = false;
        _nbExt = 0;
        _done = true;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }

        Point2D o1 = c1.Location();
        Point2D o2 = c2.Location();

        Vector2D do1O2 = new(o1, o2);
        double sqDCenters = do1O2.SquareMagnitude();
        if (sqDCenters < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            _isPar = true;
            _nbExt = 1;
            _done = true;
            double dr = c1.Radius() - c2.Radius();
            _sqDist[0] = dr * dr;
            return;
        }

        int noSol, kk;
        double u1, u2;
        double r1 = c1.Radius(), r2 = c2.Radius();
        Span<double> usol2 = stackalloc double[2];
        Span<double> usol1 = stackalloc double[2];
        Span<Point2D> p1 = [new(), new()];
        Span<Point2D> p2 = [new(), new()];
        Vector2D o1O2 = do1O2 / Math.Sqrt(sqDCenters);

        p1[0] = o1.Translated(r1 * o1O2);
        usol1[0] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, p1[0]);
        p1[1] = o1.Translated(-r1 * o1O2);
        usol1[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, p1[1]);

        p2[0] = o2.Translated(r2 * o1O2);
        usol2[0] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c2, p2[0]);
        p2[1] = o2.Translated(-r2 * o1O2);
        usol2[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c2, p2[1]);

        for (noSol = 0; noSol <= 1; noSol++)
        {
            u1 = usol1[noSol];
            for (kk = 0; kk <= 1; kk++)
            {
                u2 = usol2[kk];
                _sqDist[_nbExt] = p2[kk].SquareDistance(p1[noSol]);
                _point[2 * _nbExt] = new POnCurv2d(u1, p1[noSol]);
                _point[2 * _nbExt + 1] = new POnCurv2d(u2, p2[kk]);
                _nbExt++;
            }
        }
    }

    /// <summary>The distances between a circle and an ellipse.</summary>
    public ExtElC2d(in Circle2D c1, in Ellipse2D c2)
    {
        _isPar = false;
        _done = false;
        _nbExt = 0;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }

        int i, j;

        ExtPElC2d extElip = new(c1.Location(), c2, global::Kerf.Engine.Foundation.Precision.Confusion(), 0.0, 2.0 * M_PI);

        if (extElip.IsDone)
        {
            for (i = 1; i <= extElip.ExtCount; i++)
            {
                ExtPElC2d extCirc = new(extElip.Point(i).Value(), c1, global::Kerf.Engine.Foundation.Precision.Confusion(), 0.0, 2.0 * M_PI);
                if (extCirc.IsDone)
                {
                    for (j = 1; j <= extCirc.ExtCount; j++)
                    {
                        _sqDist[_nbExt] = extCirc.SquareDistance(j);
                        _point[2 * _nbExt] = extCirc.Point(j);
                        _point[2 * _nbExt + 1] = extElip.Point(i);
                        _nbExt++;
                    }
                }
                _done = true;
            }
        }
    }

    /// <summary>The distances between a circle and a hyperbola.</summary>
    public ExtElC2d(in Circle2D c1, in Hyperbola2D c2)
    {
        _isPar = false;
        _done = false;
        _nbExt = 0;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }

        int i, j;

        ExtPElC2d extHyp = new(c1.Location(), c2, global::Kerf.Engine.Foundation.Precision.Confusion(), RealFirst(), RealLast());

        if (extHyp.IsDone)
        {
            for (i = 1; i <= extHyp.ExtCount; i++)
            {
                ExtPElC2d extCirc = new(extHyp.Point(i).Value(), c1, global::Kerf.Engine.Foundation.Precision.Confusion(), 0.0, 2.0 * M_PI);
                if (extCirc.IsDone)
                {
                    for (j = 1; j <= extCirc.ExtCount; j++)
                    {
                        _sqDist[_nbExt] = extCirc.SquareDistance(j);
                        _point[2 * _nbExt] = extCirc.Point(j);
                        _point[2 * _nbExt + 1] = extHyp.Point(i);
                        _nbExt++;
                    }
                }
                _done = true;
            }
        }
    }

    /// <summary>The distances between a circle and a parabola.</summary>
    public ExtElC2d(in Circle2D c1, in Parabola2D c2)
    {
        _isPar = false;
        _done = false;
        _nbExt = 0;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }

        int i, j;

        ExtPElC2d extParab = new(c1.Location(), c2, global::Kerf.Engine.Foundation.Precision.Confusion(), RealFirst(), RealLast());

        if (extParab.IsDone)
        {
            for (i = 1; i <= extParab.ExtCount; i++)
            {
                ExtPElC2d extCirc = new(extParab.Point(i).Value(), c1, global::Kerf.Engine.Foundation.Precision.Confusion(), 0.0, 2.0 * M_PI);
                if (extCirc.IsDone)
                {
                    for (j = 1; j <= extCirc.ExtCount; j++)
                    {
                        _sqDist[_nbExt] = extCirc.SquareDistance(j);
                        _point[2 * _nbExt] = extCirc.Point(j);
                        _point[2 * _nbExt + 1] = extParab.Point(i);
                        _nbExt++;
                    }
                }
                _done = true;
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _done;

    /// <summary>True if the two curves are parallel.</summary>
    public bool IsParallel
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _isPar;
        }
    }

    /// <summary>The number of extremum distances.</summary>
    public int ExtCount
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }

            return _nbExt;
        }
    }

    /// <summary>The value of the Nth extremum square distance.</summary>
    public double SquareDistance(int n = 1)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        return _sqDist[n - 1];
    }

    /// <summary>The points of the Nth extremum distance: P1 on the first curve, P2 on the second.</summary>
    public void Points(int n, out POnCurv2d p1, out POnCurv2d p2)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }
        p1 = _point[2 * (n - 1)];
        p2 = _point[2 * (n - 1) + 1];
    }
}
