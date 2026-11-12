// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Extrema;

/// <summary>
/// All the extremum distances (minimum or maximum) between two elementary curves: a line and an elementary curve,
/// or two circles.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (OCCT gives it no Perform).</remarks>
internal class ExtElC
{
    [System.Runtime.CompilerServices.InlineArray(MaxExtrema)]
    private struct SquareDistances
    {
        private double _element;
    }

    [System.Runtime.CompilerServices.InlineArray(2 * MaxExtrema)]
    private struct PointPairs
    {
        private POnCurv _element;
    }

    private bool _done;
    private bool _isPar;
    private int _nbExt;
    private const int MaxExtrema = 6;

    // Unit 10J: fixed-size fields in place of a double[] and a POnCurv[,] per object (the rectangular array was 1.7% of
    // the Booleans' time, in Array.CreateInstanceMDArray). _point[2 * i + j] is OCCT's myPoint[i][j].
    private SquareDistances _sqDist;

    // Extrema_POnCurv's default is all zeros, so the points need no filling
    private PointPairs _point;

    public ExtElC()
    {
        _done = false;
        _isPar = false;
        _nbExt = 0;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }
    }

    // Function:
    //   Find min distance between 2 straight lines.

    // Method:
    //   Let D1 and D2, be 2 directions of straight lines C1 and C2.
    //   2 cases are considered:
    //   1- if Angle(D1,D2) < AngTol, straight lines are parallel.
    //      The distance is the distance between a point of C1 and the straight line C2.
    //   2- if Angle(D1,D2) > AngTol:
    //      Let P1=C1(u1) and P2=C2(u2).
    //      Then we must find u1 and u2 such as the distance P1P2 is minimal.
    //      Target function is:
    //        F(u1, u2) = ((L1x+D1x*u1)-(L2x+D2x*u2))^2 +
    //                    ((L1y+D1y*u1)-(L2y+D2y*u2))^2 +
    //                    ((L1z+D1z*u1)-(L2z+D2z*u2))^2 --> min,
    //      where L1 and L2 are lines locations, D1 and D2 are lines directions.
    //    Let simplify the function F

    //      F(u1, u2) = (D1x*u1-D2x*u2-Lx)^2 + (D1y*u1-D2y*u2-Ly)^2 + (D1z*u1-D2z*u2-Lz)^2,
    //    where L is a vector L1L2.

    //    In point of minimum, the condition
    //      {dF/du1 = 0
    //      {dF/du2 = 0

    //    must be satisfied.

    //      dF/du1 = 2*D1x*(D1x*u1-D2x*u2-Lx) +
    //               2*D1y*(D1y*u1-D2y*u2-Ly) +
    //               2*D1z*(D1z*u1-D2z*u2-Lz) =
    //             = 2*((D1^2)*u1-(D1.D2)*u2-L.D1) =
    //             = 2*(u1-(D1.D2)*u2-L.D1)
    //      dF/du2 = -2*D2x*(D1x*u1-D2x*u2-Lx) -
    //                2*D2y*(D1y*u1-D2y*u2-Ly) -
    //                2*D2z*(D1z*u1-D2z*u2-Lz)=
    //             = -2*((D2.D1)*u1-(D2^2)*u2-(D2.L)) =
    //             = -2*((D2.D1)*u1-u2-(D2.L))

    //   Consequently, we have two-equation system with two variables:

    //     {u1 - (D1.D2)*u2 = L.D1
    //     {(D1.D2)*u1 - u2 = L.D2

    //   This system has one solution if (D1.D2)^2 != 1
    //   (if straight lines are not parallel).

    /// <summary>
    /// The distance between two lines. AngTol is not used: the lines are parallel when their angle is below
    /// <see cref="Precision.Angular"/>.
    /// </summary>
    public ExtElC(in Line3D c1, in Line3D c2, double angTol)
    {
        _ = angTol; // OCCT: the tolerance is not used
        _done = false;
        _nbExt = 0;
        _isPar = false;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }

        _isPar = LinesAreParallel(c1, c2, out double u1, out double u2);

        if (_isPar)
        {
            _sqDist[0] = c2.SquareDistanceTo(c1.Location);
            _nbExt = 1;
            _done = true;
            return;
        }

        // Here myIsPar == false;

        Point3D p1 = global::Kerf.Engine.Geometry.ElCLib.Value(u1, c1), p2 = global::Kerf.Engine.Geometry.ElCLib.Value(u2, c2);
        _sqDist[_nbExt] = p1.SquareDistanceTo(p2);
        _point[2 * _nbExt] = new POnCurv(u1, p1);
        _point[2 * _nbExt + 1] = new POnCurv(u2, p2);
        _nbExt = 1;
        _done = true;
    }

    /// <summary>
    /// The line-line case without the object (unit 15AL): true if the lines are parallel, else the parameters on each of
    /// their one extremum, whose points are <c>ElCLib.Value(u1, c1)</c> and <c>ElCLib.Value(u2, c2)</c>. The constructor
    /// computes the same from it.
    /// </summary>
    internal static bool LinesAreParallel(in Line3D c1, in Line3D c2, out double u1, out double u2)
    {
        Direction3D d1 = c1.Position.Direction, d2 = c2.Position.Direction;
        double cosA = d1.Dot(d2);
        double sqSinA = 1.0 - cosA * cosA;
        u1 = 0.0;
        u2 = 0.0;
        if (sqSinA < Constants.Resolution() || d1.IsParallel(d2, global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            return true;
        }

        Xyz l1L2 = c2.Location.XYZ() - c1.Location.XYZ();
        double d1L = d1.XYZ().Dot(l1L2), d2L = d2.XYZ().Dot(l1L2);
        u1 = (d1L - cosA * d2L) / sqSinA;
        u2 = (cosA * d1L - d2L) / sqSinA;

        return global::Kerf.Engine.Foundation.Precision.IsInfinite(u1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(u2);
    }

    /// <summary>Computes the extrema when the line and the circle are in one plane.</summary>
    protected bool PlanarLineCircleExtrema(in Line3D lin, in Circle3D circ)
    {
        Direction3D dirC = circ.Axis.Direction, dirL = lin.Direction;

        if (Math.Abs(dirC.Dot(dirL)) > global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return false;
        }

        // The line is in the circle-plane completely
        //(or parallel to the circle-plane).
        // Therefore, we are looking for extremas and
        // intersections in 2D-space.

        Xyz cLoc = circ.Location.XYZ();
        Xyz dCx = circ.Position.XDirection.XYZ(),
               dCy = circ.Position.YDirection.XYZ();

        Xyz lLoc = lin.Location.XYZ();
        Xyz lDir = lin.Direction.XYZ();

        Xyz vecCL = lLoc - cLoc;

        // Center of 2D-circle
        Point2D pc = new(0.0, 0.0);

        Frame2D circAxis = new(pc, new Direction2D(Direction2D.D.X), new Direction2D(Direction2D.D.Y));
        Circle2D circ2d = new(circAxis, circ.Radius);

        Point2D pl = new(vecCL.Dot(dCx), vecCL.Dot(dCy));
        Direction2D dl = new(lDir.Dot(dCx), lDir.Dot(dCy));
        Line2D lin2d = new(pl, dl);

        // Extremas
        ExtElC2d ext2d = new(lin2d, circ2d, global::Kerf.Engine.Foundation.Precision.Confusion());
        // Intersections
        AnalyticIntersection inters = new(lin2d, circ2d);

        _done = ext2d.IsDone || inters.IsDone;

        if (!_done)
        {
            return true;
        }

        int nbExtr = ext2d.ExtCount;
        int nbSol = inters.PointCount;

        int nbSum = nbExtr + nbSol;
        for (int extrID = 1; extrID <= nbSum; extrID++)
        {
            int delta = extrID - nbExtr;

            double linPar = 0.0, circPar = 0.0;

            if (delta < 1)
            {
                POnCurv2d pLin2d, pCirc2d;
                ext2d.Points(extrID, out pLin2d, out pCirc2d);
                linPar = pLin2d.Parameter();
                circPar = pCirc2d.Parameter();
            }
            else
            {
                linPar = inters.Point(delta).ParamOnFirst();
                circPar = inters.Point(delta).ParamOnSecond();
            }

            Point3D pOnL = global::Kerf.Engine.Geometry.ElCLib.LineValue(linPar, lin.Position),
                   pOnC = global::Kerf.Engine.Geometry.ElCLib.CircleValue(circPar, circ.Position, circ.Radius);

            _sqDist[_nbExt] = pOnL.SquareDistanceTo(pOnC);
            _point[2 * _nbExt].SetValues(linPar, pOnL);
            _point[2 * _nbExt + 1].SetValues(circPar, pOnC);
            _nbExt++;
        }

        return true;
    }

    //=================================================================================================
    // Find extreme distances between straight line C1 and circle C2.
    //
    // Method:
    //   Let P1=C1(u1) and P2=C2(u2) be two solution points
    //        D the direction of straight line C1
    //        T tangent at point P2;
    //  Then, ( P1P2.D = 0. (1)
    //         ( P1P2.T = 0. (2)
    //  Let O1 and O2 be the origins of C1 and C2;
    //  Then, (1) <=> (O1P2-u1*D).D = 0.         as O1P1 = u1*D
    //             <=> u1 = O1P2.D                as D.D = 1.
    //         (2) <=> P1O2.T = 0.                as O2P2.T = 0.
    //             <=> ((P2O1.D)D+O1O2).T = 0.    as P1O1 = -u1*D = (P2O1.D)D
    //             <=> (((P2O2+O2O1).D)D+O1O2).T = 0.
    //             <=> ((P2O2.D)(D.T)+((O2O1.D)D-O2O1).T = 0.
    //  We are in the reference frame of the circle; let:
    //         Cos = std::cos(u2) and Sin = std::sin(u2),
    //         P2 (R*Cos,R*Sin,0.),
    //         T (-R*Sin,R*Cos,0.),
    //         D (Dx,Dy,Dz),
    //         V (Vx,Vy,Vz) = (O2O1.D)D-O2O1;
    //  Then, the equation by Cos and Sin is as follows:
    //    -(2*R*R*Dx*Dy)   * Cos**2  +       A1
    //   R*R*(Dx**2-Dy**2) * Cos*Sin +    2* A2
    //         R*Vy        * Cos     +       A3
    //        -R*Vx        * Sin     +       A4
    //      R*R*Dx*Dy                = 0.    A5
    // Use the algorithm math_TrigonometricFunctionRoots to solve this equation.
    //=================================================================================================

    /// <summary>The distances between a line and a circle. Tol is not used.</summary>
    public ExtElC(in Line3D c1, in Circle3D c2, double tol)
    {
        _ = tol; // OCCT: the tolerance is not used
        double dx, dy, dz, ro2O1, tolRO2O1;
        double r, a1, a2, a3, a4, a5, aTol;
        Direction3D x2, y2, z2, d, d1;
        //
        _isPar = false;
        _done = false;
        _nbExt = 0;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }

        if (PlanarLineCircleExtrema(c1, c2))
        {
            return;
        }

        // Calculate T1 in the reference of the circle ...
        d = c1.Direction;
        d1 = d;
        x2 = c2.XAxis.Direction;
        y2 = c2.YAxis.Direction;
        z2 = c2.Axis.Direction;
        dx = d.Dot(x2);
        dy = d.Dot(y2);
        dz = d.Dot(z2);
        //
        d = d.WithCoord(dx, dy, dz);
        RefineDir(ref d);
        d.Coord(out dx, out dy, out dz);
        //
        // Calculate V in the reference frame of the circle:
        Point3D o1 = c1.Location;
        Point3D o2 = c2.Location;
        Vector3D o2O1 = new(o2, o1);
        //
        tolRO2O1 = Constants.Resolution();
        ro2O1 = o2O1.Magnitude;
        if (ro2O1 > tolRO2O1)
        {
            Direction3D do2O1 = new();
            //
            o2O1 = o2O1.Multiplied(1.0 / ro2O1);
            do2O1 = do2O1.WithCoord(o2O1.Dot(new Vector3D(x2)), o2O1.Dot(new Vector3D(y2)), o2O1.Dot(new Vector3D(z2)));
            RefineDir(ref do2O1);
            o2O1 = o2O1.WithXYZ(ro2O1 * do2O1.XYZ());
        }
        else
        {
            o2O1 = o2O1.WithCoord(o2O1.Dot(new Vector3D(x2)), o2O1.Dot(new Vector3D(y2)), o2O1.Dot(new Vector3D(z2)));
        }
        //
        Xyz vxyz = (d.XYZ() * (o2O1.Dot(new Vector3D(d)))) - o2O1.XYZ();
        //
        aTol = 1.0e-12;
        // Calculate the coefficients of the equation by Cos and Sin ...
        // [divided by R]
        r = c2.Radius;
        a5 = r * dx * dy;
        a1 = -2.0 * a5;
        a2 = 0.5 * r * (dx * dx - dy * dy); // /2.;
        a3 = vxyz.Y();
        a4 = -vxyz.X();
        //
        if (a1 >= -aTol && a1 <= aTol)
        {
            a1 = 0.0;
        }
        if (a2 >= -aTol && a2 <= aTol)
        {
            a2 = 0.0;
        }
        if (a3 >= -aTol && a3 <= aTol)
        {
            a3 = 0.0;
        }
        if (a4 >= -aTol && a4 <= aTol)
        {
            a4 = 0.0;
        }
        if (a5 >= -aTol && a5 <= aTol)
        {
            a5 = 0.0;
        }
        //
        ExtremaExtElC_TrigonometricRoots sol = new(a1, a2, a3, a4, a5, 0.0, M_PI + M_PI);
        if (!sol.IsDone)
        {
            return;
        }
        if (sol.InfiniteRoots())
        {
            _isPar = true;
            _sqDist[0] = r * r;
            _nbExt = 1;
            _done = true;
            return;
        }
        // Storage of solutions ...
        int noSol, nbSol;
        double u1, u2;
        Point3D p1, p2;
        //
        nbSol = sol.SolutionCount;
        for (noSol = 1; noSol <= nbSol; ++noSol)
        {
            u2 = sol.Value(noSol);
            p2 = global::Kerf.Engine.Geometry.ElCLib.Value(u2, c2);
            u1 = (new Vector3D(o1, p2)).Dot(new Vector3D(d1));
            p1 = global::Kerf.Engine.Geometry.ElCLib.Value(u1, c1);
            _sqDist[_nbExt] = p1.SquareDistanceTo(p2);
            _point[2 * _nbExt].SetValues(u1, p1);
            _point[2 * _nbExt + 1].SetValues(u2, p2);
            _nbExt++;
        }
        _done = true;
    }

    /// <summary>The distances between a line and an ellipse.</summary>
    public ExtElC(in Line3D c1, in Ellipse3D c2)
    {
        /*-----------------------------------------------------------------------------
        Function:
          Find extreme distances between straight line C1 and ellipse C2.

        Method:
          Let P1=C1(u1) and P2=C2(u2) two solution points
                D the direction of straight line C1
            T the tangent to point P2;
          Then, ( P1P2.D = 0. (1)
                 ( P1P2.T = 0. (2)
          Let O1 and O2 be the origins of C1 and C2;
          Then, (1) <=> (O1P2-u1*D).D = 0.        as O1P1 = u1*D
                 <=> u1 = O1P2.D              as D.D = 1.
                 (2) <=> P1O2.T = 0.              as O2P2.T = 0.
                     <=> ((P2O1.D)D+O1O2).T = 0.  as P1O1 = -u1*D = (P2O1.D)D
                 <=> (((P2O2+O2O1).D)D+O1O2).T = 0.
                 <=> ((P2O2.D)(D.T)+((O2O1.D)D-O2O1).T = 0.
          We are in the reference of the ellipse; let:
                 Cos = std::cos(u2) and Sin = std::sin(u2),
                 P2 (MajR*Cos,MinR*Sin,0.),
                 T (-MajR*Sin,MinR*Cos,0.),
             D (Dx,Dy,Dz),
             V (Vx,Vy,Vz) = (O2O1.D)D-O2O1;
          Then,  get the following equation by Cos and Sin:
            -(2*MajR*MinR*Dx*Dy)             * Cos**2  +
           (MajR*MajR*Dx**2-MinR*MinR*Dy**2) * Cos*Sin +
                 MinR*Vy                     * Cos     +
               - MajR*Vx                     * Sin     +
              MinR*MajR*Dx*Dy                = 0.
          Use algorithm math_TrigonometricFunctionRoots to solve this equation.
        -----------------------------------------------------------------------------*/
        _isPar = false;
        _done = false;
        _nbExt = 0;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }

        // Calculate T1 the reference of the ellipse ...
        Direction3D d = c1.Direction;
        Direction3D d1 = d;
        Direction3D x2, y2, z2;
        x2 = c2.XAxis.Direction;
        y2 = c2.YAxis.Direction;
        z2 = c2.Axis.Direction;
        double dx = d.Dot(x2);
        double dy = d.Dot(y2);
        double dz = d.Dot(z2);
        d = d.WithCoord(dx, dy, dz);

        // Calculate V ...
        Point3D o1 = c1.Location;
        Point3D o2 = c2.Location;
        Vector3D o2O1 = new(o2, o1);
        o2O1 = o2O1.WithCoord(o2O1.Dot(new Vector3D(x2)), o2O1.Dot(new Vector3D(y2)), o2O1.Dot(new Vector3D(z2)));
        Xyz vxyz = (d.XYZ() * (o2O1.Dot(new Vector3D(d)))) - o2O1.XYZ();

        // Calculate the coefficients of the equation by Cos and Sin ...
        double majR = c2.MajorRadius;
        double minR = c2.MinorRadius;
        double a5 = majR * minR * dx * dy;
        double a1 = -2.0 * a5;
        double R2 = majR * majR;
        double r2 = minR * minR;
        double a2 = (R2 * dx * dx - r2 * dy * dy - R2 + r2) / 2.0;
        double a3 = minR * vxyz.Y();
        double a4 = -majR * vxyz.X();
        //
        double eps = 1.0e-12;
        //
        if (Math.Abs(a5) <= eps)
        {
            a5 = 0.0;
        }
        if (Math.Abs(a1) <= eps)
        {
            a1 = 0.0;
        }
        if (Math.Abs(a2) <= eps)
        {
            a2 = 0.0;
        }
        if (Math.Abs(a3) <= eps)
        {
            a3 = 0.0;
        }
        if (Math.Abs(a4) <= eps)
        {
            a4 = 0.0;
        }
        //
        ExtremaExtElC_TrigonometricRoots sol = new(a1, a2, a3, a4, a5, 0.0, M_PI + M_PI);
        if (!sol.IsDone)
        {
            return;
        }
        //
        if (sol.InfiniteRoots())
        {
            _isPar = true;
            Point3D p = global::Kerf.Engine.Geometry.ElCLib.EllipseValue(0.0, c2.Position, c2.MajorRadius, c2.MinorRadius);
            _sqDist[0] = c1.SquareDistanceTo(p);
            _nbExt = 1;
            _done = true;
            return;
        }

        // Storage of solutions ...
        Point3D p1, p2;
        double u1, u2;
        int nbSol = sol.SolutionCount;
        for (int noSol = 1; noSol <= nbSol; noSol++)
        {
            u2 = sol.Value(noSol);
            p2 = global::Kerf.Engine.Geometry.ElCLib.Value(u2, c2);
            u1 = (new Vector3D(o1, p2)).Dot(new Vector3D(d1));
            p1 = global::Kerf.Engine.Geometry.ElCLib.Value(u1, c1);
            _sqDist[_nbExt] = p1.SquareDistanceTo(p2);
            _point[2 * _nbExt] = new POnCurv(u1, p1);
            _point[2 * _nbExt + 1] = new POnCurv(u2, p2);
            _nbExt++;
        }
        _done = true;
    }

    /// <summary>The distances between a line and a hyperbola.</summary>
    public ExtElC(in Line3D c1, in Hyperbola3D c2)
    {
        /*-----------------------------------------------------------------------------
        Function:
          Find extrema between straight line C1 and hyperbola C2.

        Method:
          Let P1=C1(u1) and P2=C2(u2) be two solution points
                D the direction of straight line C1
            T the tangent at point P2;
          Then, ( P1P2.D = 0. (1)
                ( P1P2.T = 0. (2)
          Let O1 and O2 be the origins of C1 and C2;
          Then, (1) <=> (O1P2-u1*D).D = 0.         as O1P1 = u1*D
                 <=> u1 = O1P2.D               as D.D = 1.
                 (2) <=> (P1O2 + O2P2).T= 0.
                     <=> ((P2O1.D)D+O1O2 + O2P2).T = 0.  as P1O1 = -u1*D = (P2O1.D)D
                 <=> (((P2O2+O2O1).D)D+O1O2 + O2P2).T = 0.
                 <=> (P2O2.D)(D.T)+((O2O1.D)D-O2O1).T + O2P2.T= 0.
          We are in the reference of the hyperbola; let:
                 by writing P (R* Chu, r* Shu, 0.0)
             and Chu = (v**2 + 1)/(2*v) ,
                 Shu = (V**2 - 1)/(2*v)

             T(R*Shu, r*Chu)
             D (Dx,Dy,Dz),
             V (Vx,Vy,Vz) = (O2O1.D)D-O2O1;

          Then we obtain the following equation by v:
                 (-2*R*r*Dx*Dy - R*R*Dx*Dx-r*r*Dy*Dy + R*R + r*r)     * v**4  +
             (2*R*Vx + 2*r*Vy)                                    * v**3  +
             (-2*R*Vx + 2*r*Vy)                                   * v     +
             (-2*R*r*Dx*Dy - (R*R*Dx*Dx-r*r*Dy*Dy + R*R + r*r))  = 0

          Use the algorithm math_DirectPolynomialRoots to solve this equation.
        -----------------------------------------------------------------------------*/
        _isPar = false;
        _done = false;
        _nbExt = 0;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }

        // Calculate T1 in the reference of the hyperbola...
        Direction3D d = c1.Direction;
        Direction3D d1 = d;
        Direction3D x2, y2, z2;
        x2 = c2.XAxis.Direction;
        y2 = c2.YAxis.Direction;
        z2 = c2.Axis.Direction;
        double dx = d.Dot(x2);
        double dy = d.Dot(y2);
        double dz = d.Dot(z2);
        d = d.WithCoord(dx, dy, dz);

        // Calculate V ...
        Point3D o1 = c1.Location;
        Point3D o2 = c2.Location;
        Vector3D o2O1 = new(o2, o1);
        o2O1 = o2O1.WithCoord(o2O1.Dot(new Vector3D(x2)), o2O1.Dot(new Vector3D(y2)), o2O1.Dot(new Vector3D(z2)));
        Xyz vxyz = (d.XYZ() * (o2O1.Dot(new Vector3D(d)))) - o2O1.XYZ();
        double vx = vxyz.X();
        double vy = vxyz.Y();

        // Calculate coefficients of the equation by v
        double R = c2.MajorRadius;
        double r = c2.MinorRadius;
        double a = -2 * R * r * dx * dy;
        double b = -R * R * dx * dx - r * r * dy * dy + R * R + r * r;
        double a1 = a + b;
        double a2 = 2 * R * vx + 2 * r * vy;
        double a4 = -2 * R * vx + 2 * r * vy;
        double a5 = a - b;

        DirectPolynomialRoots sol = new(a1, a2, 0.0, a4, a5);
        if (!sol.IsDone)
        {
            return;
        }

        // Store solutions ...
        Point3D p1, p2;
        double u1, u2, v;
        int nbSol = sol.SolutionCount;
        for (int noSol = 1; noSol <= nbSol; noSol++)
        {
            v = sol.Value(noSol);
            if (v > 0.0)
            {
                u2 = Math.Log(v);
                p2 = global::Kerf.Engine.Geometry.ElCLib.Value(u2, c2);
                u1 = (new Vector3D(o1, p2)).Dot(new Vector3D(d1));
                p1 = global::Kerf.Engine.Geometry.ElCLib.Value(u1, c1);
                _sqDist[_nbExt] = p1.SquareDistanceTo(p2);
                _point[2 * _nbExt] = new POnCurv(u1, p1);
                _point[2 * _nbExt + 1] = new POnCurv(u2, p2);
                _nbExt++;
            }
        }
        _done = true;
    }

    /// <summary>The distances between a line and a parabola.</summary>
    public ExtElC(in Line3D c1, in Parabola3D c2)
    {
        /*-----------------------------------------------------------------------------
        Function:
          Find extreme distances between straight line C1 and parabole C2.

        Method:
           Let P1=C1(u1) and P2=C2(u2) be two solution points
                D the direction of straight line C1
            T the tangent to point P2;
          Then, ( P1P2.D = 0. (1)
                ( P1P2.T = 0. (2)
          Let O1 and O2 be the origins of C1 and C2;
          Then, (1) <=> (O1P2-u1*D).D = 0.         as O1P1 = u1*D
                 <=> u1 = O1P2.D               as D.D = 1.
                 (2) <=> (P1O2 + O2P2).T= 0.
                     <=> ((P2O1.D)D+O1O2 + O2P2).T = 0.  as P1O1 = -u1*D = (P2O1.D)D
                 <=> (((P2O2+O2O1).D)D+O1O2 + O2P2).T = 0.
                 <=> (P2O2.D)(D.T)+((O2O1.D)D-O2O1).T + O2P2.T = 0.
          We are in the reference of the parabola; let:
                 P2 (y*y/(2*p), y, 0)
                 T (y/p, 1, 0)
             D (Dx,Dy,Dz),
             V (Vx,Vy,Vz) = (O2O1.D)D-O2O1;

          Then, get the following equation by y:
             ((1-Dx*Dx)/(2*p*p))            *  y*y*y  +        A1
             (-3*Dx*Dy/(2*p))               *  y*y    +        A2
             (1-Dy*Dy + Vx/p)               *  y      +        A3
                Vy                          = 0.               A4

          Use the algorithm math_DirectPolynomialRoots to solve this equation.
        -----------------------------------------------------------------------------*/
        _isPar = false;
        _done = false;
        _nbExt = 0;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }

        // Calculate T1 in the reference of the parabola...
        Direction3D d = c1.Direction;
        Direction3D d1 = d;
        Direction3D x2, y2, z2;
        x2 = c2.XAxis.Direction;
        y2 = c2.YAxis.Direction;
        z2 = c2.Axis.Direction;
        double dx = d.Dot(x2);
        double dy = d.Dot(y2);
        double dz = d.Dot(z2);
        d = d.WithCoord(dx, dy, dz);

        // Calculate V ...
        Point3D o1 = c1.Location;
        Point3D o2 = c2.Location;
        Vector3D o2O1 = new(o2, o1);
        o2O1 = o2O1.WithCoord(o2O1.Dot(new Vector3D(x2)), o2O1.Dot(new Vector3D(y2)), o2O1.Dot(new Vector3D(z2)));
        Xyz vxyz = (d.XYZ() * (o2O1.Dot(new Vector3D(d)))) - o2O1.XYZ();

        // Calculate coefficients of the equation by y
        double p = c2.Parameter;
        double a1 = (1 - dx * dx) / (2.0 * p * p);
        double a2 = (-3.0 * dx * dy / (2.0 * p));
        double a3 = (1 - dy * dy + vxyz.X() / p);
        double a4 = vxyz.Y();

        DirectPolynomialRoots sol = new(a1, a2, a3, a4);
        if (!sol.IsDone)
        {
            return;
        }

        // Storage of solutions ...
        Point3D p1, p2;
        double u1, u2;
        int nbSol = sol.SolutionCount;
        for (int noSol = 1; noSol <= nbSol; noSol++)
        {
            u2 = sol.Value(noSol);
            p2 = global::Kerf.Engine.Geometry.ElCLib.Value(u2, c2);
            u1 = (new Vector3D(o1, p2)).Dot(new Vector3D(d1));
            p1 = global::Kerf.Engine.Geometry.ElCLib.Value(u1, c1);
            _sqDist[_nbExt] = p1.SquareDistanceTo(p2);
            _point[2 * _nbExt] = new POnCurv(u1, p1);
            _point[2 * _nbExt + 1] = new POnCurv(u2, p2);
            _nbExt++;
        }
        _done = true;
    }

    /// <summary>
    /// The distances between two circles. Only coplanar circles give a result (IsDone is false otherwise); the
    /// circles can be parallel (concentric) or identical.
    /// </summary>
    public ExtElC(in Circle3D c1, in Circle3D c2)
    {
        bool bIsSamePlane, bIsSameAxe;
        double tolD, tolD2, tolA, d2, aDC2;
        Point3D pc1, pc2;
        Direction3D dc1, dc2;
        //
        _isPar = false;
        _done = false;
        _nbExt = 0;
        for (int idx = 0; idx < MaxExtrema; idx++)
        {
            _sqDist[idx] = RealLast();
        }
        //
        tolA = global::Kerf.Engine.Foundation.Precision.Angular();
        tolD = global::Kerf.Engine.Foundation.Precision.Confusion();
        tolD2 = tolD * tolD;
        //
        pc1 = c1.Location;
        dc1 = c1.Axis.Direction;

        pc2 = c2.Location;
        dc2 = c2.Axis.Direction;
        Plane3D plc1 = new(pc1, dc1);
        //
        d2 = plc1.SquareDistanceTo(pc2);
        bIsSamePlane = dc1.IsParallel(dc2, tolA) && d2 < tolD2;
        if (!bIsSamePlane)
        {
            return;
        }

        // Here, both circles are in the same plane.

        //
        aDC2 = pc1.SquareDistanceTo(pc2);
        bIsSameAxe = aDC2 < tolD2;
        //
        if (bIsSameAxe)
        {
            _isPar = true;
            _nbExt = 1;
            _done = true;
            double dr = c1.Radius - c2.Radius;
            _sqDist[0] = dr * dr;
            return;
        }

        bool bIn, bOut;
        int j1, j2;
        double r1, r2, d12, t11, t12, t21, t22;
        Circle3D aC1, aC2;
        Point3D p11, p12, p21, p22;
        //
        _done = true;
        //
        r1 = c1.Radius;
        r2 = c2.Radius;
        //
        j1 = 0;
        j2 = 1;
        aC1 = c1;
        aC2 = c2;
        if (r2 > r1)
        {
            j1 = 1;
            j2 = 0;
            aC1 = c2;
            aC2 = c1;
        }
        //
        r1 = aC1.Radius; // max radius
        r2 = aC2.Radius; // min radius
        //
        pc1 = aC1.Location;
        pc2 = aC2.Location;
        //
        d12 = pc1.DistanceTo(pc2);
        Vector3D vec12 = new(pc1, pc2);
        Direction3D dir12 = new(vec12);
        //
        // 1. Four common solutions
        _nbExt = 4;
        //
        p11 = new Point3D(pc1.XYZ() - r1 * dir12.XYZ());
        p12 = new Point3D(pc1.XYZ() + r1 * dir12.XYZ());
        p21 = new Point3D(pc2.XYZ() - r2 * dir12.XYZ());
        p22 = new Point3D(pc2.XYZ() + r2 * dir12.XYZ());
        //
        t11 = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC1, p11);
        t12 = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC1, p12);
        t21 = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC2, p21);
        t22 = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC2, p22);
        //
        // P11, P21
        _point[j1].SetValues(t11, p11);
        _point[j2].SetValues(t21, p21);
        _sqDist[0] = p11.SquareDistanceTo(p21);
        // P11, P22
        _point[2 * 1 + j1].SetValues(t11, p11);
        _point[2 * 1 + j2].SetValues(t22, p22);
        _sqDist[1] = p11.SquareDistanceTo(p22);
        //
        // P12, P21
        _point[2 * 2 + j1].SetValues(t12, p12);
        _point[2 * 2 + j2].SetValues(t21, p21);
        _sqDist[2] = p12.SquareDistanceTo(p21);
        //
        // P12, P22
        _point[2 * 3 + j1].SetValues(t12, p12);
        _point[2 * 3 + j2].SetValues(t22, p22);
        _sqDist[3] = p12.SquareDistanceTo(p22);
        //
        // 2. Check for intersections
        bOut = d12 > (r1 + r2 + tolD);
        bIn = d12 < (r1 - r2 - tolD);
        if (!bOut && !bIn)
        {
            bool bNbExt6;
            double alpha, beta, val, dist2;
            Span<double> t = stackalloc double[2];
            Point3D pt, pl1, pl2;
            Direction3D dLt;
            //
            alpha = 0.5 * (r1 * r1 - r2 * r2 + d12 * d12) / d12;
            val = r1 * r1 - alpha * alpha;
            if (val < 0.0)
            { // see pkv/900/L4 for details
                val = -val;
            }
            beta = Math.Sqrt(val);
            pt = new Point3D(pc1.XYZ() + alpha * dir12.XYZ());
            //
            dLt = dc1 ^ dir12;
            pl1 = new Point3D(pt.XYZ() + beta * dLt.XYZ());
            pl2 = new Point3D(pt.XYZ() - beta * dLt.XYZ());
            //
            dist2 = pl1.SquareDistanceTo(pl2);
            bNbExt6 = dist2 > tolD2;
            //
            _nbExt = 5; // just in case. see pkv/900/L4 for details
            t[j1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC1, pl1);
            t[j2] = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC2, pl1);
            _point[2 * 4 + j1].SetValues(t[j1], pl1);
            _point[2 * 4 + j2].SetValues(t[j2], pl1);
            _sqDist[4] = 0.0;
            //
            if (bNbExt6)
            {
                _nbExt = 6;
                t[j1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC1, pl2);
                t[j2] = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC2, pl2);
                _point[2 * 5 + j1].SetValues(t[j1], pl2);
                _point[2 * 5 + j2].SetValues(t[j2], pl2);
                _sqDist[5] = 0.0;
            }
            //
        } // if (!bOut || !bIn) {
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
    public void Points(int n, out POnCurv p1, out POnCurv p2)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        p1 = _point[2 * (n - 1)];
        p2 = _point[2 * (n - 1) + 1];
    }

    private static void RefineDir(ref Direction3D aDir)
    {
        int i, j, k, iK;
        double eps, x1, x2, one;
        Span<double> cx = stackalloc double[3];
        //
        iK = 3;
        eps = RealEpsilon();
        aDir.Coord(out cx[0], out cx[1], out cx[2]);
        //
        for (i = 0; i < iK; ++i)
        {
            one = (cx[i] > 0.0) ? 1.0 : -1.0;
            x1 = one - eps;
            x2 = one + eps;
            //
            if (cx[i] > x1 && cx[i] < x2)
            {
                j = (i + 1) % iK;
                k = (i + 2) % iK;
                cx[i] = one;
                cx[j] = 0.0;
                cx[k] = 0.0;
                aDir = aDir.WithCoord(cx[0], cx[1], cx[2]);
                return;
            }
        }
    }

    //=================================================================================================
    // class    : ExtremaExtElC_TrigonometricRoots
    // purpose  : Internal class (provides sorted roots of a trigonometric polynomial).
    //            Code duplicated from IntAna_IntQuadQuad.cxx.
    //            Reliable solution for near-zero coefficient problems
    //            with recovery attempt if coeff < 1.e-10.
    //=================================================================================================
    private sealed class ExtremaExtElC_TrigonometricRoots
    {
        private readonly double[] _roots = new double[4];
        private readonly bool _done;
        private readonly int _nbRoots;
        private readonly bool infinite_roots;

        public ExtremaExtElC_TrigonometricRoots(double CC, double SC, double C, double S, double Cte, double binf, double bsup)
        {
            _nbRoots = 0;
            infinite_roots = false;
            int i, nbessai;
            double cc, sc, c, s, cte;
            //
            nbessai = 1;
            cc = CC;
            sc = SC;
            c = C;
            s = S;
            cte = Cte;
            _done = false;
            while (nbessai <= 2 && !_done)
            {
                //-- F= AA*CN*CN+2*BB*CN*SN+CC*CN+DD*SN+EE;
                TrigonometricFunctionRoots mtfr = new(cc, sc, c, s, cte, binf, bsup);
                //
                if (mtfr.IsDone)
                {
                    _done = true;
                    if (mtfr.InfiniteRoots())
                    {
                        infinite_roots = true;
                    }
                    else
                    { // else #1
                        bool triee;
                        int j, svNbRoots;
                        double twoPI, maxCoef, precision;
                        //
                        twoPI = M_PI + M_PI;
                        _nbRoots = mtfr.SolutionCount;
                        for (i = 0; i < _nbRoots; ++i)
                        {
                            _roots[i] = mtfr.Value(i + 1);
                            if (_roots[i] < 0.0)
                            {
                                _roots[i] = _roots[i] + twoPI;
                            }
                            if (_roots[i] > twoPI)
                            {
                                _roots[i] = _roots[i] - twoPI;
                            }
                        }
                        //
                        //-- The direct search gives unreliable results.
                        maxCoef = Max(CC, SC);
                        maxCoef = Max(maxCoef, C);
                        maxCoef = Max(maxCoef, S);
                        maxCoef = Max(maxCoef, Cte);
                        precision = Max(1.0e-8, 1.0e-12 * maxCoef);

                        svNbRoots = _nbRoots;
                        for (i = 0; i < svNbRoots; ++i)
                        {
                            double y;
                            double co = Math.Cos(_roots[i]);
                            double si = Math.Sin(_roots[i]);
                            y = co * (CC * co + (SC + SC) * si + C) + S * si + Cte;
                            // modified by OCC  Tue Oct  3 18:43:00 2006
                            if (Math.Abs(y) > precision)
                            {
                                _nbRoots--;
                                _roots[i] = 1000.0;
                            }
                        }
                        //
                        do
                        {
                            double t;
                            //
                            triee = true;
                            for (i = 1, j = 0; i < svNbRoots; ++i, ++j)
                            {
                                if (_roots[i] < _roots[j])
                                {
                                    triee = false;
                                    t = _roots[i];
                                    _roots[i] = _roots[j];
                                    _roots[j] = t;
                                }
                            }
                        } while (!triee);
                        //
                        infinite_roots = false;
                        if (_nbRoots == 0)
                        { //--!!!!! Detect case Pol = Cte ( 1e-50 ) !!!!
                            if ((Math.Abs(CC) + Math.Abs(SC) + Math.Abs(C) + Math.Abs(S)) < 1e-10)
                            {
                                if (Math.Abs(Cte) < 1e-10)
                                {
                                    infinite_roots = true;
                                }
                            }
                        }
                    } // else #1
                } // if(MTFR.IsDone()) {
                else
                {
                    // try to set very small coefficients to ZERO
                    if (Math.Abs(CC) < 1e-10)
                    {
                        cc = 0.0;
                    }
                    if (Math.Abs(SC) < 1e-10)
                    {
                        sc = 0.0;
                    }
                    if (Math.Abs(C) < 1e-10)
                    {
                        c = 0.0;
                    }
                    if (Math.Abs(S) < 1e-10)
                    {
                        s = 0.0;
                    }
                    if (Math.Abs(Cte) < 1e-10)
                    {
                        cte = 0.0;
                    }
                    nbessai++;
                }
            } // while (nbessai<=2 && !done) {
        }

        public bool IsDone => _done;

        public bool IsARoot(double u)
        {
            double pIpPI, eps;
            //
            eps = RealEpsilon();
            pIpPI = M_PI + M_PI;
            for (int i = 0; i < _nbRoots; i++)
            {
                if (Math.Abs(u - _roots[i]) <= eps)
                {
                    return true;
                }
                if (Math.Abs(u - _roots[i] - pIpPI) <= eps)
                {
                    return true;
                }
            }
            return false;
        }

        public int SolutionCount
        {
            get
            {
                if (!_done)
                {
                    throw new NotDoneException();
                }
                return _nbRoots;
            }
        }

        public bool InfiniteRoots()
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return infinite_roots;
        }

        public double Value(int n)
        {
            if ((!_done) || (n > _nbRoots))
            {
                throw new NotDoneException();
            }
            return _roots[n - 1];
        }
    }
}
