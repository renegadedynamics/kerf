// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElCS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElCS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Thu Jul  7 14:37:05 2005 OCC9134

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.ResultType;

namespace Kerf.Engine.Extrema;

/// <summary>
/// All the distances between an elementary curve and an elementary surface. These distances can be maximum or
/// minimum. OCCT implements line/plane, line/cylinder, line/sphere, circle/plane, circle/cylinder, circle/sphere and
/// hyperbola/plane; the other pairs throw <see cref="MethodNotImplementedException"/>.
/// </summary>
/// <remarks>
/// An algorithm object: construct it with the curve and the surface, or empty and call <c>Perform</c>. When the
/// result is parallel (<see cref="IsParallel"/>), only <see cref="SquareDistance"/> is meaningful: as in OCCT,
/// <see cref="Points"/> then reads arrays that this computation didn't fill (null, or those of an earlier
/// <c>Perform</c>).
/// </remarks>
public class ExtElCS
{
    private bool _done;
    private int _nbExt;
    private bool _isPar;
    private NCollection_HArray1<double>? _sqDist;
    private NCollection_HArray1<POnCurv>? _point1;
    private NCollection_HArray1<POnSurf>? _point2;

    public ExtElCS()
    {
        _done = false;
        _isPar = false;
        _nbExt = 0;
    }

    /// <summary>The distances between a line and a plane. The line can be on the plane or on a parallel plane.</summary>
    public ExtElCS(in Line3D c, in Plane3D s)
    {
        Perform(c, s);
    }

    public void Perform(in Line3D c, in Plane3D s)
    {
        _done = true;
        _isPar = false;
        _nbExt = 0;

        if (c.Direction.IsNormal(s.Axis.Direction, global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            _sqDist = new NCollection_HArray1<double>(1, 1);
            _sqDist.SetValue(1, s.SquareDistanceTo(c));
            _isPar = true;
            _nbExt = 1;
        }
    }

    /// <summary>The distances between a line and a cylinder.</summary>
    public ExtElCS(in Line3D c, in Cylinder s)
    {
        Perform(c, s);
    }

    public void Perform(in Line3D c, in Cylinder s)
    {
        _done = false;
        _nbExt = 0;
        _isPar = false;

        CoordinateSystem pos = s.Position;

        bool isParallel = false;

        double radius = s.Radius;
        ExtElC extrem = new(new Line3D(pos.Axis), c, global::Kerf.Engine.Foundation.Precision.Angular());
        if (extrem.IsParallel)
        {
            isParallel = true;
        }
        else
        {
            int i;

            POnCurv myPOnC1, myPOnC2;
            extrem.Points(1, out myPOnC1, out myPOnC2);
            Point3D ponAxis = myPOnC1.Value();
            Point3D pc = myPOnC2.Value();

            // line intersects the cylinder
            if (radius - ponAxis.DistanceTo(pc) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                Quadric theQuadric = new(s);
                IntConicQuad inters = new(c, theQuadric);
                if (inters.IsDone && inters.IsInQuadric)
                {
                    isParallel = true;
                }
                else if (inters.IsDone)
                {
                    _nbExt = inters.PointCount;
                    if (_nbExt > 0)
                    {
                        // Not more than 2 additional points from perpendiculars.
                        _sqDist = new NCollection_HArray1<double>(1, _nbExt + 2);
                        _point1 = new NCollection_HArray1<POnCurv>(1, _nbExt + 2);
                        _point2 = new NCollection_HArray1<POnSurf>(1, _nbExt + 2);
                        double u, v, w;
                        for (i = 1; i <= _nbExt; i++)
                        {
                            _sqDist.SetValue(i, 0.0);
                            Point3D P_int = inters.Point(i);
                            w = inters.ParamOnConic(i);
                            POnCurv ponC = new(w, P_int);
                            _point1.SetValue(i, ponC);
                            global::Kerf.Engine.Geometry.ElSLib.CylinderParameters(pos, radius, P_int, out u, out v);
                            POnSurf ponS = new(u, v, P_int);
                            _point2.SetValue(i, ponS);
                        }
                    }
                }
            }
            else
            {
                // line is tangent or outside of the cylinder
                ExtPElS exPS = new(pc, s, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (exPS.IsDone)
                {
                    _nbExt = exPS.ExtCount;
                    _sqDist = new NCollection_HArray1<double>(1, _nbExt);
                    _point1 = new NCollection_HArray1<POnCurv>(1, _nbExt);
                    _point2 = new NCollection_HArray1<POnSurf>(1, _nbExt);

                    for (i = 1; i <= _nbExt; i++)
                    {
                        _point1.SetValue(i, myPOnC2);
                        _point2.SetValue(i, exPS.Point(i));
                        _sqDist.SetValue(i, (myPOnC2.Value()).SquareDistanceTo(exPS.Point(i).Value()));
                    }
                }
            }

            _done = true;
        }

        if (isParallel)
        {
            // Line direction is similar to cylinder axis of rotation.
            // The case is possible when either extrema returned parallel status
            // or Intersection tool returned infinite number of solutions.
            // This is possible due to Intersection algorithm uses more precise
            // characteristics to consider given geometries parallel.
            // In the latter case there may be several extremas, thus we look for
            // the one with the lowest distance and use it as a final solution.
            _sqDist = new NCollection_HArray1<double>(1, 1);
            double dist = extrem.SquareDistance(1);
            int nbExt = extrem.ExtCount;
            for (int i = 2; i <= nbExt; i++)
            {
                double d = extrem.SquareDistance(i);
                if (d < dist)
                {
                    dist = d;
                }
            }

            dist = Math.Sqrt(dist) - radius;
            _sqDist.SetValue(1, dist * dist);
            _done = true;
            _isPar = true;
            _nbExt = 1;
        }
    }

    /// <summary>The distances between a line and a cone (not implemented in OCCT).</summary>
    public ExtElCS(in Line3D c, in Cone s)
    {
        Perform(c, s);
    }

    // void Extrema_ExtElCS::Perform(const gp_Lin& C,
    //			      const gp_Cone& S)
    /// <exception cref="MethodNotImplementedException">Always.</exception>
    public void Perform(in Line3D c, in Cone s)
    {
        throw new MethodNotImplementedException();
    }

    /// <summary>The distances between a line and a sphere.</summary>
    public ExtElCS(in Line3D c, in Sphere s)
    {
        Perform(c, s);
    }

    public void Perform(in Line3D c, in Sphere s)
    {
        // In case of intersection - return four points:
        // 2 intersection points and 2 perpendicular.
        // No intersection - only min and max.

        _done = false;
        _nbExt = 0;
        _isPar = false;
        int startIdx = 0;

        Point3D center = s.Location;

        ExtPElC extrem = new(center, c, global::Kerf.Engine.Foundation.Precision.Angular(), RealFirst(), RealLast());

        int i;
        if (extrem.IsDone && extrem.ExtCount > 0)
        {
            POnCurv myPOnC1 = extrem.Point(1);
            if (myPOnC1.Value().DistanceTo(center) <= s.Radius)
            {
                IntConicQuad linSphere = new(c, new Quadric(s));
                if (linSphere.IsDone)
                {
                    _nbExt = linSphere.PointCount;
                    startIdx = _nbExt;
                    // Not more than 2 additional points from perpendiculars.
                    _sqDist = new NCollection_HArray1<double>(1, _nbExt + 2);
                    _point1 = new NCollection_HArray1<POnCurv>(1, _nbExt + 2);
                    _point2 = new NCollection_HArray1<POnSurf>(1, _nbExt + 2);

                    for (i = 1; i <= _nbExt; i++)
                    {
                        POnCurv cPnt = new(linSphere.ParamOnConic(i), linSphere.Point(i));

                        double u, v;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(s, linSphere.Point(i), out u, out v);
                        POnSurf sPnt = new(u, v, linSphere.Point(i));

                        _point1.SetValue(i, cPnt);
                        _point2.SetValue(i, sPnt);
                        _sqDist.SetValue(i, (cPnt.Value()).SquareDistanceTo(sPnt.Value()));
                    }
                }
            }

            ExtPElS exPS = new(myPOnC1.Value(), s, global::Kerf.Engine.Foundation.Precision.Confusion());
            if (exPS.IsDone)
            {
                if (startIdx == 0)
                {
                    _nbExt = exPS.ExtCount;
                    _sqDist = new NCollection_HArray1<double>(1, _nbExt);
                    _point1 = new NCollection_HArray1<POnCurv>(1, _nbExt);
                    _point2 = new NCollection_HArray1<POnSurf>(1, _nbExt);
                }
                else
                {
                    _nbExt += exPS.ExtCount;
                }

                for (i = startIdx + 1; i <= _nbExt; i++)
                {
                    _point1!.SetValue(i, myPOnC1);
                    _point2!.SetValue(i, exPS.Point(i - startIdx));
                    _sqDist!.SetValue(i, (myPOnC1.Value()).SquareDistanceTo(exPS.Point(i - startIdx).Value()));
                }
            }
        }
        _done = true;
    }

    /// <summary>The distances between a line and a torus (not implemented in OCCT).</summary>
    public ExtElCS(in Line3D c, in Torus s)
    {
        Perform(c, s);
    }

    // void Extrema_ExtElCS::Perform(const gp_Lin& C,
    //			      const gp_Torus& S)
    /// <exception cref="MethodNotImplementedException">Always.</exception>
    public void Perform(in Line3D c, in Torus s)
    {
        throw new MethodNotImplementedException();
    }

    //        Circle-?

    /// <summary>The distances between a circle and a plane.</summary>
    public ExtElCS(in Circle3D c, in Plane3D s)
    {
        Perform(c, s);
    }

    public void Perform(in Circle3D c, in Plane3D s)
    {
        _done = true;
        _isPar = false;
        _nbExt = 0;

        Frame3D pos = c.Position;
        Direction3D nCirc = pos.Direction;
        Direction3D nPln = s.Axis.Direction;

        bool isParallel = false;

        if (nCirc.IsParallel(nPln, global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            isParallel = true;
        }
        else
        {
            Direction3D extLine = nCirc ^ nPln;
            extLine = extLine ^ nCirc;
            //
            Direction3D xDir = pos.XDirection;
            Span<double> T = stackalloc double[2];
            T[0] = xDir.AngleWithRef(extLine, nCirc);
            if (T[0] < 0.0)
            {
                // Put in period
                T[0] += M_PI;
            }
            T[1] = T[0] + M_PI;
            //
            _nbExt = 2;
            // Check intersection
            IntConicQuad inter = new(c, s, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());

            if (inter.IsDone && inter.IsInQuadric)
            {
                isParallel = true;
            }
            else if (inter.IsDone)
            {
                if (inter.PointCount > 1)
                {
                    _nbExt += inter.PointCount;
                }
            }

            if (!isParallel)
            {
                _point1 = new NCollection_HArray1<POnCurv>(1, _nbExt);
                _sqDist = new NCollection_HArray1<double>(1, _nbExt);
                _point2 = new NCollection_HArray1<POnSurf>(1, _nbExt);

                int i;
                Point3D pc, pp;
                double u, v;
                POnCurv pOnC = new();
                POnSurf pOnS = new();
                for (i = 0; i < 2; ++i)
                {
                    pc = global::Kerf.Engine.Geometry.ElCLib.CircleValue(T[i], c.Position, c.Radius);
                    pOnC.SetValues(T[i], pc);
                    _point1.SetValue(i + 1, pOnC);
                    global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(s.Position, pc, out u, out v);
                    pp = global::Kerf.Engine.Geometry.ElSLib.PlaneValue(u, v, s.Position);
                    pOnS.SetParameters(u, v, pp);
                    _point2.SetValue(i + 1, pOnS);
                    _sqDist.SetValue(i + 1, pc.SquareDistanceTo(pp));
                }
                //
                if (_nbExt > 2)
                {
                    // Add intersection points
                    for (i = 1; i <= inter.PointCount; ++i)
                    {
                        double t = inter.ParamOnConic(i);
                        pc = global::Kerf.Engine.Geometry.ElCLib.CircleValue(t, c.Position, c.Radius);
                        pOnC.SetValues(t, pc);
                        _point1.SetValue(i + 2, pOnC);
                        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(s.Position, pc, out u, out v);
                        pp = global::Kerf.Engine.Geometry.ElSLib.PlaneValue(u, v, s.Position);
                        pOnS.SetParameters(u, v, pp);
                        _point2.SetValue(i + 2, pOnS);
                        _sqDist.SetValue(i + 2, pc.SquareDistanceTo(pp));
                    }
                }
            }
        }

        if (isParallel)
        {
            _sqDist = new NCollection_HArray1<double>(1, 1);
            _sqDist.SetValue(1, s.SquareDistanceTo(c.Location));
            _isPar = true;
            _nbExt = 1;
        }
    }

    /// <summary>The distances between a circle and a cylinder.</summary>
    public ExtElCS(in Circle3D c, in Cylinder s)
    {
        Perform(c, s);
    }

    //  Modified by skv - Thu Jul  7 14:37:05 2005 OCC9134 Begin
    // Implementation of the method.
    public void Perform(in Circle3D c, in Cylinder s)
    {
        _done = false;
        _isPar = false;
        _nbExt = 0;

        // Get an axis line of the cylinder.
        Line3D axis = new(s.Axis);

        // Compute extrema between the circle and the line.
        ExtElC extC = new(axis, c, 0.0);

        if (!extC.IsDone)
        {
            return;
        }

        bool isParallel = false;

        if (extC.IsParallel)
        {
            isParallel = true;
        }
        else
        {
            int nbExt = extC.ExtCount;
            int i;
            int curI = 1;
            double tolConf = global::Kerf.Engine.Foundation.Precision.Confusion();
            double cylRad = s.Radius;

            // Check whether two objects have intersection points
            Quadric cylQuad = new(s);
            IntConicQuad circCylInter = new(c, cylQuad);
            int nbInter = 0;
            if (circCylInter.IsDone && circCylInter.IsInQuadric)
            {
                isParallel = true;
            }
            else if (circCylInter.IsDone)
            {
                nbInter = circCylInter.PointCount;
            }

            if (!isParallel)
            {
                // Compute the extremas.
                _nbExt = 2 * nbExt + nbInter;
                _sqDist = new NCollection_HArray1<double>(1, _nbExt);
                _point1 = new NCollection_HArray1<POnCurv>(1, _nbExt);
                _point2 = new NCollection_HArray1<POnSurf>(1, _nbExt);

                for (i = 1; i <= nbExt; i++)
                {
                    POnCurv pOnAxis;
                    POnCurv pOnCirc;
                    double sqDist = extC.SquareDistance(i);
                    double dist = Math.Sqrt(sqDist);

                    extC.Points(i, out pOnAxis, out pOnCirc);

                    if (sqDist <= (tolConf * tolConf))
                    {
                        _nbExt -= 2;
                        continue;
                    }

                    Direction3D dir = new(pOnAxis.Value().XYZ().Subtracted(pOnCirc.Value().XYZ()));
                    ReadOnlySpan<double> shift = [dist + cylRad, dist - cylRad];
                    int j;

                    for (j = 0; j < 2; j++)
                    {
                        Vector3D vec = new(dir);
                        Point3D pntOnCyl;

                        vec = vec.Multiplied(shift[j]);
                        pntOnCyl = pOnCirc.Value().Translated(vec);

                        double u;
                        double v;

                        global::Kerf.Engine.Geometry.ElSLib.Parameters(s, pntOnCyl, out u, out v);

                        POnSurf pOnSurf = new(u, v, pntOnCyl);

                        _point1.SetValue(curI, pOnCirc);
                        _point2.SetValue(curI, pOnSurf);
                        _sqDist.SetValue(curI++, shift[j] * shift[j]);
                    }
                }

                // Adding intersection points to the list of extremas
                for (i = 1; i <= nbInter; i++)
                {
                    double u;
                    double v;

                    Point3D interPnt = circCylInter.Point(i);

                    u = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, interPnt);
                    POnCurv pOnCirc = new(u, interPnt);

                    global::Kerf.Engine.Geometry.ElSLib.Parameters(s, interPnt, out u, out v);
                    POnSurf pOnCyl = new(u, v, interPnt);
                    _point1.SetValue(curI, pOnCirc);
                    _point2.SetValue(curI, pOnCyl);
                    _sqDist.SetValue(curI++, 0.0);
                }
            }
        }

        _done = true;

        if (isParallel)
        {
            // The case is possible when either extrema returned parallel status
            // or Intersection tool returned infinite number of solutions.
            // This is possible due to Intersection algorithm uses more precise
            // characteristics to consider given geometries parallel.
            // In the latter case there may be several extremas, thus we look for
            // the one with the lowest distance and use it as a final solution.

            _isPar = true;
            _nbExt = 1;
            _sqDist = new NCollection_HArray1<double>(1, 1);
            double dist = extC.SquareDistance(1);

            int nbExt = extC.ExtCount;
            for (int i = 2; i <= nbExt; i++)
            {
                double d = extC.SquareDistance(i);
                if (d < dist)
                {
                    dist = d;
                }
            }

            dist = Math.Sqrt(dist) - s.Radius;
            _sqDist.SetValue(1, dist * dist);
        }
    }

    //  Modified by skv - Thu Jul  7 14:37:05 2005 OCC9134 End

    /// <summary>The distances between a circle and a cone (not implemented in OCCT).</summary>
    public ExtElCS(in Circle3D c, in Cone s)
    {
        Perform(c, s);
    }

    // void Extrema_ExtElCS::Perform(const gp_Circ& C,
    //			 const gp_Cone& S)
    /// <exception cref="MethodNotImplementedException">Always.</exception>
    public void Perform(in Circle3D c, in Cone s)
    {
        throw new MethodNotImplementedException();
    }

    /// <summary>The distances between a circle and a sphere: the minimal ones only.</summary>
    public ExtElCS(in Circle3D c, in Sphere s)
    {
        Perform(c, s);
    }

    public void Perform(in Circle3D c, in Sphere s)
    {
        _done = false;
        _isPar = false;
        _nbExt = 0;

        if (new Line3D(c.Axis).SquareDistanceTo(s.Location) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            // Circle and sphere are parallel
            _isPar = true;
            _done = true;
            _nbExt = 1;

            // Compute distance from circle to the sphere
            double sqDistLoc = c.Location.SquareDistanceTo(s.Location);
            double sqDist = sqDistLoc + c.Radius * c.Radius;
            double dist = Math.Sqrt(sqDist) - s.Radius;
            _sqDist = new NCollection_HArray1<double>(1, 1);
            _sqDist.SetValue(1, dist * dist);
            return;
        }

        // Intersect sphere with circle's plane
        Plane3D cPln = new(c.Location, c.Axis.Direction);
        QuadQuadGeo inter = new(cPln, s);
        if (!inter.IsDone)
        {
            // not done
            return;
        }

        if (inter.TypeInter() != Circle)
        {
            // Intersection is empty or just a point.
            // The parallel case has already been considered,
            // thus, here we have to find only one minimal solution
            _nbExt = 1;
            _done = true;

            _sqDist = new NCollection_HArray1<double>(1, 1);
            _point1 = new NCollection_HArray1<POnCurv>(1, 1);
            _point2 = new NCollection_HArray1<POnSurf>(1, 1);

            // Compute parameter on circle
            double t = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, s.Location);
            // Compute point on circle
            Point3D pOnC = global::Kerf.Engine.Geometry.ElCLib.Value(t, c);

            // Compute parameters on sphere
            double u, v;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(s, pOnC, out u, out v);
            // Compute point on sphere
            Point3D pOnS = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, s);

            // Save solution
            _point1.SetValue(1, new POnCurv(t, pOnC));
            _point2.SetValue(1, new POnSurf(u, v, pOnS));
            _sqDist.SetValue(1, pOnC.SquareDistanceTo(pOnS));
            return;
        }

        // Here, the intersection is a circle

        // Intersection circle
        Circle3D cInt = inter.Circle(1);

        // Perform intersection of the input circle with the intersection circle
        ExtElC extC = new(c, cInt);
        bool isExtremaCircCircValid = extC.IsDone // Check if intersection is done
                                      && !extC.IsParallel // Parallel case has already been considered
                                      && extC.ExtCount > 0; // Check that some solutions have been found
        if (!isExtremaCircCircValid)
        {
            // not done
            return;
        }

        _done = true;

        // Few solutions
        double nbExt = extC.ExtCount;
        // Find the minimal distance
        double minSqDist = RealLast();
        for (int i = 1; i <= nbExt; ++i)
        {
            double sqDist = extC.SquareDistance(i);
            if (sqDist < minSqDist)
            {
                minSqDist = sqDist;
            }
        }

        // Collect all solutions close to the minimal one
        NCollection_List<int> sols = new();
        for (int i = 1; i <= nbExt; ++i)
        {
            double diff = extC.SquareDistance(i) - minSqDist;
            if (diff < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                sols.Append(i);
            }
        }

        // Save all minimal solutions
        _nbExt = sols.Extent();

        _sqDist = new NCollection_HArray1<double>(1, _nbExt);
        _point1 = new NCollection_HArray1<POnCurv>(1, _nbExt);
        _point2 = new NCollection_HArray1<POnSurf>(1, _nbExt);

        NCollection_List<int>.Iterator it = new(sols);
        for (int iSol = 1; it.More(); it.Next(), ++iSol)
        {
            POnCurv p1, p2;
            extC.Points(it.Value(), out p1, out p2);

            // Compute parameters on sphere
            double u, v;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(s, p1.Value(), out u, out v);
            // Compute point on sphere
            Point3D pOnS = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, s);

            // Save solution
            _point1.SetValue(iSol, p1);
            _point2.SetValue(iSol, new POnSurf(u, v, pOnS));
            _sqDist.SetValue(iSol, p1.Value().SquareDistanceTo(pOnS));
        }
    }

    /// <summary>The distances between a circle and a torus (not implemented in OCCT).</summary>
    public ExtElCS(in Circle3D c, in Torus s)
    {
        Perform(c, s);
    }

    // void Extrema_ExtElCS::Perform(const gp_Circ& C,
    //			      const gp_Torus& S)
    /// <exception cref="MethodNotImplementedException">Always.</exception>
    public void Perform(in Circle3D c, in Torus s)
    {
        throw new MethodNotImplementedException();
    }

    /// <summary>The distances between a hyperbola and a plane.</summary>
    public ExtElCS(in Hyperbola3D c, in Plane3D s)
    {
        Perform(c, s);
    }

    public void Perform(in Hyperbola3D c, in Plane3D s)
    {
        _done = true;
        _isPar = false;
        _nbExt = 0;

        Frame3D pos = c.Position;
        Direction3D nHypr = pos.Direction;
        Direction3D nPln = s.Axis.Direction;

        if (nHypr.IsParallel(nPln, global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            _sqDist = new NCollection_HArray1<double>(1, 1);
            _sqDist.SetValue(1, s.SquareDistanceTo(c.Location));
            _isPar = true;
            _nbExt = 1;
        }
        else
        {
            Direction3D xDir = pos.XDirection;
            Direction3D yDir = pos.YDirection;

            double a = c.MinorRadius * (nPln.Dot(yDir));
            double b = c.MajorRadius * (nPln.Dot(xDir));

            if (Math.Abs(b) > Math.Abs(a))
            {
                double t = -0.5 * Math.Log((a + b) / (b - a));
                Point3D ph = global::Kerf.Engine.Geometry.ElCLib.HyperbolaValue(t, pos, c.MajorRadius, c.MinorRadius);
                POnCurv pc = new(t, ph);
                _point1 = new NCollection_HArray1<POnCurv>(1, 1);
                _point1.SetValue(1, pc);

                _sqDist = new NCollection_HArray1<double>(1, 1);
                _sqDist.SetValue(1, s.SquareDistanceTo(ph));

                double u, v;
                global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(s.Position, ph, out u, out v);
                Point3D pp = global::Kerf.Engine.Geometry.ElSLib.PlaneValue(u, v, s.Position);
                POnSurf ps = new(u, v, pp);
                _point2 = new NCollection_HArray1<POnSurf>(1, 1);
                _point2.SetValue(1, ps);

                _nbExt = 1;
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _done;

    /// <summary>The number of extremum distances.</summary>
    /// <exception cref="NotDoneException">The computation was not done.</exception>
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
    /// <exception cref="OutOfRangeException">N is not in [1, NbExt].</exception>
    public double SquareDistance(int n = 1)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        return _sqDist!.Value(n);
    }

    /// <summary>The points of the Nth extremum distance: P1 on the curve, P2 on the surface.</summary>
    /// <exception cref="OutOfRangeException">N is not in [1, NbExt].</exception>
    public void Points(int n, out POnCurv p1, out POnSurf p2)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        // OCCT: a parallel result fills no points, so this reads null handles or those of an earlier Perform
        p1 = _point1!.Value(n);
        p2 = _point2!.Value(n);
    }

    /// <summary>True if the curve is on a parallel surface.</summary>
    /// <exception cref="NotDoneException">The computation was not done.</exception>
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
}
