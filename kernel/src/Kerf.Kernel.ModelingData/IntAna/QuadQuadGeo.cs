// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_QuadQuadGeo.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_QuadQuadGeo.lxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_QuadQuadGeo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//----------------------------------------------------------------------
//-- Purposse: Geometric Intersection between two Natural Quadric
//--          If the intersection is not a conic,
//--          analytical methods must be called.
//----------------------------------------------------------------------

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.ResultType;

namespace Kerf.Engine.Intersection;

/// <summary>
/// Geometric intersections between two natural quadrics (planes, cylinders, cones, spheres, and some tori):
/// 1 point, 1 or 2 lines, 1 point and 1 line, 1 or 2 circles, 1 or 2 ellipses, 1 parabola, 1 or 2 hyperbolas;
/// Empty (no intersection), Same (identical quadrics), or NoGeometricSolution (there may be an intersection, but an
/// analytic algorithm, <c>IntAna_IntQuadQuad</c>, must compute it).
/// </summary>
/// <remarks>An algorithm object: construct it with the quadrics, or empty and call <c>Perform</c>.</remarks>
public class QuadQuadGeo
{
    private bool _done;
    private int _nbint;
    private ResultType _typeres;
    private Point3D _pt1;
    private Point3D _pt2;
    private Point3D _pt3;
    private Point3D _pt4;
    private Direction3D _dir1 = new();
    private Direction3D _dir2 = new();
    private Direction3D _dir3 = new();
    private Direction3D _dir4 = new();
    private double _param1;
    private double _param2;
    private double _param3;
    private double _param4;
    private double _param1bis;
    private double _param2bis;
    private double _epsilonDistance;
    private double _epsilonAngleCone;
    private double _epsilonMiniCircleRadius;
    private double _epsilonCylinderDeltaRadius;
    private double _epsilonCylinderDeltaDistance;
    private double _epsilonAxesPara;
    private bool _commonGen;
    private Point3D _pChar;

    //=======================================================================
    // class :  AxeOperator
    // purpose  : V a r i o u s   O p e r a t i o n s   o n   A x 1
    //=======================================================================
    private sealed class AxeOperator
    {
        private Point3D _ptintersect;
        private readonly Axis3D _axe1;
        private readonly Axis3D _axe2;
        private readonly double _thedistance;
        private readonly bool _theparallel;
        private readonly bool _thecoplanar;
        private readonly bool _thenormal;
        //
        private readonly double _epsilonDistance;
        private readonly double _epsilonAxesPara;

        public AxeOperator(in Axis3D a1, in Axis3D a2)
            : this(a1, a2, 1.0e-14, global::Kerf.Engine.Foundation.Precision.Angular())
        {
        }

        public AxeOperator(in Axis3D a1, in Axis3D a2, double epsDistance, double epsAxesPara)
        {
            _axe1 = a1;
            _axe2 = a2;
            _epsilonDistance = epsDistance;
            _epsilonAxesPara = epsAxesPara;
            //---------------------------------------------------------------------
            Direction3D v1 = _axe1.Direction();
            Direction3D v2 = _axe2.Direction();
            Point3D p1 = _axe1.Location();
            Point3D p2 = _axe2.Location();
            //
            RefineDir(ref v1);
            RefineDir(ref v2);
            _thecoplanar = false;
            _thenormal = false;

            //--- check if the two axis are parallel
            _theparallel = v1.IsParallel(v2, _epsilonAxesPara);
            //--- Distance between the two axis
            Xyz perp = a1.Direction().XYZ().Crossed(a2.Direction().XYZ());
            if (_theparallel)
            {
                Line3D l1 = new(a1);
                _thedistance = l1.Distance(a2.Location());
            }
            else
            {
                _thedistance = Math.Abs(new Vector3D(perp.Normalized()).Dot(new Vector3D(_axe1.Location(), _axe2.Location())));
            }
            //--- check if Axis are Coplanar
            double d33;
            if (_thedistance < _epsilonDistance)
            {
                d33 = Det33(v1.X(),
                            v1.Y(),
                            v1.Z(),
                            v2.X(),
                            v2.Y(),
                            v2.Z(),
                            p1.X() - p2.X(),
                            p1.Y() - p2.Y(),
                            p1.Z() - p2.Z());
                if (Math.Abs(d33) <= _epsilonDistance)
                {
                    _thecoplanar = true;
                }
            }

            _thenormal = Math.Abs(v1.Dot(v2)) < _epsilonAxesPara;

            //--- check if the two axis are concurrent
            if (_thecoplanar && !_theparallel)
            {
                double smx = p2.X() - p1.X();
                double smy = p2.Y() - p1.Y();
                double smz = p2.Z() - p1.Z();
                double det1, det2, det3, a;
                det1 = v1.Y() * v2.X() - v1.X() * v2.Y();
                det2 = v1.Z() * v2.Y() - v1.Y() * v2.Z();
                det3 = v1.Z() * v2.X() - v1.X() * v2.Z();

                if ((det1 != 0.0) && ((Math.Abs(det1) >= Math.Abs(det2)) && (Math.Abs(det1) >= Math.Abs(det3))))
                {
                    a = (smy * v2.X() - smx * v2.Y()) / det1;
                }
                else if ((det2 != 0.0)
                         && ((Math.Abs(det2) >= Math.Abs(det1)) && (Math.Abs(det2) >= Math.Abs(det3))))
                {
                    a = (smz * v2.Y() - smy * v2.Z()) / det2;
                }
                else
                {
                    a = (smz * v2.X() - smx * v2.Z()) / det3;
                }
                _ptintersect = _ptintersect.WithCoord(p1.X() + a * v1.X(), p1.Y() + a * v1.Y(), p1.Z() + a * v1.Z());
            }
            else
            {
                _ptintersect = _ptintersect.WithCoord(0, 0, 0); //-- To avoid FPE
            }
        }

        public void Distance(out double dist, out double param1, out double param2)
        {
            // OCCT: leaves the outputs unset when the axes are parallel (D == 0); 0 here
            dist = 0.0;
            param1 = 0.0;
            param2 = 0.0;
            Vector3D o1O2 = new(_axe1.Location(), _axe2.Location());
            Direction3D u1 = _axe1.Direction(); //-- just to see.
            Direction3D u2 = _axe2.Direction();

            Direction3D n = u1.Crossed(u2);
            double d = Det33(u1.X(), u2.X(), n.X(), u1.Y(), u2.Y(), n.Y(), u1.Z(), u2.Z(), n.Z());
            if (d != 0.0)
            {
                dist = Det33(u1.X(), u2.X(), o1O2.X(), u1.Y(), u2.Y(), o1O2.Y(), u1.Z(), u2.Z(), o1O2.Z()) / d;
                param1 =
                    Det33(o1O2.X(), u2.X(), n.X(), o1O2.Y(), u2.Y(), n.Y(), o1O2.Z(), u2.Z(), n.Z()) / (-d);
                //------------------------------------------------------------
                //-- We solve P1 * Dir1 + P2 * Dir2 + d * N = O1O2
                //-- i.e.: Perpendicular segment: O1+P1 D1
                //--                               O2-P2 D2
                param2 = Det33(u1.X(), o1O2.X(), n.X(), u1.Y(), o1O2.Y(), n.Y(), u1.Z(), o1O2.Z(), n.Z()) / (d);
            }
        }

        public Point3D PtIntersect() => _ptintersect;

        public bool Coplanar() => _thecoplanar;

        public bool Same() => _theparallel && (_thedistance < _epsilonDistance);

        public double Distance() => _thedistance;

        public bool Intersect() => (_thecoplanar && (!_theparallel));

        public bool Parallel() => _theparallel;

        public bool Normal() => _thenormal;

        private static double Det33(double a11,
                                    double a12,
                                    double a13,
                                    double a21,
                                    double a22,
                                    double a23,
                                    double a31,
                                    double a32,
                                    double a33)
        {
            double theReturn =
                a11 * (a22 * a33 - a32 * a23) - a21 * (a12 * a33 - a32 * a13) + a31 * (a12 * a23 - a22 * a13);
            return theReturn;
        }
    }

    //=======================================================================
    // function : DirToAx2
    // purpose  : returns a gp_Ax2 where D is the main direction
    //=======================================================================
    private static Frame3D DirToAx2(in Point3D p, in Direction3D d)
    {
        double x = d.X();
        double ax = Math.Abs(x);
        double y = d.Y();
        double ay = Math.Abs(y);
        double z = d.Z();
        double az = Math.Abs(z);
        if ((ax == 0.0) || ((ax < ay) && (ax < az)))
        {
            return (new Frame3D(p, d, new Direction3D(new Vector3D(0.0, -z, y))));
        }
        else if ((ay == 0.0) || ((ay < ax) && (ay < az)))
        {
            return (new Frame3D(p, d, new Direction3D(new Vector3D(-z, 0.0, x))));
        }
        else
        {
            return (new Frame3D(p, d, new Direction3D(new Vector3D(-y, x, 0.0))));
        }
    }

    //=======================================================================
    // function : EstimDist
    // purpose  : returns a minimal distance from apex to any solution
    //=======================================================================
    private static double EstimDist(in Cone con1, in Cone con2)
    {
        // It is supposed that axes of cones are coplanar and
        // distance between them > Precision::Confusion()
        Point3D pa1 = con1.Apex(), pa2 = con2.Apex();

        Point3D p3 = pa1.Translated(new Vector3D(con1.Position().Direction()));

        MakePln mkPln = new(pa1, pa2, p3);
        if (!mkPln.IsDone)
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        Plane3D pln = mkPln.Value();

        Line3D ax1 = new(pa1, con1.Position().Direction());
        Line2D ax12d = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(pln, ax1);
        Span<Line2D> lines1 = [new Line2D(), new Line2D()];
        double ang1 = con1.SemiAngle();
        lines1[0] = ax12d.Rotated(ax12d.Location(), ang1);
        lines1[1] = ax12d.Rotated(ax12d.Location(), -ang1);
        //
        Line3D ax2 = new(pa2, con2.Position().Direction());
        Line2D ax22d = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(pln, ax2);
        Span<Line2D> lines2 = [new Line2D(), new Line2D()];
        double ang2 = con2.SemiAngle();
        lines2[0] = ax22d.Rotated(ax22d.Location(), ang2);
        lines2[1] = ax22d.Rotated(ax22d.Location(), -ang2);

        Span<double> minDist = [global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Foundation.Precision.Infinite()];
        int i, j, k;
        AnalyticIntersection inter = new();
        for (i = 0; i < 2; ++i)
        {
            for (j = 0; j < 2; ++j)
            {
                inter.Perform(lines1[i], lines2[j]);
                if (inter.IsDone)
                {
                    int nbPoints = inter.PointCount;
                    for (k = 1; k <= nbPoints; ++k)
                    {
                        IntPoint intP = inter.Point(k);
                        double par1 = Math.Abs(intP.ParamOnFirst());
                        minDist[0] = Min(par1, minDist[0]);
                        double par2 = Math.Abs(intP.ParamOnSecond());
                        minDist[1] = Min(par2, minDist[1]);
                    }
                }
            }
        }

        double dist = Max(minDist[0], minDist[1]);
        return dist;
    }

    /// <summary>Empty constructor.</summary>
    public QuadQuadGeo()
    {
        _done = false;
        _nbint = 0;
        _typeres = Empty;
        _pt1 = new Point3D(0, 0, 0);
        _pt2 = new Point3D(0, 0, 0);
        _pt3 = new Point3D(0, 0, 0);
        _pt4 = new Point3D(0, 0, 0);
        _param1 = 0;
        _param2 = 0;
        _param3 = 0;
        _param4 = 0;
        _param1bis = 0;
        _param2bis = 0;
        _commonGen = false;
        _pChar = new Point3D(0, 0, 0);
        InitTolerances();
    }

    /// <summary>Initializes the values of inner tolerances.</summary>
    protected void InitTolerances()
    {
        _epsilonDistance = 1.0e-14;
        _epsilonAngleCone = global::Kerf.Engine.Foundation.Precision.Angular();
        _epsilonMiniCircleRadius = 0.01 * global::Kerf.Engine.Foundation.Precision.Confusion();
        _epsilonCylinderDeltaRadius = 1.0e-13;
        _epsilonCylinderDeltaDistance = global::Kerf.Engine.Foundation.Precision.Confusion();
        _epsilonAxesPara = global::Kerf.Engine.Foundation.Precision.Angular();
    }

    /// <summary>
    /// The intersection between two planes. TolAng is the angular tolerance below which the planes are parallel;
    /// Tol the distance below which parallel planes are identical.
    /// </summary>
    public QuadQuadGeo(in Plane3D p1, in Plane3D p2, double tolAng, double tol)
        : this()
    {
        Perform(p1, p2, tolAng, tol);
    }

    /// <summary>
    /// Intersects two planes. TolAng is the angular tolerance below which the planes are parallel; Tol the distance
    /// below which parallel planes are identical.
    /// </summary>
    public void Perform(in Plane3D p1, in Plane3D p2, double tolAng, double tol)
    {
        double a1, b1, c1, d1, a2, b2, c2, d2, dist1, dist2, mvd;
        //
        _done = false;
        _param2bis = 0.0;
        //
        p1.Coefficients(out a1, out b1, out c1, out d1);
        p2.Coefficients(out a2, out b2, out c2, out d2);
        //
        Vector3D vn1 = new(a1, b1, c1);
        Vector3D vn2 = new(a2, b2, c2);
        Vector3D vd = vn1.Crossed(vn2);
        //
        Point3D locP1 = p1.Location();
        Point3D locP2 = p2.Location();
        //
        dist1 = a2 * locP1.X() + b2 * locP1.Y() + c2 * locP1.Z() + d2;
        dist2 = a1 * locP2.X() + b1 * locP2.Y() + c1 * locP2.Z() + d1;
        //
        mvd = vd.Magnitude();
        if (mvd <= tolAng)
        {
            // normalles are collinear - planes are same or parallel
            _typeres = (Math.Abs(dist1) <= tol && Math.Abs(dist2) <= tol) ? Same : Empty;
        }
        else
        {
            double denom, denom2, ddenom, par1, par2;
            double x1, y1, z1, x2, y2, z2, eps;
            //
            eps = 1.0e-16;
            denom = a1 * a2 + b1 * b2 + c1 * c2;
            denom2 = denom * denom;
            ddenom = 1.0 - denom2;

            denom = (Math.Abs(ddenom) <= eps) ? eps : ddenom;

            par1 = dist1 / denom;
            par2 = -dist2 / denom;

            Vector3D inter1 = vn1.Crossed(vd);
            Vector3D inter2 = vn2.Crossed(vd);

            x1 = locP1.X() + par1 * inter1.X();
            y1 = locP1.Y() + par1 * inter1.Y();
            z1 = locP1.Z() + par1 * inter1.Z();
            x2 = locP2.X() + par2 * inter2.X();
            y2 = locP2.Y() + par2 * inter2.Y();
            z2 = locP2.Z() + par2 * inter2.Z();

            _pt1 = new Point3D((x1 + x2) * 0.5, (y1 + y2) * 0.5, (z1 + z2) * 0.5);
            _dir1 = new Direction3D(vd);
            _typeres = global::Kerf.Engine.Intersection.ResultType.Line;
            _nbint = 1;
            //
            //-------------------------------------------------------
            // When the value of the angle between the planes is small
            // the origin of intersection line is computed with error
            // [ ~0.0001 ] that can not br considered as small one
            // e.g.
            // for {A~=2.e-6, dist1=4.2e-5, dist2==1.e-4} =>
            // {denom=3.4e-12, par1=12550297.6, par2=32605552.9, etc}
            // So,
            // the origin should be refined if it is possible
            //
            double treshAng, treshDist;
            //
            treshAng = 2.0e-6; // 1.e-4 deg
            treshDist = 1.0e-12;
            //
            if (mvd < treshAng)
            {
                double aDist1, aDist2;
                //
                aDist1 = a1 * _pt1.X() + b1 * _pt1.Y() + c1 * _pt1.Z() + d1;
                aDist2 = a2 * _pt1.X() + b2 * _pt1.Y() + c2 * _pt1.Z() + d2;
                //
                if (Math.Abs(aDist1) > treshDist || Math.Abs(aDist2) > treshDist)
                {
                    bool bIsDone, bIsParallel;
                    IntConicQuad icq = new();
                    //
                    // 1.
                    Direction3D dn1 = new(vn1);
                    Line3D l1 = new(_pt1, dn1);
                    //
                    icq.Perform(l1, p1, tolAng, tol);
                    bIsDone = icq.IsDone;
                    if (!bIsDone)
                    {
                        return;
                    }
                    //
                    Point3D pnt1 = icq.Point(1);
                    //----------------------------------
                    // 2.
                    Direction3D dl2 = _dir1.Crossed(dn1);
                    Line3D l2 = new(pnt1, dl2);
                    //
                    icq.Perform(l2, p2, tolAng, tol);
                    bIsDone = icq.IsDone;
                    if (!bIsDone)
                    {
                        return;
                    }
                    //
                    bIsParallel = icq.IsParallel;
                    if (bIsParallel)
                    {
                        return;
                    }
                    //
                    Point3D pnt2 = icq.Point(1);
                    //
                    _pt1 = pnt2;
                }
            }
        }
        _done = true;
    }

    /// <summary>
    /// The intersection between a plane and a cylinder. Tolang is the angular tolerance below which the axis is
    /// parallel to the plane. Tol decides between a circle and an ellipse (the circle when the ellipse is within Tol
    /// of it). H, the height of the cylinder, helps decide whether the plane and the cylinder are parallel.
    /// </summary>
    public QuadQuadGeo(in Plane3D p, in Cylinder cl, double tolang, double tol, double h = 0)
        : this()
    {
        Perform(p, cl, tolang, tol, h);
    }

    /// <summary>
    /// Intersects a plane and a cylinder. Tolang is the angular tolerance below which the axis is parallel to the
    /// plane. Tol decides between a circle and an ellipse. H, the height of the cylinder, helps decide whether the
    /// plane and the cylinder are parallel.
    /// </summary>
    public void Perform(in Plane3D p, in Cylinder cl, double Tolang, double tol, double H = 0)
    {
        _done = false;
        double dist, radius;
        double a, b, c, d;
        double x, y, z;
        double sint, cost, h;
        Xyz axex, axey, omega = new();

        _param2bis = 0.0;
        radius = cl.Radius();

        Line3D axec = new(cl.Axis());
        Xyz normp = p.Axis().Direction().XYZ();

        p.Coefficients(out a, out b, out c, out d);
        axec.Location().Coord(out x, out y, out z);
        // The axis/plane distance is evaluated at the origin of the axis.
        dist = a * x + b * y + c * z + d;

        double tolang = Tolang;
        double toltang = tol;
        bool newparams = false;

        Vector3D ldv = new(axec.Direction());
        Vector3D npv = new(normp);
        double dA = Math.Abs(ldv.Angle(npv));
        if (dA > (M_PI / 4.0))
        {
            double dang = Math.Abs(ldv.Angle(npv)) - M_PI / 2.0;
            double dangle = Math.Abs(dang);
            if (dangle > Tolang)
            {
                double sinda = Math.Abs(Math.Sin(dangle));
                double dif = Math.Abs(sinda - tol);
                // When the cylinder face height H is known and small, use the deviation
                // of the axis over this height (sinda * H) as an additional criterion.
                // If the axis deviates from the plane by less than a small multiple of
                // the tolerance over the full height of the face, the axis is effectively
                // parallel for this face, and the Line intersection type should be used
                // instead of the Ellipse (which degenerates for nearly-parallel cases).
                if (dif < tol || (H > 0.0 && sinda * H < 2.0 * tol))
                {
                    tolang = sinda * 2.0;
                    // Relax the linear tolerance so that the endpoint distance check
                    // in IntAna_IntConicQuad (P.Distance(aP2) > Tol) does not reject
                    // the parallel status when the deviation over H is small.
                    toltang = Max(tol, sinda * H * 1.01);
                    newparams = true;
                }
            }
        }

        _nbint = 0;
        IntConicQuad inter = new(axec, p, tolang, toltang, H);

        if (inter.IsParallel)
        {
            // The result of the Plane-Cylinder intersection is of line type.
            // There are 1 or 2 lines

            _typeres = global::Kerf.Engine.Intersection.ResultType.Line;
            omega.SetCoord(x - dist * a, y - dist * b, z - dist * c);

            if (Math.Abs(Math.Abs(dist) - radius) < tol)
            {
                _nbint = 1;
                _pt1 = _pt1.WithXYZ(omega);

                if (newparams)
                {
                    Xyz omegaXYZ = new(x, y, z);
                    Xyz omegaXYZtrnsl = omegaXYZ + 100.0 * axec.Direction().XYZ();
                    double xt, yt, zt, distt;
                    omegaXYZtrnsl.Coord(out xt, out yt, out zt);
                    distt = a * xt + b * yt + c * zt + d;
                    Xyz omega1 = new(omegaXYZtrnsl.X() - distt * a,
                                        omegaXYZtrnsl.Y() - distt * b,
                                        omegaXYZtrnsl.Z() - distt * c);
                    Point3D ppt1 = new();
                    ppt1 = ppt1.WithXYZ(omega1);
                    Vector3D vv1 = new(_pt1, ppt1);
                    Direction3D dd1 = new(vv1);
                    _dir1 = dd1;
                }
                else
                {
                    _dir1 = axec.Direction();
                }
            }
            else if (Math.Abs(dist) < radius)
            {
                _nbint = 2;
                h = Math.Sqrt(radius * radius - dist * dist);
                axey = axec.Direction().XYZ().Crossed(normp); // axey is normalized

                _pt1 = _pt1.WithXYZ(omega - h * axey);
                _pt2 = _pt2.WithXYZ(omega + h * axey);

                if (newparams)
                {
                    Xyz omegaXYZ = new(x, y, z);
                    Xyz omegaXYZtrnsl = omegaXYZ + 100.0 * axec.Direction().XYZ();
                    double xt, yt, zt, distt, ht;
                    omegaXYZtrnsl.Coord(out xt, out yt, out zt);
                    distt = a * xt + b * yt + c * zt + d;
                    double sqrtArg = radius * radius - distt * distt;
                    ht = (sqrtArg > 0.0) ? Math.Sqrt(sqrtArg) : 0.0;

                    Xyz omega1 = new(omegaXYZtrnsl.X() - distt * a,
                                        omegaXYZtrnsl.Y() - distt * b,
                                        omegaXYZtrnsl.Z() - distt * c);
                    Point3D ppt1 = new(), ppt2 = new();
                    ppt1 = ppt1.WithXYZ(omega1 - ht * axey);
                    ppt2 = ppt2.WithXYZ(omega1 + ht * axey);
                    Vector3D vv1 = new(_pt1, ppt1);
                    Vector3D vv2 = new(_pt2, ppt2);
                    Direction3D dd1 = new(vv1);
                    Direction3D dd2 = new(vv2);
                    _dir1 = dd1;
                    _dir2 = dd2;
                }
                else
                {
                    _dir1 = axec.Direction();
                    _dir2 = axec.Direction();
                }
            }
            //  else nbint = 0

            // debug JAG : nbint = 0 should be replaced by typeres = IntAna_Empty
            // and not just be removed...

            else
            {
                _typeres = Empty;
            }
        }
        else
        { // There is one intersection point. It is the center of the circle
          // or of the solution ellipse.

            _nbint = 1;
            axey = normp.Crossed(axec.Direction().XYZ());
            sint = axey.Modulus();

            _pt1 = inter.Point(1);

            if (sint < tol / radius)
            {
                // Construct a circle with the X and Y axes of the cylinder
                _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;

                _dir1 = axec.Direction(); // Z axis
                _dir2 = cl.Position().XDirection();
                _param1 = radius;
            }
            else
            {
                // construct an ellipse
                _typeres = global::Kerf.Engine.Intersection.ResultType.Ellipse;
                cost = Math.Abs(axec.Direction().XYZ().Dot(normp));
                axex = axey.Crossed(normp);

                _dir1 = _dir1.WithXYZ(normp); // Modified in this block
                _dir2 = _dir2.WithXYZ(axex);

                _param1 = radius / cost;
                _param1bis = radius;
            }
        }

        _done = true;
    }

    /// <summary>
    /// The intersection between a plane and a cone. Tolang is the angular tolerance used to decide whether the
    /// axis is parallel or perpendicular to the plane and whether a generatrix is parallel to it; Tol the distance
    /// below which the apex is in the plane.
    /// </summary>
    public QuadQuadGeo(in Plane3D p, in Cone co, double tolang, double tol)
        : this()
    {
        Perform(p, co, tolang, tol);
    }

    /// <summary>Intersects a plane and a cone (see the constructor for the tolerances).</summary>
    public void Perform(in Plane3D p, in Cone co, double tolang, double tol)
    {
        _done = false;
        _nbint = 0;

        double a, b, c, d;
        double x, y, z;
        double dist, sint, cost, sina, cosa, angl, costa;
        double dh;
        Xyz axex, axey;

        Line3D axec = new(co.Axis());
        p.Coefficients(out a, out b, out c, out d);
        Point3D apex = co.Apex();

        apex.Coord(out x, out y, out z);
        dist = a * x + b * y + c * z + d; // signed distance from cone apex to plane

        Xyz normp = p.Axis().Direction().XYZ();
        if (!p.Direct())
        { //-- lbr 14 Jan 97
            normp.Reverse();
        }

        axey = normp.Crossed(co.Axis().Direction().XYZ());
        axex = axey.Crossed(normp);

        angl = co.SemiAngle();

        cosa = Math.Cos(angl);
        sina = Math.Abs(Math.Sin(angl));

        // Angle between the plane normal and the cone axis, brought between 0 and PI/2.

        sint = axey.Modulus();
        cost = Math.Abs(co.Axis().Direction().XYZ().Dot(normp));

        // The calculation of costa determines if the plane contains
        // a generatrix of the cone: we calculate std::sin((PI/2. - t) - angl)

        costa = cost * cosa - sint * sina; // sin((PI/2 -t)-angl)=cos(t+angl)
                                           // with t brought back between 0 and pi/2.

        if (Math.Abs(dist) < tol)
        {
            // we consider that the plane contains the apex of the cone.
            // the possible solutions are therefore: 1 point, 1 line, 2 lines
            // according to the inclination of the plane.

            if (Math.Abs(costa) < tolang)
            { // plane parallel to the generatrix
                _typeres = global::Kerf.Engine.Intersection.ResultType.Line;
                _nbint = 1;
                Xyz ptonaxe = apex.XYZ() + 10.0 * (co.Axis().Direction().XYZ());
                // point on the axis of the cone on the positive z side

                dist = a * ptonaxe.X() + b * ptonaxe.Y() + c * ptonaxe.Z() + d;
                ptonaxe = ptonaxe - dist * normp;
                _pt1 = apex;
                _dir1 = _dir1.WithXYZ(ptonaxe - _pt1.XYZ());
            }
            else if (cost < sina)
            { // plane "interior" to the cone
                _typeres = global::Kerf.Engine.Intersection.ResultType.Line;
                _nbint = 2;
                _pt1 = apex;
                _pt2 = apex;
                dh = Math.Sqrt(sina * sina - cost * cost) / cosa;
                _dir1 = _dir1.WithXYZ(axex + dh * axey);
                _dir2 = _dir2.WithXYZ(axex - dh * axey);
            }
            else
            { // plane "exterior" to the cone
                _typeres = global::Kerf.Engine.Intersection.ResultType.Point;
                _nbint = 1;
                _pt1 = apex;
            }
        }
        else
        {
            // Possible solutions: circle, ellipse, parabola, hyperbola according to
            // the inclination of the plane.
            double deltacenter, distance;

            if (cost < tolang)
            {
                // The plane contains the direction of the cone axis. The solution is
                // the hyperbola
                _typeres = global::Kerf.Engine.Intersection.ResultType.Hyperbola;
                _nbint = 2;
                _pt1 = _pt1.WithXYZ(apex.XYZ() - dist * normp);
                _pt2 = _pt1;
                _dir1 = new Direction3D(normp);
                _dir2 = _dir2.WithXYZ(axex);
                _param1 = _param2 = Math.Abs(dist / Math.Tan(angl));
                _param1bis = _param2bis = Math.Abs(dist);
            }
            else
            {
                IntConicQuad inter = new(axec, p, tolang); // we necessarily have 1 point.

                Point3D center = inter.Point(1);

                // Depending on the position of the intersection relative to the apex
                // of the cone, we change the x axis to -x and y to -y. The parameter of the apex
                // on axec is negative (see definition of the cone)

                distance = apex.Distance(center);

                if (inter.ParamOnConic(1) + co.RefRadius() / Math.Tan(angl) < 0.0)
                {
                    axex.Reverse();
                    axey.Reverse();
                }

                if (Math.Abs(costa) < tolang)
                { // plane parallel to a generatrix
                    _typeres = global::Kerf.Engine.Intersection.ResultType.Parabola;
                    _nbint = 1;
                    deltacenter = distance / 2.0 / cosa;
                    axex.Normalize();
                    _pt1 = _pt1.WithXYZ(center.XYZ() - deltacenter * axex);
                    _dir1 = new Direction3D(normp);
                    _dir2 = _dir2.WithXYZ(axex);
                    _param1 = deltacenter * sina * sina;
                }
                else if (sint < tolang)
                { // plane perpendicular to the axis
                    _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
                    _nbint = 1;
                    _pt1 = center;
                    _dir1 = co.Position().Direction();
                    _dir2 = co.Position().XDirection();
                    _param1 = apex.Distance(center) * Math.Abs(Math.Tan(angl));
                }
                else if (cost < sina)
                {
                    _typeres = global::Kerf.Engine.Intersection.ResultType.Hyperbola;
                    _nbint = 2;
                    axex.Normalize();

                    deltacenter = sint * sina * sina * distance / (sina * sina - cost * cost);
                    _pt1 = _pt1.WithXYZ(center.XYZ() - deltacenter * axex);
                    _pt2 = _pt1;
                    _dir1 = new Direction3D(normp);
                    _dir2 = _dir2.WithXYZ(axex);
                    _param1 = _param2 = cost * sina * cosa * distance / (sina * sina - cost * cost);
                    _param1bis = _param2bis = cost * sina * distance / Math.Sqrt(sina * sina - cost * cost);
                }
                else
                { // here cost > sina
                    _typeres = global::Kerf.Engine.Intersection.ResultType.Ellipse;
                    _nbint = 1;
                    double radius = cost * sina * cosa * distance / (cost * cost - sina * sina);
                    deltacenter = sint * sina * sina * distance / (cost * cost - sina * sina);
                    axex.Normalize();
                    _pt1 = _pt1.WithXYZ(center.XYZ() + deltacenter * axex);
                    _dir1 = new Direction3D(normp);
                    _dir2 = _dir2.WithXYZ(axex);
                    _param1 = radius;
                    _param1bis = cost * sina * distance / Math.Sqrt(cost * cost - sina * sina);
                }
            }
        }

        //-- Difficult to handle further (Value ProjLib, Params ... )
        //-- hyperbolas that are too extreme
        //-- Return False -> Treatment by biparametric method
        const double ellipseLimit = 1.0E+9;   // OCC513(apo) 1000000
        const double hyperbolaLimit = 2.0E+6; // OCC537(apo) 50000
        if (_typeres == global::Kerf.Engine.Intersection.ResultType.Ellipse && _nbint >= 1)
        {
            if (Math.Abs(_param1) > ellipseLimit || Math.Abs(_param1bis) > ellipseLimit)
            {
                _done = false;
                return;
            }
        }
        if (_typeres == global::Kerf.Engine.Intersection.ResultType.Hyperbola && _nbint >= 2)
        {
            if (Math.Abs(_param2) > hyperbolaLimit || Math.Abs(_param2bis) > hyperbolaLimit)
            {
                _done = false;
                return;
            }
        }
        if (_typeres == global::Kerf.Engine.Intersection.ResultType.Hyperbola && _nbint >= 1)
        {
            if (Math.Abs(_param1) > hyperbolaLimit || Math.Abs(_param1bis) > hyperbolaLimit)
            {
                _done = false;
                return;
            }
        }

        _done = true;
    }

    /// <summary>The intersection between a plane and a sphere.</summary>
    public QuadQuadGeo(in Plane3D p, in Sphere s)
        : this()
    {
        Perform(p, s);
    }

    /// <summary>Intersects a plane and a sphere.</summary>
    public void Perform(in Plane3D p, in Sphere s)
    {
        _done = false;
        double a, b, c, d, dist, radius;
        double x, y, z;

        _nbint = 0;
        // debug JAG : on met typeres = IntAna_Empty par defaut...
        _typeres = Empty;

        p.Coefficients(out a, out b, out c, out d);
        s.Location().Coord(out x, out y, out z);
        radius = s.Radius();

        dist = a * x + b * y + c * z + d;

        if (Math.Abs(Math.Abs(dist) - radius) < Epsilon(radius))
        {
            // we have a single solution: the projection point of the sphere center
            // onto the plane
            _nbint = 1;
            _typeres = global::Kerf.Engine.Intersection.ResultType.Point;
            _pt1 = _pt1.WithCoord(x - dist * a, y - dist * b, z - dist * c);
        }
        else if (Math.Abs(dist) < radius)
        {
            // we have a circle solution
            _nbint = 1;
            _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
            _pt1 = _pt1.WithCoord(x - dist * a, y - dist * b, z - dist * c);
            _dir1 = p.Axis().Direction();
            if (!p.Direct())
            {
                _dir1 = _dir1.Reversed();
            }
            _dir2 = p.Position().XDirection();
            _param1 = Math.Sqrt(radius * radius - dist * dist);
        }
        _param2bis = 0.0; //-- to avoid param2bis unused warning
        _done = true;
    }

    /// <summary>The intersection between two cylinders.</summary>
    public QuadQuadGeo(in Cylinder cyl1, in Cylinder cyl2, double tol)
        : this()
    {
        Perform(cyl1, cyl2, tol);
    }

    /// <summary>Intersects two cylinders.</summary>
    public void Perform(in Cylinder cyl1, in Cylinder cyl2, double tol)
    {
        _done = true;
        //---------------------------- Parallel axes -------------------------
        AxeOperator a1A2 = new(cyl1.Axis(),
                               cyl2.Axis(),
                               _epsilonCylinderDeltaDistance,
                               _epsilonAxesPara);
        double r1 = cyl1.Radius();
        double r2 = cyl2.Radius();
        double rmR, RmR_Relative;
        rmR = (r1 > r2) ? (r1 - r2) : (r2 - r1);
        {
            double rmax;
            rmax = (r1 > r2) ? r1 : r2;
            RmR_Relative = rmR / rmax;
        }

        double distA1A2 = a1A2.Distance();

        if (a1A2.Parallel())
        {
            if (distA1A2 <= tol)
            {
                if (rmR <= tol)
                {
                    _typeres = Same;
                }
                else
                {
                    _typeres = Empty;
                }
            }
            else
            { //-- DistA1A2 > Tol
                Point3D P1 = cyl1.Location();
                Point3D p2t = cyl2.Location();
                Point3D P2 = new();
                //-- P2t is projected on the plane (P1,DirCylX,DirCylY)
                Direction3D dirCyl = cyl1.Position().Direction();
                double projP2OnDirCyl1 = new Vector3D(dirCyl).Dot(new Vector3D(P1, p2t));

                // P2 is a projection the location of the 2nd cylinder on the base
                // of the 1st cylinder
                P2 = P2.WithCoord(p2t.X() - projP2OnDirCyl1 * dirCyl.X(),
                            p2t.Y() - projP2OnDirCyl1 * dirCyl.Y(),
                            p2t.Z() - projP2OnDirCyl1 * dirCyl.Z());
                //--
                double r1pR2 = r1 + r2;
                if (distA1A2 > (r1pR2 + tol))
                {
                    _typeres = Empty;
                    _nbint = 0;
                }
                else if ((r1pR2 - distA1A2) <= RealSmall())
                {
                    //-- 1 Tangent line -------------------------------------OK
                    _typeres = global::Kerf.Engine.Intersection.ResultType.Line;

                    _nbint = 1;
                    _dir1 = dirCyl;
                    double R1_R1pR2 = r1 / r1pR2;
                    _pt1 = _pt1.WithCoord(P1.X() + R1_R1pR2 * (P2.X() - P1.X()),
                                 P1.Y() + R1_R1pR2 * (P2.Y() - P1.Y()),
                                 P1.Z() + R1_R1pR2 * (P2.Z() - P1.Z()));
                }
                else if (distA1A2 > rmR)
                {
                    //-- 2 lines ---------------------------------------------OK
                    _typeres = global::Kerf.Engine.Intersection.ResultType.Line;
                    _nbint = 2;
                    _dir1 = dirCyl;
                    _dir2 = _dir1;

                    double r1R1 = r1 * r1;

                    /*
                                  P1
                                  o
                                * | *
                              * O1|   *
                          A o-----o-----o B
                              *   |   *
                                * | *
                                  o
                                  P2

                      Two cylinders have axes collinear. Therefore, problem can be reformulated as
                      to find intersection point of two circles (the bases of the cylinders) on
                      the plane: 1st circle has center P1 and radius R1 (the radius of the
                      1st cylinder) and 2nd circle has center P2 and radius R2 (the radius of the
                      2nd cylinder). The plane is the base of the 1st cylinder. Points A and B
                      are intersection point of these circles. Distance P1P2 is equal to DistA1A2.
                      O1 is the intersection point of P1P2 and AB segments.

                      At that, if distance AB < Tol we consider that the circles are tangent and
                      has only one intersection point.

                        AB = 2*R1*sin(angle AP1P2).
                      Accordingly,
                        AB^2 < Tol^2 => 4*R1*R1*sin(angle AP1P2)^2 < Tol*Tol.
                    */

                    // Cosine and Square of Sine of the A-P1-P2 angle
                    double cos = 0.5 * (r1R1 - r2 * r2 + distA1A2 * distA1A2) / (r1 * distA1A2);
                    double sin2 = 1 - cos * cos;

                    bool isTangent = ((4.0 * r1R1 * sin2) < tol * tol);

                    // Normalized vector P1P2
                    Vector3D dirA1A2 = new((P2.XYZ() - P1.XYZ()) / distA1A2);

                    if (isTangent)
                    {
                        // Intercept the segment from P1 point along P1P2 direction
                        // and having |P1O1| length
                        _nbint = 1;
                        _pt1 = _pt1.WithXYZ(P1.XYZ() + dirA1A2.XYZ() * r1 * cos);
                    }
                    else
                    {
                        // Sine of the A-P1-P2 angle (if aSin2 < 0 then isTangent == TRUE =>
                        // go to another branch)
                        double sin = Math.Sqrt(sin2);

                        // 1. Rotate P1P2 to the angle A-P1-P2 relative to P1
                        //(clockwise and anticlockwise for getting
                        // two intersection points).
                        // 2. Intercept the segment from P1 along direction,
                        // determined in the preview paragraph and having R1 length
                        Direction3D xDir = cyl1.Position().XDirection(), yDir = cyl1.Position().YDirection();
                        Vector3D r1Xdir = new(r1 * xDir.XYZ()), r1Ydir = new(r1 * yDir.XYZ());

                        // Source 2D-coordinates of the P1P2 vector normalized
                        // in coordinate system, based on the X- and Y-directions
                        // of the 1st cylinder in the plane of the 1st cylinder base
                        //(P1 is the origin of the coordinate system).
                        double dx = dirA1A2.Dot(new Vector3D(xDir)), dy = dirA1A2.Dot(new Vector3D(yDir));

                        // New coordinate (after rotation) of the P1P2 vector normalized.
                        double newDx = dx * cos - dy * sin, newDy = dy * cos + dx * sin;
                        _pt1 = _pt1.WithXYZ(P1.XYZ() + newDx * r1Xdir.XYZ() + newDy * r1Ydir.XYZ());

                        newDx = dx * cos + dy * sin;
                        newDy = dy * cos - dx * sin;
                        _pt2 = _pt2.WithXYZ(P1.XYZ() + newDx * r1Xdir.XYZ() + newDy * r1Ydir.XYZ());
                    }
                }
                else if (distA1A2 > (rmR - tol))
                {
                    //-- 1 Tangent ------------------------------------------OK
                    _typeres = global::Kerf.Engine.Intersection.ResultType.Line;
                    _nbint = 1;
                    _dir1 = dirCyl;
                    double R1_RmR = r1 / rmR;

                    if (r1 < r2)
                    {
                        R1_RmR = -R1_RmR;
                    }

                    _pt1 = _pt1.WithCoord(P1.X() + R1_RmR * (P2.X() - P1.X()),
                                 P1.Y() + R1_RmR * (P2.Y() - P1.Y()),
                                 P1.Z() + R1_RmR * (P2.Z() - P1.Z()));
                }
                else
                {
                    _nbint = 0;
                    _typeres = Empty;
                }
            }
        }
        else
        { //-- No Parallel Axis ---------------------------------OK
            if ((RmR_Relative <= _epsilonCylinderDeltaRadius) && a1A2.Intersect())
            {
                //-- PI/2 between the two axis   and   Intersection
                //-- and identical radius
                _typeres = global::Kerf.Engine.Intersection.ResultType.Ellipse;
                _nbint = 2;
                Direction3D dirCyl1 = cyl1.Position().Direction();
                Direction3D dirCyl2 = cyl2.Position().Direction();
                _pt1 = _pt2 = a1A2.PtIntersect();

                double a = dirCyl1.Angle(dirCyl2);
                double b;
                b = Math.Abs(Math.Sin(0.5 * (M_PI - a)));
                a = Math.Abs(Math.Sin(0.5 * a));

                if (a == 0.0 || b == 0.0)
                {
                    _typeres = Same;
                    return;
                }

                Vector3D dircyl1 = new(dirCyl1);
                Vector3D dircyl2 = new(dirCyl2);
                _dir1 = new Direction3D(dircyl1.Added(dircyl2));
                _dir2 = new Direction3D(dircyl1.Subtracted(dircyl2));

                _param2 = cyl1.Radius() / a;
                _param1 = cyl1.Radius() / b;
                _param2bis = _param1bis = cyl1.Radius();
                if (_param1 < _param1bis)
                {
                    a = _param1;
                    _param1 = _param1bis;
                    _param1bis = a;
                }

                if (_param2 < _param2bis)
                {
                    a = _param2;
                    _param2 = _param2bis;
                    _param2bis = a;
                }
            }
            else
            {
                if (Math.Abs(distA1A2 - cyl1.Radius() - cyl2.Radius()) < tol)
                {
                    _typeres = global::Kerf.Engine.Intersection.ResultType.Point;
                    double d, p1, p2;

                    Direction3D d1 = cyl1.Axis().Direction();
                    Direction3D d2 = cyl2.Axis().Direction();
                    a1A2.Distance(out d, out p1, out p2);
                    Point3D p = cyl1.Axis().Location();
                    Point3D P1 = new(p.X() - p1 * d1.X(), p.Y() - p1 * d1.Y(), p.Z() - p1 * d1.Z());

                    p = cyl2.Axis().Location();

                    Point3D P2 = new(p.X() - p2 * d2.X(), p.Y() - p2 * d2.Y(), p.Z() - p2 * d2.Z());

                    Vector3D p1P2 = new(P1, P2);
                    d1 = new Direction3D(p1P2);
                    p1 = cyl1.Radius();

                    _pt1 = _pt1.WithCoord(P1.X() + p1 * d1.X(), P1.Y() + p1 * d1.Y(), P1.Z() + p1 * d1.Z());
                    _nbint = 1;
                }
                else
                {
                    _typeres = NoGeometricSolution;
                }
            }
        }
    }

    /// <summary>The intersection between a cylinder and a cone (only when they are coaxial).</summary>
    public QuadQuadGeo(in Cylinder cyl, in Cone con, double tol)
        : this()
    {
        Perform(cyl, con, tol);
    }

    /// <summary>Intersects a cylinder and a cone (only when they are coaxial). Tol is not used.</summary>
    public void Perform(in Cylinder cyl, in Cone con, double tol)
    {
        _ = tol;
        _done = true;
        AxeOperator a1A2 = new(cyl.Axis(), con.Axis());
        if (a1A2.Same())
        {
            Point3D pt = con.Apex();
            double dist = cyl.Radius() / (Math.Tan(con.SemiAngle()));
            Direction3D dir = cyl.Position().Direction();
            _pt1 = _pt1.WithCoord(pt.X() + dist * dir.X(), pt.Y() + dist * dir.Y(), pt.Z() + dist * dir.Z());
            _pt2 = _pt2.WithCoord(pt.X() - dist * dir.X(), pt.Y() - dist * dir.Y(), pt.Z() - dist * dir.Z());
            _dir1 = _dir2 = dir;
            _param1 = _param2 = cyl.Radius();
            _nbint = 2;
            _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
        }
        else
        {
            _typeres = NoGeometricSolution;
        }
    }

    /// <summary>The intersection between a cylinder and a sphere (only when the center is on the axis).</summary>
    public QuadQuadGeo(in Cylinder cyl, in Sphere sph, double tol)
        : this()
    {
        Perform(cyl, sph, tol);
    }

    /// <summary>Intersects a cylinder and a sphere (only when the center is on the axis). Tol is not used.</summary>
    public void Perform(in Cylinder cyl, in Sphere sph, double tol)
    {
        _ = tol;
        _done = true;
        Point3D pt = sph.Location();
        AxeOperator a1A2 = new(cyl.Axis(), sph.Position().Axis());
        if ((a1A2.Intersect() && pt.Distance(a1A2.PtIntersect()) == 0.0) || (a1A2.Same()))
        {
            if (sph.Radius() < cyl.Radius())
            {
                _typeres = Empty;
            }
            else
            {
                double dist = Math.Sqrt(sph.Radius() * sph.Radius() - cyl.Radius() * cyl.Radius());
                Direction3D dir = cyl.Position().Direction();
                _dir1 = _dir2 = dir;
                _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
                _pt1 = _pt1.WithCoord(pt.X() + dist * dir.X(), pt.Y() + dist * dir.Y(), pt.Z() + dist * dir.Z());
                _nbint = 1;
                _param1 = cyl.Radius();
                if (dist > RealEpsilon())
                {
                    _pt2 = _pt2.WithCoord(pt.X() - dist * dir.X(), pt.Y() - dist * dir.Y(), pt.Z() - dist * dir.Z());
                    _param2 = cyl.Radius();
                    _nbint = 2;
                }
            }
        }
        else
        {
            _typeres = NoGeometricSolution;
        }
    }

    /// <summary>The intersection between two cones.</summary>
    public QuadQuadGeo(in Cone con1, in Cone con2, double tol)
        : this()
    {
        Perform(con1, con2, tol);
    }

    /// <summary>Intersects two cones.</summary>
    public void Perform(in Cone con1, in Cone con2, double tol)
    {
        _done = true;
        //
        double tg1, tg2, da1A2, aTol2;
        Point3D pApex1, pApex2;

        double TOL_APEX_CONF = 1.0e-10;

        //
        tg1 = Math.Tan(con1.SemiAngle());
        tg2 = Math.Tan(con2.SemiAngle());

        if ((tg1 * tg2) < 0.0)
        {
            tg2 = -tg2;
        }
        //
        aTol2 = tol * tol;
        pApex1 = con1.Apex();
        pApex2 = con2.Apex();
        da1A2 = pApex1.SquareDistance(pApex2);
        //
        AxeOperator a1A2 = new(con1.Axis(), con2.Axis());
        //
        double tolAng = _epsilonAngleCone;
        if ((Math.Abs(tg1 - tg2) < tol) && (a1A2.Parallel()))
        {
            double distA1A2 = a1A2.Distance();
            if (distA1A2 > 100.0 * tol)
            {
                double minSolDist = EstimDist(con1, con2);
                if (minSolDist < Epsilon(1.0))
                {
                    tolAng = tol;
                }
                else
                {
                    tolAng = Max(_epsilonAngleCone, tol / minSolDist);
                    tolAng = Min(tolAng, tol);
                }
            }
        }

        // 1
        if (a1A2.Same())
        {
            //-- two circles
            double x;
            Point3D P = con1.Apex();
            Direction3D D = con1.Position().Direction();
            double d = new Vector3D(D).Dot(new Vector3D(P, con2.Apex()));

            if (Math.Abs(tg1 - tg2) > _epsilonAngleCone)
            {
                if (Math.Abs(d) < TOL_APEX_CONF)
                {
                    _typeres = global::Kerf.Engine.Intersection.ResultType.Point;
                    _nbint = 1;
                    _pt1 = P;
                    return;
                }
                x = (d * tg2) / (tg1 + tg2);
                _pt1 = _pt1.WithCoord(P.X() + x * D.X(), P.Y() + x * D.Y(), P.Z() + x * D.Z());
                _param1 = Math.Abs(x * tg1);

                x = (d * tg2) / (tg2 - tg1);
                _pt2 = _pt2.WithCoord(P.X() + x * D.X(), P.Y() + x * D.Y(), P.Z() + x * D.Z());
                _param2 = Math.Abs(x * tg1);
                _dir1 = _dir2 = D;
                _nbint = 2;
                _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
            }
            else
            {
                if (Math.Abs(d) < TOL_APEX_CONF)
                {
                    _typeres = Same;
                }
                else
                {
                    _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
                    _nbint = 1;
                    x = d * 0.5;
                    _pt1 = _pt1.WithCoord(P.X() + x * D.X(), P.Y() + x * D.Y(), P.Z() + x * D.Z());
                    _param1 = Math.Abs(x * tg1);
                    _dir1 = D;
                }
            }
        } //-- fin A1A2.Same
        // 2
        else if ((Math.Abs(tg1 - tg2) < tolAng) && (a1A2.Parallel()))
        {
            double distA1A2 = a1A2.Distance();
            Direction3D da1 = con1.Position().Direction();
            Vector3D o1O2 = new(con1.Apex(), con2.Apex());
            Direction3D o1O2n = new(o1O2); // normalization of the vector before projection
            double O1O2_DA1 = new Vector3D(da1).Dot(new Vector3D(o1O2n));

            Vector3D O1_Proj_A2 = new(o1O2n.X() - O1O2_DA1 * da1.X(),
                                    o1O2n.Y() - O1O2_DA1 * da1.Y(),
                                    o1O2n.Z() - O1O2_DA1 * da1.Z());
            Direction3D db1 = new(O1_Proj_A2);

            double yO1O2 = o1O2.Dot(new Vector3D(da1));
            double abstg1 = Math.Abs(tg1);
            double x2 = (distA1A2 / abstg1 - yO1O2) * 0.5;
            double x1 = x2 + yO1O2;

            Point3D P1 = new(con1.Apex().X() + x1 * (da1.X() + abstg1 * db1.X()),
                            con1.Apex().Y() + x1 * (da1.Y() + abstg1 * db1.Y()),
                            con1.Apex().Z() + x1 * (da1.Z() + abstg1 * db1.Z()));

            Point3D mo1O2 = new(0.5 * (con1.Apex().X() + con2.Apex().X()),
                               0.5 * (con1.Apex().Y() + con2.Apex().Y()),
                               0.5 * (con1.Apex().Z() + con2.Apex().Z()));
            Vector3D p1MO1O2 = new(P1, mo1O2);

            Direction3D DA1_X_DB1 = da1.Crossed(db1);
            Direction3D orthoPln = DA1_X_DB1.Crossed(new Direction3D(p1MO1O2));

            QuadQuadGeo INTER_QUAD_PLN = new(new Plane3D(P1, orthoPln), con1, tol, tol);
            if (INTER_QUAD_PLN.IsDone)
            {
                switch (INTER_QUAD_PLN.TypeInter())
                {
                    case global::Kerf.Engine.Intersection.ResultType.Ellipse:
                    {
                        _typeres = global::Kerf.Engine.Intersection.ResultType.Ellipse;
                        Ellipse3D e = INTER_QUAD_PLN.Ellipse(1);
                        _pt1 = e.Location();
                        _dir1 = e.Position().Direction();
                        _dir2 = e.Position().XDirection();
                        _param1 = e.MajorRadius();
                        _param1bis = e.MinorRadius();
                        _nbint = 1;
                        break;
                    }
                    case global::Kerf.Engine.Intersection.ResultType.Circle:
                    {
                        _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
                        Circle3D c = INTER_QUAD_PLN.Circle(1);
                        _pt1 = c.Location();
                        _dir1 = c.Position().XDirection();
                        _dir2 = c.Position().YDirection();
                        _param1 = c.Radius();
                        _nbint = 1;
                        break;
                    }
                    case global::Kerf.Engine.Intersection.ResultType.Hyperbola:
                    {
                        _typeres = global::Kerf.Engine.Intersection.ResultType.Hyperbola;
                        Hyperbola3D h = INTER_QUAD_PLN.Hyperbola(1);
                        _pt1 = _pt2 = h.Location();
                        _dir1 = h.Position().Direction();
                        _dir2 = h.Position().XDirection();
                        _param1 = _param2 = h.MajorRadius();
                        _param1bis = _param2bis = h.MinorRadius();
                        _nbint = 2;
                        break;
                    }
                    case global::Kerf.Engine.Intersection.ResultType.Line:
                    {
                        _typeres = global::Kerf.Engine.Intersection.ResultType.Line;
                        Line3D h = INTER_QUAD_PLN.Line(1);
                        _pt1 = _pt2 = h.Location();
                        _dir1 = _dir2 = h.Position().Direction();
                        _param1 = _param2 = 0.0;
                        _param1bis = _param2bis = 0.0;
                        _nbint = 2;
                        break;
                    }
                    default:
                        _typeres = NoGeometricSolution;
                        break;
                }
            }
        }
        // 3
        else if (da1A2 < aTol2)
        {
            //
            // When apices are coincided there can be 3 possible cases
            // 3.1 - empty solution (iRet=0)
            // 3.2 - one line  when cone1 touches cone2 (iRet=1)
            // 3.3 - two lines when cone1 intersects cone2 (iRet=2)
            //
            double gamma, beta1, beta2;
            double aD1, r1, tgBeta1, tgBeta2, halfPI;
            double cosGamma, sinGamma, dx, r2, rd2, aD2;
            Point2D p0 = new(), pa1 = new(), p1 = new(), pa2;
            Vector2D vAx2 = new();
            Axis3D ax1, ax2;
            //
            // Preliminary analysis. Determination of iRet
            //
            halfPI = 0.5 * M_PI;
            aD1 = 1.0;
            pa1 = pa1.WithCoord(aD1, 0.0);
            p0 = p0.WithCoord(0.0, 0.0);
            //
            ax1 = con1.Axis();
            ax2 = con2.Axis();
            gamma = ax1.Angle(ax2);
            if (gamma > halfPI)
            {
                gamma = M_PI - gamma;
            }
            cosGamma = Math.Cos(gamma);
            sinGamma = Math.Sin(gamma);
            //
            beta1 = con1.SemiAngle();
            tgBeta1 = Math.Tan(beta1);
            tgBeta1 = Math.Abs(tgBeta1);
            //
            beta2 = con2.SemiAngle();
            tgBeta2 = Math.Tan(beta2);
            tgBeta2 = Math.Abs(tgBeta2);
            //
            r1 = aD1 * tgBeta1;
            p1 = p1.WithCoord(aD1, r1);
            //
            // PA2
            vAx2 = vAx2.WithCoord(cosGamma, sinGamma);
            Direction2D dAx2 = new(vAx2);
            Line2D lAx2 = new(p0, dAx2);
            _ = lAx2;
            //
            Vector2D aV = new(p0, p1);
            dx = vAx2.Dot(aV);
            pa2 = p0.Translated(dx * new Vector2D(dAx2));
            //
            // aR2
            dx = pa2.Distance(p0);
            r2 = dx * tgBeta2;
            //
            // aRD2
            rd2 = pa2.Distance(p1);
            //
            if (rd2 > (r2 + tol))
            {
                _typeres = Empty; // nothing
                return;
            }
            //
            int iRet = 1; // touch case => 1 line
            if (rd2 < (r2 - tol))
            {
                iRet = 2; // intersection => couple of lines
            }
            //
            // Finding the solution in 3D
            //
            double da;
            Point3D qApex1, qa1 = new(), qa2 = new(), qx, qx1, qx2;
            Direction3D d3Ax1, d3Ax2;
            Line3D lin;
            QuadQuadGeo intr = new();
            //
            qApex1 = con1.Apex();
            d3Ax1 = ax1.Direction();
            qa1 = qa1.WithCoord(qApex1.X() + aD1 * d3Ax1.X(),
                          qApex1.Y() + aD1 * d3Ax1.Y(),
                          qApex1.Z() + aD1 * d3Ax1.Z());
            //
            dx = d3Ax1.Dot(ax2.Direction());
            if (dx < 0.0)
            {
                ax2 = ax2.Reversed();
            }
            d3Ax2 = ax2.Direction();
            //
            aD2 = aD1 * Math.Sqrt((1.0 + tgBeta1 * tgBeta1) / (1.0 + tgBeta2 * tgBeta2));
            //
            qa2 = qa2.WithCoord(qApex1.X() + aD2 * d3Ax2.X(),
                          qApex1.Y() + aD2 * d3Ax2.Y(),
                          qApex1.Z() + aD2 * d3Ax2.Z());
            //
            Plane3D pln1 = new(qa1, d3Ax1);
            Plane3D pln2 = new(qa2, d3Ax2);
            //
            intr.Perform(pln1, pln2, tol, tol);
            if (!intr.IsDone || 0 == intr.SolutionCount)
            {
                _typeres = NoGeometricSolution;
                return;
            }
            //
            lin = intr.Line(1);
            Direction3D dLin = lin.Direction();
            Vector3D vLin = new(dLin);
            Point3D orig = lin.Location();
            Vector3D vr = new(qa1, orig);
            dx = vLin.Dot(vr);
            qx = orig.Translated(dx * vLin);
            //
            // Final part
            //
            _typeres = global::Kerf.Engine.Intersection.ResultType.Line;
            //
            _param1 = 0.0;
            _param2 = 0.0;
            _param1bis = 0.0;
            _param2bis = 0.0;
            //
            if (iRet == 1)
            {
                // one line
                _nbint = 1;
                _pt1 = qApex1;
                Vector3D vx = new(qApex1, qx);
                _dir1 = new Direction3D(vx);
            }
            else
            { // iRet=2
                // two lines
                _nbint = 2;
                da = qa1.Distance(qx);
                dx = Math.Sqrt(r1 * r1 - da * da);
                qx1 = qx.Translated(dx * vLin);
                qx2 = qx.Translated(-dx * vLin);
                //
                _pt1 = qApex1;
                _pt2 = qApex1;
                Vector3D vx1 = new(qApex1, qx1);
                _dir1 = new Direction3D(vx1);
                Vector3D vx2 = new(qApex1, qx2);
                _dir2 = new Direction3D(vx2);
            }
        } // else if (aDA1A2<aTol2) {
        // Case when cones have common generatrix
        else if (a1A2.Intersect())
        {
            // Check if apex of one cone belongs another one
            double u, v, tol2 = tol * tol;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(con2, pApex1, out u, out v);
            Point3D p = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, con2);
            if (pApex1.SquareDistance(p) > tol2)
            {
                _typeres = NoGeometricSolution;
                return;
            }
            //
            global::Kerf.Engine.Geometry.ElSLib.Parameters(con1, pApex2, out u, out v);
            p = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, con1);
            if (pApex2.SquareDistance(p) > tol2)
            {
                _typeres = NoGeometricSolution;
                return;
            }

            // Cones have a common generatrix passing through apexes
            _commonGen = true;

            // common generatrix of cones
            Line3D gen = new(pApex1, new Direction3D(new Vector3D(pApex1, pApex2)));

            // Intersection point of axes
            Point3D pAxeInt = a1A2.PtIntersect();

            // Characteristic point of intersection curve
            u = global::Kerf.Engine.Geometry.ElCLib.Parameter(gen, pAxeInt);
            _pChar = global::Kerf.Engine.Geometry.ElCLib.Value(u, gen);

            // Other generatrixes of cones laying in maximal plane
            Line3D gen1 = gen.Rotated(con1.Axis(), M_PI);
            Line3D gen2 = gen.Rotated(con2.Axis(), M_PI);
            //
            // Intersection point of generatrixes
            Direction3D n; // solution plane normal
            Direction3D aD1 = gen1.Direction();

            Direction3D aD2 = aD1.Crossed(gen.Direction());

            if (aD1.IsParallel(gen2.Direction(), global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                n = aD1.Crossed(aD2);
            }
            else if (gen1.SquareDistance(gen2) > tol2)
            {
                // Something wrong ???
                _typeres = NoGeometricSolution;
                return;
            }
            else
            {
                Direction3D d1 = gen1.Position().Direction();
                Direction3D d2 = gen2.Position().Direction();
                Point3D o1 = gen1.Location();
                Point3D o2 = gen2.Location();
                double d1DotD2 = d1.Dot(d2);
                double sin = 1.0 - d1DotD2 * d1DotD2;
                Vector3D o1O2 = new(o1, o2);
                double u2 = (d1.XYZ() * (o1O2.Dot(new Vector3D(d1))) - (o1O2.XYZ())).Dot(d2.XYZ());
                u2 /= sin;
                Point3D pGint = global::Kerf.Engine.Geometry.ElCLib.Value(u2, gen2);

                aD1 = new Direction3D(new Vector3D(pGint, _pChar));
                n = aD1.Crossed(aD2);
            }
            // Plane that must contain intersection curves
            Plane3D intPln = new(_pChar, n);

            QuadQuadGeo INTER_QUAD_PLN = new(intPln, con1, tol, tol);

            if (INTER_QUAD_PLN.IsDone)
            {
                switch (INTER_QUAD_PLN.TypeInter())
                {
                    case global::Kerf.Engine.Intersection.ResultType.Ellipse:
                    {
                        _typeres = global::Kerf.Engine.Intersection.ResultType.Ellipse;
                        Ellipse3D e = INTER_QUAD_PLN.Ellipse(1);
                        _pt1 = e.Location();
                        _dir1 = e.Position().Direction();
                        _dir2 = e.Position().XDirection();
                        _param1 = e.MajorRadius();
                        _param1bis = e.MinorRadius();
                        _nbint = 1;
                        break;
                    }
                    case global::Kerf.Engine.Intersection.ResultType.Circle:
                    {
                        _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
                        Circle3D c = INTER_QUAD_PLN.Circle(1);
                        _pt1 = c.Location();
                        _dir1 = c.Position().XDirection();
                        _dir2 = c.Position().YDirection();
                        _param1 = c.Radius();
                        _nbint = 1;
                        break;
                    }
                    case global::Kerf.Engine.Intersection.ResultType.Parabola:
                    {
                        _typeres = global::Kerf.Engine.Intersection.ResultType.Parabola;
                        Parabola3D prb = INTER_QUAD_PLN.Parabola(1);
                        _pt1 = prb.Location();
                        _dir1 = prb.Position().Direction();
                        _dir2 = prb.Position().XDirection();
                        _param1 = prb.Focal();
                        _nbint = 1;
                        break;
                    }
                    case global::Kerf.Engine.Intersection.ResultType.Hyperbola:
                    {
                        _typeres = global::Kerf.Engine.Intersection.ResultType.Hyperbola;
                        Hyperbola3D h = INTER_QUAD_PLN.Hyperbola(1);
                        _pt1 = _pt2 = h.Location();
                        _dir1 = h.Position().Direction();
                        _dir2 = h.Position().XDirection();
                        _param1 = _param2 = h.MajorRadius();
                        _param1bis = _param2bis = h.MinorRadius();
                        _nbint = 2;
                        break;
                    }
                    default:
                        _typeres = NoGeometricSolution;
                        break;
                }
            }
        }
        else
        {
            _typeres = NoGeometricSolution;
        }
    }

    /// <summary>The intersection between a sphere and a cone (only when the center is on the axis).</summary>
    public QuadQuadGeo(in Sphere sph, in Cone con, double tol)
        : this()
    {
        Perform(sph, con, tol);
    }

    /// <summary>Intersects a sphere and a cone (only when the center is on the axis). Tol is not used.</summary>
    public void Perform(in Sphere sph, in Cone con, double tol)
    {
        _ = tol;
        //
        _done = true;
        //
        AxeOperator a1A2 = new(con.Axis(), sph.Position().Axis());
        Point3D pt = sph.Location();
        //
        if ((a1A2.Intersect() && (pt.Distance(a1A2.PtIntersect()) == 0.0)) || a1A2.Same())
        {
            Point3D conApex = con.Apex();
            double dApexSphCenter = pt.Distance(conApex);
            Direction3D conDir;
            if (dApexSphCenter > RealEpsilon())
            {
                conDir = new Direction3D(new Vector3D(conApex, pt));
            }
            else
            {
                conDir = con.Position().Direction();
            }

            double rad = sph.Radius();
            double tga = Math.Tan(con.SemiAngle());

            //-- 2 circles
            //-- x: Roots of    (x**2 + y**2 = Rad**2)
            //--                tga = y / (x+dApexSphCenter)
            double tgatga = tga * tga;
            DirectPolynomialRoots eq = new(1.0 + tgatga,
                                                2.0 * tgatga * dApexSphCenter,
                                                -rad * rad + dApexSphCenter * dApexSphCenter * tgatga);
            if (eq.IsDone)
            {
                int nbsol = eq.SolutionCount;
                if (nbsol == 0)
                {
                    _typeres = Empty;
                }
                else
                {
                    _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
                    if (nbsol >= 1)
                    {
                        double x = eq.Value(1);
                        double dApexSphCenterpx = dApexSphCenter + x;
                        _nbint = 1;
                        _pt1 = _pt1.WithCoord(conApex.X() + (dApexSphCenterpx) * conDir.X(),
                                     conApex.Y() + (dApexSphCenterpx) * conDir.Y(),
                                     conApex.Z() + (dApexSphCenterpx) * conDir.Z());
                        _param1 = tga * dApexSphCenterpx;
                        _param1 = Math.Abs(_param1);
                        _dir1 = conDir;
                        if (_param1 <= _epsilonMiniCircleRadius)
                        {
                            _typeres = PointAndCircle;
                            _param1 = 0.0;
                        }
                    }
                    if (nbsol >= 2)
                    {
                        double x = eq.Value(2);
                        double dApexSphCenterpx = dApexSphCenter + x;
                        _nbint = 2;
                        _pt2 = _pt2.WithCoord(conApex.X() + (dApexSphCenterpx) * conDir.X(),
                                     conApex.Y() + (dApexSphCenterpx) * conDir.Y(),
                                     conApex.Z() + (dApexSphCenterpx) * conDir.Z());
                        _param2 = tga * dApexSphCenterpx;
                        _param2 = Math.Abs(_param2);
                        _dir2 = conDir;
                        if (_param2 <= _epsilonMiniCircleRadius)
                        {
                            _typeres = PointAndCircle;
                            _param2 = 0.0;
                        }
                    }
                }
            }
            else
            {
                _done = false;
            }
        }
        else
        {
            _typeres = NoGeometricSolution;
        }
    }

    /// <summary>The intersection between two spheres.</summary>
    public QuadQuadGeo(in Sphere sph1, in Sphere sph2, double tol)
        : this()
    {
        Perform(sph1, sph2, tol);
    }

    /// <summary>Intersects two spheres.</summary>
    public void Perform(in Sphere sph1, in Sphere sph2, double tol)
    {
        _done = true;
        Point3D o1 = sph1.Location();
        Point3D o2 = sph2.Location();
        double dO1O2 = o1.Distance(o2);
        double r1 = sph1.Radius();
        double r2 = sph2.Radius();
        double rmin, rmax;
        _typeres = Empty;
        _param2bis = 0.0; //-- to avoid param2bis unused warning

        if (r1 > r2)
        {
            rmin = r2;
            rmax = r1;
        }
        else
        {
            rmin = r1;
            rmax = r2;
        }

        if (dO1O2 <= tol && (Math.Abs(r1 - r2) <= tol))
        {
            _typeres = Same;
        }
        else
        {
            if (dO1O2 <= tol)
            {
                return;
            }
            Direction3D dir = new(new Vector3D(o1, o2));
            double t = rmax - dO1O2 - rmin;

            //----------------------------------------------------------------------
            //--        |----------------- R1 --------------------|
            //--        |----dO1O2-----|-----------R2----------|
            //--                                            --->--<-- t
            //--
            //--        |------ R1 ------|---------dO1O2----------|
            //--     |-------------------R2-----------------------|
            //--  --->--<-- t
            //----------------------------------------------------------------------
            if (t >= 0.0 && t <= tol)
            {
                _typeres = global::Kerf.Engine.Intersection.ResultType.Point;
                _nbint = 1;
                double t2;
                if (r1 == rmax)
                {
                    t2 = (r1 + (r2 + dO1O2)) * 0.5;
                }
                else
                {
                    t2 = (-r1 + (dO1O2 - r2)) * 0.5;
                }

                _pt1 = _pt1.WithCoord(o1.X() + t2 * dir.X(), o1.Y() + t2 * dir.Y(), o1.Z() + t2 * dir.Z());
            }
            else
            {
                //-----------------------------------------------------------------
                //--        |----------------- dO1O2 --------------------|
                //--        |----R1-----|-----------R2----------|-Tol-|
                //--
                //--        |----------------- Rmax --------------------|
                //--        |----Rmin----|-------dO1O2-------|-Tol-|
                //--
                //-----------------------------------------------------------------
                if ((dO1O2 > (r1 + r2 + tol)) || (rmax > (dO1O2 + rmin + tol)))
                {
                    _typeres = Empty;
                }
                else
                {
                    //---------------------------------------------------------------
                    //--
                    //--
                    //---------------------------------------------------------------
                    double alpha = 0.5 * (r1 * r1 - r2 * r2 + dO1O2 * dO1O2) / (dO1O2);
                    double beta = r1 * r1 - alpha * alpha;
                    beta = (beta > 0.0) ? Math.Sqrt(beta) : 0.0;

                    if (beta <= _epsilonMiniCircleRadius)
                    {
                        _typeres = global::Kerf.Engine.Intersection.ResultType.Point;
                        alpha = (r1 + (dO1O2 - r2)) * 0.5;
                    }
                    else
                    {
                        _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
                        _dir1 = dir;
                        _param1 = beta;
                    }
                    _pt1 = _pt1.WithCoord(o1.X() + alpha * dir.X(), o1.Y() + alpha * dir.Y(), o1.Z() + alpha * dir.Z());

                    _nbint = 1;
                }
            }
        }
    }

    /// <summary>The intersection between a plane and a torus.</summary>
    public QuadQuadGeo(in Plane3D pln, in Torus tor, double tol)
        : this()
    {
        Perform(pln, tor, tol);
    }

    /// <summary>Intersects a plane and a torus (only a plane normal to the axis, or containing it).</summary>
    public void Perform(in Plane3D pln, in Torus tor, double tol)
    {
        _done = true;
        //
        double rMin, rMaj;
        //
        rMin = tor.MinorRadius();
        rMaj = tor.MajorRadius();
        if (rMin >= rMaj)
        {
            _typeres = NoGeometricSolution;
            return;
        }
        //
        Axis3D plnAx = pln.Axis();
        Axis3D torAx = tor.Axis();
        //
        bool bParallel, bNormal;
        //
        bParallel = torAx.IsParallel(plnAx, _epsilonAxesPara);
        bNormal = !bParallel ? torAx.IsNormal(plnAx, _epsilonAxesPara) : false;
        if (!bNormal && !bParallel)
        {
            _typeres = NoGeometricSolution;
            return;
        }
        //
        double dist;
        //
        Point3D torLoc = torAx.Location();
        if (bParallel)
        {
            double dt, x, y, z, a, b, c, d, dr, tolNum;
            //
            tolNum = _epsilonCylinderDeltaRadius;
            //
            pln.Coefficients(out a, out b, out c, out d);
            torLoc.Coord(out x, out y, out z);
            dist = a * x + b * y + c * z + d;
            //
            dr = Math.Abs(dist) - rMin;
            if (dr > tolNum)
            {
                _typeres = Empty;
                return;
            }
            //
            if (Math.Abs(dr) < tolNum)
            {
                dist = (dist < 0) ? -rMin : rMin;
            }
            //
            _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
            //
            _pt1 = _pt1.WithCoord(x - dist * a, y - dist * b, z - dist * c);
            dt = Math.Sqrt(Math.Abs(rMin * rMin - dist * dist));
            _param1 = rMaj + dt;
            _dir1 = torAx.Direction();
            _nbint = 1;
            if ((dr < -tolNum) && (dt > tol))
            {
                _pt2 = _pt1;
                _param2 = rMaj - dt;
                _dir2 = _dir1;
                _nbint = 2;
            }
        }
        //
        else
        {
            dist = pln.Distance(torLoc);
            if (dist > _epsilonDistance)
            {
                _typeres = NoGeometricSolution;
                return;
            }
            //
            _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
            _param2 = _param1 = rMin;
            _dir2 = _dir1 = plnAx.Direction();
            _nbint = 2;
            //
            Direction3D dir = torAx.Direction() ^ _dir1;
            _pt1 = _pt1.WithXYZ(torLoc.XYZ() + rMaj * dir.XYZ());
            _pt2 = _pt2.WithXYZ(torLoc.XYZ() - rMaj * dir.XYZ());
        }
    }

    /// <summary>The intersection between a cylinder and a torus.</summary>
    public QuadQuadGeo(in Cylinder cyl, in Torus tor, double tol)
        : this()
    {
        Perform(cyl, tor, tol);
    }

    /// <summary>Intersects a cylinder and a torus (only when they are coaxial).</summary>
    public void Perform(in Cylinder cyl, in Torus tor, double tol)
    {
        _done = true;
        //
        double rMin, rMaj;
        //
        rMin = tor.MinorRadius();
        rMaj = tor.MajorRadius();
        if (rMin >= rMaj)
        {
            _typeres = NoGeometricSolution;
            return;
        }
        //
        Axis3D cylAx = cyl.Axis();
        Axis3D torAx = tor.Axis();
        //
        Line3D lin = new(torAx);
        Point3D locCyl = cyl.Location();
        //
        if (!torAx.IsParallel(cylAx, _epsilonAxesPara)
            || (lin.Distance(locCyl) > _epsilonDistance))
        {
            _typeres = NoGeometricSolution;
            return;
        }
        //
        double rCyl;
        //
        rCyl = cyl.Radius();
        if (((rCyl + tol) < (rMaj - rMin)) || ((rCyl - tol) > (rMaj + rMin)))
        {
            _typeres = Empty;
            return;
        }
        //
        _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
        //
        double dist = Math.Sqrt(Math.Abs(rMin * rMin - (rCyl - rMaj) * (rCyl - rMaj)));
        Xyz torLoc = torAx.Location().XYZ();
        //
        _dir1 = torAx.Direction();
        _pt1 = _pt1.WithXYZ(torLoc + dist * _dir1.XYZ());
        _param1 = rCyl;
        _nbint = 1;
        if ((dist > tol) && (rCyl > (rMaj - rMin)) && (rCyl < (rMaj + rMin)))
        {
            _dir2 = _dir1;
            _pt2 = _pt2.WithXYZ(torLoc - dist * _dir2.XYZ());
            _param2 = _param1;
            _nbint = 2;
        }
    }

    /// <summary>The intersection between a cone and a torus.</summary>
    public QuadQuadGeo(in Cone con, in Torus tor, double tol)
        : this()
    {
        Perform(con, tor, tol);
    }

    /// <summary>Intersects a cone and a torus (only when they are coaxial).</summary>
    public void Perform(in Cone con, in Torus tor, double tol)
    {
        _done = true;
        //
        double rMin, rMaj;
        //
        rMin = tor.MinorRadius();
        rMaj = tor.MajorRadius();
        if (rMin >= rMaj)
        {
            _typeres = NoGeometricSolution;
            return;
        }
        //
        Axis3D conAx = con.Axis();
        Axis3D torAx = tor.Axis();
        //
        Line3D lin = new(torAx);
        Point3D conApex = con.Apex();
        //
        if (!torAx.IsParallel(conAx, _epsilonAxesPara)
            || (lin.Distance(conApex) > _epsilonDistance))
        {
            _typeres = NoGeometricSolution;
            return;
        }
        //
        double angle, dist, dt;
        Span<double> param = stackalloc double[4];
        int i;
        Point3D torLoc, pct = new(), pn = new();
        Span<Point3D> pt = stackalloc Point3D[4];
        Span<Direction3D> dir = [new Direction3D(), new Direction3D(), new Direction3D(), new Direction3D()];
        //
        angle = con.SemiAngle();
        torLoc = torAx.Location();
        //
        pn = pn.WithXYZ(torLoc.XYZ() + rMaj * tor.YAxis().Direction().XYZ());
        Direction3D dn = new(new Vector3D(torLoc, pn));
        Axis3D axCLRot = new(conApex, dn);
        Line3D conL = lin.Rotated(axCLRot, angle);
        Direction3D dl = conL.Position().Direction();
        Direction3D xDir = tor.XAxis().Direction();
        //
        _typeres = Empty;
        //
        for (i = 0; i < 2; ++i)
        {
            if (i != 0)
            {
                xDir = xDir.Reversed();
            }
            pct = pct.WithXYZ(torLoc.XYZ() + rMaj * xDir.XYZ());
            //
            dist = conL.Distance(pct);
            if (dist > rMin + tol)
            {
                continue;
            }
            //
            _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
            //
            Xyz ph = pct.XYZ() - dist * conL.Normal(pct).Direction().XYZ();
            dt = Math.Sqrt(Math.Abs(rMin * rMin - dist * dist));
            //
            Point3D p = new();
            Xyz dVal = dt * dl.XYZ();
            p = p.WithXYZ(ph + dVal);
            param[_nbint] = lin.Distance(p);
            pt[_nbint] = pt[_nbint].WithXYZ(p.XYZ() - param[_nbint] * xDir.XYZ());
            dir[_nbint] = torAx.Direction();
            ++_nbint;
            if ((dist < rMin) && (dt > tol))
            {
                p = p.WithXYZ(ph - dVal);
                param[_nbint] = lin.Distance(p);
                pt[_nbint] = pt[_nbint].WithXYZ(p.XYZ() - param[_nbint] * xDir.XYZ());
                dir[_nbint] = dir[_nbint - 1];
                ++_nbint;
            }
        }
        //
        for (i = 0; i < _nbint; ++i)
        {
            switch (i)
            {
                case 0:
                {
                    _pt1 = pt[i];
                    _param1 = param[i];
                    _dir1 = dir[i];
                    break;
                }
                case 1:
                {
                    _pt2 = pt[i];
                    _param2 = param[i];
                    _dir2 = dir[i];
                    break;
                }
                case 2:
                {
                    _pt3 = pt[i];
                    _param3 = param[i];
                    _dir3 = dir[i];
                    break;
                }
                case 3:
                {
                    _pt4 = pt[i];
                    _param4 = param[i];
                    _dir4 = dir[i];
                    break;
                }
                default:
                    break;
            }
        }
    }

    /// <summary>The intersection between a sphere and a torus.</summary>
    public QuadQuadGeo(in Sphere sph, in Torus tor, double tol)
        : this()
    {
        Perform(sph, tor, tol);
    }

    /// <summary>Intersects a sphere and a torus (only when the center is on the axis).</summary>
    public void Perform(in Sphere sph, in Torus tor, double tol)
    {
        _done = true;
        //
        double rMin, rMaj;
        //
        rMin = tor.MinorRadius();
        rMaj = tor.MajorRadius();
        if (rMin >= rMaj)
        {
            _typeres = NoGeometricSolution;
            return;
        }
        //
        Axis3D torAx = tor.Axis();
        Line3D lin = new(torAx);
        Point3D sphLoc = sph.Location();
        //
        if (lin.Distance(sphLoc) > _epsilonDistance)
        {
            _typeres = NoGeometricSolution;
            return;
        }
        //
        double rSph, dist;
        Point3D torLoc = new();
        //
        Direction3D xDir = tor.XAxis().Direction();
        torLoc = torLoc.WithXYZ(torAx.Location().XYZ() + rMaj * xDir.XYZ());
        rSph = sph.Radius();
        //
        Vector3D vec12 = new(torLoc, sphLoc);
        dist = vec12.Magnitude();
        if (((dist - tol) > (rMin + rSph)) || ((dist + tol) < Math.Abs(rMin - rSph)))
        {
            _typeres = Empty;
            return;
        }
        //
        _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
        //
        double alpha, beta;
        //
        alpha = 0.5 * (rMin * rMin - rSph * rSph + dist * dist) / dist;
        beta = Math.Sqrt(Math.Abs(rMin * rMin - alpha * alpha));
        //
        Direction3D dir12 = new(vec12);
        Xyz ph = torLoc.XYZ() + alpha * dir12.XYZ();
        Direction3D dc = tor.YAxis().Direction() ^ dir12;
        //
        Point3D p = new();
        Xyz dVal = beta * dc.XYZ();
        p = p.WithXYZ(ph + dVal);
        _param1 = lin.Distance(p);
        _pt1 = _pt1.WithXYZ(p.XYZ() - _param1 * xDir.XYZ());
        _dir1 = torAx.Direction();
        _nbint = 1;
        if ((dist < (rSph + rMin)) && (dist > Math.Abs(rSph - rMin)) && (dVal.Modulus() > tol))
        {
            p = p.WithXYZ(ph - dVal);
            _param2 = lin.Distance(p);
            _pt2 = _pt2.WithXYZ(p.XYZ() - _param2 * xDir.XYZ());
            _dir2 = _dir1;
            _nbint = 2;
        }
    }

    /// <summary>The intersection between two tori.</summary>
    public QuadQuadGeo(in Torus tor1, in Torus tor2, double tol)
        : this()
    {
        Perform(tor1, tor2, tol);
    }

    /// <summary>Intersects two tori (only when they are coaxial).</summary>
    public void Perform(in Torus tor1, in Torus tor2, double tol)
    {
        _done = true;
        //
        double rMin1, rMin2, rMaj1, rMaj2;
        //
        rMin1 = tor1.MinorRadius();
        rMaj1 = tor1.MajorRadius();
        rMin2 = tor2.MinorRadius();
        rMaj2 = tor2.MajorRadius();
        //
        Axis3D ax1 = tor1.Axis();
        Axis3D ax2 = tor2.Axis();
        //
        Point3D loc1 = ax1.Location();
        Point3D loc2 = ax2.Location();
        //
        Line3D l1 = new(ax1);
        if (!ax1.IsParallel(ax2, _epsilonAxesPara) || (l1.Distance(loc2) > _epsilonDistance))
        {
            _typeres = NoGeometricSolution;
            return;
        }
        //
        if (loc1.IsEqual(loc2, tol) && (Math.Abs(rMin1 - rMin2) <= tol)
            && (Math.Abs(rMaj1 - rMaj2) <= tol))
        {
            _typeres = Same;
            return;
        }
        //
        if (rMin1 >= rMaj1 || rMin2 >= rMaj2)
        {
            _typeres = NoGeometricSolution;
            return;
        }
        //
        double dist;
        Point3D p1 = new(), p2 = new();
        //
        Direction3D xDir1 = tor1.XAxis().Direction();
        p1 = p1.WithXYZ(loc1.XYZ() + rMaj1 * xDir1.XYZ());
        p2 = p2.WithXYZ(loc2.XYZ() + rMaj2 * xDir1.XYZ());
        //
        Vector3D v12 = new(p1, p2);
        dist = v12.Magnitude();
        if (((dist - tol) > (rMin1 + rMin2)) || ((dist + tol) < Math.Abs(rMin1 - rMin2)))
        {
            _typeres = Empty;
            return;
        }
        //
        _typeres = global::Kerf.Engine.Intersection.ResultType.Circle;
        //
        double alpha, beta;
        //
        alpha = 0.5 * (rMin1 * rMin1 - rMin2 * rMin2 + dist * dist) / dist;
        beta = Math.Sqrt(Math.Abs(rMin1 * rMin1 - alpha * alpha));
        //
        Direction3D dir12 = new(v12);
        Xyz ph = p1.XYZ() + alpha * dir12.XYZ();
        Direction3D dc = tor1.YAxis().Direction() ^ dir12;
        //
        Point3D p = new();
        Xyz dVal = beta * dc.XYZ();
        p = p.WithXYZ(ph + dVal);
        _param1 = l1.Distance(p);
        _pt1 = _pt1.WithXYZ(p.XYZ() - _param1 * xDir1.XYZ());
        _dir1 = ax1.Direction();
        _nbint = 1;
        if ((dist < (rMin1 + rMin2)) && (dist > Math.Abs(rMin1 - rMin2)) && dVal.Modulus() > tol)
        {
            p = p.WithXYZ(ph - dVal);
            _param2 = l1.Distance(p);
            _pt2 = _pt2.WithXYZ(p.XYZ() - _param2 * xDir1.XYZ());
            _dir2 = _dir1;
            _nbint = 2;
        }
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone => _done;

    /// <summary>The type of intersection.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public ResultType TypeInter()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _typeres;
    }

    /// <summary>The number of solutions.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public int SolutionCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _nbint;
        }
    }

    /// <summary>The point solution of range n (for a Point or PointAndCircle result).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">n is out of range.</exception>
    public Point3D Point(int n)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (n > _nbint || n < 1)
        {
            throw new DomainErrorException();
        }
        if (_typeres == PointAndCircle)
        {
            if (n != 1)
            {
                throw new DomainErrorException();
            }
            if (_param1 == 0.0)
            {
                return (_pt1);
            }
            return (_pt2);
        }
        else if (_typeres == global::Kerf.Engine.Intersection.ResultType.Point)
        {
            if (n == 1)
            {
                return (_pt1);
            }
            return (_pt2);
        }

        // WNT (what can you expect from MicroSoft ?)
        return new Point3D(0, 0, 0);
    }

    /// <summary>The line solution of range n.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">n is out of range, or the result is not lines.</exception>
    public Line3D Line(int n)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if ((n > _nbint) || (n < 1) || (_typeres != global::Kerf.Engine.Intersection.ResultType.Line))
        {
            throw new DomainErrorException();
        }
        if (n == 1)
        {
            return (new Line3D(_pt1, _dir1));
        }
        else
        {
            return (new Line3D(_pt2, _dir2));
        }
    }

    /// <summary>The circle solution of range n.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">n is out of range, or the result is not circles.</exception>
    public Circle3D Circle(int n)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_typeres == PointAndCircle)
        {
            if (n != 1)
            {
                throw new DomainErrorException();
            }
            if (_param2 == 0.0)
            {
                return (new Circle3D(DirToAx2(_pt1, _dir1), _param1));
            }
            return (new Circle3D(DirToAx2(_pt2, _dir2), _param2));
        }
        else if ((n > _nbint) || (n < 1) || (_typeres != global::Kerf.Engine.Intersection.ResultType.Circle))
        {
            throw new DomainErrorException();
        }
        if (n == 1)
        {
            return (new Circle3D(DirToAx2(_pt1, _dir1), _param1));
        }
        else if (n == 2)
        {
            return (new Circle3D(DirToAx2(_pt2, _dir2), _param2));
        }
        else if (n == 3)
        {
            return (new Circle3D(DirToAx2(_pt3, _dir3), _param3));
        }
        else
        {
            return (new Circle3D(DirToAx2(_pt4, _dir4), _param4));
        }
    }

    /// <summary>The ellipse solution of range n.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">n is out of range, or the result is not ellipses.</exception>
    public Ellipse3D Ellipse(int n)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if ((n > _nbint) || (n < 1) || (_typeres != global::Kerf.Engine.Intersection.ResultType.Ellipse))
        {
            throw new DomainErrorException();
        }

        if (n == 1)
        {
            double r1 = _param1, r2 = _param1bis, tmp;
            if (r1 < r2)
            {
                tmp = r1;
                r1 = r2;
                r2 = tmp;
            }
            Frame3D ax2 = new(_pt1, _dir1, _dir2);
            Ellipse3D elips = new(ax2, r1, r2);
            return elips;
        }
        else
        {
            double r1 = _param2, r2 = _param2bis, tmp;
            if (r1 < r2)
            {
                tmp = r1;
                r1 = r2;
                r2 = tmp;
            }
            Frame3D ax2 = new(_pt2, _dir2, _dir1);
            Ellipse3D elips = new(ax2, r1, r2);
            return elips;
        }
    }

    /// <summary>The parabola solution (n must be 1).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">The result is not a parabola.</exception>
    /// <exception cref="OutOfRangeException">n is not 1.</exception>
    public Parabola3D Parabola(int n)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_typeres != global::Kerf.Engine.Intersection.ResultType.Parabola)
        {
            throw new DomainErrorException();
        }
        if ((n > _nbint) || (n != 1))
        {
            throw new OutOfRangeException();
        }
        return (new Parabola3D(new Frame3D(_pt1, _dir1, _dir2), _param1));
    }

    /// <summary>The hyperbola solution of range n.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">n is out of range, or the result is not hyperbolas.</exception>
    public Hyperbola3D Hyperbola(int n)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if ((n > _nbint) || (n < 1) || (_typeres != global::Kerf.Engine.Intersection.ResultType.Hyperbola))
        {
            throw new DomainErrorException();
        }
        if (n == 1)
        {
            return (new Hyperbola3D(new Frame3D(_pt1, _dir1, _dir2), _param1, _param1bis));
        }
        else
        {
            return (new Hyperbola3D(new Frame3D(_pt2, _dir1, _dir2.Reversed()), _param2, _param2bis));
        }
    }

    /// <summary>True if the two cones have a common generatrix (set by the cone/cone intersection).</summary>
    public bool HasCommonGen => _commonGen;

    /// <summary>The characteristic point of the intersection of two cones with a common generatrix.</summary>
    public Point3D PChar() => _pChar;

    //=================================================================================================

    private static void RefineDir(ref Direction3D aDir)
    {
        int k, m, n;
        Span<double> c = stackalloc double[3];
        //
        aDir.Coord(out c[0], out c[1], out c[2]);
        //
        m = 0;
        n = 0;
        for (k = 0; k < 3; ++k)
        {
            if (c[k] == 1.0 || c[k] == -1.0)
            {
                ++m;
            }
            else if (c[k] != 0.0)
            {
                ++n;
            }
        }
        //
        if (m != 0 && n != 0)
        {
            double eps, r1, r2, num;
            //
            eps = RealEpsilon();
            r1 = 1.0 - eps;
            r2 = 1.0 + eps;
            //
            for (k = 0; k < 3; ++k)
            {
                m = (c[k] > 0.0) ? 1 : 0;
                num = (m != 0) ? c[k] : -c[k];
                if (num > r1 && num < r2)
                {
                    if (m != 0)
                    {
                        c[k] = 1.0;
                    }
                    else
                    {
                        c[k] = -1.0;
                    }
                    //
                    c[(k + 1) % 3] = 0.0;
                    c[(k + 2) % 3] = 0.0;
                    break;
                }
            }
            aDir = aDir.WithCoord(c[0], c[1], c[2]);
        }
    }
}
