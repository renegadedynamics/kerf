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

using static Occt.IntAna_ResultType;

namespace Occt;

/// <summary>
/// Geometric intersections between two natural quadrics (planes, cylinders, cones, spheres, and some tori):
/// 1 point, 1 or 2 lines, 1 point and 1 line, 1 or 2 circles, 1 or 2 ellipses, 1 parabola, 1 or 2 hyperbolas;
/// Empty (no intersection), Same (identical quadrics), or NoGeometricSolution (there may be an intersection, but an
/// analytic algorithm, <c>IntAna_IntQuadQuad</c>, must compute it).
/// </summary>
/// <remarks>An algorithm object: construct it with the quadrics, or empty and call <c>Perform</c>.</remarks>
public class IntAna_QuadQuadGeo
{
    private bool done;
    private int nbint;
    private IntAna_ResultType typeres;
    private gp_Pnt pt1;
    private gp_Pnt pt2;
    private gp_Pnt pt3;
    private gp_Pnt pt4;
    private gp_Dir dir1 = new();
    private gp_Dir dir2 = new();
    private gp_Dir dir3 = new();
    private gp_Dir dir4 = new();
    private double param1;
    private double param2;
    private double param3;
    private double param4;
    private double param1bis;
    private double param2bis;
    private double myEPSILON_DISTANCE;
    private double myEPSILON_ANGLE_CONE;
    private double myEPSILON_MINI_CIRCLE_RADIUS;
    private double myEPSILON_CYLINDER_DELTA_RADIUS;
    private double myEPSILON_CYLINDER_DELTA_DISTANCE;
    private double myEPSILON_AXES_PARA;
    private bool myCommonGen;
    private gp_Pnt myPChar;

    //=======================================================================
    // class :  AxeOperator
    // purpose  : V a r i o u s   O p e r a t i o n s   o n   A x 1
    //=======================================================================
    private sealed class AxeOperator
    {
        private gp_Pnt ptintersect;
        private readonly gp_Ax1 Axe1;
        private readonly gp_Ax1 Axe2;
        private readonly double thedistance;
        private readonly bool theparallel;
        private readonly bool thecoplanar;
        private readonly bool thenormal;
        //
        private readonly double myEPSILON_DISTANCE;
        private readonly double myEPSILON_AXES_PARA;

        public AxeOperator(in gp_Ax1 A1, in gp_Ax1 A2)
            : this(A1, A2, 1.0e-14, Precision.Angular())
        {
        }

        public AxeOperator(in gp_Ax1 A1, in gp_Ax1 A2, double theEpsDistance, double theEpsAxesPara)
        {
            Axe1 = A1;
            Axe2 = A2;
            myEPSILON_DISTANCE = theEpsDistance;
            myEPSILON_AXES_PARA = theEpsAxesPara;
            //---------------------------------------------------------------------
            gp_Dir V1 = Axe1.Direction();
            gp_Dir V2 = Axe2.Direction();
            gp_Pnt P1 = Axe1.Location();
            gp_Pnt P2 = Axe2.Location();
            //
            RefineDir(ref V1);
            RefineDir(ref V2);
            thecoplanar = false;
            thenormal = false;

            //--- check if the two axis are parallel
            theparallel = V1.IsParallel(V2, myEPSILON_AXES_PARA);
            //--- Distance between the two axis
            gp_XYZ perp = A1.Direction().XYZ().Crossed(A2.Direction().XYZ());
            if (theparallel)
            {
                gp_Lin L1 = new(A1);
                thedistance = L1.Distance(A2.Location());
            }
            else
            {
                thedistance = Math.Abs(new gp_Vec(perp.Normalized()).Dot(new gp_Vec(Axe1.Location(), Axe2.Location())));
            }
            //--- check if Axis are Coplanar
            double D33;
            if (thedistance < myEPSILON_DISTANCE)
            {
                D33 = Det33(V1.X(),
                            V1.Y(),
                            V1.Z(),
                            V2.X(),
                            V2.Y(),
                            V2.Z(),
                            P1.X() - P2.X(),
                            P1.Y() - P2.Y(),
                            P1.Z() - P2.Z());
                if (Math.Abs(D33) <= myEPSILON_DISTANCE)
                {
                    thecoplanar = true;
                }
            }

            thenormal = Math.Abs(V1.Dot(V2)) < myEPSILON_AXES_PARA;

            //--- check if the two axis are concurrent
            if (thecoplanar && !theparallel)
            {
                double smx = P2.X() - P1.X();
                double smy = P2.Y() - P1.Y();
                double smz = P2.Z() - P1.Z();
                double Det1, Det2, Det3, A;
                Det1 = V1.Y() * V2.X() - V1.X() * V2.Y();
                Det2 = V1.Z() * V2.Y() - V1.Y() * V2.Z();
                Det3 = V1.Z() * V2.X() - V1.X() * V2.Z();

                if ((Det1 != 0.0) && ((Math.Abs(Det1) >= Math.Abs(Det2)) && (Math.Abs(Det1) >= Math.Abs(Det3))))
                {
                    A = (smy * V2.X() - smx * V2.Y()) / Det1;
                }
                else if ((Det2 != 0.0)
                         && ((Math.Abs(Det2) >= Math.Abs(Det1)) && (Math.Abs(Det2) >= Math.Abs(Det3))))
                {
                    A = (smz * V2.Y() - smy * V2.Z()) / Det2;
                }
                else
                {
                    A = (smz * V2.X() - smx * V2.Z()) / Det3;
                }
                ptintersect.SetCoord(P1.X() + A * V1.X(), P1.Y() + A * V1.Y(), P1.Z() + A * V1.Z());
            }
            else
            {
                ptintersect.SetCoord(0, 0, 0); //-- To avoid FPE
            }
        }

        public void Distance(out double dist, out double Param1, out double Param2)
        {
            // OCCT: leaves the outputs unset when the axes are parallel (D == 0); 0 here
            dist = 0.0;
            Param1 = 0.0;
            Param2 = 0.0;
            gp_Vec O1O2 = new(Axe1.Location(), Axe2.Location());
            gp_Dir U1 = Axe1.Direction(); //-- just to see.
            gp_Dir U2 = Axe2.Direction();

            gp_Dir N = U1.Crossed(U2);
            double D = Det33(U1.X(), U2.X(), N.X(), U1.Y(), U2.Y(), N.Y(), U1.Z(), U2.Z(), N.Z());
            if (D != 0.0)
            {
                dist = Det33(U1.X(), U2.X(), O1O2.X(), U1.Y(), U2.Y(), O1O2.Y(), U1.Z(), U2.Z(), O1O2.Z()) / D;
                Param1 =
                    Det33(O1O2.X(), U2.X(), N.X(), O1O2.Y(), U2.Y(), N.Y(), O1O2.Z(), U2.Z(), N.Z()) / (-D);
                //------------------------------------------------------------
                //-- We solve P1 * Dir1 + P2 * Dir2 + d * N = O1O2
                //-- i.e.: Perpendicular segment: O1+P1 D1
                //--                               O2-P2 D2
                Param2 = Det33(U1.X(), O1O2.X(), N.X(), U1.Y(), O1O2.Y(), N.Y(), U1.Z(), O1O2.Z(), N.Z()) / (D);
            }
        }

        public gp_Pnt PtIntersect() => ptintersect;

        public bool Coplanar() => thecoplanar;

        public bool Same() => theparallel && (thedistance < myEPSILON_DISTANCE);

        public double Distance() => thedistance;

        public bool Intersect() => (thecoplanar && (!theparallel));

        public bool Parallel() => theparallel;

        public bool Normal() => thenormal;

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
    private static gp_Ax2 DirToAx2(in gp_Pnt P, in gp_Dir D)
    {
        double x = D.X();
        double ax = Math.Abs(x);
        double y = D.Y();
        double ay = Math.Abs(y);
        double z = D.Z();
        double az = Math.Abs(z);
        if ((ax == 0.0) || ((ax < ay) && (ax < az)))
        {
            return (new gp_Ax2(P, D, new gp_Dir(new gp_Vec(0.0, -z, y))));
        }
        else if ((ay == 0.0) || ((ay < ax) && (ay < az)))
        {
            return (new gp_Ax2(P, D, new gp_Dir(new gp_Vec(-z, 0.0, x))));
        }
        else
        {
            return (new gp_Ax2(P, D, new gp_Dir(new gp_Vec(-y, x, 0.0))));
        }
    }

    //=======================================================================
    // function : EstimDist
    // purpose  : returns a minimal distance from apex to any solution
    //=======================================================================
    private static double EstimDist(in gp_Cone theCon1, in gp_Cone theCon2)
    {
        // It is supposed that axes of cones are coplanar and
        // distance between them > Precision::Confusion()
        gp_Pnt aPA1 = theCon1.Apex(), aPA2 = theCon2.Apex();

        gp_Pnt aP3 = aPA1.Translated(new gp_Vec(theCon1.Position().Direction()));

        gce_MakePln aMkPln = new(aPA1, aPA2, aP3);
        if (!aMkPln.IsDone())
        {
            return Precision.Infinite();
        }

        gp_Pln aPln = aMkPln.Value();

        gp_Lin anAx1 = new(aPA1, theCon1.Position().Direction());
        gp_Lin2d anAx12d = ProjLib.Project(aPln, anAx1);
        Span<gp_Lin2d> Lines1 = [new gp_Lin2d(), new gp_Lin2d()];
        double anAng1 = theCon1.SemiAngle();
        Lines1[0] = anAx12d.Rotated(anAx12d.Location(), anAng1);
        Lines1[1] = anAx12d.Rotated(anAx12d.Location(), -anAng1);
        //
        gp_Lin anAx2 = new(aPA2, theCon2.Position().Direction());
        gp_Lin2d anAx22d = ProjLib.Project(aPln, anAx2);
        Span<gp_Lin2d> Lines2 = [new gp_Lin2d(), new gp_Lin2d()];
        double anAng2 = theCon2.SemiAngle();
        Lines2[0] = anAx22d.Rotated(anAx22d.Location(), anAng2);
        Lines2[1] = anAx22d.Rotated(anAx22d.Location(), -anAng2);

        Span<double> aMinDist = [Precision.Infinite(), Precision.Infinite()];
        int i, j, k;
        IntAna2d_AnaIntersection anInter = new();
        for (i = 0; i < 2; ++i)
        {
            for (j = 0; j < 2; ++j)
            {
                anInter.Perform(Lines1[i], Lines2[j]);
                if (anInter.IsDone())
                {
                    int aNbPoints = anInter.NbPoints();
                    for (k = 1; k <= aNbPoints; ++k)
                    {
                        IntAna2d_IntPoint anIntP = anInter.Point(k);
                        double aPar1 = Math.Abs(anIntP.ParamOnFirst());
                        aMinDist[0] = Min(aPar1, aMinDist[0]);
                        double aPar2 = Math.Abs(anIntP.ParamOnSecond());
                        aMinDist[1] = Min(aPar2, aMinDist[1]);
                    }
                }
            }
        }

        double aDist = Max(aMinDist[0], aMinDist[1]);
        return aDist;
    }

    /// <summary>Empty constructor.</summary>
    public IntAna_QuadQuadGeo()
    {
        done = false;
        nbint = 0;
        typeres = IntAna_Empty;
        pt1 = new gp_Pnt(0, 0, 0);
        pt2 = new gp_Pnt(0, 0, 0);
        pt3 = new gp_Pnt(0, 0, 0);
        pt4 = new gp_Pnt(0, 0, 0);
        param1 = 0;
        param2 = 0;
        param3 = 0;
        param4 = 0;
        param1bis = 0;
        param2bis = 0;
        myCommonGen = false;
        myPChar = new gp_Pnt(0, 0, 0);
        InitTolerances();
    }

    /// <summary>Initializes the values of inner tolerances.</summary>
    protected void InitTolerances()
    {
        myEPSILON_DISTANCE = 1.0e-14;
        myEPSILON_ANGLE_CONE = Precision.Angular();
        myEPSILON_MINI_CIRCLE_RADIUS = 0.01 * Precision.Confusion();
        myEPSILON_CYLINDER_DELTA_RADIUS = 1.0e-13;
        myEPSILON_CYLINDER_DELTA_DISTANCE = Precision.Confusion();
        myEPSILON_AXES_PARA = Precision.Angular();
    }

    /// <summary>
    /// The intersection between two planes. TolAng is the angular tolerance below which the planes are parallel;
    /// Tol the distance below which parallel planes are identical.
    /// </summary>
    public IntAna_QuadQuadGeo(in gp_Pln P1, in gp_Pln P2, double TolAng, double Tol)
        : this()
    {
        Perform(P1, P2, TolAng, Tol);
    }

    /// <summary>
    /// Intersects two planes. TolAng is the angular tolerance below which the planes are parallel; Tol the distance
    /// below which parallel planes are identical.
    /// </summary>
    public void Perform(in gp_Pln P1, in gp_Pln P2, double TolAng, double Tol)
    {
        double A1, B1, C1, D1, A2, B2, C2, D2, dist1, dist2, aMVD;
        //
        done = false;
        param2bis = 0.0;
        //
        P1.Coefficients(out A1, out B1, out C1, out D1);
        P2.Coefficients(out A2, out B2, out C2, out D2);
        //
        gp_Vec aVN1 = new(A1, B1, C1);
        gp_Vec aVN2 = new(A2, B2, C2);
        gp_Vec vd = aVN1.Crossed(aVN2);
        //
        gp_Pnt aLocP1 = P1.Location();
        gp_Pnt aLocP2 = P2.Location();
        //
        dist1 = A2 * aLocP1.X() + B2 * aLocP1.Y() + C2 * aLocP1.Z() + D2;
        dist2 = A1 * aLocP2.X() + B1 * aLocP2.Y() + C1 * aLocP2.Z() + D1;
        //
        aMVD = vd.Magnitude();
        if (aMVD <= TolAng)
        {
            // normalles are collinear - planes are same or parallel
            typeres = (Math.Abs(dist1) <= Tol && Math.Abs(dist2) <= Tol) ? IntAna_Same : IntAna_Empty;
        }
        else
        {
            double denom, denom2, ddenom, par1, par2;
            double X1, Y1, Z1, X2, Y2, Z2, aEps;
            //
            aEps = 1.0e-16;
            denom = A1 * A2 + B1 * B2 + C1 * C2;
            denom2 = denom * denom;
            ddenom = 1.0 - denom2;

            denom = (Math.Abs(ddenom) <= aEps) ? aEps : ddenom;

            par1 = dist1 / denom;
            par2 = -dist2 / denom;

            gp_Vec inter1 = aVN1.Crossed(vd);
            gp_Vec inter2 = aVN2.Crossed(vd);

            X1 = aLocP1.X() + par1 * inter1.X();
            Y1 = aLocP1.Y() + par1 * inter1.Y();
            Z1 = aLocP1.Z() + par1 * inter1.Z();
            X2 = aLocP2.X() + par2 * inter2.X();
            Y2 = aLocP2.Y() + par2 * inter2.Y();
            Z2 = aLocP2.Z() + par2 * inter2.Z();

            pt1 = new gp_Pnt((X1 + X2) * 0.5, (Y1 + Y2) * 0.5, (Z1 + Z2) * 0.5);
            dir1 = new gp_Dir(vd);
            typeres = IntAna_Line;
            nbint = 1;
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
            double aTreshAng, aTreshDist;
            //
            aTreshAng = 2.0e-6; // 1.e-4 deg
            aTreshDist = 1.0e-12;
            //
            if (aMVD < aTreshAng)
            {
                double aDist1, aDist2;
                //
                aDist1 = A1 * pt1.X() + B1 * pt1.Y() + C1 * pt1.Z() + D1;
                aDist2 = A2 * pt1.X() + B2 * pt1.Y() + C2 * pt1.Z() + D2;
                //
                if (Math.Abs(aDist1) > aTreshDist || Math.Abs(aDist2) > aTreshDist)
                {
                    bool bIsDone, bIsParallel;
                    IntAna_IntConicQuad aICQ = new();
                    //
                    // 1.
                    gp_Dir aDN1 = new(aVN1);
                    gp_Lin aL1 = new(pt1, aDN1);
                    //
                    aICQ.Perform(aL1, P1, TolAng, Tol);
                    bIsDone = aICQ.IsDone();
                    if (!bIsDone)
                    {
                        return;
                    }
                    //
                    gp_Pnt aPnt1 = aICQ.Point(1);
                    //----------------------------------
                    // 2.
                    gp_Dir aDL2 = dir1.Crossed(aDN1);
                    gp_Lin aL2 = new(aPnt1, aDL2);
                    //
                    aICQ.Perform(aL2, P2, TolAng, Tol);
                    bIsDone = aICQ.IsDone();
                    if (!bIsDone)
                    {
                        return;
                    }
                    //
                    bIsParallel = aICQ.IsParallel();
                    if (bIsParallel)
                    {
                        return;
                    }
                    //
                    gp_Pnt aPnt2 = aICQ.Point(1);
                    //
                    pt1 = aPnt2;
                }
            }
        }
        done = true;
    }

    /// <summary>
    /// The intersection between a plane and a cylinder. Tolang is the angular tolerance below which the axis is
    /// parallel to the plane. Tol decides between a circle and an ellipse (the circle when the ellipse is within Tol
    /// of it). H, the height of the cylinder, helps decide whether the plane and the cylinder are parallel.
    /// </summary>
    public IntAna_QuadQuadGeo(in gp_Pln P, in gp_Cylinder Cl, double Tolang, double Tol, double H = 0)
        : this()
    {
        Perform(P, Cl, Tolang, Tol, H);
    }

    /// <summary>
    /// Intersects a plane and a cylinder. Tolang is the angular tolerance below which the axis is parallel to the
    /// plane. Tol decides between a circle and an ellipse. H, the height of the cylinder, helps decide whether the
    /// plane and the cylinder are parallel.
    /// </summary>
    public void Perform(in gp_Pln P, in gp_Cylinder Cl, double Tolang, double Tol, double H = 0)
    {
        done = false;
        double dist, radius;
        double A, B, C, D;
        double X, Y, Z;
        double sint, cost, h;
        gp_XYZ axex, axey, omega = new();

        param2bis = 0.0;
        radius = Cl.Radius();

        gp_Lin axec = new(Cl.Axis());
        gp_XYZ normp = P.Axis().Direction().XYZ();

        P.Coefficients(out A, out B, out C, out D);
        axec.Location().Coord(out X, out Y, out Z);
        // The axis/plane distance is evaluated at the origin of the axis.
        dist = A * X + B * Y + C * Z + D;

        double tolang = Tolang;
        double toltang = Tol;
        bool newparams = false;

        gp_Vec ldv = new(axec.Direction());
        gp_Vec npv = new(normp);
        double dA = Math.Abs(ldv.Angle(npv));
        if (dA > (M_PI / 4.0))
        {
            double dang = Math.Abs(ldv.Angle(npv)) - M_PI / 2.0;
            double dangle = Math.Abs(dang);
            if (dangle > Tolang)
            {
                double sinda = Math.Abs(Math.Sin(dangle));
                double dif = Math.Abs(sinda - Tol);
                // When the cylinder face height H is known and small, use the deviation
                // of the axis over this height (sinda * H) as an additional criterion.
                // If the axis deviates from the plane by less than a small multiple of
                // the tolerance over the full height of the face, the axis is effectively
                // parallel for this face, and the Line intersection type should be used
                // instead of the Ellipse (which degenerates for nearly-parallel cases).
                if (dif < Tol || (H > 0.0 && sinda * H < 2.0 * Tol))
                {
                    tolang = sinda * 2.0;
                    // Relax the linear tolerance so that the endpoint distance check
                    // in IntAna_IntConicQuad (P.Distance(aP2) > Tol) does not reject
                    // the parallel status when the deviation over H is small.
                    toltang = Max(Tol, sinda * H * 1.01);
                    newparams = true;
                }
            }
        }

        nbint = 0;
        IntAna_IntConicQuad inter = new(axec, P, tolang, toltang, H);

        if (inter.IsParallel())
        {
            // The result of the Plane-Cylinder intersection is of line type.
            // There are 1 or 2 lines

            typeres = IntAna_Line;
            omega.SetCoord(X - dist * A, Y - dist * B, Z - dist * C);

            if (Math.Abs(Math.Abs(dist) - radius) < Tol)
            {
                nbint = 1;
                pt1.SetXYZ(omega);

                if (newparams)
                {
                    gp_XYZ omegaXYZ = new(X, Y, Z);
                    gp_XYZ omegaXYZtrnsl = omegaXYZ + 100.0 * axec.Direction().XYZ();
                    double Xt, Yt, Zt, distt;
                    omegaXYZtrnsl.Coord(out Xt, out Yt, out Zt);
                    distt = A * Xt + B * Yt + C * Zt + D;
                    gp_XYZ omega1 = new(omegaXYZtrnsl.X() - distt * A,
                                        omegaXYZtrnsl.Y() - distt * B,
                                        omegaXYZtrnsl.Z() - distt * C);
                    gp_Pnt ppt1 = new();
                    ppt1.SetXYZ(omega1);
                    gp_Vec vv1 = new(pt1, ppt1);
                    gp_Dir dd1 = new(vv1);
                    dir1 = dd1;
                }
                else
                {
                    dir1 = axec.Direction();
                }
            }
            else if (Math.Abs(dist) < radius)
            {
                nbint = 2;
                h = Math.Sqrt(radius * radius - dist * dist);
                axey = axec.Direction().XYZ().Crossed(normp); // axey is normalized

                pt1.SetXYZ(omega - h * axey);
                pt2.SetXYZ(omega + h * axey);

                if (newparams)
                {
                    gp_XYZ omegaXYZ = new(X, Y, Z);
                    gp_XYZ omegaXYZtrnsl = omegaXYZ + 100.0 * axec.Direction().XYZ();
                    double Xt, Yt, Zt, distt, ht;
                    omegaXYZtrnsl.Coord(out Xt, out Yt, out Zt);
                    distt = A * Xt + B * Yt + C * Zt + D;
                    double anSqrtArg = radius * radius - distt * distt;
                    ht = (anSqrtArg > 0.0) ? Math.Sqrt(anSqrtArg) : 0.0;

                    gp_XYZ omega1 = new(omegaXYZtrnsl.X() - distt * A,
                                        omegaXYZtrnsl.Y() - distt * B,
                                        omegaXYZtrnsl.Z() - distt * C);
                    gp_Pnt ppt1 = new(), ppt2 = new();
                    ppt1.SetXYZ(omega1 - ht * axey);
                    ppt2.SetXYZ(omega1 + ht * axey);
                    gp_Vec vv1 = new(pt1, ppt1);
                    gp_Vec vv2 = new(pt2, ppt2);
                    gp_Dir dd1 = new(vv1);
                    gp_Dir dd2 = new(vv2);
                    dir1 = dd1;
                    dir2 = dd2;
                }
                else
                {
                    dir1 = axec.Direction();
                    dir2 = axec.Direction();
                }
            }
            //  else nbint = 0

            // debug JAG : nbint = 0 should be replaced by typeres = IntAna_Empty
            // and not just be removed...

            else
            {
                typeres = IntAna_Empty;
            }
        }
        else
        { // There is one intersection point. It is the center of the circle
          // or of the solution ellipse.

            nbint = 1;
            axey = normp.Crossed(axec.Direction().XYZ());
            sint = axey.Modulus();

            pt1 = inter.Point(1);

            if (sint < Tol / radius)
            {
                // Construct a circle with the X and Y axes of the cylinder
                typeres = IntAna_Circle;

                dir1 = axec.Direction(); // Z axis
                dir2 = Cl.Position().XDirection();
                param1 = radius;
            }
            else
            {
                // construct an ellipse
                typeres = IntAna_Ellipse;
                cost = Math.Abs(axec.Direction().XYZ().Dot(normp));
                axex = axey.Crossed(normp);

                dir1.SetXYZ(normp); // Modified in this block
                dir2.SetXYZ(axex);

                param1 = radius / cost;
                param1bis = radius;
            }
        }

        done = true;
    }

    /// <summary>
    /// The intersection between a plane and a cone. Tolang is the angular tolerance used to decide whether the
    /// axis is parallel or perpendicular to the plane and whether a generatrix is parallel to it; Tol the distance
    /// below which the apex is in the plane.
    /// </summary>
    public IntAna_QuadQuadGeo(in gp_Pln P, in gp_Cone Co, double Tolang, double Tol)
        : this()
    {
        Perform(P, Co, Tolang, Tol);
    }

    /// <summary>Intersects a plane and a cone (see the constructor for the tolerances).</summary>
    public void Perform(in gp_Pln P, in gp_Cone Co, double Tolang, double Tol)
    {
        done = false;
        nbint = 0;

        double A, B, C, D;
        double X, Y, Z;
        double dist, sint, cost, sina, cosa, angl, costa;
        double dh;
        gp_XYZ axex, axey;

        gp_Lin axec = new(Co.Axis());
        P.Coefficients(out A, out B, out C, out D);
        gp_Pnt apex = Co.Apex();

        apex.Coord(out X, out Y, out Z);
        dist = A * X + B * Y + C * Z + D; // signed distance from cone apex to plane

        gp_XYZ normp = P.Axis().Direction().XYZ();
        if (!P.Direct())
        { //-- lbr 14 Jan 97
            normp.Reverse();
        }

        axey = normp.Crossed(Co.Axis().Direction().XYZ());
        axex = axey.Crossed(normp);

        angl = Co.SemiAngle();

        cosa = Math.Cos(angl);
        sina = Math.Abs(Math.Sin(angl));

        // Angle between the plane normal and the cone axis, brought between 0 and PI/2.

        sint = axey.Modulus();
        cost = Math.Abs(Co.Axis().Direction().XYZ().Dot(normp));

        // The calculation of costa determines if the plane contains
        // a generatrix of the cone: we calculate std::sin((PI/2. - t) - angl)

        costa = cost * cosa - sint * sina; // sin((PI/2 -t)-angl)=cos(t+angl)
                                           // with t brought back between 0 and pi/2.

        if (Math.Abs(dist) < Tol)
        {
            // we consider that the plane contains the apex of the cone.
            // the possible solutions are therefore: 1 point, 1 line, 2 lines
            // according to the inclination of the plane.

            if (Math.Abs(costa) < Tolang)
            { // plane parallel to the generatrix
                typeres = IntAna_Line;
                nbint = 1;
                gp_XYZ ptonaxe = apex.XYZ() + 10.0 * (Co.Axis().Direction().XYZ());
                // point on the axis of the cone on the positive z side

                dist = A * ptonaxe.X() + B * ptonaxe.Y() + C * ptonaxe.Z() + D;
                ptonaxe = ptonaxe - dist * normp;
                pt1 = apex;
                dir1.SetXYZ(ptonaxe - pt1.XYZ());
            }
            else if (cost < sina)
            { // plane "interior" to the cone
                typeres = IntAna_Line;
                nbint = 2;
                pt1 = apex;
                pt2 = apex;
                dh = Math.Sqrt(sina * sina - cost * cost) / cosa;
                dir1.SetXYZ(axex + dh * axey);
                dir2.SetXYZ(axex - dh * axey);
            }
            else
            { // plane "exterior" to the cone
                typeres = IntAna_Point;
                nbint = 1;
                pt1 = apex;
            }
        }
        else
        {
            // Possible solutions: circle, ellipse, parabola, hyperbola according to
            // the inclination of the plane.
            double deltacenter, distance;

            if (cost < Tolang)
            {
                // The plane contains the direction of the cone axis. The solution is
                // the hyperbola
                typeres = IntAna_Hyperbola;
                nbint = 2;
                pt1.SetXYZ(apex.XYZ() - dist * normp);
                pt2 = pt1;
                dir1 = new gp_Dir(normp);
                dir2.SetXYZ(axex);
                param1 = param2 = Math.Abs(dist / Math.Tan(angl));
                param1bis = param2bis = Math.Abs(dist);
            }
            else
            {
                IntAna_IntConicQuad inter = new(axec, P, Tolang); // we necessarily have 1 point.

                gp_Pnt center = inter.Point(1);

                // Depending on the position of the intersection relative to the apex
                // of the cone, we change the x axis to -x and y to -y. The parameter of the apex
                // on axec is negative (see definition of the cone)

                distance = apex.Distance(center);

                if (inter.ParamOnConic(1) + Co.RefRadius() / Math.Tan(angl) < 0.0)
                {
                    axex.Reverse();
                    axey.Reverse();
                }

                if (Math.Abs(costa) < Tolang)
                { // plane parallel to a generatrix
                    typeres = IntAna_Parabola;
                    nbint = 1;
                    deltacenter = distance / 2.0 / cosa;
                    axex.Normalize();
                    pt1.SetXYZ(center.XYZ() - deltacenter * axex);
                    dir1 = new gp_Dir(normp);
                    dir2.SetXYZ(axex);
                    param1 = deltacenter * sina * sina;
                }
                else if (sint < Tolang)
                { // plane perpendicular to the axis
                    typeres = IntAna_Circle;
                    nbint = 1;
                    pt1 = center;
                    dir1 = Co.Position().Direction();
                    dir2 = Co.Position().XDirection();
                    param1 = apex.Distance(center) * Math.Abs(Math.Tan(angl));
                }
                else if (cost < sina)
                {
                    typeres = IntAna_Hyperbola;
                    nbint = 2;
                    axex.Normalize();

                    deltacenter = sint * sina * sina * distance / (sina * sina - cost * cost);
                    pt1.SetXYZ(center.XYZ() - deltacenter * axex);
                    pt2 = pt1;
                    dir1 = new gp_Dir(normp);
                    dir2.SetXYZ(axex);
                    param1 = param2 = cost * sina * cosa * distance / (sina * sina - cost * cost);
                    param1bis = param2bis = cost * sina * distance / Math.Sqrt(sina * sina - cost * cost);
                }
                else
                { // here cost > sina
                    typeres = IntAna_Ellipse;
                    nbint = 1;
                    double radius = cost * sina * cosa * distance / (cost * cost - sina * sina);
                    deltacenter = sint * sina * sina * distance / (cost * cost - sina * sina);
                    axex.Normalize();
                    pt1.SetXYZ(center.XYZ() + deltacenter * axex);
                    dir1 = new gp_Dir(normp);
                    dir2.SetXYZ(axex);
                    param1 = radius;
                    param1bis = cost * sina * distance / Math.Sqrt(cost * cost - sina * sina);
                }
            }
        }

        //-- Difficult to handle further (Value ProjLib, Params ... )
        //-- hyperbolas that are too extreme
        //-- Return False -> Treatment by biparametric method
        const double EllipseLimit = 1.0E+9;   // OCC513(apo) 1000000
        const double HyperbolaLimit = 2.0E+6; // OCC537(apo) 50000
        if (typeres == IntAna_Ellipse && nbint >= 1)
        {
            if (Math.Abs(param1) > EllipseLimit || Math.Abs(param1bis) > EllipseLimit)
            {
                done = false;
                return;
            }
        }
        if (typeres == IntAna_Hyperbola && nbint >= 2)
        {
            if (Math.Abs(param2) > HyperbolaLimit || Math.Abs(param2bis) > HyperbolaLimit)
            {
                done = false;
                return;
            }
        }
        if (typeres == IntAna_Hyperbola && nbint >= 1)
        {
            if (Math.Abs(param1) > HyperbolaLimit || Math.Abs(param1bis) > HyperbolaLimit)
            {
                done = false;
                return;
            }
        }

        done = true;
    }

    /// <summary>The intersection between a plane and a sphere.</summary>
    public IntAna_QuadQuadGeo(in gp_Pln P, in gp_Sphere S)
        : this()
    {
        Perform(P, S);
    }

    /// <summary>Intersects a plane and a sphere.</summary>
    public void Perform(in gp_Pln P, in gp_Sphere S)
    {
        done = false;
        double A, B, C, D, dist, radius;
        double X, Y, Z;

        nbint = 0;
        // debug JAG : on met typeres = IntAna_Empty par defaut...
        typeres = IntAna_Empty;

        P.Coefficients(out A, out B, out C, out D);
        S.Location().Coord(out X, out Y, out Z);
        radius = S.Radius();

        dist = A * X + B * Y + C * Z + D;

        if (Math.Abs(Math.Abs(dist) - radius) < Epsilon(radius))
        {
            // we have a single solution: the projection point of the sphere center
            // onto the plane
            nbint = 1;
            typeres = IntAna_Point;
            pt1.SetCoord(X - dist * A, Y - dist * B, Z - dist * C);
        }
        else if (Math.Abs(dist) < radius)
        {
            // we have a circle solution
            nbint = 1;
            typeres = IntAna_Circle;
            pt1.SetCoord(X - dist * A, Y - dist * B, Z - dist * C);
            dir1 = P.Axis().Direction();
            if (!P.Direct())
            {
                dir1.Reverse();
            }
            dir2 = P.Position().XDirection();
            param1 = Math.Sqrt(radius * radius - dist * dist);
        }
        param2bis = 0.0; //-- to avoid param2bis unused warning
        done = true;
    }

    /// <summary>The intersection between two cylinders.</summary>
    public IntAna_QuadQuadGeo(in gp_Cylinder Cyl1, in gp_Cylinder Cyl2, double Tol)
        : this()
    {
        Perform(Cyl1, Cyl2, Tol);
    }

    /// <summary>Intersects two cylinders.</summary>
    public void Perform(in gp_Cylinder Cyl1, in gp_Cylinder Cyl2, double Tol)
    {
        done = true;
        //---------------------------- Parallel axes -------------------------
        AxeOperator A1A2 = new(Cyl1.Axis(),
                               Cyl2.Axis(),
                               myEPSILON_CYLINDER_DELTA_DISTANCE,
                               myEPSILON_AXES_PARA);
        double R1 = Cyl1.Radius();
        double R2 = Cyl2.Radius();
        double RmR, RmR_Relative;
        RmR = (R1 > R2) ? (R1 - R2) : (R2 - R1);
        {
            double Rmax;
            Rmax = (R1 > R2) ? R1 : R2;
            RmR_Relative = RmR / Rmax;
        }

        double DistA1A2 = A1A2.Distance();

        if (A1A2.Parallel())
        {
            if (DistA1A2 <= Tol)
            {
                if (RmR <= Tol)
                {
                    typeres = IntAna_Same;
                }
                else
                {
                    typeres = IntAna_Empty;
                }
            }
            else
            { //-- DistA1A2 > Tol
                gp_Pnt P1 = Cyl1.Location();
                gp_Pnt P2t = Cyl2.Location();
                gp_Pnt P2 = new();
                //-- P2t is projected on the plane (P1,DirCylX,DirCylY)
                gp_Dir DirCyl = Cyl1.Position().Direction();
                double ProjP2OnDirCyl1 = new gp_Vec(DirCyl).Dot(new gp_Vec(P1, P2t));

                // P2 is a projection the location of the 2nd cylinder on the base
                // of the 1st cylinder
                P2.SetCoord(P2t.X() - ProjP2OnDirCyl1 * DirCyl.X(),
                            P2t.Y() - ProjP2OnDirCyl1 * DirCyl.Y(),
                            P2t.Z() - ProjP2OnDirCyl1 * DirCyl.Z());
                //--
                double R1pR2 = R1 + R2;
                if (DistA1A2 > (R1pR2 + Tol))
                {
                    typeres = IntAna_Empty;
                    nbint = 0;
                }
                else if ((R1pR2 - DistA1A2) <= RealSmall())
                {
                    //-- 1 Tangent line -------------------------------------OK
                    typeres = IntAna_Line;

                    nbint = 1;
                    dir1 = DirCyl;
                    double R1_R1pR2 = R1 / R1pR2;
                    pt1.SetCoord(P1.X() + R1_R1pR2 * (P2.X() - P1.X()),
                                 P1.Y() + R1_R1pR2 * (P2.Y() - P1.Y()),
                                 P1.Z() + R1_R1pR2 * (P2.Z() - P1.Z()));
                }
                else if (DistA1A2 > RmR)
                {
                    //-- 2 lines ---------------------------------------------OK
                    typeres = IntAna_Line;
                    nbint = 2;
                    dir1 = DirCyl;
                    dir2 = dir1;

                    double aR1R1 = R1 * R1;

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
                    double aCos = 0.5 * (aR1R1 - R2 * R2 + DistA1A2 * DistA1A2) / (R1 * DistA1A2);
                    double aSin2 = 1 - aCos * aCos;

                    bool isTangent = ((4.0 * aR1R1 * aSin2) < Tol * Tol);

                    // Normalized vector P1P2
                    gp_Vec DirA1A2 = new((P2.XYZ() - P1.XYZ()) / DistA1A2);

                    if (isTangent)
                    {
                        // Intercept the segment from P1 point along P1P2 direction
                        // and having |P1O1| length
                        nbint = 1;
                        pt1.SetXYZ(P1.XYZ() + DirA1A2.XYZ() * R1 * aCos);
                    }
                    else
                    {
                        // Sine of the A-P1-P2 angle (if aSin2 < 0 then isTangent == TRUE =>
                        // go to another branch)
                        double aSin = Math.Sqrt(aSin2);

                        // 1. Rotate P1P2 to the angle A-P1-P2 relative to P1
                        //(clockwise and anticlockwise for getting
                        // two intersection points).
                        // 2. Intercept the segment from P1 along direction,
                        // determined in the preview paragraph and having R1 length
                        gp_Dir aXDir = Cyl1.Position().XDirection(), aYDir = Cyl1.Position().YDirection();
                        gp_Vec aR1Xdir = new(R1 * aXDir.XYZ()), aR1Ydir = new(R1 * aYDir.XYZ());

                        // Source 2D-coordinates of the P1P2 vector normalized
                        // in coordinate system, based on the X- and Y-directions
                        // of the 1st cylinder in the plane of the 1st cylinder base
                        //(P1 is the origin of the coordinate system).
                        double aDx = DirA1A2.Dot(new gp_Vec(aXDir)), aDy = DirA1A2.Dot(new gp_Vec(aYDir));

                        // New coordinate (after rotation) of the P1P2 vector normalized.
                        double aNewDx = aDx * aCos - aDy * aSin, aNewDy = aDy * aCos + aDx * aSin;
                        pt1.SetXYZ(P1.XYZ() + aNewDx * aR1Xdir.XYZ() + aNewDy * aR1Ydir.XYZ());

                        aNewDx = aDx * aCos + aDy * aSin;
                        aNewDy = aDy * aCos - aDx * aSin;
                        pt2.SetXYZ(P1.XYZ() + aNewDx * aR1Xdir.XYZ() + aNewDy * aR1Ydir.XYZ());
                    }
                }
                else if (DistA1A2 > (RmR - Tol))
                {
                    //-- 1 Tangent ------------------------------------------OK
                    typeres = IntAna_Line;
                    nbint = 1;
                    dir1 = DirCyl;
                    double R1_RmR = R1 / RmR;

                    if (R1 < R2)
                    {
                        R1_RmR = -R1_RmR;
                    }

                    pt1.SetCoord(P1.X() + R1_RmR * (P2.X() - P1.X()),
                                 P1.Y() + R1_RmR * (P2.Y() - P1.Y()),
                                 P1.Z() + R1_RmR * (P2.Z() - P1.Z()));
                }
                else
                {
                    nbint = 0;
                    typeres = IntAna_Empty;
                }
            }
        }
        else
        { //-- No Parallel Axis ---------------------------------OK
            if ((RmR_Relative <= myEPSILON_CYLINDER_DELTA_RADIUS) && A1A2.Intersect())
            {
                //-- PI/2 between the two axis   and   Intersection
                //-- and identical radius
                typeres = IntAna_Ellipse;
                nbint = 2;
                gp_Dir DirCyl1 = Cyl1.Position().Direction();
                gp_Dir DirCyl2 = Cyl2.Position().Direction();
                pt1 = pt2 = A1A2.PtIntersect();

                double A = DirCyl1.Angle(DirCyl2);
                double B;
                B = Math.Abs(Math.Sin(0.5 * (M_PI - A)));
                A = Math.Abs(Math.Sin(0.5 * A));

                if (A == 0.0 || B == 0.0)
                {
                    typeres = IntAna_Same;
                    return;
                }

                gp_Vec dircyl1 = new(DirCyl1);
                gp_Vec dircyl2 = new(DirCyl2);
                dir1 = new gp_Dir(dircyl1.Added(dircyl2));
                dir2 = new gp_Dir(dircyl1.Subtracted(dircyl2));

                param2 = Cyl1.Radius() / A;
                param1 = Cyl1.Radius() / B;
                param2bis = param1bis = Cyl1.Radius();
                if (param1 < param1bis)
                {
                    A = param1;
                    param1 = param1bis;
                    param1bis = A;
                }

                if (param2 < param2bis)
                {
                    A = param2;
                    param2 = param2bis;
                    param2bis = A;
                }
            }
            else
            {
                if (Math.Abs(DistA1A2 - Cyl1.Radius() - Cyl2.Radius()) < Tol)
                {
                    typeres = IntAna_Point;
                    double d, p1, p2;

                    gp_Dir D1 = Cyl1.Axis().Direction();
                    gp_Dir D2 = Cyl2.Axis().Direction();
                    A1A2.Distance(out d, out p1, out p2);
                    gp_Pnt P = Cyl1.Axis().Location();
                    gp_Pnt P1 = new(P.X() - p1 * D1.X(), P.Y() - p1 * D1.Y(), P.Z() - p1 * D1.Z());

                    P = Cyl2.Axis().Location();

                    gp_Pnt P2 = new(P.X() - p2 * D2.X(), P.Y() - p2 * D2.Y(), P.Z() - p2 * D2.Z());

                    gp_Vec P1P2 = new(P1, P2);
                    D1 = new gp_Dir(P1P2);
                    p1 = Cyl1.Radius();

                    pt1.SetCoord(P1.X() + p1 * D1.X(), P1.Y() + p1 * D1.Y(), P1.Z() + p1 * D1.Z());
                    nbint = 1;
                }
                else
                {
                    typeres = IntAna_NoGeometricSolution;
                }
            }
        }
    }

    /// <summary>The intersection between a cylinder and a cone (only when they are coaxial).</summary>
    public IntAna_QuadQuadGeo(in gp_Cylinder Cyl, in gp_Cone Con, double Tol)
        : this()
    {
        Perform(Cyl, Con, Tol);
    }

    /// <summary>Intersects a cylinder and a cone (only when they are coaxial). Tol is not used.</summary>
    public void Perform(in gp_Cylinder Cyl, in gp_Cone Con, double Tol)
    {
        _ = Tol;
        done = true;
        AxeOperator A1A2 = new(Cyl.Axis(), Con.Axis());
        if (A1A2.Same())
        {
            gp_Pnt Pt = Con.Apex();
            double dist = Cyl.Radius() / (Math.Tan(Con.SemiAngle()));
            gp_Dir dir = Cyl.Position().Direction();
            pt1.SetCoord(Pt.X() + dist * dir.X(), Pt.Y() + dist * dir.Y(), Pt.Z() + dist * dir.Z());
            pt2.SetCoord(Pt.X() - dist * dir.X(), Pt.Y() - dist * dir.Y(), Pt.Z() - dist * dir.Z());
            dir1 = dir2 = dir;
            param1 = param2 = Cyl.Radius();
            nbint = 2;
            typeres = IntAna_Circle;
        }
        else
        {
            typeres = IntAna_NoGeometricSolution;
        }
    }

    /// <summary>The intersection between a cylinder and a sphere (only when the center is on the axis).</summary>
    public IntAna_QuadQuadGeo(in gp_Cylinder Cyl, in gp_Sphere Sph, double Tol)
        : this()
    {
        Perform(Cyl, Sph, Tol);
    }

    /// <summary>Intersects a cylinder and a sphere (only when the center is on the axis). Tol is not used.</summary>
    public void Perform(in gp_Cylinder Cyl, in gp_Sphere Sph, double Tol)
    {
        _ = Tol;
        done = true;
        gp_Pnt Pt = Sph.Location();
        AxeOperator A1A2 = new(Cyl.Axis(), Sph.Position().Axis());
        if ((A1A2.Intersect() && Pt.Distance(A1A2.PtIntersect()) == 0.0) || (A1A2.Same()))
        {
            if (Sph.Radius() < Cyl.Radius())
            {
                typeres = IntAna_Empty;
            }
            else
            {
                double dist = Math.Sqrt(Sph.Radius() * Sph.Radius() - Cyl.Radius() * Cyl.Radius());
                gp_Dir dir = Cyl.Position().Direction();
                dir1 = dir2 = dir;
                typeres = IntAna_Circle;
                pt1.SetCoord(Pt.X() + dist * dir.X(), Pt.Y() + dist * dir.Y(), Pt.Z() + dist * dir.Z());
                nbint = 1;
                param1 = Cyl.Radius();
                if (dist > RealEpsilon())
                {
                    pt2.SetCoord(Pt.X() - dist * dir.X(), Pt.Y() - dist * dir.Y(), Pt.Z() - dist * dir.Z());
                    param2 = Cyl.Radius();
                    nbint = 2;
                }
            }
        }
        else
        {
            typeres = IntAna_NoGeometricSolution;
        }
    }

    /// <summary>The intersection between two cones.</summary>
    public IntAna_QuadQuadGeo(in gp_Cone Con1, in gp_Cone Con2, double Tol)
        : this()
    {
        Perform(Con1, Con2, Tol);
    }

    /// <summary>Intersects two cones.</summary>
    public void Perform(in gp_Cone Con1, in gp_Cone Con2, double Tol)
    {
        done = true;
        //
        double tg1, tg2, aDA1A2, aTol2;
        gp_Pnt aPApex1, aPApex2;

        double TOL_APEX_CONF = 1.0e-10;

        //
        tg1 = Math.Tan(Con1.SemiAngle());
        tg2 = Math.Tan(Con2.SemiAngle());

        if ((tg1 * tg2) < 0.0)
        {
            tg2 = -tg2;
        }
        //
        aTol2 = Tol * Tol;
        aPApex1 = Con1.Apex();
        aPApex2 = Con2.Apex();
        aDA1A2 = aPApex1.SquareDistance(aPApex2);
        //
        AxeOperator A1A2 = new(Con1.Axis(), Con2.Axis());
        //
        double aTolAng = myEPSILON_ANGLE_CONE;
        if ((Math.Abs(tg1 - tg2) < Tol) && (A1A2.Parallel()))
        {
            double DistA1A2 = A1A2.Distance();
            if (DistA1A2 > 100.0 * Tol)
            {
                double aMinSolDist = EstimDist(Con1, Con2);
                if (aMinSolDist < Epsilon(1.0))
                {
                    aTolAng = Tol;
                }
                else
                {
                    aTolAng = Max(myEPSILON_ANGLE_CONE, Tol / aMinSolDist);
                    aTolAng = Min(aTolAng, Tol);
                }
            }
        }

        // 1
        if (A1A2.Same())
        {
            //-- two circles
            double x;
            gp_Pnt P = Con1.Apex();
            gp_Dir D = Con1.Position().Direction();
            double d = new gp_Vec(D).Dot(new gp_Vec(P, Con2.Apex()));

            if (Math.Abs(tg1 - tg2) > myEPSILON_ANGLE_CONE)
            {
                if (Math.Abs(d) < TOL_APEX_CONF)
                {
                    typeres = IntAna_Point;
                    nbint = 1;
                    pt1 = P;
                    return;
                }
                x = (d * tg2) / (tg1 + tg2);
                pt1.SetCoord(P.X() + x * D.X(), P.Y() + x * D.Y(), P.Z() + x * D.Z());
                param1 = Math.Abs(x * tg1);

                x = (d * tg2) / (tg2 - tg1);
                pt2.SetCoord(P.X() + x * D.X(), P.Y() + x * D.Y(), P.Z() + x * D.Z());
                param2 = Math.Abs(x * tg1);
                dir1 = dir2 = D;
                nbint = 2;
                typeres = IntAna_Circle;
            }
            else
            {
                if (Math.Abs(d) < TOL_APEX_CONF)
                {
                    typeres = IntAna_Same;
                }
                else
                {
                    typeres = IntAna_Circle;
                    nbint = 1;
                    x = d * 0.5;
                    pt1.SetCoord(P.X() + x * D.X(), P.Y() + x * D.Y(), P.Z() + x * D.Z());
                    param1 = Math.Abs(x * tg1);
                    dir1 = D;
                }
            }
        } //-- fin A1A2.Same
        // 2
        else if ((Math.Abs(tg1 - tg2) < aTolAng) && (A1A2.Parallel()))
        {
            double DistA1A2 = A1A2.Distance();
            gp_Dir DA1 = Con1.Position().Direction();
            gp_Vec O1O2 = new(Con1.Apex(), Con2.Apex());
            gp_Dir O1O2n = new(O1O2); // normalization of the vector before projection
            double O1O2_DA1 = new gp_Vec(DA1).Dot(new gp_Vec(O1O2n));

            gp_Vec O1_Proj_A2 = new(O1O2n.X() - O1O2_DA1 * DA1.X(),
                                    O1O2n.Y() - O1O2_DA1 * DA1.Y(),
                                    O1O2n.Z() - O1O2_DA1 * DA1.Z());
            gp_Dir DB1 = new(O1_Proj_A2);

            double yO1O2 = O1O2.Dot(new gp_Vec(DA1));
            double ABSTG1 = Math.Abs(tg1);
            double X2 = (DistA1A2 / ABSTG1 - yO1O2) * 0.5;
            double X1 = X2 + yO1O2;

            gp_Pnt P1 = new(Con1.Apex().X() + X1 * (DA1.X() + ABSTG1 * DB1.X()),
                            Con1.Apex().Y() + X1 * (DA1.Y() + ABSTG1 * DB1.Y()),
                            Con1.Apex().Z() + X1 * (DA1.Z() + ABSTG1 * DB1.Z()));

            gp_Pnt MO1O2 = new(0.5 * (Con1.Apex().X() + Con2.Apex().X()),
                               0.5 * (Con1.Apex().Y() + Con2.Apex().Y()),
                               0.5 * (Con1.Apex().Z() + Con2.Apex().Z()));
            gp_Vec P1MO1O2 = new(P1, MO1O2);

            gp_Dir DA1_X_DB1 = DA1.Crossed(DB1);
            gp_Dir OrthoPln = DA1_X_DB1.Crossed(new gp_Dir(P1MO1O2));

            IntAna_QuadQuadGeo INTER_QUAD_PLN = new(new gp_Pln(P1, OrthoPln), Con1, Tol, Tol);
            if (INTER_QUAD_PLN.IsDone())
            {
                switch (INTER_QUAD_PLN.TypeInter())
                {
                    case IntAna_Ellipse:
                    {
                        typeres = IntAna_Ellipse;
                        gp_Elips E = INTER_QUAD_PLN.Ellipse(1);
                        pt1 = E.Location();
                        dir1 = E.Position().Direction();
                        dir2 = E.Position().XDirection();
                        param1 = E.MajorRadius();
                        param1bis = E.MinorRadius();
                        nbint = 1;
                        break;
                    }
                    case IntAna_Circle:
                    {
                        typeres = IntAna_Circle;
                        gp_Circ C = INTER_QUAD_PLN.Circle(1);
                        pt1 = C.Location();
                        dir1 = C.Position().XDirection();
                        dir2 = C.Position().YDirection();
                        param1 = C.Radius();
                        nbint = 1;
                        break;
                    }
                    case IntAna_Hyperbola:
                    {
                        typeres = IntAna_Hyperbola;
                        gp_Hypr H = INTER_QUAD_PLN.Hyperbola(1);
                        pt1 = pt2 = H.Location();
                        dir1 = H.Position().Direction();
                        dir2 = H.Position().XDirection();
                        param1 = param2 = H.MajorRadius();
                        param1bis = param2bis = H.MinorRadius();
                        nbint = 2;
                        break;
                    }
                    case IntAna_Line:
                    {
                        typeres = IntAna_Line;
                        gp_Lin H = INTER_QUAD_PLN.Line(1);
                        pt1 = pt2 = H.Location();
                        dir1 = dir2 = H.Position().Direction();
                        param1 = param2 = 0.0;
                        param1bis = param2bis = 0.0;
                        nbint = 2;
                        break;
                    }
                    default:
                        typeres = IntAna_NoGeometricSolution;
                        break;
                }
            }
        }
        // 3
        else if (aDA1A2 < aTol2)
        {
            //
            // When apices are coincided there can be 3 possible cases
            // 3.1 - empty solution (iRet=0)
            // 3.2 - one line  when cone1 touches cone2 (iRet=1)
            // 3.3 - two lines when cone1 intersects cone2 (iRet=2)
            //
            double aGamma, aBeta1, aBeta2;
            double aD1, aR1, aTgBeta1, aTgBeta2, aHalfPI;
            double aCosGamma, aSinGamma, aDx, aR2, aRD2, aD2;
            gp_Pnt2d aP0 = new(), aPA1 = new(), aP1 = new(), aPA2;
            gp_Vec2d aVAx2 = new();
            gp_Ax1 aAx1, aAx2;
            //
            // Preliminary analysis. Determination of iRet
            //
            aHalfPI = 0.5 * M_PI;
            aD1 = 1.0;
            aPA1.SetCoord(aD1, 0.0);
            aP0.SetCoord(0.0, 0.0);
            //
            aAx1 = Con1.Axis();
            aAx2 = Con2.Axis();
            aGamma = aAx1.Angle(aAx2);
            if (aGamma > aHalfPI)
            {
                aGamma = M_PI - aGamma;
            }
            aCosGamma = Math.Cos(aGamma);
            aSinGamma = Math.Sin(aGamma);
            //
            aBeta1 = Con1.SemiAngle();
            aTgBeta1 = Math.Tan(aBeta1);
            aTgBeta1 = Math.Abs(aTgBeta1);
            //
            aBeta2 = Con2.SemiAngle();
            aTgBeta2 = Math.Tan(aBeta2);
            aTgBeta2 = Math.Abs(aTgBeta2);
            //
            aR1 = aD1 * aTgBeta1;
            aP1.SetCoord(aD1, aR1);
            //
            // PA2
            aVAx2.SetCoord(aCosGamma, aSinGamma);
            gp_Dir2d aDAx2 = new(aVAx2);
            gp_Lin2d aLAx2 = new(aP0, aDAx2);
            _ = aLAx2;
            //
            gp_Vec2d aV = new(aP0, aP1);
            aDx = aVAx2.Dot(aV);
            aPA2 = aP0.Translated(aDx * new gp_Vec2d(aDAx2));
            //
            // aR2
            aDx = aPA2.Distance(aP0);
            aR2 = aDx * aTgBeta2;
            //
            // aRD2
            aRD2 = aPA2.Distance(aP1);
            //
            if (aRD2 > (aR2 + Tol))
            {
                typeres = IntAna_Empty; // nothing
                return;
            }
            //
            int iRet = 1; // touch case => 1 line
            if (aRD2 < (aR2 - Tol))
            {
                iRet = 2; // intersection => couple of lines
            }
            //
            // Finding the solution in 3D
            //
            double aDa;
            gp_Pnt aQApex1, aQA1 = new(), aQA2 = new(), aQX, aQX1, aQX2;
            gp_Dir aD3Ax1, aD3Ax2;
            gp_Lin aLin;
            IntAna_QuadQuadGeo aIntr = new();
            //
            aQApex1 = Con1.Apex();
            aD3Ax1 = aAx1.Direction();
            aQA1.SetCoord(aQApex1.X() + aD1 * aD3Ax1.X(),
                          aQApex1.Y() + aD1 * aD3Ax1.Y(),
                          aQApex1.Z() + aD1 * aD3Ax1.Z());
            //
            aDx = aD3Ax1.Dot(aAx2.Direction());
            if (aDx < 0.0)
            {
                aAx2.Reverse();
            }
            aD3Ax2 = aAx2.Direction();
            //
            aD2 = aD1 * Math.Sqrt((1.0 + aTgBeta1 * aTgBeta1) / (1.0 + aTgBeta2 * aTgBeta2));
            //
            aQA2.SetCoord(aQApex1.X() + aD2 * aD3Ax2.X(),
                          aQApex1.Y() + aD2 * aD3Ax2.Y(),
                          aQApex1.Z() + aD2 * aD3Ax2.Z());
            //
            gp_Pln aPln1 = new(aQA1, aD3Ax1);
            gp_Pln aPln2 = new(aQA2, aD3Ax2);
            //
            aIntr.Perform(aPln1, aPln2, Tol, Tol);
            if (!aIntr.IsDone() || 0 == aIntr.NbSolutions())
            {
                typeres = IntAna_NoGeometricSolution;
                return;
            }
            //
            aLin = aIntr.Line(1);
            gp_Dir aDLin = aLin.Direction();
            gp_Vec aVLin = new(aDLin);
            gp_Pnt aOrig = aLin.Location();
            gp_Vec aVr = new(aQA1, aOrig);
            aDx = aVLin.Dot(aVr);
            aQX = aOrig.Translated(aDx * aVLin);
            //
            // Final part
            //
            typeres = IntAna_Line;
            //
            param1 = 0.0;
            param2 = 0.0;
            param1bis = 0.0;
            param2bis = 0.0;
            //
            if (iRet == 1)
            {
                // one line
                nbint = 1;
                pt1 = aQApex1;
                gp_Vec aVX = new(aQApex1, aQX);
                dir1 = new gp_Dir(aVX);
            }
            else
            { // iRet=2
                // two lines
                nbint = 2;
                aDa = aQA1.Distance(aQX);
                aDx = Math.Sqrt(aR1 * aR1 - aDa * aDa);
                aQX1 = aQX.Translated(aDx * aVLin);
                aQX2 = aQX.Translated(-aDx * aVLin);
                //
                pt1 = aQApex1;
                pt2 = aQApex1;
                gp_Vec aVX1 = new(aQApex1, aQX1);
                dir1 = new gp_Dir(aVX1);
                gp_Vec aVX2 = new(aQApex1, aQX2);
                dir2 = new gp_Dir(aVX2);
            }
        } // else if (aDA1A2<aTol2) {
        // Case when cones have common generatrix
        else if (A1A2.Intersect())
        {
            // Check if apex of one cone belongs another one
            double u, v, tol2 = Tol * Tol;
            ElSLib.Parameters(Con2, aPApex1, out u, out v);
            gp_Pnt p = ElSLib.Value(u, v, Con2);
            if (aPApex1.SquareDistance(p) > tol2)
            {
                typeres = IntAna_NoGeometricSolution;
                return;
            }
            //
            ElSLib.Parameters(Con1, aPApex2, out u, out v);
            p = ElSLib.Value(u, v, Con1);
            if (aPApex2.SquareDistance(p) > tol2)
            {
                typeres = IntAna_NoGeometricSolution;
                return;
            }

            // Cones have a common generatrix passing through apexes
            myCommonGen = true;

            // common generatrix of cones
            gp_Lin aGen = new(aPApex1, new gp_Dir(new gp_Vec(aPApex1, aPApex2)));

            // Intersection point of axes
            gp_Pnt aPAxeInt = A1A2.PtIntersect();

            // Characteristic point of intersection curve
            u = ElCLib.Parameter(aGen, aPAxeInt);
            myPChar = ElCLib.Value(u, aGen);

            // Other generatrixes of cones laying in maximal plane
            gp_Lin aGen1 = aGen.Rotated(Con1.Axis(), M_PI);
            gp_Lin aGen2 = aGen.Rotated(Con2.Axis(), M_PI);
            //
            // Intersection point of generatrixes
            gp_Dir aN; // solution plane normal
            gp_Dir aD1 = aGen1.Direction();

            gp_Dir aD2 = aD1.Crossed(aGen.Direction());

            if (aD1.IsParallel(aGen2.Direction(), Precision.Angular()))
            {
                aN = aD1.Crossed(aD2);
            }
            else if (aGen1.SquareDistance(aGen2) > tol2)
            {
                // Something wrong ???
                typeres = IntAna_NoGeometricSolution;
                return;
            }
            else
            {
                gp_Dir D1 = aGen1.Position().Direction();
                gp_Dir D2 = aGen2.Position().Direction();
                gp_Pnt O1 = aGen1.Location();
                gp_Pnt O2 = aGen2.Location();
                double D1DotD2 = D1.Dot(D2);
                double aSin = 1.0 - D1DotD2 * D1DotD2;
                gp_Vec O1O2 = new(O1, O2);
                double U2 = (D1.XYZ() * (O1O2.Dot(new gp_Vec(D1))) - (O1O2.XYZ())).Dot(D2.XYZ());
                U2 /= aSin;
                gp_Pnt aPGint = ElCLib.Value(U2, aGen2);

                aD1 = new gp_Dir(new gp_Vec(aPGint, myPChar));
                aN = aD1.Crossed(aD2);
            }
            // Plane that must contain intersection curves
            gp_Pln anIntPln = new(myPChar, aN);

            IntAna_QuadQuadGeo INTER_QUAD_PLN = new(anIntPln, Con1, Tol, Tol);

            if (INTER_QUAD_PLN.IsDone())
            {
                switch (INTER_QUAD_PLN.TypeInter())
                {
                    case IntAna_Ellipse:
                    {
                        typeres = IntAna_Ellipse;
                        gp_Elips E = INTER_QUAD_PLN.Ellipse(1);
                        pt1 = E.Location();
                        dir1 = E.Position().Direction();
                        dir2 = E.Position().XDirection();
                        param1 = E.MajorRadius();
                        param1bis = E.MinorRadius();
                        nbint = 1;
                        break;
                    }
                    case IntAna_Circle:
                    {
                        typeres = IntAna_Circle;
                        gp_Circ C = INTER_QUAD_PLN.Circle(1);
                        pt1 = C.Location();
                        dir1 = C.Position().XDirection();
                        dir2 = C.Position().YDirection();
                        param1 = C.Radius();
                        nbint = 1;
                        break;
                    }
                    case IntAna_Parabola:
                    {
                        typeres = IntAna_Parabola;
                        gp_Parab Prb = INTER_QUAD_PLN.Parabola(1);
                        pt1 = Prb.Location();
                        dir1 = Prb.Position().Direction();
                        dir2 = Prb.Position().XDirection();
                        param1 = Prb.Focal();
                        nbint = 1;
                        break;
                    }
                    case IntAna_Hyperbola:
                    {
                        typeres = IntAna_Hyperbola;
                        gp_Hypr H = INTER_QUAD_PLN.Hyperbola(1);
                        pt1 = pt2 = H.Location();
                        dir1 = H.Position().Direction();
                        dir2 = H.Position().XDirection();
                        param1 = param2 = H.MajorRadius();
                        param1bis = param2bis = H.MinorRadius();
                        nbint = 2;
                        break;
                    }
                    default:
                        typeres = IntAna_NoGeometricSolution;
                        break;
                }
            }
        }
        else
        {
            typeres = IntAna_NoGeometricSolution;
        }
    }

    /// <summary>The intersection between a sphere and a cone (only when the center is on the axis).</summary>
    public IntAna_QuadQuadGeo(in gp_Sphere Sph, in gp_Cone Con, double Tol)
        : this()
    {
        Perform(Sph, Con, Tol);
    }

    /// <summary>Intersects a sphere and a cone (only when the center is on the axis). Tol is not used.</summary>
    public void Perform(in gp_Sphere Sph, in gp_Cone Con, double Tol)
    {
        _ = Tol;
        //
        done = true;
        //
        AxeOperator A1A2 = new(Con.Axis(), Sph.Position().Axis());
        gp_Pnt Pt = Sph.Location();
        //
        if ((A1A2.Intersect() && (Pt.Distance(A1A2.PtIntersect()) == 0.0)) || A1A2.Same())
        {
            gp_Pnt ConApex = Con.Apex();
            double dApexSphCenter = Pt.Distance(ConApex);
            gp_Dir ConDir;
            if (dApexSphCenter > RealEpsilon())
            {
                ConDir = new gp_Dir(new gp_Vec(ConApex, Pt));
            }
            else
            {
                ConDir = Con.Position().Direction();
            }

            double Rad = Sph.Radius();
            double tga = Math.Tan(Con.SemiAngle());

            //-- 2 circles
            //-- x: Roots of    (x**2 + y**2 = Rad**2)
            //--                tga = y / (x+dApexSphCenter)
            double tgatga = tga * tga;
            math_DirectPolynomialRoots Eq = new(1.0 + tgatga,
                                                2.0 * tgatga * dApexSphCenter,
                                                -Rad * Rad + dApexSphCenter * dApexSphCenter * tgatga);
            if (Eq.IsDone())
            {
                int nbsol = Eq.NbSolutions();
                if (nbsol == 0)
                {
                    typeres = IntAna_Empty;
                }
                else
                {
                    typeres = IntAna_Circle;
                    if (nbsol >= 1)
                    {
                        double x = Eq.Value(1);
                        double dApexSphCenterpx = dApexSphCenter + x;
                        nbint = 1;
                        pt1.SetCoord(ConApex.X() + (dApexSphCenterpx) * ConDir.X(),
                                     ConApex.Y() + (dApexSphCenterpx) * ConDir.Y(),
                                     ConApex.Z() + (dApexSphCenterpx) * ConDir.Z());
                        param1 = tga * dApexSphCenterpx;
                        param1 = Math.Abs(param1);
                        dir1 = ConDir;
                        if (param1 <= myEPSILON_MINI_CIRCLE_RADIUS)
                        {
                            typeres = IntAna_PointAndCircle;
                            param1 = 0.0;
                        }
                    }
                    if (nbsol >= 2)
                    {
                        double x = Eq.Value(2);
                        double dApexSphCenterpx = dApexSphCenter + x;
                        nbint = 2;
                        pt2.SetCoord(ConApex.X() + (dApexSphCenterpx) * ConDir.X(),
                                     ConApex.Y() + (dApexSphCenterpx) * ConDir.Y(),
                                     ConApex.Z() + (dApexSphCenterpx) * ConDir.Z());
                        param2 = tga * dApexSphCenterpx;
                        param2 = Math.Abs(param2);
                        dir2 = ConDir;
                        if (param2 <= myEPSILON_MINI_CIRCLE_RADIUS)
                        {
                            typeres = IntAna_PointAndCircle;
                            param2 = 0.0;
                        }
                    }
                }
            }
            else
            {
                done = false;
            }
        }
        else
        {
            typeres = IntAna_NoGeometricSolution;
        }
    }

    /// <summary>The intersection between two spheres.</summary>
    public IntAna_QuadQuadGeo(in gp_Sphere Sph1, in gp_Sphere Sph2, double Tol)
        : this()
    {
        Perform(Sph1, Sph2, Tol);
    }

    /// <summary>Intersects two spheres.</summary>
    public void Perform(in gp_Sphere Sph1, in gp_Sphere Sph2, double Tol)
    {
        done = true;
        gp_Pnt O1 = Sph1.Location();
        gp_Pnt O2 = Sph2.Location();
        double dO1O2 = O1.Distance(O2);
        double R1 = Sph1.Radius();
        double R2 = Sph2.Radius();
        double Rmin, Rmax;
        typeres = IntAna_Empty;
        param2bis = 0.0; //-- to avoid param2bis unused warning

        if (R1 > R2)
        {
            Rmin = R2;
            Rmax = R1;
        }
        else
        {
            Rmin = R1;
            Rmax = R2;
        }

        if (dO1O2 <= Tol && (Math.Abs(R1 - R2) <= Tol))
        {
            typeres = IntAna_Same;
        }
        else
        {
            if (dO1O2 <= Tol)
            {
                return;
            }
            gp_Dir Dir = new(new gp_Vec(O1, O2));
            double t = Rmax - dO1O2 - Rmin;

            //----------------------------------------------------------------------
            //--        |----------------- R1 --------------------|
            //--        |----dO1O2-----|-----------R2----------|
            //--                                            --->--<-- t
            //--
            //--        |------ R1 ------|---------dO1O2----------|
            //--     |-------------------R2-----------------------|
            //--  --->--<-- t
            //----------------------------------------------------------------------
            if (t >= 0.0 && t <= Tol)
            {
                typeres = IntAna_Point;
                nbint = 1;
                double t2;
                if (R1 == Rmax)
                {
                    t2 = (R1 + (R2 + dO1O2)) * 0.5;
                }
                else
                {
                    t2 = (-R1 + (dO1O2 - R2)) * 0.5;
                }

                pt1.SetCoord(O1.X() + t2 * Dir.X(), O1.Y() + t2 * Dir.Y(), O1.Z() + t2 * Dir.Z());
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
                if ((dO1O2 > (R1 + R2 + Tol)) || (Rmax > (dO1O2 + Rmin + Tol)))
                {
                    typeres = IntAna_Empty;
                }
                else
                {
                    //---------------------------------------------------------------
                    //--
                    //--
                    //---------------------------------------------------------------
                    double Alpha = 0.5 * (R1 * R1 - R2 * R2 + dO1O2 * dO1O2) / (dO1O2);
                    double Beta = R1 * R1 - Alpha * Alpha;
                    Beta = (Beta > 0.0) ? Math.Sqrt(Beta) : 0.0;

                    if (Beta <= myEPSILON_MINI_CIRCLE_RADIUS)
                    {
                        typeres = IntAna_Point;
                        Alpha = (R1 + (dO1O2 - R2)) * 0.5;
                    }
                    else
                    {
                        typeres = IntAna_Circle;
                        dir1 = Dir;
                        param1 = Beta;
                    }
                    pt1.SetCoord(O1.X() + Alpha * Dir.X(), O1.Y() + Alpha * Dir.Y(), O1.Z() + Alpha * Dir.Z());

                    nbint = 1;
                }
            }
        }
    }

    /// <summary>The intersection between a plane and a torus.</summary>
    public IntAna_QuadQuadGeo(in gp_Pln Pln, in gp_Torus Tor, double Tol)
        : this()
    {
        Perform(Pln, Tor, Tol);
    }

    /// <summary>Intersects a plane and a torus (only a plane normal to the axis, or containing it).</summary>
    public void Perform(in gp_Pln Pln, in gp_Torus Tor, double Tol)
    {
        done = true;
        //
        double aRMin, aRMaj;
        //
        aRMin = Tor.MinorRadius();
        aRMaj = Tor.MajorRadius();
        if (aRMin >= aRMaj)
        {
            typeres = IntAna_NoGeometricSolution;
            return;
        }
        //
        gp_Ax1 aPlnAx = Pln.Axis();
        gp_Ax1 aTorAx = Tor.Axis();
        //
        bool bParallel, bNormal;
        //
        bParallel = aTorAx.IsParallel(aPlnAx, myEPSILON_AXES_PARA);
        bNormal = !bParallel ? aTorAx.IsNormal(aPlnAx, myEPSILON_AXES_PARA) : false;
        if (!bNormal && !bParallel)
        {
            typeres = IntAna_NoGeometricSolution;
            return;
        }
        //
        double aDist;
        //
        gp_Pnt aTorLoc = aTorAx.Location();
        if (bParallel)
        {
            double aDt, X, Y, Z, A, B, C, D, aDR, aTolNum;
            //
            aTolNum = myEPSILON_CYLINDER_DELTA_RADIUS;
            //
            Pln.Coefficients(out A, out B, out C, out D);
            aTorLoc.Coord(out X, out Y, out Z);
            aDist = A * X + B * Y + C * Z + D;
            //
            aDR = Math.Abs(aDist) - aRMin;
            if (aDR > aTolNum)
            {
                typeres = IntAna_Empty;
                return;
            }
            //
            if (Math.Abs(aDR) < aTolNum)
            {
                aDist = (aDist < 0) ? -aRMin : aRMin;
            }
            //
            typeres = IntAna_Circle;
            //
            pt1.SetCoord(X - aDist * A, Y - aDist * B, Z - aDist * C);
            aDt = Math.Sqrt(Math.Abs(aRMin * aRMin - aDist * aDist));
            param1 = aRMaj + aDt;
            dir1 = aTorAx.Direction();
            nbint = 1;
            if ((aDR < -aTolNum) && (aDt > Tol))
            {
                pt2 = pt1;
                param2 = aRMaj - aDt;
                dir2 = dir1;
                nbint = 2;
            }
        }
        //
        else
        {
            aDist = Pln.Distance(aTorLoc);
            if (aDist > myEPSILON_DISTANCE)
            {
                typeres = IntAna_NoGeometricSolution;
                return;
            }
            //
            typeres = IntAna_Circle;
            param2 = param1 = aRMin;
            dir2 = dir1 = aPlnAx.Direction();
            nbint = 2;
            //
            gp_Dir aDir = aTorAx.Direction() ^ dir1;
            pt1.SetXYZ(aTorLoc.XYZ() + aRMaj * aDir.XYZ());
            pt2.SetXYZ(aTorLoc.XYZ() - aRMaj * aDir.XYZ());
        }
    }

    /// <summary>The intersection between a cylinder and a torus.</summary>
    public IntAna_QuadQuadGeo(in gp_Cylinder Cyl, in gp_Torus Tor, double Tol)
        : this()
    {
        Perform(Cyl, Tor, Tol);
    }

    /// <summary>Intersects a cylinder and a torus (only when they are coaxial).</summary>
    public void Perform(in gp_Cylinder Cyl, in gp_Torus Tor, double Tol)
    {
        done = true;
        //
        double aRMin, aRMaj;
        //
        aRMin = Tor.MinorRadius();
        aRMaj = Tor.MajorRadius();
        if (aRMin >= aRMaj)
        {
            typeres = IntAna_NoGeometricSolution;
            return;
        }
        //
        gp_Ax1 aCylAx = Cyl.Axis();
        gp_Ax1 aTorAx = Tor.Axis();
        //
        gp_Lin aLin = new(aTorAx);
        gp_Pnt aLocCyl = Cyl.Location();
        //
        if (!aTorAx.IsParallel(aCylAx, myEPSILON_AXES_PARA)
            || (aLin.Distance(aLocCyl) > myEPSILON_DISTANCE))
        {
            typeres = IntAna_NoGeometricSolution;
            return;
        }
        //
        double aRCyl;
        //
        aRCyl = Cyl.Radius();
        if (((aRCyl + Tol) < (aRMaj - aRMin)) || ((aRCyl - Tol) > (aRMaj + aRMin)))
        {
            typeres = IntAna_Empty;
            return;
        }
        //
        typeres = IntAna_Circle;
        //
        double aDist = Math.Sqrt(Math.Abs(aRMin * aRMin - (aRCyl - aRMaj) * (aRCyl - aRMaj)));
        gp_XYZ aTorLoc = aTorAx.Location().XYZ();
        //
        dir1 = aTorAx.Direction();
        pt1.SetXYZ(aTorLoc + aDist * dir1.XYZ());
        param1 = aRCyl;
        nbint = 1;
        if ((aDist > Tol) && (aRCyl > (aRMaj - aRMin)) && (aRCyl < (aRMaj + aRMin)))
        {
            dir2 = dir1;
            pt2.SetXYZ(aTorLoc - aDist * dir2.XYZ());
            param2 = param1;
            nbint = 2;
        }
    }

    /// <summary>The intersection between a cone and a torus.</summary>
    public IntAna_QuadQuadGeo(in gp_Cone Con, in gp_Torus Tor, double Tol)
        : this()
    {
        Perform(Con, Tor, Tol);
    }

    /// <summary>Intersects a cone and a torus (only when they are coaxial).</summary>
    public void Perform(in gp_Cone Con, in gp_Torus Tor, double Tol)
    {
        done = true;
        //
        double aRMin, aRMaj;
        //
        aRMin = Tor.MinorRadius();
        aRMaj = Tor.MajorRadius();
        if (aRMin >= aRMaj)
        {
            typeres = IntAna_NoGeometricSolution;
            return;
        }
        //
        gp_Ax1 aConAx = Con.Axis();
        gp_Ax1 aTorAx = Tor.Axis();
        //
        gp_Lin aLin = new(aTorAx);
        gp_Pnt aConApex = Con.Apex();
        //
        if (!aTorAx.IsParallel(aConAx, myEPSILON_AXES_PARA)
            || (aLin.Distance(aConApex) > myEPSILON_DISTANCE))
        {
            typeres = IntAna_NoGeometricSolution;
            return;
        }
        //
        double anAngle, aDist, aDt;
        Span<double> aParam = stackalloc double[4];
        int i;
        gp_Pnt aTorLoc, aPCT = new(), aPN = new();
        Span<gp_Pnt> aPt = stackalloc gp_Pnt[4];
        Span<gp_Dir> aDir = [new gp_Dir(), new gp_Dir(), new gp_Dir(), new gp_Dir()];
        //
        anAngle = Con.SemiAngle();
        aTorLoc = aTorAx.Location();
        //
        aPN.SetXYZ(aTorLoc.XYZ() + aRMaj * Tor.YAxis().Direction().XYZ());
        gp_Dir aDN = new(new gp_Vec(aTorLoc, aPN));
        gp_Ax1 anAxCLRot = new(aConApex, aDN);
        gp_Lin aConL = aLin.Rotated(anAxCLRot, anAngle);
        gp_Dir aDL = aConL.Position().Direction();
        gp_Dir aXDir = Tor.XAxis().Direction();
        //
        typeres = IntAna_Empty;
        //
        for (i = 0; i < 2; ++i)
        {
            if (i != 0)
            {
                aXDir.Reverse();
            }
            aPCT.SetXYZ(aTorLoc.XYZ() + aRMaj * aXDir.XYZ());
            //
            aDist = aConL.Distance(aPCT);
            if (aDist > aRMin + Tol)
            {
                continue;
            }
            //
            typeres = IntAna_Circle;
            //
            gp_XYZ aPh = aPCT.XYZ() - aDist * aConL.Normal(aPCT).Direction().XYZ();
            aDt = Math.Sqrt(Math.Abs(aRMin * aRMin - aDist * aDist));
            //
            gp_Pnt aP = new();
            gp_XYZ aDVal = aDt * aDL.XYZ();
            aP.SetXYZ(aPh + aDVal);
            aParam[nbint] = aLin.Distance(aP);
            aPt[nbint].SetXYZ(aP.XYZ() - aParam[nbint] * aXDir.XYZ());
            aDir[nbint] = aTorAx.Direction();
            ++nbint;
            if ((aDist < aRMin) && (aDt > Tol))
            {
                aP.SetXYZ(aPh - aDVal);
                aParam[nbint] = aLin.Distance(aP);
                aPt[nbint].SetXYZ(aP.XYZ() - aParam[nbint] * aXDir.XYZ());
                aDir[nbint] = aDir[nbint - 1];
                ++nbint;
            }
        }
        //
        for (i = 0; i < nbint; ++i)
        {
            switch (i)
            {
                case 0:
                {
                    pt1 = aPt[i];
                    param1 = aParam[i];
                    dir1 = aDir[i];
                    break;
                }
                case 1:
                {
                    pt2 = aPt[i];
                    param2 = aParam[i];
                    dir2 = aDir[i];
                    break;
                }
                case 2:
                {
                    pt3 = aPt[i];
                    param3 = aParam[i];
                    dir3 = aDir[i];
                    break;
                }
                case 3:
                {
                    pt4 = aPt[i];
                    param4 = aParam[i];
                    dir4 = aDir[i];
                    break;
                }
                default:
                    break;
            }
        }
    }

    /// <summary>The intersection between a sphere and a torus.</summary>
    public IntAna_QuadQuadGeo(in gp_Sphere Sph, in gp_Torus Tor, double Tol)
        : this()
    {
        Perform(Sph, Tor, Tol);
    }

    /// <summary>Intersects a sphere and a torus (only when the center is on the axis).</summary>
    public void Perform(in gp_Sphere Sph, in gp_Torus Tor, double Tol)
    {
        done = true;
        //
        double aRMin, aRMaj;
        //
        aRMin = Tor.MinorRadius();
        aRMaj = Tor.MajorRadius();
        if (aRMin >= aRMaj)
        {
            typeres = IntAna_NoGeometricSolution;
            return;
        }
        //
        gp_Ax1 aTorAx = Tor.Axis();
        gp_Lin aLin = new(aTorAx);
        gp_Pnt aSphLoc = Sph.Location();
        //
        if (aLin.Distance(aSphLoc) > myEPSILON_DISTANCE)
        {
            typeres = IntAna_NoGeometricSolution;
            return;
        }
        //
        double aRSph, aDist;
        gp_Pnt aTorLoc = new();
        //
        gp_Dir aXDir = Tor.XAxis().Direction();
        aTorLoc.SetXYZ(aTorAx.Location().XYZ() + aRMaj * aXDir.XYZ());
        aRSph = Sph.Radius();
        //
        gp_Vec aVec12 = new(aTorLoc, aSphLoc);
        aDist = aVec12.Magnitude();
        if (((aDist - Tol) > (aRMin + aRSph)) || ((aDist + Tol) < Math.Abs(aRMin - aRSph)))
        {
            typeres = IntAna_Empty;
            return;
        }
        //
        typeres = IntAna_Circle;
        //
        double anAlpha, aBeta;
        //
        anAlpha = 0.5 * (aRMin * aRMin - aRSph * aRSph + aDist * aDist) / aDist;
        aBeta = Math.Sqrt(Math.Abs(aRMin * aRMin - anAlpha * anAlpha));
        //
        gp_Dir aDir12 = new(aVec12);
        gp_XYZ aPh = aTorLoc.XYZ() + anAlpha * aDir12.XYZ();
        gp_Dir aDC = Tor.YAxis().Direction() ^ aDir12;
        //
        gp_Pnt aP = new();
        gp_XYZ aDVal = aBeta * aDC.XYZ();
        aP.SetXYZ(aPh + aDVal);
        param1 = aLin.Distance(aP);
        pt1.SetXYZ(aP.XYZ() - param1 * aXDir.XYZ());
        dir1 = aTorAx.Direction();
        nbint = 1;
        if ((aDist < (aRSph + aRMin)) && (aDist > Math.Abs(aRSph - aRMin)) && (aDVal.Modulus() > Tol))
        {
            aP.SetXYZ(aPh - aDVal);
            param2 = aLin.Distance(aP);
            pt2.SetXYZ(aP.XYZ() - param2 * aXDir.XYZ());
            dir2 = dir1;
            nbint = 2;
        }
    }

    /// <summary>The intersection between two tori.</summary>
    public IntAna_QuadQuadGeo(in gp_Torus Tor1, in gp_Torus Tor2, double Tol)
        : this()
    {
        Perform(Tor1, Tor2, Tol);
    }

    /// <summary>Intersects two tori (only when they are coaxial).</summary>
    public void Perform(in gp_Torus Tor1, in gp_Torus Tor2, double Tol)
    {
        done = true;
        //
        double aRMin1, aRMin2, aRMaj1, aRMaj2;
        //
        aRMin1 = Tor1.MinorRadius();
        aRMaj1 = Tor1.MajorRadius();
        aRMin2 = Tor2.MinorRadius();
        aRMaj2 = Tor2.MajorRadius();
        //
        gp_Ax1 anAx1 = Tor1.Axis();
        gp_Ax1 anAx2 = Tor2.Axis();
        //
        gp_Pnt aLoc1 = anAx1.Location();
        gp_Pnt aLoc2 = anAx2.Location();
        //
        gp_Lin aL1 = new(anAx1);
        if (!anAx1.IsParallel(anAx2, myEPSILON_AXES_PARA) || (aL1.Distance(aLoc2) > myEPSILON_DISTANCE))
        {
            typeres = IntAna_NoGeometricSolution;
            return;
        }
        //
        if (aLoc1.IsEqual(aLoc2, Tol) && (Math.Abs(aRMin1 - aRMin2) <= Tol)
            && (Math.Abs(aRMaj1 - aRMaj2) <= Tol))
        {
            typeres = IntAna_Same;
            return;
        }
        //
        if (aRMin1 >= aRMaj1 || aRMin2 >= aRMaj2)
        {
            typeres = IntAna_NoGeometricSolution;
            return;
        }
        //
        double aDist;
        gp_Pnt aP1 = new(), aP2 = new();
        //
        gp_Dir aXDir1 = Tor1.XAxis().Direction();
        aP1.SetXYZ(aLoc1.XYZ() + aRMaj1 * aXDir1.XYZ());
        aP2.SetXYZ(aLoc2.XYZ() + aRMaj2 * aXDir1.XYZ());
        //
        gp_Vec aV12 = new(aP1, aP2);
        aDist = aV12.Magnitude();
        if (((aDist - Tol) > (aRMin1 + aRMin2)) || ((aDist + Tol) < Math.Abs(aRMin1 - aRMin2)))
        {
            typeres = IntAna_Empty;
            return;
        }
        //
        typeres = IntAna_Circle;
        //
        double anAlpha, aBeta;
        //
        anAlpha = 0.5 * (aRMin1 * aRMin1 - aRMin2 * aRMin2 + aDist * aDist) / aDist;
        aBeta = Math.Sqrt(Math.Abs(aRMin1 * aRMin1 - anAlpha * anAlpha));
        //
        gp_Dir aDir12 = new(aV12);
        gp_XYZ aPh = aP1.XYZ() + anAlpha * aDir12.XYZ();
        gp_Dir aDC = Tor1.YAxis().Direction() ^ aDir12;
        //
        gp_Pnt aP = new();
        gp_XYZ aDVal = aBeta * aDC.XYZ();
        aP.SetXYZ(aPh + aDVal);
        param1 = aL1.Distance(aP);
        pt1.SetXYZ(aP.XYZ() - param1 * aXDir1.XYZ());
        dir1 = anAx1.Direction();
        nbint = 1;
        if ((aDist < (aRMin1 + aRMin2)) && (aDist > Math.Abs(aRMin1 - aRMin2)) && aDVal.Modulus() > Tol)
        {
            aP.SetXYZ(aPh - aDVal);
            param2 = aL1.Distance(aP);
            pt2.SetXYZ(aP.XYZ() - param2 * aXDir1.XYZ());
            dir2 = dir1;
            nbint = 2;
        }
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone() => done;

    /// <summary>The type of intersection.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public IntAna_ResultType TypeInter()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return typeres;
    }

    /// <summary>The number of solutions.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public int NbSolutions()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return nbint;
    }

    /// <summary>The point solution of range n (for a Point or PointAndCircle result).</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    /// <exception cref="Standard_DomainError">n is out of range.</exception>
    public gp_Pnt Point(int n)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (n > nbint || n < 1)
        {
            throw new Standard_DomainError();
        }
        if (typeres == IntAna_PointAndCircle)
        {
            if (n != 1)
            {
                throw new Standard_DomainError();
            }
            if (param1 == 0.0)
            {
                return (pt1);
            }
            return (pt2);
        }
        else if (typeres == IntAna_Point)
        {
            if (n == 1)
            {
                return (pt1);
            }
            return (pt2);
        }

        // WNT (what can you expect from MicroSoft ?)
        return new gp_Pnt(0, 0, 0);
    }

    /// <summary>The line solution of range n.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    /// <exception cref="Standard_DomainError">n is out of range, or the result is not lines.</exception>
    public gp_Lin Line(int n)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if ((n > nbint) || (n < 1) || (typeres != IntAna_Line))
        {
            throw new Standard_DomainError();
        }
        if (n == 1)
        {
            return (new gp_Lin(pt1, dir1));
        }
        else
        {
            return (new gp_Lin(pt2, dir2));
        }
    }

    /// <summary>The circle solution of range n.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    /// <exception cref="Standard_DomainError">n is out of range, or the result is not circles.</exception>
    public gp_Circ Circle(int n)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (typeres == IntAna_PointAndCircle)
        {
            if (n != 1)
            {
                throw new Standard_DomainError();
            }
            if (param2 == 0.0)
            {
                return (new gp_Circ(DirToAx2(pt1, dir1), param1));
            }
            return (new gp_Circ(DirToAx2(pt2, dir2), param2));
        }
        else if ((n > nbint) || (n < 1) || (typeres != IntAna_Circle))
        {
            throw new Standard_DomainError();
        }
        if (n == 1)
        {
            return (new gp_Circ(DirToAx2(pt1, dir1), param1));
        }
        else if (n == 2)
        {
            return (new gp_Circ(DirToAx2(pt2, dir2), param2));
        }
        else if (n == 3)
        {
            return (new gp_Circ(DirToAx2(pt3, dir3), param3));
        }
        else
        {
            return (new gp_Circ(DirToAx2(pt4, dir4), param4));
        }
    }

    /// <summary>The ellipse solution of range n.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    /// <exception cref="Standard_DomainError">n is out of range, or the result is not ellipses.</exception>
    public gp_Elips Ellipse(int n)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if ((n > nbint) || (n < 1) || (typeres != IntAna_Ellipse))
        {
            throw new Standard_DomainError();
        }

        if (n == 1)
        {
            double R1 = param1, R2 = param1bis, aTmp;
            if (R1 < R2)
            {
                aTmp = R1;
                R1 = R2;
                R2 = aTmp;
            }
            gp_Ax2 anAx2 = new(pt1, dir1, dir2);
            gp_Elips anElips = new(anAx2, R1, R2);
            return anElips;
        }
        else
        {
            double R1 = param2, R2 = param2bis, aTmp;
            if (R1 < R2)
            {
                aTmp = R1;
                R1 = R2;
                R2 = aTmp;
            }
            gp_Ax2 anAx2 = new(pt2, dir2, dir1);
            gp_Elips anElips = new(anAx2, R1, R2);
            return anElips;
        }
    }

    /// <summary>The parabola solution (n must be 1).</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    /// <exception cref="Standard_DomainError">The result is not a parabola.</exception>
    /// <exception cref="Standard_OutOfRange">n is not 1.</exception>
    public gp_Parab Parabola(int n)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (typeres != IntAna_Parabola)
        {
            throw new Standard_DomainError();
        }
        if ((n > nbint) || (n != 1))
        {
            throw new Standard_OutOfRange();
        }
        return (new gp_Parab(new gp_Ax2(pt1, dir1, dir2), param1));
    }

    /// <summary>The hyperbola solution of range n.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    /// <exception cref="Standard_DomainError">n is out of range, or the result is not hyperbolas.</exception>
    public gp_Hypr Hyperbola(int n)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if ((n > nbint) || (n < 1) || (typeres != IntAna_Hyperbola))
        {
            throw new Standard_DomainError();
        }
        if (n == 1)
        {
            return (new gp_Hypr(new gp_Ax2(pt1, dir1, dir2), param1, param1bis));
        }
        else
        {
            return (new gp_Hypr(new gp_Ax2(pt2, dir1, dir2.Reversed()), param2, param2bis));
        }
    }

    /// <summary>True if the two cones have a common generatrix (set by the cone/cone intersection).</summary>
    public bool HasCommonGen() => myCommonGen;

    /// <summary>The characteristic point of the intersection of two cones with a common generatrix.</summary>
    public gp_Pnt PChar() => myPChar;

    //=================================================================================================

    private static void RefineDir(ref gp_Dir aDir)
    {
        int k, m, n;
        Span<double> aC = stackalloc double[3];
        //
        aDir.Coord(out aC[0], out aC[1], out aC[2]);
        //
        m = 0;
        n = 0;
        for (k = 0; k < 3; ++k)
        {
            if (aC[k] == 1.0 || aC[k] == -1.0)
            {
                ++m;
            }
            else if (aC[k] != 0.0)
            {
                ++n;
            }
        }
        //
        if (m != 0 && n != 0)
        {
            double aEps, aR1, aR2, aNum;
            //
            aEps = RealEpsilon();
            aR1 = 1.0 - aEps;
            aR2 = 1.0 + aEps;
            //
            for (k = 0; k < 3; ++k)
            {
                m = (aC[k] > 0.0) ? 1 : 0;
                aNum = (m != 0) ? aC[k] : -aC[k];
                if (aNum > aR1 && aNum < aR2)
                {
                    if (m != 0)
                    {
                        aC[k] = 1.0;
                    }
                    else
                    {
                        aC[k] = -1.0;
                    }
                    //
                    aC[(k + 1) % 3] = 0.0;
                    aC[(k + 2) % 3] = 0.0;
                    break;
                }
            }
            aDir.SetCoord(aC[0], aC[1], aC[2]);
        }
    }
}
