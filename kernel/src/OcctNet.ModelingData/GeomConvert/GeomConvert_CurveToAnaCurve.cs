// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_CurveToAnaCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_CurveToAnaCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2001-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Occt.GeomConvert_ConvType;

namespace Occt;

/// <summary>
/// Recognises a curve as a line, a circle or an ellipse within a tolerance, and builds that analytical curve with
/// the parameters corresponding to a range of the original.
/// </summary>
/// <remarks>
/// The static Compute* functions return the input curve itself when it already is of the requested type (as OCCT
/// returns the handle), and a new curve otherwise; ConvertToAnalytical works on the basis of a trimmed curve.
/// </remarks>
public sealed class GeomConvert_CurveToAnaCurve
{
    private Geom_Curve? myCurve;
    private double myGap;
    private GeomConvert_ConvType myConvType;
    private CurveKind myTarget;

    /// <summary>An empty converter (MinGap, target line); <see cref="Init"/> gives it a curve.</summary>
    public GeomConvert_CurveToAnaCurve()
    {
        myGap = global::Kerf.Engine.Foundation.Precision.Infinite();
        myConvType = GeomConvert_MinGap;
        myTarget = Line;
    }

    /// <summary>A converter of C (MinGap, target line).</summary>
    public GeomConvert_CurveToAnaCurve(Geom_Curve C)
    {
        myGap = global::Kerf.Engine.Foundation.Precision.Infinite();
        myConvType = GeomConvert_MinGap;
        myTarget = Line;
        myCurve = C;
    }

    /// <summary>Sets the curve to convert and resets the gap.</summary>
    public void Init(Geom_Curve C)
    {
        myCurve = C;
        myGap = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>
    /// Converts the range [F, L] of the curve to an analytical curve within theTol, according to the conversion
    /// type and target. On success, sets theResultCurve and returns true; NewF and NewL are the corresponding
    /// parameters on the result.
    /// </summary>
    public bool ConvertToAnalytical(double tol,
                                    ref Geom_Curve? theResultCurve,
                                    double F,
                                    double L,
                                    out double NewF,
                                    out double NewL)
    {
        if (myCurve is null)
        {
            NewF = 0.0;
            NewL = 0.0;
            return false;
        }

        Geom_Curve aCurve = myCurve;
        while (aCurve is Geom_TrimmedCurve)
        {
            Geom_TrimmedCurve aTrimmed = (Geom_TrimmedCurve)aCurve;
            aCurve = aTrimmed.BasisCurve();
        }

        Geom_Curve? C =
            ComputeCurve(aCurve, tol, F, L, out NewF, out NewL, ref myGap, myConvType, myTarget);

        if (C is null)
        {
            return false;
        }

        theResultCurve = C;
        return true;
    }

    /// <summary>
    /// True if the points lie within tolerance of the line through the two farthest apart; Deviation is then the
    /// largest distance. False (Deviation unchanged) for fewer than 2 points or all points confused.
    /// </summary>
    public static bool IsLinear(NCollection_Array1<Point3D> aPoles,
                                double tolerance,
                                ref double Deviation)
    {
        int nbPoles = aPoles.Length();
        if (nbPoles < 2)
        {
            return false;
        }

        double dMax = 0;
        int iMax1 = 0, iMax2 = 0;

        int i;
        for (i = 1; i < nbPoles; i++)
        {
            for (int j = i + 1; j <= nbPoles; j++)
            {
                double dist = aPoles[i].SquareDistance(aPoles[j]);
                if (dist > dMax)
                {
                    dMax = dist;
                    iMax1 = i;
                    iMax2 = j;
                }
            }
        }

        if (dMax < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return false;
        }

        double tol2 = tolerance * tolerance;
        var avec = new Vector3D(aPoles[iMax1], aPoles[iMax2]);
        var adir = new Direction3D(avec);
        var alin = new Line3D(aPoles[iMax1], adir);

        double aMax = 0.0;
        for (i = 1; i <= nbPoles; i++)
        {
            double dist = alin.SquareDistance(aPoles[i]);
            if (dist > tol2)
            {
                return false;
            }

            if (dist > aMax)
            {
                aMax = dist;
            }
        }

        Deviation = Math.Sqrt(aMax);

        return true;
    }

    /// <summary>The line from P1 through P2; cf and cl are the parameters of P1 and P2 on it.</summary>
    public static Line3D GetLine(in Point3D P1, in Point3D P2, out double cf, out double cl)
    {
        var avec = new Vector3D(P1, P2);
        var adir = new Direction3D(avec);
        var alin = new Line3D(P1, adir);
        cf = global::Kerf.Engine.Geometry.ElCLib.Parameter(alin, P1);
        cl = global::Kerf.Engine.Geometry.ElCLib.Parameter(alin, P2);
        return alin;
    }

    /// <summary>
    /// The line approximating curve over [c1, c2] within tolerance (from its poles for Bezier and B-spline curves,
    /// from 23 points otherwise), or null. cf, cl and Deviation are set on success (and cf, cl on some failures).
    /// </summary>
    public static Geom_Line? ComputeLine(Geom_Curve? curve,
                                         double tolerance,
                                         double c1,
                                         double c2,
                                         ref double cf,
                                         ref double cl,
                                         ref double Deviation)
    {
        Geom_Line? line = null;
        if (curve is null)
        {
            return line;
        }

        line = curve as Geom_Line; // who knows
        if (line is not null)
        {
            cf = c1;
            cl = c2;
            Deviation = 0.0;
            return line;
        }

        Point3D P1 = curve.Value(c1);
        Point3D P2 = curve.Value(c2);
        if (P1.SquareDistance(P2) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return line;
        }

        cf = c1;
        cl = c2;

        NCollection_HArray1<Point3D> Poles;
        int nbPoles;
        Geom_BSplineCurve? bsc = curve as Geom_BSplineCurve;
        if (bsc is not null)
        {
            nbPoles = bsc.NbPoles();
            Poles = new NCollection_HArray1<Point3D>(bsc.InternalPoles());
        }
        else
        {
            Geom_BezierCurve? bzc = curve as Geom_BezierCurve;
            if (bzc is not null)
            {
                nbPoles = bzc.NbPoles();
                Poles = new NCollection_HArray1<Point3D>(bzc.InternalPoles());
            }
            else
            {
                nbPoles = 23;
                Poles = new NCollection_HArray1<Point3D>(1, nbPoles);
                double dt = (c2 - c1) / (nbPoles - 1);
                Poles.SetValue(1, P1);
                Poles.SetValue(nbPoles, P2);
                int i;
                for (i = 2; i < nbPoles; ++i)
                {
                    Poles.SetValue(i, curve.Value(c1 + (i - 1) * dt));
                }
            }
        }

        if (!IsLinear(Poles, tolerance, ref Deviation))
        {
            return line; // non
        }

        Line3D alin = GetLine(P1, P2, out cf, out cl);
        line = new Geom_Line(alin);
        return line;
    }

    /// <summary>
    /// The circle through P0, P1 and P2, its X axis towards P0. False (Circ unchanged) for points beyond
    /// sqrt(Precision::Infinite()), aligned points or a null radius.
    /// </summary>
    public static bool GetCircle(ref Circle3D crc,
                                 in Point3D P0,
                                 in Point3D P1,
                                 in Point3D P2)
    {
        //  Control if points are not aligned (should be done by MakeCirc
        double aMaxCoord = Math.Sqrt(global::Kerf.Engine.Foundation.Precision.Infinite());
        if (Math.Abs(P0.X()) > aMaxCoord || Math.Abs(P0.Y()) > aMaxCoord || Math.Abs(P0.Z()) > aMaxCoord)
        {
            return false;
        }

        if (Math.Abs(P1.X()) > aMaxCoord || Math.Abs(P1.Y()) > aMaxCoord || Math.Abs(P1.Z()) > aMaxCoord)
        {
            return false;
        }

        if (Math.Abs(P2.X()) > aMaxCoord || Math.Abs(P2.Y()) > aMaxCoord || Math.Abs(P2.Z()) > aMaxCoord)
        {
            return false;
        }

        //  Building the circle
        var mkc = new gce_MakeCirc(P0, P1, P2);
        if (!mkc.IsDone())
        {
            return false;
        }

        crc = mkc.Value();
        if (crc.Radius() < Constants.Resolution())
        {
            return false;
        }

        //  Align to P0
        Point3D PC = crc.Location();
        Frame3D axe = crc.Position();
        var VX = new Vector3D(PC, P0);
        axe.SetXDirection(new Direction3D(VX));
        crc.SetPosition(axe);
        return true;
    }

    /// <summary>
    /// The circle through the points at c1, (2c1 + c2) / 3 and (c1 + 2c2) / 3, if 21 points of [c1, c2] are within
    /// tol of it; null otherwise. cf and cl are the parameters on the circle (cf in [0, 2Pi)).
    /// </summary>
    public static Geom_Curve? ComputeCircle(Geom_Curve c3d,
                                            double tol,
                                            double c1,
                                            double c2,
                                            ref double cf,
                                            ref double cl,
                                            ref double Deviation)
    {
        if (c3d is Geom_Circle)
        {
            cf = c1;
            cl = c2;
            Deviation = 0.0;
            Geom_Circle aCirc = (Geom_Circle)c3d;
            return aCirc;
        }

        Geom_Circle? circ = null;
        Point3D P0, P1, P2;
        double ca = (c1 + c1 + c2) / 3;
        double cb = (c1 + c2 + c2) / 3;
        P0 = c3d.Value(c1);
        P1 = c3d.Value(ca);
        P2 = c3d.Value(cb);

        var crc = new Circle3D();
        if (!GetCircle(ref crc, P0, P1, P2))
        {
            return circ;
        }

        //  Reste a controler que c est bien un cercle : prendre 20 points
        double du = (c2 - c1) / 20;
        int i;
        double aMax = 0.0;
        for (i = 0; i <= 20; i++)
        {
            double u = c1 + (du * i);
            Point3D PP = c3d.Value(u);
            double dist = crc.Distance(PP);
            if (dist > tol)
            {
                return circ; // not done
            }

            if (dist > aMax)
            {
                aMax = dist;
            }
        }

        Deviation = aMax;

        // defining the parameters
        double PI2 = 2 * M_PI;

        cf = global::Kerf.Engine.Geometry.ElCLib.Parameter(crc, c3d.Value(c1));
        cf = global::Kerf.Engine.Geometry.ElCLib.InPeriod(cf, 0.0, PI2);

        // first parameter should be closed to zero

        if (Math.Abs(cf) < global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(PI2 - cf) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            cf = 0.0;
        }

        double cm = global::Kerf.Engine.Geometry.ElCLib.Parameter(crc, c3d.Value((c1 + c2) / 2.0));
        cm = global::Kerf.Engine.Geometry.ElCLib.InPeriod(cm, cf, cf + PI2);

        cl = global::Kerf.Engine.Geometry.ElCLib.Parameter(crc, c3d.Value(c2));
        cl = global::Kerf.Engine.Geometry.ElCLib.InPeriod(cl, cm, cm + PI2);

        circ = new Geom_Circle(crc);
        return circ;
    }

    private static bool IsArrayPntPlanar(NCollection_HArray1<Point3D> HAP,
                                         ref Direction3D Norm,
                                         double prec)
    {
        int size = HAP.Length();
        if (size < 3)
        {
            return false;
        }

        Point3D P1 = HAP.Value(1);
        Point3D P2 = HAP.Value(2);
        Point3D P3 = HAP.Value(3);
        double dist1 = P1.Distance(P2);
        double dist2 = P1.Distance(P3);
        if (dist1 < prec || dist2 < prec)
        {
            return false;
        }

        var V1 = new Vector3D(P1, P2);
        var V2 = new Vector3D(P1, P3);
        if (V1.IsParallel(V2, prec))
        {
            return false;
        }

        Vector3D NV = V1.Crossed(V2);

        int i;
        for (i = 1; i <= 3; ++i)
        {
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(NV.Coord(i)))
            {
                return false;
            }
        }

        if (NV.Magnitude() < Constants.Resolution())
        {
            return false;
        }

        if (size > 3)
        {
            for (i = 4; i <= size; i++)
            {
                Point3D PN = HAP.Value(i);
                dist1 = P1.Distance(PN);
                if (dist1 < prec || global::Kerf.Engine.Foundation.Precision.IsInfinite(dist1))
                {
                    return false;
                }

                var VN = new Vector3D(P1, PN);
                if (!NV.IsNormal(VN, prec))
                {
                    return false;
                }
            }
        }

        Norm = new Direction3D(NV);
        return true;
    }

    private static bool ConicDefinition(double a,
                                        double b1,
                                        double c,
                                        double d1,
                                        double e1,
                                        double f,
                                        bool IsParab,
                                        bool IsEllip,
                                        ref Point3D Center,
                                        ref Direction3D MainAxis,
                                        ref double Rmin,
                                        ref double Rmax)
    {
        double Xcen = 0.0, Ycen = 0.0, Xax = 0.0, Yax = 0.0;
        double b, d, e;
        //  conic : a*x2 + 2*b*x*y + c*y2 + 2*d*x + 2*e*y + f = 0.
        // Equation (a,b,c,d,e,f);
        b = b1 / 2.0;
        d = d1 / 2.0;
        e = e1 / 2.0; // change of variable

        double eps = 1.0E-08; // ?? same as ComputedForm

        if (IsParab)
        {
        }
        else
        {
            //   -> Central conic, general case
            //  We use the determinants of the matrices:
            //               | a b d |
            //  gdet (3x3) = | b c e |  and pdet (2x2) = | a b |
            //               | d e f |                    | b c |
            double gdet = a * c * f + 2 * b * d * e - c * d * d - a * e * e - b * b * f;
            double pdet = a * c - b * b;

            Xcen = (b * e - c * d) / pdet;
            Ycen = (b * d - a * e) / pdet;

            double term1 = a - c;
            double term2 = 2 * b;
            double cos2t;
            double auxil;

            if (Math.Abs(term2) <= eps && Math.Abs(term1) <= eps)
            {
                cos2t = 1.0;
                auxil = 0.0;
            }
            else
            {
                if (Math.Abs(term1) < eps)
                {
                    return false;
                }

                double t2d = term2 / term1; // skl 21.11.2001
                cos2t = 1.0 / Math.Sqrt(1 + t2d * t2d);
                auxil = Math.Sqrt(term1 * term1 + term2 * term2);
            }

            // OCCT: cos2t and sint are taken positive, so the sign of the rotation is lost and the axis is
            // always within [0, 45] degrees of X: ellipses rotated otherwise in the fitting frame are rejected
            double cost = Math.Sqrt((1 + cos2t) / 2.0);
            double sint = Math.Sqrt((1 - cos2t) / 2.0);

            double aprim = (a + c + auxil) / 2.0;
            double cprim = (a + c - auxil) / 2.0;

            if (Math.Abs(aprim) < Constants.Resolution() || Math.Abs(cprim) < Constants.Resolution())
            {
                return false;
            }

            term1 = -gdet / (aprim * pdet);
            term2 = -gdet / (cprim * pdet);

            if (IsEllip)
            {
                if (term1 <= eps || term2 <= eps)
                {
                    return false;
                }

                Xax = cost;
                Yax = sint;
                Rmin = Math.Sqrt(term1);
                Rmax = Math.Sqrt(term2);
                if (Rmax < Rmin)
                {
                    Rmax = Math.Sqrt(term1);
                    Rmin = Math.Sqrt(term2);
                }
            }
            else if (term1 <= eps)
            {
                if (-term1 <= eps || term2 <= eps)
                {
                    return false;
                }

                Xax = -sint;
                Yax = cost;
                Rmin = Math.Sqrt(-term1);
                Rmax = Math.Sqrt(term2);
            }
            else
            {
                if (term1 <= eps || -term2 <= eps)
                {
                    return false;
                }

                Xax = cost;
                Yax = sint;
                Rmin = Math.Sqrt(-term2);
                Rmax = Math.Sqrt(term1);
            }
        }

        Center.SetCoord(Xcen, Ycen, 0.0);
        MainAxis.SetCoord(Xax, Yax, 0.0);
        return true;
    }

    /// <summary>
    /// The ellipse through 5 points of [c1, c2] (of its first four fifths if the curve is closed), if they are
    /// coplanar and 20 points of the range are within tol of it; null otherwise (and for near-circles, which
    /// ComputeCircle recognises). cf and cl are the parameters on the ellipse (cf in [0, 2Pi)).
    /// </summary>
    public static Geom_Curve? ComputeEllipse(Geom_Curve c3d,
                                             double tol,
                                             double c1,
                                             double c2,
                                             ref double cf,
                                             ref double cl,
                                             ref double Deviation)
    {
        if (c3d is Geom_Ellipse)
        {
            cf = c1;
            cl = c2;
            Deviation = 0.0;
            Geom_Ellipse anElips = (Geom_Ellipse)c3d;
            return anElips;
        }

        Geom_Curve? res = null;
        double prec = global::Kerf.Engine.Foundation.Precision.PConfusion();

        double AF, BF, CF, DF, EF, Q1, Q2, Q3, c2n;
        int i;

        Point3D PStart = c3d.Value(c1);
        Point3D PEnd = c3d.Value(c2);

        bool IsClos = PStart.Distance(PEnd) < prec;
        if (IsClos)
        {
            c2n = c2 - (c2 - c1) / 5;
        }
        else
        {
            c2n = c2;
        }

        //
        var aBC = new Xyz();
        var AP = new NCollection_HArray1<Point3D>(1, 5);
        AP.SetValue(1, PStart);
        aBC += PStart.XYZ();
        double dc = (c2n - c1) / 4;
        for (i = 1; i < 5; i++)
        {
            Point3D aP = c3d.Value(c1 + dc * i);
            AP.SetValue(i + 1, aP);
            aBC += aP.XYZ();
        }

        aBC /= 5;
        aBC *= -1;
        var aTrans = new Vector3D(aBC);
        for (i = 1; i <= 5; ++i)
        {
            AP[i].Translate(aTrans);
        }

        var ndir = new Direction3D();
        if (!IsArrayPntPlanar(AP, ref ndir, prec))
        {
            return res;
        }

        if (Math.Abs(ndir.X()) < Constants.Resolution() && Math.Abs(ndir.Y()) < Constants.Resolution()
            && Math.Abs(ndir.Z()) < Constants.Resolution())
        {
            return res;
        }

        var AX = new CoordinateSystem(new Point3D(0, 0, 0), ndir);
        var Tr = new Transform3D();
        Tr.SetTransformation(AX);
        Transform3D Tr2 = Tr.Inverted();

        var Dt = new Matrix(1, 5, 1, 5);
        math_Vector F = new(1, 5), Sl = new(1, 5);

        double XN, YN, ZN = 0.0;
        Point3D PT, PP;
        for (i = 1; i <= 5; i++)
        {
            PT = AP.Value(i).Transformed(Tr);
            PT.Coord(out XN, out YN, out ZN);
            Dt[i, 1] = XN * XN;
            Dt[i, 2] = XN * YN;
            Dt[i, 3] = YN * YN;
            Dt[i, 4] = XN;
            Dt[i, 5] = YN;
            F[i] = -1.0;
        }

        var aSolver = new Gauss(Dt);
        if (!aSolver.IsDone)
        {
            return res;
        }

        aSolver.Solve(F, Sl);

        AF = Sl[1];
        BF = Sl[2];
        CF = Sl[3];
        DF = Sl[4];
        EF = Sl[5];

        Q1 = AF * CF + BF * EF * DF / 4 - CF * DF * DF / 4 - BF * BF / 4 - AF * EF * EF / 4;
        Q2 = AF * CF - BF * BF / 4;
        Q3 = AF + CF;

        double Rmax = 0.0, Rmin = 0.0;
        var Center = new Point3D();
        var MainAxis = new Direction3D();
        bool IsParab = false, IsEllip = false;

        if (Q2 > 0 && Q1 * Q3 < 0)
        {
            // ellipse
            IsEllip = true;
            if (ConicDefinition(AF, BF, CF, DF, EF, 1.0, IsParab, IsEllip, ref Center, ref MainAxis, ref Rmin, ref Rmax))
            {
                // create ellipse
                if (Rmax - Rmin < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    return res; // really it is circle, which must be recognized in other method
                }

                aTrans *= -1;
                Center.SetZ(ZN);
                Point3D NewCenter = Center.Transformed(Tr2);
                var Ptmp = new Point3D(Center.X() + MainAxis.X() * 10,
                                      Center.Y() + MainAxis.Y() * 10,
                                      Center.Z() + MainAxis.Z() * 10);
                Point3D NewPtmp = Ptmp.Transformed(Tr2);
                var NewMainAxis = new Direction3D(NewPtmp.X() - NewCenter.X(),
                                             NewPtmp.Y() - NewCenter.Y(),
                                             NewPtmp.Z() - NewCenter.Z());
                var ax2 = new Frame3D(NewCenter, ndir, NewMainAxis);

                var anEllipse = new Ellipse3D(ax2, Rmax, Rmin);
                anEllipse.Translate(aTrans);
                var gell = new Geom_Ellipse(anEllipse);

                // test for 20 points
                double param2 = 0;
                dc = (c2 - c1) / 20;
                for (i = 1; i <= 20; i++)
                {
                    PP = c3d.Value(c1 + i * dc);
                    double aPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(anEllipse, PP);
                    double dist = gell.Value(aPar).Distance(PP);
                    if (dist > tol)
                    {
                        return res; // not done
                    }

                    if (dist > param2)
                    {
                        param2 = dist;
                    }
                }

                Deviation = param2;

                double PI2 = 2 * M_PI;
                cf = global::Kerf.Engine.Geometry.ElCLib.Parameter(anEllipse, c3d.Value(c1));
                cf = global::Kerf.Engine.Geometry.ElCLib.InPeriod(cf, 0.0, PI2);

                // first parameter should be closed to zero

                if (Math.Abs(cf) < global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(PI2 - cf) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    cf = 0.0;
                }

                double cm = global::Kerf.Engine.Geometry.ElCLib.Parameter(anEllipse, c3d.Value((c1 + c2) / 2.0));
                cm = global::Kerf.Engine.Geometry.ElCLib.InPeriod(cm, cf, cf + PI2);

                cl = global::Kerf.Engine.Geometry.ElCLib.Parameter(anEllipse, c3d.Value(c2));
                cl = global::Kerf.Engine.Geometry.ElCLib.InPeriod(cl, cm, cm + PI2);

                res = gell;
            }
        }

        // OCCT leaves the hyperbola (Q2 < 0, Q1 != 0) and parabola (Q2 == 0, Q1 != 0) cases commented out
        return res;
    }

    /// <summary>
    /// Converts theC3d over [c1, c2] to a line, circle or ellipse within tolerance, according to theConvType
    /// (Target: theTarget only; Simplest: the first of line, circle, ellipse that fits; MinGap: the one of
    /// smallest deviation). Returns null if none fits; cf and cl are the parameters on the result (c1, c2 on
    /// failure) and theGap its deviation (-1 when MinGap finds nothing, unchanged for a null curve).
    /// </summary>
    public static Geom_Curve? ComputeCurve(Geom_Curve? theC3d,
                                           double tolerance,
                                           double c1,
                                           double c2,
                                           out double cf,
                                           out double cl,
                                           ref double theGap,
                                           GeomConvert_ConvType theConvType = GeomConvert_MinGap,
                                           CurveKind theTarget = Line)
    {
        cf = c1;
        cl = c2;
        var newc3d = new Geom_Curve?[3];
        Geom_Curve? c3d = theC3d;
        if (c3d is null)
        {
            return c3d;
        }

        Point3D P1 = c3d.Value(c1);
        Point3D P2 = c3d.Value(c2);
        Point3D P3 = c3d.Value(c1 + (c2 - c1) / 2);
        Span<double> d = [RealLast(), RealLast(), RealLast()];
        Span<double> fp = [0.0, 0.0, 0.0];
        Span<double> lp = [0.0, 0.0, 0.0];

        if (c3d is Geom_TrimmedCurve)
        {
            Geom_TrimmedCurve aTc = (Geom_TrimmedCurve)c3d;
            c3d = aTc.BasisCurve();
        }

        if (theConvType == GeomConvert_Target)
        {
            theGap = RealLast();
            if (theTarget == Line)
            {
                newc3d[0] = ComputeLine(c3d, tolerance, c1, c2, ref fp[0], ref lp[0], ref theGap);
                cf = fp[0];
                cl = lp[0];
                return newc3d[0];
            }

            if (theTarget == Circle)
            {
                newc3d[1] = ComputeCircle(c3d, tolerance, c1, c2, ref fp[1], ref lp[1], ref theGap);
                cf = fp[1];
                cl = lp[1];
                return newc3d[1];
            }

            if (theTarget == Ellipse)
            {
                newc3d[2] = ComputeEllipse(c3d, tolerance, c1, c2, ref fp[2], ref lp[2], ref theGap);
                cf = fp[2];
                cl = lp[2];
                return newc3d[2];
            }
        }

        //
        if (theConvType == GeomConvert_Simplest)
        {
            theGap = RealLast();
            newc3d[0] = ComputeLine(c3d, tolerance, c1, c2, ref fp[0], ref lp[0], ref theGap);
            if (newc3d[0] is not null)
            {
                cf = fp[0];
                cl = lp[0];
                return newc3d[0];
            }

            theGap = RealLast();
            newc3d[1] = ComputeCircle(c3d, tolerance, c1, c2, ref fp[1], ref lp[1], ref theGap);
            if (newc3d[1] is not null)
            {
                cf = fp[1];
                cl = lp[1];
                return newc3d[1];
            }

            theGap = RealLast();
            newc3d[2] = ComputeEllipse(c3d, tolerance, c1, c2, ref fp[2], ref lp[2], ref theGap);
            if (newc3d[2] is not null)
            {
                cf = fp[2];
                cl = lp[2];
                return newc3d[2];
            }

            // Conversion failed, returns null curve
            return newc3d[0];
        }

        //  theConvType == GeomConvert_MinGap
        // recognition in case of small curve
        int imin = -1;
        if ((P1.Distance(P2) < 2 * tolerance) && (P1.Distance(P3) < 2 * tolerance))
        {
            newc3d[1] = ComputeCircle(c3d, tolerance, c1, c2, ref fp[1], ref lp[1], ref d[1]);
            newc3d[0] = ComputeLine(c3d, tolerance, c1, c2, ref fp[0], ref lp[0], ref d[0]);
            imin = 1;
            if (newc3d[1] is null || d[0] < d[1])
            {
                imin = 0;
            }
        }
        else
        {
            d[0] = RealLast();
            newc3d[0] = ComputeLine(c3d, tolerance, c1, c2, ref fp[0], ref lp[0], ref d[0]);
            double tol = Min(tolerance, d[0]);
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(c1) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(c2))
            {
                d[1] = RealLast();
                newc3d[1] = ComputeCircle(c3d, tol, c1, c2, ref fp[1], ref lp[1], ref d[1]);
                tol = Min(tol, d[1]);
                d[2] = RealLast();
                newc3d[2] = ComputeEllipse(c3d, tol, c1, c2, ref fp[2], ref lp[2], ref d[2]);
            }

            double dd = RealLast();
            for (int i = 0; i < 3; ++i)
            {
                if (newc3d[i] is null)
                {
                    continue;
                }

                if (d[i] < dd)
                {
                    dd = d[i];
                    imin = i;
                }
            }
        }

        if (imin >= 0)
        {
            cf = fp[imin];
            cl = lp[imin];
            theGap = d[imin];
            return newc3d[imin];
        }
        else
        {
            cf = c1;
            cl = c2;
            theGap = -1.0;
            return newc3d[0]; // must be null curve;
        }
    }

    /// <summary>The deviation of the result of the last ConvertToAnalytical from the original curve.</summary>
    public double Gap() => myGap;

    /// <summary>The conversion type.</summary>
    public GeomConvert_ConvType GetConvType() => myConvType;

    /// <summary>Sets the conversion type.</summary>
    public void SetConvType(GeomConvert_ConvType theConvType) => myConvType = theConvType;

    /// <summary>The target curve type (for GeomConvert_Target).</summary>
    public CurveKind GetTarget() => myTarget;

    /// <summary>Sets the target curve type.</summary>
    public void SetTarget(CurveKind theTarget) => myTarget = theTarget;
}
