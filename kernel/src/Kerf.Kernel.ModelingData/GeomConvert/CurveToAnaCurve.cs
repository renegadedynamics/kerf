// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_CurveToAnaCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_CurveToAnaCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2001-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;
using static Kerf.Engine.Geometry.Conversion.ConvType;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Recognises a curve as a line, a circle or an ellipse within a tolerance, and builds that analytical curve with
/// the parameters corresponding to a range of the original.
/// </summary>
/// <remarks>
/// The static Compute* functions return the input curve itself when it already is of the requested type (as OCCT
/// returns the handle), and a new curve otherwise; ConvertToAnalytical works on the basis of a trimmed curve.
/// </remarks>
internal sealed class CurveToAnaCurve
{
    private Curve3D? _curve;
    private double _gap;
    private ConvType _convType;
    private CurveKind _target;

    /// <summary>An empty converter (MinGap, target line); <see cref="Init"/> gives it a curve.</summary>
    public CurveToAnaCurve()
    {
        _gap = global::Kerf.Engine.Foundation.Precision.Infinite();
        _convType = MinGap;
        _target = Line;
    }

    /// <summary>A converter of C (MinGap, target line).</summary>
    public CurveToAnaCurve(Curve3D c)
    {
        _gap = global::Kerf.Engine.Foundation.Precision.Infinite();
        _convType = MinGap;
        _target = Line;
        _curve = c;
    }

    /// <summary>Sets the curve to convert and resets the gap.</summary>
    public void Init(Curve3D c)
    {
        _curve = c;
        _gap = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>
    /// Converts the range [F, L] of the curve to an analytical curve within theTol, according to the conversion
    /// type and target. On success, sets theResultCurve and returns true; NewF and NewL are the corresponding
    /// parameters on the result.
    /// </summary>
    public bool ConvertToAnalytical(double tol,
                                    ref Curve3D? resultCurve,
                                    double f,
                                    double l,
                                    out double newF,
                                    out double newL)
    {
        if (_curve is null)
        {
            newF = 0.0;
            newL = 0.0;
            return false;
        }

        Curve3D curve = _curve;
        while (curve is TrimmedCurve3D)
        {
            TrimmedCurve3D trimmed = (TrimmedCurve3D)curve;
            curve = trimmed.BasisCurve;
        }

        Curve3D? c =
            ComputeCurve(curve, tol, f, l, out newF, out newL, ref _gap, _convType, _target);

        if (c is null)
        {
            return false;
        }

        resultCurve = c;
        return true;
    }

    /// <summary>
    /// True if the points lie within tolerance of the line through the two farthest apart; Deviation is then the
    /// largest distance. False (Deviation unchanged) for fewer than 2 points or all points confused.
    /// </summary>
    public static bool IsLinear(NCollection_Array1<Point3D> aPoles,
                                double tolerance,
                                ref double deviation)
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
                double dist = aPoles[i].SquareDistanceTo(aPoles[j]);
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

        double max = 0.0;
        for (i = 1; i <= nbPoles; i++)
        {
            double dist = alin.SquareDistanceTo(aPoles[i]);
            if (dist > tol2)
            {
                return false;
            }

            if (dist > max)
            {
                max = dist;
            }
        }

        deviation = Math.Sqrt(max);

        return true;
    }

    /// <summary>The line from P1 through P2; cf and cl are the parameters of P1 and P2 on it.</summary>
    public static Line3D GetLine(in Point3D p1, in Point3D p2, out double cf, out double cl)
    {
        var avec = new Vector3D(p1, p2);
        var adir = new Direction3D(avec);
        var alin = new Line3D(p1, adir);
        cf = global::Kerf.Engine.Geometry.ElCLib.Parameter(alin, p1);
        cl = global::Kerf.Engine.Geometry.ElCLib.Parameter(alin, p2);
        return alin;
    }

    /// <summary>
    /// The line approximating curve over [c1, c2] within tolerance (from its poles for Bezier and B-spline curves,
    /// from 23 points otherwise), or null. cf, cl and Deviation are set on success (and cf, cl on some failures).
    /// </summary>
    public static LineCurve3D? ComputeLine(Curve3D? curve,
                                         double tolerance,
                                         double c1,
                                         double c2,
                                         ref double cf,
                                         ref double cl,
                                         ref double deviation)
    {
        LineCurve3D? line = null;
        if (curve is null)
        {
            return line;
        }

        line = curve as LineCurve3D; // who knows
        if (line is not null)
        {
            cf = c1;
            cl = c2;
            deviation = 0.0;
            return line;
        }

        Point3D p1 = curve.Value(c1);
        Point3D p2 = curve.Value(c2);
        if (p1.SquareDistanceTo(p2) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return line;
        }

        cf = c1;
        cl = c2;

        NCollection_HArray1<Point3D> poles;
        int nbPoles;
        BSplineCurve3D? bsc = curve as BSplineCurve3D;
        if (bsc is not null)
        {
            nbPoles = bsc.PoleCount;
            poles = new NCollection_HArray1<Point3D>(bsc.InternalPoles());
        }
        else
        {
            BezierCurve3D? bzc = curve as BezierCurve3D;
            if (bzc is not null)
            {
                nbPoles = bzc.PoleCount;
                poles = new NCollection_HArray1<Point3D>(bzc.InternalPoles());
            }
            else
            {
                nbPoles = 23;
                poles = new NCollection_HArray1<Point3D>(1, nbPoles);
                double dt = (c2 - c1) / (nbPoles - 1);
                poles.SetValue(1, p1);
                poles.SetValue(nbPoles, p2);
                int i;
                for (i = 2; i < nbPoles; ++i)
                {
                    poles.SetValue(i, curve.Value(c1 + (i - 1) * dt));
                }
            }
        }

        if (!IsLinear(poles, tolerance, ref deviation))
        {
            return line; // non
        }

        Line3D alin = GetLine(p1, p2, out cf, out cl);
        line = new LineCurve3D(alin);
        return line;
    }

    /// <summary>
    /// The circle through P0, P1 and P2, its X axis towards P0. False (Circ unchanged) for points beyond
    /// sqrt(Precision::Infinite()), aligned points or a null radius.
    /// </summary>
    public static bool GetCircle(ref Circle3D crc,
                                 in Point3D p0,
                                 in Point3D p1,
                                 in Point3D p2)
    {
        //  Control if points are not aligned (should be done by MakeCirc
        double maxCoord = Math.Sqrt(global::Kerf.Engine.Foundation.Precision.Infinite());
        if (Math.Abs(p0.X) > maxCoord || Math.Abs(p0.Y) > maxCoord || Math.Abs(p0.Z) > maxCoord)
        {
            return false;
        }

        if (Math.Abs(p1.X) > maxCoord || Math.Abs(p1.Y) > maxCoord || Math.Abs(p1.Z) > maxCoord)
        {
            return false;
        }

        if (Math.Abs(p2.X) > maxCoord || Math.Abs(p2.Y) > maxCoord || Math.Abs(p2.Z) > maxCoord)
        {
            return false;
        }

        //  Building the circle
        var mkc = new MakeCirc(p0, p1, p2);
        if (!mkc.IsDone)
        {
            return false;
        }

        crc = mkc.Value();
        if (crc.Radius < Constants.Resolution())
        {
            return false;
        }

        //  Align to P0
        Point3D pc = crc.Location;
        Frame3D axe = crc.Position;
        var vx = new Vector3D(pc, p0);
        axe = axe.WithXDirection(new Direction3D(vx));
        crc = crc.WithPosition(axe);
        return true;
    }

    /// <summary>
    /// The circle through the points at c1, (2c1 + c2) / 3 and (c1 + 2c2) / 3, if 21 points of [c1, c2] are within
    /// tol of it; null otherwise. cf and cl are the parameters on the circle (cf in [0, 2Pi)).
    /// </summary>
    public static Curve3D? ComputeCircle(Curve3D c3d,
                                            double tol,
                                            double c1,
                                            double c2,
                                            ref double cf,
                                            ref double cl,
                                            ref double deviation)
    {
        if (c3d is CircleCurve3D)
        {
            cf = c1;
            cl = c2;
            deviation = 0.0;
            CircleCurve3D aCirc = (CircleCurve3D)c3d;
            return aCirc;
        }

        CircleCurve3D? circ = null;
        Point3D p0, p1, p2;
        double ca = (c1 + c1 + c2) / 3;
        double cb = (c1 + c2 + c2) / 3;
        p0 = c3d.Value(c1);
        p1 = c3d.Value(ca);
        p2 = c3d.Value(cb);

        var crc = new Circle3D();
        if (!GetCircle(ref crc, p0, p1, p2))
        {
            return circ;
        }

        //  Reste a controler que c est bien un cercle : prendre 20 points
        double du = (c2 - c1) / 20;
        int i;
        double max = 0.0;
        for (i = 0; i <= 20; i++)
        {
            double u = c1 + (du * i);
            Point3D pp = c3d.Value(u);
            double dist = crc.DistanceTo(pp);
            if (dist > tol)
            {
                return circ; // not done
            }

            if (dist > max)
            {
                max = dist;
            }
        }

        deviation = max;

        // defining the parameters
        double pi2 = 2 * M_PI;

        cf = global::Kerf.Engine.Geometry.ElCLib.Parameter(crc, c3d.Value(c1));
        cf = global::Kerf.Engine.Geometry.ElCLib.InPeriod(cf, 0.0, pi2);

        // first parameter should be closed to zero

        if (Math.Abs(cf) < global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(pi2 - cf) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            cf = 0.0;
        }

        double cm = global::Kerf.Engine.Geometry.ElCLib.Parameter(crc, c3d.Value((c1 + c2) / 2.0));
        cm = global::Kerf.Engine.Geometry.ElCLib.InPeriod(cm, cf, cf + pi2);

        cl = global::Kerf.Engine.Geometry.ElCLib.Parameter(crc, c3d.Value(c2));
        cl = global::Kerf.Engine.Geometry.ElCLib.InPeriod(cl, cm, cm + pi2);

        circ = new CircleCurve3D(crc);
        return circ;
    }

    private static bool IsArrayPntPlanar(NCollection_HArray1<Point3D> hap,
                                         ref Direction3D norm,
                                         double prec)
    {
        int size = hap.Length();
        if (size < 3)
        {
            return false;
        }

        Point3D p1 = hap.Value(1);
        Point3D p2 = hap.Value(2);
        Point3D p3 = hap.Value(3);
        double dist1 = p1.DistanceTo(p2);
        double dist2 = p1.DistanceTo(p3);
        if (dist1 < prec || dist2 < prec)
        {
            return false;
        }

        var v1 = new Vector3D(p1, p2);
        var v2 = new Vector3D(p1, p3);
        if (v1.IsParallel(v2, prec))
        {
            return false;
        }

        Vector3D nv = v1.Crossed(v2);

        int i;
        for (i = 1; i <= 3; ++i)
        {
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(nv.Coord(i)))
            {
                return false;
            }
        }

        if (nv.Magnitude < Constants.Resolution())
        {
            return false;
        }

        if (size > 3)
        {
            for (i = 4; i <= size; i++)
            {
                Point3D pn = hap.Value(i);
                dist1 = p1.DistanceTo(pn);
                if (dist1 < prec || global::Kerf.Engine.Foundation.Precision.IsInfinite(dist1))
                {
                    return false;
                }

                var vn = new Vector3D(p1, pn);
                if (!nv.IsNormal(vn, prec))
                {
                    return false;
                }
            }
        }

        norm = new Direction3D(nv);
        return true;
    }

    private static bool ConicDefinition(double a,
                                        double b1,
                                        double c,
                                        double d1,
                                        double e1,
                                        double f,
                                        bool isParab,
                                        bool isEllip,
                                        ref Point3D center,
                                        ref Direction3D mainAxis,
                                        ref double rmin,
                                        ref double rmax)
    {
        double xcen = 0.0, ycen = 0.0, xax = 0.0, yax = 0.0;
        double b, d, e;
        //  conic : a*x2 + 2*b*x*y + c*y2 + 2*d*x + 2*e*y + f = 0.
        // Equation (a,b,c,d,e,f);
        b = b1 / 2.0;
        d = d1 / 2.0;
        e = e1 / 2.0; // change of variable

        double eps = 1.0E-08; // ?? same as ComputedForm

        if (isParab)
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

            xcen = (b * e - c * d) / pdet;
            ycen = (b * d - a * e) / pdet;

            double term1 = a - c;
            double term2 = 2 * b;
            double cos2t, sin2t;
            double auxil;

            if (Math.Abs(term2) <= eps && Math.Abs(term1) <= eps)
            {
                cos2t = 1.0;
                sin2t = 0.0;
                auxil = 0.0;
            }
            else
            {
                // the rotation t of the axes, with its sign: cos 2t = (a - c) / auxil and sin 2t = 2b / auxil, t in
                // (-90, 90] degrees (OCCT took cos 2t = 1 / sqrt(1 + tan^2 2t) and sin t positive, so the axes came out
                // within 45 degrees of X, and gave up where a = c; Kerf decision 1492)
                auxil = Math.Sqrt(term1 * term1 + term2 * term2);
                cos2t = term1 / auxil;
                sin2t = term2 / auxil;
            }

            double cost = Math.Sqrt((1 + cos2t) / 2.0);
            double sint = Math.Sqrt((1 - cos2t) / 2.0);
            if (sin2t < 0.0)
            {
                sint = -sint;
            }

            double aprim = (a + c + auxil) / 2.0;
            double cprim = (a + c - auxil) / 2.0;

            if (Math.Abs(aprim) < Constants.Resolution() || Math.Abs(cprim) < Constants.Resolution())
            {
                return false;
            }

            term1 = -gdet / (aprim * pdet);
            term2 = -gdet / (cprim * pdet);

            if (isEllip)
            {
                if (term1 <= eps || term2 <= eps)
                {
                    return false;
                }

                // term1 is the squared semi-axis along (cos t, sin t), term2 the one across: the main axis is the
                // longer one's
                if (term1 >= term2)
                {
                    xax = cost;
                    yax = sint;
                    rmax = Math.Sqrt(term1);
                    rmin = Math.Sqrt(term2);
                }
                else
                {
                    xax = -sint;
                    yax = cost;
                    rmax = Math.Sqrt(term2);
                    rmin = Math.Sqrt(term1);
                }
            }
            else if (term1 <= eps)
            {
                if (-term1 <= eps || term2 <= eps)
                {
                    return false;
                }

                xax = -sint;
                yax = cost;
                rmin = Math.Sqrt(-term1);
                rmax = Math.Sqrt(term2);
            }
            else
            {
                if (term1 <= eps || -term2 <= eps)
                {
                    return false;
                }

                xax = cost;
                yax = sint;
                rmin = Math.Sqrt(-term2);
                rmax = Math.Sqrt(term1);
            }
        }

        center = center.WithCoord(xcen, ycen, 0.0);
        mainAxis = mainAxis.WithCoord(xax, yax, 0.0);
        return true;
    }

    /// <summary>
    /// The ellipse through 5 points of [c1, c2] (of its first four fifths if the curve is closed), if they are
    /// coplanar and 20 points of the range are within tol of it; null otherwise (and for near-circles, which
    /// ComputeCircle recognises). cf and cl are the parameters on the ellipse (cf in [0, 2Pi)).
    /// </summary>
    public static Curve3D? ComputeEllipse(Curve3D c3d,
                                             double tol,
                                             double c1,
                                             double c2,
                                             ref double cf,
                                             ref double cl,
                                             ref double deviation)
    {
        if (c3d is EllipseCurve3D)
        {
            cf = c1;
            cl = c2;
            deviation = 0.0;
            EllipseCurve3D elips = (EllipseCurve3D)c3d;
            return elips;
        }

        Curve3D? res = null;
        double prec = global::Kerf.Engine.Foundation.Precision.PConfusion();

        double af, bf, CF, df, ef, q1, q2, q3, c2n;
        int i;

        Point3D pStart = c3d.Value(c1);
        Point3D pEnd = c3d.Value(c2);

        bool isClos = pStart.DistanceTo(pEnd) < prec;
        if (isClos)
        {
            c2n = c2 - (c2 - c1) / 5;
        }
        else
        {
            c2n = c2;
        }

        //
        var bc = new Xyz();
        var ap = new NCollection_HArray1<Point3D>(1, 5);
        ap.SetValue(1, pStart);
        bc += pStart.XYZ();
        double dc = (c2n - c1) / 4;
        for (i = 1; i < 5; i++)
        {
            Point3D p = c3d.Value(c1 + dc * i);
            ap.SetValue(i + 1, p);
            bc += p.XYZ();
        }

        bc /= 5;
        bc *= -1;
        var trans = new Vector3D(bc);
        for (i = 1; i <= 5; ++i)
        {
            ap[i] = ap[i].Translated(trans);
        }

        var ndir = new Direction3D();
        if (!IsArrayPntPlanar(ap, ref ndir, prec))
        {
            return res;
        }

        if (Math.Abs(ndir.X) < Constants.Resolution() && Math.Abs(ndir.Y) < Constants.Resolution()
            && Math.Abs(ndir.Z) < Constants.Resolution())
        {
            return res;
        }

        var ax = new CoordinateSystem(new Point3D(0, 0, 0), ndir);
        var tr = new Transform3D();
        tr = tr.WithTransformation(ax);
        Transform3D tr2 = tr.Inverted();

        var dt = new Matrix(1, 5, 1, 5);
        math_Vector f = new(1, 5), sl = new(1, 5);

        double xn, yn, zn = 0.0;
        Point3D pt, pp;
        for (i = 1; i <= 5; i++)
        {
            pt = ap.Value(i).Transformed(tr);
            pt.Coord(out xn, out yn, out zn);
            dt[i, 1] = xn * xn;
            dt[i, 2] = xn * yn;
            dt[i, 3] = yn * yn;
            dt[i, 4] = xn;
            dt[i, 5] = yn;
            f[i] = -1.0;
        }

        var solver = new Gauss(dt);
        if (!solver.IsDone)
        {
            return res;
        }

        solver.Solve(f, sl);

        af = sl[1];
        bf = sl[2];
        CF = sl[3];
        df = sl[4];
        ef = sl[5];

        q1 = af * CF + bf * ef * df / 4 - CF * df * df / 4 - bf * bf / 4 - af * ef * ef / 4;
        q2 = af * CF - bf * bf / 4;
        q3 = af + CF;

        double rmax = 0.0, rmin = 0.0;
        var center = new Point3D();
        var mainAxis = new Direction3D();
        bool isParab = false, isEllip = false;

        if (q2 > 0 && q1 * q3 < 0)
        {
            // ellipse
            isEllip = true;
            if (ConicDefinition(af, bf, CF, df, ef, 1.0, isParab, isEllip, ref center, ref mainAxis, ref rmin, ref rmax))
            {
                // create ellipse
                if (rmax - rmin < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    return res; // really it is circle, which must be recognized in other method
                }

                trans *= -1;
                center = center.WithZ(zn);
                Point3D newCenter = center.Transformed(tr2);
                var ptmp = new Point3D(center.X + mainAxis.X * 10,
                                      center.Y + mainAxis.Y * 10,
                                      center.Z + mainAxis.Z * 10);
                Point3D newPtmp = ptmp.Transformed(tr2);
                var newMainAxis = new Direction3D(newPtmp.X - newCenter.X,
                                             newPtmp.Y - newCenter.Y,
                                             newPtmp.Z - newCenter.Z);
                var ax2 = new Frame3D(newCenter, ndir, newMainAxis);

                var ellipse = new Ellipse3D(ax2, rmax, rmin);
                ellipse = ellipse.Translated(trans);
                var gell = new EllipseCurve3D(ellipse);

                // test for 20 points
                double param2 = 0;
                dc = (c2 - c1) / 20;
                for (i = 1; i <= 20; i++)
                {
                    pp = c3d.Value(c1 + i * dc);
                    double par = global::Kerf.Engine.Geometry.ElCLib.Parameter(ellipse, pp);
                    double dist = gell.Value(par).DistanceTo(pp);
                    if (dist > tol)
                    {
                        return res; // not done
                    }

                    if (dist > param2)
                    {
                        param2 = dist;
                    }
                }

                deviation = param2;

                double pi2 = 2 * M_PI;
                cf = global::Kerf.Engine.Geometry.ElCLib.Parameter(ellipse, c3d.Value(c1));
                cf = global::Kerf.Engine.Geometry.ElCLib.InPeriod(cf, 0.0, pi2);

                // first parameter should be closed to zero

                if (Math.Abs(cf) < global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(pi2 - cf) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    cf = 0.0;
                }

                double cm = global::Kerf.Engine.Geometry.ElCLib.Parameter(ellipse, c3d.Value((c1 + c2) / 2.0));
                cm = global::Kerf.Engine.Geometry.ElCLib.InPeriod(cm, cf, cf + pi2);

                cl = global::Kerf.Engine.Geometry.ElCLib.Parameter(ellipse, c3d.Value(c2));
                cl = global::Kerf.Engine.Geometry.ElCLib.InPeriod(cl, cm, cm + pi2);

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
    public static Curve3D? ComputeCurve(Curve3D? theC3d,
                                           double tolerance,
                                           double c1,
                                           double c2,
                                           out double cf,
                                           out double cl,
                                           ref double gap,
                                           ConvType convType = MinGap,
                                           CurveKind target = Line)
    {
        cf = c1;
        cl = c2;
        var newc3d = new Curve3D?[3];
        Curve3D? c3d = theC3d;
        if (c3d is null)
        {
            return c3d;
        }

        Point3D p1 = c3d.Value(c1);
        Point3D p2 = c3d.Value(c2);
        Point3D p3 = c3d.Value(c1 + (c2 - c1) / 2);
        Span<double> d = [RealLast(), RealLast(), RealLast()];
        Span<double> fp = [0.0, 0.0, 0.0];
        Span<double> lp = [0.0, 0.0, 0.0];

        if (c3d is TrimmedCurve3D)
        {
            TrimmedCurve3D tc = (TrimmedCurve3D)c3d;
            c3d = tc.BasisCurve;
        }

        if (convType == Target)
        {
            gap = RealLast();
            if (target == Line)
            {
                newc3d[0] = ComputeLine(c3d, tolerance, c1, c2, ref fp[0], ref lp[0], ref gap);
                cf = fp[0];
                cl = lp[0];
                return newc3d[0];
            }

            if (target == Circle)
            {
                newc3d[1] = ComputeCircle(c3d, tolerance, c1, c2, ref fp[1], ref lp[1], ref gap);
                cf = fp[1];
                cl = lp[1];
                return newc3d[1];
            }

            if (target == Ellipse)
            {
                newc3d[2] = ComputeEllipse(c3d, tolerance, c1, c2, ref fp[2], ref lp[2], ref gap);
                cf = fp[2];
                cl = lp[2];
                return newc3d[2];
            }
        }

        //
        if (convType == Simplest)
        {
            gap = RealLast();
            newc3d[0] = ComputeLine(c3d, tolerance, c1, c2, ref fp[0], ref lp[0], ref gap);
            if (newc3d[0] is not null)
            {
                cf = fp[0];
                cl = lp[0];
                return newc3d[0];
            }

            gap = RealLast();
            newc3d[1] = ComputeCircle(c3d, tolerance, c1, c2, ref fp[1], ref lp[1], ref gap);
            if (newc3d[1] is not null)
            {
                cf = fp[1];
                cl = lp[1];
                return newc3d[1];
            }

            gap = RealLast();
            newc3d[2] = ComputeEllipse(c3d, tolerance, c1, c2, ref fp[2], ref lp[2], ref gap);
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
        if ((p1.DistanceTo(p2) < 2 * tolerance) && (p1.DistanceTo(p3) < 2 * tolerance))
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
            gap = d[imin];
            return newc3d[imin];
        }
        else
        {
            cf = c1;
            cl = c2;
            gap = -1.0;
            return newc3d[0]; // must be null curve;
        }
    }

    /// <summary>The deviation of the result of the last ConvertToAnalytical from the original curve.</summary>
    public double Gap() => _gap;

    /// <summary>The conversion type.</summary>
    public ConvType GetConvType() => _convType;

    /// <summary>Sets the conversion type.</summary>
    public void SetConvType(ConvType convType) => _convType = convType;

    /// <summary>The target curve type (for GeomConvert_Target).</summary>
    public CurveKind GetTarget() => _target;

    /// <summary>Sets the target curve type.</summary>
    public void SetTarget(CurveKind target) => _target = target;
}
