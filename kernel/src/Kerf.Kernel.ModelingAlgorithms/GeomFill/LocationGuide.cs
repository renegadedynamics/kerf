// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_LocationGuide.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_LocationGuide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Numerics;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The location law of a sweep along a guide: a trihedron law with a guide and, with a section law, a rotation
/// about the tangent that keeps the section in contact with the guide (the "contact" modes of the pipe shell).
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="Set"/>, <see cref="EraseRotation"/>, <see cref="SetCurve"/>,
/// <see cref="SetTrsf"/>, <see cref="SetInterval"/>, <see cref="SetTolerance"/> and <see cref="SetOrigine"/> are
/// public. It keeps the caller's trihedron law and section law, as OCCT keeps the handles, and sets the trihedron
/// law's curve and origin. OCCT's debug-only TraceRevol and the commented-out derivatives of the rotation (D1 and
/// D2 return false with a rotation, as in OCCT) are not ported.
/// </remarks>
internal class LocationGuide : SurfaceLocationLaw
{
    protected NCollection_HArray2<Point2D> _poles2d;

    private readonly TrihedronWithGuide _law;
    private SurfaceSectionLaw? _sec;
    private CurveAdaptorBase? _curve;
    private readonly CurveAdaptorBase _guide;
    private CurveAdaptorBase? _trimmed;
    private readonly int _nbPts;
    private bool _rotation;
    private double _origParam1;
    private double _origParam2;
    private double _uf;
    private double _ul;
    private double _firstS;
    private double _lastS;
    private double _ratio;
    private bool _withTrans;
    private Matrix3x3 _trans;
    private readonly math_Vector _tolRes;
    private readonly math_Vector _inf;
    private readonly math_Vector _sup;
    private readonly math_Vector _x;
    private readonly math_Vector _r;
    private SurfacePipeError _status;

    private static void InGoodPeriod(double prec, double period, ref double current)
    {
        double diff = current - prec;
        int nb = (int)Math.Truncate(diff / period);
        current -= nb * period;
        diff = current - prec;
        if (diff > period / 2)
        {
            current -= period;
        }
        else if (diff < -period / 2)
        {
            current += period;
        }
    }

    public LocationGuide(TrihedronWithGuide triedre)
    {
        _tolRes = new math_Vector(1, 3);
        _inf = new math_Vector(1, 3, 0.0);
        _sup = new math_Vector(1, 3, 0.0);
        _x = new math_Vector(1, 3);
        _r = new math_Vector(1, 3);
        _status = SurfacePipeError.PipeOk;

        _tolRes.Init(1.0e-6);
        _law = triedre; // loi de triedre
        _sec = null;    // loi de section
        _curve = null;
        _firstS = _lastS = -505e77;

        _nbPts = 21;              // nb points pour les calculs
        CurveAdaptorBase guide = _law.Guide(); // courbe guide
        if (!guide.IsPeriodic)
        {
            double f, l, delta;
            f = guide.FirstParameter();
            l = guide.LastParameter();
            delta = (l - f) / 100;
            f -= delta;
            l += delta;
            guide = guide.Trim(f, l, delta * 1.0e-7); // courbe guide
        } // if
        _guide = guide;

        _poles2d = new NCollection_HArray2<Point2D>(1, 2, 1, _nbPts);
        _rotation = false;  // contact ou non
        _origParam1 = 0;    // param pour ACR quand trajectoire
        _origParam2 = 1;    // et guide pas meme sens de parcourt
        _trans = new Matrix3x3();
        _trans = _trans.WithIdentity();
        _withTrans = false;
    }

    private void SetRotation(double precAngle, ref double lastAngle)
    {
        if (_curve is null)
        {
            throw new ConstructionErrorException("GeomFill_LocationGuide::The path is not set !!");
        }

        // repere fixe
        CoordinateSystem rep = new CoordinateSystem(Constants.Origin(), Constants.DZ(), Constants.DX());
        Vector3D T = new Vector3D(), n = new Vector3D(), b = new Vector3D();
        int ii, deg;
        bool isconst, israt = false;
        double t, v, w, oldAngle = 0, angle, deltaG, diff;
        double curAngle = precAngle, a1;
        Point2D p1 = new Point2D(), p2 = new Point2D();
        RevolutionSurface revol; // surface de revolution
        TrimmedCurve3D s;
        CurveSurfaceIntersectionPoint pInt = new CurveSurfaceIntersectionPoint(); // intersection guide/Revol
        NCollection_HArray1<int>? mult = null;
        NCollection_HArray1<double>? knots = null, weights = null;
        NCollection_HArray1<Point3D>? poles = null;

        double u = 0, uPeriod = 0;
        double f = _curve.FirstParameter();
        double l = _curve.LastParameter();
        bool ok, uperiodic = _sec!.IsUPeriodic;

        deltaG = (_guide.LastParameter() - _guide.FirstParameter()) / 5;
        Curve3D? mySection = null;
        double tol = 1.0e-9;

        _sec.SectionShape(out int nbPoles, out int nbKnots, out deg);

        if (_sec.IsConstant(ref tol))
        {
            mySection = _sec.ConstantSection();
            _uf = mySection.FirstParameter;
            _ul = mySection.LastParameter;

            isconst = true;
        }
        else
        {
            isconst = false;
            israt = _sec.IsRational;
            mult = new NCollection_HArray1<int>(1, nbKnots);
            _sec.Mults(mult);
            knots = new NCollection_HArray1<double>(1, nbKnots);
            _sec.Knots(knots);
            poles = new NCollection_HArray1<Point3D>(1, nbPoles);
            weights = new NCollection_HArray1<double>(1, nbPoles);
            _uf = knots.Value(1);
            _ul = knots.Value(nbKnots);
        }

        // Bornes de calculs
        double delta;
        delta = _guide.LastParameter() - _guide.FirstParameter();
        _inf[1] = _guide.FirstParameter() - delta / 10;
        _sup[1] = _guide.LastParameter() + delta / 10;

        _inf[2] = -M_PI;
        _sup[2] = 3 * M_PI;

        delta = _ul - _uf;
        _inf[3] = _uf - delta / 10;
        _sup[3] = _ul + delta / 10;

        // JALONNEMENT
        if (uperiodic)
        {
            uPeriod = _ul - _uf;
        }

        for (ii = 1; ii <= _nbPts; ii++)
        {
            t = (double)(_nbPts - ii) * f + (double)(ii - 1) * l;
            t /= _nbPts - 1;
            _curve.D0(t, out Point3D p);
            ok = _law.D0(t, ref T, ref n, ref b);
            if (!ok)
            {
                _status = _law.ErrorStatus();
                return; // Y a rien a faire.
            }
            Direction3D d = new Direction3D(T);
            if (_withTrans)
            {
                Matrix3x3 m = new Matrix3x3(n.XYZ(), b.XYZ(), T.XYZ());
                m *= _trans;
                d = new Direction3D(m.Column(3));
            }
            Axis3D ax = new Axis3D(p, d); // axe pour la surface de revoltuion

            // calculer transfo entre triedre et Oxyz
            Direction3D n2 = new Direction3D(n);
            CoordinateSystem n3 = new CoordinateSystem(p, d, n2);
            Transform3D transfo = new Transform3D().WithTransformation(n3, rep);

            // transformer la section
            if (!isconst)
            {
                u = _firstS + (t - _curve.FirstParameter()) * _ratio;
                _sec.D0(u, poles!, weights!);

                if (israt)
                {
                    mySection = new BSplineCurve3D(poles!, weights!, knots!, mult!, deg, _sec.IsUPeriodic);
                }
                else
                {
                    mySection = new BSplineCurve3D(poles!, knots!, mult!, deg, _sec.IsUPeriodic);
                }
                s = new TrimmedCurve3D(mySection, _uf, _ul);
            }
            else
            {
                s = new TrimmedCurve3D((Curve3D)mySection!.Copy(), _uf, _ul);
            }
            s.Transform(transfo);

            // Surface de revolution
            revol = new RevolutionSurface(s, ax);

            SurfaceAdaptor gArevol = new SurfaceAdaptor(revol);
            ExtCS distMini = new ExtCS(_guide, gArevol, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion());
            POnCurv pc = new POnCurv();
            POnSurf ps;
            double theU = 0.0, theV = 0.0;

            if (!distMini.IsDone || distMini.ExtCount == 0)
            {
                bool sos = false;
                if (ii > 1)
                {
                    // Intersection de secour entre surf revol et guide
                    // equation
                    _x[1] = _poles2d.Value(1, ii - 1).Y;
                    _x[2] = _poles2d.Value(2, ii - 1).X;
                    _x[3] = _poles2d.Value(2, ii - 1).Y;
                    FunctionGuide e = new FunctionGuide(_sec, _guide, u);
                    e.SetParam(u, p, T.XYZ(), n.XYZ());
                    // resolution   =>  angle
                    FunctionSetRoot result = new FunctionSetRoot(e, _tolRes);
                    result.Perform(e, _x, _inf, _sup);

                    if (result.IsDone && (result.FunctionSetErrors().Norm() < _tolRes[1] * _tolRes[1]))
                    {
                        sos = true;
                        math_Vector rr = new math_Vector(1, 3);
                        result.Root(rr);
                        pInt.SetValues(p, rr[2], rr[3], rr[1], TransitionOnCurve.Out);
                        theU = pInt.U();
                        theV = pInt.V();
                    }
                }
                if (!sos)
                {
                    _status = SurfacePipeError.ImpossibleContact;
                    return;
                }
                // OCCT: after the rescue, Pc is still the default point (parameter 0), which gives w below.
            }
            else
            { // on prend le point d'intersection
                // d'angle le plus proche de P
                double minDist = RealLast();
                int jref = 0;
                for (int j = 1; j <= distMini.ExtCount; j++)
                {
                    double dist = distMini.SquareDistance(j);
                    if (dist < minDist)
                    {
                        minDist = dist;
                        jref = j;
                    }
                }
                minDist = Math.Sqrt(minDist);
                distMini.Points(jref, out pc, out ps);

                ps.Parameter(out theU, out theV);
                a1 = theU;

                InGoodPeriod(curAngle, 2 * M_PI, ref a1);
            } // else

            // Controle de w
            w = pc.Parameter();

            if (ii > 1)
            {
                diff = w - _poles2d.Value(1, ii - 1).Y;
                if (Math.Abs(diff) > deltaG)
                {
                    if (_guide.IsPeriodic)
                    {
                        InGoodPeriod(_poles2d.Value(1, ii - 1).Y, _guide.Period(), ref w);
                        diff = w - _poles2d.Value(1, ii - 1).Y;
                    }
                }
            }

            // Recadrage de l'angle.
            angle = theU;

            if (ii > 1)
            {
                diff = angle - oldAngle;
                if (Math.Abs(diff) > M_PI)
                {
                    InGoodPeriod(oldAngle, 2 * M_PI, ref angle);
                    diff = angle - oldAngle;
                }
            }

            // Recadrage du V
            v = theV;

            if (ii > 1)
            {
                if (uperiodic)
                {
                    InGoodPeriod(_poles2d.Value(2, ii - 1).Y, uPeriod, ref v);
                }
                diff = v - _poles2d.Value(2, ii - 1).Y;
            }

            p1 = p1.WithCoord(t, w); // on stocke les parametres
            p2 = p2.WithCoord(angle, v);
            curAngle = angle;
            _poles2d.SetValue(1, ii, p1);
            _poles2d.SetValue(2, ii, p2);
            oldAngle = angle;
        }

        lastAngle = curAngle;
        _rotation = true; // C'est pret !
    }

    /// <summary>
    /// Sets the section law (parameters SFirst to SLast along the path) and, if rotat, computes the rotation that
    /// keeps the section in contact with the guide, starting from PrecAngle; LastAngle is the angle at the end.
    /// </summary>
    public void Set(SurfaceSectionLaw section, bool rotat, double sFirst, double sLast, double precAngle,
                    ref double lastAngle)
    {
        _status = SurfacePipeError.PipeOk;
        _firstS = sFirst;
        _lastS = sLast;
        lastAngle = precAngle;
        if (_curve is null)
        {
            _ratio = 0.0;
        }
        else
        {
            _ratio = (sLast - sFirst) / (_curve.LastParameter() - _curve.FirstParameter());
        }
        _sec = section;

        if (rotat)
        {
            SetRotation(precAngle, ref lastAngle);
        }
        else
        {
            _rotation = false;
        }
    }

    /// <summary>Removes the rotation (and an impossible-contact status).</summary>
    public void EraseRotation()
    {
        _rotation = false;
        if (_status == SurfacePipeError.ImpossibleContact)
        {
            _status = SurfacePipeError.PipeOk;
        }
    }

    /// <remarks>
    /// OCCT: the copy has no curve when <see cref="Set"/> computes its rotation, so copying a law with a rotation
    /// throws Standard_ConstructionError; and the start angle it passes is the first path parameter.
    /// </remarks>
    public override SurfaceLocationLaw Copy()
    {
        double la = 0;
        TrihedronWithGuide l;
        l = (TrihedronWithGuide)_law.Copy();
        LocationGuide copy = new LocationGuide(l);
        copy.SetOrigine(_origParam1, _origParam2);
        copy.Set(_sec!, _rotation, _firstS, _lastS, _poles2d.Value(1, 1).X, ref la);
        copy.SetTrsf(_trans);

        return copy;
    }

    /// <summary>
    /// Sets the path and, with a rotation, computes the poles on the guide (the intersections of the guide with
    /// the surface of revolution of the section at myNbPts points).
    /// </summary>
    public override bool SetCurve(CurveAdaptorBase c)
    {
        double lastAngle = 0;
        _curve = c;
        _trimmed = c;

        if (_curve is not null)
        {
            _law.SetCurve(c);
            _law.Origine(_origParam1, _origParam2);
            _status = _law.ErrorStatus();

            if (_rotation)
            {
                // OCCT: the start angle passed is the first path parameter, not the first angle.
                SetRotation(_poles2d.Value(1, 1).X, ref lastAngle);
            }
        }
        return _status == SurfacePipeError.PipeOk;
    }

    public override CurveAdaptorBase? GetCurve() => _curve;

    public override void SetTrsf(in Matrix3x3 transfo)
    {
        _trans = transfo;
        Matrix3x3 aux = new Matrix3x3().WithIdentity();
        aux -= _trans;
        _withTrans = false; // Au cas ou Trans = I
        for (int ii = 1; ii <= 3 && !_withTrans; ii++)
        {
            for (int jj = 1; jj <= 3 && !_withTrans; jj++)
            {
                if (Math.Abs(aux.Value(ii, jj)) > 1.0e-14)
                {
                    _withTrans = true;
                }
            }
        }
    }

    public override bool D0(double param, ref Matrix3x3 m, ref Vector3D v)
    {
        bool ok;
        Vector3D t = new Vector3D(), n = new Vector3D(), b = new Vector3D();

        _curve!.D0(param, out Point3D p);
        v = v.WithXYZ(p.XYZ());
        ok = _law.D0(param, ref t, ref n, ref b);
        if (!ok)
        {
            _status = _law.ErrorStatus();
            return ok;
        }
        m = m.WithCols(n.XYZ(), b.XYZ(), t.XYZ());

        if (_withTrans)
        {
            m *= _trans;
        }

        if (_rotation)
        {
            double u = _firstS + (param - _curve.FirstParameter()) * _ratio;
            return Rotate(param, u, p, ref m);
        }

        return true;
    }

    // The rotation of D0 and D0 with poles (OCCT writes it twice): the angle about the tangent that puts the
    // section on the guide, applied to the normal and binormal columns of M.
    private bool Rotate(double param, double u, in Point3D p, ref Matrix3x3 m)
    {
        // initialisations germe
        InitX(param);

        int iter = 100;
        Xyz t, b, n;
        t = m.Column(3);
        b = m.Column(2);
        n = m.Column(1);

        // Intersection entre surf revol et guide
        // equation
        FunctionGuide e = new FunctionGuide(_sec!, _guide, u);
        e.SetParam(param, p, t, n);

        // resolution   =>  angle
        FunctionSetRoot result = new FunctionSetRoot(e, _tolRes, iter);
        result.Perform(e, _x, _inf, _sup);

        if (result.IsDone)
        {
            // solution
            result.Root(_r);

            // rotation
            Matrix3x3 rot = new Matrix3x3().WithRotation(t, _r[2]);
            b *= rot;
            n *= rot;

            m = m.WithCols(n, b, t);
        }
        else
        {
            _status = SurfacePipeError.ImpossibleContact;
            return false;
        }
        return true;
    }

    /// <summary>The location; with a rotation, the intersection (C0) of the surface of revolution and the guide.</summary>
    public override bool D0(double param, ref Matrix3x3 m, ref Vector3D v, NCollection_Array1<Point2D> poles2d)
    {
        Vector3D t = new Vector3D(), n = new Vector3D(), b = new Vector3D();
        bool ok;

        _curve!.D0(param, out Point3D p);
        v = v.WithXYZ(p.XYZ());
        ok = _law.D0(param, ref t, ref n, ref b);
        if (!ok)
        {
            _status = _law.ErrorStatus();
            return ok;
        }
        m = m.WithCols(n.XYZ(), b.XYZ(), t.XYZ());

        if (_withTrans)
        {
            m *= _trans;
        }

        if (_rotation)
        {
            return Rotate(param, _firstS + (param - _curve.FirstParameter()) * _ratio, p, ref m);
        }

        return true;
    }

    /// <summary>The location and its first derivative; false with a rotation (OCCT's computation is commented out).</summary>
    public override bool D1(double param, ref Matrix3x3 m, ref Vector3D v, ref Matrix3x3 dm, ref Vector3D dv,
                            NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d)
    {
        Vector3D t = new Vector3D(), n = new Vector3D(), b = new Vector3D(), dt = new Vector3D(), dn = new Vector3D(),
               db = new Vector3D();
        bool ok;

        _curve!.D1(param, out Point3D p, out dv);
        v = v.WithXYZ(p.XYZ());
        ok = _law.D1(param, ref t, ref dt, ref n, ref dn, ref b, ref db);
        if (!ok)
        {
            _status = _law.ErrorStatus();
            return ok;
        }

        m = m.WithCols(n.XYZ(), b.XYZ(), t.XYZ());
        dm = dm.WithCols(dn.XYZ(), db.XYZ(), dt.XYZ());

        if (_withTrans)
        {
            m *= _trans;
            dm *= _trans;
        }

        if (_rotation)
        {
            return false;
        } // if_rotation

        return true;
    }

    /// <summary>The location and its first two derivatives; false with a rotation (OCCT's computation is commented out).</summary>
    public override bool D2(double param, ref Matrix3x3 m, ref Vector3D v, ref Matrix3x3 dm, ref Vector3D dv, ref Matrix3x3 d2M,
                            ref Vector3D d2V, NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                            NCollection_Array1<Vector2D> d2Poles2d)
    {
        Vector3D t = new Vector3D(), n = new Vector3D(), b = new Vector3D(), dt = new Vector3D(), dn = new Vector3D(),
               db = new Vector3D(), d2T = new Vector3D(), d2N = new Vector3D(), d2B = new Vector3D();
        bool ok;

        _curve!.D2(param, out Point3D p, out dv, out d2V);
        v = v.WithXYZ(p.XYZ());
        ok = _law.D2(param, ref t, ref dt, ref d2T, ref n, ref dn, ref d2N, ref b, ref db, ref d2B);
        if (!ok)
        {
            _status = _law.ErrorStatus();
            return ok;
        }

        // OCCT: the caller's incoming matrices are multiplied here, before they are set from the trihedron below,
        // so D2 never applies the transformation (D0 and D1 do).
        if (_withTrans)
        {
            m *= _trans;
            dm *= _trans;
            d2M *= _trans;
        }

        if (_rotation)
        {
            return false;
        } // if_rotation
        else
        {
            m = m.WithCols(n.XYZ(), b.XYZ(), t.XYZ());
            dm = dm.WithCols(dn.XYZ(), db.XYZ(), dt.XYZ());
            d2M = d2M.WithCols(d2N.XYZ(), d2B.XYZ(), d2T.XYZ());
        }

        return true;
    }

    public override bool HasFirstRestriction => false;

    public override bool HasLastRestriction => false;

    public override int TraceNumber() => 0;

    public override SurfacePipeError ErrorStatus() => _status;

    public override int NbIntervals(Continuity s)
    {
        int Nb_Sec, Nb_Law;
        Nb_Sec = _trimmed!.NbIntervals(s);
        Nb_Law = _law.NbIntervals(s);

        if (Nb_Sec == 1)
        {
            return Nb_Law;
        }
        else if (Nb_Law == 1)
        {
            return Nb_Sec;
        }

        NCollection_Array1<double> intC = new(1, Nb_Sec + 1);
        NCollection_Array1<double> intL = new(1, Nb_Law + 1);
        NCollection_Sequence<double> inter = new();
        _trimmed.Intervals(intC, s);
        _law.Intervals(intL, s);

        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(intC, intL, inter, global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.99);
        return inter.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        int Nb_Sec, Nb_Law;
        Nb_Sec = _trimmed!.NbIntervals(s);
        Nb_Law = _law.NbIntervals(s);

        if (Nb_Sec == 1)
        {
            _law.Intervals(t, s);
            return;
        }
        else if (Nb_Law == 1)
        {
            _trimmed.Intervals(t, s);
            return;
        }

        NCollection_Array1<double> intC = new(1, Nb_Sec + 1);
        NCollection_Array1<double> intL = new(1, Nb_Law + 1);
        NCollection_Sequence<double> inter = new();
        _trimmed.Intervals(intC, s);
        _law.Intervals(intL, s);

        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(intC, intL, inter, global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.99);
        for (int ii = 1; ii <= inter.Length(); ii++)
        {
            t[ii] = inter[ii];
        }
    }

    public override void SetInterval(double first, double last)
    {
        _law.SetInterval(first, last);
        _trimmed = _curve!.Trim(first, last, 0);
    }

    public override void GetInterval(out double first, out double last)
    {
        first = _trimmed!.FirstParameter();
        last = _trimmed.LastParameter();
    }

    public override void GetDomain(out double first, out double last)
    {
        first = _curve!.FirstParameter();
        last = _curve.LastParameter();
    }

    public override void SetTolerance(double tol3d, double tol2d)
    {
        _tolRes[1] = _guide.Resolution(tol3d);
        Resolution(1, tol3d, out _tolRes[2], out _tolRes[3]);
    }

    public override void Resolution(int index, double tol, out double tolU, out double tolV)
    {
        tolU = tol / 100;
        tolV = tol / 100;
    }

    /// <summary>1: the trihedrons are normed.</summary>
    public override double GetMaximalNorm() => 1.0;

    public override void GetAverageLaw(ref Matrix3x3 am, ref Vector3D av)
    {
        int ii;
        double u, delta;
        Vector3D v = new Vector3D(), v1 = new Vector3D(), v2 = new Vector3D(), v3 = new Vector3D();

        _law.GetAverageLaw(ref v1, ref v2, ref v3);
        // OCCT: the columns are (tangent, normal, binormal) here, but (normal, binormal, tangent) in D0, D1 and D2.
        am = am.WithCols(v1.XYZ(), v2.XYZ(), v3.XYZ());

        av = av.WithCoord(0.0, 0.0, 0.0);
        delta = (_trimmed!.LastParameter() - _trimmed.FirstParameter()) / 10;
        u = _trimmed.FirstParameter();
        // OCCT: myNbPts + 1 (22) points with a tenth of the interval as step, so most lie past its end.
        for (ii = 0; ii <= _nbPts; ii++, u += delta)
        {
            v = v.WithXYZ(_trimmed.Value(u).XYZ());
            av += v;
        }
        av = av / (_nbPts + 1);
    }

    /// <summary>A new copy of the constant section of the section law.</summary>
    public Curve3D Section() => _sec!.ConstantSection();

    /// <summary>The guide (trimmed a little beyond the trihedron's guide when that is not periodic).</summary>
    public CurveAdaptorBase Guide() => _guide;

    public override bool IsRotation(ref double error) => false;

    public override void Rotation(ref Point3D center)
    {
        throw new MethodNotImplementedException("GeomFill_LocationGuide::Rotation");
    }

    public override bool IsTranslation(ref double error) => false;

    // Finds an initial value by interpolation.
    private void InitX(double param)
    {
        int ideb = 1, ifin = _poles2d.RowLength(), idemi;
        double valeur, t1, t2;

        valeur = _poles2d.Value(1, ideb).X;
        if (param == valeur)
        {
            ifin = ideb + 1;
        }

        valeur = _poles2d.Value(1, ifin).X;
        if (param == valeur)
        {
            ideb = ifin - 1;
        }

        while (ideb + 1 != ifin)
        {
            idemi = (ideb + ifin) / 2;
            valeur = _poles2d.Value(1, idemi).X;
            if (valeur < param)
            {
                ideb = idemi;
            }
            else
            {
                if (valeur > param)
                {
                    ifin = idemi;
                }
                else
                {
                    ideb = idemi;
                    ifin = ideb + 1;
                }
            }
        }

        t1 = _poles2d.Value(1, ideb).X;
        t2 = _poles2d.Value(1, ifin).X;
        double diff = t2 - t1;

        double w1, w2;
        w1 = _poles2d.Value(1, ideb).Coord(2);
        w2 = _poles2d.Value(1, ifin).Coord(2);
        Point2D p1 = _poles2d.Value(2, ideb);
        Point2D p2 = _poles2d.Value(2, ifin);

        if (diff > 1.0e-7)
        {
            double b = (param - t1) / diff, a = (t2 - param) / diff;
            _x[1] = a * w1 + b * w2;
            _x[2] = a * p1.Coord(1) + b * p2.Coord(1); // angle
            _x[3] = a * p1.Coord(2) + b * p2.Coord(2); // param isov
        }
        else
        {
            _x[1] = (w1 + w2) / 2;
            _x[2] = (p1.Coord(1) + p2.Coord(1)) / 2;
            _x[3] = (p1.Coord(2) + p2.Coord(2)) / 2;
        }

        if (_guide.IsPeriodic)
        {
            _x[1] = global::Kerf.Engine.Geometry.ElCLib.InPeriod(_x[1], _guide.FirstParameter(), _guide.LastParameter());
        }
        _x[2] = global::Kerf.Engine.Geometry.ElCLib.InPeriod(_x[2], 0, 2 * M_PI);
        if (_sec!.IsUPeriodic)
        {
            _x[3] = global::Kerf.Engine.Geometry.ElCLib.InPeriod(_x[3], _uf, _ul);
        }
    }

    /// <summary>Sets the origins of the curvilinear abscissas, for a multi-edge path (ACR).</summary>
    public void SetOrigine(double param1, double param2)
    {
        _origParam1 = param1;
        _origParam2 = param2;
    }

    /// <summary>The distance from the path to the guide (the current point on it) at myNbPts parameters.</summary>
    public SurfacePipeError ComputeAutomaticLaw(out NCollection_HArray1<Point2D> parAndRad)
    {
        Vector3D T = new Vector3D(), n = new Vector3D(), b = new Vector3D();
        int ii;
        double t;

        SurfacePipeError theStatus = SurfacePipeError.PipeOk;

        double f = _curve!.FirstParameter();
        double l = _curve.LastParameter();

        parAndRad = new NCollection_HArray1<Point2D>(1, _nbPts);
        for (ii = 1; ii <= _nbPts; ii++)
        {
            t = (double)(_nbPts - ii) * f + (double)(ii - 1) * l;
            t /= _nbPts - 1;
            _curve.D0(t, out Point3D p);
            bool ok = _law.D0(t, ref T, ref n, ref b);
            if (!ok)
            {
                theStatus = _law.ErrorStatus();
                return theStatus;
            }
            Point3D pointOnGuide = _law.CurrentPointOnGuide();
            double curWidth = p.DistanceTo(pointOnGuide);

            Point2D paramWithRadius = new Point2D(t, curWidth);
            parAndRad.SetValue(ii, paramWithRadius);
        }

        return theStatus;
    }
}
