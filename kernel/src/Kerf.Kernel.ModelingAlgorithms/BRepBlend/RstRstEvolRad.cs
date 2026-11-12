// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_RstRstEvolRad.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_RstRstEvolRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Fillets.BlendFunctions;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics;
using Kerf.Geometry;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The function of an evolving-radius fillet (the radius a law of the guide's parameter) between two restrictions (arcs of two surfaces) along a guide: the
/// variables are the parameters on the two restrictions.
/// </summary>
/// <remarks>A working object (decision 21): it caches its last solution, which the walkers read.</remarks>
internal class RstRstEvolRad : RstRstFunction, IEnvelopeSections
{
    private readonly SurfaceAdaptorBase _surf1;
    private readonly SurfaceAdaptorBase _surf2;
    private readonly Curve2DAdaptorBase _rst1;
    private readonly Curve2DAdaptorBase _rst2;
    private readonly CurveOnSurface _cons1;
    private readonly CurveOnSurface _cons2;
    private readonly CurveAdaptorBase _guide;
    private CurveAdaptorBase _tguide;
    private Point3D _ptrst1;
    private Point3D _ptrst2;
    private Point2D _pt2drst1;
    private Point2D _pt2drst2;
    private double _prmrst1;
    private double _prmrst2;
    private bool _istangent;
    private Vector3D _tgrst1;
    private Vector2D _tg2drst1;
    private Vector3D _tgrst2;
    private Vector2D _tg2drst2;
    private double _ray;
    private double _dray;
    private int _choix;
    private Point3D _ptgui;
    private Vector3D _d1gui;
    private Vector3D _d2gui;
    private Vector3D _nplan;
    private double _normtg;
    private double _d;
    private SurfaceAdaptorBase? _surfref1;
    private Curve2DAdaptorBase? _rstref1;
    private SurfaceAdaptorBase? _surfref2;
    private Curve2DAdaptorBase? _rstref2;
    private double _maxang;
    private double _minang;
    private double _distmin;
    private SectionShape _sShape;
    // OCCT leaves myTConv uninitialised until GetShape sets it; here it starts at its first value.
    private ParameterisationType _tConv;
    private LawsFunction _tevol;
    private readonly LawsFunction _fevol;

    private static void FusionneIntervalles(NCollection_Array1<double> i1, NCollection_Array1<double> i2,
                                            NCollection_Sequence<double> seq)
    {
        int ind1 = 1, ind2 = 1;
        double epspar = global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.99;
        // it is supposed that positioning works with PConfusion()/2
        double v1, v2;

        while ((ind1 <= i1.Upper()) && (ind2 <= i2.Upper()))
        {
            v1 = i1[ind1];
            v2 = i2[ind2];
            if (Math.Abs(v1 - v2) <= epspar)
            {
                // Here the elements of I1 and I2 fit.
                seq.Append((v1 + v2) / 2);
                ind1++;
                ind2++;
            }
            else if (v1 < v2)
            {
                // Here the element of I1 fits.
                seq.Append(v1);
                ind1++;
            }
            else
            {
                // Here the element of TABLE2 fits.
                seq.Append(v2);
                ind2++;
            }
        }

        if (ind1 > i1.Upper())
        {
            // Here I1 is exhausted, completed with the end of TABLE2
            for (; ind2 <= i2.Upper(); ind2++)
            {
                seq.Append(i2[ind2]);
            }
        }

        if (ind2 > i2.Upper())
        {
            // Here I2 is exhausted, completed with the end of I1
            for (; ind1 <= i1.Upper(); ind1++)
            {
                seq.Append(i1[ind1]);
            }
        }
    }

    private static void T3dto2d(out double a, out double b, in Vector3D A, in Vector3D B, in Vector3D c)
    {
        double ab = A.Dot(B);
        double ac = A.Dot(c);
        double bc = B.Dot(c);
        double bb = B.Dot(B);
        double cc = c.Dot(c);
        double deno = (bb * cc - bc * bc);
        a = (ab * cc - ac * bc) / deno;
        b = (ac * bb - ab * bc) / deno;
    }

    public RstRstEvolRad(SurfaceAdaptorBase surf1, Curve2DAdaptorBase rst1, SurfaceAdaptorBase surf2,
                                    Curve2DAdaptorBase rst2, CurveAdaptorBase cGuide, LawsFunction evol)
    {
        _surf1 = surf1;
        _surf2 = surf2;
        _rst1 = rst1;
        _rst2 = rst2;
        _cons1 = new CurveOnSurface(rst1, surf1);
        _cons2 = new CurveOnSurface(rst2, surf2);
        _guide = cGuide;
        _tguide = cGuide;
        _istangent = true;
        _maxang = RealFirst();
        _minang = RealLast();
        _distmin = RealLast();
        _sShape = SectionShape.Rational;

        _tevol = evol;
        _fevol = evol;
        _first = cGuide.FirstParameter();
        _last = cGuide.LastParameter();
    }

    public override int NbVariables()
    {
        return 2;
    }

    public override int NbEquations()
    {
        return 2;
    }

    // Kerf (decision 2216): the rolling ball's equations, as RstRstConstRad's (decision 2162), of the radius the law
    // gives where the section crosses the guide. As there (decision 2162): OCCT puts both
    // points in the plane square to the guide and the centre in it, at the radius from both; the ball touches each
    // edge where the centre's foot on it is, so the centre is at the radius from both points and its chord to each is
    // square to that edge's tangent, which puts it off the guide's plane wherever the edges aren't square to it. The
    // guide's plane only places the section, through the points' midpoint, as ConstRad's and SurfRstConstRad's do, so
    // the functions meet where one hands over to another.
    //   f1 = nplan . (ptrst1 + ptrst2) / 2 + d
    //   f2 = (centre - ptrst2) . t2, t2 the second edge's unit tangent
    // the centre the point at the radius from both points with its chord to the first square to that edge (BallCentre).
    // Scratch of the last evaluation, read by IsSolution and the sections.
    private Vector3D _d11, _d21, _t1, _t2, _dt1, _dt2;
    private Point3D _center;
    private bool _isBall;

    // The sides of the centre OCCT's CenterCircleRst1Rst2 takes: its bisector, (ptrst2 - ptrst1) ^ np, reversed for
    // choix 3 and 4.
    private Vector3D Side(in Point3D ptRst1, in Point3D ptRst2, in Vector3D np)
    {
        Vector3D side = new Vector3D(ptRst1, ptRst2).Crossed(np);
        return _choix > 2 ? side.Reversed() : side;
    }

    // Evaluates both edges (to the second order) at x, their unit tangents and those's derivatives, and the centre.
    private void Ball(math_Vector x, in Vector3D np)
    {
        _cons1.D2(x[1], out _ptrst1, out _d11, out Vector3D d2rst1);
        _cons2.D2(x[2], out _ptrst2, out _d21, out Vector3D d2rst2);
        double inv1 = ConstRad.InverseNorm(_d11), inv2 = ConstRad.InverseNorm(_d21);
        _t1 = inv1 * _d11;
        _t2 = inv2 * _d21;
        _dt1 = ConstRad.UnitNormalD1(_t1, inv1, d2rst1);
        _dt2 = ConstRad.UnitNormalD1(_t2, inv2, d2rst2);
        _isBall = RstRstConstRad.BallCentre(_ptrst1, _t1, _ptrst2, _ray, Side(_ptrst1, _ptrst2, np), out _center);
    }

    private void BallValues(math_Vector f)
    {
        f[1] = (_nplan.XYZ().Dot(_ptrst1.XYZ()) + _nplan.XYZ().Dot(_ptrst2.XYZ())) / 2 + _d;
        f[2] = new Vector3D(_ptrst2, _center).Dot(_t2);
    }

    // The centre's derivatives in the two parameters (BallCentreD1).
    private void CentreD1(out Vector3D dcu, out Vector3D dcv)
    {
        Vector3D c1 = new(_ptrst1, _center), c2 = new(_ptrst2, _center);
        dcu = RstRstConstRad.BallCentreD1(_center, _ptrst1, _t1, _ptrst2, -_d11.Dot(_t1) + c1.Dot(_dt1), -c1.Dot(_d11), 0.0);
        dcv = RstRstConstRad.BallCentreD1(_center, _ptrst1, _t1, _ptrst2, 0.0, 0.0, -c2.Dot(_d21));
    }

    private void BallDerivatives(Matrix d)
    {
        CentreD1(out Vector3D dcu, out Vector3D dcv);
        d[1, 1] = _nplan.Dot(_d11) / 2;
        d[1, 2] = _nplan.Dot(_d21) / 2;
        d[2, 1] = dcu.Dot(_t2);
        d[2, 2] = (dcv - _d21).Dot(_t2) + new Vector3D(_ptrst2, _center).Dot(_dt2);
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        Ball(x, _nplan);
        BallValues(f);
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        Ball(x, _nplan);
        BallDerivatives(d);
        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        Ball(x, _nplan);
        BallValues(f);
        BallDerivatives(d);
        return true;
    }

    private const double Eps = 1.0e-15;

    /// <summary>The reference surfaces and restrictions <see cref="Decroch"/> tests against.</summary>
    public void Set(SurfaceAdaptorBase surfRef1, Curve2DAdaptorBase rstRef1, SurfaceAdaptorBase surfRef2,
                    Curve2DAdaptorBase rstRef2)
    {
        _surfref1 = surfRef1;
        _surfref2 = surfRef2;
        _rstref1 = rstRef1;
        _rstref2 = rstRef2;
    }

    public override void Set(double param)
    {
        _d1gui = new Vector3D(0.0, 0.0, 0.0);
        _nplan = new Vector3D(0.0, 0.0, 0.0);
        _tguide.D2(param, out _ptgui, out _d1gui, out _d2gui);
        _normtg = _d1gui.Magnitude;
        _nplan = _nplan.WithXYZ(_d1gui.Normalized().XYZ());
        Xyz nplanXYZ = _nplan.XYZ();
        Xyz ptguiXYZ = _ptgui.XYZ();
        _d = nplanXYZ.Dot(ptguiXYZ);
        _d = _d * (-1.0);
        _tevol.D1(param, out _ray, out _dray);
    }

    public override void Set(double first, double last)
    {
        _tguide = _guide.Trim(first, last, 1.0e-12);
        _tevol = _fevol.Trim(first, last, 1.0e-12);
        _first = first;
        _last = last;
    }

    // Kerf (unit 15AK): the sections are the circles of the envelope the ball of changing radius sweeps (EnvelopeCircle),
    // which take the derivative of the centre along the walk, and their derivatives its second. The interval the
    // function is set on:
    private double _first, _last;

    // Positions the guide and the law at prm, as Set(double) does; the derivative of the guide's plane's normal.
    private Vector3D Position(double prm)
    {
        _tguide.D2(prm, out _ptgui, out _d1gui, out _d2gui);
        _tevol.D1(prm, out _ray, out _dray);
        _normtg = _d1gui.Magnitude;
        _nplan = _d1gui.Normalized();
        _d = -_nplan.XYZ().Dot(_ptgui.XYZ());
        return Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);
    }

    // The derivative of the ball's centre along the walk at prm and x, with the walk's derivative of x, the function
    // left positioned there. False where there's no ball or the walk has no derivative.
    private bool WalkCentre(double prm, math_Vector x, math_Vector dx, out Vector3D dc)
    {
        Matrix grad = new(1, 2, 1, 2);
        Vector3D dnplan = Position(prm);
        dc = new Vector3D();
        Ball(x, _nplan);
        if (!_isBall)
        {
            return false;
        }

        BallDerivatives(grad);
        GuideSecondMember(dx, dnplan);
        Gauss resol = new(grad, 1.0e-9);
        if (!resol.IsDone)
        {
            return false;
        }

        resol.Solve(dx);
        CentreD1(out Vector3D dcu, out Vector3D dcv);
        dc = Vector3D.LinearForm(dx[1], dcu, dx[2], dcv, CentreDRay());
        return true;
    }

    // The same on the walk's solution at prm + dt, found by Newton's method from x and its derivative dx at prm.
    private bool WalkCentreNear(double prm, math_Vector x, math_Vector dx, double dt, out Vector3D dc)
    {
        math_Vector y = new(1, 2), f = new(1, 2), step = new(1, 2), dy = new(1, 2);
        Matrix grad = new(1, 2, 1, 2);
        dc = new Vector3D();
        for (int i = 1; i <= 2; i++)
        {
            y[i] = x[i] + dt * dx[i];
        }

        Position(prm + dt);
        for (int iter = 0; iter < 8; iter++)
        {
            Ball(y, _nplan);
            if (!_isBall)
            {
                return false;
            }

            BallValues(f);
            BallDerivatives(grad);
            Gauss resol = new(grad, 1.0e-14);
            if (!resol.IsDone)
            {
                return false;
            }

            resol.Solve(-f, step);
            bool small = true;
            for (int i = 1; i <= 2; i++)
            {
                y[i] += step[i];
                small &= Math.Abs(step[i]) <= 1.0e-13 * (1.0 + Math.Abs(y[i]));
            }

            if (small)
            {
                break;
            }
        }

        return WalkCentre(prm + dt, y, dy, out dc);
    }

    // The second derivative of the centre along the walk at prm and x: the equations give the first, so its difference
    // between the walk's solutions a ten-thousandth of the radius either side, or on one side at an end of the interval
    // the function is set on. It moves the function about the point.
    private bool WalkCentreD2(double prm, math_Vector x, out Vector3D d2c)
    {
        math_Vector dx = new(1, 2);
        d2c = new Vector3D();
        if (!WalkCentre(prm, x, dx, out Vector3D dc))
        {
            return false;
        }

        double speed = dc.Magnitude;
        if (speed <= 0.0 || _last <= _first)
        {
            return false;
        }

        double h = Math.Min(1.0e-4 * Math.Abs(_ray) / speed, (_last - _first) / 4);
        Vector3D a, b;
        if (prm - h >= _first && prm + h <= _last)
        {
            if (!WalkCentreNear(prm, x, dx, h, out a) || !WalkCentreNear(prm, x, dx, -h, out b))
            {
                return false;
            }

            d2c = (a - b) / (2 * h);
            return true;
        }

        if (prm + h > _last)
        {
            h = -h;
        }

        if (!WalkCentreNear(prm, x, dx, h, out a) || !WalkCentreNear(prm, x, dx, 2 * h, out b))
        {
            return false;
        }

        d2c = Vector3D.LinearForm(-3, dc, 4, a, -1, b) / (2 * h);
        return true;
    }

    public bool SectionGap(in BlendPoint near, double prm, double before, double after, out double gap, out double radius)
    {
        math_Vector x = new(1, 2);
        x[1] = near.ParameterOnC1();
        x[2] = near.ParameterOnC2();
        CurveAdaptorBase tguide = _tguide;
        LawsFunction tevol = _tevol;
        double first = _first, last = _last;
        gap = 0.0;
        Point3D m2 = new Point3D();
        bool ok = ArcMiddle(x, prm, before, prm, out Point3D m1) && ArcMiddle(x, prm, prm, after, out m2);
        if (ok)
        {
            gap = m1.DistanceTo(m2);
        }

        radius = Math.Abs(_ray);
        _tguide = tguide;
        _tevol = tevol;
        _first = first;
        _last = last;
        return ok;
    }

    public bool SectionMiddle(in BlendPoint at, out Point3D middle, out double radius)
    {
        math_Vector y = new(1, 2), dx = new(1, 2);
        y[1] = at.ParameterOnC1();
        y[2] = at.ParameterOnC2();
        double prm = at.Parameter();
        middle = new Point3D();
        Position(prm);
        bool ok = SolveAt(y) && WalkCentre(prm, y, dx, out Vector3D dc) && Envelope(dc, out EnvelopeCircle e) && e.ArcMiddle(out middle);
        radius = Math.Abs(_ray);
        return ok;
    }

    public bool LeastAdvance(in BlendPoint near, double prm, out double advance)
    {
        math_Vector y = new(1, 2), dx = new(1, 2);
        y[1] = near.ParameterOnC1();
        y[2] = near.ParameterOnC2();
        advance = 0.0;
        Position(prm);
        // A point of the walk is its own solution: Newton's method only between the walk's points.
        if ((prm != near.Parameter() && !SolveAt(y)) || !WalkCentreD2(prm, y, out Vector3D d2c) || !WalkCentre(prm, y, dx, out Vector3D dc)
            || !Envelope(dc, out EnvelopeCircle e))
        {
            return false;
        }

        _tevol.D2(prm, out _, out _, out double d2ray);
        advance = e.LeastAdvance(_center, dc, d2c, _ray, _dray, d2ray);
        return true;
    }

    // The walk's solution at the parameter the function is positioned at, by Newton's method from y, which it replaces.
    private bool SolveAt(math_Vector y)
    {
        math_Vector f = new(1, 2), step = new(1, 2);
        Matrix grad = new(1, 2, 1, 2);
        bool small = false;
        for (int iter = 0; iter < 12 && !small; iter++)
        {
            Ball(y, _nplan);
            if (!_isBall)
            {
                return false;
            }

            BallValues(f);
            BallDerivatives(grad);
            Gauss resol = new(grad, 1.0e-14);
            if (!resol.IsDone)
            {
                return false;
            }

            resol.Solve(-f, step);
            small = true;
            for (int i = 1; i <= 2; i++)
            {
                y[i] += step[i];
                small &= Math.Abs(step[i]) <= 1.0e-12 * (1.0 + Math.Abs(y[i]));
            }
        }

        return small;
    }

    // The middle of the arc of the envelope's circle at prm, on the walk's solution there (Newton's method from x),
    // with the guide and the law as they are between first and last.
    private bool ArcMiddle(math_Vector x, double prm, double first, double last, out Point3D middle)
    {
        math_Vector y = new(1, 2), dx = new(1, 2);
        middle = new Point3D();
        Set(first, last);
        Position(prm);
        y.Initialized(x);
        return SolveAt(y) && WalkCentre(prm, y, dx, out Vector3D dc) && Envelope(dc, out EnvelopeCircle e) && e.ArcMiddle(out middle);
    }

    // The side of its plane the great circle's normal is on, as the sections orient it, at the last positioning.
    private Vector3D Side()
    {
        Vector3D np = _choix % 2 != 0 ? _nplan.Reversed() : _nplan;
        return ConstRad.SectionNormal(new Vector3D(_center, _ptrst1), new Vector3D(_center, _ptrst2), np);
    }

    // The envelope's circle at the last positioning, from the centre's derivative along the walk.
    private bool Envelope(in Vector3D dc, out EnvelopeCircle circle)
    {
        Vector3D zero = new Vector3D();
        Vector3D u1 = new(_center, _ptrst1), u2 = new(_center, _ptrst2);
        return EnvelopeCircle.Make(0, _center, dc, zero, zero, _ray, _dray, 0.0, 0.0, ConstRad.InverseNorm(u1) * u1, zero, zero,
                                   ConstRad.InverseNorm(u2) * u2, zero, zero, Side(), out circle);
    }

    public override void GetTolerance(math_Vector tolerance, double tol)
    {
        tolerance[1] = _cons1.Resolution(tol);
        tolerance[2] = _cons2.Resolution(tol);
    }

    public override void GetBounds(math_Vector infBound, math_Vector supBound)
    {
        infBound[1] = _cons1.FirstParameter();
        infBound[2] = _cons2.FirstParameter();
        supBound[1] = _cons1.LastParameter();
        supBound[2] = _cons2.LastParameter();
    }

    /// <summary>
    /// True if <paramref name="sol"/> solves the function within <paramref name="tol"/>; then computes the tangents
    /// and updates the extreme angles and the minimal distance of the sections (called once per accepted point).
    /// </summary>
    public override bool IsSolution(math_Vector sol, double tol)
    {
        math_Vector valsol = new(1, 2), secmember = new(1, 2);
        Matrix gradsol = new(1, 2, 1, 2);

        Vector3D dnplan, d1urst1, d1vrst1, d1urst2, d1vrst2;
        Point3D bid;

        double cosa, sina, angle;

        Values(sol, valsol, gradsol);

        if (Math.Abs(valsol[1]) <= tol && Math.Abs(valsol[2]) <= tol)
        {
            // The ball (decision 2162): none where the points are too far apart, as OCCT's CenterCircleRst1Rst2.
            if (!_isBall)
            {
                _istangent = true;
                return false;
            }

            // Calculation of tangents
            _prmrst1 = sol[1];
            _pt2drst1 = _rst1.Value(_prmrst1);
            _prmrst2 = sol[2];
            _pt2drst2 = _rst2.Value(_prmrst2);

            dnplan = Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);
            GuideSecondMember(secmember, dnplan);

            Gauss resol = new(gradsol);

            if (resol.IsDone)
            {
                resol.Solve(secmember);
                _istangent = false;
            }
            else
            {
                Svd singRS = new(gradsol);
                if (singRS.IsDone)
                {
                    math_Vector dedt = new(1, 2);
                    dedt.Initialized(secmember);
                    singRS.Solve(dedt, secmember, 1.0e-6);
                    _istangent = false;
                }
                else
                {
                    _istangent = true;
                }
            }

            if (!_istangent)
            {
                _tgrst1 = secmember[1] * _d11;
                _tgrst2 = secmember[2] * _d21;

                double a, b;
                _surf1.D1(_pt2drst1.X, _pt2drst1.Y, out bid, out d1urst1, out d1vrst1);
                T3dto2d(out a, out b, _tgrst1, d1urst1, d1vrst1);
                _tg2drst1 = _tg2drst1.WithCoord(a, b);
                _surf2.D1(_pt2drst2.X, _pt2drst2.Y, out bid, out d1urst2, out d1vrst2);
                // Kerf: OCCT passes tgrst1 here where tgrst2 is meant.
                T3dto2d(out a, out b, _tgrst2, d1urst2, d1vrst2);
                _tg2drst2 = _tg2drst2.WithCoord(a, b);
            }

            Vector3D n1 = new(_center, _ptrst1), n2 = new(_center, _ptrst2);

            n1 = n1.Normalized();
            n2 = n2.Normalized();

            cosa = n1.Dot(n2);
            sina = _nplan.Dot(n1.Crossed(n2));

            if (_choix % 2 != 0)
            {
                sina = -sina; // nplan is changed into -nplan
            }

            angle = Math.Acos(cosa);
            if (sina < 0.0)
            {
                angle = 2.0 * M_PI - angle;
            }

            // Kerf (unit 15AK): the section is the envelope's circle, so the angle is its arc's.
            if (!_istangent)
            {
                CentreD1(out Vector3D dcu, out Vector3D dcv);
                if (Envelope(Vector3D.LinearForm(secmember[1], dcu, secmember[2], dcv, CentreDRay()), out EnvelopeCircle envelope))
                {
                    angle = envelope.Angle();
                    if (angle < 0.0)
                    {
                        angle += 2.0 * M_PI;
                    }
                }
            }

            if (angle > _maxang)
            {
                _maxang = angle;
            }
            if (angle < _minang)
            {
                _minang = angle;
            }
            _distmin = Min(_distmin, _ptrst1.DistanceTo(_ptrst2));

            return true;
        }
        _istangent = true;
        return false;
    }

    // The guide's term of the equations' derivative along the walk, with the sign the solvers take it (-df/dt): f1
    // moves with the guide's plane, f2 with the centre as the radius changes.
    private void GuideSecondMember(math_Vector secmember, in Vector3D dnplan)
    {
        Vector3D temp = new((_ptrst1.XYZ() + _ptrst2.XYZ()) / 2 - _ptgui.XYZ());
        secmember[1] = _normtg - dnplan.Dot(temp);
        secmember[2] = -CentreDRay().Dot(_t2);
    }

    // The centre's derivative along the guide at fixed points: its distance from both points moves with the radius.
    private Vector3D CentreDRay()
    {
        return RstRstConstRad.BallCentreD1(_center, _ptrst1, _t1, _ptrst2, 0.0, -_ray * _dray, -_ray * _dray);
    }

    public override double GetMinimalDistance()
    {
        return _distmin;
    }

    public override Point3D PointOnRst1() => _ptrst1;

    public override Point3D PointOnRst2() => _ptrst2;

    public override Point2D Pnt2dOnRst1() => _pt2drst1;

    public override Point2D Pnt2dOnRst2() => _pt2drst2;

    public override double ParameterOnRst1() => _prmrst1;

    public override double ParameterOnRst2() => _prmrst2;

    public override bool IsTangencyPoint => _istangent;

    public override Vector3D TangentOnRst1()
    {
        if (_istangent)
        {
            throw new DomainErrorException();
        }
        return _tgrst1;
    }

    public override Vector2D Tangent2dOnRst1()
    {
        if (_istangent)
        {
            throw new DomainErrorException();
        }
        return _tg2drst1;
    }

    public override Vector3D TangentOnRst2()
    {
        if (_istangent)
        {
            throw new DomainErrorException();
        }
        return _tgrst2;
    }

    public override Vector2D Tangent2dOnRst2()
    {
        if (_istangent)
        {
            throw new DomainErrorException();
        }
        return _tg2drst2;
    }

    /// <summary>Which of the restrictions the section at <paramref name="sol"/> leaves its reference surface on.</summary>
    public override DecrochStatus Decroch(math_Vector sol, out Vector3D nRst1, out Vector3D tgRst1,
                                                out Vector3D nRst2, out Vector3D tgRst2)
    {
        Vector3D nRst1InPlane, nRst2InPlane;
        Point3D ptTmp1, ptTmp2;
        Vector3D d1u, d1v, centptrst;
        double norm;
        double u, v;

        _rstref1!.Value(sol[1]).Coord(out u, out v);
        _surfref1!.D1(u, v, out ptTmp1, out d1u, out d1v);
        // Normal to the reference surface 1
        nRst1 = d1u.Crossed(d1v);
        _rstref2!.Value(sol[2]).Coord(out u, out v);
        _surfref2!.D1(u, v, out ptTmp2, out d1u, out d1v);
        // Normal to the reference surface 2
        nRst2 = d1u.Crossed(d1v);

        // The ball's (decision 2162): its centre, the section in the plane through it and both points, the reference
        // surfaces' normals projected into it.
        Ball(sol, _nplan);
        Point3D center = _center;
        Vector3D np = ConstRad.SectionNormal(new Vector3D(center, ptTmp1), new Vector3D(center, ptTmp2), _nplan);

        nRst1InPlane = nRst1 - np.Dot(nRst1) * np;
        norm = nRst1InPlane.Magnitude;
        nRst1InPlane = norm > Eps ? -nRst1InPlane / norm : -nRst1InPlane; // OCCT's sign, where the chord is square to it

        centptrst = new Vector3D(center, ptTmp1);

        if (centptrst.Dot(nRst1InPlane) < 0.0)
        {
            nRst1InPlane = nRst1InPlane.Reversed();
        }

        tgRst1 = np.Crossed(centptrst);

        nRst2InPlane = nRst2 - np.Dot(nRst2) * np;
        norm = nRst2InPlane.Magnitude;
        nRst2InPlane = norm > Eps ? -nRst2InPlane / norm : -nRst2InPlane; // OCCT's sign, where the chord is square to it
        centptrst = new Vector3D(center, ptTmp2);

        if (centptrst.Dot(nRst2InPlane) < 0.0)
        {
            nRst2InPlane = nRst2InPlane.Reversed();
        }

        tgRst2 = np.Crossed(centptrst);

        if (_choix % 2 != 0)
        {
            tgRst1 = tgRst1.Reversed();
            tgRst2 = tgRst2.Reversed();
        }

        // The vectors are returned
        if (nRst1InPlane.Dot(tgRst1) > -1.0e-10)
        {
            if (nRst2InPlane.Dot(tgRst2) < 1.0e-10)
            {
                return DecrochStatus.DecrochBoth;
            }
            else
            {
                return DecrochStatus.DecrochRst1;
            }
        }
        else
        {
            if (nRst2InPlane.Dot(tgRst2) < 1.0e-10)
            {
                return DecrochStatus.DecrochRst2;
            }
            else
            {
                return DecrochStatus.NoDecroch;
            }
        }
    }

    /// <summary>Sets the "quadrant" <paramref name="choix"/> (1 to 4) in which the fillet lies.</summary>
    public void Set(int choix)
    {
        _choix = choix;
    }

    /// <summary>Sets the type of section generation for the approximations.</summary>
    public void Set(SectionShape typeSection)
    {
        _sShape = typeSection;
    }

    /// <summary>
    /// The center of the circle of radius the fillet's through the two points, in the plane of normal
    /// <paramref name="np"/>: false if they are too far apart. <paramref name="center"/> is written only on success;
    /// <paramref name="vdMed"/> is the direction of the perpendicular bisector.
    /// </summary>
    public bool CenterCircleRst1Rst2(in Point3D ptRst1, in Point3D ptRst2, in Vector3D np, ref Point3D center,
                                     out Vector3D vdMed)
    {
        Vector3D rst1rst2 = new(ptRst1, ptRst2);
        Vector3D vdmedNor; // directing vector of the perpendicular bisector
        double norm2;
        double dist; // distance between the middle of PtRst1,PtRst2 and Center

        // Calculate the center of the circle
        vdMed = rst1rst2.Crossed(np);
        norm2 = rst1rst2.SquareMagnitude;
        dist = _ray * _ray - 0.25 * norm2;

        if (_choix > 2)
        {
            vdMed = vdMed.Reversed();
        }

        if (dist < -1.0E-07)
        {
            return false;
        }

        if (dist > 1.0E-07)
        {
            dist = Math.Sqrt(dist);
            vdmedNor = vdMed.Normalized();
            center = center.WithXYZ(0.5 * rst1rst2.XYZ() + ptRst1.XYZ() + dist * vdmedNor.XYZ());
        }
        else
        {
            center = center.WithXYZ(0.5 * rst1rst2.XYZ() + ptRst1.XYZ());
        }

        return true;
    }

    /// <summary>The circular section at <paramref name="param"/> through the points U and V, and its bounds.</summary>
    public void Section(double param, double u, double v, out double pdeb, out double pfin, ref Circle3D c)
    {
        Vector3D ns, np;
        Point3D center;

        // Kerf (unit 15AK): the envelope's circle; the ball's great circle where the walk has no derivative.
        math_Vector x = new(1, 2), dx = new(1, 2);
        x[1] = u;
        x[2] = v;
        if (WalkCentre(param, x, dx, out Vector3D dc) && Envelope(dc, out EnvelopeCircle e))
        {
            Vector3D plane = e.Plane;
            c = c.WithRadius(e.Radius);
            c = c.WithPosition(new Frame3D(e.Centre, new Direction3D(plane), new Direction3D(e.N1)));
            pdeb = 0;
            pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, _ptrst2);
            if (pfin > 1.5 * M_PI)
            {
                plane = plane.Reversed();
                c = c.WithPosition(new Frame3D(e.Centre, new Direction3D(plane), new Direction3D(e.N1)));
                pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, _ptrst2);
            }
            if (pfin < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                pfin += global::Kerf.Engine.Foundation.Precision.PConfusion();
            }
            return;
        }

        _tguide.D1(param, out _ptgui, out _d1gui);
        np = _d1gui.Normalized();
        _ray = _tevol.Value(param);

        // The ball's circle (decision 2162): its centre, in the plane through it and both points.
        Ball(x, np);
        center = _center;
        np = ConstRad.SectionNormal(new Vector3D(center, _ptrst1), new Vector3D(center, _ptrst2), np);

        c = c.WithRadius(Math.Abs(_ray));
        ns = new Vector3D(center, _ptrst1).Normalized();

        if (_choix % 2 != 0)
        {
            np = np.Reversed();
        }

        c = c.WithPosition(new Frame3D(center, new Direction3D(np), new Direction3D(ns)));
        pdeb = 0; // ElCLib::Parameter(C, pts);
        pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, _ptrst2);

        // Test of angles negative and almost null : Special Case
        if (pfin > 1.5 * M_PI)
        {
            np = np.Reversed();
            c = c.WithPosition(new Frame3D(center, new Direction3D(np), new Direction3D(ns)));
            pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, _ptrst2);
        }
        if (pfin < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            pfin += global::Kerf.Engine.Foundation.Precision.PConfusion();
        }
    }

    public override bool IsRational
    {
        get
        {
            return (_sShape == SectionShape.Rational || _sShape == SectionShape.QuasiAngular);
        }
    }

    public override double GetSectionSize()
    {
        return _maxang * Math.Abs(_ray);
    }

    public override void GetMinimalWeight(NCollection_Array1<double> weights)
    {
        global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.GetMinimalWeights(_sShape, _tConv, _minang, _maxang, weights);
        // It is supposed that it does not depend on the Radius!
    }

    // Kerf (unit 15AK): the envelope's circle takes the derivative of the centre's path, so the sections are as smooth
    // as the law one class down and the guide two, as BlendFunc_EvolRad's.
    public override int NbIntervals(Continuity s)
    {
        int Nb_Int_Courbe, Nb_Int_Loi;
        Nb_Int_Courbe = _guide.NbIntervals(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s)));
        Nb_Int_Loi = _fevol.NbIntervals(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));

        if (Nb_Int_Loi == 1)
        {
            return Nb_Int_Courbe;
        }

        NCollection_Array1<double> intC = new(1, Nb_Int_Courbe + 1);
        NCollection_Array1<double> intL = new(1, Nb_Int_Loi + 1);
        NCollection_Sequence<double> inter = new();
        _guide.Intervals(intC, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s)));
        _fevol.Intervals(intL, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));

        FusionneIntervalles(intC, intL, inter);
        return inter.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        int Nb_Int_Courbe, Nb_Int_Loi;
        Nb_Int_Courbe = _guide.NbIntervals(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s)));
        Nb_Int_Loi = _fevol.NbIntervals(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));

        if (Nb_Int_Loi == 1)
        {
            _guide.Intervals(t, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s)));
        }
        else
        {
            NCollection_Array1<double> intC = new(1, Nb_Int_Courbe + 1);
            NCollection_Array1<double> intL = new(1, Nb_Int_Loi + 1);
            NCollection_Sequence<double> inter = new();
            _guide.Intervals(intC, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s)));
            _fevol.Intervals(intL, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));

            FusionneIntervalles(intC, intL, inter);
            for (int ii = 1; ii <= inter.Length(); ii++)
            {
                t[ii] = inter[ii];
            }
        }
    }

    public override void GetShape(out int nbPoles, out int nbKnots, out int degree, out int nbPoles2d)
    {
        nbPoles2d = 2;
        global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.GetShape(_sShape, _maxang, out nbPoles, out nbKnots, out degree, ref _tConv);
    }

    /// <summary>The tolerances to use in the approximations.</summary>
    public override void GetTolerance(double boundTol, double surfTol, double angleTol, math_Vector tol3d,
                                      math_Vector tol1d)
    {
        int low = tol3d.Lower(), up = tol3d.Upper();
        double tol;
        tol = global::Kerf.Engine.Construction.Surfaces.GeomFill.GetTolerance(_tConv, _minang, Math.Abs(_ray), angleTol, surfTol);
        tol1d.Init(surfTol);
        tol3d.Init(surfTol);
        tol3d[low + 1] = tol3d[up - 1] = Min(tol, surfTol);
        tol3d[low] = tol3d[up] = Min(tol, boundTol);
    }

    public override void Knots(NCollection_Array1<double> tKnots)
    {
        global::Kerf.Engine.Construction.Surfaces.GeomFill.Knots(_tConv, tKnots);
    }

    public override void Mults(NCollection_Array1<int> tMults)
    {
        global::Kerf.Engine.Construction.Surfaces.GeomFill.Mults(_tConv, tMults);
    }

    public override void Section(in BlendPoint p, NCollection_Array1<Point3D> poles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<double> weights)
    {
        Vector3D ns, ns2;
        Point3D center;

        double prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();

        _tguide.D1(prm, out _ptgui, out _d1gui);
        _nplan = _d1gui.Normalized();
        _ray = _tevol.Value(prm);

        math_Vector x = new(1, 2);
        x[1] = p.ParameterOnC1();
        x[2] = p.ParameterOnC2();

        Point2D pt2d1 = _rst1.Value(x[1]);
        Point2D pt2d2 = _rst2.Value(x[2]);

        // The ball's (decision 2162).
        Ball(x, _nplan);
        _distmin = Min(_distmin, _ptrst1.DistanceTo(_ptrst2));

        poles2d[poles2d.Lower()] = poles2d[poles2d.Lower()].WithCoord(pt2d1.X, pt2d1.Y);
        poles2d[poles2d.Upper()] = poles2d[poles2d.Upper()].WithCoord(pt2d2.X, pt2d2.Y);

        // Linear case
        if (_sShape == SectionShape.Linear)
        {
            poles[low] = _ptrst1;
            poles[upp] = _ptrst2;
            weights[low] = 1.0;
            weights[upp] = 1.0;
            return;
        }

        // Kerf (unit 15AK): the envelope's circle; the ball's great circle where the walk has no derivative.
        math_Vector dx = new(1, 2);
        if (WalkCentre(prm, x, dx, out Vector3D dc) && Envelope(dc, out EnvelopeCircle e))
        {
            global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, e.N1, e.N2, e.Plane, _ptrst1, _ptrst2, e.Radius, e.Centre, poles, weights);
            return;
        }

        center = _center;

        // normals to the section with points
        ns = new Vector3D(center, _ptrst1).Normalized();
        ns2 = new Vector3D(center, _ptrst2).Normalized();

        if (_choix % 2 != 0)
        {
            _nplan = _nplan.Reversed();
        }

        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns, ns2, ConstRad.SectionNormal(ns, ns2, _nplan), _ptrst1, _ptrst2, Math.Abs(_ray), center, poles, weights);
    }

    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                                 NCollection_Array1<double> weights, NCollection_Array1<double> dWeights)
    {
        Vector3D dnplan, d1n1 = new(), d1n2 = new();
        Vector3D tgct = new();
        Vector3D d1urst, d1vrst;
        Point3D center, notUsed;

        math_Vector sol = new(1, 2), secmember = new(1, 2);
        Matrix gradsol = new(1, 2, 1, 2);

        double prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();
        bool istgt;

        // Kerf (unit 15AK): the centre's second derivative along the walk, which the derivative of the envelope's circle
        // takes; found first, as finding it moves the function about this point.
        Vector3D d2c = new();
        bool second = false;
        if (_sShape != SectionShape.Linear)
        {
            sol[1] = p.ParameterOnC1();
            sol[2] = p.ParameterOnC2();
            second = WalkCentreD2(prm, sol, out d2c);
        }

        _tguide.D2(prm, out _ptgui, out _d1gui, out _d2gui);
        _tevol.D1(prm, out _ray, out _dray);
        _normtg = _d1gui.Magnitude;
        _nplan = _d1gui.Normalized();
        dnplan = Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

        sol[1] = _prmrst1 = p.ParameterOnC1();
        sol[2] = _prmrst2 = p.ParameterOnC2();
        _pt2drst1 = _rst1.Value(_prmrst1);
        _pt2drst2 = _rst2.Value(_prmrst2);

        // The ball's (decision 2216): its centre, which moves with the points and the radius.
        Ball(sol, _nplan);
        BallDerivatives(gradsol);
        GuideSecondMember(secmember, dnplan);

        Gauss resol = new(gradsol, 1.0e-9);

        if (resol.IsDone)
        {
            istgt = false;
            resol.Solve(secmember);
        }
        else
        {
            Svd singRS = new(gradsol);
            if (singRS.IsDone)
            {
                math_Vector dedt = new(1, 2);
                dedt.Initialized(secmember);
                singRS.Solve(dedt, secmember, 1.0e-6);
                istgt = false;
            }
            else
            {
                istgt = true;
            }
        }

        if (!_isBall)
        {
            return false;
        }
        center = _center;

        Vector3D n1 = new(center, _ptrst1), n2 = new(center, _ptrst2);
        double norm1 = n1.Magnitude, norm2 = n2.Magnitude;
        n1 = n1.Normalized();
        n2 = n2.Normalized();

        if (!istgt)
        {
            // secmember contains derivatives of parameters on curves compared to t
            _tgrst1 = secmember[1] * _d11;
            _tgrst2 = secmember[2] * _d21;

            // The centre's derivative along the walk, then the unit chords'.
            CentreD1(out Vector3D dcu, out Vector3D dcv);
            tgct = Vector3D.LinearForm(secmember[1], dcu, secmember[2], dcv, CentreDRay());
            d1n1 = (_tgrst1 - tgct) / norm1;
            d1n1 = d1n1 - n1.Dot(d1n1) * n1;
            d1n2 = (_tgrst2 - tgct) / norm2;
            d1n2 = d1n2 - n2.Dot(d1n2) * n2;
        }

        // Tops 2d

        poles2d[poles2d.Lower()] = poles2d[poles2d.Lower()].WithCoord(_pt2drst1.X, _pt2drst1.Y);
        poles2d[poles2d.Upper()] = poles2d[poles2d.Upper()].WithCoord(_pt2drst2.X, _pt2drst2.Y);
        if (!istgt)
        {
            double a, b;
            _surf1.D1(_pt2drst1.X, _pt2drst1.Y, out notUsed, out d1urst, out d1vrst);
            T3dto2d(out a, out b, _tgrst1, d1urst, d1vrst);
            dPoles2d[poles2d.Lower()] = dPoles2d[poles2d.Lower()].WithCoord(a, b);

            _surf2.D1(_pt2drst2.X, _pt2drst2.Y, out notUsed, out d1urst, out d1vrst);
            T3dto2d(out a, out b, _tgrst2, d1urst, d1vrst);
            dPoles2d[poles2d.Upper()] = dPoles2d[poles2d.Upper()].WithCoord(a, b);
        }

        // Linear case
        if (_sShape == SectionShape.Linear)
        {
            poles[low] = _ptrst1;
            poles[upp] = _ptrst2;
            weights[low] = 1.0;
            weights[upp] = 1.0;
            if (!istgt)
            {
                dPoles[low] = _tgrst1;
                dPoles[upp] = _tgrst2;
                dWeights[low] = 0.0;
                dWeights[upp] = 0.0;
            }
            return (!istgt);
        }

        // Case of the circle: the envelope's (unit 15AK), with its derivative along the walk.
        if (!istgt && second)
        {
            Vector3D zero = new();
            _tevol.D2(prm, out _, out _, out double d2ray);
            if (EnvelopeCircle.Make(1, center, tgct, d2c, zero, _ray, _dray, d2ray, 0.0, n1, d1n1, zero, n2, d1n2, zero,
                                    Side(), out EnvelopeCircle e))
            {
                return global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, e.N1, e.N2, e.DN1, e.DN2, e.Plane, e.DPlane, _ptrst1, _ptrst2, _tgrst1, _tgrst2,
                                          e.Radius, e.DRadius, e.Centre, e.DCentre, poles, dPoles, weights, dWeights);
            }
        }

        // Where the envelope has no circle: the ball's great circle (decision 2162), and no derivative.
        Vector3D np = _nplan;
        if (_choix % 2 != 0)
        {
            np = np.Reversed();
        }

        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, n1, n2, ConstRad.SectionNormal(n1, n2, np), _ptrst1, _ptrst2, Math.Abs(_ray), center, poles, weights);
        return false;
    }

    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Vector3D> d2Poles, NCollection_Array1<Point2D> poles2d,
                                 NCollection_Array1<Vector2D> dPoles2d, NCollection_Array1<Vector2D> d2Poles2d,
                                 NCollection_Array1<double> weigths, NCollection_Array1<double> dWeigths,
                                 NCollection_Array1<double> d2Weigths)
    {
        return false;
    }

    public override void Resolution(int ic2d, double tol, out double tolU, out double tolV)
    {
        if (ic2d == 1)
        {
            tolU = _surf1.UResolution(tol);
            tolV = _surf1.VResolution(tol);
        }
        else
        {
            tolU = _surf2.UResolution(tol);
            tolV = _surf2.VResolution(tol);
        }
    }
}
