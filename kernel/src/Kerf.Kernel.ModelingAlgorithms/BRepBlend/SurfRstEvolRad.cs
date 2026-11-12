// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstEvolRad.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstEvolRad.cxx
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
/// The function of an evolving-radius fillet (the radius a law of the guide's parameter) between a surface and a
/// restriction (an arc of another surface) along a guide: the variables are (U, V) on the surface and the parameter
/// on the restriction.
/// </summary>
/// <remarks>A working object (decision 21): it caches its last solution, which the walkers read.</remarks>
internal class SurfRstEvolRad : SurfRstFunction, IEnvelopeSections
{
    private const double Eps = 1.0e-15;

    private readonly SurfaceAdaptorBase _surf;
    private readonly SurfaceAdaptorBase _surfrst;
    private readonly Curve2DAdaptorBase _rst;
    private readonly CurveOnSurface _cons;
    private readonly CurveAdaptorBase _guide;
    private CurveAdaptorBase _tguide;
    private Point3D _pts;
    private Point3D _ptrst;
    private Point2D _pt2ds;
    private Point2D _pt2drst;
    private double _prmrst;
    private bool _istangent;
    private Vector3D _tgs;
    private Vector2D _tg2ds;
    private Vector3D _tgrst;
    private Vector2D _tg2drst;
    private double _ray;
    private double _dray;
    private int _choix;
    private Point3D _ptgui;
    private Vector3D _d1gui;
    private Vector3D _d2gui;
    private Vector3D _nplan;
    private double _normtg;
    private double _d;
    private SurfaceAdaptorBase? _surfref;
    private Curve2DAdaptorBase? _rstref;
    private double _maxang;
    private double _minang;
    private double _distmin;
    private SectionShape _sShape;
    // OCCT leaves myTConv uninitialised until GetShape sets it; here it starts at its first value.
    private ParameterisationType _tConv;
    private LawsFunction _tevol;
    private readonly LawsFunction _fevol;
    private double _sg1;

    private static void FusionneIntervalles(NCollection_Array1<double> i1, NCollection_Array1<double> i2,
                                            NCollection_Sequence<double> seq)
    {
        int ind1 = 1, ind2 = 1;
        double epspar = global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.99;
        // it is supposed that positioning works with PConfusion()/2
        double v1, v2;
        // Initialisation : IND1 and IND2 point at the first element of each of 2 tables to be processed. INDS points
        // at the last element created by TABSOR

        //--- TABSOR is filled by parsing TABLE1 and TABLE2 simultaneously and eliminating multiple occurrences ---

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
            //----- Here I1 is exhausted, completed with the end of TABLE2 -------

            for (; ind2 <= i2.Upper(); ind2++)
            {
                seq.Append(i2[ind2]);
            }
        }

        if (ind2 > i2.Upper())
        {
            //----- Here I2 is exhausted, completed with the end of I1 -------

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

    public SurfRstEvolRad(SurfaceAdaptorBase surf, SurfaceAdaptorBase surfRst, Curve2DAdaptorBase rst,
                                     CurveAdaptorBase cGuide, LawsFunction evol)
    {
        _surf = surf;
        _surfrst = surfRst;
        _rst = rst;
        _cons = new CurveOnSurface(rst, surfRst);
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
        return 3;
    }

    public override int NbEquations()
    {
        return 3;
    }

    // Kerf (decision 2216): the rolling ball's equations, as SurfRstConstRad's (decision 2158), of the radius the law
    // gives where the section crosses the guide. The centre is at the radius from the surface's contact along its own
    // unit normal (OCCT projects the normal into the plane square to the guide and puts both contacts in that plane);
    // the ball touches the restriction where the centre's foot on it is, so the centre minus that point is square to
    // the restriction's tangent and as long as the radius; the guide's plane only places the section, through the
    // contacts' midpoint, as EvolRad's does, so the two functions meet where one hands over to the other.
    //   f1 = nplan . (pts + ptrst) / 2 + d
    //   f2 = (centre - ptrst) . t, t the restriction's unit tangent
    //   f3 = |centre - ptrst|^2 - ray^2
    // Scratch of the last evaluation, read by IsSolution and the sections.
    private Vector3D _nn, _dnu, _dnv, _drst, _d2rst, _trst, _dtrst;
    private double _invnorm, _invnormrst;
    private Point3D _center;

    // Evaluates the surface (to the second order) and the restriction at x, the centre, and the derivatives of the
    // reversed unit normal -nn in u and v (_dnu, _dnv) and of the restriction's unit tangent in w (_dtrst).
    private void Ball(math_Vector x, out Vector3D d1u1, out Vector3D d1v1)
    {
        _surf.D2(x[1], x[2], out _pts, out d1u1, out d1v1, out Vector3D d2u1, out Vector3D d2v1, out Vector3D d2uv1);
        _cons.D2(x[3], out _ptrst, out _drst, out _d2rst);

        Vector3D ns = d1u1.Crossed(d1v1);
        _invnorm = ConstRad.InverseNorm(ns);
        _nn = _invnorm * ns;
        _center = _pts.Translated(-_ray * _nn);
        _dnu = -ConstRad.UnitNormalD1(_nn, _invnorm, d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1)));
        _dnv = -ConstRad.UnitNormalD1(_nn, _invnorm, d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1)));

        _invnormrst = ConstRad.InverseNorm(_drst);
        _trst = _invnormrst * _drst;
        _dtrst = ConstRad.UnitNormalD1(_trst, _invnormrst, _d2rst);
    }

    private void BallValues(math_Vector f)
    {
        Vector3D corde = new(_ptrst, _center);
        f[1] = (_nplan.XYZ().Dot(_pts.XYZ()) + _nplan.XYZ().Dot(_ptrst.XYZ())) / 2 + _d;
        f[2] = corde.Dot(_trst);
        f[3] = corde.SquareMagnitude - _ray * _ray;
    }

    private void BallDerivatives(Matrix d, in Vector3D d1u1, in Vector3D d1v1)
    {
        Vector3D corde = new(_ptrst, _center);
        Vector3D dcu = d1u1 + _ray * _dnu, dcv = d1v1 + _ray * _dnv;

        d[1, 1] = _nplan.Dot(d1u1) / 2;
        d[1, 2] = _nplan.Dot(d1v1) / 2;
        d[1, 3] = _nplan.Dot(_drst) / 2;

        d[2, 1] = dcu.Dot(_trst);
        d[2, 2] = dcv.Dot(_trst);
        d[2, 3] = corde.Dot(_dtrst) - _drst.Dot(_trst);

        d[3, 1] = 2 * corde.Dot(dcu);
        d[3, 2] = 2 * corde.Dot(dcv);
        d[3, 3] = -2 * corde.Dot(_drst);
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        Ball(x, out _, out _);
        BallValues(f);
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        Ball(x, out Vector3D d1u1, out Vector3D d1v1);
        BallDerivatives(d, d1u1, d1v1);
        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        Ball(x, out Vector3D d1u1, out Vector3D d1v1);
        BallValues(f);
        BallDerivatives(d, d1u1, d1v1);
        return true;
    }

    // The guide's term of the equations' derivative along the walk, with the sign the solvers take it (-df/dt): f1
    // moves with the guide's plane, f2 and f3 with the radius, the centre moving by -dray along the unit normal.
    private void GuideSecondMember(math_Vector secmember, in Vector3D dnplan)
    {
        Vector3D temp = new((_pts.XYZ() + _ptrst.XYZ()) / 2 - _ptgui.XYZ());
        Vector3D corde = new(_ptrst, _center);
        secmember[1] = _normtg - dnplan.Dot(temp);
        secmember[2] = _dray * _nn.Dot(_trst);
        secmember[3] = 2 * _dray * corde.Dot(_nn) + 2 * _ray * _dray;
    }

    /// <summary>The reference surface and restriction <see cref="Decroch"/> tests against.</summary>
    public void Set(SurfaceAdaptorBase surfRef, Curve2DAdaptorBase rstRef)
    {
        _surfref = surfRef;
        _rstref = rstRef;
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
        _ray = _sg1 * _ray;
        _dray = _sg1 * _dray;
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
        _ray = _sg1 * _ray;
        _dray = _sg1 * _dray;
        _normtg = _d1gui.Magnitude;
        _nplan = _d1gui.Normalized();
        _d = -_nplan.XYZ().Dot(_ptgui.XYZ());
        return Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);
    }

    // The derivative of the ball's centre along the walk at prm and x, with the walk's derivative of x, the function
    // left positioned there. False where the walk has none.
    private bool WalkCentre(double prm, math_Vector x, math_Vector dx, out Vector3D dc)
    {
        Matrix grad = new(1, 3, 1, 3);
        Vector3D dnplan = Position(prm);
        Ball(x, out Vector3D d1u1, out Vector3D d1v1);
        BallDerivatives(grad, d1u1, d1v1);
        GuideSecondMember(dx, dnplan);
        Gauss resol = new(grad, 1.0e-9);
        if (!resol.IsDone)
        {
            dc = new Vector3D();
            return false;
        }

        resol.Solve(dx);
        Vector3D tgs = Vector3D.LinearForm(dx[1], d1u1, dx[2], d1v1);
        Vector3D dnw = Vector3D.LinearForm(dx[1], _dnu, dx[2], _dnv);
        dc = Vector3D.LinearForm(_ray, dnw, _dray, -_nn, tgs);
        return true;
    }

    // The second derivative of the centre along the walk at prm and x, from the equations' second order (unit 15AK2,
    // decision 33305). With J x' = s the walk's first order (J the equations' derivatives in u, v and the restriction's
    // parameter, s the guide's term), J x'' = s' - J' x', each term's derivative along the walk: the surface's third
    // derivatives (the unit normal's second), the restriction's third (its unit tangent's second), the guide's third
    // (its plane's normal's second) and the law's second. Then the centre c = pts - ray nn, whose first derivative is
    // tgs - ray' nn + ray dnw (dnw the reversed unit normal's), moves by
    //   c'' = tgs' - ray'' nn + 2 ray' dnw + ray dnw'.
    // The function is left positioned at prm and x.
    private bool WalkCentreD2(double prm, math_Vector x, out Vector3D d2c)
    {
        math_Vector dx = new(1, 3), rhs = new(1, 3);
        Matrix grad = new(1, 3, 1, 3);
        d2c = new Vector3D();
        Vector3D dnplan = Position(prm);
        Ball(x, out Vector3D d1u, out Vector3D d1v);
        BallDerivatives(grad, d1u, d1v);
        GuideSecondMember(dx, dnplan);
        Gauss resol = new(grad, 1.0e-9);
        if (!resol.IsDone)
        {
            return false;
        }

        resol.Solve(dx);
        double up = dx[1], vp = dx[2], tp = dx[3];

        _surf.D3(x[1], x[2], out _, out _, out _, out Vector3D d2u, out Vector3D d2v, out Vector3D d2uv, out Vector3D d3u,
                 out Vector3D d3v, out Vector3D d3uuv, out Vector3D d3uvv);
        _cons.D3(x[3], out _, out _, out _, out Vector3D d3rst);
        _tguide.D3(prm, out _, out _, out _, out Vector3D d3gui);
        _tevol.D2(prm, out _, out _, out double d2ray);
        d2ray = _sg1 * d2ray;

        // The reversed unit normal's second derivatives, from those of d1u ^ d1v
        Vector3D nu = d2u.Crossed(d1v).Added(d1u.Crossed(d2uv)), nv = d2uv.Crossed(d1v).Added(d1u.Crossed(d2v));
        Vector3D nuu = d3u.Crossed(d1v) + 2 * d2u.Crossed(d2uv) + d1u.Crossed(d3uuv);
        Vector3D nuv = d3uuv.Crossed(d1v) + d2u.Crossed(d2v) + d1u.Crossed(d3uvv);
        Vector3D nvv = d3uvv.Crossed(d1v) + 2 * d2uv.Crossed(d2v) + d1u.Crossed(d3v);
        Vector3D dnuu = -ConstRad.UnitNormalD2(_nn, _invnorm, nu, nu, nuu);
        Vector3D dnuv = -ConstRad.UnitNormalD2(_nn, _invnorm, nu, nv, nuv);
        Vector3D dnvv = -ConstRad.UnitNormalD2(_nn, _invnorm, nv, nv, nvv);
        Vector3D d2trst = ConstRad.UnitNormalD2(_trst, _invnormrst, _d2rst, _d2rst, d3rst);
        Vector3D d2nplan = ConstRad.UnitNormalD2(_nplan, 1.0 / _normtg, _d2gui, _d2gui, d3gui);

        // Each quantity's derivative along the walk
        Vector3D tgs = Vector3D.LinearForm(up, d1u, vp, d1v);
        Vector3D wd1u = Vector3D.LinearForm(up, d2u, vp, d2uv), wd1v = Vector3D.LinearForm(up, d2uv, vp, d2v);
        Vector3D dnw = Vector3D.LinearForm(up, _dnu, vp, _dnv);
        Vector3D wdnu = Vector3D.LinearForm(up, dnuu, vp, dnuv), wdnv = Vector3D.LinearForm(up, dnuv, vp, dnvv);
        Vector3D wptrst = tp * _drst, wdrst = tp * _d2rst, wtrst = tp * _dtrst, wdtrst = tp * d2trst;
        Vector3D dc = Vector3D.LinearForm(_ray, dnw, _dray, -_nn, tgs);
        Vector3D corde = new(_ptrst, _center), wcorde = dc - wptrst;
        Vector3D dcu = d1u + _ray * _dnu, dcv = d1v + _ray * _dnv;
        Vector3D wdcu = wd1u + _dray * _dnu + _ray * wdnu, wdcv = wd1v + _dray * _dnv + _ray * wdnv;

        // J' x'
        double m1 = ((dnplan.Dot(d1u) + _nplan.Dot(wd1u)) * up + (dnplan.Dot(d1v) + _nplan.Dot(wd1v)) * vp
                     + (dnplan.Dot(_drst) + _nplan.Dot(wdrst)) * tp) / 2;
        double m2 = (wdcu.Dot(_trst) + dcu.Dot(wtrst)) * up + (wdcv.Dot(_trst) + dcv.Dot(wtrst)) * vp
                    + (wcorde.Dot(_dtrst) + corde.Dot(wdtrst) - wdrst.Dot(_trst) - _drst.Dot(wtrst)) * tp;
        double m3 = 2 * (wcorde.Dot(dcu) + corde.Dot(wdcu)) * up + 2 * (wcorde.Dot(dcv) + corde.Dot(wdcv)) * vp
                    - 2 * (wcorde.Dot(_drst) + corde.Dot(wdrst)) * tp;

        // s'
        Vector3D temp = new((_pts.XYZ() + _ptrst.XYZ()) / 2 - _ptgui.XYZ());
        Vector3D wtemp = (tgs + wptrst) / 2 - _d1gui;
        rhs[1] = _nplan.Dot(_d2gui) - d2nplan.Dot(temp) - dnplan.Dot(wtemp) - m1;
        rhs[2] = d2ray * _nn.Dot(_trst) + _dray * (_nn.Dot(wtrst) - dnw.Dot(_trst)) - m2;
        rhs[3] = 2 * d2ray * corde.Dot(_nn) + 2 * _dray * (wcorde.Dot(_nn) - corde.Dot(dnw)) + 2 * _dray * _dray + 2 * _ray * d2ray - m3;
        resol.Solve(rhs);
        double upp = rhs[1], vpp = rhs[2];

        Vector3D wtgs = up * wd1u + upp * d1u + vp * wd1v + vpp * d1v;
        Vector3D wdnw = up * wdnu + upp * _dnu + vp * wdnv + vpp * _dnv;
        d2c = wtgs - d2ray * _nn + 2 * _dray * dnw + _ray * wdnw;
        return true;
    }

    public bool SectionGap(in BlendPoint near, double prm, double before, double after, out double gap, out double radius)
    {
        math_Vector x = new(1, 3);
        near.ParametersOnS(out x[1], out x[2]);
        x[3] = near.ParameterOnC();
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

    public bool SectionEnd(in BlendPoint at, out EnvelopeEnd end) => End(at, at.Parameter(), out end);

    public bool LeastAdvance(in BlendPoint near, double prm, out double advance)
    {
        bool ok = End(near, prm, out EnvelopeEnd end);
        advance = ok ? end.LeastAdvance() : 0.0;
        return ok;
    }

    private bool End(in BlendPoint near, double prm, out EnvelopeEnd end)
    {
        math_Vector y = new(1, 3), dx = new(1, 3);
        near.ParametersOnS(out y[1], out y[2]);
        y[3] = near.ParameterOnC();
        end = default;
        Position(prm);
        // A point of the walk is its own solution: Newton's method only between the walk's points.
        if ((prm != near.Parameter() && !SolveAt(y)) || !WalkCentreD2(prm, y, out Vector3D d2c) || !WalkCentre(prm, y, dx, out Vector3D dc)
            || !Envelope(dc, out EnvelopeCircle e))
        {
            return false;
        }

        _tevol.D2(prm, out _, out _, out double d2ray);
        end = new EnvelopeEnd(e, _center, dc, d2c, _ray, _dray, _sg1 * d2ray);
        return true;
    }

    // The walk's solution at the parameter the function is positioned at, by Newton's method from y, which it replaces.
    private bool SolveAt(math_Vector y)
    {
        math_Vector f = new(1, 3), step = new(1, 3);
        Matrix grad = new(1, 3, 1, 3);
        bool small = false;
        for (int iter = 0; iter < 12 && !small; iter++)
        {
            Ball(y, out Vector3D d1u1, out Vector3D d1v1);
            BallValues(f);
            BallDerivatives(grad, d1u1, d1v1);
            Gauss resol = new(grad, 1.0e-14);
            if (!resol.IsDone)
            {
                return false;
            }

            resol.Solve(-f, step);
            small = true;
            for (int i = 1; i <= 3; i++)
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
        math_Vector y = new(1, 3), dx = new(1, 3);
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
        return ConstRad.SectionNormal(new Vector3D(_center, _pts), new Vector3D(_center, _ptrst), np);
    }

    // The unit vectors from the ball's centre to its contacts at the last positioning: along the surface's normal, and
    // along the chord to the restriction.
    private void Directions(out Vector3D u1, out Vector3D u2)
    {
        u1 = _ray > 0.0 ? _nn : -_nn;
        Vector3D corde = new(_center, _ptrst);
        u2 = ConstRad.InverseNorm(corde) * corde;
    }

    // The envelope's circle at the last positioning, from the centre's derivative along the walk.
    private bool Envelope(in Vector3D dc, out EnvelopeCircle circle)
    {
        Vector3D zero = new Vector3D();
        Directions(out Vector3D u1, out Vector3D u2);
        return EnvelopeCircle.Make(0, _center, dc, zero, zero, _ray, _dray, 0.0, 0.0, u1, zero, zero, u2, zero, zero, Side(),
                                   out circle);
    }

    public override void GetTolerance(math_Vector tolerance, double tol)
    {
        tolerance[1] = _surf.UResolution(tol);
        tolerance[2] = _surf.VResolution(tol);
        tolerance[3] = _cons.Resolution(tol);
    }

    public override void GetBounds(math_Vector infBound, math_Vector supBound)
    {
        infBound[1] = _surf.FirstUParameter();
        infBound[2] = _surf.FirstVParameter();
        infBound[3] = _cons.FirstParameter();
        supBound[1] = _surf.LastUParameter();
        supBound[2] = _surf.LastVParameter();
        supBound[3] = _cons.LastParameter();

        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(infBound[1]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(supBound[1]))
        {
            double range = (supBound[1] - infBound[1]);
            infBound[1] -= range;
            supBound[1] += range;
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(infBound[2]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(supBound[2]))
        {
            double range = (supBound[2] - infBound[2]);
            infBound[2] -= range;
            supBound[2] += range;
        }
    }

    /// <summary>
    /// True if <paramref name="sol"/> solves the function within <paramref name="tol"/>; then computes the tangents
    /// and updates the extreme angles and the minimal distance of the sections (called once per accepted point).
    /// </summary>
    public override bool IsSolution(math_Vector sol, double tol)
    {
        math_Vector valsol = new(1, 3), secmember = new(1, 3);
        Matrix gradsol = new(1, 3, 1, 3);

        Vector3D dnplan, d1u1, d1v1, d1urst, d1vrst, d1, ns, ns2, resul;
        Point3D bid;
        double cosa, sina, angle;

        Ball(sol, out d1u1, out d1v1);
        BallValues(valsol);
        BallDerivatives(gradsol, d1u1, d1v1);
        if (Math.Abs(valsol[1]) <= tol && Math.Abs(valsol[2]) <= tol && Math.Abs(valsol[3]) <= 2 * tol * Math.Abs(_ray))
        {
            // Calculation of tangents

            _pt2ds = new Point2D(sol[1], sol[2]);
            _prmrst = sol[3];
            _pt2drst = _rst.Value(_prmrst);
            d1 = _drst;
            dnplan = Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);
            GuideSecondMember(secmember, dnplan);

            // The ball's (decision 2158): the reversed unit normal, and the centre's chord to the restriction.
            ns = -_nn;
            resul = new Vector3D(_ptrst, _center);

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
                    math_Vector dedt = new(1, 3);
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
                _tgs = Vector3D.LinearForm(secmember[1], d1u1, secmember[2], d1v1);
                _tgrst = secmember[3] * d1;
                _tg2ds = _tg2ds.WithCoord(secmember[1], secmember[2]);
                _surfrst.D1(_pt2drst.X, _pt2drst.Y, out bid, out d1urst, out d1vrst);
                double a, b;
                T3dto2d(out a, out b, _tgrst, d1urst, d1vrst);
                _tg2drst = _tg2drst.WithCoord(a, b);
            }

            // Kerf (unit 15AK): the section is the envelope's circle, so the angle below is its arc's.
            EnvelopeCircle envelope = default;
            bool hasEnvelope = false;
            if (!_istangent)
            {
                Vector3D dnw = Vector3D.LinearForm(secmember[1], _dnu, secmember[2], _dnv);
                hasEnvelope = Envelope(Vector3D.LinearForm(_ray, dnw, _dray, ns, _tgs), out envelope);
            }

            // update of maxang
            if (_ray > 0.0)
            {
                ns = ns.Reversed();
            }
            ns2 = -resul.Normalized();

            cosa = ns.Dot(ns2);
            sina = _nplan.Dot(ns.Crossed(ns2));
            if (_choix % 2 != 0)
            {
                sina = -sina; // nplan is changed to -nplan
            }

            angle = Math.Acos(cosa);
            if (sina < 0.0)
            {
                angle = 2.0 * M_PI - angle;
            }

            if (hasEnvelope)
            {
                angle = envelope.Angle();
                if (angle < 0.0)
                {
                    angle += 2.0 * M_PI;
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
            _distmin = Min(_distmin, _pts.DistanceTo(_ptrst));

            return true;
        }
        _istangent = true;
        return false;
    }

    public override double GetMinimalDistance()
    {
        return _distmin;
    }

    public override Point3D PointOnS() => _pts;

    public override Point3D PointOnRst() => _ptrst;

    public override Point2D Pnt2dOnS() => _pt2ds;

    public override Point2D Pnt2dOnRst() => _pt2drst;

    public override double ParameterOnRst() => _prmrst;

    public override bool IsTangencyPoint => _istangent;

    public override Vector3D TangentOnS()
    {
        if (_istangent)
        {
            throw new DomainErrorException();
        }
        return _tgs;
    }

    public override Vector2D Tangent2dOnS()
    {
        if (_istangent)
        {
            throw new DomainErrorException();
        }
        return _tg2ds;
    }

    public override Vector3D TangentOnRst()
    {
        if (_istangent)
        {
            throw new DomainErrorException();
        }
        return _tgrst;
    }

    public override Vector2D Tangent2dOnRst()
    {
        if (_istangent)
        {
            throw new DomainErrorException();
        }
        return _tg2drst;
    }

    /// <summary>True if the section at <paramref name="sol"/> leaves the reference surface (see <see cref="Set(SurfaceAdaptorBase, Curve2DAdaptorBase)"/>).</summary>
    public override bool Decroch(math_Vector sol, out Vector3D ns, out Vector3D tgS)
    {
        Vector3D tgRst, nRst, nRstInPlane, nsInPlane, np;
        Point3D bid, center = new();
        Vector3D d1u, d1v;
        double norm;

        _surf.D1(sol[1], sol[2], out bid, out d1u, out d1v);
        ns = d1u.Crossed(d1v);

        // The ball's (decision 2158): the centre along the surface's own normal, the section in the plane square to
        // that normal and to the centre's chord to the restriction, the reference surface's normal projected into it.
        nsInPlane = -ConstRad.InverseNorm(ns) * ns;
        center = center.WithXYZ(bid.XYZ() + _ray * nsInPlane.XYZ());
        np = ConstRad.SectionNormal(ns, new Vector3D(center, _cons.Value(sol[3])), _nplan);
        if (_choix > 2)
        {
            nsInPlane = nsInPlane.Reversed();
        }
        tgS = np.Crossed(new Vector3D(center, bid));
        if (_choix % 2 == 1)
        {
            tgS = tgS.Reversed();
        }
        double u, v;
        _rstref!.Value(sol[3]).Coord(out u, out v);
        _surfref!.D1(u, v, out bid, out d1u, out d1v);
        nRst = d1u.Crossed(d1v);
        nRstInPlane = nRst - np.Dot(nRst) * np;
        norm = nRstInPlane.Magnitude;
        if (norm < Eps)
        {
            return false; // Singularity or Incoherence.
        }
        nRstInPlane /= norm;
        Vector3D centptrst = new(center, bid);
        if (centptrst.Dot(nRstInPlane) < 0.0)
        {
            nRstInPlane = nRstInPlane.Reversed();
        }
        tgRst = np.Crossed(centptrst);
        if (_choix % 2 == 1)
        {
            tgRst = tgRst.Reversed();
        }

        double dot, nt = nRstInPlane.Magnitude;
        nt *= tgRst.Magnitude;
        if (Math.Abs(nt) < 1.0e-7)
        {
            return false; // Singularity or Incoherence.
        }
        dot = nRstInPlane.Dot(tgRst);
        dot /= nt;

        return (dot < 1.0e-10);
    }

    /// <summary>Sets the "quadrant" <paramref name="choix"/> (1 to 4) in which the fillet lies.</summary>
    public void Set(int choix)
    {
        _choix = choix;
        switch (_choix)
        {
            case 1:
            case 2:
                _sg1 = -1;
                break;
            case 3:
            case 4:
                _sg1 = 1;
                break;
            default:
                _sg1 = -1;
                break;
        }
    }

    /// <summary>Sets the type of section generation for the approximations.</summary>
    public void Set(SectionShape typeSection)
    {
        _sShape = typeSection;
    }

    /// <summary>The circular section at <paramref name="param"/> through the points (U, V) and W, and its bounds.</summary>
    public void Section(double param, double u, double v, double w, out double pdeb, out double pfin, ref Circle3D c)
    {
        Vector3D d1u1, d1v1;
        Vector3D ns, np;
        Point3D center = new();

        // Kerf (unit 15AK): the envelope's circle; the ball's great circle where the walk has no derivative.
        math_Vector x = new(1, 3), dx = new(1, 3);
        x[1] = u;
        x[2] = v;
        x[3] = w;
        if (WalkCentre(param, x, dx, out Vector3D dc) && Envelope(dc, out EnvelopeCircle e))
        {
            Vector3D plane = e.Plane;
            c = c.WithRadius(e.Radius);
            c = c.WithPosition(new Frame3D(e.Centre, new Direction3D(plane), new Direction3D(e.N1)));
            pdeb = 0;
            pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, _ptrst);
            if (pfin > 1.5 * M_PI)
            {
                plane = plane.Reversed();
                c = c.WithPosition(new Frame3D(e.Centre, new Direction3D(plane), new Direction3D(e.N1)));
                pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, _ptrst);
            }
            if (pfin < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                pfin += global::Kerf.Engine.Foundation.Precision.PConfusion();
            }
            return;
        }

        _tguide.D1(param, out _ptgui, out _d1gui);
        np = _d1gui.Normalized();
        _ray = _sg1 * _tevol.Value(param);

        _surf.D1(u, v, out _pts, out d1u1, out d1v1);
        _ptrst = _cons.Value(w);

        // The ball's circle (decision 2158): the surface's own normal, the plane square to it and to the chord.
        ns = d1u1.Crossed(d1v1);
        ns = -ConstRad.InverseNorm(ns) * ns;
        center = center.WithXYZ(_pts.XYZ() + _ray * ns.XYZ());
        np = ConstRad.SectionNormal(ns, new Vector3D(center, _ptrst), np);
        c = c.WithRadius(Math.Abs(_ray));

        if (_ray > 0)
        {
            ns = ns.Reversed();
        }

        if (_choix % 2 != 0)
        {
            np = np.Reversed();
        }

        c = c.WithPosition(new Frame3D(center, new Direction3D(np), new Direction3D(ns)));
        pdeb = 0; // ElCLib::Parameter(C,pts);
        pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, _ptrst);

        // Test negative and almost null angles : Special case
        if (pfin > 1.5 * M_PI)
        {
            np = np.Reversed();
            c = c.WithPosition(new Frame3D(center, new Direction3D(np), new Direction3D(ns)));
            pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, _ptrst);
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
        Vector3D d1u1, d1v1;
        Vector3D ns, ns2;
        Point3D center = new();

        double u1, v1, w;

        double prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();

        _tguide.D1(prm, out _ptgui, out _d1gui);
        _nplan = _d1gui.Normalized();
        _ray = _sg1 * _tevol.Value(prm);

        p.ParametersOnS(out u1, out v1);
        w = p.ParameterOnC(); // jlr : point on curve not on surface
        Point2D pt2d = _rst.Value(w);

        _surf.D1(u1, v1, out _pts, out d1u1, out d1v1);
        _ptrst = _cons.Value(w);
        _distmin = Min(_distmin, _pts.DistanceTo(_ptrst));

        poles2d[poles2d.Lower()] = poles2d[poles2d.Lower()].WithCoord(u1, v1);
        poles2d[poles2d.Upper()] = poles2d[poles2d.Upper()].WithCoord(pt2d.X, pt2d.Y);

        // Linear Case
        if (_sShape == SectionShape.Linear)
        {
            poles[low] = _pts;
            poles[upp] = _ptrst;
            weights[low] = 1.0;
            weights[upp] = 1.0;
            return;
        }

        // Kerf (unit 15AK): the envelope's circle; the ball's great circle where the walk has no derivative.
        math_Vector x = new(1, 3), dx = new(1, 3);
        x[1] = u1;
        x[2] = v1;
        x[3] = w;
        if (WalkCentre(prm, x, dx, out Vector3D dc) && Envelope(dc, out EnvelopeCircle e))
        {
            global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, e.N1, e.N2, e.Plane, _pts, _ptrst, e.Radius, e.Centre, poles, weights);
            return;
        }

        // The ball's circle (decision 2158): the surface's own normal, the plane square to it and to the chord.
        ns = d1u1.Crossed(d1v1);
        ns = -ConstRad.InverseNorm(ns) * ns;

        center = center.WithXYZ(_pts.XYZ() + _ray * ns.XYZ());

        ns2 = new Vector3D(center, _ptrst).Normalized();
        if (_ray > 0)
        {
            ns = ns.Reversed();
        }
        if (_choix % 2 != 0)
        {
            _nplan = _nplan.Reversed();
        }

        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns, ns2, ConstRad.SectionNormal(ns, ns2, _nplan), _pts, _ptrst,
                                                                    Math.Abs(_ray), center, poles, weights);
    }

    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                                 NCollection_Array1<double> weights, NCollection_Array1<double> dWeights)
    {
        Vector3D d1u1, d1v1, d1;
        Vector3D ns, ns2, dnplan, dnw = new(), dn2w = new();
        Vector3D tgct = new(), resul;
        Vector3D d1urst, d1vrst;
        Point3D center, bid;

        math_Vector sol = new(1, 3), secmember = new(1, 3);
        Matrix gradsol = new(1, 3, 1, 3);

        double prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();
        bool istgt;

        // Kerf (unit 15AK): the centre's second derivative along the walk, which the derivative of the envelope's circle
        // takes; found first, as finding it positions the function afresh.
        Vector3D d2c = new();
        bool second = false;
        if (_sShape != SectionShape.Linear)
        {
            p.ParametersOnS(out sol[1], out sol[2]);
            sol[3] = p.ParameterOnC();
            second = WalkCentreD2(prm, sol, out d2c);
        }

        _tguide.D2(prm, out _ptgui, out _d1gui, out _d2gui);
        _tevol.D1(prm, out _ray, out _dray);
        _ray = _sg1 * _ray;
        _dray = _sg1 * _dray;
        _normtg = _d1gui.Magnitude;
        _nplan = _d1gui.Normalized();
        dnplan = Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

        p.ParametersOnS(out sol[1], out sol[2]);
        sol[3] = _prmrst = p.ParameterOnC();
        _pt2drst = _rst.Value(_prmrst);

        // The ball's (decision 2216): the centre along the surface's own normal, moving with the radius.
        Ball(sol, out d1u1, out d1v1);
        BallDerivatives(gradsol, d1u1, d1v1);
        d1 = _drst;
        GuideSecondMember(secmember, dnplan);
        center = _center;
        resul = new Vector3D(_ptrst, _center);

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
                math_Vector dedt = new(1, 3);
                dedt.Initialized(secmember);
                singRS.Solve(dedt, secmember, 1.0e-6);
                istgt = false;
            }
            else
            {
                istgt = true;
            }
        }

        ns = -_nn;
        ns2 = -resul.Normalized();
        if (!istgt)
        {
            _tgs = Vector3D.LinearForm(secmember[1], d1u1, secmember[2], d1v1);
            _tgrst = secmember[3] * d1;

            // The normal's derivative along the walk, then the unit chord's.
            dnw = Vector3D.LinearForm(secmember[1], _dnu, secmember[2], _dnv);
            dn2w = Vector3D.LinearForm(_ray, dnw, -1.0, _tgrst, _tgs);
            dn2w = Vector3D.LinearForm(_dray, ns, dn2w);
            dn2w = ConstRad.InverseNorm(resul) * dn2w;
            dn2w = Vector3D.LinearForm(ns2.Dot(dn2w), ns2, -1.0, dn2w);
        }

        // Tops 2D

        poles2d[poles2d.Lower()] = poles2d[poles2d.Lower()].WithCoord(sol[1], sol[2]);
        poles2d[poles2d.Upper()] = poles2d[poles2d.Upper()].WithCoord(_pt2drst.X, _pt2drst.Y);
        if (!istgt)
        {
            dPoles2d[poles2d.Lower()] = dPoles2d[poles2d.Lower()].WithCoord(secmember[1], secmember[2]);
            _surfrst.D1(_pt2drst.X, _pt2drst.Y, out bid, out d1urst, out d1vrst);
            double a, b;
            T3dto2d(out a, out b, _tgrst, d1urst, d1vrst);
            dPoles2d[poles2d.Upper()] = dPoles2d[poles2d.Upper()].WithCoord(a, b);
        }

        // Linear Case
        if (_sShape == SectionShape.Linear)
        {
            poles[low] = _pts;
            poles[upp] = _ptrst;
            weights[low] = 1.0;
            weights[upp] = 1.0;
            if (!istgt)
            {
                dPoles[low] = _tgs;
                dPoles[upp] = _tgrst;
                dWeights[low] = 0.0;
                dWeights[upp] = 0.0;
            }
            return (!istgt);
        }

        // Case of the circle: the envelope's (unit 15AK), with its derivative along the walk.
        if (!istgt && second)
        {
            Vector3D zero = new();
            tgct = Vector3D.LinearForm(_ray, dnw, _dray, ns, _tgs);
            _tevol.D2(prm, out _, out _, out double d2ray);
            double f1 = _ray > 0.0 ? -1.0 : 1.0;
            if (EnvelopeCircle.Make(1, center, tgct, d2c, zero, _ray, _dray, _sg1 * d2ray, 0.0, f1 * ns, f1 * dnw, zero, ns2, dn2w, zero,
                                    Side(), out EnvelopeCircle e))
            {
                return global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, e.N1, e.N2, e.DN1, e.DN2, e.Plane, e.DPlane, _pts, _ptrst, _tgs, _tgrst,
                                          e.Radius, e.DRadius, e.Centre, e.DCentre, poles, dPoles, weights, dWeights);
            }
        }

        // Where the envelope has no circle: the ball's great circle (decision 2158), and no derivative.
        if (_ray > 0.0)
        {
            ns = ns.Reversed();
        }
        Vector3D np = _nplan;
        if (_choix % 2 != 0)
        {
            np = np.Reversed();
        }

        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns, ns2, ConstRad.SectionNormal(ns, ns2, np), _pts, _ptrst, Math.Abs(_ray), center, poles, weights);
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
            tolU = _surf.UResolution(tol);
            tolV = _surf.VResolution(tol);
        }
        else
        {
            tolU = _surfrst.UResolution(tol);
            tolV = _surfrst.VResolution(tol);
        }
    }
}
