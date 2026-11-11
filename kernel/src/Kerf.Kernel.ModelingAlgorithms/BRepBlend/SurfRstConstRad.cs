// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstConstRad.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstConstRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.BlendFunctions;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics;
using Kerf.Geometry;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The function of a constant-radius fillet between a surface and a restriction (an arc of another surface) along a
/// guide: the variables are (U, V) on the surface and the parameter on the restriction.
/// </summary>
/// <remarks>A working object (decision 21): it caches its last solution, which the walkers read.</remarks>
internal class SurfRstConstRad : SurfRstFunction
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

    public SurfRstConstRad(SurfaceAdaptorBase surf, SurfaceAdaptorBase surfRst, Curve2DAdaptorBase rst,
                                     CurveAdaptorBase cGuide)
    {
        _surf = surf;
        _surfrst = surfRst;
        _rst = rst;
        _cons = new CurveOnSurface(rst, surfRst);
        _guide = cGuide;
        _tguide = cGuide;
        _prmrst = 0.0;
        _istangent = true;
        _ray = 0.0;
        _choix = 0;
        _normtg = 0.0;
        _d = 0.0;
        _maxang = RealFirst();
        _minang = RealLast();
        _distmin = RealLast();
        _sShape = SectionShape.Rational;
    }

    public override int NbVariables()
    {
        return 3;
    }

    public override int NbEquations()
    {
        return 3;
    }

    // Kerf (decision 30011): the rolling ball's equations, as BlendFunc_ConstRad's (decision 30001). The centre is at
    // the radius from the surface's contact along its own unit normal (OCCT projects the normal into the plane square
    // to the guide); the ball touches the restriction where the centre's foot on it is, so the centre minus that point
    // is square to the restriction's tangent and as long as the radius; the guide's plane only places the section,
    // through the contacts' midpoint, as ConstRad's does, so the two functions meet where one hands over to the other.
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

    // The guide's term of the equations' derivative along the walk, with the sign the solvers take it (-df/dt): only
    // f1 moves with the guide.
    private void GuideSecondMember(math_Vector secmember, in Vector3D dnplan)
    {
        Vector3D temp = new((_pts.XYZ() + _ptrst.XYZ()) / 2 - _ptgui.XYZ());
        secmember[1] = _normtg - dnplan.Dot(temp);
        secmember[2] = 0;
        secmember[3] = 0;
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
    }

    public override void Set(double first, double last)
    {
        _tguide = _guide.Trim(first, last, 1.0e-12);
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

            // The ball's (decision 30011): the reversed unit normal, and the centre's chord to the restriction.
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

        // The ball's (decision 30011): the centre along the surface's own normal, the section in the plane square to
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

    /// <summary>Sets the radius and the "quadrant" <paramref name="choix"/> (1 to 4) in which the fillet lies.</summary>
    public void Set(double radius, int choix)
    {
        _choix = choix;
        switch (_choix)
        {
            case 1:
            case 2:
                _ray = -Math.Abs(radius);
                break;
            case 3:
            case 4:
                _ray = Math.Abs(radius);
                break;
            default:
                _ray = -Math.Abs(radius);
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

        _tguide.D1(param, out _ptgui, out _d1gui);
        np = _d1gui.Normalized();

        _surf.D1(u, v, out _pts, out d1u1, out d1v1);
        _ptrst = _cons.Value(w);

        // The ball's circle (decision 30011): the surface's own normal, the plane square to it and to the chord.
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

    public override int NbIntervals(Continuity s)
    {
        return _guide.NbIntervals(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        _guide.Intervals(t, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
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

        // The ball's circle (decision 30011): the surface's own normal, the plane square to it and to the chord.
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

        double norm;

        math_Vector sol = new(1, 3), secmember = new(1, 3);
        Matrix gradsol = new(1, 3, 1, 3);

        double prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();
        bool istgt;

        _tguide.D2(prm, out _ptgui, out _d1gui, out _d2gui);
        _normtg = _d1gui.Magnitude;
        _nplan = _d1gui.Normalized();
        dnplan = Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

        p.ParametersOnS(out sol[1], out sol[2]);
        sol[3] = _prmrst = p.ParameterOnC();
        _pt2drst = _rst.Value(_prmrst);

        // The ball's (decision 30011): the centre along the surface's own normal, which doesn't move with the guide.
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
            norm = resul.Magnitude;
            dn2w = dn2w.Divided(norm);
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

        // Case of the circle
        if (!istgt)
        {
            tgct = _tgs.Added(_ray * dnw);
        }

        if (_ray > 0.0)
        {
            ns = ns.Reversed();
            if (!istgt)
            {
                dnw = dnw.Reversed();
            }
        }
        Vector3D np = _nplan;
        if (_choix % 2 != 0)
        {
            np = np.Reversed();
            dnplan = dnplan.Reversed();
        }

        // The plane of the ball's circle, square to the normal and the chord, on the guide plane's side.
        Vector3D zero = new();
        ConstRad.SectionNormal(ns, ns2, dnw, dn2w, zero, zero, np, dnplan, zero, out Vector3D npBall, out Vector3D dnpBall, out _);
        if (!istgt)
        {
            return global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns, ns2, dnw, dn2w, npBall, dnpBall, _pts, _ptrst, _tgs, _tgrst,
                                      Math.Abs(_ray), 0, center, tgct, poles, dPoles, weights, dWeights);
        }
        else
        {
            global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns, ns2, npBall, _pts, _ptrst, Math.Abs(_ray), center, poles, weights);
            return false;
        }
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
