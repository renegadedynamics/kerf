// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstConstRad.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstConstRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.BlendFunctions;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Fillets.BlendFunctions.SectionShape;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The function of a constant-radius fillet between a surface and a restriction (an arc of another surface) along a
/// guide: the variables are (U, V) on the surface and the parameter on the restriction.
/// </summary>
/// <remarks>A working object (decision 21): it caches its last solution, which the walkers read.</remarks>
public class SurfRstConstRad : SurfRstFunction
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

    public override bool Value(math_Vector x, math_Vector f)
    {
        Vector3D d1u1, d1v1, ns, vref = new();
        double norm;

        _surf.D1(x[1], x[2], out _pts, out d1u1, out d1v1);
        _ptrst = _cons.Value(x[3]);

        f[1] = _nplan.XYZ().Dot(_pts.XYZ()) + _d;
        f[2] = _nplan.XYZ().Dot(_ptrst.XYZ()) + _d;

        ns = d1u1.Crossed(d1v1);
        norm = _nplan.Crossed(ns).Magnitude();
        ns = Vector3D.LinearForm(_nplan.Dot(ns) / norm, _nplan, -1.0 / norm, ns);
        vref = Vector3D.LinearForm(_ray, ns, new Vector3D(_ptrst, _pts));
        vref /= _ray;
        f[3] = (vref.SquareMagnitude() - 1) * _ray * _ray;
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        Vector3D d1u1, d1v1, d2u1, d2v1, d2uv1, d1;
        Vector3D ns, ncrossns, resul = new(), temp, vref = new();

        double norm, ndotns, grosterme;

        _surf.D2(x[1], x[2], out _pts, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
        _cons.D1(x[3], out _ptrst, out d1);

        d[1, 1] = _nplan.Dot(d1u1);
        d[1, 2] = _nplan.Dot(d1v1);
        d[1, 3] = 0.0;

        d[2, 1] = 0.0;
        d[2, 2] = 0.0;
        d[2, 3] = _nplan.Dot(d1);

        ns = d1u1.Crossed(d1v1);
        ncrossns = _nplan.Crossed(ns);
        norm = ncrossns.Magnitude();
        ndotns = _nplan.Dot(ns);

        vref = Vector3D.LinearForm(ndotns, _nplan, -1.0, ns);
        vref = vref.Divided(norm);
        vref = Vector3D.LinearForm(_ray, vref, new Vector3D(_ptrst, _pts));

        // Derivative by u1
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
        resul = Vector3D.LinearForm(-_ray / norm * (grosterme * ndotns - _nplan.Dot(temp)), _nplan, _ray * grosterme / norm, ns,
                            -_ray / norm, temp, d1u1);

        d[3, 1] = resul.Dot(vref);
        d[3, 1] = d[3, 1] * 2.0;

        // Derivative by v1
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
        resul = Vector3D.LinearForm(-_ray / norm * (grosterme * ndotns - _nplan.Dot(temp)), _nplan, _ray * grosterme / norm, ns,
                            -_ray / norm, temp, d1v1);

        d[3, 2] = resul.Dot(vref);
        d[3, 2] = d[3, 2] * 2.0;

        d[3, 3] = d1.Dot(vref);
        d[3, 3] = d[3, 3] * (-2.0);

        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        Vector3D d1u1, d1v1, d1;
        Vector3D d2u1, d2v1, d2uv1;
        Vector3D ns, ncrossns, resul = new(), temp, vref = new();

        double norm, ndotns, grosterme;

        _surf.D2(x[1], x[2], out _pts, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
        _cons.D1(x[3], out _ptrst, out d1);

        f[1] = _nplan.XYZ().Dot(_pts.XYZ()) + _d;
        f[2] = _nplan.XYZ().Dot(_ptrst.XYZ()) + _d;

        d[1, 1] = _nplan.Dot(d1u1);
        d[1, 2] = _nplan.Dot(d1v1);
        d[1, 3] = 0.0;

        d[2, 1] = 0.0;
        d[2, 2] = 0.0;
        d[2, 3] = _nplan.Dot(d1);

        ns = d1u1.Crossed(d1v1);
        ncrossns = _nplan.Crossed(ns);
        norm = ncrossns.Magnitude();
        ndotns = _nplan.Dot(ns);

        vref = Vector3D.LinearForm(ndotns, _nplan, -1.0, ns);
        vref = vref.Divided(norm);
        vref = Vector3D.LinearForm(_ray, vref, new Vector3D(_ptrst, _pts));

        temp = vref / _ray;
        f[3] = (temp.SquareMagnitude() - 1) * _ray * _ray; // more stable numerically

        // Derivative by u1
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
        resul = Vector3D.LinearForm(-_ray / norm * (grosterme * ndotns - _nplan.Dot(temp)), _nplan, _ray * grosterme / norm, ns,
                            -_ray / norm, temp, d1u1);

        d[3, 1] = resul.Dot(vref);
        d[3, 1] = d[3, 1] * 2.0;

        // Derivative by v1
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
        resul = Vector3D.LinearForm(-_ray / norm * (grosterme * ndotns - _nplan.Dot(temp)), _nplan, _ray * grosterme / norm, ns,
                            -_ray / norm, temp, d1v1);

        d[3, 2] = resul.Dot(vref);
        d[3, 2] = d[3, 2] * 2.0;

        d[3, 3] = d1.Dot(vref);
        d[3, 3] = d[3, 3] * (-2.0);

        return true;
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
        _normtg = _d1gui.Magnitude();
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

        Vector3D dnplan = new(), d1u1, d1v1, d1urst, d1vrst, d1, temp = new(), ns, ns2, ncrossns, resul = new();
        Point3D bid;
        double norm, ndotns, grosterme;
        double cosa, sina, angle;

        Values(sol, valsol, gradsol);
        if (Math.Abs(valsol[1]) <= tol && Math.Abs(valsol[2]) <= tol && Math.Abs(valsol[3]) <= 2 * tol * Math.Abs(_ray))
        {
            // Calculation of tangents

            _pt2ds = new Point2D(sol[1], sol[2]);
            _prmrst = sol[3];
            _pt2drst = _rst.Value(_prmrst);
            _surf.D1(sol[1], sol[2], out _pts, out d1u1, out d1v1);
            _cons.D1(sol[3], out _ptrst, out d1);
            dnplan = Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

            temp = temp.WithXYZ(_pts.XYZ() - _ptgui.XYZ());
            secmember[1] = _normtg - dnplan.Dot(temp);

            temp = temp.WithXYZ(_ptrst.XYZ() - _ptgui.XYZ());
            secmember[2] = _normtg - dnplan.Dot(temp);

            ns = d1u1.Crossed(d1v1);
            ncrossns = _nplan.Crossed(ns);
            ndotns = _nplan.Dot(ns);
            norm = ncrossns.Magnitude();

            grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
            temp = Vector3D.LinearForm(_ray / norm * (dnplan.Dot(ns) - grosterme * ndotns), _nplan, _ray * ndotns / norm, dnplan,
                               _ray * grosterme / norm, ns);

            ns = Vector3D.LinearForm(ndotns / norm, _nplan, -1.0 / norm, ns);
            resul = Vector3D.LinearForm(_ray, ns, new Vector3D(_ptrst, _pts));
            secmember[3] = -2.0 * (temp.Dot(resul));

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
                _surfrst.D1(_pt2drst.X(), _pt2drst.Y(), out bid, out d1urst, out d1vrst);
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
            _distmin = Min(_distmin, _pts.Distance(_ptrst));

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
        Vector3D tgRst, nRst, nRstInPlane, nsInPlane;
        Point3D bid, center = new();
        Vector3D d1u, d1v;
        double norm, unsurnorm;

        _surf.D1(sol[1], sol[2], out bid, out d1u, out d1v);
        ns = nsInPlane = d1u.Crossed(d1v);

        norm = _nplan.Crossed(ns).Magnitude();
        unsurnorm = 1.0 / norm;
        nsInPlane = Vector3D.LinearForm(_nplan.Dot(ns) * unsurnorm, _nplan, -unsurnorm, ns);

        center = center.WithXYZ(bid.XYZ() + _ray * nsInPlane.XYZ());
        if (_choix > 2)
        {
            nsInPlane = nsInPlane.Reversed();
        }
        tgS = _nplan.Crossed(new Vector3D(center, bid));
        if (_choix % 2 == 1)
        {
            tgS = tgS.Reversed();
        }
        double u, v;
        _rstref!.Value(sol[3]).Coord(out u, out v);
        _surfref!.D1(u, v, out bid, out d1u, out d1v);
        nRst = d1u.Crossed(d1v);
        norm = _nplan.Crossed(nRst).Magnitude();
        unsurnorm = 1.0 / norm;
        nRstInPlane = new Vector3D();
        nRstInPlane = Vector3D.LinearForm(_nplan.Dot(nRst) * unsurnorm, _nplan, -unsurnorm, nRst);
        Vector3D centptrst = new(center, bid);
        if (centptrst.Dot(nRstInPlane) < 0.0)
        {
            nRstInPlane = nRstInPlane.Reversed();
        }
        tgRst = _nplan.Crossed(centptrst);
        if (_choix % 2 == 1)
        {
            tgRst = tgRst.Reversed();
        }

        double dot, nt = nRstInPlane.Magnitude();
        nt *= tgRst.Magnitude();
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
        double norm;
        Point3D center = new();

        _tguide.D1(param, out _ptgui, out _d1gui);
        np = _d1gui.Normalized();

        _surf.D1(u, v, out _pts, out d1u1, out d1v1);
        _ptrst = _cons.Value(w);

        ns = d1u1.Crossed(d1v1);

        norm = _nplan.Crossed(ns).Magnitude();
        ns = Vector3D.LinearForm(_nplan.Dot(ns) / norm, _nplan, -1.0 / norm, ns);
        center = center.WithXYZ(_pts.XYZ() + _ray * ns.XYZ());
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

        double norm, u1, v1, w;

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
        _distmin = Min(_distmin, _pts.Distance(_ptrst));

        poles2d[poles2d.Lower()] = poles2d[poles2d.Lower()].WithCoord(u1, v1);
        poles2d[poles2d.Upper()] = poles2d[poles2d.Upper()].WithCoord(pt2d.X(), pt2d.Y());

        // Linear Case
        if (_sShape == SectionShape.Linear)
        {
            poles[low] = _pts;
            poles[upp] = _ptrst;
            weights[low] = 1.0;
            weights[upp] = 1.0;
            return;
        }

        ns = d1u1.Crossed(d1v1);
        norm = _nplan.Crossed(ns).Magnitude();

        ns = Vector3D.LinearForm(_nplan.Dot(ns) / norm, _nplan, -1.0 / norm, ns);

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

        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns, ns2, _nplan, _pts, _ptrst, Math.Abs(_ray), center, poles, weights);
    }

    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                                 NCollection_Array1<double> weights, NCollection_Array1<double> dWeights)
    {
        Vector3D d1u1, d1v1, d2u1, d2v1, d2uv1, d1;
        Vector3D ns, ns2, dnplan = new(), dnw = new(), dn2w = new();
        Vector3D ncrossns;
        Vector3D resulu = new(), resulv = new(), temp = new(), tgct = new(), resul = new();
        Vector3D d1urst, d1vrst;
        Point3D center = new(), bid;

        double norm, ndotns, grosterme;

        math_Vector sol = new(1, 3), valsol = new(1, 3), secmember = new(1, 3);
        Matrix gradsol = new(1, 3, 1, 3);

        double prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();
        bool istgt;

        _tguide.D2(prm, out _ptgui, out _d1gui, out _d2gui);
        _normtg = _d1gui.Magnitude();
        _nplan = _d1gui.Normalized();
        dnplan = Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

        p.ParametersOnS(out sol[1], out sol[2]);
        sol[3] = _prmrst = p.ParameterOnC();
        _pt2drst = _rst.Value(_prmrst);

        Values(sol, valsol, gradsol);

        _surf.D2(sol[1], sol[2], out _pts, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
        _cons.D1(sol[3], out _ptrst, out d1);

        temp = temp.WithXYZ(_pts.XYZ() - _ptgui.XYZ());
        secmember[1] = _normtg - dnplan.Dot(temp);

        temp = temp.WithXYZ(_ptrst.XYZ() - _ptgui.XYZ());
        secmember[2] = _normtg - dnplan.Dot(temp);

        ns = d1u1.Crossed(d1v1);
        ncrossns = _nplan.Crossed(ns);
        ndotns = _nplan.Dot(ns);
        norm = ncrossns.Magnitude();
        if (norm < Eps)
        {
            norm = 1; // Not enough, but it is not necessary to stop
            // OCCT_DEBUG trace left out.
        }

        // Derivative of n1 corresponding to w

        grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
        dnw = Vector3D.LinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, _nplan, ndotns / norm, dnplan,
                          grosterme / norm, ns);

        temp = Vector3D.LinearForm(ndotns / norm, _nplan, -1.0 / norm, ns);
        resul = Vector3D.LinearForm(_ray, temp, new Vector3D(_ptrst, _pts));
        secmember[3] = dnw.Dot(resul);
        secmember[3] = -2.0 * _ray * secmember[3];

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

        if (!istgt)
        {
            _tgs = Vector3D.LinearForm(secmember[1], d1u1, secmember[2], d1v1);
            _tgrst = secmember[3] * d1;

            // Derivative of n1 corresponding to u1
            temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
            grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
            resulu = Vector3D.LinearForm(-(grosterme * ndotns - _nplan.Dot(temp)) / norm, _nplan, grosterme / norm, ns,
                                 -1.0 / norm, temp);

            // Derivative of n1 corresponding to v1
            temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
            grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
            resulv = Vector3D.LinearForm(-(grosterme * ndotns - _nplan.Dot(temp)) / norm, _nplan, grosterme / norm, ns,
                                 -1.0 / norm, temp);

            dnw = Vector3D.LinearForm(secmember[1], resulu, secmember[2], resulv, dnw);
            ns = Vector3D.LinearForm(ndotns / norm, _nplan, -1.0 / norm, ns);

            dn2w = Vector3D.LinearForm(_ray, dnw, -1.0, _tgrst, _tgs);
            norm = resul.Magnitude();
            dn2w = dn2w.Divided(norm);
            ns2 = -resul.Normalized();
            dn2w = Vector3D.LinearForm(ns2.Dot(dn2w), ns2, -1.0, dn2w);
        }
        else
        {
            ns = Vector3D.LinearForm(ndotns / norm, _nplan, -1.0 / norm, ns);
            ns2 = -resul.Normalized();
        }

        // Tops 2D

        poles2d[poles2d.Lower()] = poles2d[poles2d.Lower()].WithCoord(sol[1], sol[2]);
        poles2d[poles2d.Upper()] = poles2d[poles2d.Upper()].WithCoord(_pt2drst.X(), _pt2drst.Y());
        if (!istgt)
        {
            dPoles2d[poles2d.Lower()] = dPoles2d[poles2d.Lower()].WithCoord(secmember[1], secmember[2]);
            _surfrst.D1(_pt2drst.X(), _pt2drst.Y(), out bid, out d1urst, out d1vrst);
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
        center = center.WithXYZ(_pts.XYZ() + _ray * ns.XYZ());
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
        if (_choix % 2 != 0)
        {
            _nplan = _nplan.Reversed();
            dnplan = dnplan.Reversed();
        }
        if (!istgt)
        {
            return global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns, ns2, dnw, dn2w, _nplan, dnplan, _pts, _ptrst, _tgs, _tgrst,
                                      Math.Abs(_ray), 0, center, tgct, poles, dPoles, weights, dWeights);
        }
        else
        {
            global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns, ns2, _nplan, _pts, _ptrst, Math.Abs(_ray), center, poles, weights);
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
