// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_CSConstRad.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_CSConstRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Modified 10/09/1996 PMN Ajout de (Nb)Intervalles, IsRationnal
//                       + Utilisation de GeomFill::GetCircle dans Section.

using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Fillets.BlendFunctions.SectionShape;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>
/// The function of a constant-radius fillet between a surface and a curve along a guide curve: its variables are
/// U, V on the surface and W on the curve.
/// </summary>
public class CSConstRad : CSFunction
{
    private readonly SurfaceAdaptorBase _surf;
    private readonly CurveAdaptorBase _curv;
    private readonly CurveAdaptorBase _guide;
    private Point3D _pts;
    private Point3D _ptc;
    private Point2D _pt2d;
    private double _prmc;
    private bool _istangent;
    private Vector3D _tgs;
    private Vector2D _tg2d;
    private Vector3D _tgc;
    private double _ray;
    private int _choix;
    private Point3D _ptgui;
    private Vector3D _d1gui;
    private Vector3D _d2gui;
    private Vector3D _nplan;
    private double _normtg;
    private double _d;
    private double _maxang;
    private double _minang;
    private SectionShape _sShape;
    private ParameterisationType _tConv;

    public CSConstRad(SurfaceAdaptorBase s, CurveAdaptorBase c, CurveAdaptorBase cGuide)
    {
        _surf = s;
        _curv = c;
        _guide = cGuide;
        _prmc = 0.0;
        _istangent = true;
        _ray = 0.0;
        _choix = 0;
        _normtg = 0.0;
        _d = 0.0;
        _maxang = RealFirst();
        _minang = RealLast();
        _sShape = SectionShape.Rational;
        _tConv = TgtThetaOver2;
    }

    /// <summary>Returns 3.</summary>
    public override int NbEquations()
    {
        return 3;
    }

    /// <summary>Sets the radius and the "quadrant".</summary>
    public void Set(double radius, int choix)
    {
        _choix = choix;
        _ray = -Math.Abs(radius);
    }

    /// <summary>Sets the type of the sections the approximation makes.</summary>
    public void Set(SectionShape typeSection)
    {
        _sShape = typeSection;
    }

    public override void Set(double param)
    {
        _guide.D2(param, out _ptgui, out _d1gui, out _d2gui);
        _normtg = _d1gui.Magnitude;
        _nplan = _d1gui.Normalized();
        _d = -(_nplan.XYZ().Dot(_ptgui.XYZ()));
    }

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override void Set(double first, double last)
    {
        throw new MethodNotImplementedException("BlendFunc_CSConstRad::Set");
    }

    public override void GetTolerance(math_Vector tolerance, double tol)
    {
        tolerance[1] = _surf.UResolution(tol);
        tolerance[2] = _surf.VResolution(tol);
        tolerance[3] = _curv.Resolution(tol);
    }

    public override void GetBounds(math_Vector infBound, math_Vector supBound)
    {
        infBound[1] = _surf.FirstUParameter();
        infBound[2] = _surf.FirstVParameter();
        infBound[3] = _curv.FirstParameter();
        supBound[1] = _surf.LastUParameter();
        supBound[2] = _surf.LastVParameter();
        supBound[3] = _curv.LastParameter();

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

    public override bool IsSolution(math_Vector sol, double tol)
    {
        math_Vector valsol = new(1, 3), secmember = new(1, 3);
        Matrix gradsol = new(1, 3, 1, 3);

        Vector3D dnplan = new Vector3D(), temp = new Vector3D(), ns, ns2, ncrossns, resul = new Vector3D();
        double norm, ndotns, grosterme;
        double cosa, sina, angle;

        Values(sol, valsol, gradsol);
        if (Math.Abs(valsol[1]) <= tol && Math.Abs(valsol[2]) <= tol && Math.Abs(valsol[3]) <= tol * tol)
        {
            // Calcul des tangentes

            _pt2d = new Point2D(sol[1], sol[2]);
            _prmc = sol[3];
            _surf.D1(sol[1], sol[2], out _pts, out Vector3D d1u1, out Vector3D d1v1);
            _curv.D1(sol[3], out _ptc, out Vector3D d1);
            dnplan = Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

            temp = temp.WithXYZ(_pts.XYZ() - _ptgui.XYZ());
            secmember[1] = _normtg - dnplan.Dot(temp);

            temp = temp.WithXYZ(_ptc.XYZ() - _ptgui.XYZ());
            secmember[2] = _normtg - dnplan.Dot(temp);

            ns = d1u1.Crossed(d1v1);
            ncrossns = _nplan.Crossed(ns);
            ndotns = _nplan.Dot(ns);
            norm = ncrossns.Magnitude;

            grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
            temp = Vector3D.LinearForm(_ray / norm * (dnplan.Dot(ns) - grosterme * ndotns), _nplan, _ray * ndotns / norm, dnplan,
                               _ray * grosterme / norm, ns);

            ns = Vector3D.LinearForm(ndotns / norm, _nplan, -1.0 / norm, ns);
            resul = Vector3D.LinearForm(_ray, ns, new Vector3D(_ptc, _pts));

            secmember[3] = -2.0 * (temp.Dot(resul));

            Gauss resol = new(gradsol);
            if (resol.IsDone)
            {
                resol.Solve(secmember);

                _tgs = Vector3D.LinearForm(secmember[1], d1u1, secmember[2], d1v1);
                _tgc = secmember[3] * d1;
                _tg2d = _tg2d.WithCoord(secmember[1], secmember[2]);
                _istangent = false;
            }
            else
            {
                _istangent = true;
            }

            // mise a jour de maxang

            ns2 = -resul.Normalized();

            cosa = ns.Dot(ns2);
            sina = _nplan.Dot(ns.Crossed(ns2));
            if (_choix % 2 != 0)
            {
                sina = -sina; // nplan est change en -nplan
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

            return true;
        }

        _istangent = true;
        return false;
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        _surf.D1(x[1], x[2], out _pts, out Vector3D d1u1, out Vector3D d1v1);
        _ptc = _curv.Value(x[3]);

        f[1] = _nplan.XYZ().Dot(_pts.XYZ()) + _d;
        f[2] = _nplan.XYZ().Dot(_ptc.XYZ()) + _d;

        Vector3D vref = new Vector3D(), ns = d1u1.Crossed(d1v1);
        double norm = _nplan.Crossed(ns).Magnitude;
        ns = Vector3D.LinearForm(_nplan.Dot(ns) / norm, _nplan, -1.0 / norm, ns);
        vref = Vector3D.LinearForm(_ray, ns, new Vector3D(_ptc, _pts));

        f[3] = vref.SquareMagnitude - _ray * _ray;

        _pt2d = new Point2D(x[1], x[2]);
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        Vector3D ns, ncrossns, resul = new Vector3D(), temp, vref = new Vector3D();

        double norm, ndotns, grosterme;

        _surf.D2(x[1], x[2], out _pts, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1,
                out Vector3D d2uv1);
        _curv.D1(x[3], out _ptc, out Vector3D d1);

        d[1, 1] = _nplan.Dot(d1u1);
        d[1, 2] = _nplan.Dot(d1v1);
        d[1, 3] = 0.0;

        d[2, 1] = 0.0;
        d[2, 2] = 0.0;
        d[2, 3] = _nplan.Dot(d1);

        ns = d1u1.Crossed(d1v1);
        ncrossns = _nplan.Crossed(ns);
        norm = ncrossns.Magnitude;
        ndotns = _nplan.Dot(ns);

        vref = Vector3D.LinearForm(ndotns, _nplan, -1.0, ns);
        vref = vref.Divided(norm);
        vref = Vector3D.LinearForm(_ray, vref, new Vector3D(_ptc, _pts));

        // Derivee par rapport a u1
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
        resul = Vector3D.LinearForm(-_ray / norm * (grosterme * ndotns - _nplan.Dot(temp)), _nplan, _ray * grosterme / norm, ns,
                            -_ray / norm, temp, d1u1);

        d[3, 1] = 2.0 * (resul.Dot(vref));

        // Derivee par rapport a v1
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
        resul = Vector3D.LinearForm(-_ray / norm * (grosterme * ndotns - _nplan.Dot(temp)), _nplan, _ray * grosterme / norm, ns,
                            -_ray / norm, temp, d1v1);

        d[3, 2] = 2.0 * (resul.Dot(vref));

        d[3, 3] = -2.0 * (d1.Dot(vref));

        _pt2d = new Point2D(x[1], x[2]);
        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        Vector3D ns, ncrossns, resul = new Vector3D(), temp, vref = new Vector3D();

        double norm, ndotns, grosterme;

        _surf.D2(x[1], x[2], out _pts, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1,
                out Vector3D d2uv1);
        _curv.D1(x[3], out _ptc, out Vector3D d1);

        f[1] = _nplan.XYZ().Dot(_pts.XYZ()) + _d;
        f[2] = _nplan.XYZ().Dot(_ptc.XYZ()) + _d;

        d[1, 1] = _nplan.Dot(d1u1);
        d[1, 2] = _nplan.Dot(d1v1);
        d[1, 3] = 0.0;

        d[2, 1] = 0.0;
        d[2, 2] = 0.0;
        d[2, 3] = _nplan.Dot(d1);

        ns = d1u1.Crossed(d1v1);
        ncrossns = _nplan.Crossed(ns);
        norm = ncrossns.Magnitude;
        ndotns = _nplan.Dot(ns);

        vref = Vector3D.LinearForm(ndotns, _nplan, -1.0, ns);
        vref = vref.Divided(norm);
        vref = Vector3D.LinearForm(_ray, vref, new Vector3D(_ptc, _pts));

        f[3] = vref.SquareMagnitude - _ray * _ray;

        // Derivee par rapport a u1
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
        resul = Vector3D.LinearForm(-_ray / norm * (grosterme * ndotns - _nplan.Dot(temp)), _nplan, _ray * grosterme / norm, ns,
                            -_ray / norm, temp, d1u1);

        d[3, 1] = 2.0 * (resul.Dot(vref));

        // Derivee par rapport a v1
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
        resul = Vector3D.LinearForm(-_ray / norm * (grosterme * ndotns - _nplan.Dot(temp)), _nplan, _ray * grosterme / norm, ns,
                            -_ray / norm, temp, d1v1);

        d[3, 2] = 2.0 * (resul.Dot(vref));

        d[3, 3] = -2.0 * (d1.Dot(vref));

        _pt2d = new Point2D(x[1], x[2]);
        return true;
    }

    public override Point3D PointOnS()
    {
        return _pts;
    }

    public override Point3D PointOnC()
    {
        return _ptc;
    }

    public override Point2D Pnt2d()
    {
        return _pt2d;
    }

    public override double ParameterOnC()
    {
        return _prmc;
    }

    public override bool IsTangencyPoint
    {
        get
        {
            return _istangent;
        }
    }

    public override Vector3D TangentOnS()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_CSConstRad::TangentOnS");
        }

        return _tgs;
    }

    public override Vector3D TangentOnC()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_CSConstRad::TangentOnC");
        }

        return _tgc;
    }

    public override Vector2D Tangent2d()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_CSConstRad::Tangent2d");
        }

        return _tg2d;
    }

    public override void Tangent(double u, double v, out Vector3D tgS, out Vector3D nmS)
    {
        _surf.D1(u, v, out Point3D bid, out Vector3D d1u, out Vector3D d1v);
        Vector3D ns;
        nmS = ns = d1u.Crossed(d1v);

        double norm = _nplan.Crossed(ns).Magnitude;
        ns = Vector3D.LinearForm(_nplan.Dot(ns) / norm, _nplan, -1.0 / norm, ns);
        Point3D center = new Point3D(bid.XYZ() + _ray * ns.XYZ());
        tgS = _nplan.Crossed(new Vector3D(center, bid));
        if (_choix % 2 == 1)
        {
            tgS = tgS.Reversed();
        }
    }

    /// <summary>The circular section at <paramref name="param"/> through (U, V) and W, with its parameter range.</summary>
    public void Section(double param, double u, double v, double w, out double pdeb, out double pfin, ref Circle3D c)
    {
        Vector3D ns;
        double norm;
        Point3D center = new Point3D();

        _guide.D1(param, out _ptgui, out _d1gui);
        _nplan = _d1gui.Normalized();

        _surf.D1(u, v, out _pts, out Vector3D d1u1, out Vector3D d1v1);
        _ptc = _curv.Value(w);

        ns = d1u1.Crossed(d1v1);
        norm = _nplan.Crossed(ns).Magnitude;
        ns = Vector3D.LinearForm(_nplan.Dot(ns) / norm, _nplan, -1.0 / norm, ns);
        center = center.WithXYZ(_pts.XYZ() + _ray * ns.XYZ());
        c = c.WithRadius(Math.Abs(_ray));
        if (_choix % 2 == 0)
        {
            c = c.WithPosition(new Frame3D(center, new Direction3D(_nplan), new Direction3D(ns)));
        }
        else
        {
            c = c.WithPosition(new Frame3D(center, new Direction3D(_nplan.Reversed()), new Direction3D(ns)));
        }

        pdeb = 0.0;
        pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, _ptc);
    }

    /// <summary>Not implemented: throws <see cref="DomainErrorException"/>.</summary>
    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Vector3D> d2Poles, NCollection_Array1<Point2D> poles2d,
                                 NCollection_Array1<Vector2D> dPoles2d, NCollection_Array1<Vector2D> d2Poles2d,
                                 NCollection_Array1<double> weights, NCollection_Array1<double> dWeights,
                                 NCollection_Array1<double> d2Weights)
    {
        throw new DomainErrorException("BlendFunc_CSConstRad::Section : Not implemented");
    }

    /// <summary>
    /// The points and tangents of the section at <paramref name="param"/> through (U, V) and W, sampled at
    /// tabP.Length() points; false when the tangents cannot be computed.
    /// </summary>
    public bool GetSection(double param, double u, double v, double w, NCollection_Array1<Point3D> tabP,
                           NCollection_Array1<Vector3D> tabV)
    {
        int nbPoint = tabP.Length();
        if (nbPoint != tabV.Length() || nbPoint < 2)
        {
            throw new RangeErrorException();
        }

        int i, lowp = tabP.Lower(), lowv = tabV.Lower();

        Vector3D ns, dnplan = new Vector3D(), dnw = new Vector3D(), dn2w = new Vector3D(), ncrn = new Vector3D(),
               dncrn = new Vector3D(), ns2;
        Vector3D ncrossns, resul = new Vector3D();
        Vector3D resulu = new Vector3D(), resulv = new Vector3D(), temp = new Vector3D();

        double norm, ndotns, grosterme;
        double lambda, cosa, sina;
        double angle = 0.0, dangle = 0.0;
        math_Vector sol = new(1, 3), valsol = new(1, 3), secmember = new(1, 3);
        Matrix gradsol = new(1, 3, 1, 3);

        _guide.D2(param, out _ptgui, out _d1gui, out _d2gui);
        _normtg = _d1gui.Magnitude;
        _nplan = _d1gui.Normalized();
        dnplan = Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

        sol[1] = u;
        sol[2] = v;
        sol[3] = w;

        Values(sol, valsol, gradsol);

        _surf.D2(u, v, out _pts, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1, out Vector3D d2uv1);
        _curv.D1(w, out _ptc, out Vector3D d1);

        temp = temp.WithXYZ(_pts.XYZ() - _ptgui.XYZ());
        secmember[1] = _normtg - dnplan.Dot(temp);

        temp = temp.WithXYZ(_ptc.XYZ() - _ptgui.XYZ());
        secmember[2] = _normtg - dnplan.Dot(temp);

        ns = d1u1.Crossed(d1v1);
        ncrossns = _nplan.Crossed(ns);
        ndotns = _nplan.Dot(ns);
        norm = ncrossns.Magnitude;

        // Derivee de n1 par rapport a w

        grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
        dnw = Vector3D.LinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, _nplan, ndotns / norm, dnplan,
                          grosterme / norm, ns);

        temp = Vector3D.LinearForm(ndotns / norm, _nplan, -1.0 / norm, ns);
        resul = Vector3D.LinearForm(_ray, temp, new Vector3D(_ptc, _pts));
        secmember[3] = -2.0 * _ray * (dnw.Dot(resul)); // jag 950105 il manquait ray

        Gauss resol = new(gradsol);
        if (resol.IsDone)
        {
            resol.Solve(secmember);

            _tgs = Vector3D.LinearForm(secmember[1], d1u1, secmember[2], d1v1);
            _tgc = secmember[3] * d1;

            // Derivee de n1 par rapport a u1
            temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
            grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
            resulu = Vector3D.LinearForm(-(grosterme * ndotns - _nplan.Dot(temp)) / norm, _nplan, grosterme / norm, ns,
                                 -1.0 / norm, temp);

            // Derivee de n1 par rapport a v1
            temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
            grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
            resulv = Vector3D.LinearForm(-(grosterme * ndotns - _nplan.Dot(temp)) / norm, _nplan, grosterme / norm, ns,
                                 -1.0 / norm, temp);

            dnw = Vector3D.LinearForm(secmember[1], resulu, secmember[2], resulv, dnw);
            ns = Vector3D.LinearForm(ndotns / norm, _nplan, -1.0 / norm, ns);

            dn2w = Vector3D.LinearForm(_ray, dnw, -1.0, _tgc, _tgs);
            norm = resul.Magnitude;
            dn2w = dn2w.Divided(norm);
            ns2 = -resul.Normalized();
            dn2w = Vector3D.LinearForm(ns2.Dot(dn2w), ns2, -1.0, dn2w);

            if (_choix % 2 != 0)
            {
                _nplan = _nplan.Reversed();
                dnplan = dnplan.Reversed();
            }

            tabP[lowp] = _pts;
            tabP[lowp + nbPoint - 1] = _ptc;

            tabV[lowv] = _tgs;
            tabV[lowv + nbPoint - 1] = _tgc;

            if (nbPoint > 2)
            {
                cosa = ns.Dot(ns2);
                sina = _nplan.Dot(ns.Crossed(ns2));
                angle = Math.Acos(cosa);
                if (sina < 0.0)
                {
                    angle = 2.0 * M_PI - angle;
                }

                dangle = -(dnw.Dot(ns2) + ns.Dot(dn2w)) / sina;
                ncrn = _nplan.Crossed(ns);
                dncrn = dnplan.Crossed(ns).Added(_nplan.Crossed(dnw));
            }

            for (i = 2; i <= nbPoint - 1; i++)
            {
                lambda = (double)(i - 1) / (double)(nbPoint - 1);
                cosa = Math.Cos(lambda * angle);
                sina = Math.Sin(lambda * angle);
                tabP[lowp + i - 1] = tabP[lowp + i - 1].WithXYZ(_pts.XYZ() + Math.Abs(_ray) * ((cosa - 1) * ns.XYZ() + sina * ncrn.XYZ()));

                temp = Vector3D.LinearForm(-sina, ns, cosa, ncrn);
                temp = temp.Multiplied(lambda * dangle);
                temp = temp.Added(((cosa - 1) * dnw).Added(sina * dncrn));
                temp = temp.Multiplied(Math.Abs(_ray));
                temp = temp.Added(_tgs);
                tabV[lowv + i - 1] = temp;
            }

            return true;
        }

        return false;
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
        // On suppose que cela ne depend pas du Rayon!
    }

    public override int NbIntervals(Continuity s)
    {
        return _curv.NbIntervals(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        _curv.Intervals(t, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
    }

    public override void GetShape(out int nbPoles, out int nbKnots, out int degree, out int nbPoles2d)
    {
        nbPoles2d = 1;
        global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.GetShape(_sShape, _maxang, out nbPoles, out nbKnots, out degree, ref _tConv);
    }

    //=======================================================================
    // function : GetTolerance
    // purpose  : Determine les Tolerance a utiliser dans les approximations.
    //=======================================================================
    public override void GetTolerance(double boundTol, double surfTol, double angleTol, math_Vector tol3d,
                                      math_Vector tol1d)
    {
        int low = tol3d.Lower();
        int up = tol3d.Upper();
        double tol = global::Kerf.Engine.Construction.Surfaces.GeomFill.GetTolerance(_tConv, _minang, Math.Abs(_ray), angleTol, surfTol);
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
        Point3D center = new Point3D();
        double norm, u1, v1, w;

        double prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();

        _guide.D1(prm, out _ptgui, out _d1gui);
        _nplan = _d1gui.Normalized();

        p.ParametersOnS(out u1, out v1);
        w = p.ParameterOnC();

        _surf.D1(u1, v1, out _pts, out Vector3D d1u1, out Vector3D d1v1);
        _ptc = _curv.Value(w);

        poles2d[poles2d.Lower()] = poles2d[poles2d.Lower()].WithCoord(u1, v1);

        // Cas Linear
        if (_sShape == SectionShape.Linear)
        {
            poles[low] = _pts;
            poles[upp] = _ptc;
            weights[low] = 1.0;
            weights[upp] = 1.0;
            return;
        }

        ns = d1u1.Crossed(d1v1);
        norm = _nplan.Crossed(ns).Magnitude;

        ns = Vector3D.LinearForm(_nplan.Dot(ns) / norm, _nplan, -1.0 / norm, ns);

        center = center.WithXYZ(_pts.XYZ() + _ray * ns.XYZ());

        ns2 = new Vector3D(center, _ptc).Normalized();
        if (_choix % 2 != 0)
        {
            _nplan = _nplan.Reversed();
        }

        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns, ns2, _nplan, _pts, _ptc, Math.Abs(_ray), center, poles, weights);
    }

    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                                 NCollection_Array1<double> weights, NCollection_Array1<double> dWeights)
    {
        Vector3D ns, ns2, dnplan = new Vector3D(), dnw = new Vector3D(), dn2w = new Vector3D();
        Vector3D ncrossns;
        Vector3D resulu = new Vector3D(), resulv = new Vector3D(), temp = new Vector3D(), tgct = new Vector3D(),
               resul = new Vector3D();

        Point3D center = new Point3D();
        double norm, ndotns, grosterme;

        math_Vector sol = new(1, 3), valsol = new(1, 3), secmember = new(1, 3);
        Matrix gradsol = new(1, 3, 1, 3);

        double prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();
        bool istgt;

        _guide.D2(prm, out _ptgui, out _d1gui, out _d2gui);
        _normtg = _d1gui.Magnitude;
        _nplan = _d1gui.Normalized();
        dnplan = Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

        p.ParametersOnS(out sol[1], out sol[2]);
        sol[3] = p.ParameterOnC();

        Values(sol, valsol, gradsol);

        _surf.D2(sol[1], sol[2], out _pts, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1,
                out Vector3D d2uv1);
        _curv.D1(sol[3], out _ptc, out Vector3D d1);

        temp = temp.WithXYZ(_pts.XYZ() - _ptgui.XYZ());
        secmember[1] = _normtg - dnplan.Dot(temp);

        temp = temp.WithXYZ(_ptc.XYZ() - _ptgui.XYZ());
        secmember[2] = _normtg - dnplan.Dot(temp);

        ns = d1u1.Crossed(d1v1);
        ncrossns = _nplan.Crossed(ns);
        ndotns = _nplan.Dot(ns);
        norm = ncrossns.Magnitude;

        // Derivee de n1 par rapport a w

        grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
        dnw = Vector3D.LinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, _nplan, ndotns / norm, dnplan,
                          grosterme / norm, ns);

        temp = Vector3D.LinearForm(ndotns / norm, _nplan, -1.0 / norm, ns);
        resul = Vector3D.LinearForm(_ray, temp, new Vector3D(_ptc, _pts));
        secmember[3] = -2.0 * _ray * (dnw.Dot(resul)); // jag 950105 il manquait ray

        Gauss resol = new(gradsol);

        if (resol.IsDone)
        {
            resol.Solve(secmember);

            _tgs = Vector3D.LinearForm(secmember[1], d1u1, secmember[2], d1v1);
            _tgc = secmember[3] * d1;

            // Derivee de n1 par rapport a u1
            temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
            grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
            resulu = Vector3D.LinearForm(-(grosterme * ndotns - _nplan.Dot(temp)) / norm, _nplan, grosterme / norm, ns,
                                 -1.0 / norm, temp);

            // Derivee de n1 par rapport a v1
            temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
            grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
            resulv = Vector3D.LinearForm(-(grosterme * ndotns - _nplan.Dot(temp)) / norm, _nplan, grosterme / norm, ns,
                                 -1.0 / norm, temp);

            dnw = Vector3D.LinearForm(secmember[1], resulu, secmember[2], resulv, dnw);
            ns = Vector3D.LinearForm(ndotns / norm, _nplan, -1.0 / norm, ns);

            dn2w = Vector3D.LinearForm(_ray, dnw, -1.0, _tgc, _tgs);
            norm = resul.Magnitude;
            dn2w = dn2w.Divided(norm);
            ns2 = -resul.Normalized();
            dn2w = Vector3D.LinearForm(ns2.Dot(dn2w), ns2, -1.0, dn2w);

            istgt = false;
        }
        else
        {
            ns = Vector3D.LinearForm(ndotns / norm, _nplan, -1.0 / norm, ns);
            ns2 = -resul.Normalized();
            istgt = true;
        }

        // Les poles 2d

        poles2d[poles2d.Lower()] = poles2d[poles2d.Lower()].WithCoord(sol[1], sol[2]);
        if (!istgt)
        {
            dPoles2d[poles2d.Lower()] = dPoles2d[poles2d.Lower()].WithCoord(secmember[1], secmember[2]);
        }

        // Cas Linear
        if (_sShape == SectionShape.Linear)
        {
            poles[low] = _pts;
            poles[upp] = _ptc;
            weights[low] = 1.0;
            weights[upp] = 1.0;
            if (!istgt)
            {
                dPoles[low] = _tgs;
                dPoles[upp] = _tgc;
                dWeights[low] = 0.0;
                dWeights[upp] = 0.0;
            }

            return (!istgt);
        }

        // Cas du cercle
        center = center.WithXYZ(_pts.XYZ() + _ray * ns.XYZ());
        if (!istgt)
        {
            tgct = _tgs.Added(_ray * dnw);
        }

        if (_choix % 2 != 0)
        {
            _nplan = _nplan.Reversed();
            dnplan = dnplan.Reversed();
        }

        if (!istgt)
        {
            return global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns, ns2, dnw, dn2w, _nplan, dnplan, _pts, _ptc, _tgs, _tgc, Math.Abs(_ray), 0,
                                      center, tgct, poles, dPoles, weights, dWeights);
        }
        else
        {
            global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns, ns2, _nplan, _pts, _ptc, Math.Abs(_ray), center, poles, weights);
            return false;
        }
    }

    public override void Resolution(int ic2d, double tol, out double tolU, out double tolV)
    {
        tolU = _surf.UResolution(tol);
        tolV = _surf.VResolution(tol);
    }
}
