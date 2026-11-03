// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstEvolRad.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstEvolRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
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
/// The function of an evolving-radius fillet (the radius a law of the guide's parameter) between a surface and a
/// restriction (an arc of another surface) along a guide: the variables are (U, V) on the surface and the parameter
/// on the restriction.
/// </summary>
/// <remarks>A working object (decision 21): it caches its last solution, which the walkers read.</remarks>
public class SurfRstEvolRad : SurfRstFunction
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
        ns.SetLinearForm(_nplan.Dot(ns) / norm, _nplan, -1.0 / norm, ns);
        vref.SetLinearForm(_ray, ns, new Vector3D(_ptrst, _pts));
        f[3] = vref.SquareMagnitude() - _ray * _ray;
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

        vref.SetLinearForm(ndotns, _nplan, -1.0, ns);
        vref.Divide(norm);
        vref.SetLinearForm(_ray, vref, new Vector3D(_ptrst, _pts));

        // Derivative by u1
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-_ray / norm * (grosterme * ndotns - _nplan.Dot(temp)), _nplan, _ray * grosterme / norm, ns,
                            -_ray / norm, temp, d1u1);

        d[3, 1] = 2.0 * (resul.Dot(vref));

        // Derivative by v1
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-_ray / norm * (grosterme * ndotns - _nplan.Dot(temp)), _nplan, _ray * grosterme / norm, ns,
                            -_ray / norm, temp, d1v1);

        d[3, 2] = 2.0 * (resul.Dot(vref));

        d[3, 3] = -2.0 * (d1.Dot(vref));

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

        vref.SetLinearForm(ndotns, _nplan, -1.0, ns);
        vref.Divide(norm);
        vref.SetLinearForm(_ray, vref, new Vector3D(_ptrst, _pts));

        f[3] = vref.SquareMagnitude() - _ray * _ray;

        // Derivative by u1
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-_ray / norm * (grosterme * ndotns - _nplan.Dot(temp)), _nplan, _ray * grosterme / norm, ns,
                            -_ray / norm, temp, d1u1);

        d[3, 1] = 2.0 * (resul.Dot(vref));

        // Derivative by v1
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
        resul.SetLinearForm(-_ray / norm * (grosterme * ndotns - _nplan.Dot(temp)), _nplan, _ray * grosterme / norm, ns,
                            -_ray / norm, temp, d1v1);

        d[3, 2] = 2.0 * (resul.Dot(vref));

        d[3, 3] = -2.0 * (d1.Dot(vref));

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
        _nplan.SetXYZ(_d1gui.Normalized().XYZ());
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
            dnplan.SetLinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

            temp.SetXYZ(_pts.XYZ() - _ptgui.XYZ());
            secmember[1] = _normtg - dnplan.Dot(temp);

            temp.SetXYZ(_ptrst.XYZ() - _ptgui.XYZ());
            secmember[2] = _normtg - dnplan.Dot(temp);

            ns = d1u1.Crossed(d1v1);
            ncrossns = _nplan.Crossed(ns);
            ndotns = _nplan.Dot(ns);
            norm = ncrossns.Magnitude();

            grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
            Vector3D dnw = new();
            dnw.SetLinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, _nplan, ndotns / norm, dnplan,
                              grosterme / norm, ns);

            ns.SetLinearForm(ndotns / norm, _nplan, -1.0 / norm, ns);
            resul.SetLinearForm(_ray, ns, new Vector3D(_ptrst, _pts));

            secmember[3] = -2.0 * _ray * (dnw.Dot(resul)) - 2.0 * _dray * (ns.Dot(resul)) + 2.0 * _ray * _dray;

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
                _tgs.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
                _tgrst = secmember[3] * d1;
                _tg2ds.SetCoord(secmember[1], secmember[2]);
                _surfrst.D1(_pt2drst.X(), _pt2drst.Y(), out bid, out d1urst, out d1vrst);
                double a, b;
                T3dto2d(out a, out b, _tgrst, d1urst, d1vrst);
                _tg2drst.SetCoord(a, b);
                _istangent = false;
            }
            else
            {
                _istangent = true;
            }

            // update of maxang
            if (_ray > 0.0)
            {
                ns.Reverse();
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
        nsInPlane.SetLinearForm(_nplan.Dot(ns) * unsurnorm, _nplan, -unsurnorm, ns);

        center.SetXYZ(bid.XYZ() + _ray * nsInPlane.XYZ());
        if (_choix > 2)
        {
            nsInPlane.Reverse();
        }
        tgS = _nplan.Crossed(new Vector3D(center, bid));
        if (_choix % 2 == 1)
        {
            tgS.Reverse();
        }
        double u, v;
        _rstref!.Value(sol[3]).Coord(out u, out v);
        _surfref!.D1(u, v, out bid, out d1u, out d1v);
        nRst = d1u.Crossed(d1v);
        norm = _nplan.Crossed(nRst).Magnitude();
        unsurnorm = 1.0 / norm;
        nRstInPlane = new Vector3D();
        nRstInPlane.SetLinearForm(_nplan.Dot(nRst) * unsurnorm, _nplan, -unsurnorm, nRst);
        Vector3D centptrst = new(center, bid);
        if (centptrst.Dot(nRstInPlane) < 0.0)
        {
            nRstInPlane.Reverse();
        }
        tgRst = _nplan.Crossed(centptrst);
        if (_choix % 2 == 1)
        {
            tgRst.Reverse();
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
        double norm;
        Point3D center = new();

        _tguide.D1(param, out _ptgui, out _d1gui);
        np = _d1gui.Normalized();
        _ray = _sg1 * _tevol.Value(param);

        _surf.D1(u, v, out _pts, out d1u1, out d1v1);
        _ptrst = _cons.Value(w);

        ns = d1u1.Crossed(d1v1);

        norm = _nplan.Crossed(ns).Magnitude();
        ns.SetLinearForm(_nplan.Dot(ns) / norm, _nplan, -1.0 / norm, ns);
        center.SetXYZ(_pts.XYZ() + _ray * ns.XYZ());
        c.SetRadius(Math.Abs(_ray));

        if (_ray > 0)
        {
            ns.Reverse();
        }

        if (_choix % 2 != 0)
        {
            np.Reverse();
        }

        c.SetPosition(new Frame3D(center, new Direction3D(np), new Direction3D(ns)));
        pdeb = 0; // ElCLib::Parameter(C,pts);
        pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, _ptrst);

        // Test negative and almost null angles : Special case
        if (pfin > 1.5 * M_PI)
        {
            np.Reverse();
            c.SetPosition(new Frame3D(center, new Direction3D(np), new Direction3D(ns)));
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
        int Nb_Int_Courbe, Nb_Int_Loi;
        Nb_Int_Courbe = _guide.NbIntervals(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
        Nb_Int_Loi = _fevol.NbIntervals(s);

        if (Nb_Int_Loi == 1)
        {
            return Nb_Int_Courbe;
        }

        NCollection_Array1<double> intC = new(1, Nb_Int_Courbe + 1);
        NCollection_Array1<double> intL = new(1, Nb_Int_Loi + 1);
        NCollection_Sequence<double> inter = new();
        _guide.Intervals(intC, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
        _fevol.Intervals(intL, s);

        FusionneIntervalles(intC, intL, inter);
        return inter.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        int Nb_Int_Courbe, Nb_Int_Loi;
        Nb_Int_Courbe = _guide.NbIntervals(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
        Nb_Int_Loi = _fevol.NbIntervals(s);

        if (Nb_Int_Loi == 1)
        {
            _guide.Intervals(t, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
        }
        else
        {
            NCollection_Array1<double> intC = new(1, Nb_Int_Courbe + 1);
            NCollection_Array1<double> intL = new(1, Nb_Int_Loi + 1);
            NCollection_Sequence<double> inter = new();
            _guide.Intervals(intC, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
            _fevol.Intervals(intL, s);

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

        double norm, u1, v1, w;

        double prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();

        _tguide.D1(prm, out _ptgui, out _d1gui);
        _ray = _tevol.Value(prm);
        _ray = _sg1 * _ray;
        _nplan = _d1gui.Normalized();

        p.ParametersOnS(out u1, out v1);
        w = p.ParameterOnC(); // jlr : point on curve not on surface
        Point2D pt2d = _rst.Value(w);

        _surf.D1(u1, v1, out _pts, out d1u1, out d1v1);
        _ptrst = _cons.Value(w);
        _distmin = Min(_distmin, _pts.Distance(_ptrst));

        poles2d[poles2d.Lower()].SetCoord(u1, v1);
        poles2d[poles2d.Upper()].SetCoord(pt2d.X(), pt2d.Y());

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

        ns.SetLinearForm(_nplan.Dot(ns) / norm, _nplan, -1.0 / norm, ns);

        center.SetXYZ(_pts.XYZ() + _ray * ns.XYZ());

        ns2 = new Vector3D(center, _ptrst).Normalized();
        if (_ray > 0)
        {
            ns.Reverse();
        }
        if (_choix % 2 != 0)
        {
            _nplan.Reverse();
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

        double norm, ndotns, grosterme, dray;

        math_Vector sol = new(1, 3), valsol = new(1, 3), secmember = new(1, 3);
        Matrix gradsol = new(1, 3, 1, 3);

        double prm = p.Parameter(), rayprim;
        int low = poles.Lower();
        int upp = poles.Upper();
        bool istgt;

        _tguide.D2(prm, out _ptgui, out _d1gui, out _d2gui);
        _tevol.D1(prm, out _ray, out dray);
        _ray = _sg1 * _ray;
        dray = _sg1 * dray;
        _normtg = _d1gui.Magnitude();
        _nplan = _d1gui.Normalized();
        dnplan.SetLinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

        p.ParametersOnS(out sol[1], out sol[2]);
        sol[3] = _prmrst = p.ParameterOnC();
        _pt2drst = _rst.Value(_prmrst);

        Values(sol, valsol, gradsol);

        _surf.D2(sol[1], sol[2], out _pts, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
        _cons.D1(sol[3], out _ptrst, out d1);

        temp.SetXYZ(_pts.XYZ() - _ptgui.XYZ());
        secmember[1] = _normtg - dnplan.Dot(temp);

        temp.SetXYZ(_ptrst.XYZ() - _ptgui.XYZ());
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
        dnw.SetLinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, _nplan, ndotns / norm, dnplan,
                          grosterme / norm, ns);

        temp.SetLinearForm(ndotns / norm, _nplan, -1.0 / norm, ns);
        resul.SetLinearForm(_ray, temp, new Vector3D(_ptrst, _pts));
        // secmember(3) = -2.*ray*(dnw.Dot(resul)); // jag 950105 il manquait ray
        secmember[3] = -2.0 * _ray * (dnw.Dot(resul)) - 2.0 * dray * (temp.Dot(resul)) + 2.0 * _ray * dray;
        Gauss resol = new(gradsol);

        if (resol.IsDone)
        {
            resol.Solve(secmember);
            istgt = false;
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
            _tgs.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
            _tgrst = secmember[3] * d1;

            // Derivative of n1 corresponding to u1
            temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
            grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
            resulu.SetLinearForm(-(grosterme * ndotns - _nplan.Dot(temp)) / norm, _nplan, grosterme / norm, ns,
                                 -1.0 / norm, temp);

            // Derivative of n1 corresponding to v1
            temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
            grosterme = ncrossns.Dot(_nplan.Crossed(temp)) / norm / norm;
            resulv.SetLinearForm(-(grosterme * ndotns - _nplan.Dot(temp)) / norm, _nplan, grosterme / norm, ns,
                                 -1.0 / norm, temp);

            dnw.SetLinearForm(secmember[1], resulu, secmember[2], resulv, dnw);
            ns.SetLinearForm(ndotns / norm, _nplan, -1.0 / norm, ns);

            dn2w.SetLinearForm(_ray, dnw, -1.0, _tgrst, _tgs);
            dn2w.SetLinearForm(dray, ns, dn2w);
            norm = resul.Magnitude();
            dn2w.Divide(norm);
            ns2 = -resul.Normalized();
            dn2w.SetLinearForm(ns2.Dot(dn2w), ns2, -1.0, dn2w);

            istgt = false;
        }
        else
        {
            ns.SetLinearForm(ndotns / norm, _nplan, -1.0 / norm, ns);
            ns2 = -resul.Normalized();
            istgt = true;
        }

        // Tops 2D

        poles2d[poles2d.Lower()].SetCoord(sol[1], sol[2]);
        poles2d[poles2d.Upper()].SetCoord(_pt2drst.X(), _pt2drst.Y());
        if (!istgt)
        {
            dPoles2d[poles2d.Lower()].SetCoord(secmember[1], secmember[2]);
            _surfrst.D1(_pt2drst.X(), _pt2drst.Y(), out bid, out d1urst, out d1vrst);
            double a, b;
            T3dto2d(out a, out b, _tgrst, d1urst, d1vrst);
            dPoles2d[poles2d.Upper()].SetCoord(a, b);
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
        center.SetXYZ(_pts.XYZ() + _ray * ns.XYZ());
        if (!istgt)
        {
            tgct.SetLinearForm(_ray, dnw, dray, ns, _tgs);
        }

        if (_ray > 0.0)
        {
            ns.Reverse();
            if (!istgt)
            {
                dnw.Reverse();
            }
        }
        if (_choix % 2 != 0)
        {
            _nplan.Reverse();
            dnplan.Reverse();
        }
        if (!istgt)
        {
            if (_ray < 0.0)
            { // to avoid std::abs(dray) some lines below
                rayprim = -dray;
            }
            else
            {
                rayprim = dray;
            }

            return global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns, ns2, dnw, dn2w, _nplan, dnplan, _pts, _ptrst, _tgs, _tgrst,
                                      Math.Abs(_ray), rayprim, center, tgct, poles, dPoles, weights, dWeights);
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
