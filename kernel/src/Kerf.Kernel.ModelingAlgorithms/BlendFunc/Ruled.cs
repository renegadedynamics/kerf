// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Ruled.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Ruled.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>
/// The function of a ruled blend between two surfaces along a guide curve: the segment between two points in the
/// plane normal to the guide, normal to both surfaces' sections.
/// </summary>
internal class Ruled : BlendFunction
{
    private readonly SurfaceAdaptorBase _surf1;
    private readonly SurfaceAdaptorBase _surf2;
    private readonly CurveAdaptorBase _curv;
    private Point3D _pts1;
    private Point3D _pts2;
    private bool _istangent;
    private Vector3D _tg1;
    private Vector2D _tg12d;
    private Vector3D _tg2;
    private Vector2D _tg22d;
    private Point3D _ptgui;
    private Vector3D _d1gui;
    private Vector3D _d2gui;
    private Vector3D _nplan;
    private double _normtg;
    private double _d;
    private readonly double _distmin;

    public Ruled(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, CurveAdaptorBase c)
    {
        _surf1 = s1;
        _surf2 = s2;
        _curv = c;
        _istangent = true;
        _normtg = 0.0;
        _d = 0.0;
        _distmin = RealLast();
    }

    /// <summary>Returns 4.</summary>
    public override int NbEquations()
    {
        return 4;
    }

    public override void Set(double param)
    {
        _curv.D2(param, out _ptgui, out _d1gui, out _d2gui);
        _normtg = _d1gui.Magnitude;
        _nplan = _d1gui.Normalized();
        _d = -(_nplan.XYZ().Dot(_ptgui.XYZ()));
        _istangent = true;
    }

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override void Set(double first, double last)
    {
        throw new MethodNotImplementedException("BlendFunc_Ruled::Set");
    }

    public override void GetTolerance(math_Vector tolerance, double tol)
    {
        tolerance[1] = _surf1.UResolution(tol);
        tolerance[2] = _surf1.VResolution(tol);
        tolerance[3] = _surf2.UResolution(tol);
        tolerance[4] = _surf2.VResolution(tol);
    }

    public override void GetBounds(math_Vector infBound, math_Vector supBound)
    {
        infBound[1] = _surf1.FirstUParameter();
        infBound[2] = _surf1.FirstVParameter();
        infBound[3] = _surf2.FirstUParameter();
        infBound[4] = _surf2.FirstVParameter();
        supBound[1] = _surf1.LastUParameter();
        supBound[2] = _surf1.LastVParameter();
        supBound[3] = _surf2.LastUParameter();
        supBound[4] = _surf2.LastVParameter();

        for (int i = 1; i <= 4; i++)
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(infBound[i]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(supBound[i]))
            {
                double range = (supBound[i] - infBound[i]);
                infBound[i] -= range;
                supBound[i] += range;
            }
        }
    }

    public override bool IsSolution(math_Vector sol, double tol)
    {
        math_Vector valsol = new(1, 4), secmember = new(1, 4);
        Matrix gradsol = new(1, 4, 1, 4);

        Vector3D dnplan = new Vector3D(), temp = new Vector3D(), ns, ncrossns;
        double norm, ndotns, grosterme;

        Values(sol, valsol, gradsol);
        if (Math.Abs(valsol[1]) <= tol && Math.Abs(valsol[2]) <= tol && Math.Abs(valsol[3]) <= tol
            && Math.Abs(valsol[4]) <= tol)
        {
            // Calcul des tangentes

            _surf1.D1(sol[1], sol[2], out _pts1, out Vector3D d1u1, out Vector3D d1v1);
            _surf2.D1(sol[3], sol[4], out _pts2, out Vector3D d1u2, out Vector3D d1v2);

            dnplan = Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

            secmember[1] = _normtg - dnplan.Dot(new Vector3D(_ptgui, _pts1));
            secmember[2] = _normtg - dnplan.Dot(new Vector3D(_ptgui, _pts2));

            ns = d1u1.Crossed(d1v1);
            ncrossns = _nplan.Crossed(ns);
            ndotns = _nplan.Dot(ns);
            norm = ncrossns.Magnitude;

            // Derivee de nor1 par rapport au parametre sur la ligne guide
            grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
            temp = Vector3D.LinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, _nplan, ndotns / norm, dnplan,
                               grosterme / norm, ns);

            secmember[3] = -(temp.Dot(new Vector3D(_pts1, _pts2)));

            ns = d1u2.Crossed(d1v2);
            ncrossns = _nplan.Crossed(ns);
            ndotns = _nplan.Dot(ns);
            norm = ncrossns.Magnitude;

            // Derivee de nor2 par rapport au parametre sur la ligne guide
            grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
            temp = Vector3D.LinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, _nplan, ndotns / norm, dnplan,
                               grosterme / norm, ns);

            secmember[4] = -(temp.Dot(new Vector3D(_pts1, _pts2)));

            Gauss resol = new(gradsol);
            if (resol.IsDone)
            {
                resol.Solve(secmember);

                _tg1 = Vector3D.LinearForm(secmember[1], d1u1, secmember[2], d1v1);
                _tg2 = Vector3D.LinearForm(secmember[3], d1u2, secmember[4], d1v2);
                _tg12d = _tg12d.WithCoord(secmember[1], secmember[2]);
                _tg22d = _tg22d.WithCoord(secmember[3], secmember[4]);
                _istangent = false;
            }
            else
            {
                _istangent = true;
            }

            return true;
        }

        _istangent = true;
        return false;
    }

    public override double GetMinimalDistance()
    {
        return _distmin;
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        _surf1.D1(x[1], x[2], out _pts1, out Vector3D d1u1, out Vector3D d1v1);
        _surf2.D1(x[3], x[4], out _pts2, out Vector3D d1u2, out Vector3D d1v2);

        Vector3D temp = new Vector3D(_pts1, _pts2);

        Vector3D ns1 = d1u1.Crossed(d1v1);
        Vector3D ns2 = d1u2.Crossed(d1v2);

        double norm1 = _nplan.Crossed(ns1).Magnitude;
        double norm2 = _nplan.Crossed(ns2).Magnitude;
        ns1 = Vector3D.LinearForm(_nplan.Dot(ns1) / norm1, _nplan, -1.0 / norm1, ns1);
        ns2 = Vector3D.LinearForm(_nplan.Dot(ns2) / norm2, _nplan, -1.0 / norm2, ns2);

        f[1] = (_nplan.XYZ().Dot(_pts1.XYZ())) + _d;
        f[2] = (_nplan.XYZ().Dot(_pts2.XYZ())) + _d;

        f[3] = temp.Dot(ns1);
        f[4] = temp.Dot(ns2);
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        Vector3D nor1 = new Vector3D(), nor2 = new Vector3D(), p1p2;
        Vector3D ns1, ns2, ncrossns1, ncrossns2, resul = new Vector3D(), temp;

        double norm1, norm2, ndotns1, ndotns2, grosterme;

        _surf1.D2(x[1], x[2], out _pts1, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1,
                 out Vector3D d2uv1);
        _surf2.D2(x[3], x[4], out _pts2, out Vector3D d1u2, out Vector3D d1v2, out Vector3D d2u2, out Vector3D d2v2,
                 out Vector3D d2uv2);

        d[1, 1] = _nplan.Dot(d1u1);
        d[1, 2] = _nplan.Dot(d1v1);
        d[1, 3] = 0.0;
        d[1, 4] = 0.0;
        d[2, 1] = 0.0;
        d[2, 2] = 0.0;
        d[2, 3] = _nplan.Dot(d1u2);
        d[2, 4] = _nplan.Dot(d1v2);

        ns1 = d1u1.Crossed(d1v1);
        ns2 = d1u2.Crossed(d1v2);
        ncrossns1 = _nplan.Crossed(ns1);
        ncrossns2 = _nplan.Crossed(ns2);
        norm1 = ncrossns1.Magnitude;
        norm2 = ncrossns2.Magnitude;

        ndotns1 = _nplan.Dot(ns1);
        ndotns2 = _nplan.Dot(ns2);

        nor1 = Vector3D.LinearForm(ndotns1 / norm1, _nplan, -1.0 / norm1, ns1);
        nor2 = Vector3D.LinearForm(ndotns2 / norm2, _nplan, -1.0 / norm2, ns2);

        p1p2 = new Vector3D(_pts1, _pts2);

        // Derivee de nor1 par rapport a u1

        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns1.Dot(_nplan.Crossed(temp)) / norm1 / norm1;
        resul = Vector3D.LinearForm(-(grosterme * ndotns1 - _nplan.Dot(temp)) / norm1, _nplan, grosterme / norm1, ns1,
                            -1.0 / norm1, temp);

        d[3, 1] = -(d1u1.Dot(nor1)) + p1p2.Dot(resul);

        // Derivee par rapport a v1

        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns1.Dot(_nplan.Crossed(temp)) / norm1 / norm1;
        resul = Vector3D.LinearForm(-(grosterme * ndotns1 - _nplan.Dot(temp)) / norm1, _nplan, grosterme / norm1, ns1,
                            -1.0 / norm1, temp);

        d[3, 2] = -(d1v1.Dot(nor1)) + p1p2.Dot(resul);

        d[3, 3] = d1u2.Dot(nor1);
        d[3, 4] = d1v2.Dot(nor1);

        d[4, 1] = -(d1u1.Dot(nor2));
        d[4, 2] = -(d1v1.Dot(nor2));

        // Derivee de nor2 par rapport a u2
        temp = d2u2.Crossed(d1v2).Added(d1u2.Crossed(d2uv2));
        grosterme = ncrossns2.Dot(_nplan.Crossed(temp)) / norm2 / norm2;
        resul = Vector3D.LinearForm(-(grosterme * ndotns2 - _nplan.Dot(temp)) / norm2, _nplan, grosterme / norm2, ns2,
                            -1.0 / norm2, temp);

        d[4, 3] = d1u2.Dot(nor2) + p1p2.Dot(resul);

        // Derivee par rapport a v2
        temp = d2uv2.Crossed(d1v2).Added(d1u2.Crossed(d2v2));
        grosterme = ncrossns2.Dot(_nplan.Crossed(temp)) / norm2 / norm2;
        resul = Vector3D.LinearForm(-(grosterme * ndotns2 - _nplan.Dot(temp)) / norm2, _nplan, grosterme / norm2, ns2,
                            -1.0 / norm2, temp);

        d[4, 4] = d1v2.Dot(nor2) + p1p2.Dot(resul);

        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        Vector3D nor1 = new Vector3D(), nor2 = new Vector3D(), p1p2;
        Vector3D ns1, ns2, ncrossns1, ncrossns2, resul = new Vector3D(), temp;

        double norm1, norm2, ndotns1, ndotns2, grosterme;

        _surf1.D2(x[1], x[2], out _pts1, out Vector3D d1u1, out Vector3D d1v1, out Vector3D d2u1, out Vector3D d2v1,
                 out Vector3D d2uv1);
        _surf2.D2(x[3], x[4], out _pts2, out Vector3D d1u2, out Vector3D d1v2, out Vector3D d2u2, out Vector3D d2v2,
                 out Vector3D d2uv2);

        p1p2 = new Vector3D(_pts1, _pts2);

        ns1 = d1u1.Crossed(d1v1);
        ns2 = d1u2.Crossed(d1v2);
        ncrossns1 = _nplan.Crossed(ns1);
        ncrossns2 = _nplan.Crossed(ns2);
        norm1 = ncrossns1.Magnitude;
        norm2 = ncrossns2.Magnitude;

        ndotns1 = _nplan.Dot(ns1);
        ndotns2 = _nplan.Dot(ns2);

        nor1 = Vector3D.LinearForm(ndotns1 / norm1, _nplan, -1.0 / norm1, ns1);
        nor2 = Vector3D.LinearForm(ndotns2 / norm2, _nplan, -1.0 / norm2, ns2);

        f[1] = (_nplan.XYZ().Dot(_pts1.XYZ())) + _d;
        f[2] = (_nplan.XYZ().Dot(_pts2.XYZ())) + _d;

        f[3] = p1p2.Dot(nor1);
        f[4] = p1p2.Dot(nor2);

        d[1, 1] = _nplan.Dot(d1u1);
        d[1, 2] = _nplan.Dot(d1v1);
        d[1, 3] = 0.0;
        d[1, 4] = 0.0;
        d[2, 1] = 0.0;
        d[2, 2] = 0.0;
        d[2, 3] = _nplan.Dot(d1u2);
        d[2, 4] = _nplan.Dot(d1v2);

        // Derivee de nor1 par rapport a u1

        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns1.Dot(_nplan.Crossed(temp)) / norm1 / norm1;
        resul = Vector3D.LinearForm(-(grosterme * ndotns1 - _nplan.Dot(temp)) / norm1, _nplan, grosterme / norm1, ns1,
                            -1.0 / norm1, temp);

        d[3, 1] = -(d1u1.Dot(nor1)) + p1p2.Dot(resul);

        // Derivee par rapport a v1

        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns1.Dot(_nplan.Crossed(temp)) / norm1 / norm1;
        resul = Vector3D.LinearForm(-(grosterme * ndotns1 - _nplan.Dot(temp)) / norm1, _nplan, grosterme / norm1, ns1,
                            -1.0 / norm1, temp);

        d[3, 2] = -(d1v1.Dot(nor1)) + p1p2.Dot(resul);

        d[3, 3] = d1u2.Dot(nor1);
        d[3, 4] = d1v2.Dot(nor1);

        d[4, 1] = -(d1u1.Dot(nor2));
        d[4, 2] = -(d1v1.Dot(nor2));

        // Derivee de nor2 par rapport a u2
        temp = d2u2.Crossed(d1v2).Added(d1u2.Crossed(d2uv2));
        grosterme = ncrossns2.Dot(_nplan.Crossed(temp)) / norm2 / norm2;
        resul = Vector3D.LinearForm(-(grosterme * ndotns2 - _nplan.Dot(temp)) / norm2, _nplan, grosterme / norm2, ns2,
                            -1.0 / norm2, temp);

        d[4, 3] = d1u2.Dot(nor2) + p1p2.Dot(resul);

        // Derivee par rapport a v2
        temp = d2uv2.Crossed(d1v2).Added(d1u2.Crossed(d2v2));
        grosterme = ncrossns2.Dot(_nplan.Crossed(temp)) / norm2 / norm2;
        resul = Vector3D.LinearForm(-(grosterme * ndotns2 - _nplan.Dot(temp)) / norm2, _nplan, grosterme / norm2, ns2,
                            -1.0 / norm2, temp);

        d[4, 4] = d1v2.Dot(nor2) + p1p2.Dot(resul);

        return true;
    }

    public override void Tangent(double u1, double v1, double u2, double v2, out Vector3D tgF, out Vector3D tgL,
                                 out Vector3D nmF, out Vector3D nmL)
    {
        _surf2.D1(u2, v2, out _, out Vector3D d1u, out Vector3D d1v);
        nmL = d1u.Crossed(d1v);

        _surf1.D1(u1, v1, out _, out d1u, out d1v);
        nmF = d1u.Crossed(d1v);

        tgF = tgL = new Vector3D(_pts1, _pts2);
    }

    public override Point3D PointOnS1()
    {
        return _pts1;
    }

    public override Point3D PointOnS2()
    {
        return _pts2;
    }

    public override bool IsTangencyPoint
    {
        get
        {
            return _istangent;
        }
    }

    public override Vector3D TangentOnS1()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_Ruled::TangentOnS1");
        }

        return _tg1;
    }

    public override Vector3D TangentOnS2()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_Ruled::TangentOnS2");
        }

        return _tg2;
    }

    public override Vector2D Tangent2dOnS1()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_Ruled::Tangent2dOnS1");
        }

        return _tg12d;
    }

    public override Vector2D Tangent2dOnS2()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_Ruled::Tangent2dOnS2");
        }

        return _tg22d;
    }

    /// <summary>
    /// The points and tangents of the section at <paramref name="param"/> through (U1, V1) and (U2, V2), sampled at
    /// tabP.Length() points; false when the tangents cannot be computed.
    /// </summary>
    public bool GetSection(double param, double u1, double v1, double u2, double v2, NCollection_Array1<Point3D> tabP,
                           NCollection_Array1<Vector3D> tabV)
    {
        int nbPoint = tabP.Length();
        if (nbPoint != tabV.Length() || nbPoint < 2)
        {
            throw new RangeErrorException();
        }

        int i, lowp = tabP.Lower(), lowv = tabV.Lower();

        Vector3D dnplan = new Vector3D(), temp = new Vector3D(), ns, ncrossns;
        double norm, ndotns, grosterme, lambda;

        math_Vector sol = new(1, 4), valsol = new(1, 4), secmember = new(1, 4);
        Matrix gradsol = new(1, 4, 1, 4);

        _curv.D2(param, out _ptgui, out _d1gui, out _d2gui);
        _normtg = _d1gui.Magnitude;
        _nplan = _d1gui.Normalized();
        _d = -(_nplan.XYZ().Dot(_ptgui.XYZ()));

        sol[1] = u1;
        sol[2] = v1;
        sol[3] = u2;
        sol[4] = v2;

        Values(sol, valsol, gradsol);

        _surf1.D1(sol[1], sol[2], out _pts1, out Vector3D d1u1, out Vector3D d1v1);
        _surf2.D1(sol[3], sol[4], out _pts2, out Vector3D d1u2, out Vector3D d1v2);

        dnplan = Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

        secmember[1] = _normtg - dnplan.Dot(new Vector3D(_ptgui, _pts1));
        secmember[2] = _normtg - dnplan.Dot(new Vector3D(_ptgui, _pts2));

        ns = d1u1.Crossed(d1v1);
        ncrossns = _nplan.Crossed(ns);
        ndotns = _nplan.Dot(ns);
        norm = ncrossns.Magnitude;

        // Derivee de nor1 par rapport au parametre sur la ligne guide
        grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
        temp = Vector3D.LinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, _nplan, ndotns / norm, dnplan,
                           grosterme / norm, ns);

        secmember[3] = -(temp.Dot(new Vector3D(_pts1, _pts2)));

        ns = d1u2.Crossed(d1v2);
        ncrossns = _nplan.Crossed(ns);
        ndotns = _nplan.Dot(ns);
        norm = ncrossns.Magnitude;

        // Derivee de nor2 par rapport au parametre sur la ligne guide
        grosterme = ncrossns.Dot(dnplan.Crossed(ns)) / norm / norm;
        temp = Vector3D.LinearForm((dnplan.Dot(ns) - grosterme * ndotns) / norm, _nplan, ndotns / norm, dnplan,
                           grosterme / norm, ns);

        secmember[4] = -(temp.Dot(new Vector3D(_pts1, _pts2)));

        Gauss resol = new(gradsol);
        if (resol.IsDone)
        {
            resol.Solve(secmember);

            _tg1 = Vector3D.LinearForm(secmember[1], d1u1, secmember[2], d1v1);
            _tg2 = Vector3D.LinearForm(secmember[3], d1u2, secmember[4], d1v2);

            tabP[lowp] = _pts1;
            tabP[lowp + nbPoint - 1] = _pts2;

            tabV[lowv] = _tg1;
            tabV[lowv + nbPoint - 1] = _tg2;

            for (i = 2; i <= nbPoint - 1; i++)
            {
                lambda = (double)(i - 1) / (double)(nbPoint - 1);
                tabP[lowp + i - 1] = tabP[lowp + i - 1].WithXYZ((1.0 - lambda) * _pts1.XYZ() + lambda * _pts2.XYZ());
                tabV[lowv + i - 1] = Vector3D.LinearForm(1.0 - lambda, _tg1, lambda, _tg2);
            }

            return true;
        }

        return false;
    }

    /// <summary>Returns false.</summary>
    public override bool IsRational
    {
        get
        {
            return false;
        }
    }

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double GetSectionSize()
    {
        throw new MethodNotImplementedException("BlendFunc_Ruled::GetSectionSize()");
    }

    public override void GetMinimalWeight(NCollection_Array1<double> weigths)
    {
        weigths.Init(1);
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
        nbPoles = 2;
        nbKnots = 2;
        degree = 1;
        nbPoles2d = 2;
    }

    //=======================================================================
    // function : GetTolerance
    // purpose  : Determine les Tolerance a utiliser dans les approximations.
    //=======================================================================
    public override void GetTolerance(double boundTol, double surfTol, double angleTol, math_Vector tol3d,
                                      math_Vector tol1D)
    {
        tol3d.Init(boundTol);
    }

    public override void Knots(NCollection_Array1<double> tKnots)
    {
        tKnots[tKnots.Lower()] = 0.0;
        tKnots[tKnots.Upper()] = 1.0;
    }

    public override void Mults(NCollection_Array1<int> tMults)
    {
        tMults[tMults.Lower()] = tMults[tMults.Upper()] = 2;
    }

    /// <summary>Not computed: returns false.</summary>
    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Vector3D> d2Poles, NCollection_Array1<Point2D> poles2d,
                                 NCollection_Array1<Vector2D> dPoles2d, NCollection_Array1<Vector2D> d2Poles2d,
                                 NCollection_Array1<double> weights, NCollection_Array1<double> dWeights,
                                 NCollection_Array1<double> d2Weights)
    {
        return false;
    }

    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                                 NCollection_Array1<double> weights, NCollection_Array1<double> dWeights)
    {
        int lowp = poles.Lower(), lowp2d = poles2d.Lower();
        double u, v;
        poles[lowp] = p.PointOnS1();
        poles[lowp + 1] = p.PointOnS2();
        p.ParametersOnS1(out u, out v);
        poles2d[lowp2d] = poles2d[lowp2d].WithCoord(u, v);
        p.ParametersOnS2(out u, out v);
        poles2d[lowp2d + 1] = poles2d[lowp2d + 1].WithCoord(u, v);
        weights[lowp] = 1.0;
        weights[lowp + 1] = 1.0;
        if (!p.IsTangencyPoint)
        {
            dPoles[lowp] = p.TangentOnS1();
            dPoles[lowp + 1] = p.TangentOnS2();
            dPoles2d[lowp2d] = p.Tangent2dOnS1();
            dPoles2d[lowp2d + 1] = p.Tangent2dOnS2();
            dWeights[lowp] = 0.0;
            dWeights[lowp + 1] = 0.0;
            return true;
        }

        return false;
    }

    public override void Section(in BlendPoint p, NCollection_Array1<Point3D> poles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<double> weights)
    {
        int lowp = poles.Lower(), lowp2d = poles2d.Lower();
        double u, v;
        poles[lowp] = p.PointOnS1();
        poles[lowp + 1] = p.PointOnS2();
        p.ParametersOnS1(out u, out v);
        poles2d[lowp2d] = poles2d[lowp2d].WithCoord(u, v);
        p.ParametersOnS2(out u, out v);
        poles2d[lowp2d + 1] = poles2d[lowp2d + 1].WithCoord(u, v);
        weights[lowp] = 1.0;
        weights[lowp + 1] = 1.0;
    }

    /// <summary>The axis of rotation of the section plane at the guide parameter <paramref name="prm"/>.</summary>
    public Axis3D AxeRot(double prm)
    {
        Axis3D axrot = new Axis3D();
        Vector3D dirax, dnplan = new Vector3D();
        Point3D oriax = new Point3D();

        _curv.D2(prm, out _ptgui, out _d1gui, out _d2gui);
        _normtg = _d1gui.Magnitude;
        _nplan = _d1gui.Normalized();
        dnplan = Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

        dirax = _nplan.Crossed(dnplan);
        axrot = axrot.WithDirection(new Direction3D(dirax));
        oriax = oriax.WithXYZ(_ptgui.XYZ() + (_normtg / dnplan.Magnitude) * dnplan.Normalized().XYZ());
        axrot = axrot.WithLocation(oriax);
        return axrot;
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
