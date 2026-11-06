// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ChAsym.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ChAsym.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>
/// The function of an asymmetric chamfer between two surfaces along a guide curve: a distance on the first surface
/// and an angle.
/// </summary>
internal class ChAsym : BlendFunction
{
    private readonly SurfaceAdaptorBase _surf1;
    private readonly SurfaceAdaptorBase _surf2;
    private readonly CurveAdaptorBase _curv;
    private CurveAdaptorBase _tcurv;
    private double _param;
    private double _dist1;
    private double _angle;
    private double _tgang;
    private Vector3D _nplan;
    private Point3D _pt1;
    private Vector3D _tsurf1;
    private Point3D _pt2;
    private readonly math_Vector _fx;
    private readonly Matrix _dx;
    private bool _istangent;
    private Vector3D _tg1;
    private Vector2D _tg12d;
    private Vector3D _tg2;
    private Vector2D _tg22d;
    private int _choix;
    private double _distmin;

    public ChAsym(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, CurveAdaptorBase c)
    {
        _surf1 = s1;
        _surf2 = s2;
        _curv = c;
        _tcurv = c;
        _param = 0;
        _dist1 = RealLast();
        _angle = RealLast();
        _tgang = RealLast();
        _fx = new math_Vector(1, 4);
        _dx = new Matrix(1, 4, 1, 4);
        _istangent = true;
        _choix = 0;
        _distmin = RealLast();
        _ = _angle;
    }

    /// <summary>Returns 4.</summary>
    public override int NbEquations()
    {
        return 4;
    }

    public override void Set(double param)
    {
        _param = param;
    }

    //=======================================================================
    // function : Set
    // purpose  : Segmente la courbe a sa partie utile.
    //           La precision est prise arbitrairement petite !?
    //=======================================================================
    public override void Set(double first, double last)
    {
        _tcurv = _curv.Trim(first, last, 1.0e-12);
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

        Vector3D np, dnp, nsurf1, dwtsurf1;
        double normg;

        _tcurv.D2(_param, out Point3D ptgui, out Vector3D d1gui, out Vector3D d2gui);
        normg = d1gui.Magnitude;
        np = d1gui.Normalized();
        dnp = (d2gui - np.Dot(d2gui) * np) / normg;

        if (_choix % 2 != 0)
        {
            np = np.Reversed();
            dnp = dnp.Reversed();
            normg = -normg;
        }

        _surf1.D1(sol[1], sol[2], out _pt1, out Vector3D d1u1, out Vector3D d1v1);
        nsurf1 = d1u1.Crossed(d1v1);
        _tsurf1 = nsurf1.Crossed(np);
        dwtsurf1 = nsurf1.Crossed(dnp);

        _surf2.D1(sol[3], sol[4], out _pt2, out Vector3D d1u2, out Vector3D d1v2);

        Vector3D pguis1 = new Vector3D(ptgui, _pt1), pguis2 = new Vector3D(ptgui, _pt2);
        Vector3D crossVec, s1s2 = new Vector3D(_pt1, _pt2);
        double pScaInv = 1.0 / _tsurf1.Dot(s1s2), f4, temp;
        double maxpiv = 1.0e-9;
        double nordu1 = d1u1.Magnitude, nordv1 = d1v1.Magnitude;

        temp = 2.0 * (nordu1 + nordv1) * s1s2.Magnitude + 2.0 * nordu1 * nordv1;

        Values(sol, valsol, gradsol);

        if (Math.Abs(valsol[1]) < tol && Math.Abs(valsol[2]) < tol
            && Math.Abs(valsol[3]) < 2.0 * _dist1 * tol
            && Math.Abs(valsol[4]) < tol * (1.0 + _tgang) * Math.Abs(pScaInv) * temp)
        {
            secmember[1] = normg - dnp.Dot(pguis1);
            secmember[2] = normg - dnp.Dot(pguis2);
            secmember[3] = -2.0 * d1gui.Dot(pguis1);

            crossVec = _tsurf1.Crossed(s1s2);
            f4 = np.Dot(crossVec) * pScaInv;
            temp = dnp.Dot(crossVec) + np.Dot(dwtsurf1.Crossed(s1s2));

            temp -= f4 * dwtsurf1.Dot(s1s2);
            secmember[4] = pScaInv * temp;

            Gauss resol = new(gradsol, maxpiv);

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
                    math_Vector dedt = new(1, 4);
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
                _tg1 = Vector3D.LinearForm(secmember[1], d1u1, secmember[2], d1v1);
                _tg2 = Vector3D.LinearForm(secmember[3], d1u2, secmember[4], d1v2);
                _tg12d = _tg12d.WithCoord(secmember[1], secmember[2]);
                _tg22d = _tg22d.WithCoord(secmember[3], secmember[4]);
            }

            _distmin = Min(_distmin, _pt1.DistanceTo(_pt2));

            return true;
        }

        _istangent = true;
        return false;
    }

    public override double GetMinimalDistance()
    {
        return _distmin;
    }

    /// <summary>The values (from <paramref name="degF"/> 0) and the derivatives (to <paramref name="degL"/> 1).</summary>
    public bool ComputeValues(math_Vector x, int degF, int degL)
    {
        if (degF > degL)
        {
            return false;
        }

        Vector3D np, d1u1, d1v1, d2u1 = new Vector3D(), d2v1 = new Vector3D(), d2uv1 = new Vector3D(), d1u2 = new Vector3D(),
               d1v2 = new Vector3D(), nsurf1;
        double pScaInv, f4;

        _tcurv.D1(_param, out Point3D ptgui, out Vector3D d1gui);
        _nplan = d1gui.Normalized();
        np = _nplan;

        if (_choix % 2 != 0)
        {
            np = np.Reversed();
        }

        if ((degF == 0) && (degL == 0))
        {
            _surf1.D1(x[1], x[2], out _pt1, out d1u1, out d1v1);
            _pt2 = _surf2.Value(x[3], x[4]);
        }
        else
        {
            _surf1.D2(x[1], x[2], out _pt1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            _surf2.D1(x[3], x[4], out _pt2, out d1u2, out d1v2);
        }

        nsurf1 = d1u1.Crossed(d1v1);
        _tsurf1 = nsurf1.Crossed(np);

        Vector3D nps1 = new Vector3D(ptgui, _pt1), s1s2 = new Vector3D(_pt1, _pt2);
        pScaInv = 1.0 / _tsurf1.Dot(s1s2);
        f4 = np.Dot(_tsurf1.Crossed(s1s2)) * pScaInv;

        if (degF == 0)
        {
            double dist;
            dist = ptgui.XYZ().Dot(np.XYZ());

            _fx[1] = _pt1.XYZ().Dot(np.XYZ()) - dist;
            _fx[2] = _pt2.XYZ().Dot(np.XYZ()) - dist;
            _fx[3] = _dist1 * _dist1 - nps1.SquareMagnitude;
            _fx[4] = _tgang - f4;
        }

        if (degL == 1)
        {
            double temp;
            Vector3D tempVec;
            Vector3D d1utsurf1, d1vtsurf1;
            d1utsurf1 = (d2u1.Crossed(d1v1) + d1u1.Crossed(d2uv1)).Crossed(np);
            d1vtsurf1 = (d2uv1.Crossed(d1v1) + d1u1.Crossed(d2v1)).Crossed(np);

            _dx[1, 1] = np.Dot(d1u1);
            _dx[1, 2] = np.Dot(d1v1);
            _dx[1, 3] = 0.0;
            _dx[1, 4] = 0.0;

            _dx[2, 1] = 0.0;
            _dx[2, 2] = 0.0;
            _dx[2, 3] = np.Dot(d1u2);
            _dx[2, 4] = np.Dot(d1v2);

            tempVec = -2.0 * nps1;
            _dx[3, 1] = d1u1.Dot(tempVec);
            _dx[3, 2] = d1v1.Dot(tempVec);
            _dx[3, 3] = 0.0;
            _dx[3, 4] = 0.0;

            temp = f4 * (d1utsurf1.Dot(s1s2) - _tsurf1.Dot(d1u1));
            temp += np.Dot(_tsurf1.Crossed(d1u1) - d1utsurf1.Crossed(s1s2));
            _dx[4, 1] = temp * pScaInv;

            temp = f4 * (d1vtsurf1.Dot(s1s2) - _tsurf1.Dot(d1v1));
            temp += np.Dot(_tsurf1.Crossed(d1v1) - d1vtsurf1.Crossed(s1s2));
            _dx[4, 2] = temp * pScaInv;

            temp = f4 * _tsurf1.Dot(d1u2) - np.Dot(_tsurf1.Crossed(d1u2));
            _dx[4, 3] = temp * pScaInv;

            temp = f4 * _tsurf1.Dot(d1v2) - np.Dot(_tsurf1.Crossed(d1v2));
            _dx[4, 4] = temp * pScaInv;
        }

        return true;
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        bool error = ComputeValues(x, 0, 0);
        f.Initialized(_fx);
        return error;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        bool error = ComputeValues(x, 1, 1);
        d.Initialized(_dx);
        return error;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        bool error = ComputeValues(x, 0, 1);
        f.Initialized(_fx);
        d.Initialized(_dx);
        return error;
    }

    public override Point3D PointOnS1()
    {
        return _pt1;
    }

    public override Point3D PointOnS2()
    {
        return _pt2;
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
            throw new DomainErrorException("BlendFunc_ChAsym::TangentOnS1");
        }

        return _tg1;
    }

    public override Vector2D Tangent2dOnS1()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_ChAsym::Tangent2dOnS1");
        }

        return _tg12d;
    }

    public override Vector3D TangentOnS2()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_ChAsym::TangentOnS2");
        }

        return _tg2;
    }

    public override Vector2D Tangent2dOnS2()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_ChAsym::Tangent2dOnS2");
        }

        return _tg22d;
    }

    public override bool TwistOnS1()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_ChAsym::TwistOnS1");
        }

        return _tg1.Dot(_nplan) < 0.0;
    }

    public override bool TwistOnS2()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_ChAsym::TwistOnS2");
        }

        return _tg2.Dot(_nplan) < 0.0;
    }

    //=======================================================================
    // function : Tangent
    // purpose  : TgF,NmF et TgL,NmL les tangentes et normales respectives
    //           aux surfaces S1 et S2
    //=======================================================================
    public override void Tangent(double u1, double v1, double u2, double v2, out Vector3D tgF, out Vector3D tgL,
                                 out Vector3D nmF, out Vector3D nmL)
    {
        Vector3D np;
        bool revF = false;
        bool revL = false;

        _tcurv.D1(_param, out _, out Vector3D d1gui);
        np = d1gui.Normalized();

        _surf1.D1(u1, v1, out _, out Vector3D d1u1, out Vector3D d1v1);
        nmF = d1u1.Crossed(d1v1);

        _surf2.D1(u2, v2, out _, out Vector3D d1u2, out Vector3D d1v2);
        nmL = d1u2.Crossed(d1v2);

        tgF = (np.Crossed(nmF)).Normalized();
        tgL = (np.Crossed(nmL)).Normalized();

        if ((_choix == 2) || (_choix == 5))
        {
            revF = true;
            revL = true;
        }

        if ((_choix == 4) || (_choix == 7))
        {
            revL = true;
        }

        if ((_choix == 3) || (_choix == 8))
        {
            revF = true;
        }

        if (revF)
        {
            tgF = tgF.Reversed();
        }

        if (revL)
        {
            tgL = tgL.Reversed();
        }
    }

    /// <summary>The section at (U1, V1), (U2, V2) as a line with its parameter range.</summary>
    public void Section(double param, double u1, double v1, double u2, double v2, out double pdeb, out double pfin,
                        ref Line3D c)
    {
        Point3D pt1 = _surf1.Value(u1, v1);
        Point3D pt2 = _surf2.Value(u2, v2);
        Direction3D dir = new Direction3D(new Vector3D(pt1, pt2));

        c = c.WithLocation(pt1);
        c = c.WithDirection(dir);

        pdeb = 0.0;
        pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, pt2);
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
        throw new MethodNotImplementedException("BlendFunc_ChAsym::GetSectionSize()");
    }

    public override void GetMinimalWeight(NCollection_Array1<double> weights)
    {
        weights.Init(1);
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
        nbPoles2d = 2;
        nbKnots = 2;
        degree = 1;
    }

    //=======================================================================
    // function : GetTolerance
    // purpose  : Determine les Tolerances a utiliser dans les approximations.
    //=======================================================================
    public override void GetTolerance(double boundTol, double surfTol, double angleTol, math_Vector tol3d,
                                      math_Vector tol1D)
    {
        tol3d.Init(boundTol);
    }

    public override void Knots(NCollection_Array1<double> tKnots)
    {
        tKnots[1] = 0.0;
        tKnots[2] = 1.0;
    }

    public override void Mults(NCollection_Array1<int> tMults)
    {
        tMults[1] = 2;
        tMults[2] = 2;
    }

    public override void Section(in BlendPoint p, NCollection_Array1<Point3D> poles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<double> weights)
    {
        double u1, v1, u2, v2, prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();
        math_Vector x = new(1, 4), f = new(1, 4);

        p.ParametersOnS1(out u1, out v1);
        p.ParametersOnS2(out u2, out v2);
        x[1] = u1;
        x[2] = v1;
        x[3] = u2;
        x[4] = v2;
        poles2d[poles2d.Lower()] = poles2d[poles2d.Lower()].WithCoord(u1, v1);
        poles2d[poles2d.Upper()] = poles2d[poles2d.Upper()].WithCoord(u2, v2);

        Set(prm);
        Value(x, f);
        poles[low] = PointOnS1();
        poles[upp] = PointOnS2();
        weights[low] = 1.0;
        weights[upp] = 1.0;
    }

    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                                 NCollection_Array1<double> weights, NCollection_Array1<double> dWeights)
    {
        math_Vector sol = new(1, 4), valsol = new(1, 4), secmember = new(1, 4);
        Matrix gradsol = new(1, 4, 1, 4);
        double prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();

        p.ParametersOnS1(out sol[1], out sol[2]);
        p.ParametersOnS2(out sol[3], out sol[4]);
        Set(prm);

        poles2d[poles2d.Lower()] = poles2d[poles2d.Lower()].WithCoord(sol[1], sol[2]);
        poles2d[poles2d.Upper()] = poles2d[poles2d.Upper()].WithCoord(sol[3], sol[4]);

        // OCCT: the poles are the points of the previous computation, set before the D1s below (decision 1040).
        poles[low] = PointOnS1();
        poles[upp] = PointOnS2();
        weights[low] = 1.0;
        weights[upp] = 1.0;

        Vector3D np, dnp, nsurf1, dwtsurf1;
        double normg;

        _tcurv.D2(_param, out Point3D ptgui, out Vector3D d1gui, out Vector3D d2gui);
        normg = d1gui.Magnitude;
        np = d1gui.Normalized();
        dnp = (d2gui - np.Dot(d2gui) * np) / normg;

        if (_choix % 2 != 0)
        {
            np = np.Reversed();
            dnp = dnp.Reversed();
            normg = -normg;
        }

        _surf1.D1(sol[1], sol[2], out _pt1, out Vector3D d1u1, out Vector3D d1v1);
        nsurf1 = d1u1.Crossed(d1v1);
        _tsurf1 = nsurf1.Crossed(np);
        dwtsurf1 = nsurf1.Crossed(dnp);

        _surf2.D1(sol[3], sol[4], out _pt2, out Vector3D d1u2, out Vector3D d1v2);

        Vector3D pguis1 = new Vector3D(ptgui, _pt1), pguis2 = new Vector3D(ptgui, _pt2);
        Vector3D crossVec, s1s2 = new Vector3D(_pt1, _pt2);
        double pScaInv = 1.0 / _tsurf1.Dot(s1s2), f4, temp;
        double maxpiv = 1.0e-9;
        double nordu1 = d1u1.Magnitude, nordv1 = d1v1.Magnitude;

        temp = 2.0 * (nordu1 + nordv1) * s1s2.Magnitude + 2.0 * nordu1 * nordv1;

        Values(sol, valsol, gradsol);

        secmember[1] = normg - dnp.Dot(pguis1);
        secmember[2] = normg - dnp.Dot(pguis2);
        secmember[3] = -2.0 * d1gui.Dot(pguis1);

        crossVec = _tsurf1.Crossed(s1s2);
        f4 = np.Dot(crossVec) * pScaInv;
        temp = dnp.Dot(crossVec) + np.Dot(dwtsurf1.Crossed(s1s2));

        temp -= f4 * dwtsurf1.Dot(s1s2);
        secmember[4] = pScaInv * temp;

        Gauss resol = new(gradsol, maxpiv);

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
                math_Vector dedt = new(1, 4);
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
            _tg1 = Vector3D.LinearForm(secmember[1], d1u1, secmember[2], d1v1);
            _tg2 = Vector3D.LinearForm(secmember[3], d1u2, secmember[4], d1v2);
            _tg12d = _tg12d.WithCoord(secmember[1], secmember[2]);
            _tg22d = _tg22d.WithCoord(secmember[3], secmember[4]);
        }

        _distmin = Min(_distmin, _pt1.DistanceTo(_pt2));

        if (!_istangent)
        {
            dPoles2d[poles2d.Lower()] = dPoles2d[poles2d.Lower()].WithCoord(Tangent2dOnS1().X, Tangent2dOnS1().Y);
            dPoles2d[poles2d.Upper()] = dPoles2d[poles2d.Upper()].WithCoord(Tangent2dOnS2().X, Tangent2dOnS2().Y);
            dPoles[low] = TangentOnS1();
            dPoles[upp] = TangentOnS2();
            dWeights[low] = 0.0;
            dWeights[upp] = 0.0;
        }

        return (!_istangent);
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

    /// <summary>Sets the distance on the first surface, the angle and the "quadrant".</summary>
    public void Set(double dist1, double angle, int choix)
    {
        _dist1 = Math.Abs(dist1);
        _angle = angle;
        _tgang = Math.Tan(angle);
        _choix = choix;
    }
}
