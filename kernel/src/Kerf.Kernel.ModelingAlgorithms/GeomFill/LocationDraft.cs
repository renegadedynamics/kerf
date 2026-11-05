// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_LocationDraft.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_LocationDraft.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The location law of a draft sweep: a <see cref="DraftTrihedron"/> along the curve and, with a stop
/// surface, the trace of the generatrix on that surface as a 2D restriction.
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="SetCurve"/>, <see cref="SetStopSurf"/>, <see cref="SetAngle"/>,
/// <see cref="SetTrsf"/> and <see cref="SetInterval"/> are public. It keeps the caller's curve and surface
/// adaptors and only evaluates them. The seed search, which OCCT repeats in D0, D1 and D2, is a private helper.
/// </remarks>
internal class LocationDraft : SurfaceLocationLaw
{
    protected NCollection_HArray1<Point2D> _poles2d;

    private Matrix3x3 _trans;
    private readonly DraftTrihedron _law;
    private SurfaceAdaptorBase? _surf;
    private CurveAdaptorBase? _curve;
    private CurveAdaptorBase? _trimmed;
    private readonly Direction3D _dir;
    private double _angle;
    private readonly int _nbPts;
    private bool _intersec;
    private bool _withTrans;

    public LocationDraft(in Direction3D direction, double angle)
    {
        _dir = direction; // direction de depouille

        _angle = angle; // angle de depouille (teta prime)

        _surf = null;
        _law = new DraftTrihedron(new Vector3D(_dir), angle); // triedre
        _nbPts = 41;                                                    // nb de points utilises pour les calculs
        _poles2d = new NCollection_HArray1<Point2D>(1, 2 * _nbPts);
        _intersec = false; // intersection avec surface d'arret ?
        _withTrans = false;
        _trans = new Matrix3x3();
    }

    public override SurfaceLocationLaw Copy()
    {
        TrihedronLaw law;
        law = _law.Copy();
        LocationDraft copy = new LocationDraft(_dir, _angle);
        copy.SetCurve(_curve!);
        copy.SetStopSurf(_surf);
        if (_withTrans)
        {
            copy.SetTrsf(_trans);
        }

        return copy;
    }

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

    /// <summary>
    /// Sets the curve and computes the poles on the stop surface (the intersections of the generatrix with the
    /// surface at myNbPts points of the curve).
    /// </summary>
    public override bool SetCurve(CurveAdaptorBase c)
    {
        _curve = c;
        _trimmed = c;
        bool isOK = _law.SetCurve(c);

        Prepare();
        return isOK;
    }

    /// <summary>Sets the stop surface (null for none).</summary>
    public void SetStopSurf(SurfaceAdaptorBase? surf)
    {
        _surf = surf;
        Prepare();
    }

    public void SetAngle(double angle)
    {
        _angle = angle;
        _law.SetAngle(_angle);
        Prepare();
    }

    // Lays the milestones of the intersection of the draft with the surface.
    private void Prepare()
    {
        if (_surf is null)
        {
            _intersec = false;
            return;
        }

        _intersec = true;

        int ii, jj;
        double f, l, t;
        Vector3D d, T = new Vector3D(), n = new Vector3D(), b = new Vector3D();
        LineCurve3D L;
        CurveSurfaceIntersectionPoint p1, p2;
        f = _curve!.FirstParameter();
        l = _curve.LastParameter();

        for (ii = 1; ii <= _nbPts; ii++)
        {
            t = (double)(_nbPts - ii) * f + (double)(ii - 1) * l;
            t /= _nbPts - 1;
            _curve.D0(t, out Point3D P);
            _law.D0(t, ref T, ref n, ref b);

            // Generatrice
            d = Math.Cos(_angle) * b + Math.Sin(_angle) * n;

            L = new LineCurve3D(P, new Direction3D(d));

            HInter Int = new HInter(); // intersection surface / generatrice
            CurveAdaptor ac = new CurveAdaptor(L);
            Int.Perform(ac, _surf); // calcul de l'intersection

            if (Int.PointCount > 0) // il y a au moins 1 intersection
            {
                p1 = Int.Point(1); // 1er  point d'intersection

                for (jj = 2; jj <= Int.PointCount; jj++)
                {
                    p2 = Int.Point(jj);
                    if (p1.W() > p2.W())
                    {
                        p1 = p2; // point le plus proche
                    }
                } // for_jj

                Point2D p = new Point2D(p1.W(), t);      // point de la courbe
                Point2D q = new Point2D(p1.U(), p1.V()); // point sur la surface
                _poles2d.SetValue(2 * ii - 1, p);          // point de la courbe (indice impair)
                _poles2d.SetValue(2 * ii, q);              // point sur la surface (indice pair)
            }
            else
            { // au moins un point ou il n'y a pas intersection
                _intersec = false;
            }
        } // for_ii
    }

    public override CurveAdaptorBase? GetCurve() => _curve;

    public override bool D0(double param, ref Matrix3x3 m, ref Vector3D v)
    {
        bool ok;
        Vector3D t = new Vector3D(), n = new Vector3D(), b = new Vector3D();

        _trimmed!.D0(param, out Point3D p);
        v = v.WithXYZ(p.XYZ());

        ok = _law.D0(param, ref t, ref n, ref b);
        if (!ok)
        {
            return ok;
        }
        m = m.WithCols(n.XYZ(), b.XYZ(), t.XYZ());

        if (_withTrans)
        {
            m *= _trans;
        }

        return true;
    }

    // The seed of the search: OCCT's block repeated in D0, D1 and D2. Finds the interval of Param among the
    // milestones and interpolates linearly, or takes the milestone; leaves zeros past the last one.
    private void Seed(double param, out double w0, out double u0, out double v0)
    {
        double t1, t2, paramt1, t2Param;
        u0 = 0;
        v0 = 0;
        w0 = 0;

        int ii = 1;
        // on recherche l'intervalle auquel appartient Param
        while (ii < 2 * _nbPts && _poles2d.Value(ii).Coord(2) < param)
        {
            ii = ii + 2;
        }

        if (ii < 2 * _nbPts && !IsEqual(_poles2d.Value(ii).Coord(2), param))
        {
            // interpolation lineaire pour initialiser le germe de la recherche
            // OCCT: for Param before the first milestone, ii is 1 and OCCT reads the poles at -1 and 0 out of
            // bounds; the port's array throws.
            t1 = _poles2d.Value(ii).Coord(2);
            t2 = _poles2d.Value(ii - 2).Coord(2);

            paramt1 = (param - t1) / (t2 - t1);
            t2Param = (t2 - param) / (t2 - t1);

            w0 = _poles2d.Value(ii - 2).Coord(1) * paramt1 + _poles2d.Value(ii).Coord(1) * t2Param;
            u0 = _poles2d.Value(ii - 1).Coord(1) * paramt1 + _poles2d.Value(ii + 1).Coord(1) * t2Param;
            v0 = _poles2d.Value(ii - 1).Coord(2) * paramt1 + _poles2d.Value(ii + 1).Coord(2) * t2Param;
        } // if

        // on est sur un param ou les points ont deja ete calcules
        else if (ii < 2 * _nbPts && IsEqual(_poles2d.Value(ii).Coord(2), param))
        {
            w0 = _poles2d.Value(ii).Coord(1);
            u0 = _poles2d.Value(ii + 1).Coord(1);
            v0 = _poles2d.Value(ii + 1).Coord(2);
        } // else if
    }

    /// <summary>The location and the intersection (C0) on the stop surface.</summary>
    public override bool D0(double param, ref Matrix3x3 m, ref Vector3D v, NCollection_Array1<Point2D> poles2d)
    {
        bool ok;
        Vector3D d, t = new Vector3D(), n = new Vector3D(), b = new Vector3D();

        _curve!.D0(param, out Point3D P);
        v = v.WithXYZ(P.XYZ());

        ok = _law.D0(param, ref t, ref n, ref b);
        if (!ok)
        {
            return ok;
        }
        m = m.WithCols(n.XYZ(), b.XYZ(), t.XYZ());

        if (_withTrans)
        {
            m *= _trans;
        }

        if (_intersec)
        {
            // la generatrice intersecte la surface d'arret
            // la generatrice
            d = Math.Cos(_angle) * b + Math.Sin(_angle) * n;

            LineCurve3D l = new LineCurve3D(P, new Direction3D(d));
            CurveAdaptor g = new CurveAdaptor(l);

            Seed(param, out double w0, out double u0, out double v0);

            // recherche de la solution (pt d'intersection generatrice / surface)
            // point initial
            math_Vector x = new math_Vector(1, 3);
            x[1] = w0;
            x[2] = u0;
            x[3] = v0;

            // tolerance sur X
            math_Vector xTol = new math_Vector(1, 3);
            xTol.Init(0.00001);

            // tolerance sur F
            double fTol = 0.0000001;
            int iter = 100;

            // fonction dont il faut trouver la racine : G(W)-S(U,V)=0
            FunctionDraft e = new FunctionDraft(_surf!, g);

            // resolution
            NewtonFunctionSetRoot result = new NewtonFunctionSetRoot(e, xTol, fTol, iter);
            result.Perform(e, x);

            if (result.IsDone)
            {
                math_Vector r = new math_Vector(1, 3);
                result.Root(r); // solution

                Point2D p = new Point2D(r[2], r[3]);  // point sur la surface
                Point2D q = new Point2D(r[1], param); // point de la courbe
                poles2d.SetValue(1, p);
                poles2d.SetValue(2, q);
            }
            else
            {
                return false;
            }
        } // if_Intersec

        // la generatrice n'intersecte pas la surface d'arret
        return true;
    }

    /// <summary>The location, the intersection (C1) on the stop surface and their first derivatives.</summary>
    public override bool D1(double param, ref Matrix3x3 m, ref Vector3D v, ref Matrix3x3 dm, ref Vector3D dv,
                            NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d)
    {
        bool ok;
        Vector3D d, t = new Vector3D(), n = new Vector3D(), b = new Vector3D(), dt = new Vector3D(), dn = new Vector3D(),
               db = new Vector3D();

        _curve!.D1(param, out Point3D P, out dv);
        v = v.WithXYZ(P.XYZ());

        ok = _law.D1(param, ref t, ref dt, ref n, ref dn, ref b, ref db);
        if (!ok)
        {
            return false;
        }

        m = m.WithCols(n.XYZ(), b.XYZ(), t.XYZ());
        dm = dm.WithCols(dn.XYZ(), db.XYZ(), dt.XYZ());

        if (_withTrans)
        {
            m *= _trans;
            dm *= _trans;
        }

        if (_intersec)
        { // la generatrice intersecte la surface d'arret
            // la generatrice
            d = Math.Cos(_angle) * b + Math.Sin(_angle) * n;

            LineCurve3D l = new LineCurve3D(P, new Direction3D(d));
            CurveAdaptor g = new CurveAdaptor(l);

            // recherche de la solution (pt d'intersection  generatrice / surface)
            Seed(param, out double w0, out double u0, out double v0);

            // germe
            math_Vector x = new math_Vector(1, 3);
            x[1] = w0;
            x[2] = u0;
            x[3] = v0;

            // tolerance sur X
            math_Vector xTol = new math_Vector(1, 3);
            xTol.Init(0.0001);

            // tolerance sur F
            double fTol = 0.000001;
            int iter = 100;

            // fonction dont il faut trouver la racine : G(W)-S(U,V)=0
            FunctionDraft e = new FunctionDraft(_surf!, g);

            // resolution
            NewtonFunctionSetRoot result = new NewtonFunctionSetRoot(e, xTol, fTol, iter);
            result.Perform(e, x);

            if (result.IsDone)
            {
                math_Vector r = new math_Vector(1, 3);
                result.Root(r); // solution

                Point2D p = new Point2D(r[2], r[3]);  // point sur la surface
                Point2D q = new Point2D(r[1], param); // point de la courbe
                poles2d.SetValue(1, p);
                poles2d.SetValue(2, q);

                // derivee de la fonction par rapport a Param
                math_Vector dedt = new math_Vector(1, 3, 0);
                e.DerivT(_trimmed!, param, r[1], dn, _angle, dedt); // dE/dt => DEDT

                math_Vector dsdt = new math_Vector(1, 3, 0);

                Matrix dedx = new Matrix(1, 3, 1, 3, 0);
                e.Derivatives(r, dedx); // dE/dx au point R => DEDX

                // resolution du syst. lin. : DEDX*DSDT = -DEDT
                Gauss ga = new Gauss(dedx);
                if (ga.IsDone)
                {
                    ga.Solve(dedt.Opposite(), dsdt);              // resolution du syst. lin.
                    Vector2D dp = new Vector2D(dsdt[2], dsdt[3]); // surface
                    Vector2D dq = new Vector2D(dsdt[1], 1);       //  courbe
                    dPoles2d.SetValue(1, dp);
                    dPoles2d.SetValue(2, dq);
                } // if
            } // if_Result
            else
            { // la generatrice n'intersecte pas la surface d'arret
                return false;
            }
        } // if_Intersec
        return true;
    }

    /// <summary>The location, the intersection (C2) on the stop surface and their first and second derivatives.</summary>
    public override bool D2(double param, ref Matrix3x3 m, ref Vector3D v, ref Matrix3x3 dm, ref Vector3D dv, ref Matrix3x3 d2M,
                            ref Vector3D d2V, NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                            NCollection_Array1<Vector2D> d2Poles2d)
    {
        bool ok;
        Vector3D d, T = new Vector3D(), n = new Vector3D(), b = new Vector3D(), dt = new Vector3D(), dn = new Vector3D(),
               db = new Vector3D(), d2T = new Vector3D(), d2N = new Vector3D(), d2B = new Vector3D();

        _curve!.D2(param, out Point3D P, out dv, out d2V);
        v = v.WithXYZ(P.XYZ());

        ok = _law.D2(param, ref T, ref dt, ref d2T, ref n, ref dn, ref d2N, ref b, ref db, ref d2B);
        if (!ok)
        {
            return ok;
        }

        m = m.WithCols(n.XYZ(), b.XYZ(), T.XYZ());
        dm = dm.WithCols(dn.XYZ(), db.XYZ(), dt.XYZ());
        d2M = d2M.WithCols(d2N.XYZ(), d2B.XYZ(), d2T.XYZ());

        if (_withTrans)
        {
            m *= _trans;
            dm *= _trans;
            d2M *= _trans;
        }
        if (_intersec)
        { // la generatrice intersecte la surface d'arret
            // la generatrice
            d = Math.Cos(_angle) * b + Math.Sin(_angle) * n;

            LineCurve3D l = new LineCurve3D(P, new Direction3D(d));
            CurveAdaptor g = new CurveAdaptor(l);

            Seed(param, out double w0, out double u0, out double v0);

            // recherche de la solution (pt d'intersection generatrice / surface)
            // germe
            math_Vector x = new math_Vector(1, 3);
            x[1] = w0;
            x[2] = u0;
            x[3] = v0;

            // tolerance sur X
            math_Vector xTol = new math_Vector(1, 3);
            xTol.Init(0.0001);

            // tolerance sur F
            double fTol = 0.000001;
            int iter = 150;

            // fonction dont il faut trouver la racine : G(W)-S(U,V)=0
            FunctionDraft e = new FunctionDraft(_surf!, g);

            // resolution
            NewtonFunctionSetRoot result = new NewtonFunctionSetRoot(e, xTol, fTol, iter);
            result.Perform(e, x);

            if (result.IsDone)
            {
                math_Vector r = new math_Vector(1, 3);
                result.Root(r); // solution

                // solution
                Point2D p = new Point2D(r[2], r[3]);  // point sur la surface
                Point2D q = new Point2D(r[1], param); // point de la courbe
                poles2d.SetValue(1, p);
                poles2d.SetValue(2, q);

                // premiere derivee de la fonction
                math_Vector dedt = new math_Vector(1, 3, 0);
                e.DerivT(_trimmed!, param, r[1], dn, _angle, dedt); // dE/dt => DEDT

                math_Vector dsdt = new math_Vector(1, 3, 0);

                Matrix dedx = new Matrix(1, 3, 1, 3, 0);
                e.Derivatives(r, dedx); // dE/dx => DEDX

                // resolution du syst. lin.
                Gauss ga = new Gauss(dedx);
                if (ga.IsDone)
                {
                    ga.Solve(dedt.Opposite(), dsdt);
                    Vector2D dp = new Vector2D(dsdt[2], dsdt[3]); // surface
                    Vector2D dq = new Vector2D(dsdt[1], 1);       //  courbe
                    dPoles2d.SetValue(1, dp);
                    dPoles2d.SetValue(2, dq);
                } // if

                // deuxieme derivee
                SurfaceTensor d2EDX2 = new SurfaceTensor(3, 3, 3);
                e.Deriv2X(r, d2EDX2); // d2E/dx2

                math_Vector d2EDT2 = new math_Vector(1, 3, 0);

                e.Deriv2T(_trimmed!, param, r[1], d2N, _angle, d2EDT2); // d2E/dt2

                Matrix d2EDTDX = new Matrix(1, 3, 1, 3, 0);
                e.DerivTX(dn, _angle, d2EDTDX); // d2E/dtdx

                math_Vector d2SDT2 = new math_Vector(1, 3, 0); // d2s/dt2
                Matrix t = new Matrix(1, 3, 1, 3, 0);
                d2EDX2.Multiply(dsdt, t);

                // resolution du syst. lin.
                Gauss ga1 = new Gauss(dedx);
                if (ga1.IsDone)
                {
                    ga1.Solve(-t * dsdt - 2 * d2EDTDX * dsdt - d2EDT2, d2SDT2);
                    Vector2D d2p = new Vector2D(d2SDT2[2], d2SDT2[3]); // surface
                    Vector2D d2q = new Vector2D(d2SDT2[1], 0);         // courbe
                    d2Poles2d.SetValue(1, d2p);
                    d2Poles2d.SetValue(2, d2q);
                } // if
                else
                { // la generatrice n'intersecte pas la surface d'arret
                    return false;
                }
            } // if_Result
        } // if_Intersec
        return true;
    }

    public override bool HasFirstRestriction => false;

    public override bool HasLastRestriction => _intersec;

    public override int TraceNumber()
    {
        if (_intersec)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    public override int NbIntervals(Continuity s) => _law.NbIntervals(s);

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        _law.Intervals(t, s);
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

    public override void Resolution(int index, double tol, out double tolU, out double tolV)
    {
        if (index == 1)
        {
            tolU = _surf!.UResolution(tol);
            tolV = _surf.VResolution(tol);
        }
        else
        {
            tolU = tol;
            tolV = tol;
        }
    }

    /// <summary>1: the trihedrons are normed.</summary>
    public override double GetMaximalNorm() => 1.0;

    public override void GetAverageLaw(ref Matrix3x3 am, ref Vector3D av)
    {
        int ii;
        double u, delta;
        Vector3D v1 = new Vector3D(), v2 = new Vector3D(), v3 = new Vector3D(), v = new Vector3D();

        _law.GetAverageLaw(ref v1, ref v2, ref v3);
        // OCCT: the columns are (tangent, normal, binormal) here, but (normal, binormal, tangent) in D0, D1 and D2.
        am = am.WithCols(v1.XYZ(), v2.XYZ(), v3.XYZ());

        av = av.WithCoord(0.0, 0.0, 0.0);
        delta = (_trimmed!.LastParameter() - _trimmed.FirstParameter()) / 10;
        u = _trimmed.FirstParameter();
        for (ii = 0; ii <= 10; ii++, u += delta)
        {
            v = v.WithXYZ(_trimmed.Value(u).XYZ());
            av += v;
        }
        av /= 11;
    }

    public override bool IsTranslation(ref double error) => _law.IsConstant;

    public override bool IsRotation(ref double error)
    {
        CurveKind type;
        error = 0;
        type = _curve!.GetType();
        if (type == Circle)
        {
            return _law.IsOnlyBy3dCurve;
        }
        return false;
    }

    public override void Rotation(ref Point3D centre)
    {
        centre = _curve!.Circle().Location;
    }

    /// <summary>Whether the generatrix meets the stop surface all along the curve.</summary>
    public bool IsIntersec => _intersec;

    /// <summary>The draft direction.</summary>
    public Direction3D Direction() => _dir;
}
