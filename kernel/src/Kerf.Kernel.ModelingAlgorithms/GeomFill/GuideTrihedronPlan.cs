// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_GuideTrihedronPlan.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_GuideTrihedronPlan.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A trihedron law with a guide, by planes: the normal points from the path to where the guide crosses the plane
/// normal to the path's (Frenet) tangent.
/// </summary>
/// <remarks>
/// As in OCCT, the class has its own <c>myCurve</c> and <c>myTrimmed</c> (hiding the base's, which stay null), so
/// the base's <see cref="TrihedronLaw.GetInterval"/> throws (OCCT dereferences a null handle there;
/// occt-issues.md). <see cref="D2"/> computes only the Frenet trihedron of the path and returns false, as in OCCT.
/// </remarks>
internal class GuideTrihedronPlan : TrihedronWithGuide
{
    private new CurveAdaptorBase? _trimmed;
    private new CurveAdaptorBase? _curve;
    private readonly NCollection_HArray2<Point2D> _pole;
    private readonly math_Vector _x;
    private readonly math_Vector _xTol;
    private readonly math_Vector _inf;
    private readonly math_Vector _sup;
    private readonly Frenet _frenet;
    private readonly int _nbPts;
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

    public GuideTrihedronPlan(CurveAdaptorBase guide)
    {
        _x = new math_Vector(1, 1);
        _xTol = new math_Vector(1, 1);
        _inf = new math_Vector(1, 1);
        _sup = new math_Vector(1, 1);
        _status = SurfacePipeError.PipeOk;
        _curve = null;
        _guide = guide; // guide
        _trimG = guide;
        _nbPts = 20; // nb points pour calculs
        _pole = new NCollection_HArray2<Point2D>(1, 1, 1, _nbPts); // tab pr stocker Pprime (pt sur guide)
        _frenet = new Frenet();
        _xTol.Init(1.0e-6);
        _xTol[1] = _guide.Resolution(1.0e-6);
    }

    /// <summary>Computes myNbPts points on the guide (its intersections with the normal planes of the path).</summary>
    private void Init()
    {
        _status = SurfacePipeError.PipeOk;
        Vector3D tangent = new Vector3D(), normal = new Vector3D(), biNormal = new Vector3D();
        int ii;
        double t, deltaG, w = 0.0;
        double f = _curve!.FirstParameter();
        double l = _curve.LastParameter();

        PlaneSurface plan;
        SurfaceAdaptor pl;
        CurveSurfaceIntersectionPoint pInt;
        HInter Int = new HInter();
        _frenet.SetCurve(_curve);
        deltaG = (_guide!.LastParameter() - _guide.FirstParameter()) / 2;

        _inf[1] = _guide.FirstParameter() - deltaG;
        _sup[1] = _guide.LastParameter() + deltaG;

        if (!_guide.IsPeriodic)
        {
            _trimG = _guide.Trim(_guide.FirstParameter() - deltaG / 100, _guide.LastParameter() + deltaG / 100,
                                   deltaG * 1.0e-7);
        }
        else
        {
            _trimG = _guide;
        }

        deltaG /= 3;
        for (ii = 1; ii <= _nbPts; ii++)
        {
            t = (double)(_nbPts - ii) * f + (double)(ii - 1) * l;
            t /= (_nbPts - 1);
            _curve.D0(t, out Point3D p);
            _frenet.D0(t, ref tangent, ref normal, ref biNormal);
            plan = new PlaneSurface(p, new Direction3D(tangent));
            pl = new SurfaceAdaptor(plan);

            Int.Perform(_trimG, pl); // intersection plan / guide
            if (Int.PointCount == 0)
            {
                w = (Math.Abs(_guide.LastParameter() - w) > Math.Abs(_guide.FirstParameter() - w)
                         ? _guide.FirstParameter()
                         : _guide.LastParameter());

                _status = SurfacePipeError.PlaneNotIntersectGuide;
            }
            else
            {
                Point3D pmin;
                pInt = Int.Point(1);
                pmin = pInt.Pnt();
                double dmin = p.DistanceTo(pmin);
                for (int jj = 2; jj <= Int.PointCount; jj++)
                {
                    pmin = Int.Point(jj).Pnt();
                    if (p.DistanceTo(pmin) < dmin)
                    {
                        pInt = Int.Point(jj);
                        dmin = p.DistanceTo(pmin);
                    }
                } // for_jj

                w = pInt.W();
            }
            if (ii > 1)
            {
                double diff = w - _pole.Value(1, ii - 1).Y;
                if (Math.Abs(diff) > deltaG)
                {
                    if (_guide.IsPeriodic)
                    {
                        InGoodPeriod(_pole.Value(1, ii - 1).Y, _guide.Period(), ref w);
                    }
                }
            }

            Point2D p1 = new Point2D(t, w); // on stocke les parametres
            _pole.SetValue(1, ii, p1);
        } // for_ii
    }

    public override bool SetCurve(CurveAdaptorBase c)
    {
        _curve = c;
        if (_curve is not null)
        {
            Init();
        }
        return true;
    }

    public override CurveAdaptorBase Guide() => _guide!;

    public override bool D0(double param, ref Vector3D tangent, ref Vector3D normal, ref Vector3D biNormal)
    {
        Point3D pprime;
        _curve!.D0(param, out Point3D p);
        _frenet.D0(param, ref tangent, ref normal, ref biNormal);

        // initialisation de la recherche
        InitX(param);

        int iter = 50;

        // fonction dont il faut trouver la racine : G(W)-Pl(U,V)=0
        PlanFunc e = new PlanFunc(p, tangent, _guide!);

        // resolution
        FunctionRoot result = new FunctionRoot(e, _x[1], _xTol[1], _inf[1], _sup[1], iter);

        if (result.IsDone)
        {
            double res = result.Root();
            pprime = _trimG!.Value(res); // pt sur courbe guide
            Vector3D n = new Vector3D(p, pprime); // vecteur definissant la normale du triedre

            normal = n.Normalized();
            biNormal = tangent.Crossed(normal);
            biNormal = biNormal.Normalized();
        }
        else
        { // Erreur...
            _status = SurfacePipeError.PlaneNotIntersectGuide;
            return false;
        }

        return true;
    }

    public override bool D1(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D normal, ref Vector3D dNormal,
                            ref Vector3D biNormal, ref Vector3D dBiNormal)
    {
        // triedre de frenet sur la trajectoire
        _curve!.D1(param, out Point3D p, out Vector3D to);
        _frenet.D1(param, ref tangent, ref dTangent, ref normal, ref dNormal, ref biNormal, ref dBiNormal);

        // tolerance sur E
        int iter = 50;

        // fonction dont il faut trouver la racine : G(W)-Pl(U,V)=0
        InitX(param);
        PlanFunc e = new PlanFunc(p, tangent, _guide!);

        // resolution
        FunctionRoot result = new FunctionRoot(e, _x[1], _xTol[1], _inf[1], _sup[1], iter);

        if (result.IsDone)
        {
            double res = result.Root();
            _trimG!.D1(res, out Point3D pg, out Vector3D tg);
            Vector3D n = new Vector3D(p, pg), dn = new Vector3D(); // vecteur definissant la normale du triedre
            double norm = n.Magnitude;
            if (norm < 1.0e-12)
            {
                norm = 1.0;
            }
            n /= norm;

            normal = n;
            biNormal = tangent.Crossed(normal);

            // derivee premiere du triedre
            double dtg_dt;
            e.Derivative(res, out double dedx);
            e.DEDT(res, to, dTangent, out double dedt);
            dtg_dt = -dedt / dedx;

            dn = Vector3D.LinearForm(dtg_dt, tg, -1, to);
            dNormal = Vector3D.LinearForm(-(n * dn), n, dn);
            dNormal /= norm;
            dBiNormal = Vector3D.LinearForm(tangent.Crossed(dNormal), dTangent.Crossed(normal));
        }
        else
        { // Erreur...
            _status = SurfacePipeError.PlaneNotIntersectGuide;
            return false;
        }

        return true;
    }

    /// <summary>Computes the Frenet trihedron of the path and its derivatives, and returns false (as OCCT).</summary>
    public override bool D2(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D d2Tangent, ref Vector3D normal,
                            ref Vector3D dNormal, ref Vector3D d2Normal, ref Vector3D biNormal, ref Vector3D dBiNormal,
                            ref Vector3D d2BiNormal)
    {
        _curve!.D2(param, out _, out _, out _);
        // triedre de Frenet sur la trajectoire
        _frenet.D2(param, ref tangent, ref dTangent, ref d2Tangent, ref normal, ref dNormal, ref d2Normal, ref biNormal,
                  ref dBiNormal, ref d2BiNormal);
        return false;
    }

    public override TrihedronLaw Copy()
    {
        GuideTrihedronPlan copy = new GuideTrihedronPlan(_guide!);
        // OCCT sets the curve even when it is null
        copy.SetCurve(_curve!);
        return copy;
    }

    public override SurfacePipeError ErrorStatus() => _status;

    private static Continuity CurveShape(Continuity s)
    {
        switch (s)
        {
            case C0:
                return C1;
            case C1:
                return C2;
            case C2:
                return C3;
            default:
                return CN;
        }
    }

    /// <summary>The intervals of the path (a provisional version in OCCT: the guide is not taken into account).</summary>
    public override int NbIntervals(Continuity s) => _curve!.NbIntervals(CurveShape(s));

    public override void Intervals(NCollection_Array1<double> tt, Continuity s)
    {
        _curve!.Intervals(tt, CurveShape(s));
    }

    public override void SetInterval(double first, double last)
    {
        _trimmed = _curve!.Trim(first, last, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    public override void GetAverageLaw(ref Vector3D aTangent, ref Vector3D aNormal, ref Vector3D aBiNormal)
    {
        int ii;
        double t, delta = (_curve!.LastParameter() - _curve.FirstParameter()) / 20.001;

        aTangent = aTangent.WithCoord(0.0, 0.0, 0.0);
        aNormal = aNormal.WithCoord(0.0, 0.0, 0.0);
        aBiNormal = aBiNormal.WithCoord(0.0, 0.0, 0.0);
        Vector3D T = new Vector3D(), n = new Vector3D(), b = new Vector3D();

        for (ii = 1; ii <= 20; ii++)
        {
            t = _curve.FirstParameter() + (ii - 1) * delta;
            D0(t, ref T, ref n, ref b);
            aTangent += T;
            aNormal += n;
            aBiNormal += b;
        }
        aTangent /= 20;
        aNormal /= 20;
        aBiNormal /= 20;
    }

    public override bool IsConstant
    {
        get
        {
            if ((_curve!.GetType() == Line) && (_guide!.GetType() == Line))
            {
                double angle;
                angle = _curve.Line().AngleTo(_guide.Line());
                if ((angle < 1.0e-12) || ((2 * M_PI - angle) < 1.0e-12))
                {
                    return true;
                }
            }

            return false;
        }
    }

    /// <summary>Returns false.</summary>
    public override bool IsOnlyBy3dCurve => false;

    /// <summary>Does nothing.</summary>
    public override void Origine(double orACR1, double orACR2)
    {
    }

    /// <summary>Interpolates a starting value for the root search.</summary>
    private void InitX(double param)
    {
        int ideb = 1, ifin = _pole.RowLength(), idemi;
        double valeur, t1, t2;

        valeur = _pole.Value(1, ideb).X;
        if (param == valeur)
        {
            ifin = ideb + 1;
        }

        valeur = _pole.Value(1, ifin).X;
        if (param == valeur)
        {
            ideb = ifin - 1;
        }

        while (ideb + 1 != ifin)
        {
            idemi = (ideb + ifin) / 2;
            valeur = _pole.Value(1, idemi).X;
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

        t1 = _pole.Value(1, ideb).X;
        t2 = _pole.Value(1, ifin).X;
        double diff = t2 - t1;
        if (diff > 1.0e-7)
        {
            double b = (param - t1) / diff, a = (t2 - param) / diff;
            _x[1] = _pole.Value(1, ideb).Coord(2) * a + _pole.Value(1, ifin).Coord(2) * b; // param guide
        }
        else
        {
            _x[1] = (_pole.Value(1, ideb).Coord(2) + _pole.Value(1, ifin).Coord(2)) / 2;
        }

        if (_guide!.IsPeriodic)
        {
            _x[1] = global::Kerf.Engine.Geometry.ElCLib.InPeriod(_x[1], _guide.FirstParameter(), _guide.LastParameter());
        }
    }
}
