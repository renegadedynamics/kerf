// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_RstRstEvolRad.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_RstRstEvolRad.cxx
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
using static Kerf.Engine.Fillets.Blend.DecrochStatus;
using static Kerf.Engine.Fillets.BlendFunctions.SectionShape;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The function of an evolving-radius fillet (the radius a law of the guide's parameter) between two restrictions (arcs of two surfaces) along a guide: the
/// variables are the parameters on the two restrictions.
/// </summary>
/// <remarks>A working object (decision 21): it caches its last solution, which the walkers read.</remarks>
public class RstRstEvolRad : RstRstFunction
{
    private const double Eps = 1.0e-15;

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
    }

    public override int NbVariables()
    {
        return 2;
    }

    public override int NbEquations()
    {
        return 2;
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        _ptrst1 = _cons1.Value(x[1]);
        _ptrst2 = _cons2.Value(x[2]);

        f[1] = _nplan.XYZ().Dot(_ptrst1.XYZ()) + _d;
        f[2] = _nplan.XYZ().Dot(_ptrst2.XYZ()) + _d;

        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        Vector3D d11, d21;

        _cons1.D1(x[1], out _ptrst1, out d11);
        _cons2.D1(x[2], out _ptrst2, out d21);

        d[1, 1] = _nplan.Dot(d11);
        d[1, 2] = 0.0;

        d[2, 1] = 0.0;
        d[2, 2] = _nplan.Dot(d21);

        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        Value(x, f);
        Derivatives(x, d);

        return true;
    }

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
        _normtg = _d1gui.Magnitude();
        _nplan.SetXYZ(_d1gui.Normalized().XYZ());
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

        Vector3D dnplan = new(), d1urst1, d1vrst1, d1urst2, d1vrst2, d11, d21, temp = new();
        Point3D bid;

        double cosa, sina, angle;

        Values(sol, valsol, gradsol);

        if (Math.Abs(valsol[1]) <= tol && Math.Abs(valsol[2]) <= tol)
        {
            // Calculation of tangents
            _prmrst1 = sol[1];
            _pt2drst1 = _rst1.Value(_prmrst1);
            _prmrst2 = sol[2];
            _pt2drst2 = _rst2.Value(_prmrst2);

            _cons1.D1(sol[1], out _ptrst1, out d11);
            _cons2.D1(sol[2], out _ptrst2, out d21);

            dnplan.SetLinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

            temp.SetXYZ(_ptrst1.XYZ() - _ptgui.XYZ());
            secmember[1] = _normtg - dnplan.Dot(temp);

            temp.SetXYZ(_ptrst2.XYZ() - _ptgui.XYZ());
            secmember[2] = _normtg - dnplan.Dot(temp);

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
                    // OCCT: DEDT has 3 entries for this 2x2 system; native's release build (its dimension checks
                    // compiled out) copies secmember's 2 and solves on them, as here.
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
                _tgrst1 = secmember[1] * d11;
                _tgrst2 = secmember[2] * d21;

                double a, b;
                _surf1.D1(_pt2drst1.X(), _pt2drst1.Y(), out bid, out d1urst1, out d1vrst1);
                T3dto2d(out a, out b, _tgrst1, d1urst1, d1vrst1);
                _tg2drst1.SetCoord(a, b);
                _surf2.D1(_pt2drst2.X(), _pt2drst2.Y(), out bid, out d1urst2, out d1vrst2);
                T3dto2d(out a, out b, _tgrst1, d1urst2, d1vrst2); // OCCT: tgrst1 where tgrst2 is meant
                _tg2drst2.SetCoord(a, b);
            }

            Point3D center = new();
            Vector3D notUsed;
            bool isCenter;

            isCenter = CenterCircleRst1Rst2(_ptrst1, _ptrst2, _nplan, ref center, out notUsed);

            if (!isCenter)
            {
                return false;
            }

            Vector3D n1 = new(center, _ptrst1), n2 = new(center, _ptrst2);

            n1.Normalize();
            n2.Normalize();

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

            if (angle > _maxang)
            {
                _maxang = angle;
            }
            if (angle < _minang)
            {
                _minang = angle;
            }
            _distmin = Min(_distmin, _ptrst1.Distance(_ptrst2));

            return true;
        }
        _istangent = true;
        return false;
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
        Vector3D nRst1InPlane = new(), nRst2InPlane = new();
        Point3D ptTmp1, ptTmp2, center = new();
        Vector3D d1u, d1v, centptrst = new(), notUsed;
        double norm, unsurnorm;
        double u, v;

        _rstref1!.Value(sol[1]).Coord(out u, out v);
        _surfref1!.D1(u, v, out ptTmp1, out d1u, out d1v);
        // Normal to the reference surface 1
        nRst1 = d1u.Crossed(d1v);
        _rstref2!.Value(sol[2]).Coord(out u, out v);
        _surfref2!.D1(u, v, out ptTmp2, out d1u, out d1v);
        // Normal to the reference surface 2
        nRst2 = d1u.Crossed(d1v);

        CenterCircleRst1Rst2(ptTmp1, ptTmp2, _nplan, ref center, out notUsed);

        norm = _nplan.Crossed(nRst1).Magnitude();
        unsurnorm = 1.0 / norm;

        nRst1InPlane.SetLinearForm(_nplan.Dot(nRst1) * unsurnorm, _nplan, -unsurnorm, nRst1);

        centptrst.SetXYZ(ptTmp1.XYZ() - center.XYZ());

        if (centptrst.Dot(nRst1InPlane) < 0.0)
        {
            nRst1InPlane.Reverse();
        }

        tgRst1 = _nplan.Crossed(centptrst);

        norm = _nplan.Crossed(nRst2).Magnitude();
        unsurnorm = 1.0 / norm;
        nRst2InPlane.SetLinearForm(_nplan.Dot(nRst2) * unsurnorm, _nplan, -unsurnorm, nRst2);
        centptrst.SetXYZ(ptTmp2.XYZ() - center.XYZ());

        if (centptrst.Dot(nRst2InPlane) < 0.0)
        {
            nRst2InPlane.Reverse();
        }

        tgRst2 = _nplan.Crossed(centptrst);

        if (_choix % 2 != 0)
        {
            tgRst1.Reverse();
            tgRst2.Reverse();
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
        norm2 = rst1rst2.SquareMagnitude();
        dist = _ray * _ray - 0.25 * norm2;

        if (_choix > 2)
        {
            vdMed.Reverse();
        }

        if (dist < -1.0E-07)
        {
            return false;
        }

        if (dist > 1.0E-07)
        {
            dist = Math.Sqrt(dist);
            vdmedNor = vdMed.Normalized();
            center.SetXYZ(0.5 * rst1rst2.XYZ() + ptRst1.XYZ() + dist * vdmedNor.XYZ());
        }
        else
        {
            center.SetXYZ(0.5 * rst1rst2.XYZ() + ptRst1.XYZ());
        }

        return true;
    }

    /// <summary>The circular section at <paramref name="param"/> through the points U and V, and its bounds.</summary>
    public void Section(double param, double u, double v, out double pdeb, out double pfin, ref Circle3D c)
    {
        Vector3D ns, np, notUsed;
        Point3D center = new();

        _tguide.D1(param, out _ptgui, out _d1gui);
        np = _d1gui.Normalized();
        _ray = _tevol.Value(param);
        _ptrst1 = _cons1.Value(u);
        _ptrst2 = _cons2.Value(v);

        CenterCircleRst1Rst2(_ptrst1, _ptrst2, np, ref center, out notUsed);

        c.SetRadius(Math.Abs(_ray));
        ns = new Vector3D(center, _ptrst1).Normalized();

        if (_choix % 2 != 0)
        {
            np.Reverse();
        }

        c.SetPosition(new Frame3D(center, new Direction3D(np), new Direction3D(ns)));
        pdeb = 0; // ElCLib::Parameter(C, pts);
        pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, _ptrst2);

        // Test of angles negative and almost null : Special Case
        if (pfin > 1.5 * M_PI)
        {
            np.Reverse();
            c.SetPosition(new Frame3D(center, new Direction3D(np), new Direction3D(ns)));
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
        Vector3D n1, n2, notUsed;
        Point3D center = new();
        double u, v;

        double prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();

        _tguide.D1(prm, out _ptgui, out _d1gui);
        _ray = _tevol.Value(prm);
        _nplan = _d1gui.Normalized();

        u = p.ParameterOnC1();
        v = p.ParameterOnC2();

        Point2D pt2d1 = _rst1.Value(u);
        Point2D pt2d2 = _rst2.Value(v);

        _ptrst1 = _cons1.Value(u);
        _ptrst2 = _cons2.Value(v);
        _distmin = Min(_distmin, _ptrst1.Distance(_ptrst2));

        poles2d[poles2d.Lower()].SetCoord(pt2d1.X(), pt2d1.Y());
        poles2d[poles2d.Upper()].SetCoord(pt2d2.X(), pt2d2.Y());

        // Linear case
        if (_sShape == SectionShape.Linear)
        {
            poles[low] = _ptrst1;
            poles[upp] = _ptrst2;
            weights[low] = 1.0;
            weights[upp] = 1.0;
            return;
        }

        // Calculate the center of the circle
        CenterCircleRst1Rst2(_ptrst1, _ptrst2, _nplan, ref center, out notUsed);

        // normals to the section with points
        n1 = new Vector3D(center, _ptrst1).Normalized();
        n2 = new Vector3D(center, _ptrst2).Normalized();

        if (_choix % 2 != 0)
        {
            _nplan.Reverse();
        }

        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, n1, n2, _nplan, _ptrst1, _ptrst2, Math.Abs(_ray), center, poles, weights);
    }

    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                                 NCollection_Array1<double> weights, NCollection_Array1<double> dWeights)
    {
        Vector3D d11, d21;
        Vector3D dnplan = new(), d1n1 = new(), d1n2 = new();
        Vector3D temp = new(), tgct = new();
        Vector3D d1urst, d1vrst;
        Point3D center = new(), notUsed;

        double norm2, normmed, dist;

        math_Vector sol = new(1, 2), valsol = new(1, 2), secmember = new(1, 2);
        Matrix gradsol = new(1, 2, 1, 2);

        double prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();
        bool istgt;

        _tguide.D2(prm, out _ptgui, out _d1gui, out _d2gui);
        _normtg = _d1gui.Magnitude();
        _nplan = _d1gui.Normalized();
        dnplan.SetLinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);
        _tevol.D1(prm, out _ray, out _dray);

        sol[1] = _prmrst1 = p.ParameterOnC1();
        sol[2] = _prmrst2 = p.ParameterOnC2();
        _pt2drst1 = _rst1.Value(_prmrst1);
        _pt2drst2 = _rst2.Value(_prmrst2);

        Values(sol, valsol, gradsol);

        _cons1.D1(sol[1], out _ptrst1, out d11);
        _cons2.D1(sol[2], out _ptrst2, out d21);

        temp.SetXYZ(_ptrst1.XYZ() - _ptgui.XYZ());
        secmember[1] = _normtg - dnplan.Dot(temp);

        temp.SetXYZ(_ptrst2.XYZ() - _ptgui.XYZ());
        secmember[2] = _normtg - dnplan.Dot(temp);

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

        Vector3D med;
        Vector3D rst1rst2 = new(_ptrst1, _ptrst2);
        bool isCenter;

        isCenter = CenterCircleRst1Rst2(_ptrst1, _ptrst2, _nplan, ref center, out med);
        if (!isCenter)
        {
            return false;
        }

        normmed = med.Magnitude();
        med.Normalize();
        Vector3D n1 = new(center, _ptrst1), n2 = new(center, _ptrst2);

        if (!istgt)
        {
            // secmember contains derivatives of parameters on curves compared to t
            _tgrst1 = secmember[1] * d11;
            _tgrst2 = secmember[2] * d21;

            Vector3D d1rst1rst2;

            norm2 = rst1rst2.SquareMagnitude();
            d1rst1rst2 = _tgrst2 - _tgrst1;
            dist = _ray * _ray - 0.25 * norm2;
            double invdray = _dray / _ray;

            if (dist > 1.0E-07)
            {
                Vector3D d1P1P2CrosNp, dmed;
                d1P1P2CrosNp = d1rst1rst2.Crossed(_nplan) + rst1rst2.Crossed(dnplan);
                // derivative of the perpendicular bisector
                dmed = d1P1P2CrosNp - med.Dot(d1P1P2CrosNp) * med;
                dmed /= normmed;
                dist = Math.Sqrt(dist);
                double d1Dist;

                d1Dist = (_ray * _dray - 0.25 * rst1rst2.Dot(d1rst1rst2)) / dist;

                if (_choix > 2)
                {
                    dmed.Reverse();
                }

                // the derivative of coefficient Dist is located in dmed
                dmed.SetLinearForm(dist, dmed, d1Dist, med);
                d1rst1rst2 *= 0.5;
                // derivative of the Normal to the curve in P1
                d1n1 = -(d1rst1rst2 + dmed + invdray * n1) / _ray;

                // derivative of the Normal to the curve in P2
                d1n2 = (d1rst1rst2 - dmed - invdray * n2) / _ray;
            }
            else
            {
                d1rst1rst2 *= 0.5;
                // Normal to the curve in P1
                d1n1 = -(d1rst1rst2 + invdray * n1) / _ray;

                // Normal to the curve in P2
                d1n2 = (d1rst1rst2 - invdray * n2) / _ray;
            }
        }

        n1.Normalize();
        n2.Normalize();

        // Tops 2d

        poles2d[poles2d.Lower()].SetCoord(_pt2drst1.X(), _pt2drst1.Y());
        poles2d[poles2d.Upper()].SetCoord(_pt2drst2.X(), _pt2drst2.Y());
        if (!istgt)
        {
            double a, b;
            _surf1.D1(_pt2drst1.X(), _pt2drst1.Y(), out notUsed, out d1urst, out d1vrst);
            T3dto2d(out a, out b, _tgrst1, d1urst, d1vrst);
            dPoles2d[poles2d.Lower()].SetCoord(a, b);

            _surf2.D1(_pt2drst2.X(), _pt2drst2.Y(), out notUsed, out d1urst, out d1vrst);
            T3dto2d(out a, out b, _tgrst2, d1urst, d1vrst);
            dPoles2d[poles2d.Upper()].SetCoord(a, b);
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

        // Case of the circle
        // tangent to the center of the circle
        if (!istgt)
        {
            tgct.SetLinearForm(-_ray, d1n1, -_dray, n1, _tgrst1);
        }

        if (_choix % 2 != 0)
        {
            _nplan.Reverse();
            dnplan.Reverse();
        }

        if (!istgt)
        {
            return global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, n1, n2, d1n1, d1n2, _nplan, dnplan, _ptrst1, _ptrst2, _tgrst1, _tgrst2,
                                      Math.Abs(_ray), _dray, center, tgct, poles, dPoles, weights, dWeights);
        }
        else
        {
            global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, n1, n2, _nplan, _ptrst1, _ptrst2, Math.Abs(_ray), center, poles, weights);
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
