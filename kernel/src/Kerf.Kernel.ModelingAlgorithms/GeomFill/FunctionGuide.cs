// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_FunctionGuide.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_FunctionGuide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The function G(w) - S(teta, v) of a sweep with a guide, G the guide and S the surface of revolution of the
/// section about the current axis, with its derivatives.
/// </summary>
/// <remarks>
/// A working object: <see cref="SetParam"/> rebuilds the surface of revolution. It keeps the caller's section law
/// and guide, and only evaluates them; the section it revolves is a copy or a curve it builds.
/// OCCT declares <c>Deriv2T</c> but its definition is commented out; it is not ported.
/// </remarks>
internal sealed class FunctionGuide : FunctionSetWithDerivatives
{
    private readonly CurveAdaptorBase _theGuide;
    private readonly SurfaceSectionLaw _theLaw;
    private readonly bool _isconst;
    private Curve3D? _theCurve;
    private readonly Curve3D? _theConst;
    private Surface? _theSurface;
    private readonly double _first;
    private readonly double _last;
    private readonly double _theUonS;
    private Xyz _centre;
    private Xyz _dir;

    /// <summary>Initialises the section and the guide.</summary>
    public FunctionGuide(SurfaceSectionLaw s, CurveAdaptorBase guide, double paramOnLaw = 0.0)
    {
        _theGuide = guide;
        _theLaw = s;
        _isconst = false;
        _first = 0.0;
        _last = 0.0;
        _theUonS = paramOnLaw;

        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (_theLaw.IsConstant(ref tol))
        {
            _isconst = true;
            _theConst = _theLaw.ConstantSection();
            _first = _theConst.FirstParameter;
            _last = _theConst.LastParameter;
        }
        else
        {
            _isconst = false;
            _theConst = null;
        }
        _theCurve = null;
    }

    /// <summary>Builds the surface of revolution of the section about the axis (C, D), with DX as X direction.</summary>
    public void SetParam(double param, in Point3D c, in Xyz d, in Xyz dx)
    {
        _centre = c.XYZ();
        _dir = d;

        // repere fixe
        CoordinateSystem rep = new CoordinateSystem(Constants.Origin(), Constants.DZ(), Constants.DX());

        // calculer transfo entre triedre et Oxyz
        Direction3D b2 = new Direction3D(dx);
        CoordinateSystem repTriedre = new CoordinateSystem(c, new Direction3D(d), b2);
        Transform3D transfo = new Transform3D().WithTransformation(repTriedre, rep);

        if (_isconst)
        {
            _theCurve = new TrimmedCurve3D((Curve3D)_theConst!.Copy(), _first, _last);
        }
        else
        {
            _theLaw.SectionShape(out int nbPoles, out int nbKnots, out int deg);
            NCollection_Array1<int> mult = new(1, nbKnots);
            _theLaw.Mults(mult);
            NCollection_Array1<double> knots = new(1, nbKnots);
            _theLaw.Knots(knots);
            NCollection_Array1<Point3D> poles = new(1, nbPoles);
            NCollection_Array1<double> weights = new(1, nbPoles);
            _theLaw.D0(_theUonS, poles, weights);
            if (_theLaw.IsRational)
            {
                _theCurve = new BSplineCurve3D(poles, weights, knots, mult, deg, _theLaw.IsUPeriodic);
            }
            else
            {
                _theCurve = new BSplineCurve3D(poles, knots, mult, deg, _theLaw.IsUPeriodic);
            }
        }

        Axis3D axe = new Axis3D(c, new Direction3D(_dir));
        _theCurve.Transform(transfo);
        _theSurface = new RevolutionSurface(_theCurve, axe);
    }

    /// <summary>3: (w, u, v).</summary>
    public override int NbVariables() => 3;

    public override int NbEquations() => 3;

    public override bool Value(math_Vector x, math_Vector f)
    {
        _theGuide.D0(x[1], out Point3D p);
        _theSurface!.D0(x[2], x[3], out Point3D p1);

        f[1] = p.Coord(1) - p1.Coord(1);
        f[2] = p.Coord(2) - p1.Coord(2);
        f[3] = p.Coord(3) - p1.Coord(3);

        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        _theGuide.D1(x[1], out _, out Vector3D dp);
        _theSurface!.D1(x[2], x[3], out _, out Vector3D dp1U, out Vector3D dp1V);

        int i;
        for (i = 1; i <= 3; i++)
        {
            d[i, 1] = dp.Coord(i);
            d[i, 2] = -dp1U.Coord(i);
            d[i, 3] = -dp1V.Coord(i);
        } // for

        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        _theGuide.D1(x[1], out Point3D p, out Vector3D dp);                              // derivee de la generatrice
        _theSurface!.D1(x[2], x[3], out Point3D p1, out Vector3D dp1U, out Vector3D dp1V); // derivee de la new surface

        int i;
        for (i = 1; i <= 3; i++)
        {
            f[i] = p.Coord(i) - p1.Coord(i);

            d[i, 1] = dp.Coord(i);
            d[i, 2] = -dp1U.Coord(i);
            d[i, 3] = -dp1V.Coord(i);
        } // for

        return true;
    }

    /// <summary>The first derivative in t.</summary>
    public bool DerivT(math_Vector x, in Xyz dCentre, in Xyz dDir, math_Vector dfdt)
    {
        Vector3D ds = new Vector3D();
        DSDT(x[2], x[3], dCentre, dDir, ref ds);

        _theCurve!.D0(x[1], out Point3D p);

        dfdt[1] = p.Coord(1) - ds.Coord(1);
        dfdt[2] = p.Coord(2) - ds.Coord(2);
        dfdt[3] = p.Coord(3) - ds.Coord(3);

        return true;
    }

    // The derivative of the surface in t at (U, V).
    private void DSDT(double u, double v, in Xyz dc, in Xyz dDir, ref Vector3D ds)
    {
        // C origine sur l'axe de revolution
        // Vdir vecteur unitaire definissant la direction de l'axe de revolution
        // Q(v) point de parametre V sur la courbe de revolution
        // OM (u,v) = OC + CQ * std::cos(U) + (CQ.Vdir)(1-std::cos(U)) * Vdir +
        //            (Vdir^CQ)* std::sin(U)

        _theCurve!.D0(v, out Point3D pc); // Q(v)
        // OCCT's Q is a reference into Pc, which is not read afterwards: a copy here.
        Xyz q = pc.XYZ(), dq = new Xyz(0, 0, 0); // Q
        // OCCT prints "Not implemented" to std::cout here when the section is not constant; the port prints nothing.

        q.Subtract(_centre); // CQ
        dq -= dc;

        Xyz dVcrossCQ = new Xyz();
        dVcrossCQ.SetLinearForm(dDir.Crossed(q), _dir.Crossed(dq)); // Vdir^CQ
        dVcrossCQ.Multiply(Math.Sin(u));                           //(Vdir^CQ)*Sin(U)

        double cosU = Math.Cos(u);
        Xyz dVdotCQ = new Xyz();
        dVdotCQ.SetLinearForm(dDir.Dot(q) + _dir.Dot(dq), _dir, _dir.Dot(q), dDir); //(CQ.Vdir)(1-std::cos(U))Vdir
        dVdotCQ.Add(dVcrossCQ);                                                    // addition des composantes

        dq.Multiply(cosU);
        dq.Add(dVdotCQ);
        dq.Add(dc);
        ds = ds.WithXYZ(dq);
    }
}
