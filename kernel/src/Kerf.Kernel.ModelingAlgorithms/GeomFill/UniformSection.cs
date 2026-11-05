// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_UniformSection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_UniformSection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>A constant section law: the same B-spline section for every v in [First, Last].</summary>
/// <remarks>
/// A working object (decision 21). It keeps a copy of the section and, when the section is a B-spline curve, the
/// caller's curve itself, as OCCT keeps the handle (it only reads it). <see cref="ConstantSection"/> and
/// <see cref="BSplineSurface"/> build new, unfrozen geometry.
/// </remarks>
internal class UniformSection : SurfaceSectionLaw
{
    private readonly Curve3D _section;
    private readonly BSplineCurve3D _curve;
    private readonly double _first;
    private readonly double _last;

    /// <summary>A constant section law of the section C, between the law's parameters FirstParameter and LastParameter.</summary>
    public UniformSection(Curve3D c, double firstParameter = 0.0, double lastParameter = 1.0)
    {
        _first = firstParameter;
        _last = lastParameter;
        _section = (Curve3D)c.Copy();
        BSplineCurve3D? curve = c as BSplineCurve3D;
        if (curve is null)
        {
            curve = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(c, QuasiAngular);
            if (curve.IsPeriodic)
            {
                int m = curve.Degree / 2 + 1;
                curve.RemoveKnot(1, m, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }
        _curve = curve;
    }

    public override bool D0(double param, NCollection_Array1<Point3D> poles, NCollection_Array1<double> weights)
    {
        poles.Assign(_curve.InternalPoles());
        weights.Assign(_curve.InternalWeightsArray());
        return true;
    }

    public override bool D1(double param, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                            NCollection_Array1<double> weights, NCollection_Array1<double> dWeights)
    {
        poles.Assign(_curve.InternalPoles());
        weights.Assign(_curve.InternalWeightsArray());
        Vector3D v0 = new Vector3D(0, 0, 0);
        dPoles.Init(v0);
        dWeights.Init(0);
        return true;
    }

    public override bool D2(double param, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                            NCollection_Array1<Vector3D> d2Poles, NCollection_Array1<double> weights,
                            NCollection_Array1<double> dWeights, NCollection_Array1<double> d2Weights)
    {
        poles.Assign(_curve.InternalPoles());
        weights.Assign(_curve.InternalWeightsArray());
        Vector3D v0 = new Vector3D(0, 0, 0);
        dPoles.Init(v0);
        dWeights.Init(0);
        d2Poles.Init(v0);
        d2Weights.Init(0);
        return true;
    }

    /// <summary>The surface swept by the section: a new, unfrozen surface of degree 1 in v.</summary>
    public override BSplineSurface? BSplineSurface()
    {
        int ii, nbPoles = _curve.PoleCount;
        NCollection_Array2<Point3D> poles = new(1, nbPoles, 1, 2);
        NCollection_Array1<double> curKnots = _curve.InternalKnots();
        NCollection_Array1<double> uKnots = new(curKnots);
        NCollection_Array1<double> vKnots = new(1, 2);
        NCollection_Array1<int> curMults = _curve.InternalMultiplicities();
        NCollection_Array1<int> uMults = new(curMults);
        NCollection_Array1<int> vMults = new(1, 2);

        for (ii = 1; ii <= nbPoles; ii++)
        {
            poles[ii, 1] = poles[ii, 2] = _curve.Pole(ii);
        }

        vKnots[1] = _first;
        vKnots[2] = _last;

        vMults.Init(2);

        BSplineSurface bs = new BSplineSurface(poles, uKnots, vKnots, uMults, vMults, _curve.Degree, 1,
                                                         _curve.IsPeriodic);

        return bs;
    }

    public override void SectionShape(out int nbPoles, out int nbKnots, out int degree)
    {
        nbPoles = _curve.PoleCount;
        nbKnots = _curve.KnotCount;
        degree = _curve.Degree;
    }

    public override void Knots(NCollection_Array1<double> tKnots)
    {
        tKnots.Assign(_curve.InternalKnots());
    }

    public override void Mults(NCollection_Array1<int> tMults)
    {
        tMults.Assign(_curve.InternalMultiplicities());
    }

    public override bool IsRational => _curve.IsRational;

    public override bool IsUPeriodic => _curve.IsPeriodic;

    public override bool IsVPeriodic => true;

    public override int NbIntervals(Continuity s) => 1;

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        t[t.Lower()] = _first;
        t[t.Upper()] = _last;
    }

    public override void SetInterval(double first, double last)
    {
        // Ne fait Rien
    }

    public override void GetInterval(out double f, out double l)
    {
        f = _first;
        l = _last;
    }

    public override void GetDomain(out double f, out double l)
    {
        f = _first;
        l = _last;
    }

    public override void GetTolerance(double boundTol, double surfTol, double angleTol, NCollection_Array1<double> tol3d)
    {
        tol3d.Init(surfTol);
        if (boundTol < surfTol)
        {
            tol3d[tol3d.Lower()] = boundTol;
            tol3d[tol3d.Upper()] = boundTol;
        }
    }

    public override Point3D BarycentreOfSurf()
    {
        // OCCT: starts at the section's first parameter but steps to the B-spline's last one.
        double u = _section.FirstParameter, delta;
        Point3D p;
        Point3D bary = new Point3D();

        delta = (_curve.LastParameter - u) / 20;
        bary = bary.WithCoord(0.0, 0.0, 0.0);
        for (int ii = 0; ii <= 20; ii++, u += delta)
        {
            p = _curve.Value(u);
            bary = new Point3D(bary.XYZ() + p.XYZ());
        }

        bary = new Point3D(bary.XYZ() / 21.0);
        return bary;
    }

    public override double MaximalSection()
    {
        CurveAdaptor ac = new CurveAdaptor(_section);
        return AbscissaPoint.Length(ac);
    }

    public override void GetMinimalWeight(NCollection_Array1<double> weights)
    {
        weights.Assign(_curve.InternalWeightsArray());
    }

    public override bool IsConstant(ref double error)
    {
        error = 0.0;
        return true;
    }

    /// <summary>A new copy of the section.</summary>
    public override Curve3D ConstantSection()
    {
        Curve3D c;
        c = (Curve3D)_section.Copy();
        return c;
    }
}
