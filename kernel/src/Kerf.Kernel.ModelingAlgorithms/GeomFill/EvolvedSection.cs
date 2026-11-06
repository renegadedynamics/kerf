// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_EvolvedSection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_EvolvedSection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>A section law that scales one section by a law: section(v) = Law(v) * C, scaled about the origin.</summary>
/// <remarks>
/// A working object (decision 21), as <see cref="UniformSection"/>: it keeps a copy of the section and, when
/// the section is a B-spline curve, the caller's curve itself (read only, as OCCT keeps the handle), and a trimmed
/// copy of the law (<see cref="LawsFunction.Trim"/>). <see cref="ConstantSection"/> builds new, unfrozen geometry.
/// </remarks>
internal class EvolvedSection : SurfaceSectionLaw
{
    private readonly double _first;
    private readonly double _last;
    private readonly Curve3D _section;
    private readonly LawsFunction _law;
    private LawsFunction _tLaw;
    private readonly BSplineCurve3D _curve;

    /// <summary>The section C scaled by the law L, over L's bounds.</summary>
    public EvolvedSection(Curve3D c, LawsFunction l)
    {
        l.Bounds(out _first, out _last);
        _section = (Curve3D)c.Copy();
        _law = l.Trim(_first, _last, 1.0e-20);
        _tLaw = _law;
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

    public override bool D0(double u, NCollection_Array1<Point3D> poles, NCollection_Array1<double> weights)
    {
        double val;
        int ii, l = poles.Length();
        val = _tLaw.Value(u);
        poles.Assign(_curve.InternalPoles());
        for (ii = 1; ii <= l; ii++)
        {
            poles[ii] = new Point3D(poles[ii].XYZ() * val);
        }
        weights.Assign(_curve.InternalWeightsArray());

        return true;
    }

    public override bool D1(double u, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                            NCollection_Array1<double> weights, NCollection_Array1<double> dWeights)
    {
        double val, dval;
        int ii, l = poles.Length();
        _tLaw.D1(u, out val, out dval);

        poles.Assign(_curve.InternalPoles());
        weights.Assign(_curve.InternalWeightsArray());
        for (ii = 1; ii <= l; ii++)
        {
            dPoles[ii] = dPoles[ii].WithXYZ(poles[ii].XYZ());
            dPoles[ii] *= dval;
            poles[ii] = new Point3D(poles[ii].XYZ() * val);
        }
        dWeights.Init(0);

        return true;
    }

    public override bool D2(double u, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                            NCollection_Array1<Vector3D> d2Poles, NCollection_Array1<double> weights,
                            NCollection_Array1<double> dWeights, NCollection_Array1<double> d2Weights)
    {
        double val, dval, d2val;
        int ii, l = poles.Length();
        _tLaw.D2(u, out val, out dval, out d2val);
        poles.Assign(_curve.InternalPoles());
        weights.Assign(_curve.InternalWeightsArray());

        for (ii = 1; ii <= l; ii++)
        {
            dPoles[ii] = dPoles[ii].WithXYZ(poles[ii].XYZ());
            d2Poles[ii] = dPoles[ii];
            d2Poles[ii] *= d2val;
            dPoles[ii] *= dval;
            poles[ii] = new Point3D(poles[ii].XYZ() * val);
        }

        dWeights.Init(0);
        d2Weights.Init(0);

        return true;
    }

    /// <summary>Null: OCCT's construction of the surface is commented out.</summary>
    public override BSplineSurface? BSplineSurface() => null;

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

    public override bool IsVPeriodic => (Math.Abs(_law.Value(_first) - _law.Value(_last)) < global::Kerf.Engine.Foundation.Precision.Confusion());

    public override int NbIntervals(Continuity s) => _law.NbIntervals(s);

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        _law.Intervals(t, s);
    }

    public override void SetInterval(double f, double l)
    {
        _tLaw = _law.Trim(f, l, global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override void GetInterval(out double f, out double l)
    {
        _tLaw.Bounds(out f, out l);
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
        // OCCT: starts at the section's first parameter but steps to the B-spline's last one (as UniformSection).
        double u = _section.FirstParameter, delta, b;
        int ii;
        Point3D p;
        Point3D bary = new Point3D();

        delta = (_curve.LastParameter - u) / 20;
        bary = bary.WithCoord(0.0, 0.0, 0.0);
        for (ii = 0; ii <= 20; ii++, u += delta)
        {
            p = _curve.Value(u);
            bary = new Point3D(bary.XYZ() + p.XYZ());
        }

        u = _first;
        delta = (_last - _first) / 20;
        for (ii = 0, b = 0.0; ii <= 20; ii++, u += delta)
        {
            b += _law.Value(u);
        }
        bary = new Point3D(bary.XYZ() * (b / (21 * 21)));
        return bary;
    }

    public override double MaximalSection()
    {
        double l, val, max, u, delta;
        int ii;
        CurveAdaptor ac = new CurveAdaptor(_section);
        l = AbscissaPoint.Length(ac);

        delta = (_last - _first) / 20;
        for (ii = 0, max = 0.0, u = _first; ii <= 20; ii++, u += delta)
        {
            val = _law.Value(u);
            if (val > max)
            {
                max = val;
            }
        }
        return l * max;
    }

    public override void GetMinimalWeight(NCollection_Array1<double> weights)
    {
        weights.Assign(_curve.InternalWeightsArray());
    }

    public override bool IsConstant(ref double error)
    {
        //  double isconst = false;
        bool isconst = false;
        error = 0.0;
        return isconst;
    }

    /// <summary>Throws StdFail_NotDone: the law is never constant (<see cref="IsConstant"/>).</summary>
    public override Curve3D ConstantSection()
    {
        double err = 0, scale;
        if (!IsConstant(ref err))
        {
            throw new NotDoneException("The Law is not Constant!");
        }
        Transform3D t = new Transform3D();
        Point3D p = new Point3D(0, 0, 0);
        scale = _law.Value(_first) + _law.Value((_first + _last) / 2) + _law.Value(_last);
        t = t.WithScale(p, scale / 3);

        Curve3D c;
        c = (Curve3D)_section.Copy();
        c.Transform(t);
        return c;
    }
}
