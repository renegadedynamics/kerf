// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SweepFunction.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SweepFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The function swept by <see cref="SurfaceSweep"/>: a section law placed by a location law, as the
/// <see cref="SweepFunction"/> that <see cref="SweepApproximation"/> approximates.
/// </summary>
/// <remarks>
/// A working object (decision 21): it keeps the caller's laws, as OCCT keeps the handles, and
/// <see cref="SetInterval"/> and <see cref="SetTolerance"/> change them.
/// </remarks>
internal class SurfaceSweepFunction : SweepFunction
{
    private readonly SurfaceLocationLaw _loc;
    private readonly SurfaceSectionLaw _sec;
    private readonly double _myf;
    private readonly double _myfOnS;
    private readonly double _ratio;
    private Matrix3x3 _m = new Matrix3x3();
    private Matrix3x3 _dm = new Matrix3x3();
    private Matrix3x3 _d2M = new Matrix3x3();
    private Vector3D _v = new Vector3D();
    private Vector3D _dv = new Vector3D();
    private Vector3D _d2V = new Vector3D();

    /// <summary>
    /// The sweep of Section along Location: the section law is evaluated at
    /// FirstParameterOnS + (v - FirstParameter) * RatioParameterOnS.
    /// </summary>
    public SurfaceSweepFunction(SurfaceSectionLaw section,
                                  SurfaceLocationLaw location,
                                  double firstParameter,
                                  double firstParameterOnS,
                                  double ratioParameterOnS)
    {
        _loc = location;
        _sec = section;
        _myf = firstParameter;
        _myfOnS = firstParameterOnS;
        _ratio = ratioParameterOnS;
    }

    public override bool D0(double param,
                            double first,
                            double last,
                            NCollection_Array1<Point3D> poles,
                            NCollection_Array1<Point2D> poles2d,
                            NCollection_Array1<double> weigths)
    {
        int ii, l;
        bool ok;
        double t = _myfOnS + (param - _myf) * _ratio;
        l = poles.Length();

        ok = _loc.D0(param, ref _m, ref _v, poles2d);
        if (!ok)
        {
            return ok;
        }
        ok = _sec.D0(t, poles, weigths);
        if (!ok)
        {
            return ok;
        }

        for (ii = 1; ii <= l; ii++)
        {
            Xyz aux = poles[ii].XYZ();
            aux *= _m;
            aux += _v.XYZ();
            poles[ii] = new Point3D(aux);
        }
        return true;
    }

    public override bool D1(double param,
                            double first,
                            double last,
                            NCollection_Array1<Point3D> poles,
                            NCollection_Array1<Vector3D> dPoles,
                            NCollection_Array1<Point2D> poles2d,
                            NCollection_Array1<Vector2D> dPoles2d,
                            NCollection_Array1<double> weigths,
                            NCollection_Array1<double> dWeigths)
    {
        int ii, l;
        bool ok;
        double t = _myfOnS + (param - _myf) * _ratio;
        Xyz pPrim;
        l = poles.Length();

        ok = _loc.D1(param, ref _m, ref _v, ref _dm, ref _dv, poles2d, dPoles2d);
        if (!ok)
        {
            return ok;
        }
        ok = _sec.D1(t, poles, dPoles, weigths, dWeigths);
        if (!ok)
        {
            return ok;
        }

        for (ii = 1; ii <= l; ii++)
        {
            pPrim = dPoles[ii].XYZ();
            Xyz p = poles[ii].XYZ();
            pPrim *= _ratio;
            dWeigths[ii] *= _ratio;
            pPrim *= _m;
            pPrim += _dm * p;
            pPrim += _dv.XYZ();
            dPoles[ii] = dPoles[ii].WithXYZ(pPrim);

            p *= _m;
            p += _v.XYZ();
            poles[ii] = new Point3D(p);
        }
        return true;
    }

    public override bool D2(double param,
                            double first,
                            double last,
                            NCollection_Array1<Point3D> poles,
                            NCollection_Array1<Vector3D> dPoles,
                            NCollection_Array1<Vector3D> d2Poles,
                            NCollection_Array1<Point2D> poles2d,
                            NCollection_Array1<Vector2D> dPoles2d,
                            NCollection_Array1<Vector2D> d2Poles2d,
                            NCollection_Array1<double> weigths,
                            NCollection_Array1<double> dWeigths,
                            NCollection_Array1<double> d2Weigths)
    {
        int ii, l;
        bool ok;
        double t = _myfOnS + (param - _myf) * _ratio;
        double squareratio = _ratio * _ratio;
        l = poles.Length();

        ok = _loc.D2(param, ref _m, ref _v, ref _dm, ref _dv, ref _d2M, ref _d2V, poles2d, dPoles2d, d2Poles2d);
        if (!ok)
        {
            return ok;
        }
        ok = _sec.D2(t, poles, dPoles, d2Poles, weigths, dWeigths, d2Weigths);
        if (!ok)
        {
            return ok;
        }

        for (ii = 1; ii <= l; ii++)
        {
            Xyz pSecn = d2Poles[ii].XYZ();
            Xyz pPrim = dPoles[ii].XYZ();
            pPrim *= _ratio;
            dWeigths[ii] *= _ratio;
            pSecn *= squareratio;
            d2Weigths[ii] *= squareratio;
            Xyz p = poles[ii].XYZ();

            pSecn *= _m;
            pSecn += 2 * (_dm * pPrim);
            pSecn += _d2M * p;
            pSecn += _d2V.XYZ();
            d2Poles[ii] = d2Poles[ii].WithXYZ(pSecn);

            pPrim *= _m;
            pPrim += _dm * p;
            pPrim += _dv.XYZ();
            dPoles[ii] = dPoles[ii].WithXYZ(pPrim);

            p *= _m;
            p += _v.XYZ();
            poles[ii] = new Point3D(p);
        }
        return true;
    }

    public override int Nb2dCurves() => _loc.Nb2dCurves();

    public override void SectionShape(out int nbPoles, out int nbKnots, out int degree)
    {
        _sec.SectionShape(out nbPoles, out nbKnots, out degree);
    }

    public override void Knots(NCollection_Array1<double> tKnots)
    {
        _sec.Knots(tKnots);
    }

    public override void Mults(NCollection_Array1<int> tMults)
    {
        _sec.Mults(tMults);
    }

    public override bool IsRational => _sec.IsRational;

    public override int NbIntervals(Continuity s)
    {
        int Nb_Sec, Nb_Loc;
        Nb_Sec = _sec.NbIntervals(s);
        Nb_Loc = _loc.NbIntervals(s);

        if (Nb_Sec == 1)
        {
            return Nb_Loc;
        }
        else if (Nb_Loc == 1)
        {
            return Nb_Sec;
        }

        NCollection_Array1<double> intS = new(1, Nb_Sec + 1);
        NCollection_Array1<double> intL = new(1, Nb_Loc + 1);
        NCollection_Sequence<double> inter = new();
        double t;
        int ii;
        _sec.Intervals(intS, s);
        for (ii = 1; ii <= Nb_Sec + 1; ii++)
        {
            t = (intS[ii] - _myfOnS) / _ratio + _myf;
            intS[ii] = t;
        }
        _loc.Intervals(intL, s);

        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(intS, intL, inter, global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.99);
        return inter.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> T, Continuity s)
    {
        int Nb_Sec, Nb_Loc, ii;
        Nb_Sec = _sec.NbIntervals(s);
        Nb_Loc = _loc.NbIntervals(s);

        if (Nb_Sec == 1)
        {
            _loc.Intervals(T, s);
            return;
        }
        else if (Nb_Loc == 1)
        {
            double tt;
            _sec.Intervals(T, s);
            for (ii = 1; ii <= Nb_Sec + 1; ii++)
            {
                tt = (T[ii] - _myfOnS) / _ratio + _myf;
                T[ii] = tt;
            }
            return;
        }

        NCollection_Array1<double> intS = new(1, Nb_Sec + 1);
        NCollection_Array1<double> intL = new(1, Nb_Loc + 1);
        NCollection_Sequence<double> inter = new();
        double t;

        _sec.Intervals(intS, s);
        for (ii = 1; ii <= Nb_Sec + 1; ii++)
        {
            t = (intS[ii] - _myfOnS) / _ratio + _myf;
            intS[ii] = t;
        }
        _loc.Intervals(intL, s);

        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(intS, intL, inter, global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.99);
        for (ii = 1; ii <= inter.Length(); ii++)
        {
            T[ii] = inter[ii];
        }
    }

    public override void SetInterval(double first, double last)
    {
        double uf, ul;
        _loc.SetInterval(first, last);
        uf = _myf + (first - _myf) * _ratio;
        ul = _myf + (last - _myf) * _ratio;
        _sec.SetInterval(uf, ul);
    }

    public override void GetTolerance(double boundTol, double surfTol, double angleTol, NCollection_Array1<double> tol3d)
    {
        _sec.GetTolerance(boundTol, surfTol, angleTol, tol3d);
    }

    public override void Resolution(int index, double tol, out double tolU, out double tolV)
    {
        _loc.Resolution(index, tol, out tolU, out tolV);
    }

    public override void SetTolerance(double tol3d, double tol2d)
    {
        _sec.SetTolerance(tol3d, tol2d);
        _loc.SetTolerance(tol3d, tol2d);
    }

    public override Point3D BarycentreOfSurf()
    {
        Point3D bary;
        Vector3D translate = new Vector3D();
        Matrix3x3 m = new Matrix3x3();

        bary = _sec.BarycentreOfSurf();
        _loc.GetAverageLaw(ref m, ref translate);
        bary = new Point3D(bary.XYZ() * m);
        bary = new Point3D(bary.XYZ() + translate.XYZ());

        return bary;
    }

    public override double MaximalSection()
    {
        double l = _sec.MaximalSection();
        l *= _loc.GetMaximalNorm();
        return l;
    }

    public override void GetMinimalWeight(NCollection_Array1<double> weigths)
    {
        _sec.GetMinimalWeight(weigths);
    }
}
