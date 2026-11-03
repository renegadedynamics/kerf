// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CurveAndTrihedron.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CurveAndTrihedron.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A location law defined by a curve and a trihedron law: M(t) has the normal, binormal and tangent as columns,
/// and V(t) is the point of the curve.
/// </summary>
/// <remarks>
/// A working object (decision 21). The law keeps the trihedron law it is given, as OCCT keeps the handle, and
/// sets its curve and interval: hand it a law of its own (<see cref="TrihedronLaw.Copy"/>) if the caller
/// still uses it.
/// </remarks>
public class CurveAndTrihedron : SurfaceLocationLaw
{
    private bool _withTrans;
    private readonly TrihedronLaw _law;
    private CurveAdaptorBase? _curve;
    private CurveAdaptorBase? _trimmed;
    private Point3D _point;
    private Vector3D _v1;
    private Vector3D _v2;
    private Vector3D _v3;
    private Matrix3x3 _trans;

    public CurveAndTrihedron(TrihedronLaw trihedron)
    {
        _law = trihedron;
        _curve = null;
        _trans = new Matrix3x3();
        _trans = _trans.WithIdentity();
        _withTrans = false;
    }

    public override SurfaceLocationLaw Copy()
    {
        TrihedronLaw law;
        law = _law.Copy();
        CurveAndTrihedron copy = new CurveAndTrihedron(_law.Copy());
        copy.SetCurve(_curve!);
        copy.SetTrsf(_trans);
        return copy;
    }

    public override bool SetCurve(CurveAdaptorBase c)
    {
        _curve = c;
        _trimmed = c;
        return _law.SetCurve(c);
    }

    public override CurveAdaptorBase? GetCurve() => _curve;

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

    public override bool D0(double param, ref Matrix3x3 m, ref Vector3D v)
    {
        bool ok;
        _trimmed!.D0(param, out _point);
        v = v.WithXYZ(_point.XYZ());

        ok = _law.D0(param, ref _v1, ref _v2, ref _v3);
        m = m.WithCols(_v2.XYZ(), _v3.XYZ(), _v1.XYZ());

        if (_withTrans)
        {
            m *= _trans;
        }

        return ok;
    }

    public override bool D0(double param, ref Matrix3x3 m, ref Vector3D v, NCollection_Array1<Point2D> poles2d)
    {
        bool ok;
        _trimmed!.D0(param, out _point);
        v = v.WithXYZ(_point.XYZ());

        ok = _law.D0(param, ref _v1, ref _v2, ref _v3);
        m = m.WithCols(_v2.XYZ(), _v3.XYZ(), _v1.XYZ());

        if (_withTrans)
        {
            m *= _trans;
        }

        return ok;
    }

    public override bool D1(double param, ref Matrix3x3 m, ref Vector3D v, ref Matrix3x3 dm, ref Vector3D dv,
                            NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d)
    {
        bool ok;
        _trimmed!.D1(param, out _point, out dv);
        v = v.WithXYZ(_point.XYZ());

        Vector3D dv1 = new Vector3D(), dv2 = new Vector3D(), dv3 = new Vector3D();
        ok = _law.D1(param, ref _v1, ref dv1, ref _v2, ref dv2, ref _v3, ref dv3);
        m = m.WithCols(_v2.XYZ(), _v3.XYZ(), _v1.XYZ());
        dm = dm.WithCols(dv2.XYZ(), dv3.XYZ(), dv1.XYZ());

        if (_withTrans)
        {
            m *= _trans;
            dm *= _trans;
        }

        return ok;
    }

    public override bool D2(double param, ref Matrix3x3 m, ref Vector3D v, ref Matrix3x3 dm, ref Vector3D dv, ref Matrix3x3 d2M,
                            ref Vector3D d2V, NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                            NCollection_Array1<Vector2D> d2Poles2d)
    {
        bool ok;
        _trimmed!.D2(param, out _point, out dv, out d2V);
        v = v.WithXYZ(_point.XYZ());

        Vector3D dv1 = new Vector3D(), dv2 = new Vector3D(), dv3 = new Vector3D();
        Vector3D d2V1 = new Vector3D(), d2V2 = new Vector3D(), d2V3 = new Vector3D();
        ok = _law.D2(param, ref _v1, ref dv1, ref d2V1, ref _v2, ref dv2, ref d2V2, ref _v3, ref dv3, ref d2V3);

        m = m.WithCols(_v2.XYZ(), _v3.XYZ(), _v1.XYZ());
        dm = dm.WithCols(dv2.XYZ(), dv3.XYZ(), dv1.XYZ());
        d2M = d2M.WithCols(d2V2.XYZ(), d2V3.XYZ(), d2V1.XYZ());

        if (_withTrans)
        {
            m *= _trans;
            dm *= _trans;
            d2M *= _trans;
        }
        return ok;
    }

    public override int NbIntervals(Continuity s)
    {
        int Nb_Sec, Nb_Law;
        Nb_Sec = _trimmed!.NbIntervals(s);
        Nb_Law = _law.NbIntervals(s);

        if (Nb_Sec == 1)
        {
            return Nb_Law;
        }
        else if (Nb_Law == 1)
        {
            return Nb_Sec;
        }

        NCollection_Array1<double> intC = new(1, Nb_Sec + 1);
        NCollection_Array1<double> intL = new(1, Nb_Law + 1);
        NCollection_Sequence<double> inter = new();
        _trimmed.Intervals(intC, s);
        _law.Intervals(intL, s);

        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(intC, intL, inter, global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.99);
        return inter.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        int Nb_Sec, Nb_Law;
        Nb_Sec = _trimmed!.NbIntervals(s);
        Nb_Law = _law.NbIntervals(s);

        if (Nb_Sec == 1)
        {
            _law.Intervals(t, s);
            return;
        }
        else if (Nb_Law == 1)
        {
            _trimmed.Intervals(t, s);
            return;
        }

        NCollection_Array1<double> intC = new(1, Nb_Sec + 1);
        NCollection_Array1<double> intL = new(1, Nb_Law + 1);
        NCollection_Sequence<double> inter = new();
        _trimmed.Intervals(intC, s);
        _law.Intervals(intL, s);

        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(intC, intL, inter, global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.99);
        for (int ii = 1; ii <= inter.Length(); ii++)
        {
            t[ii] = inter[ii];
        }
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

    /// <summary>1: the trihedrons are normed.</summary>
    public override double GetMaximalNorm() => 1.0;

    public override void GetAverageLaw(ref Matrix3x3 am, ref Vector3D av)
    {
        int ii;
        double u, delta;
        Vector3D v = new Vector3D();

        _law.GetAverageLaw(ref _v1, ref _v2, ref _v3);
        // OCCT: the columns are (tangent, normal, binormal) here, but (normal, binormal, tangent) in D0, D1 and D2.
        am = am.WithCols(_v1.XYZ(), _v2.XYZ(), _v3.XYZ());

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

    public override bool IsTranslation(ref double error)
    {
        CurveKind type;
        error = 0;
        type = _curve!.GetType();
        if (type == Line)
        {
            return _law.IsConstant || _law.IsOnlyBy3dCurve;
        }
        return false;
    }

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
        centre = _curve!.Circle().Location();
    }
}
