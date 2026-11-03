// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SnglrFunc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SnglrFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>The function C'(t) ^ C''(t) of a curve C, as a curve (Frenet's search for singular points).</summary>
/// <remarks>A working adaptor (decision 101): it keeps the caller's curve and never changes it.</remarks>
public class SnglrFunc : CurveAdaptorBase
{
    private readonly CurveAdaptorBase _hCurve;
    private double _ratio;

    public SnglrFunc(CurveAdaptorBase hc)
    {
        _hCurve = hc;
        _ratio = 1;
    }

    /// <summary>A shallow copy of the adaptor.</summary>
    public override CurveAdaptorBase ShallowCopy()
    {
        SnglrFunc copy = new SnglrFunc(_hCurve.ShallowCopy());
        copy._ratio = _ratio;
        return copy;
    }

    public void SetRatio(double ratio)
    {
        _ratio = ratio;
    }

    public override double FirstParameter() => _hCurve.FirstParameter();

    public override double LastParameter() => _hCurve.LastParameter();

    public override int NbIntervals(Continuity s)
    {
        Continuity hcs = C0;
        if (s == C0)
        {
            hcs = C2;
        }
        else if (s == C1)
        {
            hcs = C3;
        }
        else if (s >= C2)
        {
            hcs = CN;
        }
        return _hCurve.NbIntervals(hcs);
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        Continuity hcs = C0;
        if (s == C0)
        {
            hcs = C2;
        }
        else if (s == C1)
        {
            hcs = C3;
        }
        else if (s >= C2)
        {
            hcs = CN;
        }
        _hCurve.Intervals(t, hcs);
    }

    public override bool IsPeriodic => _hCurve.IsPeriodic;

    public override double Period() => _hCurve.Period();

    public sealed override Point3D EvalD0(double u)
    {
        Curve3D.ResD2 res = _hCurve.EvalD2(u);
        return new Point3D((res.D1 * _ratio).Crossed(res.D2).XYZ());
    }

    internal sealed override Curve3D.ResD1 EvalD1(double u)
    {
        Curve3D.ResD3 res = _hCurve.EvalD3(u);
        Vector3D dc = res.D1 * _ratio;
        return new Curve3D.ResD1 { Point = new Point3D(dc.Crossed(res.D2).XYZ()), D1 = dc.Crossed(res.D3) };
    }

    internal sealed override Curve3D.ResD2 EvalD2(double u)
    {
        Curve3D.ResD3 res = _hCurve.EvalD3(u);
        Vector3D d4 = _hCurve.EvalDN(u, 4);
        return new Curve3D.ResD2
        {
            Point = new Point3D((res.D1.Crossed(res.D2) * _ratio).XYZ()),
            D1 = res.D1.Crossed(res.D3) * _ratio,
            D2 = (res.D2.Crossed(res.D3) + res.D1.Crossed(d4)) * _ratio,
        };
    }

    internal sealed override Curve3D.ResD3 EvalD3(double u)
    {
        Curve3D.ResD3 res = _hCurve.EvalD3(u);
        Vector3D d4 = _hCurve.EvalDN(u, 4);
        Vector3D d5 = _hCurve.EvalDN(u, 5);
        return new Curve3D.ResD3
        {
            Point = new Point3D((res.D1.Crossed(res.D2) * _ratio).XYZ()),
            D1 = res.D1.Crossed(res.D3) * _ratio,
            D2 = (res.D2.Crossed(res.D3) + res.D1.Crossed(d4)) * _ratio,
            D3 = (res.D1.Crossed(d5) + res.D2.Crossed(d4) * 2) * _ratio,
        };
    }

    public sealed override Vector3D EvalDN(double u, int n)
    {
        Standard_RangeError_Raise_if(n < 1, "Exception: Geom2d_OffsetCurve::DN(). N<1.");
        switch (n)
        {
            case 1:
                return EvalD1(u).D1;
            case 2:
                return EvalD2(u).D2;
            case 3:
                return EvalD3(u).D3;
            default:
                throw new MethodNotImplementedException("Exception: Derivative order is greater than 3. "
                                                  + "Cannot compute of derivative.");
        }
    }

    public override double Resolution(double r3D) => global::Kerf.Engine.Foundation.Precision.Parametric(r3D);

    public override CurveKind GetType() => CurveKind.OtherCurve;
}
