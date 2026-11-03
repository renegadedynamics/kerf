// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SnglrFunc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SnglrFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>The function C'(t) ^ C''(t) of a curve C, as a curve (Frenet's search for singular points).</summary>
/// <remarks>A working adaptor (decision 101): it keeps the caller's curve and never changes it.</remarks>
public class GeomFill_SnglrFunc : CurveAdaptorBase
{
    private readonly CurveAdaptorBase myHCurve;
    private double ratio;

    public GeomFill_SnglrFunc(CurveAdaptorBase HC)
    {
        myHCurve = HC;
        ratio = 1;
    }

    /// <summary>A shallow copy of the adaptor.</summary>
    public override CurveAdaptorBase ShallowCopy()
    {
        GeomFill_SnglrFunc aCopy = new GeomFill_SnglrFunc(myHCurve.ShallowCopy());
        aCopy.ratio = ratio;
        return aCopy;
    }

    public void SetRatio(double Ratio)
    {
        ratio = Ratio;
    }

    public override double FirstParameter() => myHCurve.FirstParameter();

    public override double LastParameter() => myHCurve.LastParameter();

    public override int NbIntervals(Continuity S)
    {
        Continuity HCS = C0;
        if (S == C0)
        {
            HCS = C2;
        }
        else if (S == C1)
        {
            HCS = C3;
        }
        else if (S >= C2)
        {
            HCS = CN;
        }
        return myHCurve.NbIntervals(HCS);
    }

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        Continuity HCS = C0;
        if (S == C0)
        {
            HCS = C2;
        }
        else if (S == C1)
        {
            HCS = C3;
        }
        else if (S >= C2)
        {
            HCS = CN;
        }
        myHCurve.Intervals(T, HCS);
    }

    public override bool IsPeriodic => myHCurve.IsPeriodic;

    public override double Period() => myHCurve.Period();

    public sealed override Point3D EvalD0(double theU)
    {
        Curve3D.ResD2 aRes = myHCurve.EvalD2(theU);
        return new Point3D((aRes.D1 * ratio).Crossed(aRes.D2).XYZ());
    }

    public sealed override Curve3D.ResD1 EvalD1(double theU)
    {
        Curve3D.ResD3 aRes = myHCurve.EvalD3(theU);
        Vector3D aDC = aRes.D1 * ratio;
        return new Curve3D.ResD1 { Point = new Point3D(aDC.Crossed(aRes.D2).XYZ()), D1 = aDC.Crossed(aRes.D3) };
    }

    public sealed override Curve3D.ResD2 EvalD2(double theU)
    {
        Curve3D.ResD3 aRes = myHCurve.EvalD3(theU);
        Vector3D aD4 = myHCurve.EvalDN(theU, 4);
        return new Curve3D.ResD2
        {
            Point = new Point3D((aRes.D1.Crossed(aRes.D2) * ratio).XYZ()),
            D1 = aRes.D1.Crossed(aRes.D3) * ratio,
            D2 = (aRes.D2.Crossed(aRes.D3) + aRes.D1.Crossed(aD4)) * ratio,
        };
    }

    public sealed override Curve3D.ResD3 EvalD3(double theU)
    {
        Curve3D.ResD3 aRes = myHCurve.EvalD3(theU);
        Vector3D aD4 = myHCurve.EvalDN(theU, 4);
        Vector3D aD5 = myHCurve.EvalDN(theU, 5);
        return new Curve3D.ResD3
        {
            Point = new Point3D((aRes.D1.Crossed(aRes.D2) * ratio).XYZ()),
            D1 = aRes.D1.Crossed(aRes.D3) * ratio,
            D2 = (aRes.D2.Crossed(aRes.D3) + aRes.D1.Crossed(aD4)) * ratio,
            D3 = (aRes.D1.Crossed(aD5) + aRes.D2.Crossed(aD4) * 2) * ratio,
        };
    }

    public sealed override Vector3D EvalDN(double theU, int theN)
    {
        Standard_RangeError_Raise_if(theN < 1, "Exception: Geom2d_OffsetCurve::DN(). N<1.");
        switch (theN)
        {
            case 1:
                return EvalD1(theU).D1;
            case 2:
                return EvalD2(theU).D2;
            case 3:
                return EvalD3(theU).D3;
            default:
                throw new MethodNotImplementedException("Exception: Derivative order is greater than 3. "
                                                  + "Cannot compute of derivative.");
        }
    }

    public override double Resolution(double R3D) => global::Kerf.Engine.Foundation.Precision.Parametric(R3D);

    public override CurveKind GetType() => CurveKind.OtherCurve;
}
