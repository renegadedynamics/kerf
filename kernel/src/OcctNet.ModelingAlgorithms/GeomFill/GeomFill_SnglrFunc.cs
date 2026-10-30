// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SnglrFunc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SnglrFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>The function C'(t) ^ C''(t) of a curve C, as a curve (Frenet's search for singular points).</summary>
/// <remarks>A working adaptor (decision 101): it keeps the caller's curve and never changes it.</remarks>
public class GeomFill_SnglrFunc : Adaptor3d_Curve
{
    private readonly Adaptor3d_Curve myHCurve;
    private double ratio;

    public GeomFill_SnglrFunc(Adaptor3d_Curve HC)
    {
        myHCurve = HC;
        ratio = 1;
    }

    /// <summary>A shallow copy of the adaptor.</summary>
    public override Adaptor3d_Curve ShallowCopy()
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

    public override int NbIntervals(GeomAbs_Shape S)
    {
        GeomAbs_Shape HCS = GeomAbs_C0;
        if (S == GeomAbs_C0)
        {
            HCS = GeomAbs_C2;
        }
        else if (S == GeomAbs_C1)
        {
            HCS = GeomAbs_C3;
        }
        else if (S >= GeomAbs_C2)
        {
            HCS = GeomAbs_CN;
        }
        return myHCurve.NbIntervals(HCS);
    }

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        GeomAbs_Shape HCS = GeomAbs_C0;
        if (S == GeomAbs_C0)
        {
            HCS = GeomAbs_C2;
        }
        else if (S == GeomAbs_C1)
        {
            HCS = GeomAbs_C3;
        }
        else if (S >= GeomAbs_C2)
        {
            HCS = GeomAbs_CN;
        }
        myHCurve.Intervals(T, HCS);
    }

    public override bool IsPeriodic() => myHCurve.IsPeriodic();

    public override double Period() => myHCurve.Period();

    public sealed override gp_Pnt EvalD0(double theU)
    {
        Geom_Curve.ResD2 aRes = myHCurve.EvalD2(theU);
        return new gp_Pnt((aRes.D1 * ratio).Crossed(aRes.D2).XYZ());
    }

    public sealed override Geom_Curve.ResD1 EvalD1(double theU)
    {
        Geom_Curve.ResD3 aRes = myHCurve.EvalD3(theU);
        gp_Vec aDC = aRes.D1 * ratio;
        return new Geom_Curve.ResD1 { Point = new gp_Pnt(aDC.Crossed(aRes.D2).XYZ()), D1 = aDC.Crossed(aRes.D3) };
    }

    public sealed override Geom_Curve.ResD2 EvalD2(double theU)
    {
        Geom_Curve.ResD3 aRes = myHCurve.EvalD3(theU);
        gp_Vec aD4 = myHCurve.EvalDN(theU, 4);
        return new Geom_Curve.ResD2
        {
            Point = new gp_Pnt((aRes.D1.Crossed(aRes.D2) * ratio).XYZ()),
            D1 = aRes.D1.Crossed(aRes.D3) * ratio,
            D2 = (aRes.D2.Crossed(aRes.D3) + aRes.D1.Crossed(aD4)) * ratio,
        };
    }

    public sealed override Geom_Curve.ResD3 EvalD3(double theU)
    {
        Geom_Curve.ResD3 aRes = myHCurve.EvalD3(theU);
        gp_Vec aD4 = myHCurve.EvalDN(theU, 4);
        gp_Vec aD5 = myHCurve.EvalDN(theU, 5);
        return new Geom_Curve.ResD3
        {
            Point = new gp_Pnt((aRes.D1.Crossed(aRes.D2) * ratio).XYZ()),
            D1 = aRes.D1.Crossed(aRes.D3) * ratio,
            D2 = (aRes.D2.Crossed(aRes.D3) + aRes.D1.Crossed(aD4)) * ratio,
            D3 = (aRes.D1.Crossed(aD5) + aRes.D2.Crossed(aD4) * 2) * ratio,
        };
    }

    public sealed override gp_Vec EvalDN(double theU, int theN)
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
                throw new Standard_NotImplemented("Exception: Derivative order is greater than 3. "
                                                  + "Cannot compute of derivative.");
        }
    }

    public override double Resolution(double R3D) => Precision.Parametric(R3D);

    public override GeomAbs_CurveType GetType() => GeomAbs_CurveType.GeomAbs_OtherCurve;
}
