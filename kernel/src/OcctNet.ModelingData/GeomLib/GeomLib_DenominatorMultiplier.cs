// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_DenominatorMultiplier.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_DenominatorMultiplier.cxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_DenominatorMultiplierPtr.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The evaluator of a function a(u, v) of two variables, used by GeomLib.CancelDenominatorDerivative in one
/// direction: for a rational surface N/D, the surface (a·N)/(a·D) has a denominator whose first derivative in U
/// vanishes on the boundaries.
/// </summary>
public sealed class GeomLib_DenominatorMultiplier
{
    // OCCT keeps the handle; the surface is only read.
    private readonly Geom_BSplineSurface mySurface;
    private readonly NCollection_Array1<double> myKnotFlatVector;

    public GeomLib_DenominatorMultiplier(Geom_BSplineSurface Surface, NCollection_Array1<double> KnotVector)
    {
        mySurface = Surface;
        myKnotFlatVector = new NCollection_Array1<double>(1, KnotVector.Length());
        int i;
        for (i = 1; i <= KnotVector.Length(); i++)
        {
            myKnotFlatVector.SetValue(i, KnotVector[i]);
        }
    }

    /// <summary>The value a(UParameter, VParameter).</summary>
    public double Value(double UParameter, double VParameter)
    {
        double Dumaxv, Duminv, dDduumaxv, dDduuminv, Dv, Buv = 0.0;
        // gp_Pnt         HermPnt;
        gp_Pnt N;
        gp_Vec Nu, Nv;
        NCollection_Array2<gp_Pnt> surface_poles = mySurface.InternalPoles();
        NCollection_Array2<double>? surface_weights = mySurface.InternalWeights();
        NCollection_Array1<double> surface_u_knots = mySurface.InternalUKnots();
        NCollection_Array1<int> surface_u_mults = mySurface.InternalUMultiplicities();
        NCollection_Array1<double> surface_v_knots = mySurface.InternalVKnots();
        NCollection_Array1<int> surface_v_mults = mySurface.InternalVMultiplicities();
        int udegree = mySurface.UDegree();
        int vdegree = mySurface.VDegree();
        BSplSLib.HomogeneousD1(mySurface.UKnot(mySurface.LastUKnotIndex()),
                               VParameter,
                               0,
                               0,
                               surface_poles,
                               surface_weights,
                               surface_u_knots,
                               surface_v_knots,
                               surface_u_mults,
                               surface_v_mults,
                               udegree,
                               vdegree,
                               mySurface.IsURational(),
                               mySurface.IsVRational(),
                               mySurface.IsUPeriodic(),
                               mySurface.IsVPeriodic(),
                               out N,
                               out Nu,
                               out Nv,
                               out Dumaxv,
                               out dDduumaxv,
                               out Dv);
        BSplSLib.HomogeneousD1(mySurface.UKnot(1),
                               VParameter,
                               0,
                               0,
                               surface_poles,
                               surface_weights,
                               surface_u_knots,
                               surface_v_knots,
                               surface_u_mults,
                               surface_v_mults,
                               udegree,
                               vdegree,
                               mySurface.IsURational(),
                               mySurface.IsVRational(),
                               mySurface.IsUPeriodic(),
                               mySurface.IsVPeriodic(),
                               out N,
                               out Nu,
                               out Nv,
                               out Duminv,
                               out dDduuminv,
                               out Dv);
        math_Matrix BSplineBasisDeriv = new(1, 2, 1, 4, 0.0);
        double B1prim0, Bprelastprim1,
          lambda = (mySurface.Weight(1, 1) / mySurface.Weight(mySurface.NbUPoles(), 1));
        int index, i;

        BSplCLib.EvalBsplineBasis(1, 4, myKnotFlatVector, 0.0, out index, BSplineBasisDeriv);
        B1prim0 = BSplineBasisDeriv[2, 2];
        BSplCLib.EvalBsplineBasis(1, 4, myKnotFlatVector, 1.0, out index, BSplineBasisDeriv);
        Bprelastprim1 = BSplineBasisDeriv[2, 3];
        math_Matrix BSplineBasisValue = new(1, 1, 1, 4, 0.0);
        BSplCLib.EvalBsplineBasis(0, 4, myKnotFlatVector, UParameter, out index, BSplineBasisValue);
        NCollection_Array1<double> value = new(0, 5);
        NCollection_Array1<double> Polesenv = new(0, 5); // poles of a(u,v)
        for (i = 0; i <= 5; i++)
        {
            Polesenv[i] = 0.0;
        }
        Polesenv[0] = Duminv;
        Polesenv[1] = Duminv - dDduuminv / B1prim0;
        Polesenv[4] = lambda * lambda * (Dumaxv - dDduumaxv / Bprelastprim1);
        Polesenv[5] = lambda * lambda * Dumaxv;
        if (myKnotFlatVector.Length() == 8)
        {
            value[0] = BSplineBasisValue[1, 1]; // values of the basic functions
            value[1] = BSplineBasisValue[1, 2];
            value[2] = 0.0;
            value[3] = 0.0;
            value[4] = BSplineBasisValue[1, 3];
            value[5] = BSplineBasisValue[1, 4];
        }
        if (myKnotFlatVector.Length() == 9)
        {
            if (index == 1)
            {
                value[0] = BSplineBasisValue[1, 1];
                value[1] = BSplineBasisValue[1, 2];
                value[2] = BSplineBasisValue[1, 3];
                value[3] = 0.0;
                value[4] = BSplineBasisValue[1, 4];
                value[5] = 0.0;
            }
            else
            {
                value[0] = 0.0;
                value[1] = BSplineBasisValue[1, 1];
                value[2] = BSplineBasisValue[1, 2];
                value[3] = 0.0;
                value[4] = BSplineBasisValue[1, 3];
                value[5] = BSplineBasisValue[1, 4];
            }
            Polesenv[2] = (0.5 * (Polesenv[0] + Polesenv[5]));
        }
        if (myKnotFlatVector.Length() == 10)
        {
            if (index == 1)
            {
                value[0] = BSplineBasisValue[1, 1];
                value[1] = BSplineBasisValue[1, 2];
                value[2] = BSplineBasisValue[1, 3];
                value[3] = BSplineBasisValue[1, 4];
                value[4] = 0.0;
                value[5] = 0.0;
            }
            if (index == 2)
            {
                value[0] = 0.0;
                value[1] = BSplineBasisValue[1, 1];
                value[2] = BSplineBasisValue[1, 2];
                value[3] = BSplineBasisValue[1, 3];
                value[4] = BSplineBasisValue[1, 4];
                value[5] = 0.0;
            }
            if (index == 3)
            {
                value[0] = 0.0;
                value[1] = 0.0;
                value[2] = BSplineBasisValue[1, 1];
                value[3] = BSplineBasisValue[1, 2];
                value[4] = BSplineBasisValue[1, 3];
                value[5] = BSplineBasisValue[1, 4];
            }
            Polesenv[2] = (0.5 * (Polesenv[0] + Polesenv[5]));
            Polesenv[3] = Polesenv[2];
        }
        for (i = 0; i <= 5; i++)
        {
            Buv += Polesenv[i] * value[i];
        }
        return Buv;
    }
}
