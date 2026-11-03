// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_DenominatorMultiplier.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_DenominatorMultiplier.cxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_DenominatorMultiplierPtr.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Splines;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Library;

/// <summary>
/// The evaluator of a function a(u, v) of two variables, used by GeomLib.CancelDenominatorDerivative in one
/// direction: for a rational surface N/D, the surface (a·N)/(a·D) has a denominator whose first derivative in U
/// vanishes on the boundaries.
/// </summary>
public sealed class DenominatorMultiplier
{
    // OCCT keeps the handle; the surface is only read.
    private readonly BSplineSurface _surface;
    private readonly NCollection_Array1<double> _knotFlatVector;

    public DenominatorMultiplier(BSplineSurface surface, NCollection_Array1<double> knotVector)
    {
        _surface = surface;
        _knotFlatVector = new NCollection_Array1<double>(1, knotVector.Length());
        int i;
        for (i = 1; i <= knotVector.Length(); i++)
        {
            _knotFlatVector.SetValue(i, knotVector[i]);
        }
    }

    /// <summary>The value a(UParameter, VParameter).</summary>
    public double Value(double uParameter, double vParameter)
    {
        double dumaxv, duminv, dDduumaxv, dDduuminv, dv, buv = 0.0;
        // gp_Pnt         HermPnt;
        Point3D n;
        Vector3D nu, nv;
        NCollection_Array2<Point3D> surface_poles = _surface.InternalPoles();
        NCollection_Array2<double>? surface_weights = _surface.InternalWeights();
        NCollection_Array1<double> surface_u_knots = _surface.InternalUKnots();
        NCollection_Array1<int> surface_u_mults = _surface.InternalUMultiplicities();
        NCollection_Array1<double> surface_v_knots = _surface.InternalVKnots();
        NCollection_Array1<int> surface_v_mults = _surface.InternalVMultiplicities();
        int udegree = _surface.UDegree;
        int vdegree = _surface.VDegree;
        SurfaceBSplSLib.HomogeneousD1(_surface.UKnot(_surface.LastUKnotIndex()),
                               vParameter,
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
                               _surface.IsURational,
                               _surface.IsVRational,
                               _surface.IsUPeriodic,
                               _surface.IsVPeriodic,
                               out n,
                               out nu,
                               out nv,
                               out dumaxv,
                               out dDduumaxv,
                               out dv);
        SurfaceBSplSLib.HomogeneousD1(_surface.UKnot(1),
                               vParameter,
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
                               _surface.IsURational,
                               _surface.IsVRational,
                               _surface.IsUPeriodic,
                               _surface.IsVPeriodic,
                               out n,
                               out nu,
                               out nv,
                               out duminv,
                               out dDduuminv,
                               out dv);
        Matrix bSplineBasisDeriv = new(1, 2, 1, 4, 0.0);
        double b1prim0, bprelastprim1,
          lambda = (_surface.Weight(1, 1) / _surface.Weight(_surface.UPoleCount, 1));
        int index, i;

        global::Kerf.Engine.Geometry.Splines.BSplCLib.EvalBsplineBasis(1, 4, _knotFlatVector, 0.0, out index, bSplineBasisDeriv);
        b1prim0 = bSplineBasisDeriv[2, 2];
        global::Kerf.Engine.Geometry.Splines.BSplCLib.EvalBsplineBasis(1, 4, _knotFlatVector, 1.0, out index, bSplineBasisDeriv);
        bprelastprim1 = bSplineBasisDeriv[2, 3];
        Matrix bSplineBasisValue = new(1, 1, 1, 4, 0.0);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.EvalBsplineBasis(0, 4, _knotFlatVector, uParameter, out index, bSplineBasisValue);
        NCollection_Array1<double> value = new(0, 5);
        NCollection_Array1<double> polesenv = new(0, 5); // poles of a(u,v)
        for (i = 0; i <= 5; i++)
        {
            polesenv[i] = 0.0;
        }
        polesenv[0] = duminv;
        polesenv[1] = duminv - dDduuminv / b1prim0;
        polesenv[4] = lambda * lambda * (dumaxv - dDduumaxv / bprelastprim1);
        polesenv[5] = lambda * lambda * dumaxv;
        if (_knotFlatVector.Length() == 8)
        {
            value[0] = bSplineBasisValue[1, 1]; // values of the basic functions
            value[1] = bSplineBasisValue[1, 2];
            value[2] = 0.0;
            value[3] = 0.0;
            value[4] = bSplineBasisValue[1, 3];
            value[5] = bSplineBasisValue[1, 4];
        }
        if (_knotFlatVector.Length() == 9)
        {
            if (index == 1)
            {
                value[0] = bSplineBasisValue[1, 1];
                value[1] = bSplineBasisValue[1, 2];
                value[2] = bSplineBasisValue[1, 3];
                value[3] = 0.0;
                value[4] = bSplineBasisValue[1, 4];
                value[5] = 0.0;
            }
            else
            {
                value[0] = 0.0;
                value[1] = bSplineBasisValue[1, 1];
                value[2] = bSplineBasisValue[1, 2];
                value[3] = 0.0;
                value[4] = bSplineBasisValue[1, 3];
                value[5] = bSplineBasisValue[1, 4];
            }
            polesenv[2] = (0.5 * (polesenv[0] + polesenv[5]));
        }
        if (_knotFlatVector.Length() == 10)
        {
            if (index == 1)
            {
                value[0] = bSplineBasisValue[1, 1];
                value[1] = bSplineBasisValue[1, 2];
                value[2] = bSplineBasisValue[1, 3];
                value[3] = bSplineBasisValue[1, 4];
                value[4] = 0.0;
                value[5] = 0.0;
            }
            if (index == 2)
            {
                value[0] = 0.0;
                value[1] = bSplineBasisValue[1, 1];
                value[2] = bSplineBasisValue[1, 2];
                value[3] = bSplineBasisValue[1, 3];
                value[4] = bSplineBasisValue[1, 4];
                value[5] = 0.0;
            }
            if (index == 3)
            {
                value[0] = 0.0;
                value[1] = 0.0;
                value[2] = bSplineBasisValue[1, 1];
                value[3] = bSplineBasisValue[1, 2];
                value[4] = bSplineBasisValue[1, 3];
                value[5] = bSplineBasisValue[1, 4];
            }
            polesenv[2] = (0.5 * (polesenv[0] + polesenv[5]));
            polesenv[3] = polesenv[2];
        }
        for (i = 0; i <= 5; i++)
        {
            buv += polesenv[i] * value[i];
        }
        return buv;
    }
}
