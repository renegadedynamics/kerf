// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_DistFunction.hxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_DistFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The negated square distance between a point on the curve and the chord between the points of parameters U1 and
/// U2, used to find the maximal deflection of the chord.
/// </summary>
/// <remarks>Keeps a reference to the curve, as OCCT does.</remarks>
public class DistFunction : Function
{
    private readonly CurveAdaptorBase _curve;
    private readonly Line3D _lin;
    private readonly double _u1;
    private readonly double _u2;

    public DistFunction(CurveAdaptorBase curve, double u1, double u2)
    {
        _curve = curve;
        _u1 = u1;
        _u2 = u2;
        Point3D p1 = curve.Value(u1), p2 = curve.Value(u2);
        if (p1.SquareDistance(p2) > Constants.Resolution())
        {
            _lin = new Line3D(p1, new Direction3D(p2.XYZ() - p1.XYZ()));
        }
        else
        {
            curve.D0(u1 + .01 * (u2 - u1), out p2);
            _lin = new Line3D(p1, new Direction3D(p2.XYZ() - p1.XYZ()));
        }
    }

    /// <summary>A copy, on the same curve.</summary>
    public DistFunction(DistFunction other)
    {
        _curve = other._curve;
        _lin = other._lin;
        _u1 = other._u1;
        _u2 = other._u2;
    }

    /// <remarks>Out of [U1, U2] it returns false with F = 0, where OCCT leaves F unchanged (decision 32).</remarks>
    public override bool Value(double x, out double f)
    {
        if (x < _u1 || x > _u2)
        {
            f = 0.0;
            return false;
        }
        f = -_lin.SquareDistance(_curve.Value(x));
        return true;
    }
}

/// <summary><see cref="DistFunction"/> as a function of one variable for the multi-variable minimisers.</summary>
public class DistFunctionMV : MultipleVarFunction
{
    private readonly DistFunction _maxCurvLinDist;

    public DistFunctionMV(DistFunction curvLinDist)
    {
        _maxCurvLinDist = curvLinDist;
    }

    public override bool Value(math_Vector x, out double f)
    {
        bool ok = _maxCurvLinDist.Value(x[1], out f);
        return ok;
    }

    public override int VariableCount
    {
        get
        {
            return 1;
        }
    }
}
