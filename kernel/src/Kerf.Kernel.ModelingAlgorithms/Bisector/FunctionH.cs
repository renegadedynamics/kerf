// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_FunctionH.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_FunctionH.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Offsets.Bisectors;

/// <summary>
/// H(v) = (T1.P2(v) - P1) * ||T(v)||^2 - (T(v).P2(v) - P1) * ||T1||^2, used by Bisector_BisecCC to find the point of
/// the second curve paired with a point of the first.
/// </summary>
internal sealed class FunctionH : FunctionWithDerivative
{
    private readonly Curve2D _curve2;
    private readonly Point2D _p1;
    private readonly Vector2D _t1;

    /// <summary>The function of the curve C2, for the point P1 of tangent T1.</summary>
    public FunctionH(Curve2D c2, in Point2D p1, in Vector2D t1)
    {
        _p1 = p1;
        Vector2D aT1 = t1;
        aT1 = aT1.Normalized();
        _t1 = aT1;
        _curve2 = c2;
    }

    //                F = P1P2.(||T2||T1 + T2)
    /// <inheritdoc/>
    public override bool Value(double x, out double f)
    {
        _curve2.D1(x, out Point2D p2, out Vector2D t2); // point on C2, tangent to C2 at V.
        double normT2 = t2.Magnitude;
        double ax = normT2 * _t1.X - t2.X;
        double ay = normT2 * _t1.Y - t2.Y;

        f = (_p1.X - p2.X) * ax + (_p1.Y - p2.Y) * ay;

        return true;
    }

    /// <inheritdoc/>
    public override bool Derivative(double x, out double d)
    {
        return Values(x, out double _, out d);
    }

    /// <inheritdoc/>
    public override bool Values(double x, out double f, out double d)
    {
        // point on C2, tangent to C2 at V, second derivative of C2 at V.
        _curve2.D2(x, out Point2D p2, out Vector2D t2, out Vector2D t2v);

        double normT2 = t2.Magnitude;
        double ax = normT2 * _t1.X - t2.X;
        double ay = normT2 * _t1.Y - t2.Y;

        f = (_p1.X - p2.X) * ax + (_p1.Y - p2.Y) * ay;

        double scal = t2.Dot(t2v) / normT2;
        double dAx = scal * _t1.X - t2v.X;
        double dAy = scal * _t1.Y - t2v.Y;

        d = -t2.X * ax - t2.Y * ay + (_p1.X - p2.X) * dAx + (_p1.Y - p2.Y) * dAy;

        return true;
    }
}
