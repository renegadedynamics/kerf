// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_FunctionH.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_FunctionH.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// H(v) = (T1.P2(v) - P1) * ||T(v)||^2 - (T(v).P2(v) - P1) * ||T1||^2, used by Bisector_BisecCC to find the point of
/// the second curve paired with a point of the first.
/// </summary>
public sealed class Bisector_FunctionH : FunctionWithDerivative
{
    private readonly Geom2d_Curve curve2;
    private readonly Point2D p1;
    private readonly Vector2D t1;

    /// <summary>The function of the curve C2, for the point P1 of tangent T1.</summary>
    public Bisector_FunctionH(Geom2d_Curve C2, in Point2D P1, in Vector2D T1)
    {
        p1 = P1;
        Vector2D aT1 = T1;
        aT1.Normalize();
        t1 = aT1;
        curve2 = C2;
    }

    //                F = P1P2.(||T2||T1 + T2)
    /// <inheritdoc/>
    public override bool Value(double X, out double F)
    {
        curve2.D1(X, out Point2D P2, out Vector2D T2); // point on C2, tangent to C2 at V.
        double NormT2 = T2.Magnitude();
        double Ax = NormT2 * t1.X() - T2.X();
        double Ay = NormT2 * t1.Y() - T2.Y();

        F = (p1.X() - P2.X()) * Ax + (p1.Y() - P2.Y()) * Ay;

        return true;
    }

    /// <inheritdoc/>
    public override bool Derivative(double X, out double D)
    {
        return Values(X, out double _, out D);
    }

    /// <inheritdoc/>
    public override bool Values(double X, out double F, out double D)
    {
        // point on C2, tangent to C2 at V, second derivative of C2 at V.
        curve2.D2(X, out Point2D P2, out Vector2D T2, out Vector2D T2v);

        double NormT2 = T2.Magnitude();
        double Ax = NormT2 * t1.X() - T2.X();
        double Ay = NormT2 * t1.Y() - T2.Y();

        F = (p1.X() - P2.X()) * Ax + (p1.Y() - P2.Y()) * Ay;

        double Scal = T2.Dot(T2v) / NormT2;
        double dAx = Scal * t1.X() - T2v.X();
        double dAy = Scal * t1.Y() - T2v.Y();

        D = -T2.X() * Ax - T2.Y() * Ay + (p1.X() - P2.X()) * dAx + (p1.Y() - P2.Y()) * dAy;

        return true;
    }
}
