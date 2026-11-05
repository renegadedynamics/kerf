// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_FunctionInter.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_FunctionInter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Offsets.Bisectors;

/// <summary>
/// F(u) = ||PC(u) - PBis1(u)|| - ||PC(u) - PBis2(u)||, used by Bisector_Inter to intersect two neighbouring bisectors
/// parameterised by the same curve.
/// </summary>
internal sealed class FunctionInter : FunctionWithDerivative
{
    private Curve2D _curve = null!;
    private BisectorCurve _bisector1 = null!;
    private BisectorCurve _bisector2 = null!;

    /// <summary>An empty function: Perform must be called before it is evaluated.</summary>
    public FunctionInter()
    {
    }

    /// <summary>The function of the curve C and the bisectors B1 and B2.</summary>
    public FunctionInter(Curve2D c, BisectorCurve b1, BisectorCurve b2)
    {
        _curve = c;
        _bisector1 = b1;
        _bisector2 = b2;
    }

    /// <summary>Sets the curve C and the bisectors B1 and B2.</summary>
    public void Perform(Curve2D c, BisectorCurve b1, BisectorCurve b2)
    {
        _curve = c;
        _bisector1 = b1;
        _bisector2 = b2;
    }

    /// <inheritdoc/>
    public override bool Value(double x, out double f)
    {
        Point2D pc = _curve.Value(x);
        Point2D pb1 = _bisector1.Value(x);
        Point2D pb2 = _bisector2.Value(x);

        f = pc.DistanceTo(pb1) - pc.DistanceTo(pb2);

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
        double f1, f2, df1, df2;

        _curve.D1(x, out Point2D pc, out Vector2D tc);
        _bisector1.D1(x, out Point2D pb1, out Vector2D tb1);
        _bisector2.D1(x, out Point2D pb2, out Vector2D tb2);

        f1 = pc.DistanceTo(pb1);
        f2 = pc.DistanceTo(pb2);

        f = f1 - f2;

        if (Math.Abs(f1) < Constants.Resolution())
        {
            df1 = global::Kerf.Engine.Foundation.Precision.Infinite();
        }
        else
        {
            df1 = ((pc.X - pb1.X) * (tc.X - tb1.X) + (pc.Y - pb1.Y) * (tc.Y - tb1.Y)) / f1;
        }
        if (Math.Abs(f2) < Constants.Resolution())
        {
            df2 = global::Kerf.Engine.Foundation.Precision.Infinite();
        }
        else
        {
            df2 = ((pc.X - pb2.X) * (tc.X - tb2.X) + (pc.Y - pb2.Y) * (tc.Y - tb2.Y)) / f2;
        }
        d = df1 - df2;

        return true;
    }
}
