// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_FuncCurExt2d.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The function whose roots are the extrema of curvature of a 2D curve: F = dKC/dU, KC = (V1 ^ V2) / |V1|^3, with its
/// derivative by finite differences.
/// </summary>
internal sealed class GeomLProp_FuncCurExt2d : FunctionWithDerivative
{
    private readonly Geom2d_Curve myCurve; // The analyzed curve.
    private readonly double myEpsX;        // Parametric tolerance for finite differences.

    internal GeomLProp_FuncCurExt2d(Geom2d_Curve theCurve, double theTol)
    {
        myCurve = theCurve;
        myEpsX = theTol;
    }

    /// <summary>dKC/dU at the parameter; false if the first derivative is degenerate.</summary>
    public override bool Value(double theU, out double theF)
    {
        theF = 0.0; // OCCT leaves it unset when it returns false

        myCurve.D3(theU, out Point2D _, out Vector2D aDeriv1, out Vector2D aDeriv2, out Vector2D aDeriv3);
        double aCross12 = aDeriv1.Crossed(aDeriv2);
        double aCross13 = aDeriv1.Crossed(aDeriv3);
        double aDot12 = aDeriv1.Dot(aDeriv2);
        double aSqNorm1 = aDeriv1.SquareMagnitude();
        double aNorm1 = Math.Sqrt(aSqNorm1);
        double aNorm1Cb = aSqNorm1 * aNorm1;     // ||V1||^3
        double aNorm1Pow5 = aNorm1Cb * aSqNorm1; // ||V1||^5

        if (aNorm1Pow5 < Constants.Resolution())
        {
            return false;
        }

        theF = aCross13 / aNorm1Cb - 3.0 * aCross12 * aDot12 / aNorm1Pow5;
        return true;
    }

    public override bool Derivative(double theU, out double theD)
    {
        return Values(theU, out _, out theD);
    }

    /// <summary>F and its derivative by a forward (or, at the end, backward) difference of myEpsX / 100.</summary>
    public override bool Values(double theU, out double theF, out double theD)
    {
        theD = 0.0; // OCCT leaves it unset when it returns false
        double aFNext;
        double aDeltaU = myEpsX / 100.0;

        if (theU + aDeltaU > myCurve.LastParameter())
        {
            aDeltaU = -aDeltaU;
        }

        if (!Value(theU, out theF) || !Value(theU + aDeltaU, out aFNext))
        {
            return false;
        }

        theD = (aFNext - theF) / aDeltaU;
        return true;
    }

    /// <summary>
    /// True if the parameter is a minimum of the radius of curvature: |KC(theU)| &gt; |KC(theU + eps)|.
    /// </summary>
    internal bool IsMinKC(double theU)
    {
        double aDeltaU = myEpsX;

        // Compute curvature at theU.
        myCurve.D3(theU, out Point2D aPnt, out Vector2D aDeriv1, out Vector2D aDeriv2, out Vector2D aDeriv3);
        double aCross12 = aDeriv1.Crossed(aDeriv2);
        double aSqNorm1 = aDeriv1.SquareMagnitude();
        double aNorm1 = Math.Sqrt(aSqNorm1);
        double aNorm1Cb = aSqNorm1 * aNorm1;

        if (aNorm1Cb < Constants.Resolution())
        {
            return false;
        }

        double aCurvCurr = aCross12 / aNorm1Cb;

        if (theU + aDeltaU > myCurve.LastParameter())
        {
            aDeltaU = -aDeltaU;
        }

        // Compute curvature at the probe point.
        myCurve.D3(theU + aDeltaU, out aPnt, out aDeriv1, out aDeriv2, out aDeriv3);
        aCross12 = aDeriv1.Crossed(aDeriv2);
        aSqNorm1 = aDeriv1.SquareMagnitude();
        aNorm1 = Math.Sqrt(aSqNorm1);
        aNorm1Cb = aSqNorm1 * aNorm1;

        if (aNorm1Cb < Constants.Resolution())
        {
            return false;
        }

        double aCurvProbe = aCross12 / aNorm1Cb;

        return Math.Abs(aCurvCurr) > Math.Abs(aCurvProbe);
    }
}
