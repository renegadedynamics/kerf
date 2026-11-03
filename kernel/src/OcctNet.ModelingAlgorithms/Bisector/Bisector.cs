// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>Tools of the Bisector package.</summary>
public static class Bisector
{
    /// <summary>
    /// True if the curve is convex on the side Sign (1 left, -1 right): tested at its middle parameter, with a
    /// tolerance of 1e-5 on the cross product of its first and second derivatives.
    /// </summary>
    public static bool IsConvex(Curve2D Cu, double Sign)
    {
        double U1 = (Cu.LastParameter() + Cu.FirstParameter()) / 2.0;
        Cu.D2(U1, out Point2D _, out Vector2D V1, out Vector2D V2);
        double Tol = 1.0e-5;
        return Sign * (V1 ^ V2) < Tol;
    }
}
