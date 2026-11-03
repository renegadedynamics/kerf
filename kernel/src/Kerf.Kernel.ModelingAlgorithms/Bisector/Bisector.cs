// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Offsets.Bisectors;

/// <summary>Tools of the Bisector package.</summary>
public static class Bisector
{
    /// <summary>
    /// True if the curve is convex on the side Sign (1 left, -1 right): tested at its middle parameter, with a
    /// tolerance of 1e-5 on the cross product of its first and second derivatives.
    /// </summary>
    public static bool IsConvex(Curve2D cu, double sign)
    {
        double u1 = (cu.LastParameter() + cu.FirstParameter()) / 2.0;
        cu.D2(u1, out Point2D _, out Vector2D v1, out Vector2D v2);
        double tol = 1.0e-5;
        return sign * (v1 ^ v2) < tol;
    }
}
