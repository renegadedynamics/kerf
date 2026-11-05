// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_TheMultiLineOfApprox.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_TheMultiLineOfApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>
/// OCCT's instantiation of <see cref="MultiLine{TheLine}"/> over <see cref="ApproxLine"/>.
/// </summary>
/// <remarks>A sealed subclass of the generic class with OCCT's constructors (decision 607); the generic algorithms use
/// the generic class directly.</remarks>
internal sealed class TheMultiLineOfApprox
    : MultiLine<ApproxLine>
{
    public TheMultiLineOfApprox()
    {
    }

    public TheMultiLineOfApprox(ApproxLine line, SvSurfaces? ptrSvSurfaces, int nbP3d, int nbP2d, bool approxU1V1,
        bool approxU2V2, double xo, double yo, double zo, double u1o, double v1o, double u2o, double v2o,
        bool p2DOnFirst, int indMin = 0, int indMax = 0)
        : base(line, ptrSvSurfaces, nbP3d, nbP2d, approxU1V1, approxU2V2, xo, yo, zo, u1o, v1o, u2o, v2o, p2DOnFirst,
               indMin, indMax)
    {
    }

    public TheMultiLineOfApprox(ApproxLine line, int nbP3d, int nbP2d, bool approxU1V1, bool approxU2V2, double xo, double yo,
        double zo, double u1o, double v1o, double u2o, double v2o, bool p2DOnFirst, int indMin = 0, int indMax = 0)
        : base(line, nbP3d, nbP2d, approxU1V1, approxU2V2, xo, yo, zo, u1o, v1o, u2o, v2o, p2DOnFirst, indMin, indMax)
    {
    }
}
