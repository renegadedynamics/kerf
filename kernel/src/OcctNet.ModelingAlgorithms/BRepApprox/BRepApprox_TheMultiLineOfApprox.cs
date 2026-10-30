// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_TheMultiLineOfApprox.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_TheMultiLineOfApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// OCCT's instantiation of <see cref="ApproxInt_MultiLine{TheLine}"/> over <see cref="BRepApprox_ApproxLine"/>.
/// </summary>
/// <remarks>A sealed subclass of the generic class with OCCT's constructors (decision 607); the generic algorithms use
/// the generic class directly.</remarks>
public sealed class BRepApprox_TheMultiLineOfApprox
    : ApproxInt_MultiLine<BRepApprox_ApproxLine>
{
    public BRepApprox_TheMultiLineOfApprox()
    {
    }

    public BRepApprox_TheMultiLineOfApprox(BRepApprox_ApproxLine line, ApproxInt_SvSurfaces? PtrSvSurfaces, int NbP3d, int NbP2d, bool ApproxU1V1,
        bool ApproxU2V2, double xo, double yo, double zo, double u1o, double v1o, double u2o, double v2o,
        bool P2DOnFirst, int IndMin = 0, int IndMax = 0)
        : base(line, PtrSvSurfaces, NbP3d, NbP2d, ApproxU1V1, ApproxU2V2, xo, yo, zo, u1o, v1o, u2o, v2o, P2DOnFirst,
               IndMin, IndMax)
    {
    }

    public BRepApprox_TheMultiLineOfApprox(BRepApprox_ApproxLine line, int NbP3d, int NbP2d, bool ApproxU1V1, bool ApproxU2V2, double xo, double yo,
        double zo, double u1o, double v1o, double u2o, double v2o, bool P2DOnFirst, int IndMin = 0, int IndMax = 0)
        : base(line, NbP3d, NbP2d, ApproxU1V1, ApproxU2V2, xo, yo, zo, u1o, v1o, u2o, v2o, P2DOnFirst, IndMin, IndMax)
    {
    }
}
