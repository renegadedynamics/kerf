// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_TheMultiLineOfWLApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_TheMultiLineOfWLApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// OCCT's instantiation of <see cref="ApproxInt_MultiLine{TheLine}"/> over <see cref="IntPatch_WLine"/>.
/// </summary>
/// <remarks>A sealed subclass of the generic class with OCCT's constructors (decision 607); the generic algorithms use
/// the generic class directly.</remarks>
public sealed class GeomInt_TheMultiLineOfWLApprox
    : ApproxInt_MultiLine<IntPatch_WLine>
{
    public GeomInt_TheMultiLineOfWLApprox()
    {
    }

    public GeomInt_TheMultiLineOfWLApprox(IntPatch_WLine line, ApproxInt_SvSurfaces? PtrSvSurfaces, int NbP3d, int NbP2d, bool ApproxU1V1,
        bool ApproxU2V2, double xo, double yo, double zo, double u1o, double v1o, double u2o, double v2o,
        bool P2DOnFirst, int IndMin = 0, int IndMax = 0)
        : base(line, PtrSvSurfaces, NbP3d, NbP2d, ApproxU1V1, ApproxU2V2, xo, yo, zo, u1o, v1o, u2o, v2o, P2DOnFirst,
               IndMin, IndMax)
    {
    }

    public GeomInt_TheMultiLineOfWLApprox(IntPatch_WLine line, int NbP3d, int NbP2d, bool ApproxU1V1, bool ApproxU2V2, double xo, double yo,
        double zo, double u1o, double v1o, double u2o, double v2o, bool P2DOnFirst, int IndMin = 0, int IndMax = 0)
        : base(line, NbP3d, NbP2d, ApproxU1V1, ApproxU2V2, xo, yo, zo, u1o, v1o, u2o, v2o, P2DOnFirst, IndMin, IndMax)
    {
    }
}
