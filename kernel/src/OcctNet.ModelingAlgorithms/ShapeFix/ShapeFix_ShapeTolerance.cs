// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_ShapeTolerance.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_ShapeTolerance.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

// SKELETON: the signatures other classes call; the bodies are being ported.
public sealed class ShapeFix_ShapeTolerance
{
    public ShapeFix_ShapeTolerance()
    {
    }

    internal bool LimitTolerance(in TopoDS_Shape shape, double tmin, double tmax = 0.0, TopAbs_ShapeEnum styp = TopAbs_SHAPE) =>
        throw new NotImplementedException();

    internal void SetTolerance(in TopoDS_Shape shape, double preci, TopAbs_ShapeEnum styp = TopAbs_SHAPE) =>
        throw new NotImplementedException();
}
