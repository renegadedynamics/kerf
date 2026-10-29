// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TCompound.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TCompound.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An all-purpose set of shapes.</summary>
public class TopoDS_TCompound : TopoDS_TShape
{
    /// <summary>An empty TCompound.</summary>
    public TopoDS_TCompound() : base(TopAbs_ShapeEnum.TopAbs_COMPOUND)
    {
        Orientable(false);
    }

    /// <summary>An empty TCompound.</summary>
    public override TopoDS_TShape EmptyCopy() => new TopoDS_TCompound();
}
