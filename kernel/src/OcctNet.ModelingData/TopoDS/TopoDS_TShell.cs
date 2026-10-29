// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TShell.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TShell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A set of faces connected by their edges.</summary>
public class TopoDS_TShell : TopoDS_TShape
{
    /// <summary>An empty TShell.</summary>
    public TopoDS_TShell() : base(TopAbs_ShapeEnum.TopAbs_SHELL)
    {
    }

    /// <summary>An empty TShell.</summary>
    public override TopoDS_TShape EmptyCopy() => new TopoDS_TShell();
}
