// Port of: src/ModelingData/TKBRep/TopTools/TopTools.hxx
// Port of: src/ModelingData/TKBRep/TopTools/TopTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// TopTools::Dump, which comes with TopTools_ShapeSet
public static partial class TopTools
{
    /// <summary>Dumps the topological structure of the shape (without its geometry) in a readable form.</summary>
    public static void Dump(in TopoDS_Shape Sh, TextWriter S)
    {
        TopTools_ShapeSet SSet = new();
        SSet.Add(Sh);
        SSet.Dump(Sh, S);
        SSet.Dump(S);
    }
}
