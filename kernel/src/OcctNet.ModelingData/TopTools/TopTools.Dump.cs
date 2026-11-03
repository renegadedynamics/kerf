// Port of: src/ModelingData/TKBRep/TopTools/TopTools.hxx
// Port of: src/ModelingData/TKBRep/TopTools/TopTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

using Occt;

namespace Kerf.Engine.Exchange.Brep;

// TopTools::Dump, which comes with TopTools_ShapeSet
public static partial class ShapeTools
{
    /// <summary>Dumps the topological structure of the shape (without its geometry) in a readable form.</summary>
    public static void Dump(in Shape sh, TextWriter s)
    {
        ShapeSet sSet = new();
        sSet.Add(sh);
        sSet.Dump(sh, s);
        sSet.Dump(s);
    }
}
