// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeInfo.hxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeInfo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Gives information on an object, see template DataInfo. This class is for Shape.</summary>
public static class TransferBRep_ShapeInfo
{
    /// <summary>Returns the Type attached to an object. Here, TShape (Shape has no Dynamic Type).</summary>
    public static Type Type(in TopoDS_Shape ent)
    {
        return typeof(TopoDS_TShape);
    }

    /// <summary>Returns Type Name (string). Here, the true name of the Type of a Shape.</summary>
    public static string TypeName(in TopoDS_Shape ent)
    {
        if (ent.IsNull())
        {
            return "TopoDS_Shape";
        }
        switch (ent.ShapeType())
        {
            case TopAbs_VERTEX:
                return "TopoDS_Vertex";
            case TopAbs_EDGE:
                return "TopoDS_Edge";
            case TopAbs_WIRE:
                return "TopoDS_Wire";
            case TopAbs_FACE:
                return "TopoDS_Face";
            case TopAbs_SHELL:
                return "TopoDS_Shell";
            case TopAbs_SOLID:
                return "TopoDS_Solid";
            case TopAbs_COMPSOLID:
                return "TopoDS_CompSolid";
            case TopAbs_COMPOUND:
                return "TopoDS_Compound";
            default:
                break;
        }
        return "TopoDS_Shape";
    }
}
