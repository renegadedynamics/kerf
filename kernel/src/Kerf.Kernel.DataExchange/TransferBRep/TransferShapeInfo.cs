// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeInfo.hxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeInfo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>Gives information on an object, see template DataInfo. This class is for Shape.</summary>
internal static class TransferShapeInfo
{
    /// <summary>Returns the Type attached to an object. Here, TShape (Shape has no Dynamic Type).</summary>
    public static Type Type(in Shape ent)
    {
        return typeof(TShape);
    }

    /// <summary>Returns Type Name (string). Here, the true name of the Type of a Shape.</summary>
    public static string TypeName(in Shape ent)
    {
        if (ent.IsNull)
        {
            return "TopoDS_Shape";
        }
        switch (ent.Type)
        {
            case ShapeType.Vertex:
                return "TopoDS_Vertex";
            case ShapeType.Edge:
                return "TopoDS_Edge";
            case ShapeType.Wire:
                return "TopoDS_Wire";
            case ShapeType.Face:
                return "TopoDS_Face";
            case ShapeType.Shell:
                return "TopoDS_Shell";
            case ShapeType.Solid:
                return "TopoDS_Solid";
            case ShapeType.CompSolid:
                return "TopoDS_CompSolid";
            case ShapeType.Compound:
                return "TopoDS_Compound";
            default:
                break;
        }
        return "TopoDS_Shape";
    }
}
