// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ObjectType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The type byte of an object in the position-based binary format (<see cref="BinTools_ShapeWriter"/>,
/// <see cref="BinTools_ShapeReader"/>). The bytes after <see cref="BinTools_ObjectType_EndShape"/> are shapes: 200 +
/// 4 × shape type + orientation.
/// </summary>
public enum BinTools_ObjectType : byte
{
    BinTools_ObjectType_Unknown = 0,

    /// <summary>A reference by an 8-bit offset.</summary>
    BinTools_ObjectType_Reference8,

    /// <summary>A reference by a 16-bit offset.</summary>
    BinTools_ObjectType_Reference16,

    /// <summary>A reference by a 32-bit offset.</summary>
    BinTools_ObjectType_Reference32,

    /// <summary>A reference by a 64-bit offset.</summary>
    BinTools_ObjectType_Reference64,
    BinTools_ObjectType_Location,
    BinTools_ObjectType_SimpleLocation,
    BinTools_ObjectType_EmptyLocation,
    BinTools_ObjectType_LocationEnd,
    BinTools_ObjectType_Curve,
    BinTools_ObjectType_EmptyCurve,
    BinTools_ObjectType_Curve2d,
    BinTools_ObjectType_EmptyCurve2d,
    BinTools_ObjectType_Surface,
    BinTools_ObjectType_EmptySurface,
    BinTools_ObjectType_Polygon3d,
    BinTools_ObjectType_EmptyPolygon3d,
    BinTools_ObjectType_PolygonOnTriangulation,
    BinTools_ObjectType_EmptyPolygonOnTriangulation,
    BinTools_ObjectType_Triangulation,
    BinTools_ObjectType_EmptyTriangulation,

    /// <summary>The null shape.</summary>
    BinTools_ObjectType_EmptyShape = 198,

    /// <summary>The end of a shape record.</summary>
    BinTools_ObjectType_EndShape = 199,
}
