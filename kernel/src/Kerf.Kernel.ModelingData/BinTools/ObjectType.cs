// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ObjectType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Brep.Binary;

/// <summary>
/// The type byte of an object in the position-based binary format (<see cref="ShapeWriter"/>,
/// <see cref="ShapeReader"/>). The bytes after <see cref="ObjectType_EndShape"/> are shapes: 200 +
/// 4 × shape type + orientation.
/// </summary>
internal enum ObjectType : byte
{
    ObjectType_Unknown = 0,

    /// <summary>A reference by an 8-bit offset.</summary>
    ObjectType_Reference8,

    /// <summary>A reference by a 16-bit offset.</summary>
    ObjectType_Reference16,

    /// <summary>A reference by a 32-bit offset.</summary>
    ObjectType_Reference32,

    /// <summary>A reference by a 64-bit offset.</summary>
    ObjectType_Reference64,
    ObjectType_Location,
    ObjectType_SimpleLocation,
    ObjectType_EmptyLocation,
    ObjectType_LocationEnd,
    ObjectType_Curve,
    ObjectType_EmptyCurve,
    ObjectType_Curve2d,
    ObjectType_EmptyCurve2d,
    ObjectType_Surface,
    ObjectType_EmptySurface,
    ObjectType_Polygon3d,
    ObjectType_EmptyPolygon3d,
    ObjectType_PolygonOnTriangulation,
    ObjectType_EmptyPolygonOnTriangulation,
    ObjectType_Triangulation,
    ObjectType_EmptyTriangulation,

    /// <summary>The null shape.</summary>
    ObjectType_EmptyShape = 198,

    /// <summary>The end of a shape record.</summary>
    ObjectType_EndShape = 199,
}
