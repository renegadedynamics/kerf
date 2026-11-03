// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_FrozenShape.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_LockedShape.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_UnCompatibleShapes.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An attempt was made to modify a shape already shared or protected (OCCT's Free flag is off).</summary>
[OcctName("TopoDS_FrozenShape")]
public class TopoDS_FrozenShape : Standard_DomainError
{
    public TopoDS_FrozenShape() { }
    public TopoDS_FrozenShape(string? message) : base(message) { }
}

/// <summary>
/// An attempt was made to modify a geometry of a shape already shared or protected: the shape is Locked, which in
/// the port includes every frozen shape (docs/phase3-design.md, section 4.3).
/// </summary>
[OcctName("TopoDS_LockedShape")]
public class TopoDS_LockedShape : Standard_DomainError
{
    public TopoDS_LockedShape() { }
    public TopoDS_LockedShape(string? message) : base(message) { }
}

/// <summary>An attempt was made to add a shape to a shape of a type that can't contain it.</summary>
[OcctName("TopoDS_UnCompatibleShapes")]
public class TopoDS_UnCompatibleShapes : Standard_DomainError
{
    public TopoDS_UnCompatibleShapes() { }
    public TopoDS_UnCompatibleShapes(string? message) : base(message) { }
}

internal static class TopoDS_Raise
{
    internal static void TopoDS_FrozenShape_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new TopoDS_FrozenShape(message);
        }
    }
}
