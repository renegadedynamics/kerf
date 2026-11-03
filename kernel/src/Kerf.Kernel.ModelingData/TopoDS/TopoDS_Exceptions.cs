// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_FrozenShape.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_LockedShape.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_UnCompatibleShapes.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Topology;

/// <summary>An attempt was made to modify a shape already shared or protected (OCCT's Free flag is off).</summary>
public class FrozenShapeException : DomainErrorException
{
    public FrozenShapeException() { }
    public FrozenShapeException(string? message) : base(message) { }
}

/// <summary>
/// An attempt was made to modify a geometry of a shape already shared or protected: the shape is Locked, which in
/// the port includes every frozen shape (docs/phase3-design.md, section 4.3).
/// </summary>
public class LockedShapeException : DomainErrorException
{
    public LockedShapeException() { }
    public LockedShapeException(string? message) : base(message) { }
}

/// <summary>An attempt was made to add a shape to a shape of a type that can't contain it.</summary>
public class IncompatibleShapesException : DomainErrorException
{
    public IncompatibleShapesException() { }
    public IncompatibleShapesException(string? message) : base(message) { }
}

internal static class Raise
{
    internal static void TopoDS_FrozenShape_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new FrozenShapeException(message);
        }
    }
}
