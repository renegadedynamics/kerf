// Port of: src/ModelingData/TKBRep/TopoDS/DomainErrorException.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/ImmutableObjectException.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_UnCompatibleShapes.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Topology;

/// <summary>An attempt was made to add a shape to a shape of a type that can't contain it.</summary>
internal class IncompatibleShapesException : DomainErrorException
{
    public IncompatibleShapesException() { }
    public IncompatibleShapesException(string? message) : base(message) { }
}

internal static class Raise
{
    internal static void DomainError_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new DomainErrorException(message);
        }
    }
}
