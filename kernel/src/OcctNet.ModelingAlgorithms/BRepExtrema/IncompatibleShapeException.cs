// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_UnCompatibleShape.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>A parameter was asked of a solution whose support isn't of that kind (an edge or a face).</summary>
public class IncompatibleShapeException : DomainErrorException
{
    public IncompatibleShapeException() { }

    public IncompatibleShapeException(string? message) : base(message) { }
}
