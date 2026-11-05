// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_HShape.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_HShape.lxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_HShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;

namespace Kerf.Engine;

/// <summary>A shape held by reference (OCCT's handle to a shape), as a working object.</summary>
internal class TopoDS_HShape : Transient
{
    private Shape _shape;

    /// <summary>A holder of a null shape.</summary>
    public TopoDS_HShape() => _shape = new Shape();

    /// <summary>A holder of <paramref name="aShape"/>.</summary>
    public TopoDS_HShape(in Shape aShape) => _shape = aShape;

    /// <summary>Loads <paramref name="aShape"/>.</summary>
    public void Shape(in Shape aShape) => _shape = aShape;

    /// <summary>The shape.</summary>
    public Shape Shape() => _shape;

    /// <summary>A reference to the stored shape.</summary>
    internal ref Shape ChangeShape() => ref _shape;
}
