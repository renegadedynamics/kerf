// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_HShape.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_HShape.lxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_HShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A shape held by reference (OCCT's handle to a shape), as a working object.</summary>
public class TopoDS_HShape : Standard_Transient
{
    private TopoDS_Shape myShape;

    /// <summary>A holder of a null shape.</summary>
    public TopoDS_HShape() => myShape = new TopoDS_Shape();

    /// <summary>A holder of <paramref name="aShape"/>.</summary>
    public TopoDS_HShape(in TopoDS_Shape aShape) => myShape = aShape;

    /// <summary>Loads <paramref name="aShape"/>.</summary>
    public void Shape(in TopoDS_Shape aShape) => myShape = aShape;

    /// <summary>The shape.</summary>
    public TopoDS_Shape Shape() => myShape;

    /// <summary>A reference to the stored shape.</summary>
    internal ref TopoDS_Shape ChangeShape() => ref myShape;
}
