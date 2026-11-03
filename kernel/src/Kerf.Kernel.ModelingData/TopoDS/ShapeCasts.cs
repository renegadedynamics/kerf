// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Topology;

/// <summary>
/// Narrows a <see cref="Shape"/> to one of the eight shape types, checking the type: <c>TopoDS.Edge(S)</c>
/// throws <see cref="TypeMismatchException"/> unless S is null or an edge.
/// </summary>
public static class ShapeCasts
{
    /// <summary>Casts <paramref name="shape"/> to a TopoDS_Vertex.</summary>
    /// <exception cref="TypeMismatchException">The shape is neither null nor of type VERTEX.</exception>
    public static Vertex Vertex(in Shape shape)
    {
        Standard_TypeMismatch_Raise_if(shape.IsNull ? false : shape.ShapeType() != ShapeType.Vertex,
                                       "TopoDS::Vertex");
        return new Vertex(shape.TShape(), shape.Location(), shape.Orientation());
    }

    /// <summary>Casts <paramref name="shape"/> to a TopoDS_Edge.</summary>
    /// <exception cref="TypeMismatchException">The shape is neither null nor of type EDGE.</exception>
    public static Edge Edge(in Shape shape)
    {
        Standard_TypeMismatch_Raise_if(shape.IsNull ? false : shape.ShapeType() != ShapeType.Edge,
                                       "TopoDS::Edge");
        return new Edge(shape.TShape(), shape.Location(), shape.Orientation());
    }

    /// <summary>Casts <paramref name="shape"/> to a TopoDS_Wire.</summary>
    /// <exception cref="TypeMismatchException">The shape is neither null nor of type WIRE.</exception>
    public static Wire Wire(in Shape shape)
    {
        Standard_TypeMismatch_Raise_if(shape.IsNull ? false : shape.ShapeType() != ShapeType.Wire,
                                       "TopoDS::Wire");
        return new Wire(shape.TShape(), shape.Location(), shape.Orientation());
    }

    /// <summary>Casts <paramref name="shape"/> to a TopoDS_Face.</summary>
    /// <exception cref="TypeMismatchException">The shape is neither null nor of type FACE.</exception>
    public static Face Face(in Shape shape)
    {
        Standard_TypeMismatch_Raise_if(shape.IsNull ? false : shape.ShapeType() != ShapeType.Face,
                                       "TopoDS::Face");
        return new Face(shape.TShape(), shape.Location(), shape.Orientation());
    }

    /// <summary>Casts <paramref name="shape"/> to a TopoDS_Shell.</summary>
    /// <exception cref="TypeMismatchException">The shape is neither null nor of type SHELL.</exception>
    public static Shell Shell(in Shape shape)
    {
        Standard_TypeMismatch_Raise_if(shape.IsNull ? false : shape.ShapeType() != ShapeType.Shell,
                                       "TopoDS::Shell");
        return new Shell(shape.TShape(), shape.Location(), shape.Orientation());
    }

    /// <summary>Casts <paramref name="shape"/> to a TopoDS_Solid.</summary>
    /// <exception cref="TypeMismatchException">The shape is neither null nor of type SOLID.</exception>
    public static Solid Solid(in Shape shape)
    {
        Standard_TypeMismatch_Raise_if(shape.IsNull ? false : shape.ShapeType() != ShapeType.Solid,
                                       "TopoDS::Solid");
        return new Solid(shape.TShape(), shape.Location(), shape.Orientation());
    }

    /// <summary>Casts <paramref name="shape"/> to a TopoDS_CompSolid.</summary>
    /// <exception cref="TypeMismatchException">The shape is neither null nor of type COMPSOLID.</exception>
    public static CompSolid CompSolid(in Shape shape)
    {
        Standard_TypeMismatch_Raise_if(shape.IsNull ? false : shape.ShapeType() != ShapeType.CompSolid,
                                       "TopoDS::CompSolid");
        return new CompSolid(shape.TShape(), shape.Location(), shape.Orientation());
    }

    /// <summary>Casts <paramref name="shape"/> to a TopoDS_Compound.</summary>
    /// <exception cref="TypeMismatchException">The shape is neither null nor of type COMPOUND.</exception>
    public static Compound Compound(in Shape shape)
    {
        Standard_TypeMismatch_Raise_if(shape.IsNull ? false : shape.ShapeType() != ShapeType.Compound,
                                       "TopoDS::Compound");
        return new Compound(shape.TShape(), shape.Location(), shape.Orientation());
    }

    // OCCT.NET: C++ passes a derived variable to a non-const TopoDS_Shape& and sees the change in it. C# can't pass
    // a ref through the implicit conversion, so ported code writes TopoDS.AsShapeRef(ref e) there. The subtypes have
    // TopoDS_Shape's exact fields, so the reinterpretation is safe (docs/phase3-design.md, section 2).

    /// <summary>A TopoDS_Shape reference to the TopoDS_Vertex variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref Shape AsShapeRef(ref Vertex shape) => ref Unsafe.As<Vertex, Shape>(ref shape);

    /// <summary>A TopoDS_Shape reference to the TopoDS_Edge variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref Shape AsShapeRef(ref Edge shape) => ref Unsafe.As<Edge, Shape>(ref shape);

    /// <summary>A TopoDS_Shape reference to the TopoDS_Wire variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref Shape AsShapeRef(ref Wire shape) => ref Unsafe.As<Wire, Shape>(ref shape);

    /// <summary>A TopoDS_Shape reference to the TopoDS_Face variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref Shape AsShapeRef(ref Face shape) => ref Unsafe.As<Face, Shape>(ref shape);

    /// <summary>A TopoDS_Shape reference to the TopoDS_Shell variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref Shape AsShapeRef(ref Shell shape) => ref Unsafe.As<Shell, Shape>(ref shape);

    /// <summary>A TopoDS_Shape reference to the TopoDS_Solid variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref Shape AsShapeRef(ref Solid shape) => ref Unsafe.As<Solid, Shape>(ref shape);

    /// <summary>A TopoDS_Shape reference to the TopoDS_CompSolid variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref Shape AsShapeRef(ref CompSolid shape) => ref Unsafe.As<CompSolid, Shape>(ref shape);

    /// <summary>A TopoDS_Shape reference to the TopoDS_Compound variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref Shape AsShapeRef(ref Compound shape) => ref Unsafe.As<Compound, Shape>(ref shape);
}
