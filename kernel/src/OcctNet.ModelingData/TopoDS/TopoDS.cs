// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// Narrows a <see cref="TopoDS_Shape"/> to one of the eight shape types, checking the type: <c>TopoDS.Edge(S)</c>
/// throws <see cref="Standard_TypeMismatch"/> unless S is null or an edge.
/// </summary>
public static class TopoDS
{
    /// <summary>Casts <paramref name="theShape"/> to a TopoDS_Vertex.</summary>
    /// <exception cref="Standard_TypeMismatch">The shape is neither null nor of type VERTEX.</exception>
    public static TopoDS_Vertex Vertex(in TopoDS_Shape theShape)
    {
        Standard_TypeMismatch_Raise_if(theShape.IsNull() ? false : theShape.ShapeType() != TopAbs_ShapeEnum.TopAbs_VERTEX,
                                       "TopoDS::Vertex");
        return new TopoDS_Vertex(theShape.TShape(), theShape.Location(), theShape.Orientation());
    }

    /// <summary>Casts <paramref name="theShape"/> to a TopoDS_Edge.</summary>
    /// <exception cref="Standard_TypeMismatch">The shape is neither null nor of type EDGE.</exception>
    public static TopoDS_Edge Edge(in TopoDS_Shape theShape)
    {
        Standard_TypeMismatch_Raise_if(theShape.IsNull() ? false : theShape.ShapeType() != TopAbs_ShapeEnum.TopAbs_EDGE,
                                       "TopoDS::Edge");
        return new TopoDS_Edge(theShape.TShape(), theShape.Location(), theShape.Orientation());
    }

    /// <summary>Casts <paramref name="theShape"/> to a TopoDS_Wire.</summary>
    /// <exception cref="Standard_TypeMismatch">The shape is neither null nor of type WIRE.</exception>
    public static TopoDS_Wire Wire(in TopoDS_Shape theShape)
    {
        Standard_TypeMismatch_Raise_if(theShape.IsNull() ? false : theShape.ShapeType() != TopAbs_ShapeEnum.TopAbs_WIRE,
                                       "TopoDS::Wire");
        return new TopoDS_Wire(theShape.TShape(), theShape.Location(), theShape.Orientation());
    }

    /// <summary>Casts <paramref name="theShape"/> to a TopoDS_Face.</summary>
    /// <exception cref="Standard_TypeMismatch">The shape is neither null nor of type FACE.</exception>
    public static TopoDS_Face Face(in TopoDS_Shape theShape)
    {
        Standard_TypeMismatch_Raise_if(theShape.IsNull() ? false : theShape.ShapeType() != TopAbs_ShapeEnum.TopAbs_FACE,
                                       "TopoDS::Face");
        return new TopoDS_Face(theShape.TShape(), theShape.Location(), theShape.Orientation());
    }

    /// <summary>Casts <paramref name="theShape"/> to a TopoDS_Shell.</summary>
    /// <exception cref="Standard_TypeMismatch">The shape is neither null nor of type SHELL.</exception>
    public static TopoDS_Shell Shell(in TopoDS_Shape theShape)
    {
        Standard_TypeMismatch_Raise_if(theShape.IsNull() ? false : theShape.ShapeType() != TopAbs_ShapeEnum.TopAbs_SHELL,
                                       "TopoDS::Shell");
        return new TopoDS_Shell(theShape.TShape(), theShape.Location(), theShape.Orientation());
    }

    /// <summary>Casts <paramref name="theShape"/> to a TopoDS_Solid.</summary>
    /// <exception cref="Standard_TypeMismatch">The shape is neither null nor of type SOLID.</exception>
    public static TopoDS_Solid Solid(in TopoDS_Shape theShape)
    {
        Standard_TypeMismatch_Raise_if(theShape.IsNull() ? false : theShape.ShapeType() != TopAbs_ShapeEnum.TopAbs_SOLID,
                                       "TopoDS::Solid");
        return new TopoDS_Solid(theShape.TShape(), theShape.Location(), theShape.Orientation());
    }

    /// <summary>Casts <paramref name="theShape"/> to a TopoDS_CompSolid.</summary>
    /// <exception cref="Standard_TypeMismatch">The shape is neither null nor of type COMPSOLID.</exception>
    public static TopoDS_CompSolid CompSolid(in TopoDS_Shape theShape)
    {
        Standard_TypeMismatch_Raise_if(theShape.IsNull() ? false : theShape.ShapeType() != TopAbs_ShapeEnum.TopAbs_COMPSOLID,
                                       "TopoDS::CompSolid");
        return new TopoDS_CompSolid(theShape.TShape(), theShape.Location(), theShape.Orientation());
    }

    /// <summary>Casts <paramref name="theShape"/> to a TopoDS_Compound.</summary>
    /// <exception cref="Standard_TypeMismatch">The shape is neither null nor of type COMPOUND.</exception>
    public static TopoDS_Compound Compound(in TopoDS_Shape theShape)
    {
        Standard_TypeMismatch_Raise_if(theShape.IsNull() ? false : theShape.ShapeType() != TopAbs_ShapeEnum.TopAbs_COMPOUND,
                                       "TopoDS::Compound");
        return new TopoDS_Compound(theShape.TShape(), theShape.Location(), theShape.Orientation());
    }

    // OCCT.NET: C++ passes a derived variable to a non-const TopoDS_Shape& and sees the change in it. C# can't pass
    // a ref through the implicit conversion, so ported code writes TopoDS.AsShapeRef(ref e) there. The subtypes have
    // TopoDS_Shape's exact fields, so the reinterpretation is safe (docs/phase3-design.md, section 2).

    /// <summary>A TopoDS_Shape reference to the TopoDS_Vertex variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref TopoDS_Shape AsShapeRef(ref TopoDS_Vertex theShape) => ref Unsafe.As<TopoDS_Vertex, TopoDS_Shape>(ref theShape);

    /// <summary>A TopoDS_Shape reference to the TopoDS_Edge variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref TopoDS_Shape AsShapeRef(ref TopoDS_Edge theShape) => ref Unsafe.As<TopoDS_Edge, TopoDS_Shape>(ref theShape);

    /// <summary>A TopoDS_Shape reference to the TopoDS_Wire variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref TopoDS_Shape AsShapeRef(ref TopoDS_Wire theShape) => ref Unsafe.As<TopoDS_Wire, TopoDS_Shape>(ref theShape);

    /// <summary>A TopoDS_Shape reference to the TopoDS_Face variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref TopoDS_Shape AsShapeRef(ref TopoDS_Face theShape) => ref Unsafe.As<TopoDS_Face, TopoDS_Shape>(ref theShape);

    /// <summary>A TopoDS_Shape reference to the TopoDS_Shell variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref TopoDS_Shape AsShapeRef(ref TopoDS_Shell theShape) => ref Unsafe.As<TopoDS_Shell, TopoDS_Shape>(ref theShape);

    /// <summary>A TopoDS_Shape reference to the TopoDS_Solid variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref TopoDS_Shape AsShapeRef(ref TopoDS_Solid theShape) => ref Unsafe.As<TopoDS_Solid, TopoDS_Shape>(ref theShape);

    /// <summary>A TopoDS_Shape reference to the TopoDS_CompSolid variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref TopoDS_Shape AsShapeRef(ref TopoDS_CompSolid theShape) => ref Unsafe.As<TopoDS_CompSolid, TopoDS_Shape>(ref theShape);

    /// <summary>A TopoDS_Shape reference to the TopoDS_Compound variable, for a C++ non-const TopoDS_Shape&amp; argument.</summary>
    internal static ref TopoDS_Shape AsShapeRef(ref TopoDS_Compound theShape) => ref Unsafe.As<TopoDS_Compound, TopoDS_Shape>(ref theShape);
}
