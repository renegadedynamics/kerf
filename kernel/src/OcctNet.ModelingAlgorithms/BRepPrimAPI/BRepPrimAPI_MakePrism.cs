// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakePrism.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakePrism.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Topology;

namespace Occt;

/// <summary>
/// Builds the linear swept topology of a shape (a prism): vertex to edge, edge to face, wire to shell, face to solid,
/// shell to compsolid, compound to compound; along a vector, or semi-infinite or infinite along a direction.
/// </summary>
/// <remarks>
/// <para>Without copy, OCCT shares the base's sub-shapes in the prism and writes pcurves and parameters into them. A
/// base holding frozen TShapes (any shape a caller has) is swept as an exact copy, and the result holds the base's own
/// sub-shapes wherever the sweep left them unchanged (<see cref="BRepPrimAPI_MakeSweep"/>); the history answers for
/// the caller's sub-shapes, and the base is never changed.</para>
/// <para>The result, <see cref="BRepPrimAPI_MakeSweep.FirstShape()"/>, <see cref="LastShape(in Shape)"/> and the
/// history are frozen when returned (design 4.2).</para>
/// </remarks>
public class BRepPrimAPI_MakePrism : BRepPrimAPI_MakeSweep
{
    private readonly BRepSweep_Prism myPrism;

    // perform checks on the argument
    private static Shape check(in Shape S)
    {
        BRepLib.BuildCurves3d(S);
        return S;
    }

    /// <summary>
    /// The prism of base <paramref name="S"/> and vector <paramref name="V"/>. With <paramref name="Copy"/>, S's
    /// sub-shapes are copied; with <paramref name="Canonize"/>, the extrusions of lines and circles are planes and
    /// cylinders.
    /// </summary>
    public BRepPrimAPI_MakePrism(in Shape S, in Vector3D V, bool Copy = false, bool Canonize = true)
    {
        myPrism = new BRepSweep_Prism(check(GeneratingShapeForWrite(S)), V, Copy, Canonize);
        Build();
    }

    /// <summary>
    /// A semi-infinite (<paramref name="Inf"/> false) or infinite prism of base <paramref name="S"/> along
    /// <paramref name="D"/>.
    /// </summary>
    public BRepPrimAPI_MakePrism(in Shape S, in Direction3D D, bool Inf = true, bool Copy = false, bool Canonize = true)
    {
        myPrism = new BRepSweep_Prism(check(GeneratingShapeForWrite(S)), D, Inf, Copy, Canonize);
        Build();
    }

    /// <summary>The internal sweeping algorithm (on the copy of a frozen base).</summary>
    internal BRepSweep_Prism Prism() => myPrism;

    /// <summary>Builds the resulting shape (redefined from MakeShape).</summary>
    public override void Build(ProgressRange theRange = default)
    {
        myShape = myPrism.Shape();
        KeepUnchangedGenerating(ref myShape);
        Done();
    }

    internal override Shape InternalFirstShape() => FromSwept(myPrism.FirstShape());

    internal override Shape InternalLastShape() => FromSwept(myPrism.LastShape());

    /// <summary>The list of shapes generated from the shape <paramref name="S"/>.</summary>
    internal override NCollection_List<Shape> InternalGenerated(in Shape S)
    {
        myGenerated.Clear();
        Shape aS = ToSwept(S);
        if (myPrism.IsUsed(aS) && myPrism.GenIsUsed(aS))
        {
            myGenerated.Append(FromSwept(myPrism.Shape(aS)));
        }
        return myGenerated;
    }

    /// <summary>The shape at the bottom of the prism generated with <paramref name="theShape"/>, a sub-shape of the base (frozen).</summary>
    public Shape FirstShape(in Shape theShape) => Published(InternalFirstShape(theShape));

    /// <summary><see cref="FirstShape(in Shape)"/> without freezing.</summary>
    internal Shape InternalFirstShape(in Shape theShape) => FromSwept(myPrism.FirstShape(ToSwept(theShape)));

    /// <summary>The shape at the top of the prism generated with <paramref name="theShape"/>, a sub-shape of the base (frozen).</summary>
    public Shape LastShape(in Shape theShape) => Published(InternalLastShape(theShape));

    /// <summary><see cref="LastShape(in Shape)"/> without freezing.</summary>
    internal Shape InternalLastShape(in Shape theShape) => FromSwept(myPrism.LastShape(ToSwept(theShape)));

    /// <summary>True if the shape <paramref name="S"/> has been deleted.</summary>
    public override bool IsDeleted(in Shape S)
    {
        return !myPrism.IsUsed(ToSwept(S));
    }
}
