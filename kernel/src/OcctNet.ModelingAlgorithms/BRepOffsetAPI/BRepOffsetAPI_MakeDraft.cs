// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeDraft.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeDraft.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepBuilderAPI_TransitionMode;
using static Occt.BRepFill_TransitionStyle;

namespace Occt;

/// <summary>
/// A draft surface (<see cref="BRepFill_Draft"/>): a ruled skin swept from a wire, a face's outer wire or a shell's
/// free boundary along a direction at an angle, up to a length, a surface or a shape, sewn to the face or shell.
/// </summary>
/// <remarks>A maker (decision 414): the input is only read; <see cref="BRepBuilderAPI_MakeShape.Shape"/> freezes.</remarks>
public sealed class BRepOffsetAPI_MakeDraft : BRepBuilderAPI_MakeShape
{
    private readonly BRepFill_Draft myDraft;

    /// <summary>The draft of <paramref name="Shape"/> (a wire, face or shell) along <paramref name="Dir"/> at <paramref name="Angle"/>.</summary>
    /// <exception cref="ConstructionErrorException">Another shape type, or a shell without a free boundary wire.</exception>
    public BRepOffsetAPI_MakeDraft(in Shape Shape, in Direction3D Dir, double Angle)
    {
        myDraft = new BRepFill_Draft(Shape, Dir, Angle);
        NotDone();
    }

    /// <summary>The transition at the wire's corners (right or round) and the angular control of the sweep.</summary>
    public void SetOptions(BRepBuilderAPI_TransitionMode Style = BRepBuilderAPI_RightCorner, double AngleMin = 0.01,
                           double AngleMax = 3.0)
    {
        BRepFill_TransitionStyle style = BRepFill_Right;
        if (Style == BRepBuilderAPI_RoundCorner)
        {
            style = BRepFill_Round;
        }
        myDraft.SetOptions(style, AngleMin, AngleMax);
    }

    /// <summary>True for an internal draft.</summary>
    public void SetDraft(bool IsInternal = false) => myDraft.SetDraft(IsInternal);

    /// <summary>Builds the draft up to the length <paramref name="LengthMax"/>.</summary>
    public void Perform(double LengthMax)
    {
        myDraft.Perform(LengthMax);
        if (myDraft.IsDone())
        {
            Done();
            myShape = myDraft.InternalShape();
        }
    }

    /// <summary>Builds the draft up to the surface <paramref name="Surface"/>.</summary>
    public void Perform(Surface Surface, bool KeepInsideSurface = true)
    {
        myDraft.Perform(Surface, KeepInsideSurface);
        if (myDraft.IsDone())
        {
            Done();
            myShape = myDraft.InternalShape();
        }
    }

    /// <summary>Builds the draft up to the shape <paramref name="StopShape"/>.</summary>
    public void Perform(in Shape StopShape, bool KeepOutSide = true)
    {
        myDraft.Perform(StopShape, KeepOutSide);
        if (myDraft.IsDone())
        {
            Done();
            myShape = myDraft.InternalShape();
        }
    }

    /// <summary>The draft skin alone (frozen).</summary>
    public Shell Shell() => myDraft.Shell();

    internal override NCollection_List<Shape> InternalGenerated(in Shape S) => myDraft.InternalGenerated(S);
}
