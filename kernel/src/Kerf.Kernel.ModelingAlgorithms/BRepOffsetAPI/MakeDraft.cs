// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeDraft.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeDraft.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Construction.TransitionMode;
using static Kerf.Engine.Construction.Filling.TransitionStyle;

namespace Kerf.Engine.Features.Api;

/// <summary>
/// A draft surface (<see cref="Draft"/>): a ruled skin swept from a wire, a face's outer wire or a shell's
/// free boundary along a direction at an angle, up to a length, a surface or a shape, sewn to the face or shell.
/// </summary>
/// <remarks>A maker (decision 414): the input is only read; <see cref="MakeShape.Shape"/> freezes.</remarks>
internal sealed class MakeDraft : MakeShape
{
    private readonly global::Kerf.Engine.Construction.Filling.Draft _draft;

    /// <summary>The draft of <paramref name="shape"/> (a wire, face or shell) along <paramref name="dir"/> at <paramref name="angle"/>.</summary>
    /// <exception cref="ConstructionErrorException">Another shape type, or a shell without a free boundary wire.</exception>
    public MakeDraft(in Shape shape, in Direction3D dir, double angle)
    {
        _draft = new global::Kerf.Engine.Construction.Filling.Draft(shape, dir, angle);
        NotDone();
    }

    /// <summary>The transition at the wire's corners (right or round) and the angular control of the sweep.</summary>
    public void SetOptions(global::Kerf.Construction.TransitionMode Style = global::Kerf.Construction.TransitionMode.RightCorner, double angleMin = 0.01,
                           double angleMax = 3.0)
    {
        TransitionStyle style = TransitionStyle.Right;
        if (Style == global::Kerf.Construction.TransitionMode.RoundCorner)
        {
            style = TransitionStyle.Round;
        }
        _draft.SetOptions(style, angleMin, angleMax);
    }

    /// <summary>True for an internal draft.</summary>
    public void SetDraft(bool isInternal = false) => _draft.SetDraft(isInternal);

    /// <summary>Builds the draft up to the length <paramref name="lengthMax"/>.</summary>
    public void Perform(double lengthMax)
    {
        _draft.Perform(lengthMax);
        if (_draft.IsDone)
        {
            Done();
            _shape = _draft.InternalShape();
        }
    }

    /// <summary>Builds the draft up to the surface <paramref name="surface"/>.</summary>
    public void Perform(Surface surface, bool keepInsideSurface = true)
    {
        _draft.Perform(surface, keepInsideSurface);
        if (_draft.IsDone)
        {
            Done();
            _shape = _draft.InternalShape();
        }
    }

    /// <summary>Builds the draft up to the shape <paramref name="stopShape"/>.</summary>
    public void Perform(in Shape stopShape, bool keepOutSide = true)
    {
        _draft.Perform(stopShape, keepOutSide);
        if (_draft.IsDone)
        {
            Done();
            _shape = _draft.InternalShape();
        }
    }

    /// <summary>The draft skin alone (frozen).</summary>
    public Shell Shell() => _draft.Shell();

    internal override NCollection_List<Shape> InternalGenerated(in Shape s) => _draft.InternalGenerated(s);
}
