// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Section.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Section.lxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Section.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The definition of a section of a pipe shell: the profile (a wire or a vertex) with its locations removed, the vertex
/// of the spine it is placed on, and the contact and correction options.
/// </summary>
/// <remarks>
/// <para>
/// A value type (rules, section 3; OCCT copies it by value in BRepFill_PipeShell's sequence of sections):
/// <see cref="Set"/> is internal with the twin <see cref="WithIsLaw"/>; equality compares the fields exactly (shapes
/// by TShape, location and orientation).
/// </para>
/// <para>
/// The profile is rebuilt without locations by ShapeUpgrade_RemoveLocations (new edges, the caller's geometry), or
/// becomes a closed wire of one degenerated edge for a vertex. <see cref="Wire"/> and <see cref="ModifiedShape"/>
/// freeze that wire before handing it out; ported callers that build on it (the sweeps) read
/// <see cref="InternalWire"/> and <see cref="InternalModifiedShape"/> (decision 903). The original shape and the
/// vertex are the caller's.
/// </para>
/// </remarks>
public partial struct BRepFill_Section : IEquatable<BRepFill_Section>
{
    private Shape myOriginalShape;
    private Wire wire;
    private Vertex vertex;
    private bool islaw;
    private bool ispunctual;
    private bool contact;
    private bool correction;

    /// <summary>An empty section.</summary>
    public BRepFill_Section()
    {
        myOriginalShape = new Shape();
        wire = new Wire();
        vertex = new Vertex();
        islaw = false;
        ispunctual = false;
        contact = false;
        correction = false;
    }

    /// <summary>The section <paramref name="Profile"/> (a wire or a vertex) placed on the spine's vertex <paramref name="V"/>.</summary>
    /// <exception cref="FailureException">The profile is neither a wire nor a vertex.</exception>
    public BRepFill_Section(in Shape Profile, in Vertex V, bool WithContact, bool WithCorrection)
    {
        vertex = V;
        islaw = false;
        ispunctual = false;
        contact = WithContact;
        correction = WithCorrection;
        wire = new Wire();
        myOriginalShape = Profile;

        ShapeUpgrade_RemoveLocations RemLoc = new();
        RemLoc.SetRemoveLevel(ShapeType.Compound);
        RemLoc.Remove(Profile);
        Shape aProfile = RemLoc.InternalGetResult();

        if (aProfile.ShapeType() == global::Kerf.Topology.ShapeType.Wire)
        {
            wire = ShapeCasts.Wire(aProfile);
        }
        else if (aProfile.ShapeType() == global::Kerf.Topology.ShapeType.Vertex)
        {
            ispunctual = true;
            Vertex aVertex = ShapeCasts.Vertex(aProfile);
            BrepBuilder BB = new();

            Edge DegEdge = new();
            BB.MakeEdge(ref DegEdge);
            BB.Add(DegEdge, aVertex.Oriented(Forward));
            BB.Add(DegEdge, aVertex.Oriented(Reversed));
            BB.Degenerated(DegEdge, true);

            BB.MakeWire(ref wire);
            BB.Add(wire, DegEdge);
            wire.Closed(true);
        }
        else
        {
            throw new FailureException("BRepFill_Section: bad shape type of section");
        }
    }

    /// <summary>Sets whether the section is scaled by a law.</summary>
    internal void Set(bool IsLaw)
    {
        islaw = IsLaw;
    }

    /// <summary>This section with <see cref="IsLaw"/> set to <paramref name="IsLaw"/>.</summary>
    public readonly BRepFill_Section WithIsLaw(bool IsLaw)
    {
        BRepFill_Section aCopy = this;
        aCopy.Set(IsLaw);
        return aCopy;
    }

    /// <summary>The profile as given.</summary>
    public readonly Shape OriginalShape() => myOriginalShape;

    /// <summary>The profile's wire without locations, frozen.</summary>
    public readonly Wire Wire()
    {
        wire.TShape()?.FreezeGraph();
        return wire;
    }

    /// <summary><see cref="Wire"/> without freezing it, for the sweeps that build on it.</summary>
    internal readonly Wire InternalWire() => wire;

    /// <summary>The spine's vertex the section is placed on.</summary>
    public readonly Vertex Vertex() => vertex;

    /// <summary>
    /// The sub-shape of <see cref="Wire"/> made from <paramref name="theShape"/> (the profile, one of its edges or
    /// vertices), frozen; null otherwise.
    /// </summary>
    public readonly Shape ModifiedShape(in Shape theShape)
    {
        wire.TShape()?.FreezeGraph();
        return InternalModifiedShape(theShape);
    }

    /// <summary><see cref="ModifiedShape"/> without freezing the wire, for ported callers.</summary>
    internal readonly Shape InternalModifiedShape(in Shape theShape)
    {
        Shape aModifiedShape = new();

        switch (theShape.ShapeType())
        {
            case global::Kerf.Topology.ShapeType.Wire:
                if (theShape.IsSame(myOriginalShape))
                {
                    aModifiedShape = wire;
                }
                break;
            case ShapeType.Edge:
                {
                    Iterator itor = new(myOriginalShape);
                    Iterator itw = new(wire);
                    for (; itor.More(); itor.Next(), itw.Next())
                    {
                        Shape anOriginalEdge = itor.Value();
                        Shape anEdge = itw.Value();
                        if (anOriginalEdge.IsSame(theShape))
                        {
                            aModifiedShape = anEdge;
                            break;
                        }
                    }
                }
                break;
            case global::Kerf.Topology.ShapeType.Vertex:
                if (theShape.IsSame(myOriginalShape))
                {
                    Explorer Explo = new(wire, global::Kerf.Topology.ShapeType.Vertex);
                    aModifiedShape = Explo.Current();
                }
                else
                {
                    Explorer ExpOrig = new(myOriginalShape, global::Kerf.Topology.ShapeType.Vertex);
                    Explorer ExpWire = new(wire, global::Kerf.Topology.ShapeType.Vertex);
                    for (; ExpOrig.More(); ExpOrig.Next(), ExpWire.Next())
                    {
                        Shape anOriginalVertex = ExpOrig.Current();
                        Shape aVertex = ExpWire.Current();
                        if (anOriginalVertex.IsSame(theShape))
                        {
                            aModifiedShape = aVertex;
                            break;
                        }
                    }
                }
                break;
            default:
                break;
        }

        return aModifiedShape;
    }

    /// <summary>True if the section is scaled by a law.</summary>
    public readonly bool IsLaw() => islaw;

    /// <summary>True if the profile is a vertex.</summary>
    public readonly bool IsPunctual() => ispunctual;

    /// <summary>True if the section is translated to touch the spine.</summary>
    public readonly bool WithContact() => contact;

    /// <summary>True if the section is rotated to be orthogonal to the spine's tangent.</summary>
    public readonly bool WithCorrection() => correction;

    /// <inheritdoc/>
    public readonly bool Equals(BRepFill_Section theOther) =>
        myOriginalShape.Equals(theOther.myOriginalShape) && wire.Equals(theOther.wire) && vertex.Equals(theOther.vertex)
        && islaw == theOther.islaw && ispunctual == theOther.ispunctual && contact == theOther.contact
        && correction == theOther.correction;

    /// <inheritdoc/>
    public override readonly bool Equals(object? theObj) => theObj is BRepFill_Section aSection && Equals(aSection);

    /// <inheritdoc/>
    public override readonly int GetHashCode() =>
        HashCode.Combine(myOriginalShape, wire, vertex, islaw, ispunctual, contact, correction);

    /// <summary>Exact equality.</summary>
    public static bool operator ==(in BRepFill_Section theLeft, in BRepFill_Section theRight) => theLeft.Equals(theRight);

    /// <summary>Exact inequality.</summary>
    public static bool operator !=(in BRepFill_Section theLeft, in BRepFill_Section theRight) => !theLeft.Equals(theRight);
}
