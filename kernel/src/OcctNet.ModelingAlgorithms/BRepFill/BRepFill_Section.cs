// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Section.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Section.lxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Section.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

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
    private TopoDS_Shape myOriginalShape;
    private TopoDS_Wire wire;
    private TopoDS_Vertex vertex;
    private bool islaw;
    private bool ispunctual;
    private bool contact;
    private bool correction;

    /// <summary>An empty section.</summary>
    public BRepFill_Section()
    {
        myOriginalShape = new TopoDS_Shape();
        wire = new TopoDS_Wire();
        vertex = new TopoDS_Vertex();
        islaw = false;
        ispunctual = false;
        contact = false;
        correction = false;
    }

    /// <summary>The section <paramref name="Profile"/> (a wire or a vertex) placed on the spine's vertex <paramref name="V"/>.</summary>
    /// <exception cref="Standard_Failure">The profile is neither a wire nor a vertex.</exception>
    public BRepFill_Section(in TopoDS_Shape Profile, in TopoDS_Vertex V, bool WithContact, bool WithCorrection)
    {
        vertex = V;
        islaw = false;
        ispunctual = false;
        contact = WithContact;
        correction = WithCorrection;
        wire = new TopoDS_Wire();
        myOriginalShape = Profile;

        ShapeUpgrade_RemoveLocations RemLoc = new();
        RemLoc.SetRemoveLevel(TopAbs_COMPOUND);
        RemLoc.Remove(Profile);
        TopoDS_Shape aProfile = RemLoc.InternalGetResult();

        if (aProfile.ShapeType() == TopAbs_WIRE)
        {
            wire = TopoDS.Wire(aProfile);
        }
        else if (aProfile.ShapeType() == TopAbs_VERTEX)
        {
            ispunctual = true;
            TopoDS_Vertex aVertex = TopoDS.Vertex(aProfile);
            BRep_Builder BB = new();

            TopoDS_Edge DegEdge = new();
            BB.MakeEdge(ref DegEdge);
            BB.Add(DegEdge, aVertex.Oriented(TopAbs_FORWARD));
            BB.Add(DegEdge, aVertex.Oriented(TopAbs_REVERSED));
            BB.Degenerated(DegEdge, true);

            BB.MakeWire(ref wire);
            BB.Add(wire, DegEdge);
            wire.Closed(true);
        }
        else
        {
            throw new Standard_Failure("BRepFill_Section: bad shape type of section");
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
    public readonly TopoDS_Shape OriginalShape() => myOriginalShape;

    /// <summary>The profile's wire without locations, frozen.</summary>
    public readonly TopoDS_Wire Wire()
    {
        wire.TShape()?.FreezeGraph();
        return wire;
    }

    /// <summary><see cref="Wire"/> without freezing it, for the sweeps that build on it.</summary>
    internal readonly TopoDS_Wire InternalWire() => wire;

    /// <summary>The spine's vertex the section is placed on.</summary>
    public readonly TopoDS_Vertex Vertex() => vertex;

    /// <summary>
    /// The sub-shape of <see cref="Wire"/> made from <paramref name="theShape"/> (the profile, one of its edges or
    /// vertices), frozen; null otherwise.
    /// </summary>
    public readonly TopoDS_Shape ModifiedShape(in TopoDS_Shape theShape)
    {
        wire.TShape()?.FreezeGraph();
        return InternalModifiedShape(theShape);
    }

    /// <summary><see cref="ModifiedShape"/> without freezing the wire, for ported callers.</summary>
    internal readonly TopoDS_Shape InternalModifiedShape(in TopoDS_Shape theShape)
    {
        TopoDS_Shape aModifiedShape = new();

        switch (theShape.ShapeType())
        {
            case TopAbs_WIRE:
                if (theShape.IsSame(myOriginalShape))
                {
                    aModifiedShape = wire;
                }
                break;
            case TopAbs_EDGE:
                {
                    TopoDS_Iterator itor = new(myOriginalShape);
                    TopoDS_Iterator itw = new(wire);
                    for (; itor.More(); itor.Next(), itw.Next())
                    {
                        TopoDS_Shape anOriginalEdge = itor.Value();
                        TopoDS_Shape anEdge = itw.Value();
                        if (anOriginalEdge.IsSame(theShape))
                        {
                            aModifiedShape = anEdge;
                            break;
                        }
                    }
                }
                break;
            case TopAbs_VERTEX:
                if (theShape.IsSame(myOriginalShape))
                {
                    TopExp_Explorer Explo = new(wire, TopAbs_VERTEX);
                    aModifiedShape = Explo.Current();
                }
                else
                {
                    TopExp_Explorer ExpOrig = new(myOriginalShape, TopAbs_VERTEX);
                    TopExp_Explorer ExpWire = new(wire, TopAbs_VERTEX);
                    for (; ExpOrig.More(); ExpOrig.Next(), ExpWire.Next())
                    {
                        TopoDS_Shape anOriginalVertex = ExpOrig.Current();
                        TopoDS_Shape aVertex = ExpWire.Current();
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
