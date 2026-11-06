// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Section.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Section.lxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Section.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Construction.Filling;

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
internal partial struct FillingSection : IEquatable<FillingSection>
{
    private Shape _originalShape;
    private Wire _wire;
    private Vertex _vertex;
    private bool _islaw;
    private bool _ispunctual;
    private bool _contact;
    private bool _correction;

    /// <summary>An empty section.</summary>
    public FillingSection()
    {
        _originalShape = new Shape();
        _wire = new Wire();
        _vertex = new Vertex();
        _islaw = false;
        _ispunctual = false;
        _contact = false;
        _correction = false;
    }

    /// <summary>The section <paramref name="profile"/> (a wire or a vertex) placed on the spine's vertex <paramref name="v"/>.</summary>
    /// <exception cref="FailureException">The profile is neither a wire nor a vertex.</exception>
    public FillingSection(in Shape profile, in Vertex v, bool withContact, bool withCorrection)
    {
        _vertex = v;
        _islaw = false;
        _ispunctual = false;
        _contact = withContact;
        _correction = withCorrection;
        _wire = new Wire();
        _originalShape = profile;

        RemoveLocations remLoc = new();
        remLoc.SetRemoveLevel(ShapeType.Compound);
        remLoc.Remove(profile);
        Shape aProfile = remLoc.InternalGetResult();

        if (aProfile.Type == global::Kerf.Topology.ShapeType.Wire)
        {
            _wire = ShapeCasts.Wire(aProfile);
        }
        else if (aProfile.Type == global::Kerf.Topology.ShapeType.Vertex)
        {
            _ispunctual = true;
            Vertex vertex = ShapeCasts.Vertex(aProfile);
            BrepBuilder bb = new();

            Edge degEdge = new();
            bb.MakeEdge(ref degEdge);
            bb.Add(degEdge, vertex.Oriented(Forward));
            bb.Add(degEdge, vertex.Oriented(Reversed));
            bb.Degenerated(degEdge, true);

            bb.MakeWire(ref _wire);
            bb.Add(_wire, degEdge);
            _wire.Closed(true);
        }
        else
        {
            throw new FailureException("BRepFill_Section: bad shape type of section");
        }
    }

    /// <summary>Sets whether the section is scaled by a law.</summary>
    internal void Set(bool isLaw)
    {
        _islaw = isLaw;
    }

    /// <summary>This section with <see cref="IsLaw"/> set to <paramref name="isLaw"/>.</summary>
    public readonly FillingSection WithIsLaw(bool isLaw)
    {
        FillingSection copy = this;
        copy.Set(isLaw);
        return copy;
    }

    /// <summary>The profile as given.</summary>
    public readonly Shape OriginalShape() => _originalShape;

    /// <summary>The profile's wire without locations, frozen.</summary>
    public readonly Wire Wire()
    {
        _wire.TShape()?.FreezeGraph();
        return _wire;
    }

    /// <summary><see cref="Wire"/> without freezing it, for the sweeps that build on it.</summary>
    internal readonly Wire InternalWire() => _wire;

    /// <summary>The spine's vertex the section is placed on.</summary>
    public readonly Vertex Vertex() => _vertex;

    /// <summary>
    /// The sub-shape of <see cref="Wire"/> made from <paramref name="shape"/> (the profile, one of its edges or
    /// vertices), frozen; null otherwise.
    /// </summary>
    public readonly Shape ModifiedShape(in Shape shape)
    {
        _wire.TShape()?.FreezeGraph();
        return InternalModifiedShape(shape);
    }

    /// <summary><see cref="ModifiedShape"/> without freezing the wire, for ported callers.</summary>
    internal readonly Shape InternalModifiedShape(in Shape shape)
    {
        Shape modifiedShape = new();

        switch (shape.Type)
        {
            case global::Kerf.Topology.ShapeType.Wire:
                if (shape.IsSame(_originalShape))
                {
                    modifiedShape = _wire;
                }
                break;
            case ShapeType.Edge:
                {
                    Iterator itor = new(_originalShape);
                    Iterator itw = new(_wire);
                    for (; itor.More(); itor.Next(), itw.Next())
                    {
                        Shape originalEdge = itor.Value();
                        Shape edge = itw.Value();
                        if (originalEdge.IsSame(shape))
                        {
                            modifiedShape = edge;
                            break;
                        }
                    }
                }
                break;
            case global::Kerf.Topology.ShapeType.Vertex:
                if (shape.IsSame(_originalShape))
                {
                    Explorer explo = new(_wire, global::Kerf.Topology.ShapeType.Vertex);
                    modifiedShape = explo.Current();
                }
                else
                {
                    Explorer expOrig = new(_originalShape, global::Kerf.Topology.ShapeType.Vertex);
                    Explorer expWire = new(_wire, global::Kerf.Topology.ShapeType.Vertex);
                    for (; expOrig.More(); expOrig.Next(), expWire.Next())
                    {
                        Shape originalVertex = expOrig.Current();
                        Shape vertex = expWire.Current();
                        if (originalVertex.IsSame(shape))
                        {
                            modifiedShape = vertex;
                            break;
                        }
                    }
                }
                break;
            default:
                break;
        }

        return modifiedShape;
    }

    /// <summary>True if the section is scaled by a law.</summary>
    public readonly bool IsLaw => _islaw;

    /// <summary>True if the profile is a vertex.</summary>
    public readonly bool IsPunctual => _ispunctual;

    /// <summary>True if the section is translated to touch the spine.</summary>
    public readonly bool WithContact() => _contact;

    /// <summary>True if the section is rotated to be orthogonal to the spine's tangent.</summary>
    public readonly bool WithCorrection() => _correction;

    /// <inheritdoc/>
    public readonly bool Equals(FillingSection other) =>
        _originalShape.Equals(other._originalShape) && _wire.Equals(other._wire) && _vertex.Equals(other._vertex)
        && _islaw == other._islaw && _ispunctual == other._ispunctual && _contact == other._contact
        && _correction == other._correction;

    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is FillingSection section && Equals(section);

    /// <inheritdoc/>
    public override readonly int GetHashCode() =>
        HashCode.Combine(_originalShape, _wire, _vertex, _islaw, _ispunctual, _contact, _correction);

    /// <summary>Exact equality.</summary>
    public static bool operator ==(in FillingSection left, in FillingSection right) => left.Equals(right);

    /// <summary>Exact inequality.</summary>
    public static bool operator !=(in FillingSection left, in FillingSection right) => !left.Equals(right);
}
