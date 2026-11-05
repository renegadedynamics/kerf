// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeOffsetShape.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeOffsetShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Offsets.OffsetMode;
using static Kerf.Geometry.JoinKind;

namespace Kerf.Engine.Offsets.Api;

/// <summary>
/// Builds the offset shape of a shape: by join (<see cref="PerformByJoin"/>, BRepOffset_MakeOffset, with arcs or
/// intersections between the offset faces) or by the simple algorithm without intersections
/// (<see cref="PerformBySimple"/>, BRepOffset_MakeSimpleOffset).
/// </summary>
/// <remarks>
/// <para>
/// A positive offset goes outside the shape, a negative one inside. The surfaces of the shape must be at least C1. The
/// offset value should be small enough to avoid self-intersections; the algorithm may fail where more than three edges
/// meet at a vertex.
/// </para>
/// <para>
/// A maker (decision 414): <see cref="MakeShape.Shape"/>, <see cref="MakeShape.Generated"/>
/// and <see cref="MakeShape.Modified"/> hand out frozen shapes. The caller's shape is never written:
/// both algorithms run on it in a working overlay where they would write into it (unit 10H5), and the history answers
/// for the caller's sub-shapes. As in OCCT, a run reported done may give a null shape.
/// </para>
/// </remarks>
internal class MakeOffsetShape : MakeShape
{
    protected enum OffsetAlgo_Type
    {
        OffsetAlgo_NONE,
        OffsetAlgo_JOIN,
        OffsetAlgo_SIMPLE,
    }

    protected OffsetAlgo_Type _lastUsedAlgo;

    protected MakeOffset _offsetShape = new();
    protected MakeSimpleOffset _simpleOffsetShape = new();

    //=================================================================================================

    /// <summary>Constructor does nothing.</summary>
    public MakeOffsetShape()
    {
        _lastUsedAlgo = OffsetAlgo_Type.OffsetAlgo_NONE;
    }

    //=================================================================================================

    /// <summary>
    /// Constructs a shape parallel to <paramref name="s"/> (a face, a shell, a solid or a compound of these): outside S
    /// for a positive <paramref name="offset"/>, inside for a negative one. <paramref name="tol"/> is the coincidence
    /// tolerance; <paramref name="intersection"/> intersects all the parallels, not only those of adjacent faces;
    /// <paramref name="selfInter"/> is not implemented; <paramref name="Join"/> fills the gaps between parallels with
    /// pipes and spheres (GeomAbs_Arc) or by extending and intersecting them (GeomAbs_Intersection);
    /// <paramref name="removeIntEdges"/> removes the INTERNAL edges from the result.
    /// </summary>
    public void PerformByJoin(in Shape s, double offset, double tol, global::Kerf.Offsets.OffsetMode mode = global::Kerf.Offsets.OffsetMode.Skin,
                              bool intersection = false, bool selfInter = false, JoinKind Join = Arc,
                              bool removeIntEdges = false, ProgressRange range = default)
    {
        NotDone();
        _lastUsedAlgo = OffsetAlgo_Type.OffsetAlgo_JOIN;

        _offsetShape.Initialize(s, offset, tol, mode, intersection, selfInter, Join, false, removeIntEdges);
        _offsetShape.MakeOffsetShape(range);

        if (!_offsetShape.IsDone)
        {
            return;
        }

        _shape = _offsetShape.InternalShape();
        Done();
    }

    //=================================================================================================

    /// <summary>Constructs the offset shape with the simple algorithm, without computing intersections.</summary>
    public void PerformBySimple(in Shape s, double offsetValue)
    {
        NotDone();
        _lastUsedAlgo = OffsetAlgo_Type.OffsetAlgo_SIMPLE;

        _simpleOffsetShape.Initialize(s, offsetValue);
        _simpleOffsetShape.Perform();

        if (!_simpleOffsetShape.IsDone)
        {
            return;
        }

        _shape = _simpleOffsetShape.InternalGetResultShape();
        Done();
    }

    //=================================================================================================

    /// <summary>The underlying intersection / arc algorithm.</summary>
    public virtual MakeOffset MakeOffset() => _offsetShape;

    //=================================================================================================

    /// <summary>Does nothing.</summary>
    public override void Build(ProgressRange range = default) { }

    //=================================================================================================

    internal override NCollection_List<Shape> InternalGenerated(in Shape s)
    {
        _generated.Clear();
        if (_lastUsedAlgo == OffsetAlgo_Type.OffsetAlgo_JOIN)
        {
            _generated.Assign(_offsetShape.InternalGenerated(s));
        }
        else if (_lastUsedAlgo == OffsetAlgo_Type.OffsetAlgo_SIMPLE)
        {
            Shape genShape = _simpleOffsetShape.InternalGenerated(s);
            if (!genShape.IsNull && !genShape.IsSame(s))
            {
                _generated.Append(genShape);
            }
        }

        return _generated;
    }

    //=================================================================================================

    internal override NCollection_List<Shape> InternalModified(in Shape s)
    {
        _generated.Clear();
        if (_lastUsedAlgo == OffsetAlgo_Type.OffsetAlgo_JOIN)
        {
            _generated.Assign(_offsetShape.InternalModified(s));
        }
        else if (_lastUsedAlgo == OffsetAlgo_Type.OffsetAlgo_SIMPLE)
        {
            Shape genShape = _simpleOffsetShape.InternalModified(s);
            if (!genShape.IsNull && !genShape.IsSame(s))
            {
                _generated.Append(genShape);
            }
        }

        return _generated;
    }

    //=================================================================================================

    /// <summary>True if the shape has been removed from the result.</summary>
    public override bool IsDeleted(in Shape s)
    {
        if (_lastUsedAlgo == OffsetAlgo_Type.OffsetAlgo_JOIN)
        {
            return _offsetShape.IsDeleted(s);
        }
        return false;
    }

    //=================================================================================================

    /// <summary>The offset join type.</summary>
    public JoinKind GetJoinType() => _offsetShape.GetJoinType();
}
