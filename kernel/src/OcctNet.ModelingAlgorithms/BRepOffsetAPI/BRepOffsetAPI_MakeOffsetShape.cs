// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeOffsetShape.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeOffsetShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepOffset_Mode;
using static Kerf.Geometry.JoinKind;

namespace Occt;

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
/// A maker (decision 414): <see cref="BRepBuilderAPI_MakeShape.Shape"/>, <see cref="BRepBuilderAPI_MakeShape.Generated"/>
/// and <see cref="BRepBuilderAPI_MakeShape.Modified"/> hand out frozen shapes. The caller's shape is never written:
/// both algorithms copy it at the entry where they would write into it (decision 1005), and the history answers for
/// the caller's sub-shapes. As in OCCT, a run reported done may give a null shape.
/// </para>
/// </remarks>
public class BRepOffsetAPI_MakeOffsetShape : BRepBuilderAPI_MakeShape
{
    protected enum OffsetAlgo_Type
    {
        OffsetAlgo_NONE,
        OffsetAlgo_JOIN,
        OffsetAlgo_SIMPLE,
    }

    protected OffsetAlgo_Type myLastUsedAlgo;

    protected BRepOffset_MakeOffset myOffsetShape = new();
    protected BRepOffset_MakeSimpleOffset mySimpleOffsetShape = new();

    //=================================================================================================

    /// <summary>Constructor does nothing.</summary>
    public BRepOffsetAPI_MakeOffsetShape()
    {
        myLastUsedAlgo = OffsetAlgo_Type.OffsetAlgo_NONE;
    }

    //=================================================================================================

    /// <summary>
    /// Constructs a shape parallel to <paramref name="S"/> (a face, a shell, a solid or a compound of these): outside S
    /// for a positive <paramref name="Offset"/>, inside for a negative one. <paramref name="Tol"/> is the coincidence
    /// tolerance; <paramref name="Intersection"/> intersects all the parallels, not only those of adjacent faces;
    /// <paramref name="SelfInter"/> is not implemented; <paramref name="Join"/> fills the gaps between parallels with
    /// pipes and spheres (GeomAbs_Arc) or by extending and intersecting them (GeomAbs_Intersection);
    /// <paramref name="RemoveIntEdges"/> removes the INTERNAL edges from the result.
    /// </summary>
    public void PerformByJoin(in Shape S, double Offset, double Tol, BRepOffset_Mode Mode = BRepOffset_Skin,
                              bool Intersection = false, bool SelfInter = false, JoinKind Join = Arc,
                              bool RemoveIntEdges = false, ProgressRange theRange = default)
    {
        NotDone();
        myLastUsedAlgo = OffsetAlgo_Type.OffsetAlgo_JOIN;

        myOffsetShape.Initialize(S, Offset, Tol, Mode, Intersection, SelfInter, Join, false, RemoveIntEdges);
        myOffsetShape.MakeOffsetShape(theRange);

        if (!myOffsetShape.IsDone())
        {
            return;
        }

        myShape = myOffsetShape.InternalShape();
        Done();
    }

    //=================================================================================================

    /// <summary>Constructs the offset shape with the simple algorithm, without computing intersections.</summary>
    public void PerformBySimple(in Shape theS, double theOffsetValue)
    {
        NotDone();
        myLastUsedAlgo = OffsetAlgo_Type.OffsetAlgo_SIMPLE;

        mySimpleOffsetShape.Initialize(theS, theOffsetValue);
        mySimpleOffsetShape.Perform();

        if (!mySimpleOffsetShape.IsDone())
        {
            return;
        }

        myShape = mySimpleOffsetShape.InternalGetResultShape();
        Done();
    }

    //=================================================================================================

    /// <summary>The underlying intersection / arc algorithm.</summary>
    public virtual BRepOffset_MakeOffset MakeOffset() => myOffsetShape;

    //=================================================================================================

    /// <summary>Does nothing.</summary>
    public override void Build(ProgressRange theRange = default) { }

    //=================================================================================================

    internal override NCollection_List<Shape> InternalGenerated(in Shape S)
    {
        myGenerated.Clear();
        if (myLastUsedAlgo == OffsetAlgo_Type.OffsetAlgo_JOIN)
        {
            myGenerated.Assign(myOffsetShape.InternalGenerated(S));
        }
        else if (myLastUsedAlgo == OffsetAlgo_Type.OffsetAlgo_SIMPLE)
        {
            Shape aGenShape = mySimpleOffsetShape.InternalGenerated(S);
            if (!aGenShape.IsNull && !aGenShape.IsSame(S))
            {
                myGenerated.Append(aGenShape);
            }
        }

        return myGenerated;
    }

    //=================================================================================================

    internal override NCollection_List<Shape> InternalModified(in Shape S)
    {
        myGenerated.Clear();
        if (myLastUsedAlgo == OffsetAlgo_Type.OffsetAlgo_JOIN)
        {
            myGenerated.Assign(myOffsetShape.InternalModified(S));
        }
        else if (myLastUsedAlgo == OffsetAlgo_Type.OffsetAlgo_SIMPLE)
        {
            Shape aGenShape = mySimpleOffsetShape.InternalModified(S);
            if (!aGenShape.IsNull && !aGenShape.IsSame(S))
            {
                myGenerated.Append(aGenShape);
            }
        }

        return myGenerated;
    }

    //=================================================================================================

    /// <summary>True if the shape has been removed from the result.</summary>
    public override bool IsDeleted(in Shape S)
    {
        if (myLastUsedAlgo == OffsetAlgo_Type.OffsetAlgo_JOIN)
        {
            return myOffsetShape.IsDeleted(S);
        }
        return false;
    }

    //=================================================================================================

    /// <summary>The offset join type.</summary>
    public JoinKind GetJoinType() => myOffsetShape.GetJoinType();
}
