// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeThickSolid.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeThickSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepOffset_Mode;
using static Occt.GeomAbs_JoinType;

namespace Occt;

/// <summary>
/// Builds a thick solid (a hollowed solid, or a thickened shell): <see cref="MakeThickSolidByJoin"/> removes the closing
/// faces from the shape and offsets the rest, closing the gap with the shape itself; <see cref="MakeThickSolidBySimple"/>
/// offsets a shell with the simple algorithm and closes it with walls.
/// </summary>
/// <remarks>
/// <para>
/// A positive offset thickens outward, a negative one inward. The closing faces must belong to the shape. The surfaces
/// must be at least C1, and the offset small enough to avoid self-intersections.
/// </para>
/// <para>
/// A maker (decision 414), as <see cref="BRepOffsetAPI_MakeOffsetShape"/>: the caller's shape is never written (decision
/// 1005), and the result holds the caller's faces where the algorithm left them unchanged.
/// </para>
/// </remarks>
public class BRepOffsetAPI_MakeThickSolid : BRepOffsetAPI_MakeOffsetShape
{
    //=================================================================================================

    /// <summary>An empty algorithm.</summary>
    public BRepOffsetAPI_MakeThickSolid()
    {
        // Build only solids.
        mySimpleOffsetShape.SetBuildSolidFlag(true);
    }

    //=================================================================================================

    /// <summary>
    /// Makes a thick solid from <paramref name="S"/>: the faces <paramref name="ClosingFaces"/> are removed and the rest
    /// offset by <paramref name="Offset"/>, the gap closed by the shape's own faces. The other arguments are those of
    /// <see cref="BRepOffsetAPI_MakeOffsetShape.PerformByJoin"/>.
    /// </summary>
    public void MakeThickSolidByJoin(in TopoDS_Shape S, NCollection_List<TopoDS_Shape> ClosingFaces, double Offset,
                                     double Tol, BRepOffset_Mode Mode = BRepOffset_Skin, bool Intersection = false,
                                     bool SelfInter = false, GeomAbs_JoinType Join = GeomAbs_Arc,
                                     bool RemoveIntEdges = false, Message_ProgressRange theRange = default)
    {
        NotDone();
        myLastUsedAlgo = OffsetAlgo_Type.OffsetAlgo_JOIN;

        myOffsetShape.Initialize(S, Offset, Tol, Mode, Intersection, SelfInter, Join, false, RemoveIntEdges);
        NCollection_List<TopoDS_Shape>.Iterator it = new(ClosingFaces);
        for (; it.More(); it.Next())
        {
            myOffsetShape.AddFace(TopoDS.Face(it.Value()));
        }

        myOffsetShape.MakeThickSolid(theRange);
        if (!myOffsetShape.IsDone())
        {
            return;
        }

        myShape = myOffsetShape.InternalShape();
        Done();
    }

    //=================================================================================================

    /// <summary>Makes a thick solid with the simple algorithm (the shell offset, closed by walls).</summary>
    public void MakeThickSolidBySimple(in TopoDS_Shape theS, double theOffsetValue)
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

    /// <summary>Does nothing.</summary>
    public override void Build(Message_ProgressRange theRange = default) { }

    //=================================================================================================

    internal override NCollection_List<TopoDS_Shape> InternalModified(in TopoDS_Shape F)
    {
        myGenerated.Clear();

        // OCCT.NET: the caller's face F is looked up through its working copy (decision 1005).
        TopoDS_Shape aF = myOffsetShape.InternalCopied(F);
        if (myLastUsedAlgo == OffsetAlgo_Type.OffsetAlgo_JOIN && myOffsetShape.InternalOffsetFacesFromShapes().HasImage(aF))
        {
            if (myOffsetShape.InternalClosingFaces().Contains(aF))
            {
                myOffsetShape.InternalOffsetFacesFromShapes().LastImage(aF, myGenerated);

                // Reverse generated shapes in case of small solids.
                // Useful only for faces without influence on others.
                NCollection_List<TopoDS_Shape>.Iterator it = new(myGenerated);
                for (; it.More(); it.Next())
                {
                    it.ChangeValue().Reverse();
                    it.ChangeValue() = myOffsetShape.InternalFinal(it.Value());
                }
            }
        }
        else if (myLastUsedAlgo == OffsetAlgo_Type.OffsetAlgo_SIMPLE)
        {
            TopoDS_Shape aModShape = mySimpleOffsetShape.InternalModified(F);
            if (!aModShape.IsNull())
            {
                myGenerated.Append(aModShape);
            }
        }

        return myGenerated;
    }
}
