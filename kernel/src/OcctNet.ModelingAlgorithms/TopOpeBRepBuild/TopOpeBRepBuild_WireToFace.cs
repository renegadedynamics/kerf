// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireToFace.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireToFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopOpeBRepTool_OutCurveType;

namespace Occt;

/// <summary>
/// Builds faces from a set of wires and a face: the wires are classified against each other on the face's surface,
/// and each outer wire makes a face with the wires inside it.
/// </summary>
/// <remarks>
/// <para>
/// The edges of the wires must have a pcurve on the face's surface, unless it is a plane. The faces are empty copies
/// of the face (same surface, tolerance and orientation) holding the wires themselves.
/// </para>
/// <para>
/// OCCT flags every wire closed and stores computed pcurves on edges without one: it writes into the caller's wires.
/// The port's <see cref="MakeFaces"/> works on a copy of frozen wires (<see cref="BRepTools_CopyAtEntry"/>) and gives
/// the caller's wires and edges back wherever nothing was written, so the faces hold the caller's own wires as
/// native's do when the wires are already flagged closed (decision 793). The faces handed out are frozen.
/// </para>
/// </remarks>
public sealed class TopOpeBRepBuild_WireToFace
{
    private readonly NCollection_List<TopoDS_Shape> myLW = new();

    /// <summary>An empty set of wires.</summary>
    public TopOpeBRepBuild_WireToFace()
    {
    }

    /// <summary>Removes the wires.</summary>
    public void Init()
    {
        myLW.Clear();
    }

    /// <summary>Adds the wire <paramref name="W"/>.</summary>
    public void AddWire(in TopoDS_Wire W)
    {
        myLW.Append(W);
    }

    /// <summary>
    /// Replaces the contents of <paramref name="LF"/> by the faces the wires bound on <paramref name="F"/> (frozen).
    /// </summary>
    public void MakeFaces(in TopoDS_Face F, NCollection_List<TopoDS_Shape> LF)
    {
        // OCCT.NET: copy at entry (decision 793)
        TopoDS_Shape aHolder = HolderOf(myLW);
        BRepTools_CopyAtEntry anEntry = new(aHolder);
        if (!anEntry.IsCopied())
        {
            InternalMakeFaces(F, LF);
        }
        else
        {
            NCollection_List<TopoDS_Shape> aSaved = new(myLW);
            myLW.Clear();
            for (NCollection_List<TopoDS_Shape>.Iterator it = new(aSaved); it.More(); it.Next())
            {
                myLW.Append(anEntry.Copied(it.Value()));
            }
            try
            {
                InternalMakeFaces(F, LF);
            }
            finally
            {
                myLW.Assign(aSaved);
            }
            anEntry.Finish(new TopoDS_Shape(), LF);
        }
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(LF); it.More(); it.Next())
        {
            it.Value().TShape()?.FreezeGraph();
        }
    }

    /// <summary>OCCT's <c>MakeFaces</c>, in place and without freezing, for ported callers.</summary>
    internal void InternalMakeFaces(in TopoDS_Face F, NCollection_List<TopoDS_Shape> LF)
    {
        LF.Clear();

        TopOpeBRepBuild_WireEdgeSet wes = new(F);
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(myLW); it.More(); it.Next())
        {
            wes.AddShape(it.Value());
        }

        bool ForceClass = true;
        TopOpeBRepBuild_FaceBuilder FB = new();
        FB.InitFaceBuilder(wes, F, ForceClass);

        TopOpeBRepDS_BuildTool BT = new(TopOpeBRepTool_APPROX);
        TopOpeBRepBuild_Builder B = new(BT);
        B.MakeFaces(F, FB, LF);
    }

    /// <summary>
    /// OCCT.NET: an unfrozen compound holding <paramref name="theShapes"/> as children, appended directly so their
    /// Free flags don't change, to copy them at once with <see cref="BRepTools_CopyAtEntry"/> (shared sub-shapes stay
    /// shared).
    /// </summary>
    internal static TopoDS_Shape HolderOf(NCollection_List<TopoDS_Shape> theShapes)
    {
        TopoDS_Compound aHolder = new();
        new BRep_Builder().MakeCompound(ref aHolder);
        TopoDS_TShape aHolderTShape = aHolder.TShape()!;
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(theShapes); it.More(); it.Next())
        {
            if (!it.Value().IsNull())
            {
                aHolderTShape.myShapes.Append(it.Value());
            }
        }
        return aHolder;
    }
}
