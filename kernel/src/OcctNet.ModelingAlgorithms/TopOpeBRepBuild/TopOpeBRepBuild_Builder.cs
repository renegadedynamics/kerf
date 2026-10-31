// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_Builder.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_Builder.cxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_Merge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepCheck_Status;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The legacy Booleans' builder: splits and merges the shapes of a TopOpeBRepDS data structure. Here, only the face
/// maker of <see cref="TopOpeBRepBuild_WireToFace"/>.
/// </summary>
/// <remarks>
/// <para>
/// Only what the face maker of the 2D offset reaches is ported (plan 2.11, decision 585): the constructor, the build
/// tool accessors and <see cref="MakeFaces"/> with Merge.cxx's <c>CorrectUnclosedWire</c> and
/// <c>CorrectEdgeOrientation</c>. The Boolean algorithm (Perform, the split, merge, section and KPart members, about
/// 25,000 lines over 30 files) works on TopOpeBRepDS's data structure, not ported, and is left out, with the data
/// structure field, so <see cref="MakeFaces"/> never finds a new surface for the face (OCCT's <c>hns</c> branch, which
/// needs <c>Perform</c>). The constructor's <c>InitSection</c> only resets the section's fields, which are left out.
/// </para>
/// <para>A working object; it keeps a copy of its build tool, as in OCCT.</para>
/// </remarks>
public class TopOpeBRepBuild_Builder
{
    protected readonly TopOpeBRepDS_BuildTool myBuildTool;

    /// <summary>A builder with a copy of the build tool <paramref name="BT"/>.</summary>
    public TopOpeBRepBuild_Builder(TopOpeBRepDS_BuildTool BT)
    {
        myBuildTool = new TopOpeBRepDS_BuildTool(BT);
        // InitSection: resets the section's fields (not ported)
    }

    /// <summary>The build tool.</summary>
    public TopOpeBRepDS_BuildTool ChangeBuildTool() => myBuildTool;

    /// <summary>The build tool.</summary>
    public TopOpeBRepDS_BuildTool BuildTool() => myBuildTool;

    // Perform, Clear, the split, merge, section and KPart members, DataStructure: not ported (plan 2.11).

    // Merge.cxx: puts the wire's edges in connection order, reversing those that run backwards
    private static void CorrectEdgeOrientation(ref TopoDS_Shape aWire)
    {
        NCollection_List<TopoDS_Shape> anEdgeList = new(), anAuxList = new(), aTrueEdgeList = new();
        BRep_Builder BB = new();
        TopoDS_Vertex vf, vl, v1f, v1l;
        bool append = true;

        TopoDS_Iterator tdi = new(aWire, false, false);
        for (; tdi.More(); tdi.Next())
        {
            anEdgeList.Append(tdi.Value());
        }

        int n = anEdgeList.Extent();
        if (n <= 1)
        {
            return;
        }

        NCollection_List<TopoDS_Shape>.Iterator anIt = new(anEdgeList);

        TopoDS_Shape anCurEdge = anIt.Value();
        TopExp.Vertices(TopoDS.Edge(anCurEdge), out vf, out vl, true);
        aTrueEdgeList.Append(anCurEdge);
        anIt.Next();

        while (n > 0 && append)
        {
            for (; anIt.More(); anIt.Next())
            {
                append = false;
                anCurEdge = anIt.Value();
                TopExp.Vertices(TopoDS.Edge(anCurEdge), out v1f, out v1l, true);
                if (v1f.IsSame(vl))
                {
                    aTrueEdgeList.Append(anCurEdge);
                    vl = v1l;
                    append = true;
                    continue;
                }
                if (v1l.IsSame(vf))
                {
                    aTrueEdgeList.Append(anCurEdge);
                    vf = v1f;
                    append = true;
                    continue;
                }

                if (v1l.IsSame(vl))
                {
                    TopoDS_Shape anRevEdge = anCurEdge.Reversed();
                    aTrueEdgeList.Append(anRevEdge);
                    vl = v1f;
                    append = true;
                    continue;
                }
                if (v1f.IsSame(vf))
                {
                    TopoDS_Shape anRevEdge = anCurEdge.Reversed();
                    aTrueEdgeList.Append(anRevEdge);
                    vf = v1l;
                    append = true;
                    continue;
                }

                anAuxList.Append(anCurEdge);
            }

            anEdgeList.Assign(anAuxList);
            anAuxList.Clear(); // something wrong in Assign when list contains 1 element.
            n = anEdgeList.Extent();
            anIt.Initialize(anEdgeList);
        }

        if (n > 0)
        {
            aTrueEdgeList.Append(anEdgeList);
        }

        aWire.Nullify();
        TopoDS_Wire aNewWire = new();
        BB.MakeWire(ref aNewWire);
        aWire = aNewWire;
        anIt.Initialize(aTrueEdgeList);
        for (; anIt.More(); anIt.Next())
        {
            BB.Add(aWire, anIt.Value());
        }
    }

    // Merge.cxx: removes the wire's edges with fewer than two vertices, then the edges at a vertex of one edge only
    private static void CorrectUnclosedWire(in TopoDS_Shape aWire)
    {
        BRep_Builder BB = new();
        // OCCT removes children while its TopoDS_Iterator is on them (the iterator's node is freed: undefined
        // behaviour, which in practice goes on to the next child). The port iterates a snapshot of the children.
        NCollection_List<TopoDS_Shape> aChildren = new();
        TopoDS_Iterator tdi = new(aWire, false, false);
        for (; tdi.More(); tdi.Next())
        {
            aChildren.Append(tdi.Value());
        }
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(aChildren); it.More(); it.Next())
        {
            TopoDS_Shape ed = it.Value();
            int nbv = ed.NbChildren();
            if (nbv <= 1)
            {
                BB.Remove(aWire, ed);
            }
        }

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> VElists = new(TopTools_ShapeMapHasher.Instance);
        VElists.Clear();
        TopExp.MapShapesAndAncestors(aWire, TopAbs_VERTEX, TopAbs_EDGE, VElists);
        int nbVer = VElists.Extent(), i;

        for (i = 1; i <= nbVer; i++)
        {
            NCollection_List<TopoDS_Shape> Elist = VElists.FindFromIndex(i);
            if (Elist.Extent() == 1)
            {
                TopoDS_Shape anEdge = Elist.First();
                BB.Remove(aWire, anEdge);
            }
        }
    }

    /// <summary>
    /// Appends to <paramref name="L"/> the faces of <paramref name="FABU"/>, each an empty copy of
    /// <paramref name="aFace"/> with the face builder's wires (old ones as they are, new ones made of its edges and
    /// corrected when BRepCheck finds them invalid), every wire flagged closed.
    /// </summary>
    /// <remarks>
    /// Writes the Closed flag of old wires, as in OCCT: they must then be unfrozen (see
    /// <see cref="TopOpeBRepBuild_WireToFace.MakeFaces"/>, which copies frozen ones).
    /// </remarks>
    public void MakeFaces(in TopoDS_Shape aFace, TopOpeBRepBuild_FaceBuilder FABU, NCollection_List<TopoDS_Shape> L)
    {
        TopoDS_Shape newFace;
        TopoDS_Shape newWire = new();

        for (FABU.InitFace(); FABU.MoreFace(); FABU.NextFace())
        {
            myBuildTool.CopyFace(aFace, out newFace);
            // OCCT: with a data structure (set by Perform, not ported) holding a new surface for aFace (hns), the new
            // face and the edges of its new wires take that surface

            for (FABU.InitWire(); FABU.MoreWire(); FABU.NextWire())
            {
                bool isold = FABU.IsOldWire();
                if (isold)
                {
                    newWire = FABU.OldWire();
                }
                else
                {
                    myBuildTool.MakeWire(ref newWire);
                    for (FABU.InitEdge(); FABU.MoreEdge(); FABU.NextEdge())
                    {
                        TopoDS_Shape E = FABU.Edge();
                        myBuildTool.AddWireEdge(newWire, E);
                    }
                }
                //----------- IFV
                if (!isold)
                {
                    BRepCheck_Analyzer bca = new(newWire, false);
                    if (!bca.IsValid())
                    {
                        newWire.Free(true);
                        CorrectUnclosedWire(newWire);
                        BRepCheck_Result bcr = bca.Result(newWire)!;
                        NCollection_List<BRepCheck_Status>.Iterator itl = new(bcr.Status());
                        for (; itl.More(); itl.Next())
                        {
                            if (itl.Value() == BRepCheck_BadOrientationOfSubshape)
                            {
                                CorrectEdgeOrientation(ref newWire);
                                break;
                            }
                        }
                    }
                }
                myBuildTool.Closed(newWire, true); // NYI : check exact du caractere closed du wire
                myBuildTool.AddFaceWire(newFace, newWire);
            }

            L.Append(newFace);
        }
    }
}
