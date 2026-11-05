// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_Builder.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_Builder.cxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_Merge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Checks.StatusCheck;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Legacy.Build;

/// <summary>
/// The legacy Booleans' builder: splits and merges the shapes of a TopOpeBRepDS data structure. Here, only the face
/// maker of <see cref="WireToFace"/>.
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
internal class LegacyBuilder
{
    protected readonly DsBuildTool _buildTool;

    /// <summary>A builder with a copy of the build tool <paramref name="bt"/>.</summary>
    public LegacyBuilder(DsBuildTool bt)
    {
        _buildTool = new DsBuildTool(bt);
        // InitSection: resets the section's fields (not ported)
    }

    /// <summary>The build tool.</summary>
    public DsBuildTool ChangeBuildTool() => _buildTool;

    /// <summary>The build tool.</summary>
    public DsBuildTool BuildTool() => _buildTool;

    // Perform, Clear, the split, merge, section and KPart members, DataStructure: not ported (plan 2.11).

    // Merge.cxx: puts the wire's edges in connection order, reversing those that run backwards
    private static void CorrectEdgeOrientation(ref Shape aWire)
    {
        NCollection_List<Shape> edgeList = new(), auxList = new(), trueEdgeList = new();
        BrepBuilder bb = new();
        Vertex vf, vl, v1f, v1l;
        bool append = true;

        Iterator tdi = new(aWire, false, false);
        for (; tdi.More(); tdi.Next())
        {
            edgeList.Append(tdi.Value());
        }

        int n = edgeList.Extent();
        if (n <= 1)
        {
            return;
        }

        NCollection_List<Shape>.Iterator it = new(edgeList);

        Shape curEdge = it.Value();
        Exploration.Vertices(ShapeCasts.Edge(curEdge), out vf, out vl, true);
        trueEdgeList.Append(curEdge);
        it.Next();

        while (n > 0 && append)
        {
            for (; it.More(); it.Next())
            {
                append = false;
                curEdge = it.Value();
                Exploration.Vertices(ShapeCasts.Edge(curEdge), out v1f, out v1l, true);
                if (v1f.IsSame(vl))
                {
                    trueEdgeList.Append(curEdge);
                    vl = v1l;
                    append = true;
                    continue;
                }
                if (v1l.IsSame(vf))
                {
                    trueEdgeList.Append(curEdge);
                    vf = v1f;
                    append = true;
                    continue;
                }

                if (v1l.IsSame(vl))
                {
                    Shape revEdge = curEdge.Reversed();
                    trueEdgeList.Append(revEdge);
                    vl = v1f;
                    append = true;
                    continue;
                }
                if (v1f.IsSame(vf))
                {
                    Shape revEdge = curEdge.Reversed();
                    trueEdgeList.Append(revEdge);
                    vf = v1l;
                    append = true;
                    continue;
                }

                auxList.Append(curEdge);
            }

            edgeList.Assign(auxList);
            auxList.Clear(); // something wrong in Assign when list contains 1 element.
            n = edgeList.Extent();
            it.Initialize(edgeList);
        }

        if (n > 0)
        {
            trueEdgeList.Append(edgeList);
        }

        aWire.Nullify();
        Wire newWire = new();
        bb.MakeWire(ref newWire);
        aWire = newWire;
        it.Initialize(trueEdgeList);
        for (; it.More(); it.Next())
        {
            bb.Add(aWire, it.Value());
        }
    }

    // Merge.cxx: removes the wire's edges with fewer than two vertices, then the edges at a vertex of one edge only
    private static void CorrectUnclosedWire(in Shape aWire)
    {
        BrepBuilder bb = new();
        // OCCT removes children while its TopoDS_Iterator is on them (the iterator's node is freed: undefined
        // behaviour, which in practice goes on to the next child). The port iterates a snapshot of the children.
        NCollection_List<Shape> children = new();
        Iterator tdi = new(aWire, false, false);
        for (; tdi.More(); tdi.Next())
        {
            children.Append(tdi.Value());
        }
        for (NCollection_List<Shape>.Iterator it = new(children); it.More(); it.Next())
        {
            Shape ed = it.Value();
            int nbv = ed.ChildCount;
            if (nbv <= 1)
            {
                bb.Remove(aWire, ed);
            }
        }

        IndexedMap<Shape, NCollection_List<Shape>> vElists = new(ShapeHasher.Instance);
        vElists.Clear();
        Exploration.MapShapesAndAncestors(aWire, ShapeType.Vertex, ShapeType.Edge, vElists);
        int nbVer = vElists.Count, i;

        for (i = 1; i <= nbVer; i++)
        {
            NCollection_List<Shape> elist = vElists.ValueAt(i - 1);
            if (elist.Extent() == 1)
            {
                Shape edge = elist.First();
                bb.Remove(aWire, edge);
            }
        }
    }

    /// <summary>
    /// Appends to <paramref name="l"/> the faces of <paramref name="fabu"/>, each an empty copy of
    /// <paramref name="aFace"/> with the face builder's wires (old ones as they are, new ones made of its edges and
    /// corrected when BRepCheck finds them invalid), every wire flagged closed.
    /// </summary>
    /// <remarks>
    /// Writes the Closed flag of old wires, as in OCCT: they must then be unfrozen (see
    /// <see cref="WireToFace.MakeFaces"/>, which copies frozen ones).
    /// </remarks>
    public void MakeFaces(in Shape aFace, BuildFaceBuilder fabu, NCollection_List<Shape> l)
    {
        Shape newFace;
        Shape newWire = new();

        for (fabu.InitFace(); fabu.MoreFace(); fabu.NextFace())
        {
            _buildTool.CopyFace(aFace, out newFace);
            // OCCT: with a data structure (set by Perform, not ported) holding a new surface for aFace (hns), the new
            // face and the edges of its new wires take that surface

            for (fabu.InitWire(); fabu.MoreWire(); fabu.NextWire())
            {
                bool isold = fabu.IsOldWire();
                if (isold)
                {
                    newWire = fabu.OldWire();
                }
                else
                {
                    _buildTool.MakeWire(ref newWire);
                    for (fabu.InitEdge(); fabu.MoreEdge(); fabu.NextEdge())
                    {
                        Shape e = fabu.Edge();
                        _buildTool.AddWireEdge(newWire, e);
                    }
                }
                //----------- IFV
                if (!isold)
                {
                    Analyzer bca = new(newWire, false);
                    if (!bca.IsValid())
                    {
                        newWire.Free(true);
                        CorrectUnclosedWire(newWire);
                        ResultCheck bcr = bca.Result(newWire)!;
                        NCollection_List<StatusCheck>.Iterator itl = new(bcr.Status());
                        for (; itl.More(); itl.Next())
                        {
                            if (itl.Value() == StatusCheck.BadOrientationOfSubshape)
                            {
                                CorrectEdgeOrientation(ref newWire);
                                break;
                            }
                        }
                    }
                }
                _buildTool.Closed(newWire, true); // NYI : check exact du caractere closed du wire
                _buildTool.AddFaceWire(newFace, newWire);
            }

            l.Append(newFace);
        }
    }
}
