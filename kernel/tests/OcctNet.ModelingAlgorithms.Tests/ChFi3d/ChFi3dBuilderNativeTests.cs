// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The fillet builder end to end (unit 7N2): the data structure ChFi3d_Builder::Compute hands to the reconstruction,
/// against native OCCT's as tools/dsrec records it just before <c>TopOpeBRepBuild_HBuilder::Perform</c>.
/// </summary>
public class ChFi3dBuilderNativeTests
{
    /// <summary>The reconstruction tests plug in: it records the data structure at Perform and builds nothing.</summary>
    internal sealed class RecordingReconstruction : ChFi3d_Reconstruction
    {
        private readonly TopoDS_Shape myInput;

        internal RecordingReconstruction(in TopoDS_Shape theInput) => myInput = theInput;

        /// <summary>The data structure's text at Perform (<see cref="TopOpeBRepDS_TextDump"/>), null before.</summary>
        internal string? Text { get; private set; }

        internal TopOpeBRepDS_HDataStructure? HDS { get; private set; }

        internal int NbPerform { get; private set; }

        public override void Perform(TopOpeBRepDS_HDataStructure HDS)
        {
            NbPerform++;
            this.HDS = HDS;
            Text = TopOpeBRepDS_TextDump.Write(HDS.DS(), myInput);
        }

        public override void MergeSolid(in TopoDS_Shape S, TopAbs_State TB)
        {
        }

        public override bool IsSplit(in TopoDS_Shape S, TopAbs_State ToBuild) => false;

        public override NCollection_List<TopoDS_Shape> Splits(in TopoDS_Shape S, TopAbs_State ToBuild) => new();

        public override NCollection_List<TopoDS_Shape> Merged(in TopoDS_Shape S, TopAbs_State ToBuild) => new();

        public override NCollection_List<TopoDS_Shape> NewFaces(int I) => new();

        public override NCollection_List<TopoDS_Shape> NewEdges(int I) => new();

        public override TopOpeBRepDS_HDataStructure? DataStructure() => HDS;
    }

    /// <summary>An edge of a fillet case: its index in TopExp::MapShapes(input, EDGE) and its radii (r2 &lt; 0: constant).</summary>
    internal readonly record struct CaseEdge(int K, double R1, double R2 = -1);

    internal static TopoDS_Edge EdgeOf(in TopoDS_Shape S, int k)
    {
        NCollection_IndexedMap<TopoDS_Shape> M = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, TopAbs_EDGE, M);
        return TopoDS.Edge(M[k]);
    }

    /// <summary>tools/dsrec's EdgesAtVertex1: the edges at the input's first vertex, in ancestor order.</summary>
    internal static List<int> EdgesAtVertex1(in TopoDS_Shape S)
    {
        NCollection_IndexedMap<TopoDS_Shape> ME = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, TopAbs_EDGE, ME);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> M = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(S, TopAbs_VERTEX, TopAbs_EDGE, M);
        List<int> r = [];
        foreach (TopoDS_Shape e in M[1])
        {
            int k = ME.FindIndex(e);
            if (!r.Contains(k))
            {
                r.Add(k);
            }
        }
        return r;
    }

    /// <summary>tools/dsrec's choice of the cylinder's top edge: not degenerated, not the seam, at Z &gt; 50.</summary>
    private static int CylinderTopEdge(in TopoDS_Shape S)
    {
        NCollection_IndexedMap<TopoDS_Shape> M = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, TopAbs_EDGE, M);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> EF = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(S, TopAbs_EDGE, TopAbs_FACE, EF);
        for (int k = 1; k <= M.Extent(); k++)
        {
            TopoDS_Edge E = TopoDS.Edge(M[k]);
            if (!BRep_Tool.Degenerated(E) && !BRep_Tool.IsClosed(E, TopoDS.Face(EF.FindFromKey(E).First())))
            {
                Geom_Curve C = BRep_Tool.Curve(E, out double f, out _)!;
                if (C.Value(f).Z() > 50)
                {
                    return k;
                }
            }
        }
        return 0;
    }

    /// <summary>tools/dsrec/main.cpp's fillet cases: the edges and radii of each.</summary>
    internal static List<CaseEdge> FilletEdges(string theName, in TopoDS_Shape S)
    {
        switch (theName)
        {
            case "box_edge":
                return [new(5, 10)];
            case "box_two_edges":
            {
                List<int> c = EdgesAtVertex1(S);
                return [new(c[0], 10), new(c[1], 10)];
            }
            case "box_corner":
            {
                List<int> c = EdgesAtVertex1(S);
                return [new(c[0], 10), new(c[1], 10), new(c[2], 10)];
            }
            case "box_all_edges":
                return Enumerable.Range(1, 12).Select(k => new CaseEdge(k, 10)).ToList();
            case "box_edge_evolving":
                return [new(5, 5, 15)];
            case "cylinder_top_edge":
                return [new(CylinderTopEdge(S), 10)];
            case "B1_partial_cylinder":
                return [new(9, 10), new(6, 10), new(5, 10)];
            case "C1_partial_cone":
                return [new(3, 10)];
            case "D1_partial_cone_apex":
                return [new(3, 10), new(2, 10), new(5, 10)];
            case "E1_partial_sphere":
                return [new(7, 10)];
            case "F1_partial_torus":
                return [new(4, 10), new(1, 10), new(5, 10)];
            case "K1_box_cut_cylinder":
                return [new(5, 5)];
            default:
                throw new ArgumentException(theName);
        }
    }

    /// <summary>
    /// BRepFilletAPI_MakeFillet's Add (constant or evolving radius) and Build on the port's ChFi3d_FilBuilder, with the
    /// recording reconstruction in place of TopOpeBRepBuild_HBuilder.
    /// </summary>
    internal static (ChFi3d_FilBuilder Builder, RecordingReconstruction Rec) RunFillet(in TopoDS_Shape S,
                                                                                       IEnumerable<CaseEdge> theEdges)
    {
        ChFi3d_FilBuilder aB = new(S);
        RecordingReconstruction aRec = new(aB.WorkingShape());
        aB.myCoup = aRec;
        foreach (CaseEdge e in theEdges)
        {
            TopoDS_Edge E = EdgeOf(S, e.K);
            aB.Add(E);
            int IC = aB.Contains(E, out int IinC);
            if (IC != 0)
            {
                double r1 = e.R1, r2 = e.R2 < 0 ? e.R1 : e.R2;
                if (Math.Abs(r1 - r2) < Precision.Confusion())
                {
                    r1 = r2 = (r1 + r2) * 0.5;
                }
                aB.SetRadius(new gp_XY(0.0, r1), IC, IinC);
                aB.SetRadius(new gp_XY(1.0, r2), IC, IinC);
            }
        }
        aB.Compute();
        return (aB, aRec);
    }

    public static TheoryData<string> FilletCaseNames()
    {
        TheoryData<string> aData = new();
        foreach (string aName in new[]
                 {
                     "box_edge", "box_two_edges", "box_corner", "box_all_edges", "box_edge_evolving", "cylinder_top_edge",
                     "B1_partial_cylinder", "C1_partial_cone", "D1_partial_cone_apex", "E1_partial_sphere",
                     "F1_partial_torus", "K1_box_cut_cylinder",
                 })
        {
            aData.Add(aName);
        }
        return aData;
    }

    [Theory]
    [MemberData(nameof(FilletCaseNames))]
    public void TheDataStructureAtPerformIsNatives(string theName)
    {
        DsCase aCase = DsRecordings.Cases[theName];
        TopoDS_Shape anInput = aCase.Input;
        (ChFi3d_FilBuilder _, RecordingReconstruction aRec) = RunFillet(anInput, FilletEdges(theName, anInput));
        Assert.Equal(aCase.PerformCalls, aRec.NbPerform);
        Assert.NotNull(aRec.Text);
        List<string> aDiffs = DsRecordings.Differences(aCase.DsText, aRec.Text!, 1.0e-9);
        Assert.True(aDiffs.Count == 0, theName + ":\n" + string.Join("\n", aDiffs));
    }
}
