// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
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

        /// <summary>The queries the builder made after Perform, by member (OCCT.NET: unit 7R's measurements).</summary>
        internal Dictionary<string, int> Calls { get; } = new();

        private void count(string theMember) => Calls[theMember] = Calls.GetValueOrDefault(theMember) + 1;

        public override void MergeSolid(in TopoDS_Shape S, TopAbs_State TB) => count("MergeSolid " + S.ShapeType() + " " + TB);

        public override bool IsSplit(in TopoDS_Shape S, TopAbs_State ToBuild)
        {
            count("IsSplit " + S.ShapeType() + " " + ToBuild);
            return false;
        }

        public override NCollection_List<TopoDS_Shape> Splits(in TopoDS_Shape S, TopAbs_State ToBuild)
        {
            count("Splits " + S.ShapeType() + " " + ToBuild);
            return new();
        }

        public override NCollection_List<TopoDS_Shape> Merged(in TopoDS_Shape S, TopAbs_State ToBuild)
        {
            count("Merged " + S.ShapeType() + " " + ToBuild);
            return new();
        }

        public override NCollection_List<TopoDS_Shape> NewFaces(int I)
        {
            count("NewFaces");
            return new();
        }

        public override NCollection_List<TopoDS_Shape> NewEdges(int I)
        {
            count("NewEdges");
            return new();
        }

        public override TopOpeBRepDS_HDataStructure? DataStructure() => HDS;
    }

    /// <summary>
    /// An edge of a case: its index in TopExp::MapShapes(input, EDGE) and its parameters (a fillet's radii, r2 &lt; 0:
    /// constant; a chamfer's distances, r2 &lt; 0: one distance; Mode 1: a distance and an angle in degrees).
    /// </summary>
    internal readonly record struct CaseEdge(int K, double R1, double R2 = -1, int Mode = 0);

    internal static TopoDS_Edge EdgeOf(in TopoDS_Shape S, int k)
    {
        NCollection_IndexedMap<TopoDS_Shape> M = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, TopAbs_EDGE, M);
        return TopoDS.Edge(M[k]);
    }

    /// <summary>tools/dsrec's FaceOf: the first face of the edge in TopExp::MapShapesAndAncestors(S, EDGE, FACE).</summary>
    internal static TopoDS_Face FaceOf(in TopoDS_Shape S, in TopoDS_Edge E)
    {
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> M = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(S, TopAbs_EDGE, TopAbs_FACE, M);
        return TopoDS.Face(M.FindFromKey(E).First());
    }

    /// <summary>The case's operation (true: chamfer) and edges, as tools/dsrec records them in the status line.</summary>
    internal static (bool Chamfer, List<CaseEdge> Edges) CaseEdges(DsCase theCase)
    {
        string[] t = theCase.StatusTokens;
        int i = Array.IndexOf(t, "fillet");
        bool aChamfer = false;
        if (i < 0)
        {
            i = Array.IndexOf(t, "chamfer");
            aChamfer = true;
        }
        List<CaseEdge> r = [];
        foreach (string e in t.Skip(i + 1))
        {
            string[] p = e.Split(':');
            r.Add(new CaseEdge(int.Parse(p[0], CultureInfo.InvariantCulture), double.Parse(p[1], CultureInfo.InvariantCulture),
                               double.Parse(p[2], CultureInfo.InvariantCulture), int.Parse(p[3], CultureInfo.InvariantCulture)));
        }
        return (aChamfer, r);
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

    /// <summary>BRepFilletAPI_MakeChamfer's Add, AddDA and Build on the port's ChFi3d_ChBuilder.</summary>
    internal static (ChFi3d_ChBuilder Builder, RecordingReconstruction Rec) RunChamfer(in TopoDS_Shape S,
                                                                                       IEnumerable<CaseEdge> theEdges)
    {
        ChFi3d_ChBuilder aB = new(S);
        RecordingReconstruction aRec = new(aB.WorkingShape());
        aB.myCoup = aRec;
        foreach (CaseEdge e in theEdges)
        {
            TopoDS_Edge E = EdgeOf(S, e.K);
            if (e.Mode == 1)
            {
                aB.AddDA(e.R1, e.R2 * M_PI / 180.0, E, FaceOf(S, E));
            }
            else if (e.R2 < 0)
            {
                aB.Add(e.R1, E);
            }
            else
            {
                aB.Add(e.R1, e.R2, E, FaceOf(S, E));
            }
        }
        aB.Compute();
        return (aB, aRec);
    }

    internal static (ChFi3d_Builder Builder, RecordingReconstruction Rec) Run(DsCase theCase)
    {
        TopoDS_Shape anInput = theCase.Input;
        (bool aChamfer, List<CaseEdge> anEdges) = CaseEdges(theCase);
        if (aChamfer)
        {
            (ChFi3d_ChBuilder c, RecordingReconstruction rc) = RunChamfer(anInput, anEdges);
            return (c, rc);
        }
        (ChFi3d_FilBuilder f, RecordingReconstruction rf) = RunFillet(anInput, anEdges);
        return (f, rf);
    }

    public static TheoryData<string> CaseNames()
    {
        TheoryData<string> aData = new();
        foreach (string aName in DsRecordings.Cases.Keys)
        {
            aData.Add(aName);
        }
        return aData;
    }

    [Theory]
    [MemberData(nameof(CaseNames))]
    public void TheDataStructureAtPerformIsNatives(string theName)
    {
        DsCase aCase = DsRecordings.Cases[theName];
        (ChFi3d_Builder aB, RecordingReconstruction aRec) = Run(aCase);
        Assert.Equal(aCase.PerformCalls, aRec.NbPerform);
        if (aCase.PerformCalls == 0)
        {
            Assert.False(aB.IsDone());
            return;
        }
        Assert.NotNull(aRec.Text);
        List<string> aDiffs = DsRecordings.Differences(aCase.DsText, aRec.Text!, 0.0);
        Assert.True(aDiffs.Count == 0, theName + ":\n" + string.Join("\n", aDiffs));
        Assert.Equal(aCase.DsText, aRec.Text); // byte for byte
    }

    [Theory]
    [MemberData(nameof(CaseNames))]
    public void TheDiagnosticsAreNatives(string theName)
    {
        // what build123d.NET reads after a fillet (plan 2.6): the faulty contours, the stripes' ChFiDS_ErrorStatus, the
        // faulty vertices; recorded for the fillets (BRepFilletAPI_MakeChamfer does not expose them)
        DsCase aCase = DsRecordings.Cases[theName];
        if (CaseEdges(aCase).Chamfer)
        {
            return;
        }
        (ChFi3d_Builder aB, RecordingReconstruction _) = Run(aCase);
        Assert.Equal(aCase.StatusInts("contours")[0], aB.NbElements());
        List<int> aFaulty = aCase.StatusInts("faulty");
        Assert.Equal(aFaulty[0], aB.NbFaultyContours());
        for (int i = 1; i <= aB.NbFaultyContours(); i++)
        {
            Assert.Equal(aFaulty[i], aB.FaultyContour(i));
        }
        List<int> aStatuses = aCase.StatusInts("statuses");
        for (int ic = 1; ic <= aB.NbElements(); ic++)
        {
            Assert.Equal((ChFiDS_ErrorStatus)aStatuses[ic - 1], aB.StripeStatus(ic));
        }
        Assert.Equal(aCase.StatusInts("faultyvertices")[0], aB.NbFaultyVertices());
        Assert.Equal(aCase.StatusInts("hasresult")[0] == 1, aB.HasResult());
        Assert.Equal(aCase.Done, aB.IsDone());
    }
}
