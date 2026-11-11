// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Fillets;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Geometry;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Topology;
using System.Globalization;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The fillet builder end to end (unit 7N2): the data structure ChFi3d_Builder::Compute hands to the reconstruction,
/// against native OCCT's as tools/dsrec records it just before <c>TopOpeBRepBuild_HBuilder::Perform</c>.
/// </summary>
public class ChFi3dBuilderNativeTests
{
    /// <summary>The reconstruction tests plug in: it records the data structure at Perform and builds nothing.</summary>
    internal sealed class RecordingReconstruction : Reconstruction
    {
        private readonly Shape myInput;

        internal RecordingReconstruction(in Shape theInput) => myInput = theInput;

        /// <summary>The data structure's text at Perform (<see cref="TextDump"/>), null before.</summary>
        internal string? Text { get; private set; }

        internal DsHDataStructure? HDS { get; private set; }

        internal int NbPerform { get; private set; }

        public override void Perform(DsHDataStructure HDS)
        {
            NbPerform++;
            this.HDS = HDS;
            Text = TextDump.Write(HDS.DS(), myInput);
        }

        /// <summary>The queries the builder made after Perform, by member (Kerf: unit 7R's measurements).</summary>
        internal Dictionary<string, int> Calls { get; } = new();

        private void count(string theMember) => Calls[theMember] = Calls.GetValueOrDefault(theMember) + 1;

        public override void MergeSolid(in Shape S, Containment TB) => count("MergeSolid " + Kerf.Engine.Foundation.OcctTypeNames.Of(S.Type) + " " + Kerf.Engine.Foundation.OcctTypeNames.Of(TB));

        public override bool IsSplit(in Shape S, Containment ToBuild)
        {
            count("IsSplit " + Kerf.Engine.Foundation.OcctTypeNames.Of(S.Type) + " " + Kerf.Engine.Foundation.OcctTypeNames.Of(ToBuild));
            return false;
        }

        public override NCollection_List<Shape> Splits(in Shape S, Containment ToBuild)
        {
            count("Splits " + Kerf.Engine.Foundation.OcctTypeNames.Of(S.Type) + " " + Kerf.Engine.Foundation.OcctTypeNames.Of(ToBuild));
            return new();
        }

        public override NCollection_List<Shape> Merged(in Shape S, Containment ToBuild)
        {
            count("Merged " + Kerf.Engine.Foundation.OcctTypeNames.Of(S.Type) + " " + Kerf.Engine.Foundation.OcctTypeNames.Of(ToBuild));
            return new();
        }

        public override NCollection_List<Shape> NewFaces(int I)
        {
            count("NewFaces");
            return new();
        }

        public override NCollection_List<Shape> NewEdges(int I)
        {
            count("NewEdges");
            return new();
        }

        public override DsHDataStructure? DataStructure() => HDS;
    }

    /// <summary>
    /// An edge of a case: its index in TopExp::MapShapes(input, EDGE) and its parameters (a fillet's radii, r2 &lt; 0:
    /// constant; a chamfer's distances, r2 &lt; 0: one distance; Mode 1: a distance and an angle in degrees).
    /// </summary>
    internal readonly record struct CaseEdge(int K, double R1, double R2 = -1, int Mode = 0);

    internal static Edge EdgeOf(in Shape S, int k)
    {
        IndexedSet<Shape> M = new(ShapeHasher.Instance);
        Exploration.MapShapes(S, ShapeType.Edge, M);
        return ShapeCasts.Edge(M[k - 1]);
    }

    /// <summary>tools/dsrec's FaceOf: the first face of the edge in TopExp::MapShapesAndAncestors(S, EDGE, FACE).</summary>
    internal static Face FaceOf(in Shape S, in Edge E)
    {
        IndexedMap<Shape, NCollection_List<Shape>> M = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(S, ShapeType.Edge, ShapeType.Face, M);
        return ShapeCasts.Face(M.GetValue(E).First());
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
    internal static (FilBuilder Builder, RecordingReconstruction Rec) RunFillet(in Shape S,
                                                                                       IEnumerable<CaseEdge> theEdges)
    {
        FilBuilder aB = new(S);
        RecordingReconstruction aRec = new(aB.WorkingShape());
        aB._coup = aRec;
        foreach (CaseEdge e in theEdges)
        {
            Edge E = EdgeOf(S, e.K);
            aB.Add(E);
            int IC = aB.Contains(E, out int IinC);
            if (IC != 0)
            {
                double r1 = e.R1, r2 = e.R2 < 0 ? e.R1 : e.R2;
                if (Math.Abs(r1 - r2) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    r1 = r2 = (r1 + r2) * 0.5;
                }
                aB.SetRadius(new Xy(0.0, r1), IC, IinC);
                aB.SetRadius(new Xy(1.0, r2), IC, IinC);
            }
        }
        aB.Compute();
        return (aB, aRec);
    }

    /// <summary>BRepFilletAPI_MakeChamfer's Add, AddDA and Build on the port's ChFi3d_ChBuilder.</summary>
    internal static (ChBuilder Builder, RecordingReconstruction Rec) RunChamfer(in Shape S,
                                                                                       IEnumerable<CaseEdge> theEdges)
    {
        ChBuilder aB = new(S);
        RecordingReconstruction aRec = new(aB.WorkingShape());
        aB._coup = aRec;
        foreach (CaseEdge e in theEdges)
        {
            Edge E = EdgeOf(S, e.K);
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

    internal static (FilletBuilderBase Builder, RecordingReconstruction Rec) Run(DsCase theCase)
    {
        Shape anInput = theCase.Input;
        (bool aChamfer, List<CaseEdge> anEdges) = CaseEdges(theCase);
        if (aChamfer)
        {
            (ChBuilder c, RecordingReconstruction rc) = RunChamfer(anInput, anEdges);
            return (c, rc);
        }
        (FilBuilder f, RecordingReconstruction rf) = RunFillet(anInput, anEdges);
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
        (FilletBuilderBase aB, RecordingReconstruction aRec) = Run(aCase);
        Assert.Equal(aCase.PerformCalls, aRec.NbPerform);
        if (aCase.PerformCalls == 0)
        {
            Assert.False(aB.IsDone);
            return;
        }
        Assert.NotNull(aRec.Text);
        if (BallSections.TryGetValue(theName, out double aTol))
        {
            // Decisions 2156 and 2158: the walked sections are the rolling ball's, native's the 2D fillet in the guide's
            // plane; the same data structure, line for line and number for number, its numbers within aTol of native's
            // (relative), or where the surfaces moved (aTol < 0) only its shape
            string[] e = aCase.DsText.Split('\n'), a = aRec.Text!.Split('\n');
            if (aTol < -1)
            {
                return;
            }
            Assert.Equal(e.Length, a.Length);
            for (int i = 0; i < e.Length; i++)
            {
                Assert.True(e[i].Split(' ').Length == a[i].Split(' ').Length, theName + " line " + (i + 1));
            }
            if (aTol > 0)
            {
                List<string> aClose = DsRecordings.Differences(aCase.DsText, aRec.Text!, aTol);
                Assert.True(aClose.Count == 0, theName + ":\n" + string.Join("\n", aClose));
            }
            return;
        }
        List<string> aDiffs = DsRecordings.Differences(aCase.DsText, aRec.Text!, 0.0);
        Assert.True(aDiffs.Count == 0, theName + ":\n" + string.Join("\n", aDiffs));
        Assert.Equal(aCase.DsText, aRec.Text); // byte for byte
    }

    /// <summary>
    /// The walked cases whose sections are the rolling ball's since decisions 2156 and 2158, with how close their numbers
    /// stay to native's, where the corners' tolerances and the walks' points
    /// moved onto the ball: the pyramid and the torus in shape only, the cone, whose walk takes fewer points, not at all
    /// (its volume in <see cref="BallVolumes"/>).
    /// </summary>
    private static readonly Dictionary<string, double> BallSections = new()
    {
        // The evolving radius on a box's edge (decision 2216): between planes, so the ball's sections are OCCT's but for
        // the last few bits of the walk's knots and the tolerances
        ["box_edge_evolving"] = 1.0e-9,
        ["pyramid_apex_four_edges"] = -1,
        ["D1_partial_cone_apex"] = -2,
        ["F1_partial_torus"] = -1,
    };

    /// <summary>
    /// The pipeline's volumes where the fillet moved onto the ball (decision 2159): native's is the off-ball 2D fillet's.
    /// </summary>
    private static readonly Dictionary<string, double> BallVolumes = new()
    {
        ["D1_partial_cone_apex"] = 66874.229805177427,
    };

    [Theory]
    [MemberData(nameof(CaseNames))]
    public void TheDiagnosticsAreNatives(string theName)
    {
        // what Kerf.Modeling reads after a fillet (plan 2.6): the faulty contours, the stripes' ChFiDS_ErrorStatus, the
        // faulty vertices; recorded for the fillets (BRepFilletAPI_MakeChamfer does not expose them)
        DsCase aCase = DsRecordings.Cases[theName];
        if (CaseEdges(aCase).Chamfer)
        {
            return;
        }
        (FilletBuilderBase aB, RecordingReconstruction _) = Run(aCase);
        Assert.Equal(aCase.StatusInts("contours")[0], aB.NbElements());
        List<int> aFaulty = aCase.StatusInts("faulty");
        Assert.Equal(aFaulty[0], aB.FaultyContourCount);
        for (int i = 1; i <= aB.FaultyContourCount; i++)
        {
            Assert.Equal(aFaulty[i], aB.FaultyContour(i));
        }
        List<int> aStatuses = aCase.StatusInts("statuses");
        for (int ic = 1; ic <= aB.NbElements(); ic++)
        {
            Assert.Equal((ErrorStatus)aStatuses[ic - 1], aB.StripeStatus(ic));
        }
        Assert.Equal(aCase.StatusInts("faultyvertices")[0], aB.FaultyVertexCount);
        Assert.Equal(aCase.StatusInts("hasresult")[0] == 1, aB.HasResult);
        Assert.Equal(aCase.Done, aB.IsDone);
    }

    /// <summary>
    /// The whole pipeline (unit 7N2's builder, unit 7R's reconstruction) on a case: the builder with its default
    /// reconstruction.
    /// </summary>
    internal static FilletBuilderBase RunEndToEnd(DsCase theCase)
    {
        Shape anInput = theCase.Input;
        (bool aChamfer, List<CaseEdge> anEdges) = CaseEdges(theCase);
        FilletBuilderBase aB;
        if (aChamfer)
        {
            ChBuilder c = new(anInput);
            foreach (CaseEdge e in anEdges)
            {
                Edge E = EdgeOf(anInput, e.K);
                if (e.Mode == 1)
                {
                    c.AddDA(e.R1, e.R2 * M_PI / 180.0, E, FaceOf(anInput, E));
                }
                else if (e.R2 < 0)
                {
                    c.Add(e.R1, E);
                }
                else
                {
                    c.Add(e.R1, e.R2, E, FaceOf(anInput, E));
                }
            }
            aB = c;
        }
        else
        {
            FilBuilder f = new(anInput);
            foreach (CaseEdge e in anEdges)
            {
                Edge E = EdgeOf(anInput, e.K);
                f.Add(E);
                int IC = f.Contains(E, out int IinC);
                double r1 = e.R1, r2 = e.R2 < 0 ? e.R1 : e.R2;
                if (Math.Abs(r1 - r2) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    r1 = r2 = (r1 + r2) * 0.5;
                }
                f.SetRadius(new Xy(0.0, r1), IC, IinC);
                f.SetRadius(new Xy(1.0, r2), IC, IinC);
            }
            aB = f;
        }
        aB.Compute();
        return aB;
    }

    /// <summary>
    /// The cases that reach Perform, end to end: done, a valid result; and where unit 7R recorded native's result for
    /// the same case (tools/dsrec --recon), the same volume, area and face, edge and vertex counts.
    /// </summary>
    /// <summary>
    /// Cases whose result BRepCheck rejects, as native's (decision 1125: native's data structure is the same byte for
    /// byte, and native's result fails BRepCheck with the same faults).
    /// </summary>
    private static readonly HashSet<string> KnownInvalidResults =
    [
        "b3d_fail_box_top_edges_r6" /* FACE: IntersectingWires; native's too, and BadOrientationOfSubshape */,
        "pyramid_base_edges" /* UnorientableShape, SelfIntersectingWire: native's the same, its geometry equal */,
    ];

    /// <summary>
    /// Cases whose native result is degenerate (faces of negative area): not compared with it (decision 1125).
    /// </summary>
    private static readonly HashSet<string> NativeDegenerate = []; // b3d_fail_box_top_edges_r6 is native's since unit 7R3 (decision 1158)

    [Theory]
    [MemberData(nameof(CaseNames))]
    public void TheWholePipelineGivesAResult(string theName)
    {
        DsCase aCase = DsRecordings.Cases[theName];
        if (aCase.PerformCalls == 0)
        {
            return;
        }
        FilletBuilderBase aB = RunEndToEnd(aCase);
        Assert.True(aB.IsDone, theName);
        Shape aMine = aB.Shape();
        bool aValid = new Analyzer(aMine).IsValid();
        // attributed to the reconstruction (unit 7R), whose recording has no native result for these unit 7N2 cases
        Assert.True(aValid != KnownInvalidResults.Contains(theName), theName + " " + ReconComparison.Faults(aMine));
        if (!NativeDegenerate.Contains(theName) && ReconRecordings.Cases.TryGetValue(theName, out ReconCase? aRecon)
            && aRecon.Result is Shape aNative)
        {
            if (BallVolumes.TryGetValue(theName, out double aBall))
            {
                Assert.Equal(aBall, ReconComparison.Volume(aMine), 1.0e-6 * aBall);
                return;
            }
            else
            {
                Assert.Equal(ReconComparison.Volume(aNative), ReconComparison.Volume(aMine), 1.0e-6 * Math.Abs(ReconComparison.Volume(aNative)) + 1.0e-9);
                Assert.Equal(ReconComparison.Area(aNative), ReconComparison.Area(aMine), 1.0e-6 * ReconComparison.Area(aNative));
            }
            foreach (ShapeType T in new[] { ShapeType.Solid, ShapeType.Shell, ShapeType.Face, ShapeType.Edge, ShapeType.Vertex })
            {
                Assert.Equal(ReconComparison.Count(aNative, T), ReconComparison.Count(aMine, T));
            }
        }
    }
}
