// OCCT.NET: the seam between the fillet builder (ChFi3d_Builder, unit 7N2) and the topological reconstruction that
// replaces TopOpeBRepBuild_HBuilder (unit 7R, option C of docs/phase7-plan.md 2.1, decision 992). Not a port of an
// OCCT file: its members are exactly the HBuilder members TKFillet calls.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// What <c>ChFi3d_Builder</c> asks of OCCT's <c>TopOpeBRepBuild_HBuilder</c> (its field <c>myCoup</c>): build the
/// topology from the legacy data structure the fillet filled, then answer for the shapes.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's fillets never build a face themselves: <c>ChFi3d_Builder::Compute</c> fills a
/// <see cref="TopOpeBRepDS_HDataStructure"/> (surfaces, curves, points and their interferences with the input's
/// shapes) and hands it to <c>myCoup->Perform(myDS)</c>, which splits the faces and edges, builds the fillet faces and
/// assembles the shells and solids. OCCT.NET does not port TopOpeBRepBuild (decision 992): unit 7R implements this
/// interface on the Boolean engine. The call sites, in OCCT's order:
/// </para>
/// <list type="bullet">
/// <item><description><c>ChFi3d_Builder::Compute</c> (ChFi3d_Builder.cxx:471–574): <see cref="Perform"/> once, then
/// <see cref="MergeSolid"/> (state IN) for every solid and shell of the input, <see cref="IsSplit"/> and
/// <see cref="Splits"/> (IN) for every edge of the data structure (raising the tolerances of the split edges'
/// vertices), <see cref="Merged"/> (IN) for every solid and shell to assemble the result (or <c>badShape</c>), then
/// <see cref="NewFaces"/> for every surface (same-parameter on the new faces).</description></item>
/// <item><description><c>ChFi3d_Builder::Generated</c> (ChFi3d_Builder.cxx:968): <see cref="NewFaces"/> of the
/// surfaces made from an edge or vertex.</description></item>
/// <item><description><c>ChFi3d_FilBuilder::SetRegul</c> (:2606–2624) and <c>ChFi3d_ChBuilder::SetRegul</c>
/// (:2236–2243): <see cref="NewEdges"/> of a regularity's curve, <see cref="NewFaces"/> of its surfaces and
/// <see cref="Merged"/> (IN) of its faces, to code the edges' continuities.</description></item>
/// <item><description>BRepFilletAPI_MakeFillet and MakeChamfer (unit 7N3) through <c>Builder()</c>:
/// <see cref="DataStructure"/> (<c>NbSurfaces</c>), <see cref="NewFaces"/>, and <see cref="IsSplit"/> and
/// <see cref="Splits"/> with the states OUT, IN and ON (the history of the input's faces).</description></item>
/// </list>
/// <para>
/// The lists returned are the reconstruction's own (OCCT returns <c>const&amp;</c> into the builder); callers only
/// read them. Until 7R lands, <see cref="Create"/> gives a reconstruction whose
/// <see cref="Perform"/> throws <see cref="NotImplementedException"/>("7R"), so <c>Compute</c> stops exactly where
/// OCCT's hands over, with the data structure complete.
/// </para>
/// </remarks>
internal abstract class ChFi3d_Reconstruction
{
    /// <summary><c>TopOpeBRepBuild_HBuilder::Perform(HDS)</c>: builds the topology from the data structure.</summary>
    public abstract void Perform(TopOpeBRepDS_HDataStructure HDS);

    /// <summary><c>MergeSolid(S, State)</c>: assembles the parts of the solid or shell <paramref name="S"/> (a shape
    /// of the data structure) in the state <paramref name="TB"/>.</summary>
    public abstract void MergeSolid(in TopoDS_Shape S, TopAbs_State TB);

    /// <summary><c>IsSplit(S, State)</c>: true if the edge or face <paramref name="S"/> was split, with parts in the
    /// state <paramref name="ToBuild"/>.</summary>
    public abstract bool IsSplit(in TopoDS_Shape S, TopAbs_State ToBuild);

    /// <summary><c>Splits(S, State)</c>: the parts of <paramref name="S"/> in the state <paramref name="ToBuild"/>.</summary>
    public abstract NCollection_List<TopoDS_Shape> Splits(in TopoDS_Shape S, TopAbs_State ToBuild);

    /// <summary><c>Merged(S, State)</c>: what <see cref="MergeSolid"/> made of <paramref name="S"/> (empty if it was
    /// not merged).</summary>
    public abstract NCollection_List<TopoDS_Shape> Merged(in TopoDS_Shape S, TopAbs_State ToBuild);

    /// <summary><c>NewFaces(I)</c>: the faces built on the data structure's surface <paramref name="I"/>.</summary>
    public abstract NCollection_List<TopoDS_Shape> NewFaces(int I);

    /// <summary><c>NewEdges(I)</c>: the edges built on the data structure's curve <paramref name="I"/>.</summary>
    public abstract NCollection_List<TopoDS_Shape> NewEdges(int I);

    /// <summary><c>DataStructure()</c>: the data structure given to <see cref="Perform"/> (null before).</summary>
    public abstract TopOpeBRepDS_HDataStructure? DataStructure();

    // Where unit 7R plugs in.

    /// <summary>
    /// The reconstruction for a builder: OCCT's <c>new TopOpeBRepBuild_HBuilder(mkbuildtool())</c>
    /// (ChFi3d_Builder_1.cxx:346), <paramref name="BT"/> being the build tool OCCT gives it (B-spline curves of
    /// degree 1 approximations off, no translation, no overwrite).
    /// </summary>
    internal static ChFi3d_Reconstruction Create(TopOpeBRepDS_BuildTool BT) => new Pending(BT);

    /// <summary>The stand-in until unit 7R: <see cref="Perform"/> throws; the queries before it answer empty.</summary>
    private sealed class Pending : ChFi3d_Reconstruction
    {
        private readonly TopOpeBRepDS_BuildTool myBuildTool;

        internal Pending(TopOpeBRepDS_BuildTool BT) => myBuildTool = BT;

        public override void Perform(TopOpeBRepDS_HDataStructure HDS) =>
            throw new NotImplementedException("7R: the fillet's topological reconstruction (TopOpeBRepBuild_HBuilder::Perform)");

        public override void MergeSolid(in TopoDS_Shape S, TopAbs_State TB) => throw new NotImplementedException("7R");

        public override bool IsSplit(in TopoDS_Shape S, TopAbs_State ToBuild) => false;

        public override NCollection_List<TopoDS_Shape> Splits(in TopoDS_Shape S, TopAbs_State ToBuild) => new();

        public override NCollection_List<TopoDS_Shape> Merged(in TopoDS_Shape S, TopAbs_State ToBuild) => new();

        public override NCollection_List<TopoDS_Shape> NewFaces(int I) => new();

        public override NCollection_List<TopoDS_Shape> NewEdges(int I) => new();

        public override TopOpeBRepDS_HDataStructure? DataStructure() => null;

        public override string ToString() => "ChFi3d_Reconstruction (pending unit 7R), " + myBuildTool;
    }
}
