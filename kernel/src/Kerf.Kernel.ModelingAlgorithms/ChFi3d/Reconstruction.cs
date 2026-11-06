// Kerf: the seam between the fillet builder (ChFi3d_Builder, unit 7N2) and the topological reconstruction that
// replaces TopOpeBRepBuild_HBuilder (unit 7R, option C of docs/phase7-plan.md 2.1, decision 992). Not a port of an
// OCCT file: its members are exactly the HBuilder members TKFillet calls.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Topology;

namespace Kerf.Engine.Fillets;

/// <summary>
/// What <c>ChFi3d_Builder</c> asks of OCCT's <c>TopOpeBRepBuild_HBuilder</c> (its field <c>myCoup</c>): build the
/// topology from the legacy data structure the fillet filled, then answer for the shapes.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's fillets never build a face themselves: <c>ChFi3d_Builder::Compute</c> fills a
/// <see cref="DsHDataStructure"/> (surfaces, curves, points and their interferences with the input's
/// shapes) and hands it to <c>myCoup->Perform(myDS)</c>, which splits the faces and edges, builds the fillet faces and
/// assembles the shells and solids. Kerf does not port TopOpeBRepBuild (decision 992): unit 7R implements this
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
/// read them. <see cref="Create"/> gives unit 7R's <see cref="BOPReconstruction"/>.
/// </para>
/// </remarks>
internal abstract class Reconstruction
{
    /// <summary><c>TopOpeBRepBuild_HBuilder::Perform(HDS)</c>: builds the topology from the data structure.</summary>
    public abstract void Perform(DsHDataStructure hds);

    /// <summary><c>MergeSolid(S, State)</c>: assembles the parts of the solid or shell <paramref name="s"/> (a shape
    /// of the data structure) in the state <paramref name="tb"/>.</summary>
    public abstract void MergeSolid(in Shape s, Containment tb);

    /// <summary><c>IsSplit(S, State)</c>: true if the edge or face <paramref name="s"/> was split, with parts in the
    /// state <paramref name="toBuild"/>.</summary>
    public abstract bool IsSplit(in Shape s, Containment toBuild);

    /// <summary><c>Splits(S, State)</c>: the parts of <paramref name="s"/> in the state <paramref name="toBuild"/>.</summary>
    public abstract NCollection_List<Shape> Splits(in Shape s, Containment toBuild);

    /// <summary><c>Merged(S, State)</c>: what <see cref="MergeSolid"/> made of <paramref name="s"/> (empty if it was
    /// not merged).</summary>
    public abstract NCollection_List<Shape> Merged(in Shape s, Containment toBuild);

    /// <summary><c>NewFaces(I)</c>: the faces built on the data structure's surface <paramref name="i"/>.</summary>
    public abstract NCollection_List<Shape> NewFaces(int i);

    /// <summary><c>NewEdges(I)</c>: the edges built on the data structure's curve <paramref name="i"/>.</summary>
    public abstract NCollection_List<Shape> NewEdges(int i);

    /// <summary><c>DataStructure()</c>: the data structure given to <see cref="Perform"/> (null before).</summary>
    public abstract DsHDataStructure? DataStructure();

    // Where unit 7R plugs in.

    /// <summary>
    /// The reconstruction for a builder: OCCT's <c>new TopOpeBRepBuild_HBuilder(mkbuildtool())</c>
    /// (ChFi3d_Builder_1.cxx:346), <paramref name="bt"/> being the build tool OCCT gives it (B-spline curves of
    /// degree 1 approximations off, no translation, no overwrite).
    /// </summary>
    internal static Reconstruction Create(DsBuildTool bt) => new BOPReconstruction(bt);
}
