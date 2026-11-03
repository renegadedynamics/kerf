// Port of: src/ModelingData/TKGeomBase/GTests/AdvApp2Var_Framework_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using static Kerf.Engine.Geometry.IsoType;

namespace OcctNet.ModelingData.Tests;

public class AdvApp2Var_FrameworkTest
{
    private static NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>> makeUFrontier()
    {
        NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>> aUFrontier = new();

        NCollection_Sequence<AdvApp2Var_Iso> aUStrip1 = new();
        aUStrip1.Append(new AdvApp2Var_Iso(IsoV, 30.0, 0.0, 1.0, 0.0, 1.0, 0, 0, 0));
        aUStrip1.Append(new AdvApp2Var_Iso(IsoV, 40.0, 0.0, 1.0, 0.0, 1.0, 0, 0, 0));
        aUFrontier.Append(aUStrip1);

        NCollection_Sequence<AdvApp2Var_Iso> aUStrip2 = new();
        aUStrip2.Append(new AdvApp2Var_Iso(IsoV, 30.0, 1.0, 2.0, 0.0, 1.0, 0, 0, 0));
        aUStrip2.Append(new AdvApp2Var_Iso(IsoV, 40.0, 1.0, 2.0, 0.0, 1.0, 0, 0, 0));
        aUFrontier.Append(aUStrip2);

        return aUFrontier;
    }

    private static NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>> makeVFrontier()
    {
        NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>> aVFrontier = new();

        NCollection_Sequence<AdvApp2Var_Iso> aVStrip1 = new();
        aVStrip1.Append(new AdvApp2Var_Iso(IsoU, 10.0, 0.0, 1.0, 0.0, 1.0, 0, 0, 0));
        aVStrip1.Append(new AdvApp2Var_Iso(IsoU, 20.0, 0.0, 1.0, 0.0, 1.0, 0, 0, 0));
        aVFrontier.Append(aVStrip1);

        NCollection_Sequence<AdvApp2Var_Iso> aVStrip2 = new();
        aVStrip2.Append(new AdvApp2Var_Iso(IsoU, 10.0, 0.0, 1.0, 1.0, 2.0, 0, 0, 0));
        aVStrip2.Append(new AdvApp2Var_Iso(IsoU, 20.0, 0.0, 1.0, 1.0, 2.0, 0, 0, 0));
        aVFrontier.Append(aVStrip2);

        return aVFrontier;
    }

    private static NCollection_Sequence<AdvApp2Var_Node> makeNodeGrid()
    {
        NCollection_Sequence<AdvApp2Var_Node> aNodes = new();
        aNodes.Append(new AdvApp2Var_Node(new Xy(0.0, 0.0), 0, 0));
        aNodes.Append(new AdvApp2Var_Node(new Xy(1.0, 0.0), 0, 0));
        aNodes.Append(new AdvApp2Var_Node(new Xy(0.0, 1.0), 0, 0));
        aNodes.Append(new AdvApp2Var_Node(new Xy(1.0, 1.0), 0, 0));
        return aNodes;
    }

    [Fact]
    public void IsoLookup_ReturnsMatchingIso()
    {
        NCollection_Sequence<AdvApp2Var_Node> aNodes = makeNodeGrid();
        AdvApp2Var_Framework aFramework = new(aNodes, makeUFrontier(), makeVFrontier());

        AdvApp2Var_Iso anIsoU = aFramework.IsoU(20.0, 0.0, 1.0);
        Assert.Equal(IsoU, anIsoU.Type());
        Assert.Equal(20.0, anIsoU.Constante());
        Assert.Equal(0.0, anIsoU.T0());
        Assert.Equal(1.0, anIsoU.T1());

        AdvApp2Var_Iso anIsoV = aFramework.IsoV(1.0, 2.0, 40.0);
        Assert.Equal(IsoV, anIsoV.Type());
        Assert.Equal(40.0, anIsoV.Constante());
        Assert.Equal(1.0, anIsoV.T0());
        Assert.Equal(2.0, anIsoV.T1());
    }

    [Fact]
    public void NodeIndexMapping_UsesGridLayout()
    {
        NCollection_Sequence<AdvApp2Var_Node> aNodes = makeNodeGrid();
        AdvApp2Var_Framework aFramework = new(aNodes, makeUFrontier(), makeVFrontier());

        Assert.Equal(2, aFramework.FirstNode(IsoU, 2, 1));
        Assert.Equal(5, aFramework.LastNode(IsoU, 2, 1));

        Assert.Equal(4, aFramework.FirstNode(IsoV, 2, 1));
        Assert.Equal(5, aFramework.LastNode(IsoV, 2, 1));
    }

    [Fact]
    public void UpdateInU_InsertsCutIsoAndNodes()
    {
        NCollection_Sequence<AdvApp2Var_Node> aNodes = makeNodeGrid();
        AdvApp2Var_Framework aFramework = new(aNodes, makeUFrontier(), makeVFrontier());
        aFramework.UpdateInU(0.5);

        AdvApp2Var_Iso aCutIso = aFramework.IsoU(0.5, 0.0, 1.0);
        Assert.Equal(IsoU, aCutIso.Type());
        Assert.Equal(0.5, aCutIso.Constante());
        Assert.Equal(0.0, aCutIso.T0());
        Assert.Equal(1.0, aCutIso.T1());

        AdvApp2Var_Node aBottomCutNode = aFramework.Node(0.5, 0.0);
        AdvApp2Var_Node aTopCutNode = aFramework.Node(0.5, 1.0);
        Assert.NotNull(aBottomCutNode);
        Assert.NotNull(aTopCutNode);
        Assert.Equal(0.5, aBottomCutNode.Coord().X());
        Assert.Equal(0.0, aBottomCutNode.Coord().Y());
        Assert.Equal(0.5, aTopCutNode.Coord().X());
        Assert.Equal(1.0, aTopCutNode.Coord().Y());

        Assert.Equal(2, aFramework.FirstNode(IsoU, 2, 1));
        Assert.Equal(6, aFramework.LastNode(IsoU, 2, 1));
    }

    // The port: the constructor copies the strips, as OCCT's copies its sequences (decision 194).
    [Fact]
    public void Constructor_CopiesTheSequences()
    {
        NCollection_Sequence<AdvApp2Var_Node> aNodes = makeNodeGrid();
        NCollection_Sequence<NCollection_Sequence<AdvApp2Var_Iso>> aUFrontier = makeUFrontier();
        AdvApp2Var_Framework aFramework = new(aNodes, aUFrontier, makeVFrontier());
        aFramework.UpdateInU(0.5);
        Assert.Equal(4, aNodes.Length());
        Assert.Equal(2, aUFrontier.Length());
        Assert.Equal(2, aUFrontier.Value(1).Length());
    }
}
