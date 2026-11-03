// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_SplitCommonVertex.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_SplitCommonVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Splits the vertices two wires of a face have in common: valid in the BRep model, not in STEP, so before writing
/// STEP each wire must get its own vertex.
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="Init"/>, <see cref="Perform"/>, then <see cref="Shape"/>. It writes into
/// no shape it is given: the edges holding a common vertex are copied with a new vertex
/// (<see cref="ShapeBuild_Edge"/>'s copy) and replaced through the context, whose apply builds new shapes. So frozen
/// shapes need no copy at the entry, and the context answers for the caller's shapes as in OCCT. <see cref="Shape"/>
/// freezes the result.
/// </remarks>
public class ShapeFix_SplitCommonVertex : ShapeFix_Root
{
    // OCCT declares its own private myShape, hiding ShapeFix_Root's (which the messages are attached to, and which
    // stays null here).
    private new TopoDS_Shape myShape;
#pragma warning disable CS0414 // set and never read, as in OCCT
    private TopoDS_Shape myResult;
    private int          myStatus;
#pragma warning restore CS0414

    /// <summary>Empty constructor.</summary>
    public ShapeFix_SplitCommonVertex()
    {
        myShape  = new TopoDS_Shape();
        myResult = new TopoDS_Shape();
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        SetPrecision(Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Loads the shape (and creates a context if there is none).</summary>
    public void Init(in TopoDS_Shape S)
    {
        myShape = S;
        if (Context() is null)
        {
            SetContext(new ShapeBuild_ReShape());
        }
        myResult = myShape;
        Context()!.InternalApply(myShape);
    }

    /// <summary>Splits the common vertices of the wires of each face.</summary>
    public void Perform()
    {
        TopAbs_ShapeEnum st = myShape.ShapeType();
        if (st > TopAbs_FACE)
        {
            return;
        }
        for (var itf = new TopExp_Explorer(myShape, TopAbs_FACE); itf.More(); itf.Next())
        {
            TopoDS_Shape tmpFace = Context()!.InternalApply(itf.Current());
            TopoDS_Face  F       = TopoDS.Face(tmpFace);
            if (F.IsNull())
            {
                continue;
            }
            // analys face and split if necessary
            var wires = new NCollection_Sequence<TopoDS_Shape>();
            for (var itw = new TopoDS_Iterator(F, false); itw.More(); itw.Next())
            {
                if (itw.Value().ShapeType() != TopAbs_WIRE)
                {
                    continue;
                }
                wires.Append(itw.Value());
            }
            if (wires.Length() < 2)
            {
                continue;
            }
            var MapVV = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            MapVV.Clear();
            for (int nw1 = 1; nw1 < wires.Length(); nw1++)
            {
                TopoDS_Wire w1 = TopoDS.Wire(wires.Value(nw1));
                for (int nw2 = nw1 + 1; nw2 <= wires.Length(); nw2++)
                {
                    TopoDS_Wire w2    = TopoDS.Wire(wires.Value(nw2));
                    var         sewd2 = new ShapeExtend_WireData(w2);

                    for (var expv1 = new TopExp_Explorer(w1, TopAbs_VERTEX); expv1.More(); expv1.Next())
                    {
                        TopoDS_Vertex V1 = TopoDS.Vertex(expv1.Current());
                        for (var expv2 = new TopExp_Explorer(w2, TopAbs_VERTEX); expv2.More(); expv2.Next())
                        {
                            TopoDS_Vertex V2 = TopoDS.Vertex(expv2.Current());
                            if (V1 == V2)
                            {
                                // common vertex exists
                                TopoDS_Vertex Vnew = new();
                                if (MapVV.IsBound(V2))
                                {
                                    Vnew = TopoDS.Vertex(MapVV.Find(V2));
                                }
                                else
                                {
                                    Point3D P   = BRep_Tool.Pnt(V2);
                                    double tol = BRep_Tool.Tolerance(V2);
                                    var    B   = new BRep_Builder();
                                    B.MakeVertex(ref Vnew, P, tol);
                                    MapVV.Bind(V2, Vnew);
                                }
                                var sbe = new ShapeBuild_Edge();
                                var sae = new ShapeAnalysis_Edge();
                                for (int ne2 = 1; ne2 <= sewd2.NbEdges(); ne2++)
                                {
                                    TopoDS_Edge   E       = sewd2.Edge(ne2);
                                    TopoDS_Vertex FV      = sae.FirstVertex(E);
                                    TopoDS_Vertex LV      = sae.LastVertex(E);
                                    bool          IsCoinc = false;
                                    if (FV == V2)
                                    {
                                        FV      = Vnew;
                                        IsCoinc = true;
                                    }
                                    if (LV == V2)
                                    {
                                        LV      = Vnew;
                                        IsCoinc = true;
                                    }
                                    if (IsCoinc)
                                    {
                                        TopoDS_Edge NewE = sbe.InternalCopyReplaceVertices(E, FV, LV);
                                        Context()!.Replace(E, NewE);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (!MapVV.IsEmpty())
            {
                SendWarning(new Msg("Fix.SplitCommonVertex.MSG0"));
            }
        }

        myShape = Context()!.InternalApply(myShape);
    }

    /// <summary>Returns the resulting shape, frozen.</summary>
    public TopoDS_Shape Shape()
    {
        TopoDS_Shape aShape = myShape;
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary>The resulting shape, unfrozen, for ported code that goes on building it.</summary>
    internal TopoDS_Shape InternalShape() => myShape;
}
