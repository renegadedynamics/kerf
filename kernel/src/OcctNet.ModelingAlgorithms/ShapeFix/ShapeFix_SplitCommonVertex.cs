// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_SplitCommonVertex.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_SplitCommonVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.ShapeType;

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
    private new Shape myShape;
#pragma warning disable CS0414 // set and never read, as in OCCT
    private Shape myResult;
    private int          myStatus;
#pragma warning restore CS0414

    /// <summary>Empty constructor.</summary>
    public ShapeFix_SplitCommonVertex()
    {
        myShape  = new Shape();
        myResult = new Shape();
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        SetPrecision(Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Loads the shape (and creates a context if there is none).</summary>
    public void Init(in Shape S)
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
        ShapeType st = myShape.ShapeType();
        if (st > ShapeType.Face)
        {
            return;
        }
        for (var itf = new Explorer(myShape, ShapeType.Face); itf.More(); itf.Next())
        {
            Shape tmpFace = Context()!.InternalApply(itf.Current());
            Face  F       = ShapeCasts.Face(tmpFace);
            if (F.IsNull)
            {
                continue;
            }
            // analys face and split if necessary
            var wires = new NCollection_Sequence<Shape>();
            for (var itw = new Iterator(F, false); itw.More(); itw.Next())
            {
                if (itw.Value().ShapeType() != ShapeType.Wire)
                {
                    continue;
                }
                wires.Append(itw.Value());
            }
            if (wires.Length() < 2)
            {
                continue;
            }
            var MapVV = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
            MapVV.Clear();
            for (int nw1 = 1; nw1 < wires.Length(); nw1++)
            {
                Wire w1 = ShapeCasts.Wire(wires.Value(nw1));
                for (int nw2 = nw1 + 1; nw2 <= wires.Length(); nw2++)
                {
                    Wire w2    = ShapeCasts.Wire(wires.Value(nw2));
                    var         sewd2 = new ShapeExtend_WireData(w2);

                    for (var expv1 = new Explorer(w1, ShapeType.Vertex); expv1.More(); expv1.Next())
                    {
                        Vertex V1 = ShapeCasts.Vertex(expv1.Current());
                        for (var expv2 = new Explorer(w2, ShapeType.Vertex); expv2.More(); expv2.Next())
                        {
                            Vertex V2 = ShapeCasts.Vertex(expv2.Current());
                            if (V1 == V2)
                            {
                                // common vertex exists
                                Vertex Vnew = new();
                                if (MapVV.IsBound(V2))
                                {
                                    Vnew = ShapeCasts.Vertex(MapVV.Find(V2));
                                }
                                else
                                {
                                    Point3D P   = Tool.Pnt(V2);
                                    double tol = Tool.Tolerance(V2);
                                    var    B   = new BrepBuilder();
                                    B.MakeVertex(ref Vnew, P, tol);
                                    MapVV.Bind(V2, Vnew);
                                }
                                var sbe = new ShapeBuild_Edge();
                                var sae = new ShapeAnalysis_Edge();
                                for (int ne2 = 1; ne2 <= sewd2.NbEdges(); ne2++)
                                {
                                    Edge   E       = sewd2.Edge(ne2);
                                    Vertex FV      = sae.FirstVertex(E);
                                    Vertex LV      = sae.LastVertex(E);
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
                                        Edge NewE = sbe.InternalCopyReplaceVertices(E, FV, LV);
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
    public Shape Shape()
    {
        Shape aShape = myShape;
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary>The resulting shape, unfrozen, for ported code that goes on building it.</summary>
    internal Shape InternalShape() => myShape;
}
