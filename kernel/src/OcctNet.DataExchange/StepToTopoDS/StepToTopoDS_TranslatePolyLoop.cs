// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslatePolyLoop.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslatePolyLoop.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.StepToTopoDS_TranslatePolyLoopError;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>Translates a poly_loop (of a faceted brep) into a wire of line edges on the given planar face.</summary>
/// <remarks><see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 1247).</remarks>
public sealed class StepToTopoDS_TranslatePolyLoop : StepToTopoDS_Root
{
    private StepToTopoDS_TranslatePolyLoopError myError;
    private Shape myResult = new Shape();

    public StepToTopoDS_TranslatePolyLoop()
    {
        myError = StepToTopoDS_TranslatePolyLoopError.StepToTopoDS_TranslatePolyLoopOther;
        done = false;
    }

    public StepToTopoDS_TranslatePolyLoop(StepShape_PolyLoop PL, StepToTopoDS_Tool T, Surface S, in Face F,
                                          in StepData_Factors theLocalFactors)
    {
        Init(PL, T, S, F, theLocalFactors);
    }

    public StepToTopoDS_TranslatePolyLoop(StepShape_PolyLoop PL, StepToTopoDS_Tool T, Surface S, in Face F)
        : this(PL, T, S, F, new StepData_Factors())
    {
    }

    /// <remarks>Writes the pcurves of the edges into <paramref name="TopoFace"/>, which must be the reader's own.</remarks>
    public void Init(StepShape_PolyLoop PL, StepToTopoDS_Tool aTool, Surface GeomSurf, in Face TopoFace,
                     in StepData_Factors theLocalFactors)
    {
        if (!aTool.IsBound(PL))
        {
            var B = new BrepBuilder();
            Transfer_TransientProcess TP = aTool.TransientProcess();

            //: S4136    double preci = BRepAPI::Precision();
            int i;
            StepGeom_CartesianPoint P1, P2;
            CartesianPoint GP1, GP2;
            Vertex V1 = new Vertex(), V2 = new Vertex();
            Edge E = new Edge();
            Wire W = new Wire();
            LineCurve3D L;
            LineCurve2D L2d;
            Vector3D V;
            Vector2D V2d;
            double Magn;
            PlaneSurface SP = GeomSurf as PlaneSurface;
            if (SP is null)
            {
                TP.AddFail(PL, "Surface not planar in a FacetedBRep !");
            }

            var STSU = new SurfaceAnalysis(GeomSurf);
            int Nb = PL.NbPolygon();
            var Poly = new NCollection_HArray1<StepGeom_CartesianPoint>(1, Nb + 1);

            for (i = 1; i <= Nb; i++)
            {
                Poly.SetValue(i, PL.PolygonValue(i));
            }

            Nb++;
            Poly.SetValue(Nb, PL.PolygonValue(1));
            P1 = Poly.Value(1);
            GP1 = StepToGeom.MakeCartesianPoint(P1, theLocalFactors);
            if (aTool.IsVertexBound(P1))
            {
                V1 = aTool.FindVertex(P1);
            }
            else
            {
                B.MakeVertex(ref V1, GP1.Pnt(), Kerf.Engine.Foundation.Precision.Confusion()); //: S4136: preci
                aTool.BindVertex(P1, V1);
            }

            B.MakeWire(ref W);
            for (i = 2; i <= Nb; i++)
            {
                P2 = Poly.Value(i);
                if (P1 == P2)
                {
                    continue; // peut arriver (KK)  CKY 9-DEC-1997
                }

                var PP = new StepToTopoDS_PointPair(P1, P2);
                GP2 = StepToGeom.MakeCartesianPoint(P2, theLocalFactors);
                Shape aBoundEdge;
                bool isbound = aTool.IsEdgeBound(PP);
                if (!isbound)
                {
                    if (aTool.IsVertexBound(P2))
                    {
                        V2 = aTool.FindVertex(P2);
                    }
                    else
                    {
                        B.MakeVertex(ref V2, GP2.Pnt(), Kerf.Engine.Foundation.Precision.Confusion()); //: S4136: preci
                        aTool.BindVertex(P2, V2);
                    }

                    V = new Vector3D(GP1.Pnt(), GP2.Pnt());
                    L = new LineCurve3D(GP1.Pnt(), new Direction3D(V));
                    B.MakeEdge(ref E, L, Kerf.Engine.Foundation.Precision.Confusion()); //: S4136: preci
                    V1.Orientation(Forward);
                    V2.Orientation(Reversed);
                    B.Add(E, V1);
                    B.Add(E, V2);
                    Magn = V.Magnitude();
                    B.UpdateVertex(V1, 0.0, E, 0.0);  //: S4136: preci
                    B.UpdateVertex(V2, Magn, E, 0.0); //: S4136: preci
                }
                else
                {
                    aBoundEdge = aTool.FindEdge(PP);
                    E = ShapeCasts.Edge(aBoundEdge);
                    //  Il faut qu en finale l edge soit vue
                    //  - via sa premiere face, orientation combinee = celle de cette premiere face
                    //  - via sa deuxieme face, orientation combinee INVERSE de la precedente
                    if (TopoFace.Orientation() == Forward)
                    {
                        E.Reverse();
                    }

                    V2 = aTool.FindVertex(P2);
                }

                Point2D V2p1 = STSU.ValueOfUV(GP1.Pnt(), Precision());
                Point2D V2p2 = STSU.ValueOfUV(GP2.Pnt(), Precision());
                if (E.Orientation() == Forward)
                {
                    V2d = new Vector2D(V2p1, V2p2);
                    L2d = new LineCurve2D(V2p1, new Direction2D(V2d));
                }
                else
                {
                    V2d = new Vector2D(V2p2, V2p1);
                    L2d = new LineCurve2D(V2p2, new Direction2D(V2d));
                }

                B.UpdateEdge(E, L2d, TopoFace, 0.0);
                Edge EB = E; // pour le binding : cumul des orientations !
                EB.Orientation(TopoFace.Orientation());
                if (!isbound)
                {
                    aTool.BindEdge(PP, EB);
                }

                if (!E.IsNull)
                {
                    B.Add(W, E);
                }

                P1 = P2;
                GP1 = GP2;
                V1 = V2;
            }

            W.Closed(Tool.IsClosed(W));
            aTool.Bind(PL, W);
            myResult = W;
            myError = StepToTopoDS_TranslatePolyLoopError.StepToTopoDS_TranslatePolyLoopDone;
            done = true;
        }
        else
        {
            myResult = ShapeCasts.Wire(aTool.Find(PL));
            myError = StepToTopoDS_TranslatePolyLoopError.StepToTopoDS_TranslatePolyLoopDone;
            done = true;
        }
    }

    public void Init(StepShape_PolyLoop PL, StepToTopoDS_Tool aTool, Surface GeomSurf, in Face TopoFace)
        => Init(PL, aTool, GeomSurf, TopoFace, new StepData_Factors());

    /// <summary>The wire, frozen.</summary>
    /// <exception cref="NotDoneException">The translation wasn't done.</exception>
    public Shape Value()
    {
        Shape aShape = InternalValue();
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!done, "StepToTopoDS_TranslatePolyLoop::Value() - no result");
        return myResult;
    }

    public StepToTopoDS_TranslatePolyLoopError Error() => myError;
}
