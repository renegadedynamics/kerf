// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslatePolyLoop.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslatePolyLoop.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.StepToTopoDS_TranslatePolyLoopError;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>Translates a poly_loop (of a faceted brep) into a wire of line edges on the given planar face.</summary>
/// <remarks><see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 1247).</remarks>
public sealed class StepToTopoDS_TranslatePolyLoop : StepToTopoDS_Root
{
    private StepToTopoDS_TranslatePolyLoopError myError;
    private TopoDS_Shape myResult = new TopoDS_Shape();

    public StepToTopoDS_TranslatePolyLoop()
    {
        myError = StepToTopoDS_TranslatePolyLoopOther;
        done = false;
    }

    public StepToTopoDS_TranslatePolyLoop(StepShape_PolyLoop PL, StepToTopoDS_Tool T, Geom_Surface S, in TopoDS_Face F,
                                          in StepData_Factors theLocalFactors)
    {
        Init(PL, T, S, F, theLocalFactors);
    }

    public StepToTopoDS_TranslatePolyLoop(StepShape_PolyLoop PL, StepToTopoDS_Tool T, Geom_Surface S, in TopoDS_Face F)
        : this(PL, T, S, F, new StepData_Factors())
    {
    }

    /// <remarks>Writes the pcurves of the edges into <paramref name="TopoFace"/>, which must be the reader's own.</remarks>
    public void Init(StepShape_PolyLoop PL, StepToTopoDS_Tool aTool, Geom_Surface GeomSurf, in TopoDS_Face TopoFace,
                     in StepData_Factors theLocalFactors)
    {
        if (!aTool.IsBound(PL))
        {
            var B = new BRep_Builder();
            Transfer_TransientProcess TP = aTool.TransientProcess();

            //: S4136    double preci = BRepAPI::Precision();
            int i;
            StepGeom_CartesianPoint P1, P2;
            Geom_CartesianPoint GP1, GP2;
            TopoDS_Vertex V1 = new TopoDS_Vertex(), V2 = new TopoDS_Vertex();
            TopoDS_Edge E = new TopoDS_Edge();
            TopoDS_Wire W = new TopoDS_Wire();
            Geom_Line L;
            Geom2d_Line L2d;
            gp_Vec V;
            gp_Vec2d V2d;
            double Magn;
            Geom_Plane SP = GeomSurf as Geom_Plane;
            if (SP is null)
            {
                TP.AddFail(PL, "Surface not planar in a FacetedBRep !");
            }

            var STSU = new ShapeAnalysis_Surface(GeomSurf);
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
                B.MakeVertex(ref V1, GP1.Pnt(), Occt.Precision.Confusion()); //: S4136: preci
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
                TopoDS_Shape aBoundEdge;
                bool isbound = aTool.IsEdgeBound(PP);
                if (!isbound)
                {
                    if (aTool.IsVertexBound(P2))
                    {
                        V2 = aTool.FindVertex(P2);
                    }
                    else
                    {
                        B.MakeVertex(ref V2, GP2.Pnt(), Occt.Precision.Confusion()); //: S4136: preci
                        aTool.BindVertex(P2, V2);
                    }

                    V = new gp_Vec(GP1.Pnt(), GP2.Pnt());
                    L = new Geom_Line(GP1.Pnt(), new gp_Dir(V));
                    B.MakeEdge(ref E, L, Occt.Precision.Confusion()); //: S4136: preci
                    V1.Orientation(TopAbs_FORWARD);
                    V2.Orientation(TopAbs_REVERSED);
                    B.Add(E, V1);
                    B.Add(E, V2);
                    Magn = V.Magnitude();
                    B.UpdateVertex(V1, 0.0, E, 0.0);  //: S4136: preci
                    B.UpdateVertex(V2, Magn, E, 0.0); //: S4136: preci
                }
                else
                {
                    aBoundEdge = aTool.FindEdge(PP);
                    E = TopoDS.Edge(aBoundEdge);
                    //  Il faut qu en finale l edge soit vue
                    //  - via sa premiere face, orientation combinee = celle de cette premiere face
                    //  - via sa deuxieme face, orientation combinee INVERSE de la precedente
                    if (TopoFace.Orientation() == TopAbs_FORWARD)
                    {
                        E.Reverse();
                    }

                    V2 = aTool.FindVertex(P2);
                }

                gp_Pnt2d V2p1 = STSU.ValueOfUV(GP1.Pnt(), Precision());
                gp_Pnt2d V2p2 = STSU.ValueOfUV(GP2.Pnt(), Precision());
                if (E.Orientation() == TopAbs_FORWARD)
                {
                    V2d = new gp_Vec2d(V2p1, V2p2);
                    L2d = new Geom2d_Line(V2p1, new gp_Dir2d(V2d));
                }
                else
                {
                    V2d = new gp_Vec2d(V2p2, V2p1);
                    L2d = new Geom2d_Line(V2p2, new gp_Dir2d(V2d));
                }

                B.UpdateEdge(E, L2d, TopoFace, 0.0);
                TopoDS_Edge EB = E; // pour le binding : cumul des orientations !
                EB.Orientation(TopoFace.Orientation());
                if (!isbound)
                {
                    aTool.BindEdge(PP, EB);
                }

                if (!E.IsNull())
                {
                    B.Add(W, E);
                }

                P1 = P2;
                GP1 = GP2;
                V1 = V2;
            }

            W.Closed(BRep_Tool.IsClosed(W));
            aTool.Bind(PL, W);
            myResult = W;
            myError = StepToTopoDS_TranslatePolyLoopDone;
            done = true;
        }
        else
        {
            myResult = TopoDS.Wire(aTool.Find(PL));
            myError = StepToTopoDS_TranslatePolyLoopDone;
            done = true;
        }
    }

    public void Init(StepShape_PolyLoop PL, StepToTopoDS_Tool aTool, Geom_Surface GeomSurf, in TopoDS_Face TopoFace)
        => Init(PL, aTool, GeomSurf, TopoFace, new StepData_Factors());

    /// <summary>The wire, frozen.</summary>
    /// <exception cref="StdFail_NotDone">The translation wasn't done.</exception>
    public TopoDS_Shape Value()
    {
        TopoDS_Shape aShape = InternalValue();
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal TopoDS_Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!done, "StepToTopoDS_TranslatePolyLoop::Value() - no result");
        return myResult;
    }

    public StepToTopoDS_TranslatePolyLoopError Error() => myError;
}
