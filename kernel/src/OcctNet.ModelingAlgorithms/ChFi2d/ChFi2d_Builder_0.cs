// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_Builder_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Modified : 08/07/97 : JPI : traitement des edges degeneres comme pour les fillets 2D
// Modified:	Fri Sep 25 09:38:04 1998
//              status = ChFi2d_NotAuthorized si les aretes ne sont pas
//              des droites ou des cercles; fonction IsLineOrCircle
//              (BUC60288)

using static Occt.ChFi2d_ConstructionError;
using static Occt.GeomAbs_CurveType;
using static Occt.TopAbs_Orientation;

namespace Occt;

public sealed partial class ChFi2d_Builder
{
    //=================================================================================================

    /// <summary>
    /// Adds a chamfer between the two connected edges <paramref name="E1"/> and <paramref name="E2"/>, at the
    /// distances <paramref name="D1"/> along <paramref name="E1"/> and <paramref name="D2"/> along <paramref name="E2"/>,
    /// and returns it (frozen). The edge has sense only if <see cref="Status"/> is <see cref="ChFi2d_IsDone"/>.
    /// </summary>
    public TopoDS_Edge AddChamfer(in TopoDS_Edge E1, in TopoDS_Edge E2, double D1, double D2)
    {
        return Published(InternalAddChamfer(E1, E2, D1, D2));
    }

    private TopoDS_Edge InternalAddChamfer(in TopoDS_Edge E1, in TopoDS_Edge E2, double D1, double D2)
    {
        TopoDS_Vertex commonVertex = new();
        TopoDS_Edge basisEdge1, basisEdge2;
        TopoDS_Edge E1Mod = new(), E2Mod = new(), chamfer = new();

        bool hasConnection = ChFi2d.CommonVertex(E1, E2, ref commonVertex);
        if (!hasConnection)
        {
            return chamfer;
        }

        if (IsAFillet(E1) || IsAChamfer(E1) || IsAFillet(E2) || IsAChamfer(E2))
        {
            status = ChFi2d_NotAuthorized;
            return chamfer;
        } //  if (IsAChamfer ...

        if (!IsLineOrCircle(E1, newFace) || !IsLineOrCircle(E2, newFace))
        {
            status = ChFi2d_NotAuthorized;
            return chamfer;
        } //  if (!IsLineOrCircle ...

        // EE1 and EE2 are copies of E1 and E2 with the good orientation
        // on <newFace>
        TopoDS_Edge EE1 = new(), EE2 = new();
        status = ChFi2d.FindConnectedEdges(newFace, commonVertex, ref EE1, ref EE2);
        if (EE1.IsSame(E2))
        {
            TopAbs_Orientation orient = EE1.Orientation();
            EE1 = EE2;
            EE2 = E2;
            EE2.Orientation(orient);
        }

        ComputeChamfer(commonVertex, EE1, EE2, D1, D2, ref E1Mod, ref E2Mod, ref chamfer);
        if (status == ChFi2d_IsDone || status == ChFi2d_FirstEdgeDegenerated
            || status == ChFi2d_LastEdgeDegenerated || status == ChFi2d_BothEdgesDegenerated)
        {
            //  if (status == ChFi2d_IsDone) {
            BuildNewWire(EE1, EE2, E1Mod, chamfer, E2Mod);
            basisEdge1 = BasisEdge(EE1);
            basisEdge2 = BasisEdge(EE2);
            UpDateHistory(basisEdge1, basisEdge2, E1Mod, E2Mod, chamfer, 2);
            status = ChFi2d_IsDone;
            return TopoDS.Edge(chamfers.Value(chamfers.Length()));
        }
        return chamfer;
    } // AddChamfer

    //=================================================================================================

    /// <summary>
    /// Adds a chamfer between the two edges connected to the vertex <paramref name="V"/>, making the angle
    /// <paramref name="Ang"/> (radians) with the edge <paramref name="E"/>, one of its ends on <paramref name="E"/> at the
    /// distance <paramref name="D"/>; returns it (frozen). The edge has sense only if <see cref="Status"/> is
    /// <see cref="ChFi2d_IsDone"/>.
    /// </summary>
    public TopoDS_Edge AddChamfer(in TopoDS_Edge E, in TopoDS_Vertex V, double D, double Ang)
    {
        return Published(InternalAddChamfer(E, V, D, Ang));
    }

    private TopoDS_Edge InternalAddChamfer(in TopoDS_Edge E, in TopoDS_Vertex V, double D, double Ang)
    {
        TopoDS_Edge aChamfer = new(), adjEdge1 = new(), adjEdge2 = new();
        status = ChFi2d.FindConnectedEdges(newFace, V, ref adjEdge1, ref adjEdge2);
        if (status == ChFi2d_ConnexionError)
        {
            return aChamfer;
        }

        // adjEdge1 is a copy of E  with the good orientation
        // on <newFace>
        if (adjEdge2.IsSame(E))
        {
            TopAbs_Orientation orient = adjEdge2.Orientation();
            adjEdge2 = adjEdge1;
            adjEdge1 = E;
            adjEdge1.Orientation(orient);
        }

        if (IsAFillet(adjEdge1) || IsAChamfer(adjEdge1) || IsAFillet(adjEdge2) || IsAChamfer(adjEdge2))
        {
            status = ChFi2d_NotAuthorized;
            return aChamfer;
        } //  if (IsAChamfer ...

        if (!IsLineOrCircle(adjEdge1, newFace) || !IsLineOrCircle(adjEdge2, newFace))
        {
            status = ChFi2d_NotAuthorized;
            return aChamfer;
        } //  if (!IsLineOrCircle ...

        TopoDS_Edge E1 = new(), E2 = new();
        ComputeChamfer(V, adjEdge1, D, Ang, adjEdge2, ref E1, ref E2, ref aChamfer);
        TopoDS_Edge basisEdge1, basisEdge2;
        if (status == ChFi2d_IsDone || status == ChFi2d_FirstEdgeDegenerated
            || status == ChFi2d_LastEdgeDegenerated || status == ChFi2d_BothEdgesDegenerated)
        {
            //  if (status == ChFi2d_IsDone) {
            BuildNewWire(adjEdge1, adjEdge2, E1, aChamfer, E2);
            basisEdge1 = BasisEdge(adjEdge1);
            basisEdge2 = BasisEdge(adjEdge2);
            UpDateHistory(basisEdge1, basisEdge2, E1, E2, aChamfer, 2);
            status = ChFi2d_IsDone;
            return TopoDS.Edge(chamfers.Value(chamfers.Length()));
        }
        return aChamfer;
    }

    //=================================================================================================

    // The chamfer computed from a vertex, two edges and two distances. TrimE1, TrimE2 and Chamfer have sense only if
    // the status is IsDone.
    private void ComputeChamfer(in TopoDS_Vertex V,
                                in TopoDS_Edge E1,
                                in TopoDS_Edge E2,
                                double D1,
                                double D2,
                                ref TopoDS_Edge TrimE1,
                                ref TopoDS_Edge TrimE2,
                                ref TopoDS_Edge Chamfer)
    {
        TopoDS_Vertex newExtr1 = new(), newExtr2 = new();
        bool Degen1, Degen2;
        Chamfer = BuildChamferEdge(V, E1, E2, D1, D2, ref newExtr1, ref newExtr2);
        if (status != ChFi2d_IsDone)
        {
            return;
        }
        TrimE1 = BuildNewEdge(E1, V, newExtr1, out Degen1);
        TrimE2 = BuildNewEdge(E2, V, newExtr2, out Degen2);
        if (Degen1 && Degen2)
        {
            status = ChFi2d_BothEdgesDegenerated;
        }
        if (Degen1 && !Degen2)
        {
            status = ChFi2d_FirstEdgeDegenerated;
        }
        if (!Degen1 && Degen2)
        {
            status = ChFi2d_LastEdgeDegenerated;
        }
        //   TrimE1 = BuildNewEdge(E1, V, newExtr1);
        //  TrimE2 = BuildNewEdge(E2, V, newExtr2);
    } // ComputeChamfer

    //=================================================================================================

    // The chamfer computed from an edge, a vertex, a distance and an angle. TrimE1, TrimE2 and Chamfer have sense only
    // if the status is IsDone.
    private void ComputeChamfer(in TopoDS_Vertex V,
                                in TopoDS_Edge E1,
                                double D,
                                double Ang,
                                in TopoDS_Edge E2,
                                ref TopoDS_Edge TrimE1,
                                ref TopoDS_Edge TrimE2,
                                ref TopoDS_Edge Chamfer)
    {
        TopoDS_Vertex newExtr1 = new(), newExtr2 = new();
        bool Degen1, Degen2;
        Chamfer = BuildChamferEdge(V, E1, D, Ang, E2, ref newExtr1, ref newExtr2);
        if (status != ChFi2d_IsDone)
        {
            return;
        }
        TrimE1 = BuildNewEdge(E1, V, newExtr1, out Degen1);
        TrimE2 = BuildNewEdge(E2, V, newExtr2, out Degen2);
        if (Degen1 && Degen2)
        {
            status = ChFi2d_BothEdgesDegenerated;
        }
        if (Degen1 && !Degen2)
        {
            status = ChFi2d_FirstEdgeDegenerated;
        }
        if (!Degen1 && Degen2)
        {
            status = ChFi2d_LastEdgeDegenerated;
        }
        //   TrimE1 = BuildNewEdge(E1, V, newExtr1);
        //   TrimE2 = BuildNewEdge(E2, V, newExtr2);
    } // ComputeChamfer

    //=================================================================================================

    /// <summary>
    /// Modifies the chamfer <paramref name="Chamfer"/> built between the edges <paramref name="E1"/> and
    /// <paramref name="E2"/> (only <paramref name="E2"/> is used, as in OCCT), and returns the new chamfer (frozen).
    /// </summary>
    public TopoDS_Edge ModifyChamfer(in TopoDS_Edge Chamfer, in TopoDS_Edge E1, in TopoDS_Edge E2, double D1, double D2)
    {
        _ = E1; // unused in OCCT
        TopoDS_Vertex aVertex = InternalRemoveChamfer(Chamfer);
        TopoDS_Edge adjEdge1 = new(), adjEdge2 = new();
        status = ChFi2d.FindConnectedEdges(newFace, aVertex, ref adjEdge1, ref adjEdge2);
        TopoDS_Edge aChamfer = new();
        if (status == ChFi2d_ConnexionError)
        {
            return aChamfer;
        }

        // adjEdge1 and adjEdge2 are copies of E1 and E2 with the good orientation
        // on <newFace>
        if (adjEdge1.IsSame(E2))
        {
            TopAbs_Orientation orient = adjEdge1.Orientation();
            adjEdge1 = adjEdge2;
            adjEdge2 = E2;
            adjEdge2.Orientation(orient);
        }

        aChamfer = InternalAddChamfer(adjEdge1, adjEdge2, D1, D2);
        return Published(aChamfer);
    } // ModifyChamfer

    //=================================================================================================

    /// <summary>
    /// Modifies the chamfer <paramref name="Chamfer"/>: the new one makes the angle <paramref name="Ang"/> with the edge
    /// <paramref name="E"/>, one of its ends on <paramref name="E"/> at the distance <paramref name="D"/> from the vertex
    /// it is built on. Returns the new chamfer (frozen).
    /// </summary>
    public TopoDS_Edge ModifyChamfer(in TopoDS_Edge Chamfer, in TopoDS_Edge E, double D, double Ang)
    {
        TopoDS_Vertex aVertex = InternalRemoveChamfer(Chamfer);
        TopoDS_Edge adjEdge1 = new(), adjEdge2 = new();
        status = ChFi2d.FindConnectedEdges(newFace, aVertex, ref adjEdge1, ref adjEdge2);
        TopoDS_Edge aChamfer = new();
        if (status == ChFi2d_ConnexionError)
        {
            return aChamfer;
        }

        if (adjEdge1.IsSame(E))
        {
            aChamfer = InternalAddChamfer(adjEdge1, aVertex, D, Ang);
        }
        else
        {
            aChamfer = InternalAddChamfer(adjEdge2, aVertex, D, Ang);
        }
        return Published(aChamfer);
    } // ModifyChamfer

    //=================================================================================================

    /// <summary>
    /// Removes the chamfer <paramref name="Chamfer"/> and returns the vertex connecting the two edges adjacent to it
    /// (frozen; null if <paramref name="Chamfer"/> is not a chamfer of this builder).
    /// </summary>
    public TopoDS_Vertex RemoveChamfer(in TopoDS_Edge Chamfer)
    {
        TopoDS_Vertex aVertex = InternalRemoveChamfer(Chamfer);
        aVertex.TShape()?.FreezeGraph();
        return aVertex;
    }

    private TopoDS_Vertex InternalRemoveChamfer(in TopoDS_Edge Chamfer)
    {
        TopoDS_Vertex commonVertex = new();

        int i = 1;
        bool IsFind = false;
        while (i <= chamfers.Length())
        {
            TopoDS_Edge aChamfer = TopoDS.Edge(chamfers.Value(i));
            if (aChamfer.IsSame(Chamfer))
            {
                chamfers.Remove(i);
                IsFind = true;
                break;
            }
            i++;
        }
        if (!IsFind)
        {
            return commonVertex;
        }

        TopExp.Vertices(Chamfer, out TopoDS_Vertex firstVertex, out TopoDS_Vertex lastVertex);

        TopoDS_Edge adjEdge1 = new(), adjEdge2 = new();
        status = ChFi2d.FindConnectedEdges(newFace, firstVertex, ref adjEdge1, ref adjEdge2);
        if (status == ChFi2d_ConnexionError)
        {
            return commonVertex;
        }

        TopoDS_Edge basisEdge1, basisEdge2, E1, E2;
        // E1 and E2 are the adjacentes edges to Chamfer

        if (adjEdge1.IsSame(Chamfer))
        {
            E1 = adjEdge2;
        }
        else
        {
            E1 = adjEdge1;
        }
        basisEdge1 = BasisEdge(E1);
        status = ChFi2d.FindConnectedEdges(newFace, lastVertex, ref adjEdge1, ref adjEdge2);
        if (status == ChFi2d_ConnexionError)
        {
            return commonVertex;
        }
        if (adjEdge1.IsSame(Chamfer))
        {
            E2 = adjEdge2;
        }
        else
        {
            E2 = adjEdge1;
        }
        basisEdge2 = BasisEdge(E2);
        TopoDS_Vertex connectionE1Chamfer = new(), connectionE2Chamfer = new();
        bool hasConnection = ChFi2d.CommonVertex(basisEdge1, basisEdge2, ref commonVertex);
        if (!hasConnection)
        {
            status = ChFi2d_ConnexionError;
            return commonVertex;
        }
        hasConnection = ChFi2d.CommonVertex(E1, Chamfer, ref connectionE1Chamfer);
        if (!hasConnection)
        {
            status = ChFi2d_ConnexionError;
            return commonVertex;
        }
        hasConnection = ChFi2d.CommonVertex(E2, Chamfer, ref connectionE2Chamfer);
        if (!hasConnection)
        {
            status = ChFi2d_ConnexionError;
            return commonVertex;
        }

        // rebuild edges on wire
        TopoDS_Edge newEdge1 = new(), newEdge2 = new();
        TopoDS_Vertex v, v1, v2;
        var makeEdge = new BRepLib_MakeEdge();
        TopLoc_Location loc;
        double first, last;

        TopExp.Vertices(E1, out firstVertex, out lastVertex);
        TopExp.Vertices(basisEdge1, out v1, out v2);
        if (v1.IsSame(commonVertex))
        {
            v = v2;
        }
        else
        {
            v = v1;
        }

        if (firstVertex.IsSame(v) || lastVertex.IsSame(v))
        {
            // It means the edge support only one fillet. In this case
            // the new edge must be the basis edge.
            newEdge1 = basisEdge1;
        }
        else
        {
            // It means the edge support one fillet on each end.
            if (firstVertex.IsSame(connectionE1Chamfer))
            {
                //  syntaxe invalide sur NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(E1, loc, first, last);
                Geom_Curve? curve = BRep_Tool.Curve(E1, out loc, out first, out last);
                makeEdge.Init(curve!, commonVertex, lastVertex);
                newEdge1 = makeEdge.InternalEdge();
                newEdge1.Orientation(basisEdge1.Orientation());
                newEdge1.Location(basisEdge1.Location());
            } // if (firstVertex ...
            else if (lastVertex.IsSame(connectionE1Chamfer))
            {
                //  syntaxe invalide sur NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(E1, loc, first, last);
                Geom_Curve? curve = BRep_Tool.Curve(E1, out loc, out first, out last);
                makeEdge.Init(curve!, firstVertex, commonVertex);
                newEdge1 = makeEdge.InternalEdge();
                newEdge1.Orientation(basisEdge1.Orientation());
                newEdge1.Location(basisEdge1.Location());
            } // else if (lastVertex ...
        } // else ...

        TopExp.Vertices(basisEdge2, out v1, out v2);
        if (v1.IsSame(commonVertex))
        {
            v = v2;
        }
        else
        {
            v = v1;
        }

        TopExp.Vertices(E2, out firstVertex, out lastVertex);
        if (firstVertex.IsSame(v) || lastVertex.IsSame(v))
        {
            // It means the edge support only one fillet. In this case
            // the new edge must be the basis edge.
            newEdge2 = basisEdge2;
        }
        else
        {
            // It means the edge support one fillet on each end.
            if (firstVertex.IsSame(connectionE2Chamfer))
            {
                //  syntaxe invalide sur NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(E2, loc, first, last);
                Geom_Curve? curve = BRep_Tool.Curve(E2, out loc, out first, out last);
                makeEdge.Init(curve!, commonVertex, lastVertex);
                newEdge2 = makeEdge.InternalEdge();
                newEdge2.Orientation(basisEdge2.Orientation());
                newEdge2.Location(basisEdge2.Location());
            } // if (firstVertex ...
            else if (lastVertex.IsSame(connectionE2Chamfer))
            {
                //  syntaxe invalide sur NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(E2, loc, first, last);
                Geom_Curve? curve = BRep_Tool.Curve(E2, out loc, out first, out last);
                makeEdge.Init(curve!, firstVertex, commonVertex);
                newEdge2 = makeEdge.InternalEdge();
                newEdge2.Orientation(basisEdge2.Orientation());
                newEdge2.Location(basisEdge2.Location());
            } // else if (lastVertex ...
        } // else ...

        // rebuild the newFace
        var Ex = new TopExp_Explorer(newFace, TopAbs_ShapeEnum.TopAbs_EDGE);
        TopoDS_Wire newWire = new();

        var B = new BRep_Builder();
        B.MakeWire(ref newWire);

        while (Ex.More())
        {
            TopoDS_Edge theEdge = TopoDS.Edge(Ex.Current());
            if (!theEdge.IsSame(E1) && !theEdge.IsSame(E2) && !theEdge.IsSame(Chamfer))
            {
                B.Add(newWire, theEdge);
            }
            else
            {
                if (theEdge.IsEqual(E1))
                {
                    B.Add(newWire, newEdge1);
                }
                else if (theEdge.IsEqual(E2))
                {
                    B.Add(newWire, newEdge2);
                }
            } // else
            Ex.Next();
        } // while ...

        var Adaptor3dSurface = new BRepAdaptor_Surface(refFace);
        var mFace = new BRepLib_MakeFace(Adaptor3dSurface.Plane(), newWire);
        newFace.Nullify();
        newFace = mFace.InternalFace();

        UpDateHistory(basisEdge1, basisEdge2, newEdge1, newEdge2);

        return commonVertex;
    } // RemoveChamfer

    //=================================================================================================

    // The chamfer between AdjEdge1 and AdjEdge2 at V, at the distances D1 and D2; NewExtr1 and NewExtr2 receive the new
    // extremities of the two edges. The edge has sense only if the status is IsDone.
    private TopoDS_Edge BuildChamferEdge(in TopoDS_Vertex V,
                                         in TopoDS_Edge AdjEdge1,
                                         in TopoDS_Edge AdjEdge2,
                                         double D1,
                                         double D2,
                                         ref TopoDS_Vertex NewExtr1,
                                         ref TopoDS_Vertex NewExtr2)
    {
        TopoDS_Edge chamfer = new();
        if (D1 <= 0 || D2 <= 0)
        {
            status = ChFi2d_ParametersError;
            return chamfer;
        } // if ( D1 <=0 ...

        double param1, param2;
        gp_Pnt p1 = ComputePoint(V, AdjEdge1, D1, out param1);
        gp_Pnt p2 = ComputePoint(V, AdjEdge2, D2, out param2);

        double tol = Precision.Confusion();
        var B = new BRep_Builder();
        B.MakeVertex(ref NewExtr1, p1, tol);
        B.MakeVertex(ref NewExtr2, p2, tol);
        NewExtr1.Orientation(TopAbs_FORWARD);
        NewExtr2.Orientation(TopAbs_REVERSED);

        // chamfer edge construction
        Geom_Surface? refSurf = BRep_Tool.Surface(refFace, out TopLoc_Location loc);
        _ = refSurf; // unused, as in OCCT
        _ = loc;
        var myVec = new gp_Vec(p1, p2);
        var myDir = new gp_Dir(myVec);
        var newLine = new Geom_Line(p1, myDir);
        double param = ElCLib.Parameter(newLine.Lin(), p2);
        B.MakeEdge(ref chamfer, newLine, tol);
        B.Range(chamfer, 0.0, param);
        B.Add(chamfer, NewExtr1);
        B.UpdateVertex(NewExtr1, 0.0, chamfer, tol);
        B.Add(chamfer, NewExtr2);
        B.UpdateVertex(NewExtr2, param, chamfer, tol);
        OrientChamfer(ref chamfer, AdjEdge1, V);

        // set the orientation of NewExtr1 and NewExtr2 for the adjacent edges
        TopoDS_Vertex V1 = TopExp.FirstVertex(AdjEdge1);
        TopoDS_Vertex V2 = TopExp.LastVertex(AdjEdge1);
        if (V1.IsSame(V))
        {
            NewExtr1.Orientation(V1.Orientation());
        }
        else
        {
            NewExtr1.Orientation(V2.Orientation());
        }

        V1 = TopExp.FirstVertex(AdjEdge2);
        V2 = TopExp.LastVertex(AdjEdge2);
        if (V1.IsSame(V))
        {
            NewExtr2.Orientation(V1.Orientation());
        }
        else
        {
            NewExtr2.Orientation(V2.Orientation());
        }
        UpdateNewVertex(B, NewExtr1, param1, AdjEdge1, tol);
        UpdateNewVertex(B, NewExtr2, param2, AdjEdge2, tol);

        status = ChFi2d_IsDone;
        return chamfer;
    } // BuildChamferEdge

    //=================================================================================================

    // The chamfer at V making the angle Ang with AdjEdge1, one end on AdjEdge1 at the distance D; NewExtr1 and NewExtr2
    // receive the new extremities of the two edges. The edge has sense only if the status is IsDone.
    private TopoDS_Edge BuildChamferEdge(in TopoDS_Vertex V,
                                         in TopoDS_Edge AdjEdge1,
                                         double D,
                                         double Ang,
                                         in TopoDS_Edge AdjEdge2,
                                         ref TopoDS_Vertex NewExtr1,
                                         ref TopoDS_Vertex NewExtr2)
    {
        TopoDS_Edge chamfer = new();
        if (D <= 0 || Ang <= 0)
        {
            status = ChFi2d_ParametersError;
            return chamfer;
        } // if ( D <= 0 ...

        double param1, param2;
        gp_Pnt p1 = ComputePoint(V, AdjEdge1, D, out param1);
        gp_Pnt p = BRep_Tool.Pnt(V);
        var myVec = new gp_Vec(p1, p);

        // compute the tangent vector on AdjEdge2 at the vertex V.
        var c = new BRepAdaptor_Curve(AdjEdge2, refFace);
        double first, last;
        first = c.FirstParameter();
        last = c.LastParameter();

        gp_Pnt aPoint;
        gp_Vec tan;
        c.D1(first, out aPoint, out tan);
        if (aPoint.Distance(p) > Precision.Confusion())
        {
            c.D1(last, out aPoint, out tan);
        }
        // tangent orientation
        TopExp.Vertices(AdjEdge2, out TopoDS_Vertex v1, out TopoDS_Vertex v2);
        TopAbs_Orientation orient;
        if (v1.IsSame(V))
        {
            orient = v1.Orientation();
        }
        else
        {
            orient = v2.Orientation();
        }
        if (orient == TopAbs_REVERSED)
        {
            tan *= -1;
        }

        // compute the chamfer geometric support
        var RotAxe = new gp_Ax1(p1, new gp_Dir(tan ^ myVec));
        gp_Vec vecLin = myVec.Rotated(RotAxe, -Ang);
        var myDir = new gp_Dir(vecLin);
        var newLine = new Geom_Line(p1, myDir);
        var B1 = new BRep_Builder();
        B1.MakeEdge(ref chamfer, newLine, Precision.Confusion());
        gp_Pnt p2 = ComputePoint(refFace, newLine, AdjEdge2, out param2);

        double tol = Precision.Confusion();
        var B = new BRep_Builder();
        B.MakeVertex(ref NewExtr1, p1, tol);
        B.MakeVertex(ref NewExtr2, p2, tol);
        NewExtr1.Orientation(TopAbs_FORWARD);
        NewExtr2.Orientation(TopAbs_REVERSED);

        // chamfer edge construction
        double param = ElCLib.Parameter(newLine.Lin(), p2);
        B.MakeEdge(ref chamfer, newLine, tol);
        B.Range(chamfer, 0.0, param);
        B.Add(chamfer, NewExtr1);
        B.UpdateVertex(NewExtr1, 0.0, chamfer, tol);
        B.Add(chamfer, NewExtr2);
        B.UpdateVertex(NewExtr2, param, chamfer, tol);
        OrientChamfer(ref chamfer, AdjEdge1, V);

        // set the orientation of NewExtr1 and NewExtr2 for the adjacent edges
        TopoDS_Vertex V1 = TopExp.FirstVertex(AdjEdge1);
        TopoDS_Vertex V2 = TopExp.LastVertex(AdjEdge1);
        if (V1.IsSame(V))
        {
            NewExtr1.Orientation(V1.Orientation());
        }
        else
        {
            NewExtr1.Orientation(V2.Orientation());
        }

        V1 = TopExp.FirstVertex(AdjEdge2);
        V2 = TopExp.LastVertex(AdjEdge2);
        if (V1.IsSame(V))
        {
            NewExtr2.Orientation(V1.Orientation());
        }
        else
        {
            NewExtr2.Orientation(V2.Orientation());
        }
        UpdateNewVertex(B, NewExtr1, param1, AdjEdge1, tol);
        UpdateNewVertex(B, NewExtr2, param2, AdjEdge2, tol);

        status = ChFi2d_IsDone;
        return chamfer;
    }

    // OCCT.NET: frozen (decision 5933). OCCT's B.UpdateVertex(NewExtr, param, AdjEdge, tol) gives the new vertex its
    // points on the adjacent edge's curves (the vertex isn't on the edge, so nothing else of the edge is read), raises
    // its tolerance, and sets the edge's Modified flag. When the adjacent edge is frozen (the caller's, or one the
    // builder published), the same call goes through an EmptyCopied stand-in, which has the edge's representations and
    // location: the vertex gets OCCT's points, and the edge's flag is left as it is.
    private static void UpdateNewVertex(BRep_Builder B, in TopoDS_Vertex V, double Par, in TopoDS_Edge E, double Tol)
    {
        if (E.TShape()!.IsFrozen() && !BRep_Tool.Degenerated(E))
        {
            B.UpdateVertex(V, Par, TopoDS.Edge(E.EmptyCopied()), Tol);
            return;
        }
        B.UpdateVertex(V, Par, E, Tol);
    }

    //=================================================================================================

    // The point at the distance D from V along E (its parameter in Param).
    private static gp_Pnt ComputePoint(in TopoDS_Vertex V, in TopoDS_Edge E, double D, out double Param)
    {
        // geometric support
        var c = new BRepAdaptor_Curve(E);
        double first, last;
        first = c.FirstParameter();
        last = c.LastParameter();

        gp_Pnt thePoint = new();
        if (c.GetType() == GeomAbs_Line)
        {
            gp_Pnt p1, p2;
            TopExp.Vertices(E, out TopoDS_Vertex v1, out TopoDS_Vertex v2);
            p1 = BRep_Tool.Pnt(v1);
            p2 = BRep_Tool.Pnt(v2);
            var myVec = new gp_Vec(p1, p2);
            myVec.Normalize();
            myVec *= D;
            if (v2.IsSame(V))
            {
                myVec *= -1; // change the sense of myVec
                thePoint = p2.Translated(myVec);
            } // if (v2 ...
            else
            {
                thePoint = p1.Translated(myVec);
            }

            Param = ElCLib.Parameter(c.Line(), thePoint);
            // szv:OCC20823-begin
            c.D0(Param, out thePoint);
            // szv:OCC20823-end
            return thePoint;
        } // if (C->IsKind(TYPE ...

        if (c.GetType() == GeomAbs_Circle)
        {
            gp_Circ cir = c.Circle();
            double radius = cir.Radius();
            TopExp.Vertices(E, out TopoDS_Vertex v1, out TopoDS_Vertex v2);
            double param1, param2;
            if (V.IsSame(v1))
            {
                param1 = BRep_Tool.Parameter(v1, E);
                param2 = BRep_Tool.Parameter(v2, E);
            }
            else
            {
                param1 = BRep_Tool.Parameter(v2, E);
                param2 = BRep_Tool.Parameter(v1, E);
            }
            double deltaAlpha = D / radius;
            if (param1 > param2)
            {
                Param = param1 - deltaAlpha;
            }
            else
            {
                Param = param1 + deltaAlpha;
            }
            c.D0(Param, out thePoint);
            return thePoint;
        } // if (C->IsKind(TYPE ...

        else
        {
            // in all other case than lines and circles.
            gp_Pnt p;
            TopExp.Vertices(E, out TopoDS_Vertex v1, out TopoDS_Vertex v2);
            if (V.IsSame(v1))
            {
                p = BRep_Tool.Pnt(v1);
            }
            else
            {
                p = BRep_Tool.Pnt(v2);
            }

            GeomAdaptor_Curve cc = c.Curve();
            if (p.Distance(c.Value(first)) <= Precision.Confusion())
            {
                var computePoint = new GCPnts_AbscissaPoint(cc, D, first);
                Param = computePoint.Parameter();
            }
            else
            {
                var computePoint = new GCPnts_AbscissaPoint(cc, D, last);
                Param = computePoint.Parameter();
            }
            thePoint = cc.Value(Param);
            return thePoint;
        } // else ...
    } // ComputePoint

    //=================================================================================================

    // The first intersection (along L) of the line L with the edge E on the face F; Param is its parameter on E.
    private static gp_Pnt ComputePoint(in TopoDS_Face F, Geom_Line L, in TopoDS_Edge E, out double Param)
    {
        var Adaptor3dSurface = new BRepAdaptor_Surface(F);
        var refSurf = new Geom_Plane(Adaptor3dSurface.Plane());
        Geom2d_Curve lin2d = GeomAPI.To2d(L, refSurf.Pln())!;
        Geom2d_Curve? c2d;
        double first, last;
        c2d = BRep_Tool.CurveOnSurface(E, F, out first, out last);
        var adaptorL = new Geom2dAdaptor_Curve(lin2d);
        var adaptorC = new Geom2dAdaptor_Curve(c2d!);
        var Intersection = new Geom2dInt_GInter(adaptorL,
                                                adaptorC,
                                                Precision.PIntersection(),
                                                Precision.PIntersection());
        double paramOnLine = 1E300;
        gp_Pnt2d p2d = new();
        Param = 0.0; // OCCT leaves it unset when nothing is found (the caller then reads an uninitialised value)
        if (Intersection.IsDone())
        {
            int i = 1;
            while (i <= Intersection.NbPoints())
            {
                IntRes2d_IntersectionPoint iP = Intersection.Point(i);
                if (iP.ParamOnFirst() < paramOnLine)
                {
                    p2d = iP.Value();
                    paramOnLine = iP.ParamOnFirst();
                    Param = iP.ParamOnSecond();
                } // if (iP.ParamOnFirst ...
                i++;
            } // while ( i <= ...
        } // if (Intersection.IsDone ...

        gp_Pnt thePoint = Adaptor3dSurface.Value(p2d.X(), p2d.Y());
        return thePoint;
    } // ComputePoint

    //=================================================================================================

    private static void OrientChamfer(ref TopoDS_Edge chamfer, in TopoDS_Edge E, in TopoDS_Vertex V)
    {
        TopAbs_Orientation vOrient, orient = E.Orientation();
        TopExp.Vertices(E, out TopoDS_Vertex v1, out TopoDS_Vertex v2);
        if (v1.IsSame(V))
        {
            vOrient = v2.Orientation();
        }
        else
        {
            vOrient = v1.Orientation();
        }

        if ((orient == TopAbs_FORWARD && vOrient == TopAbs_FORWARD)
            || (orient == TopAbs_REVERSED && vOrient == TopAbs_REVERSED))
        {
            chamfer.Orientation(TopAbs_FORWARD);
        }
        else
        {
            chamfer.Orientation(TopAbs_REVERSED);
        }
    } // OrientChamfer
}
