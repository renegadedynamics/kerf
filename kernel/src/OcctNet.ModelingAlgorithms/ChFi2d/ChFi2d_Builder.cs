// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_Builder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_Builder.lxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Schematiq.Collections;
using static Occt.ChFi2d_ConstructionError;
using static Occt.GccEnt_Position;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>Builds fillets and chamfers on the wire of a planar face.</summary>
/// <remarks>
/// <para>A working object (decision 21), as OCCT's. The face it is given is never written: each fillet or chamfer
/// builds a new face holding the face's untouched edges. What it hands out (<see cref="Result"/>, the fillets and
/// chamfers, the descendant edges, the vertices <see cref="RemoveFillet"/> returns) is frozen (decision 853); later
/// additions build new faces around them.</para>
/// <para>The history maps the face's edges to their trimmed descendants. Its iteration order isn't OCCT's (a hash order
/// there too); <see cref="BasisEdge"/> finds the one key whose value is the edge, so the order doesn't change results.</para>
/// </remarks>
public sealed partial class ChFi2d_Builder
{
    private ChFi2d_ConstructionError status;
    private TopoDS_Face refFace = new();
    private TopoDS_Face newFace = new();
    private readonly NCollection_Sequence<TopoDS_Shape> fillets = new();
    private readonly NCollection_Sequence<TopoDS_Shape> chamfers = new();
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> history = new(TopTools_ShapeMapHasher.Instance);

    //=================================================================================================

    public ChFi2d_Builder()
    {
        status = ChFi2d_NotPlanar;
    }

    //=================================================================================================

    /// <summary>The face <paramref name="F"/> can be built on a closed or an open wire.</summary>
    /// <remarks>
    /// OCCT builds the 3D curves of the face's edges that have none, in place (BRepLib::BuildCurves3d). On a frozen face
    /// with such an edge the port builds them on a copy (<see cref="BRepLib.Curves3dBuilt(in TopoDS_Shape, out bool)"/>),
    /// which then stands for the face (decision 854).
    /// </remarks>
    public ChFi2d_Builder(in TopoDS_Face F)
    {
        if (F.IsNull())
        {
            status = ChFi2d_NoFace;
            return;
        }
        //  syntaxe invalide sur NT
        //  const occ::handle<Geom_Surface>&  surf = BRep_Tool::Surface( F, Loc);
        //  if (surf->IsKind(STANDARD_TYPE(Geom_Plane))) {
        if (BRep_Tool.Surface(F, out _)!.IsKind(typeof(Geom_Plane)))
        {
            newFace = refFace = F;
            newFace.Orientation(TopAbs_FORWARD);
            if (BRepTools.HoldsFrozen(newFace) && HasEdgeWithout3dCurve(newFace)) // OCCT.NET: frozen
            {
                refFace = TopoDS.Face(BRepLib.Curves3dBuilt(F, out _));
                newFace = refFace;
                newFace.Orientation(TopAbs_FORWARD);
            }
            else
            {
                BRepLib.BuildCurves3d(newFace);
            }
            status = ChFi2d_Ready;
        }
        else
        {
            status = ChFi2d_NotPlanar;
        }
    } // ChFi2d_Builder

    // OCCT.NET: frozen. True if an edge of the face has no 3D curve (BuildCurves3d would write into it).
    private static bool HasEdgeWithout3dCurve(in TopoDS_Face F)
    {
        for (var anExp = new TopExp_Explorer(F, TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            if (BRep_Tool.Curve(TopoDS.Edge(anExp.Current()), out _, out _, out _) is null)
            {
                return true;
            }
        }
        return false;
    }

    //=================================================================================================

    public void Init(in TopoDS_Face F)
    {
        if (F.IsNull())
        {
            status = ChFi2d_NoFace;
            return;
        }
        fillets.Clear();
        chamfers.Clear();
        history.Clear();
        //  syntaxe invalide sur NT
        //  const occ::handle<Geom_Surface>&  surf = BRep_Tool::Surface( F, Loc);
        //  if (surf->IsKind(STANDARD_TYPE(Geom_Plane))) {
        if (BRep_Tool.Surface(F, out _)!.IsKind(typeof(Geom_Plane)))
        {
            newFace = refFace = F;
            newFace.Orientation(TopAbs_FORWARD);
            status = ChFi2d_Ready;
        }
        else
        {
            status = ChFi2d_NotPlanar;
        }
    } // Init

    //=================================================================================================

    /// <summary>
    /// Initialises the builder with a face <paramref name="RefFace"/> and a face <paramref name="ModFace"/> derived from
    /// it, to modify the fillets and chamfers already built on <paramref name="ModFace"/>.
    /// </summary>
    public void Init(in TopoDS_Face RefFace, in TopoDS_Face ModFace)
    {
        if (RefFace.IsNull() || ModFace.IsNull())
        {
            status = ChFi2d_NoFace;
            return;
        }
        fillets.Clear();
        chamfers.Clear();
        history.Clear();
        TopLoc_Location loc;
        //  syntaxe invalide sur NT
        //  const occ::handle<Geom_Surface>&  surf = BRep_Tool::Surface( RefFace, Loc);
        //  if (!surf->IsKind(STANDARD_TYPE(Geom_Plane))) {
        if (!BRep_Tool.Surface(RefFace, out loc)!.IsKind(typeof(Geom_Plane)))
        {
            status = ChFi2d_NotPlanar;
            return;
        }

        refFace = RefFace;
        newFace = ModFace;
        newFace.Orientation(TopAbs_FORWARD);
        status = ChFi2d_Ready;

        // Research in newFace all the edges which not appear in RefFace
        // The sequence newEdges will contains this edges.

        NCollection_Sequence<TopoDS_Shape> newEdges = new();
        NCollection_IndexedMap<TopoDS_Shape> refEdgesMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(refFace, TopAbs_ShapeEnum.TopAbs_EDGE, refEdgesMap);
        var ex = new TopExp_Explorer(newFace, TopAbs_ShapeEnum.TopAbs_EDGE);
        while (ex.More())
        {
            TopoDS_Edge currentEdge = TopoDS.Edge(ex.Current());
            if (!refEdgesMap.Contains(currentEdge))
            {
                newEdges.Append(currentEdge);
            }
            ex.Next();
        } // while (ex ...

        // update of history, fillets and chamfers fields
        int i = 1;
        double first, last;
        TopoDS_Edge basisEdge = new();
        while (i <= newEdges.Length())
        {
            TopoDS_Edge currentEdge = TopoDS.Edge(newEdges.Value(i));
            if (IsIssuedFrom(currentEdge, refEdgesMap, ref basisEdge))
            {
                history.Bind(basisEdge, currentEdge);
            }
            else
            {
                // this edge is a chamfer or a fillet
                //  syntaxe invalide sur NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(currentEdge, loc, first, last);
                Geom_Curve? curve = BRep_Tool.Curve(currentEdge, out loc, out first, out last);
                if (curve!.IsKind(typeof(Geom_Circle)))
                {
                    fillets.Append(currentEdge);
                }
                else if (curve.IsKind(typeof(Geom_Line)))
                {
                    chamfers.Append(currentEdge);
                }
                else
                {
                    status = ChFi2d_InitialisationError;
                    return;
                } // else ...
            } // this edge is ...
            i++;
        } // while ...
    } // Init

    //=======================================================================
    // function : IsIssuedFrom
    // purpose  : Search in <Map> if <E> has a parent edge. If a parent has
    //           been find, this edge is returned in <BasisEdge>, else <E> is
    //           returned in <BasisEdge>.
    //=======================================================================
    private static bool IsIssuedFrom(in TopoDS_Edge E, NCollection_IndexedMap<TopoDS_Shape> Map, ref TopoDS_Edge BasisEdge)
    {
        double f1, L1, f2, L2;
        //  syntaxe invalide sur NT
        //  const occ::handle<Geom_Curve>& c1 =
        //    BRep_Tool::Curve(E, loc1, f1, L1);
        Geom_Curve? c1 = BRep_Tool.Curve(E, out _, out f1, out L1);

        for (int i = 1; i <= Map.Extent(); i++)
        {
            TopoDS_Edge currentEdge = TopoDS.Edge(Map.FindKey(i));
            //  syntaxe invalide sur NT
            //    const occ::handle<Geom_Curve>& c2 =
            //      BRep_Tool::Curve(currentEdge, loc2, f2, L2);
            Geom_Curve? c2 = BRep_Tool.Curve(currentEdge, out _, out f2, out L2);
            if (c1 == c2
                && (((f1 > f2 && f1 < L2) || (L1 > f2 && L1 < L2))
                    || ((f1 > L2 && f1 < f2) || (L1 > L2 && L1 < f2))))
            {
                BasisEdge = currentEdge;
                BasisEdge.Orientation(E.Orientation());
                return true;
            } //  if (c1 == c2
        } // for (int i ...

        return false;
    } // IsIssuedFrom

    //=================================================================================================

    /// <summary>
    /// Adds a fillet of radius <paramref name="Radius"/> between the two edges connected to the vertex
    /// <paramref name="V"/>, and returns it (frozen). The edge has sense only if <see cref="Status"/> is
    /// <see cref="ChFi2d_IsDone"/>.
    /// </summary>
    public TopoDS_Edge AddFillet(in TopoDS_Vertex V, double Radius)
    {
        TopoDS_Edge aFillet = InternalAddFillet(V, Radius);
        return Published(aFillet);
    }

    private TopoDS_Edge InternalAddFillet(in TopoDS_Vertex V, double Radius)
    {
        TopoDS_Edge adjEdge1 = new(), adjEdge2 = new(), basisEdge1, basisEdge2;
        TopoDS_Edge adjEdge1Mod = new(), adjEdge2Mod = new(), fillet = new();
        status = ChFi2d.FindConnectedEdges(newFace, V, ref adjEdge1, ref adjEdge2);
        if (status == ChFi2d_ConnexionError)
        {
            return fillet;
        }

        if (IsAFillet(adjEdge1) || IsAChamfer(adjEdge1) || IsAFillet(adjEdge2) || IsAChamfer(adjEdge2))
        {
            status = ChFi2d_NotAuthorized;
            return fillet;
        } //  if (IsAFillet ...

        if (!IsLineOrCircle(adjEdge1, newFace) || !IsLineOrCircle(adjEdge2, newFace))
        {
            status = ChFi2d_NotAuthorized;
            return fillet;
        } //  if (!IsLineOrCircle ...

        ComputeFillet(V, adjEdge1, adjEdge2, Radius, ref adjEdge1Mod, ref adjEdge2Mod, ref fillet);
        if (status == ChFi2d_IsDone || status == ChFi2d_FirstEdgeDegenerated
            || status == ChFi2d_LastEdgeDegenerated || status == ChFi2d_BothEdgesDegenerated)
        {
            BuildNewWire(adjEdge1, adjEdge2, adjEdge1Mod, fillet, adjEdge2Mod);
            basisEdge1 = BasisEdge(adjEdge1);
            basisEdge2 = BasisEdge(adjEdge2);
            UpDateHistory(basisEdge1, basisEdge2, adjEdge1Mod, adjEdge2Mod, fillet, 1);
            status = ChFi2d_IsDone;
            return TopoDS.Edge(fillets.Value(fillets.Length()));
        }
        return fillet;
    } // AddFillet

    //=================================================================================================

    /// <summary>
    /// Modifies the radius of the fillet <paramref name="Fillet"/> and returns the new fillet (frozen). The edge has
    /// sense only if <see cref="Status"/> is <see cref="ChFi2d_IsDone"/>.
    /// </summary>
    public TopoDS_Edge ModifyFillet(in TopoDS_Edge Fillet, double Radius)
    {
        TopoDS_Vertex aVertex = InternalRemoveFillet(Fillet);
        TopoDS_Edge aFillet = InternalAddFillet(aVertex, Radius);
        return Published(aFillet);
    } // ModifyFillet

    //=================================================================================================

    /// <summary>
    /// Removes the fillet <paramref name="Fillet"/> and returns the vertex connecting the two edges adjacent to it
    /// (frozen; null if <paramref name="Fillet"/> is not a fillet of this builder).
    /// </summary>
    public TopoDS_Vertex RemoveFillet(in TopoDS_Edge Fillet)
    {
        TopoDS_Vertex aVertex = InternalRemoveFillet(Fillet);
        aVertex.TShape()?.FreezeGraph();
        return aVertex;
    }

    private TopoDS_Vertex InternalRemoveFillet(in TopoDS_Edge Fillet)
    {
        TopoDS_Vertex commonVertex = new();
        int i = 1;
        bool IsFind = false;
        while (i <= fillets.Length())
        {
            TopoDS_Edge aFillet = TopoDS.Edge(fillets.Value(i));
            if (aFillet.IsSame(Fillet))
            {
                fillets.Remove(i);
                IsFind = true;
                break;
            }
            i++;
        }
        if (!IsFind)
        {
            return commonVertex;
        }

        TopExp.Vertices(Fillet, out TopoDS_Vertex firstVertex, out TopoDS_Vertex lastVertex);

        TopoDS_Edge adjEdge1 = new(), adjEdge2 = new();
        status = ChFi2d.FindConnectedEdges(newFace, firstVertex, ref adjEdge1, ref adjEdge2);
        if (status == ChFi2d_ConnexionError)
        {
            return commonVertex;
        }

        TopoDS_Edge basisEdge1, basisEdge2, E1, E2;
        // E1 and E2 are the adjacent edges to Fillet

        if (adjEdge1.IsSame(Fillet))
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
        if (adjEdge1.IsSame(Fillet))
        {
            E2 = adjEdge2;
        }
        else
        {
            E2 = adjEdge1;
        }
        basisEdge2 = BasisEdge(E2);
        TopoDS_Vertex connectionE1Fillet = new(), connectionE2Fillet = new();
        bool hasConnection = ChFi2d.CommonVertex(basisEdge1, basisEdge2, ref commonVertex);
        if (!hasConnection)
        {
            status = ChFi2d_ConnexionError;
            return commonVertex;
        }
        hasConnection = ChFi2d.CommonVertex(E1, Fillet, ref connectionE1Fillet);
        if (!hasConnection)
        {
            status = ChFi2d_ConnexionError;
            return commonVertex;
        }
        hasConnection = ChFi2d.CommonVertex(E2, Fillet, ref connectionE2Fillet);
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
            if (firstVertex.IsSame(connectionE1Fillet))
            {
                //  syntaxe invalide sur NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(E1, loc, first, last);
                Geom_Curve? curve = BRep_Tool.Curve(E1, out loc, out first, out last);
                makeEdge.Init(curve!, commonVertex, lastVertex);
                newEdge1 = makeEdge.InternalEdge();
                newEdge1.Orientation(E1.Orientation());
                newEdge1.Location(E1.Location());
            } // if (firstVertex ...
            else if (lastVertex.IsSame(connectionE1Fillet))
            {
                //  syntax wrong on NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(E1, loc, first, last);
                Geom_Curve? curve = BRep_Tool.Curve(E1, out loc, out first, out last);
                makeEdge.Init(curve!, firstVertex, commonVertex);
                newEdge1 = makeEdge.InternalEdge();
                newEdge1.Orientation(E1.Orientation());
                newEdge1.Location(E1.Location());
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
            if (firstVertex.IsSame(connectionE2Fillet))
            {
                //  syntax wrong on NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(E2, loc, first, last);
                Geom_Curve? curve = BRep_Tool.Curve(E2, out loc, out first, out last);
                makeEdge.Init(curve!, commonVertex, lastVertex);
                newEdge2 = makeEdge.InternalEdge();
                newEdge2.Orientation(E2.Orientation());
                newEdge2.Location(E2.Location());
            } // if (firstVertex ...
            else if (lastVertex.IsSame(connectionE2Fillet))
            {
                //  syntax wrong on NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(E2, loc, first, last);
                Geom_Curve? curve = BRep_Tool.Curve(E2, out loc, out first, out last);
                makeEdge.Init(curve!, firstVertex, commonVertex);
                newEdge2 = makeEdge.InternalEdge();
                newEdge2.Orientation(E2.Orientation());
                newEdge2.Location(E2.Location());
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
            if (!theEdge.IsSame(E1) && !theEdge.IsSame(E2) && !theEdge.IsSame(Fillet))
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
    } // RemoveFillet

    //=================================================================================================

    private void ComputeFillet(in TopoDS_Vertex V,
                               in TopoDS_Edge E1,
                               in TopoDS_Edge E2,
                               double Radius,
                               ref TopoDS_Edge TrimE1,
                               ref TopoDS_Edge TrimE2,
                               ref TopoDS_Edge Fillet)
    {
        TopoDS_Vertex newExtr1 = new(), newExtr2 = new();
        bool Degen1, Degen2;
        Fillet = BuildFilletEdge(V, E1, E2, Radius, ref newExtr1, ref newExtr2);
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
    } // ComputeFillet

    //=================================================================================================

    // Replaces in newFace OldE1 and OldE2 by E1, Fillet and E2, or by Fillet and E2 if E1 is degenerated, or by E1
    // and Fillet if E2 is degenerated, or by Fillet if both are.
    private void BuildNewWire(in TopoDS_Edge OldE1,
                              in TopoDS_Edge OldE2,
                              in TopoDS_Edge E1,
                              in TopoDS_Edge Fillet,
                              in TopoDS_Edge E2)
    {
        bool aClosedStatus = true;

        // OCCT: every edge of the face goes into one wire, so a face with holes loses them as separate wires; the
        // closed flag is the first wire's
        var Ex = new TopExp_Explorer(refFace, TopAbs_ShapeEnum.TopAbs_WIRE);
        while (Ex.More())
        {
            TopoDS_Wire aWire = TopoDS.Wire(Ex.Current());
            aClosedStatus = aWire.Closed();
            break;
        }

        bool filletIsAdded = false;

        Ex.Init(newFace, TopAbs_ShapeEnum.TopAbs_EDGE);
        TopoDS_Wire newWire = new();
        var B = new BRep_Builder();
        B.MakeWire(ref newWire);

        while (Ex.More())
        {
            TopoDS_Edge theEdge = TopoDS.Edge(Ex.Current());
            if (!theEdge.IsSame(OldE1) && !theEdge.IsSame(OldE2))
            {
                B.Add(newWire, theEdge);
            }
            else
            {
                if (theEdge.IsEqual(OldE1))
                {
                    if (status != ChFi2d_FirstEdgeDegenerated && status != ChFi2d_BothEdgesDegenerated)
                    {
                        B.Add(newWire, E1);
                    }
                    if (!filletIsAdded)
                    {
                        B.Add(newWire, Fillet);
                        filletIsAdded = true;
                    } // if ( !filletIsAdded ...
                } // if (theEdge == ...
                else
                {
                    if (status != ChFi2d_LastEdgeDegenerated && status != ChFi2d_BothEdgesDegenerated)
                    {
                        B.Add(newWire, E2);
                    }
                    if (!filletIsAdded)
                    {
                        B.Add(newWire, Fillet);
                        filletIsAdded = true;
                    } // if ( !filletIsAdded ...
                } // else ...
            } // else ...
            Ex.Next();
        } // while ...

        newWire.Closed(aClosedStatus);
        var Adaptor3dSurface = new BRepAdaptor_Surface(refFace);
        var mFace = new BRepLib_MakeFace(Adaptor3dSurface.Plane(), newWire);
        newFace = mFace.InternalFace();
    } // BuildNewWire

    //=================================================================================================

    // Changes OldExtr of E1 by NewExtr.
    private TopoDS_Edge BuildNewEdge(in TopoDS_Edge E1, in TopoDS_Vertex OldExtr, in TopoDS_Vertex NewExtr)
    {
        var makeEdge = new BRepLib_MakeEdge();
        double first, last;
        TopExp.Vertices(E1, out TopoDS_Vertex firstVertex, out TopoDS_Vertex lastVertex);
        //  syntaxe invalide sur NT
        //      const occ::handle<Geom_Curve>& curve =
        //	BRep_Tool::Curve(E1, first, last);
        Geom_Curve? curve = BRep_Tool.Curve(E1, out first, out last);
        if (firstVertex.IsSame(OldExtr))
        {
            makeEdge.Init(curve!, NewExtr, lastVertex);
        }
        else
        {
            makeEdge.Init(curve!, firstVertex, NewExtr);
        }
        TopoDS_Edge anEdge = makeEdge.InternalEdge();
        anEdge.Orientation(E1.Orientation());
        //  anEdge.Location(E1.Location());
        return anEdge;
    }

    //=======================================================================
    // function : BuildNewEdge
    // purpose  : special flag if the new edge is degenerated
    //=======================================================================

    // Changes OldExtr of E1 by NewExtr; returns E1 and IsDegenerated true if the new edge is degenerated.
    private TopoDS_Edge BuildNewEdge(in TopoDS_Edge E1, in TopoDS_Vertex OldExtr, in TopoDS_Vertex NewExtr, out bool IsDegenerated)
    {
        var makeEdge = new BRepLib_MakeEdge();
        double first, last;
        IsDegenerated = false;
        TopExp.Vertices(E1, out TopoDS_Vertex firstVertex, out TopoDS_Vertex lastVertex);
        gp_Pnt Pnew = BRep_Tool.Pnt(NewExtr);
        bool PonctualEdge = false;
        double Tol = Precision.Confusion();
        //  syntax wrong on NT
        //      const occ::handle<Geom_Curve>& curve =
        //	BRep_Tool::Curve(E1, first, last);
        Geom_Curve? curve = BRep_Tool.Curve(E1, out first, out last);
        if (firstVertex.IsSame(OldExtr))
        {
            makeEdge.Init(curve!, NewExtr, lastVertex);
            gp_Pnt PV = BRep_Tool.Pnt(lastVertex);
            PonctualEdge = (Pnew.Distance(PV) < Tol);
        }
        else
        {
            makeEdge.Init(curve!, firstVertex, NewExtr);
            gp_Pnt PV = BRep_Tool.Pnt(firstVertex);
            PonctualEdge = (Pnew.Distance(PV) < Tol);
        }
        TopoDS_Edge anEdge;
        BRepLib_EdgeError error = makeEdge.Error();
        if (error == BRepLib_EdgeError.BRepLib_LineThroughIdenticPoints || PonctualEdge)
        {
            IsDegenerated = true;
            anEdge = E1;
        }
        else
        {
            anEdge = makeEdge.InternalEdge();
            anEdge.Orientation(E1.Orientation());
            //    anEdge.Location(E1.Location());
        }
        return anEdge;
    }

    //=================================================================================================

    // Writes NewEdge in fillets if Id is 1, or in chamfers if Id is 2, and the modifications in history: TrimE1 is given
    // by E1, TrimE2 by E2, if they are not degenerated.
    private void UpDateHistory(in TopoDS_Edge E1,
                               in TopoDS_Edge E2,
                               in TopoDS_Edge TrimE1,
                               in TopoDS_Edge TrimE2,
                               in TopoDS_Edge NewEdge,
                               int Id)
    {
        if (Id == 1) // the new edge is a fillet
        {
            fillets.Append(NewEdge);
        }
        else // the new edge is a chamfer
        {
            chamfers.Append(NewEdge);
        }

        history.UnBind(E1);
        if (status != ChFi2d_FirstEdgeDegenerated && status != ChFi2d_BothEdgesDegenerated)
        {
            if (!E1.IsSame(TrimE1))
            {
                history.Bind(E1, TrimE1);
            }
        }
        history.UnBind(E2);
        if (status != ChFi2d_LastEdgeDegenerated && status != ChFi2d_BothEdgesDegenerated)
        {
            if (!E2.IsSame(TrimE2))
            {
                history.Bind(E2, TrimE2);
            }
        }
    } // UpDateHistory

    //=================================================================================================

    // Writes the modifications in history: TrimE1 is given by E1, TrimE2 by E2.
    private void UpDateHistory(in TopoDS_Edge E1, in TopoDS_Edge E2, in TopoDS_Edge TrimE1, in TopoDS_Edge TrimE2)
    {
        if (history.IsBound(E1))
        {
            history.UnBind(E1);
        }
        if (!E1.IsSame(TrimE1))
        {
            history.Bind(E1, TrimE1);
        }
        if (history.IsBound(E2))
        {
            history.UnBind(E2);
        }
        if (!E2.IsSame(TrimE2))
        {
            history.Bind(E2, TrimE2);
        }
    } // UpDateHistory

    //=================================================================================================

    /// <summary>
    /// The parent edge of <paramref name="E"/>: the edge of the initial face it was trimmed from, or
    /// <paramref name="E"/> itself if it is a basis edge (or not on the face).
    /// </summary>
    public TopoDS_Edge BasisEdge(in TopoDS_Edge E)
    {
        TopoDS_Edge anEdge;
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> iterator in history)
        {
            anEdge = TopoDS.Edge(iterator.Value);
            if (anEdge.IsSame(E))
            {
                TopoDS_Edge anotherEdge = TopoDS.Edge(iterator.Key);
                return anotherEdge;
            } // if (anEdge.IsSame ...
        } // while (Iterator.More ...
        return E;
    } // BasisEdge

    //=================================================================================================

    // Builds the fillet edge between AdjEdge1 and AdjEdge2 at V; NewExtr1 and NewExtr2 receive the new extremities of
    // the two edges. The edge has sense only if the status is IsDone (or one of the degenerated statuses).
    private TopoDS_Edge BuildFilletEdge(in TopoDS_Vertex V,
                                        in TopoDS_Edge AdjEdge1,
                                        in TopoDS_Edge AdjEdge2,
                                        double Radius,
                                        ref TopoDS_Vertex NewExtr1,
                                        ref TopoDS_Vertex NewExtr2)
    {
        TopoDS_Edge E1, E2;
        E1 = AdjEdge1;
        E2 = AdjEdge2;
        TopoDS_Vertex V1 = TopExp.FirstVertex(E1);
        TopoDS_Vertex V2 = TopExp.LastVertex(E1);
        TopoDS_Vertex V3 = TopExp.FirstVertex(E2);
        TopoDS_Vertex V4 = TopExp.LastVertex(E2);

        //========================================================================
        //    The first arc is found.                                        +
        //========================================================================

        TopAbs_Orientation O1;
        TopAbs_Orientation OE1;
        OE1 = E1.Orientation();
        E1.Orientation(TopAbs_FORWARD);
        E2.Orientation(TopAbs_FORWARD);
        TopoDS_Shape aLocalShape = E1.EmptyCopied();
        TopoDS_Edge Ebid1 = TopoDS.Edge(aLocalShape);
        aLocalShape = E2.EmptyCopied();
        TopoDS_Edge Ebid2 = TopoDS.Edge(aLocalShape);
        //  TopoDS_Edge Ebid1 = TopoDS::Edge(E1.EmptyCopied());
        //  TopoDS_Edge Ebid2 = TopoDS::Edge(E2.EmptyCopied());
        _ = Ebid1; // unused, as in OCCT
        _ = Ebid2;
        double param1, param2, param3, param4;

        //========================================================================
        //    Save non-modified parts of edges concerned.      +
        //========================================================================

        if (V1.IsSame(V))
        {
            param1 = BRep_Tool.Parameter(V1, E1);
            param2 = BRep_Tool.Parameter(V2, E1);
            O1 = V2.Orientation();
        }
        else
        {
            param1 = BRep_Tool.Parameter(V2, E1);
            param2 = BRep_Tool.Parameter(V1, E1);
            O1 = V1.Orientation();
        }
        if (V3.IsSame(V))
        {
            param3 = BRep_Tool.Parameter(V3, E2);
            param4 = BRep_Tool.Parameter(V4, E2);
        }
        else
        {
            param3 = BRep_Tool.Parameter(V4, E2);
            param4 = BRep_Tool.Parameter(V3, E2);
        }

        //========================================================================
        //    Restore geometric supports.                            +
        //========================================================================

        Geom2d_Curve? C1, C2;
        double ufirst1, ulast1, ufirst2, ulast2, U1, U2, PU1, PU2, Vv1, Vv2;
        double PPU1, PPU2;
        C1 = BRep_Tool.CurveOnSurface(E1, newFace, out ufirst1, out ulast1);
        C2 = BRep_Tool.CurveOnSurface(E2, newFace, out ufirst2, out ulast2);

        //========================================================================
        //   Determination of the face for fillet.                                +
        //========================================================================

        gp_Pnt2d p;
        gp_Vec2d Ve1, Ve2;
        gp_Vec2d Ve3, Ve4;
        bool Sens1, Sens2;

        Geom2d_Curve basisC1, basisC2;
        if (C1 is Geom2d_TrimmedCurve T1)
        {
            basisC1 = T1.BasisCurve();
        }
        else
        {
            basisC1 = C1!;
        }
        if (C2 is Geom2d_TrimmedCurve T2)
        {
            basisC2 = T2.BasisCurve();
        }
        else
        {
            basisC2 = C2!;
        }

        if (basisC1.DynamicType() == typeof(Geom2d_Circle))
        {
            var CC1 = (Geom2d_Circle)basisC1;
            ElCLib.D1(param1, CC1.Circ2d(), out p, out Ve1);
            Sens1 = (CC1.Circ2d()).IsDirect();
        } // if (C1->DynamicType() ...
        else
        {
            var CC1 = (basisC1 as Geom2d_Line)!; // OCCT dereferences down_cast's result unchecked
            ElCLib.D1(param1, CC1.Lin2d(), out p, out Ve1);
            Sens1 = true;
        } // else ...
        if (basisC2.DynamicType() == typeof(Geom2d_Circle))
        {
            var CC2 = (Geom2d_Circle)basisC2;
            ElCLib.D1(param3, CC2.Circ2d(), out p, out Ve2);
            Sens2 = (CC2.Circ2d()).IsDirect();
        } // if (C2->DynamicType() ...
        else
        {
            var CC2 = (basisC2 as Geom2d_Line)!;
            ElCLib.D1(param3, CC2.Lin2d(), out p, out Ve2);
            Sens2 = true;
        } // else ...

        TopoDS_Edge filletEdge = new();

        double cross = Ve1.Crossed(Ve2);
        Ve3 = Ve1;
        Ve4 = Ve2;

        // processing of tangency or downcast point
        if (Ve1.IsParallel(Ve2, Precision.Angular()))
        {
            // Ve1 and Ve2 are parallel : cross at 0
            cross = 0.0;
            if (param1 < param2)
            {
                Ve3 = -Ve1;
            }
            if (param3 > param4)
            {
                Ve4 = -Ve2;
            }

            if (!Ve4.IsOpposite(Ve3, Precision.Angular()))
            {
                // There is a true tangency point and the calculation is stopped
                status = ChFi2d_TangencyError;
                return filletEdge;
            }
            // Otherwise this is a downcast point, and the calculation is continued
        }

        GccEnt_Position Qual1, Qual2;
        if (cross < 0.0)
        {
            if (param3 > param4)
            {
                if (Sens1)
                {
                    Qual1 = GccEnt_enclosed;
                }
                else
                {
                    Qual1 = GccEnt_outside;
                }
            }
            else
            {
                if (Sens1)
                {
                    Qual1 = GccEnt_outside;
                }
                else
                {
                    Qual1 = GccEnt_enclosed;
                }
            }
            if (param1 > param2)
            {
                if (Sens2)
                {
                    Qual2 = GccEnt_outside;
                }
                else
                {
                    Qual2 = GccEnt_enclosed;
                }
            }
            else
            {
                if (Sens2)
                {
                    Qual2 = GccEnt_enclosed;
                }
                else
                {
                    Qual2 = GccEnt_outside;
                }
            }
        } // if (cross < 0 ...
        else
        {
            if (param3 > param4)
            {
                if (Sens1)
                {
                    Qual1 = GccEnt_outside;
                }
                else
                {
                    Qual1 = GccEnt_enclosed;
                }
            }
            else
            {
                if (Sens1)
                {
                    Qual1 = GccEnt_enclosed;
                }
                else
                {
                    Qual1 = GccEnt_outside;
                }
            }
            if (param1 > param2)
            {
                if (Sens2)
                {
                    Qual2 = GccEnt_enclosed;
                }
                else
                {
                    Qual2 = GccEnt_outside;
                }
            }
            else
            {
                if (Sens2)
                {
                    Qual2 = GccEnt_outside;
                }
                else
                {
                    Qual2 = GccEnt_enclosed;
                }
            }
        } // else ...

        double Tol = Precision.Confusion();
        // C++ converts the handles to adaptors implicitly (decision 805)
        var Fillet = new Geom2dGcc_Circ2d2TanRad(new Geom2dGcc_QualifiedCurve(new Geom2dAdaptor_Curve(basisC1), Qual1),
                                                 new Geom2dGcc_QualifiedCurve(new Geom2dAdaptor_Curve(basisC2), Qual2),
                                                 Radius,
                                                 Tol);
        if (!Fillet.IsDone() || Fillet.NbSolutions() == 0)
        {
            status = ChFi2d_ComputationError;
            return filletEdge;
        }
        else if (Fillet.NbSolutions() >= 1)
        {
            status = ChFi2d_IsDone;
            int numsol = 1;
            int nsol = 1;
            TopoDS_Vertex Vertex1 = new(), Vertex2 = new();
            gp_Pnt2d Ptg1, Ptg2;
            double dist;
            double dist1 = 1.0e40;
            bool inside = false;
            while (nsol <= Fillet.NbSolutions())
            {
                Fillet.Tangency1(nsol, out PU1, out PU2, out Ptg1);
                dist = Ptg1.Distance(p);
                if (basisC1.DynamicType() == typeof(Geom2d_Line))
                {
                    inside = (PU2 < param1 && PU2 > param2) || (PU2 < param2 && PU2 > param1);
                    if (inside && dist < dist1)
                    {
                        numsol = nsol;
                        dist1 = dist;
                    } // if ((((inside && ...
                } // if (C1->DynamicType( ...
                else
                {
                    Fillet.Tangency2(nsol, out PPU1, out PPU2, out Ptg2);
                    dist = Ptg2.Distance(p);
                    inside = (PPU2 < param3 && PPU2 > param4) || (PPU2 < param4 && PPU2 > param3);
                    //  case of arc of circle passing on the sewing
                    if ((basisC2.DynamicType() == typeof(Geom2d_Circle))
                        && ((2 * M_PI < param3 && 2 * M_PI > param4)
                            || (2 * M_PI < param4 && 2 * M_PI > param3)))
                    {
                        //  cas param3<param4
                        inside = (param3 < PPU2 && PPU2 < 2 * M_PI) || (0 <= PPU2 && PPU2 < param4 - 2 * M_PI);
                        //  cas param4<param3
                        inside =
                            inside || (param4 < PPU2 && PPU2 < 2 * M_PI) || (0 <= PPU2 && PPU2 < param3 - 2 * M_PI);
                    }
                    if (inside && dist < dist1)
                    {
                        numsol = nsol;
                        dist1 = dist;
                    } // if ((((param3 ...
                } // else ...
                nsol++;
            } // while (nsol ...
            var cir = Fillet.ThisSolution(numsol);
            var circle = new Geom2d_Circle(cir);

            var B = new BRep_Builder();
            var Adaptor3dSurface = new BRepAdaptor_Surface(refFace);
            var refSurf = new Geom_Plane(Adaptor3dSurface.Plane());
            Fillet.Tangency1(numsol, out U1, out U2, out Ptg1);
            Fillet.Tangency2(numsol, out Vv1, out Vv2, out Ptg2);

            // check the validity of parameters
            //// modified by jgv, 08.08.2011 for bug 0022695 ////
            // inside = (U2<param1 && U2>param2) || (U2<param2 && U2>param1);
            inside = (U2 < param1 && U2 >= param2) || (U2 <= param2 && U2 > param1);
            /////////////////////////////////////////////////////
            if ((basisC1.DynamicType() == typeof(Geom2d_Circle))
                && ((2 * M_PI < param1 && 2 * M_PI > param2) || (2 * M_PI < param2 && 2 * M_PI > param1)))
            {
                // arc of circle containing the circle origin
                //  case param1<param2
                inside = (param1 < U2 && U2 < 2 * M_PI) || (0 <= U2 && U2 < param2 - 2 * M_PI);
                //  case param2<param1
                inside = inside || (param2 < U2 && U2 < 2 * M_PI) || (0 <= U2 && U2 < param1 - 2 * M_PI);
            }
            if (!inside)
            {
                status = ChFi2d_ComputationError;
                return filletEdge;
            }

            //// modified by jgv, 08.08.2011 for bug 0022695 ////
            // inside = (Vv2<param3 && Vv2>param4) || (Vv2<param4 && Vv2>param3);
            inside = (Vv2 < param3 && Vv2 >= param4) || (Vv2 <= param4 && Vv2 > param3);
            /////////////////////////////////////////////////////
            if ((basisC2.DynamicType() == typeof(Geom2d_Circle))
                && ((2 * M_PI < param3 && 2 * M_PI > param4) || (2 * M_PI < param4 && 2 * M_PI > param3)))
            {
                // arc of circle containing the circle origin
                //  cas param3<param4
                inside = (param3 < Vv2 && Vv2 < 2 * M_PI) || (0 <= Vv2 && Vv2 < param4 - 2 * M_PI);
                //  cas param4<param3
                inside = inside || (param4 < Vv2 && Vv2 < 2 * M_PI) || (0 <= Vv2 && Vv2 < param3 - 2 * M_PI);
            }
            if (!inside)
            {
                status = ChFi2d_ComputationError;
                return filletEdge;
            }

            gp_Pnt p1 = Adaptor3dSurface.Value(Ptg1.X(), Ptg1.Y());
            gp_Pnt p2 = Adaptor3dSurface.Value(Ptg2.X(), Ptg2.Y());
            B.MakeVertex(ref Vertex1, p1, Tol);
            NewExtr1 = Vertex1;
            if (Math.Abs(U2 - ufirst1) <= Precision.PConfusion())
            {
                NewExtr1 = V1;
            }
            if (Math.Abs(U2 - ulast1) <= Precision.PConfusion())
            {
                NewExtr1 = V2;
            }

            B.MakeVertex(ref Vertex2, p2, Tol);
            NewExtr2 = Vertex2;
            if (Math.Abs(Vv2 - ufirst2) <= Precision.PConfusion())
            {
                NewExtr2 = V3;
            }
            if (Math.Abs(Vv2 - ulast2) <= Precision.PConfusion())
            {
                NewExtr2 = V4;
            }

            //=======================================================================
            //   Update tops of the fillet.                                  +
            //=======================================================================
            gp_Pnt Pntbid;
            gp_Pnt2d sommet;
            Pntbid = BRep_Tool.Pnt(V);
            sommet = new gp_Pnt2d(Pntbid.X(), Pntbid.Y());

            gp_Pnt pntBid;
            gp_Pnt2d somBid;
            if (V1.IsSame(V))
            {
                pntBid = BRep_Tool.Pnt(V2);
                somBid = new gp_Pnt2d(pntBid.X(), pntBid.Y());
            }
            else
            {
                pntBid = BRep_Tool.Pnt(V1);
                somBid = new gp_Pnt2d(pntBid.X(), pntBid.Y());
            }

            gp_Vec2d vec;
            ElCLib.D1(U1, cir, out Ptg1, out vec);

            gp_Vec2d vec1 = new();
            if (basisC1.DynamicType() == typeof(Geom2d_Circle))
            {
                var CC1 = (Geom2d_Circle)basisC1;
                gp_Circ2d cir2d = CC1.Circ2d();
                double par = ElCLib.Parameter(cir2d, Ptg1);
                ElCLib.D1(par, cir2d, out _, out vec1);
            } // if (C1->DynamicType() ...
            else if (basisC1.DynamicType() == typeof(Geom2d_Line))
            {
                var CC1 = (Geom2d_Line)basisC1;
                gp_Lin2d lin2d = CC1.Lin2d();
                double par = ElCLib.Parameter(lin2d, sommet);
                vec1 = new gp_Vec2d(sommet.X() - somBid.X(), sommet.Y() - somBid.Y()); // OCCT: overwritten at once
                ElCLib.D1(par, lin2d, out _, out vec1);
            } // else if ...

            if (OE1 == TopAbs_REVERSED)
            {
                vec1.Reverse();
            } // if (OE1 ...
            bool Sense = (vec1 * vec) > 0.0;
            if (U1 > Vv1 && U1 > 2.0 * M_PI)
            {
                ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, Precision.Confusion(), ref U1, ref Vv1);
            } // if (U1 ...
            if ((O1 == TopAbs_FORWARD && OE1 == TopAbs_FORWARD)
                || (O1 == TopAbs_REVERSED && OE1 == TopAbs_REVERSED))
            {
                filletEdge = new BRepLib_MakeEdge(circle, refSurf, NewExtr1, NewExtr2, U1, Vv1).InternalEdge();
            } // if (O1 == ...
            else
            {
                filletEdge = new BRepLib_MakeEdge(circle, refSurf, NewExtr2, NewExtr1, Vv1, U1).InternalEdge();
            } // else ...
            if (!Sense)
            {
                TopAbs_Orientation S1 = filletEdge.Orientation();
                if ((O1 == TopAbs_FORWARD && OE1 == TopAbs_FORWARD)
                    || (O1 == TopAbs_REVERSED && OE1 == TopAbs_REVERSED))
                {
                    filletEdge = new BRepLib_MakeEdge(circle, refSurf, NewExtr2, NewExtr1, Vv1, U1).InternalEdge();
                }
                else
                {
                    filletEdge = new BRepLib_MakeEdge(circle, refSurf, NewExtr1, NewExtr2, U1, Vv1).InternalEdge();
                }
                if (S1 == TopAbs_FORWARD)
                {
                    filletEdge.Orientation(TopAbs_REVERSED);
                }
                else
                {
                    filletEdge.Orientation(TopAbs_FORWARD);
                }
            } // if (!Sense

        } //  else if

        BRepLib.BuildCurves3d(filletEdge);
        return filletEdge;
    } // BuildFilletEdge

    //=================================================================================================

    private bool IsAFillet(in TopoDS_Edge E)
    {
        int i = 1;
        while (i <= fillets.Length())
        {
            TopoDS_Edge currentEdge = TopoDS.Edge(fillets.Value(i));
            if (currentEdge.IsSame(E))
            {
                return true;
            }
            i++;
        }
        return false;
    } // IsAFillet

    //=================================================================================================

    private bool IsAChamfer(in TopoDS_Edge E)
    {
        int i = 1;
        while (i <= chamfers.Length())
        {
            TopoDS_Edge currentEdge = TopoDS.Edge(chamfers.Value(i));
            if (currentEdge.IsSame(E))
            {
                return true;
            }
            i++;
        }
        return false;
    } // IsAChamfer

    //=================================================================================================

    // OCCT has this function twice, as a file-static in ChFi2d_Builder.cxx and in ChFi2d_Builder_0.cxx, identical.
    private static bool IsLineOrCircle(in TopoDS_Edge E, in TopoDS_Face F)
    {
        double first, last;
        //  syntaxe invalide sur NT
        //      const occ::handle<Geom2d_Curve>& C =
        //	BRep_Tool::CurveOnSurface(E,F,first,last);
        Geom2d_Curve? C = BRep_Tool.CurveOnSurface(E, F, out first, out last);
        Geom2d_Curve basisC;
        if (C is Geom2d_TrimmedCurve TC)
        {
            basisC = TC.BasisCurve();
        }
        else
        {
            basisC = C!;
        }

        return basisC.DynamicType() == typeof(Geom2d_Circle)
               || basisC.DynamicType() == typeof(Geom2d_Line); // else ...
    } // IsLineOrCircle

    // ---- the inline accessors (ChFi2d_Builder.lxx) ----

    /// <summary>The modified face (frozen), with the initial face's orientation.</summary>
    public TopoDS_Face Result()
    {
        TopoDS_Face aFace = InternalResult();
        aFace.TShape()?.FreezeGraph();
        return aFace;
    }

    /// <summary><see cref="Result"/> without freezing, for ported code.</summary>
    internal TopoDS_Face InternalResult()
    {
        TopoDS_Face aFace = newFace;
        aFace.Orientation(refFace.Orientation());
        return aFace;
    }

    /// <summary>True if the edge <paramref name="E"/> of the initial face has been trimmed by a fillet or chamfer.</summary>
    public bool IsModified(in TopoDS_Edge E)
    {
        return history.IsBound(E);
    }

    /// <summary>The fillets built (frozen with the face that holds them).</summary>
    public ISchematiqList<TopoDS_Shape> FilletEdges() => PublishedEdges(fillets);

    /// <summary>The builder's own sequence behind <see cref="FilletEdges"/>, unfrozen.</summary>
    internal NCollection_Sequence<TopoDS_Shape> InternalFilletEdges() => fillets;

    public int NbFillet()
    {
        return fillets.Length();
    }

    /// <summary>The chamfers built (frozen with the face that holds them).</summary>
    public ISchematiqList<TopoDS_Shape> ChamferEdges() => PublishedEdges(chamfers);

    /// <summary>The builder's own sequence behind <see cref="ChamferEdges"/>, unfrozen.</summary>
    internal NCollection_Sequence<TopoDS_Shape> InternalChamferEdges() => chamfers;

    public int NbChamfer()
    {
        return chamfers.Length();
    }

    public bool HasDescendant(in TopoDS_Edge E)
    {
        return history.IsBound(E);
    }

    /// <summary>
    /// The edge trimmed from <paramref name="E"/> (frozen with the face that holds it).
    /// </summary>
    /// <exception cref="Standard_NoSuchObject"><paramref name="E"/> has no descendant.</exception>
    public TopoDS_Edge DescendantEdge(in TopoDS_Edge E)
    {
        FreezeResult();
        return InternalDescendantEdge(E);
    }

    /// <summary><see cref="DescendantEdge"/> without freezing, for ported code.</summary>
    internal TopoDS_Edge InternalDescendantEdge(in TopoDS_Edge E)
    {
        return TopoDS.Edge(history.Find(E));
    } // DescendantEdge

    public ChFi2d_ConstructionError Status()
    {
        return status;
    }

    // ---- publication (decision 853) ----

    // Freezes the face being built, which holds every fillet, chamfer and descendant edge handed out.
    internal void FreezeResult()
    {
        newFace.TShape()?.FreezeGraph();
    }

    private TopoDS_Edge Published(in TopoDS_Edge theEdge)
    {
        FreezeResult();
        theEdge.TShape()?.FreezeGraph();
        return theEdge;
    }

    private ISchematiqList<TopoDS_Shape> PublishedEdges(NCollection_Sequence<TopoDS_Shape> theEdges)
    {
        FreezeResult();
        if (theEdges.IsEmpty())
        {
            return PackedList<TopoDS_Shape>.Empty;
        }
        var anEdges = new TopoDS_Shape[theEdges.Length()];
        for (int i = 1; i <= theEdges.Length(); i++)
        {
            anEdges[i - 1] = theEdges.Value(i);
            anEdges[i - 1].TShape()?.FreezeGraph();
        }
        return PackedList.Own(anEdges);
    }
}
