// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_Builder_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Modified : 08/07/97 : JPI : traitement des edges degeneres comme pour les fillets 2D
// Modified:	Fri Sep 25 09:38:04 1998
//              status = ChFi2d_NotAuthorized si les aretes ne sont pas
//              des droites ou des cercles; fonction IsLineOrCircle
//              (BUC60288)

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Planar.ConstructionError;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets.Planar;

internal sealed partial class PlanarBuilder
{
    //=================================================================================================

    /// <summary>
    /// Adds a chamfer between the two connected edges <paramref name="e1"/> and <paramref name="e2"/>, at the
    /// distances <paramref name="d1"/> along <paramref name="e1"/> and <paramref name="d2"/> along <paramref name="e2"/>,
    /// and returns it (frozen). The edge has sense only if <see cref="Status"/> is <see cref="IsDone"/>.
    /// </summary>
    public Edge AddChamfer(in Edge e1, in Edge e2, double d1, double d2)
    {
        return Published(InternalAddChamfer(e1, e2, d1, d2));
    }

    private Edge InternalAddChamfer(in Edge e1, in Edge e2, double d1, double d2)
    {
        Vertex commonVertex = new();
        Edge basisEdge1, basisEdge2;
        Edge e1Mod = new(), e2Mod = new(), chamfer = new();

        bool hasConnection = global::Kerf.Engine.Fillets.Planar.ChFi2d.CommonVertex(e1, e2, ref commonVertex);
        if (!hasConnection)
        {
            return chamfer;
        }

        if (IsAFillet(e1) || IsAChamfer(e1) || IsAFillet(e2) || IsAChamfer(e2))
        {
            _status = ConstructionError.NotAuthorized;
            return chamfer;
        } //  if (IsAChamfer ...

        if (!IsLineOrCircle(e1, _newFace) || !IsLineOrCircle(e2, _newFace))
        {
            _status = ConstructionError.NotAuthorized;
            return chamfer;
        } //  if (!IsLineOrCircle ...

        // EE1 and EE2 are copies of E1 and E2 with the good orientation
        // on <newFace>
        Edge ee1 = new(), ee2 = new();
        _status = global::Kerf.Engine.Fillets.Planar.ChFi2d.FindConnectedEdges(_newFace, commonVertex, ref ee1, ref ee2);
        if (ee1.IsSame(e2))
        {
            Orientation orient = ee1.Orientation;
            ee1 = ee2;
            ee2 = e2;
            ee2.Orient(orient);
        }

        ComputeChamfer(commonVertex, ee1, ee2, d1, d2, ref e1Mod, ref e2Mod, ref chamfer);
        if (_status == ConstructionError.IsDone || _status == ConstructionError.FirstEdgeDegenerated
            || _status == ConstructionError.LastEdgeDegenerated || _status == ConstructionError.BothEdgesDegenerated)
        {
            //  if (status == ChFi2d_IsDone) {
            BuildNewWire(ee1, ee2, e1Mod, chamfer, e2Mod);
            basisEdge1 = BasisEdge(ee1);
            basisEdge2 = BasisEdge(ee2);
            UpDateHistory(basisEdge1, basisEdge2, e1Mod, e2Mod, chamfer, 2);
            _status = ConstructionError.IsDone;
            return ShapeCasts.Edge(_chamfers.Value(_chamfers.Length()));
        }
        return chamfer;
    } // AddChamfer

    //=================================================================================================

    /// <summary>
    /// Adds a chamfer between the two edges connected to the vertex <paramref name="v"/>, making the angle
    /// <paramref name="ang"/> (radians) with the edge <paramref name="e"/>, one of its ends on <paramref name="e"/> at the
    /// distance <paramref name="d"/>; returns it (frozen). The edge has sense only if <see cref="Status"/> is
    /// <see cref="IsDone"/>.
    /// </summary>
    public Edge AddChamfer(in Edge e, in Vertex v, double d, double ang)
    {
        return Published(InternalAddChamfer(e, v, d, ang));
    }

    private Edge InternalAddChamfer(in Edge e, in Vertex v, double d, double ang)
    {
        Edge chamfer = new(), adjEdge1 = new(), adjEdge2 = new();
        _status = global::Kerf.Engine.Fillets.Planar.ChFi2d.FindConnectedEdges(_newFace, v, ref adjEdge1, ref adjEdge2);
        if (_status == ConstructionError.ConnexionError)
        {
            return chamfer;
        }

        // adjEdge1 is a copy of E  with the good orientation
        // on <newFace>
        if (adjEdge2.IsSame(e))
        {
            Orientation orient = adjEdge2.Orientation;
            adjEdge2 = adjEdge1;
            adjEdge1 = e;
            adjEdge1.Orient(orient);
        }

        if (IsAFillet(adjEdge1) || IsAChamfer(adjEdge1) || IsAFillet(adjEdge2) || IsAChamfer(adjEdge2))
        {
            _status = ConstructionError.NotAuthorized;
            return chamfer;
        } //  if (IsAChamfer ...

        if (!IsLineOrCircle(adjEdge1, _newFace) || !IsLineOrCircle(adjEdge2, _newFace))
        {
            _status = ConstructionError.NotAuthorized;
            return chamfer;
        } //  if (!IsLineOrCircle ...

        Edge e1 = new(), e2 = new();
        ComputeChamfer(v, adjEdge1, d, ang, adjEdge2, ref e1, ref e2, ref chamfer);
        Edge basisEdge1, basisEdge2;
        if (_status == ConstructionError.IsDone || _status == ConstructionError.FirstEdgeDegenerated
            || _status == ConstructionError.LastEdgeDegenerated || _status == ConstructionError.BothEdgesDegenerated)
        {
            //  if (status == ChFi2d_IsDone) {
            BuildNewWire(adjEdge1, adjEdge2, e1, chamfer, e2);
            basisEdge1 = BasisEdge(adjEdge1);
            basisEdge2 = BasisEdge(adjEdge2);
            UpDateHistory(basisEdge1, basisEdge2, e1, e2, chamfer, 2);
            _status = ConstructionError.IsDone;
            return ShapeCasts.Edge(_chamfers.Value(_chamfers.Length()));
        }
        return chamfer;
    }

    //=================================================================================================

    // The chamfer computed from a vertex, two edges and two distances. TrimE1, TrimE2 and Chamfer have sense only if
    // the status is IsDone.
    private void ComputeChamfer(in Vertex v,
                                in Edge e1,
                                in Edge e2,
                                double d1,
                                double d2,
                                ref Edge trimE1,
                                ref Edge trimE2,
                                ref Edge chamfer)
    {
        Vertex newExtr1 = new(), newExtr2 = new();
        bool degen1, degen2;
        chamfer = BuildChamferEdge(v, e1, e2, d1, d2, ref newExtr1, ref newExtr2);
        if (_status != ConstructionError.IsDone)
        {
            return;
        }
        trimE1 = BuildNewEdge(e1, v, newExtr1, out degen1);
        trimE2 = BuildNewEdge(e2, v, newExtr2, out degen2);
        if (degen1 && degen2)
        {
            _status = ConstructionError.BothEdgesDegenerated;
        }
        if (degen1 && !degen2)
        {
            _status = ConstructionError.FirstEdgeDegenerated;
        }
        if (!degen1 && degen2)
        {
            _status = ConstructionError.LastEdgeDegenerated;
        }
        //   TrimE1 = BuildNewEdge(E1, V, newExtr1);
        //  TrimE2 = BuildNewEdge(E2, V, newExtr2);
    } // ComputeChamfer

    //=================================================================================================

    // The chamfer computed from an edge, a vertex, a distance and an angle. TrimE1, TrimE2 and Chamfer have sense only
    // if the status is IsDone.
    private void ComputeChamfer(in Vertex v,
                                in Edge e1,
                                double d,
                                double ang,
                                in Edge e2,
                                ref Edge trimE1,
                                ref Edge trimE2,
                                ref Edge chamfer)
    {
        Vertex newExtr1 = new(), newExtr2 = new();
        bool degen1, degen2;
        chamfer = BuildChamferEdge(v, e1, d, ang, e2, ref newExtr1, ref newExtr2);
        if (_status != ConstructionError.IsDone)
        {
            return;
        }
        trimE1 = BuildNewEdge(e1, v, newExtr1, out degen1);
        trimE2 = BuildNewEdge(e2, v, newExtr2, out degen2);
        if (degen1 && degen2)
        {
            _status = ConstructionError.BothEdgesDegenerated;
        }
        if (degen1 && !degen2)
        {
            _status = ConstructionError.FirstEdgeDegenerated;
        }
        if (!degen1 && degen2)
        {
            _status = ConstructionError.LastEdgeDegenerated;
        }
        //   TrimE1 = BuildNewEdge(E1, V, newExtr1);
        //   TrimE2 = BuildNewEdge(E2, V, newExtr2);
    } // ComputeChamfer

    //=================================================================================================

    /// <summary>
    /// Modifies the chamfer <paramref name="chamfer"/> built between the edges <paramref name="e1"/> and
    /// <paramref name="e2"/> (only <paramref name="e2"/> is used, as in OCCT), and returns the new chamfer (frozen).
    /// </summary>
    public Edge ModifyChamfer(in Edge chamfer, in Edge e1, in Edge e2, double d1, double d2)
    {
        _ = e1; // OCCT: E1 is unused
        Vertex vertex = InternalRemoveChamfer(chamfer);
        Edge adjEdge1 = new(), adjEdge2 = new();
        _status = global::Kerf.Engine.Fillets.Planar.ChFi2d.FindConnectedEdges(_newFace, vertex, ref adjEdge1, ref adjEdge2);
        Edge aChamfer = new();
        if (_status == ConstructionError.ConnexionError)
        {
            return aChamfer;
        }

        // adjEdge1 and adjEdge2 are copies of E1 and E2 with the good orientation
        // on <newFace>
        if (adjEdge1.IsSame(e2))
        {
            Orientation orient = adjEdge1.Orientation;
            adjEdge1 = adjEdge2;
            adjEdge2 = e2;
            adjEdge2.Orient(orient);
        }

        aChamfer = InternalAddChamfer(adjEdge1, adjEdge2, d1, d2);
        return Published(aChamfer);
    } // ModifyChamfer

    //=================================================================================================

    /// <summary>
    /// Modifies the chamfer <paramref name="chamfer"/>: the new one makes the angle <paramref name="ang"/> with the edge
    /// <paramref name="e"/>, one of its ends on <paramref name="e"/> at the distance <paramref name="d"/> from the vertex
    /// it is built on. Returns the new chamfer (frozen).
    /// </summary>
    public Edge ModifyChamfer(in Edge chamfer, in Edge e, double d, double ang)
    {
        Vertex vertex = InternalRemoveChamfer(chamfer);
        Edge adjEdge1 = new(), adjEdge2 = new();
        _status = global::Kerf.Engine.Fillets.Planar.ChFi2d.FindConnectedEdges(_newFace, vertex, ref adjEdge1, ref adjEdge2);
        Edge aChamfer = new();
        if (_status == ConstructionError.ConnexionError)
        {
            return aChamfer;
        }

        if (adjEdge1.IsSame(e))
        {
            aChamfer = InternalAddChamfer(adjEdge1, vertex, d, ang);
        }
        else
        {
            aChamfer = InternalAddChamfer(adjEdge2, vertex, d, ang);
        }
        return Published(aChamfer);
    } // ModifyChamfer

    //=================================================================================================

    /// <summary>
    /// Removes the chamfer <paramref name="chamfer"/> and returns the vertex connecting the two edges adjacent to it
    /// (frozen; null if <paramref name="chamfer"/> is not a chamfer of this builder).
    /// </summary>
    public Vertex RemoveChamfer(in Edge chamfer)
    {
        Vertex vertex = InternalRemoveChamfer(chamfer);
        vertex.TShape()?.FreezeGraph();
        return vertex;
    }

    private Vertex InternalRemoveChamfer(in Edge chamfer)
    {
        Vertex commonVertex = new();

        int i = 1;
        bool isFind = false;
        while (i <= _chamfers.Length())
        {
            Edge aChamfer = ShapeCasts.Edge(_chamfers.Value(i));
            if (aChamfer.IsSame(chamfer))
            {
                _chamfers.Remove(i);
                isFind = true;
                break;
            }
            i++;
        }
        if (!isFind)
        {
            return commonVertex;
        }

        Exploration.Vertices(chamfer, out Vertex firstVertex, out Vertex lastVertex);

        Edge adjEdge1 = new(), adjEdge2 = new();
        _status = global::Kerf.Engine.Fillets.Planar.ChFi2d.FindConnectedEdges(_newFace, firstVertex, ref adjEdge1, ref adjEdge2);
        if (_status == ConstructionError.ConnexionError)
        {
            return commonVertex;
        }

        Edge basisEdge1, basisEdge2, e1, e2;
        // E1 and E2 are the adjacentes edges to Chamfer

        if (adjEdge1.IsSame(chamfer))
        {
            e1 = adjEdge2;
        }
        else
        {
            e1 = adjEdge1;
        }
        basisEdge1 = BasisEdge(e1);
        _status = global::Kerf.Engine.Fillets.Planar.ChFi2d.FindConnectedEdges(_newFace, lastVertex, ref adjEdge1, ref adjEdge2);
        if (_status == ConstructionError.ConnexionError)
        {
            return commonVertex;
        }
        if (adjEdge1.IsSame(chamfer))
        {
            e2 = adjEdge2;
        }
        else
        {
            e2 = adjEdge1;
        }
        basisEdge2 = BasisEdge(e2);
        Vertex connectionE1Chamfer = new(), connectionE2Chamfer = new();
        bool hasConnection = global::Kerf.Engine.Fillets.Planar.ChFi2d.CommonVertex(basisEdge1, basisEdge2, ref commonVertex);
        if (!hasConnection)
        {
            _status = ConstructionError.ConnexionError;
            return commonVertex;
        }
        hasConnection = global::Kerf.Engine.Fillets.Planar.ChFi2d.CommonVertex(e1, chamfer, ref connectionE1Chamfer);
        if (!hasConnection)
        {
            _status = ConstructionError.ConnexionError;
            return commonVertex;
        }
        hasConnection = global::Kerf.Engine.Fillets.Planar.ChFi2d.CommonVertex(e2, chamfer, ref connectionE2Chamfer);
        if (!hasConnection)
        {
            _status = ConstructionError.ConnexionError;
            return commonVertex;
        }

        // rebuild edges on wire
        Edge newEdge1 = new(), newEdge2 = new();
        Vertex v, v1, v2;
        var makeEdge = new LibraryMakeEdge();
        Placement loc;
        double first, last;

        Exploration.Vertices(e1, out firstVertex, out lastVertex);
        Exploration.Vertices(basisEdge1, out v1, out v2);
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
                Curve3D? curve = Tool.Curve(e1, out loc, out first, out last);
                makeEdge.Init(curve!, commonVertex, lastVertex);
                newEdge1 = makeEdge.InternalEdge();
                newEdge1.Orient(basisEdge1.Orientation);
                newEdge1.Location(basisEdge1.Placement);
            } // if (firstVertex ...
            else if (lastVertex.IsSame(connectionE1Chamfer))
            {
                //  syntaxe invalide sur NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(E1, loc, first, last);
                Curve3D? curve = Tool.Curve(e1, out loc, out first, out last);
                makeEdge.Init(curve!, firstVertex, commonVertex);
                newEdge1 = makeEdge.InternalEdge();
                newEdge1.Orient(basisEdge1.Orientation);
                newEdge1.Location(basisEdge1.Placement);
            } // else if (lastVertex ...
        } // else ...

        Exploration.Vertices(basisEdge2, out v1, out v2);
        if (v1.IsSame(commonVertex))
        {
            v = v2;
        }
        else
        {
            v = v1;
        }

        Exploration.Vertices(e2, out firstVertex, out lastVertex);
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
                Curve3D? curve = Tool.Curve(e2, out loc, out first, out last);
                makeEdge.Init(curve!, commonVertex, lastVertex);
                newEdge2 = makeEdge.InternalEdge();
                newEdge2.Orient(basisEdge2.Orientation);
                newEdge2.Location(basisEdge2.Placement);
            } // if (firstVertex ...
            else if (lastVertex.IsSame(connectionE2Chamfer))
            {
                //  syntaxe invalide sur NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(E2, loc, first, last);
                Curve3D? curve = Tool.Curve(e2, out loc, out first, out last);
                makeEdge.Init(curve!, firstVertex, commonVertex);
                newEdge2 = makeEdge.InternalEdge();
                newEdge2.Orient(basisEdge2.Orientation);
                newEdge2.Location(basisEdge2.Placement);
            } // else if (lastVertex ...
        } // else ...

        // rebuild the newFace, wire by wire (decision 1474)
        Edge e1Copy = e1, e2Copy = e2, chamferCopy = chamfer, newEdge1Copy = newEdge1, newEdge2Copy = newEdge2;
        RebuildFace(null, theEdge => !theEdge.IsSame(e1Copy) && !theEdge.IsSame(e2Copy) && !theEdge.IsSame(chamferCopy) ? null
                               : theEdge.IsEqual(e1Copy) ? [newEdge1Copy]
                               : theEdge.IsEqual(e2Copy) ? [newEdge2Copy]
                               : []);

        UpDateHistory(basisEdge1, basisEdge2, newEdge1, newEdge2);

        return commonVertex;
    } // RemoveChamfer

    //=================================================================================================

    // The chamfer between AdjEdge1 and AdjEdge2 at V, at the distances D1 and D2; NewExtr1 and NewExtr2 receive the new
    // extremities of the two edges. The edge has sense only if the status is IsDone.
    private Edge BuildChamferEdge(in Vertex v,
                                         in Edge adjEdge1,
                                         in Edge adjEdge2,
                                         double d1,
                                         double d2,
                                         ref Vertex newExtr1,
                                         ref Vertex newExtr2)
    {
        Edge chamfer = new();
        if (d1 <= 0 || d2 <= 0)
        {
            _status = ConstructionError.ParametersError;
            return chamfer;
        } // if ( D1 <=0 ...

        double param1, param2;
        Point3D p1 = ComputePoint(v, adjEdge1, d1, out param1);
        Point3D p2 = ComputePoint(v, adjEdge2, d2, out param2);

        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        var b = new BrepBuilder();
        b.MakeVertex(ref newExtr1, p1, tol);
        b.MakeVertex(ref newExtr2, p2, tol);
        newExtr1.Orient(Forward);
        newExtr2.Orient(Reversed);

        // chamfer edge construction
        Surface? refSurf = Tool.Surface(_refFace, out Placement loc);
        _ = refSurf; // unused, as in OCCT
        _ = loc;
        var myVec = new Vector3D(p1, p2);
        var myDir = new Direction3D(myVec);
        var newLine = new LineCurve3D(p1, myDir);
        double param = global::Kerf.Engine.Geometry.ElCLib.Parameter(newLine.Line, p2);
        b.MakeEdge(ref chamfer, newLine, tol);
        b.Range(chamfer, 0.0, param);
        b.Add(chamfer, newExtr1);
        b.UpdateVertex(newExtr1, 0.0, chamfer, tol);
        b.Add(chamfer, newExtr2);
        b.UpdateVertex(newExtr2, param, chamfer, tol);
        OrientChamfer(ref chamfer, adjEdge1, v);

        // set the orientation of NewExtr1 and NewExtr2 for the adjacent edges
        Vertex v1 = Exploration.FirstVertex(adjEdge1);
        Vertex v2 = Exploration.LastVertex(adjEdge1);
        if (v1.IsSame(v))
        {
            newExtr1.Orient(v1.Orientation);
        }
        else
        {
            newExtr1.Orient(v2.Orientation);
        }

        v1 = Exploration.FirstVertex(adjEdge2);
        v2 = Exploration.LastVertex(adjEdge2);
        if (v1.IsSame(v))
        {
            newExtr2.Orient(v1.Orientation);
        }
        else
        {
            newExtr2.Orient(v2.Orientation);
        }
        UpdateNewVertex(b, newExtr1, param1, adjEdge1, tol);
        UpdateNewVertex(b, newExtr2, param2, adjEdge2, tol);

        _status = ConstructionError.IsDone;
        return chamfer;
    } // BuildChamferEdge

    //=================================================================================================

    // The chamfer at V making the angle Ang with AdjEdge1, one end on AdjEdge1 at the distance D; NewExtr1 and NewExtr2
    // receive the new extremities of the two edges. The edge has sense only if the status is IsDone.
    private Edge BuildChamferEdge(in Vertex v,
                                         in Edge adjEdge1,
                                         double d,
                                         double ang,
                                         in Edge adjEdge2,
                                         ref Vertex newExtr1,
                                         ref Vertex newExtr2)
    {
        Edge chamfer = new();
        if (d <= 0 || ang <= 0)
        {
            _status = ConstructionError.ParametersError;
            return chamfer;
        } // if ( D <= 0 ...

        double param1, param2;
        Point3D p1 = ComputePoint(v, adjEdge1, d, out param1);
        Point3D p = Tool.Pnt(v);
        var myVec = new Vector3D(p1, p);

        // compute the tangent vector on AdjEdge2 at the vertex V.
        var c = new EdgeCurveAdaptor(adjEdge2, _refFace);
        double first, last;
        first = c.FirstParameter();
        last = c.LastParameter();

        Point3D point;
        Vector3D tan;
        c.D1(first, out point, out tan);
        if (point.DistanceTo(p) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            c.D1(last, out point, out tan);
        }
        // tangent orientation
        Exploration.Vertices(adjEdge2, out Vertex v1, out Vertex v2);
        Orientation orient;
        if (v1.IsSame(v))
        {
            orient = v1.Orientation;
        }
        else
        {
            orient = v2.Orientation;
        }
        if (orient == Reversed)
        {
            tan *= -1;
        }

        // compute the chamfer geometric support
        var rotAxe = new Axis3D(p1, new Direction3D(tan ^ myVec));
        Vector3D vecLin = myVec.Rotated(rotAxe, -ang);
        var myDir = new Direction3D(vecLin);
        var newLine = new LineCurve3D(p1, myDir);
        var b1 = new BrepBuilder();
        b1.MakeEdge(ref chamfer, newLine, global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D p2 = ComputePoint(_refFace, newLine, adjEdge2, out param2);

        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        var b = new BrepBuilder();
        b.MakeVertex(ref newExtr1, p1, tol);
        b.MakeVertex(ref newExtr2, p2, tol);
        newExtr1.Orient(Forward);
        newExtr2.Orient(Reversed);

        // chamfer edge construction
        double param = global::Kerf.Engine.Geometry.ElCLib.Parameter(newLine.Line, p2);
        b.MakeEdge(ref chamfer, newLine, tol);
        b.Range(chamfer, 0.0, param);
        b.Add(chamfer, newExtr1);
        b.UpdateVertex(newExtr1, 0.0, chamfer, tol);
        b.Add(chamfer, newExtr2);
        b.UpdateVertex(newExtr2, param, chamfer, tol);
        OrientChamfer(ref chamfer, adjEdge1, v);

        // set the orientation of NewExtr1 and NewExtr2 for the adjacent edges
        Vertex V1 = Exploration.FirstVertex(adjEdge1);
        Vertex V2 = Exploration.LastVertex(adjEdge1);
        if (V1.IsSame(v))
        {
            newExtr1.Orient(V1.Orientation);
        }
        else
        {
            newExtr1.Orient(V2.Orientation);
        }

        V1 = Exploration.FirstVertex(adjEdge2);
        V2 = Exploration.LastVertex(adjEdge2);
        if (V1.IsSame(v))
        {
            newExtr2.Orient(V1.Orientation);
        }
        else
        {
            newExtr2.Orient(V2.Orientation);
        }
        UpdateNewVertex(b, newExtr1, param1, adjEdge1, tol);
        UpdateNewVertex(b, newExtr2, param2, adjEdge2, tol);

        _status = ConstructionError.IsDone;
        return chamfer;
    }

    // Kerf: frozen (decision 855). OCCT's B.UpdateVertex(NewExtr, param, AdjEdge, tol) gives the new vertex its
    // points on the adjacent edge's curves (the vertex isn't on the edge, so nothing else of the edge is read), raises
    // its tolerance, and sets the edge's Modified flag. When the adjacent edge is read-only (frozen, the caller's or one
    // the builder published, and no working overlay here takes the write), the same call goes through an EmptyCopied
    // stand-in, which has the edge's representations and location: the vertex gets OCCT's points, and the edge's flag
    // is left as it is.
    private static void UpdateNewVertex(BrepBuilder b, in Vertex v, double par, in Edge e, double tol)
    {
        if (WorkingOverlay.IsReadOnly(e.TShape()!) && !Tool.Degenerated(e))
        {
            b.UpdateVertex(v, par, ShapeCasts.Edge(e.EmptyCopied()), tol);
            return;
        }
        b.UpdateVertex(v, par, e, tol);
    }

    //=================================================================================================

    // The point at the distance D from V along E (its parameter in Param).
    private static Point3D ComputePoint(in Vertex v, in Edge e, double d, out double param)
    {
        // geometric support
        var c = new EdgeCurveAdaptor(e);
        double first, last;
        first = c.FirstParameter();
        last = c.LastParameter();

        Point3D thePoint = new();
        if (c.GetType() == Line)
        {
            Point3D p1, p2;
            Exploration.Vertices(e, out Vertex v1, out Vertex v2);
            p1 = Tool.Pnt(v1);
            p2 = Tool.Pnt(v2);
            var myVec = new Vector3D(p1, p2);
            myVec = myVec.Normalized();
            myVec *= d;
            if (v2.IsSame(v))
            {
                myVec *= -1; // change the sense of myVec
                thePoint = p2.Translated(myVec);
            } // if (v2 ...
            else
            {
                thePoint = p1.Translated(myVec);
            }

            param = global::Kerf.Engine.Geometry.ElCLib.Parameter(c.Line(), thePoint);
            // szv:OCC20823-begin
            c.D0(param, out thePoint);
            // szv:OCC20823-end
            return thePoint;
        } // if (C->IsKind(TYPE ...

        if (c.GetType() == Circle)
        {
            Circle3D cir = c.Circle();
            double radius = cir.Radius;
            Exploration.Vertices(e, out Vertex v1, out Vertex v2);
            double param1, param2;
            if (v.IsSame(v1))
            {
                param1 = Tool.Parameter(v1, e);
                param2 = Tool.Parameter(v2, e);
            }
            else
            {
                param1 = Tool.Parameter(v2, e);
                param2 = Tool.Parameter(v1, e);
            }
            double deltaAlpha = d / radius;
            if (param1 > param2)
            {
                param = param1 - deltaAlpha;
            }
            else
            {
                param = param1 + deltaAlpha;
            }
            c.D0(param, out thePoint);
            return thePoint;
        } // if (C->IsKind(TYPE ...

        else
        {
            // in all other case than lines and circles.
            Point3D p;
            Exploration.Vertices(e, out Vertex v1, out Vertex v2);
            if (v.IsSame(v1))
            {
                p = Tool.Pnt(v1);
            }
            else
            {
                p = Tool.Pnt(v2);
            }

            CurveAdaptor cc = c.Curve();
            if (p.DistanceTo(c.Value(first)) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                var computePoint = new AbscissaPoint(cc, d, first);
                param = computePoint.Parameter();
            }
            else
            {
                var computePoint = new AbscissaPoint(cc, d, last);
                param = computePoint.Parameter();
            }
            thePoint = cc.Value(param);
            return thePoint;
        } // else ...
    } // ComputePoint

    //=================================================================================================

    // The first intersection (along L) of the line L with the edge E on the face F; Param is its parameter on E.
    private static Point3D ComputePoint(in Face f, LineCurve3D l, in Edge e, out double param)
    {
        var adaptor3dSurface = new FaceSurfaceAdaptor(f);
        var refSurf = new PlaneSurface(adaptor3dSurface.Plane());
        Curve2D lin2d = Projections.To2d(l, refSurf.Plane)!;
        Curve2D? c2d;
        double first, last;
        c2d = Tool.CurveOnSurface(e, f, out first, out last);
        var adaptorL = new CurveAdaptor2D(lin2d);
        var adaptorC = new CurveAdaptor2D(c2d!);
        var intersection = new GInter(adaptorL,
                                                adaptorC,
                                                global::Kerf.Engine.Foundation.Precision.PIntersection(),
                                                global::Kerf.Engine.Foundation.Precision.PIntersection());
        double paramOnLine = 1E300;
        Point2D p2d = new();
        // OCCT: Param is left unset, and p2d at the origin of the plane, when the line meets nothing
        param = 0.0;
        if (intersection.IsDone)
        {
            int i = 1;
            while (i <= intersection.PointCount)
            {
                IntersectionPoint2D iP = intersection.Point(i);
                if (iP.ParamOnFirst() < paramOnLine)
                {
                    p2d = iP.Value();
                    paramOnLine = iP.ParamOnFirst();
                    param = iP.ParamOnSecond();
                } // if (iP.ParamOnFirst ...
                i++;
            } // while ( i <= ...
        } // if (Intersection.IsDone ...

        Point3D thePoint = adaptor3dSurface.Value(p2d.X, p2d.Y);
        return thePoint;
    } // ComputePoint

    //=================================================================================================

    private static void OrientChamfer(ref Edge chamfer, in Edge e, in Vertex v)
    {
        Orientation vOrient, orient = e.Orientation;
        Exploration.Vertices(e, out Vertex v1, out Vertex v2);
        if (v1.IsSame(v))
        {
            vOrient = v2.Orientation;
        }
        else
        {
            vOrient = v1.Orientation;
        }

        if ((orient == Forward && vOrient == Forward)
            || (orient == Reversed && vOrient == Reversed))
        {
            chamfer.Orient(Forward);
        }
        else
        {
            chamfer.Orient(Reversed);
        }
    } // OrientChamfer
}
