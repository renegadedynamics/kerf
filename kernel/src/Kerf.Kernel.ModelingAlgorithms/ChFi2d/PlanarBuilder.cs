// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_Builder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_Builder.lxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Constraints;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Engine.Fillets.Planar.ConstructionError;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets.Planar;

/// <summary>Builds fillets and chamfers on the wire of a planar face.</summary>
/// <remarks>
/// <para>A working object (decision 21), as OCCT's. The face it is given is never written: each fillet or chamfer
/// builds a new face holding the face's untouched edges. What it hands out (<see cref="Result"/>, the fillets and
/// chamfers, the descendant edges, the vertices <see cref="RemoveFillet"/> returns) is frozen (decision 853); later
/// additions build new faces around them.</para>
/// <para>The history maps the face's edges to their trimmed descendants. Its iteration order isn't OCCT's (a hash order
/// there too); <see cref="BasisEdge"/> finds the one key whose value is the edge, so the order doesn't change results.</para>
/// </remarks>
public sealed partial class PlanarBuilder
{
    private ConstructionError _status;
    private Face _refFace = new();
    private Face _newFace = new();
    private readonly NCollection_Sequence<Shape> _fillets = new();
    private readonly NCollection_Sequence<Shape> _chamfers = new();
    private readonly Dictionary<Shape, Shape> _history = new(ShapeHasher.Instance);

    //=================================================================================================

    public PlanarBuilder()
    {
        _status = ConstructionError.NotPlanar;
    }

    //=================================================================================================

    /// <summary>The face <paramref name="f"/> can be built on a closed or an open wire.</summary>
    /// <remarks>
    /// OCCT builds the 3D curves of the face's edges that have none, in place (BRepLib::BuildCurves3d). On a frozen face
    /// the port builds them in a working overlay (unit 10H5), whose result then stands for the face (decision 854): the
    /// face itself when every edge had its curve.
    /// </remarks>
    public PlanarBuilder(in Face f)
    {
        if (f.IsNull)
        {
            _status = ConstructionError.NoFace;
            return;
        }
        //  syntaxe invalide sur NT
        //  const occ::handle<Geom_Surface>&  surf = BRep_Tool::Surface( F, Loc);
        //  if (surf->IsKind(STANDARD_TYPE(Geom_Plane))) {
        if (Tool.Surface(f, out _)!.IsKind(typeof(PlaneSurface)))
        {
            _newFace = _refFace = f;
            _newFace.Orientation(Forward);
            using (WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(f))
            {
                BrepLibrary.BuildCurves3d(_newFace);
                if (overlay is not null)
                {
                    _refFace = ShapeCasts.Face(overlay.Finish(f));
                    _newFace = _refFace;
                    _newFace.Orientation(Forward);
                }
            }
            _status = ConstructionError.Ready;
        }
        else
        {
            _status = ConstructionError.NotPlanar;
        }
    } // ChFi2d_Builder

    //=================================================================================================

    public void Init(in Face f)
    {
        if (f.IsNull)
        {
            _status = ConstructionError.NoFace;
            return;
        }
        _fillets.Clear();
        _chamfers.Clear();
        _history.Clear();
        //  syntaxe invalide sur NT
        //  const occ::handle<Geom_Surface>&  surf = BRep_Tool::Surface( F, Loc);
        //  if (surf->IsKind(STANDARD_TYPE(Geom_Plane))) {
        if (Tool.Surface(f, out _)!.IsKind(typeof(PlaneSurface)))
        {
            _newFace = _refFace = f;
            _newFace.Orientation(Forward);
            _status = ConstructionError.Ready;
        }
        else
        {
            _status = ConstructionError.NotPlanar;
        }
    } // Init

    //=================================================================================================

    /// <summary>
    /// Initialises the builder with a face <paramref name="refFace"/> and a face <paramref name="modFace"/> derived from
    /// it, to modify the fillets and chamfers already built on <paramref name="modFace"/>.
    /// </summary>
    public void Init(in Face refFace, in Face modFace)
    {
        if (refFace.IsNull || modFace.IsNull)
        {
            _status = ConstructionError.NoFace;
            return;
        }
        _fillets.Clear();
        _chamfers.Clear();
        _history.Clear();
        Placement loc;
        //  syntaxe invalide sur NT
        //  const occ::handle<Geom_Surface>&  surf = BRep_Tool::Surface( RefFace, Loc);
        //  if (!surf->IsKind(STANDARD_TYPE(Geom_Plane))) {
        if (!Tool.Surface(refFace, out loc)!.IsKind(typeof(PlaneSurface)))
        {
            _status = ConstructionError.NotPlanar;
            return;
        }

        _refFace = refFace;
        _newFace = modFace;
        _newFace.Orientation(Forward);
        _status = ConstructionError.Ready;

        // Research in newFace all the edges which not appear in RefFace
        // The sequence newEdges will contains this edges.

        NCollection_Sequence<Shape> newEdges = new();
        IndexedSet<Shape> refEdgesMap = new(ShapeHasher.Instance);
        Exploration.MapShapes(_refFace, ShapeType.Edge, refEdgesMap);
        var ex = new Explorer(_newFace, ShapeType.Edge);
        while (ex.More())
        {
            Edge currentEdge = ShapeCasts.Edge(ex.Current());
            if (!refEdgesMap.Contains(currentEdge))
            {
                newEdges.Append(currentEdge);
            }
            ex.Next();
        } // while (ex ...

        // update of history, fillets and chamfers fields
        int i = 1;
        double first, last;
        Edge basisEdge = new();
        while (i <= newEdges.Length())
        {
            Edge currentEdge = ShapeCasts.Edge(newEdges.Value(i));
            if (IsIssuedFrom(currentEdge, refEdgesMap, ref basisEdge))
            {
                _history[basisEdge] = currentEdge;
            }
            else
            {
                // this edge is a chamfer or a fillet
                //  syntaxe invalide sur NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(currentEdge, loc, first, last);
                Curve3D? curve = Tool.Curve(currentEdge, out loc, out first, out last);
                if (curve!.IsKind(typeof(CircleCurve3D)))
                {
                    _fillets.Append(currentEdge);
                }
                else if (curve.IsKind(typeof(LineCurve3D)))
                {
                    _chamfers.Append(currentEdge);
                }
                else
                {
                    _status = ConstructionError.InitialisationError;
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
    private static bool IsIssuedFrom(in Edge e, IndexedSet<Shape> map, ref Edge basisEdge)
    {
        double f1, l1, f2, l2;
        //  syntaxe invalide sur NT
        //  const occ::handle<Geom_Curve>& c1 =
        //    BRep_Tool::Curve(E, loc1, f1, L1);
        Curve3D? c1 = Tool.Curve(e, out _, out f1, out l1);

        for (int i = 0; i < map.Count; i++)
        {
            Edge currentEdge = ShapeCasts.Edge(map[i]);
            //  syntaxe invalide sur NT
            //    const occ::handle<Geom_Curve>& c2 =
            //      BRep_Tool::Curve(currentEdge, loc2, f2, L2);
            Curve3D? c2 = Tool.Curve(currentEdge, out _, out f2, out l2);
            if (c1 == c2
                && (((f1 > f2 && f1 < l2) || (l1 > f2 && l1 < l2))
                    || ((f1 > l2 && f1 < f2) || (l1 > l2 && l1 < f2))))
            {
                basisEdge = currentEdge;
                basisEdge.Orientation(e.Orientation());
                return true;
            } //  if (c1 == c2
        } // for (int i ...

        return false;
    } // IsIssuedFrom

    //=================================================================================================

    /// <summary>
    /// Adds a fillet of radius <paramref name="radius"/> between the two edges connected to the vertex
    /// <paramref name="v"/>, and returns it (frozen). The edge has sense only if <see cref="Status"/> is
    /// <see cref="IsDone"/>.
    /// </summary>
    public Edge AddFillet(in Vertex v, double radius)
    {
        Edge fillet = InternalAddFillet(v, radius);
        return Published(fillet);
    }

    private Edge InternalAddFillet(in Vertex v, double radius)
    {
        Edge adjEdge1 = new(), adjEdge2 = new(), basisEdge1, basisEdge2;
        Edge adjEdge1Mod = new(), adjEdge2Mod = new(), fillet = new();
        _status = global::Kerf.Engine.Fillets.Planar.ChFi2d.FindConnectedEdges(_newFace, v, ref adjEdge1, ref adjEdge2);
        if (_status == ConstructionError.ConnexionError)
        {
            return fillet;
        }

        if (IsAFillet(adjEdge1) || IsAChamfer(adjEdge1) || IsAFillet(adjEdge2) || IsAChamfer(adjEdge2))
        {
            _status = ConstructionError.NotAuthorized;
            return fillet;
        } //  if (IsAFillet ...

        if (!IsLineOrCircle(adjEdge1, _newFace) || !IsLineOrCircle(adjEdge2, _newFace))
        {
            _status = ConstructionError.NotAuthorized;
            return fillet;
        } //  if (!IsLineOrCircle ...

        ComputeFillet(v, adjEdge1, adjEdge2, radius, ref adjEdge1Mod, ref adjEdge2Mod, ref fillet);
        if (_status == ConstructionError.IsDone || _status == ConstructionError.FirstEdgeDegenerated
            || _status == ConstructionError.LastEdgeDegenerated || _status == ConstructionError.BothEdgesDegenerated)
        {
            BuildNewWire(adjEdge1, adjEdge2, adjEdge1Mod, fillet, adjEdge2Mod);
            basisEdge1 = BasisEdge(adjEdge1);
            basisEdge2 = BasisEdge(adjEdge2);
            UpDateHistory(basisEdge1, basisEdge2, adjEdge1Mod, adjEdge2Mod, fillet, 1);
            _status = ConstructionError.IsDone;
            return ShapeCasts.Edge(_fillets.Value(_fillets.Length()));
        }
        return fillet;
    } // AddFillet

    //=================================================================================================

    /// <summary>
    /// Modifies the radius of the fillet <paramref name="fillet"/> and returns the new fillet (frozen). The edge has
    /// sense only if <see cref="Status"/> is <see cref="IsDone"/>.
    /// </summary>
    public Edge ModifyFillet(in Edge fillet, double radius)
    {
        Vertex vertex = InternalRemoveFillet(fillet);
        Edge aFillet = InternalAddFillet(vertex, radius);
        return Published(aFillet);
    } // ModifyFillet

    //=================================================================================================

    /// <summary>
    /// Removes the fillet <paramref name="fillet"/> and returns the vertex connecting the two edges adjacent to it
    /// (frozen; null if <paramref name="fillet"/> is not a fillet of this builder).
    /// </summary>
    public Vertex RemoveFillet(in Edge fillet)
    {
        Vertex vertex = InternalRemoveFillet(fillet);
        vertex.TShape()?.FreezeGraph();
        return vertex;
    }

    private Vertex InternalRemoveFillet(in Edge fillet)
    {
        Vertex commonVertex = new();
        int i = 1;
        bool isFind = false;
        while (i <= _fillets.Length())
        {
            Edge aFillet = ShapeCasts.Edge(_fillets.Value(i));
            if (aFillet.IsSame(fillet))
            {
                _fillets.Remove(i);
                isFind = true;
                break;
            }
            i++;
        }
        if (!isFind)
        {
            return commonVertex;
        }

        Exploration.Vertices(fillet, out Vertex firstVertex, out Vertex lastVertex);

        Edge adjEdge1 = new(), adjEdge2 = new();
        _status = global::Kerf.Engine.Fillets.Planar.ChFi2d.FindConnectedEdges(_newFace, firstVertex, ref adjEdge1, ref adjEdge2);
        if (_status == ConstructionError.ConnexionError)
        {
            return commonVertex;
        }

        Edge basisEdge1, basisEdge2, e1, e2;
        // E1 and E2 are the adjacent edges to Fillet

        if (adjEdge1.IsSame(fillet))
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
        if (adjEdge1.IsSame(fillet))
        {
            e2 = adjEdge2;
        }
        else
        {
            e2 = adjEdge1;
        }
        basisEdge2 = BasisEdge(e2);
        Vertex connectionE1Fillet = new(), connectionE2Fillet = new();
        bool hasConnection = global::Kerf.Engine.Fillets.Planar.ChFi2d.CommonVertex(basisEdge1, basisEdge2, ref commonVertex);
        if (!hasConnection)
        {
            _status = ConstructionError.ConnexionError;
            return commonVertex;
        }
        hasConnection = global::Kerf.Engine.Fillets.Planar.ChFi2d.CommonVertex(e1, fillet, ref connectionE1Fillet);
        if (!hasConnection)
        {
            _status = ConstructionError.ConnexionError;
            return commonVertex;
        }
        hasConnection = global::Kerf.Engine.Fillets.Planar.ChFi2d.CommonVertex(e2, fillet, ref connectionE2Fillet);
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
            if (firstVertex.IsSame(connectionE1Fillet))
            {
                //  syntaxe invalide sur NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(E1, loc, first, last);
                Curve3D? curve = Tool.Curve(e1, out loc, out first, out last);
                makeEdge.Init(curve!, commonVertex, lastVertex);
                newEdge1 = makeEdge.InternalEdge();
                newEdge1.Orientation(e1.Orientation());
                newEdge1.Location(e1.Location());
            } // if (firstVertex ...
            else if (lastVertex.IsSame(connectionE1Fillet))
            {
                //  syntax wrong on NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(E1, loc, first, last);
                Curve3D? curve = Tool.Curve(e1, out loc, out first, out last);
                makeEdge.Init(curve!, firstVertex, commonVertex);
                newEdge1 = makeEdge.InternalEdge();
                newEdge1.Orientation(e1.Orientation());
                newEdge1.Location(e1.Location());
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
            if (firstVertex.IsSame(connectionE2Fillet))
            {
                //  syntax wrong on NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(E2, loc, first, last);
                Curve3D? curve = Tool.Curve(e2, out loc, out first, out last);
                makeEdge.Init(curve!, commonVertex, lastVertex);
                newEdge2 = makeEdge.InternalEdge();
                newEdge2.Orientation(e2.Orientation());
                newEdge2.Location(e2.Location());
            } // if (firstVertex ...
            else if (lastVertex.IsSame(connectionE2Fillet))
            {
                //  syntax wrong on NT
                //      const occ::handle<Geom_Curve>& curve =
                //	BRep_Tool::Curve(E2, loc, first, last);
                Curve3D? curve = Tool.Curve(e2, out loc, out first, out last);
                makeEdge.Init(curve!, firstVertex, commonVertex);
                newEdge2 = makeEdge.InternalEdge();
                newEdge2.Orientation(e2.Orientation());
                newEdge2.Location(e2.Location());
            } // else if (lastVertex ...
        } // else ...

        // rebuild the newFace, wire by wire (decision 1474)
        Edge e1Copy = e1, e2Copy = e2, filletCopy = fillet, newEdge1Copy = newEdge1, newEdge2Copy = newEdge2;
        RebuildFace(null, theEdge => !theEdge.IsSame(e1Copy) && !theEdge.IsSame(e2Copy) && !theEdge.IsSame(filletCopy) ? null
                               : theEdge.IsEqual(e1Copy) ? [newEdge1Copy]
                               : theEdge.IsEqual(e2Copy) ? [newEdge2Copy]
                               : []);

        UpDateHistory(basisEdge1, basisEdge2, newEdge1, newEdge2);

        return commonVertex;
    } // RemoveFillet

    //=================================================================================================

    private void ComputeFillet(in Vertex v,
                               in Edge e1,
                               in Edge e2,
                               double radius,
                               ref Edge trimE1,
                               ref Edge trimE2,
                               ref Edge fillet)
    {
        Vertex newExtr1 = new(), newExtr2 = new();
        bool degen1, degen2;
        fillet = BuildFilletEdge(v, e1, e2, radius, ref newExtr1, ref newExtr2);
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
    } // ComputeFillet

    //=================================================================================================

    // Replaces in newFace OldE1 and OldE2 by E1, Fillet and E2, or by Fillet and E2 if E1 is degenerated, or by E1
    // and Fillet if E2 is degenerated, or by Fillet if both are.
    private void BuildNewWire(in Edge oldE1,
                              in Edge oldE2,
                              in Edge e1,
                              in Edge fillet,
                              in Edge e2)
    {
        // the closed flag is the reference face's first wire's, as OCCT set it
        bool closedStatus = true;
        var exW = new Explorer(_refFace, ShapeType.Wire);
        if (exW.More())
        {
            closedStatus = ShapeCasts.Wire(exW.Current()).Closed();
        }

        bool filletIsAdded = false;
        Edge e1Copy = e1, e2Copy = e2, filletCopy = fillet, oldE1Copy = oldE1, oldE2Copy = oldE2;
        RebuildFace(closedStatus, theEdge =>
        {
            if (!theEdge.IsSame(oldE1Copy) && !theEdge.IsSame(oldE2Copy))
            {
                return null;
            }
            var anEdges = new List<Edge>();
            if (theEdge.IsEqual(oldE1Copy))
            {
                if (_status != ConstructionError.FirstEdgeDegenerated && _status != ConstructionError.BothEdgesDegenerated)
                {
                    anEdges.Add(e1Copy);
                }
            }
            else if (_status != ConstructionError.LastEdgeDegenerated && _status != ConstructionError.BothEdgesDegenerated)
            {
                anEdges.Add(e2Copy);
            }
            if (!filletIsAdded)
            {
                anEdges.Add(filletCopy);
                filletIsAdded = true;
            }
            return anEdges;
        });
    } // BuildNewWire

    // Rebuilds _newFace on the reference face's plane, wire by wire, the outer wire first: each edge for which
    // theReplace gives edges is replaced by them (null keeps it); each wire's closed flag is theClosed, or left unset
    // if it is null, as the removals left it (decision 1474; OCCT put every edge of the face into one wire, so a face
    // with holes came back with one wire holding the outer and inner edges).
    private void RebuildFace(bool? theClosed, Func<Edge, List<Edge>?> theReplace)
    {
        var b = new BrepBuilder();
        Wire anOuter = BrepTools.OuterWire(_newFace);
        var aWires = new List<Wire>();
        for (var exW = new Explorer(_newFace, ShapeType.Wire); exW.More(); exW.Next())
        {
            Wire aWire = ShapeCasts.Wire(exW.Current());
            if (aWire.IsSame(anOuter))
            {
                aWires.Insert(0, aWire);
            }
            else
            {
                aWires.Add(aWire);
            }
        }

        var adaptor3dSurface = new FaceSurfaceAdaptor(_refFace);
        LibraryMakeFace? mFace = null;
        foreach (Wire aWire in aWires)
        {
            Wire newWire = new();
            b.MakeWire(ref newWire);
            for (var ex = new Explorer(aWire, ShapeType.Edge); ex.More(); ex.Next())
            {
                Edge theEdge = ShapeCasts.Edge(ex.Current());
                List<Edge>? aNew = theReplace(theEdge);
                if (aNew is null)
                {
                    b.Add(newWire, theEdge);
                }
                else
                {
                    foreach (Edge anEdge in aNew)
                    {
                        b.Add(newWire, anEdge);
                    }
                }
            }
            if (theClosed is bool aClosed)
            {
                newWire.Closed(aClosed);
            }
            if (mFace is null)
            {
                mFace = new LibraryMakeFace(adaptor3dSurface.Plane(), newWire);
            }
            else
            {
                mFace.Add(newWire);
            }
        }
        _newFace = mFace!.InternalFace();
    }

    //=================================================================================================

    // Changes OldExtr of E1 by NewExtr.
    private Edge BuildNewEdge(in Edge e1, in Vertex oldExtr, in Vertex newExtr)
    {
        var makeEdge = new LibraryMakeEdge();
        double first, last;
        Exploration.Vertices(e1, out Vertex firstVertex, out Vertex lastVertex);
        //  syntaxe invalide sur NT
        //      const occ::handle<Geom_Curve>& curve =
        //	BRep_Tool::Curve(E1, first, last);
        Curve3D? curve = Tool.Curve(e1, out first, out last);
        if (firstVertex.IsSame(oldExtr))
        {
            makeEdge.Init(curve!, newExtr, lastVertex);
        }
        else
        {
            makeEdge.Init(curve!, firstVertex, newExtr);
        }
        Edge edge = makeEdge.InternalEdge();
        edge.Orientation(e1.Orientation());
        //  anEdge.Location(E1.Location());
        return edge;
    }

    //=======================================================================
    // function : BuildNewEdge
    // purpose  : special flag if the new edge is degenerated
    //=======================================================================

    // Changes OldExtr of E1 by NewExtr; returns E1 and IsDegenerated true if the new edge is degenerated.
    private Edge BuildNewEdge(in Edge e1, in Vertex oldExtr, in Vertex newExtr, out bool isDegenerated)
    {
        var makeEdge = new LibraryMakeEdge();
        double first, last;
        isDegenerated = false;
        Exploration.Vertices(e1, out Vertex firstVertex, out Vertex lastVertex);
        Point3D pnew = Tool.Pnt(newExtr);
        bool ponctualEdge = false;
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        //  syntax wrong on NT
        //      const occ::handle<Geom_Curve>& curve =
        //	BRep_Tool::Curve(E1, first, last);
        Curve3D? curve = Tool.Curve(e1, out first, out last);
        if (firstVertex.IsSame(oldExtr))
        {
            makeEdge.Init(curve!, newExtr, lastVertex);
            Point3D pv = Tool.Pnt(lastVertex);
            ponctualEdge = (pnew.Distance(pv) < tol);
        }
        else
        {
            makeEdge.Init(curve!, firstVertex, newExtr);
            Point3D pv = Tool.Pnt(firstVertex);
            ponctualEdge = (pnew.Distance(pv) < tol);
        }
        Edge edge;
        LibraryEdgeError error = makeEdge.Error();
        if (error == LibraryEdgeError.LineThroughIdenticPoints || ponctualEdge)
        {
            isDegenerated = true;
            edge = e1;
        }
        else
        {
            edge = makeEdge.InternalEdge();
            edge.Orientation(e1.Orientation());
            //    anEdge.Location(E1.Location());
        }
        return edge;
    }

    //=================================================================================================

    // Writes NewEdge in fillets if Id is 1, or in chamfers if Id is 2, and the modifications in history: TrimE1 is given
    // by E1, TrimE2 by E2, if they are not degenerated.
    private void UpDateHistory(in Edge e1,
                               in Edge e2,
                               in Edge trimE1,
                               in Edge trimE2,
                               in Edge newEdge,
                               int id)
    {
        if (id == 1) // the new edge is a fillet
        {
            _fillets.Append(newEdge);
        }
        else // the new edge is a chamfer
        {
            _chamfers.Append(newEdge);
        }

        _history.Remove(e1);
        if (_status != ConstructionError.FirstEdgeDegenerated && _status != ConstructionError.BothEdgesDegenerated)
        {
            if (!e1.IsSame(trimE1))
            {
                _history[e1] = trimE1;
            }
        }
        _history.Remove(e2);
        if (_status != ConstructionError.LastEdgeDegenerated && _status != ConstructionError.BothEdgesDegenerated)
        {
            if (!e2.IsSame(trimE2))
            {
                _history[e2] = trimE2;
            }
        }
    } // UpDateHistory

    //=================================================================================================

    // Writes the modifications in history: TrimE1 is given by E1, TrimE2 by E2.
    private void UpDateHistory(in Edge e1, in Edge e2, in Edge trimE1, in Edge trimE2)
    {
        if (_history.ContainsKey(e1))
        {
            _history.Remove(e1);
        }
        if (!e1.IsSame(trimE1))
        {
            _history[e1] = trimE1;
        }
        if (_history.ContainsKey(e2))
        {
            _history.Remove(e2);
        }
        if (!e2.IsSame(trimE2))
        {
            _history[e2] = trimE2;
        }
    } // UpDateHistory

    //=================================================================================================

    /// <summary>
    /// The parent edge of <paramref name="e"/>: the edge of the initial face it was trimmed from, or
    /// <paramref name="e"/> itself if it is a basis edge (or not on the face).
    /// </summary>
    public Edge BasisEdge(in Edge e)
    {
        Edge edge;
        foreach (KeyValuePair<Shape, Shape> iterator in _history)
        {
            edge = ShapeCasts.Edge(iterator.Value);
            if (edge.IsSame(e))
            {
                Edge anotherEdge = ShapeCasts.Edge(iterator.Key);
                return anotherEdge;
            } // if (anEdge.IsSame ...
        } // while (Iterator.More ...
        return e;
    } // BasisEdge

    //=================================================================================================

    // Builds the fillet edge between AdjEdge1 and AdjEdge2 at V; NewExtr1 and NewExtr2 receive the new extremities of
    // the two edges. The edge has sense only if the status is IsDone (or one of the degenerated statuses).
    private Edge BuildFilletEdge(in Vertex v,
                                        in Edge adjEdge1,
                                        in Edge adjEdge2,
                                        double radius,
                                        ref Vertex newExtr1,
                                        ref Vertex newExtr2)
    {
        Edge e1, e2;
        e1 = adjEdge1;
        e2 = adjEdge2;
        Vertex v1 = Exploration.FirstVertex(e1);
        Vertex v2 = Exploration.LastVertex(e1);
        Vertex v3 = Exploration.FirstVertex(e2);
        Vertex v4 = Exploration.LastVertex(e2);

        //========================================================================
        //    The first arc is found.                                        +
        //========================================================================

        Orientation o1;
        Orientation oe1;
        oe1 = e1.Orientation();
        e1.Orientation(Forward);
        e2.Orientation(Forward);
        Shape localShape = e1.EmptyCopied();
        Edge ebid1 = ShapeCasts.Edge(localShape);
        localShape = e2.EmptyCopied();
        Edge ebid2 = ShapeCasts.Edge(localShape);
        //  TopoDS_Edge Ebid1 = TopoDS::Edge(E1.EmptyCopied());
        //  TopoDS_Edge Ebid2 = TopoDS::Edge(E2.EmptyCopied());
        _ = ebid1; // unused, as in OCCT
        _ = ebid2;
        double param1, param2, param3, param4;

        //========================================================================
        //    Save non-modified parts of edges concerned.      +
        //========================================================================

        if (v1.IsSame(v))
        {
            param1 = Tool.Parameter(v1, e1);
            param2 = Tool.Parameter(v2, e1);
            o1 = v2.Orientation();
        }
        else
        {
            param1 = Tool.Parameter(v2, e1);
            param2 = Tool.Parameter(v1, e1);
            o1 = v1.Orientation();
        }
        if (v3.IsSame(v))
        {
            param3 = Tool.Parameter(v3, e2);
            param4 = Tool.Parameter(v4, e2);
        }
        else
        {
            param3 = Tool.Parameter(v4, e2);
            param4 = Tool.Parameter(v3, e2);
        }

        //========================================================================
        //    Restore geometric supports.                            +
        //========================================================================

        Curve2D? c1, c2;
        double ufirst1, ulast1, ufirst2, ulast2, u1, u2, pu1, pu2, vv1, vv2;
        double ppu1, ppu2;
        c1 = Tool.CurveOnSurface(e1, _newFace, out ufirst1, out ulast1);
        c2 = Tool.CurveOnSurface(e2, _newFace, out ufirst2, out ulast2);

        //========================================================================
        //   Determination of the face for fillet.                                +
        //========================================================================

        Point2D p;
        Vector2D ve1, ve2;
        Vector2D ve3, ve4;
        bool sens1, sens2;

        Curve2D basisC1, basisC2;
        if (c1 is TrimmedCurve2D t1)
        {
            basisC1 = t1.BasisCurve();
        }
        else
        {
            basisC1 = c1!;
        }
        if (c2 is TrimmedCurve2D t2)
        {
            basisC2 = t2.BasisCurve();
        }
        else
        {
            basisC2 = c2!;
        }

        if (basisC1.DynamicType() == typeof(CircleCurve2D))
        {
            var cc1 = (CircleCurve2D)basisC1;
            global::Kerf.Engine.Geometry.ElCLib.D1(param1, cc1.Circ2d(), out p, out ve1);
            sens1 = (cc1.Circ2d()).IsDirect;
        } // if (C1->DynamicType() ...
        else
        {
            var cc1 = (basisC1 as LineCurve2D)!; // OCCT dereferences down_cast's result unchecked
            global::Kerf.Engine.Geometry.ElCLib.D1(param1, cc1.Lin2d(), out p, out ve1);
            sens1 = true;
        } // else ...
        if (basisC2.DynamicType() == typeof(CircleCurve2D))
        {
            var cc2 = (CircleCurve2D)basisC2;
            global::Kerf.Engine.Geometry.ElCLib.D1(param3, cc2.Circ2d(), out p, out ve2);
            sens2 = (cc2.Circ2d()).IsDirect;
        } // if (C2->DynamicType() ...
        else
        {
            var cc2 = (basisC2 as LineCurve2D)!;
            global::Kerf.Engine.Geometry.ElCLib.D1(param3, cc2.Lin2d(), out p, out ve2);
            sens2 = true;
        } // else ...

        Edge filletEdge = new();

        double cross = ve1.Crossed(ve2);
        ve3 = ve1;
        ve4 = ve2;

        // processing of tangency or downcast point
        if (ve1.IsParallel(ve2, global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            // Ve1 and Ve2 are parallel : cross at 0
            cross = 0.0;
            if (param1 < param2)
            {
                ve3 = -ve1;
            }
            if (param3 > param4)
            {
                ve4 = -ve2;
            }

            if (!ve4.IsOpposite(ve3, global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                // There is a true tangency point and the calculation is stopped
                _status = ConstructionError.TangencyError;
                return filletEdge;
            }
            // Otherwise this is a downcast point, and the calculation is continued
        }

        Qualifier qual1, qual2;
        if (cross < 0.0)
        {
            if (param3 > param4)
            {
                if (sens1)
                {
                    qual1 = Qualifier.Enclosed;
                }
                else
                {
                    qual1 = Qualifier.Outside;
                }
            }
            else
            {
                if (sens1)
                {
                    qual1 = Qualifier.Outside;
                }
                else
                {
                    qual1 = Qualifier.Enclosed;
                }
            }
            if (param1 > param2)
            {
                if (sens2)
                {
                    qual2 = Qualifier.Outside;
                }
                else
                {
                    qual2 = Qualifier.Enclosed;
                }
            }
            else
            {
                if (sens2)
                {
                    qual2 = Qualifier.Enclosed;
                }
                else
                {
                    qual2 = Qualifier.Outside;
                }
            }
        } // if (cross < 0 ...
        else
        {
            if (param3 > param4)
            {
                if (sens1)
                {
                    qual1 = Qualifier.Outside;
                }
                else
                {
                    qual1 = Qualifier.Enclosed;
                }
            }
            else
            {
                if (sens1)
                {
                    qual1 = Qualifier.Enclosed;
                }
                else
                {
                    qual1 = Qualifier.Outside;
                }
            }
            if (param1 > param2)
            {
                if (sens2)
                {
                    qual2 = Qualifier.Enclosed;
                }
                else
                {
                    qual2 = Qualifier.Outside;
                }
            }
            else
            {
                if (sens2)
                {
                    qual2 = Qualifier.Outside;
                }
                else
                {
                    qual2 = Qualifier.Enclosed;
                }
            }
        } // else ...

        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        // C++ converts the handles to adaptors implicitly (decision 805)
        var fillet = new Circ2d2TanRad(new QualifiedCurve(new CurveAdaptor2D(basisC1), qual1),
                                                 new QualifiedCurve(new CurveAdaptor2D(basisC2), qual2),
                                                 radius,
                                                 tol);
        if (!fillet.IsDone || fillet.SolutionCount == 0)
        {
            _status = ConstructionError.ComputationError;
            return filletEdge;
        }
        else if (fillet.SolutionCount >= 1)
        {
            _status = ConstructionError.IsDone;
            int numsol = 1;
            int nsol = 1;
            Vertex vertex1 = new(), vertex2 = new();
            Point2D ptg1, ptg2;
            double dist;
            double dist1 = 1.0e40;
            bool inside = false;
            while (nsol <= fillet.SolutionCount)
            {
                fillet.Tangency1(nsol, out pu1, out pu2, out ptg1);
                dist = ptg1.Distance(p);
                if (basisC1.DynamicType() == typeof(LineCurve2D))
                {
                    inside = (pu2 < param1 && pu2 > param2) || (pu2 < param2 && pu2 > param1);
                    if (inside && dist < dist1)
                    {
                        numsol = nsol;
                        dist1 = dist;
                    } // if ((((inside && ...
                } // if (C1->DynamicType( ...
                else
                {
                    fillet.Tangency2(nsol, out ppu1, out ppu2, out ptg2);
                    dist = ptg2.Distance(p);
                    inside = (ppu2 < param3 && ppu2 > param4) || (ppu2 < param4 && ppu2 > param3);
                    //  case of arc of circle passing on the sewing
                    if ((basisC2.DynamicType() == typeof(CircleCurve2D))
                        && ((2 * M_PI < param3 && 2 * M_PI > param4)
                            || (2 * M_PI < param4 && 2 * M_PI > param3)))
                    {
                        //  cas param3<param4
                        inside = (param3 < ppu2 && ppu2 < 2 * M_PI) || (0 <= ppu2 && ppu2 < param4 - 2 * M_PI);
                        //  cas param4<param3
                        inside =
                            inside || (param4 < ppu2 && ppu2 < 2 * M_PI) || (0 <= ppu2 && ppu2 < param3 - 2 * M_PI);
                    }
                    if (inside && dist < dist1)
                    {
                        numsol = nsol;
                        dist1 = dist;
                    } // if ((((param3 ...
                } // else ...
                nsol++;
            } // while (nsol ...
            var cir = fillet.ThisSolution(numsol);
            var circle = new CircleCurve2D(cir);

            var b = new BrepBuilder();
            var adaptor3dSurface = new FaceSurfaceAdaptor(_refFace);
            var refSurf = new PlaneSurface(adaptor3dSurface.Plane());
            fillet.Tangency1(numsol, out u1, out u2, out ptg1);
            fillet.Tangency2(numsol, out vv1, out vv2, out ptg2);

            // check the validity of parameters
            //// modified by jgv, 08.08.2011 for bug 0022695 ////
            // inside = (U2<param1 && U2>param2) || (U2<param2 && U2>param1);
            inside = (u2 < param1 && u2 >= param2) || (u2 <= param2 && u2 > param1);
            /////////////////////////////////////////////////////
            if ((basisC1.DynamicType() == typeof(CircleCurve2D))
                && ((2 * M_PI < param1 && 2 * M_PI > param2) || (2 * M_PI < param2 && 2 * M_PI > param1)))
            {
                // arc of circle containing the circle origin
                //  case param1<param2
                inside = (param1 < u2 && u2 < 2 * M_PI) || (0 <= u2 && u2 < param2 - 2 * M_PI);
                //  case param2<param1
                inside = inside || (param2 < u2 && u2 < 2 * M_PI) || (0 <= u2 && u2 < param1 - 2 * M_PI);
            }
            if (!inside)
            {
                _status = ConstructionError.ComputationError;
                return filletEdge;
            }

            //// modified by jgv, 08.08.2011 for bug 0022695 ////
            // inside = (Vv2<param3 && Vv2>param4) || (Vv2<param4 && Vv2>param3);
            inside = (vv2 < param3 && vv2 >= param4) || (vv2 <= param4 && vv2 > param3);
            /////////////////////////////////////////////////////
            if ((basisC2.DynamicType() == typeof(CircleCurve2D))
                && ((2 * M_PI < param3 && 2 * M_PI > param4) || (2 * M_PI < param4 && 2 * M_PI > param3)))
            {
                // arc of circle containing the circle origin
                //  cas param3<param4
                inside = (param3 < vv2 && vv2 < 2 * M_PI) || (0 <= vv2 && vv2 < param4 - 2 * M_PI);
                //  cas param4<param3
                inside = inside || (param4 < vv2 && vv2 < 2 * M_PI) || (0 <= vv2 && vv2 < param3 - 2 * M_PI);
            }
            if (!inside)
            {
                _status = ConstructionError.ComputationError;
                return filletEdge;
            }

            Point3D p1 = adaptor3dSurface.Value(ptg1.X(), ptg1.Y());
            Point3D p2 = adaptor3dSurface.Value(ptg2.X(), ptg2.Y());
            b.MakeVertex(ref vertex1, p1, tol);
            newExtr1 = vertex1;
            if (Math.Abs(u2 - ufirst1) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                newExtr1 = v1;
            }
            if (Math.Abs(u2 - ulast1) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                newExtr1 = v2;
            }

            b.MakeVertex(ref vertex2, p2, tol);
            newExtr2 = vertex2;
            if (Math.Abs(vv2 - ufirst2) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                newExtr2 = v3;
            }
            if (Math.Abs(vv2 - ulast2) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                newExtr2 = v4;
            }

            //=======================================================================
            //   Update tops of the fillet.                                  +
            //=======================================================================
            Point3D pntbid;
            Point2D sommet;
            pntbid = Tool.Pnt(v);
            sommet = new Point2D(pntbid.X(), pntbid.Y());

            Point3D pntBid;
            Point2D somBid;
            if (v1.IsSame(v))
            {
                pntBid = Tool.Pnt(v2);
                somBid = new Point2D(pntBid.X(), pntBid.Y());
            }
            else
            {
                pntBid = Tool.Pnt(v1);
                somBid = new Point2D(pntBid.X(), pntBid.Y());
            }

            Vector2D vec;
            global::Kerf.Engine.Geometry.ElCLib.D1(u1, cir, out ptg1, out vec);

            Vector2D vec1 = new();
            if (basisC1.DynamicType() == typeof(CircleCurve2D))
            {
                var cc1 = (CircleCurve2D)basisC1;
                Circle2D cir2d = cc1.Circ2d();
                double par = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir2d, ptg1);
                global::Kerf.Engine.Geometry.ElCLib.D1(par, cir2d, out _, out vec1);
            } // if (C1->DynamicType() ...
            else if (basisC1.DynamicType() == typeof(LineCurve2D))
            {
                var cc1 = (LineCurve2D)basisC1;
                Line2D lin2d = cc1.Lin2d();
                double par = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2d, sommet);
                vec1 = new Vector2D(sommet.X() - somBid.X(), sommet.Y() - somBid.Y()); // OCCT: overwritten at once
                global::Kerf.Engine.Geometry.ElCLib.D1(par, lin2d, out _, out vec1);
            } // else if ...

            if (oe1 == Reversed)
            {
                vec1 = vec1.Reversed();
            } // if (OE1 ...
            bool sense = (vec1 * vec) > 0.0;
            if (u1 > vv1 && u1 > 2.0 * M_PI)
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, global::Kerf.Engine.Foundation.Precision.Confusion(), ref u1, ref vv1);
            } // if (U1 ...
            if ((o1 == Forward && oe1 == Forward)
                || (o1 == Reversed && oe1 == Reversed))
            {
                filletEdge = new LibraryMakeEdge(circle, refSurf, newExtr1, newExtr2, u1, vv1).InternalEdge();
            } // if (O1 == ...
            else
            {
                filletEdge = new LibraryMakeEdge(circle, refSurf, newExtr2, newExtr1, vv1, u1).InternalEdge();
            } // else ...
            if (!sense)
            {
                Orientation s1 = filletEdge.Orientation();
                if ((o1 == Forward && oe1 == Forward)
                    || (o1 == Reversed && oe1 == Reversed))
                {
                    filletEdge = new LibraryMakeEdge(circle, refSurf, newExtr2, newExtr1, vv1, u1).InternalEdge();
                }
                else
                {
                    filletEdge = new LibraryMakeEdge(circle, refSurf, newExtr1, newExtr2, u1, vv1).InternalEdge();
                }
                if (s1 == Forward)
                {
                    filletEdge.Orientation(Reversed);
                }
                else
                {
                    filletEdge.Orientation(Forward);
                }
            } // if (!Sense

        } //  else if

        BrepLibrary.BuildCurves3d(filletEdge);
        return filletEdge;
    } // BuildFilletEdge

    //=================================================================================================

    private bool IsAFillet(in Edge e)
    {
        int i = 1;
        while (i <= _fillets.Length())
        {
            Edge currentEdge = ShapeCasts.Edge(_fillets.Value(i));
            if (currentEdge.IsSame(e))
            {
                return true;
            }
            i++;
        }
        return false;
    } // IsAFillet

    //=================================================================================================

    private bool IsAChamfer(in Edge e)
    {
        int i = 1;
        while (i <= _chamfers.Length())
        {
            Edge currentEdge = ShapeCasts.Edge(_chamfers.Value(i));
            if (currentEdge.IsSame(e))
            {
                return true;
            }
            i++;
        }
        return false;
    } // IsAChamfer

    //=================================================================================================

    // OCCT has this function twice, as a file-static in ChFi2d_Builder.cxx and in ChFi2d_Builder_0.cxx, identical.
    private static bool IsLineOrCircle(in Edge e, in Face f)
    {
        double first, last;
        //  syntaxe invalide sur NT
        //      const occ::handle<Geom2d_Curve>& C =
        //	BRep_Tool::CurveOnSurface(E,F,first,last);
        Curve2D? c = Tool.CurveOnSurface(e, f, out first, out last);
        Curve2D basisC;
        if (c is TrimmedCurve2D tc)
        {
            basisC = tc.BasisCurve();
        }
        else
        {
            basisC = c!;
        }

        return basisC.DynamicType() == typeof(CircleCurve2D)
               || basisC.DynamicType() == typeof(LineCurve2D); // else ...
    } // IsLineOrCircle

    // ---- the inline accessors (ChFi2d_Builder.lxx) ----

    /// <summary>The modified face (frozen), with the initial face's orientation.</summary>
    public Face Result()
    {
        Face face = InternalResult();
        face.TShape()?.FreezeGraph();
        return face;
    }

    /// <summary><see cref="Result"/> without freezing, for ported code.</summary>
    internal Face InternalResult()
    {
        Face face = _newFace;
        face.Orientation(_refFace.Orientation());
        return face;
    }

    /// <summary>True if the edge <paramref name="e"/> of the initial face has been trimmed by a fillet or chamfer.</summary>
    public bool IsModified(in Edge e)
    {
        return _history.ContainsKey(e);
    }

    /// <summary>The fillets built (frozen with the face that holds them).</summary>
    public ISchematiqList<Shape> FilletEdges() => PublishedEdges(_fillets);

    /// <summary>The builder's own sequence behind <see cref="FilletEdges"/>, unfrozen.</summary>
    internal NCollection_Sequence<Shape> InternalFilletEdges() => _fillets;

    public int FilletCount
    {
        get
        {
            return _fillets.Length();
        }
    }

    /// <summary>The chamfers built (frozen with the face that holds them).</summary>
    public ISchematiqList<Shape> ChamferEdges() => PublishedEdges(_chamfers);

    /// <summary>The builder's own sequence behind <see cref="ChamferEdges"/>, unfrozen.</summary>
    internal NCollection_Sequence<Shape> InternalChamferEdges() => _chamfers;

    public int ChamferCount
    {
        get
        {
            return _chamfers.Length();
        }
    }

    public bool HasDescendant(in Edge e)
    {
        return _history.ContainsKey(e);
    }

    /// <summary>
    /// The edge trimmed from <paramref name="e"/> (frozen with the face that holds it).
    /// </summary>
    /// <exception cref="NoSuchObjectException"><paramref name="e"/> has no descendant.</exception>
    public Edge DescendantEdge(in Edge e)
    {
        FreezeResult();
        return InternalDescendantEdge(e);
    }

    /// <summary><see cref="DescendantEdge"/> without freezing, for ported code.</summary>
    internal Edge InternalDescendantEdge(in Edge e)
    {
        return ShapeCasts.Edge(_history.GetValue(e));
    } // DescendantEdge

    public ConstructionError Status()
    {
        return _status;
    }

    // ---- publication (decision 853) ----

    // Freezes the face being built, which holds every fillet, chamfer and descendant edge handed out.
    internal void FreezeResult()
    {
        _newFace.TShape()?.FreezeGraph();
    }

    private Edge Published(in Edge edge)
    {
        FreezeResult();
        edge.TShape()?.FreezeGraph();
        return edge;
    }

    private ISchematiqList<Shape> PublishedEdges(NCollection_Sequence<Shape> edges)
    {
        FreezeResult();
        if (edges.IsEmpty())
        {
            return PackedList<Shape>.Empty;
        }
        var anEdges = new Shape[edges.Length()];
        for (int i = 1; i <= edges.Length(); i++)
        {
            anEdges[i - 1] = edges.Value(i);
            anEdges[i - 1].TShape()?.FreezeGraph();
        }
        return PackedList.Own(anEdges);
    }
}
