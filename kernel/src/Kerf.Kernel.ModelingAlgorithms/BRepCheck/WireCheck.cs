// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Wire.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Wire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Checks.StatusCheck;
using static Kerf.Geometry.CurveKind;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Checks;

/// <summary>
/// The checks of a wire: not empty and connected (<see cref="Minimum"/>), closed (<see cref="Closed"/>), closed in
/// the parametric space of a face (<see cref="Closed2d"/>), coherently oriented (<see cref="Orientation"/>) and
/// without self-intersection on a face (<see cref="SelfIntersect"/>).
/// </summary>
public class WireCheck : ResultCheck
{
    private bool _cdone;
    private StatusCheck _cstat;

    private readonly IndexedMap<Shape, NCollection_List<Shape>> _mapVE =
        new(ShapeHasher.Instance);

    private bool _gctrl;

    /// <summary>Checks the wire on its own (<see cref="Minimum"/>).</summary>
    public WireCheck(in Wire w)
    {
        _cdone = false;
        _cstat = StatusCheck.NoError;
        _gctrl = false;
        Init(w);
    }

    private static bool IsOriented(in Shape s)
        => s.Orientation() == Forward || s.Orientation() == Reversed;

    /// <summary>The status list of the wire itself, read under the lock in parallel mode.</summary>
    private NCollection_List<StatusCheck> LockedStatusList()
    {
        bool locked = _isParallel;
        if (locked)
        {
            _mutex.Enter();
        }

        try
        {
            return _map.GetValue(_shape);
        }
        finally
        {
            if (locked)
            {
                _mutex.Exit();
            }
        }
    }

    /// <summary>Checks that the wire has edges and that they are connected through their vertices.</summary>
    public override void Minimum()
    {
        _cdone = false;
        _gctrl = true;
        if (!_min)
        {
            var newList = new NCollection_List<StatusCheck>();
            NCollection_List<StatusCheck> lst = _map.SetValueRef(_shape, newList);

            // check that the wire is "connex"
            var exp = new Explorer(_shape, ShapeType.Edge);
            int nbedge = 0;
            _mapVE.Clear();
            // fill myMapVE
            for (; exp.More(); exp.Next())
            {
                nbedge++;
                var expv = new Explorer();
                for (expv.Init(exp.Current(), ShapeType.Vertex); expv.More(); expv.Next())
                {
                    Shape vtx = expv.Current();
                    int index = _mapVE.IndexOf(vtx) + 1;
                    if (index == 0)
                    {
                        var theListOfShape = new NCollection_List<Shape>();
                        index = _mapVE.Add(vtx, theListOfShape) + 1;
                    }

                    _mapVE.ValueAt(index - 1).Append(exp.Current());
                }
            }

            // wire must have at least one edge
            if (nbedge == 0)
            {
                BRepCheckCheck.Add(lst, StatusCheck.EmptyWire);
            }
            // check if all edges are connected through vertices
            else if (nbedge >= 2)
            {
                var mapE = new HashSet<Shape>(ShapeHasher.Instance);
                exp.ReInit();
                Propagate(_mapVE, exp.Current(), mapE);
                for (exp.ReInit(); exp.More(); exp.Next())
                {
                    if (!mapE.Contains(exp.Current()))
                    {
                        BRepCheckCheck.Add(lst, StatusCheck.NotConnected);
                        break;
                    }
                }
            }

            if (lst.IsEmpty())
            {
                lst.Append(StatusCheck.NoError);
            }

            _mapVE.Clear();
            _min = true;
        }
    }

    /// <summary>
    /// Checks the wire in a shape that contains it: in a face, <see cref="SelfIntersect"/> (with the geometric
    /// controls), <see cref="Closed"/>, <see cref="Orientation"/> and <see cref="Closed2d"/>, until one fails.
    /// </summary>
    public override void InContext(in Shape s)
    {
        NCollection_List<StatusCheck> hList;
        {
            bool locked = _isParallel;
            if (locked)
            {
                _mutex.Enter();
            }

            try
            {
                if (_map.ContainsKey(s))
                {
                    return;
                }

                var newList = new NCollection_List<StatusCheck>();
                hList = _map.SetValueRef(s, newList);
            }
            finally
            {
                if (locked)
                {
                    _mutex.Exit();
                }
            }
        }

        NCollection_List<StatusCheck> lst = hList;

        // check if my wire is in <S>
        var exp = new Explorer(s, ShapeType.Wire);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(_shape))
            {
                break;
            }
        }

        if (!exp.More())
        {
            BRepCheckCheck.Add(lst, StatusCheck.SubshapeNotInShape);
            return;
        }

        StatusCheck st = StatusCheck.NoError;
        ShapeType styp = s.ShapeType();
        switch (styp)
        {
            case ShapeType.Face:
            {
                Edge ed1 = new(), ed2 = new();
                if (_gctrl)
                {
                    st = SelfIntersect(ShapeCasts.Face(s), ref ed1, ref ed2, true);
                }

                if (st != StatusCheck.NoError)
                {
                    break;
                }

                st = Closed();
                if (st != StatusCheck.NoError)
                {
                    break;
                }

                st = Orientation(ShapeCasts.Face(s));
                if (st != StatusCheck.NoError)
                {
                    break;
                }

                st = Closed2d(ShapeCasts.Face(s));
                break;
            }
            default:
                break;
        }

        if (st != StatusCheck.NoError)
        {
            BRepCheckCheck.Add(lst, st);
        }

        if (lst.IsEmpty())
        {
            lst.Append(StatusCheck.NoError);
        }
    }

    /// <summary>Nothing more than the minimum.</summary>
    public override void Blind()
    {
        if (!_blind)
        {
            // nothing more that the minimum
            _blind = true;
        }
    }

    /// <summary>
    /// Checks that the oriented edges of the wire give a closed wire: <see cref="NotConnected"/>,
    /// <see cref="RedundantEdge"/> (an edge more than twice, or twice not once FORWARD and once
    /// REVERSED), <see cref="NotClosed"/> or <see cref="NoError"/>. If the first and last edges
    /// are infinite, the wire is considered closed. With <paramref name="update"/>, the status is added to
    /// <see cref="ResultCheck.Status"/>.
    /// </summary>
    public StatusCheck Closed(bool update = false)
    {
        NCollection_List<StatusCheck> statusList = LockedStatusList();
        if (_cdone)
        {
            if (update)
            {
                BRepCheckCheck.Add(statusList, _cstat);
            }

            return _cstat;
        }

        _cdone = true;

        var itl = new NCollection_List<StatusCheck>.Iterator(statusList);
        if (itl.Value() != StatusCheck.NoError)
        {
            _cstat = itl.Value();
            return _cstat; // already saved
        }

        _cstat = StatusCheck.NoError;

        var exp = new Explorer();
        var expv = new Explorer();
        var mapS = new HashSet<Shape>(ShapeHasher.Instance);
        var cradoc = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        _mapVE.Clear();
        // Checks if the oriented edges of the wire give a "closed" wire,
        // i-e if each oriented vertex on oriented edges is found 2 times...
        for (exp.Init(_shape, ShapeType.Edge); exp.More(); exp.Next())
        {
            if (IsOriented(exp.Current()))
            {
                if (!cradoc.ContainsKey(exp.Current()))
                {
                    var theListOfShape = new NCollection_List<Shape>();
                    cradoc[exp.Current()] = theListOfShape;
                }

                cradoc.GetValue(exp.Current()).Append(exp.Current());

                mapS.Add(exp.Current());
                for (expv.Init(exp.Current(), ShapeType.Vertex); expv.More(); expv.Next())
                {
                    if (IsOriented(expv.Current()))
                    {
                        int index = _mapVE.IndexOf(expv.Current()) + 1;
                        if (index == 0)
                        {
                            var theListOfShape1 = new NCollection_List<Shape>();
                            index = _mapVE.Add(expv.Current(), theListOfShape1) + 1;
                        }

                        _mapVE.ValueAt(index - 1).Append(exp.Current());
                    }
                }
            }
        }

        int theNbori = mapS.Count;
        if (theNbori >= 2)
        {
            mapS.Clear();
            for (exp.ReInit(); exp.More(); exp.Next())
            {
                if (IsOriented(exp.Current()))
                {
                    break;
                }
            }

            Propagate(_mapVE, exp.Current(), mapS);
        }

        if (theNbori != mapS.Count)
        {
            _cstat = StatusCheck.NotConnected;
            if (update)
            {
                BRepCheckCheck.Add(statusList, _cstat);
            }

            return _cstat;
        }

        // Checks the number of occurrence of an edge : maximum 2, and in this
        // case, one time FORWARD and one time REVERSED

        // OCCT.NET: the map iterates in another order than OCCT's; the result doesn't depend on it
        bool yabug = false;
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itdm in cradoc)
        {
            if (itdm.Value.Extent() >= 3)
            {
                yabug = true;
            }
            else if (itdm.Value.Extent() == 2)
            {
                if (itdm.Value.First().Orientation() == itdm.Value.Last().Orientation())
                {
                    yabug = true;
                }
            }

            if (yabug)
            {
                _cstat = StatusCheck.RedundantEdge;
                if (update)
                {
                    BRepCheckCheck.Add(statusList, _cstat);
                }

                return _cstat;
            }
        }

        for (int i = 0; i < _mapVE.Count; i++)
        {
            if (_mapVE.ValueAt(i).Extent() % 2 != 0)
            {
                _cstat = StatusCheck.NotClosed;
                if (update)
                {
                    BRepCheckCheck.Add(statusList, _cstat);
                }

                return _cstat;
            }
        }

        if (update)
        {
            BRepCheckCheck.Add(statusList, _cstat);
        }

        return _cstat;
    }

    // Return true if distance between thePnt_f and thePnt_l is not more, than aTol3d
    private static bool IsDistanceIn3DTolerance(in Point3D pnt_f, in Point3D pnt_l, double aTol3d)
    {
        double dist = pnt_f.Distance(pnt_l);

        return dist < aTol3d;
    }

    // OCCT's PrintWarnings argument only prints in debug builds; it is left out.
    private static bool IsDistanceIn2DTolerance(FaceSurfaceAdaptor aFaceSurface,
                                                in Point2D pnt,
                                                in Point2D pntRef,
                                                double aTol3d)
    {
        double dumax = 0.01 * (aFaceSurface.LastUParameter() - aFaceSurface.FirstUParameter());
        double dvmax = 0.01 * (aFaceSurface.LastVParameter() - aFaceSurface.FirstVParameter());
        double dumin = Math.Abs(pnt.X() - pntRef.X());
        double dvmin = Math.Abs(pnt.Y() - pntRef.Y());

        if ((dumin < dumax) && (dvmin < dvmax))
        {
            return true;
        }

        dumax = aFaceSurface.UResolution(aTol3d);
        dvmax = aFaceSurface.VResolution(aTol3d);
        double um = (pnt.X() + pntRef.X()) / 2.0;
        double vm = (pnt.Y() + pntRef.Y()) / 2.0;
        aFaceSurface.D1(um, vm, out Point3D _, out Vector3D du, out Vector3D dv);
        double mdu = du.Magnitude();
        if (mdu > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            dumax = Max((aTol3d / mdu), dumax);
        }

        double mdv = dv.Magnitude();
        if (mdv > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            dvmax = Max((aTol3d / mdv), dvmax);
        }

        double tol2d = 2 * Max(dumax, dvmax);

        double dist = Max(dumin, dvmin);

        return dist < tol2d;
    }

    /// <summary>
    /// Checks that the edges of the wire give a wire closed in the parametric space of <paramref name="face"/>
    /// (and closed in 3D, <see cref="Closed"/>): <see cref="NoError"/> or <see cref="NotClosed"/>.
    /// With <paramref name="update"/>, the status is added to <see cref="ResultCheck.Status"/>.
    /// </summary>
    public StatusCheck Closed2d(in Face face, bool update = false)
    {
        NCollection_List<StatusCheck> statusList = LockedStatusList();

        // 3d closure checked too
        StatusCheck closedStat = Closed();
        if (closedStat != StatusCheck.NoError)
        {
            if (update)
            {
                BRepCheckCheck.Add(statusList, closedStat);
            }

            return closedStat;
        }

        // 20/03/02 akm vvv : (OCC234) Hence this method will be used to check
        //                    both periodic and non-periodic faces
        var faceSurface = new FaceSurfaceAdaptor(face, false);

        // count edges having FORWARD or REVERSED orientation
        int nbOrirntedEdges = 0;
        var edgeExp = new Explorer(_shape, ShapeType.Edge);
        for (; edgeExp.More(); edgeExp.Next())
        {
            if (IsOriented(edgeExp.Current()))
            {
                nbOrirntedEdges++;
            }
        }

        if (nbOrirntedEdges == 0)
        {
            if (update)
            {
                BRepCheckCheck.Add(statusList, closedStat);
            }

            return closedStat;
        }

        // all those edges must form a closed 2d contour and be found by WireExplorer

        int nbFoundEdges = 0;
        var wireExp = new WireExplorer(ShapeCasts.Wire(_shape), face);
        Edge firstEdge = wireExp.Current();
        Vertex firstVertex = wireExp.CurrentVertex();
        Edge lastEdge = new();

        for (; wireExp.More(); wireExp.Next())
        {
            nbFoundEdges++;
            lastEdge = wireExp.Current();
        }

        if (nbFoundEdges != nbOrirntedEdges)
        {
            closedStat = StatusCheck.NotClosed;
            if (update)
            {
                BRepCheckCheck.Add(statusList, closedStat);
            }

            return closedStat;
        }

        // Check distance between 2d ends of first and last edges
        //   First check if first and last edges are infinite:
        bool isFirstInfinite = false;
        bool isLastInfinite = false;
        Orientation ori;

        ori = firstEdge.Orientation();
        Tool.Range(firstEdge, out double f, out double l);
        if ((ori == Forward && global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f))
            || (ori == Reversed && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l)))
        {
            isFirstInfinite = true;
        }

        ori = lastEdge.Orientation();
        Tool.Range(lastEdge, out f, out l);

        if ((ori == Forward && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l))
            || (ori == Reversed && global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f)))
        {
            isLastInfinite = true;
        }

        if (isFirstInfinite && isLastInfinite)
        {
            if (update)
            {
                BRepCheckCheck.Add(statusList, closedStat);
            }

            return closedStat;
        }
        else if (firstVertex.IsNull)
        {
            closedStat = StatusCheck.NotClosed;

            if (update)
            {
                BRepCheckCheck.Add(statusList, closedStat);
            }

            return closedStat;
        }

        Point2D p_first, p_last, p_temp; // ends of prev edge, next edge, bidon

        // get last point
        Tool.UVPoints(lastEdge, face, out p_temp, out p_last);
        if (lastEdge.Orientation() == Reversed)
        {
            p_last = p_temp;
        }

        // get first point
        if (firstEdge.Orientation() == Reversed)
        {
            Tool.UVPoints(firstEdge, face, out p_temp, out p_first);
        }
        else
        {
            Tool.UVPoints(firstEdge, face, out p_first, out p_temp);
        }

        // Check 2d distance for periodic faces with seam edge
        if (!IsClosed2dForPeriodicFace(face, p_first, p_last, firstVertex))
        {
            closedStat = StatusCheck.NotClosed;
            if (update)
            {
                BRepCheckCheck.Add(statusList, closedStat);
            }

            return closedStat;
        }

        // check distance
        double tol3d = Max(Tool.Tolerance(firstVertex), Tool.Tolerance(wireExp.CurrentVertex()));

        Point3D pntRef = Tool.Pnt(firstVertex);
        Point3D pnt = Tool.Pnt(wireExp.CurrentVertex());

        if (!IsDistanceIn2DTolerance(faceSurface, p_first, p_last, tol3d))
        {
            closedStat = StatusCheck.NotClosed;
        }

        if (!IsDistanceIn3DTolerance(pntRef, pnt, tol3d))
        {
            closedStat = StatusCheck.NotClosed;
        }

        if (update)
        {
            BRepCheckCheck.Add(statusList, closedStat);
        }

        return closedStat;
    }

    /// <summary>
    /// Checks that the oriented edges of the wire are coherently oriented (calls <see cref="Closed"/>); on a face
    /// (pass a null face if there is none) the choice between several connected edges is made in 2D:
    /// <see cref="InvalidDegeneratedFlag"/>, <see cref="BadOrientationOfSubshape"/>,
    /// <see cref="NotClosed"/> or <see cref="NoError"/>. With <paramref name="update"/>, the status
    /// is added to <see cref="ResultCheck.Status"/>.
    /// </summary>
    public StatusCheck Orientation(in Face f, bool update = false)
    {
        StatusCheck theOstat = Closed();
        NCollection_List<StatusCheck> statusList = LockedStatusList();

        if (theOstat != StatusCheck.NotClosed && theOstat != StatusCheck.NoError)
        {
            if (update)
            {
                BRepCheckCheck.Add(statusList, theOstat);
            }

            return theOstat;
        }

        theOstat = StatusCheck.NoError;

        Vertex vf = new(), vl = new();
        Orientation orient, ortmp = Forward;
        var ledge = new NCollection_List<Shape>();
        var listOfPassedEdge = new NCollection_List<Shape>();
        var exp = new Explorer();
        var vte = new Explorer();
        var mapS = new HashSet<Shape>(ShapeHasher.Instance);
        Edge theEdge = new(), theRef = new();

        // Checks the orientation of the edges
        for (exp.Init(_shape, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            orient = edg.Orientation();
            if (IsOriented(edg))
            {
                mapS.Add(edg);
                theEdge = edg;
                theRef = edg;
                for (vte.Init(edg, ShapeType.Vertex); vte.More(); vte.Next())
                {
                    Orientation vto = vte.Current().Orientation();
                    if (vto == Forward)
                    {
                        vf = ShapeCasts.Vertex(vte.Current());
                    }
                    else if (vto == Reversed)
                    {
                        vl = ShapeCasts.Vertex(vte.Current());
                    }

                    if (!vf.IsNull && !vl.IsNull)
                    {
                        break;
                    }
                }

                if (vf.IsNull && vl.IsNull)
                {
                    theOstat = StatusCheck.InvalidDegeneratedFlag;
                }

                break;
            }
        }

        if (theOstat == StatusCheck.NoError)
        {
            int index = 1;
            int nbOriNoDegen = _mapVE.Count;
            bool isGoFwd = true;

            if (vl.IsNull)
            {
                isGoFwd = false;
            }

            while (index < nbOriNoDegen)
            {
                ledge.Clear();
                listOfPassedEdge.Clear();
                // find edges that make a chain on VL if !VL.IsNull
                // otherwise on VF.

                int ind;
                if (!vl.IsNull)
                {
                    ind = _mapVE.IndexOf(vl) + 1;
                }
                else if (!vf.IsNull)
                {
                    ind = _mapVE.IndexOf(vf) + 1;
                }
                else
                {
                    theOstat = StatusCheck.InvalidDegeneratedFlag;
                    break;
                }

                for (var itls = new NCollection_List<Shape>.Iterator(_mapVE.ValueAt(ind - 1)); itls.More(); itls.Next())
                {
                    Edge edg = ShapeCasts.Edge(itls.Value());

                    orient = edg.Orientation();
                    if (mapS.Contains(edg))
                    {
                        ortmp = GetOrientation(mapS, edg);
                    }

                    // Add to list already passed outcoming edges
                    if (mapS.Contains(edg) && ortmp == orient && !edg.IsSame(theEdge))
                    {
                        for (vte.Init(edg, ShapeType.Vertex); vte.More(); vte.Next())
                        {
                            Orientation vto = vte.Current().Orientation();
                            if (!vl.IsNull)
                            {
                                if (vto == Forward && vl.IsSame(vte.Current()))
                                {
                                    listOfPassedEdge.Append(edg);
                                    break;
                                }
                            }
                            else // VF is not null
                            {
                                if (vto == Reversed && vf.IsSame(vte.Current()))
                                {
                                    listOfPassedEdge.Append(edg);
                                    break;
                                }
                            }
                        }
                    }

                    if (!mapS.Contains(edg) || ortmp != orient)
                    {
                        for (vte.Init(edg, ShapeType.Vertex); vte.More(); vte.Next())
                        {
                            Orientation vto = vte.Current().Orientation();
                            if (!vl.IsNull)
                            {
                                if (vto == Forward && vl.IsSame(vte.Current()))
                                {
                                    // If the processing is in 2d (face not null) or
                                    // if the edge is not degenerated it is added
                                    if (!f.IsNull || !Tool.Degenerated(edg))
                                    {
                                        ledge.Append(edg);
                                    }

                                    break;
                                }
                            }
                            else
                            { // VF is not null
                                if (vto == Reversed && vf.IsSame(vte.Current()))
                                {
                                    // If the processing is in 2d (face not null) or
                                    // if the edge is not degenerated it is added
                                    if (!f.IsNull || !Tool.Degenerated(edg))
                                    {
                                        ledge.Append(edg);
                                    }

                                    break;
                                }
                            }
                        }
                    }
                }

                int nbconnex = ledge.Extent();
                bool changedesens = false;
                if (nbconnex == 0)
                {
                    if (_cstat == StatusCheck.NotClosed)
                    {
                        if (vl.IsNull)
                        {
                            if (update)
                            {
                                BRepCheckCheck.Add(statusList, theOstat);
                            }

                            return theOstat; // leave
                        }
                        else
                        {
                            index--; // because after Index++ and if there is no chain,
                            vl.Nullify(); // chain on VF is forced
                            theEdge = theRef;
                            changedesens = true;
                        }
                    }
                    else
                    {
                        theOstat = StatusCheck.BadOrientationOfSubshape;
                        if (update)
                        {
                            BRepCheckCheck.Add(statusList, theOstat);
                        }

                        return theOstat;
                    }
                }

                // JAG 03/07   else if (nbconnex >= 2 && !F.IsNull())  // Try to see in 2d
                else if (!f.IsNull)
                { // Try to see in 2d
                    Vertex pivot;
                    if (!vl.IsNull)
                    {
                        pivot = vl;
                    }
                    else
                    {
                        pivot = vf;
                    }

                    ChoixUV(pivot, theEdge, f, ledge);
                    nbconnex = ledge.Extent();
                    // 20/03/02 akm (OCC234) - The 2d exploration of wire with necessary
                    // checks is performed in Closed2d, here it's useless
                }

                if (nbconnex >= 2)
                {
                    theOstat = StatusCheck.BadOrientationOfSubshape;
                    if (update)
                    {
                        BRepCheckCheck.Add(statusList, theOstat);
                    }

                    return theOstat;
                }
                else if (nbconnex == 1)
                {
                    // offset the vertex
                    for (vte.Init(ledge.First(), ShapeType.Vertex); vte.More(); vte.Next())
                    {
                        Orientation vto = vte.Current().Orientation();
                        if (!vl.IsNull)
                        {
                            if (vto == Reversed)
                            {
                                vl = ShapeCasts.Vertex(vte.Current());
                                break;
                            }
                        }
                        else
                        { // VF is not null
                            if (vto == Forward)
                            {
                                vf = ShapeCasts.Vertex(vte.Current());
                                break;
                            }
                        }
                    }

                    mapS.Add(ledge.First());
                    theEdge = ShapeCasts.Edge(ledge.First());
                    if (!vte.More())
                    {
                        if (!vl.IsNull)
                        {
                            vl.Nullify();
                        }
                        else
                        {
                            vf.Nullify();
                        }
                    }
                }
                else if (!changedesens)
                { // nbconnex == 0
                    theOstat = StatusCheck.NotClosed;
                    if (update)
                    {
                        BRepCheckCheck.Add(statusList, theOstat);
                    }

                    return theOstat;
                }

                // Check the closure of the wire in 2d (not done in Closed())

                Vertex vRef = new();
                bool isCheckClose = false;

                if (isGoFwd && !vf.IsNull)
                {
                    vRef = vf;
                    isCheckClose = true;
                }
                else if (!isGoFwd && !vl.IsNull)
                {
                    vRef = vl;
                    isCheckClose = true;
                }

                if (index == 1 && _cstat != StatusCheck.NotClosed && isCheckClose && !f.IsNull)
                {
                    ledge.Clear();
                    ind = _mapVE.IndexOf(vRef) + 1;
                    for (var itlsh = new NCollection_List<Shape>.Iterator(_mapVE.ValueAt(ind - 1)); itlsh.More();
                         itlsh.Next())
                    {
                        Edge edg = ShapeCasts.Edge(itlsh.Value());
                        orient = edg.Orientation();
                        if (!theRef.IsSame(edg))
                        {
                            for (vte.Init(edg, ShapeType.Vertex); vte.More(); vte.Next())
                            {
                                Orientation vto = vte.Current().Orientation();
                                if (vto == Reversed && vRef.IsSame(vte.Current()))
                                {
                                    ledge.Append(edg);
                                    break;
                                }
                            }
                        }
                    }

                    ChoixUV(vRef, theRef, f, ledge);
                    if (ledge.Extent() == 0)
                    {
                        theOstat = StatusCheck.NotClosed;
                        if (update)
                        {
                            BRepCheckCheck.Add(statusList, theOstat);
                        }

                        return theOstat;
                    }
                }
                // End control closure 2d

                index++;
            }
        }

        if (update)
        {
            BRepCheckCheck.Add(statusList, theOstat);
        }

        return theOstat;
    }

    /// <summary>
    /// Checks that the wire doesn't intersect itself on the face <paramref name="f"/>:
    /// <see cref="EmptyWire"/>, <see cref="SelfIntersectingWire"/>,
    /// <see cref="NoCurveOnSurface"/>, <see cref="InvalidRange"/> or
    /// <see cref="NoError"/>. <paramref name="retE1"/> and <paramref name="retE2"/> receive the first
    /// intersecting edges found (only <paramref name="retE1"/> for an edge that intersects itself or has no pcurve);
    /// they are left unchanged otherwise. With <paramref name="update"/>, the status is added to
    /// <see cref="ResultCheck.Status"/> (not for NoCurveOnSurface and InvalidRange, as in OCCT).
    /// </summary>
    public StatusCheck SelfIntersect(in Face f, ref Edge retE1, ref Edge retE2, bool update = false)
    {
        NCollection_List<StatusCheck> statusList = LockedStatusList();

        int i, j, nbedges;
        double first1, last1, first2, last2, tolint;
        Point2D pfirst1, plast1, pfirst2, plast2;
        Point3D p3d, p3d2;
        FaceSurfaceAdaptor hs;
        var c1 = new CurveAdaptor2D();
        var c2 = new CurveAdaptor2D();
        var inter = new GInter();
        var myDomain1 = new TwoDDomain();
        // OCCT's NCollection_IndexedMap<TopoDS_Shape> hashes with std::hash and compares with operator==
        // (IsEqual: the orientation counts)
        var eMap = new IndexedSet<Shape>(EqualityComparer<Shape>.Default);
        //
        //-- check with proper tolerances if there is no
        //-- point in the tolerance of a vertex.
        tolint = 1.0e-10;
        hs = new FaceSurfaceAdaptor();
        hs.Initialize(f, false);
        //
        for (var iter1 = new Iterator(_shape); iter1.More(); iter1.Next())
        {
            if (iter1.Value().ShapeType() == ShapeType.Edge)
            {
                eMap.Add(iter1.Value());
            }
        }

        //
        nbedges = eMap.Count;
        if (nbedges == 0)
        {
            if (update)
            {
                BRepCheckCheck.Add(statusList, StatusCheck.EmptyWire);
            }

            return StatusCheck.EmptyWire;
        }

        //
        var tabDom = new TwoDDomain[nbedges];
        for (int k0 = 0; k0 < nbedges; k0++)
        {
            tabDom[k0] = new TwoDDomain();
        }

        var tabCur = new NCollection_Array1<Curve2D?>(1, nbedges);
        var boxes = new NCollection_Array1<Box2d>(1, nbedges);
        //
        for (i = 1; i <= nbedges; i++)
        {
            Edge e1 = ShapeCasts.Edge(eMap[i - 1]);
            if (i == 1)
            {
                Curve2D? pcu = Tool.CurveOnSurface(e1, f, out first1, out last1);
                if (pcu is null)
                {
                    retE1 = e1;
                    if (update)
                    {
                        BRepCheckCheck.Add(statusList, StatusCheck.SelfIntersectingWire);
                    }

                    return StatusCheck.SelfIntersectingWire;
                }

                //
                c1.Load(pcu);
                // To avoid exception in Segment if C1 is BSpline - IFV
                if (!c1.IsPeriodic)
                {
                    if (c1.FirstParameter() > first1)
                    {
                        first1 = c1.FirstParameter();
                    }

                    if (c1.LastParameter() < last1)
                    {
                        last1 = c1.LastParameter();
                    }
                }

                //
                Tool.UVPoints(e1, f, out pfirst1, out plast1);
                myDomain1.SetValues(pfirst1, first1, tolint, plast1, last1, tolint);
                //
                Add2dCurve.Add(c1, first1, last1, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref boxes[i]);
            } // if (i == 1) {
            else
            {
                c1.Load(tabCur[i]!);
                myDomain1 = tabDom[i - 1];
            }

            //
            // Self intersect of C1
            inter.Perform(c1, myDomain1, tolint, tolint);
            //
            if (inter.IsDone)
            {
                int nbp = inter.PointCount;
                //
                for (int p = 1; p <= nbp; p++)
                {
                    IntersectionPoint2D ip = inter.Point(p);
                    Transition tr1 = ip.TransitionOfFirst();
                    Transition tr2 = ip.TransitionOfSecond();
                    if (tr1.PositionOnCurve() == Position2D.Middle || tr2.PositionOnCurve() == Position2D.Middle)
                    {
                        //-- Checking of points with true tolerances (ie Tol in 3d)
                        //-- If the point of intersection is within the tolerance of a vertex
                        //-- this intersection is considered correct (no error)
                        bool localok = false;
                        Curve3D? conS = Tool.Curve(e1, out Placement l, out double _, out double _);
                        if (conS is not null)
                        {
                            //-- try to test in 3d. (ParamOnSecond gives the same result)
                            p3d = conS.Value(ip.ParamOnFirst());
                            p3d = p3d.Transformed(l.Transformation());
                        }
                        else
                        {
                            Point2D p2d = c1.Value(ip.ParamOnFirst());
                            p3d = hs.Value(p2d.X(), p2d.Y());
                        }

                        var explVtx = new Explorer();
                        for (explVtx.Init(e1, ShapeType.Vertex); !localok && explVtx.More(); explVtx.Next())
                        {
                            Point3D p3dvtt;
                            double tolvtt, p3dvttDistanceP3d;
                            //
                            Vertex vtt = ShapeCasts.Vertex(explVtx.Current());
                            p3dvtt = Tool.Pnt(vtt);
                            tolvtt = Tool.Tolerance(vtt);
                            tolvtt = tolvtt * tolvtt;
                            p3dvttDistanceP3d = p3dvtt.SquareDistance(p3d);
                            if (p3dvttDistanceP3d <= tolvtt)
                            {
                                localok = true;
                            }
                        }

                        if (!localok)
                        {
                            retE1 = e1;
                            if (update)
                            {
                                BRepCheckCheck.Add(statusList, StatusCheck.SelfIntersectingWire);
                            }

                            return StatusCheck.SelfIntersectingWire;
                        }
                    }
                }
            } // if(Inter.IsDone()) {

            //
            for (j = i + 1; j <= nbedges; j++)
            {
                Edge e2 = ShapeCasts.Edge(eMap[j - 1]);
                if (i == 1)
                {
                    tabCur[j] = Tool.CurveOnSurface(e2, f, out first2, out last2);
                    if (tabCur[j] is not null && last2 > first2)
                    {
                        c2.Load(tabCur[j]!);
                        // To avoid exception in Segment if C2 is BSpline - IFV
                        if (!c2.IsPeriodic)
                        {
                            if (c2.FirstParameter() > first2)
                            {
                                first2 = c2.FirstParameter();
                            }

                            if (c2.LastParameter() < last2)
                            {
                                last2 = c2.LastParameter();
                            }
                        }

                        //
                        Tool.UVPoints(e2, f, out pfirst2, out plast2);
                        tabDom[j - 1].SetValues(pfirst2, first2, tolint, plast2, last2, tolint);

                        Add2dCurve.Add(c2, first2, last2, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref boxes[j]);
                    }
                    else
                    {
                        if (tabCur[j] is null)
                        {
                            return StatusCheck.NoCurveOnSurface;
                        }

                        return StatusCheck.InvalidRange;
                    }
                } // if (i == 1) {
                else
                {
                    c2.Load(tabCur[j]!);
                }

                //
                if (boxes[i].IsOut(boxes[j]))
                {
                    continue;
                }

                // modified by NIZNHY-PKV Fri Oct 29 10:09:01 2010f
                if (e1.IsSame(e2))
                {
                    continue;
                }

                //
                //-- ************************************************************
                //-- ******* I n t e r s e c t i o n   C 1   and   C 2   ********
                //-- ************************************************************
                inter.Perform(c1, myDomain1, c2, tabDom[j - 1], tolint, tolint);
                //
                if (inter.IsDone)
                {
                    int nbp, nbs;
                    double IP_ParamOnFirst, IP_ParamOnSecond;
                    Transition tr1, tr2;
                    var commonVertices = new NCollection_List<Shape>();
                    var itl = new NCollection_List<Shape>.Iterator();
                    var vmap = new HashSet<Shape>(ShapeHasher.Instance);
                    //
                    var it = new Iterator(e1);
                    for (; it.More(); it.Next())
                    {
                        vmap.Add(it.Value());
                    }

                    //
                    it.Initialize(e2);
                    for (; it.More(); it.Next())
                    {
                        Shape v = it.Value();
                        if (vmap.Contains(v))
                        {
                            commonVertices.Append(v);
                        }
                    }

                    //
                    nbp = inter.PointCount;
                    nbs = inter.SegmentCount;
                    IP_ParamOnFirst = 0.0;
                    IP_ParamOnSecond = 0.0;
                    //
                    //// **** Points of intersection **** ////
                    for (int p = 1; p <= nbp; p++)
                    {
                        IntersectionPoint2D ip = inter.Point(p);
                        IP_ParamOnFirst = ip.ParamOnFirst();
                        IP_ParamOnSecond = ip.ParamOnSecond();
                        tr1 = ip.TransitionOfFirst();
                        tr2 = ip.TransitionOfSecond();
                        if (tr1.PositionOnCurve() == Position2D.Middle || tr2.PositionOnCurve() == Position2D.Middle)
                        {
                            //-- Checking of points with true tolerances (ie Tol in 3d)
                            //-- If the point of intersection is within the tolerance of a vertex
                            //-- this intersection is considered correct (no error)
                            bool localok = false;
                            //
                            Curve3D? conS = Tool.Curve(e1, out Placement l, out double f1, out double l1);
                            Curve3D? conS2 = Tool.Curve(e2, out Placement L2, out double f2, out double l2);
                            // gka protect against working out of edge range
                            if (f1 - IP_ParamOnFirst > global::Kerf.Engine.Foundation.Precision.PConfusion()
                                || IP_ParamOnFirst - l1 > global::Kerf.Engine.Foundation.Precision.PConfusion()
                                || f2 - IP_ParamOnSecond > global::Kerf.Engine.Foundation.Precision.PConfusion()
                                || IP_ParamOnSecond - l2 > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                continue;
                            }

                            double tolvtt = 0.0;
                            if (conS is not null)
                            {
                                p3d = conS.Value(IP_ParamOnFirst);
                                p3d = p3d.Transformed(l.Transformation());
                            }
                            else
                            {
                                Point2D p2d = c1.Value(IP_ParamOnFirst);
                                p3d = hs.Value(p2d.X(), p2d.Y());
                            }

                            //
                            if (conS2 is not null)
                            {
                                p3d2 = conS2.Value(IP_ParamOnSecond);
                                p3d2 = p3d2.Transformed(L2.Transformation());
                            }
                            else
                            {
                                Point2D p2d = c2.Value(IP_ParamOnSecond);
                                p3d2 = hs.Value(p2d.X(), p2d.Y());
                            }

                            itl.Initialize(commonVertices);
                            for (; itl.More(); itl.Next())
                            {
                                double p3dvttDistanceP3d, p3dvttDistanceP3d2;
                                Point3D p3dvtt;
                                //
                                Vertex vtt = ShapeCasts.Vertex(itl.Value());
                                p3dvtt = Tool.Pnt(vtt);
                                tolvtt = Tool.Tolerance(vtt);
                                tolvtt = 1.1 * tolvtt;
                                tolvtt = tolvtt * tolvtt;
                                p3dvttDistanceP3d = p3dvtt.SquareDistance(p3d);
                                p3dvttDistanceP3d2 = p3dvtt.SquareDistance(p3d2);
                                //
                                if (p3dvttDistanceP3d <= tolvtt && p3dvttDistanceP3d2 <= tolvtt)
                                {
                                    localok = true;
                                    break;
                                }
                            }

                            //-- --------------------------------------------------------
                            //-- Check maximum yawn between 2 edges
                            //--
                            //-- Check distance from edges to the curve joining
                            //-- the point of intersection with vertex (if exists)
                            if (!localok && !commonVertices.IsEmpty())
                            {
                                double distauvtxleplusproche, vParaOnEdge1, vParaOnEdge2;
                                Point3D vertexLePlusProche = new();
                                //
                                vParaOnEdge1 = 0.0;
                                vParaOnEdge2 = 0.0;
                                distauvtxleplusproche = RealLast();
                                // Find the nearest common vertex
                                itl.Initialize(commonVertices);
                                for (; itl.More(); itl.Next())
                                {
                                    double disptvtx;
                                    Point3D p3dvtt;
                                    //
                                    Vertex vtt = ShapeCasts.Vertex(itl.Value());
                                    p3dvtt = Tool.Pnt(vtt);
                                    disptvtx = p3d.Distance(p3dvtt);
                                    if (disptvtx < distauvtxleplusproche)
                                    {
                                        vertexLePlusProche = p3dvtt;
                                        distauvtxleplusproche = disptvtx;
                                        vParaOnEdge1 = Tool.Parameter(vtt, e1);
                                        vParaOnEdge2 = Tool.Parameter(vtt, e2);
                                    }
                                    // eap: case of closed edge
                                    else if (IsEqual(distauvtxleplusproche, disptvtx))
                                    {
                                        double newVParaOnEdge1 = Tool.Parameter(vtt, e1);
                                        double newVParaOnEdge2 = Tool.Parameter(vtt, e2);
                                        if (Math.Abs(IP_ParamOnFirst - vParaOnEdge1)
                                            + Math.Abs(IP_ParamOnSecond - vParaOnEdge2)
                                            > Math.Abs(IP_ParamOnFirst - newVParaOnEdge1)
                                            + Math.Abs(IP_ParamOnSecond - newVParaOnEdge2))
                                        {
                                            vertexLePlusProche = p3dvtt;
                                            vParaOnEdge1 = newVParaOnEdge1;
                                            vParaOnEdge2 = newVParaOnEdge2;
                                        }
                                    }
                                }

                                // Patch: extraordinar situation (e.g. tolerance(v) == 0.)
                                //   Modified by skv - Wed Jul 23 12:28:11 2003 OCC1764
                                if (vertexLePlusProche.Distance(p3d) <= Constants.Resolution()
                                    || vertexLePlusProche.Distance(p3d2) <= Constants.Resolution())
                                {
                                    localok = true;
                                }
                                else
                                {
                                    var lig = new Line3D(vertexLePlusProche, new Direction3D(new Vector3D(vertexLePlusProche, p3d)));
                                    double du1 = 0.1 * (IP_ParamOnFirst - vParaOnEdge1);
                                    double du2 = 0.1 * (IP_ParamOnSecond - vParaOnEdge2);
                                    double maxd1 = 0.0, maxd2 = 0.0;
                                    int k;

                                    localok = true;
                                    double tole1 = Tool.Tolerance(e1);
                                    for (k = 2; localok && k < 9; k++)
                                    {
                                        double u = vParaOnEdge1 + k * du1; // check if it works
                                        Point3D p1;
                                        if (conS is not null)
                                        {
                                            p1 = conS.Value(u);
                                            p1 = p1.Transformed(l.Transformation());
                                        }
                                        else
                                        {
                                            Point2D p2d = c1.Value(u);
                                            p1 = hs.Value(p2d.X(), p2d.Y());
                                        }

                                        double d1 = lig.Distance(p1);
                                        if (d1 > maxd1)
                                        {
                                            maxd1 = d1;
                                        }

                                        if (d1 > tole1 * 2.0)
                                        {
                                            localok = false;
                                        }
                                    }

                                    //-- same for edge2
                                    //  Modified by skv - Wed Jul 23 12:22:20 2003 OCC1764
                                    var tmpDir = new Direction3D(p3d2.XYZ().Subtracted(vertexLePlusProche.XYZ()));

                                    lig = lig.WithDirection(tmpDir);
                                    double tole2 = Tool.Tolerance(e2);
                                    for (k = 2; localok && k < 9; k++)
                                    {
                                        double u = vParaOnEdge2 + k * du2; // check if it works
                                        Point3D p2;
                                        if (conS2 is not null)
                                        {
                                            p2 = conS2.Value(u);
                                            p2 = p2.Transformed(L2.Transformation());
                                        }
                                        else
                                        {
                                            Point2D p2d = c2.Value(u);
                                            p2 = hs.Value(p2d.X(), p2d.Y());
                                        }

                                        double d2 = lig.Distance(p2);
                                        if (d2 > maxd2)
                                        {
                                            maxd2 = d2;
                                        }

                                        if (d2 > tole2 * 2.0)
                                        {
                                            localok = false;
                                        }
                                    }
                                } // end of else (construction of the line Lig)
                            } // end of if (localok == false && !CommonVertices.IsEmpty())

                            //
                            if (!localok)
                            {
                                retE1 = e1;
                                retE2 = e2;
                                if (update)
                                {
                                    BRepCheckCheck.Add(statusList, StatusCheck.SelfIntersectingWire);
                                }

                                return StatusCheck.SelfIntersectingWire;
                            } //-- localok == False
                        } // end of if(Tr1.PositionOnCurve() == IntRes2d_Middle || Tr2.PositionOnCurve() == IntRes2d_Middle)
                    } // end of for (int p=1; p <= nbp; p++)

                    ////
                    //// **** Segments of intersection **** ////
                    for (int s = 1; s <= nbs; ++s)
                    {
                        IntersectionSegment2D seg = inter.Segment(s);
                        if (seg.HasFirstPoint && seg.HasLastPoint)
                        {
                            bool localok;
                            int k;
                            var pSeg = new IntersectionPoint2D[2];
                            Position2D pcr1, pcr2;
                            //
                            localok = false;
                            pSeg[0] = seg.FirstPoint();
                            pSeg[1] = seg.LastPoint();
                            // At least one of extremities of the segment must be inside
                            // the tolerance of a common vertex
                            for (k = 0; k < 2; ++k)
                            {
                                IP_ParamOnFirst = pSeg[k].ParamOnFirst();
                                IP_ParamOnSecond = pSeg[k].ParamOnSecond();
                                tr1 = pSeg[k].TransitionOfFirst();
                                tr2 = pSeg[k].TransitionOfSecond();
                                pcr1 = tr1.PositionOnCurve();
                                pcr2 = tr2.PositionOnCurve();
                                //
                                if (pcr1 != Position2D.Middle && pcr2 != Position2D.Middle)
                                {
                                    CurveKind ct1, ct2;
                                    // ZZ
                                    ct1 = c1.GetType();
                                    ct2 = c2.GetType();
                                    if (ct1 == Line && ct2 == Line)
                                    {
                                        // check for the two lines coincidence
                                        double par_T, t11, t12, t21, t22, t1m, t2m;
                                        double aD2, tolE1, tolE2, tol2;
                                        Line2D aL1, aL2;
                                        Point2D p1m;
                                        //
                                        par_T = 0.43213918;
                                        //
                                        tolE1 = Tool.Tolerance(e1);
                                        tolE2 = Tool.Tolerance(e2);
                                        tol2 = tolE1 + tolE2;
                                        tol2 = tol2 * tol2;
                                        //
                                        aL1 = c1.Line();
                                        aL2 = c2.Line();
                                        //
                                        t11 = pSeg[0].ParamOnFirst();
                                        t12 = pSeg[1].ParamOnFirst();
                                        t21 = pSeg[0].ParamOnSecond();
                                        t22 = pSeg[1].ParamOnSecond();
                                        //
                                        t1m = (1.0 - par_T) * t11 + par_T * t12;
                                        p1m = c1.Value(t1m);
                                        //
                                        aD2 = aL2.SquareDistance(p1m);
                                        if (aD2 < tol2)
                                        {
                                            t2m = global::Kerf.Engine.Geometry.ElCLib.Parameter(aL2, p1m);
                                            if (t2m > t21 && t2m < t22)
                                            {
                                                Direction2D dir1 = aL1.Direction();
                                                Direction2D dir2 = aL2.Direction();
                                                if (dir1.IsParallel(dir2, global::Kerf.Engine.Foundation.Precision.Angular()))
                                                {
                                                    localok = false;
                                                    break; // from for (k = 0; k < 2; ++k){...
                                                }
                                            } // if (aT2m>aT21 && aT2m<aT22) {
                                        } // if (aD2<aTol2) {
                                    } // if (aCT1==GeomAbs_Line && aCT2==GeomAbs_Line) {

                                    // ZZ
                                    localok = true;
                                    break;
                                }

                                //
                                double tolvtt;
                                Curve3D? conS = Tool.Curve(e1, out Placement l, out double _, out double _);
                                Curve3D? conS2 = Tool.Curve(e2, out Placement L2, out double _, out double _);
                                if (conS is not null)
                                {
                                    p3d = conS.Value(IP_ParamOnFirst);
                                    p3d = p3d.Transformed(l.Transformation());
                                }
                                else
                                {
                                    Point2D p2d = c1.Value(IP_ParamOnFirst);
                                    p3d = hs.Value(p2d.X(), p2d.Y());
                                }

                                if (conS2 is not null)
                                {
                                    p3d2 = conS2.Value(IP_ParamOnSecond);
                                    p3d2 = p3d2.Transformed(L2.Transformation());
                                }
                                else
                                {
                                    Point2D p2d = c2.Value(IP_ParamOnSecond);
                                    p3d2 = hs.Value(p2d.X(), p2d.Y());
                                }

                                itl.Initialize(commonVertices);
                                for (; itl.More(); itl.Next())
                                {
                                    double p3dvttDistanceP3d, p3dvttDistanceP3d2;
                                    Point3D p3dvtt;
                                    //
                                    Vertex vtt = ShapeCasts.Vertex(itl.Value());
                                    p3dvtt = Tool.Pnt(vtt);
                                    tolvtt = Tool.Tolerance(vtt);
                                    tolvtt = 1.1 * tolvtt;
                                    tolvtt = tolvtt * tolvtt;
                                    p3dvttDistanceP3d = p3dvtt.SquareDistance(p3d);
                                    p3dvttDistanceP3d2 = p3dvtt.SquareDistance(p3d2);
                                    if (p3dvttDistanceP3d <= tolvtt && p3dvttDistanceP3d2 <= tolvtt)
                                    {
                                        localok = true;
                                        break;
                                    }
                                }

                                if (localok)
                                {
                                    break;
                                }
                            } // end of for (k = 0; k < 2; k++)

                            //
                            if (!localok)
                            {
                                retE1 = e1;
                                retE2 = e2;
                                if (update)
                                {
                                    BRepCheckCheck.Add(statusList, StatusCheck.SelfIntersectingWire);
                                }

                                return StatusCheck.SelfIntersectingWire;
                            } //-- localok == False
                        } // end of if(Seg.HasFirstPoint() && Seg.HasLastPoint())
                    } // end of for (int s = 1; s <= nbs; p++)
                } //-- Inter.IsDone()
            } // end of for( j = i+1; j<=Nbedges; j++)
        } // end of for(i = 1; i <= Nbedges; i++)

        //
        if (update)
        {
            BRepCheckCheck.Add(statusList, StatusCheck.NoError);
        }

        //
        return StatusCheck.NoError;
    }

    /// <summary>Adds <paramref name="status"/> to the statuses of the wire.</summary>
    public void SetStatus(StatusCheck status)
    {
        // OCCT takes no lock here (the analyzer never calls it)
        BRepCheckCheck.Add(_map.GetValue(_shape), status);
    }

    /// <summary>Sets whether <see cref="InContext"/> checks <see cref="SelfIntersect"/>.</summary>
    public void GeometricControls(bool b)
    {
        if (_gctrl != b)
        {
            if (b)
            {
                _cdone = false;
            }

            _gctrl = b;
        }
    }

    /// <summary>Whether <see cref="InContext"/> checks <see cref="SelfIntersect"/>.</summary>
    public bool GeometricControls() => _gctrl;

    // fill <mapE> with edges connected to <edg> through vertices contained in <mapVE>
    private static void Propagate(IndexedMap<Shape, NCollection_List<Shape>> mapVE,
                                  in Shape edg,
                                  HashSet<Shape> mapE)
    {
        var currentEdges = new NCollection_List<Shape>();
        currentEdges.Append(edg);

        do
        {
            var nextEdges = new NCollection_List<Shape>();
            var itrc = new NCollection_List<Shape>.Iterator(currentEdges);
            for (; itrc.More(); itrc.Next())
            {
                Shape edge = itrc.Value();
                if (!mapE.Contains(edge))
                {
                    mapE.Add(edge);
                }

                var ex = new Explorer(edge, ShapeType.Vertex);
                for (; ex.More(); ex.Next())
                {
                    Vertex vtx = ShapeCasts.Vertex(ex.Current());
                    int indv = mapVE.IndexOf(vtx) + 1;
                    if (indv != 0)
                    {
                        NCollection_List<Shape> edges = mapVE.ValueAt(indv - 1);

                        var itl = new NCollection_List<Shape>.Iterator(edges);
                        for (; itl.More(); itl.Next())
                        {
                            Shape e = itl.Value();
                            if (!edge.IsSame(e) && !mapE.Contains(e))
                            {
                                mapE.Add(e);
                                nextEdges.Append(e);
                            }
                        }
                    }
                }
            }

            currentEdges = nextEdges;
        } while (!currentEdges.IsEmpty());
    }

    // The orientation of the key of <mapE> that is the same edge as <edg>. The map compares with IsSame, so it
    // holds one such key and OCCT's map order doesn't matter.
    private static Orientation GetOrientation(HashSet<Shape> mapE, in Edge edg)
    {
        Shape key = new();
        foreach (Shape itm in mapE)
        {
            key = itm;
            if (itm.IsSame(edg))
            {
                break;
            }
        }

        return key.Orientation();
    }

    // For vertex theVertex given function find an edge along that we should go further.
    private static void ChoixUV(in Vertex vertex,
                                in Edge edge,
                                in Face face,
                                NCollection_List<Shape> lOfShape)
    {
        var it = new NCollection_List<Shape>.Iterator(lOfShape);
        while (it.More())
        {
            if (edge.IsSame(it.Value()))
            {
                lOfShape.Remove(it);
            }
            else
            {
                it.Next();
            }
        }

        double tol3d = Tool.Tolerance(vertex);

        int index = 0, indMin = 0;
        Edge eFound;
        Point2D pntRef, pnt = new();
        Vector2D derRef, der;
        double minAngle, maxAngle, angle;
        double a_gpResolution = Constants.Resolution();
        Orientation vOrientation, edgOrientation;
        double param = 0.0, firstParam, lastParam, parPiv;
        var faceSurface = new FaceSurfaceAdaptor(face, false); // no restriction

        Curve2D? c2d = Tool.CurveOnSurface(edge, face, out firstParam, out lastParam);
        if (c2d is null)
        { // JAG 10.12.96
            return;
        }

        vOrientation = vertex.Orientation();
        edgOrientation = edge.Orientation();

        parPiv = (vOrientation == edgOrientation) ? firstParam : lastParam;
        minAngle = RealLast();
        maxAngle = RealFirst();

        // OCCT passes the curve handle, converted to an adaptor by Geom2dAdaptor_Curve's constructor
        CurveDirForParameter(new CurveAdaptor2D(c2d), parPiv, out pntRef, out derRef);

        if (vOrientation != edgOrientation)
        {
            derRef = derRef.Reversed();
        }

        it.Initialize(lOfShape);

        for (; it.More(); it.Next())
        {
            index++;
            Edge e = ShapeCasts.Edge(it.Value());
            c2d = Tool.CurveOnSurface(e, face, out firstParam, out lastParam);
            if (c2d is null)
            {
                continue;
            }

            var ca = new CurveAdaptor2D(c2d);

            param = (vOrientation != e.Orientation()) ? firstParam : lastParam;
            pnt = ca.Value(param);

            if (!IsDistanceIn2DTolerance(faceSurface, pnt, pntRef, tol3d))
            {
                continue;
            }

            CurveDirForParameter(ca, param, out pnt, out der);

            if (vOrientation == e.Orientation())
            {
                der = der.Reversed();
            }

            if ((derRef.Magnitude() <= a_gpResolution) || (der.Magnitude() <= a_gpResolution))
            {
                // Vector length is too small
                continue;
            }

            angle = -derRef.Angle(der);

            if (angle < 0.0)
            {
                angle += 2.0 * M_PI;
            }

            if (face.Orientation() == Forward)
            {
                if (angle < minAngle)
                {
                    indMin = index;
                    minAngle = angle;
                }
            }
            else // theFace.Orientation() != TopAbs_FORWARD
            {
                if (angle > maxAngle)
                {
                    indMin = index;
                    maxAngle = angle;
                }
            }
        } // end of for

        // Update edge
        if (indMin == 0)
        {
            if (lOfShape.Extent() == 1)
            {
                bool isFound = true; // all right
                eFound = ShapeCasts.Edge(lOfShape.First());

                if (eFound.IsNull || Tool.Degenerated(edge) || Tool.Degenerated(eFound))
                {
                    isFound = false; // bad
                }
                else if (!IsDistanceIn2DTolerance(faceSurface, pnt, pntRef, tol3d))
                {
                    isFound = false; // bad
                }
                else
                // clousureness in 3D
                {
                    // IsDistanceIn3DTolerance
                    var bcEdg = new EdgeCurveAdaptor(edge, face);
                    var bcEvois = new EdgeCurveAdaptor(eFound, face);
                    Point3D pEdg = bcEdg.Value(parPiv);
                    Point3D pEFound = bcEvois.Value(param);

                    isFound = IsDistanceIn3DTolerance(pEdg, pEFound, tol3d);
                }

                if (!isFound)
                {
                    lOfShape.Clear();
                }
            } // if (theLOfShape.Extent() == 1)
            else // if (anIndMin == 0)
            {
                lOfShape.Clear();
            }
        }
        else
        {
            index = 1;

            while (index < indMin)
            {
                lOfShape.RemoveFirst();
                index++;
            }

            it.Initialize(lOfShape);
            it.Next();

            while (it.More())
            {
                lOfShape.Remove(it);
            }
        }
    } // End of function

    private static void CurveDirForParameter(CurveAdaptor2D aC2d, double aPrm, out Point2D pnt, out Vector2D aVec2d)
    {
        double tol = Constants.Resolution();
        int i;

        aC2d.D1(aPrm, out pnt, out aVec2d);
        //
        if (aVec2d.Magnitude() <= tol)
        {
            for (i = 2; i <= 100; i++)
            {
                aVec2d = aC2d.DN(aPrm, i);
                if (aVec2d.Magnitude() > tol)
                {
                    break;
                }
            }
        }
    }

    // Returns the parametric point of theVertex on theFace through theEdge; false if theVertex isn't a vertex of
    // theEdge or theEdge has no pcurve on theFace.
    private static bool GetPnt2d(in Vertex vertex, in Edge edge, in Face face, ref Point2D aPnt)
    {
        Curve2D? pCurve;
        double parOnEdge;

        Exploration.Vertices(edge, out Vertex firstVtx, out Vertex lastVtx);

        if (!vertex.IsSame(firstVtx) && !vertex.IsSame(lastVtx))
        {
            return false;
        }

        pCurve = Tool.CurveOnSurface(edge, face, out double _, out double _);

        if (pCurve is null)
        {
            return false;
        }

        parOnEdge = Tool.Parameter(vertex, edge);
        aPnt = pCurve.Value(parOnEdge);

        return true;
    }

    // Checks the distance between first point of the first edge and last point of the last edge in 2d for
    // periodic face.
    private static bool IsClosed2dForPeriodicFace(in Face face,
                                                  in Point2D p1,
                                                  in Point2D p2,
                                                  in Vertex vertex)
    {
        // Check 2d distance for periodic faces with seam edge.
        // Searching for seam edges
        var seamEdges = new NCollection_List<Shape>();
        var notSeams = new HashSet<Shape>(ShapeHasher.Instance);
        var closedEdges = new HashSet<Shape>(ShapeHasher.Instance);
        var exp = new Explorer(face, ShapeType.Edge);

        for (; exp.More(); exp.Next())
        {
            Edge edge = ShapeCasts.Edge(exp.Current());

            if (notSeams.Contains(edge))
            {
                continue;
            }

            if (!IsOriented(edge) || !Tool.IsClosed(edge, face))
            {
                notSeams.Add(edge);
                continue;
            }

            if (!closedEdges.Add(edge))
            {
                seamEdges.Append(edge);
            }
        }

        if (seamEdges.Extent() == 0)
        {
            return true;
        }

        // check if theVertex lies on one of the seam edges
        var faceSurface = new FaceSurfaceAdaptor(face, false);
        double tol = Tool.Tolerance(vertex);
        double uResol = faceSurface.UResolution(tol);
        double vResol = faceSurface.VResolution(tol);
        double vicinity = Math.Sqrt(uResol * uResol + vResol * vResol);
        double distP1P2 = p1.Distance(p2);

        var iter = new NCollection_List<Shape>.Iterator(seamEdges);

        for (; iter.More(); iter.Next())
        {
            Edge seamEdge = ShapeCasts.Edge(iter.Value());

            exp.Init(seamEdge, ShapeType.Vertex);
            for (; exp.More(); exp.Next())
            {
                Shape vtx = exp.Current();

                // We found an edge. Check the distance between two given points
                //  to be lower than the computed tolerance.
                if (IsOriented(vtx) && vtx.IsSame(vertex))
                {
                    Point2D pnt1 = new();
                    Point2D pnt2 = new();
                    double a2dTol;

                    if (!GetPnt2d(vertex, seamEdge, face, ref pnt1))
                    {
                        continue;
                    }

                    seamEdge = ShapeCasts.Edge(seamEdge.Reversed());

                    if (!GetPnt2d(vertex, seamEdge, face, ref pnt2))
                    {
                        continue;
                    }

                    a2dTol = pnt1.Distance(pnt2) * 1.0e-2;
                    a2dTol = Max(a2dTol, vicinity);

                    if (distP1P2 > a2dTol)
                    {
                        return false;
                    }
                }
            }
        }

        return true;
    }
}
