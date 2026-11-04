// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Shell.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Shell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Checks.StatusCheck;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Checks;

/// <summary>
/// The checks of a shell: connected (<see cref="Minimum"/>), closed (<see cref="Closed"/>), coherently oriented
/// (<see cref="Orientation"/>), and its sets of faces connected through simple edges
/// (<see cref="NbConnectedSet"/>).
/// </summary>
internal class ShellCheck : ResultCheck
{
    private int _nbori;
    private bool _cdone;
    private StatusCheck _cstat;
    private bool _odone;
    private StatusCheck _ostat;

    private readonly IndexedMap<Shape, NCollection_List<Shape>> _mapEF =
        new(ShapeHasher.Instance);

    /// <summary>Checks the shell on its own (<see cref="Minimum"/>).</summary>
    public ShellCheck(in Shell s)
    {
        _nbori = 0;
        _cdone = false;
        _cstat = StatusCheck.NoError;
        _odone = false;
        _ostat = StatusCheck.NoError;
        Init(s);
    }

    private static void Propagate(IndexedMap<Shape, NCollection_List<Shape>> mapEF,
                                  in Shape face,
                                  IndexedSet<Shape> mapF)
    {
        // Base for the traverse procedure.
        mapF.Add(face);

        // Perform well-known width-first traverse.
        for (int idx = 0; idx < mapF.Count; ++idx)
        {
            Shape aFace = mapF[idx];
            for (var ex = new Explorer(aFace, ShapeType.Edge); ex.More(); ex.Next())
            {
                Edge edg = ShapeCasts.Edge(ex.Current());
                // Test if the edge is in the map (only oriented edges are present).
                if (!mapEF.TryGetValue(edg, out NCollection_List<Shape>? list))
                {
                    continue;
                }

                for (var itl = new NCollection_List<Shape>.Iterator(list); itl.More(); itl.Next())
                {
                    // This code assumes that shape is added to an end of the map.
                    // The idea is simple: existing objects will not be added, new objects
                    // will be added to an end.
                    mapF.Add(itl.Value());
                }
            }
        }
    }

    private static bool IsOriented(in Shape s)
        => s.Orientation == Forward || s.Orientation == Reversed;

    /// <summary>The status list of the shell itself, read under the lock in parallel mode.</summary>
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

    /// <summary>Checks that the shell has faces and that they are connected through their edges.</summary>
    public override void Minimum()
    {
        _cdone = false;
        _odone = false;

        if (!_min)
        {
            var newList = new NCollection_List<StatusCheck>();
            NCollection_List<StatusCheck> lst = _map.SetValueRef(_shape, newList);

            // it is checked if the shell is "connected"
            var exp = new Explorer(_shape, ShapeType.Face);
            int nbface = 0;
            _mapEF.Clear();
            for (; exp.More(); exp.Next())
            {
                nbface++;
                var expe = new Explorer();
                for (expe.Init(exp.Current(), ShapeType.Edge); expe.More(); expe.Next())
                {
                    Shape edg = expe.Current();
                    int index = _mapEF.IndexOf(edg) + 1;
                    if (index == 0)
                    {
                        var thelist1 = new NCollection_List<Shape>();
                        index = _mapEF.Add(edg, thelist1) + 1;
                    }

                    _mapEF.ValueAt(index - 1).Append(exp.Current());
                }
            } // for (; exp.More(); exp.Next())

            if (nbface == 0)
            {
                BRepCheckCheck.Add(lst, StatusCheck.EmptyShell);
            }
            else if (nbface >= 2)
            {
                var mapF = new IndexedSet<Shape>(ShapeHasher.Instance);
                exp.ReInit();
                Propagate(_mapEF, exp.Current(), mapF);
                if (mapF.Count != nbface)
                {
                    BRepCheckCheck.Add(lst, StatusCheck.NotConnected);
                }
            } // else if (nbface >= 2)

            if (lst.IsEmpty())
            {
                lst.Append(StatusCheck.NoError);
            }

            _mapEF.Clear();
            _min = true;
        }
    }

    /// <summary>
    /// Checks the shell in a shape that contains it: in a solid, that it is closed and, unless it is unorientable,
    /// coherently oriented.
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

        Explorer exp = new(s, ShapeType.Shell);
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

        ShapeType styp = s.Type;
        switch (styp)
        {
            case ShapeType.Solid:
            {
                StatusCheck fst = Closed();
                if ((fst == StatusCheck.NotClosed && s.IsClosed) || (fst != StatusCheck.NoError))
                {
                    BRepCheckCheck.Add(lst, fst);
                }
                else if (!IsUnorientable())
                {
                    fst = Orientation();
                    BRepCheckCheck.Add(lst, fst);
                }
            }

                break;
            default:
                break;
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
            // nothing more than in the minimum
            _blind = true;
        }
    }

    /// <summary>
    /// Checks that the shell is closed: each oriented edge of its oriented faces is found twice (or once if
    /// degenerated). With <paramref name="update"/>, the status is added to <see cref="ResultCheck.Status"/>.
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

        _cdone = true; // it will be done...

        var itl = new NCollection_List<StatusCheck>.Iterator(statusList);
        if (itl.Value() != StatusCheck.NoError)
        {
            _cstat = itl.Value();
            return _cstat; // already saved
        }

        _cstat = StatusCheck.NoError;
        //
        int index, nbF;
        Explorer exp = new(), ede = new();
        var mapS = new IndexedSet<Shape>(ShapeHasher.Instance);
        var meToAvoid = new HashSet<Shape>(ShapeHasher.Instance);
        _mapEF.Clear();

        // Checks if the oriented faces of the shell give a "closed" shell,
        // i-e if each oriented edge on oriented faces is found 2 times.
        //
        exp.Init(_shape, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            Shape f = exp.Current();
            if (IsOriented(f))
            {
                ede.Init(exp.Current(), ShapeType.Edge);
                for (; ede.More(); ede.Next())
                {
                    Shape e = ede.Current();
                    if (!IsOriented(e))
                    {
                        meToAvoid.Add(e);
                    }
                }
            }
        }

        //
        exp.Init(_shape, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            Shape f = exp.Current();
            if (IsOriented(f))
            {
                // OCCT: NCollection_IndexedMap::Add returns the index (never 0), so !mapS.Add(aF) is always false
                // and a face present twice is never reported here (Orientation reports it)
                if (mapS.Add(f) + 1 == 0)
                {
                    _cstat = StatusCheck.RedundantFace;
                    if (update)
                    {
                        BRepCheckCheck.Add(statusList, _cstat);
                    }

                    return _cstat;
                }
                //
                ede.Init(exp.Current(), ShapeType.Edge);
                for (; ede.More(); ede.Next())
                {
                    Shape e = ede.Current();
                    if (!meToAvoid.Contains(e))
                    {
                        index = _mapEF.IndexOf(e) + 1;
                        if (index == 0)
                        {
                            var thelist = new NCollection_List<Shape>();
                            index = _mapEF.Add(e, thelist) + 1;
                        }

                        _mapEF.ValueAt(index - 1).Append(f);
                    }
                }
            }
        }

        //
        _nbori = mapS.Count;
        if (_nbori >= 2)
        {
            mapS.Clear();
            // Search for the first oriented face
            Shape f = new();
            exp.Init(_shape, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                f = exp.Current();
                if (IsOriented(f))
                {
                    break;
                }
            }

            Propagate(_mapEF, f, mapS);
        }

        //
        //
        nbF = mapS.Count;
        if (_nbori != nbF)
        {
            _cstat = StatusCheck.NotConnected;
            if (update)
            {
                BRepCheckCheck.Add(statusList, _cstat);
            }

            return _cstat;
        }

        //
        //
        int i, nbedges, nboc, nbSet;
        //
        nbedges = _mapEF.Count;
        for (i = 1; i <= nbedges; ++i)
        {
            nboc = _mapEF.ValueAt(i - 1).Extent();
            if (nboc == 0 || nboc >= 3)
            {
                var theSet = new NCollection_List<Shape>();
                nbSet = NbConnectedSetCore(theSet, false);
                // If there is more than one closed cavity the shell is considered invalid
                // this corresponds to the criteria of a solid (not those of a shell)
                if (nbSet > 1)
                {
                    _cstat = StatusCheck.InvalidMultiConnexity;
                    if (update)
                    {
                        BRepCheckCheck.Add(statusList, _cstat);
                    }

                    return _cstat;
                }
            }
            else if (nboc == 1)
            {
                if (!Tool.Degenerated(ShapeCasts.Edge(_mapEF.KeyAt(i - 1))))
                {
                    _cstat = StatusCheck.NotClosed;
                    if (update)
                    {
                        BRepCheckCheck.Add(statusList, _cstat);
                    }

                    return _cstat;
                }
            }
        }

        if (update)
        {
            BRepCheckCheck.Add(statusList, _cstat);
        }

        return _cstat;
    }

    /// <summary>
    /// Checks that the faces of the shell are coherently oriented (each edge shared by two faces is used in opposite
    /// orientations), and if not, whether the shell could be oriented. With <paramref name="update"/>, the status is
    /// added to <see cref="ResultCheck.Status"/>.
    /// </summary>
    public StatusCheck Orientation(bool update = false)
    {
        NCollection_List<StatusCheck> statusList = LockedStatusList();

        if (_odone)
        {
            if (update)
            {
                BRepCheckCheck.Add(statusList, _ostat);
            }

            return _ostat;
        }

        _odone = true;

        _ostat = Closed();
        if (_ostat != StatusCheck.NotClosed && _ostat != StatusCheck.NoError)
        {
            if (update)
            {
                BRepCheckCheck.Add(statusList, _ostat);
            }

            return _ostat;
        }

        _ostat = StatusCheck.NoError;

        // First the orientation of each face in relation to the shell is found.
        // It is used to check BRepCheck_RedundantFace
        var mapOfShapeOrientation = new Dictionary<Shape, int>(ShapeHasher.Instance);
        Explorer exp = new(), ede = new();

        for (exp.Init(_shape, ShapeType.Face); exp.More(); exp.Next())
        {
            if (!mapOfShapeOrientation.TryAdd(exp.Current(), (int)exp.Current().Orientation))
            {
                // OCCT's Bind replaces the value of a bound key
                mapOfShapeOrientation[exp.Current()] = (int)exp.Current().Orientation;
                _ostat = StatusCheck.RedundantFace;
                if (update)
                {
                    BRepCheckCheck.Add(statusList, _ostat);
                }
                else
                {
                    return _ostat;
                }
            }
        }

        // Then the orientation of faces by their connectivity is checked
        // BRepCheck_BadOrientationOfSubshape and
        //         BRepCheck_SubshapeNotInShape are checked;
        int nbedges = _mapEF.Count;
        Face fref = new();
        Orientation orf;

        for (int i = 0; i < nbedges; i++)
        {
            Edge edg = ShapeCasts.Edge(_mapEF.KeyAt(i));
            if (Tool.Degenerated(edg))
            {
                continue;
            }

            NCollection_List<Shape> lface = _mapEF.ValueAt(i);
            var lite = new NCollection_List<Shape>.Iterator(lface);

            if (lface.Extent() <= 2)
            {
                lite.Initialize(lface);
                fref = ShapeCasts.Face(lite.Value());

                if (!mapOfShapeOrientation.ContainsKey(fref))
                {
                    _ostat = StatusCheck.SubshapeNotInShape;
                    if (update)
                    {
                        BRepCheckCheck.Add(statusList, _ostat);
                    }

                    // quit because no workaround for the incoherence is possible
                    return _ostat;
                }

                lite.Next();

                if (lite.More()) // Edge of connectivity
                {
                    int iorf = mapOfShapeOrientation.GetValue(fref);
                    orf = (Orientation)iorf;
                    fref.Orient(orf);

                    // edge is examined
                    if (!lite.Value().IsSame(fref)) // edge non "closed"
                    {
                        for (ede.Init(fref, ShapeType.Edge); ede.More(); ede.Next())
                        {
                            if (ede.Current().IsSame(edg))
                            {
                                break;
                            }
                        }

                        Orientation orient = ede.Current().Orientation;
                        Face fcur = ShapeCasts.Face(lite.Value());

                        if (!mapOfShapeOrientation.ContainsKey(fcur))
                        {
                            _ostat = StatusCheck.SubshapeNotInShape;
                            if (update)
                            {
                                BRepCheckCheck.Add(statusList, _ostat);
                            }

                            // quit because no workaround for the incoherence is possible
                            return _ostat;
                        }

                        int oriFCur = mapOfShapeOrientation.GetValue(fcur);
                        orf = (Orientation)oriFCur;
                        fcur.Orient(orf);

                        for (ede.Init(fcur, ShapeType.Edge); ede.More(); ede.Next())
                        {
                            if (ede.Current().IsSame(edg))
                            {
                                break;
                            }
                        }

                        if (ede.Current().Orientation == orient)
                        {
                            // The loop is continued on the edges as many times
                            // as the same edge is present in the wire
                            bool bfound = false;
                            ede.Next();
                            for (; ede.More(); ede.Next())
                            {
                                if (ede.Current().IsSame(edg))
                                {
                                    bfound = true;
                                    break;
                                }
                            }

                            if (!bfound || (ede.Current().Orientation == orient))
                            {
                                _ostat = StatusCheck.BadOrientationOfSubshape;
                                if (update)
                                {
                                    BRepCheckCheck.Add(statusList, _ostat);
                                    break;
                                }

                                return _ostat;
                            }
                        }
                    }
                }
            }
            else // more than two faces
            {
                int numF = 0, numR = 0;
                var fmap = new HashSet<Shape>(ShapeHasher.Instance);

                for (lite.Initialize(lface); lite.More(); lite.Next())
                {
                    Face fcur = ShapeCasts.Face(lite.Value());
                    if (!mapOfShapeOrientation.ContainsKey(fcur))
                    {
                        _ostat = StatusCheck.SubshapeNotInShape;
                        if (update)
                        {
                            BRepCheckCheck.Add(statusList, _ostat);
                        }

                        // quit because no workaround for the incoherence is possible
                        return _ostat;
                    }

                    int iorf = mapOfShapeOrientation.GetValue(fcur);
                    orf = (Orientation)iorf;
                    fcur.Orient(orf);

                    for (ede.Init(fcur, ShapeType.Edge); ede.More(); ede.Next())
                    {
                        if (ede.Current().IsSame(edg))
                        {
                            break;
                        }
                    }

                    if (fmap.Contains(fcur)) // edge is "closed" on Fcur, we meet Fcur twice
                    {
                        ede.Next();
                        for (; ede.More(); ede.Next())
                        {
                            if (ede.Current().IsSame(edg))
                            {
                                break;
                            }
                        }
                    }

                    Orientation orient = ede.Current().Orientation;
                    if (orient == Forward)
                    {
                        numF++;
                    }
                    else
                    {
                        numR++;
                    }

                    fmap.Add(fcur);
                }

                if (numF != numR)
                {
                    _ostat = StatusCheck.BadOrientationOfSubshape;
                    if (update)
                    {
                        BRepCheckCheck.Add(statusList, _ostat);
                        break;
                    }

                    return _ostat;
                }
            }
        }

        // If at least one incorrectly oriented face has been found, it is checked if the shell can be oriented.
        //          i.e. : if by modification of the orientation of a face it is possible to find
        //          a coherent orientation. (it is not possible on a Moebius band)
        //          BRepCheck_UnorientableShape is checked
        if (_ostat == StatusCheck.BadOrientationOfSubshape)
        {
            if (!fref.IsNull)
            {
                if (nbedges > 0)
                {
                    var alre = new HashSet<Shape>(ShapeHasher.Instance);
                    var voisin = new NCollection_List<Shape>();
                    voisin.Append(fref);
                    alre.Clear();
                    while (!voisin.IsEmpty())
                    {
                        fref = ShapeCasts.Face(voisin.First());
                        voisin.RemoveFirst();
                        if (!mapOfShapeOrientation.ContainsKey(fref))
                        {
                            _ostat = StatusCheck.SubshapeNotInShape;
                            if (update)
                            {
                                BRepCheckCheck.Add(statusList, _ostat);
                            }

                            // quit because no workaround for the incoherence is possible
                            return _ostat;
                        }

                        int iorf = mapOfShapeOrientation.GetValue(fref);
                        orf = (Orientation)iorf;
                        fref.Orient(orf);

                        var edFcur = new Explorer();
                        alre.Add(fref);

                        for (ede.Init(fref, ShapeType.Edge); ede.More(); ede.Next())
                        {
                            Edge edg = ShapeCasts.Edge(ede.Current());
                            Orientation orient = edg.Orientation;
                            NCollection_List<Shape> lface = _mapEF.GetValue(edg);
                            var lite = new NCollection_List<Shape>.Iterator(lface);

                            Face fcur = ShapeCasts.Face(lite.Value());
                            if (fcur.IsSame(fref))
                            {
                                lite.Next();
                                if (lite.More())
                                {
                                    fcur = ShapeCasts.Face(lite.Value());
                                }
                                else
                                {
                                    // from the free border one goes to the next edge
                                    continue;
                                }
                            }

                            if (!mapOfShapeOrientation.ContainsKey(fcur))
                            {
                                _ostat = StatusCheck.SubshapeNotInShape;
                                if (update)
                                {
                                    BRepCheckCheck.Add(statusList, _ostat);
                                }

                                // quit because no workaround for the incoherence is possible
                                return _ostat;
                            }

                            int oriFCur = mapOfShapeOrientation.GetValue(fcur);
                            orf = (Orientation)oriFCur;
                            fcur.Orient(orf);

                            for (edFcur.Init(fcur, ShapeType.Edge); edFcur.More(); edFcur.Next())
                            {
                                if (edFcur.Current().IsSame(edg))
                                {
                                    break;
                                }
                            }

                            if (edFcur.Current().Orientation == orient)
                            {
                                if (alre.Contains(fcur))
                                {
                                    // It is necessary to return a face that has been already examined or returned
                                    // if one gets nowhere, the shell cannot be oriented.
                                    _ostat = StatusCheck.UnorientableShape;
                                    if (update)
                                    {
                                        BRepCheckCheck.Add(statusList, _ostat);
                                    }

                                    // quit, otherwise there is a risk of taking too much time.
                                    return _ostat;
                                }

                                orf = global::Kerf.Engine.Topology.TopAbs.Reverse(orf);
                                mapOfShapeOrientation.GetValueRef(fcur) = (int)orf;
                            }

                            if (alre.Add(fcur))
                            {
                                voisin.Append(fcur);
                            }
                        }
                    }
                }
            }
        }

        if (update)
        {
            BRepCheckCheck.Add(statusList, _ostat);
        }

        return _ostat;
    }

    /// <summary>Marks the shell as unorientable.</summary>
    public void SetUnorientable()
    {
        bool locked = _isParallel;
        if (locked)
        {
            _mutex.Enter();
        }

        try
        {
            BRepCheckCheck.Add(_map.GetValue(_shape), StatusCheck.UnorientableShape);
        }
        finally
        {
            if (locked)
            {
                _mutex.Exit();
            }
        }
    }

    /// <summary>True if the shell is marked unorientable (or, once checked, isn't coherently oriented).</summary>
    public bool IsUnorientable()
    {
        if (_odone)
        {
            return _ostat != StatusCheck.NoError;
        }

        NCollection_List<StatusCheck> statusList = LockedStatusList();
        for (var itl = new NCollection_List<StatusCheck>.Iterator(statusList); itl.More(); itl.Next())
        {
            if (itl.Value() == StatusCheck.UnorientableShape)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// The sets of faces connected through simple edges, starting from the edges shared by more than two faces, as
    /// new shells appended to <paramref name="sets"/>; returns the number of shells in the list.
    /// </summary>
    /// <remarks>
    /// The shells are frozen (they are published, phase3-design.md 4.2), and with them any unfrozen face of the
    /// checked shell. The sets are found from the multiconnected edges in hash order, so the order of the shells may
    /// differ from OCCT's (decision 18).
    /// </remarks>
    public int NbConnectedSet(NCollection_List<Shape> sets) => NbConnectedSetCore(sets, true);

    /// <summary>NbConnectedSet, with the shells unfrozen for <see cref="Closed"/>, which only counts them.</summary>
    private int NbConnectedSetCore(NCollection_List<Shape> sets, bool toFreeze)
    {
        // The connections are found
        var parents = new IndexedMap<Shape, NCollection_List<Shape>>(
            ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(_shape, ShapeType.Edge, ShapeType.Face, parents);
        // All faces are taken
        var theFaces = new HashSet<Shape>(ShapeHasher.Instance);
        var exsh = new Explorer(_shape, ShapeType.Face);
        for (; exsh.More(); exsh.Next())
        {
            theFaces.Add(exsh.Current());
        }

        // The edges that are not oriented or have more than 2 connections are missing
        int iCur;
        var theMultiEd = new HashSet<Shape>(ShapeHasher.Instance);
        var theUnOriEd = new HashSet<Shape>(ShapeHasher.Instance);
        for (iCur = 1; iCur <= parents.Count; iCur++)
        {
            Edge ed = ShapeCasts.Edge(parents.KeyAt(iCur - 1));
            if (parents.ValueAt(iCur - 1).Extent() > 2)
            {
                theMultiEd.Add(ed);
            }

            if (ed.Orientation != Reversed && ed.Orientation != Forward)
            {
                theUnOriEd.Add(ed);
            }
        }

        // Starting from multiconnected edges propagation by simple connections
        var lconx1 = new NCollection_List<Shape>.Iterator();
        var lconx2 = new NCollection_List<Shape>.Iterator();
        Shell curShell = new();
        Shape adFac;
        var lesCur = new NCollection_List<Shape>();
        var brb = new BrepBuilder();
        bool newCur = true;
        brb.MakeShell(ref curShell);
        foreach (Shape ed in theMultiEd)
        {
            if (!theUnOriEd.Contains(ed))
            {
                for (lconx1.Initialize(parents.GetValue(ed)); lconx1.More(); lconx1.Next())
                {
                    if (theFaces.Contains(lconx1.Value()))
                    {
                        adFac = lconx1.Value();
                        brb.Add(curShell, adFac);
                        theFaces.Remove(adFac);
                        newCur = false;
                        if (theFaces.Count == 0)
                        {
                            // OCCT: leaves before the shell is appended, so a last set of one face is lost (occt-issues.md)
                            break;
                        }

                        lesCur.Append(adFac);
                        while (!lesCur.IsEmpty())
                        {
                            adFac = lesCur.First();
                            lesCur.RemoveFirst();
                            for (exsh.Init(adFac, ShapeType.Edge); exsh.More(); exsh.Next())
                            {
                                Shape ced = exsh.Current();
                                if (!theMultiEd.Contains(ced))
                                {
                                    for (lconx2.Initialize(parents.GetValue(ced)); lconx2.More(); lconx2.Next())
                                    {
                                        if (theFaces.Contains(lconx2.Value()))
                                        {
                                            adFac = lconx2.Value();
                                            brb.Add(curShell, adFac);
                                            theFaces.Remove(adFac);
                                            newCur = false;
                                            if (theFaces.Count == 0)
                                            {
                                                break;
                                            }

                                            lesCur.Append(adFac);
                                        }
                                    }
                                }

                                if (theFaces.Count == 0)
                                {
                                    break;
                                }
                            }
                        }

                        if (!newCur)
                        {
                            curShell.Closed(Tool.IsClosed(curShell));
                            if (toFreeze)
                            {
                                // OCCT.NET: the shell is published to the caller (phase3-design.md 4.2)
                                curShell.TShape()!.FreezeGraph();
                            }

                            sets.Append(curShell);
                            curShell.Nullify();
                            newCur = true;
                            brb.MakeShell(ref curShell);
                        }
                    }

                    if (theFaces.Count == 0)
                    {
                        break;
                    }
                }
            }

            if (theFaces.Count == 0)
            {
                break;
            }
        }

        return sets.Extent();
    }
}
