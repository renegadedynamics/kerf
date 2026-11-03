// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_FaceRestrictor.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_FaceRestrictor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Build;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans.Algo;

/// <summary>
/// Builds the faces bounded by a set of closed, non-intersecting wires on the surface of a face: each outer wire makes
/// a face with the wires inside it.
/// </summary>
/// <remarks>
/// <para>
/// Without orientation control (the default), the wires must be correctly oriented and the faces are made by
/// <see cref="WireToFace"/>. With it, the closed wires are first oriented so that no face is infinite,
/// then nested by classification.
/// </para>
/// <para>
/// Edges without a pcurve on the surface get one (the projection of their 3D curve with <c>Proj</c>, else their first
/// pcurve placed on the surface), and the wires are flagged closed: OCCT writes into the caller's wires.
/// <see cref="Perform"/> works on a copy of frozen wires (<see cref="CopyAtEntry"/>) and gives the
/// caller's wires and edges back wherever nothing was written (on a plane with closed-flagged wires, nothing is), so
/// the faces hold the caller's own wires as native's do (decision 793). <see cref="Current"/> hands out frozen faces;
/// <see cref="InternalCurrent"/> doesn't, for ported callers.
/// </para>
/// </remarks>
public sealed class FaceRestrictor
{
    private bool _done;
    private bool _modeProj;
    private Face _face = new();
    private readonly NCollection_List<Shape> _wires = new();
    private readonly NCollection_List<Shape> _faces = new();
    private bool _correction;
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> _keyIsIn = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> _keyContains = new(ShapeHasher.Instance);

    /// <summary>An empty face restrictor.</summary>
    public FaceRestrictor()
    {
        // OCCT: the flags are left uninitialised until Init (false here)
    }

    /// <summary>
    /// Sets the face <paramref name="f"/> whose surface the new faces are on; <paramref name="proj"/>: missing pcurves
    /// are projections of the 3D curves; <paramref name="correctionOrientation"/>: the wires are oriented and nested
    /// by classification.
    /// </summary>
    public void Init(in Face f, bool proj = false, bool correctionOrientation = false)
    {
        _face = f;
        _modeProj = proj;
        _correction = correctionOrientation;
    }

    /// <summary>Adds the (closed) wire <paramref name="w"/>.</summary>
    public void Add(in Wire w)
    {
        _wires.Append(w);
    }

    /// <summary>Removes the wires and the faces.</summary>
    public void Clear()
    {
        _wires.Clear();
        _faces.Clear();
    }

    // Puts on S the edge's first pcurve; true if it has none
    private static bool ChangePCurve(in Edge e, Surface s, in Placement L)
    {
        BrepBuilder bb = new();
        Curve2D? c2 = null;
        Surface? se = null;
        Placement le = new();
        double f = 0.0, l = 0.0;
        Tool.CurveOnSurface(e, ref c2, ref se, ref le, ref f, ref l, 1);
        if (c2 is not null)
        {
            bb.UpdateEdge(e, c2, s, L, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        return c2 is null;
    }

    // Puts on S the projection of the edge's 3D curve; false if it has none
    private static bool ProjCurve3d(in Edge e, Surface s, in Placement L)
    {
        BrepBuilder bb = new();
        Curve3D? c = Tool.Curve(e, out Placement le, out double f, out double l);
        if (c is null)
        {
            return false;
        }

        TrimmedCurve3D ct = new(c, f, l);

        Placement ll = L.Inverted().Multiplied(le);
        ct.Transform(ll.Transformation());

        Curve2D? c2 = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(ct, s);
        bb.UpdateEdge(e, c2, s, L, global::Kerf.Engine.Foundation.Precision.Confusion());
        return true;
    }

    /// <summary>Builds the faces bounded by the wires.</summary>
    public void Perform()
    {
        // OCCT.NET: copy at entry (decision 793)
        Shape holder = WireToFace.HolderOf(_wires);
        CopyAtEntry entry = new(holder);
        if (!entry.IsCopied)
        {
            InternalPerform();
            return;
        }
        NCollection_List<Shape> saved = new(_wires);
        _wires.Clear();
        for (NCollection_List<Shape>.Iterator it = new(saved); it.More(); it.Next())
        {
            _wires.Append(entry.Copied(it.Value()));
        }
        try
        {
            InternalPerform();
        }
        finally
        {
            // the caller's wires, with the orientations the correction mode gave their copies
            NCollection_List<Shape>.Iterator itc = new(_wires);
            for (NCollection_List<Shape>.Iterator it = new(saved); it.More() && itc.More(); it.Next(), itc.Next())
            {
                it.ChangeValue() = it.Value().Oriented(itc.Value().Orientation());
            }
            _wires.Assign(saved);
        }
        entry.Finish(new Shape(), _faces);
    }

    /// <summary>OCCT's <c>Perform</c>, in place, for ported callers whose wires are unfrozen.</summary>
    internal void InternalPerform()
    {
        if (_correction)
        {
            PerformWithCorrection();
            return;
        }

        _done = false;
        NCollection_List<Shape>.Iterator it = new(_wires);

        //--------------------------------------------------------------------
        // return geometry of the reference face.
        //--------------------------------------------------------------------
        Surface s = Tool.Surface(_face, out Placement l)!;

        //-----------------------------------------------------------------------
        // test if edge is on S. otherwise  add S to the first pcurve.
        // or projection of the edge on F.
        //----------------------------------------------------------------------
        Explorer exp = new();

        WireToFace wtf = new();

        for (; it.More(); it.Next())
        {
            // update the surface on edges.
            Wire w = ShapeCasts.Wire(it.Value());

            for (exp.Init(w, ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());
                Curve2D? c2 = Tool.CurveOnSurface(e, s, l, out _, out _);
                if (c2 is null)
                {
                    // the edge is not on S
                    if (_modeProj)
                    {
                        // Projection of 3d curve on S
                        if (!ProjCurve3d(e, s, l))
                        {
                            return;
                        }
                    }
                    else
                    {
                        // Displacement of the 1st pcurve on S
                        // OCCT: ChangePCurve returns true when the edge has NO pcurve, so after moving one this
                        // also projects the 3D curve, and fails (without having made a face) if there is none
                        bool yaPCurve = ChangePCurve(e, s, l);
                        if (!yaPCurve)
                        {
                            if (!ProjCurve3d(e, s, l))
                            {
                                return;
                            }
                        }
                    }
                }
            }
            wtf.AddWire(w);
        }

        wtf.InternalMakeFaces(_face, _faces);

        _done = true;
    }

    /// <summary>Whether the faces are built.</summary>
    public bool IsDone => _done;

    /// <summary>Whether there is a current face.</summary>
    public bool More() => !_faces.IsEmpty();

    /// <summary>Moves to the next face.</summary>
    public void Next()
    {
        _faces.RemoveFirst();
    }

    /// <summary>The current face (frozen).</summary>
    public Face Current()
    {
        Face face = InternalCurrent();
        face.TShape()?.FreezeGraph();
        return face;
    }

    /// <summary>The current face, unfrozen, for ported callers that go on building on it.</summary>
    internal Face InternalCurrent() => ShapeCasts.Face(_faces.First());

    // Whether the wire is closed (flag or same end vertices)
    private static bool IsClosed(in Wire w)
    {
        if (w.Closed())
        {
            return true;
        }
        Exploration.Vertices(w, out Vertex v1, out Vertex v2);
        return v1.IsSame(v2);
    }

    // Whether a point of the first edge of wir is inside F
    private static bool IsInside(in Wire wir, in Face F)
    {
        Explorer exp = new();
        exp.Init(wir, ShapeType.Edge);
        if (exp.More())
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            // OCCT: dereferences the pcurve unchecked (native crashes on an edge without one on F; the port throws)
            Curve2D c2d = Tool.CurveOnSurface(edg, F, out double f, out double l)!;
            double prm;

            if (!global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f) && !global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l))
            {
                prm = (f + l) / 2.0;
            }
            else
            {
                if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f) && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l))
                {
                    prm = 0.0;
                }
                else if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f))
                {
                    prm = l - 1.0;
                }
                else
                {
                    prm = f + 1.0;
                }
            }

            Point2D pt2d = c2d.Value(prm);
            FClass2d fClass2d = new(F, global::Kerf.Engine.Foundation.Precision.PConfusion());
            Containment st2 = fClass2d.Perform(pt2d, false);
            return st2 == In;
        }
        return false;
    }

    private static void Store(in Wire w2,
                              in Wire w1,
                              NCollection_DataMap<Shape, NCollection_List<Shape>> keyIsIn,
                              NCollection_DataMap<Shape, NCollection_List<Shape>> keyContains)
    {
        if (!keyIsIn.IsBound(w2))
        {
            NCollection_List<Shape> empty = new();
            keyIsIn.Bind(w2, empty);
        }
        keyIsIn[w2].Append(w1);

        if (!keyContains.IsBound(w1))
        {
            NCollection_List<Shape> empty = new();
            keyContains.Bind(w1, empty);
        }
        keyContains[w1].Append(w2);
    }

    private static void BuildFaceIn(in Face f,
                                    in Wire w,
                                    NCollection_DataMap<Shape, NCollection_List<Shape>> keyContains,
                                    NCollection_DataMap<Shape, NCollection_List<Shape>> keyIsIn,
                                    Orientation orientation,
                                    NCollection_List<Shape> faces)
    {
        BrepBuilder b = new();

        if (!keyContains.IsBound(w) || keyContains[w].IsEmpty())
        {
            return;
        }

        // Removal of W in KeyIsIn.
        NCollection_List<Shape>.Iterator it = new();
        for (it.Initialize(keyContains[w]); it.More(); it.Next())
        {
            Wire wi = ShapeCasts.Wire(it.Value());
            NCollection_List<Shape> l2 = keyIsIn[wi];
            NCollection_List<Shape>.Iterator it2 = new();
            for (it2.Initialize(l2); it2.More(); it2.Next())
            {
                if (it2.Value().IsSame(w))
                {
                    l2.Remove(it2);
                    break;
                }
            }
        }

        NCollection_List<Shape> wireExt = new();

        for (it.Initialize(keyContains[w]); it.More(); it.Next())
        {
            Wire wi = ShapeCasts.Wire(it.Value());
            NCollection_List<Shape> l2 = keyIsIn[wi];

            if (l2.IsEmpty())
            {
                wireExt.Append(wi);
            }
        }

        for (it.Initialize(wireExt); it.More(); it.Next())
        {
            Wire wi = ShapeCasts.Wire(it.Value());
            NCollection_List<Shape> l2 = keyIsIn[wi];
            if (l2.IsEmpty())
            {
                if (orientation == Forward)
                {
                    Wire nwi = wi;
                    nwi.Reverse();
                    b.Add(f, nwi);
                    BuildFaceIn(f, wi, keyContains, keyIsIn, Reversed, faces);
                }
                else
                {
                    Shape localShape = faces.First().EmptyCopied();
                    Face nf = ShapeCasts.Face(localShape);
                    b.Add(nf, wi);
                    faces.Append(nf);
                    BuildFaceIn(nf, wi, keyContains, keyIsIn, Forward, faces);
                }
            }
        }
    }

    // Perform with the closed wires oriented so that no face is infinite, and nested by classification
    private void PerformWithCorrection()
    {
        BrepBuilder b = new();

        _done = false;
        NCollection_List<Shape>.Iterator it = new(_wires);
        //---------------------------------------------------------
        // Reorientation of all closed wires to the left.
        //---------------------------------------------------------
        for (; it.More(); it.Next())
        {
            ref Shape w = ref it.ChangeValue();
            Shape localShape = _face.EmptyCopied();
            Face nf = ShapeCasts.Face(localShape);
            nf.Orientation(Forward);
            b.Add(nf, w);

            if (IsClosed(ShapeCasts.Wire(w)))
            {
                FClass2d fClass2d = new(nf, global::Kerf.Engine.Foundation.Precision.PConfusion());
                if (fClass2d.PerformInfinitePoint() != Out)
                {
                    w.Reverse();
                }
            }
        }
        //---------------------------------------------------------
        // Classification of wires ones compared to the others.
        //---------------------------------------------------------
        for (it.Initialize(_wires); it.More(); it.Next())
        {
            Wire w1 = ShapeCasts.Wire(it.Value());
            NCollection_List<Shape>.Iterator it2 = new(_wires);

            if (IsClosed(w1))
            {
                Shape localShape = _face.EmptyCopied();
                Face nf = ShapeCasts.Face(localShape);
                nf.Orientation(Forward);
                b.Add(nf, w1);

                while (it2.More())
                {
                    Wire w2 = ShapeCasts.Wire(it2.Value());
                    if (!w1.IsSame(w2) && IsInside(w2, nf))
                    {
                        Store(w2, w1, _keyIsIn, _keyContains);
                    }
                    it2.Next();
                }
            }
        }
        NCollection_List<Shape> wireExt = new();

        for (it.Initialize(_wires); it.More(); it.Next())
        {
            Wire w = ShapeCasts.Wire(it.Value());
            if (!_keyIsIn.IsBound(w) || _keyIsIn[w].IsEmpty())
            {
                wireExt.Append(w);
            }
        }

        for (it.Initialize(wireExt); it.More(); it.Next())
        {
            Wire w = ShapeCasts.Wire(it.Value());
            if (!_keyIsIn.IsBound(w) || _keyIsIn[w].IsEmpty())
            {
                Shape localShape = _face.EmptyCopied();
                Face newFace = ShapeCasts.Face(localShape);
                newFace.Orientation(Forward);
                b.Add(newFace, w);
                _faces.Append(newFace);
                //--------------------------------------------
                // Construction of a face by exterior wire.
                //--------------------------------------------
                BuildFaceIn(newFace, w, _keyContains, _keyIsIn, Forward, _faces);
            }
        }
        _done = true;
    }
}
