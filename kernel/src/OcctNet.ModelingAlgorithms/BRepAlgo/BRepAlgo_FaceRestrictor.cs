// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_FaceRestrictor.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_FaceRestrictor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// Builds the faces bounded by a set of closed, non-intersecting wires on the surface of a face: each outer wire makes
/// a face with the wires inside it.
/// </summary>
/// <remarks>
/// <para>
/// Without orientation control (the default), the wires must be correctly oriented and the faces are made by
/// <see cref="TopOpeBRepBuild_WireToFace"/>. With it, the closed wires are first oriented so that no face is infinite,
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
public sealed class BRepAlgo_FaceRestrictor
{
    private bool myDone;
    private bool modeProj;
    private Face myFace = new();
    private readonly NCollection_List<Shape> wires = new();
    private readonly NCollection_List<Shape> faces = new();
    private bool myCorrection;
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> keyIsIn = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> keyContains = new(ShapeHasher.Instance);

    /// <summary>An empty face restrictor.</summary>
    public BRepAlgo_FaceRestrictor()
    {
        // OCCT: the flags are left uninitialised until Init (false here)
    }

    /// <summary>
    /// Sets the face <paramref name="F"/> whose surface the new faces are on; <paramref name="Proj"/>: missing pcurves
    /// are projections of the 3D curves; <paramref name="CorrectionOrientation"/>: the wires are oriented and nested
    /// by classification.
    /// </summary>
    public void Init(in Face F, bool Proj = false, bool CorrectionOrientation = false)
    {
        myFace = F;
        modeProj = Proj;
        myCorrection = CorrectionOrientation;
    }

    /// <summary>Adds the (closed) wire <paramref name="W"/>.</summary>
    public void Add(in Wire W)
    {
        wires.Append(W);
    }

    /// <summary>Removes the wires and the faces.</summary>
    public void Clear()
    {
        wires.Clear();
        faces.Clear();
    }

    // Puts on S the edge's first pcurve; true if it has none
    private static bool ChangePCurve(in Edge E, Surface S, in Placement L)
    {
        BrepBuilder BB = new();
        Curve2D? C2 = null;
        Surface? SE = null;
        Placement LE = new();
        double f = 0.0, l = 0.0;
        Tool.CurveOnSurface(E, ref C2, ref SE, ref LE, ref f, ref l, 1);
        if (C2 is not null)
        {
            BB.UpdateEdge(E, C2, S, L, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        return C2 is null;
    }

    // Puts on S the projection of the edge's 3D curve; false if it has none
    private static bool ProjCurve3d(in Edge E, Surface S, in Placement L)
    {
        BrepBuilder BB = new();
        Curve3D? C = Tool.Curve(E, out Placement LE, out double f, out double l);
        if (C is null)
        {
            return false;
        }

        TrimmedCurve3D CT = new(C, f, l);

        Placement LL = L.Inverted().Multiplied(LE);
        CT.Transform(LL.Transformation());

        Curve2D? C2 = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(CT, S);
        BB.UpdateEdge(E, C2, S, L, global::Kerf.Engine.Foundation.Precision.Confusion());
        return true;
    }

    /// <summary>Builds the faces bounded by the wires.</summary>
    public void Perform()
    {
        // OCCT.NET: copy at entry (decision 793)
        Shape aHolder = TopOpeBRepBuild_WireToFace.HolderOf(wires);
        CopyAtEntry anEntry = new(aHolder);
        if (!anEntry.IsCopied)
        {
            InternalPerform();
            return;
        }
        NCollection_List<Shape> aSaved = new(wires);
        wires.Clear();
        for (NCollection_List<Shape>.Iterator it = new(aSaved); it.More(); it.Next())
        {
            wires.Append(anEntry.Copied(it.Value()));
        }
        try
        {
            InternalPerform();
        }
        finally
        {
            // the caller's wires, with the orientations the correction mode gave their copies
            NCollection_List<Shape>.Iterator itc = new(wires);
            for (NCollection_List<Shape>.Iterator it = new(aSaved); it.More() && itc.More(); it.Next(), itc.Next())
            {
                it.ChangeValue() = it.Value().Oriented(itc.Value().Orientation());
            }
            wires.Assign(aSaved);
        }
        anEntry.Finish(new Shape(), faces);
    }

    /// <summary>OCCT's <c>Perform</c>, in place, for ported callers whose wires are unfrozen.</summary>
    internal void InternalPerform()
    {
        if (myCorrection)
        {
            PerformWithCorrection();
            return;
        }

        myDone = false;
        NCollection_List<Shape>.Iterator it = new(wires);

        //--------------------------------------------------------------------
        // return geometry of the reference face.
        //--------------------------------------------------------------------
        Surface S = Tool.Surface(myFace, out Placement L)!;

        //-----------------------------------------------------------------------
        // test if edge is on S. otherwise  add S to the first pcurve.
        // or projection of the edge on F.
        //----------------------------------------------------------------------
        Explorer Exp = new();

        TopOpeBRepBuild_WireToFace WTF = new();

        for (; it.More(); it.Next())
        {
            // update the surface on edges.
            Wire W = ShapeCasts.Wire(it.Value());

            for (Exp.Init(W, ShapeType.Edge); Exp.More(); Exp.Next())
            {
                Edge E = ShapeCasts.Edge(Exp.Current());
                Curve2D? C2 = Tool.CurveOnSurface(E, S, L, out _, out _);
                if (C2 is null)
                {
                    // the edge is not on S
                    if (modeProj)
                    {
                        // Projection of 3d curve on S
                        if (!ProjCurve3d(E, S, L))
                        {
                            return;
                        }
                    }
                    else
                    {
                        // Displacement of the 1st pcurve on S
                        // OCCT: ChangePCurve returns true when the edge has NO pcurve, so after moving one this
                        // also projects the 3D curve, and fails (without having made a face) if there is none
                        bool YaPCurve = ChangePCurve(E, S, L);
                        if (!YaPCurve)
                        {
                            if (!ProjCurve3d(E, S, L))
                            {
                                return;
                            }
                        }
                    }
                }
            }
            WTF.AddWire(W);
        }

        WTF.InternalMakeFaces(myFace, faces);

        myDone = true;
    }

    /// <summary>Whether the faces are built.</summary>
    public bool IsDone() => myDone;

    /// <summary>Whether there is a current face.</summary>
    public bool More() => !faces.IsEmpty();

    /// <summary>Moves to the next face.</summary>
    public void Next()
    {
        faces.RemoveFirst();
    }

    /// <summary>The current face (frozen).</summary>
    public Face Current()
    {
        Face aFace = InternalCurrent();
        aFace.TShape()?.FreezeGraph();
        return aFace;
    }

    /// <summary>The current face, unfrozen, for ported callers that go on building on it.</summary>
    internal Face InternalCurrent() => ShapeCasts.Face(faces.First());

    // Whether the wire is closed (flag or same end vertices)
    private static bool IsClosed(in Wire W)
    {
        if (W.Closed())
        {
            return true;
        }
        Exploration.Vertices(W, out Vertex V1, out Vertex V2);
        return V1.IsSame(V2);
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
            Curve2D C2d = Tool.CurveOnSurface(edg, F, out double f, out double l)!;
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

            Point2D pt2d = C2d.Value(prm);
            BRepTopAdaptor_FClass2d FClass2d = new(F, global::Kerf.Engine.Foundation.Precision.PConfusion());
            Containment st2 = FClass2d.Perform(pt2d, false);
            return st2 == In;
        }
        return false;
    }

    private static void Store(in Wire W2,
                              in Wire W1,
                              NCollection_DataMap<Shape, NCollection_List<Shape>> keyIsIn,
                              NCollection_DataMap<Shape, NCollection_List<Shape>> keyContains)
    {
        if (!keyIsIn.IsBound(W2))
        {
            NCollection_List<Shape> empty = new();
            keyIsIn.Bind(W2, empty);
        }
        keyIsIn[W2].Append(W1);

        if (!keyContains.IsBound(W1))
        {
            NCollection_List<Shape> empty = new();
            keyContains.Bind(W1, empty);
        }
        keyContains[W1].Append(W2);
    }

    private static void BuildFaceIn(in Face F,
                                    in Wire W,
                                    NCollection_DataMap<Shape, NCollection_List<Shape>> KeyContains,
                                    NCollection_DataMap<Shape, NCollection_List<Shape>> KeyIsIn,
                                    Orientation Orientation,
                                    NCollection_List<Shape> Faces)
    {
        BrepBuilder B = new();

        if (!KeyContains.IsBound(W) || KeyContains[W].IsEmpty())
        {
            return;
        }

        // Removal of W in KeyIsIn.
        NCollection_List<Shape>.Iterator it = new();
        for (it.Initialize(KeyContains[W]); it.More(); it.Next())
        {
            Wire WI = ShapeCasts.Wire(it.Value());
            NCollection_List<Shape> L2 = KeyIsIn[WI];
            NCollection_List<Shape>.Iterator it2 = new();
            for (it2.Initialize(L2); it2.More(); it2.Next())
            {
                if (it2.Value().IsSame(W))
                {
                    L2.Remove(it2);
                    break;
                }
            }
        }

        NCollection_List<Shape> WireExt = new();

        for (it.Initialize(KeyContains[W]); it.More(); it.Next())
        {
            Wire WI = ShapeCasts.Wire(it.Value());
            NCollection_List<Shape> L2 = KeyIsIn[WI];

            if (L2.IsEmpty())
            {
                WireExt.Append(WI);
            }
        }

        for (it.Initialize(WireExt); it.More(); it.Next())
        {
            Wire WI = ShapeCasts.Wire(it.Value());
            NCollection_List<Shape> L2 = KeyIsIn[WI];
            if (L2.IsEmpty())
            {
                if (Orientation == Forward)
                {
                    Wire NWI = WI;
                    NWI.Reverse();
                    B.Add(F, NWI);
                    BuildFaceIn(F, WI, KeyContains, KeyIsIn, Reversed, Faces);
                }
                else
                {
                    Shape aLocalShape = Faces.First().EmptyCopied();
                    Face NF = ShapeCasts.Face(aLocalShape);
                    B.Add(NF, WI);
                    Faces.Append(NF);
                    BuildFaceIn(NF, WI, KeyContains, KeyIsIn, Forward, Faces);
                }
            }
        }
    }

    // Perform with the closed wires oriented so that no face is infinite, and nested by classification
    private void PerformWithCorrection()
    {
        BrepBuilder B = new();

        myDone = false;
        NCollection_List<Shape>.Iterator it = new(wires);
        //---------------------------------------------------------
        // Reorientation of all closed wires to the left.
        //---------------------------------------------------------
        for (; it.More(); it.Next())
        {
            ref Shape W = ref it.ChangeValue();
            Shape aLocalShape = myFace.EmptyCopied();
            Face NF = ShapeCasts.Face(aLocalShape);
            NF.Orientation(Forward);
            B.Add(NF, W);

            if (IsClosed(ShapeCasts.Wire(W)))
            {
                BRepTopAdaptor_FClass2d FClass2d = new(NF, global::Kerf.Engine.Foundation.Precision.PConfusion());
                if (FClass2d.PerformInfinitePoint() != Out)
                {
                    W.Reverse();
                }
            }
        }
        //---------------------------------------------------------
        // Classification of wires ones compared to the others.
        //---------------------------------------------------------
        for (it.Initialize(wires); it.More(); it.Next())
        {
            Wire W1 = ShapeCasts.Wire(it.Value());
            NCollection_List<Shape>.Iterator it2 = new(wires);

            if (IsClosed(W1))
            {
                Shape aLocalShape = myFace.EmptyCopied();
                Face NF = ShapeCasts.Face(aLocalShape);
                NF.Orientation(Forward);
                B.Add(NF, W1);

                while (it2.More())
                {
                    Wire W2 = ShapeCasts.Wire(it2.Value());
                    if (!W1.IsSame(W2) && IsInside(W2, NF))
                    {
                        Store(W2, W1, keyIsIn, keyContains);
                    }
                    it2.Next();
                }
            }
        }
        NCollection_List<Shape> WireExt = new();

        for (it.Initialize(wires); it.More(); it.Next())
        {
            Wire W = ShapeCasts.Wire(it.Value());
            if (!keyIsIn.IsBound(W) || keyIsIn[W].IsEmpty())
            {
                WireExt.Append(W);
            }
        }

        for (it.Initialize(WireExt); it.More(); it.Next())
        {
            Wire W = ShapeCasts.Wire(it.Value());
            if (!keyIsIn.IsBound(W) || keyIsIn[W].IsEmpty())
            {
                Shape aLocalShape = myFace.EmptyCopied();
                Face NewFace = ShapeCasts.Face(aLocalShape);
                NewFace.Orientation(Forward);
                B.Add(NewFace, W);
                faces.Append(NewFace);
                //--------------------------------------------
                // Construction of a face by exterior wire.
                //--------------------------------------------
                BuildFaceIn(NewFace, W, keyContains, keyIsIn, Forward, faces);
            }
        }
        myDone = true;
    }
}
