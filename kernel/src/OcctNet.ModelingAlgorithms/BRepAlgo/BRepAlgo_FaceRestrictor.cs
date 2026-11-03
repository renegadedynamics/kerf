// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_FaceRestrictor.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_FaceRestrictor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

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
/// <see cref="Perform"/> works on a copy of frozen wires (<see cref="BRepTools_CopyAtEntry"/>) and gives the
/// caller's wires and edges back wherever nothing was written (on a plane with closed-flagged wires, nothing is), so
/// the faces hold the caller's own wires as native's do (decision 793). <see cref="Current"/> hands out frozen faces;
/// <see cref="InternalCurrent"/> doesn't, for ported callers.
/// </para>
/// </remarks>
public sealed class BRepAlgo_FaceRestrictor
{
    private bool myDone;
    private bool modeProj;
    private TopoDS_Face myFace = new();
    private readonly NCollection_List<TopoDS_Shape> wires = new();
    private readonly NCollection_List<TopoDS_Shape> faces = new();
    private bool myCorrection;
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> keyIsIn = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> keyContains = new(TopTools_ShapeMapHasher.Instance);

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
    public void Init(in TopoDS_Face F, bool Proj = false, bool CorrectionOrientation = false)
    {
        myFace = F;
        modeProj = Proj;
        myCorrection = CorrectionOrientation;
    }

    /// <summary>Adds the (closed) wire <paramref name="W"/>.</summary>
    public void Add(in TopoDS_Wire W)
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
    private static bool ChangePCurve(in TopoDS_Edge E, Geom_Surface S, in Placement L)
    {
        BRep_Builder BB = new();
        Geom2d_Curve? C2 = null;
        Geom_Surface? SE = null;
        Placement LE = new();
        double f = 0.0, l = 0.0;
        BRep_Tool.CurveOnSurface(E, ref C2, ref SE, ref LE, ref f, ref l, 1);
        if (C2 is not null)
        {
            BB.UpdateEdge(E, C2, S, L, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        return C2 is null;
    }

    // Puts on S the projection of the edge's 3D curve; false if it has none
    private static bool ProjCurve3d(in TopoDS_Edge E, Geom_Surface S, in Placement L)
    {
        BRep_Builder BB = new();
        Geom_Curve? C = BRep_Tool.Curve(E, out Placement LE, out double f, out double l);
        if (C is null)
        {
            return false;
        }

        Geom_TrimmedCurve CT = new(C, f, l);

        Placement LL = L.Inverted().Multiplied(LE);
        CT.Transform(LL.Transformation());

        Geom2d_Curve? C2 = GeomProjLib.Curve2d(CT, S);
        BB.UpdateEdge(E, C2, S, L, global::Kerf.Engine.Foundation.Precision.Confusion());
        return true;
    }

    /// <summary>Builds the faces bounded by the wires.</summary>
    public void Perform()
    {
        // OCCT.NET: copy at entry (decision 793)
        TopoDS_Shape aHolder = TopOpeBRepBuild_WireToFace.HolderOf(wires);
        BRepTools_CopyAtEntry anEntry = new(aHolder);
        if (!anEntry.IsCopied())
        {
            InternalPerform();
            return;
        }
        NCollection_List<TopoDS_Shape> aSaved = new(wires);
        wires.Clear();
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(aSaved); it.More(); it.Next())
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
            NCollection_List<TopoDS_Shape>.Iterator itc = new(wires);
            for (NCollection_List<TopoDS_Shape>.Iterator it = new(aSaved); it.More() && itc.More(); it.Next(), itc.Next())
            {
                it.ChangeValue() = it.Value().Oriented(itc.Value().Orientation());
            }
            wires.Assign(aSaved);
        }
        anEntry.Finish(new TopoDS_Shape(), faces);
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
        NCollection_List<TopoDS_Shape>.Iterator it = new(wires);

        //--------------------------------------------------------------------
        // return geometry of the reference face.
        //--------------------------------------------------------------------
        Geom_Surface S = BRep_Tool.Surface(myFace, out Placement L)!;

        //-----------------------------------------------------------------------
        // test if edge is on S. otherwise  add S to the first pcurve.
        // or projection of the edge on F.
        //----------------------------------------------------------------------
        TopExp_Explorer Exp = new();

        TopOpeBRepBuild_WireToFace WTF = new();

        for (; it.More(); it.Next())
        {
            // update the surface on edges.
            TopoDS_Wire W = TopoDS.Wire(it.Value());

            for (Exp.Init(W, TopAbs_EDGE); Exp.More(); Exp.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(Exp.Current());
                Geom2d_Curve? C2 = BRep_Tool.CurveOnSurface(E, S, L, out _, out _);
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
    public TopoDS_Face Current()
    {
        TopoDS_Face aFace = InternalCurrent();
        aFace.TShape()?.FreezeGraph();
        return aFace;
    }

    /// <summary>The current face, unfrozen, for ported callers that go on building on it.</summary>
    internal TopoDS_Face InternalCurrent() => TopoDS.Face(faces.First());

    // Whether the wire is closed (flag or same end vertices)
    private static bool IsClosed(in TopoDS_Wire W)
    {
        if (W.Closed())
        {
            return true;
        }
        TopExp.Vertices(W, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        return V1.IsSame(V2);
    }

    // Whether a point of the first edge of wir is inside F
    private static bool IsInside(in TopoDS_Wire wir, in TopoDS_Face F)
    {
        TopExp_Explorer exp = new();
        exp.Init(wir, TopAbs_EDGE);
        if (exp.More())
        {
            TopoDS_Edge edg = TopoDS.Edge(exp.Current());
            // OCCT: dereferences the pcurve unchecked (native crashes on an edge without one on F; the port throws)
            Geom2d_Curve C2d = BRep_Tool.CurveOnSurface(edg, F, out double f, out double l)!;
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
            TopAbs_State st2 = FClass2d.Perform(pt2d, false);
            return st2 == TopAbs_IN;
        }
        return false;
    }

    private static void Store(in TopoDS_Wire W2,
                              in TopoDS_Wire W1,
                              NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> keyIsIn,
                              NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> keyContains)
    {
        if (!keyIsIn.IsBound(W2))
        {
            NCollection_List<TopoDS_Shape> empty = new();
            keyIsIn.Bind(W2, empty);
        }
        keyIsIn[W2].Append(W1);

        if (!keyContains.IsBound(W1))
        {
            NCollection_List<TopoDS_Shape> empty = new();
            keyContains.Bind(W1, empty);
        }
        keyContains[W1].Append(W2);
    }

    private static void BuildFaceIn(in TopoDS_Face F,
                                    in TopoDS_Wire W,
                                    NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> KeyContains,
                                    NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> KeyIsIn,
                                    TopAbs_Orientation Orientation,
                                    NCollection_List<TopoDS_Shape> Faces)
    {
        BRep_Builder B = new();

        if (!KeyContains.IsBound(W) || KeyContains[W].IsEmpty())
        {
            return;
        }

        // Removal of W in KeyIsIn.
        NCollection_List<TopoDS_Shape>.Iterator it = new();
        for (it.Initialize(KeyContains[W]); it.More(); it.Next())
        {
            TopoDS_Wire WI = TopoDS.Wire(it.Value());
            NCollection_List<TopoDS_Shape> L2 = KeyIsIn[WI];
            NCollection_List<TopoDS_Shape>.Iterator it2 = new();
            for (it2.Initialize(L2); it2.More(); it2.Next())
            {
                if (it2.Value().IsSame(W))
                {
                    L2.Remove(it2);
                    break;
                }
            }
        }

        NCollection_List<TopoDS_Shape> WireExt = new();

        for (it.Initialize(KeyContains[W]); it.More(); it.Next())
        {
            TopoDS_Wire WI = TopoDS.Wire(it.Value());
            NCollection_List<TopoDS_Shape> L2 = KeyIsIn[WI];

            if (L2.IsEmpty())
            {
                WireExt.Append(WI);
            }
        }

        for (it.Initialize(WireExt); it.More(); it.Next())
        {
            TopoDS_Wire WI = TopoDS.Wire(it.Value());
            NCollection_List<TopoDS_Shape> L2 = KeyIsIn[WI];
            if (L2.IsEmpty())
            {
                if (Orientation == TopAbs_FORWARD)
                {
                    TopoDS_Wire NWI = WI;
                    NWI.Reverse();
                    B.Add(F, NWI);
                    BuildFaceIn(F, WI, KeyContains, KeyIsIn, TopAbs_REVERSED, Faces);
                }
                else
                {
                    TopoDS_Shape aLocalShape = Faces.First().EmptyCopied();
                    TopoDS_Face NF = TopoDS.Face(aLocalShape);
                    B.Add(NF, WI);
                    Faces.Append(NF);
                    BuildFaceIn(NF, WI, KeyContains, KeyIsIn, TopAbs_FORWARD, Faces);
                }
            }
        }
    }

    // Perform with the closed wires oriented so that no face is infinite, and nested by classification
    private void PerformWithCorrection()
    {
        BRep_Builder B = new();

        myDone = false;
        NCollection_List<TopoDS_Shape>.Iterator it = new(wires);
        //---------------------------------------------------------
        // Reorientation of all closed wires to the left.
        //---------------------------------------------------------
        for (; it.More(); it.Next())
        {
            ref TopoDS_Shape W = ref it.ChangeValue();
            TopoDS_Shape aLocalShape = myFace.EmptyCopied();
            TopoDS_Face NF = TopoDS.Face(aLocalShape);
            NF.Orientation(TopAbs_FORWARD);
            B.Add(NF, W);

            if (IsClosed(TopoDS.Wire(W)))
            {
                BRepTopAdaptor_FClass2d FClass2d = new(NF, global::Kerf.Engine.Foundation.Precision.PConfusion());
                if (FClass2d.PerformInfinitePoint() != TopAbs_OUT)
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
            TopoDS_Wire W1 = TopoDS.Wire(it.Value());
            NCollection_List<TopoDS_Shape>.Iterator it2 = new(wires);

            if (IsClosed(W1))
            {
                TopoDS_Shape aLocalShape = myFace.EmptyCopied();
                TopoDS_Face NF = TopoDS.Face(aLocalShape);
                NF.Orientation(TopAbs_FORWARD);
                B.Add(NF, W1);

                while (it2.More())
                {
                    TopoDS_Wire W2 = TopoDS.Wire(it2.Value());
                    if (!W1.IsSame(W2) && IsInside(W2, NF))
                    {
                        Store(W2, W1, keyIsIn, keyContains);
                    }
                    it2.Next();
                }
            }
        }
        NCollection_List<TopoDS_Shape> WireExt = new();

        for (it.Initialize(wires); it.More(); it.Next())
        {
            TopoDS_Wire W = TopoDS.Wire(it.Value());
            if (!keyIsIn.IsBound(W) || keyIsIn[W].IsEmpty())
            {
                WireExt.Append(W);
            }
        }

        for (it.Initialize(WireExt); it.More(); it.Next())
        {
            TopoDS_Wire W = TopoDS.Wire(it.Value());
            if (!keyIsIn.IsBound(W) || keyIsIn[W].IsEmpty())
            {
                TopoDS_Shape aLocalShape = myFace.EmptyCopied();
                TopoDS_Face NewFace = TopoDS.Face(aLocalShape);
                NewFace.Orientation(TopAbs_FORWARD);
                B.Add(NewFace, W);
                faces.Append(NewFace);
                //--------------------------------------------
                // Construction of a face by exterior wire.
                //--------------------------------------------
                BuildFaceIn(NewFace, W, keyContains, keyIsIn, TopAbs_FORWARD, faces);
            }
        }
        myDone = true;
    }
}
