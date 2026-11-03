// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Face.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Face.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Face.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//: l2 abv 10.01.99: USA60022 7289: corrections for reversed face
// gka 11.01.99 file PRO7755.stp #2018: work-around error in BRepLib_MakeFace
//: p4 abv, pdn 23.02.99: PRO9234 #15720: call BRepTools::Update() for faces
//    rln 03.03.99 S4135: transmission of parameter precision to SA_Surface::NbSingularities
//: q5 abv 19.03.99 code improvement
//%14 pdn 15.03.99 adding function for fixing null area wires
//%15 pdn 20.03.99 code improvement
//    abv 09.04.99 S4136: improve tolerance management; remove unused flag Closed
// #4  szv          S4163: optimization
//    smh 31.01.01 BUC60810 : Case of small wire on face in solid
// sln 25.09.2001  checking order of 3d and 2d representation curves
// abv 19.10.2001  FixAddNaturalBound improved and extracted as separate fix
// skl,pdn 14.05.2002  OCC55 (correction precision for small faces)

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.SurfaceKind;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// Fixes a face: its wires (through <see cref="ShapeFix_Wire"/>), their orientation, a missing seam on a closed
/// surface, missing natural bounds on a sphere or a torus, small-area wires, loop wires, intersecting wires, the
/// apex of a cone, and the split of a face holding several outer wires.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). Load a face with <see cref="Init(in Face)"/>, or a surface with
/// <see cref="Init(Surface, double, bool)"/> and its wires with <see cref="Add"/>; then call
/// <see cref="Perform"/> or the fixes one by one. The modes (<c>...Mode()</c>) are three-position flags: -1 is the
/// default, 0 off, 1 on. Fixes may change the face into a shell or a compound of faces (<see cref="Result"/>).
/// </para>
/// <para>
/// The fixes edit the face in place, as OCCT's. A face holding frozen shapes (any face a caller holds, or one built
/// from a caller's wires) is copied at the first fix (<see cref="CopyAtEntry"/>, decision 583), so the
/// caller's shapes never change; <see cref="Face"/> and <see cref="Result"/> give back the caller's own sub-shapes
/// wherever the fixes left them unchanged, re-key the context to the caller's shapes and freeze what they return.
/// Set the context before the first fix. After a public result, a further fix copies again.
/// </para>
/// </remarks>
public partial class ShapeFix_Face : ShapeFix_Root
{
    protected ShapeAnalysis_Surface? mySurf;
    protected Face            myFace;
    protected Shape           myResult;
    protected ShapeFix_Wire          myFixWire;
    protected bool                   myFwd;
    protected int                    myStatus;

    private int myFixWireMode;
    private int myFixOrientationMode;
    private int myFixAddNaturalBoundMode;
    private int myFixMissingSeamMode;
    private int myFixSmallAreaWireMode;
    private int myRemoveSmallAreaFaceMode;
    private int myFixLoopWiresMode;
    private int myFixIntersectingWiresMode;
    private int myFixSplitFaceMode;
    private int myAutoCorrectPrecisionMode;
    private int myFixPeriodicDegenerated;

    private static bool IsSurfaceUVInfinite(Surface theSurf)
    {
        theSurf.Bounds(out double UMin, out double UMax, out double VMin, out double VMax);

        return (Kerf.Engine.Foundation.Precision.IsInfinite(UMin) || Kerf.Engine.Foundation.Precision.IsInfinite(UMax) || Kerf.Engine.Foundation.Precision.IsInfinite(VMin)
                || Kerf.Engine.Foundation.Precision.IsInfinite(VMax));
    }

    private static bool IsSurfaceUVPeriodic(SurfaceAdaptor theSurf)
    {
        return ((theSurf.IsUPeriodic && theSurf.IsVPeriodic) || theSurf.GetType() == SurfaceKind.Sphere);
    }

    /// <summary>Creates an empty tool.</summary>
    public ShapeFix_Face()
    {
        myFace    = new Face();
        myResult  = new Shape();
        myFwd     = true;
        myStatus  = 0;
        myFixWire = new ShapeFix_Wire();
        ClearModes();
    }

    /// <summary>Creates a tool and loads a face.</summary>
    public ShapeFix_Face(in Face face)
    {
        myFace    = new Face();
        myResult  = new Shape();
        myFwd     = true;
        myStatus  = 0;
        myFixWire = new ShapeFix_Wire();
        ClearModes();
        Init(face);
    }

    /// <summary>Sets all modes to their defaults.</summary>
    public virtual void ClearModes()
    {
        myFixWireMode              = -1;
        myFixOrientationMode       = -1;
        myFixAddNaturalBoundMode   = -1;
        myFixMissingSeamMode       = -1;
        myFixSmallAreaWireMode     = -1;
        myRemoveSmallAreaFaceMode  = -1;
        myFixIntersectingWiresMode = -1;
        myFixLoopWiresMode         = -1;
        myFixSplitFaceMode         = -1;
        myAutoCorrectPrecisionMode = 1;
        myFixPeriodicDegenerated   = -1;
    }

    /// <summary>Sets the message registrator (and the wire tool's).</summary>
    public override void SetMsgRegistrator(ShapeExtend_BasicMsgRegistrator? msgreg)
    {
        base.SetMsgRegistrator(msgreg);
        myFixWire.SetMsgRegistrator(msgreg);
    }

    /// <summary>Sets the basic precision (and the wire tool's).</summary>
    public override void SetPrecision(double preci)
    {
        base.SetPrecision(preci);
        myFixWire.SetPrecision(preci);
    }

    /// <summary>Sets the minimal tolerance (and the wire tool's).</summary>
    public override void SetMinTolerance(double mintol)
    {
        base.SetMinTolerance(mintol);
        myFixWire.SetMinTolerance(mintol);
    }

    /// <summary>Sets the maximal tolerance (and the wire tool's).</summary>
    public override void SetMaxTolerance(double maxtol)
    {
        base.SetMaxTolerance(maxtol);
        myFixWire.SetMaxTolerance(maxtol);
    }

    /// <summary>
    /// Starts the creation of a face on a surface (with no wire yet): wires are added with <see cref="Add"/>. With
    /// <paramref name="fwd"/> false the face is reversed.
    /// </summary>
    public void Init(Surface surf, double preci, bool fwd = true)
    {
        myStatus = 0;
        var sas = new ShapeAnalysis_Surface(surf);
        Init(sas, preci, fwd);
    }

    /// <summary>As the surface form, sharing an analyser of the surface.</summary>
    public void Init(ShapeAnalysis_Surface surf, double preci, bool fwd = true)
    {
        myStatus = 0;
        mySurf   = surf;
        SetPrecision(preci);
        var B = new BrepBuilder();
        B.MakeFace(ref myFace, mySurf.Surface()!, Kerf.Engine.Foundation.Precision.Confusion());
        myShape = myFace;
        myFwd   = fwd;
        if (!fwd)
        {
            myFace.Orientation(Reversed);
        }
        loaded();
    }

    /// <summary>Loads a face (the precision isn't changed).</summary>
    public void Init(in Face theFace)
    {
        myStatus = 0;
        // Check if surface is null. It doesn't make sense to create ShapeAnalysis_Surface in that case.
        Surface? aSurface = Tool.Surface(theFace);
        if (aSurface is not null)
        {
            mySurf = new ShapeAnalysis_Surface(aSurface);
        }
        myFwd   = (theFace.Orientation() != Reversed);
        myFace  = theFace;
        myShape = myFace;
        loaded();
    }

    /// <summary>Returns (modifiable) the mode for fixing the wires (default on).</summary>
    public ref int FixWireMode() => ref myFixWireMode;

    /// <summary>Returns (modifiable) the mode for fixing the orientation of the wires (default on).</summary>
    public ref int FixOrientationMode() => ref myFixOrientationMode;

    /// <summary>Returns (modifiable) the mode for adding the natural bounds of a sphere or a torus (default on).</summary>
    public ref int FixAddNaturalBoundMode() => ref myFixAddNaturalBoundMode;

    /// <summary>Returns (modifiable) the mode for adding a missing seam (default on).</summary>
    public ref int FixMissingSeamMode() => ref myFixMissingSeamMode;

    /// <summary>Returns (modifiable) the mode for removing small-area wires (default off).</summary>
    public ref int FixSmallAreaWireMode() => ref myFixSmallAreaWireMode;

    /// <summary>Returns (modifiable) the mode for removing a face whose wires all have small areas (default off).</summary>
    public ref int RemoveSmallAreaFaceMode() => ref myRemoveSmallAreaFaceMode;

    /// <summary>Returns (modifiable) the mode for fixing intersecting wires (default on).</summary>
    public ref int FixIntersectingWiresMode() => ref myFixIntersectingWiresMode;

    /// <summary>Returns (modifiable) the mode for splitting loop wires (default on).</summary>
    public ref int FixLoopWiresMode() => ref myFixLoopWiresMode;

    /// <summary>Returns (modifiable) the mode for splitting a face with several outer wires (default on).</summary>
    public ref int FixSplitFaceMode() => ref myFixSplitFaceMode;

    /// <summary>Returns (modifiable) the mode for lowering the precision to half the smallest edge (default on).</summary>
    public ref int AutoCorrectPrecisionMode() => ref myAutoCorrectPrecisionMode;

    /// <summary>Returns (modifiable) the mode for adding the degenerated edge at a cone's apex (default on).</summary>
    public ref int FixPeriodicDegeneratedMode() => ref myFixPeriodicDegenerated;

    /// <summary>
    /// Adds a wire to the face being built (it must lie on the face's surface). The face loaded by
    /// <see cref="Init(in Face)"/> is the caller's: when it is frozen, the wire is added to a copy of it that
    /// shares its sub-shapes (<c>// OCCT.NET: frozen</c>; OCCT adds it to the caller's face).
    /// </summary>
    public void Add(in Wire wire)
    {
        if (wire.IsNull)
        {
            return;
        }
        var B = new BrepBuilder();
        if (myFace.TShape()!.IsFrozen)
        {
            // OCCT.NET: frozen. A shallow copy of the frozen face, holding its sub-shapes as they are.
            Face aCopy = ShapeCasts.Face(myFace.EmptyCopied());
            TShape aTShape = aCopy.TShape()!;
            for (var anIt = new Iterator(myFace, false, false); anIt.More(); anIt.Next())
            {
                aTShape._shapes.Append(anIt.Value());
            }
            myFace = aCopy;
            myShape = myFace;
        }
        // szv#4:S4163:12Mar99 SGI warns
        Shape fc = myFace.Oriented(Forward); //: l2 abv 10 Jan 99: Oriented()
        B.Add(fc, wire);
        loaded();
    }

    /// <summary>
    /// Tries to split a wire (needed if some segments were removed in ShapeFix_Wire::FixSelfIntersection).
    /// </summary>
    private static bool SplitWire(in Face face, in Wire wire, NCollection_Sequence<Shape> aResWires)
    {
        var UsedEdges = new NCollection_Map<int>();
        var sewd = new ShapeExtend_WireData(wire);
        int i, j, k;
        var sae = new ShapeAnalysis_Edge();
        for (i = 1; i <= sewd.NbEdges(); i++)
        {
            if (UsedEdges.Contains(i))
            {
                continue;
            }
            Edge E1 = sewd.Edge(i);
            UsedEdges.Add(i);
            Vertex V0, V1;
            V0 = sae.FirstVertex(E1);
            V1 = sae.LastVertex(E1);
            var sewd1 = new ShapeExtend_WireData();
            sewd1.Add(E1);
            bool IsConnectedEdge = true;
            // Set to true when null pcurves prevent the 2D closure verification:
            // the partial wire collected so far is discarded for this start edge.
            bool aIsAbandonedSplit = false;
            for (j = 2; j <= sewd.NbEdges() && IsConnectedEdge; j++)
            {
                var E2 = new Edge();
                for (k = 2; k <= sewd.NbEdges(); k++)
                {
                    if (UsedEdges.Contains(k))
                    {
                        continue;
                    }
                    E2 = sewd.Edge(k);
                    if (sae.FirstVertex(E2).IsSame(V1))
                    {
                        sewd1.Add(E2);
                        UsedEdges.Add(k);
                        V1 = sae.LastVertex(E2);
                        break;
                    }
                }
                if (k > sewd.NbEdges())
                {
                    IsConnectedEdge = false;
                    break;
                }
                if (V1.IsSame(V0))
                {
                    // check that V0 and V1 are same in 2d too
                    Curve2D? curve1 = Tool.CurveOnSurface(E1, face, out double a1, out double b1);
                    Curve2D? curve2 = Tool.CurveOnSurface(E2, face, out double a2, out double b2);
                    if (curve1 is null || curve2 is null)
                    {
                        aIsAbandonedSplit = true;
                        break;
                    }
                    if (E1.Orientation() == Reversed)
                    {
                        a1 = b1;
                    }
                    if (E2.Orientation() == Reversed)
                    {
                        b2 = a2;
                    }
                    curve1.D0(a1, out Point2D v0);
                    curve2.D0(b2, out Point2D v1);
                    var anAdaptor = new SurfaceAdaptor(Tool.Surface(face)!);
                    double tol = Max(Tool.Tolerance(V0), Tool.Tolerance(V1));
                    double maxResolution = 2 * Max(anAdaptor.UResolution(tol), anAdaptor.VResolution(tol));
                    if (v0.SquareDistance(v1) < maxResolution)
                    {
                        // new wire is closed, put it into sequence
                        aResWires.Append(sewd1.InternalWire());
                        break;
                    }
                }
            }
            if (aIsAbandonedSplit)
            {
                continue;
            }
            if (!IsConnectedEdge)
            {
                // create new notclosed wire
                aResWires.Append(sewd1.InternalWire());
            }
            if (UsedEdges.Extent() == sewd.NbEdges())
            {
                break;
            }
        }

        return true;
    }

    /// <summary>
    /// Performs all the fixes, as the modes allow. Returns true if something was done (see <see cref="Status"/>:
    /// DONE1 wires fixed, DONE2 orientation fixed, DONE3 missing seam added, DONE4 small-area wire removed, DONE5
    /// natural bounds added, DONE6 intersecting wires fixed, DONE7 loop wire split (or two-edge coincident wire
    /// removed), DONE8 face split).
    /// </summary>
    public bool Perform(ProgressRange theProgress = default)
    {
        enterWork();
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myFixWire.SetContext(Context());
        ShapeFix_Wire theAdvFixWire = myFixWire;

        // Skip if an earlier fix already removed this face or replaced it with a non-face.
        if (Context() is not null)
        {
            Shape anApplied = Context()!.InternalApply(myFace);
            if (anApplied.IsNull || anApplied.ShapeType() != global::Kerf.Topology.ShapeType.Face)
            {
                myResult = anApplied;
                return false;
            }
        }

        var B = new BrepBuilder();
        Shape aInitFace = myFace;
        // perform first part of fixes on wires
        bool isfixReorder = false;
        bool isReplaced   = false;

        // gka fix in order to avoid lost messages (following OCC21771)
        var aMapReorderedWires = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);

        double aSavPreci = Precision();
        if (NeedFix(myFixWireMode))
        {
            theAdvFixWire.SetFace(myFace, mySurf);

            int usFixLackingMode = theAdvFixWire.FixLackingMode();
            // int usFixNotchedEdgesMode = theAdvFixWire->FixNotchedEdgesMode(); // CR0024983
            int usFixSelfIntersectionMode = theAdvFixWire.FixSelfIntersectionMode();
            theAdvFixWire.FixLackingMode() = 0;
            // theAdvFixWire->FixNotchedEdgesMode() = false; // CR0024983
            theAdvFixWire.FixSelfIntersectionMode() = 0;

            bool         @fixed = false;
            Shape S     = myFace;
            if (Context() is not null)
            {
                S = Context()!.InternalApply(myFace);
            }
            Shape emptyCopied = S.EmptyCopied();
            Face  tmpFace     = ShapeCasts.Face(emptyCopied);
            tmpFace.Orientation(Forward);

            // skl 29.03.2010 (OCC21623)
            if (myAutoCorrectPrecisionMode != 0)
            {
                double size     = ShapeFix.LeastEdgeSize(S);
                double newpreci = Min(aSavPreci, size / 2.0);
                newpreci        = newpreci * 1.00001;
                if (aSavPreci > newpreci && newpreci > Kerf.Engine.Foundation.Precision.Confusion())
                {
                    SetPrecision(newpreci);
                    theAdvFixWire.SetPrecision(newpreci);
                }
            }

            isfixReorder = false;

            for (var iter = new Iterator(S, false); iter.More(); iter.Next())
            {
                if (iter.Value().ShapeType() != ShapeType.Wire)
                {
                    B.Add(tmpFace, iter.Value());
                    continue;
                }
                Wire wire = ShapeCasts.Wire(iter.Value());
                theAdvFixWire.Load(wire);
                if (theAdvFixWire.NbEdges() == 0)
                {
                    if (theAdvFixWire.WireData()!.NbNonManifoldEdges() != 0)
                    {
                        B.Add(tmpFace, wire);
                    }
                    else
                    {
                        @fixed = true;
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE5);
                    }
                    continue;
                }
                if (theAdvFixWire.Perform(theProgress))
                {
                    // fixed = true;
                    isfixReorder = (theAdvFixWire.StatusReorder(ShapeExtend_DONE) || isfixReorder);
                    @fixed        = (theAdvFixWire.StatusSmall(ShapeExtend_DONE)
                                    || theAdvFixWire.StatusConnected(ShapeExtend_DONE)
                                    || theAdvFixWire.StatusEdgeCurves(ShapeExtend_DONE)
                                    || theAdvFixWire.StatusNotches(ShapeExtend_DONE) || // CR0024983
                                    theAdvFixWire.StatusFixTails(ShapeExtend_DONE)
                                    || theAdvFixWire.StatusDegenerated(ShapeExtend_DONE)
                                    || theAdvFixWire.StatusClosed(ShapeExtend_DONE));
                    Wire w = theAdvFixWire.InternalWire();
                    if (@fixed)
                    {
                        Context()?.Replace(wire, w);
                        if (theAdvFixWire.NbEdges() == 0)
                        {
                            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE5);
                            continue;
                        }
                    }
                    else if (!wire.IsSame(w))
                    {
                        aMapReorderedWires.Bind(wire, w);
                    }

                    wire = w;
                }
                B.Add(tmpFace, wire);
            }

            theAdvFixWire.FixLackingMode()          = usFixLackingMode;
            theAdvFixWire.FixSelfIntersectionMode() = usFixSelfIntersectionMode;
            if (!myFwd)
            {
                tmpFace.Orientation(Reversed);
            }

            if (@fixed)
            {
                Context()?.Replace(S, tmpFace);
                isReplaced = true;
            }
            if (@fixed || isfixReorder)
            {
                myFace = tmpFace;
                if (!theAdvFixWire.StatusReorder(ShapeExtend_DONE5))
                {
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                }
            }
        }

        myResult = myFace;
        Shape savShape = myFace; // gka BUG 6555

        // Specific case for conic surfaces
        if (NeedFix(myFixPeriodicDegenerated))
        {
            this.fixPeriodicDegenerated();
        }

        // fix missing seam
        if (NeedFix(myFixMissingSeamMode))
        {
            if (fixMissingSeam())
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
            }
        }

        // cycle by all possible faces coming from FixMissingSeam
        // each face is processed as if it was single
        var exp = new Explorer(myResult, global::Kerf.Topology.ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            myFace = ShapeCasts.Face(exp.Current());
            bool NeedCheckSplitWire = false;

            // perform second part of fixes on wires
            if (NeedFix(myFixWireMode))
            {
                theAdvFixWire.SetFace(myFace, mySurf);

                int usFixSmallMode       = theAdvFixWire.FixSmallMode();
                int usFixConnectedMode   = theAdvFixWire.FixConnectedMode();
                int usFixEdgeCurvesMode  = theAdvFixWire.FixEdgeCurvesMode();
                int usFixDegeneratedMode = theAdvFixWire.FixDegeneratedMode();
                theAdvFixWire.FixSmallMode()       = 0;
                theAdvFixWire.FixConnectedMode()   = 0;
                theAdvFixWire.FixEdgeCurvesMode()  = 0;
                theAdvFixWire.FixDegeneratedMode() = 0;

                bool         @fixed = false;
                Shape S     = myFace;
                if (Context() is not null)
                {
                    S = Context()!.InternalApply(myFace);
                }
                Shape emptyCopied = S.EmptyCopied();
                Face  tmpFace     = ShapeCasts.Face(emptyCopied);
                tmpFace.Orientation(Forward);
                for (var iter = new Iterator(S, false); iter.More(); iter.Next())
                {
                    if (iter.Value().ShapeType() != ShapeType.Wire)
                    {
                        B.Add(tmpFace, iter.Value());
                        continue;
                    }

                    Wire wire = ShapeCasts.Wire(iter.Value());
                    theAdvFixWire.Load(wire);
                    if (theAdvFixWire.NbEdges() == 0)
                    {
                        if (theAdvFixWire.WireData()!.NbNonManifoldEdges() != 0)
                        {
                            B.Add(tmpFace, wire);
                        }
                        else
                        {
                            @fixed = true;
                            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE5);
                        }
                        continue;
                    }
                    if (theAdvFixWire.Perform())
                    {
                        isfixReorder = theAdvFixWire.StatusReorder(ShapeExtend_DONE);
                        @fixed        = (theAdvFixWire.StatusLacking(ShapeExtend_DONE)
                                        || theAdvFixWire.StatusSelfIntersection(ShapeExtend_DONE)
                                        || theAdvFixWire.StatusNotches(ShapeExtend_DONE)
                                        || theAdvFixWire.StatusFixTails(ShapeExtend_DONE));
                        Wire w = theAdvFixWire.InternalWire();
                        if (@fixed)
                        {
                            Context()?.Replace(wire, w);
                        }
                        else if (!wire.IsSame(w))
                        {
                            aMapReorderedWires.Bind(wire, w);
                        }

                        wire = w;
                    }
                    if (theAdvFixWire.StatusRemovedSegment())
                    {
                        NeedCheckSplitWire = true;
                    }

                    // fix for loop of wire
                    var aLoopWires = new NCollection_Sequence<Shape>();
                    if (NeedFix(myFixLoopWiresMode) && fixLoopWire(aLoopWires))
                    {
                        if (aLoopWires.Length() > 1)
                        {
                            SendWarning(wire, new Msg("FixAdvFace.FixLoopWire.MSG0")); // Wire was split on several wires
                        }
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE7);
                        @fixed = true;
                        int k = 1;
                        for (; k <= aLoopWires.Length(); k++)
                        {
                            B.Add(tmpFace, aLoopWires.Value(k));
                        }
                    }
                    else
                    {
                        B.Add(tmpFace, wire);
                    }
                }

                theAdvFixWire.FixSmallMode()       = usFixSmallMode;
                theAdvFixWire.FixConnectedMode()   = usFixConnectedMode;
                theAdvFixWire.FixEdgeCurvesMode()  = usFixEdgeCurvesMode;
                theAdvFixWire.FixDegeneratedMode() = usFixDegeneratedMode;

                if (@fixed)
                {
                    if (!myFwd)
                    {
                        tmpFace.Orientation(Reversed);
                    }
                    if (!isReplaced && !aInitFace.IsSame(myResult) && Context() is not null)
                    { // gka 06.09.04 BUG 6555
                        Context()!.Replace(aInitFace, savShape);
                    }
                    Context()?.Replace(S, tmpFace);
                    myFace = tmpFace;
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                }
            }

            if (NeedCheckSplitWire)
            {
                // try to split wire - it is needed if some segments were removed
                // in ShapeFix_Wire::FixSelfIntersection()
                Shape S = myFace;
                if (Context() is not null)
                {
                    S = Context()!.InternalApply(myFace);
                }
                Shape emptyCopied = S.EmptyCopied();
                Face  tmpFace     = ShapeCasts.Face(emptyCopied);
                tmpFace.Orientation(Forward);
                var aWires = new NCollection_Sequence<Shape>();
                int nbw = 0;
                for (var iter = new Iterator(S, false); iter.More(); iter.Next())
                {
                    if (iter.Value().ShapeType() != ShapeType.Wire)
                    {
                        B.Add(tmpFace, iter.Value());
                        continue;
                    }
                    if (iter.Value().Orientation() != Forward && iter.Value().Orientation() != Reversed)
                    {
                        B.Add(tmpFace, ShapeCasts.Wire(iter.Value()));
                        continue;
                    }
                    nbw++;
                    Wire wire = ShapeCasts.Wire(iter.Value());
                    SplitWire(tmpFace, wire, aWires);
                }
                if (nbw < aWires.Length())
                {
                    for (int iw = 1; iw <= aWires.Length(); iw++)
                    {
                        B.Add(tmpFace, aWires.Value(iw));
                    }
                    Context()?.Replace(S, tmpFace);
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE8);
                    myFace = tmpFace;
                }
            }

            // fix intersecting wires
            if (fixWiresTwoCoincEdges())
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE7);
            }
            if (NeedFix(myFixIntersectingWiresMode))
            {
                if (fixIntersectingWires())
                {
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE6);
                }
            }

            // fix orientation
            var MapWires = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            MapWires.Clear();
            if (NeedFix(myFixOrientationMode))
            {
                if (fixOrientation(MapWires))
                {
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                }
            }

            BrepTools.Update(myFace);

            // fix natural bounds
            bool NeedSplit = true;
            if (fixAddNaturalBound())
            {
                NeedSplit = false;
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE5);
            }

            // split face
            if (NeedFix(myFixSplitFaceMode) && NeedSplit && MapWires.Extent() > 1)
            {
                if (fixSplitFace(MapWires))
                {
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE8);
                }
            }
        }

        // return the original preci
        SetPrecision(aSavPreci);
        theAdvFixWire.SetPrecision(aSavPreci);

        // cycle by all possible faces coming from FixAddNaturalBound
        // each face is processed as if it was single
        for (exp.Init(myResult, global::Kerf.Topology.ShapeType.Face); exp.More(); exp.Next())
        {
            myFace = ShapeCasts.Face(exp.Current());

            // fix small-area wires
            if (NeedFix(myFixSmallAreaWireMode, false))
            {
                bool isRemoveFace = NeedFix(myRemoveSmallAreaFaceMode, false);
                if (fixSmallAreaWire(isRemoveFace))
                {
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                }
            }
        }

        if (Context() is not null)
        {
            if (Status(ShapeExtend_DONE) && !isReplaced && !aInitFace.IsSame(savShape))
            {
                // gka fix in order to avoid lost messages (following OCC21771)
                if (aMapReorderedWires.Extent() != 0)
                {
                    var aItW = new Iterator(aInitFace, false);
                    for (; aItW.More(); aItW.Next())
                    {
                        Shape aCurW = aItW.Value();
                        while (aMapReorderedWires.IsBound(aCurW))
                        {
                            Shape aFixW = aMapReorderedWires.Find(aCurW);
                            Context()!.Replace(aCurW, aFixW);
                            aCurW = aFixW;
                        }
                    }
                }
                Context()!.Replace(aInitFace, savShape);
            }
            myResult = Context()!.InternalApply(aInitFace); // gka 06.09.04
        }
        else if (!Status(ShapeExtend_DONE))
        {
            myResult = aInitFace;
        }

        return Status(ShapeExtend_DONE);
    }

    // Shift all pcurves of edges in the given wire on the given face
    // to vector <vec>
    private static void Shift2dWire(in Wire w, in Face f, Vector2D vec, ShapeAnalysis_Surface theSurface,
                                    bool recompute3d = false)
    {
        var tr2d = new Transform2D();
        tr2d.SetTranslation(new Vector2D(vec.XY()));
        var sae = new ShapeAnalysis_Edge();
        var sbe = new ShapeBuild_Edge();
        var B = new BrepBuilder();
        for (var ei = new Iterator(w, false); ei.More(); ei.Next())
        {
            Edge edge = ShapeCasts.Edge(ei.Value());
            if (!sae.PCurve(edge, f, out Curve2D? C2d, out double cf, out double cl, true))
            {
                continue;
            }
            C2d = EditPCurve(edge, C2d!, c => c.Transform(tr2d));
            if (recompute3d)
            {
                // recompute 3d curve and vertex
                sbe.RemoveCurve3d(edge);
                sbe.BuildCurve3d(edge);
                B.UpdateVertex(sae.FirstVertex(edge), theSurface.Value(C2d.Value(cf)), 0.0);
            }
        }
    }

    /// <summary>
    /// OCCT.NET: frozen. Edits in place the pcurve <paramref name="theC2d"/> of the edge, as OCCT does: when it is
    /// frozen (a curve a caller's shape holds), the edit is made on a copy, which is put in its place in the edge's
    /// representations (OCCT edits the stored curve in place, and with it any other edge sharing it). Returns the
    /// curve edited.
    /// </summary>
    internal static Curve2D EditPCurve(in Edge theEdge, Curve2D theC2d, Action<Curve2D> theEdit)
    {
        if (!theC2d.IsFrozen)
        {
            theEdit(theC2d);
            return theC2d;
        }
        var aCopy = (Curve2D)theC2d.Copy();
        theEdit(aCopy);
        var anIt = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)theEdge.TShape()!).ChangeCurves());
        for (; anIt.More(); anIt.Next())
        {
            if (anIt.Value() is not GCurve aGC)
            {
                continue;
            }
            if (aGC.IsCurveOnSurface() && ReferenceEquals(aGC.PCurve(), theC2d))
            {
                aGC.PCurve(aCopy);
            }
            if (aGC.IsCurveOnClosedSurface && ReferenceEquals(aGC.PCurve2(), theC2d))
            {
                aGC.PCurve2(aCopy);
            }
        }
        return aCopy;
    }

    // Cut interval from the sequence of intervals
    private static bool CutInterval(NCollection_Sequence<Point2D> intervals, in Point2D toAddI, double period)
    {
        if (intervals.Length() <= 0)
        {
            return false;
        }
        for (int j = 0; j < 2; j++)
        { // try twice, align to bottom and to top
            for (int i = 1; i <= intervals.Length(); i++)
            {
                Point2D interval = intervals[i];
                // ACIS907, OCC921 a054a.sat (face 124)
                double shift = ShapeAnalysis.AdjustByPeriod((j != 0 ? toAddI.X() : toAddI.Y()),
                                                            0.5 * (interval.X() + interval.Y()),
                                                            period);
                var toAdd = new Point2D(toAddI.X() + shift, toAddI.Y() + shift);
                if (toAdd.Y() <= interval.X() || toAdd.X() >= interval.Y())
                {
                    continue;
                }
                if (toAdd.X() > interval.X())
                {
                    if (toAdd.Y() < interval.Y())
                    {
                        intervals.InsertBefore(i, interval);
                        intervals.ChangeValue(i + 1).SetX(toAdd.Y()); // i++...
                    }
                    intervals.ChangeValue(i).SetY(toAdd.X());
                }
                else if (toAdd.Y() < interval.Y())
                {
                    intervals.ChangeValue(i).SetX(toAdd.Y());
                }
                else
                {
                    intervals.Remove(i--);
                }
            }
        }
        return true;
    }

    // Find middle of the biggest interval
    private static double FindBestInterval(NCollection_Sequence<Point2D> intervals)
    {
        double shift = 0.0, max = -1.0;
        for (int i = 1; i <= intervals.Length(); i++)
        {
            Point2D interval = intervals[i];
            if (interval.Y() - interval.X() <= max)
            {
                continue;
            }
            max   = interval.Y() - interval.X();
            shift = interval.X() + 0.5 * max;
        }
        return shift;
    }

    /// <summary>
    /// Adds the natural boundary of a sphere or a torus to a face that misses it (the face's wires are then holes), or
    /// makes a face without wires naturally bounded. Returns true if done.
    /// </summary>
    public bool FixAddNaturalBound()
    {
        enterWork();
        return fixAddNaturalBound();
    }

    // Detect missing natural boundary on spherical surfaces and add it if
    // necessary
    // pdn 981202: add natural bounds if missing (on sphere only)
    //: abv 28.08.01: rewritten and extended for toruses
    private bool fixAddNaturalBound()
    {
        if (mySurf is null)
        {
            return false;
        }

        if (Context() is not null)
        {
            Shape S = Context()!.InternalApply(myFace);
            myFace = ShapeCasts.Face(S);
        }

        // collect wires in sequence
        var ws = new NCollection_Sequence<Shape>();
        var vs = new NCollection_Sequence<Shape>();
        var wi = new Iterator(myFace, false);
        for (; wi.More(); wi.Next())
        {
            if (wi.Value().ShapeType() == ShapeType.Wire
                && (wi.Value().Orientation() == Forward || wi.Value().Orientation() == Reversed))
            {
                ws.Append(wi.Value());
            }
            else
            {
                vs.Append(wi.Value());
            }
        }

        // deal with the case of an empty face: just create a new face by a standard tool
        if (ws.IsEmpty() && !IsSurfaceUVInfinite(mySurf.Surface()!))
        {
            var aFaceBuilder = new BRepBuilderAPI_MakeFace(mySurf.Surface()!, Kerf.Engine.Foundation.Precision.Confusion());

            Face aNewFace = aFaceBuilder.InternalFace();
            aNewFace.Orientation(myFace.Orientation());

            Context()?.Replace(myFace, aNewFace);

            // taking into account orientation
            myFace = aNewFace;

            // gka 11.01.99 file PRO7755.stp entity #2018 surface #1895: error BRepLib_MakeFace func
            // IsDegenerated
            ShapeFix_Edge sfe = myFixWire.FixEdgeTool();
            for (var Eed = new Explorer(myFace, ShapeType.Edge); Eed.More(); Eed.Next())
            {
                Edge edg = ShapeCasts.Edge(Eed.Current());
                sfe.FixVertexTolerance(edg, myFace);
            }

            //    B.UpdateFace (myFace,myPrecision);
            SendWarning(myFace, new Msg("FixAdvFace.FixOrientation.MSG0")); // Face created with natural bounds
            BrepTools.Update(myFace);
            myResult = myFace;
            return true;
        }

        // check if surface doesn't need natural bounds
        if (!isNeedAddNaturalBound(ws))
        {
            return false;
        }

        // Collect information on free intervals in U and V
        var intU = new NCollection_Sequence<Point2D>();
        var intV = new NCollection_Sequence<Point2D>();
        var centers = new NCollection_Sequence<Point2D>();
        mySurf.Bounds(out double SUF, out double SUL, out double SVF, out double SVL);
        intU.Append(new Point2D(SUF, SUL));
        intV.Append(new Point2D(SVF, SVL));
        int nb = ws.Length();
        int i;

        for (i = 1; i <= nb; i++)
        {
            //     Bnd_Box2d B;
            Wire aw        = ShapeCasts.Wire(ws.Value(i));
            Face aWireFace = ShapeCasts.Face(myFace.EmptyCopied());
            var aB = new BrepBuilder();
            aB.Add(aWireFace, aw);
            ShapeAnalysis.GetFaceUVBounds(aWireFace, out double Umin, out double Umax, out double Vmin, out double Vmax);

            // PTV 01.11.2002 ACIS907, OCC921 end
            if (mySurf.IsUClosed())
            {
                CutInterval(intU, new Point2D(Umin, Umax), SUL - SUF);
            }
            if (mySurf.IsVClosed())
            {
                CutInterval(intV, new Point2D(Vmin, Vmax), SVL - SVF);
            }
            centers.Append(new Point2D(0.5 * (Umin + Umax), 0.5 * (Vmin + Vmax)));
        }

        // find best interval and thus compute shift
        var shift = new Point2D(0.0, 0.0);
        if (mySurf.IsUClosed())
        {
            shift.SetX(FindBestInterval(intU));
        }
        if (mySurf.IsVClosed())
        {
            shift.SetY(FindBestInterval(intV));
        }

        // Adjust all other wires to be inside outer one
        var center = new Point2D(shift.X() + 0.5 * (SUL - SUF), shift.Y() + 0.5 * (SVL - SVF));
        for (i = 1; i <= nb; i++)
        {
            Wire wire = ShapeCasts.Wire(ws.Value(i));
            var sh = new Point2D(0.0, 0.0);
            if (mySurf.IsUClosed())
            {
                sh.SetX(ShapeAnalysis.AdjustByPeriod(centers[i].X(), center.X(), SUL - SUF));
            }
            if (mySurf.IsVClosed())
            {
                sh.SetY(ShapeAnalysis.AdjustByPeriod(centers[i].Y(), center.Y(), SVL - SVF));
            }
            Shift2dWire(wire, myFace, new Vector2D(sh.XY()), mySurf);
        }

        // Create naturally bounded surface and add that wire to sequence
        Surface surf = Tool.Surface(myFace, out Placement L)!;
        var mf = new BRepBuilderAPI_MakeFace(surf, Kerf.Engine.Foundation.Precision.Confusion());
        Face ftmp = mf.InternalFace();
        ftmp.Location(L);
        for (wi.Initialize(ftmp, false); wi.More(); wi.Next())
        {
            if (wi.Value().ShapeType() != ShapeType.Wire)
            {
                continue;
            }
            Wire wire = ShapeCasts.Wire(wi.Value());
            ws.Append(wire);
            if (shift.XY().Modulus() < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                continue;
            }
            Shift2dWire(wire, myFace, new Vector2D(shift.XY()), mySurf, true);
        }

        // Fix possible case on sphere when gap contains degenerated edge
        // and thus has a common part with natural boundary
        // Such hole should be merged with boundary
        if (mySurf.Adaptor3d()!.GetType() == SurfaceKind.Sphere && ws.Length() == nb + 1)
        {
            var bnd = new ShapeExtend_WireData(ShapeCasts.Wire(ws.Last()));
            // code to become separate method FixTouchingWires()
            for (i = 1; i <= nb; i++)
            {
                var sbwd = new ShapeExtend_WireData(ShapeCasts.Wire(ws.Value(i)));
                for (int j = 1; j <= sbwd.NbEdges(); j++)
                {
                    if (!Tool.Degenerated(sbwd.Edge(j)))
                    {
                        continue;
                    }
                    // find corresponding place in boundary
                    var sae = new ShapeAnalysis_Edge();
                    Vertex V = sae.FirstVertex(sbwd.Edge(j));
                    int k;
                    for (k = 1; k <= bnd.NbEdges(); k++)
                    {
                        if (!Tool.Degenerated(bnd.Edge(k)))
                        {
                            continue;
                        }
                        if (BrepTools.Compare(V, sae.FirstVertex(bnd.Edge(k))))
                        {
                            break;
                        }
                    }
                    if (k > bnd.NbEdges())
                    {
                        continue;
                    }
                    // and insert hole to that place
                    var B2 = new BrepBuilder();
                    B2.Degenerated(sbwd.Edge(j), false);
                    B2.Degenerated(bnd.Edge(k), false);
                    sbwd.SetLast(j);
                    bnd.Add(sbwd, k + 1);
                    ws.Remove(i--);
                    nb--;
                    myFixWire.SetFace(myFace, mySurf);
                    myFixWire.Load(bnd);
                    myFixWire.FixConnected();
                    myFixWire.FixDegenerated();
                    ws.SetValue(ws.Length(), bnd.InternalWire());
                    break;
                }
            }
        }

        // Create resulting face
        var B = new BrepBuilder();
        Shape S2 = myFace.EmptyCopied();
        S2.Orientation(Forward);
        for (i = 1; i <= ws.Length(); i++)
        {
            B.Add(S2, ws.Value(i));
        }
        for (i = 1; i <= vs.Length(); i++)
        {
            B.Add(S2, vs.Value(i));
        }
        if (!myFwd)
        {
            S2.Orientation(Reversed);
        }
        Context()?.Replace(myFace, S2);
        myFace = ShapeCasts.Face(S2);
        BrepTools.Update(myFace);

        SendWarning(myFace, new Msg("FixAdvFace.FixOrientation.MSG0")); // Face created with natural bounds
        return true;
    }

    /// <summary>Fixes the orientation of the wires on the face. Returns true if a wire was reversed.</summary>
    public bool FixOrientation()
    {
        enterWork();
        var MapWires = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        MapWires.Clear();
        return fixOrientation(MapWires);
    }

    private bool isNeedAddNaturalBound(NCollection_Sequence<Shape> theOrientedWires)
    {
        // if fix is not needed
        if (!NeedFix(myFixAddNaturalBoundMode))
        {
            return false;
        }
        // if surface is not double-closed
        if (mySurf is not null && !IsSurfaceUVPeriodic(mySurf.Adaptor3d()!))
        {
            return false;
        }
        // if face has an OUTER bound
        if (ShapeAnalysis.IsOuterBound(myFace))
        {
            return false;
        }
        // check that not any wire has a seam edge and not any edge is degenerated.
        // because the presence of a seam or degenerated edge indicates that this wire should be an
        // external one, and in case of its incorrect orientation, this will be corrected.
        int aNbOriented = theOrientedWires.Length();
        for (int i = 1; i <= aNbOriented; i++)
        {
            Wire aWire = ShapeCasts.Wire(theOrientedWires.Value(i));
            for (var anEdgeIt = new Iterator(aWire); anEdgeIt.More(); anEdgeIt.Next())
            {
                Edge anEdge = ShapeCasts.Edge(anEdgeIt.Value());
                if (Tool.Degenerated(anEdge))
                {
                    return false;
                }
                if (Tool.IsClosed(anEdge, myFace))
                {
                    return false;
                }
            }
        }

        return true;
    }

    /// <summary>
    /// Fixes the orientation of the wires on the face, and fills <paramref name="MapWires"/> with the outer wires and
    /// the wires inside each. Returns true if a wire was reversed.
    /// </summary>
    public bool FixOrientation(NCollection_DataMap<Shape, NCollection_List<Shape>> MapWires)
    {
        enterWork();
        return fixOrientation(MapWires);
    }

    private bool fixOrientation(NCollection_DataMap<Shape, NCollection_List<Shape>> MapWires)
    {
        bool done = false;

        if (Context() is not null)
        {
            Shape S = Context()!.InternalApply(myFace);
            myFace = ShapeCasts.Face(S);
        }
        var ws = new NCollection_Sequence<Shape>();
        var allSubShapes = new NCollection_Sequence<Shape>();
        // smh: BUC60810 : protection against very small wires (one-edge, null-length)
        var VerySmallWires = new NCollection_Sequence<Shape>();
        for (var wi = new Iterator(myFace, false); wi.More(); wi.Next())
        {
            if (wi.Value().ShapeType() == ShapeType.Vertex
                || (wi.Value().Orientation() != Forward && wi.Value().Orientation() != Reversed))
            {
                allSubShapes.Append(wi.Value());
                // ws.Append (wi.Value());
                continue;
            }

            var ei = new Iterator(wi.Value(), false);
            Edge anEdge;
            double length = RealLast();
            if (ei.More())
            {
                anEdge = ShapeCasts.Edge(ei.Value());
                ei.Next();
                if (!ei.More())
                {
                    length = 0;
                    var sae = new ShapeAnalysis_Edge();
                    if (sae.Curve3d(anEdge, out Curve3D? c3d, out double First, out double Last))
                    {
                        Point3D pntIni = c3d!.Value(First);
                        Xyz prev;
                        prev = pntIni.XYZ();
                        int NbControl = 10;
                        for (int j = 1; j < NbControl; j++)
                        {
                            double prm     = ((NbControl - 1 - j) * First + j * Last) / (NbControl - 1);
                            Point3D pntCurr = c3d.Value(prm);
                            Xyz curr    = pntCurr.XYZ();
                            Xyz delta   = curr - prev;
                            length += delta.Modulus();
                            prev = curr;
                        }
                    }
                }
            }
            else
            {
                length = 0;
            }
            if (length > Kerf.Engine.Foundation.Precision.Confusion())
            {
                ws.Append(wi.Value());
                allSubShapes.Append(wi.Value());
            }
            else
            {
                VerySmallWires.Append(wi.Value());
            }
        }
        if (VerySmallWires.Length() > 0)
        {
            done = true;
        }

        int nb    = ws.Length();
        int nbAll = allSubShapes.Length();
        var B = new BrepBuilder();

        // if no wires, just do nothing
        if (nb <= 0)
        {
            return false;
        }

        bool isAddNaturalBounds = isNeedAddNaturalBound(ws);
        var aSeqReversed = new NCollection_Sequence<int>();
        // if wire is only one, check its orientation
        if (nb == 1)
        {
            // skl 12.04.2002 for cases with nbwires>1 (VerySmallWires>1)
            // make face with only one wire (ws.Value(1))
            Shape dummy = myFace.EmptyCopied();
            Face af = ShapeCasts.Face(dummy);
            af.Orientation(Forward);
            B.Add(af, ws.Value(1));

            if (!isAddNaturalBounds && !ShapeAnalysis.IsOuterBound(af))
            {
                var sbdw = new ShapeExtend_WireData(ShapeCasts.Wire(ws.Value(1)));
                sbdw.Reverse(myFace);
                ws.SetValue(1, sbdw.InternalWire());
                SendWarning(sbdw.InternalWire(), new Msg("FixAdvFace.FixOrientation.MSG5")); // Wire on face was reversed
                done = true;
            }
        }
        // in case of several wires, perform complex analysis
        // TEST CAUTION
        // Multiple wires: relative orientations
        // Each wire must "contain" all the others
        // Obviously, in the case of leopard skin, there may be a problem
        else if (mySurf is not null)
        {
            // Take each wire (NB: curves present!)
            // In principle, we should reject unclosed wires (see missing seam?)
            // We classify it relative to the others, which must all be either IN or
            // OUT. Otherwise, there is nesting -> SDB. If IN, OK, if OUT, we reverse
            // (NB: not myClos here, so no stitching problem)
            // If there is at least one inversion, the face must be redone (see myRebil)
            bool uclosed = mySurf.IsUClosed();
            bool vclosed = mySurf.IsVClosed();
            mySurf.Bounds(out double SUF, out double SUL, out double SVF, out double SVL);
            double uRange = SUL - SUF;
            double vRange = SVL - SVF;

            var MW = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            var SI = new NCollection_DataMap<Shape, int>(ShapeHasher.Instance);
            var MapIntWires = new NCollection_Map<Shape>(ShapeHasher.Instance);
            MW.Clear();
            SI.Clear();
            MapIntWires.Clear();
            int i;

            var aWireBoxes = new NCollection_Array1<Box2d>(1, nb);
            double uMiddle = 0, vMiddle = 0;
            bool isFirst = true;
            // create Bounding boxes for each wire
            for (i = 1; i <= nb; i++)
            {
                Shape aShape = ws.Value(i);
                Wire aWire = ShapeCasts.Wire(aShape);
                var aBox = new Box2d();
                var ew = new Iterator(aWire);
                for (; ew.More(); ew.Next())
                {
                    Edge ed = ShapeCasts.Edge(ew.Value());
                    Curve2D? cw = Tool.CurveOnSurface(ed, myFace, out double cf, out double cl);
                    if (cw is null)
                    {
                        continue;
                    }
                    var gac = new CurveAdaptor2D();
                    double aFirst = cw.FirstParameter();
                    double aLast  = cw.LastParameter();
                    if (cw is BSplineCurve2D && (cf < aFirst || cl > aLast))
                    {
                        // avoiding problems with segment in Bnd_Box
                        gac.Load(cw);
                    }
                    else
                    {
                        gac.Load(cw, cf, cl);
                    }
                    Add2dCurve.Add(gac, Kerf.Engine.Foundation.Precision.Confusion(), ref aBox);
                }

                aBox.Get(out double aXMin, out double aYMin, out double aXMax, out double aYMax);
                if (isFirst)
                {
                    isFirst = false;
                    uMiddle = (aXMin + aXMax) * 0.5;
                    vMiddle = (aYMin + aYMax) * 0.5;
                }
                else
                {
                    double xShift = 0, yShift = 0;
                    if (mySurf.IsUClosed())
                    {
                        xShift = ShapeAnalysis.AdjustByPeriod(0.5 * (aXMin + aXMax), uMiddle, uRange);
                    }
                    if (mySurf.IsVClosed())
                    {
                        yShift = ShapeAnalysis.AdjustByPeriod(0.5 * (aYMin + aYMax), vMiddle, vRange);
                    }
                    aBox.Update(aXMin + xShift, aYMin + yShift, aXMax + xShift, aYMax + yShift);
                }
                aWireBoxes[i] = aBox;
            }

            for (i = 1; i <= nb; i++)
            {
                Shape asw = ws.Value(i);
                Wire aw = ShapeCasts.Wire(asw);
                Box2d aBox1 = aWireBoxes.Value(i);
                Shape dummy = myFace.EmptyCopied();
                Face af = ShapeCasts.Face(dummy);
                af.Orientation(Forward);
                B.Add(af, aw);
                // PTV OCC945 06.11.2002 files ie_exhaust-A.stp (entities 3782,  3787)
                // tolerance is too big. It is seems that to identify placement of 2d point
                // it is enough Precision::PConfusion(), cause wea re know that 2d point in TopAbs_ON
                // BRepTopAdaptor_FClass2d clas (af,toluv);
                bool CheckShift = true;
                var clas = new BRepTopAdaptor_FClass2d(af, Kerf.Engine.Foundation.Precision.PConfusion());
                Containment sta = Out;
                Containment staout = clas.PerformInfinitePoint();
                var IntWires = new NCollection_List<Shape>();
                int aWireIt = 0;
                for (int j = 1; j <= nbAll; j++)
                {
                    aWireIt++;
                    // if(i==j) continue;
                    Shape aSh2 = allSubShapes.Value(j);
                    if (aw == aSh2)
                    {
                        continue;
                    }
                    Containment stb = Unknown;
                    if (aSh2.ShapeType() == ShapeType.Vertex)
                    {
                        aWireIt--;
                        Point3D aP = Tool.Pnt(ShapeCasts.Vertex(aSh2));
                        Point2D p2d = mySurf.ValueOfUV(aP, Kerf.Engine.Foundation.Precision.Confusion());
                        stb = clas.Perform(p2d, false);
                        if (stb == staout && (uclosed || vclosed))
                        {
                            var p2d1 = new Point2D();
                            if (uclosed)
                            {
                                p2d1.SetCoord(p2d.X() + uRange, p2d.Y());
                                stb = clas.Perform(p2d1, false);
                            }
                            if (stb == staout && vclosed)
                            {
                                p2d1.SetCoord(p2d.X(), p2d.Y() + vRange);
                                stb = clas.Perform(p2d1, false);
                            }
                        }
                    }
                    else if (aSh2.ShapeType() == ShapeType.Wire)
                    {
                        CheckShift = true;
                        Wire bw = ShapeCasts.Wire(aSh2);
                        // int numin =0;
                        Box2d aBox2 = aWireBoxes.Value(aWireIt);
                        if (aBox2.IsOut(aBox1))
                        {
                            continue;
                        }

                        var ew = new Iterator(bw);
                        for (; ew.More(); ew.Next())
                        {
                            Edge ed = ShapeCasts.Edge(ew.Value());
                            Curve2D? cw = Tool.CurveOnSurface(ed, myFace, out double cf, out double cl);
                            if (cw is null)
                            {
                                continue;
                            }
                            Point2D unp = cw.Value((cf + cl) / 2.0);
                            Containment ste = clas.Perform(unp, false);
                            if (ste == Out || ste == In)
                            {
                                if (stb == Unknown)
                                {
                                    stb = ste;
                                }
                                else
                                {
                                    if (!(stb == ste))
                                    {
                                        sta = Unknown;
                                        SI.Bind(aw, 0);
                                        j = nbAll;
                                        break;
                                    }
                                }
                            }

                            bool found = false;
                            var unp1 = new Point2D();
                            if (stb == staout && CheckShift)
                            {
                                CheckShift = false;
                                if (uclosed)
                                {
                                    unp1.SetCoord(unp.X() + uRange, unp.Y());
                                    found = (staout != clas.Perform(unp1, false));
                                    if (!found)
                                    {
                                        unp1.SetX(unp.X() - uRange);
                                        found = (staout != clas.Perform(unp1, false));
                                    }
                                }
                                if (vclosed && !found)
                                {
                                    unp1.SetCoord(unp.X(), unp.Y() + vRange);
                                    found = (staout != clas.Perform(unp1, false));
                                    if (!found)
                                    {
                                        unp1.SetY(unp.Y() - vRange);
                                        found = (staout != clas.Perform(unp1, false));
                                    }
                                }
                                // Additional check of diagonal steps for toroidal surfaces
                                if (!found && uclosed && vclosed)
                                {
                                    for (double dX = -1.0; dX <= 1.0 && !found; dX += 2.0)
                                    {
                                        for (double dY = -1.0; dY <= 1.0 && !found; dY += 2.0)
                                        {
                                            unp1.SetCoord(unp.X() + uRange * dX, unp.Y() + vRange * dY);
                                            found = (staout != clas.Perform(unp1, false));
                                        }
                                    }
                                }
                            }
                            if (found)
                            {
                                if (stb == In)
                                {
                                    stb = Out;
                                }
                                else
                                {
                                    stb = In;
                                }
                                Shift2dWire(bw, myFace, new Vector2D(unp1.XY() - unp.XY()), mySurf);
                            }
                        }
                    }
                    if (stb == staout)
                    {
                        sta = In;
                    }
                    else
                    {
                        IntWires.Append(aSh2);
                        MapIntWires.Add(aSh2);
                    }
                }

                if (sta == Unknown)
                { // ERREUR
                    SendWarning(aw, new Msg("FixAdvFace.FixOrientation.MSG11")); // Cannot orient wire
                }
                else
                {
                    MW.Bind(aw, IntWires);
                    if (sta == Out)
                    {
                        if (staout == In)
                        {
                            // wire is OUT but InfinitePoint is IN => need to reverse
                            var sewd = new ShapeExtend_WireData(aw);
                            sewd.Reverse(myFace);
                            ws.SetValue(i, sewd.InternalWire());
                            SendWarning(sewd.InternalWire(), new Msg("FixAdvFace.FixOrientation.MSG5")); // Wire on face was reversed
                            aSeqReversed.Append(i);
                            done = true;
                            SI.Bind(ws.Value(i), 1);
                            MapWires.Bind(ws.Value(i), IntWires);
                        }
                        else
                        {
                            SI.Bind(aw, 1);
                            MapWires.Bind(aw, IntWires);
                        }
                    }
                    else
                    {
                        if (staout == Out)
                        {
                            SI.Bind(aw, 2);
                        }
                        else
                        {
                            SI.Bind(aw, 3);
                        }
                    }
                }
            }

            for (i = 1; i <= nb; i++)
            {
                Wire aw = ShapeCasts.Wire(ws.Value(i));
                int tmpi = SI.Find(aw);
                if (tmpi > 1)
                {
                    if (!MapIntWires.Contains(aw))
                    {
                        NCollection_List<Shape> IW = MW.Find(aw);
                        if (tmpi == 3)
                        {
                            // wire is OUT but InfinitePoint is IN => need to reverse
                            var sewd = new ShapeExtend_WireData(aw);
                            sewd.Reverse(myFace);
                            ws.SetValue(i, sewd.InternalWire());
                            SendWarning(sewd.InternalWire(), new Msg("FixAdvFace.FixOrientation.MSG5")); // Wire on face was reversed
                            aSeqReversed.Append(i);
                            done = true;
                            MapWires.Bind(ws.Value(i), IW);
                        }
                        else
                        {
                            MapWires.Bind(aw, IW);
                        }
                    }
                    else
                    {
                        if (tmpi == 2)
                        {
                            // wire is IN but InfinitePoint is OUT => need to reverse
                            var sewd = new ShapeExtend_WireData(aw);
                            sewd.Reverse(myFace);
                            ws.SetValue(i, sewd.InternalWire());
                            SendWarning(sewd.InternalWire(), new Msg("FixAdvFace.FixOrientation.MSG5")); // Wire on face was reversed
                            aSeqReversed.Append(i);
                            done = true;
                        }
                    }
                }
            }
        }

        if (isAddNaturalBounds && nb == aSeqReversed.Length())
        {
            done = false;
        }

        // Should I rebuild? if myRebil is set
        if (done)
        {
            Shape S = myFace.EmptyCopied();
            S.Orientation(Forward);
            int i = 1;
            for (; i <= nb; i++)
            {
                B.Add(S, ws.Value(i));
            }

            if (nb < nbAll)
            {
                for (i = 1; i <= nbAll; i++)
                {
                    Shape aS2 = allSubShapes.Value(i);
                    if (aS2.ShapeType() != ShapeType.Wire
                        || (aS2.Orientation() != Forward && aS2.Orientation() != Reversed))
                    {
                        B.Add(S, aS2);
                    }
                }
            }

            if (!myFwd)
            {
                S.Orientation(Reversed);
            }
            Context()?.Replace(myFace, S);
            myFace = ShapeCasts.Face(S);
            BrepTools.Update(myFace);
        }
        return done;
    }

    //: i7 abv 18 Sep 98: ProSTEP TR9 r0501-ug.stp: algorithm of fixing missing seam changed
    // test whether the wire is opened on period of periodical surface
    private static bool CheckWire(in Wire wire, in Face face, double dU, double dV,
                                  out int isuopen, out int isvopen, out bool isDeg)
    {
        var vec = new Xy();
        vec.SetX(0);
        vec.SetY(0);
        var sae = new ShapeAnalysis_Edge();

        isuopen = isvopen = 0;
        isDeg = true;
        for (var ed = new Iterator(wire); ed.More(); ed.Next())
        {
            Edge edge = ShapeCasts.Edge(ed.Value());
            if (!Tool.Degenerated(edge))
            {
                isDeg = false;
            }
            if (!sae.PCurve(edge, face, out Curve2D? c2d, out double f, out double l, true))
            {
                return false;
            }
            vec += c2d!.Value(l).XY() - c2d.Value(f).XY();
        }

        double aDelta = Math.Abs(vec.X()) - dU;
        if (Math.Abs(aDelta) < 0.1 * dU)
        {
            if (vec.X() > 0.0)
            {
                isuopen = 1;
            }
            else
            {
                isuopen = -1;
            }
        }
        else
        {
            isuopen = 0;
        }

        aDelta = Math.Abs(vec.Y()) - dV;
        if (Math.Abs(aDelta) < 0.1 * dV)
        {
            if (vec.Y() > 0.0)
            {
                isvopen = 1;
            }
            else
            {
                isvopen = -1;
            }
        }
        else
        {
            isvopen = 0;
        }

        return isuopen != 0 || isvopen != 0;
    }

    /// <summary>
    /// Adds a missing seam to a face on a closed surface whose wires are open in 2d by a period: the face is split by
    /// ShapeFix_ComposeShell. Returns true if done; the result may then be a shell (<see cref="Result"/>).
    /// </summary>
    public bool FixMissingSeam()
    {
        enterWork();
        return fixMissingSeam();
    }

    private bool fixMissingSeam()
    {
        if (mySurf is null)
        {
            return false;
        }

        bool uclosed = mySurf.IsUClosed();
        bool vclosed = mySurf.IsVClosed();

        if (!uclosed && !vclosed)
        {
            return false;
        }

        if (Context() is not null)
        {
            Shape S0 = Context()!.InternalApply(myFace);
            myFace = ShapeCasts.Face(S0);
        }

        //%pdn: surface should be made periodic before (see ShapeCustom_Surface)!
        if (mySurf.Surface() is global::Kerf.Geometry.BSplineSurface BSpl)
        {
            if (!BSpl.IsUPeriodic && !BSpl.IsVPeriodic)
            {
                return false;
            }
        }

        double URange, VRange;
        mySurf.Bounds(out double SUF, out double SUL, out double SVF, out double SVL);
        BrepTools.UVBounds(myFace, out double fU1, out double fU2, out double fV1, out double fV2);

        // pdn OCC55 fix to faces without the wires to avoid identical first and last parameters
        if (Kerf.Engine.Foundation.Precision.IsInfinite(SUF) || Kerf.Engine.Foundation.Precision.IsInfinite(SUL))
        {
            if (Kerf.Engine.Foundation.Precision.IsInfinite(SUF))
            {
                SUF = fU1;
            }
            if (Kerf.Engine.Foundation.Precision.IsInfinite(SUL))
            {
                SUL = fU2;
            }
            if (Math.Abs(SUL - SUF) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                if (Kerf.Engine.Foundation.Precision.IsInfinite(SUF))
                {
                    SUF -= 1000.0;
                }
                else
                {
                    SUL += 1000.0;
                }
            }
        }
        if (Kerf.Engine.Foundation.Precision.IsInfinite(SVF) || Kerf.Engine.Foundation.Precision.IsInfinite(SVL))
        {
            if (Kerf.Engine.Foundation.Precision.IsInfinite(SVF))
            {
                SVF = fV1;
            }
            if (Kerf.Engine.Foundation.Precision.IsInfinite(SVL))
            {
                SVL = fV2;
            }
            if (Math.Abs(SVL - SVF) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                if (Kerf.Engine.Foundation.Precision.IsInfinite(SVF))
                {
                    SVF -= 1000.0;
                }
                else
                {
                    SVL += 1000.0;
                }
            }
        }

        URange = Min(Math.Abs(SUL - SUF), Kerf.Engine.Foundation.Precision.Infinite());
        VRange = Min(Math.Abs(SVL - SVF), Kerf.Engine.Foundation.Precision.Infinite());
        //  double UTol = 0.2 * URange, VTol = 0.2 * VRange;
        int ismodeu = 0, ismodev = 0; // szv#4:S4163:12Mar99 was Boolean
        int isdeg1 = 0, isdeg2 = 0;

        var ws = new NCollection_Sequence<Shape>();
        var aSeqNonManif = new NCollection_Sequence<Shape>();
        for (var wi = new Iterator(myFace, false); wi.More(); wi.Next())
        {
            if (wi.Value().ShapeType() != ShapeType.Wire
                || (wi.Value().Orientation() != Forward && wi.Value().Orientation() != Reversed))
            {
                aSeqNonManif.Append(wi.Value());
                continue;
            }
            ws.Append(wi.Value());
        }

        Wire w1 = new Wire(), w2 = new Wire();
        int i;
        for (i = 1; i <= ws.Length(); i++)
        {
            Wire wire = ShapeCasts.Wire(ws.Value(i));
            if (!CheckWire(wire, myFace, URange, VRange, out int isuopen, out int isvopen, out bool isdeg))
            {
                continue;
            }
            if (w1.IsNull)
            {
                w1      = wire;
                ismodeu = isuopen;
                ismodev = isvopen;
                isdeg1  = isdeg ? i : 0;
            }
            else if (w2.IsNull)
            {
                if (ismodeu == -isuopen && ismodev == -isvopen)
                {
                    w2     = wire;
                    isdeg2 = isdeg ? i : 0;
                }
                else if (ismodeu == isuopen && ismodev == isvopen)
                {
                    w2     = wire;
                    isdeg2 = isdeg ? 1 : 0; // OCCT: the flag, not the index i as in the other branch (isdeg2 = isdeg)
                    //: abv 29.08.01: If wires are contraversal, reverse one of them
                    // If first one is single degenerated edge, reverse it; else second
                    if (isdeg1 != 0)
                    {
                        w1.Reverse();
                        ismodeu = -ismodeu;
                        ismodev = -ismodev;
                    }
                    else
                    {
                        w2.Reverse();
                    }
                }
            }
            //    else return false; //  abort
            else
            {
                //: abv 30.08.09: if more than one open wires and more than two of them are
                // completely degenerated, remove any of them
                if (isdeg || isdeg1 != 0 || isdeg2 != 0)
                {
                    ws.Remove(isdeg ? i : isdeg2 != 0 ? isdeg2 : isdeg1);
                    w1.Nullify();
                    w2.Nullify();
                    i = 0;
                    continue;
                }
            }
        }

        var B = new BrepBuilder();
        var aTorSurf = mySurf.Surface() as ToroidalSurface;
        bool anIsDegeneratedTor = (aTorSurf is null ? false : aTorSurf.MajorRadius() < aTorSurf.MinorRadius());
        // if the second wire is not null, we don't need mark the torus as degenerated
        // and should process it as a regular one.
        if (anIsDegeneratedTor && !w2.IsNull)
        {
            anIsDegeneratedTor = false;
        }

        if (w1.IsNull)
        {
            return false;
        }
        else if (w2.IsNull)
        {
            // For spheres and BSpline cone-like surfaces(bug 24055):
            // If only one of wires limiting face on surface is open in 2d,
            // this may means that degenerated edge should be added, and
            // then usual procedure applied
            var p = new Point2D();
            var d = new Direction2D();
            double aRange;

            if (ismodeu != 0 && anIsDegeneratedTor)
            {
                double aRa  = aTorSurf!.MajorRadius();
                double aRi  = aTorSurf.MinorRadius();
                double aPhi = Math.Acos(-aRa / aRi);
                p.SetCoord(0.0, (ismodeu > 0 ? M_PI + aPhi : aPhi));

                double aXCoord = -ismodeu;
                d.SetCoord(aXCoord, 0.0);
                aRange = 2.0 * M_PI;
            }
            else if (ismodeu != 0 && mySurf.Surface() is SphericalSurface)
            {
                p.SetCoord((ismodeu < 0 ? 0.0 : 2.0 * M_PI), ismodeu * 0.5 * M_PI);
                double aXCoord = -ismodeu;
                d.SetCoord(aXCoord, 0.0);
                aRange = 2.0 * M_PI;
            }
            else if (ismodev != 0 && mySurf.Surface() is global::Kerf.Geometry.BSplineSurface)
            {
                double uCoord;
                if (mySurf.Value(SUF, SVF).Distance(mySurf.Value(SUF, (SVF + SVL) / 2)) < Kerf.Engine.Foundation.Precision.Confusion())
                {
                    uCoord = SUF;
                }
                else if (mySurf.Value(SUL, SVF).Distance(mySurf.Value(SUL, (SVF + SVL) / 2)) < Kerf.Engine.Foundation.Precision.Confusion())
                {
                    uCoord = SUL;
                }
                else
                {
                    return false;
                }

                p.SetCoord(uCoord, (ismodev < 0 ? 0.0 : VRange));
                d.SetCoord(0.0, -ismodev);
                aRange = VRange;
            }
            else if (ismodeu != 0 && mySurf.Surface() is global::Kerf.Geometry.BSplineSurface)
            {
                double vCoord;
                if (mySurf.Value(SUF, SVF).Distance(mySurf.Value((SUF + SUL) / 2, SVF)) < Kerf.Engine.Foundation.Precision.Confusion())
                {
                    vCoord = SVF;
                }
                else if (mySurf.Value(SUL, SVL).Distance(mySurf.Value((SUF + SUL) / 2, SVL)) < Kerf.Engine.Foundation.Precision.Confusion())
                {
                    vCoord = SVL;
                }
                else
                {
                    return false;
                }

                p.SetCoord((ismodeu < 0 ? 0.0 : URange), vCoord);
                double aXCoord = -ismodeu;
                d.SetCoord(aXCoord, 0.0);
                aRange = URange;
            }
            else
            {
                return false;
            }

            var line = new LineCurve2D(p, d);
            var edge = new Edge();
            B.MakeEdge(ref edge);
            B.Degenerated(edge, true);
            B.UpdateEdge(edge, line, myFace, Kerf.Engine.Foundation.Precision.Confusion());
            B.Range(edge, myFace, 0.0, aRange);
            var V = new Vertex();
            B.MakeVertex(ref V, mySurf.Value(p.X(), p.Y()), Kerf.Engine.Foundation.Precision.Confusion());
            V.Orientation(Forward);
            B.Add(edge, V);
            V.Orientation(Reversed);
            B.Add(edge, V);
            B.MakeWire(ref w2);
            B.Add(w2, edge);
            ws.Append(w2);
        }

        // Check consistency of orientations of the two wires that need to be connected by a seam
        double uf = SUF, vf = SVF;
        int coord = (ismodeu != 0 ? 1 : 0);
        int isneg = (ismodeu != 0 ? ismodeu : -ismodev);
        double period = (ismodeu != 0 ? URange : VRange);
        Shape S;
        Span<double> m1 = stackalloc double[4]; // m1[c][k] is m1[2 * c + k]
        Span<double> m2 = stackalloc double[4];
        S = myFace.EmptyCopied();
        B.Add(S, w1);
        ShapeAnalysis.GetFaceUVBounds(ShapeCasts.Face(S), out m1[0], out m1[1], out m1[2], out m1[3]);
        S = myFace.EmptyCopied();
        B.Add(S, w2);
        ShapeAnalysis.GetFaceUVBounds(ShapeCasts.Face(S), out m2[0], out m2[1], out m2[2], out m2[3]);

        // For the case when surface is closed only in one direction it is necessary to check
        // validity of orientation of the open wires in parametric space.
        // In case of U closed surface wire with minimal V coordinate should be directed in positive
        // direction by U In case of V closed surface wire with minimal U coordinate should be directed in
        // negative direction by V
        if (!vclosed || !uclosed || anIsDegeneratedTor)
        {
            double deltaOther = 0.5 * (m2[2 * coord + 0] + m2[2 * coord + 1]) - 0.5 * (m1[2 * coord + 0] + m1[2 * coord + 1]);
            if (deltaOther * isneg < 0)
            {
                w1.Reverse();
                w2.Reverse();
            }
        }

        // sort original wires
        var sfw = new ShapeFix_Wire();
        sfw.SetFace(myFace, mySurf);
        sfw.SetPrecision(Precision());
        var wd1 = new ShapeExtend_WireData(w1);
        var wd2 = new ShapeExtend_WireData(w2);
        sfw.Load(wd1);
        sfw.FixReorder();
        sfw.Load(wd2);
        sfw.FixReorder();
        Wire w11 = wd1.InternalWire();
        Wire w21 = wd2.InternalWire();

        //: abv 29.08.01: reconstruct face taking into account reversing
        Shape dummy = myFace.EmptyCopied();
        Face tmpF = ShapeCasts.Face(dummy);
        tmpF.Orientation(Forward);
        for (i = 1; i <= ws.Length(); i++)
        {
            Wire wire = ShapeCasts.Wire(ws.Value(i));
            if (wire.IsSame(w1))
            {
                wire = w11;
            }
            else if (wire.IsSame(w2))
            {
                wire = w21;
            }
            else
            {
                // other wires (not boundary) are considered as holes; make sure to have them oriented
                // accordingly
                Shape curface = tmpF.EmptyCopied();
                B.Add(curface, wire);
                curface.Orientation(myFace.Orientation());
                if (ShapeAnalysis.IsOuterBound(ShapeCasts.Face(curface)))
                {
                    wire.Reverse();
                }
            }
            B.Add(tmpF, wire);
        }

        tmpF.Orientation(myFace.Orientation());

        // A special kind of FixShifted is necessary for torus-like
        // surfaces to adjust wires by period ALONG the missing SEAM direction
        // tr9_r0501-ug.stp #187640
        if (uclosed && vclosed && !anIsDegeneratedTor)
        {
            double shiftw2 = ShapeAnalysis.AdjustByPeriod(
              0.5 * (m2[2 * coord + 0] + m2[2 * coord + 1]),
              0.5 * (m1[2 * coord + 0] + m1[2 * coord + 1] + isneg * (period + Kerf.Engine.Foundation.Precision.PConfusion())),
              period);
            m1[2 * coord + 0] = Min(m1[2 * coord + 0], m2[2 * coord + 0] + shiftw2);
            m1[2 * coord + 1] = Max(m1[2 * coord + 1], m2[2 * coord + 1] + shiftw2);
            for (var it = new Iterator(tmpF, false); it.More(); it.Next())
            {
                if (it.Value().ShapeType() != ShapeType.Wire)
                {
                    continue;
                }
                Wire w = ShapeCasts.Wire(it.Value());
                if (w == w11)
                {
                    continue;
                }
                double shift;
                if (w == w21)
                {
                    shift = shiftw2;
                }
                else
                {
                    S = tmpF.EmptyCopied();
                    B.Add(S, w);
                    ShapeAnalysis.GetFaceUVBounds(ShapeCasts.Face(S), out m2[0], out m2[1], out m2[2], out m2[3]);
                    shift = ShapeAnalysis.AdjustByPeriod(0.5 * (m2[2 * coord + 0] + m2[2 * coord + 1]),
                                                         0.5 * (m1[2 * coord + 0] + m1[2 * coord + 1]),
                                                         period);
                }
                if (shift != 0.0)
                {
                    var V = new Vector2D(0.0, 0.0);
                    V.SetCoord(coord + 1, shift);
                    var sae1 = new ShapeAnalysis_Edge();
                    for (var iw = new Iterator(w); iw.More(); iw.Next())
                    {
                        Edge E = ShapeCasts.Edge(iw.Value());
                        if (!sae1.PCurve(E, tmpF, out Curve2D? C, out _, out _))
                        {
                            continue;
                        }
                        EditPCurve(E, C!, c => c.Translate(V));
                    }
                }
            }
            // abv 05 Feb 02: OCC34
            // by the way, select proper split place by V to avoid extra intersections
            if (m1[2 * coord + 1] - m1[2 * coord + 0] <= period)
            {
                double other = 0.5 * (m1[2 * coord + 0] + m1[2 * coord + 1] - period);
                if (ismodeu != 0)
                {
                    vf = other;
                }
                else
                {
                    uf = other;
                }
            }
        }

        // find the best place by u and v to insert a seam
        // (so as to minimize splitting edges as possible)
        var sae = new ShapeAnalysis_Edge();
        int foundU = 0, foundV = 0;
        int nb1 = wd1.NbEdges();
        int nb2 = wd2.NbEdges();
        for (int i1 = 1; i1 <= nb1 + nb2; i1++)
        {
            Edge edge1 = (i1 <= nb1 ? wd1.Edge(i1) : wd2.Edge(i1 - nb1));
            if (!sae.PCurve(edge1, tmpF, out Curve2D? c2d, out double f, out double l, true))
            {
                return false;
            }
            Point2D pos1 = new Point2D(c2d!.Value(l).XY());
            // the best place is end of edge which is nearest to 0
            bool skipU = !uclosed;
            if (uclosed && ismodeu != 0)
            {
                pos1.SetX(pos1.X() + ShapeAnalysis.AdjustByPeriod(pos1.X(), SUF, URange));
                if (foundU == 2 && Math.Abs(pos1.X()) > Math.Abs(uf))
                {
                    skipU = true;
                }
                else if (foundU == 0 || (foundU == 1 && Math.Abs(pos1.X()) < Math.Abs(uf)))
                {
                    foundU = 1;
                    uf     = pos1.X();
                }
            }
            bool skipV = !vclosed;
            if (vclosed && ismodeu == 0)
            {
                pos1.SetY(pos1.Y() + ShapeAnalysis.AdjustByPeriod(pos1.Y(), SVF, VRange));
                if (foundV == 2 && Math.Abs(pos1.Y()) > Math.Abs(vf))
                {
                    skipV = true;
                }
                else if (foundV == 0 || (foundV == 1 && Math.Abs(pos1.Y()) < Math.Abs(vf)))
                {
                    foundV = 1;
                    vf     = pos1.Y();
                }
            }
            if (skipU && skipV)
            {
                if (i1 <= nb1)
                {
                    continue;
                }
                else
                {
                    break;
                }
            }
            // or yet better - if it is end of some edges on both wires
            for (int i2 = 1; i1 <= nb1 && i2 <= nb2; i2++)
            {
                Edge edge2 = wd2.Edge(i2);
                if (!sae.PCurve(edge2, tmpF, out c2d, out f, out l, true))
                {
                    return false;
                }
                Point2D pos2 = new Point2D(c2d!.Value(f).XY());
                if (uclosed && ismodeu != 0)
                {
                    pos2.SetX(pos2.X() + ShapeAnalysis.AdjustByPeriod(pos2.X(), pos1.X(), URange));
                    if (Math.Abs(pos2.X() - pos1.X()) < Kerf.Engine.Foundation.Precision.PConfusion()
                        && (foundU != 2 || Math.Abs(pos1.X()) < Math.Abs(uf)))
                    {
                        foundU = 2;
                        uf     = pos1.X();
                    }
                }
                if (vclosed && ismodeu == 0)
                {
                    pos2.SetY(pos2.Y() + ShapeAnalysis.AdjustByPeriod(pos2.Y(), pos1.Y(), VRange));
                    if (Math.Abs(pos2.Y() - pos1.Y()) < Kerf.Engine.Foundation.Precision.PConfusion()
                        && (foundV != 2 || Math.Abs(pos1.Y()) < Math.Abs(vf)))
                    {
                        foundV = 2;
                        vf     = pos1.Y();
                    }
                }
            }
        }

        // pdn fixing RTS on offsets
        if (uf < SUF || uf > SUL)
        {
            uf += ShapeAnalysis.AdjustToPeriod(uf, SUF, SUF + URange);
        }
        if (vf < SVF || vf > SVL)
        {
            vf += ShapeAnalysis.AdjustToPeriod(vf, SVF, SVF + VRange);
        }

        // Create fictive grid and call ComposeShell to insert a seam
        var RTS = new TrimmedSurface(mySurf.Surface()!, uf, uf + URange, vf, vf + VRange);
        var grid = new NCollection_HArray2<Surface>(1, 1, 1, 1);
        grid.SetValue(1, 1, RTS); // mySurf->Surface() );
        var G = new ShapeExtend_CompositeSurface(grid);
        var L = new Placement();

        // addition non-manifold topology
        int j = 1;
        for (; j <= aSeqNonManif.Length(); j++)
        {
            B.Add(tmpF, aSeqNonManif.Value(j));
        }

        var CompShell = new ShapeFix_ComposeShell();
        CompShell.Init(G, L, tmpF, Kerf.Engine.Foundation.Precision.Confusion()); // myPrecision
        if (Context() is null)
        {
            SetContext(new ShapeBuild_ReShape());
        }
        CompShell.ClosedMode() = true;
        CompShell.SetContext(Context());
        CompShell.SetMaxTolerance(MaxTolerance());
        CompShell.Perform();

        // abv 03.07.00: CAX-IF TRJ4: trj4_k1_goe-tu-214.stp: #785: reset mySurf
        mySurf = new ShapeAnalysis_Surface(RTS);

        myResult = CompShell.InternalResult();

        Context()!.Replace(myFace, myResult);

        // Remove small wires and / or faces that can be generated by ComposeShell
        // (see tests bugs step bug30052_4, de step_3 E6)
        int nbFaces = 0;
        var expF = new Explorer(myResult, global::Kerf.Topology.ShapeType.Face);
        for (; expF.More(); expF.Next())
        {
            Face aFace = ShapeCasts.Face(expF.Value());
            var aExpW = new Explorer(aFace, ShapeType.Wire);
            int nbWires = 0;
            for (; aExpW.More(); aExpW.Next())
            {
                var aSfw = new ShapeFix_Wire(ShapeCasts.Wire(aExpW.Value()), aFace, Precision());
                aSfw.SetContext(Context());
                if (aSfw.NbEdges() != 0)
                {
                    aSfw.FixSmall(true, Precision());
                }
                if (aSfw.NbEdges() == 0)
                {
                    Context()!.Remove(aExpW.Value());
                    continue;
                }
                nbWires++;
            }
            if (nbWires == 0)
            {
                Context()!.Remove(aFace);
                continue;
            }
            nbFaces++;
        }

        myResult = Context()!.InternalApply(myResult);
        for (var exp = new Explorer(myResult, global::Kerf.Topology.ShapeType.Face); exp.More(); exp.Next())
        {
            myFace = ShapeCasts.Face(Context()!.InternalApply(exp.Current()));
            if (myFace.IsNull)
            {
                continue;
            }
            if (nbFaces > 1)
            {
                fixSmallAreaWire(true);
                Shape aShape = Context()!.InternalApply(myFace);
                if (aShape.IsNull)
                {
                    continue;
                }
                myFace = ShapeCasts.Face(aShape);
            }
            BrepTools.Update(myFace); //: p4
        }
        myResult = Context()!.InternalApply(myResult);

        SendWarning(new Msg("FixAdvFace.FixMissingSeam.MSG0")); // Missing seam-edge added
        return true;
    }

    /// <summary>
    /// Removes the wires of null area from the face; with <paramref name="theIsRemoveSmallFace"/>, removes the face
    /// (through the context) when all its wires are. Returns true if a wire was removed and the face kept.
    /// </summary>
    public bool FixSmallAreaWire(bool theIsRemoveSmallFace)
    {
        enterWork();
        return fixSmallAreaWire(theIsRemoveSmallFace);
    }

    //%14 pdn 24.02.99 PRO10109, USA60293 fix wire on face with small area.
    private bool fixSmallAreaWire(bool theIsRemoveSmallFace)
    {
        if (Context() is not null)
        {
            Shape aShape0 = Context()!.InternalApply(myFace);
            myFace = ShapeCasts.Face(aShape0);
        }

        var aBuilder = new BrepBuilder();
        int nbRemoved = 0, nbWires = 0;

        Shape anEmptyCopy = myFace.EmptyCopied();
        Face aFace = ShapeCasts.Face(anEmptyCopy);
        aFace.Orientation(Forward);

        double aTolerance3d = base.Precision();
        for (var aWIt = new Iterator(myFace, false); aWIt.More(); aWIt.Next())
        {
            Shape aShape = aWIt.Value();
            if (aShape.ShapeType() != ShapeType.Wire && aShape.Orientation() != Forward
                && aShape.Orientation() != Reversed)
            {
                continue;
            }

            // OCCT: the test is && where || is meant, so a FORWARD or REVERSED vertex (or an INTERNAL wire) gets here, and
            // TopoDS::Wire, unchecked in OCCT's release build, reinterprets the vertex as a wire: it has no edge, is not
            // small, and is kept. The port reinterprets it the same way rather than throw.
            Wire aWire = aShape.ShapeType() == ShapeType.Wire
                ? ShapeCasts.Wire(aShape)
                : new Wire(aShape.TShape(), aShape.Location(), aShape.Orientation());
            var anAnalyzer = new ShapeAnalysis_Wire(aWire, myFace, aTolerance3d);
            if (anAnalyzer.CheckSmallArea(aWire))
            {
                // Null area wire detected, wire skipped
                SendWarning(aWire, new Msg("FixAdvFace.FixSmallAreaWire.MSG0"));
                ++nbRemoved;
            }
            else
            {
                aBuilder.Add(aFace, aWire);
                ++nbWires;
            }
        }

        if (nbRemoved <= 0)
        {
            return false;
        }

        if (nbWires <= 0)
        {
            if (theIsRemoveSmallFace && Context() is not null)
            {
                Context()!.Remove(myFace);
            }

            return false;
        }
        aFace.Orientation(myFace.Orientation());
        Context()?.Replace(myFace, aFace);

        myFace = aFace;
        return true;
    }

    private static void FindNext(in Shape aVert, in Shape ainitEdge,
                                 NCollection_IndexedMap<Shape> aMapVertices,
                                 NCollection_DataMap<Shape, NCollection_List<Shape>> aMapVertexEdges,
                                 NCollection_Map<Shape> aMapSmallEdges,
                                 NCollection_Map<Shape> aMapSeemEdges,
                                 NCollection_Map<Shape> aMapEdges,
                                 ShapeExtend_WireData aWireData)
    {
        var aItV = new Iterator(ainitEdge);
        Shape anextVert = aVert;
        bool isFind = false;
        for (; aItV.More() && !isFind; aItV.Next())
        {
            if (!aItV.Value().IsSame(aVert))
            {
                isFind    = true;
                anextVert = aItV.Value();
            }
        }

        if (!isFind && !aMapSmallEdges.Contains(ainitEdge))
        {
            return;
        }
        if (isFind && aMapVertices.Contains(anextVert))
        {
            return;
        }

        NCollection_List<Shape> aledges = aMapVertexEdges.Find(anextVert);
        var liter = new NCollection_List<Shape>.Iterator(aledges);
        isFind = false;
        Shape anextEdge;
        for (; liter.More() && !isFind; liter.Next())
        {
            if (!aMapEdges.Contains(liter.Value()) && !liter.Value().IsSame(ainitEdge))
            {
                anextEdge = liter.Value();
                aWireData.Add(anextEdge);
                if (aMapSeemEdges.Contains(anextEdge))
                {
                    aWireData.Add(anextEdge.Reversed());
                }
                isFind = true;
                aMapEdges.Add(anextEdge);
                FindNext(anextVert, anextEdge, aMapVertices, aMapVertexEdges, aMapSmallEdges, aMapSeemEdges, aMapEdges,
                         aWireData);
            }
        }
    }

    private static bool isClosed2D(in Face aFace, in Wire aWire)
    {
        bool isClosed = true;
        var asaw = new ShapeAnalysis_Wire(aWire, aFace, Kerf.Engine.Foundation.Precision.Confusion());
        for (int i = 1; i <= asaw.NbEdges() && isClosed; i++)
        {
            Edge edge1 = asaw.WireData()!.Edge(i);
            // checking that wire is closed in 2D space with tolerance of vertex.
            var sae = new ShapeAnalysis_Edge();
            Vertex v1 = sae.FirstVertex(edge1);
            asaw.SetPrecision(Tool.Tolerance(v1));
            asaw.CheckGap2d(i);
            isClosed = (asaw.LastCheckStatus(ShapeExtend_OK));
        }
        return isClosed;
    }

    /// <summary>
    /// Splits the wire loaded in the wire tool (<see cref="FixWireTool"/>) at its loops (vertices shared by more than
    /// two edges) into the wires appended to <paramref name="aResWires"/>. Returns true if done and the wires are
    /// closed in 2d.
    /// </summary>
    public bool FixLoopWire(NCollection_Sequence<Shape> aResWires)
    {
        enterWork();
        return fixLoopWire(aResWires);
    }

    private bool fixLoopWire(NCollection_Sequence<Shape> aResWires)
    {
        var aMapVertices = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        var aMapVertexEdges = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var aMapSmallEdges = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var aMapSeemEdges = new NCollection_Map<Shape>(ShapeHasher.Instance);
        if (!FixWireTool().Analyzer().CheckLoop(aMapVertices, aMapVertexEdges, aMapSmallEdges, aMapSeemEdges))
        {
            return false;
        }

        var aMapEdges = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var aSeqWires = new NCollection_Sequence<Shape>();

        // collecting wires from common vertex belonging more than 2 edges
        int i = 1;
        for (; i <= aMapVertices.Extent(); i++)
        {
            Shape aVert = aMapVertices.FindKey(i);
            NCollection_List<Shape> aledges = aMapVertexEdges.Find(aVert);
            var liter = new NCollection_List<Shape>.Iterator(aledges);
            for (; liter.More(); liter.Next())
            {
                Edge Edge = ShapeCasts.Edge(liter.Value());
                if (aMapEdges.Contains(Edge))
                {
                    continue;
                }

                var aWireData = new ShapeExtend_WireData();
                aWireData.Add(Edge);
                if (aMapSeemEdges.Contains(Edge))
                {
                    aWireData.Add(Edge.Reversed());
                }
                aMapEdges.Add(Edge);
                FindNext(aVert, Edge, aMapVertices, aMapVertexEdges, aMapSmallEdges, aMapSeemEdges, aMapEdges, aWireData);
                if (aWireData.NbEdges() == 1 && aMapSmallEdges.Contains(aWireData.Edge(1)))
                {
                    continue;
                }
                Wire aWire = aWireData.InternalWire();
                Exploration.Vertices(aWire, out Vertex aV1, out Vertex aV2);

                if (aV1.IsSame(aV2))
                {
                    var asewd = new ShapeExtend_WireData(aWire);
                    var asfw = new ShapeFix_Wire();
                    asfw.Load(asewd);
                    asfw.FixReorder();
                    Wire awire2 = asfw.InternalWire();
                    aResWires.Append(awire2);
                }
                else
                {
                    aSeqWires.Append(aWireData.InternalWire());
                }
            }
        }

        if (aSeqWires.Length() == 1)
        {
            aResWires.Append(aSeqWires.Value(1));
        }
        else
        {
            // collecting whole wire from two not closed wires having two common vertices.
            for (i = 1; i <= aSeqWires.Length(); i++)
            {
                Wire aWire = ShapeCasts.Wire(aSeqWires.Value(i));
                Exploration.Vertices(aWire, out Vertex aV1, out Vertex aV2);
                int j = i + 1;
                for (; j <= aSeqWires.Length(); j++)
                {
                    Wire aWire2 = ShapeCasts.Wire(aSeqWires.Value(j));
                    Exploration.Vertices(aWire2, out Vertex aV21, out Vertex aV22);
                    if ((aV1.IsSame(aV21) || aV1.IsSame(aV22)) && (aV2.IsSame(aV21) || aV2.IsSame(aV22)))
                    {
                        var asewd = new ShapeExtend_WireData(aWire);
                        asewd.Add(aWire2);
                        var asfw = new ShapeFix_Wire();
                        asfw.Load(asewd);
                        asfw.FixReorder();
                        aResWires.Append(asfw.InternalWire());
                        aSeqWires.Remove(j--);
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE7);
                        break;
                    }
                }
                if (j <= aSeqWires.Length())
                {
                    aSeqWires.Remove(i--);
                }
            }
            if (aSeqWires.Length() < 3)
            {
                for (i = 1; i <= aSeqWires.Length(); i++)
                {
                    aResWires.Append(aSeqWires.Value(i));
                }
            }
            else
            {
                // collecting wires having one common vertex
                for (i = 1; i <= aSeqWires.Length(); i++)
                {
                    Wire aWire = ShapeCasts.Wire(aSeqWires.Value(i));
                    Exploration.Vertices(aWire, out Vertex aV1, out Vertex aV2);
                    int j = i + 1;
                    for (; j <= aSeqWires.Length(); j++)
                    {
                        Wire aWire2 = ShapeCasts.Wire(aSeqWires.Value(j));
                        Exploration.Vertices(aWire2, out Vertex aV21, out Vertex aV22);
                        if ((aV1.IsSame(aV21) || aV1.IsSame(aV22)) || (aV2.IsSame(aV21) || aV2.IsSame(aV22)))
                        {
                            var asewd = new ShapeExtend_WireData(aWire);
                            asewd.Add(aWire2);
                            var asfw = new ShapeFix_Wire();
                            asfw.Load(asewd);
                            asfw.FixReorder();
                            aWire = asfw.InternalWire();
                            Exploration.Vertices(aWire, out aV1, out aV2);
                            aSeqWires.Remove(j--);
                            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE7);
                        }
                    }
                    aResWires.Append(aWire);
                }
            }
        }
        bool isClosed = true;

        // checking that obtained wires is closed in 2D space
        if (mySurf is not null && mySurf.Adaptor3d()!.GetType() != Plane)
        {
            Shape emptyCopied = myFace.EmptyCopied();
            Face tmpFace = ShapeCasts.Face(emptyCopied);
            tmpFace.Orientation(Forward);

            for (i = 1; i <= aResWires.Length() && isClosed; i++)
            {
                Wire awire = ShapeCasts.Wire(aResWires.Value(i));
                isClosed = isClosed2D(tmpFace, awire);
            }
        }

        return !aResWires.IsEmpty() && isClosed;
    }

    private bool SplitEdge(ShapeExtend_WireData sewd, int num, double param, in Vertex vert, double preci,
                           NCollection_DataMap<Shape, Box2d> boxes)
    {
        Edge edge = sewd.Edge(num);
        Edge newE1 = new Edge(), newE2 = new Edge();
        var aTool = new ShapeFix_SplitTool();
        if (aTool.SplitEdge(edge, param, vert, myFace, ref newE1, ref newE2, preci, 0.01 * preci))
        {
            splitEdgeDone(sewd, num, edge, newE1, newE2, boxes);
            return true;
        }
        return false;
    }

    private bool SplitEdge(ShapeExtend_WireData sewd, int num, double param1, double param2, in Vertex vert,
                           double preci, NCollection_DataMap<Shape, Box2d> boxes)
    {
        Edge edge = sewd.Edge(num);
        Edge newE1 = new Edge(), newE2 = new Edge();
        var aTool = new ShapeFix_SplitTool();
        if (aTool.SplitEdge(edge, param1, param2, vert, myFace, ref newE1, ref newE2, preci, 0.01 * preci))
        {
            splitEdgeDone(sewd, num, edge, newE1, newE2, boxes);
            return true;
        }
        return false;
    }

    // The common tail of OCCT's two SplitEdge methods (identical in OCCT).
    private void splitEdgeDone(ShapeExtend_WireData sewd, int num, in Edge edge, in Edge newE1,
                               in Edge newE2, NCollection_DataMap<Shape, Box2d> boxes)
    {
        // change context
        var wd = new ShapeExtend_WireData();
        wd.Add(newE1);
        wd.Add(newE2);
        Context()?.Replace(edge, wd.InternalWire());
        for (var exp = new Explorer(wd.InternalWire(), ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge E = ShapeCasts.Edge(exp.Current());
            BrepTools.Update(E);
        }

        // change sewd and boxes
        sewd.Set(newE1, num);
        if (num == sewd.NbEdges())
        {
            sewd.Add(newE2);
        }
        else
        {
            sewd.Add(newE2, num + 1);
        }

        boxes.UnBind(edge);
        Surface S = Tool.Surface(myFace, out Placement L)!;
        var sae = new ShapeAnalysis_Edge();
        foreach (Edge aNewE in (ReadOnlySpan<Edge>)[newE1, newE2])
        {
            if (sae.PCurve(aNewE, S, L, out Curve2D? c2d, out double cf, out double cl, false))
            {
                var box = new Box2d();
                var gac = new CurveAdaptor2D();
                double aFirst = c2d!.FirstParameter();
                double aLast  = c2d.LastParameter();
                if (c2d is BSplineCurve2D && (cf < aFirst || cl > aLast))
                {
                    // pdn avoiding problems with segment in Bnd_Box
                    gac.Load(c2d);
                }
                else
                {
                    gac.Load(c2d, cf, cl);
                }
                Add2dCurve.Add(gac, Kerf.Engine.Foundation.Precision.Confusion(), ref box);
                boxes.Bind(aNewE, box);
            }
        }
    }

    /// <summary>Fixes the intersections between the wires of the face. Returns true if done.</summary>
    public bool FixIntersectingWires()
    {
        enterWork();
        return fixIntersectingWires();
    }

    private bool fixIntersectingWires()
    {
        var ITool = new ShapeFix_IntersectionTool(Context(), Precision(), MaxTolerance());
        return ITool.FixIntersectingWires(ref myFace);
    }

    /// <summary>Removes the wires made of two coincident edges (a face must keep another wire). Returns true if done.</summary>
    public bool FixWiresTwoCoincEdges()
    {
        enterWork();
        return fixWiresTwoCoincEdges();
    }

    private bool fixWiresTwoCoincEdges()
    {
        if (Context() is not null)
        {
            Shape S = Context()!.InternalApply(myFace);
            myFace = ShapeCasts.Face(S);
        }

        Orientation ori = myFace.Orientation();
        Shape emptyCopied = myFace.EmptyCopied();
        Face face = ShapeCasts.Face(emptyCopied);
        face.Orientation(Forward);
        int nbWires = 0;
        var B = new BrepBuilder();

        for (var it = new Iterator(myFace, false); it.More(); it.Next())
        {
            if (it.Value().ShapeType() != ShapeType.Wire
                || (it.Value().Orientation() != Forward && it.Value().Orientation() != Reversed))
            {
                continue;
            }
            nbWires++;
        }
        if (nbWires < 2)
        {
            return false;
        }
        bool isFixed = false;
        for (var wi = new Iterator(myFace, false); wi.More(); wi.Next())
        {
            if (wi.Value().ShapeType() != ShapeType.Wire
                || (wi.Value().Orientation() != Forward && wi.Value().Orientation() != Reversed))
            {
                B.Add(face, wi.Value());
                continue;
            }
            Wire wire = ShapeCasts.Wire(wi.Value());
            var sewd = new ShapeExtend_WireData(wire);
            if (sewd.NbEdges() == 2)
            {
                Edge E1 = sewd.Edge(1);
                Edge E2 = sewd.Edge(2);
                E1.Orientation(Forward);
                E2.Orientation(Forward);
                if (!(E1 == E2))
                {
                    B.Add(face, wire);
                }
                else
                {
                    isFixed = true;
                }
            }
            else
            {
                B.Add(face, wire);
            }
        }
        if (isFixed)
        {
            face.Orientation(ori);
            Context()?.Replace(myFace, face);
            myFace = face;
        }

        return isFixed;
    }

    /// <summary>
    /// Splits the face into several faces, one per outer wire of <paramref name="MapWires"/> (filled by
    /// <see cref="FixOrientation(NCollection_DataMap{Shape, NCollection_List{Shape}})"/>) with the wires
    /// inside it. Returns true if done; the result is then a compound (<see cref="Result"/>).
    /// </summary>
    public bool FixSplitFace(NCollection_DataMap<Shape, NCollection_List<Shape>> MapWires)
    {
        enterWork();
        return fixSplitFace(MapWires);
    }

    private bool fixSplitFace(NCollection_DataMap<Shape, NCollection_List<Shape>> MapWires)
    {
        var B = new BrepBuilder();
        var faces = new NCollection_Sequence<Shape>();
        Shape S = myFace;
        if (Context() is not null)
        {
            S = Context()!.InternalApply(myFace);
        }
        int NbWires = 0, NbWiresNew = 0, NbEdges;
        for (var iter = new Iterator(S, false); iter.More(); iter.Next())
        {
            Shape aShape = iter.Value();
            if (aShape.ShapeType() != ShapeType.Wire
                || (aShape.Orientation() != Forward && aShape.Orientation() != Reversed))
            {
                continue;
            }
            Wire wire = ShapeCasts.Wire(aShape);
            NbWires++;
            if (MapWires.IsBound(wire))
            {
                // if wire not closed --> stop split and return false
                var sewd = new ShapeExtend_WireData(wire);
                NbEdges = sewd.NbEdges();
                if (NbEdges == 0)
                {
                    continue;
                }
                //
                Edge E1 = sewd.Edge(1);
                Edge E2 = sewd.Edge(NbEdges);
                var sae = new ShapeAnalysis_Edge();
                Vertex V1 = sae.FirstVertex(E1);
                Vertex V2 = sae.LastVertex(E2);
                if (!V1.IsSame(V2))
                {
                    return false;
                }
                // create face
                Shape emptyCopied = S.EmptyCopied();
                Face tmpFace = ShapeCasts.Face(emptyCopied);
                tmpFace.Orientation(Forward);
                B.Add(tmpFace, wire);
                NbWiresNew++;
                NCollection_List<Shape> IntWires = MapWires.Find(wire);
                var liter = new NCollection_List<Shape>.Iterator(IntWires);
                for (; liter.More(); liter.Next())
                {
                    Shape aShapeEmptyCopied = tmpFace.EmptyCopied();
                    Face aFace = ShapeCasts.Face(aShapeEmptyCopied);
                    aFace.Orientation(Forward);
                    B.Add(aFace, liter.Value());
                    var clas = new BRepTopAdaptor_FClass2d(aFace, Kerf.Engine.Foundation.Precision.PConfusion());
                    Containment staout = clas.PerformInfinitePoint();
                    if (staout == In)
                    {
                        B.Add(tmpFace, liter.Value());
                    }
                    else
                    {
                        B.Add(tmpFace, liter.Value().Reversed());
                    }
                    NbWiresNew++;
                }
                if (!myFwd)
                {
                    tmpFace.Orientation(Reversed);
                }
                faces.Append(tmpFace);
            }
        }

        if (NbWires != NbWiresNew)
        {
            return false;
        }

        if (faces.Length() > 1)
        {
            var Comp = new Compound();
            B.MakeCompound(ref Comp);
            for (int i = 1; i <= faces.Length(); i++)
            {
                B.Add(Comp, faces[i]);
            }
            myResult = Comp;

            Context()?.Replace(myFace, myResult);

            for (var exp = new Explorer(myResult, global::Kerf.Topology.ShapeType.Face); exp.More(); exp.Next())
            {
                myFace = ShapeCasts.Face(exp.Current());
                BrepTools.Update(myFace);
            }
            return true;
        }

        return false;
    }

    // Checks whether the passed wire makes up a periodic loop on passed conical surface
    private static bool IsPeriodicConicalLoop(ConicalSurface? theSurf, in Wire theWire, double theTolerance,
                                              ref double theMinU, ref double theMaxU, ref double theMinV,
                                              ref double theMaxV, ref bool isUDecrease)
    {
        if (theSurf is null)
        {
            return false;
        }

        var aSAE = new ShapeAnalysis_Edge();
        var aLoc = new Placement();

        double aCumulDeltaU = 0.0, aCumulDeltaUAbs = 0.0;
        double aMinU = RealLast();
        double aMinV = aMinU;
        double aMaxU = -aMinU;
        double aMaxV = aMaxU;

        // Iterate over the edges to check whether the wire is periodic on conical surface
        var aWireIter = new Iterator(theWire, false);
        for (; aWireIter.More(); aWireIter.Next())
        {
            Edge aCurrentEdge = ShapeCasts.Edge(aWireIter.Value());

            aSAE.PCurve(aCurrentEdge, theSurf, aLoc, out Curve2D? aC2d, out double aPFirst, out double aPLast, true);

            if (aC2d is null)
            {
                return false;
            }

            Point2D aUVFirst = aC2d.Value(aPFirst), aUVLast = aC2d.Value(aPLast);

            double aUFirst = aUVFirst.X(), aULast = aUVLast.X();
            double aVFirst = aUVFirst.Y(), aVLast = aUVLast.Y();

            double aCurMaxU = Max(aUFirst, aULast), aCurMinU = Min(aUFirst, aULast);
            double aCurMaxV = Max(aVFirst, aVLast), aCurMinV = Min(aVFirst, aVLast);

            if (aCurMinU < aMinU)
            {
                aMinU = aCurMinU;
            }
            if (aCurMaxU > aMaxU)
            {
                aMaxU = aCurMaxU;
            }
            if (aCurMinV < aMinV)
            {
                aMinV = aCurMinV;
            }
            if (aCurMaxV > aMaxV)
            {
                aMaxV = aCurMaxV;
            }

            double aDeltaU = aULast - aUFirst;

            aCumulDeltaU += aDeltaU;
            aCumulDeltaUAbs += Math.Abs(aDeltaU);
        }

        theMinU     = aMinU;
        theMaxU     = aMaxU;
        theMinV     = aMinV;
        theMaxV     = aMaxV;
        isUDecrease = (aCumulDeltaU < 0);

        bool is2PIDelta   = Math.Abs(aCumulDeltaUAbs - 2 * M_PI) <= theTolerance;
        bool isAroundApex = Math.Abs(theMaxU - theMinU) > 2 * M_PI - theTolerance;

        return is2PIDelta && isAroundApex;
    }

    /// <summary>
    /// Adds the degenerated edge at the apex of a cone to a face whose single wire goes around the cone by a period.
    /// Returns true if done.
    /// </summary>
    public bool FixPeriodicDegenerated()
    {
        enterWork();
        return fixPeriodicDegenerated();
    }

    private bool fixPeriodicDegenerated()
    {
        /* =====================
         *  Prepare fix routine
         * ===================== */

        if (Context() is not null)
        {
            Shape aSh = Context()!.InternalApply(myFace);
            myFace = ShapeCasts.Face(aSh);
        }

        /* ================================================
         *  Check if fix can be applied on the passed face
         * ================================================ */

        // Collect all wires owned by the face
        var aWireSeq = new NCollection_Sequence<Shape>();
        for (var aWireIt = new Iterator(myFace, false); aWireIt.More(); aWireIt.Next())
        {
            Shape aSubSh = aWireIt.Value();
            if (aSubSh.ShapeType() != ShapeType.Wire
                || (aSubSh.Orientation() != Forward && aSubSh.Orientation() != Reversed))
            {
                continue;
            }

            aWireSeq.Append(aWireIt.Value());
        }

        // Get number of wires and surface
        int aNbWires = aWireSeq.Length();
        Surface? aSurface = Tool.Surface(myFace);

        // Only single wires on conical surfaces are checked
        if (aNbWires != 1 || aSurface is null || aSurface.GetType() != typeof(ConicalSurface))
        {
            return false;
        }

        // Get the single wire
        Wire aSoleWire = ShapeCasts.Wire(aWireSeq.Value(1));

        // Check whether this wire is belting the conical surface by period
        var aConeSurf = (ConicalSurface)aSurface;
        double aMinLoopU = 0.0, aMaxLoopU = 0.0, aMinLoopV = 0.0, aMaxLoopV = 0.0;
        bool isUDecrease = false;

        bool isConicLoop = IsPeriodicConicalLoop(aConeSurf, aSoleWire, Precision(), ref aMinLoopU, ref aMaxLoopU,
                                                 ref aMinLoopV, ref aMaxLoopV, ref isUDecrease);

        if (!isConicLoop)
        {
            return false;
        }

        /* ===============
         *  Retrieve apex
         * =============== */

        // Get base circle of the conical surface (the circle it was built from)
        Curve3D aConeBaseCrv = aConeSurf.VIso(0.0);
        var aConeBaseCirc = (CircleCurve3D)aConeBaseCrv;

        // Retrieve conical props
        double aConeBaseR = aConeBaseCirc.Radius();
        double aSemiAngle = aConeSurf.SemiAngle();

        if (Math.Abs(aSemiAngle) <= Kerf.Engine.Foundation.Precision.Confusion())
        {
            return false; // Bad surface
        }

        // Find the V parameter of the apex
        double aConeBaseH = aConeBaseR / Math.Sin(aSemiAngle);
        double anApexV    = -aConeBaseH;

        // Get apex vertex
        Vertex anApex = new BRepBuilderAPI_MakeVertex(aConeSurf.Apex()).InternalVertex();

        // ====================================
        //  Build degenerated edge in the apex
        // ====================================

        var anApexEdge = new Edge();
        var aBuilder = new BrepBuilder();
        aBuilder.MakeEdge(ref anApexEdge);

        // Check if positional relationship between the initial wire and apex
        // line in 2D is going to be consistent
        if (Math.Abs(anApexV - aMinLoopV) <= Precision() || Math.Abs(anApexV - aMaxLoopV) <= Precision()
            || (anApexV < aMaxLoopV && anApexV > aMinLoopV))
        {
            return false;
        }

        LineCurve2D? anApexCurve2d = null;

        // Apex curve below the wire
        if (anApexV < aMinLoopV)
        {
            anApexCurve2d = new LineCurve2D(new Point2D(aMinLoopU, anApexV), new Direction2D(Direction2D.D.X));
            if (!isUDecrease)
            {
                aSoleWire.Reverse();
            }
        }

        // Apex curve above the wire
        if (anApexV > aMaxLoopV)
        {
            anApexCurve2d = new LineCurve2D(new Point2D(aMaxLoopU, anApexV), new Direction2D(Direction2D.D.NX));
            if (isUDecrease)
            {
                aSoleWire.Reverse();
            }
        }

        // Create degenerated edge & wire for apex
        aBuilder.UpdateEdge(anApexEdge, anApexCurve2d!, myFace, Precision());
        aBuilder.Add(anApexEdge, anApex);
        aBuilder.Add(anApexEdge, anApex.Reversed());
        aBuilder.Degenerated(anApexEdge, true);
        aBuilder.Range(anApexEdge, 0, Math.Abs(aMaxLoopU - aMinLoopU));
        Wire anApexWire = new BRepBuilderAPI_MakeWire(anApexEdge).InternalWire();

        // ===============================================================
        //  Finalize the fix building new face and setting up the results
        // ===============================================================

        // Collect the resulting set of wires
        var aNewWireSeq = new NCollection_Sequence<Shape>();
        aNewWireSeq.Append(aSoleWire);
        aNewWireSeq.Append(anApexWire);

        // Assemble new face
        Face aNewFace = ShapeCasts.Face(myFace.EmptyCopied());
        aNewFace.Orientation(Forward);
        var aFaceBuilder = new BrepBuilder();
        for (int i = 1; i <= aNewWireSeq.Length(); i++)
        {
            Wire aNewWire = ShapeCasts.Wire(aNewWireSeq.Value(i));
            aFaceBuilder.Add(aNewFace, aNewWire);
        }
        aNewFace.Orientation(myFace.Orientation());

        // Adjust the resulting state of the healing tool
        myResult = aNewFace;
        Context()?.Replace(myFace, myResult);

        return true;
    }

    /// <summary>Returns the status of the last <see cref="Perform"/> (see there for the DONE bits).</summary>
    public bool Status(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatus, status);

    /// <summary>Returns the tool fixing the wires.</summary>
    public ShapeFix_Wire FixWireTool() => myFixWire;
}
