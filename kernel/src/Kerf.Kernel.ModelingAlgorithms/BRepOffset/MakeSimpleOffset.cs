// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeSimpleOffset.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeSimpleOffset.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Offsets.SimpleOffsetStatus;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Offsets;

/// <summary>The status of <see cref="MakeSimpleOffset"/>.</summary>
public enum SimpleOffsetStatus
{
    OK,
    NullInputShape,
    ErrorOffsetComputation,
    ErrorWallFaceComputation,
    ErrorInvalidNbShells,
    ErrorNonClosedShell,
}

/// <summary>
/// The simple offset algorithm: builds the offset of a shape without intersection, as a shell, or as a solid closed by
/// wall faces between the free boundaries of the input and their images (<see cref="SetBuildSolidFlag"/>).
/// </summary>
/// <remarks>
/// <para>Algorithm:</para>
/// <list type="number">
/// <item><description>Build source-image maps for vertices, edges and faces (<see cref="SimpleOffset"/>).
/// The image of a shared edge is built from the corresponding edge of the first face met.</description></item>
/// <item><description>Run <see cref="Modifier"/> to obtain the offset shape.</description></item>
/// <item><description>Ensure the topological integrity of the output shape.</description></item>
/// </list>
/// <para>
/// The result depends on the smoothness of the input: a smooth (G1) input gives a good result. In general the
/// tolerances grow, to cover the gaps between neighbouring faces in the output, with the offset distance and the
/// quality of the joints between the input faces.
/// </para>
/// <para>
/// The modifier, in its immutable-input mode, is the copy of the input (decision 973): the offset shell never writes
/// into the caller's shape. Building the solid does: the walls give the input's free edges pcurves (in place, as
/// OCCT), and <c>ShapeFix_Edge</c> makes them same-parameter. So, with the solid flag, the algorithm runs on the
/// caller's shape in a working overlay (unit 10H6, <see cref="WorkingOverlay"/>; in place inside an operation's
/// overlay that takes writes): the writes into the caller's frozen edges go to working copies, and the result holds new
/// TShapes only for the sub-shapes that changed. A caller's unfrozen shape (a shell it built of frozen faces) is worked
/// on with its unfrozen TShapes copied, so the run never writes into it. <see cref="Generated"/> and
/// <see cref="Modified"/> answer for the caller's sub-shapes; <see cref="Modified"/> of a sub-shape changed in place
/// is what stands for it in the result.
/// </para>
/// <para>
/// <see cref="GetResultShape"/>, <see cref="Generated"/> and <see cref="Modified"/> freeze what they return (a
/// publication point); ported code reads the <c>Internal</c> forms.
/// </para>
/// </remarks>
public sealed class MakeSimpleOffset
{
    // Input data.
    // Input shape.
    private Shape _inputShape = new();
    // Offset value.
    private double _offsetValue;
    // Tolerance (for singularities)
    private double _tolerance;
    // Solid building flag. True means solid construction.
    private bool _isBuildSolid;

    // Internal data.
    // Maximal angle in faces junction. This value helps to estimate result tolerance.
    private double _maxAngle;
    // Error message.
    private SimpleOffsetStatus _error;
    // Done state.
    private bool _isDone;
    // Map of vertex - wall edge.
    // Used to build shared edge between adjacent wall faces.
    private readonly Dictionary<Vertex, Edge> _mapVE =
        new(EqualityComparer<Vertex>.Default); // the default hasher of OCCT (IsEqual)
    // Used for histrory support.
    private readonly Modifier _builder = new();
    // Used for history support.
    private readonly ReShapeBuilder _reShape;

    // Output data.
    // Result shape.
    private Shape _resShape = new();

    // Kerf: the shape the algorithm works on (the input, or the input with its unfrozen TShapes copied), and the working
    // overlay of the solid's run (unit 10H6); null for the shell, and for a run in place inside an operation's overlay.
    private Shape _shape = new();
    private WorkingOverlay? _overlay;
    private Dictionary<TShape, TShape>? _copies; // the caller's unfrozen TShapes -> their copies

    //=================================================================================================

    /// <summary>Constructor. Does nothing.</summary>
    public MakeSimpleOffset()
    {
        _offsetValue = 0.0;
        _tolerance = global::Kerf.Engine.Foundation.Precision.Confusion();
        _isBuildSolid = false;
        _maxAngle = 0.0;
        _error = SimpleOffsetStatus.OK;
        _isDone = false;
        _reShape = new ReShapeBuilder();
    }

    //=================================================================================================

    /// <summary>Prepares the offset of <paramref name="inputShape"/> by <paramref name="offsetValue"/>.</summary>
    public MakeSimpleOffset(in Shape inputShape, double offsetValue)
    {
        _inputShape = inputShape;
        _offsetValue = offsetValue;
        _tolerance = global::Kerf.Engine.Foundation.Precision.Confusion();
        _isBuildSolid = false;
        _maxAngle = 0.0;
        _error = SimpleOffsetStatus.OK;
        _isDone = false;
        _reShape = new ReShapeBuilder();
    }

    //=================================================================================================

    /// <summary>Initialises the shape to offset and the offset value.</summary>
    public void Initialize(in Shape inputShape, double offsetValue)
    {
        _inputShape = inputShape;
        _offsetValue = offsetValue;
        Clear();
    }

    //=================================================================================================

    /// <summary>The error message.</summary>
    public string GetErrorMessage()
    {
        string error = "";

        if (_error == SimpleOffsetStatus.NullInputShape)
        {
            error = "Null input shape";
            return error;
        }
        else if (_error == SimpleOffsetStatus.ErrorOffsetComputation)
        {
            error = "Error during offset construction";
            return error;
        }
        else if (_error == SimpleOffsetStatus.ErrorWallFaceComputation)
        {
            error = "Error during building wall face";
            return error;
        }
        else if (_error == SimpleOffsetStatus.ErrorInvalidNbShells)
        {
            error = "Result contains two or more shells";
            return error;
        }
        else if (_error == SimpleOffsetStatus.ErrorNonClosedShell)
        {
            error = "Result shell is not closed";
            return error;
        }

        return error;
    }

    /// <summary>The error code.</summary>
    public SimpleOffsetStatus GetError() => _error;

    // Inline methods.
    /// <summary>The solid building flag.</summary>
    public bool GetBuildSolidFlag() => _isBuildSolid;

    /// <summary>Sets the solid building flag (false, the default, builds a shell).</summary>
    public void SetBuildSolidFlag(bool buildFlag) => _isBuildSolid = buildFlag;

    /// <summary>The offset value.</summary>
    public double GetOffsetValue() => _offsetValue;

    /// <summary>Sets the offset value.</summary>
    public void SetOffsetValue(double offsetValue) => _offsetValue = offsetValue;

    /// <summary>The tolerance (used for handling singularities).</summary>
    public double GetTolerance() => _tolerance;

    /// <summary>Sets the tolerance (used for handling singularities).</summary>
    public void SetTolerance(double value) => _tolerance = value;

    /// <summary>True if the offset was computed.</summary>
    public bool IsDone => _isDone;

    /// <summary>The result shape (frozen).</summary>
    public Shape GetResultShape()
    {
        _resShape.TShape()?.FreezeGraph();
        return _resShape;
    }

    /// <summary><see cref="GetResultShape"/> without freezing, for ported code.</summary>
    internal Shape InternalGetResultShape() => _resShape;

    //=================================================================================================

    // Clears previous result.
    private void Clear()
    {
        _isDone = false;
        _error = SimpleOffsetStatus.OK;
        _maxAngle = 0.0;
        _mapVE.Clear();
        _reShape.Clear(); // Clear possible stored modifications.
        // Kerf: the previous run's overlay
        _overlay = null;
        _copies = null;
        _shape = _inputShape;
    }

    //=================================================================================================

    /// <summary>The maximal offset value that keeps the result's tolerance below <paramref name="expectedToler"/>.</summary>
    public double GetSafeOffset(double expectedToler)
    {
        if (_inputShape.IsNull)
        {
            return 0.0; // Input shape is null.
        }

        // Compute max angle in faces junctions.
        if (_maxAngle == 0.0)
        { // Non-initialized.
            ComputeMaxAngle();
        }

        double maxTol = 0.0;
        maxTol = Tool.MaxTolerance(_inputShape, ShapeType.Vertex);

        // OCCT: a shape without sharp junctions has myMaxAngle 0 (recomputed on every call): the division gives an
        // infinite (or NaN) offset
        double expOffset = Max((expectedToler - maxTol) / (2.0 * _maxAngle),
                                 0.0); // Minimal distance can't be lower than 0.0.
        return expOffset;
    }

    //=================================================================================================

    /// <summary>Computes the offset shape.</summary>
    public void Perform()
    {
        // Clear result of previous computations.
        Clear();

        // Check shape existence.
        if (_inputShape.IsNull)
        {
            _error = SimpleOffsetStatus.NullInputShape;
            return;
        }

        if (_maxAngle == 0.0)
        { // Non-initialized.
            ComputeMaxAngle();
        }

        // Kerf: the walls of the solid write into the input's free edges, in a working overlay (unit 10H6; the walls'
        // faces, made by BRepLib_MakeFace, write into the same overlay and share the caller's edges)
        if (_isBuildSolid && WorkingOverlay.Writing is null)
        {
            _shape = WorkingOverlay.WithUnfrozenCopied(_inputShape, out _copies);
            _overlay = WorkingOverlay.Begin(_shape, _reShape);
        }
        try
        {
            PerformCore();
        }
        finally
        {
            _overlay?.Dispose();
        }
    }

    private void PerformCore()
    {
        _builder.Init(_shape);
        var mapper = new SimpleOffset(_shape, _offsetValue, _tolerance);
        _builder.Perform(mapper);

        if (!_builder.IsDone)
        {
            _error = SimpleOffsetStatus.ErrorOffsetComputation;
            FinishEntry();
            return;
        }

        _resShape = _builder.InternalModifiedShape(_shape);

        // Fix degeneracy. Degenerated edge should be mapped to the degenerated.
        var bb = new BrepBuilder();
        var expSE = new Explorer(_shape, ShapeType.Edge);
        for (; expSE.More(); expSE.Next())
        {
            Edge currEdge = ShapeCasts.Edge(expSE.Current());

            if (!Tool.Degenerated(currEdge))
            {
                continue;
            }

            Edge edge = ShapeCasts.Edge(_builder.InternalModifiedShape(currEdge));
            bb.Degenerated(edge, true);
        }

        // Restore walls for solid.
        if (_isBuildSolid && !BuildMissingWalls())
        {
            FinishEntry();
            return;
        }

        FinishEntry();
        _isDone = true;
    }

    // Kerf: the end of the solid's overlay: the result holds new TShapes for the caller's sub-shapes the run changed, the
    // caller's own for the rest (and for the unfrozen ones copied at the start that it left equal); the healing context
    // records each sub-shape changed in place as replaced (unit 10H6).
    private void FinishEntry()
    {
        if (_overlay is not null && !_overlay.IsFinished)
        {
            _resShape = WorkingOverlay.UncopiedWhereUnchanged(_overlay.Finish(_resShape), _copies);
        }
    }

    //=============================================================================
    // function : tgtfaces
    // purpose  : check the angle at the border between two squares.
    //           Two shares should have a shared front edge.
    //=============================================================================
    private static void Tgtfaces(in Edge ed, in Face f1, in Face f2, bool couture,
                                 ref double resAngle)
    {
        // Check that pcurves exist on both faces of edge.
        double first, last;
        Curve2D? curve;
        curve = Tool.CurveOnSurface(ed, f1, out first, out last);
        if (curve is null)
        {
            return;
        }
        curve = Tool.CurveOnSurface(ed, f2, out first, out last);
        if (curve is null)
        {
            return;
        }

        double u;
        Edge e = ed;
        // OCCT.NET: OCCT makes aBAS1 and aBAS2, then copies them into HS1 and HS2; the copies are made directly
        var hs1 = new FaceSurfaceAdaptor(f1, false);
        FaceSurfaceAdaptor hs2;
        if (couture)
        {
            hs2 = hs1;
        }
        else
        {
            hs2 = new FaceSurfaceAdaptor(f2, false);
        }
        // case when edge lies on the one face

        e.Orient(Forward);
        var c2d1 = new EdgeCurve2DAdaptor(e, f1);
        if (couture)
        {
            e.Orient(Reversed);
        }
        var c2d2 = new EdgeCurve2DAdaptor(e, f2);

        bool rev1 = (f1.Orientation == Reversed);
        bool rev2 = (f2.Orientation == Reversed);
        double f, l, eps;
        Tool.Range(e, out f, out l);
        // OCCT.NET: OCCT's unused local Extrema_LocateExtPC ext is left out

        eps = (l - f) / 100.0;
        f += eps; // to avoid calculations on
        l -= eps; // points of pointed squares.
        Point2D p;
        Point3D pp1, pp2; //,PP;
        Vector3D du1, dv1;
        Vector3D du2, dv2;
        Vector3D d1, d2;
        double norm;

        const int nbpnt = 23;
        for (int i = 0; i <= nbpnt; i++)
        {
            // First suppose that this is sameParameter
            u = f + (l - f) * i / nbpnt;

            // take derivatives of surfaces at the same u, and compute normals
            c2d1.D0(u, out p);
            hs1.D1(p.X, p.Y, out pp1, out du1, out dv1);
            d1 = (du1.Crossed(dv1));
            norm = d1.Magnitude;
            if (norm > 1.0e-12)
            {
                d1 /= norm;
            }
            else
            {
                continue; // skip degenerated point
            }
            if (rev1)
            {
                d1 = d1.Reversed();
            }

            c2d2.D0(u, out p);
            hs2.D1(p.X, p.Y, out pp2, out du2, out dv2);
            d2 = (du2.Crossed(dv2));
            norm = d2.Magnitude;
            if (norm > 1.0e-12)
            {
                d2 /= norm;
            }
            else
            {
                continue; // skip degenerated point
            }
            if (rev2)
            {
                d2 = d2.Reversed();
            }

            // Compute angle.
            double currentAng = d1.AngleTo(d2);

            resAngle = Max(resAngle, currentAng);
        }
    }

    //=============================================================================
    // function : ComputeMaxAngleOnShape
    // purpose  : Code the regularities on all edges of the shape, boundary of
    //            two faces that do not have it.
    //=============================================================================
    private static void ComputeMaxAngleOnShape(in Shape s, ref double resAngle)
    {
        var m = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(s, ShapeType.Edge, ShapeType.Face, m);
        var it = new NCollection_List<Shape>.Iterator();
        var ex = new Explorer();
        Face f1 = new(), f2 = new();
        bool found, couture;
        for (int i = 0; i < m.Count; i++)
        {
            Edge e = ShapeCasts.Edge(m.KeyAt(i));
            found = false;
            couture = false;
            f1.Nullify();
            for (it.Initialize(m.ValueAt(i)); it.More() && !found; it.Next())
            {
                if (f1.IsNull)
                {
                    f1 = ShapeCasts.Face(it.Value());
                }
                else
                {
                    if (!f1.IsSame(ShapeCasts.Face(it.Value())))
                    {
                        found = true;
                        f2 = ShapeCasts.Face(it.Value());
                    }
                }
            }
            if (!found && !f1.IsNull)
            { // is it a sewing edge?
                Orientation orE = e.Orientation;
                Edge curE;
                for (ex.Init(f1, ShapeType.Edge); ex.More() && !found; ex.Next())
                {
                    curE = ShapeCasts.Edge(ex.Current());
                    if (e.IsSame(curE) && orE != curE.Orientation)
                    {
                        found = true;
                        couture = true;
                        f2 = f1;
                    }
                }
            }
            if (found)
            {
                if (Tool.Continuity(e, f1, f2) <= C0)
                {
                    try
                    {
                        Tgtfaces(e, f1, f2, couture, ref resAngle);
                    }
                    catch (FailureException)
                    {
                    }
                }
            }
        }
    }

    //=============================================================================
    // function : ComputeMaxAngle
    // purpose  : Computes max angle in faces junction
    //=============================================================================
    private void ComputeMaxAngle()
    {
        ComputeMaxAngleOnShape(_inputShape, ref _maxAngle);
    }

    //=============================================================================
    // function : BuildMissingWalls
    // purpose  : Builds walls to the result solid.
    //=============================================================================
    private bool BuildMissingWalls()
    {
        // Internal list of new faces.
        Compound newFaces = new();
        var bb = new BrepBuilder();
        bb.MakeCompound(ref newFaces);

        // Compute outer bounds of original shape.
        var fb = new FreeBounds(_shape);
        Compound freeWires = fb.InternalGetClosedWires();

        // Build linear faces on each edge and its image.
        var expCW = new Explorer(freeWires, ShapeType.Wire);
        for (; expCW.More(); expCW.Next())
        {
            Wire curWire = ShapeCasts.Wire(expCW.Current());

            // Iterate over outer edges in outer wires.
            var expWE = new Explorer(curWire, ShapeType.Edge);
            for (; expWE.More(); expWE.Next())
            {
                Edge curEdge = ShapeCasts.Edge(expWE.Current());

                Face newFace = BuildWallFace(curEdge);

                if (newFace.IsNull)
                {
                    _error = SimpleOffsetStatus.ErrorWallFaceComputation;
                    return false;
                }

                bb.Add(newFaces, newFace);
            }
        }

        // Update edges from wall faces.
        var sfe = new EdgeFixer();
        sfe.SetContext(_reShape);
        var expCE = new Explorer(newFaces, ShapeType.Edge);
        for (; expCE.More(); expCE.Next())
        {
            // Fix same parameter and same range flags.
            Edge currEdge = ShapeCasts.Edge(expCE.Current());
            sfe.FixSameParameter(currEdge);
        }

        // Update result to be compound.
        Compound resCompound = new();
        bb.MakeCompound(ref resCompound);

        // Add old faces the result.
        var expSF = new Explorer(_shape, ShapeType.Face);
        for (; expSF.More(); expSF.Next())
        {
            bb.Add(resCompound, expSF.Current());
        }

        // Add new faces the result.
        expSF.Init(_resShape, ShapeType.Face);
        for (; expSF.More(); expSF.Next())
        {
            bb.Add(resCompound, expSF.Current());
        }

        // Add wall faces to the result.
        var expCF = new Explorer(newFaces, ShapeType.Face);
        for (; expCF.More(); expCF.Next())
        {
            Face f = ShapeCasts.Face(expCF.Current());
            bb.Add(resCompound, f);
        }

        // Apply stored modifications.
        resCompound = ShapeCasts.Compound(_reShape.InternalApply(resCompound));

        // Create result shell.
        var quilt = new Quilt();
        quilt.Add(resCompound);
        Shape shells = quilt.InternalShells();

        var expSSh = new Explorer(shells, ShapeType.Shell);
        Shell resShell = new();
        for (; expSSh.More(); expSSh.Next())
        {
            if (!resShell.IsNull)
            {
                // Shell is not null -> explorer contains two or more shells.
                _error = SimpleOffsetStatus.ErrorInvalidNbShells;
                return false;
            }
            resShell = ShapeCasts.Shell(expSSh.Current());
        }

        // OCCT: with no shell at all (Quilt made none), aResShell is null and IsClosed dereferences it
        if (!Tool.IsClosed(resShell))
        {
            _error = SimpleOffsetStatus.ErrorNonClosedShell;
            return false;
        }

        // Create result solid.
        Solid resSolid = new();
        bb.MakeSolid(ref resSolid);
        bb.Add(resSolid, resShell);
        _resShape = resSolid;

        return true;
    }

    //=================================================================================================

    // Builds face on specified wall.
    private Face BuildWallFace(in Edge origEdge)
    {
        Face resFace = new();

        // Get offset edge. offset edge is reversed to create correct wire.
        Edge newEdge = ShapeCasts.Edge(_builder.InternalModifiedShape(origEdge));
        newEdge.Orient(Reversed);

        Exploration.Vertices(newEdge, out Vertex newV1, out Vertex newV2);

        // Wire contour is:
        // theOrigEdge (forcible forward) -> wall1 -> aNewEdge (forcible reversed) -> wall2
        // Firstly it is necessary to create copy of original shape with forward direction.
        // This simplifies walls creation.
        Edge origCopy = origEdge;
        origCopy.Orient(Forward);
        Exploration.Vertices(origCopy, out Vertex v1, out Vertex v2);

        // To simplify work with map.
        Vertex forwardV1 = ShapeCasts.Vertex(v1.Oriented(Forward));
        Vertex forwardV2 = ShapeCasts.Vertex(v2.Oriented(Forward));

        // Check existence of edges in stored map: Edge1
        Edge wall1;
        if (_mapVE.ContainsKey(forwardV2))
        {
            // Edge exists - get it from map.
            wall1 = _mapVE.GetValue(forwardV2);
        }
        else
        {
            // Edge does not exist - create it and add to the map.
            var me1 = new LibraryMakeEdge(ShapeCasts.Vertex(v2.Oriented(Forward)),
                                            ShapeCasts.Vertex(newV2.Oriented(Reversed)));
            if (!me1.IsDone)
            {
                return resFace;
            }
            wall1 = me1.InternalEdge();

            _mapVE[forwardV2] = wall1;
        }

        // Check existence of edges in stored map: Edge2
        Edge wall2;
        if (_mapVE.ContainsKey(forwardV1))
        {
            // Edge exists - get it from map.
            wall2 = ShapeCasts.Edge(_mapVE.GetValue(forwardV1).Oriented(Reversed));
        }
        else
        {
            // Edge does not exist - create it and add to the map.
            var me2 = new LibraryMakeEdge(ShapeCasts.Vertex(v1.Oriented(Forward)),
                                            ShapeCasts.Vertex(newV1.Oriented(Reversed)));
            if (!me2.IsDone)
            {
                return resFace;
            }
            wall2 = me2.InternalEdge();

            _mapVE[forwardV1] = wall2;

            // Orient it in reversed direction.
            wall2.Orient(Reversed);
        }

        var bb = new BrepBuilder();

        Wire wire = new();
        bb.MakeWire(ref wire);
        bb.Add(wire, origCopy);
        bb.Add(wire, wall1);
        bb.Add(wire, newEdge);
        bb.Add(wire, wall2);

        // Build 3d curves on wire
        BrepLibrary.BuildCurves3d(wire);

        // Try to build using simple planar approach.
        Face f = new();
        try
        {
            // Call of face maker is wrapped by try/catch since it generates exceptions sometimes.
            var fm = new LibraryMakeFace(wire, true);
            if (fm.IsDone)
            {
                f = fm.InternalFace();
            }
        }
        catch (FailureException)
        {
        }

        if (f.IsNull) // Exception in face maker or result is not computed.
        {
            // Build using thrusections.
            bool toReverse = false;
            double fpar, lpar, fparOE, lparOE;
            Curve3D edgeCurve = Tool.Curve(origEdge, out fpar, out lpar)!;
            var trEdgeCurve = new TrimmedCurve3D(edgeCurve, fpar, lpar);
            Curve3D offsetCurve = Tool.Curve(newEdge, out fparOE, out lparOE)!;
            var trOffsetCurve = new TrimmedCurve3D(offsetCurve, fparOE, lparOE);

            var thrusecGenerator = new SurfaceGenerator();
            thrusecGenerator.AddCurve(trEdgeCurve);
            thrusecGenerator.AddCurve(trOffsetCurve);
            thrusecGenerator.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
            Surface theSurf = thrusecGenerator.Surface()!;
            // theSurf = new Geom_SurfaceOfLinearExtrusion( TrOffsetCurve, OffsetDir );
            theSurf.ParameterBounds(out double uf, out double ul, out double vf, out double vl);
            Placement loc = new();
            LineCurve2D edgeLine2d, oeLine2d, line2d, line2d2;
            edgeLine2d = new LineCurve2D(new Point2D(0.0, vf), new Direction2D(Direction2D.D.X));
            bb.UpdateEdge(origEdge, edgeLine2d, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
            oeLine2d = new LineCurve2D(new Point2D(0.0, vl), new Direction2D(Direction2D.D.X));
            bb.UpdateEdge(newEdge, oeLine2d, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
            double uonV1 = (toReverse) ? ul : uf;
            double uonV2 = (toReverse) ? uf : ul;
            line2d = new LineCurve2D(new Point2D(uonV2, 0.0), new Direction2D(Direction2D.D.Y));
            line2d2 = new LineCurve2D(new Point2D(uonV1, 0.0), new Direction2D(Direction2D.D.Y));
            if (wall1.IsSame(wall2))
            {
                bb.UpdateEdge(wall1, line2d, line2d2, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                Curve3D bSplC34 = theSurf.UIso(uf);
                bb.UpdateEdge(wall1, bSplC34, global::Kerf.Engine.Foundation.Precision.Confusion());
                bb.Range(wall1, vf, vl);
            }
            else
            {
                bb.SameParameter(wall1, false);
                bb.SameRange(wall1, false);
                bb.SameParameter(wall2, false);
                bb.SameRange(wall2, false);
                bb.UpdateEdge(wall1, line2d, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                bb.Range(wall1, theSurf, loc, vf, vl);
                bb.UpdateEdge(wall2, line2d2, theSurf, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                bb.Range(wall2, theSurf, loc, vf, vl);
                Curve3D bSplC3 = theSurf.UIso(uonV2);
                bb.UpdateEdge(wall1, bSplC3, global::Kerf.Engine.Foundation.Precision.Confusion());
                bb.Range(wall1, vf, vl, true); // only for 3d curve
                Curve3D bSplC4 = theSurf.UIso(uonV1);
                bb.UpdateEdge(wall2, bSplC4, global::Kerf.Engine.Foundation.Precision.Confusion());
                bb.Range(wall2, vf, vl, true); // only for 3d curve
            }

            f = new LibraryMakeFace(theSurf, wire).InternalFace();
        }

        return f;
    }

    //=================================================================================================

    /// <summary>The shape generated from a sub-shape of the input (frozen); null if none.</summary>
    /// <exception cref="NoSuchObjectException">The shape isn't a sub-shape of the input.</exception>
    public Shape Generated(in Shape shape)
    {
        Shape res = InternalGenerated(shape);
        res.TShape()?.FreezeGraph();
        return res;
    }

    /// <summary><see cref="Generated"/> without freezing, for ported code.</summary>
    internal Shape InternalGenerated(in Shape shape)
    {
        // Shape generated by modification.
        Shape res;
        // Kerf: the modifier ran on the caller's shape (its unfrozen TShapes copied) in the solid's overlay, if any; what
        // stands for its result after the overlay's end
        res = _builder.InternalModifiedShape(WorkingOverlay.CopiedIn(shape, _copies));
        if (_overlay is { IsFinished: true })
        {
            res = _overlay.Final(res);
        }

        if (res.IsNull)
        {
            return res;
        }

        // Shape modifications obtained in scope of shape healing.
        res = _reShape.InternalApply(res);

        return res;
    }

    //=================================================================================================

    /// <summary>The shape a sub-shape of the input was modified into by the healing of the walls (frozen); null if
    /// none.</summary>
    public Shape Modified(in Shape shape)
    {
        Shape res = InternalModified(shape);
        res.TShape()?.FreezeGraph();
        return res;
    }

    /// <summary><see cref="Modified"/> without freezing, for ported code.</summary>
    internal Shape InternalModified(in Shape shape)
    {
        Shape emptyShape = new();

        // Get modification status and new shape.
        int modStatus = _reShape.InternalStatus(shape, out Shape res);

        if (modStatus == 0)
        {
            return emptyShape; // No modifications are applied to the shape or its sub-shapes.
        }

        return res;
    }
}
