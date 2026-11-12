// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Tools;
using Kerf.Topology;

namespace Kerf.Engine.Booleans;

/// <summary>
/// The intersection of the arguments of a Boolean operation (the "pave filler"): it fills the data structure
/// <see cref="DS"/> with the interferences of the arguments' sub-shapes, which the builders then use.
/// </summary>
/// <remarks>
/// <para>
/// The steps, in OCCT's order: the vertex/vertex, vertex/edge, edge/edge, vertex/face and edge/face interferences
/// (with the pave blocks of the edges updated after each), the repeated intersection of the vertices whose tolerance
/// grew, the forced edge/edge and edge/face interferences, the face/face intersections and their section curves,
/// the split edges, the blocks, the self-interference check, the p-curves and the degenerated edges.
/// </para>
/// <para>
/// A working object (plan section 6). Its intersection loops run through <see cref="BooleanParallel"/> with a
/// context per worker; the result is the serial one at every degree of parallelism (decision 580).
/// </para>
/// <para>
/// <b>Non-destructive mode.</b> The filler turns non-destructive by itself when an argument is
/// <see cref="Shape.Locked()"/> (<c>BOPAlgo_PaveFiller_10</c>), and frozen shapes are locked, so a published
/// shape is never written (decision 577): an argument sub-shape that must change is replaced by a new one. Inside an
/// operation's working overlay a frozen shape takes writes (into its working copy), so it isn't locked there: the
/// filler is destructive on it, as OCCT's on an unlocked shape, and the operation's result holds what stands for each
/// sub-shape it changed.
/// </para>
/// <para>
/// The allocator constructor goes; the protected typedefs are written out (<c>BOPAlgo_DataMapOfPaveBlockBndBox</c>
/// is <c>NCollection_DataMap&lt;BOPDS_PaveBlock, Bnd_Box&gt;</c>, <c>BOPAlgo_DataMapOfPaveBlockListOfInteger</c>
/// <c>NCollection_DataMap&lt;BOPDS_PaveBlock, NCollection_List&lt;int&gt;&gt;</c> and
/// <c>BOPAlgo_DataMapOfIntegerMapOfPaveBlock</c> <c>NCollection_DataMap&lt;int,
/// NCollection_Map&lt;BOPDS_PaveBlock&gt;&gt;</c>).
/// </para>
/// </remarks>
internal partial class PaveFiller : BooleanAlgorithm
{
    // List of operations to be supported by the Progress Indicator
    private enum BOPAlgo_PIOperation
    {
        PIOperation_Prepare = 0,
        PIOperation_PerformVV,
        PIOperation_PerformVE,
        PIOperation_PerformEE,
        PIOperation_PerformVF,
        PIOperation_PerformEF,
        PIOperation_RepeatIntersection,
        PIOperation_ForceInterfEE,
        PIOperation_ForceInterfEF,
        PIOperation_PerformFF,
        PIOperation_MakeSplitEdges,
        PIOperation_MakeBlocks,
        PIOperation_MakePCurves,
        PIOperation_ProcessDE,
        PIOperation_Last
    }

    /// <summary>A range of an edge and its minimal distance to another shape.</summary>
    protected struct EdgeRangeDistance
    {
        public double First;
        public double Last;
        public double Distance;

        public EdgeRangeDistance()
            : this(0.0, 0.0, RealLast())
        {
        }

        public EdgeRangeDistance(double first, double last = 0.0, double distance = double.MaxValue)
        {
            First = first;
            Last = last;
            Distance = distance;
        }
    }

    protected NCollection_List<Shape> _arguments = new();
    protected DS? _ds;
    protected DataIterator? _iterator;
    protected IntersectionContext? _context;
    protected SectionAttribute _sectionAttribute = new();
    protected bool _nonDestructive;
    protected bool _isPrimary;
    protected bool _avoidBuildPCurve;
    protected global::Kerf.Booleans.GlueMode _glue;

    //! Fence map of intersected faces and pave blocks
    protected NCollection_IntDataMap<HashSet<PaveBlock>> _fpbDone = new();
    //! Sub-shapes with increased tolerance during the operation
    protected NCollection_IntMap _increasedSS = new();
    //! Vertices located close to E/E or E/F intersection points
    protected NCollection_IntMap _vertsToAvoidExtension = new();
    //! Kerf: vertices grown over an edge's crossing of a face, each with the widest stretch such an edge touches the face over
    protected Dictionary<int, double> _touchSpans = new();
    //! Kerf: how many tolerances from a vertex a crossing may be and still be covered by growing it; 0, OCCT's, any
    //! distance the steps allow (decision 2276)
    protected double _crossingsApart;
    //! Kerf: with crossings apart, whether a touch that hides two crossings is given as the two (decision 2286)
    protected bool _touchesApart = true;
    //! Kerf: with crossings apart, whether a piece of an edge too small to keep is taken into its neighbour (decision 33708)
    protected bool _smallPiecesTaken;
    //! Kerf: with crossings apart, each point an edge touches a face at, with the edge, the face and how long the stretch
    //! they're within a tolerance over is (decision 2287)
    protected List<(int Edge, int Face, global::Kerf.Geometry.Point3D At, double Span)> _touchCrossings = new();
    //! Map to store minimal distances between shapes
    protected Dictionary<Pair, NCollection_List<EdgeRangeDistance>> _distances = new();

    public PaveFiller()
    {
        _ds = null;
        _iterator = null;
        _nonDestructive = false;
        _isPrimary = true;
        _avoidBuildPCurve = false;
        _glue = global::Kerf.Booleans.GlueMode.Off;
    }

    /// <summary>The data structure (filled by <see cref="Perform"/>).</summary>
    public DS DS() => _ds!;

    /// <summary>The data structure (OCCT's pointer form; null before <see cref="Perform"/>).</summary>
    public DS? PDS() => _ds;

    /// <summary>The iterator of interfering pairs.</summary>
    public DataIterator? Iterator() => _iterator;

    /// <summary>Sets the arguments (copies the list).</summary>
    public void SetArguments(NCollection_List<Shape> ls) => _arguments = new NCollection_List<Shape>(ls);

    /// <summary>Adds an argument.</summary>
    public void AddArgument(in Shape shape) => _arguments.Append(shape);

    /// <summary>The arguments.</summary>
    public NCollection_List<Shape> Arguments() => _arguments;

    /// <summary>The intersection context (null before <see cref="Perform"/>).</summary>
    public IntersectionContext? Context() => _context;

    /// <summary>Sets the options of the section curves (approximation, p-curves).</summary>
    public void SetSectionAttribute(in SectionAttribute secAttr) => _sectionAttribute = secAttr;

    /// <summary>
    /// The non-destructive mode: the arguments are not modified (new shapes are made where needed). Locked (frozen)
    /// arguments turn it on whatever is set here.
    /// </summary>
    public void SetNonDestructive(bool flag) => _nonDestructive = flag;

    /// <summary>Whether the operation is non-destructive.</summary>
    public bool NonDestructive() => _nonDestructive;

    /// <summary>
    /// Kerf (decision 2276): a crossing of two edges, of an edge and a face, or of a section's curve and a vertex's edge
    /// further from a vertex than <paramref name="tolerances"/> times the tolerances in play isn't covered by growing that
    /// vertex, as OCCT's steps do, but gets a vertex of its own, and the pieces between such vertices are tested for
    /// coincidence again once the sections are made. 0, the default, is OCCT's behaviour. With it on (decisions 2285 to
    /// 32903): a vertex two tangent faces share isn't grown over the end of their curve where that's far from it; an edge's
    /// touch of a face that hides two crossings far apart is given as the two, unless <paramref name="touches"/> is false;
    /// a section's curve that stops short within such a touch is carried on to the crossing; and each section edge's curves
    /// end at its vertices, in space and on each face. With <paramref name="smallPieces"/> (decision 33708), a piece of an
    /// edge between two vertices that is no longer than their tolerances, where one of them covers all of it, is taken
    /// into the piece next to it at that vertex.
    /// </summary>
    public void SetCrossingsApart(double tolerances, bool touches = true, bool smallPieces = false)
    {
        _crossingsApart = tolerances;
        _touchesApart = touches;
        _smallPiecesTaken = smallPieces;
    }

    /// <summary>How many tolerances from a vertex a crossing may be and still be covered by growing it (0: any).</summary>
    public double CrossingsApart() => _crossingsApart;

    /// <summary>Sets the glue option.</summary>
    public void SetGlue(global::Kerf.Booleans.GlueMode glue) => _glue = glue;

    /// <summary>The glue option.</summary>
    public global::Kerf.Booleans.GlueMode Glue() => _glue;

    /// <summary>Disables the building of the p-curves of the section edges on planar faces.</summary>
    public void SetAvoidBuildPCurve(bool value) => _avoidBuildPCurve = value;

    /// <summary>Whether the building of the p-curves on planar faces is avoided.</summary>
    public bool IsAvoidBuildPCurve => _avoidBuildPCurve;

    protected void SetIsPrimary(bool flag) => _isPrimary = flag;

    protected bool IsPrimary => _isPrimary;

    public override void Clear()
    {
        base.Clear();
        _iterator = null;
        _ds = null;
        _increasedSS.Clear();
        _touchSpans.Clear();
    }

    /// <summary>OCCT's initialisation: the checks, the data structure, the context, the iterator and the mode.</summary>
    protected virtual void Init(ProgressRange range)
    {
        if (_arguments.Extent() == 0)
        {
            AddError(new AlertTooFewArguments());
            return;
        }
        //
        using var ps = new ProgressScope(range, "Initialization of Intersection algorithm", 1);
        for (NCollection_List<Shape>.Iterator it = new(_arguments); it.More(); it.Next())
        {
            if (it.Value().IsNull)
            {
                AddError(new AlertNullInputShapes());
                return;
            }
        }
        //
        // 0 Clear
        Clear();
        //
        // 1.myDS
        _ds = new DS();
        _ds.SetArguments(_arguments);
        _ds.Init(_fuzzyValue);
        //
        // 2 myContext
        _context = new IntersectionContext();
        //
        // 3.myIterator
        _iterator = new DataIterator();
        _iterator.SetRunParallel(_runParallel);
        _iterator.SetDS(_ds);
        _iterator.Prepare(_context, _useOBB, _fuzzyValue);
        //
        // 4 NonDestructive flag
        SetNonDestructive();
    }

    /// <summary>Performs the intersection; an exception becomes <see cref="AlertIntersectionFailed"/>.</summary>
    public override void Perform(ProgressRange range = default)
    {
        try
        {
            //
            PerformInternal(range);
        }
        //
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            AddError(new AlertIntersectionFailed());
        }
    }

    protected virtual void PerformInternal(ProgressRange range)
    {
        using var ps = new ProgressScope(range, "Performing intersection of shapes", 100);

        Init(ps.Next(5));
        if (HasErrors)
        {
            return;
        }

        // Compute steps of the PI
        PISteps steps = new((int)BOPAlgo_PIOperation.PIOperation_Last);
        AnalyzeProgress(95, steps);
        //
        Prepare(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_Prepare)));
        if (HasErrors)
        {
            return;
        }
        // 00
        PerformVV(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PerformVV)));
        if (HasErrors)
        {
            return;
        }
        // 01
        PerformVE(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PerformVE)));
        if (HasErrors)
        {
            return;
        }
        //
        UpdatePaveBlocksWithSDVertices();
        // 11
        PerformEE(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PerformEE)));
        if (HasErrors)
        {
            return;
        }
        UpdatePaveBlocksWithSDVertices();
        // 02
        PerformVF(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PerformVF)));
        if (HasErrors)
        {
            return;
        }
        UpdatePaveBlocksWithSDVertices();
        // 12
        PerformEF(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PerformEF)));
        if (HasErrors)
        {
            return;
        }
        UpdatePaveBlocksWithSDVertices();
        UpdateInterfsWithSDVertices();

        // Repeat Intersection with increased vertices
        RepeatIntersection(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_RepeatIntersection)));
        if (HasErrors)
        {
            return;
        }
        // Force intersection of edges after increase
        // of the tolerance values of their vertices
        ForceInterfEE(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_ForceInterfEE)));
        if (HasErrors)
        {
            return;
        }
        // Force Edge/Face intersection after increase
        // of the tolerance values of their vertices
        ForceInterfEF(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_ForceInterfEF)));
        if (HasErrors)
        {
            return;
        }
        //
        // 22
        PerformFF(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PerformFF)));
        if (HasErrors)
        {
            return;
        }
        //
        UpdateBlocksWithSharedVertices();
        //
        _ds!.RefineFaceInfoIn();
        //
        MakeSplitEdges(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_MakeSplitEdges)));
        if (HasErrors)
        {
            return;
        }
        //
        UpdatePaveBlocksWithSDVertices();
        //
        MakeBlocks(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_MakeBlocks)));
        if (HasErrors)
        {
            return;
        }
        if (_crossingsApart > 0)
        {
            // The sections' ends have split the edges since the coincidences were looked for
            ForceInterfEE(new ProgressRange(), afterBlocks: true);
        }
        //
        CheckSelfInterference();
        //
        UpdateInterfsWithSDVertices();
        _ds.ReleasePaveBlocks();
        _ds.RefineFaceInfoOn();
        //
        RemoveMicroEdges();
        //
        MakePCurves(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_MakePCurves)));
        if (HasErrors)
        {
            return;
        }
        //
        if (_crossingsApart > 0)
        {
            SectionsEndAtTheirVertices();
        }

        ProcessDE(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_ProcessDE)));
        if (HasErrors)
        {
            return;
        }
    }

    /// <summary>Repeats the intersection of the vertices whose tolerance has been increased.</summary>
    protected void RepeatIntersection(ProgressRange range)
    {
        // Find all vertices with increased tolerance
        NCollection_IntMap extraInterfMap = new();
        int nbS = _ds!.SourceShapeCount;
        using var ps = new ProgressScope(range, "Repeat intersection", 3);
        for (int i = 0; i < nbS; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() != ShapeType.Vertex)
            {
                continue;
            }
            // Check if the tolerance of the original vertex has been increased
            if (_increasedSS.Contains(i))
            {
                extraInterfMap.Add(i);
                continue;
            }

            // Check if the vertex created a new vertex with greater tolerance
            int nVSD = 0; // OCCT: uninitialised, read only when HasShapeSD writes it
            if (!_ds.HasShapeSD(i, ref nVSD))
            {
                continue;
            }

            if (_increasedSS.Contains(nVSD))
            {
                extraInterfMap.Add(i);
            }
        }

        if (extraInterfMap.IsEmpty())
        {
            return;
        }

        // Update iterator of pairs of shapes with interfering boxes
        _iterator!.IntersectExt(extraInterfMap);

        // Perform intersections with vertices

        PerformVV(ps.Next());
        if (HasErrors)
        {
            return;
        }
        UpdatePaveBlocksWithSDVertices();

        PerformVE(ps.Next());
        if (HasErrors)
        {
            return;
        }
        UpdatePaveBlocksWithSDVertices();

        PerformVF(ps.Next());
        if (HasErrors)
        {
            return;
        }

        UpdatePaveBlocksWithSDVertices();
        UpdateInterfsWithSDVertices();
    }

    protected override void FillPIConstants(double whole, PISteps steps)
    {
        if (!_nonDestructive)
        {
            steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_Prepare, 1 * whole / 100.0);
        }
    }

    protected override void FillPISteps(PISteps steps)
    {
        // Get number of all intersecting pairs
        int vvSize = 0, veSize = 0, eeSize = 0, vfSize = 0, efSize = 0, ffSize = 0;

        _iterator!.Initialize(ShapeType.Vertex, ShapeType.Vertex);
        vvSize = _iterator.ExpectedLength();

        _iterator.Initialize(ShapeType.Vertex, ShapeType.Edge);
        veSize = _iterator.ExpectedLength();

        _iterator.Initialize(ShapeType.Edge, ShapeType.Edge);
        eeSize = _iterator.ExpectedLength();

        _iterator.Initialize(ShapeType.Vertex, ShapeType.Face);
        vfSize = _iterator.ExpectedLength();

        if (_glue != global::Kerf.Booleans.GlueMode.Full)
        {
            _iterator.Initialize(ShapeType.Edge, ShapeType.Face);
            efSize = _iterator.ExpectedLength();
        }

        _iterator.Initialize(ShapeType.Face, ShapeType.Face);
        ffSize = _iterator.ExpectedLength();

        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PerformVV, vvSize);
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PerformVE, 2 * veSize);
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PerformEE, 5 * eeSize);
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PerformVF, 5 * vfSize);
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PerformEF, 10 * efSize);
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_RepeatIntersection, 0.2 * (vvSize + veSize + vfSize));
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_ForceInterfEE, 2 * eeSize);
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_ForceInterfEF, 2 * efSize);
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PerformFF, (_glue == global::Kerf.Booleans.GlueMode.Full ? 1 : 30) * ffSize);
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_MakeSplitEdges, eeSize);
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_MakeBlocks, (_glue == global::Kerf.Booleans.GlueMode.Full ? 0 : 5) * ffSize);
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_MakePCurves, _avoidBuildPCurve ? 0 : 0.2 * (eeSize + efSize));
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_ProcessDE, 0.1 * eeSize);
    }
}
