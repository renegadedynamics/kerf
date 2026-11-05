// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_BuilderAlgo.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_BuilderAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans.Api;

/// <summary>
/// The API of the General Fuse algorithm: splits the arguments by each other and returns all the splits.
/// </summary>
/// <remarks>
/// <para>
/// Options besides the base class's: the non-destructive mode (the arguments aren't modified, copies are made where
/// needed; always so for frozen arguments, decision 577), the glue options, the check of input solids for being
/// inverted, and the collection of the history. Errors: <see cref="AlertTooFewArguments"/>,
/// <see cref="AlertIntersectionFailed"/>, <see cref="AlertBuilderFailed"/>; the intersection's and
/// the builder's warnings are collected in the report.
/// </para>
/// <para>
/// A working object (decision 579): <see cref="MakeShape.Shape"/>, <c>Modified</c>,
/// <c>Generated</c>, <see cref="SectionEdges"/> and <see cref="History"/> publish, freezing the whole result first;
/// ported code that goes on building reads <see cref="Algo.InternalShape"/>,
/// <see cref="InternalHistory"/> and <see cref="InternalSectionEdges"/>. The functional twins are in
/// <see cref="BRepAlgoApi"/>.
/// </para>
/// </remarks>
internal class BuilderAlgo : Algo
{
    // Inputs
    protected NCollection_List<Shape> _arguments = new(); //!< Arguments of the operation

    // Options
    protected bool _nonDestructive;  //!< Non-destructive mode management
    protected global::Kerf.Booleans.GlueMode _glue; //!< Gluing mode management
    protected bool _checkInverted;   //!< Check for inverted solids management
    protected bool _fillHistory;     //!< Controls the history collection

    // Tools
    protected bool _isIntersectionNeeded;         //!< Whether the intersection of the arguments is performed
    private protected PaveFiller? _dsFiller; //!< Intersection tool
    private protected BooleanBuilder? _builder;     //!< Building tool
    protected History? _history;           //!< The history of the whole operation
    protected History? _simplifierHistory; //!< The history of the result's simplification (unit 6P)

    /// <summary>An empty algorithm.</summary>
    public BuilderAlgo()
    {
        _nonDestructive = false;
        _glue = global::Kerf.Booleans.GlueMode.Off;
        _checkInverted = true;
        _fillHistory = true;
        _isIntersectionNeeded = true;
        _dsFiller = null;
        _builder = null;
    }

    /// <summary>An algorithm using the intersections already computed by <paramref name="aPF"/>.</summary>
    public BuilderAlgo(PaveFiller aPF)
    {
        _nonDestructive = false;
        _glue = global::Kerf.Booleans.GlueMode.Off;
        _checkInverted = true;
        _fillHistory = true;
        _isIntersectionNeeded = false;
        _builder = null;
        _dsFiller = aPF;
    }

    /// <summary>Sets the arguments (copies the list).</summary>
    public void SetArguments(NCollection_List<Shape> ls) => _arguments = new NCollection_List<Shape>(ls);

    /// <summary>The arguments.</summary>
    public NCollection_List<Shape> Arguments() => _arguments;

    /// <summary>
    /// Sets the non-destructive mode: the arguments are not modified, a copy of a sub-shape is made in the result
    /// where it must change. Frozen arguments are always treated so (decision 577).
    /// </summary>
    public void SetNonDestructive(bool flag) => _nonDestructive = flag;

    public bool NonDestructive() => _nonDestructive;

    /// <summary>Sets the glue option, which speeds up the intersection of arguments that only touch.</summary>
    public void SetGlue(global::Kerf.Booleans.GlueMode glue) => _glue = glue;

    public global::Kerf.Booleans.GlueMode Glue() => _glue;

    /// <summary>Enables or disables the check of the input solids for being inverted.</summary>
    public void SetCheckInverted(bool check) => _checkInverted = check;

    public bool CheckInverted() => _checkInverted;

    /// <summary>Performs the algorithm.</summary>
    public override void Build(ProgressRange range = default)
    {
        // Setting not done status
        NotDone();
        // Destroy the tools if necessary
        Clear();
        using var ps = new ProgressScope(range, "Performing General Fuse operation", 100);
        // If necessary perform intersection of the argument shapes
        IntersectShapes(_arguments, ps.Next(70));
        if (HasErrors)
        {
            return;
        }
        // Initialization of the Building tool
        _builder = new BooleanBuilder();
        // Set arguments to builder
        _builder.SetArguments(_arguments);
        // Build the result basing on intersection results
        BuildResult(ps.Next(30));
    }

    /// <summary>
    /// Simplifies the result by unifying the tangential edges and faces (ShapeUpgrade_UnifySameDomain). The fuzzy value
    /// of the operation is the unifier's linear tolerance; the non-destructive mode is its safe input mode. The history
    /// of the simplification is merged into the history of the operation, so <c>Modified</c>, <c>Generated</c> and
    /// <see cref="IsDeleted"/> take it into account.
    /// </summary>
    /// <remarks>
    /// Runs on the unfrozen result (decision 579). The result shares the arguments' untouched sub-shapes, which are
    /// frozen: the unifier copies its input at the entry (decision 766), so the arguments are never written, whatever the
    /// non-destructive flag, and the sub-shapes it leaves unchanged are the result's (and the arguments') own.
    /// </remarks>
    /// <param name="unifyEdges">Controls the edges unification (true by default).</param>
    /// <param name="unifyFaces">Controls the faces unification (true by default).</param>
    public void SimplifyResult(bool unifyEdges = true, bool unifyFaces = true) =>
        SimplifyResult(unifyEdges, unifyFaces, global::Kerf.Engine.Foundation.Precision.Angular());

    /// <inheritdoc cref="SimplifyResult(bool, bool)"/>
    /// <param name="unifyEdges">Controls the edges unification.</param>
    /// <param name="unifyFaces">Controls the faces unification.</param>
    /// <param name="angularTol">Angular criteria for tangency of edges and faces (Precision::Angular() by default).</param>
    public void SimplifyResult(bool unifyEdges, bool unifyFaces, double angularTol)
    {
        if (HasErrors)
        {
            return;
        }

        if (!unifyEdges && !unifyFaces)
        {
            return;
        }

        // Simplification tool
        UnifySameDomain unifier = new(_shape, unifyEdges, unifyFaces, true);
        // Pass options
        unifier.SetLinearTolerance(_options._fuzzyValue);
        unifier.SetAngularTolerance(angularTol);
        unifier.SetSafeInputMode(_nonDestructive);
        unifier.AllowInternalEdges(false);
        // Perform simplification
        unifier.Build();
        // Overwrite result with simplified shape
        _shape = unifier.InternalShape();
        // Keep simplification history
        _simplifierHistory = unifier.InternalHistory();
        if (_fillHistory)
        {
            // Merge simplification history into result history
            _history!.Merge(_simplifierHistory!);
        }
    }

    /// <summary>The algorithm's own list behind <c>Modified</c>: the splits of <paramref name="s"/> in the result.</summary>
    internal override NCollection_List<Shape> InternalModified(in Shape s)
    {
        _generated.Clear();
        if (_fillHistory && _history is not null)
        {
            foreach (Shape aS in _history.Modified(s))
            {
                _generated.Append(aS);
            }
        }
        return _generated;
    }

    /// <summary>The algorithm's own list behind <c>Generated</c>: the shapes generated from <paramref name="s"/>.</summary>
    internal override NCollection_List<Shape> InternalGenerated(in Shape s)
    {
        _generated.Clear();
        if (_fillHistory && _history is not null)
        {
            foreach (Shape aS in _history.Generated(s))
            {
                _generated.Append(aS);
            }
        }
        return _generated;
    }

    /// <summary>True if <paramref name="aS"/> has been removed: neither it nor a split of it is in the result.</summary>
    public override bool IsDeleted(in Shape aS) => _fillHistory && _history is not null && _history.IsRemoved(aS);

    /// <summary>True if any argument has been modified.</summary>
    public virtual bool HasModified => _fillHistory && _history is not null && _history.HasModified;

    /// <summary>True if any argument has generated shapes.</summary>
    public virtual bool HasGenerated => _fillHistory && _history is not null && _history.HasGenerated;

    /// <summary>True if any argument has been deleted.</summary>
    public virtual bool HasDeleted => _fillHistory && _history is not null && _history.HasRemoved;

    /// <summary>Enables or disables the collection of the history.</summary>
    public void SetToFillHistory(bool histFlag) => _fillHistory = histFlag;

    /// <summary>True if the history is collected.</summary>
    public bool HasHistory => _fillHistory;

    /// <summary>The section edges: the edges where the arguments intersect (frozen, with the whole result).</summary>
    public IReadOnlyList<Shape> SectionEdges()
    {
        BeforePublishingLists();
        return PublishedList(InternalSectionEdges());
    }

    /// <summary>The algorithm's own list behind <see cref="SectionEdges"/>, unfrozen.</summary>
    internal NCollection_List<Shape> InternalSectionEdges()
    {
        _generated.Clear();
        if (_builder is null)
        {
            return _generated;
        }

        // Fence map to avoid duplicated section edges in the result list
        HashSet<Shape> mFence = new(ShapeHasher.Instance);
        // Intersection results
        DS pDS = _dsFiller!.PDS()!;
        // Iterate on all Face/Face interferences and take section edges
        List<InterfFF> fFs = pDS.InterfFF();
        int nbFF = fFs.Count;
        for (int i = 0; i < nbFF; ++i)
        {
            InterfFF fFi = fFs[i];
            // Section curves between pair of faces
            List<SectionCurve> sectionCurves = fFi.Curves();
            int nbC = sectionCurves.Count;
            for (int j = 0; j < nbC; ++j)
            {
                SectionCurve curve = sectionCurves[j];
                // Section edges created from the curve
                for (NCollection_List<PaveBlock>.Iterator itPB = new(curve.PaveBlocks()); itPB.More(); itPB.Next())
                {
                    PaveBlock pb = itPB.Value();
                    Shape se = pDS.Shape(pb.Edge());
                    if (!mFence.Add(se))
                    {
                        continue;
                    }
                    // Take into account simplification of the result shape
                    if (_simplifierHistory is not null)
                    {
                        if (_simplifierHistory.IsRemoved(se))
                        {
                            continue;
                        }

                        IReadOnlyList<Shape> lseIm = _simplifierHistory.Modified(se);
                        if (lseIm.Count != 0)
                        {
                            foreach (Shape seIm in lseIm)
                            {
                                if (mFence.Add(seIm))
                                {
                                    _generated.Append(seIm);
                                }
                            }
                        }
                        else
                        {
                            _generated.Append(se);
                        }
                    }
                    else
                    {
                        _generated.Append(se);
                    }
                }
            }
        }
        return _generated;
    }

    /// <summary>The intersection tool (a working object of the Boolean component).</summary>
    internal PaveFiller? DSFiller() => _dsFiller;

    /// <summary>The building tool.</summary>
    internal BooleanBuilder? Builder() => _builder;

    /// <summary>
    /// The history of the operation, null if it isn't collected. The result is frozen first, so every shape the
    /// history lists is frozen.
    /// </summary>
    public History? History()
    {
        BeforePublishingLists();
        return InternalHistory();
    }

    /// <summary>OCCT's <c>History()</c>, without freezing the result.</summary>
    internal History? InternalHistory() => _fillHistory ? _history : null;

    /// <summary>Sets the options of the derived classes on the intersection tool; nothing here.</summary>
    protected virtual void SetAttributes()
    {
    }

    /// <summary>Intersects the given shapes with a new intersection tool, unless one was given.</summary>
    protected void IntersectShapes(NCollection_List<Shape> theArgs, ProgressRange range)
    {
        if (!_isIntersectionNeeded)
        {
            return;
        }

        // Create new Filler
        _dsFiller = new PaveFiller();
        // Set arguments for intersection
        _dsFiller.SetArguments(theArgs);
        // Set options for intersection
        _dsFiller.SetRunParallel(_options._runParallel);
        _dsFiller.SetFuzzyValue(_options._fuzzyValue);
        _dsFiller.SetNonDestructive(_nonDestructive);
        _dsFiller.SetGlue(_glue);
        _dsFiller.SetUseOBB(_options._useOBB);
        // Set Face/Face intersection options to the intersection algorithm
        SetAttributes();
        // Perform intersection
        _dsFiller.Perform(range);
        // Check for the errors during intersection
        GetReport().Merge(_dsFiller.GetReport());
    }

    /// <summary>Builds the result with the building tool from the intersections.</summary>
    protected void BuildResult(ProgressRange range = default)
    {
        // Set options to the builder
        _builder!.SetRunParallel(_options._runParallel);
        _builder.SetCheckInverted(_checkInverted);
        _builder.SetToFillHistory(_fillHistory);
        // Perform building of the result with pre-calculated intersections
        _builder.PerformWithFiller(_dsFiller!, range);
        // Merge the warnings of the Building part
        GetReport().Merge(_builder.GetReport());
        // Check for the errors
        if (_builder.HasErrors)
        {
            return;
        }
        // Set done status
        Done();
        // Get the result shape
        _shape = _builder.InternalShape();
        // Fill history
        if (_fillHistory)
        {
            _history = new History();
            _history.Merge(_builder.InternalHistory());
        }
    }

    /// <summary>Clears the algorithm from previous runs.</summary>
    public override void Clear()
    {
        base.Clear();
        if (_isIntersectionNeeded)
        {
            _dsFiller = null;
        }
        _builder = null;
        _history = null;
        _simplifierHistory = null;
    }

    // The lists published by Modified, Generated, SectionEdges and History hold sub-shapes of the result: the whole
    // result is frozen first (decision 414).
    private protected override void BeforePublishingLists() => _shape.TShape()?.FreezeGraph();
}
