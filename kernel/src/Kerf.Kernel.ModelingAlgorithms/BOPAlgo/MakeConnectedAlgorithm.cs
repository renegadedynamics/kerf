// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakeConnected.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakeConnected.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2018-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Engine.Booleans.GlueMode;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>
/// Makes touching shapes of the same dimension connected (glues them so that they share their coinciding parts),
/// associates the materials (the input shapes) on both sides of each boundary element, and can make the connected
/// shape periodic and repeat it.
/// </summary>
/// <remarks>
/// A working object (decision 414's pattern). <see cref="Shape"/>, <see cref="PeriodicShape"/>, the materials, the
/// origins, <see cref="GetModified"/> and <see cref="History"/> publish: they freeze the result graphs first. Ported
/// code reads the <c>Internal*</c> forms. The gluing runs non-destructively, as in OCCT, so the arguments are never
/// written. The periodicity natively writes into the connected shape in place: the periodicity tool does it in working
/// overlays, so the connected shape then holds what stands for each sub-shape it changed (the arguments' frozen
/// sub-shapes are never written), and the materials of a changed shape are those of what stands for it. The
/// materials map keys elements with their orientation (OCCT's default hasher).
/// </remarks>
public class MakeConnectedAlgorithm : Options
{
    protected NCollection_List<Shape> _arguments = new();                          //!< Input shapes for making them connected
    protected IndexedSet<Shape> _allInputsMap = new(ShapeHasher.Instance); //!< Map of all BRep sub-elements of the input shapes
    protected MakePeriodicAlgorithm _periodicityMaker = new();                             //!< Tool for making the shape periodic
    protected Dictionary<Shape, NCollection_List<Shape>> _materials = new(EqualityComparer<Shape>.Default); //!< Map of the materials associations
    protected Dictionary<Shape, NCollection_List<Shape>> _origins = new(ShapeHasher.Instance); //!< Map of origins
    protected History? _glueHistory;                                            //!< Gluing History
    protected History? _history;                                                //!< Final History of shapes modifications
    protected Shape _glued = new();                                                //!< The resulting connected (glued) shape
    protected Shape _shape = new();                                                //!< The resulting shape
    private History _gluedImages = new();                                       //!< Kerf: the connected shape's sub-shapes changed in place

    public MakeConnectedAlgorithm()
    {
    }

    /// <summary>Sets the shapes to make connected (the list is copied).</summary>
    public void SetArguments(NCollection_List<Shape> theArgs) => _arguments = new NCollection_List<Shape>(theArgs);

    /// <summary>Adds a shape to make connected.</summary>
    public void AddArgument(in Shape s) => _arguments.Append(s);

    /// <summary>The shapes to make connected.</summary>
    public NCollection_List<Shape> Arguments() => _arguments;

    /// <summary>Performs the operation: glues the arguments and associates the materials.</summary>
    public void Perform()
    {
        // Check the input data
        CheckData();
        if (HasErrors)
        {
            return;
        }

        _history ??= new History();

        // Glue the arguments
        MakeConnected();
        if (HasErrors)
        {
            return;
        }

        // Perform material associations for the faces
        AssociateMaterials();
        if (HasErrors)
        {
            return;
        }
    }

    /// <summary>Makes the connected shape periodic with the parameters.</summary>
    public void MakePeriodic(in MakePeriodicAlgorithm.PeriodicityParams theParams)
    {
        if (HasErrors)
        {
            return;
        }

        // Kerf: the periodicity tool's history starts again from the connected shape as it stands now: the gluing
        // history is brought up to date with the sub-shapes the earlier runs changed in place
        if (_gluedImages.HasModified)
        {
            _glueHistory ??= new History();
            _glueHistory.Merge(_gluedImages);
            _gluedImages = new History();
        }

        // Make the shape periodic
        _periodicityMaker.Clear();
        _periodicityMaker.SetShape(_glued);
        _periodicityMaker.SetPeriodicityParameters(theParams);
        _periodicityMaker.SetRunParallel(_runParallel);
        _periodicityMaker.Perform();
        // Kerf: the trim writes into the connected shape in place natively; the periodicity tool does it in a working
        // overlay, and the connected shape is what stands for it now
        UpdateGlued();
        if (_periodicityMaker.HasErrors)
        {
            // Add warning informing the user that periodicity with
            // given parameters is not possible
            AddWarning(new AlertUnableToMakePeriodic(_shape));
            return;
        }

        _shape = _periodicityMaker.InternalShape();

        // Update history, materials, origins
        Update();
    }

    // Kerf: the connected shape as the periodicity tool left it, and its sub-shapes the tool changed in place
    // (natively the same shapes, changed), for the gluing history when the shape is made periodic again
    private void UpdateGlued()
    {
        _glued = _periodicityMaker.InternalInputShape();
        _gluedImages.Merge(_periodicityMaker.InternalInputImages());
    }

    /// <summary>Repeats the periodic shape in the direction the number of times.</summary>
    public void RepeatShape(int directionID, int times)
    {
        if (HasErrors)
        {
            return;
        }

        if (_periodicityMaker.InternalShape().IsNull || _periodicityMaker.HasErrors)
        {
            // The shape has not been made periodic yet
            AddWarning(new AlertShapeIsNotPeriodic(_shape));
            return;
        }

        // Repeat the shape
        _shape = _periodicityMaker.InternalRepeatShape(directionID, times);
        // Kerf: the gluing writes into the connected shape in place natively (as the trim, above)
        UpdateGlued();

        // Update history, materials, origins
        Update();
    }

    /// <summary>Clears the repetitions, keeping the shape periodic.</summary>
    public void ClearRepetitions()
    {
        if (HasErrors)
        {
            return;
        }

        if (_periodicityMaker.InternalShape().IsNull || _periodicityMaker.HasErrors)
        {
            // The shape has not been made periodic yet
            AddWarning(new AlertShapeIsNotPeriodic(_shape));
            return;
        }

        // Clear repetitions
        _periodicityMaker.ClearRepetitions();
        _shape = _periodicityMaker.InternalShape();

        // Update history, materials, origins
        Update();
    }

    /// <summary>The periodicity tool (a working object; its own accessors publish).</summary>
    public MakePeriodicAlgorithm PeriodicityTool() => _periodicityMaker;

    /// <summary>The materials (input shapes) on the positive side of the element (frozen).</summary>
    public ISchematiqList<Shape> MaterialsOnPositiveSide(in Shape s)
    {
        FreezeResults();
        return Published(InternalMaterialsOnPositiveSide(s));
    }

    /// <summary>The materials (input shapes) on the negative side of the element (frozen).</summary>
    public ISchematiqList<Shape> MaterialsOnNegativeSide(in Shape s)
    {
        FreezeResults();
        return Published(InternalMaterialsOnNegativeSide(s));
    }

    internal NCollection_List<Shape> InternalMaterialsOnPositiveSide(in Shape s) => Materials(s, Forward);

    internal NCollection_List<Shape> InternalMaterialsOnNegativeSide(in Shape s) => Materials(s, Reversed);

    private NCollection_List<Shape> Materials(in Shape s, Orientation side)
    {
        foreach (Shape aS in Candidates(s))
        {
            if (_materials.TryGetValue(aS.Oriented(side), out NCollection_List<Shape>? lm))
            {
                return lm;
            }
        }
        return EmptyList();
    }

    // Kerf: the shape, then what stands for it if the periodicity tool changed it in place (in a working overlay),
    // which natively is the shape itself
    private IEnumerable<Shape> Candidates(Shape s)
    {
        yield return s;
        foreach (Shape now in _periodicityMaker.InternalInPlace().Modified(s))
        {
            yield return now;
        }
    }

    /// <summary>The history of the operations (null before Perform); the result graphs are frozen first.</summary>
    public History? History()
    {
        FreezeResults();
        return _history;
    }

    /// <summary>The history, without freezing.</summary>
    internal History? InternalHistory() => _history;

    /// <summary>The shapes modified from the input shape (frozen).</summary>
    public ISchematiqList<Shape> GetModified(in Shape s)
    {
        FreezeResults();
        return _history is null ? PackedList<Shape>.Empty : _history.Modified(s);
    }

    /// <summary>The input shapes the shape of the result comes from (frozen).</summary>
    public ISchematiqList<Shape> GetOrigins(in Shape s)
    {
        FreezeResults();
        return Published(InternalGetOrigins(s));
    }

    internal NCollection_List<Shape> InternalGetOrigins(in Shape s)
        => _origins.TryGetValue(s, out NCollection_List<Shape>? lOr) ? lOr : EmptyList();

    /// <summary>The connected (glued) shape (frozen).</summary>
    public Shape Shape()
    {
        _glued.TShape()?.FreezeGraph();
        return _glued;
    }

    /// <summary>The connected shape, unfrozen, for ported code.</summary>
    internal Shape InternalShape() => _glued;

    /// <summary>The resulting shape: the connected shape made periodic and repeated (frozen).</summary>
    public Shape PeriodicShape()
    {
        _shape.TShape()?.FreezeGraph();
        return _shape;
    }

    /// <summary>The resulting shape, unfrozen, for ported code.</summary>
    internal Shape InternalPeriodicShape() => _shape;

    private void FreezeResults()
    {
        _glued.TShape()?.FreezeGraph();
        _shape.TShape()?.FreezeGraph();
        _periodicityMaker.InternalShape().TShape()?.FreezeGraph();
        _periodicityMaker.InternalRepeatedShape().TShape()?.FreezeGraph();
    }

    private static ISchematiqList<Shape> Published(NCollection_List<Shape> list)
        => list.IsEmpty() ? PackedList<Shape>.Empty : new PackedList<Shape>([.. list]);

    /// <summary>Clears the contents of the algorithm from previous runs.</summary>
    public override void Clear()
    {
        base.Clear();
        _arguments.Clear();
        _allInputsMap.Clear();
        _periodicityMaker.Clear();
        _origins.Clear();
        _materials.Clear();
        _glueHistory?.Clear();
        _gluedImages = new History();
        _history?.Clear();
        _glued.Nullify();
        _shape.Nullify();
    }

    /// <summary>Checks the validity of input data.</summary>
    protected void CheckData()
    {
        // Check the number of arguments
        if (_arguments.IsEmpty())
        {
            // Not enough arguments
            AddError(new AlertTooFewArguments());
            return;
        }

        // Check that all shapes in arguments are of the same type

        // Extract the shapes from the compound arguments
        NCollection_List<Shape> la = new();
        // Fence map
        HashSet<Shape> mFence = new(ShapeHasher.Instance);

        NCollection_List<Shape>.Iterator itLA = new(_arguments);
        for (; itLA.More(); itLA.Next())
        {
            AlgoTools.TreatCompound(itLA.Value(), la, mFence);
        }

        if (la.IsEmpty())
        {
            // It seems that all argument shapes are empty compounds
            AddError(new AlertTooFewArguments());
            return;
        }

        // Check dimensions of the extracted non-compound shapes
        itLA.Initialize(la);
        int iDim = AlgoTools.Dimension(itLA.Value());
        for (itLA.Next(); itLA.More(); itLA.Next())
        {
            if (iDim != AlgoTools.Dimension(itLA.Value()))
            {
                // The arguments are of different type
                AddError(new AlertMultiDimensionalArguments());
                return;
            }
        }
    }

    /// <summary>Glues the arguments.</summary>
    protected void MakeConnected()
    {
        // Initialize the history
        _glueHistory ??= new History();

        if (_arguments.Extent() == 1)
        {
            // No need to glue the single shape
            _shape = _arguments.First();
        }
        else
        {
            // Glue the shapes
            BooleanBuilder gluer = new();
            gluer.SetArguments(_arguments);
            gluer.SetGlue(GlueMode.GlueShift);
            gluer.SetRunParallel(_runParallel);
            gluer.SetNonDestructive(true);
            gluer.Perform();
            if (gluer.HasErrors)
            {
                // Unable to glue the shapes
                Compound cw = new();
                new BrepBuilder().MakeCompound(ref cw);
                for (NCollection_List<Shape>.Iterator it = new(_arguments); it.More(); it.Next())
                {
                    new BrepBuilder().Add(cw, it.Value());
                }
                AddError(new AlertUnableToGlue(cw));
                return;
            }
            _shape = gluer.InternalShape();
            // Save the gluing history
            _glueHistory.Merge(gluer.Arguments(), new MakePeriodicAlgorithm.BuilderHistory(gluer));
            _history!.Merge(_glueHistory);
        }

        // Keep the glued shape
        _glued = _shape;

        // Fill the map of origins
        FillOrigins();
    }

    /// <summary>Fills the map of origins.</summary>
    protected void FillOrigins()
    {
        _origins.Clear();

        // Map the history shapes of the arguments
        if (_allInputsMap.Count == 0)
        {
            NCollection_List<Shape>.Iterator itLA = new(_arguments);
            for (; itLA.More(); itLA.Next())
            {
                Exploration.MapShapes(itLA.Value(), _allInputsMap);
            }
        }

        int nbS = _allInputsMap.Count;
        for (int i = 0; i < nbS; ++i)
        {
            Shape s = _allInputsMap[i];
            if (!global::Kerf.Engine.Topology.History.IsSupportedType(s))
            {
                continue;
            }

            // Get Modified & Generated shapes
            for (int j = 0; j < 2; ++j)
            {
                ISchematiqList<Shape> lh = j == 0 ? _history!.Modified(s) : _history!.Generated(s);
                foreach (Shape hs in lh)
                {
                    if (!_origins.TryGetValue(hs, out NCollection_List<Shape>? pLOr))
                    {
                        pLOr = _origins.SetValueRef(hs, new NCollection_List<Shape>());
                    }
                    if (!pLOr.Contains(s))
                    {
                        pLOr.Append(s);
                    }
                }
            }
        }
    }

    /// <summary>Associates the materials for the border elements.</summary>
    protected void AssociateMaterials()
    {
        _materials.Clear();

        // Extract all non-compound shapes from the result
        NCollection_List<Shape> lShapes = new();
        HashSet<Shape> mFence = new(ShapeHasher.Instance);
        AlgoTools.TreatCompound(_shape, lShapes, mFence);

        if (lShapes.IsEmpty())
        {
            return;
        }

        // Define the element type and the material type
        ShapeType elemType;
        ShapeType materialType = lShapes.First().Type;
        if (materialType == ShapeType.Solid || materialType == ShapeType.CompSolid)
        {
            elemType = ShapeType.Face;
        }
        else if (materialType == ShapeType.Face || materialType == ShapeType.Shell)
        {
            elemType = ShapeType.Edge;
        }
        else if (materialType == ShapeType.Edge || materialType == ShapeType.Wire)
        {
            elemType = ShapeType.Vertex;
        }
        else
        {
            return;
        }

        NCollection_List<Shape>.Iterator itLS = new(lShapes);
        for (; itLS.More(); itLS.Next())
        {
            Shape s = itLS.Value();
            NCollection_List<Shape> lOr = InternalGetOrigins(s);
            Shape sOr = lOr.IsEmpty() ? s : lOr.First();

            Explorer exp = new(s, elemType);
            for (; exp.More(); exp.Next())
            {
                Shape element = exp.Current();
                if (!_materials.TryGetValue(element, out NCollection_List<Shape>? pLM))
                {
                    pLM = _materials.SetValueRef(element, new NCollection_List<Shape>());
                }
                pLM.Append(sOr);
            }
        }
    }

    /// <summary>Updates the history, material associations and origins map after periodicity operations.</summary>
    protected void Update()
    {
        // Update history
        _history!.Clear();
        if (_glueHistory is not null)
        {
            _history.Merge(_glueHistory);
        }
        if (_periodicityMaker.InternalHistory() is { } pHistory)
        {
            _history.Merge(pHistory);
        }

        // Fill the map of origins
        FillOrigins();

        // Update the material associations after making the shape periodic
        AssociateMaterials();
    }

    // A new empty list each time: callers can't change a shared one
    private static NCollection_List<Shape> EmptyList() => new();
}
