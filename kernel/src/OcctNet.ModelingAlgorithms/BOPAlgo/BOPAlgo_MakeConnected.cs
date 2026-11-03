// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakeConnected.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakeConnected.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2018-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Schematiq.Collections;
using static Occt.BOPAlgo_GlueEnum;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Makes touching shapes of the same dimension connected (glues them so that they share their coinciding parts),
/// associates the materials (the input shapes) on both sides of each boundary element, and can make the connected
/// shape periodic and repeat it.
/// </summary>
/// <remarks>
/// A working object (decision 414's pattern). <see cref="Shape"/>, <see cref="PeriodicShape"/>, the materials, the
/// origins, <see cref="GetModified"/> and <see cref="History"/> publish: they freeze the result graphs first. Ported
/// code reads the <c>Internal*</c> forms. The gluing runs non-destructively, as in OCCT, so the arguments are never
/// written. The periodicity natively writes into the connected shape in place, so its frozen parts (the arguments'
/// untouched sub-shapes) are replaced by copies first, and the materials of a replaced shape are its copy's. The
/// materials map keys elements with their orientation (OCCT's default hasher).
/// </remarks>
public class BOPAlgo_MakeConnected : BOPAlgo_Options
{
    protected NCollection_List<Shape> myArguments = new();                          //!< Input shapes for making them connected
    protected NCollection_IndexedMap<Shape> myAllInputsMap = new(ShapeHasher.Instance); //!< Map of all BRep sub-elements of the input shapes
    protected BOPAlgo_MakePeriodic myPeriodicityMaker = new();                             //!< Tool for making the shape periodic
    protected NCollection_DataMap<Shape, NCollection_List<Shape>> myMaterials = new(EqualityComparer<Shape>.Default); //!< Map of the materials associations
    protected NCollection_DataMap<Shape, NCollection_List<Shape>> myOrigins = new(ShapeHasher.Instance); //!< Map of origins
    protected History? myGlueHistory;                                            //!< Gluing History
    protected History? myHistory;                                                //!< Final History of shapes modifications
    protected Shape myGlued = new();                                                //!< The resulting connected (glued) shape
    protected Shape myShape = new();                                                //!< The resulting shape
    private History myCopies = new();                                            //!< OCCT.NET: the frozen shapes replaced by copies

    public BOPAlgo_MakeConnected()
    {
    }

    /// <summary>Sets the shapes to make connected (the list is copied).</summary>
    public void SetArguments(NCollection_List<Shape> theArgs) => myArguments = new NCollection_List<Shape>(theArgs);

    /// <summary>Adds a shape to make connected.</summary>
    public void AddArgument(in Shape theS) => myArguments.Append(theS);

    /// <summary>The shapes to make connected.</summary>
    public NCollection_List<Shape> Arguments() => myArguments;

    /// <summary>Performs the operation: glues the arguments and associates the materials.</summary>
    public void Perform()
    {
        // Check the input data
        CheckData();
        if (HasErrors())
        {
            return;
        }

        myHistory ??= new History();

        // Glue the arguments
        MakeConnected();
        if (HasErrors())
        {
            return;
        }

        // Perform material associations for the faces
        AssociateMaterials();
        if (HasErrors())
        {
            return;
        }
    }

    /// <summary>Makes the connected shape periodic with the parameters.</summary>
    public void MakePeriodic(in BOPAlgo_MakePeriodic.PeriodicityParams theParams)
    {
        if (HasErrors())
        {
            return;
        }

        // OCCT.NET: making the shape periodic and repeating it writes into the connected shape in place natively
        // (the trim and the gluing of the repetitions are destructive), and the connected shape shares the
        // arguments' untouched sub-shapes. Its frozen parts are replaced by copies here, in place in its unfrozen
        // parents, so that those writes reach the connected shape as in OCCT while the arguments are never written.
        History aCopyHistory = new();
        myGlued = BrepTools.CopyFrozenForWrite(myGlued, aCopyHistory);
        myGlueHistory ??= new History();
        myGlueHistory.Merge(aCopyHistory);
        myCopies.Merge(aCopyHistory);

        // Make the shape periodic
        myPeriodicityMaker.Clear();
        myPeriodicityMaker.SetShape(myGlued);
        myPeriodicityMaker.SetPeriodicityParameters(theParams);
        myPeriodicityMaker.SetRunParallel(myRunParallel);
        myPeriodicityMaker.Perform();
        if (myPeriodicityMaker.HasErrors())
        {
            // Add warning informing the user that periodicity with
            // given parameters is not possible
            AddWarning(new BOPAlgo_AlertUnableToMakePeriodic(myShape));
            return;
        }

        myShape = myPeriodicityMaker.InternalShape();

        // Update history, materials, origins
        Update();
    }

    /// <summary>Repeats the periodic shape in the direction the number of times.</summary>
    public void RepeatShape(int theDirectionID, int theTimes)
    {
        if (HasErrors())
        {
            return;
        }

        if (myPeriodicityMaker.InternalShape().IsNull || myPeriodicityMaker.HasErrors())
        {
            // The shape has not been made periodic yet
            AddWarning(new BOPAlgo_AlertShapeIsNotPeriodic(myShape));
            return;
        }

        // Repeat the shape
        myShape = myPeriodicityMaker.InternalRepeatShape(theDirectionID, theTimes);

        // Update history, materials, origins
        Update();
    }

    /// <summary>Clears the repetitions, keeping the shape periodic.</summary>
    public void ClearRepetitions()
    {
        if (HasErrors())
        {
            return;
        }

        if (myPeriodicityMaker.InternalShape().IsNull || myPeriodicityMaker.HasErrors())
        {
            // The shape has not been made periodic yet
            AddWarning(new BOPAlgo_AlertShapeIsNotPeriodic(myShape));
            return;
        }

        // Clear repetitions
        myPeriodicityMaker.ClearRepetitions();
        myShape = myPeriodicityMaker.InternalShape();

        // Update history, materials, origins
        Update();
    }

    /// <summary>The periodicity tool (a working object; its own accessors publish).</summary>
    public BOPAlgo_MakePeriodic PeriodicityTool() => myPeriodicityMaker;

    /// <summary>The materials (input shapes) on the positive side of the element (frozen).</summary>
    public ISchematiqList<Shape> MaterialsOnPositiveSide(in Shape theS)
    {
        FreezeResults();
        return Published(InternalMaterialsOnPositiveSide(theS));
    }

    /// <summary>The materials (input shapes) on the negative side of the element (frozen).</summary>
    public ISchematiqList<Shape> MaterialsOnNegativeSide(in Shape theS)
    {
        FreezeResults();
        return Published(InternalMaterialsOnNegativeSide(theS));
    }

    internal NCollection_List<Shape> InternalMaterialsOnPositiveSide(in Shape theS) => Materials(theS, Forward);

    internal NCollection_List<Shape> InternalMaterialsOnNegativeSide(in Shape theS) => Materials(theS, Reversed);

    private NCollection_List<Shape> Materials(in Shape theS, Orientation theSide)
    {
        foreach (Shape aS in Candidates(theS))
        {
            if (myMaterials.Find(aS.Oriented(theSide), out NCollection_List<Shape> aLM))
            {
                return aLM;
            }
        }
        return EmptyList();
    }

    // OCCT.NET: the shape, then the copies that replaced it if it was frozen (CopyFrozenForWrite), which natively are
    // the shape itself: the connected shape's copies, and the periodicity tool's
    private IEnumerable<Shape> Candidates(Shape theS)
    {
        yield return theS;
        List<Shape> aFirst = [theS, .. myCopies.Modified(theS)];
        foreach (Shape aS in aFirst.Skip(1))
        {
            yield return aS;
        }
        foreach (Shape aS in aFirst)
        {
            foreach (Shape aCopy in myPeriodicityMaker.InternalCopies().Modified(aS))
            {
                yield return aCopy;
            }
        }
    }

    /// <summary>The history of the operations (null before Perform); the result graphs are frozen first.</summary>
    public History? History()
    {
        FreezeResults();
        return myHistory;
    }

    /// <summary>The history, without freezing.</summary>
    internal History? InternalHistory() => myHistory;

    /// <summary>The shapes modified from the input shape (frozen).</summary>
    public ISchematiqList<Shape> GetModified(in Shape theS)
    {
        FreezeResults();
        return myHistory is null ? PackedList<Shape>.Empty : myHistory.Modified(theS);
    }

    /// <summary>The input shapes the shape of the result comes from (frozen).</summary>
    public ISchematiqList<Shape> GetOrigins(in Shape theS)
    {
        FreezeResults();
        return Published(InternalGetOrigins(theS));
    }

    internal NCollection_List<Shape> InternalGetOrigins(in Shape theS)
        => myOrigins.Find(theS, out NCollection_List<Shape> aLOr) ? aLOr : EmptyList();

    /// <summary>The connected (glued) shape (frozen).</summary>
    public Shape Shape()
    {
        myGlued.TShape()?.FreezeGraph();
        return myGlued;
    }

    /// <summary>The connected shape, unfrozen, for ported code.</summary>
    internal Shape InternalShape() => myGlued;

    /// <summary>The resulting shape: the connected shape made periodic and repeated (frozen).</summary>
    public Shape PeriodicShape()
    {
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    /// <summary>The resulting shape, unfrozen, for ported code.</summary>
    internal Shape InternalPeriodicShape() => myShape;

    private void FreezeResults()
    {
        myGlued.TShape()?.FreezeGraph();
        myShape.TShape()?.FreezeGraph();
        myPeriodicityMaker.InternalShape().TShape()?.FreezeGraph();
        myPeriodicityMaker.InternalRepeatedShape().TShape()?.FreezeGraph();
    }

    private static ISchematiqList<Shape> Published(NCollection_List<Shape> theList)
        => theList.IsEmpty() ? PackedList<Shape>.Empty : new PackedList<Shape>([.. theList]);

    /// <summary>Clears the contents of the algorithm from previous runs.</summary>
    public override void Clear()
    {
        base.Clear();
        myArguments.Clear();
        myAllInputsMap.Clear();
        myPeriodicityMaker.Clear();
        myOrigins.Clear();
        myMaterials.Clear();
        myCopies = new History();
        myGlueHistory?.Clear();
        myHistory?.Clear();
        myGlued.Nullify();
        myShape.Nullify();
    }

    /// <summary>Checks the validity of input data.</summary>
    protected void CheckData()
    {
        // Check the number of arguments
        if (myArguments.IsEmpty())
        {
            // Not enough arguments
            AddError(new BOPAlgo_AlertTooFewArguments());
            return;
        }

        // Check that all shapes in arguments are of the same type

        // Extract the shapes from the compound arguments
        NCollection_List<Shape> aLA = new();
        // Fence map
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);

        NCollection_List<Shape>.Iterator itLA = new(myArguments);
        for (; itLA.More(); itLA.Next())
        {
            BOPTools_AlgoTools.TreatCompound(itLA.Value(), aLA, aMFence);
        }

        if (aLA.IsEmpty())
        {
            // It seems that all argument shapes are empty compounds
            AddError(new BOPAlgo_AlertTooFewArguments());
            return;
        }

        // Check dimensions of the extracted non-compound shapes
        itLA.Initialize(aLA);
        int iDim = BOPTools_AlgoTools.Dimension(itLA.Value());
        for (itLA.Next(); itLA.More(); itLA.Next())
        {
            if (iDim != BOPTools_AlgoTools.Dimension(itLA.Value()))
            {
                // The arguments are of different type
                AddError(new BOPAlgo_AlertMultiDimensionalArguments());
                return;
            }
        }
    }

    /// <summary>Glues the arguments.</summary>
    protected void MakeConnected()
    {
        // Initialize the history
        myGlueHistory ??= new History();

        if (myArguments.Extent() == 1)
        {
            // No need to glue the single shape
            myShape = myArguments.First();
        }
        else
        {
            // Glue the shapes
            BOPAlgo_Builder aGluer = new();
            aGluer.SetArguments(myArguments);
            aGluer.SetGlue(BOPAlgo_GlueShift);
            aGluer.SetRunParallel(myRunParallel);
            aGluer.SetNonDestructive(true);
            aGluer.Perform();
            if (aGluer.HasErrors())
            {
                // Unable to glue the shapes
                Compound aCW = new();
                new BrepBuilder().MakeCompound(ref aCW);
                for (NCollection_List<Shape>.Iterator it = new(myArguments); it.More(); it.Next())
                {
                    new BrepBuilder().Add(aCW, it.Value());
                }
                AddError(new BOPAlgo_AlertUnableToGlue(aCW));
                return;
            }
            myShape = aGluer.InternalShape();
            // Save the gluing history
            myGlueHistory.Merge(aGluer.Arguments(), new BOPAlgo_MakePeriodic.BuilderHistory(aGluer));
            myHistory!.Merge(myGlueHistory);
        }

        // Keep the glued shape
        myGlued = myShape;

        // Fill the map of origins
        FillOrigins();
    }

    /// <summary>Fills the map of origins.</summary>
    protected void FillOrigins()
    {
        myOrigins.Clear();

        // Map the history shapes of the arguments
        if (myAllInputsMap.IsEmpty())
        {
            NCollection_List<Shape>.Iterator itLA = new(myArguments);
            for (; itLA.More(); itLA.Next())
            {
                Exploration.MapShapes(itLA.Value(), myAllInputsMap);
            }
        }

        int aNbS = myAllInputsMap.Extent();
        for (int i = 1; i <= aNbS; ++i)
        {
            Shape aS = myAllInputsMap[i];
            if (!global::Kerf.Engine.Topology.History.IsSupportedType(aS))
            {
                continue;
            }

            // Get Modified & Generated shapes
            for (int j = 0; j < 2; ++j)
            {
                ISchematiqList<Shape> aLH = j == 0 ? myHistory!.Modified(aS) : myHistory!.Generated(aS);
                foreach (Shape aHS in aLH)
                {
                    if (!myOrigins.Find(aHS, out NCollection_List<Shape> pLOr))
                    {
                        pLOr = myOrigins.Bound(aHS, new NCollection_List<Shape>());
                    }
                    if (!pLOr.Contains(aS))
                    {
                        pLOr.Append(aS);
                    }
                }
            }
        }
    }

    /// <summary>Associates the materials for the border elements.</summary>
    protected void AssociateMaterials()
    {
        myMaterials.Clear();

        // Extract all non-compound shapes from the result
        NCollection_List<Shape> aLShapes = new();
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
        BOPTools_AlgoTools.TreatCompound(myShape, aLShapes, aMFence);

        if (aLShapes.IsEmpty())
        {
            return;
        }

        // Define the element type and the material type
        ShapeType anElemType;
        ShapeType aMaterialType = aLShapes.First().ShapeType();
        if (aMaterialType == ShapeType.Solid || aMaterialType == ShapeType.CompSolid)
        {
            anElemType = ShapeType.Face;
        }
        else if (aMaterialType == ShapeType.Face || aMaterialType == ShapeType.Shell)
        {
            anElemType = ShapeType.Edge;
        }
        else if (aMaterialType == ShapeType.Edge || aMaterialType == ShapeType.Wire)
        {
            anElemType = ShapeType.Vertex;
        }
        else
        {
            return;
        }

        NCollection_List<Shape>.Iterator itLS = new(aLShapes);
        for (; itLS.More(); itLS.Next())
        {
            Shape aS = itLS.Value();
            NCollection_List<Shape> aLOr = InternalGetOrigins(aS);
            Shape aSOr = aLOr.IsEmpty() ? aS : aLOr.First();

            Explorer anExp = new(aS, anElemType);
            for (; anExp.More(); anExp.Next())
            {
                Shape anElement = anExp.Current();
                if (!myMaterials.Find(anElement, out NCollection_List<Shape> pLM))
                {
                    pLM = myMaterials.Bound(anElement, new NCollection_List<Shape>());
                }
                pLM.Append(aSOr);
            }
        }
    }

    /// <summary>Updates the history, material associations and origins map after periodicity operations.</summary>
    protected void Update()
    {
        // Update history
        myHistory!.Clear();
        if (myGlueHistory is not null)
        {
            myHistory.Merge(myGlueHistory);
        }
        if (myPeriodicityMaker.InternalHistory() is { } aPHistory)
        {
            myHistory.Merge(aPHistory);
        }

        // Fill the map of origins
        FillOrigins();

        // Update the material associations after making the shape periodic
        AssociateMaterials();
    }

    // A new empty list each time: callers can't change a shared one
    private static NCollection_List<Shape> EmptyList() => new();
}
