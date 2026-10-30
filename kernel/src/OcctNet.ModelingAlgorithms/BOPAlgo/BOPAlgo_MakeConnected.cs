// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakeConnected.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakeConnected.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2018-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using static Occt.BOPAlgo_GlueEnum;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

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
    protected NCollection_List<TopoDS_Shape> myArguments = new();                          //!< Input shapes for making them connected
    protected NCollection_IndexedMap<TopoDS_Shape> myAllInputsMap = new(TopTools_ShapeMapHasher.Instance); //!< Map of all BRep sub-elements of the input shapes
    protected BOPAlgo_MakePeriodic myPeriodicityMaker = new();                             //!< Tool for making the shape periodic
    protected NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMaterials = new(EqualityComparer<TopoDS_Shape>.Default); //!< Map of the materials associations
    protected NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myOrigins = new(TopTools_ShapeMapHasher.Instance); //!< Map of origins
    protected BRepTools_History? myGlueHistory;                                            //!< Gluing History
    protected BRepTools_History? myHistory;                                                //!< Final History of shapes modifications
    protected TopoDS_Shape myGlued = new();                                                //!< The resulting connected (glued) shape
    protected TopoDS_Shape myShape = new();                                                //!< The resulting shape
    private BRepTools_History myCopies = new();                                            //!< OCCT.NET: the frozen shapes replaced by copies

    public BOPAlgo_MakeConnected()
    {
    }

    /// <summary>Sets the shapes to make connected (the list is copied).</summary>
    public void SetArguments(NCollection_List<TopoDS_Shape> theArgs) => myArguments = new NCollection_List<TopoDS_Shape>(theArgs);

    /// <summary>Adds a shape to make connected.</summary>
    public void AddArgument(in TopoDS_Shape theS) => myArguments.Append(theS);

    /// <summary>The shapes to make connected.</summary>
    public NCollection_List<TopoDS_Shape> Arguments() => myArguments;

    /// <summary>Performs the operation: glues the arguments and associates the materials.</summary>
    public void Perform()
    {
        // Check the input data
        CheckData();
        if (HasErrors())
        {
            return;
        }

        myHistory ??= new BRepTools_History();

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
        BRepTools_History aCopyHistory = new();
        myGlued = BRepTools.CopyFrozenForWrite(myGlued, aCopyHistory);
        myGlueHistory ??= new BRepTools_History();
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

        if (myPeriodicityMaker.InternalShape().IsNull() || myPeriodicityMaker.HasErrors())
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

        if (myPeriodicityMaker.InternalShape().IsNull() || myPeriodicityMaker.HasErrors())
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
    public IBentoList<TopoDS_Shape> MaterialsOnPositiveSide(in TopoDS_Shape theS)
    {
        FreezeResults();
        return Published(InternalMaterialsOnPositiveSide(theS));
    }

    /// <summary>The materials (input shapes) on the negative side of the element (frozen).</summary>
    public IBentoList<TopoDS_Shape> MaterialsOnNegativeSide(in TopoDS_Shape theS)
    {
        FreezeResults();
        return Published(InternalMaterialsOnNegativeSide(theS));
    }

    internal NCollection_List<TopoDS_Shape> InternalMaterialsOnPositiveSide(in TopoDS_Shape theS) => Materials(theS, TopAbs_FORWARD);

    internal NCollection_List<TopoDS_Shape> InternalMaterialsOnNegativeSide(in TopoDS_Shape theS) => Materials(theS, TopAbs_REVERSED);

    private NCollection_List<TopoDS_Shape> Materials(in TopoDS_Shape theS, TopAbs_Orientation theSide)
    {
        foreach (TopoDS_Shape aS in Candidates(theS))
        {
            if (myMaterials.Find(aS.Oriented(theSide), out NCollection_List<TopoDS_Shape> aLM))
            {
                return aLM;
            }
        }
        return EmptyList();
    }

    // OCCT.NET: the shape, then the copies that replaced it if it was frozen (CopyFrozenForWrite), which natively are
    // the shape itself: the connected shape's copies, and the periodicity tool's
    private IEnumerable<TopoDS_Shape> Candidates(TopoDS_Shape theS)
    {
        yield return theS;
        List<TopoDS_Shape> aFirst = [theS, .. myCopies.Modified(theS)];
        foreach (TopoDS_Shape aS in aFirst.Skip(1))
        {
            yield return aS;
        }
        foreach (TopoDS_Shape aS in aFirst)
        {
            foreach (TopoDS_Shape aCopy in myPeriodicityMaker.InternalCopies().Modified(aS))
            {
                yield return aCopy;
            }
        }
    }

    /// <summary>The history of the operations (null before Perform); the result graphs are frozen first.</summary>
    public BRepTools_History? History()
    {
        FreezeResults();
        return myHistory;
    }

    /// <summary>The history, without freezing.</summary>
    internal BRepTools_History? InternalHistory() => myHistory;

    /// <summary>The shapes modified from the input shape (frozen).</summary>
    public IBentoList<TopoDS_Shape> GetModified(in TopoDS_Shape theS)
    {
        FreezeResults();
        return myHistory is null ? PackedList<TopoDS_Shape>.Empty : myHistory.Modified(theS);
    }

    /// <summary>The input shapes the shape of the result comes from (frozen).</summary>
    public IBentoList<TopoDS_Shape> GetOrigins(in TopoDS_Shape theS)
    {
        FreezeResults();
        return Published(InternalGetOrigins(theS));
    }

    internal NCollection_List<TopoDS_Shape> InternalGetOrigins(in TopoDS_Shape theS)
        => myOrigins.Find(theS, out NCollection_List<TopoDS_Shape> aLOr) ? aLOr : EmptyList();

    /// <summary>The connected (glued) shape (frozen).</summary>
    public TopoDS_Shape Shape()
    {
        myGlued.TShape()?.FreezeGraph();
        return myGlued;
    }

    /// <summary>The connected shape, unfrozen, for ported code.</summary>
    internal TopoDS_Shape InternalShape() => myGlued;

    /// <summary>The resulting shape: the connected shape made periodic and repeated (frozen).</summary>
    public TopoDS_Shape PeriodicShape()
    {
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    /// <summary>The resulting shape, unfrozen, for ported code.</summary>
    internal TopoDS_Shape InternalPeriodicShape() => myShape;

    private void FreezeResults()
    {
        myGlued.TShape()?.FreezeGraph();
        myShape.TShape()?.FreezeGraph();
        myPeriodicityMaker.InternalShape().TShape()?.FreezeGraph();
        myPeriodicityMaker.InternalRepeatedShape().TShape()?.FreezeGraph();
    }

    private static IBentoList<TopoDS_Shape> Published(NCollection_List<TopoDS_Shape> theList)
        => theList.IsEmpty() ? PackedList<TopoDS_Shape>.Empty : new PackedList<TopoDS_Shape>([.. theList]);

    /// <summary>Clears the contents of the algorithm from previous runs.</summary>
    public override void Clear()
    {
        base.Clear();
        myArguments.Clear();
        myAllInputsMap.Clear();
        myPeriodicityMaker.Clear();
        myOrigins.Clear();
        myMaterials.Clear();
        myCopies = new BRepTools_History();
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
        NCollection_List<TopoDS_Shape> aLA = new();
        // Fence map
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);

        NCollection_List<TopoDS_Shape>.Iterator itLA = new(myArguments);
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
        myGlueHistory ??= new BRepTools_History();

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
                TopoDS_Compound aCW = new();
                new BRep_Builder().MakeCompound(ref aCW);
                for (NCollection_List<TopoDS_Shape>.Iterator it = new(myArguments); it.More(); it.Next())
                {
                    new BRep_Builder().Add(aCW, it.Value());
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
            NCollection_List<TopoDS_Shape>.Iterator itLA = new(myArguments);
            for (; itLA.More(); itLA.Next())
            {
                TopExp.MapShapes(itLA.Value(), myAllInputsMap);
            }
        }

        int aNbS = myAllInputsMap.Extent();
        for (int i = 1; i <= aNbS; ++i)
        {
            TopoDS_Shape aS = myAllInputsMap[i];
            if (!BRepTools_History.IsSupportedType(aS))
            {
                continue;
            }

            // Get Modified & Generated shapes
            for (int j = 0; j < 2; ++j)
            {
                IBentoList<TopoDS_Shape> aLH = j == 0 ? myHistory!.Modified(aS) : myHistory!.Generated(aS);
                foreach (TopoDS_Shape aHS in aLH)
                {
                    if (!myOrigins.Find(aHS, out NCollection_List<TopoDS_Shape> pLOr))
                    {
                        pLOr = myOrigins.Bound(aHS, new NCollection_List<TopoDS_Shape>());
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
        NCollection_List<TopoDS_Shape> aLShapes = new();
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
        BOPTools_AlgoTools.TreatCompound(myShape, aLShapes, aMFence);

        if (aLShapes.IsEmpty())
        {
            return;
        }

        // Define the element type and the material type
        TopAbs_ShapeEnum anElemType;
        TopAbs_ShapeEnum aMaterialType = aLShapes.First().ShapeType();
        if (aMaterialType == TopAbs_SOLID || aMaterialType == TopAbs_COMPSOLID)
        {
            anElemType = TopAbs_FACE;
        }
        else if (aMaterialType == TopAbs_FACE || aMaterialType == TopAbs_SHELL)
        {
            anElemType = TopAbs_EDGE;
        }
        else if (aMaterialType == TopAbs_EDGE || aMaterialType == TopAbs_WIRE)
        {
            anElemType = TopAbs_VERTEX;
        }
        else
        {
            return;
        }

        NCollection_List<TopoDS_Shape>.Iterator itLS = new(aLShapes);
        for (; itLS.More(); itLS.Next())
        {
            TopoDS_Shape aS = itLS.Value();
            NCollection_List<TopoDS_Shape> aLOr = InternalGetOrigins(aS);
            TopoDS_Shape aSOr = aLOr.IsEmpty() ? aS : aLOr.First();

            TopExp_Explorer anExp = new(aS, anElemType);
            for (; anExp.More(); anExp.Next())
            {
                TopoDS_Shape anElement = anExp.Current();
                if (!myMaterials.Find(anElement, out NCollection_List<TopoDS_Shape> pLM))
                {
                    pLM = myMaterials.Bound(anElement, new NCollection_List<TopoDS_Shape>());
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
    private static NCollection_List<TopoDS_Shape> EmptyList() => new();
}
