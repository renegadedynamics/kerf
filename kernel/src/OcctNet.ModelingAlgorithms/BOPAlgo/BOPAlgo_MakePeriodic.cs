// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakePeriodic.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakePeriodic.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2018-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;
using Bento.Collections;
using static Occt.BOPAlgo_GlueEnum;

namespace Occt;

/// <summary>
/// Makes a shape periodic in the X, Y and Z directions: optionally trims it to the period, then splits its opposite
/// sides by each other so that they look the same, and associates the identical opposite sub-shapes (twins). The
/// periodic shape can then be repeated in its periodic directions.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 414's pattern). <see cref="Shape"/>, <see cref="RepeatShape"/>,
/// <see cref="RepeatedShape"/>, <see cref="GetTwins"/> and <see cref="History"/> publish: they freeze the result
/// graphs first. Ported code reads the <c>Internal*</c> forms. The input is never written: OCCT's own splits run
/// non-destructively, and its trim is a Common, which leaves a frozen input alone (decision 577).
/// </para>
/// <para>
/// The twins of a repeated shape are grouped by iterating a shape-keyed map, which natively runs in address order: the
/// groups are the same, but the order of a shape's twins in their list may differ from native's (occt-issues.md).
/// </para>
/// </remarks>
public class BOPAlgo_MakePeriodic : BOPAlgo_Options
{
    /// <summary>Three flags (OCCT's <c>bool[3]</c>), a value.</summary>
    [InlineArray(3)]
    public struct Flags3
    {
        private bool myElement0;
    }

    /// <summary>Three reals (OCCT's <c>double[3]</c>), a value.</summary>
    [InlineArray(3)]
    public struct Reals3
    {
        private double myElement0;
    }

    /// <summary>All periodicity parameters: for X, Y and Z, whether periodic, the period, and the trimming.</summary>
    /// <remarks>
    /// A value with OCCT's public fields: <c>new PeriodicityParams()</c> has OCCT's defaults (not periodic, trimmed);
    /// <c>default</c> is all zeros (not trimmed).
    /// </remarks>
    public struct PeriodicityParams : IEquatable<PeriodicityParams>
    {
        /// <summary>Whether the shape should be periodic in X, Y and Z.</summary>
        public Flags3 myPeriodic;

        /// <summary>The X, Y and Z periods (meaningful for the periodic directions only).</summary>
        public Reals3 myPeriod;

        /// <summary>Whether the input is already trimmed in X, Y and Z (else it is trimmed to the period).</summary>
        public Flags3 myIsTrimmed;

        /// <summary>The start of the X, Y and Z periods, for trimming.</summary>
        public Reals3 myPeriodFirst;

        public PeriodicityParams()
        {
            Clear();
        }

        internal void Clear()
        {
            myPeriodic[0] = myPeriodic[1] = myPeriodic[2] = false;
            myPeriod[0] = myPeriod[1] = myPeriod[2] = 0.0;
            myIsTrimmed[0] = myIsTrimmed[1] = myIsTrimmed[2] = true;
            myPeriodFirst[0] = myPeriodFirst[1] = myPeriodFirst[2] = 0.0;
        }

        public readonly bool Equals(PeriodicityParams theOther)
        {
            for (int i = 0; i < 3; ++i)
            {
                if (myPeriodic[i] != theOther.myPeriodic[i] || !myPeriod[i].Equals(theOther.myPeriod[i])
                    || myIsTrimmed[i] != theOther.myIsTrimmed[i] || !myPeriodFirst[i].Equals(theOther.myPeriodFirst[i]))
                {
                    return false;
                }
            }
            return true;
        }

        public override readonly bool Equals(object? theObj) => theObj is PeriodicityParams aP && Equals(aP);

        public override readonly int GetHashCode()
        {
            HashCode aH = new();
            for (int i = 0; i < 3; ++i)
            {
                aH.Add(myPeriodic[i]);
                aH.Add(myPeriod[i]);
                aH.Add(myIsTrimmed[i]);
                aH.Add(myPeriodFirst[i]);
            }
            return aH.ToHashCode();
        }

        public static bool operator ==(PeriodicityParams theLeft, PeriodicityParams theRight) => theLeft.Equals(theRight);

        public static bool operator !=(PeriodicityParams theLeft, PeriodicityParams theRight) => !theLeft.Equals(theRight);

        public override readonly string ToString()
        {
            System.Text.StringBuilder aB = new();
            for (int i = 0; i < 3; ++i)
            {
                aB.Append("XYZ"[i]).Append(": ");
                aB.Append(myPeriodic[i] ? gp.Format("periodic", myPeriod[i]) : "not periodic");
                aB.Append(myIsTrimmed[i] ? ", trimmed" : ", " + gp.Format("first", myPeriodFirst[i]));
                aB.Append(i < 2 ? "; " : "");
            }
            return aB.ToString();
        }
    }

    // Periodic/Trim/Repeat directions
    private static readonly gp_Dir[] MY_DIRECTIONS = [gp.DX(), gp.DY(), gp.DZ()];

    protected TopoDS_Shape myInputShape = new();                    //!< Input shape to make periodic
    protected PeriodicityParams myPeriodicityParams = new();        //!< Periodicity parameters
    protected TopoDS_Shape myShape = new();                         //!< Resulting periodic shape (base for repetitions)
    protected TopoDS_Shape myRepeatedShape = new();                 //!< Resulting shape after making repetitions of the base
    protected double[] myRepeatPeriod = new double[3];              //!< XYZ repeat period
    protected NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myRepeatedTwins = new(TopTools_ShapeMapHasher.Instance); //!< Map of associations of the identical sub-shapes
    protected NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myTwins = new(TopTools_ShapeMapHasher.Instance); //!< Map of associations of the identical sub-shapes
    protected BRepTools_History? mySplitHistory;                    //!< Split history - history of shapes modification
    protected BRepTools_History? myHistory;                         //!< Final history of shapes modifications
    private BRepTools_History myCopies = new();                     //!< OCCT.NET: the frozen shapes replaced by copies

    public BOPAlgo_MakePeriodic()
    {
        myRepeatPeriod[0] = myRepeatPeriod[1] = myRepeatPeriod[2] = 0.0;
    }

    /// <summary>Sets the shape to make periodic.</summary>
    public void SetShape(in TopoDS_Shape theShape) => myInputShape = theShape;

    /// <summary>Sets all periodicity parameters.</summary>
    public void SetPeriodicityParameters(in PeriodicityParams theParams) => myPeriodicityParams = theParams;

    /// <summary>The periodicity parameters.</summary>
    public PeriodicityParams PeriodicityParameters() => myPeriodicityParams;

    /// <summary>Makes the shape periodic (or not) in the direction (0 X, 1 Y, 2 Z; taken modulo 3) with the period.</summary>
    public void MakePeriodic(int theDirectionID, bool theIsPeriodic, double thePeriod = 0.0)
    {
        int id = ToDirectionID(theDirectionID);
        myPeriodicityParams.myPeriodic[id] = theIsPeriodic;
        myPeriodicityParams.myPeriod[id] = theIsPeriodic ? thePeriod : 0.0;
    }

    public bool IsPeriodic(int theDirectionID) => myPeriodicityParams.myPeriodic[ToDirectionID(theDirectionID)];

    public double Period(int theDirectionID)
    {
        int id = ToDirectionID(theDirectionID);
        return myPeriodicityParams.myPeriodic[id] ? myPeriodicityParams.myPeriod[id] : 0.0;
    }

    public void MakeXPeriodic(bool theIsPeriodic, double thePeriod = 0.0) => MakePeriodic(0, theIsPeriodic, thePeriod);

    public bool IsXPeriodic() => IsPeriodic(0);

    public double XPeriod() => Period(0);

    public void MakeYPeriodic(bool theIsPeriodic, double thePeriod = 0.0) => MakePeriodic(1, theIsPeriodic, thePeriod);

    public bool IsYPeriodic() => IsPeriodic(1);

    public double YPeriod() => Period(1);

    public void MakeZPeriodic(bool theIsPeriodic, double thePeriod = 0.0) => MakePeriodic(2, theIsPeriodic, thePeriod);

    public bool IsZPeriodic() => IsPeriodic(2);

    public double ZPeriod() => Period(2);

    /// <summary>
    /// Sets whether the input is trimmed in the direction (it is by default); if not, it is trimmed to the period
    /// starting at <paramref name="theFirst"/>. Ignored for a direction that isn't periodic.
    /// </summary>
    public void SetTrimmed(int theDirectionID, bool theIsTrimmed, double theFirst = 0.0)
    {
        int id = ToDirectionID(theDirectionID);
        if (IsPeriodic(id))
        {
            myPeriodicityParams.myIsTrimmed[id] = theIsTrimmed;
            myPeriodicityParams.myPeriodFirst[id] = !theIsTrimmed ? theFirst : 0.0;
        }
    }

    public bool IsInputTrimmed(int theDirectionID) => myPeriodicityParams.myIsTrimmed[ToDirectionID(theDirectionID)];

    public double PeriodFirst(int theDirectionID)
    {
        int id = ToDirectionID(theDirectionID);
        return !myPeriodicityParams.myIsTrimmed[id] ? myPeriodicityParams.myPeriodFirst[id] : 0.0;
    }

    // OCCT declares the start of the period of SetX/Y/ZTrimmed as a bool (`const bool theFirst = 0.0`), so it is 0 or 1
    /// <summary>SetTrimmed(0, ...). OCCT: <paramref name="theFirst"/> is a bool, giving a start of 0 or 1 (kept).</summary>
    public void SetXTrimmed(bool theIsTrimmed, bool theFirst = false) => SetTrimmed(0, theIsTrimmed, theFirst ? 1.0 : 0.0); // OCCT

    public bool IsInputXTrimmed() => IsInputTrimmed(0);

    public double XPeriodFirst() => PeriodFirst(0);

    /// <summary>SetTrimmed(1, ...). OCCT: <paramref name="theFirst"/> is a bool, giving a start of 0 or 1 (kept).</summary>
    public void SetYTrimmed(bool theIsTrimmed, bool theFirst = false) => SetTrimmed(1, theIsTrimmed, theFirst ? 1.0 : 0.0); // OCCT

    public bool IsInputYTrimmed() => IsInputTrimmed(1);

    public double YPeriodFirst() => PeriodFirst(1);

    /// <summary>SetTrimmed(2, ...). OCCT: <paramref name="theFirst"/> is a bool, giving a start of 0 or 1 (kept).</summary>
    public void SetZTrimmed(bool theIsTrimmed, bool theFirst = false) => SetTrimmed(2, theIsTrimmed, theFirst ? 1.0 : 0.0); // OCCT

    public bool IsInputZTrimmed() => IsInputTrimmed(2);

    public double ZPeriodFirst() => PeriodFirst(2);

    /// <summary>Repeats the periodic shape in the direction (negative times: in the negative direction); returns it, frozen.</summary>
    public TopoDS_Shape RepeatShape(int theDirectionID, int theTimes)
    {
        InternalRepeatShape(theDirectionID, theTimes);
        return RepeatedShape();
    }

    public TopoDS_Shape XRepeat(int theTimes) => RepeatShape(0, theTimes);

    public TopoDS_Shape YRepeat(int theTimes) => RepeatShape(1, theTimes);

    public TopoDS_Shape ZRepeat(int theTimes) => RepeatShape(2, theTimes);

    /// <summary>The repeated shape (frozen).</summary>
    public TopoDS_Shape RepeatedShape()
    {
        myRepeatedShape.TShape()?.FreezeGraph();
        return myRepeatedShape;
    }

    /// <summary>The repeated shape, unfrozen, for ported code.</summary>
    internal TopoDS_Shape InternalRepeatedShape() => myRepeatedShape;

    /// <summary>Clears the repetitions, keeping the shape periodic.</summary>
    public void ClearRepetitions()
    {
        myRepeatPeriod[0] = myRepeatPeriod[1] = myRepeatPeriod[2] = 0.0;
        myRepeatedShape.Nullify();
        myRepeatedTwins.Clear();
        if (myHistory is not null)
        {
            myHistory.Clear();
            if (mySplitHistory is not null)
            {
                myHistory.Merge(mySplitHistory);
            }
        }
    }

    /// <summary>The periodic shape (frozen).</summary>
    public TopoDS_Shape Shape()
    {
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    /// <summary>The periodic shape, unfrozen, for ported code.</summary>
    internal TopoDS_Shape InternalShape() => myShape;

    /// <summary>
    /// The sub-shapes identical to <paramref name="theS"/> on the opposite sides (of the repeated shape once it is
    /// repeated). The result graphs are frozen first.
    /// </summary>
    public IBentoList<TopoDS_Shape> GetTwins(in TopoDS_Shape theS)
    {
        FreezeResults();
        return new PackedList<TopoDS_Shape>([.. InternalGetTwins(theS)]);
    }

    /// <summary>OCCT's GetTwins, the algorithm's own list (empty if none), without freezing.</summary>
    internal NCollection_List<TopoDS_Shape> InternalGetTwins(in TopoDS_Shape theS)
    {
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMap =
            myRepeatedTwins.IsEmpty() ? myTwins : myRepeatedTwins;
        if (aMap.Find(theS, out NCollection_List<TopoDS_Shape> aTwins))
        {
            return aTwins;
        }
        // OCCT.NET: a frozen shape the algorithm replaced by a copy (decision on CopyFrozenForWrite) has the twins of
        // its copy, which natively is the shape itself
        foreach (TopoDS_Shape aCopy in myCopies.Modified(theS))
        {
            if (aMap.Find(aCopy, out aTwins))
            {
                return aTwins;
            }
        }
        return new NCollection_List<TopoDS_Shape>();
    }

    /// <summary>The history of the operation (null before it is performed); the result graphs are frozen first.</summary>
    public BRepTools_History? History()
    {
        FreezeResults();
        return myHistory;
    }

    /// <summary>The history, without freezing.</summary>
    internal BRepTools_History? InternalHistory() => myHistory;

    /// <summary>OCCT.NET: the frozen shapes the algorithm replaced by copies to write into, modified into the copies.</summary>
    internal BRepTools_History InternalCopies() => myCopies;

    private void FreezeResults()
    {
        myShape.TShape()?.FreezeGraph();
        myRepeatedShape.TShape()?.FreezeGraph();
    }

    /// <summary>Clears the algorithm from previous runs.</summary>
    public override void Clear()
    {
        base.Clear();
        myCopies = new BRepTools_History();
        myPeriodicityParams.Clear();
        myShape.Nullify();
        mySplitHistory?.Clear();
        myHistory?.Clear();
        ClearRepetitions();
    }

    /// <summary>The ID of the periodic direction (0, 1, 2) of an integer.</summary>
    public static int ToDirectionID(int theDirectionID) => Math.Abs(theDirectionID % 3);

    /// <summary>Performs the operation.</summary>
    public void Perform()
    {
        // Check the validity of input data
        CheckData();
        if (HasErrors())
        {
            return;
        }

        // Trim the shape to fit to the required period in
        // required periodic directions
        Trim();
        if (HasErrors())
        {
            return;
        }

        // Make the shape identical on the opposite sides in
        // required periodic directions
        MakeIdentical();
        if (HasErrors())
        {
            return;
        }
    }

    /// <summary>Checks the validity of input data.</summary>
    protected void CheckData()
    {
        if ((!IsXPeriodic() || XPeriod() < Precision.Confusion())
            && (!IsYPeriodic() || YPeriod() < Precision.Confusion())
            && (!IsZPeriodic() || ZPeriod() < Precision.Confusion()))
        {
            // Add error informing the user that no periodicity is required
            // or no valid period is set.

            AddError(new BOPAlgo_AlertNoPeriodicityRequired());
            return;
        }
    }

    // Adds the shape <theWhat> to the shape <theWhere>
    private static void AddToShape(in TopoDS_Shape theWhat, ref TopoDS_Shape theWhere)
    {
        if (theWhere.IsNull())
        {
            TopoDS_Compound aC = new();
            new BRep_Builder().MakeCompound(ref aC);
            theWhere = aC;
        }
        new BRep_Builder().Add(theWhere, theWhat);
    }

    // Adds the shape in the list <theLWhat> to the shape <theWhere>
    private static void AddToShape(NCollection_List<TopoDS_Shape> theLWhat, ref TopoDS_Shape theWhere)
    {
        NCollection_List<TopoDS_Shape>.Iterator it = new(theLWhat);
        for (; it.More(); it.Next())
        {
            AddToShape(it.Value(), ref theWhere);
        }
    }

    /// <summary>Makes the trim of the shape to fit to the periodic bounds.</summary>
    protected void Trim()
    {
        // Check if trim is required at all
        if (IsInputXTrimmed() && IsInputYTrimmed() && IsInputZTrimmed())
        {
            return;
        }

        // Compute bounding box for the shape to use it as a starting
        // volume for trimming. If required, the volume will be modified
        // to the requested trimming size in requested directions.
        Bnd_Box aBox = new();
        BRepBndLib.Add(myInputShape, ref aBox);
        // Enlarge box to avoid overlapping with the shape
        aBox.Enlarge(0.1 * Math.Sqrt(aBox.SquareExtent()));

        // Get Corner points of the bounding box
        gp_Pnt aPMin = aBox.CornerMin();
        gp_Pnt aPMax = aBox.CornerMax();

        // Update corner points according to the requested trim parameters
        for (int i = 0; i < 3; ++i)
        {
            if (IsInputTrimmed(i))
            {
                continue;
            }

            aPMin.SetCoord(i + 1, PeriodFirst(i));
            aPMax.SetCoord(i + 1, PeriodFirst(i) + Period(i));
        }

        // Build Trimming solid using corner points
        BRepPrimAPI_MakeBox aMBox = new(aPMin, aPMax);
        TopoDS_Shape aTrimBox = aMBox.InternalSolid();

        // OCCT.NET: the Common below is destructive and natively writes into the input (tolerances, p-curves): the
        // frozen parts of the input (the caller's shape, or the arguments' sub-shapes a connected shape shares) are
        // replaced by copies first, which it then writes into
        BRepTools_History aCopyHistory = new();
        myInputShape = BRepTools.CopyFrozenForWrite(myInputShape, aCopyHistory);
        myCopies.Merge(aCopyHistory);

        // Perform trimming of the shape by solid
        BRepAlgoAPI_Common aCommon = new();
        // Set Object
        NCollection_List<TopoDS_Shape> anObj = new();
        anObj.Append(myInputShape);
        aCommon.SetArguments(anObj);
        // Set Tool
        NCollection_List<TopoDS_Shape> aTool = new();
        aTool.Append(aTrimBox);
        aCommon.SetTools(aTool);
        // Set the parallel processing mode
        aCommon.SetRunParallel(myRunParallel);
        // Build
        aCommon.Build();
        if (aCommon.HasErrors())
        {
            // Unable to trim the shape
            // Merge errors from Common operation
            myReport.Merge(aCommon.GetReport());
            // Add new error saving the shapes for analysis
            TopoDS_Shape aWS = new();
            AddToShape(myInputShape, ref aWS);
            AddToShape(aTrimBox, ref aWS);
            AddError(new BOPAlgo_AlertUnableToTrim(aWS));
            return;
        }
        // Get the trimmed shape
        myShape = aCommon.InternalShape();
        // Fill the History for the object only
        mySplitHistory = new BRepTools_History();
        mySplitHistory.Merge(aCopyHistory);
        mySplitHistory.Merge(anObj, new MakeShapeHistory(aCommon));
    }

    /// <summary>Makes the shape look the same on the opposite sides in the required periodic directions.</summary>
    protected void MakeIdentical()
    {
        if (myShape.IsNull())
        {
            myShape = myInputShape;
        }

        mySplitHistory ??= new BRepTools_History();

        // Split the negative side of the shape with the geometry
        // located on the positive side
        SplitNegative();
        if (HasErrors())
        {
            return;
        }

        // Split the positive side of the shape with the geometry
        // located on the negative side.
        // Make sure that the opposite sides have identical geometries.
        // Make associations between identical opposite shapes.
        SplitPositive();

        myHistory = new BRepTools_History();
        myHistory.Merge(mySplitHistory);
    }

    /// <summary>Splits the negative side of the shape with the geometry located on the positive side.</summary>
    protected void SplitNegative()
    {
        // Copy geometry from positive side of the shape to the negative first.
        // So, translate the shape in negative periodic directions only.
        //
        // To avoid conflicts when copying geometries from positive periodic sides
        // perform split of each periodic side in a separate operation.
        for (int i = 0; i < 3; ++i)
        {
            if (!IsPeriodic(i))
            {
                continue;
            }

            // Translate the shape to the negative side
            gp_Trsf aNegTrsf = new();
            aNegTrsf.SetTranslationPart(new gp_Vec(MY_DIRECTIONS[i].Reversed()) * Period(i));
            BRepBuilderAPI_Transform aNegT = new(myShape, aNegTrsf, false);

            // Split the negative side of the shape.
            NCollection_List<TopoDS_Shape> aTools = new();
            aTools.Append(aNegT.InternalShape());
            SplitShape(aTools, mySplitHistory);
        }
    }

    // Associates the shape <theS> with the shape <theTwin> in the map.
    private static void AddTwin(in TopoDS_Shape theS, in TopoDS_Shape theTwin,
                                NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMap)
    {
        if (!theMap.Find(theS, out NCollection_List<TopoDS_Shape> aTwins))
        {
            theMap.Bound(theS, new NCollection_List<TopoDS_Shape>()).Append(theTwin);
            return;
        }

        // Check if the twin shape is not yet present in the list
        NCollection_List<TopoDS_Shape>.Iterator itLT = new(aTwins);
        for (; itLT.More(); itLT.Next())
        {
            if (theTwin.IsSame(itLT.Value()))
            {
                break;
            }
        }

        if (!itLT.More())
        {
            aTwins.Append(theTwin);
        }
    }

    /// <summary>
    /// Splits the positive side of the shape with the geometry of the negative side, and associates the identical
    /// opposite sub-shapes.
    /// </summary>
    protected void SplitPositive()
    {
        // Prepare map of the sub-shapes of the input shape to make
        // associations of the opposite shapes
        NCollection_IndexedMap<TopoDS_Shape> aSubShapesMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(myShape, aSubShapesMap);
        int aNbS = aSubShapesMap.Extent();

        // Translate the shape to the positive periodic directions to make the
        // shapes look identical on the opposite sides.
        NCollection_List<TopoDS_Shape> aTools = new();

        // Remember the history of shapes translation
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aTranslationHistMap = new(TopTools_ShapeMapHasher.Instance);

        // Make translations for all periodic directions
        for (int i = 0; i < 3; ++i)
        {
            if (!IsPeriodic(i))
            {
                continue;
            }

            // Translate the shape to the positive side
            gp_Trsf aPosTrsf = new();
            aPosTrsf.SetTranslationPart(new gp_Vec(MY_DIRECTIONS[i]) * Period(i));
            BRepBuilderAPI_Transform aTranslator = new(myShape, aPosTrsf, false);
            aTools.Append(aTranslator.InternalShape());

            // Fill the translation history map
            for (int j = 1; j <= aNbS; ++j)
            {
                TopoDS_Shape aS = aSubShapesMap[j];
                if (BRepTools_History.IsSupportedType(aS))
                {
                    NCollection_List<TopoDS_Shape> aSM = aTranslator.InternalModified(aS);
                    int anIndex = aTranslationHistMap.FindIndex(aS);
                    if (anIndex == 0)
                    {
                        anIndex = aTranslationHistMap.Add(aS, new NCollection_List<TopoDS_Shape>());
                    }
                    aTranslationHistMap[anIndex].Append(aSM.First());
                }
            }
        }

        // Keep the split shape history and history of tools modifications
        // during the split for making association of the opposite identical shapes
        BRepTools_History aSplitShapeHist = new(), aSplitToolsHist = new();
        // Split the positive side of the shape
        SplitShape(aTools, aSplitShapeHist, aSplitToolsHist);
        if (HasErrors())
        {
            return;
        }

        mySplitHistory!.Merge(aSplitShapeHist);

        // Make associations between identical opposite sub-shapes
        int aNbSH = aTranslationHistMap.Extent();
        for (int i = 1; i <= aNbSH; ++i)
        {
            TopoDS_Shape pS = aTranslationHistMap.FindKey(i);
            IBentoList<TopoDS_Shape> aSIm = aSplitShapeHist.Modified(pS);
            if (aSIm.Count == 1)
            {
                pS = aSIm[0];
            }
            else if (aSIm.Count > 1)
            {
                continue;
            }

            NCollection_List<TopoDS_Shape> aLTranslated = aTranslationHistMap[i];

            NCollection_List<TopoDS_Shape>.Iterator itLT = new(aLTranslated);
            for (; itLT.More(); itLT.Next())
            {
                TopoDS_Shape aT = itLT.Value();
                // Get shapes modifications during the split
                IBentoList<TopoDS_Shape> aTSplits = aSplitToolsHist.Modified(aT);

                // Associate the shapes to each other
                foreach (TopoDS_Shape aSp in aTSplits)
                {
                    AddTwin(pS, aSp, myTwins);
                    AddTwin(aSp, pS, myTwins);
                }
            }
        }
    }

    /// <summary>Splits the shape by the given tools.</summary>
    protected void SplitShape(NCollection_List<TopoDS_Shape> theTools, BRepTools_History? theSplitShapeHistory = null,
                              BRepTools_History? theSplitToolsHistory = null)
    {
        // Make sure that the geometry from the tools will be copied to the split
        // shape. For that, the tool shapes should be given to the Boolean Operations
        // algorithm before the shape itself. This will make all coinciding parts
        // use the geometry of the first argument.

        // Intersection tool for passing ordered arguments
        BOPAlgo_PaveFiller anIntersector = new();
        anIntersector.SetArguments(theTools);
        // Add the shape
        anIntersector.AddArgument(myShape);
        // Use gluing to speed-up intersections
        anIntersector.SetGlue(BOPAlgo_GlueShift);
        // Use safe input mode, to avoid reusing geometry of the shape
        anIntersector.SetNonDestructive(true);
        // Set parallel processing mode
        anIntersector.SetRunParallel(myRunParallel);
        // Perform Intersection of the arguments
        anIntersector.Perform();
        // Check for the errors
        if (anIntersector.HasErrors())
        {
            // Unable to split the shape on opposite sides
            // Copy the intersection errors
            myReport.Merge(anIntersector.GetReport());
            // Add new error saving the shapes for analysis
            TopoDS_Shape aWS = new();
            AddToShape(theTools, ref aWS);
            AddToShape(myShape, ref aWS);
            AddError(new BOPAlgo_AlertUnableToMakeIdentical(aWS));
            return;
        }

        // Perform the splitting of the shape with the precomputed intersection results
        BRepAlgoAPI_Splitter aSplitter = new(anIntersector);
        // Set Object
        NCollection_List<TopoDS_Shape> anObj = new();
        anObj.Append(myShape);
        aSplitter.SetArguments(anObj);
        // Set Tools
        aSplitter.SetTools(theTools);
        // Use Gluing
        aSplitter.SetGlue(BOPAlgo_GlueShift);
        // Set parallel processing mode
        aSplitter.SetRunParallel(myRunParallel);
        // Perform splitting
        aSplitter.Build();
        // Check for the errors
        if (aSplitter.HasErrors())
        {
            // Unable to split the shape on opposite sides
            // Copy the splitter errors
            myReport.Merge(aSplitter.GetReport());
            // Add new error saving the shape for analysis
            TopoDS_Shape aWS = new();
            AddToShape(theTools, ref aWS);
            AddToShape(myShape, ref aWS);
            AddError(new BOPAlgo_AlertUnableToMakeIdentical(aWS));
            return;
        }
        // Get the split shape
        myShape = aSplitter.InternalShape();
        // Remember the split history
        theSplitShapeHistory?.Merge(anObj, new MakeShapeHistory(aSplitter));
        theSplitToolsHistory?.Merge(theTools, new MakeShapeHistory(aSplitter));
    }

    /// <summary>Repeats the shape in the required periodic direction; returns the repeated shape, unfrozen.</summary>
    internal TopoDS_Shape InternalRepeatShape(int theDir, int theTimes)
    {
        if (myRepeatedShape.IsNull())
        {
            myRepeatedShape = myShape;
        }

        if (!IsPeriodic(theDir))
        {
            return myRepeatedShape;
        }

        if (theTimes == 0)
        {
            return myRepeatedShape;
        }

        // Get the shape's period in the required direction
        int id = ToDirectionID(theDir);
        if (myRepeatPeriod[id] < Precision.Confusion())
        {
            myRepeatPeriod[id] = Period(id);
        }
        double aPeriod = myRepeatPeriod[id];

        // Coefficient to define in which direction the repetition will be performed:
        // theTimes is positive - in positive direction;
        // theTimes is negative - in negative direction.
        int iDir = theTimes > 0 ? 1 : -1;

        // OCCT.NET: the gluing below is destructive and writes into the shape to repeat (tolerances, p-curves), which
        // natively changes the periodic shape and the input's sub-shapes it shares. Frozen sub-shapes (the input's,
        // or the results already handed out) are replaced by copies first, which the gluing then writes into.
        CopyFrozenForWrite();

        // Create the translation history - all translated shapes will be
        // created as Generated from the shape.
        BRepTools_History aTranslationHistory = new();
        NCollection_IndexedMap<TopoDS_Shape> aSubShapesMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(myRepeatedShape, aSubShapesMap);
        int aNbS = aSubShapesMap.Extent();

        // Add shapes for gluing
        NCollection_List<TopoDS_Shape> aShapes = new();
        // Add the shape itself
        aShapes.Append(myRepeatedShape);
        for (int i = 1; i <= aNbS; ++i)
        {
            TopoDS_Shape aS = aSubShapesMap[i];
            if (BRepTools_History.IsSupportedType(aS))
            {
                aTranslationHistory.AddGenerated(aS, aS);
            }
        }

        // Create translated copies of the shape
        for (int i = 1; i <= Math.Abs(theTimes); ++i)
        {
            gp_Trsf aTrsf = new();
            aTrsf.SetTranslationPart(new gp_Vec(MY_DIRECTIONS[id]) * (iDir * i * aPeriod));
            BRepBuilderAPI_Transform aTranslator = new(myRepeatedShape, aTrsf, false);
            aShapes.Append(aTranslator.InternalShape());

            // Fill the translation history
            for (int j = 1; j <= aNbS; ++j)
            {
                TopoDS_Shape aS = aSubShapesMap[j];
                if (BRepTools_History.IsSupportedType(aS))
                {
                    NCollection_List<TopoDS_Shape> aLT = aTranslator.InternalModified(aS);
                    aTranslationHistory.AddGenerated(aS, aLT.First());
                }
            }
        }

        // Update the history with the translation History
        myHistory!.Merge(aTranslationHistory);

        // Glue the translated shapes all together
        BOPAlgo_Builder aGluer = new();
        aGluer.SetArguments(aShapes);
        // Avoid intersections of the sub-shapes
        aGluer.SetGlue(BOPAlgo_GlueFull);
        // Set parallel processing mode
        aGluer.SetRunParallel(myRunParallel);
        // Perform gluing
        aGluer.Perform();
        if (aGluer.HasErrors())
        {
            // Repetition in this direction is not possible
            // Add warning saving the shapes for analysis
            TopoDS_Shape aWS = new();
            AddToShape(aShapes, ref aWS);
            AddWarning(new BOPAlgo_AlertUnableToRepeat(aWS));
            return myRepeatedShape;
        }
        // Get glued shape
        myRepeatedShape = aGluer.InternalShape();

        // Update repetition period for the next repetitions
        myRepeatPeriod[id] += Math.Abs(theTimes) * myRepeatPeriod[id];

        // Update history with the Gluing history
        BRepTools_History aGluingHistory = new(aShapes, new BuilderHistory(aGluer));
        myHistory.Merge(aGluingHistory);

        // Update the map of twins after repetition
        UpdateTwins(aTranslationHistory, aGluingHistory);

        return myRepeatedShape;
    }

    /// <summary>
    /// OCCT.NET: replaces the frozen sub-shapes of the shape to repeat by exact copies (in place in its unfrozen parents,
    /// so the periodic shape sharing them sees the same shapes), recording the copies in the histories and the twins.
    /// </summary>
    private void CopyFrozenForWrite()
    {
        if (!BRepTools.HoldsFrozen(myRepeatedShape))
        {
            return;
        }
        BRepTools_History aCopyHistory = new();
        bool anIsShape = myRepeatedShape.IsSame(myShape);
        myRepeatedShape = BRepTools.CopyFrozenForWrite(myRepeatedShape, aCopyHistory);
        if (anIsShape)
        {
            myShape = myRepeatedShape.Oriented(myShape.Orientation());
        }
        myHistory!.Merge(aCopyHistory);
        mySplitHistory?.Merge(aCopyHistory);
        myCopies.Merge(aCopyHistory);
        myTwins = CopiedTwins(myTwins, aCopyHistory);
        myRepeatedTwins = CopiedTwins(myRepeatedTwins, aCopyHistory);
    }

    // The twins with the copied shapes in place of the frozen ones
    private static NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> CopiedTwins(
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMap, BRepTools_History theCopyHistory)
    {
        TopoDS_Shape Copied(in TopoDS_Shape theS)
        {
            IBentoList<TopoDS_Shape> aM = theCopyHistory.Modified(theS);
            return aM.Count == 1 ? aM[0].Oriented(theS.Orientation()) : theS;
        }
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aCopy = new(TopTools_ShapeMapHasher.Instance);
        foreach ((TopoDS_Shape aKey, NCollection_List<TopoDS_Shape> aList) in theMap)
        {
            NCollection_List<TopoDS_Shape> aNew = new();
            foreach (TopoDS_Shape aS in aList)
            {
                aNew.Append(Copied(aS));
            }
            aCopy.Bind(Copied(aKey), aNew);
        }
        return aCopy;
    }

    /// <summary>Updates the map of twins after repetition.</summary>
    protected void UpdateTwins(BRepTools_History theTranslationHistory, BRepTools_History theGluingHistory)
    {
        if (myTwins.IsEmpty())
        {
            return;
        }

        if (myRepeatedTwins.IsEmpty())
        {
            myRepeatedTwins = CopyTwins(myTwins);
        }

        // New twins
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aNewTwinsMap = new(TopTools_ShapeMapHasher.Instance);

        // Fence map to avoid repeated fill for the twins
        NCollection_Map<TopoDS_Shape> aMTwinsDone = new(TopTools_ShapeMapHasher.Instance);

        // Update the map of twins with the new repeated shapes
        // (native: address order; the port: insertion order, occt-issues.md)
        foreach ((TopoDS_Shape aS, NCollection_List<TopoDS_Shape> aLTwins) in myRepeatedTwins)
        {
            aMTwinsDone.Add(aS);

            // Check if the twins have not been already processed
            NCollection_List<TopoDS_Shape>.Iterator itLT = new(aLTwins);
            for (; itLT.More(); itLT.Next())
            {
                if (aMTwinsDone.Contains(itLT.Value()))
                {
                    break;
                }
            }
            if (itLT.More())
            {
                // Group of twins has already been processed
                continue;
            }

            // All shapes generated from the shape itself and generated
            // from its twins will be the new twins for the shape
            NCollection_IndexedMap<TopoDS_Shape> aNewGroup = new(TopTools_ShapeMapHasher.Instance);
            itLT.Initialize(aLTwins);

            for (bool bShape = true; itLT.More();)
            {
                TopoDS_Shape aTwin = bShape ? aS : itLT.Value();
                foreach (TopoDS_Shape aG in theTranslationHistory.Generated(aTwin))
                {
                    IBentoList<TopoDS_Shape> aLM = theGluingHistory.Modified(aG);
                    if (aLM.Count == 0)
                    {
                        aNewGroup.Add(aG);
                    }
                    else
                    {
                        foreach (TopoDS_Shape aM in aLM)
                        {
                            aNewGroup.Add(aM);
                        }
                    }
                }

                if (bShape)
                {
                    bShape = false;
                }
                else
                {
                    itLT.Next();
                }
            }

            // Associate the twins to each other
            int aNbTwins = aNewGroup.Extent();
            for (int i = 1; i <= aNbTwins; ++i)
            {
                ref NCollection_List<TopoDS_Shape> pTwins = ref aNewTwinsMap.Bound(aNewGroup[i], new NCollection_List<TopoDS_Shape>());
                for (int j = 1; j <= aNbTwins; ++j)
                {
                    if (i != j)
                    {
                        pTwins.Append(aNewGroup[j]);
                    }
                }
            }
        }

        myRepeatedTwins = aNewTwinsMap;
    }

    // OCCT's map assignment copies the lists
    private static NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> CopyTwins(
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMap)
    {
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aCopy = new(TopTools_ShapeMapHasher.Instance);
        foreach ((TopoDS_Shape aKey, NCollection_List<TopoDS_Shape> aList) in theMap)
        {
            aCopy.Bind(aKey, new NCollection_List<TopoDS_Shape>(aList));
        }
        return aCopy;
    }

    /// <summary>
    /// The history methods of a BRepBuilderAPI maker without publishing (OCCT's template history constructor and
    /// Merge take the algorithm; the port's public Modified/Generated would freeze the maker's result).
    /// </summary>
    internal sealed class MakeShapeHistory(BRepBuilderAPI_MakeShape theAlgo) : BRepTools_History.IAlgorithm
    {
        public bool IsDeleted(in TopoDS_Shape theS) => theAlgo.IsDeleted(theS);

        public IEnumerable<TopoDS_Shape> Modified(in TopoDS_Shape theS) => [.. theAlgo.InternalModified(theS)];

        public IEnumerable<TopoDS_Shape> Generated(in TopoDS_Shape theS) => [.. theAlgo.InternalGenerated(theS)];
    }

    /// <summary>The history methods of a BOPAlgo builder (BOPAlgo_BuilderShape's Modified, Generated, IsDeleted).</summary>
    internal sealed class BuilderHistory(BOPAlgo_BuilderShape theAlgo) : BRepTools_History.IAlgorithm
    {
        public bool IsDeleted(in TopoDS_Shape theS) => theAlgo.IsDeleted(theS);

        public IEnumerable<TopoDS_Shape> Modified(in TopoDS_Shape theS) => theAlgo.Modified(theS);

        public IEnumerable<TopoDS_Shape> Generated(in TopoDS_Shape theS) => theAlgo.Generated(theS);
    }
}
