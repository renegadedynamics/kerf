// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakePeriodic.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakePeriodic.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2018-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Booleans;

/// <summary>
/// Makes a shape periodic in the X, Y and Z directions: optionally trims it to the period, then splits its opposite
/// sides by each other so that they look the same, and associates the identical opposite sub-shapes (twins). The
/// periodic shape can then be repeated in its periodic directions.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 414's pattern). <see cref="Shape"/>, <see cref="RepeatShape"/>,
/// <see cref="RepeatedShape"/>, <see cref="GetTwins"/> and <see cref="History"/> publish: they freeze the result
/// graphs first. Ported code reads the <c>Internal*</c> forms. The input is never written. OCCT's splits run
/// non-destructively, but its trim (a Common) and the gluing of the repetitions are destructive and natively write into
/// the input and the periodic shape in place (tolerances, p-curves). The port runs them in a working overlay, which
/// takes the writes into their frozen parts (the caller's input, results already handed out), so results are native's;
/// the shapes, histories and twins then hold what stands for each changed shape, and the history records each shape
/// changed in place as modified into it.
/// </para>
/// <para>
/// The twins of a repeated shape are grouped by iterating a shape-keyed map, which natively runs in address order: the
/// groups are the same, but the order of a shape's twins in their list may differ from native's (occt-issues.md).
/// </para>
/// </remarks>
internal class MakePeriodicAlgorithm : Options
{
    /// <summary>Three flags (OCCT's <c>bool[3]</c>), a value.</summary>
    [InlineArray(3)]
    public struct Flags3
    {
        private bool _element0;
    }

    /// <summary>Three reals (OCCT's <c>double[3]</c>), a value.</summary>
    [InlineArray(3)]
    public struct Reals3
    {
        private double _element0;
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

        public readonly bool Equals(PeriodicityParams other)
        {
            for (int i = 0; i < 3; ++i)
            {
                if (myPeriodic[i] != other.myPeriodic[i] || !myPeriod[i].Equals(other.myPeriod[i])
                    || myIsTrimmed[i] != other.myIsTrimmed[i] || !myPeriodFirst[i].Equals(other.myPeriodFirst[i]))
                {
                    return false;
                }
            }
            return true;
        }

        public override readonly bool Equals(object? obj) => obj is PeriodicityParams p && Equals(p);

        public override readonly int GetHashCode()
        {
            HashCode h = new();
            for (int i = 0; i < 3; ++i)
            {
                h.Add(myPeriodic[i]);
                h.Add(myPeriod[i]);
                h.Add(myIsTrimmed[i]);
                h.Add(myPeriodFirst[i]);
            }
            return h.ToHashCode();
        }

        public static bool operator ==(PeriodicityParams left, PeriodicityParams right) => left.Equals(right);

        public static bool operator !=(PeriodicityParams left, PeriodicityParams right) => !left.Equals(right);

        public override readonly string ToString()
        {
            System.Text.StringBuilder b = new();
            for (int i = 0; i < 3; ++i)
            {
                b.Append("XYZ"[i]).Append(": ");
                b.Append(myPeriodic[i] ? Constants.Format("periodic", myPeriod[i]) : "not periodic");
                b.Append(myIsTrimmed[i] ? ", trimmed" : ", " + Constants.Format("first", myPeriodFirst[i]));
                b.Append(i < 2 ? "; " : "");
            }
            return b.ToString();
        }
    }

    // Periodic/Trim/Repeat directions
    private static readonly Direction3D[] MY_DIRECTIONS = [Constants.DX(), Constants.DY(), Constants.DZ()];

    protected Shape _inputShape = new();                    //!< Input shape to make periodic
    protected PeriodicityParams _periodicityParams = new();        //!< Periodicity parameters
    protected Shape _shape = new();                         //!< Resulting periodic shape (base for repetitions)
    protected Shape _repeatedShape = new();                 //!< Resulting shape after making repetitions of the base
    protected double[] _repeatPeriod = new double[3];              //!< XYZ repeat period
    protected Dictionary<Shape, NCollection_List<Shape>> _repeatedTwins = new(ShapeHasher.Instance); //!< Map of associations of the identical sub-shapes
    protected Dictionary<Shape, NCollection_List<Shape>> _twins = new(ShapeHasher.Instance); //!< Map of associations of the identical sub-shapes
    protected History? _splitHistory;                    //!< Split history - history of shapes modification
    protected History? _history;                         //!< Final history of shapes modifications
    private History _inPlace = new();                    //!< Kerf: the shapes changed in place, modified into what stands for them
    private History _inputImages = new();                //!< Kerf: the input's sub-shapes the last operation changed in place

    public MakePeriodicAlgorithm()
    {
        _repeatPeriod[0] = _repeatPeriod[1] = _repeatPeriod[2] = 0.0;
    }

    /// <summary>Sets the shape to make periodic.</summary>
    public void SetShape(in Shape shape) => _inputShape = shape;

    /// <summary>Sets all periodicity parameters.</summary>
    public void SetPeriodicityParameters(in PeriodicityParams theParams) => _periodicityParams = theParams;

    /// <summary>The periodicity parameters.</summary>
    public PeriodicityParams PeriodicityParameters() => _periodicityParams;

    /// <summary>Makes the shape periodic (or not) in the direction (0 X, 1 Y, 2 Z; taken modulo 3) with the period.</summary>
    public void MakePeriodic(int directionID, bool isPeriodic, double period = 0.0)
    {
        int id = ToDirectionID(directionID);
        _periodicityParams.myPeriodic[id] = isPeriodic;
        _periodicityParams.myPeriod[id] = isPeriodic ? period : 0.0;
    }

    public bool IsPeriodic(int directionID) => _periodicityParams.myPeriodic[ToDirectionID(directionID)];

    public double Period(int directionID)
    {
        int id = ToDirectionID(directionID);
        return _periodicityParams.myPeriodic[id] ? _periodicityParams.myPeriod[id] : 0.0;
    }

    public void MakeXPeriodic(bool isPeriodic, double period = 0.0) => MakePeriodic(0, isPeriodic, period);

    public bool IsXPeriodic => IsPeriodic(0);

    public double XPeriod() => Period(0);

    public void MakeYPeriodic(bool isPeriodic, double period = 0.0) => MakePeriodic(1, isPeriodic, period);

    public bool IsYPeriodic => IsPeriodic(1);

    public double YPeriod() => Period(1);

    public void MakeZPeriodic(bool isPeriodic, double period = 0.0) => MakePeriodic(2, isPeriodic, period);

    public bool IsZPeriodic => IsPeriodic(2);

    public double ZPeriod() => Period(2);

    /// <summary>
    /// Sets whether the input is trimmed in the direction (it is by default); if not, it is trimmed to the period
    /// starting at <paramref name="first"/>. Ignored for a direction that isn't periodic.
    /// </summary>
    public void SetTrimmed(int directionID, bool isTrimmed, double first = 0.0)
    {
        int id = ToDirectionID(directionID);
        if (IsPeriodic(id))
        {
            _periodicityParams.myIsTrimmed[id] = isTrimmed;
            _periodicityParams.myPeriodFirst[id] = !isTrimmed ? first : 0.0;
        }
    }

    public bool IsInputTrimmed(int directionID) => _periodicityParams.myIsTrimmed[ToDirectionID(directionID)];

    public double PeriodFirst(int directionID)
    {
        int id = ToDirectionID(directionID);
        return !_periodicityParams.myIsTrimmed[id] ? _periodicityParams.myPeriodFirst[id] : 0.0;
    }

    // OCCT declares the start of the period of SetX/Y/ZTrimmed as a bool (`const bool theFirst = 0.0`), so it is 0 or 1
    /// <summary>SetTrimmed(0, ...). OCCT: <paramref name="first"/> is a bool, giving a start of 0 or 1 (kept).</summary>
    public void SetXTrimmed(bool isTrimmed, bool first = false) => SetTrimmed(0, isTrimmed, first ? 1.0 : 0.0); // OCCT

    public bool IsInputXTrimmed => IsInputTrimmed(0);

    public double XPeriodFirst() => PeriodFirst(0);

    /// <summary>SetTrimmed(1, ...). OCCT: <paramref name="first"/> is a bool, giving a start of 0 or 1 (kept).</summary>
    public void SetYTrimmed(bool isTrimmed, bool first = false) => SetTrimmed(1, isTrimmed, first ? 1.0 : 0.0); // OCCT

    public bool IsInputYTrimmed => IsInputTrimmed(1);

    public double YPeriodFirst() => PeriodFirst(1);

    /// <summary>SetTrimmed(2, ...). OCCT: <paramref name="first"/> is a bool, giving a start of 0 or 1 (kept).</summary>
    public void SetZTrimmed(bool isTrimmed, bool first = false) => SetTrimmed(2, isTrimmed, first ? 1.0 : 0.0); // OCCT

    public bool IsInputZTrimmed => IsInputTrimmed(2);

    public double ZPeriodFirst() => PeriodFirst(2);

    /// <summary>Repeats the periodic shape in the direction (negative times: in the negative direction); returns it, frozen.</summary>
    public Shape RepeatShape(int directionID, int times)
    {
        InternalRepeatShape(directionID, times);
        return RepeatedShape();
    }

    public Shape XRepeat(int times) => RepeatShape(0, times);

    public Shape YRepeat(int times) => RepeatShape(1, times);

    public Shape ZRepeat(int times) => RepeatShape(2, times);

    /// <summary>The repeated shape (frozen).</summary>
    public Shape RepeatedShape()
    {
        _repeatedShape.TShape()?.FreezeGraph();
        return _repeatedShape;
    }

    /// <summary>The repeated shape, unfrozen, for ported code.</summary>
    internal Shape InternalRepeatedShape() => _repeatedShape;

    /// <summary>Clears the repetitions, keeping the shape periodic.</summary>
    public void ClearRepetitions()
    {
        _repeatPeriod[0] = _repeatPeriod[1] = _repeatPeriod[2] = 0.0;
        _repeatedShape.Nullify();
        _repeatedTwins.Clear();
        if (_history is not null)
        {
            _history.Clear();
            if (_splitHistory is not null)
            {
                _history.Merge(_splitHistory);
            }
        }
    }

    /// <summary>The periodic shape (frozen).</summary>
    public Shape Shape()
    {
        _shape.TShape()?.FreezeGraph();
        return _shape;
    }

    /// <summary>The periodic shape, unfrozen, for ported code.</summary>
    internal Shape InternalShape() => _shape;

    /// <summary>
    /// The sub-shapes identical to <paramref name="s"/> on the opposite sides (of the repeated shape once it is
    /// repeated). The result graphs are frozen first.
    /// </summary>
    public IReadOnlyList<Shape> GetTwins(in Shape s)
    {
        FreezeResults();
        return ImmutableArrays.Own<Shape>([.. InternalGetTwins(s)]);
    }

    /// <summary>OCCT's GetTwins, the algorithm's own list (empty if none), without freezing.</summary>
    internal NCollection_List<Shape> InternalGetTwins(in Shape s)
    {
        Dictionary<Shape, NCollection_List<Shape>> map =
            _repeatedTwins.Count == 0 ? _twins : _repeatedTwins;
        if (map.TryGetValue(s, out NCollection_List<Shape>? twins))
        {
            return twins;
        }
        // Kerf: a shape changed in place (in a working overlay) has the twins of what stands for it now, which natively
        // is the shape itself
        foreach (Shape copy in _inPlace.Modified(s))
        {
            if (map.TryGetValue(copy, out twins))
            {
                return twins;
            }
        }
        return new NCollection_List<Shape>();
    }

    /// <summary>The history of the operation (null before it is performed); the result graphs are frozen first.</summary>
    public History? History()
    {
        FreezeResults();
        return _history;
    }

    /// <summary>The history, without freezing.</summary>
    internal History? InternalHistory() => _history;

    /// <summary>Kerf: the shapes the algorithm changed in place, modified into what stands for them now.</summary>
    internal History InternalInPlace() => _inPlace;

    /// <summary>The input shape, as it stands after the in-place writes of the trim and the repetitions.</summary>
    internal Shape InternalInputShape() => _inputShape;

    /// <summary>
    /// Kerf: the input's sub-shapes that the last trim or repetition changed in place, modified into what stands for
    /// them now (natively the same shapes, changed).
    /// </summary>
    internal History InternalInputImages() => _inputImages;

    private void FreezeResults()
    {
        _shape.TShape()?.FreezeGraph();
        _repeatedShape.TShape()?.FreezeGraph();
    }

    /// <summary>Clears the algorithm from previous runs.</summary>
    public override void Clear()
    {
        base.Clear();
        _inPlace = new History();
        _inputImages = new History();
        _periodicityParams.Clear();
        _shape.Nullify();
        _splitHistory?.Clear();
        _history?.Clear();
        ClearRepetitions();
    }

    /// <summary>The ID of the periodic direction (0, 1, 2) of an integer.</summary>
    public static int ToDirectionID(int directionID) => Math.Abs(directionID % 3);

    /// <summary>Performs the operation.</summary>
    public void Perform()
    {
        _inputImages = new History();
        // Check the validity of input data
        CheckData();
        if (HasErrors)
        {
            return;
        }

        // Trim the shape to fit to the required period in
        // required periodic directions
        Trim();
        if (HasErrors)
        {
            return;
        }

        // Make the shape identical on the opposite sides in
        // required periodic directions
        MakeIdentical();
        if (HasErrors)
        {
            return;
        }
    }

    /// <summary>Checks the validity of input data.</summary>
    protected void CheckData()
    {
        if ((!IsXPeriodic || XPeriod() < global::Kerf.Engine.Foundation.Precision.Confusion())
            && (!IsYPeriodic || YPeriod() < global::Kerf.Engine.Foundation.Precision.Confusion())
            && (!IsZPeriodic || ZPeriod() < global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            // Add error informing the user that no periodicity is required
            // or no valid period is set.

            AddError(new AlertNoPeriodicityRequired());
            return;
        }
    }

    // Adds the shape <theWhat> to the shape <theWhere>
    private static void AddToShape(in Shape what, ref Shape theWhere)
    {
        if (theWhere.IsNull)
        {
            Compound c = new();
            new BrepBuilder().MakeCompound(ref c);
            theWhere = c;
        }
        new BrepBuilder().Add(theWhere, what);
    }

    // Adds the shape in the list <theLWhat> to the shape <theWhere>
    private static void AddToShape(NCollection_List<Shape> lWhat, ref Shape theWhere)
    {
        NCollection_List<Shape>.Iterator it = new(lWhat);
        for (; it.More(); it.Next())
        {
            AddToShape(it.Value(), ref theWhere);
        }
    }

    /// <summary>Makes the trim of the shape to fit to the periodic bounds.</summary>
    protected void Trim()
    {
        // Check if trim is required at all
        if (IsInputXTrimmed && IsInputYTrimmed && IsInputZTrimmed)
        {
            return;
        }

        // Compute bounding box for the shape to use it as a starting
        // volume for trimming. If required, the volume will be modified
        // to the requested trimming size in requested directions.
        Box box = new();
        Bounds.Add(_inputShape, ref box);
        // Enlarge box to avoid overlapping with the shape
        box.Enlarge(0.1 * Math.Sqrt(box.SquareExtent()));

        // Get Corner points of the bounding box
        Point3D pMin = box.CornerMin();
        Point3D pMax = box.CornerMax();

        // Update corner points according to the requested trim parameters
        for (int i = 0; i < 3; ++i)
        {
            if (IsInputTrimmed(i))
            {
                continue;
            }

            pMin = pMin.WithCoord(i + 1, PeriodFirst(i));
            pMax = pMax.WithCoord(i + 1, PeriodFirst(i) + Period(i));
        }

        // Build Trimming solid using corner points
        MakeBox mBox = new(pMin, pMax);
        Shape trimBox = mBox.InternalSolid();

        // Kerf: the Common below is destructive and natively writes into the input (tolerances, p-curves): it runs in
        // a working overlay, which takes the writes into the input's frozen parts (the caller's shape, or the
        // arguments' sub-shapes a connected shape shares)
        using var overlay = WorkingOverlay.Begin(_inputShape);

        // Perform trimming of the shape by solid
        Common common = new();
        // Set Object
        NCollection_List<Shape> obj = new();
        obj.Append(_inputShape);
        common.SetArguments(obj);
        // Set Tool
        NCollection_List<Shape> tool = new();
        tool.Append(trimBox);
        common.SetTools(tool);
        // Set the parallel processing mode
        common.SetRunParallel(_runParallel);
        // Build
        common.Build();
        if (common.HasErrors)
        {
            // Unable to trim the shape
            // Merge errors from Common operation
            _report.Merge(common.GetReport());
            // Add new error saving the shapes for analysis
            Shape ws = new();
            AddToShape(_inputShape, ref ws);
            AddToShape(trimBox, ref ws);
            AddError(new AlertUnableToTrim(ws));
            return;
        }
        // Get the trimmed shape
        // Kerf: the end of the overlay: the shapes and the history get what stands for each changed shape
        Shape trimmed = common.InternalShape();
        IndexedSet<Shape> trimmedShapes = new(ShapeHasher.Instance);
        Exploration.MapShapes(trimmed, trimmedShapes);
        IndexedSet<Shape> inputShapes = new(ShapeHasher.Instance);
        Exploration.MapShapes(_inputShape, inputShapes);
        _shape = overlay.Finish(trimmed, [_inputShape]);
        _inputShape = overlay.Final(_inputShape);
        _inputImages = overlay.Images(inputShapes);
        History images = overlay.Images(trimmedShapes);
        _inPlace.Merge(images);
        // Fill the History for the object only
        _splitHistory = new History();
        _splitHistory.Merge(obj, new MakeShapeHistory(common));
        MergeInPlace(_splitHistory, images, trimmedShapes);
    }

    /// <summary>Makes the shape look the same on the opposite sides in the required periodic directions.</summary>
    protected void MakeIdentical()
    {
        if (_shape.IsNull)
        {
            _shape = _inputShape;
        }

        _splitHistory ??= new History();

        // Split the negative side of the shape with the geometry
        // located on the positive side
        SplitNegative();
        if (HasErrors)
        {
            return;
        }

        // Split the positive side of the shape with the geometry
        // located on the negative side.
        // Make sure that the opposite sides have identical geometries.
        // Make associations between identical opposite shapes.
        SplitPositive();

        _history = new History();
        _history.Merge(_splitHistory);
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
            Transform3D negTrsf = new Transform3D().WithTranslationPart(new Vector3D(MY_DIRECTIONS[i].Reversed()) * Period(i));
            Transform negT = new(_shape, negTrsf, false);

            // Split the negative side of the shape.
            NCollection_List<Shape> tools = new();
            tools.Append(negT.InternalShape());
            SplitShape(tools, _splitHistory);
        }
    }

    // Associates the shape <theS> with the shape <theTwin> in the map.
    private static void AddTwin(in Shape s, in Shape twin,
                                Dictionary<Shape, NCollection_List<Shape>> map)
    {
        if (!map.TryGetValue(s, out NCollection_List<Shape>? twins))
        {
            map.SetValueRef(s, new NCollection_List<Shape>()).Append(twin);
            return;
        }

        // Check if the twin shape is not yet present in the list
        NCollection_List<Shape>.Iterator itLT = new(twins);
        for (; itLT.More(); itLT.Next())
        {
            if (twin.IsSame(itLT.Value()))
            {
                break;
            }
        }

        if (!itLT.More())
        {
            twins.Append(twin);
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
        IndexedSet<Shape> subShapesMap = new(ShapeHasher.Instance);
        Exploration.MapShapes(_shape, subShapesMap);
        int nbS = subShapesMap.Count;

        // Translate the shape to the positive periodic directions to make the
        // shapes look identical on the opposite sides.
        NCollection_List<Shape> tools = new();

        // Remember the history of shapes translation
        IndexedMap<Shape, NCollection_List<Shape>> translationHistMap = new(ShapeHasher.Instance);

        // Make translations for all periodic directions
        for (int i = 0; i < 3; ++i)
        {
            if (!IsPeriodic(i))
            {
                continue;
            }

            // Translate the shape to the positive side
            Transform3D posTrsf = new Transform3D().WithTranslationPart(new Vector3D(MY_DIRECTIONS[i]) * Period(i));
            Transform translator = new(_shape, posTrsf, false);
            tools.Append(translator.InternalShape());

            // Fill the translation history map
            for (int j = 0; j < nbS; ++j)
            {
                Shape s = subShapesMap[j];
                if (global::Kerf.Engine.Topology.History.IsSupportedType(s))
                {
                    NCollection_List<Shape> sm = translator.InternalModified(s);
                    int index = translationHistMap.IndexOf(s) + 1;
                    if (index == 0)
                    {
                        index = translationHistMap.Add(s, new NCollection_List<Shape>()) + 1;
                    }
                    translationHistMap.ValueAt(index - 1).Append(sm.First());
                }
            }
        }

        // Keep the split shape history and history of tools modifications
        // during the split for making association of the opposite identical shapes
        History splitShapeHist = new(), splitToolsHist = new();
        // Split the positive side of the shape
        SplitShape(tools, splitShapeHist, splitToolsHist);
        if (HasErrors)
        {
            return;
        }

        _splitHistory!.Merge(splitShapeHist);

        // Make associations between identical opposite sub-shapes
        int nbSH = translationHistMap.Count;
        for (int i = 0; i < nbSH; ++i)
        {
            Shape pS = translationHistMap.KeyAt(i);
            IReadOnlyList<Shape> sIm = splitShapeHist.Modified(pS);
            if (sIm.Count == 1)
            {
                pS = sIm[0];
            }
            else if (sIm.Count > 1)
            {
                continue;
            }

            NCollection_List<Shape> lTranslated = translationHistMap.ValueAt(i);

            NCollection_List<Shape>.Iterator itLT = new(lTranslated);
            for (; itLT.More(); itLT.Next())
            {
                Shape t = itLT.Value();
                // Get shapes modifications during the split
                IReadOnlyList<Shape> tSplits = splitToolsHist.Modified(t);

                // Associate the shapes to each other
                foreach (Shape sp in tSplits)
                {
                    AddTwin(pS, sp, _twins);
                    AddTwin(sp, pS, _twins);
                }
            }
        }
    }

    /// <summary>Splits the shape by the given tools.</summary>
    protected void SplitShape(NCollection_List<Shape> tools, History? splitShapeHistory = null,
                              History? splitToolsHistory = null)
    {
        // Make sure that the geometry from the tools will be copied to the split
        // shape. For that, the tool shapes should be given to the Boolean Operations
        // algorithm before the shape itself. This will make all coinciding parts
        // use the geometry of the first argument.

        // Intersection tool for passing ordered arguments
        PaveFiller intersector = new();
        intersector.SetArguments(tools);
        // Add the shape
        intersector.AddArgument(_shape);
        // Use gluing to speed-up intersections
        intersector.SetGlue(global::Kerf.Booleans.GlueMode.Shift);
        // Use safe input mode, to avoid reusing geometry of the shape
        intersector.SetNonDestructive(true);
        // Set parallel processing mode
        intersector.SetRunParallel(_runParallel);
        // Perform Intersection of the arguments
        intersector.Perform();
        // Check for the errors
        if (intersector.HasErrors)
        {
            // Unable to split the shape on opposite sides
            // Copy the intersection errors
            _report.Merge(intersector.GetReport());
            // Add new error saving the shapes for analysis
            Shape ws = new();
            AddToShape(tools, ref ws);
            AddToShape(_shape, ref ws);
            AddError(new AlertUnableToMakeIdentical(ws));
            return;
        }

        // Perform the splitting of the shape with the precomputed intersection results
        SplitterOperation splitter = new(intersector);
        // Set Object
        NCollection_List<Shape> obj = new();
        obj.Append(_shape);
        splitter.SetArguments(obj);
        // Set Tools
        splitter.SetTools(tools);
        // Use Gluing
        splitter.SetGlue(global::Kerf.Booleans.GlueMode.Shift);
        // Set parallel processing mode
        splitter.SetRunParallel(_runParallel);
        // Perform splitting
        splitter.Build();
        // Check for the errors
        if (splitter.HasErrors)
        {
            // Unable to split the shape on opposite sides
            // Copy the splitter errors
            _report.Merge(splitter.GetReport());
            // Add new error saving the shape for analysis
            Shape ws = new();
            AddToShape(tools, ref ws);
            AddToShape(_shape, ref ws);
            AddError(new AlertUnableToMakeIdentical(ws));
            return;
        }
        // Get the split shape
        _shape = splitter.InternalShape();
        // Remember the split history
        splitShapeHistory?.Merge(obj, new MakeShapeHistory(splitter));
        splitToolsHistory?.Merge(tools, new MakeShapeHistory(splitter));
    }

    /// <summary>Repeats the shape in the required periodic direction; returns the repeated shape, unfrozen.</summary>
    internal Shape InternalRepeatShape(int dir, int times)
    {
        _inputImages = new History();
        if (_repeatedShape.IsNull)
        {
            _repeatedShape = _shape;
        }

        if (!IsPeriodic(dir))
        {
            return _repeatedShape;
        }

        if (times == 0)
        {
            return _repeatedShape;
        }

        // Get the shape's period in the required direction
        int id = ToDirectionID(dir);
        if (_repeatPeriod[id] < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            _repeatPeriod[id] = Period(id);
        }
        double period = _repeatPeriod[id];

        // Coefficient to define in which direction the repetition will be performed:
        // theTimes is positive - in positive direction;
        // theTimes is negative - in negative direction.
        int iDir = times > 0 ? 1 : -1;

        // Kerf: the gluing below is destructive and writes into the shape to repeat (tolerances, p-curves), which
        // natively changes the periodic shape and the input's sub-shapes it shares. It runs in a working overlay,
        // which takes the writes into frozen sub-shapes (the input's, or the results already handed out).
        using var overlay = WorkingOverlay.Begin(_repeatedShape);

        // Create the translation history - all translated shapes will be
        // created as Generated from the shape.
        History translationHistory = new();
        IndexedSet<Shape> subShapesMap = new(ShapeHasher.Instance);
        Exploration.MapShapes(_repeatedShape, subShapesMap);
        int nbS = subShapesMap.Count;

        // Add shapes for gluing
        NCollection_List<Shape> shapes = new();
        // Add the shape itself
        shapes.Append(_repeatedShape);
        for (int i = 0; i < nbS; ++i)
        {
            Shape s = subShapesMap[i];
            if (global::Kerf.Engine.Topology.History.IsSupportedType(s))
            {
                translationHistory.AddGenerated(s, s);
            }
        }

        // Create translated copies of the shape
        for (int i = 1; i <= Math.Abs(times); ++i)
        {
            Transform3D trsf = new Transform3D().WithTranslationPart(new Vector3D(MY_DIRECTIONS[id]) * (iDir * i * period));
            Transform translator = new(_repeatedShape, trsf, false);
            shapes.Append(translator.InternalShape());

            // Fill the translation history
            for (int j = 0; j < nbS; ++j)
            {
                Shape s = subShapesMap[j];
                if (global::Kerf.Engine.Topology.History.IsSupportedType(s))
                {
                    NCollection_List<Shape> lt = translator.InternalModified(s);
                    translationHistory.AddGenerated(s, lt.First());
                }
            }
        }

        // Update the history with the translation History
        _history!.Merge(translationHistory);

        // Glue the translated shapes all together
        BooleanBuilder gluer = new();
        gluer.SetArguments(shapes);
        // Avoid intersections of the sub-shapes
        gluer.SetGlue(global::Kerf.Booleans.GlueMode.Full);
        // Set parallel processing mode
        gluer.SetRunParallel(_runParallel);
        // Perform gluing
        gluer.Perform();
        if (gluer.HasErrors)
        {
            // Repetition in this direction is not possible
            // Add warning saving the shapes for analysis
            Shape ws = new();
            AddToShape(shapes, ref ws);
            AddWarning(new AlertUnableToRepeat(ws));
            return _repeatedShape;
        }
        // Get glued shape
        Shape glued = gluer.InternalShape();

        // Update repetition period for the next repetitions
        _repeatPeriod[id] += Math.Abs(times) * _repeatPeriod[id];

        // Update history with the Gluing history
        History gluingHistory = new(shapes, new BuilderHistory(gluer));
        _history.Merge(gluingHistory);

        // Update the map of twins after repetition
        UpdateTwins(translationHistory, gluingHistory);

        // Kerf: the end of the overlay: the shapes, histories and twins get what stands for each changed shape
        IndexedSet<Shape> gluedShapes = new(ShapeHasher.Instance), periodicShapes = new(ShapeHasher.Instance);
        Exploration.MapShapes(glued, gluedShapes);
        Exploration.MapShapes(_shape, periodicShapes);
        IndexedSet<Shape> inputShapes = new(ShapeHasher.Instance);
        Exploration.MapShapes(_inputShape, inputShapes);
        _repeatedShape = overlay.Finish(glued, [_shape, _inputShape]);
        _shape = overlay.Final(_shape);
        _inputShape = overlay.Final(_inputShape);
        _inputImages = overlay.Images(inputShapes);
        History images = overlay.Images(gluedShapes);
        MergeInPlace(_history, images, gluedShapes);
        if (_splitHistory is not null)
        {
            MergeInPlace(_splitHistory, overlay.Images(periodicShapes), periodicShapes);
        }
        _inPlace.Merge(images);
        _twins = FinalTwins(_twins, overlay);
        _repeatedTwins = FinalTwins(_repeatedTwins, overlay);

        return _repeatedShape;
    }

    // Kerf: merges the in-place images of the shapes (a working overlay's Images) into the history, so its results are
    // what stands for them now; a shape that is also an initial shape of the history with only generated shapes (the
    // translations' AddGenerated(s, s)) is modified into what stands for it, as the shape itself is natively
    private static void MergeInPlace(History history, History images, IndexedSet<Shape> shapes)
    {
        history.Merge(images);
        foreach (Shape s in shapes)
        {
            IReadOnlyList<Shape> now = images.Modified(s);
            if (now.Count != 0 && history.Modified(s).Count == 0 && history.Generated(s).Count != 0
                && !history.IsRemoved(s))
            {
                history.AddModified(s, now[0]);
            }
        }
    }

    // Kerf: the twins with what stands for each shape after the overlay's end
    private static Dictionary<Shape, NCollection_List<Shape>> FinalTwins(
        Dictionary<Shape, NCollection_List<Shape>> map, WorkingOverlay overlay)
    {
        if (overlay.WorkingCount == 0)
        {
            return map;
        }
        Dictionary<Shape, NCollection_List<Shape>> aFinal = new(ShapeHasher.Instance);
        foreach ((Shape key, NCollection_List<Shape> list) in map)
        {
            NCollection_List<Shape> aNew = new();
            foreach (Shape aS in list)
            {
                aNew.Append(overlay.Final(aS));
            }
            aFinal[overlay.Final(key)] = aNew;
        }
        return aFinal;
    }

    /// <summary>Updates the map of twins after repetition.</summary>
    protected void UpdateTwins(History translationHistory, History gluingHistory)
    {
        if (_twins.Count == 0)
        {
            return;
        }

        if (_repeatedTwins.Count == 0)
        {
            _repeatedTwins = CopyTwins(_twins);
        }

        // New twins
        Dictionary<Shape, NCollection_List<Shape>> newTwinsMap = new(ShapeHasher.Instance);

        // Fence map to avoid repeated fill for the twins
        HashSet<Shape> mTwinsDone = new(ShapeHasher.Instance);

        // Update the map of twins with the new repeated shapes
        // (native: address order; the port: insertion order, occt-issues.md)
        foreach ((Shape s, NCollection_List<Shape> lTwins) in _repeatedTwins)
        {
            mTwinsDone.Add(s);

            // Check if the twins have not been already processed
            NCollection_List<Shape>.Iterator itLT = new(lTwins);
            for (; itLT.More(); itLT.Next())
            {
                if (mTwinsDone.Contains(itLT.Value()))
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
            IndexedSet<Shape> newGroup = new(ShapeHasher.Instance);
            itLT.Initialize(lTwins);

            for (bool bShape = true; itLT.More();)
            {
                Shape twin = bShape ? s : itLT.Value();
                foreach (Shape g in translationHistory.Generated(twin))
                {
                    IReadOnlyList<Shape> lm = gluingHistory.Modified(g);
                    if (lm.Count == 0)
                    {
                        newGroup.Add(g);
                    }
                    else
                    {
                        foreach (Shape m in lm)
                        {
                            newGroup.Add(m);
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
            int nbTwins = newGroup.Count;
            for (int i = 1; i <= nbTwins; ++i)
            {
                ref NCollection_List<Shape> pTwins = ref newTwinsMap.SetValueRef(newGroup[i - 1], new NCollection_List<Shape>());
                for (int j = 1; j <= nbTwins; ++j)
                {
                    if (i != j)
                    {
                        pTwins.Append(newGroup[j - 1]);
                    }
                }
            }
        }

        _repeatedTwins = newTwinsMap;
    }

    // OCCT's map assignment copies the lists
    private static Dictionary<Shape, NCollection_List<Shape>> CopyTwins(
        Dictionary<Shape, NCollection_List<Shape>> map)
    {
        Dictionary<Shape, NCollection_List<Shape>> copy = new(ShapeHasher.Instance);
        foreach ((Shape key, NCollection_List<Shape> list) in map)
        {
            copy[key] = new NCollection_List<Shape>(list);
        }
        return copy;
    }

    /// <summary>
    /// The history methods of a BRepBuilderAPI maker without publishing (OCCT's template history constructor and
    /// Merge take the algorithm; the port's public Modified/Generated would freeze the maker's result).
    /// </summary>
    internal sealed class MakeShapeHistory(MakeShape theAlgo) : global::Kerf.Engine.Topology.History.IAlgorithm
    {
        public bool IsDeleted(in Shape s) => theAlgo.IsDeleted(s);

        public IEnumerable<Shape> Modified(in Shape s) => [.. theAlgo.InternalModified(s)];

        public IEnumerable<Shape> Generated(in Shape s) => [.. theAlgo.InternalGenerated(s)];
    }

    /// <summary>The history methods of a BOPAlgo builder (BOPAlgo_BuilderShape's Modified, Generated, IsDeleted).</summary>
    internal sealed class BuilderHistory(BuilderShape theAlgo) : global::Kerf.Engine.Topology.History.IAlgorithm
    {
        public bool IsDeleted(in Shape s) => theAlgo.IsDeleted(s);

        public IEnumerable<Shape> Modified(in Shape s) => theAlgo.Modified(s);

        public IEnumerable<Shape> Generated(in Shape s) => theAlgo.Generated(s);
    }
}
