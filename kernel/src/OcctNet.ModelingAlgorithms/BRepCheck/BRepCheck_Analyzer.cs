// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Analyzer.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Analyzer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepCheck_Status;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Checks a shape and all its sub-shapes: each vertex, edge, wire, face, shell and solid is checked on its own and
/// in the context of the shapes that contain it. <see cref="IsValid()"/> is true if no fault was found;
/// <see cref="Result"/> gives the statuses of each sub-shape.
/// </summary>
/// <remarks>
/// <para>
/// With <c>GeomControls</c> false only the topology is checked. The geometric controls are: for an edge,
/// InvalidCurveOnClosedSurface, InvalidCurveOnSurface and InvalidSameParameterFlag; for a face, UnorientableShape and
/// IntersectingWires; for a wire, SelfIntersectingWire.
/// </para>
/// <para>
/// The parallel mode checks the sub-shapes on several threads (in chunks of the map of sub-shapes, as OCCT does) and
/// gives the same statuses as the serial mode. The analyzer only reads the shape; the shapes it builds to classify
/// wires and shells are its own and never handed out.
/// </para>
/// <para>
/// OCCT catches <c>Standard_Failure</c> around each check, with OCC_CATCH_SIGNALS turning access violations into
/// failures, and records <see cref="BRepCheck_CheckFail"/>. The port catches <see cref="Standard_Failure"/> and the
/// .NET exceptions that stand for those signals (null references, invalid casts of handles, indices out of range,
/// integer division by zero).
/// </para>
/// </remarks>
public class BRepCheck_Analyzer
{
    private TopoDS_Shape myShape;

    // OCCT declares this map without a hasher: the default one compares shapes with operator== (IsEqual), so a shape
    // met with two orientations (a seam edge, a vertex shared by edges) has one result per orientation, and the face
    // check (which skips shapes already seen with IsSame) puts the context on the first one only
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, BRepCheck_Result?> myMap =
        new(EqualityComparer<TopoDS_Shape>.Default);

    private bool myIsParallel;
    private bool myIsExact;

    /// <summary>
    /// Checks <paramref name="S"/>; with <paramref name="GeomControls"/> false only topological information is
    /// checked. <paramref name="theIsParallel"/> checks the sub-shapes on several threads;
    /// <paramref name="theIsExact"/> measures edges on faces exactly (see <see cref="SetExactMethod"/>).
    /// </summary>
    public BRepCheck_Analyzer(in TopoDS_Shape S,
                              bool GeomControls = true,
                              bool theIsParallel = false,
                              bool theIsExact = false)
    {
        myShape = new TopoDS_Shape();
        myIsParallel = theIsParallel;
        myIsExact = theIsExact;
        Init(S, GeomControls);
    }

    /// <summary>
    /// Checks <paramref name="theShape"/> again, from scratch; with <paramref name="B"/> false only topological
    /// information is checked.
    /// </summary>
    public void Init(in TopoDS_Shape theShape, bool B = true)
    {
        if (theShape.IsNull())
        {
            throw new Standard_NullObject("BRepCheck_Analyzer::Init() - NULL shape");
        }

        myShape = theShape;
        myMap.Clear();
        Put(theShape, B);
        Perform();
    }

    /// <summary>
    /// Sets the method that measures the distance between an edge's curves (used by the next <see cref="Init"/>): in
    /// a finite number of points (false, the default: faster, possibly not exact) or exactly with
    /// BRepLib_CheckCurveOnSurface (true: slower; only for same-parameter edges).
    /// </summary>
    public void SetExactMethod(bool theIsExact) => myIsExact = theIsExact;

    /// <summary>Whether the exact method is selected.</summary>
    public bool IsExactMethod() => myIsExact;

    /// <summary>Sets the parallel mode (used by the next <see cref="Init"/>).</summary>
    public void SetParallel(bool theIsParallel) => myIsParallel = theIsParallel;

    /// <summary>Whether the parallel mode is set.</summary>
    public bool IsParallel() => myIsParallel;

    /// <summary>
    /// True if no fault was found on <paramref name="S"/>, a sub-shape of the checked shape, or on any of its
    /// sub-shapes, on its own or in its context in <paramref name="S"/>. Only the first status of each list is read,
    /// as in OCCT.
    /// </summary>
    public bool IsValid(in TopoDS_Shape S)
    {
        if (myMap.FindFromKey(S) is { } aResult)
        {
            var itl = new NCollection_List<BRepCheck_Status>.Iterator();
            itl.Initialize(aResult.Status());
            if (itl.Value() != BRepCheck_NoError)
            {
                // a voir
                return false;
            }
        }

        for (var theIterator = new TopoDS_Iterator(S); theIterator.More(); theIterator.Next())
        {
            if (!IsValid(theIterator.Value()))
            {
                return false;
            }
        }

        switch (S.ShapeType())
        {
            case TopAbs_EDGE:
            {
                return ValidSub(S, TopAbs_VERTEX);
            }
            case TopAbs_FACE:
            {
                bool valid = ValidSub(S, TopAbs_WIRE);
                valid = valid && ValidSub(S, TopAbs_EDGE);
                valid = valid && ValidSub(S, TopAbs_VERTEX);
                return valid;
            }
            case TopAbs_SHELL:
                break;
            case TopAbs_SOLID:
                return ValidSub(S, TopAbs_SHELL);
            default:
                break;
        }

        return true;
    }

    /// <summary>
    /// True if no fault was found on the shape or any of its sub-shapes: the topology is correct and the edges'
    /// parameterisation is correct (see OCCT's documentation of BRepCheck_Analyzer::IsValid).
    /// </summary>
    public bool IsValid() => IsValid(myShape);

    /// <summary>
    /// The check of <paramref name="theSubS"/>, a sub-shape of the checked shape (null for compounds and
    /// compsolids). Throws <see cref="Standard_NoSuchObject"/> for a shape that isn't a sub-shape.
    /// </summary>
    public BRepCheck_Result? Result(in TopoDS_Shape theSubS) => myMap.FindFromKey(theSubS);

    // The exceptions OCCT's catch (Standard_Failure) handles under OCC_CATCH_SIGNALS
    private static bool IsCaught(Exception theException)
        => theException is Standard_Failure
               or NullReferenceException
               or InvalidCastException
               or IndexOutOfRangeException
               or DivideByZeroException;

    private void Put(in TopoDS_Shape theShape, bool B)
    {
        if (myMap.Contains(theShape))
        {
            return;
        }

        BRepCheck_Result? HR = null;
        switch (theShape.ShapeType())
        {
            case TopAbs_VERTEX:
                HR = new BRepCheck_Vertex(TopoDS.Vertex(theShape));
                break;
            case TopAbs_EDGE:
            {
                var aResEdge = new BRepCheck_Edge(TopoDS.Edge(theShape));
                aResEdge.GeometricControls(B);
                aResEdge.SetExactMethod(myIsExact);
                HR = aResEdge;
                break;
            }
            case TopAbs_WIRE:
            {
                var aResWire = new BRepCheck_Wire(TopoDS.Wire(theShape));
                aResWire.GeometricControls(B);
                HR = aResWire;
                break;
            }
            case TopAbs_FACE:
            {
                var aResFace = new BRepCheck_Face(TopoDS.Face(theShape));
                aResFace.GeometricControls(B);
                HR = aResFace;
                break;
            }
            case TopAbs_SHELL:
                HR = new BRepCheck_Shell(TopoDS.Shell(theShape));
                break;
            case TopAbs_SOLID:
                HR = new BRepCheck_Solid(TopoDS.Solid(theShape));
                break;
            case TopAbs_COMPSOLID:
            case TopAbs_COMPOUND:
                break;
            default:
                break;
        }

        HR?.SetParallel(myIsParallel);

        myMap.Add(theShape, HR);

        for (var theIterator = new TopoDS_Iterator(theShape); theIterator.More(); theIterator.Next())
        {
            Put(theIterator.Value(), B); // performs minimum on each shape
        }
    }

    private void Perform()
    {
        int aMapSize = myMap.Extent();
        const int aMinTaskSize = 10;
        // OSD_ThreadPool::DefaultPool()->NbThreads() is the number of logical processors
        int aNbThreads = OSD_Parallel.NbLogicalProcessors();
        int aNbTasks = aNbThreads * 10;
        int aTaskSize = (int)Math.Ceiling((double)aMapSize / aNbTasks);
        if (aTaskSize < aMinTaskSize)
        {
            aTaskSize = aMinTaskSize;
            aNbTasks = (int)Math.Ceiling((double)aMapSize / aTaskSize);
        }

        var aArrayOfArray = new NCollection_Array1<TopoDS_Shape[]>(0, aNbTasks - 1);
        // OCCT's arrays are default-constructed empty: chunks past the last shape stay empty
        for (int anI = 0; anI < aNbTasks; ++anI)
        {
            aArrayOfArray[anI] = [];
        }

        for (int anI = 1; anI <= aMapSize; ++anI)
        {
            int aVectIndex = (anI - 1) / aTaskSize;
            int aShapeIndex = (anI - 1) % aTaskSize;
            if (aShapeIndex == 0)
            {
                int aVectorSize = aTaskSize;
                int aTailSize = aMapSize - aVectIndex * aTaskSize;
                if (aTailSize < aTaskSize)
                {
                    aVectorSize = aTailSize;
                }

                aArrayOfArray[aVectIndex] = new TopoDS_Shape[aVectorSize];
            }

            aArrayOfArray[aVectIndex][aShapeIndex] = myMap.FindKey(anI);
        }

        var aParallelAnalyzer = new BRepCheck_ParallelAnalyzer(aArrayOfArray, myMap);
        OSD_Parallel.For(0, aArrayOfArray.Length(), aParallelAnalyzer.Perform, !myIsParallel);
    }

    private bool ValidSub(in TopoDS_Shape S, TopAbs_ShapeEnum SubType)
    {
        var itl = new NCollection_List<BRepCheck_Status>.Iterator();
        var exp = new TopExp_Explorer();
        for (exp.Init(S, SubType); exp.More(); exp.Next())
        {
            BRepCheck_Result RV = myMap.FindFromKey(exp.Current())!;
            for (RV.InitContextIterator(); RV.MoreShapeInContext(); RV.NextShapeInContext())
            {
                if (RV.ContextualShape().IsSame(S))
                {
                    break;
                }
            }

            if (!RV.MoreShapeInContext())
            {
                break;
            }

            for (itl.Initialize(RV.StatusOnShape()); itl.More(); itl.Next())
            {
                if (itl.Value() != BRepCheck_NoError)
                {
                    return false;
                }
            }
        }

        return true;
    }

    /// <summary>OCCT's functor for multi-threaded execution: checks one chunk of the sub-shapes in their contexts.</summary>
    private sealed class BRepCheck_ParallelAnalyzer
    {
        private readonly NCollection_Array1<TopoDS_Shape[]> myArray;
        private readonly NCollection_IndexedDataMap<TopoDS_Shape, BRepCheck_Result?> myMap;

        public BRepCheck_ParallelAnalyzer(NCollection_Array1<TopoDS_Shape[]> theArray,
                                          NCollection_IndexedDataMap<TopoDS_Shape, BRepCheck_Result?> theMap)
        {
            myArray = theArray;
            myMap = theMap;
        }

        /// <summary>A status list of a result, read under the result's lock in parallel mode.</summary>
        private static bool HasStatusOnShape(BRepCheck_Result theResult, in TopoDS_Shape theShape,
                                             Func<BRepCheck_Status, bool> theStop)
        {
            bool aLocked = theResult.IsParallel();
            if (aLocked)
            {
                theResult.myMutex.Enter();
            }

            try
            {
                if (theResult.IsStatusOnShape(theShape))
                {
                    var itl = new NCollection_List<BRepCheck_Status>.Iterator(theResult.StatusOnShape(theShape));
                    for (; itl.More(); itl.Next())
                    {
                        if (theStop(itl.Value()))
                        {
                            return true;
                        }
                    }
                }

                return false;
            }
            finally
            {
                if (aLocked)
                {
                    theResult.myMutex.Exit();
                }
            }
        }

        public void Perform(int theVectorIndex)
        {
            var exp = new TopExp_Explorer();
            TopoDS_Shape[] aVector = myArray[theVectorIndex];
            for (int aShapeIter = 0; aShapeIter < aVector.Length; ++aShapeIter)
            {
                TopoDS_Shape aShape = aVector[aShapeIter];
                TopAbs_ShapeEnum aType = aShape.ShapeType();
                BRepCheck_Result? aResult = myMap.FindFromKey(aShape);
                switch (aType)
                {
                    case TopAbs_VERTEX:
                    {
                        // There is no need to check anything.
                        break;
                    }
                    case TopAbs_EDGE:
                    {
                        try
                        {
                            var aResEdge = (BRepCheck_Edge)aResult!;
                            BRepCheck_Status ste = aResEdge.CheckPolygonOnTriangulation(TopoDS.Edge(aShape));
                            if (ste != BRepCheck_NoError)
                            {
                                aResEdge.SetStatus(ste);
                            }
                        }
                        catch (Exception anException) when (IsCaught(anException))
                        {
                            aResult?.SetFailStatus(aShape);
                        }

                        var MapS = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                        for (exp.Init(aShape, TopAbs_VERTEX); exp.More(); exp.Next())
                        {
                            TopoDS_Shape aVertex = exp.Current();
                            BRepCheck_Result? aResOfVertex = myMap.FindFromKey(aVertex);
                            try
                            {
                                if (MapS.Add(aVertex))
                                {
                                    aResOfVertex!.InContext(aShape);
                                }
                            }
                            catch (Exception anException) when (IsCaught(anException))
                            {
                                aResult?.SetFailStatus(aShape);

                                if (aResOfVertex is not null)
                                {
                                    aResOfVertex.SetFailStatus(aVertex);
                                    aResOfVertex.SetFailStatus(aShape);
                                }
                            }
                        }

                        break;
                    }
                    case TopAbs_WIRE:
                    {
                        break;
                    }
                    case TopAbs_FACE:
                    {
                        var MapS = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                        for (exp.Init(aShape, TopAbs_VERTEX); exp.More(); exp.Next())
                        {
                            BRepCheck_Result? aFaceVertexRes = myMap.FindFromKey(exp.Current());
                            try
                            {
                                if (MapS.Add(exp.Current()))
                                {
                                    aFaceVertexRes!.InContext(aShape);
                                }
                            }
                            catch (Exception anException) when (IsCaught(anException))
                            {
                                aResult?.SetFailStatus(aShape);

                                if (aFaceVertexRes is not null)
                                {
                                    aFaceVertexRes.SetFailStatus(exp.Current());
                                    aFaceVertexRes.SetFailStatus(aShape);
                                }
                            }
                        }

                        bool performwire = true;
                        bool isInvalidTolerance = false;
                        MapS.Clear();
                        for (exp.Init(aShape, TopAbs_EDGE); exp.More(); exp.Next())
                        {
                            BRepCheck_Result? aFaceEdgeRes = myMap.FindFromKey(exp.Current());
                            try
                            {
                                if (MapS.Add(exp.Current()))
                                {
                                    aFaceEdgeRes!.InContext(aShape);

                                    if (performwire)
                                    {
                                        if (HasStatusOnShape(aFaceEdgeRes,
                                                             aShape,
                                                             ste => ste == BRepCheck_NoCurveOnSurface
                                                                    || ste == BRepCheck_InvalidCurveOnSurface
                                                                    || ste == BRepCheck_InvalidRange
                                                                    || ste == BRepCheck_InvalidCurveOnClosedSurface))
                                        {
                                            performwire = false;
                                        }
                                    }
                                }
                            }
                            catch (Exception anException) when (IsCaught(anException))
                            {
                                aResult?.SetFailStatus(aShape);

                                if (aFaceEdgeRes is not null)
                                {
                                    aFaceEdgeRes.SetFailStatus(exp.Current());
                                    aFaceEdgeRes.SetFailStatus(aShape);
                                }
                            }
                        }

                        bool orientofwires = performwire;
                        for (exp.Init(aShape, TopAbs_WIRE); exp.More(); exp.Next())
                        {
                            BRepCheck_Result? aFaceWireRes = myMap.FindFromKey(exp.Current());
                            try
                            {
                                aFaceWireRes!.InContext(aShape);

                                if (orientofwires)
                                {
                                    if (HasStatusOnShape(aFaceWireRes, aShape, ste => ste != BRepCheck_NoError))
                                    {
                                        orientofwires = false;
                                    }
                                }
                            }
                            catch (Exception anException) when (IsCaught(anException))
                            {
                                aResult?.SetFailStatus(aShape);

                                if (aFaceWireRes is not null)
                                {
                                    aFaceWireRes.SetFailStatus(exp.Current());
                                    aFaceWireRes.SetFailStatus(aShape);
                                }
                            }
                        }

                        try
                        {
                            var aFaceRes = (BRepCheck_Face)aResult!;
                            if (isInvalidTolerance)
                            {
                                aFaceRes.SetStatus(BRepCheck_InvalidToleranceValue);
                            }
                            else if (performwire)
                            {
                                if (orientofwires)
                                {
                                    aFaceRes.OrientationOfWires(true); // on enregistre
                                }
                                else
                                {
                                    aFaceRes.SetUnorientable();
                                }
                            }
                            else
                            {
                                aFaceRes.SetUnorientable();
                            }
                        }
                        catch (Exception anException) when (IsCaught(anException))
                        {
                            aResult?.SetFailStatus(aShape);

                            for (exp.Init(aShape, TopAbs_WIRE); exp.More(); exp.Next())
                            {
                                BRepCheck_Result? aFaceCatchRes = myMap.FindFromKey(exp.Current());
                                if (aFaceCatchRes is not null)
                                {
                                    aFaceCatchRes.SetFailStatus(exp.Current());
                                    aFaceCatchRes.SetFailStatus(aShape);
                                    aResult!.SetFailStatus(exp.Current());
                                }
                            }
                        }

                        break;
                    }
                    case TopAbs_SHELL:
                    {
                        break;
                    }
                    case TopAbs_SOLID:
                    {
                        exp.Init(aShape, TopAbs_SHELL);
                        for (; exp.More(); exp.Next())
                        {
                            TopoDS_Shape aShell = exp.Current();
                            BRepCheck_Result? aSolidRes = myMap.FindFromKey(aShell);
                            try
                            {
                                aSolidRes!.InContext(aShape);
                            }
                            catch (Exception anException) when (IsCaught(anException))
                            {
                                aResult?.SetFailStatus(aShape);

                                if (aSolidRes is not null)
                                {
                                    aSolidRes.SetFailStatus(exp.Current());
                                    aSolidRes.SetFailStatus(aShape);
                                }
                            }
                        }

                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
        }
    }
}
