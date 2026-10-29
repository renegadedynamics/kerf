// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Result.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Result.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepCheck_Status;

namespace Occt;

/// <summary>
/// The result of checking one shape: its own statuses, and its statuses in the context of the shapes that contain it
/// (<see cref="InContext"/>). The base of BRepCheck_Vertex, Edge, Wire, Face, Shell and Solid.
/// </summary>
/// <remarks>
/// A working object, as in OCCT: the status lists it returns are its own (OCCT's
/// <c>BRepCheck_ListOfStatus</c>, <c>NCollection_List&lt;BRepCheck_Status&gt;</c>). The context iterator visits the
/// contexts in the order they were checked (OCCT's map iterates in hash order, decision 18).
/// </remarks>
public abstract class BRepCheck_Result : Standard_Transient
{
    protected TopoDS_Shape myShape;
    protected bool myMin;
    protected bool myBlind;
    protected bool myIsParallel;

    /// <summary>The status lists of the shape (keyed by itself) and of its contexts.</summary>
    protected readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<BRepCheck_Status>> myMap =
        new(TopTools_ShapeMapHasher.Instance);

    /// <summary>OCCT's <c>std::mutex myMutex</c>, taken when <see cref="IsParallel"/>.</summary>
    /// <remarks>Internal as well: OCCT makes BRepCheck_ParallelAnalyzer a friend.</remarks>
    protected internal readonly Lock myMutex = new();

    // OCCT's map iterator: a snapshot of the map taken by InitContextIterator
    private KeyValuePair<TopoDS_Shape, NCollection_List<BRepCheck_Status>>[] myIterItems = [];
    private int myIterIndex;

    protected BRepCheck_Result()
    {
        myShape = new TopoDS_Shape();
        myMin = false;
        myBlind = false;
    }

    /// <summary>Starts a check of <paramref name="S"/>: clears the statuses and runs <see cref="Minimum"/>.</summary>
    public void Init(in TopoDS_Shape S)
    {
        myShape = S;
        myMin = false;
        myBlind = false;
        myMap.Clear();
        Minimum();
    }

    /// <summary>Checks the shape in the context of <paramref name="ContextShape"/>, which contains it.</summary>
    public abstract void InContext(in TopoDS_Shape ContextShape);

    /// <summary>The checks of the shape on its own.</summary>
    public abstract void Minimum();

    /// <summary>The checks that need no context and aren't in the minimum.</summary>
    public abstract void Blind();

    /// <summary>Adds <see cref="BRepCheck_CheckFail"/> to the statuses on <paramref name="S"/>.</summary>
    public void SetFailStatus(in TopoDS_Shape S)
    {
        bool aLocked = myIsParallel;
        if (aLocked)
        {
            myMutex.Enter();
        }

        try
        {
            if (!myMap.Find(S, out NCollection_List<BRepCheck_Status>? aList))
            {
                aList = new NCollection_List<BRepCheck_Status>();
                myMap.Bind(S, aList);
            }

            BRepCheck.Add(aList, BRepCheck_CheckFail);
        }
        finally
        {
            if (aLocked)
            {
                myMutex.Exit();
            }
        }
    }

    /// <summary>The statuses of the shape itself.</summary>
    public NCollection_List<BRepCheck_Status> Status() => myMap.Find(myShape);

    public bool IsMinimum() => myMin;

    public bool IsBlind() => myBlind;

    /// <summary>Starts iterating over the contexts in which the shape was checked.</summary>
    public void InitContextIterator()
    {
        myIterItems = [.. myMap];
        myIterIndex = 0;
        // At least 1 element : the Shape itself
        // OCCT.NET: OCCT reads the key of an empty map; the port checks first
        if (MoreShapeInContext() && ContextualShape().IsSame(myShape))
        {
            myIterIndex++;
        }
    }

    public bool MoreShapeInContext() => myIterIndex < myIterItems.Length;

    public TopoDS_Shape ContextualShape() => myIterItems[myIterIndex].Key;

    public NCollection_List<BRepCheck_Status> StatusOnShape() => myIterItems[myIterIndex].Value;

    public void NextShapeInContext()
    {
        myIterIndex++;
        if (MoreShapeInContext() && ContextualShape().IsSame(myShape))
        {
            myIterIndex++;
        }
    }

    /// <summary>Sets the parallel execution flag for sub-algorithms.</summary>
    public void SetParallel(bool theIsParallel) => myIsParallel = theIsParallel;

    /// <summary>True if sub-algorithms should use parallel execution.</summary>
    public bool IsParallel() => myIsParallel;

    public bool IsStatusOnShape(in TopoDS_Shape theShape) => myMap.IsBound(theShape);

    public NCollection_List<BRepCheck_Status> StatusOnShape(in TopoDS_Shape theShape) => myMap.Find(theShape);
}
