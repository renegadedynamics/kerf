// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Splitter.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Splitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// Splits the objects by the tools: the result holds the splits of the objects only (the splits of the tools are
/// left out). Error: <see cref="BOPAlgo_AlertTooFewArguments"/> without objects or with fewer than two shapes.
/// </summary>
public class BRepAlgoAPI_Splitter : BRepAlgoAPI_BuilderAlgo
{
    protected NCollection_List<TopoDS_Shape> myTools = new(); //!< Tool arguments of the operation

    /// <summary>An empty splitter.</summary>
    public BRepAlgoAPI_Splitter()
    {
    }

    /// <summary>A splitter using the intersections already computed by <paramref name="thePF"/>.</summary>
    public BRepAlgoAPI_Splitter(BOPAlgo_PaveFiller thePF)
        : base(thePF)
    {
    }

    /// <summary>Sets the tools (copies the list).</summary>
    public void SetTools(NCollection_List<TopoDS_Shape> theLS) => myTools = new NCollection_List<TopoDS_Shape>(theLS);

    /// <summary>The tools.</summary>
    public NCollection_List<TopoDS_Shape> Tools() => myTools;

    /// <summary>Performs the split.</summary>
    public override void Build(ProgressRange theRange = default)
    {
        // Set Not Done status by default
        NotDone();
        // Clear the contents
        Clear();
        // Check for availability of arguments and tools
        if (myArguments.IsEmpty() || (myArguments.Extent() + myTools.Extent()) < 2)
        {
            AddError(new BOPAlgo_AlertTooFewArguments());
            return;
        }

        // If necessary perform intersection of the argument shapes
        using var aPS = new ProgressScope(theRange, "Performing Split operation", myIsIntersectionNeeded ? 100 : 30);
        if (myIsIntersectionNeeded)
        {
            // Combine Objects and Tools into a single list for intersection
            NCollection_List<TopoDS_Shape> aLArgs = new(myArguments);
            for (NCollection_List<TopoDS_Shape>.Iterator it = new(myTools); it.More(); it.Next())
            {
                aLArgs.Append(it.Value());
            }

            // Perform intersection
            IntersectShapes(aLArgs, aPS.Next(70));
            if (HasErrors())
            {
                return;
            }
        }

        // Initialization of the building tool
        BOPAlgo_Splitter aSplitter = new();
        myBuilder = aSplitter;
        myBuilder.SetArguments(myArguments);
        aSplitter.SetTools(myTools);

        // Build result shape basing on the intersection results
        BuildResult(aPS.Next(30));
    }
}
