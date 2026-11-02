// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CheckResult.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CheckResult.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Schematiq.Collections;

namespace Occt;

/// <summary>
/// A fault found by <see cref="BOPAlgo_ArgumentAnalyzer"/> or <see cref="BRepAlgoAPI_Check"/>: its status, the
/// argument shapes it concerns (the object and the tool), their faulty sub-shapes, and for an invalid curve on a
/// surface the largest distance and its parameter.
/// </summary>
/// <remarks>
/// OCCT's value class is a sealed class here: the analyzer fills it (internal setters) and publishes it, and nothing
/// changes it afterwards, so a C++ copy is a shared reference. The public accessors publish their shapes, freezing
/// their graphs (as an alert's <c>GetShape</c>, decision 625): they are the arguments and their sub-shapes. Ported
/// code reads the <c>Internal</c> forms, which don't freeze.
/// </remarks>
public sealed class BOPAlgo_CheckResult
{
    private TopoDS_Shape myShape1 = new();
    private TopoDS_Shape myShape2 = new();
    private BOPAlgo_CheckStatus myStatus;
    private readonly NCollection_List<TopoDS_Shape> myFaulty1 = new();
    private readonly NCollection_List<TopoDS_Shape> myFaulty2 = new();
    private double myMaxDist1;
    private double myMaxDist2;
    private double myMaxPar1;
    private double myMaxPar2;

    /// <summary>An empty result of unknown status.</summary>
    public BOPAlgo_CheckResult()
    {
        myStatus = BOPAlgo_CheckStatus.BOPAlgo_CheckUnknown;
        myMaxDist1 = 0.0;
        myMaxDist2 = 0.0;
        myMaxPar1 = 0.0;
        myMaxPar2 = 0.0;
    }

    /// <summary>Sets the ancestor shape (object) of the faulty sub-shapes.</summary>
    internal void SetShape1(in TopoDS_Shape TheShape) => myShape1 = TheShape;

    /// <summary>Adds a faulty sub-shape of the object.</summary>
    internal void AddFaultyShape1(in TopoDS_Shape TheShape) => myFaulty1.Append(TheShape);

    /// <summary>Sets the ancestor shape (tool) of the faulty sub-shapes.</summary>
    internal void SetShape2(in TopoDS_Shape TheShape) => myShape2 = TheShape;

    /// <summary>Adds a faulty sub-shape of the tool.</summary>
    internal void AddFaultyShape2(in TopoDS_Shape TheShape) => myFaulty2.Append(TheShape);

    /// <summary>The ancestor shape (object) of the faulty shapes (frozen; null if the fault isn't the object's).</summary>
    public TopoDS_Shape GetShape1() => Published(myShape1);

    /// <summary>The ancestor shape (tool) of the faulty shapes (frozen; null if the fault isn't the tool's).</summary>
    public TopoDS_Shape GetShape2() => Published(myShape2);

    /// <summary>The faulty shapes of the object (frozen).</summary>
    public ISchematiqList<TopoDS_Shape> GetFaultyShapes1() => Published(myFaulty1);

    /// <summary>The faulty shapes of the tool (frozen).</summary>
    public ISchematiqList<TopoDS_Shape> GetFaultyShapes2() => Published(myFaulty2);

    /// <summary>OCCT's <c>GetShape1</c>, without freezing.</summary>
    internal TopoDS_Shape InternalGetShape1() => myShape1;

    /// <summary>OCCT's <c>GetShape2</c>, without freezing.</summary>
    internal TopoDS_Shape InternalGetShape2() => myShape2;

    /// <summary>OCCT's <c>GetFaultyShapes1</c>, without freezing (the result's own list).</summary>
    internal NCollection_List<TopoDS_Shape> InternalGetFaultyShapes1() => myFaulty1;

    /// <summary>OCCT's <c>GetFaultyShapes2</c>, without freezing (the result's own list).</summary>
    internal NCollection_List<TopoDS_Shape> InternalGetFaultyShapes2() => myFaulty2;

    /// <summary>Sets the status of the fault.</summary>
    internal void SetCheckStatus(BOPAlgo_CheckStatus TheStatus) => myStatus = TheStatus;

    /// <summary>The status of the fault.</summary>
    public BOPAlgo_CheckStatus GetCheckStatus() => myStatus;

    /// <summary>Sets the largest distance for the first shape.</summary>
    internal void SetMaxDistance1(double theDist) => myMaxDist1 = theDist;

    /// <summary>Sets the largest distance for the second shape.</summary>
    internal void SetMaxDistance2(double theDist) => myMaxDist2 = theDist;

    /// <summary>Sets the parameter of the largest distance for the first shape.</summary>
    internal void SetMaxParameter1(double thePar) => myMaxPar1 = thePar;

    /// <summary>Sets the parameter of the largest distance for the second shape.</summary>
    internal void SetMaxParameter2(double thePar) => myMaxPar2 = thePar;

    /// <summary>The largest distance for the first shape (an invalid curve on surface).</summary>
    public double GetMaxDistance1() => myMaxDist1;

    /// <summary>The largest distance for the second shape (an invalid curve on surface).</summary>
    public double GetMaxDistance2() => myMaxDist2;

    /// <summary>The parameter on the curve of the largest distance for the first shape.</summary>
    public double GetMaxParameter1() => myMaxPar1;

    /// <summary>The parameter on the curve of the largest distance for the second shape.</summary>
    public double GetMaxParameter2() => myMaxPar2;

    private static TopoDS_Shape Published(in TopoDS_Shape theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }

    private static ISchematiqList<TopoDS_Shape> Published(NCollection_List<TopoDS_Shape> theShapes)
    {
        TopoDS_Shape[] aShapes = new TopoDS_Shape[theShapes.Extent()];
        int i = 0;
        foreach (TopoDS_Shape aS in theShapes)
        {
            aShapes[i++] = Published(aS);
        }
        return PackedList.Own(aShapes);
    }
}
