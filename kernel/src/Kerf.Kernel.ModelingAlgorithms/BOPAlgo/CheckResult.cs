// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CheckResult.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CheckResult.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Topology;

namespace Kerf.Engine.Booleans;

/// <summary>
/// A fault found by <see cref="ArgumentAnalyzer"/> or <see cref="Check"/>: its status, the
/// argument shapes it concerns (the object and the tool), their faulty sub-shapes, and for an invalid curve on a
/// surface the largest distance and its parameter.
/// </summary>
/// <remarks>
/// OCCT's value class is a sealed class here: the analyzer fills it (internal setters) and publishes it, and nothing
/// changes it afterwards, so a C++ copy is a shared reference. The public accessors publish their shapes, freezing
/// their graphs (as an alert's <c>GetShape</c>, decision 625): they are the arguments and their sub-shapes. Ported
/// code reads the <c>Internal</c> forms, which don't freeze.
/// </remarks>
internal sealed class CheckResult
{
    private Shape _shape1 = new();
    private Shape _shape2 = new();
    private CheckStatus _status;
    private readonly NCollection_List<Shape> _faulty1 = new();
    private readonly NCollection_List<Shape> _faulty2 = new();
    private double _maxDist1;
    private double _maxDist2;
    private double _maxPar1;
    private double _maxPar2;

    /// <summary>An empty result of unknown status.</summary>
    public CheckResult()
    {
        _status = CheckStatus.CheckUnknown;
        _maxDist1 = 0.0;
        _maxDist2 = 0.0;
        _maxPar1 = 0.0;
        _maxPar2 = 0.0;
    }

    /// <summary>Sets the ancestor shape (object) of the faulty sub-shapes.</summary>
    internal void SetShape1(in Shape theShape) => _shape1 = theShape;

    /// <summary>Adds a faulty sub-shape of the object.</summary>
    internal void AddFaultyShape1(in Shape theShape) => _faulty1.Append(theShape);

    /// <summary>Sets the ancestor shape (tool) of the faulty sub-shapes.</summary>
    internal void SetShape2(in Shape theShape) => _shape2 = theShape;

    /// <summary>Adds a faulty sub-shape of the tool.</summary>
    internal void AddFaultyShape2(in Shape theShape) => _faulty2.Append(theShape);

    /// <summary>The ancestor shape (object) of the faulty shapes (frozen; null if the fault isn't the object's).</summary>
    public Shape GetShape1() => Published(_shape1);

    /// <summary>The ancestor shape (tool) of the faulty shapes (frozen; null if the fault isn't the tool's).</summary>
    public Shape GetShape2() => Published(_shape2);

    /// <summary>The faulty shapes of the object (frozen).</summary>
    public IReadOnlyList<Shape> GetFaultyShapes1() => Published(_faulty1);

    /// <summary>The faulty shapes of the tool (frozen).</summary>
    public IReadOnlyList<Shape> GetFaultyShapes2() => Published(_faulty2);

    /// <summary>OCCT's <c>GetShape1</c>, without freezing.</summary>
    internal Shape InternalGetShape1() => _shape1;

    /// <summary>OCCT's <c>GetShape2</c>, without freezing.</summary>
    internal Shape InternalGetShape2() => _shape2;

    /// <summary>OCCT's <c>GetFaultyShapes1</c>, without freezing (the result's own list).</summary>
    internal NCollection_List<Shape> InternalGetFaultyShapes1() => _faulty1;

    /// <summary>OCCT's <c>GetFaultyShapes2</c>, without freezing (the result's own list).</summary>
    internal NCollection_List<Shape> InternalGetFaultyShapes2() => _faulty2;

    /// <summary>Sets the status of the fault.</summary>
    internal void SetCheckStatus(CheckStatus theStatus) => _status = theStatus;

    /// <summary>The status of the fault.</summary>
    public CheckStatus GetCheckStatus() => _status;

    /// <summary>Sets the largest distance for the first shape.</summary>
    internal void SetMaxDistance1(double dist) => _maxDist1 = dist;

    /// <summary>Sets the largest distance for the second shape.</summary>
    internal void SetMaxDistance2(double dist) => _maxDist2 = dist;

    /// <summary>Sets the parameter of the largest distance for the first shape.</summary>
    internal void SetMaxParameter1(double par) => _maxPar1 = par;

    /// <summary>Sets the parameter of the largest distance for the second shape.</summary>
    internal void SetMaxParameter2(double par) => _maxPar2 = par;

    /// <summary>The largest distance for the first shape (an invalid curve on surface).</summary>
    public double GetMaxDistance1() => _maxDist1;

    /// <summary>The largest distance for the second shape (an invalid curve on surface).</summary>
    public double GetMaxDistance2() => _maxDist2;

    /// <summary>The parameter on the curve of the largest distance for the first shape.</summary>
    public double GetMaxParameter1() => _maxPar1;

    /// <summary>The parameter on the curve of the largest distance for the second shape.</summary>
    public double GetMaxParameter2() => _maxPar2;

    private static Shape Published(in Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    private static IReadOnlyList<Shape> Published(NCollection_List<Shape> shapes)
    {
        Shape[] aShapes = new Shape[shapes.Extent()];
        int i = 0;
        foreach (Shape s in shapes)
        {
            aShapes[i++] = Published(s);
        }
        return ImmutableArrays.Own(aShapes);
    }
}
