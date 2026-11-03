// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FClass2dOfFClassifier.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FClass2dOfFClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy;
using Kerf.Engine.Classification.Topological;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Classification;

/// <summary>
/// The 2D classifier of BRepClass_FClassifier: the state of the origin of a half-line from its intersections with
/// the edges of a face.
/// </summary>
/// <remarks>A working object (decision 21). The body is TopClass_Classifier2d's (decision 285).</remarks>
public class FClass2dOfFClassifier : IFClass2d<ClassifierEdge>
{
    private bool _isSet;
    private bool _firstCompare;
    private bool _firstTrans;
    private Line2D _lin = new();
    private double _param;
    private double _tolerance;
    private readonly CurveTransition _trans = new();
    private readonly Intersector _intersector = new();
    private int _closest;
    private Containment _state;
    private bool _isHeadOrEnd;

    /// <summary>An undefined classifier.</summary>
    public FClass2dOfFClassifier()
    {
        _isSet = false;
        _firstCompare = true;
        _firstTrans = true;
        _param = 0.0;
        _tolerance = 0.0;
        _closest = 0;
        _state = Unknown;
        _isHeadOrEnd = false;
    }

    /// <summary>
    /// Starts a classification of the origin of <paramref name="l"/>; <paramref name="p"/> is the length of the
    /// segment of <paramref name="l"/> intersected and <paramref name="tol"/> its tolerance in the intersections.
    /// </summary>
    public void Reset(in Line2D l, double p, double tol)
    {
        Classifier2d.Reset(out _lin,
                                    out _param,
                                    out _tolerance,
                                    out _state,
                                    out _firstCompare,
                                    out _firstTrans,
                                    out _closest,
                                    out _isSet,
                                    out _isHeadOrEnd,
                                    l,
                                    p,
                                    tol);
    }

    /// <summary>Updates the classification with the edge <paramref name="e"/> of the boundary.</summary>
    public void Compare(in ClassifierEdge e, Orientation or)
    {
        Classifier2d.Compare(ref _closest,
                                      _intersector,
                                      _lin,
                                      ref _param,
                                      _tolerance,
                                      ref _state,
                                      ref _firstCompare,
                                      ref _firstTrans,
                                      _trans,
                                      ref _isHeadOrEnd,
                                      e,
                                      or);
    }

    /// <summary>The current value of the parameter.</summary>
    public double Parameter() => _param;

    /// <summary>The intersector (the classifier's own object).</summary>
    public Intersector Intersector() => _intersector;

    Intersection2D IFClass2d<ClassifierEdge>.Intersector() => _intersector;

    /// <summary>
    /// 0 if the last compared edge had no relevant intersection, otherwise the index of that intersection in the
    /// intersector (points first, then the two ends of each segment).
    /// </summary>
    public int ClosestIntersection() => _closest;

    /// <summary>The current state of the point.</summary>
    public Containment State() => _state;

    /// <summary>True if the closest intersection is the head or the end of its edge.</summary>
    public bool IsHeadOrEnd() => _isHeadOrEnd;
}
