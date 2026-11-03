// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FacePassiveClassifier.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FacePassiveClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// A 2D classifier of a point against edges given one by one: the state of the origin of a half-line from its
/// intersections with them (the same body as <see cref="BRepClass_FClass2dOfFClassifier"/>).
/// </summary>
/// <remarks>A working object (decision 21). The body is TopClass_Classifier2d's (decision 285).</remarks>
public class BRepClass_FacePassiveClassifier : ITopClass_FClass2d<BRepClass_Edge>
{
    private bool myIsSet;
    private bool myFirstCompare;
    private bool myFirstTrans;
    private Line2D myLin = new();
    private double myParam;
    private double myTolerance;
    private readonly TopTrans_CurveTransition myTrans = new();
    private readonly BRepClass_Intersector myIntersector = new();
    private int myClosest;
    private TopAbs_State myState;
    private bool myIsHeadOrEnd;

    /// <summary>An undefined classifier.</summary>
    public BRepClass_FacePassiveClassifier()
    {
        myIsSet = false;
        myFirstCompare = true;
        myFirstTrans = true;
        myParam = 0.0;
        myTolerance = 0.0;
        myClosest = 0;
        myState = TopAbs_UNKNOWN;
        myIsHeadOrEnd = false;
    }

    /// <summary>
    /// Starts a classification of the origin of <paramref name="L"/>; <paramref name="P"/> is the length of the
    /// segment of <paramref name="L"/> intersected and <paramref name="Tol"/> its tolerance in the intersections.
    /// </summary>
    public void Reset(in Line2D L, double P, double Tol)
    {
        TopClass_Classifier2d.Reset(out myLin,
                                    out myParam,
                                    out myTolerance,
                                    out myState,
                                    out myFirstCompare,
                                    out myFirstTrans,
                                    out myClosest,
                                    out myIsSet,
                                    out myIsHeadOrEnd,
                                    L,
                                    P,
                                    Tol);
    }

    /// <summary>Updates the classification with the edge <paramref name="E"/> of the boundary.</summary>
    public void Compare(in BRepClass_Edge E, TopAbs_Orientation Or)
    {
        TopClass_Classifier2d.Compare(ref myClosest,
                                      myIntersector,
                                      myLin,
                                      ref myParam,
                                      myTolerance,
                                      ref myState,
                                      ref myFirstCompare,
                                      ref myFirstTrans,
                                      myTrans,
                                      ref myIsHeadOrEnd,
                                      E,
                                      Or);
    }

    /// <summary>The current value of the parameter.</summary>
    public double Parameter() => myParam;

    /// <summary>The intersector (the classifier's own object).</summary>
    public BRepClass_Intersector Intersector() => myIntersector;

    IntRes2d_Intersection ITopClass_FClass2d<BRepClass_Edge>.Intersector() => myIntersector;

    /// <summary>
    /// 0 if the last compared edge had no relevant intersection, otherwise the index of that intersection in the
    /// intersector (points first, then the two ends of each segment).
    /// </summary>
    public int ClosestIntersection() => myClosest;

    /// <summary>The current state of the point.</summary>
    public TopAbs_State State() => myState;

    /// <summary>True if the closest intersection is the head or the end of its edge.</summary>
    public bool IsHeadOrEnd() => myIsHeadOrEnd;
}
