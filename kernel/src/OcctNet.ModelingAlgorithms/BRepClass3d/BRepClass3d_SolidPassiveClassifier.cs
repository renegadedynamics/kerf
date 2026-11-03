// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SolidPassiveClassifier.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SolidPassiveClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.IntCurveSurface_TransitionOnCurve;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// A 3D classifier of a point against faces given one by one: the state of the origin of a line from its nearest
/// intersection with them.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
public class BRepClass3d_SolidPassiveClassifier
{
    private bool isSet;
    private TopoDS_Face myFace = new();
    private Line3D myLin = new();
    private double myParam;
    private double myTolerance;
    private TopAbs_State myState;
    private bool hasIntersect;
    private readonly BRepClass3d_Intersector3d myIntersector = new();

    /// <summary>An undefined classifier.</summary>
    public BRepClass3d_SolidPassiveClassifier()
    {
        isSet = false;
        myParam = 0.0;
        myTolerance = 0.0;
        myState = TopAbs_UNKNOWN;
        hasIntersect = false;
    }

    /// <summary>
    /// Starts a classification of the origin of <paramref name="L"/>, with the tolerance <paramref name="Tol"/> of
    /// the intersections. <paramref name="P"/> is unused (OCCT's).
    /// </summary>
    public void Reset(in Line3D L, double P, double Tol)
    {
        myLin = L;
        myParam = RealLast();
        myTolerance = Tol;
        myState = TopAbs_UNKNOWN;
        isSet = true;
    }

    /// <summary>Updates the classification with the face <paramref name="Face"/> of the boundary (the orientation is unused).</summary>
    public void Compare(in TopoDS_Face Face, TopAbs_Orientation Or)
    {
        if (!isSet)
        {
            return;
        }

        hasIntersect = false;
        myIntersector.Perform(myLin, myParam, myTolerance, Face);
        if (myIntersector.IsDone())
        {
            if (myIntersector.HasAPoint())
            {
                hasIntersect = true;
                if (myIntersector.WParameter() < myParam)
                {
                    myParam = myIntersector.WParameter();
                    myFace = myIntersector.Face();
                    if (Math.Abs(myParam) <= myTolerance)
                    {
                        myState = TopAbs_ON;
                    }
                    else
                    {
                        //-- The intersection point between the line and a face F of the solid
                        //-- is in the face F or On a boundary of the face
                        if (myIntersector.Transition() == IntCurveSurface_Out)
                        {
                            //-- The line is going from inside the solid to outside the solid.
                            myState = TopAbs_IN;
                        }
                        else if (myIntersector.Transition() == IntCurveSurface_In)
                        {
                            myState = TopAbs_OUT;
                        }
                    }
                }
                else
                {
                    //-- No point has been found by the myIntersector.
                    //-- Or a Point has been found with a greater parameter.
                }
            } //-- myIntersector Has a point
            else
            {
                //-- The myIntersector failed.
            }
        } //-- Face has not been rejected
    }

    /// <summary>The current value of the parameter.</summary>
    public double Parameter() => myParam;

    /// <summary>True if the last compared face gave an intersection.</summary>
    public bool HasIntersection() => hasIntersect;

    /// <summary>The intersector (the classifier's own object).</summary>
    public BRepClass3d_Intersector3d Intersector() => myIntersector;

    /// <summary>The current state of the point.</summary>
    public TopAbs_State State() => myState;

    /// <summary>The face of the nearest intersection (OCCT stores it but has no accessor).</summary>
    internal TopoDS_Face NearestFace() => myFace;
}
