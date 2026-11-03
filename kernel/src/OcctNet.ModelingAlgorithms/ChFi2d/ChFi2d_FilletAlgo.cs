// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_FilletAlgo.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_FilletAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// Creates a fillet edge: an arc of the given radius tangent to two edges of a plane, which must be connected by
/// their ends (shared vertices are not needed), on any curves that support point projection and are C2.
/// </summary>
/// <remarks>
/// <para>The algorithm is iterative, close to the Newton method:</para>
/// <list type="number">
/// <item>The first edge is split into N segments where probably only one root can be found; N depends on the
/// complexity of its curve.</item>
/// <item>On each segment the value and derivative of a function are computed: its argument is a parameter on the first
/// curve, which gives a point of tangency; the centre of the fillet is on the perpendicular from it; the centre is
/// projected on the second curve; the length of the projection minus the radius is the value. The derivative is
/// computed from the value at a small shift.</item>
/// <item>Newton's method finds the point of the segment where the value is nearest to zero; if not near enough, steps 2
/// and 3 are repeated with the found point as an end of the segment.</item>
/// <item>The fillet is built from a root (the start point), the projection on the second curve (the end point) and
/// the centre; the edges are cut at the points of tangency.</item>
/// </list>
/// <para>A working object; its results are new, frozen edges. The edges it is given are only read.</para>
/// </remarks>
public sealed class ChFi2d_FilletAlgo
{
    //! Initial edges where the fillet must be computed.
    private TopoDS_Edge myEdge1 = new(), myEdge2 = new();
    //! Plane where fillet arc must be created.
    private Geom_Plane? myPlane;
    //! Underlying curves of the initial edges
    private Geom2d_Curve? myCurve1, myCurve2;
    //! Start and end parameters of curves of initial edges.
    private double myStart1, myEnd1, myStart2, myEnd2, myRadius;
    //! List of params where roots were found.
    private readonly NCollection_List<double> myResultParams = new();
    //! sequence of 0 or 1: position of the fillet relatively to the first curve
    private readonly NCollection_Sequence<int> myResultOrientation = new();
    //! position of the fillet relatively to the first curve
    private bool myStartSide;
    //! are initial edges where exchanged in the beginning: to make first edge
    //! more simple and minimize number of iterations
    private bool myEdgesExchnged;
    //! Number to avoid infinity recursion: indicates how deep the recursion is performed.
    private int myDegreeOfRecursion;

    /// <summary>An empty algorithm: call <c>Init</c> before <see cref="Perform"/>.</summary>
    public ChFi2d_FilletAlgo()
    {
        myStart1 = 0.0;
        myEnd1 = 0.0;
        myStart2 = 0.0;
        myEnd2 = 0.0;
        myRadius = 0.0;
        myStartSide = false;
        myEdgesExchnged = false;
        myDegreeOfRecursion = 0;
    }

    /// <summary>A fillet algorithm on a wire of two edges in a plane.</summary>
    public ChFi2d_FilletAlgo(in TopoDS_Wire theWire, in Plane3D thePlane)
        : this()
    {
        Init(theWire, thePlane);
    }

    /// <summary>A fillet algorithm on two edges in a plane.</summary>
    public ChFi2d_FilletAlgo(in TopoDS_Edge theEdge1, in TopoDS_Edge theEdge2, in Plane3D thePlane)
        : this()
    {
        myEdge1 = theEdge1;
        myEdge2 = theEdge2;
        Init(theEdge1, theEdge2, thePlane);
    }

    /// <summary>Initializes the algorithm by a wire of two edges in a plane.</summary>
    /// <exception cref="ConstructionErrorException">The wire has fewer than two edges.</exception>
    public void Init(in TopoDS_Wire theWire, in Plane3D thePlane)
    {
        TopoDS_Edge theEdge1 = new(), theEdge2 = new();
        var itr = new TopoDS_Iterator(theWire);
        for (; itr.More(); itr.Next())
        {
            if (theEdge1.IsNull())
            {
                theEdge1 = TopoDS.Edge(itr.Value());
            }
            else if (theEdge2.IsNull())
            {
                theEdge2 = TopoDS.Edge(itr.Value());
            }
            else
            {
                break;
            }
        }
        if (theEdge1.IsNull() || theEdge2.IsNull())
        {
            throw new ConstructionErrorException("The fillet algorithms expects a wire consisting of two edges.");
        }
        Init(theEdge1, theEdge2, thePlane);
    }

    /// <summary>Initializes the algorithm by two edges in a plane.</summary>
    public void Init(in TopoDS_Edge theEdge1, in TopoDS_Edge theEdge2, in Plane3D thePlane)
    {
        myPlane = new Geom_Plane(thePlane);

        myEdgesExchnged = false;

        var aBAC1 = new BRepAdaptor_Curve(theEdge1);
        var aBAC2 = new BRepAdaptor_Curve(theEdge2);
        if (aBAC1.GetType() < aBAC2.GetType())
        { // first curve must be more complicated
            myEdge1 = theEdge2;
            myEdge2 = theEdge1;
            myEdgesExchnged = true;
        }
        else
        {
            myEdge1 = theEdge1;
            myEdge2 = theEdge2;
        }

        Geom_Curve aCurve1 = BRep_Tool.Curve(myEdge1, out myStart1, out myEnd1)!;
        Geom_Curve aCurve2 = BRep_Tool.Curve(myEdge2, out myStart2, out myEnd2)!;

        myCurve1 = GeomProjLib.Curve2d(aCurve1, myStart1, myEnd1, myPlane)!;
        myCurve2 = GeomProjLib.Curve2d(aCurve2, myStart2, myEnd2, myPlane)!;

        while (myCurve1.IsPeriodic() && myStart1 >= myEnd1)
        {
            myEnd1 += myCurve1.Period();
        }
        while (myCurve2.IsPeriodic() && myStart2 >= myEnd2)
        {
            myEnd2 += myCurve2.Period();
        }

        if (aBAC1.GetType() == aBAC2.GetType())
        {
            if (myEnd2 - myStart2 < myEnd1 - myStart1)
            { // first curve must be parametrically shorter
                TopoDS_Edge anEdge = myEdge1;
                myEdge1 = myEdge2;
                myEdge2 = anEdge;
                Geom2d_Curve aCurve = myCurve1;
                myCurve1 = myCurve2;
                myCurve2 = aCurve;
                double a = myStart1;
                myStart1 = myStart2;
                myStart2 = a;
                a = myEnd1;
                myEnd1 = myEnd2;
                myEnd2 = a;
                myEdgesExchnged = true;
            }
        }
    }

    //! This function returns true if linear segment from start point of the
    //! fillet arc to the end point is intersected by the first or second
    //! curve: in this case fillet is invalid.
    private static bool IsRadiusIntersected(Geom2d_Curve theCurve,
                                            double theCurveMin,
                                            double theCurveMax,
                                            Point2D theStart,
                                            Point2D theEnd,
                                            bool theStartConnected)
    {
        // Check the given start and end if they are identical. If yes
        // return false
        if (theStart.SquareDistance(theEnd) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return false;
        }
        var line = new Geom2d_Line(theStart, new Direction2D(new Vector2D(theStart, theEnd)));
        var anInter = new Geom2dAPI_InterCurveCurve(theCurve, line, global::Kerf.Engine.Foundation.Precision.Confusion());
        int a;
        Point2D aPoint;
        for (a = anInter.NbPoints(); a > 0; a--)
        {
            aPoint = anInter.Point(a);
            var aProjInt = new Geom2dAPI_ProjectPointOnCurve(aPoint, theCurve, theCurveMin, theCurveMax);
            // OCCT compares the parameter of the nearest projection, not its distance, with the tolerance
            if (aProjInt.NbPoints() < 1 || aProjInt.LowerDistanceParameter() > global::Kerf.Engine.Foundation.Precision.Confusion()) // OCCT: a parameter
            {
                continue; // point is not on edge
            }

            if (aPoint.Distance(theStart) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                if (!theStartConnected)
                {
                    return true;
                }
            }
            if (aPoint.Distance(theEnd) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return true;
            }
            if (new Vector2D(aPoint, theStart).IsOpposite(new Vector2D(aPoint, theEnd), global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                return true;
            }
        }
        Geom2d_Curve aCurve = theCurve;
        for (a = anInter.NbSegments(); a > 0; a--)
        {
            // anInter.Segment(a, aCurve); //not implemented (bug in OCC)
            aPoint = aCurve.Value(aCurve.FirstParameter());

            var aProjInt = new Geom2dAPI_ProjectPointOnCurve(aPoint, theCurve, theCurveMin, theCurveMax);
            if (aProjInt.NbPoints() != 0 && aProjInt.LowerDistanceParameter() < global::Kerf.Engine.Foundation.Precision.Confusion()) // OCCT: a parameter
            { // point is on edge
                if (aPoint.Distance(theStart) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    if (!theStartConnected)
                    {
                        return true;
                    }
                }
                if (aPoint.Distance(theEnd) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    return true;
                }
                if (new Vector2D(aPoint, theStart).IsOpposite(new Vector2D(aPoint, theEnd), global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return true;
                }
            }
            aPoint = aCurve.Value(aCurve.LastParameter());

            aProjInt.Init(aPoint, theCurve, theCurveMin, theCurveMax);
            if (aProjInt.NbPoints() != 0 && aProjInt.LowerDistanceParameter() < global::Kerf.Engine.Foundation.Precision.Confusion()) // OCCT: a parameter
            { // point is on edge
                if (aPoint.Distance(theStart) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    if (!theStartConnected)
                    {
                        return true;
                    }
                }
                if (aPoint.Distance(theEnd) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    return true;
                }
                if (new Vector2D(aPoint, theStart).IsOpposite(new Vector2D(aPoint, theEnd), global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return true;
                }
            }
        }
        return false;
    }

    //! Computes the value the function in the current point.
    //! <theLimit> is end parameter of the segment
    private void FillPoint(FilletPoint thePoint, double theLimit)
    {
        // on the intersection point
        bool aValid = false;
        double aStep = global::Kerf.Engine.Foundation.Precision.Confusion();
        Point2D aCenter = new(), aPoint = new(); // center of fillet and point on curve1
        double aParam = thePoint.getParam();
        if (theLimit < aParam)
        {
            aStep = -aStep;
        }
        for (aValid = false; !aValid; aParam += aStep)
        {
            if ((aParam - aStep - theLimit) * (aParam - theLimit) <= 0)
            {
                break; // limit was exceeded
            }
            aStep *= 2;
            myCurve1!.D1(aParam, out aPoint, out Vector2D aVec);
            if (aVec.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                continue;
            }

            var aPerp = new Vector2D(((myStartSide) ? -1 : 1) * aVec.Y(), ((myStartSide) ? 1 : -1) * aVec.X());
            aPerp.Normalize();
            aPerp.Multiply(myRadius);
            aCenter = aPoint.Translated(aPerp);

            var aProjInt = new Geom2dAPI_ProjectPointOnCurve(aPoint, myCurve2!, myStart2, myEnd2);
            if (aProjInt.NbPoints() == 0
                || aPoint.Distance(aProjInt.NearestPoint()) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                aValid = true;
                break;
            }
        }
        if (aValid)
        {
            thePoint.setParam(aParam);
            thePoint.setCenter(aCenter);
            aValid = !IsRadiusIntersected(myCurve2!, myStart2, myEnd2, aPoint, aCenter, true);
        }

        var aProj = new Geom2dAPI_ProjectPointOnCurve(aCenter, myCurve2!);
        int a, aNB = aProj.NbPoints();
        for (a = aNB; a > 0; a--)
        {
            if (aPoint.SquareDistance(aProj.Point(a)) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                continue;
            }

            bool aValid2 = aValid;
            if (aValid2)
            {
                aValid2 = !IsRadiusIntersected(myCurve1!, myStart1, myEnd1, aCenter, aProj.Point(a), false);
            }

            // checking the right parameter
            double aParamProj = aProj.Parameter(a);
            while (myCurve2!.IsPeriodic() && aParamProj < myStart2)
            {
                aParamProj += myCurve2.Period();
            }

            double d = aProj.Distance(a);
            thePoint.appendValue(d * d - myRadius * myRadius,
                                 (aParamProj >= myStart2 && aParamProj <= myEnd2 && aValid2));
            if (Math.Abs(d - myRadius) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                thePoint.setParam2(aParamProj);
            }
        }
    }

    //! Computes the derivative value of the function in the current point.
    //! <theDiffStep> is small step for approximate derivative computation
    //! <theFront> is direction of the step: from or reversed
    private void FillDiff(FilletPoint thePoint, double theDiffStep, bool theFront)
    {
        double aDelta = theFront ? (theDiffStep) : (-theDiffStep);
        var aDiff = new FilletPoint(thePoint.getParam() + aDelta);
        FillPoint(aDiff, aDelta * 999.0);
        if (!thePoint.calculateDiff(aDiff))
        {
            aDiff.setParam(thePoint.getParam() - aDelta);
            FillPoint(aDiff, -aDelta * 999);
            thePoint.calculateDiff(aDiff);
        }
    }

    /// <summary>Constructs a fillet edge of the radius. True if at least one result was found.</summary>
    public bool Perform(double theRadius)
    {
        myDegreeOfRecursion = 0;
        myResultParams.Clear();
        myResultOrientation.Clear();

        double aNBSteps;
        var aGAC = new Geom2dAdaptor_Curve(myCurve1!);
        switch (aGAC.GetType())
        {
            case Line:
                aNBSteps = 2;
                break;
            case Circle:
                aNBSteps = 4;
                break;
            case Ellipse:
                aNBSteps = 5;
                break;
            case BSplineCurve:
                aNBSteps = 2 + aGAC.Degree() * aGAC.NbPoles();
                break;
            default: // unknown: maximum
                aNBSteps = 100;
                break;
        }
        // std::cout<<"aNBSteps = "<<aNBSteps<<std::endl;

        myRadius = theRadius;
        double aParam, aStep, aDStep;
        aStep = (myEnd1 - myStart1) / aNBSteps;
        aDStep = 1.0e-4 * aStep;

        int aCycle;
        for (aCycle = 2, myStartSide = false; aCycle != 0; myStartSide = !myStartSide, aCycle--)
        {
            FilletPoint? aLeft = null, aRight;

            for (aParam = myStart1 + aStep;
                 aParam < myEnd1 || Math.Abs(myEnd1 - aParam) < global::Kerf.Engine.Foundation.Precision.Confusion();
                 aParam += aStep)
            {
                if (aLeft is null)
                {
                    aLeft = new FilletPoint(aParam - aStep);
                    FillPoint(aLeft, aParam);
                    FillDiff(aLeft, aDStep, true);
                }

                aRight = new FilletPoint(aParam);
                FillPoint(aRight, aParam - aStep);
                FillDiff(aRight, aDStep, false);

                aLeft.FilterPoints(aRight);
                PerformNewton(aLeft, aRight);

                aLeft = aRight;
            } // for
        } // for

        return !myResultParams.IsEmpty();
    }

    //! Splits segment by the parameter and calls Newton method for both segments.
    //! It supplies recursive iterations of the Newton methods calls
    //! (PerformNewton calls this function and this calls Netwton two times).
    private bool ProcessPoint(FilletPoint theLeft, FilletPoint theRight, double theParameter)
    {
        if (theParameter >= theLeft.getParam() && theParameter < theRight.getParam())
        {
            double aDX = (theRight.getParam() - theLeft.getParam());
            if (theParameter - theLeft.getParam() < aDX / 100.0)
            {
                theParameter = theLeft.getParam() + aDX / 100.0;
            }
            if (theRight.getParam() - theParameter < aDX / 100.0)
            {
                theParameter = theRight.getParam() - aDX / 100.0;
            }

            // Protection on infinite loops.
            myDegreeOfRecursion++;
            double diffx = 0.001 * aDX;
            if (myDegreeOfRecursion > 100)
            {
                diffx *= 10.0;
                if (myDegreeOfRecursion > 1000)
                {
                    diffx *= 10.0;
                    if (myDegreeOfRecursion > 3000)
                    {
                        return true;
                    }
                }
            }

            FilletPoint aPoint1 = theLeft.Copy();
            var aPoint2 = new FilletPoint(theParameter);
            FillPoint(aPoint2, aPoint1.getParam());
            FillDiff(aPoint2, diffx, true);

            aPoint1.FilterPoints(aPoint2);
            PerformNewton(aPoint1, aPoint2);
            aPoint2.FilterPoints(theRight);
            PerformNewton(aPoint2, theRight);

            return true;
        }

        return false;
    }

    //! Using Newton methods computes optimal point, that can be root of the
    //! function taking into account two input points, functions value and derivatives.
    //! Performs iteration until root is found or failed to find root.
    //! Stores roots in myResultParams.
    private void PerformNewton(FilletPoint theLeft, FilletPoint theRight)
    {
        int a;
        // check the left: if this is solution store it and remove it from the list of researching points
        // of theLeft
        a = theLeft.hasSolution(myRadius);
        if (a != 0)
        {
            if (theLeft.isValid(a))
            {
                myResultParams.Append(theLeft.getParam());
                myResultOrientation.Append(myStartSide ? 1 : 0);
            }
            return;
        }

        double aDX = theRight.getParam() - theLeft.getParam();
        if (aDX < 1.0e-6 * global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            a = theRight.hasSolution(myRadius);
            if (a != 0 && theRight.isValid(a))
            {
                myResultParams.Append(theRight.getParam());
                myResultOrientation.Append(myStartSide ? 1 : 0);
            }
            return;
        }
        for (a = 1; a <= theLeft.getNBValues(); a++)
        {
            int aNear = theLeft.getNear(a);

            double aA = (theRight.getDiff(aNear) - theLeft.getDiff(a)) / aDX;
            double aB = theLeft.getDiff(a) - aA * theLeft.getParam();
            double aC = theLeft.getValue(a) - theLeft.getDiff(a) * theLeft.getParam()
                        + aA * theLeft.getParam() * theLeft.getParam() / 2.0;
            double aDet = aB * aB - 2.0 * aA * aC;

            if (Math.Abs(aA) < global::Kerf.Engine.Foundation.Precision.Confusion())
            { // linear case
                // std::cout<<"###"<<std::endl;
                if (Math.Abs(aB) > 10e-20)
                {
                    double aX0 = -aC / aB; // use extremum
                    if (aX0 > theLeft.getParam() && aX0 < theRight.getParam())
                    {
                        ProcessPoint(theLeft, theRight, aX0);
                    }
                }
                else
                {
                    ProcessPoint(theLeft,
                                 theRight,
                                 theLeft.getParam() + aDX / 2.0); // linear division otherwise
                }
            }
            else
            {
                if (Math.Abs(aB) > Math.Abs(aDet * 1000000.0))
                { // possible floating point operations accuracy errors
                    // std::cout<<"*";
                    ProcessPoint(theLeft,
                                 theRight,
                                 theLeft.getParam() + aDX / 2.0); // linear division otherwise
                }
                else
                {
                    if (aDet > 0)
                    { // two solutions
                        aDet = Math.Sqrt(aDet);
                        bool aRes = ProcessPoint(theLeft, theRight, (-aB + aDet) / aA);
                        if (!aRes)
                        {
                            aRes = ProcessPoint(theLeft, theRight, (-aB - aDet) / aA);
                        }
                        if (!aRes)
                        {
                            ProcessPoint(theLeft,
                                         theRight,
                                         theLeft.getParam() + aDX / 2.0); // linear division otherwise
                        }
                    }
                    else
                    {
                        // std::cout<<"%%%"<<std::endl;
                        double aX0 = -aB / aA; // use extremum
                        if (aX0 > theLeft.getParam() && aX0 < theRight.getParam())
                        {
                            ProcessPoint(theLeft, theRight, aX0);
                        }
                        else
                        {
                            ProcessPoint(theLeft,
                                         theRight,
                                         theLeft.getParam() + aDX / 2.0); // linear division otherwise
                        }
                    }
                }
            }
        } // for
    }

    /// <summary>
    /// The number of solutions; <paramref name="thePoint"/> (the intersecting or common point of the edges) chooses
    /// among several fillets.
    /// </summary>
    public int NbResults(in Point3D thePoint)
    {
        double aX, aY;
        Point2D aTargetPoint2d = new();
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(myPlane!.Pln().Position(), thePoint, out aX, out aY);
        aTargetPoint2d.SetCoord(aX, aY);

        // iterate through all possible solutions.
        int i = 1, nb = 0;
        var anIter = new NCollection_List<double>.Iterator(myResultParams);
        for (; anIter.More(); anIter.Next(), i++)
        {
            myStartSide = (myResultOrientation.Value(i)) != 0;
            var aPoint = new FilletPoint(anIter.Value());
            FillPoint(aPoint, anIter.Value() + 1.0);
            if (aPoint.hasSolution(myRadius) != 0)
            {
                nb++;
            }
        } // for

        return nb;
    }

    /// <summary>
    /// The fillet edge and the two edges it trims (all frozen): the solution nearest to <paramref name="thePoint"/> if
    /// <paramref name="iSolution"/> is -1, else that solution. A null edge if there is none.
    /// </summary>
    public TopoDS_Edge Result(in Point3D thePoint, ref TopoDS_Edge theEdge1, ref TopoDS_Edge theEdge2, int iSolution = -1)
    {
        TopoDS_Edge aResult = new();
        Point2D aTargetPoint2d = new();
        double aX, aY;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(myPlane!.Pln().Position(), thePoint, out aX, out aY);
        aTargetPoint2d.SetCoord(aX, aY);

        // choose the nearest circle
        double aDistance = 0.0, aP;
        FilletPoint? aNearest;
        int a, iSol = 1;
        var anIter = new NCollection_List<double>.Iterator(myResultParams);
        for (aNearest = null, a = 1; anIter.More(); anIter.Next(), a++)
        {
            myStartSide = (myResultOrientation.Value(a)) != 0;
            var aPoint = new FilletPoint(anIter.Value());
            FillPoint(aPoint, anIter.Value() + 1.0);
            if (aPoint.hasSolution(myRadius) == 0)
            {
                continue;
            }
            aP = double.MaxValue;
            if (iSolution == -1)
            {
                aP = Math.Abs(aPoint.getCenter().Distance(aTargetPoint2d) - myRadius);
            }
            else if (iSolution == iSol)
            {
                aP = 0.0;
            }
            if (aNearest is null || aP < aDistance)
            {
                aNearest = aPoint;
                aDistance = aP;
            }
            if (iSolution == iSol)
            {
                break;
            }
            iSol++;
        } // for

        if (aNearest is null)
        {
            return aResult;
        }

        // create circle edge
        Point3D aCenter = global::Kerf.Engine.Geometry.ElSLib.PlaneValue(aNearest.getCenter().X(),
                                           aNearest.getCenter().Y(),
                                           myPlane.Pln().Position());
        var aCircle = new Geom_Circle(new Frame3D(aCenter, myPlane.Pln().Axis().Direction()), myRadius);
        myCurve1!.D0(aNearest.getParam(), out Point2D aPoint2d1);
        myCurve2!.D0(aNearest.getParam2(), out Point2D aPoint2d2);
        Point3D aPoint1 = global::Kerf.Engine.Geometry.ElSLib.PlaneValue(aPoint2d1.X(), aPoint2d1.Y(), myPlane.Pln().Position());
        Point3D aPoint2 = global::Kerf.Engine.Geometry.ElSLib.PlaneValue(aPoint2d2.X(), aPoint2d2.Y(), myPlane.Pln().Position());

        var aProj = new GeomAPI_ProjectPointOnCurve(thePoint, aCircle);
        double aTargetParam = aProj.LowerDistanceParameter();
        Point3D aPointOnCircle = aProj.NearestPoint();

        // There is a bug in Open CASCADE in calculation of nearest point to a circle near the parameter
        // 0.0 Therefore I check this extrema point manually:
        Point3D p0 = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, aCircle.Circ());
        if (p0.Distance(thePoint) < aPointOnCircle.Distance(thePoint))
        {
            aTargetParam = 0.0;
            aPointOnCircle = p0;
        }

        aProj.Perform(aPoint1);
        double aParam1 = aProj.LowerDistanceParameter();
        aProj.Perform(aPoint2);
        double aParam2 = aProj.LowerDistanceParameter();
        bool aIsOut = ((aParam1 < aTargetParam && aParam2 < aTargetParam)
                       || (aParam1 > aTargetParam && aParam2 > aTargetParam));
        if (aParam1 > aParam2)
        {
            aIsOut = !aIsOut;
        }
        var aBuilder = new BRepBuilderAPI_MakeEdge(aCircle.Circ(),
                                                   aIsOut ? aParam2 : aParam1,
                                                   aIsOut ? aParam1 : aParam2);
        aResult = aBuilder.Edge();

        // divide edges
        double aStart, anEnd;
        Geom_Curve aCurve = BRep_Tool.Curve(myEdge1, out aStart, out anEnd)!;
        aCurve.D1(aNearest.getParam(), out aPoint1, out Vector3D aDir);

        aCircle.D1(aParam1, out aPoint1, out Vector3D aCircleDir);

        if ((aCircleDir.Angle(aDir) > M_PI / 2.0) ? !aIsOut : aIsOut)
        {
            aStart = aNearest.getParam();
        }
        else
        {
            anEnd = aNearest.getParam();
        }

        // Check the case when start and end are identical. This happens
        // when the edge decreases to size 0. Old ww5 allows such
        // cases. So we are again bug compatible
        if (Math.Abs(aStart - anEnd) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            anEnd = aStart + global::Kerf.Engine.Foundation.Precision.Confusion();
        }
        // Divide edge
        var aDivider1 = new BRepBuilderAPI_MakeEdge(aCurve, aStart, anEnd);
        if (myEdgesExchnged)
        {
            theEdge2 = aDivider1.Edge();
        }
        else
        {
            theEdge1 = aDivider1.Edge();
        }

        aCurve = BRep_Tool.Curve(myEdge2, out aStart, out anEnd)!;
        aCurve.D1(aNearest.getParam2(), out aPoint2, out aDir);

        aCircle.D1(aParam2, out aPoint2, out aCircleDir);

        if ((aCircleDir.Angle(aDir) > M_PI / 2.0) ? aIsOut : !aIsOut)
        {
            aStart = aNearest.getParam2();
        }
        else
        {
            anEnd = aNearest.getParam2();
        }

        // Check the case when start and end are identical. This happens
        // when the edge decreases to size 0. Old ww5 allows such
        // cases. So we are again bug compatible
        if (Math.Abs(aStart - anEnd) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            anEnd = aStart + global::Kerf.Engine.Foundation.Precision.Confusion();
        }
        var aDivider2 = new BRepBuilderAPI_MakeEdge(aCurve, aStart, anEnd);
        if (myEdgesExchnged)
        {
            theEdge1 = aDivider2.Edge();
        }
        else
        {
            theEdge2 = aDivider2.Edge();
        }

        return aResult;
    }
}

/// <summary>
/// A point on the first curve of <see cref="ChFi2d_FilletAlgo"/>, with the fillet function's values and derivatives
/// there (OCCT's private class FilletPoint, declared in ChFi2d_FilletAlgo.hxx).
/// </summary>
internal sealed class FilletPoint
{
    //! Parameter on the first curve (start fillet point).
    private double myParam;
    //! Parameter on the second curve (end fillet point).
    private double myParam2;
    //! Values and derivative values of the fillet function.
    //! May be several if there are many projections on the second curve.
    private readonly NCollection_Sequence<double> myV = new(), myD = new();
    //! Center of the fillet arc.
    private Point2D myCenter = new();
    //! Flags for storage the validity of solutions. Indexes corresponds to indexes
    //! in sequences myV, myD.
    private readonly NCollection_Sequence<bool> myValid = new();
    private readonly NCollection_Sequence<int> myNear = new();

    //! Creates a point on a first curve by parameter on this curve.
    public FilletPoint(double theParam)
    {
        myParam = theParam;
        myParam2 = 0.0;
    }

    //! Changes the point position by changing point parameter on the first curve.
    public void setParam(double theParam) { myParam = theParam; }

    //! Returns the point parameter on the first curve.
    public double getParam() { return myParam; }

    //! Returns number of found values of function in this point.
    public int getNBValues() { return myV.Length(); }

    //! Returns value of function in this point.
    public double getValue(int theIndex) { return myV.Value(theIndex); }

    //! Returns derivatives of function in this point.
    public double getDiff(int theIndex) { return myD.Value(theIndex); }

    //! Returns true if function is valid (rediuses vectors of fillet do not intersect any curve).
    public bool isValid(int theIndex) { return myValid.Value(theIndex); }

    //! Returns the index of the nearest value
    public int getNear(int theIndex) { return myNear.Value(theIndex); }

    //! Defines the parameter of the projected point on the second curve.
    public void setParam2(double theParam2) { myParam2 = theParam2; }

    //! Returns the parameter of the projected point on the second curve.
    public double getParam2() { return myParam2; }

    //! Center of the fillet.
    public void setCenter(Point2D thePoint) { myCenter = thePoint; }

    //! Center of the fillet.
    public Point2D getCenter() { return myCenter; }

    //! For debug only
    public double LowerValue()
    {
        int a, aResultIndex = 0;
        double aValue = 0.0; // set before it is read, as in OCCT
        for (a = myV.Length(); a > 0; a--)
        {
            if (aResultIndex == 0 || Math.Abs(aValue) > Math.Abs(myV.Value(a)))
            {
                aResultIndex = a;
                aValue = myV.Value(a);
            }
        }
        return aValue;
    }

    //! Appends value of the function.
    public void appendValue(double theValue, bool theValid)
    {
        int a;
        for (a = 1; a <= myV.Length(); a++)
        {
            if (theValue < myV.Value(a))
            {
                myV.InsertBefore(a, theValue);
                myValid.InsertBefore(a, theValid);
                return;
            }
        }
        myV.Append(theValue);
        myValid.Append(theValid);
    }

    //! Computes difference between this point and the given. Stores difference in myD.
    public bool calculateDiff(FilletPoint thePoint)
    {
        int a;
        bool aDiffsSet = (myD.Length() != 0);
        double aDX = thePoint.getParam() - myParam, aDY = 0.0;
        if (thePoint.myV.Length() == myV.Length())
        { // absolutely the same points
            for (a = 1; a <= myV.Length(); a++)
            {
                aDY = thePoint.myV.Value(a) - myV.Value(a);
                if (aDiffsSet)
                {
                    myD.SetValue(a, aDY / aDX);
                }
                else
                {
                    myD.Append(aDY / aDX);
                }
            }
            return true;
        }
        // between the diffeerent points searching for nearest analogs
        int b;
        for (a = 1; a <= myV.Length(); a++)
        {
            for (b = 1; b <= thePoint.myV.Length(); b++)
            {
                if (b == 1 || Math.Abs(thePoint.myV.Value(b) - myV.Value(a)) < Math.Abs(aDY))
                {
                    aDY = thePoint.myV.Value(b) - myV.Value(a);
                }
            }
            if (aDiffsSet)
            {
                if (Math.Abs(aDY / aDX) < Math.Abs(myD.Value(a)))
                {
                    myD.SetValue(a, aDY / aDX);
                }
            }
            else
            {
                myD.Append(aDY / aDX);
            }
        } // for

        return false;
    }

    //! Filters out the values and leaves the most optimal one.
    public void FilterPoints(FilletPoint thePoint)
    {
        int a, b;
        NCollection_Sequence<double> aDiffs = new();
        double aY, aY2, aDX = thePoint.getParam() - myParam;
        for (a = 1; a <= myV.Length(); a++)
        {
            // searching for near point from thePoint
            int aNear = 0;
            double aDiff = aDX * 10000.0;
            aY = myV.Value(a) + myD.Value(a) * aDX;
            for (b = 1; b <= thePoint.myV.Length(); b++)
            {
                // calculate hypothesis value of the Y2 with the constant first and second derivative
                aY2 = aY + aDX * (thePoint.myD.Value(b) - myD.Value(a)) / 2.0;
                if (aNear == 0 || Math.Abs(aY2 - thePoint.myV.Value(b)) < Math.Abs(aDiff))
                {
                    aNear = b;
                    aDiff = aY2 - thePoint.myV.Value(b);
                }
            } // for b...

            if (aNear != 0)
            {
                if (myV.Value(a) * thePoint.myV.Value(aNear) > 0)
                { // the same sign at the same sides of the interval
                    if (myV.Value(a) * myD.Value(a) > 0)
                    {
                        if (Math.Abs(myD.Value(a)) > global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            aNear = 0;
                        }
                    }
                    else
                    {
                        if (Math.Abs(myV.Value(a)) > Math.Abs(thePoint.myV.Value(aNear)))
                        {
                            if (thePoint.myV.Value(aNear) * thePoint.myD.Value(aNear) < 0
                                && Math.Abs(thePoint.myD.Value(aNear)) > global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                aNear = 0;
                            }
                        }
                    }
                }
            } // if aNear

            if (aNear != 0)
            {
                if (myV.Value(a) * thePoint.myV.Value(aNear) > 0)
                {
                    if ((myV.Value(a) + myD.Value(a) * aDX) * myV.Value(a) > global::Kerf.Engine.Foundation.Precision.Confusion()
                        && (thePoint.myV.Value(aNear) + thePoint.myD.Value(aNear) * aDX)
                               * thePoint.myV.Value(aNear)
                           > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        aNear = 0;
                    }
                }
            } // if aNear

            if (aNear != 0)
            {
                if (Math.Abs(aDiff / aDX) > 1.0e+7)
                {
                    aNear = 0;
                }
            }

            if (aNear == 0)
            { // there is no near: remove it from the list
                myV.Remove(a);
                myD.Remove(a);
                myValid.Remove(a);
                a--;
            }
            else
            {
                bool aFound = false;
                for (b = 1; b <= myNear.Length(); b++)
                {
                    if (myNear.Value(b) == aNear)
                    {
                        if (Math.Abs(aDiffs.Value(b)) < Math.Abs(aDiff))
                        { // return this 'near'
                            aFound = true;
                            myV.Remove(a);
                            myD.Remove(a);
                            myValid.Remove(a);
                            a--;
                            break;
                        }
                        else
                        { // remove the old 'near'
                            // OCCT: b indexes myNear, used here to index myV, myD and myValid too
                            myV.Remove(b);
                            myD.Remove(b);
                            myValid.Remove(b);
                            myNear.Remove(b);
                            aDiffs.Remove(b);
                            a--;
                            break;
                        }
                    }
                } // for b...
                if (!aFound)
                {
                    myNear.Append(aNear);
                    aDiffs.Append(aDiff);
                }
            } // else
        } // for a...
    }

    //! Returns a pointer to created copy of the point
    //! warning: this is not the full copy! Copies only: myParam, myV, myD, myValid
    public FilletPoint Copy()
    {
        // OCCT: myNear isn't copied (FilterPoints rebuilds it before PerformNewton reads it)
        var aCopy = new FilletPoint(myParam);
        int a;
        for (a = 1; a <= myV.Length(); a++)
        {
            aCopy.myV.Append(myV.Value(a));
            aCopy.myD.Append(myD.Value(a));
            aCopy.myValid.Append(myValid.Value(a));
        }
        return aCopy;
    }

    //! Returns the index of the solution or zero if there is no solution
    public int hasSolution(double theRadius)
    {
        int a;
        for (a = 1; a <= myV.Length(); a++)
        {
            if (Math.Abs(Math.Sqrt(Math.Abs(Math.Abs(myV.Value(a)) + theRadius * theRadius)) - theRadius)
                < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return a;
            }
        }
        return 0;
    }

    //! Removes the found value by the given index.
    public void remove(int theIndex)
    {
        myV.Remove(theIndex);
        myD.Remove(theIndex);
        myValid.Remove(theIndex);
        myNear.Remove(theIndex);
    }
}
