// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ShapeProximity.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ShapeProximity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using ProxPnt_Status = Occt.BRepExtrema_ProximityDistTool.ProxPnt_Status;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Shape proximity from the shapes' tessellations. With a finite tolerance, finds the faces of two shapes that
/// intersect (tolerance zero) or come within the tolerance of each other. With an infinite tolerance (the default),
/// computes the proximity value of two edges or two faces: the thickness of the thinnest layer holding both.
/// The meshes must exist already; results are approximate and depend on them.
/// </summary>
/// <remarks>
/// A working object (decision 21); it only reads the shapes' meshes. As in OCCT, computing the proximity value sets
/// the tolerance to it, so a later Load switches to the overlap test.
/// </remarks>
public sealed class BRepExtrema_ShapeProximity
{
    private double myTolerance;
    private bool myIsInitS1;
    private bool myIsInitS2;
    private readonly NCollection_DynamicArray<TopoDS_Shape> myShapeList1 = new();
    private readonly NCollection_DynamicArray<TopoDS_Shape> myShapeList2 = new();
    private BRepExtrema_TriangleSet myElementSet1;
    private BRepExtrema_TriangleSet myElementSet2;
    private int myNbSamples1;
    private int myNbSamples2;
    private gp_Pnt myProxPoint1;
    private gp_Pnt myProxPoint2;
    private ProxPnt_Status myProxPntStatus1, myProxPntStatus2;
    private readonly BRepExtrema_OverlapTool myOverlapTool = new();
    private readonly BRepExtrema_ProximityValueTool myProxValTool = new();

    /// <summary>An empty proximity tool.</summary>
    public BRepExtrema_ShapeProximity(double theTolerance = 2.0e+100) // Precision::Infinite()
    {
        myTolerance = theTolerance;
        myElementSet1 = new BRepExtrema_TriangleSet();
        myElementSet2 = new BRepExtrema_TriangleSet();
        myNbSamples1 = 0;
        myNbSamples2 = 0;
        // Should be initialized later
        myIsInitS1 = myIsInitS2 = false;
    }

    /// <summary>A proximity tool for the two shapes.</summary>
    public BRepExtrema_ShapeProximity(in TopoDS_Shape theShape1, in TopoDS_Shape theShape2, double theTolerance = 2.0e+100)
    {
        myTolerance = theTolerance;
        myElementSet1 = new BRepExtrema_TriangleSet();
        myElementSet2 = new BRepExtrema_TriangleSet();
        myNbSamples1 = 0;
        myNbSamples2 = 0;
        LoadShape1(theShape1);
        LoadShape2(theShape2);
    }

    /// <summary>The tolerance of the overlap test (the distance between the shapes).</summary>
    public double Tolerance() => myTolerance;

    /// <summary>Sets the tolerance of the overlap test.</summary>
    public void SetTolerance(double theTolerance) => myTolerance = theTolerance;

    /// <summary>The proximity value of the whole shapes.</summary>
    public double Proximity() => Tolerance();

    /// <summary>Sets the number of sample points on the first shape (0: every triangulation node).</summary>
    public void SetNbSamples1(int theNbSamples) => myNbSamples1 = theNbSamples;

    /// <summary>Sets the number of sample points on the second shape (0: every triangulation node).</summary>
    public void SetNbSamples2(int theNbSamples) => myNbSamples2 = theNbSamples;

    /// <summary>True if the search is completed.</summary>
    public bool IsDone() => myOverlapTool.IsDone() || myProxValTool.IsDone();

    /// <summary>The overlapped faces of the first shape (indices from 0), each with the faces of the second.</summary>
    public NCollection_DataMap<int, NCollection_PackedMapOfInteger> OverlapSubShapes1() => myOverlapTool.OverlapSubShapes1();

    /// <summary>The overlapped faces of the second shape (indices from 0), each with the faces of the first.</summary>
    public NCollection_DataMap<int, NCollection_PackedMapOfInteger> OverlapSubShapes2() => myOverlapTool.OverlapSubShapes2();

    /// <summary>The sub-shape of the first shape with the given index (from 0).</summary>
    public TopoDS_Shape GetSubShape1(int theID) => myShapeList1.Value(theID);

    /// <summary>The sub-shape of the second shape with the given index (from 0).</summary>
    public TopoDS_Shape GetSubShape2(int theID) => myShapeList2.Value(theID);

    /// <summary>The set of all the face triangles of the first shape.</summary>
    public BRepExtrema_TriangleSet ElementSet1() => myElementSet1;

    /// <summary>The set of all the face triangles of the second shape.</summary>
    public BRepExtrema_TriangleSet ElementSet2() => myElementSet2;

    /// <summary>The point of the first shape giving the proximity value.</summary>
    public gp_Pnt ProximityPoint1() => myProxPoint1;

    /// <summary>The point of the second shape giving the proximity value.</summary>
    public gp_Pnt ProximityPoint2() => myProxPoint2;

    /// <summary>The status of the point of the first shape giving the proximity value.</summary>
    public ProxPnt_Status ProxPntStatus1() => myProxPntStatus1;

    /// <summary>The status of the point of the second shape giving the proximity value.</summary>
    public ProxPnt_Status ProxPntStatus2() => myProxPntStatus2;

    // Assign a map of sub-shapes (edges/faces) of a given shape
    private static bool initSubShapes(in TopoDS_Shape theShape,
                                      NCollection_DynamicArray<TopoDS_Shape> theSubshapesList,
                                      ref BRepExtrema_TriangleSet theTriangleSet)
    {
        theSubshapesList.Clear();

        for (var anIter = new TopExp_Explorer(theShape, TopAbs_FACE); anIter.More(); anIter.Next())
        {
            theSubshapesList.Append(anIter.Current());
        }

        for (var anIter = new TopExp_Explorer(theShape, TopAbs_EDGE); anIter.More(); anIter.Next())
        {
            theSubshapesList.Append(anIter.Current());
        }

        theTriangleSet ??= new BRepExtrema_TriangleSet();

        return theTriangleSet.Init(theSubshapesList);
    }

    /// <summary>Loads the first shape.</summary>
    public bool LoadShape1(in TopoDS_Shape theShape1)
    {
        myIsInitS1 = initSubShapes(theShape1, myShapeList1, ref myElementSet1);

        if (myTolerance == Precision.Infinite())
        {
            myProxValTool.MarkDirty();
        }
        else
        {
            myOverlapTool.MarkDirty();
        }

        return myIsInitS1;
    }

    /// <summary>Loads the second shape.</summary>
    public bool LoadShape2(in TopoDS_Shape theShape2)
    {
        myIsInitS2 = initSubShapes(theShape2, myShapeList2, ref myElementSet2);

        if (myTolerance == Precision.Infinite())
        {
            myProxValTool.MarkDirty();
        }
        else
        {
            myOverlapTool.MarkDirty();
        }

        return myIsInitS2;
    }

    /// <summary>Searches for overlapped faces, or computes the proximity value when the tolerance is infinite.</summary>
    public void Perform()
    {
        if (myTolerance == Precision.Infinite())
        {
            if (!myIsInitS1 || !myIsInitS2 || myProxValTool.IsDone())
            {
                return;
            }

            myProxValTool.LoadTriangleSets(myElementSet1, myElementSet2);
            myProxValTool.LoadShapeLists(myShapeList1, myShapeList2);
            myProxValTool.SetNbSamplePoints(myNbSamples1, myNbSamples2);

            // OCCT: the proximity value is written into myTolerance, so the next Load and Perform test overlaps with it
            myProxValTool.Perform(ref myTolerance);
            myProxValTool.ProximityPoints(out myProxPoint1, out myProxPoint2);
            myProxValTool.ProximityPointsStatus(out myProxPntStatus1, out myProxPntStatus2);
        }
        else
        {
            if (!myIsInitS1 || !myIsInitS2 || myOverlapTool.IsDone())
            {
                return;
            }

            myOverlapTool.LoadTriangleSets(myElementSet1, myElementSet2);

            myOverlapTool.Perform(myTolerance);
        }
    }
}
