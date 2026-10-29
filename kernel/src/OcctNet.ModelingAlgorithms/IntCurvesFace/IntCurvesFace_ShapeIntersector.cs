// Port of: src/ModelingAlgorithms/TKTopAlgo/IntCurvesFace/IntCurvesFace_ShapeIntersector.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/IntCurvesFace/IntCurvesFace_ShapeIntersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The intersection of a line or a curve with the faces of a shape: the points IN or ON the faces, sorted along the
/// curve.
/// </summary>
/// <remarks>
/// A working object (decision 21): load a shape, then intersect any number of curves. As in OCCT, a second
/// <see cref="Load"/> adds the new shape's faces to those already loaded. It only reads the shape.
/// </remarks>
public class IntCurvesFace_ShapeIntersector
{
    private bool myIsDone;
    private int myNbFaces;
    private NCollection_Array1<int> myPtrNums = new();
    private NCollection_Array1<int> myPtrIndexNums = new();
    private readonly NCollection_Sequence<IntCurvesFace_Intersector> myIntersector = new();
    private readonly NCollection_Sequence<int> myIndexPt = new();
    private readonly NCollection_Sequence<int> myIndexFace = new();
    private readonly NCollection_Sequence<int> myIndexIntPnt = new();
    private readonly NCollection_Sequence<double> myIndexPar = new();

    /// <summary>An empty intersector.</summary>
    public IntCurvesFace_ShapeIntersector()
    {
        myIsDone = false;
        myNbFaces = 0;
    }

    /// <summary>Loads the faces of <paramref name="theShape"/>, with the tolerance <paramref name="theTol"/> (see <see cref="IntCurvesFace_Intersector"/>).</summary>
    public void Load(in TopoDS_Shape theShape, double theTol)
    {
        var Ex = new TopExp_Explorer();
        int i;
        for (myNbFaces = 0, i = 0, Ex.Init(theShape, TopAbs_FACE); Ex.More(); ++i, Ex.Next())
        {
            ++myNbFaces;
            TopoDS_Face aCurrentFace = TopoDS.Face(Ex.Current());
            myIntersector.Append(new IntCurvesFace_Intersector(aCurrentFace, theTol));
        }
    }

    /// <summary>
    /// Intersects the segment [<paramref name="theParMin"/>, <paramref name="theParMax"/>] of the line
    /// <paramref name="theL"/> (they may be ∓RealLast) with the loaded faces.
    /// </summary>
    public void Perform(in gp_Lin theL, double theParMin, double theParMax)
    {
        myIsDone = false;
        for (int i = 1; i <= myNbFaces; ++i)
        {
            myIntersector[i].Perform(theL, theParMin, theParMax);
        }

        SortResult();
    }

    /// <summary>
    /// As <see cref="Perform(in gp_Lin, double, double)"/>, but each face is intersected only up to the nearest
    /// point found so far; the faces met first are tried first at the next call.
    /// </summary>
    public void PerformNearest(in gp_Lin theL, double theParMin, double theParMax)
    {
        int i = 0;
        if (myNbFaces > 2)
        {
            if (myPtrNums.IsEmpty())
            {
                myPtrNums = new NCollection_Array1<int>(0, myNbFaces - 1);
                myPtrIndexNums = new NCollection_Array1<int>(0, myNbFaces - 1);
                for (; i < myNbFaces; ++i)
                {
                    myPtrNums[i] = 0;
                    myPtrIndexNums[i] = i + 1;
                }
            }
        }

        int anIndexFace = -1;
        double aParMax = theParMax;
        myIsDone = false;
        for (int ii = 1; ii <= myNbFaces; ++ii)
        {
            if (!myPtrNums.IsEmpty())
            {
                i = myPtrIndexNums.Value(ii - 1);
            }
            else
            {
                i = ii;
            }

            IntCurvesFace_Intersector anIntersector = myIntersector[i];
            if (theParMin < aParMax)
            {
                anIntersector.Perform(theL, theParMin, aParMax);
                if (anIntersector.IsDone())
                {
                    int n = anIntersector.NbPnt();
                    for (int j = 1; j <= n; ++j)
                    {
                        double w = anIntersector.WParameter(j);
                        if (w < aParMax)
                        {
                            aParMax = w;
                            anIndexFace = ii - 1;
                        }
                    }
                }
                else
                {
                    myIsDone = false;
                    return;
                }
            }
        }

        if (!myPtrNums.IsEmpty() && anIndexFace >= 0)
        {
            myPtrNums[anIndexFace] += 1;
            int im1;
            for (im1 = anIndexFace - 1, i = anIndexFace;
                 i >= 1 && myPtrNums.Value(i) > myPtrNums.Value(im1);
                 --i, --im1)
            {
                (myPtrIndexNums[i], myPtrIndexNums[im1]) = (myPtrIndexNums[im1], myPtrIndexNums[i]);
                (myPtrNums[i], myPtrNums[im1]) = (myPtrNums[im1], myPtrNums[i]);
            }
        }

        SortResult();
    }

    /// <summary>
    /// Intersects the part [<paramref name="theParMin"/>, <paramref name="theParMax"/>] of the curve
    /// <paramref name="theHCurve"/> with the loaded faces.
    /// </summary>
    public void Perform(Adaptor3d_Curve theHCurve, double theParMin, double theParMax)
    {
        myIsDone = false;
        for (int i = 1; i <= myNbFaces; ++i)
        {
            IntCurvesFace_Intersector anIntersector = myIntersector[i];
            anIntersector.Perform(theHCurve, theParMin, theParMax);
        }

        SortResult();
    }

    /// <summary>True when the intersection has been computed.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>The number of intersection points.</summary>
    public int NbPnt() => myIndexPt.Length();

    /// <summary>The U parameter of the <paramref name="I"/>-th point on its face's surface.</summary>
    public double UParameter(int I)
    {
        IntCurvesFace_Intersector anIntAdaptor = myIntersector[myIndexFace[myIndexPt[I]]];
        return anIntAdaptor.UParameter(myIndexIntPnt[myIndexPt[I]]);
    }

    /// <summary>The V parameter of the <paramref name="I"/>-th point on its face's surface.</summary>
    public double VParameter(int I)
    {
        IntCurvesFace_Intersector anIntAdaptor = myIntersector[myIndexFace[myIndexPt[I]]];
        return anIntAdaptor.VParameter(myIndexIntPnt[myIndexPt[I]]);
    }

    /// <summary>The parameter of the <paramref name="I"/>-th point on the line or curve.</summary>
    public double WParameter(int I)
    {
        IntCurvesFace_Intersector anIntAdaptor = myIntersector[myIndexFace[myIndexPt[I]]];
        return anIntAdaptor.WParameter(myIndexIntPnt[myIndexPt[I]]);
    }

    /// <summary>The <paramref name="I"/>-th point.</summary>
    public gp_Pnt Pnt(int I)
    {
        IntCurvesFace_Intersector anIntAdaptor = myIntersector[myIndexFace[myIndexPt[I]]];
        return anIntAdaptor.Pnt(myIndexIntPnt[myIndexPt[I]]);
    }

    /// <summary>The transition of the line or curve on the surface at the <paramref name="I"/>-th point.</summary>
    public IntCurveSurface_TransitionOnCurve Transition(int I)
    {
        IntCurvesFace_Intersector anIntAdaptor = myIntersector[myIndexFace[myIndexPt[I]]];
        return anIntAdaptor.Transition(myIndexIntPnt[myIndexPt[I]]);
    }

    /// <summary>The state of the <paramref name="I"/>-th point in its face: IN, or ON its boundary.</summary>
    public TopAbs_State State(int I)
    {
        IntCurvesFace_Intersector anIntAdaptor = myIntersector[myIndexFace[myIndexPt[I]]];
        return anIntAdaptor.State(myIndexIntPnt[myIndexPt[I]]);
    }

    /// <summary>The face of the <paramref name="I"/>-th point.</summary>
    public TopoDS_Face Face(int I)
    {
        IntCurvesFace_Intersector anIntAdaptor = myIntersector[myIndexFace[myIndexPt[I]]];
        return anIntAdaptor.Face();
    }

    //-- ================================================================================
    //-- myIntersector   : Sequence of the addresses
    //-- myIndexPt          : 1 2 3 .... n  Points before the sorting
    //-- myNumberFace       : Number of the face (of the intersector) of the point myIndexPt(i)
    //-- myNumberIntPnt     : Number of the point  myIndexPt(i) of the intersection
    // myNumberFace(myIndexPt(i))
    //-- myIndexPar         : Parameter W of point myIndexPt(i)
    //--
    //-- To sum up, for each point index of K = myIndexPt(i) on a
    //--      * the face to which it belongs                               : myNumberFace(K)
    //--      * the number of the point in the intersection for FaceCurve  : myNumberIntPnt(K)
    //--      * the parameter W of the point on the curve                  : myIndexPar(K)
    //--
    //-- SortResult Sorts the points in ascending order of W
    //-- (updating the index table TabPt(.))
    //--
    //-- ================================================================================

    /// <summary>Internal: collects the faces' points and sorts them along the curve.</summary>
    public void SortResult()
    {
        myIsDone = true;
        int aNbPnt = 0;
        myIndexPt.Clear();
        myIndexFace.Clear();
        myIndexIntPnt.Clear();
        myIndexPar.Clear();

        // Retrieval of the results
        for (int f = 1; f <= myNbFaces; ++f)
        {
            IntCurvesFace_Intersector anIntersector = myIntersector[f];
            if (anIntersector.IsDone())
            {
                int n = anIntersector.NbPnt();
                for (int j = 1; j <= n; ++j)
                {
                    myIndexPt.Append(++aNbPnt);
                    myIndexFace.Append(f);
                    myIndexIntPnt.Append(j);
                    myIndexPar.Append(anIntersector.WParameter(j));
                }
            }
            else
            {
                myIsDone = false;
                return;
            }
        }

        // Sort according to parameter  w
        bool isOK;
        do
        {
            isOK = true;
            for (int ind0 = 1; ind0 < aNbPnt; ind0++)
            {
                int ind = myIndexPt[ind0];
                int indp1 = myIndexPt[ind0 + 1];
                if (myIndexPar[ind] > myIndexPar[indp1])
                {
                    myIndexPt[ind0] = indp1;
                    myIndexPt[ind0 + 1] = ind;
                    isOK = false;
                }
            }
        } while (!isOK);
    }
}
