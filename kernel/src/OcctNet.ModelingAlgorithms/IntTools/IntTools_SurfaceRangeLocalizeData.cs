// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_SurfaceRangeLocalizeData.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_SurfaceRangeLocalizeData.lxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_SurfaceRangeLocalizeData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The localisation data of a surface for the edge/face intersector: the numbers of samples and minimal ranges in U
/// and V, the sample ranges found out of the edge, their boxes, and a grid of points with its frame.
/// </summary>
/// <remarks>
/// <para>
/// A class, a working object (IntTools_Context caches one per face; plan section 6), because it holds maps and
/// arrays. OCCT's copy constructor and <see cref="Assign"/> copy the maps but share the grid arrays (handles), as here.
/// As in OCCT, <see cref="Assign"/> keeps this object's grid when the other has none.
/// </para>
/// <para>
/// <see cref="ListRangeOut"/> lists the out ranges in insertion order, where OCCT lists them in its hash order
/// (decision 629).
/// </para>
/// </remarks>
public sealed class IntTools_SurfaceRangeLocalizeData
{
    private int myNbSampleU;
    private int myNbSampleV;
    private double myMinRangeU;
    private double myMinRangeV;
    private NCollection_Map<IntTools_SurfaceRangeSample> myMapRangeOut = new();
    private NCollection_DataMap<IntTools_SurfaceRangeSample, Bnd_Box> myMapBox = new();
    private NCollection_HArray1<double>? myUParams;
    private NCollection_HArray1<double>? myVParams;
    private NCollection_HArray2<gp_Pnt>? myGridPoints;
    private int myUIndMin;
    private int myUIndMax;
    private int myVIndMin;
    private int myVIndMax;
    private double myDeflection;

    public IntTools_SurfaceRangeLocalizeData()
    {
        myNbSampleU = 1;
        myNbSampleV = 1;
        myMinRangeU = 0.0;
        myMinRangeV = 0.0;
        myDeflection = 0.0;
        myUIndMin = 0;
        myUIndMax = 0;
        myVIndMin = 0;
        myVIndMax = 0;
    }

    public IntTools_SurfaceRangeLocalizeData(int theNbSampleU, int theNbSampleV, double theMinRangeU, double theMinRangeV)
    {
        myNbSampleU = theNbSampleU;
        myNbSampleV = theNbSampleV;
        myMinRangeU = theMinRangeU;
        myMinRangeV = theMinRangeV;
        myDeflection = 0.0;
        myUIndMin = 0;
        myUIndMax = 0;
        myVIndMin = 0;
        myVIndMax = 0;
    }

    /// <summary>A copy (OCCT's copy constructor): the maps copied, the grid arrays shared.</summary>
    public IntTools_SurfaceRangeLocalizeData(IntTools_SurfaceRangeLocalizeData Other)
    {
        Assign(Other);
    }

    /// <summary>OCCT's assignment: the maps copied, the grid arrays shared (kept if the other has none).</summary>
    internal IntTools_SurfaceRangeLocalizeData Assign(IntTools_SurfaceRangeLocalizeData Other)
    {
        myNbSampleU = Other.myNbSampleU;
        myNbSampleV = Other.myNbSampleV;
        myMinRangeU = Other.myMinRangeU;
        myMinRangeV = Other.myMinRangeV;
        myMapRangeOut.Clear();
        myMapRangeOut = new NCollection_Map<IntTools_SurfaceRangeSample>(Other.myMapRangeOut);
        myMapBox.Clear();
        myMapBox = new NCollection_DataMap<IntTools_SurfaceRangeSample, Bnd_Box>(Other.myMapBox);
        myDeflection = Other.myDeflection;
        myUIndMin = Other.myUIndMin;
        myUIndMax = Other.myUIndMax;
        myVIndMin = Other.myVIndMin;
        myVIndMax = Other.myVIndMax;

        if (Other.myUParams is not null)
        {
            myUParams = Other.myUParams;
        }

        if (Other.myVParams is not null)
        {
            myVParams = Other.myVParams;
        }

        if (Other.myGridPoints is not null)
        {
            myGridPoints = Other.myGridPoints;
        }
        return this;
    }

    public int GetNbSampleU() => myNbSampleU;

    public int GetNbSampleV() => myNbSampleV;

    public double GetMinRangeU() => myMinRangeU;

    public double GetMinRangeV() => myMinRangeV;

    /// <summary>Records the range as out (its box is kept, as in OCCT).</summary>
    public void AddOutRange(in IntTools_SurfaceRangeSample theRange)
    {
        myMapRangeOut.Add(theRange);
        //   myMapBox.UnBind(theRange);
    }

    public void AddBox(in IntTools_SurfaceRangeSample theRange, in Bnd_Box theBox) => myMapBox.Bind(theRange, theBox);

    /// <summary>The box of the range, and true, if recorded; otherwise false, the box left as it was.</summary>
    public bool FindBox(in IntTools_SurfaceRangeSample theRange, ref Bnd_Box theBox)
    {
        if (myMapBox.Find(theRange, out Bnd_Box aBox))
        {
            theBox = aBox;
            return true;
        }
        return false;
    }

    public bool IsRangeOut(in IntTools_SurfaceRangeSample theRange) => myMapRangeOut.Contains(theRange);

    /// <summary>Appends the out ranges to the list.</summary>
    public void ListRangeOut(NCollection_List<IntTools_SurfaceRangeSample> theList)
    {
        foreach (IntTools_SurfaceRangeSample aRange in myMapRangeOut)
        {
            theList.Append(aRange);
        }
    }

    public void RemoveRangeOutAll() => myMapRangeOut.Clear();

    /// <summary>Sets the grid deflection.</summary>
    public void SetGridDeflection(double theDeflection) => myDeflection = theDeflection;

    /// <summary>The grid deflection.</summary>
    public double GetGridDeflection() => myDeflection;

    /// <summary>Sets the number of U parameters of the grid (resets the frame; reallocates if it changes).</summary>
    public void SetRangeUGrid(int theNbUGrid)
    {
        myUIndMin = 0;
        myUIndMax = 0;
        myVIndMin = 0;
        myVIndMax = 0;

        if (myUParams is null || theNbUGrid != myUParams.Length())
        {
            myUParams = new NCollection_HArray1<double>(1, theNbUGrid);

            if (myVParams is not null)
            {
                myGridPoints = NewGrid(theNbUGrid, myVParams.Length());
            }
        }
    }

    /// <summary>The number of U parameters of the grid.</summary>
    public int GetRangeUGrid() => myUParams is null ? 0 : myUParams.Length();

    public void SetUParam(int theIndex, double theUParam) => myUParams!.SetValue(theIndex, theUParam);

    public double GetUParam(int theIndex) => myUParams!.Value(theIndex);

    /// <summary>Sets the number of V parameters of the grid (resets the frame; reallocates if it changes).</summary>
    public void SetRangeVGrid(int theNbVGrid)
    {
        myUIndMin = 0;
        myUIndMax = 0;
        myVIndMin = 0;
        myVIndMax = 0;

        if (myVParams is null || theNbVGrid != myVParams.Length())
        {
            myVParams = new NCollection_HArray1<double>(1, theNbVGrid);

            if (myUParams is not null)
            {
                myGridPoints = NewGrid(myUParams.Length(), theNbVGrid);
            }
        }
    }

    /// <summary>The number of V parameters of the grid.</summary>
    public int GetRangeVGrid() => myVParams is null ? 0 : myVParams.Length();

    public void SetVParam(int theIndex, double theVParam) => myVParams!.SetValue(theIndex, theVParam);

    public double GetVParam(int theIndex) => myVParams!.Value(theIndex);

    public void SetGridPoint(int theUIndex, int theVIndex, in gp_Pnt thePoint) =>
        myGridPoints!.SetValue(theUIndex, theVIndex, thePoint);

    public gp_Pnt GetGridPoint(int theUIndex, int theVIndex) => myGridPoints!.Value(theUIndex, theVIndex);

    /// <summary>
    /// Sets the frame: the grid indices strictly inside [<paramref name="theUMin"/>, <paramref name="theUMax"/>] ×
    /// [<paramref name="theVMin"/>, <paramref name="theVMax"/>].
    /// </summary>
    public void SetFrame(double theUMin, double theUMax, double theVMin, double theVMax)
    {
        myUIndMin = 0;
        myUIndMax = 0;
        myVIndMin = 0;
        myVIndMax = 0;

        if (myUParams is null || myVParams is null)
        {
            return;
        }

        int i;
        int aLmI;
        int aLen = myUParams.Length();

        // Compute frame along U.
        for (i = 1; i <= aLen; i++)
        {
            if (myUIndMin == 0 && theUMin < myUParams.Value(i))
            {
                myUIndMin = i;
            }

            aLmI = aLen - i + 1;

            if (myUIndMax == 0 && theUMax > myUParams.Value(aLmI))
            {
                myUIndMax = aLmI;
            }
        }

        if (myUIndMin == 0)
        {
            myUIndMin = aLen + 1;
        }

        // Compute frame along V.
        aLen = myVParams.Length();

        for (i = 1; i <= aLen; i++)
        {
            if (myVIndMin == 0 && theVMin < myVParams.Value(i))
            {
                myVIndMin = i;
            }

            aLmI = aLen - i + 1;

            if (myVIndMax == 0 && theVMax > myVParams.Value(aLmI))
            {
                myVIndMax = aLmI;
            }
        }

        if (myVIndMin == 0)
        {
            myVIndMin = aLen + 1;
        }
    }

    /// <summary>The number of grid points along U in the frame.</summary>
    public int GetNBUPointsInFrame() => myUIndMax - myUIndMin + 1;

    /// <summary>The number of grid points along V in the frame.</summary>
    public int GetNBVPointsInFrame() => myVIndMax - myVIndMin + 1;

    /// <summary>The grid point at the indices in the frame; the origin if outside.</summary>
    public gp_Pnt GetPointInFrame(int theUIndex, int theVIndex)
    {
        int aFrmUInd = theUIndex + myUIndMin - 1;
        int aFrmVInd = theVIndex + myVIndMin - 1;

        if (myGridPoints is null || aFrmUInd > myUIndMax || aFrmVInd > myVIndMax)
        {
            return gp.Origin();
        }

        return myGridPoints.Value(aFrmUInd, aFrmVInd);
    }

    /// <summary>The U parameter at the index in the frame; <see cref="Precision.Infinite"/> if outside.</summary>
    public double GetUParamInFrame(int theIndex)
    {
        int aFrmInd = theIndex + myUIndMin - 1;

        if (myUParams is null || aFrmInd > myUIndMax)
        {
            return Precision.Infinite();
        }

        return myUParams.Value(aFrmInd);
    }

    /// <summary>The V parameter at the index in the frame; <see cref="Precision.Infinite"/> if outside.</summary>
    public double GetVParamInFrame(int theIndex)
    {
        int aFrmInd = theIndex + myVIndMin - 1;

        if (myVParams is null || aFrmInd > myVIndMax)
        {
            return Precision.Infinite();
        }

        return myVParams.Value(aFrmInd);
    }

    /// <summary>Clears the grid of points.</summary>
    public void ClearGrid()
    {
        myDeflection = 0.0;
        myUIndMin = 0;
        myUIndMax = 0;
        myVIndMin = 0;
        myVIndMax = 0;

        myUParams = null;
        myVParams = null;
        myGridPoints = null;
    }

    // OCCT's NCollection_HArray2<gp_Pnt>(1, NbU, 1, NbV): default-constructed points.
    private static NCollection_HArray2<gp_Pnt> NewGrid(int theNbU, int theNbV) => new(1, theNbU, 1, theNbV);
}
