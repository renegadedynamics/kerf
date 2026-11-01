// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurveExplorer.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurveExplorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Explores the curves of a data structure in index order, only the kept ones by default.</summary>
/// <remarks>
/// <see cref="Curve()"/> returns a copy: to change the current curve, use
/// <c>ref DS.ChangeCurve(explorer.Index())</c> (OCCT's callers cast the constness away). OCCT's <c>thread_local</c>
/// empty curve, returned when there is none, is a new empty curve here (decision 9902).
/// </remarks>
public sealed class TopOpeBRepDS_CurveExplorer
{
    private int myIndex;
    private int myMax;
    private TopOpeBRepDS_DataStructure? myDS;
    private bool myFound;
    private bool myFindKeep;

    /// <summary>An explorer of nothing.</summary>
    public TopOpeBRepDS_CurveExplorer()
    {
        myIndex = 1;
        myMax = 0;
        myDS = null;
        myFound = false;
        myFindKeep = false;
    }

    /// <summary>An explorer of the curves of <paramref name="DS"/> (only the kept ones if <paramref name="FindKeep"/>).</summary>
    public TopOpeBRepDS_CurveExplorer(TopOpeBRepDS_DataStructure DS, bool FindKeep = true)
    {
        Init(DS, FindKeep);
    }

    /// <summary>Restarts on the curves of <paramref name="DS"/>.</summary>
    public void Init(TopOpeBRepDS_DataStructure DS, bool FindKeep = true)
    {
        myDS = DS;
        myIndex = 1;
        myMax = DS.NbCurves();
        myFindKeep = FindKeep;
        Find();
    }

    private TopOpeBRepDS_DataStructure MYDS => myDS!;

    private void Find()
    {
        myFound = false;
        while (myIndex <= myMax)
        {
            if (myFindKeep)
            {
                myFound = IsCurveKeep(myIndex);
            }
            else
            {
                myFound = IsCurve(myIndex);
            }
            if (myFound)
            {
                break;
            }
            else
            {
                myIndex++;
            }
        }
    }

    /// <summary>Whether there is a current curve.</summary>
    public bool More() => myFound;

    /// <summary>Moves to the next curve.</summary>
    public void Next()
    {
        myIndex++;
        Find();
    }

    /// <summary>The current curve (an empty one if there is none).</summary>
    public TopOpeBRepDS_Curve Curve()
    {
        if (myFound)
        {
            return MYDS.Curve(myIndex);
        }
        else
        {
            return new TopOpeBRepDS_Curve();
        }
    }

    /// <summary>Whether the data structure has a curve <paramref name="I"/> (removed curves included).</summary>
    public bool IsCurve(int I)
    {
        bool b = MYDS.myCurves.IsBound(I);
        return b;
    }

    /// <summary>Whether the data structure has a kept curve <paramref name="I"/>.</summary>
    public bool IsCurveKeep(int I)
    {
        bool b = MYDS.myCurves.IsBound(I);
        if (b)
        {
            b = MYDS.Curve(I).Keep();
        }
        return b;
    }

    /// <summary>The curve <paramref name="I"/> (an empty one if there is none).</summary>
    public TopOpeBRepDS_Curve Curve(int I)
    {
        if (IsCurve(I))
        {
            TopOpeBRepDS_Curve C = MYDS.Curve(I);
            return C;
        }
        return new TopOpeBRepDS_Curve();
    }

    /// <summary>The number of curves the explorer finds; it restarts it and leaves it at the end, as OCCT does.</summary>
    public int NbCurve()
    {
        myIndex = 1;
        myMax = MYDS.NbCurves();
        Find();
        int n = 0;
        for (; More(); Next())
        {
            n++;
        }
        return n;
    }

    /// <summary>The index of the current curve.</summary>
    public int Index() => myIndex;
}
