// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_PointExplorer.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_PointExplorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Explores the points of a data structure in index order, only the kept ones by default.</summary>
/// <remarks><see cref="Point()"/> returns a copy: to change the current point, use <c>ref DS.ChangePoint(explorer.Index())</c>.</remarks>
public sealed class TopOpeBRepDS_PointExplorer
{
    private int myIndex;
    private int myMax;
    private TopOpeBRepDS_DataStructure? myDS;
    private bool myFound;
    private readonly TopOpeBRepDS_Point myEmpty = new();
    private bool myFindKeep;

    /// <summary>An explorer of nothing.</summary>
    public TopOpeBRepDS_PointExplorer()
    {
        myIndex = 1;
        myMax = 0;
        myDS = null;
        myFound = false;
        myFindKeep = false;
    }

    /// <summary>An explorer of the points of <paramref name="DS"/> (only the kept ones if <paramref name="FindKeep"/>).</summary>
    public TopOpeBRepDS_PointExplorer(TopOpeBRepDS_DataStructure DS, bool FindKeep = true)
    {
        Init(DS, FindKeep);
    }

    /// <summary>Restarts on the points of <paramref name="DS"/>.</summary>
    public void Init(TopOpeBRepDS_DataStructure DS, bool FindKeep = true)
    {
        myIndex = 1;
        myMax = DS.NbPoints();
        myDS = DS;
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
                myFound = IsPointKeep(myIndex);
            }
            else
            {
                myFound = IsPoint(myIndex);
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

    /// <summary>Whether there is a current point.</summary>
    public bool More() => myFound;

    /// <summary>Moves to the next point.</summary>
    public void Next()
    {
        myIndex++;
        Find();
    }

    /// <summary>The current point (an empty one if there is none).</summary>
    public TopOpeBRepDS_Point Point()
    {
        if (myFound)
        {
            return MYDS.Point(myIndex);
        }
        else
        {
            return myEmpty;
        }
    }

    /// <summary>Whether the data structure has a point <paramref name="I"/>.</summary>
    public bool IsPoint(int I)
    {
        bool b = MYDS.myPoints.IsBound(I);
        return b;
    }

    /// <summary>Whether the data structure has a kept point <paramref name="I"/>.</summary>
    public bool IsPointKeep(int I)
    {
        bool b = MYDS.myPoints.IsBound(I);
        if (b)
        {
            b = MYDS.Point(I).Keep();
        }
        return b;
    }

    /// <summary>The point <paramref name="I"/> (an empty one if there is none).</summary>
    public TopOpeBRepDS_Point Point(int I)
    {
        if (IsPoint(I))
        {
            return MYDS.Point(I);
        }
        else
        {
            return myEmpty;
        }
    }

    /// <summary>The number of points the explorer finds; it restarts it and leaves it at the end, as OCCT does.</summary>
    public int NbPoint()
    {
        myIndex = 1;
        myMax = MYDS.NbPoints();
        Find();
        int n = 0;
        for (; More(); Next())
        {
            n++;
        }
        return n;
    }

    /// <summary>The index of the current point.</summary>
    public int Index() => myIndex;
}
