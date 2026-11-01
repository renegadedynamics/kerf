// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceExplorer.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceExplorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Explores the surfaces of a data structure in index order, only the kept ones by default.</summary>
/// <remarks>
/// A new surface is not kept (<see cref="TopOpeBRepDS_Surface"/>), so with the default <c>FindOnlyKeep</c> the explorer
/// skips the surfaces until something keeps them, as in OCCT.
/// </remarks>
public sealed class TopOpeBRepDS_SurfaceExplorer
{
    private int myIndex;
    private int myMax;
    private TopOpeBRepDS_DataStructure? myDS;
    private bool myFound;
    private readonly TopOpeBRepDS_Surface myEmpty = new();
    private bool myFindKeep;

    /// <summary>An explorer of nothing.</summary>
    /// <remarks>OCCT leaves FindKeep unset here; the port's is false.</remarks>
    public TopOpeBRepDS_SurfaceExplorer()
    {
        myIndex = 1;
        myMax = 0;
        myDS = null;
        myFound = false;
    }

    /// <summary>An explorer of the surfaces of <paramref name="DS"/> (only the kept ones if <paramref name="FindKeep"/>).</summary>
    public TopOpeBRepDS_SurfaceExplorer(TopOpeBRepDS_DataStructure DS, bool FindKeep = true)
    {
        Init(DS, FindKeep);
    }

    /// <summary>Restarts on the surfaces of <paramref name="DS"/>.</summary>
    public void Init(TopOpeBRepDS_DataStructure DS, bool FindKeep = true)
    {
        myIndex = 1;
        myMax = DS.NbSurfaces();
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
                myFound = IsSurfaceKeep(myIndex);
            }
            else
            {
                myFound = IsSurface(myIndex);
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

    /// <summary>Whether there is a current surface.</summary>
    public bool More() => myFound;

    /// <summary>Moves to the next surface.</summary>
    public void Next()
    {
        myIndex++;
        Find();
    }

    /// <summary>The current surface (an empty one if there is none).</summary>
    public TopOpeBRepDS_Surface Surface()
    {
        if (myFound)
        {
            return MYDS.Surface(myIndex);
        }
        else
        {
            return myEmpty;
        }
    }

    /// <summary>Whether the data structure has a surface <paramref name="I"/>.</summary>
    public bool IsSurface(int I)
    {
        bool b = MYDS.mySurfaces.IsBound(I);
        return b;
    }

    /// <summary>Whether the data structure has a kept surface <paramref name="I"/>.</summary>
    public bool IsSurfaceKeep(int I)
    {
        bool b = MYDS.mySurfaces.IsBound(I);
        if (b)
        {
            b = MYDS.Surface(I).Keep();
        }
        return b;
    }

    /// <summary>The surface <paramref name="I"/> (an empty one if there is none).</summary>
    public TopOpeBRepDS_Surface Surface(int I)
    {
        if (IsSurface(I))
        {
            return MYDS.Surface(I);
        }
        else
        {
            return myEmpty;
        }
    }

    /// <summary>The number of surfaces the explorer finds; it restarts it and leaves it at the end, as OCCT does.</summary>
    public int NbSurface()
    {
        myIndex = 1;
        myMax = MYDS.NbSurfaces();
        Find();
        int n = 0;
        for (; More(); Next())
        {
            n++;
        }
        return n;
    }

    /// <summary>The index of the current surface.</summary>
    public int Index() => myIndex;
}
