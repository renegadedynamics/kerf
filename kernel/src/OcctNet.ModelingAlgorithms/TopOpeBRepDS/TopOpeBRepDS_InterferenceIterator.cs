// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_InterferenceIterator.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_InterferenceIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Iterates over a list of interferences, stopping only on those that match the conditions set (geometry kind and
/// index, support kind and index; none by default).
/// </summary>
/// <remarks>A working object, as OCCT's iterators. Setting a condition does not move the iterator: call <see cref="Match"/>.</remarks>
public class TopOpeBRepDS_InterferenceIterator
{
    private readonly NCollection_List<TopOpeBRepDS_Interference>.Iterator myIterator = new();
    private bool myGKDef;
    private TopOpeBRepDS_Kind myGK;
    private bool myGDef;
    private int myG;
    private bool mySKDef;
    private TopOpeBRepDS_Kind mySK;
    private bool mySDef;
    private int myS;

    /// <summary>An iterator over nothing.</summary>
    public TopOpeBRepDS_InterferenceIterator()
    {
        myGKDef = false;
        myGDef = false;
        mySKDef = false;
        mySDef = false;
    }

    /// <summary>An iterator over the interferences of <paramref name="L"/>.</summary>
    public TopOpeBRepDS_InterferenceIterator(NCollection_List<TopOpeBRepDS_Interference> L)
    {
        myGKDef = false;
        myGDef = false;
        mySKDef = false;
        mySDef = false;
        Init(L);
    }

    /// <summary>Restarts on the list <paramref name="L"/>; the conditions are kept.</summary>
    public void Init(NCollection_List<TopOpeBRepDS_Interference> L)
    {
        myIterator.Initialize(L);
        Match();
    }

    /// <summary>The interferences must have the geometry kind <paramref name="GK"/>.</summary>
    public void GeometryKind(TopOpeBRepDS_Kind GK)
    {
        myGKDef = true;
        myGK = GK;
    }

    /// <summary>The interferences must have the geometry index <paramref name="G"/>.</summary>
    public void Geometry(int G)
    {
        myGDef = true;
        myG = G;
    }

    /// <summary>The interferences must have the support kind <paramref name="ST"/>.</summary>
    public void SupportKind(TopOpeBRepDS_Kind ST)
    {
        mySKDef = true;
        mySK = ST;
    }

    /// <summary>The interferences must have the support index <paramref name="S"/>.</summary>
    public void Support(int S)
    {
        mySDef = true;
        myS = S;
    }

    /// <summary>Moves to the first interference, from the current one, that matches the conditions.</summary>
    public void Match()
    {
        while (myIterator.More())
        {
            TopOpeBRepDS_Interference I = myIterator.Value();
            bool b = MatchInterference(I);
            if (!b)
            {
                myIterator.Next();
            }
            else
            {
                break;
            }
        }
    }

    /// <summary>Whether <paramref name="I"/> matches the conditions (true if none is set).</summary>
    public virtual bool MatchInterference(TopOpeBRepDS_Interference I)
    {
        bool GK = myGKDef ? (I.GeometryType() == myGK) : true;
        bool SK = mySKDef ? (I.SupportType() == mySK) : true;
        bool G = myGDef ? (I.Geometry() == myG) : true;
        bool S = mySDef ? (I.Support() == myS) : true;
        return (GK && SK && G && S);
    }

    /// <summary>Whether there is a current interference.</summary>
    public bool More() => myIterator.More();

    /// <summary>Moves to the next matching interference.</summary>
    public void Next()
    {
        if (myIterator.More())
        {
            myIterator.Next();
            Match();
        }
    }

    /// <summary>The current interference.</summary>
    public TopOpeBRepDS_Interference Value() => myIterator.Value();

    /// <summary>The underlying list iterator.</summary>
    internal NCollection_List<TopOpeBRepDS_Interference>.Iterator ChangeIterator() => myIterator;
}
