// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_AreaBuilder.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_AreaBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using static Kerf.Topology.Containment;
using static Occt.TopOpeBRepBuild_LoopEnum;

namespace Occt;

/// <summary>
/// Rebuilds areas (faces, solids) from loops: the loops of a <see cref="TopOpeBRepBuild_LoopSet"/> (wires or shells,
/// on the boundary, and blocks of elements, from interferences) are grouped into areas with a
/// <see cref="TopOpeBRepBuild_LoopClassifier"/>. The result is an iteration on the areas, each a list of loops.
/// </summary>
/// <remarks>
/// A working object. OCCT's <c>ADD_Loop_TO_LISTOFLoop</c> and the other list helpers take a debugging <c>void*</c>;
/// the port drops it. Appending a list to another moves its loops, as NCollection_List::Append(list) does.
/// </remarks>
public class TopOpeBRepBuild_AreaBuilder
{
    protected readonly NCollection_List<NCollection_List<TopOpeBRepBuild_Loop>> myArea = new();
    protected readonly NCollection_List<NCollection_List<TopOpeBRepBuild_Loop>>.Iterator myAreaIterator = new();
    protected NCollection_List<TopOpeBRepBuild_Loop>.Iterator myLoopIterator = new();
    protected bool myUNKNOWNRaise;

    /// <summary>An empty area builder.</summary>
    public TopOpeBRepBuild_AreaBuilder()
    {
        myUNKNOWNRaise = false; // no raise if UNKNOWN state found
    }

    /// <summary>The areas of the loops of <paramref name="LS"/>, classified with <paramref name="LC"/>.</summary>
    public TopOpeBRepBuild_AreaBuilder(TopOpeBRepBuild_LoopSet LS, TopOpeBRepBuild_LoopClassifier LC, bool ForceClass = false)
    {
        myUNKNOWNRaise = false; // no raise if UNKNOWN state found
// C++ calls the virtual InitAreaBuilder from the constructor, which runs this class's version
        initAreaBuilder(LS, LC, ForceClass);
    }

    /// <summary>
    /// The state of <paramref name="L"/> compared with the loops of <paramref name="LOL"/> of the kind
    /// <paramref name="what"/>: OUT at the first loop it is out of.
    /// </summary>
    protected Containment CompareLoopWithListOfLoop(TopOpeBRepBuild_LoopClassifier LC,
                                                     TopOpeBRepBuild_Loop L,
                                                     NCollection_List<TopOpeBRepBuild_Loop> LOL,
                                                     TopOpeBRepBuild_LoopEnum what)
    {
        Containment state = Unknown;
        bool totest; // L must or not be tested
        NCollection_List<TopOpeBRepBuild_Loop>.Iterator LoopIter = new();

        if (LOL.IsEmpty())
        {
            return Out;
        }

        for (LoopIter.Initialize(LOL); LoopIter.More(); LoopIter.Next())
        {
            TopOpeBRepBuild_Loop curL = LoopIter.Value();
            switch (what)
            {
                case TopOpeBRepBuild_ANYLOOP:
                    totest = true;
                    break;
                case TopOpeBRepBuild_BOUNDARY:
                    totest = curL.IsShape();
                    break;
                case TopOpeBRepBuild_BLOCK:
                    totest = !curL.IsShape();
                    break;
                default:
                    totest = false;
                    break;
            }
            if (totest)
            {
                state = LC.Compare(L, curL);
                if (state == Out)
                {
                    // <L> is out of at least one Loop of <LOL> : stop to explore
                    break;
                }
            }
        }

        return state;
    }

    /// <summary>Replaces an UNKNOWN state by <paramref name="newstate"/> (or throws, if unknown states raise).</summary>
    protected void Atomize(ref Containment state, Containment newstate)
    {
        if (myUNKNOWNRaise)
        {
            Standard_DomainError_Raise_if(state == Unknown, "AreaBuilder : Position Unknown");
        }
        else
        {
            state = newstate;
        }
    }

    /// <summary>
    /// Builds the areas of the loops of <paramref name="LS"/>, classified with <paramref name="LC"/>; with
    /// <paramref name="ForceClass"/>, shape loops are classified as blocks are.
    /// </summary>
    public virtual void InitAreaBuilder(TopOpeBRepBuild_LoopSet LS, TopOpeBRepBuild_LoopClassifier LC, bool ForceClass = false)
    {
        initAreaBuilder(LS, LC, ForceClass);
    }

    private void initAreaBuilder(TopOpeBRepBuild_LoopSet LS, TopOpeBRepBuild_LoopClassifier LC, bool ForceClass)
    {
        Containment state;
        bool Loopinside;
        bool loopoutside;

        NCollection_List<NCollection_List<TopOpeBRepBuild_Loop>>.Iterator AreaIter = new();
        NCollection_List<TopOpeBRepBuild_Loop>.Iterator LoopIter = new();
        // boundaryloops : list of boundary loops out of the areas.
        NCollection_List<TopOpeBRepBuild_Loop> boundaryloops = new();

        myArea.Clear(); // Clear the list of Area to be built

        for (LS.InitLoop(); LS.MoreLoop(); LS.NextLoop())
        {
            // process a new loop : L is the new current Loop
            TopOpeBRepBuild_Loop L = LS.Loop();
            bool boundaryL = L.IsShape();

            // L = Shape et ForceClass  : on traite L comme un block
            // L = Shape et !ForceClass : on traite L comme un pur Shape
            // L = !Shape               : on traite L comme un block
            bool traitercommeblock = (!boundaryL) || ForceClass;
            if (!traitercommeblock)
            {
                // the loop L is a boundary loop :
                // - try to insert it in an existing area, such as L is inside all
                //   the block loops. Only block loops of the area are compared.
                // - if L could not be inserted, store it in list of boundary loops.

                Loopinside = false;
                for (AreaIter.Initialize(myArea); AreaIter.More(); AreaIter.Next())
                {
                    NCollection_List<TopOpeBRepBuild_Loop> aArea = AreaIter.Value();
                    if (aArea.IsEmpty())
                    {
                        continue;
                    }
                    state = CompareLoopWithListOfLoop(LC, L, aArea, TopOpeBRepBuild_BLOCK);
                    if (state == Unknown)
                    {
                        Atomize(ref state, In);
                    }
                    Loopinside = state == In;
                    if (Loopinside)
                    {
                        break;
                    }
                } // end of Area scan

                if (Loopinside)
                {
                    NCollection_List<TopOpeBRepBuild_Loop> aArea = AreaIter.Value();
                    ADD_Loop_TO_LISTOFLoop(L, aArea);
                }
                else if (!Loopinside)
                {
                    ADD_Loop_TO_LISTOFLoop(L, boundaryloops);
                }
            } // end of boundary loop

            else
            {
                // the loop L is a block loop
                // if L is IN theArea :
                //   - stop area scan, insert L in theArea.
                //   - remove from the area all the loops outside L
                //   - make a new area with them, unless they are all boundary
                //   - if they are all boundary put them back in boundaryLoops
                // else :
                //   - create a new area with L.
                //   - insert boundary loops that are IN the new area
                //     (and remove them from 'boundaryloops')

                Loopinside = false;
                for (AreaIter.Initialize(myArea); AreaIter.More(); AreaIter.Next())
                {
                    NCollection_List<TopOpeBRepBuild_Loop> aArea = AreaIter.Value();
                    if (aArea.IsEmpty())
                    {
                        continue;
                    }
                    state = CompareLoopWithListOfLoop(LC, L, aArea, TopOpeBRepBuild_ANYLOOP);
                    if (state == Unknown)
                    {
                        Atomize(ref state, In);
                    }
                    Loopinside = state == In;
                    if (Loopinside)
                    {
                        break;
                    }
                } // end of Area scan

                if (Loopinside)
                {
                    NCollection_List<TopOpeBRepBuild_Loop> aArea = AreaIter.Value();
                    bool allShape = true;
                    NCollection_List<TopOpeBRepBuild_Loop> removedLoops = new();
                    LoopIter.Initialize(aArea);
                    while (LoopIter.More())
                    {
                        state = LC.Compare(LoopIter.Value(), L);
                        if (state == Unknown)
                        {
                            Atomize(ref state, In); // not OUT
                        }
                        loopoutside = state == Out;
                        if (loopoutside)
                        {
                            TopOpeBRepBuild_Loop curL = LoopIter.Value();
                            // remove the loop from the area
                            ADD_Loop_TO_LISTOFLoop(curL, removedLoops);

                            allShape = allShape && curL.IsShape();
                            REM_Loop_FROM_LISTOFLoop(LoopIter, AreaIter.Value());
                        }
                        else
                        {
                            LoopIter.Next();
                        }
                    }
                    // insert the loop in the area
                    ADD_Loop_TO_LISTOFLoop(L, aArea);
                    if (!removedLoops.IsEmpty())
                    {
                        if (allShape)
                        {
                            ADD_LISTOFLoop_TO_LISTOFLoop(removedLoops, boundaryloops);
                        }
                        else
                        {
                            // make a new area with the removed loops
                            NCollection_List<TopOpeBRepBuild_Loop> thelist = new();
                            myArea.Append(thelist);
                            ADD_LISTOFLoop_TO_LISTOFLoop(removedLoops, myArea.Last());
                        }
                    }
                } // Loopinside == True

                else
                {
                    bool ashapeinside, ablockinside;
                    NCollection_List<TopOpeBRepBuild_Loop> thelist1 = new();
                    myArea.Append(thelist1);
                    NCollection_List<TopOpeBRepBuild_Loop> newArea0 = myArea.Last();
                    ADD_Loop_TO_LISTOFLoop(L, newArea0);

                    LoopIter.Initialize(boundaryloops);
                    while (LoopIter.More())
                    {
                        ashapeinside = ablockinside = false;
                        state = LC.Compare(LoopIter.Value(), L);
                        if (state == Unknown)
                        {
                            Atomize(ref state, In);
                        }
                        ashapeinside = state == In;
                        if (ashapeinside)
                        {
                            state = LC.Compare(L, LoopIter.Value());
                            if (state == Unknown)
                            {
                                Atomize(ref state, In);
                            }
                            ablockinside = state == In;
                        }
                        if (ashapeinside && ablockinside)
                        {
                            TopOpeBRepBuild_Loop curL = LoopIter.Value();
                            ADD_Loop_TO_LISTOFLoop(curL, newArea0);

                            REM_Loop_FROM_LISTOFLoop(LoopIter, boundaryloops);
                        }
                        else
                        {
                            LoopIter.Next();
                        }
                    } // end of boundaryloops scan
                } // Loopinside == False
            } // end of block loop
        } // end of LoopSet LS scan

        InitArea();
    }

    /// <summary>Starts the iteration on the areas; their number.</summary>
    public int InitArea()
    {
        myAreaIterator.Initialize(myArea);
        InitLoop();
        int n = myArea.Extent();
        return n;
    }

    /// <summary>Whether there is a current area.</summary>
    public bool MoreArea()
    {
        bool b = myAreaIterator.More();
        return b;
    }

    /// <summary>Moves to the next area.</summary>
    public void NextArea()
    {
        myAreaIterator.Next();
        InitLoop();
    }

    /// <summary>Starts the iteration on the loops of the current area; their number.</summary>
    public int InitLoop()
    {
        int n = 0;
        if (myAreaIterator.More())
        {
            NCollection_List<TopOpeBRepBuild_Loop> LAL = myAreaIterator.Value();
            myLoopIterator.Initialize(LAL);
            n = LAL.Extent();
        }
        else
        { // Create an empty ListIteratorOfListOfLoop
            myLoopIterator = new NCollection_List<TopOpeBRepBuild_Loop>.Iterator();
        }
        return n;
    }

    /// <summary>Whether there is a current loop in the current area.</summary>
    public bool MoreLoop()
    {
        bool b = myLoopIterator.More();
        return b;
    }

    /// <summary>Moves to the next loop of the current area.</summary>
    public void NextLoop()
    {
        myLoopIterator.Next();
    }

    /// <summary>The current loop of the current area.</summary>
    public TopOpeBRepBuild_Loop Loop()
    {
        TopOpeBRepBuild_Loop L = myLoopIterator.Value();
        return L;
    }

    /// <summary>Appends <paramref name="L"/> to <paramref name="LOL"/>.</summary>
    public virtual void ADD_Loop_TO_LISTOFLoop(TopOpeBRepBuild_Loop L, NCollection_List<TopOpeBRepBuild_Loop> LOL)
    {
        LOL.Append(L);
    }

    /// <summary>Removes the loop of <paramref name="ITA"/> from <paramref name="A"/>; the iterator moves to the next.</summary>
    public virtual void REM_Loop_FROM_LISTOFLoop(NCollection_List<TopOpeBRepBuild_Loop>.Iterator ITA,
                                                 NCollection_List<TopOpeBRepBuild_Loop> A)
    {
        A.Remove(ITA);
    }

    /// <summary>Moves the loops of <paramref name="A1"/> to the end of <paramref name="A2"/>.</summary>
    public virtual void ADD_LISTOFLoop_TO_LISTOFLoop(NCollection_List<TopOpeBRepBuild_Loop> A1,
                                                     NCollection_List<TopOpeBRepBuild_Loop> A2)
    {
        A2.Append(A1);
    }
}
