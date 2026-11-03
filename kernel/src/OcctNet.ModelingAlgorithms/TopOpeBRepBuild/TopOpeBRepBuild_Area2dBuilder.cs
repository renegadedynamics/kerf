// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_Area2dBuilder.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_Area2dBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using static Kerf.Topology.Containment;
using static Occt.TopOpeBRepBuild_LoopEnum;

namespace Occt;

/// <summary>
/// Builds the areas of faces from a <see cref="TopOpeBRepBuild_LoopSet"/> of wires (on the boundary) and blocks of
/// edges (from interferences): as <see cref="TopOpeBRepBuild_AreaBuilder"/>, and the boundary loops left out of every
/// area make one area when there is no other.
/// </summary>
public class TopOpeBRepBuild_Area2dBuilder : TopOpeBRepBuild_AreaBuilder
{
    /// <summary>An empty area builder.</summary>
    public TopOpeBRepBuild_Area2dBuilder()
    {
    }

    /// <summary>The areas of the loops of <paramref name="LS"/>, classified with <paramref name="LC"/>.</summary>
    public TopOpeBRepBuild_Area2dBuilder(TopOpeBRepBuild_LoopSet LS, TopOpeBRepBuild_LoopClassifier LC, bool ForceClass = false)
    {
        // C++ calls the virtual InitAreaBuilder from the constructor, which runs this class's version
        initArea2dBuilder(LS, LC, ForceClass);
    }

    /// <inheritdoc/>
    public override void InitAreaBuilder(TopOpeBRepBuild_LoopSet LS, TopOpeBRepBuild_LoopClassifier LC, bool ForceClass = false)
    {
        initArea2dBuilder(LS, LC, ForceClass);
    }

    private void initArea2dBuilder(TopOpeBRepBuild_LoopSet LS, TopOpeBRepBuild_LoopClassifier LC, bool ForceClass)
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

            // L = shape et ForceClass  : on traite L comme un block
            // L = shape et !ForceClass : on traite L comme un pur shape
            // L = !shape               : on traite L comme un block
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
                        TopOpeBRepBuild_Loop lb = LoopIter.Value();
                        state = LC.Compare(lb, L);
                        if (state == Unknown)
                        {
                            Atomize(ref state, In);
                        }
                        ashapeinside = state == In;
                        if (ashapeinside)
                        {
                            state = LC.Compare(L, lb);
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

        if (!boundaryloops.IsEmpty())
        {
            if (myArea.IsEmpty())
            {
                NCollection_List<TopOpeBRepBuild_Loop> newArea3 = new();
                newArea3.Append(boundaryloops);
                myArea.Append(newArea3);
            }
        }

        InitArea();
    }
}
