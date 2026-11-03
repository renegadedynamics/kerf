// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_Area2dBuilder.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_Area2dBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;
using static Kerf.Topology.Containment;
using static Kerf.Engine.Booleans.Legacy.Build.LoopEnum;

namespace Kerf.Engine.Booleans.Legacy.Build;

/// <summary>
/// Builds the areas of faces from a <see cref="LoopSet"/> of wires (on the boundary) and blocks of
/// edges (from interferences): as <see cref="AreaBuilder"/>, and the boundary loops left out of every
/// area make one area when there is no other.
/// </summary>
public class Area2dBuilder : AreaBuilder
{
    /// <summary>An empty area builder.</summary>
    public Area2dBuilder()
    {
    }

    /// <summary>The areas of the loops of <paramref name="ls"/>, classified with <paramref name="lc"/>.</summary>
    public Area2dBuilder(LoopSet ls, LoopClassifier lc, bool forceClass = false)
    {
        // C++ calls the virtual InitAreaBuilder from the constructor, which runs this class's version
        InitArea2dBuilder(ls, lc, forceClass);
    }

    /// <inheritdoc/>
    public override void InitAreaBuilder(LoopSet ls, LoopClassifier lc, bool forceClass = false)
    {
        InitArea2dBuilder(ls, lc, forceClass);
    }

    private void InitArea2dBuilder(LoopSet ls, LoopClassifier lc, bool forceClass)
    {
        Containment state;
        bool loopinside;
        bool loopoutside;

        NCollection_List<NCollection_List<BuildLoop>>.Iterator areaIter = new();
        NCollection_List<BuildLoop>.Iterator loopIter = new();
        // boundaryloops : list of boundary loops out of the areas.
        NCollection_List<BuildLoop> boundaryloops = new();

        _area.Clear(); // Clear the list of Area to be built

        for (ls.InitLoop(); ls.MoreLoop(); ls.NextLoop())
        {
            // process a new loop : L is the new current Loop
            BuildLoop l = ls.Loop();
            bool boundaryL = l.IsShape;

            // L = shape et ForceClass  : on traite L comme un block
            // L = shape et !ForceClass : on traite L comme un pur shape
            // L = !shape               : on traite L comme un block
            bool traitercommeblock = (!boundaryL) || forceClass;
            if (!traitercommeblock)
            {
                // the loop L is a boundary loop :
                // - try to insert it in an existing area, such as L is inside all
                //   the block loops. Only block loops of the area are compared.
                // - if L could not be inserted, store it in list of boundary loops.

                loopinside = false;
                for (areaIter.Initialize(_area); areaIter.More(); areaIter.Next())
                {
                    NCollection_List<BuildLoop> area = areaIter.Value();
                    if (area.IsEmpty())
                    {
                        continue;
                    }
                    state = CompareLoopWithListOfLoop(lc, l, area, LoopEnum.Block);
                    if (state == Unknown)
                    {
                        Atomize(ref state, In);
                    }
                    loopinside = state == In;
                    if (loopinside)
                    {
                        break;
                    }
                } // end of Area scan

                if (loopinside)
                {
                    NCollection_List<BuildLoop> area = areaIter.Value();
                    ADD_Loop_TO_LISTOFLoop(l, area);
                }
                else if (!loopinside)
                {
                    ADD_Loop_TO_LISTOFLoop(l, boundaryloops);
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

                loopinside = false;
                for (areaIter.Initialize(_area); areaIter.More(); areaIter.Next())
                {
                    NCollection_List<BuildLoop> area = areaIter.Value();
                    if (area.IsEmpty())
                    {
                        continue;
                    }
                    state = CompareLoopWithListOfLoop(lc, l, area, LoopEnum.Anyloop);
                    if (state == Unknown)
                    {
                        Atomize(ref state, In);
                    }
                    loopinside = state == In;
                    if (loopinside)
                    {
                        break;
                    }
                } // end of Area scan

                if (loopinside)
                {
                    NCollection_List<BuildLoop> area = areaIter.Value();
                    bool allShape = true;
                    NCollection_List<BuildLoop> removedLoops = new();
                    loopIter.Initialize(area);
                    while (loopIter.More())
                    {
                        state = lc.Compare(loopIter.Value(), l);
                        if (state == Unknown)
                        {
                            Atomize(ref state, In); // not OUT
                        }
                        loopoutside = state == Out;
                        if (loopoutside)
                        {
                            BuildLoop curL = loopIter.Value();
                            // remove the loop from the area
                            ADD_Loop_TO_LISTOFLoop(curL, removedLoops);

                            allShape = allShape && curL.IsShape;
                            REM_Loop_FROM_LISTOFLoop(loopIter, areaIter.Value());
                        }
                        else
                        {
                            loopIter.Next();
                        }
                    }
                    // insert the loop in the area
                    ADD_Loop_TO_LISTOFLoop(l, area);
                    if (!removedLoops.IsEmpty())
                    {
                        if (allShape)
                        {
                            ADD_LISTOFLoop_TO_LISTOFLoop(removedLoops, boundaryloops);
                        }
                        else
                        {
                            // make a new area with the removed loops
                            NCollection_List<BuildLoop> thelist = new();
                            _area.Append(thelist);
                            ADD_LISTOFLoop_TO_LISTOFLoop(removedLoops, _area.Last());
                        }
                    }
                } // Loopinside == True

                else
                {
                    bool ashapeinside, ablockinside;
                    NCollection_List<BuildLoop> thelist1 = new();
                    _area.Append(thelist1);
                    NCollection_List<BuildLoop> newArea0 = _area.Last();
                    ADD_Loop_TO_LISTOFLoop(l, newArea0);

                    loopIter.Initialize(boundaryloops);
                    while (loopIter.More())
                    {
                        ashapeinside = ablockinside = false;
                        BuildLoop lb = loopIter.Value();
                        state = lc.Compare(lb, l);
                        if (state == Unknown)
                        {
                            Atomize(ref state, In);
                        }
                        ashapeinside = state == In;
                        if (ashapeinside)
                        {
                            state = lc.Compare(l, lb);
                            if (state == Unknown)
                            {
                                Atomize(ref state, In);
                            }
                            ablockinside = state == In;
                        }
                        if (ashapeinside && ablockinside)
                        {
                            BuildLoop curL = loopIter.Value();
                            ADD_Loop_TO_LISTOFLoop(curL, newArea0);

                            REM_Loop_FROM_LISTOFLoop(loopIter, boundaryloops);
                        }
                        else
                        {
                            loopIter.Next();
                        }
                    } // end of boundaryloops scan
                } // Loopinside == False
            } // end of block loop
        } // end of LoopSet LS scan

        if (!boundaryloops.IsEmpty())
        {
            if (_area.IsEmpty())
            {
                NCollection_List<BuildLoop> newArea3 = new();
                newArea3.Append(boundaryloops);
                _area.Append(newArea3);
            }
        }

        InitArea();
    }
}
