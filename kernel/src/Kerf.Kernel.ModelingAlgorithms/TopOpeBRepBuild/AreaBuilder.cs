// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_AreaBuilder.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_AreaBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;
using static Kerf.Topology.Containment;
using static Kerf.Engine.Booleans.Legacy.Build.LoopEnum;

namespace Kerf.Engine.Booleans.Legacy.Build;

/// <summary>
/// Rebuilds areas (faces, solids) from loops: the loops of a <see cref="LoopSet"/> (wires or shells,
/// on the boundary, and blocks of elements, from interferences) are grouped into areas with a
/// <see cref="LoopClassifier"/>. The result is an iteration on the areas, each a list of loops.
/// </summary>
/// <remarks>
/// A working object. OCCT's <c>ADD_Loop_TO_LISTOFLoop</c> and the other list helpers take a debugging <c>void*</c>;
/// the port drops it. Appending a list to another moves its loops, as NCollection_List::Append(list) does.
/// </remarks>
internal class AreaBuilder
{
    protected readonly NCollection_List<NCollection_List<BuildLoop>> _area = new();
    protected readonly NCollection_List<NCollection_List<BuildLoop>>.Iterator _areaIterator = new();
    protected NCollection_List<BuildLoop>.Iterator _loopIterator = new();
    protected bool _unknownRaise;

    /// <summary>An empty area builder.</summary>
    public AreaBuilder()
    {
        _unknownRaise = false; // no raise if UNKNOWN state found
    }

    /// <summary>The areas of the loops of <paramref name="ls"/>, classified with <paramref name="lc"/>.</summary>
    public AreaBuilder(LoopSet ls, LoopClassifier lc, bool forceClass = false)
    {
        _unknownRaise = false; // no raise if UNKNOWN state found
// C++ calls the virtual InitAreaBuilder from the constructor, which runs this class's version
        InitAreaBuilderCore(ls, lc, forceClass);
    }

    /// <summary>
    /// The state of <paramref name="l"/> compared with the loops of <paramref name="lol"/> of the kind
    /// <paramref name="what"/>: OUT at the first loop it is out of.
    /// </summary>
    protected Containment CompareLoopWithListOfLoop(LoopClassifier lc,
                                                     BuildLoop l,
                                                     NCollection_List<BuildLoop> lol,
                                                     LoopEnum what)
    {
        Containment state = Unknown;
        bool totest; // L must or not be tested
        NCollection_List<BuildLoop>.Iterator loopIter = new();

        if (lol.IsEmpty())
        {
            return Out;
        }

        for (loopIter.Initialize(lol); loopIter.More(); loopIter.Next())
        {
            BuildLoop curL = loopIter.Value();
            switch (what)
            {
                case LoopEnum.Anyloop:
                    totest = true;
                    break;
                case LoopEnum.Boundary:
                    totest = curL.IsShape;
                    break;
                case LoopEnum.Block:
                    totest = !curL.IsShape;
                    break;
                default:
                    totest = false;
                    break;
            }
            if (totest)
            {
                state = lc.Compare(l, curL);
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
        if (_unknownRaise)
        {
            Standard_DomainError_Raise_if(state == Unknown, "AreaBuilder : Position Unknown");
        }
        else
        {
            state = newstate;
        }
    }

    /// <summary>
    /// Builds the areas of the loops of <paramref name="ls"/>, classified with <paramref name="lc"/>; with
    /// <paramref name="forceClass"/>, shape loops are classified as blocks are.
    /// </summary>
    public virtual void InitAreaBuilder(LoopSet ls, LoopClassifier lc, bool forceClass = false)
    {
        InitAreaBuilderCore(ls, lc, forceClass);
    }

    private void InitAreaBuilderCore(LoopSet ls, LoopClassifier lc, bool forceClass)
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

            // L = Shape et ForceClass  : on traite L comme un block
            // L = Shape et !ForceClass : on traite L comme un pur Shape
            // L = !Shape               : on traite L comme un block
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
                        state = lc.Compare(loopIter.Value(), l);
                        if (state == Unknown)
                        {
                            Atomize(ref state, In);
                        }
                        ashapeinside = state == In;
                        if (ashapeinside)
                        {
                            state = lc.Compare(l, loopIter.Value());
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

        InitArea();
    }

    /// <summary>Starts the iteration on the areas; their number.</summary>
    public int InitArea()
    {
        _areaIterator.Initialize(_area);
        InitLoop();
        int n = _area.Extent();
        return n;
    }

    /// <summary>Whether there is a current area.</summary>
    public bool MoreArea()
    {
        bool b = _areaIterator.More();
        return b;
    }

    /// <summary>Moves to the next area.</summary>
    public void NextArea()
    {
        _areaIterator.Next();
        InitLoop();
    }

    /// <summary>Starts the iteration on the loops of the current area; their number.</summary>
    public int InitLoop()
    {
        int n = 0;
        if (_areaIterator.More())
        {
            NCollection_List<BuildLoop> lal = _areaIterator.Value();
            _loopIterator.Initialize(lal);
            n = lal.Extent();
        }
        else
        { // Create an empty ListIteratorOfListOfLoop
            _loopIterator = new NCollection_List<BuildLoop>.Iterator();
        }
        return n;
    }

    /// <summary>Whether there is a current loop in the current area.</summary>
    public bool MoreLoop()
    {
        bool b = _loopIterator.More();
        return b;
    }

    /// <summary>Moves to the next loop of the current area.</summary>
    public void NextLoop()
    {
        _loopIterator.Next();
    }

    /// <summary>The current loop of the current area.</summary>
    public BuildLoop Loop()
    {
        BuildLoop l = _loopIterator.Value();
        return l;
    }

    /// <summary>Appends <paramref name="l"/> to <paramref name="lol"/>.</summary>
    public virtual void ADD_Loop_TO_LISTOFLoop(BuildLoop l, NCollection_List<BuildLoop> lol)
    {
        lol.Append(l);
    }

    /// <summary>Removes the loop of <paramref name="ita"/> from <paramref name="a"/>; the iterator moves to the next.</summary>
    public virtual void REM_Loop_FROM_LISTOFLoop(NCollection_List<BuildLoop>.Iterator ita,
                                                 NCollection_List<BuildLoop> a)
    {
        a.Remove(ita);
    }

    /// <summary>Moves the loops of <paramref name="a1"/> to the end of <paramref name="a2"/>.</summary>
    public virtual void ADD_LISTOFLoop_TO_LISTOFLoop(NCollection_List<BuildLoop> a1,
                                                     NCollection_List<BuildLoop> a2)
    {
        a2.Append(a1);
    }
}
