// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Hider.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Hider.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.HiddenLines;

/// <summary>Hides the edges of a data structure by one of its faces.</summary>
/// <remarks>A working object, as OCCT's.</remarks>
public sealed class Hider
{
    private readonly HiddenLineData _ds;

    public Hider(HiddenLineData ds)
    {
        _ds = ds;
    }

    /// <summary>Own hiding of the side face FI by its boundaries: nothing (OCCT's empty method).</summary>
    public void OwnHiding(int fi) { }

    // The exceptions OCCT's catch (Standard_Failure) handles under OCC_CATCH_SIGNALS around each edge
    private static bool IsCaught(Exception exception)
        => exception is FailureException
               or NullReferenceException
               or InvalidCastException
               or IndexOutOfRangeException
               or ArgumentOutOfRangeException
               or DivideByZeroException;

    /// <summary>Hides the selected edges of the data structure by the face FI; MST caches the faces' adaptors.</summary>
    public void Hide(int fi, NCollection_DataMap<Shape, AdaptorTool> mst)
    {
        // *****************************************************************
        //
        // This algorithm hides a set of edges stored in the data structure <myDS>
        // with the hiding face number FI in <myDS>.
        //
        // Outline of the algorithm
        //
        //   1. Loop on the Edges (not hidden and not rejected by the face minmax)
        //
        //       The rejections depending of the face are
        //          - Edge above the face
        //          - Edge belonging to the face
        //          - Edge rejected by a wire minmax
        //
        //       Compute interferences with the not rejected edges of the face.
        //           Store IN and ON interferences in two sorted lists
        //               ILHidden and ILOn
        //       If ILOn is not empty
        //           Resolve ComplexTransitions in ILOn
        //           Resolve ON Intersections in ILOn
        //             An On interference may become
        //               IN  : Move it from ILOn to ILHidden
        //               OUT : Remove it from ILOn
        //       If ILHidden and ILOn are empty
        //           intersect the edge with the face and classify the Edge.
        //               - if inside and under the face hide it.
        //       Else
        //         If ILHidden is not empty
        //           Resolve ComplexTransitions in ILHidden
        //           Build Hidden parts of the edge
        //               - Hide them
        //           Build visible parts of the edge
        //           Build Parts of the edge under the boundary of the face
        //               - Hide them as Boundary
        //         If ILOn is not empty
        //           Build ON parts of the edge
        //               - Hide them as ON parts
        //           Build Parts of the edge on the boundary of the face
        //               - Hide them as ON parts on Boundary
        //
        //
        // *****************************************************************

        _ds.InitEdge(fi, mst);
        if (!_ds.MoreEdge())
        { // there is nothing to do
            return; // **********************
        }
        if (_ds.IsBadFace())
        {
            return;
        }
        EdgeInterferenceTool eit = new(_ds); // List of Intersections
        NCollection_Array1<HiddenLineEdgeData> myEData = _ds.EDataArray();

        for (; _ds.MoreEdge(); _ds.NextEdge())
        { // loop on the Edges
            int e = _ds.Edge(); // *****************

            try
            {
                bool hasOut = false;
                NCollection_List<Interference> ilHidden = new();
                NCollection_List<Interference> ilOn = new();
                eit.LoadEdge();

                for (_ds.InitInterference(); // intersections with face-edges
                     _ds.MoreInterference(); // *****************************
                     _ds.NextInterference())
                {
                    if (_ds.RejectedInterference())
                    {
                        if (_ds.AboveInterference() && _ds.SimpleHidingFace())
                        {
                            hasOut = true;
                        }
                    }
                    else
                    {
                        ref Interference Int = ref _ds.Interference();
                        switch (Int.Intersection().State())
                        {
                            case In:
                                EdgeIList.AddInterference(ilHidden, Int, eit);
                                break;
                            case On:
                                EdgeIList.AddInterference(ilOn, Int, eit);
                                break;
                            case Out:
                            case Unknown:
                                break;
                        }
                    }
                }

                //-- ============================================================
                bool modif;
                do
                {
                    modif = false;
                    NCollection_List<Interference>.Iterator itSegHidden1 = new(ilHidden);
                    while (itSegHidden1.More() && !modif)
                    {
                        ref Interference int1 = ref itSegHidden1.ChangeValue();
                        int numseg1 = int1.Intersection().SegIndex();
                        if (numseg1 != 0)
                        {
                            NCollection_List<Interference>.Iterator itSegHidden2 = new(ilHidden);
                            while (itSegHidden2.More() && !modif)
                            {
                                ref Interference int2 = ref itSegHidden2.ChangeValue();
                                int numseg2 = int2.Intersection().SegIndex();
                                if (numseg1 + numseg2 == 0)
                                {
                                    //--printf("\nHidden Traitement du segment %d  %d\n",numseg1,numseg2);
                                    // fflush(stdout);
                                    Containment stbef1, staft1, stbef2, staft2;
                                    int1.Boundary().State3D(out stbef1, out staft1);
                                    int2.Boundary().State3D(out stbef2, out staft2);
                                    if (int1.Orientation() == int2.Orientation())
                                    {
                                        if (int1.Transition() == int2.Transition())
                                        {
                                            if (stbef1 == stbef2 && staft1 == staft2 && stbef1 != On
                                                && staft1 != On)
                                            {
                                                int nind = -1;
                                                if (int1.Intersection().Index() != 0)
                                                {
                                                    nind = int1.Intersection().Index();
                                                }
                                                if (int2.Intersection().Index() != 0)
                                                {
                                                    if (nind != -1)
                                                    {
                                                        if (int1.Intersection().Index() != int2.Intersection().Index())
                                                        {
                                                            nind = -1;
                                                        }
                                                    }
                                                    else
                                                    {
                                                        nind = int2.Intersection().Index();
                                                    }
                                                }
                                                if (int1.Intersection().Index() == 0 && int2.Intersection().Index() == 0)
                                                {
                                                    nind = 0;
                                                }

                                                if (nind != -1)
                                                {
                                                    //-- printf("\n Segment Supprime\n"); fflush(stdout);
                                                    ref AlgoIntersection inter = ref int1.ChangeIntersection();
                                                    inter.SegIndex(nind);
                                                    double p1 = int1.Intersection().Parameter();
                                                    double p2 = int2.Intersection().Parameter();
                                                    inter.Parameter((p1 + p2) * 0.5);
                                                    int1.BoundaryTransition(External);

                                                    // Int1 and Int2 refer into the list's storage: not used after this
                                                    ilHidden.Remove(itSegHidden2);
                                                    modif = true;
                                                }
                                            }
                                        }
                                    }
                                }
                                if (!modif)
                                {
                                    itSegHidden2.Next();
                                }
                            }
                        }
                        if (!modif)
                        {
                            itSegHidden1.Next();
                        }
                    }
                } while (modif);

                //-- ============================================================

                if (!ilOn.IsEmpty())
                { // process the interferences on ILOn
                  // *********************************

                    EdgeIList.ProcessComplex // complex transition on ILOn
                        (ilOn, eit); // **************************

                    NCollection_List<Interference>.Iterator it = new(ilOn);

                    while (it.More())
                    { // process Intersections on the Face
                      // *********************************

                        ref Interference Int = ref it.ChangeValue();
                        Containment stbef, staft; // read the 3d states
                        Int.Boundary().State3D(out stbef, out staft); // ******************

                        switch (Int.Transition())
                        {
                            case Forward:
                                switch (staft)
                                {
                                    case Out:
                                        ilOn.Remove(it);
                                        break;
                                    case In:
                                        EdgeIList.AddInterference(ilHidden, Int, eit);
                                        ilOn.Remove(it);
                                        break;
                                    case Unknown:
                                    // OCCT_DEBUG trace left out
                                    case On:
                                        it.Next();
                                        break;
                                }
                                break;
                            case Reversed:
                                switch (stbef)
                                {
                                    case Out:
                                        ilOn.Remove(it);
                                        break;
                                    case In:
                                        EdgeIList.AddInterference(ilHidden, Int, eit);
                                        ilOn.Remove(it);
                                        break;
                                    case Unknown:
                                    // OCCT_DEBUG trace left out
                                    case On:
                                        it.Next();
                                        break;
                                }
                                break;
                            case External:
                                ilOn.Remove(it);
                                break;
                            case Internal:
                                switch (stbef)
                                {
                                    case In:
                                        switch (staft)
                                        {
                                            case In:
                                                EdgeIList.AddInterference(ilHidden, Int, eit);
                                                ilOn.Remove(it);
                                                break;
                                            case On:
                                                Int.Transition(Forward); // FORWARD  in ILOn,
                                                EdgeIList.AddInterference // REVERSED in ILHidden
                                                    (ilHidden,
                                                     new Interference(Int.Intersection(), Int.Boundary(),
                                                                              Int.Orientation(), Reversed,
                                                                              Int.BoundaryTransition()),
                                                     eit);
                                                it.Next();
                                                break;
                                            case Out:
                                                Int.Transition(Reversed); // set REVERSED
                                                EdgeIList.AddInterference(ilHidden, Int, eit);
                                                ilOn.Remove(it);
                                                break;
                                            case Unknown:
                                                // OCCT_DEBUG trace left out
                                                it.Next();
                                                break;
                                        }
                                        break;
                                    case On:
                                        switch (staft)
                                        {
                                            case In:
                                                Int.Transition(Reversed); // REVERSED in ILOn,
                                                EdgeIList.AddInterference // REVERSED in ILHidden
                                                    (ilHidden,
                                                     new Interference(Int.Intersection(), Int.Boundary(),
                                                                              Int.Orientation(), Forward,
                                                                              Int.BoundaryTransition()),
                                                     eit);
                                                break;
                                            case On:
                                                break;
                                            case Out:
                                                Int.Transition(Reversed);
                                                break;
                                            case Unknown:
                                                // OCCT_DEBUG trace left out
                                                break;
                                        }
                                        it.Next();
                                        break;
                                    case Out:
                                        switch (staft)
                                        {
                                            case In:
                                                Int.Transition(Forward); // set FORWARD
                                                EdgeIList.AddInterference(ilHidden, Int, eit);
                                                ilOn.Remove(it);
                                                break;
                                            case On:
                                                Int.Transition(Forward); // FORWARD  in ILOn
                                                it.Next();
                                                break;
                                            case Out:
                                                ilOn.Remove(it);
                                                break;
                                            case Unknown:
                                                // OCCT_DEBUG trace left out
                                                it.Next();
                                                break;
                                        }
                                        break;
                                    case Unknown:
                                        // OCCT_DEBUG trace left out
                                        // OCCT doesn't move the iterator here, and loops forever on the same
                                        // interference; the port moves on (occt-issues.md, unit 7Q2)
                                        it.Next();
                                        break;
                                }
                                break;
                        }
                    }
                }

                if (ilHidden.IsEmpty() && ilOn.IsEmpty() && !hasOut)
                {
                    HiddenLineEdgeData ed = myEData[e];
                    Containment st = _ds.Compare(e, ed); // Classification
                    if (st == In || st == On)
                    { // **************
                        ed.Status().HideAll();
                    }
                }
                else
                {
                    double p1 = 0.0, p2 = 0.0;
                    float tol1 = 0.0f, tol2 = 0.0f;

                    HiddenLineEdgeData ed = myEData[e];
                    EdgeStatus es = ed.Status();

                    bool foundHidden = false;

                    if (!ilHidden.IsEmpty())
                    {
                        EdgeIList.ProcessComplex // complex transition on ILHidden
                            (ilHidden, eit); // ******************************
                        int level = 0;
                        if (!_ds.SimpleHidingFace())
                        { // Level at Start
                            level = _ds.HidingStartLevel(e, ed, ilHidden); // **************
                        }

                        NCollection_List<Interference>.Iterator it = new(ilHidden);
                        if (_ds.SimpleHidingFace()) // remove excess interferences
                        {
                            NCollection_Sequence<double> toRemove = new();
                            Orientation prevTrans = External;
                            double prevParam = 0.0;
                            for (; it.More(); it.Next())
                            {
                                Interference Int = it.Value();
                                Orientation trans = Int.Transition();
                                if (trans == prevTrans)
                                {
                                    if (trans == Forward)
                                    {
                                        toRemove.Append(Int.Intersection().Parameter());
                                        // OCCT_DEBUG trace left out
                                    }
                                    else if (trans == Reversed)
                                    {
                                        toRemove.Append(prevParam);
                                        // OCCT_DEBUG trace left out
                                    }
                                }
                                prevTrans = trans;
                                prevParam = Int.Intersection().Parameter();
                            }
                            it.Initialize(ilHidden);
                            while (it.More())
                            {
                                double param = it.Value().Intersection().Parameter();
                                bool found = false;
                                for (int i = 1; i <= toRemove.Length(); i++)
                                {
                                    if (param == toRemove[i])
                                    {
                                        found = true;
                                        ilHidden.Remove(it);
                                        toRemove.Remove(i);
                                        break;
                                    }
                                }
                                if (!found)
                                {
                                    it.Next();
                                }
                            }
                        } // remove excess interferences

                        it.Initialize(ilHidden);
                        while (it.More())
                        { // suppress multi-inside Intersections
                          // ***********************************

                            Interference Int = it.Value();
                            switch (Int.Transition())
                            {
                                case Forward:
                                {
                                    int decal = Int.Intersection().Level();
                                    if (level > 0)
                                    {
                                        ilHidden.Remove(it);
                                    }
                                    else
                                    {
                                        it.Next();
                                    }
                                    level = level + decal;
                                }
                                    break;
                                case Reversed:
                                {
                                    level = level - Int.Intersection().Level();
                                    if (level > 0)
                                    {
                                        ilHidden.Remove(it);
                                    }
                                    else
                                    {
                                        it.Next();
                                    }
                                }
                                    break;
                                case External:
                                    it.Next();
                                    break;
                                case Internal:
                                    it.Next();
                                    break;
                                default:
                                    it.Next();
                                    break;
                            }
                        }
                        if (ilHidden.IsEmpty())
                        { // Edge hidden
                            es.HideAll(); // ***********
                        }
                        else
                        {
                            foundHidden = true;
                        }
                    }

                    if (!ilHidden.IsEmpty())
                    {
                        // IFV

                        Containment buildIN = In;
                        bool isSuspicion = true;

                        double pmax, pmin;
                        bool allInt = false;
                        bool allFor = false;
                        bool allRev = false;
                        pmin = RealLast();
                        pmax = -pmin;

                        if (ilHidden.Extent() > 1)
                        {
                            allInt = true;
                            allFor = true;
                            allRev = true;
                            NCollection_List<Interference>.Iterator it = new(ilHidden);
                            for (; it.More(); it.Next())
                            {
                                double p = it.Value().Intersection().Parameter();
                                allFor = allFor && (it.Value().Transition() == Forward);
                                allRev = allRev && (it.Value().Transition() == Reversed);
                                allInt = allInt && (it.Value().Transition() == Internal);
                                if (p < pmin)
                                {
                                    pmin = p;
                                }
                                if (p > pmax)
                                {
                                    pmax = p;
                                }
                            }
                        }

                        NCollection_List<Interference>.Iterator itl = new(ilHidden);
                        VertexList il = new(eit, itl);

                        EdgeBuilder eb = new(il);

                        eb.Builds(buildIN); // build hidden parts
                                             // ******************
                        while (eb.MoreEdges())
                        {
                            p1 = 0.0;
                            p2 = 0.0;
                            int maskP1P2 = 0;
                            while (eb.MoreVertices())
                            {
                                switch (eb.Orientation())
                                {
                                    case Forward:
                                        p1 = eb.Current().Parameter();
                                        tol1 = eb.Current().Tolerance();
                                        maskP1P2 |= 1;
                                        break;
                                    case Reversed:
                                        p2 = eb.Current().Parameter();
                                        tol2 = eb.Current().Tolerance();
                                        maskP1P2 |= 2;
                                        break;
                                    case Internal:
                                    case External:
                                        break;
                                }
                                eb.NextVertex();
                            }

                            if (maskP1P2 != 3 || p2 - p1 <= 1.0e-7)
                            {
                                eb.NextEdge();
                                continue;
                            }

                            if (allInt)
                            {
                                if (p1 < pmin)
                                {
                                    p1 = pmin;
                                }
                                if (p2 > pmax)
                                {
                                    p2 = pmax;
                                }
                                // HLRBRep_EdgeData& ed = myEData(E);
                                // TopAbs_State st = myDS->Compare(E,ed);              // Classification
                            }

                            Containment testState = In;
                            if (isSuspicion)
                            {
                                // int aNbp = 1;
                                // aTestState = myDS->SimplClassify(E, ed, aNbp, p1, p2);
                                int tmplevel = 0;
                                testState = _ds.Classify(e, ed, true, ref tmplevel, (p1 + p2) / 2.0);
                            }

                            if (testState != Out)
                            {
                                es.Hide(p1, tol1, p2, tol2,
                                        false, // under  the Face
                                        false); // inside the Face
                            }

                            eb.NextEdge();
                        }

                        eb.Builds(On); // build parts under the boundary
                                              // ******************************
                        while (eb.MoreEdges())
                        {
                            p1 = 0.0;
                            p2 = 0.0;
                            int maskP1P2 = 0;
                            while (eb.MoreVertices())
                            {
                                switch (eb.Orientation())
                                {
                                    case Forward:
                                        p1 = eb.Current().Parameter();
                                        tol1 = eb.Current().Tolerance();
                                        maskP1P2 |= 1;
                                        break;
                                    case Reversed:
                                        p2 = eb.Current().Parameter();
                                        tol2 = eb.Current().Tolerance();
                                        maskP1P2 |= 2;
                                        break;
                                    case Internal:
                                    case External:
                                        break;
                                }
                                eb.NextVertex();
                            }

                            if (maskP1P2 != 3 || p2 - p1 <= 1.0e-7)
                            {
                                eb.NextEdge();
                                continue;
                            }

                            Containment testState = In;
                            if (isSuspicion)
                            {
                                // int aNbp = 1;
                                // aTestState = myDS->SimplClassify(E, ed, aNbp, p1, p2);
                                int tmplevel = 0;
                                testState = _ds.Classify(e, ed, true, ref tmplevel, (p1 + p2) / 2.0);
                            }

                            if (testState != Out)
                            {
                                es.Hide(p1, tol1, p2, tol2,
                                        false, // under the Face
                                        true); // on the boundary
                            }

                            eb.NextEdge();
                        }
                    }

                    if (!ilOn.IsEmpty())
                    {
                        int level = 0;
                        if (!_ds.SimpleHidingFace())
                        { // Level at Start
                            level = _ds.HidingStartLevel(e, ed, ilOn); // **************
                        }
                        if (level > 0)
                        {
                            NCollection_List<Interference>.Iterator it = new(ilOn);

                            while (it.More())
                            { // suppress multi-inside Intersections
                              // ***********************************

                                Interference Int = it.Value();
                                switch (Int.Transition())
                                {
                                    case Forward:
                                    {
                                        int decal = Int.Intersection().Level();
                                        if (level > 0)
                                        {
                                            ilOn.Remove(it);
                                        }
                                        else
                                        {
                                            it.Next();
                                        }
                                        level = level + decal;
                                    }
                                        break;
                                    case Reversed:
                                        level = level - Int.Intersection().Level();
                                        if (level > 0)
                                        {
                                            ilOn.Remove(it);
                                        }
                                        else
                                        {
                                            it.Next();
                                        }
                                        break;
                                    case External:
                                    case Internal:
                                    default:
                                        it.Next();
                                        break;
                                }
                            }
                            if (ilOn.IsEmpty() && !foundHidden)
                            { // Edge hidden
                                es.HideAll(); // ***********
                            }
                        }
                    }

                    if (!ilOn.IsEmpty())
                    {
                        VertexList il = new(eit, new NCollection_List<Interference>.Iterator(ilOn));
                        EdgeBuilder eb = new(il);

                        eb.Builds(In); // build parts on the Face
                                              // ***********************
                        while (eb.MoreEdges())
                        {
                            p1 = 0.0;
                            p2 = 0.0;
                            int maskP1P2 = 0;
                            while (eb.MoreVertices())
                            {
                                switch (eb.Orientation())
                                {
                                    case Forward:
                                        p1 = eb.Current().Parameter();
                                        tol1 = eb.Current().Tolerance();
                                        maskP1P2 |= 1;
                                        break;
                                    case Reversed:
                                        p2 = eb.Current().Parameter();
                                        tol2 = eb.Current().Tolerance();
                                        maskP1P2 |= 2;
                                        break;
                                    case Internal:
                                    case External:
                                        break;
                                }
                                eb.NextVertex();
                            }

                            if (maskP1P2 != 3 || p2 - p1 <= 1.0e-7)
                            {
                                eb.NextEdge();
                                continue;
                            }

                            es.Hide(p1, tol1, p2, tol2,
                                    true, // on     the Face
                                    false); // inside the Face
                            eb.NextEdge();
                        }

                        eb.Builds(On); // build hidden parts under the boundary
                                              // *************************************
                        while (eb.MoreEdges())
                        {
                            p1 = 0.0;
                            p2 = 0.0;
                            int maskP1P2 = 0;
                            while (eb.MoreVertices())
                            {
                                switch (eb.Orientation())
                                {
                                    case Forward:
                                        p1 = eb.Current().Parameter();
                                        tol1 = eb.Current().Tolerance();
                                        maskP1P2 |= 1;
                                        break;
                                    case Reversed:
                                        p2 = eb.Current().Parameter();
                                        tol2 = eb.Current().Tolerance();
                                        maskP1P2 |= 2;
                                        break;
                                    case Internal:
                                    case External:
                                        break;
                                }
                                eb.NextVertex();
                            }

                            if (maskP1P2 != 3 || p2 - p1 <= 1.0e-7)
                            {
                                eb.NextEdge();
                                continue;
                            }

                            es.Hide(p1, tol1, p2, tol2,
                                    true, // on the Face
                                    true); // on the boundary
                            eb.NextEdge();
                        }
                    }
                }
            }
            catch (Exception exception) when (IsCaught(exception))
            {
                // OCCT_DEBUG trace left out
            }
        }
    }
}
