// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Hider.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Hider.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>Hides the edges of a data structure by one of its faces.</summary>
/// <remarks>A working object, as OCCT's.</remarks>
public sealed class HLRBRep_Hider
{
    private readonly HLRBRep_Data myDS;

    public HLRBRep_Hider(HLRBRep_Data DS)
    {
        myDS = DS;
    }

    /// <summary>Own hiding of the side face FI by its boundaries: nothing (OCCT's empty method).</summary>
    public void OwnHiding(int FI) { }

    // The exceptions OCCT's catch (Standard_Failure) handles under OCC_CATCH_SIGNALS around each edge
    private static bool IsCaught(Exception theException)
        => theException is FailureException
               or NullReferenceException
               or InvalidCastException
               or IndexOutOfRangeException
               or ArgumentOutOfRangeException
               or DivideByZeroException;

    /// <summary>Hides the selected edges of the data structure by the face FI; MST caches the faces' adaptors.</summary>
    public void Hide(int FI, NCollection_DataMap<TopoDS_Shape, BRepTopAdaptor_Tool> MST)
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

        myDS.InitEdge(FI, MST);
        if (!myDS.MoreEdge())
        { // there is nothing to do
            return; // **********************
        }
        if (myDS.IsBadFace())
        {
            return;
        }
        HLRBRep_EdgeInterferenceTool EIT = new(myDS); // List of Intersections
        NCollection_Array1<HLRBRep_EdgeData> myEData = myDS.EDataArray();

        for (; myDS.MoreEdge(); myDS.NextEdge())
        { // loop on the Edges
            int E = myDS.Edge(); // *****************

            try
            {
                bool hasOut = false;
                NCollection_List<HLRAlgo_Interference> ILHidden = new();
                NCollection_List<HLRAlgo_Interference> ILOn = new();
                EIT.LoadEdge();

                for (myDS.InitInterference(); // intersections with face-edges
                     myDS.MoreInterference(); // *****************************
                     myDS.NextInterference())
                {
                    if (myDS.RejectedInterference())
                    {
                        if (myDS.AboveInterference() && myDS.SimpleHidingFace())
                        {
                            hasOut = true;
                        }
                    }
                    else
                    {
                        ref HLRAlgo_Interference Int = ref myDS.Interference();
                        switch (Int.Intersection().State())
                        {
                            case TopAbs_IN:
                                HLRBRep_EdgeIList.AddInterference(ILHidden, Int, EIT);
                                break;
                            case TopAbs_ON:
                                HLRBRep_EdgeIList.AddInterference(ILOn, Int, EIT);
                                break;
                            case TopAbs_OUT:
                            case TopAbs_UNKNOWN:
                                break;
                        }
                    }
                }

                //-- ============================================================
                bool Modif;
                do
                {
                    Modif = false;
                    NCollection_List<HLRAlgo_Interference>.Iterator ItSegHidden1 = new(ILHidden);
                    while (ItSegHidden1.More() && !Modif)
                    {
                        ref HLRAlgo_Interference Int1 = ref ItSegHidden1.ChangeValue();
                        int numseg1 = Int1.Intersection().SegIndex();
                        if (numseg1 != 0)
                        {
                            NCollection_List<HLRAlgo_Interference>.Iterator ItSegHidden2 = new(ILHidden);
                            while (ItSegHidden2.More() && !Modif)
                            {
                                ref HLRAlgo_Interference Int2 = ref ItSegHidden2.ChangeValue();
                                int numseg2 = Int2.Intersection().SegIndex();
                                if (numseg1 + numseg2 == 0)
                                {
                                    //--printf("\nHidden Traitement du segment %d  %d\n",numseg1,numseg2);
                                    // fflush(stdout);
                                    TopAbs_State stbef1, staft1, stbef2, staft2;
                                    Int1.Boundary().State3D(out stbef1, out staft1);
                                    Int2.Boundary().State3D(out stbef2, out staft2);
                                    if (Int1.Orientation() == Int2.Orientation())
                                    {
                                        if (Int1.Transition() == Int2.Transition())
                                        {
                                            if (stbef1 == stbef2 && staft1 == staft2 && stbef1 != TopAbs_ON
                                                && staft1 != TopAbs_ON)
                                            {
                                                int nind = -1;
                                                if (Int1.Intersection().Index() != 0)
                                                {
                                                    nind = Int1.Intersection().Index();
                                                }
                                                if (Int2.Intersection().Index() != 0)
                                                {
                                                    if (nind != -1)
                                                    {
                                                        if (Int1.Intersection().Index() != Int2.Intersection().Index())
                                                        {
                                                            nind = -1;
                                                        }
                                                    }
                                                    else
                                                    {
                                                        nind = Int2.Intersection().Index();
                                                    }
                                                }
                                                if (Int1.Intersection().Index() == 0 && Int2.Intersection().Index() == 0)
                                                {
                                                    nind = 0;
                                                }

                                                if (nind != -1)
                                                {
                                                    //-- printf("\n Segment Supprime\n"); fflush(stdout);
                                                    ref HLRAlgo_Intersection inter = ref Int1.ChangeIntersection();
                                                    inter.SegIndex(nind);
                                                    double p1 = Int1.Intersection().Parameter();
                                                    double p2 = Int2.Intersection().Parameter();
                                                    inter.Parameter((p1 + p2) * 0.5);
                                                    Int1.BoundaryTransition(TopAbs_EXTERNAL);

                                                    // Int1 and Int2 refer into the list's storage: not used after this
                                                    ILHidden.Remove(ItSegHidden2);
                                                    Modif = true;
                                                }
                                            }
                                        }
                                    }
                                }
                                if (!Modif)
                                {
                                    ItSegHidden2.Next();
                                }
                            }
                        }
                        if (!Modif)
                        {
                            ItSegHidden1.Next();
                        }
                    }
                } while (Modif);

                //-- ============================================================

                if (!ILOn.IsEmpty())
                { // process the interferences on ILOn
                  // *********************************

                    HLRBRep_EdgeIList.ProcessComplex // complex transition on ILOn
                        (ILOn, EIT); // **************************

                    NCollection_List<HLRAlgo_Interference>.Iterator It = new(ILOn);

                    while (It.More())
                    { // process Intersections on the Face
                      // *********************************

                        ref HLRAlgo_Interference Int = ref It.ChangeValue();
                        TopAbs_State stbef, staft; // read the 3d states
                        Int.Boundary().State3D(out stbef, out staft); // ******************

                        switch (Int.Transition())
                        {
                            case TopAbs_FORWARD:
                                switch (staft)
                                {
                                    case TopAbs_OUT:
                                        ILOn.Remove(It);
                                        break;
                                    case TopAbs_IN:
                                        HLRBRep_EdgeIList.AddInterference(ILHidden, Int, EIT);
                                        ILOn.Remove(It);
                                        break;
                                    case TopAbs_UNKNOWN:
                                    // OCCT_DEBUG trace left out
                                    case TopAbs_ON:
                                        It.Next();
                                        break;
                                }
                                break;
                            case TopAbs_REVERSED:
                                switch (stbef)
                                {
                                    case TopAbs_OUT:
                                        ILOn.Remove(It);
                                        break;
                                    case TopAbs_IN:
                                        HLRBRep_EdgeIList.AddInterference(ILHidden, Int, EIT);
                                        ILOn.Remove(It);
                                        break;
                                    case TopAbs_UNKNOWN:
                                    // OCCT_DEBUG trace left out
                                    case TopAbs_ON:
                                        It.Next();
                                        break;
                                }
                                break;
                            case TopAbs_EXTERNAL:
                                ILOn.Remove(It);
                                break;
                            case TopAbs_INTERNAL:
                                switch (stbef)
                                {
                                    case TopAbs_IN:
                                        switch (staft)
                                        {
                                            case TopAbs_IN:
                                                HLRBRep_EdgeIList.AddInterference(ILHidden, Int, EIT);
                                                ILOn.Remove(It);
                                                break;
                                            case TopAbs_ON:
                                                Int.Transition(TopAbs_FORWARD); // FORWARD  in ILOn,
                                                HLRBRep_EdgeIList.AddInterference // REVERSED in ILHidden
                                                    (ILHidden,
                                                     new HLRAlgo_Interference(Int.Intersection(), Int.Boundary(),
                                                                              Int.Orientation(), TopAbs_REVERSED,
                                                                              Int.BoundaryTransition()),
                                                     EIT);
                                                It.Next();
                                                break;
                                            case TopAbs_OUT:
                                                Int.Transition(TopAbs_REVERSED); // set REVERSED
                                                HLRBRep_EdgeIList.AddInterference(ILHidden, Int, EIT);
                                                ILOn.Remove(It);
                                                break;
                                            case TopAbs_UNKNOWN:
                                                // OCCT_DEBUG trace left out
                                                It.Next();
                                                break;
                                        }
                                        break;
                                    case TopAbs_ON:
                                        switch (staft)
                                        {
                                            case TopAbs_IN:
                                                Int.Transition(TopAbs_REVERSED); // REVERSED in ILOn,
                                                HLRBRep_EdgeIList.AddInterference // REVERSED in ILHidden
                                                    (ILHidden,
                                                     new HLRAlgo_Interference(Int.Intersection(), Int.Boundary(),
                                                                              Int.Orientation(), TopAbs_FORWARD,
                                                                              Int.BoundaryTransition()),
                                                     EIT);
                                                break;
                                            case TopAbs_ON:
                                                break;
                                            case TopAbs_OUT:
                                                Int.Transition(TopAbs_REVERSED);
                                                break;
                                            case TopAbs_UNKNOWN:
                                                // OCCT_DEBUG trace left out
                                                break;
                                        }
                                        It.Next();
                                        break;
                                    case TopAbs_OUT:
                                        switch (staft)
                                        {
                                            case TopAbs_IN:
                                                Int.Transition(TopAbs_FORWARD); // set FORWARD
                                                HLRBRep_EdgeIList.AddInterference(ILHidden, Int, EIT);
                                                ILOn.Remove(It);
                                                break;
                                            case TopAbs_ON:
                                                Int.Transition(TopAbs_FORWARD); // FORWARD  in ILOn
                                                It.Next();
                                                break;
                                            case TopAbs_OUT:
                                                ILOn.Remove(It);
                                                break;
                                            case TopAbs_UNKNOWN:
                                                // OCCT_DEBUG trace left out
                                                It.Next();
                                                break;
                                        }
                                        break;
                                    case TopAbs_UNKNOWN:
                                        // OCCT_DEBUG trace left out
                                        // OCCT doesn't move the iterator here, and loops forever on the same
                                        // interference; the port moves on (occt-issues.md, unit 7Q2)
                                        It.Next();
                                        break;
                                }
                                break;
                        }
                    }
                }

                if (ILHidden.IsEmpty() && ILOn.IsEmpty() && !hasOut)
                {
                    HLRBRep_EdgeData ed = myEData[E];
                    TopAbs_State st = myDS.Compare(E, ed); // Classification
                    if (st == TopAbs_IN || st == TopAbs_ON)
                    { // **************
                        ed.Status().HideAll();
                    }
                }
                else
                {
                    double p1 = 0.0, p2 = 0.0;
                    float tol1 = 0.0f, tol2 = 0.0f;

                    HLRBRep_EdgeData ed = myEData[E];
                    HLRAlgo_EdgeStatus ES = ed.Status();

                    bool foundHidden = false;

                    if (!ILHidden.IsEmpty())
                    {
                        HLRBRep_EdgeIList.ProcessComplex // complex transition on ILHidden
                            (ILHidden, EIT); // ******************************
                        int level = 0;
                        if (!myDS.SimpleHidingFace())
                        { // Level at Start
                            level = myDS.HidingStartLevel(E, ed, ILHidden); // **************
                        }

                        NCollection_List<HLRAlgo_Interference>.Iterator It = new(ILHidden);
                        if (myDS.SimpleHidingFace()) // remove excess interferences
                        {
                            NCollection_Sequence<double> ToRemove = new();
                            TopAbs_Orientation PrevTrans = TopAbs_EXTERNAL;
                            double PrevParam = 0.0;
                            for (; It.More(); It.Next())
                            {
                                HLRAlgo_Interference Int = It.Value();
                                TopAbs_Orientation aTrans = Int.Transition();
                                if (aTrans == PrevTrans)
                                {
                                    if (aTrans == TopAbs_FORWARD)
                                    {
                                        ToRemove.Append(Int.Intersection().Parameter());
                                        // OCCT_DEBUG trace left out
                                    }
                                    else if (aTrans == TopAbs_REVERSED)
                                    {
                                        ToRemove.Append(PrevParam);
                                        // OCCT_DEBUG trace left out
                                    }
                                }
                                PrevTrans = aTrans;
                                PrevParam = Int.Intersection().Parameter();
                            }
                            It.Initialize(ILHidden);
                            while (It.More())
                            {
                                double aParam = It.Value().Intersection().Parameter();
                                bool found = false;
                                for (int i = 1; i <= ToRemove.Length(); i++)
                                {
                                    if (aParam == ToRemove[i])
                                    {
                                        found = true;
                                        ILHidden.Remove(It);
                                        ToRemove.Remove(i);
                                        break;
                                    }
                                }
                                if (!found)
                                {
                                    It.Next();
                                }
                            }
                        } // remove excess interferences

                        It.Initialize(ILHidden);
                        while (It.More())
                        { // suppress multi-inside Intersections
                          // ***********************************

                            HLRAlgo_Interference Int = It.Value();
                            switch (Int.Transition())
                            {
                                case TopAbs_FORWARD:
                                {
                                    int decal = Int.Intersection().Level();
                                    if (level > 0)
                                    {
                                        ILHidden.Remove(It);
                                    }
                                    else
                                    {
                                        It.Next();
                                    }
                                    level = level + decal;
                                }
                                    break;
                                case TopAbs_REVERSED:
                                {
                                    level = level - Int.Intersection().Level();
                                    if (level > 0)
                                    {
                                        ILHidden.Remove(It);
                                    }
                                    else
                                    {
                                        It.Next();
                                    }
                                }
                                    break;
                                case TopAbs_EXTERNAL:
                                    It.Next();
                                    break;
                                case TopAbs_INTERNAL:
                                    It.Next();
                                    break;
                                default:
                                    It.Next();
                                    break;
                            }
                        }
                        if (ILHidden.IsEmpty())
                        { // Edge hidden
                            ES.HideAll(); // ***********
                        }
                        else
                        {
                            foundHidden = true;
                        }
                    }

                    if (!ILHidden.IsEmpty())
                    {
                        // IFV

                        TopAbs_State aBuildIN = TopAbs_IN;
                        bool IsSuspicion = true;

                        double pmax, pmin;
                        bool allInt = false;
                        bool allFor = false;
                        bool allRev = false;
                        pmin = RealLast();
                        pmax = -pmin;

                        if (ILHidden.Extent() > 1)
                        {
                            allInt = true;
                            allFor = true;
                            allRev = true;
                            NCollection_List<HLRAlgo_Interference>.Iterator It = new(ILHidden);
                            for (; It.More(); It.Next())
                            {
                                double p = It.Value().Intersection().Parameter();
                                allFor = allFor && (It.Value().Transition() == TopAbs_FORWARD);
                                allRev = allRev && (It.Value().Transition() == TopAbs_REVERSED);
                                allInt = allInt && (It.Value().Transition() == TopAbs_INTERNAL);
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

                        NCollection_List<HLRAlgo_Interference>.Iterator Itl = new(ILHidden);
                        HLRBRep_VertexList IL = new(EIT, Itl);

                        HLRBRep_EdgeBuilder EB = new(IL);

                        EB.Builds(aBuildIN); // build hidden parts
                                             // ******************
                        while (EB.MoreEdges())
                        {
                            p1 = 0.0;
                            p2 = 0.0;
                            int aMaskP1P2 = 0;
                            while (EB.MoreVertices())
                            {
                                switch (EB.Orientation())
                                {
                                    case TopAbs_FORWARD:
                                        p1 = EB.Current().Parameter();
                                        tol1 = EB.Current().Tolerance();
                                        aMaskP1P2 |= 1;
                                        break;
                                    case TopAbs_REVERSED:
                                        p2 = EB.Current().Parameter();
                                        tol2 = EB.Current().Tolerance();
                                        aMaskP1P2 |= 2;
                                        break;
                                    case TopAbs_INTERNAL:
                                    case TopAbs_EXTERNAL:
                                        break;
                                }
                                EB.NextVertex();
                            }

                            if (aMaskP1P2 != 3 || p2 - p1 <= 1.0e-7)
                            {
                                EB.NextEdge();
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

                            TopAbs_State aTestState = TopAbs_IN;
                            if (IsSuspicion)
                            {
                                // int aNbp = 1;
                                // aTestState = myDS->SimplClassify(E, ed, aNbp, p1, p2);
                                int tmplevel = 0;
                                aTestState = myDS.Classify(E, ed, true, ref tmplevel, (p1 + p2) / 2.0);
                            }

                            if (aTestState != TopAbs_OUT)
                            {
                                ES.Hide(p1, tol1, p2, tol2,
                                        false, // under  the Face
                                        false); // inside the Face
                            }

                            EB.NextEdge();
                        }

                        EB.Builds(TopAbs_ON); // build parts under the boundary
                                              // ******************************
                        while (EB.MoreEdges())
                        {
                            p1 = 0.0;
                            p2 = 0.0;
                            int aMaskP1P2 = 0;
                            while (EB.MoreVertices())
                            {
                                switch (EB.Orientation())
                                {
                                    case TopAbs_FORWARD:
                                        p1 = EB.Current().Parameter();
                                        tol1 = EB.Current().Tolerance();
                                        aMaskP1P2 |= 1;
                                        break;
                                    case TopAbs_REVERSED:
                                        p2 = EB.Current().Parameter();
                                        tol2 = EB.Current().Tolerance();
                                        aMaskP1P2 |= 2;
                                        break;
                                    case TopAbs_INTERNAL:
                                    case TopAbs_EXTERNAL:
                                        break;
                                }
                                EB.NextVertex();
                            }

                            if (aMaskP1P2 != 3 || p2 - p1 <= 1.0e-7)
                            {
                                EB.NextEdge();
                                continue;
                            }

                            TopAbs_State aTestState = TopAbs_IN;
                            if (IsSuspicion)
                            {
                                // int aNbp = 1;
                                // aTestState = myDS->SimplClassify(E, ed, aNbp, p1, p2);
                                int tmplevel = 0;
                                aTestState = myDS.Classify(E, ed, true, ref tmplevel, (p1 + p2) / 2.0);
                            }

                            if (aTestState != TopAbs_OUT)
                            {
                                ES.Hide(p1, tol1, p2, tol2,
                                        false, // under the Face
                                        true); // on the boundary
                            }

                            EB.NextEdge();
                        }
                    }

                    if (!ILOn.IsEmpty())
                    {
                        int level = 0;
                        if (!myDS.SimpleHidingFace())
                        { // Level at Start
                            level = myDS.HidingStartLevel(E, ed, ILOn); // **************
                        }
                        if (level > 0)
                        {
                            NCollection_List<HLRAlgo_Interference>.Iterator It = new(ILOn);

                            while (It.More())
                            { // suppress multi-inside Intersections
                              // ***********************************

                                HLRAlgo_Interference Int = It.Value();
                                switch (Int.Transition())
                                {
                                    case TopAbs_FORWARD:
                                    {
                                        int decal = Int.Intersection().Level();
                                        if (level > 0)
                                        {
                                            ILOn.Remove(It);
                                        }
                                        else
                                        {
                                            It.Next();
                                        }
                                        level = level + decal;
                                    }
                                        break;
                                    case TopAbs_REVERSED:
                                        level = level - Int.Intersection().Level();
                                        if (level > 0)
                                        {
                                            ILOn.Remove(It);
                                        }
                                        else
                                        {
                                            It.Next();
                                        }
                                        break;
                                    case TopAbs_EXTERNAL:
                                    case TopAbs_INTERNAL:
                                    default:
                                        It.Next();
                                        break;
                                }
                            }
                            if (ILOn.IsEmpty() && !foundHidden)
                            { // Edge hidden
                                ES.HideAll(); // ***********
                            }
                        }
                    }

                    if (!ILOn.IsEmpty())
                    {
                        HLRBRep_VertexList IL = new(EIT, new NCollection_List<HLRAlgo_Interference>.Iterator(ILOn));
                        HLRBRep_EdgeBuilder EB = new(IL);

                        EB.Builds(TopAbs_IN); // build parts on the Face
                                              // ***********************
                        while (EB.MoreEdges())
                        {
                            p1 = 0.0;
                            p2 = 0.0;
                            int aMaskP1P2 = 0;
                            while (EB.MoreVertices())
                            {
                                switch (EB.Orientation())
                                {
                                    case TopAbs_FORWARD:
                                        p1 = EB.Current().Parameter();
                                        tol1 = EB.Current().Tolerance();
                                        aMaskP1P2 |= 1;
                                        break;
                                    case TopAbs_REVERSED:
                                        p2 = EB.Current().Parameter();
                                        tol2 = EB.Current().Tolerance();
                                        aMaskP1P2 |= 2;
                                        break;
                                    case TopAbs_INTERNAL:
                                    case TopAbs_EXTERNAL:
                                        break;
                                }
                                EB.NextVertex();
                            }

                            if (aMaskP1P2 != 3 || p2 - p1 <= 1.0e-7)
                            {
                                EB.NextEdge();
                                continue;
                            }

                            ES.Hide(p1, tol1, p2, tol2,
                                    true, // on     the Face
                                    false); // inside the Face
                            EB.NextEdge();
                        }

                        EB.Builds(TopAbs_ON); // build hidden parts under the boundary
                                              // *************************************
                        while (EB.MoreEdges())
                        {
                            p1 = 0.0;
                            p2 = 0.0;
                            int aMaskP1P2 = 0;
                            while (EB.MoreVertices())
                            {
                                switch (EB.Orientation())
                                {
                                    case TopAbs_FORWARD:
                                        p1 = EB.Current().Parameter();
                                        tol1 = EB.Current().Tolerance();
                                        aMaskP1P2 |= 1;
                                        break;
                                    case TopAbs_REVERSED:
                                        p2 = EB.Current().Parameter();
                                        tol2 = EB.Current().Tolerance();
                                        aMaskP1P2 |= 2;
                                        break;
                                    case TopAbs_INTERNAL:
                                    case TopAbs_EXTERNAL:
                                        break;
                                }
                                EB.NextVertex();
                            }

                            if (aMaskP1P2 != 3 || p2 - p1 <= 1.0e-7)
                            {
                                EB.NextEdge();
                                continue;
                            }

                            ES.Hide(p1, tol1, p2, tol2,
                                    true, // on the Face
                                    true); // on the boundary
                            EB.NextEdge();
                        }
                    }
                }
            }
            catch (Exception anException) when (IsCaught(anException))
            {
                // OCCT_DEBUG trace left out
            }
        }
    }
}
