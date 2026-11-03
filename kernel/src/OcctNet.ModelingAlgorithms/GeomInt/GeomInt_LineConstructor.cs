// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_LineConstructor.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_LineConstructor.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_LineConstructor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Foundation.Std;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Occt.IntPatch_IType;
using static Occt.IntSurf_TypeTrans;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Splits an intersection line into the parts that lie inside both surfaces' domains: <see cref="Perform"/> keeps
/// the parameter intervals between consecutive vertices whose middle point both domains classify as not out.
/// </summary>
/// <remarks>
/// A working object, as OCCT's: <see cref="Load"/> the domains and surfaces, then <see cref="Perform"/> for each
/// line. OCCT's file-local GeomInt_Vertex is a private nested struct, sorted as MSVC's <c>std::sort</c> does
/// (decision 621); the file-static functions are private static methods.
/// </remarks>
public class GeomInt_LineConstructor
{
    private static readonly double TwoPI = M_PI + M_PI;

    /// <summary>A vertex sortable by its parameter on the line (OCCT's file-local class).</summary>
    private struct GeomInt_Vertex
    {
        private IntPatch_Point myVertex;

        public GeomInt_Vertex() => myVertex = new IntPatch_Point();

        //! Initializes this class by IntPatch_Point
        public void SetVertex(in IntPatch_Point theOther)
        {
            myVertex = theOther;
            double aNewParam = global::Kerf.Engine.Geometry.ElCLib.InPeriod(theOther.ParameterOnLine(), 0.0, TwoPI);
            SetParameter(aNewParam);
        }

        //! Sets Parameter on Line
        public void SetParameter(double theParam) => myVertex.SetParameter(theParam);

        //! Returns IntPatch_Point
        public readonly IntPatch_Point Getvertex() => myVertex;
    }

    //! To provide sort
    private readonly struct VertexLess : Heap.ILess<GeomInt_Vertex>
    {
        public bool Less(GeomInt_Vertex theLeft, GeomInt_Vertex theRight)
            => theLeft.Getvertex().ParameterOnLine() < theRight.Getvertex().ParameterOnLine();
    }

    private bool done;
    private readonly NCollection_Sequence<double> seqp = new();
    private Adaptor3d_TopolTool? myDom1;
    private Adaptor3d_TopolTool? myDom2;
    private GeomAdaptor_Surface? myHS1;
    private GeomAdaptor_Surface? myHS2;

    public GeomInt_LineConstructor()
    {
        done = false;
    }

    /// <summary>Gives the domains and the surfaces of the intersection.</summary>
    public void Load(Adaptor3d_TopolTool D1, Adaptor3d_TopolTool D2, GeomAdaptor_Surface? S1, GeomAdaptor_Surface? S2)
    {
        myDom1 = D1;
        myDom2 = D2;
        myHS1 = S1;
        myHS2 = S2;
    }

    /// <summary>True if <see cref="Perform"/> succeeded.</summary>
    public bool IsDone() => done;

    /// <summary>The number of parts kept. Throws StdFail_NotDone if not done.</summary>
    public int NbParts()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return (seqp.Length() / 2);
    }

    /// <summary>The parameters on the line of the ends of part I. Throws StdFail_NotDone if not done.</summary>
    public void Part(int I, out double WFirst, out double WLast)
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        WFirst = seqp[2 * I - 1];
        WLast = seqp[2 * I];
    }

    /// <summary>Splits the line L into the parts inside both domains.</summary>
    public void Perform(IntPatch_Line L)
    {
        int i, nbvtx;
        double firstp, lastp;
        double Tol = global::Kerf.Engine.Foundation.Precision.PConfusion() * 35.0;

        IntPatch_IType typl = L.ArcType();
        if (typl == IntPatch_Analytic)
        {
            double u1, v1, u2, v2;
            IntPatch_ALine ALine = (L as IntPatch_ALine)!;
            seqp.Clear();
            nbvtx = GeomInt_LineTool.NbVertex(L);
            for (i = 1; i < nbvtx; i++)
            {
                firstp = GeomInt_LineTool.Vertex(L, i).ParameterOnLine();
                lastp = GeomInt_LineTool.Vertex(L, i + 1).ParameterOnLine();
                if (firstp != lastp)
                {
                    double pmid = (firstp + lastp) * 0.5;
                    Point3D Pmid = ALine.Value(pmid);
                    Parameters(myHS1!, myHS2!, Pmid, out u1, out v1, out u2, out v2);
                    AdjustPeriodic(myHS1!, myHS2!, ref u1, ref v1, ref u2, ref v2);
                    TopAbs_State in1 = myDom1!.Classify(new Point2D(u1, v1), Tol);
                    if (in1 != TopAbs_OUT)
                    {
                        TopAbs_State in2 = myDom2!.Classify(new Point2D(u2, v2), Tol);
                        if (in2 != TopAbs_OUT)
                        {
                            seqp.Append(firstp);
                            seqp.Append(lastp);
                        }
                    }
                }
            }
            done = true;
            return;
        } // if(typl == IntPatch_Analytic)  {
        else if (typl == IntPatch_Walking)
        {
            double u1, v1, u2, v2;
            IntPatch_WLine WLine = (L as IntPatch_WLine)!;
            seqp.Clear();
            nbvtx = GeomInt_LineTool.NbVertex(L);
            for (i = 1; i < nbvtx; i++)
            {
                firstp = GeomInt_LineTool.Vertex(L, i).ParameterOnLine();
                lastp = GeomInt_LineTool.Vertex(L, i + 1).ParameterOnLine();
                if (firstp != lastp)
                {
                    if (lastp != firstp + 1)
                    {
                        int pmid = (int)((firstp + lastp) / 2);
                        IntSurf_PntOn2S Pmid = WLine.Point(pmid);
                        Pmid.Parameters(out u1, out v1, out u2, out v2);
                        AdjustPeriodic(myHS1!, myHS2!, ref u1, ref v1, ref u2, ref v2);
                        TopAbs_State in1 = myDom1!.Classify(new Point2D(u1, v1), Tol);
                        if (in1 != TopAbs_OUT)
                        {
                            TopAbs_State in2 = myDom2!.Classify(new Point2D(u2, v2), Tol);
                            if (in2 != TopAbs_OUT)
                            {
                                seqp.Append(firstp);
                                seqp.Append(lastp);
                            }
                        }
                    }
                    else
                    {
                        if (WLine.GetCreatingWay() == IntPatch_WLine.IntPatch_WLType.IntPatch_WLImpPrm)
                        {
                            // The fix #29972.
                            // Implicit-Parametric intersector does not respect domain of
                            // the quadric surface (it takes into account the domain of the
                            // parametric surface only). It means that we cannot warrant that
                            // we have a point exactly in the quadric boundary.
                            // E.g. in the test cases "bugs modalg_5 bug25697_2",
                            //"bugs modalg_5 bug23948", "boolean bopfuse_complex G9",
                            //"boolean bopcommon_complex H7", "boolean bopcut_complex I7" etc.
                            // the WLine contains 2 points and one is out of the quadric's domain.
                            // In order to process these cases correctly, we classify a middle
                            //(between these two) point (obtained by interpolation).

                            // Other types of intersector take into account the domains of both surfaces.
                            // So, they require to reject all "outboundaried" parts of WLine. As result,
                            // more strict check (all two points of WLine are checksed) is
                            // applied in this case.

                            double aU21, aV21, aU22, aV22;
                            IntSurf_PntOn2S aPfirst = WLine.Point((int)(firstp));
                            IntSurf_PntOn2S aPlast = WLine.Point((int)(lastp));
                            aPfirst.Parameters(out u1, out v1, out u2, out v2);
                            AdjustPeriodic(myHS1!, myHS2!, ref u1, ref v1, ref u2, ref v2);
                            aPlast.Parameters(out aU21, out aV21, out aU22, out aV22);
                            AdjustPeriodic(myHS1!, myHS2!, ref aU21, ref aV21, ref aU22, ref aV22);

                            u1 = 0.5 * (u1 + aU21);
                            v1 = 0.5 * (v1 + aV21);
                            u2 = 0.5 * (u2 + aU22);
                            v2 = 0.5 * (v2 + aV22);

                            TopAbs_State in1 = myDom1!.Classify(new Point2D(u1, v1), Tol);
                            if (in1 != TopAbs_OUT)
                            {
                                TopAbs_State in2 = myDom2!.Classify(new Point2D(u2, v2), Tol);
                                if (in2 != TopAbs_OUT)
                                {
                                    seqp.Append(firstp);
                                    seqp.Append(lastp);
                                }
                            }
                        }
                        else
                        {
                            IntSurf_PntOn2S Pfirst = WLine.Point((int)(firstp));
                            Pfirst.Parameters(out u1, out v1, out u2, out v2);
                            AdjustPeriodic(myHS1!, myHS2!, ref u1, ref v1, ref u2, ref v2);
                            TopAbs_State in1 = myDom1!.Classify(new Point2D(u1, v1), Tol);
                            if (in1 != TopAbs_OUT)
                            {
                                TopAbs_State in2 = myDom2!.Classify(new Point2D(u2, v2), Tol);
                                if (in2 != TopAbs_OUT)
                                {
                                    IntSurf_PntOn2S Plast = WLine.Point((int)(lastp));
                                    Plast.Parameters(out u1, out v1, out u2, out v2);
                                    AdjustPeriodic(myHS1!, myHS2!, ref u1, ref v1, ref u2, ref v2);
                                    in1 = myDom1.Classify(new Point2D(u1, v1), Tol);
                                    if (in1 != TopAbs_OUT)
                                    {
                                        in2 = myDom2.Classify(new Point2D(u2, v2), Tol);
                                        if (in2 != TopAbs_OUT)
                                        {
                                            seqp.Append(firstp);
                                            seqp.Append(lastp);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            //
            // The One resulting curve consists of 7 segments that are
            // connected between each other.
            // The aim of the block is to reject these segments and have
            // one segment instead of 7.
            //     The other reason to do that is value of TolReached3D=49.
            //     Why -? It is not known yet.
            //                                             PKV 22.Apr.2002
            //
            int aNbParts;
            //
            aNbParts = seqp.Length() / 2;
            if (aNbParts > 1)
            {
                bool bCond;
                SurfaceKind aST1, aST2;
                aST1 = myHS1!.GetType();
                aST2 = myHS2!.GetType();
                //
                bCond = false;
                if (aST1 == Plane)
                {
                    if (aST2 == SurfaceOfExtrusion || aST2 == SurfaceOfRevolution)
                    { //+zft
                        bCond = !bCond;
                    }
                }
                else if (aST2 == Plane)
                {
                    if (aST1 == SurfaceOfExtrusion || aST1 == SurfaceOfRevolution)
                    { //+zft
                        bCond = !bCond;
                    }
                }
                //
                if (bCond)
                {
                    int aNb, anIndex, aNbTmp, jx;
                    NCollection_IndexedMap<int> aMap = new();
                    NCollection_Sequence<double> aSeqTmp = new();
                    //
                    aNb = seqp.Length();
                    for (i = 1; i <= aNb; ++i)
                    {
                        lastp = seqp[i];
                        anIndex = (int)lastp;
                        if (!aMap.Contains(anIndex))
                        {
                            aMap.Add(anIndex);
                            aSeqTmp.Append(lastp);
                        }
                        else
                        {
                            aNbTmp = aSeqTmp.Length();
                            aSeqTmp.Remove(aNbTmp);
                        }
                    }
                    //
                    seqp.Clear();
                    //
                    aNb = aSeqTmp.Length() / 2;
                    for (i = 1; i <= aNb; ++i)
                    {
                        jx = 2 * i;
                        firstp = aSeqTmp[jx - 1];
                        lastp = aSeqTmp[jx];
                        seqp.Append(firstp);
                        seqp.Append(lastp);
                    }
                } // if (bCond) {
            }
            done = true;
            return;
        } // else if(typl == IntPatch_Walking)  {
        //
        //-----------------------------------------------------------
        else if (typl != IntPatch_Restriction)
        {
            seqp.Clear();
            //
            IntPatch_GLine GLine = (L as IntPatch_GLine)!;
            //
            if (typl == IntPatch_Circle || typl == IntPatch_Ellipse)
            {
                TreatCircle(L, Tol);
                done = true;
                return;
            }
            //----------------------------
            bool intrvtested;
            double u1, v1, u2, v2;
            //
            nbvtx = GeomInt_LineTool.NbVertex(L);
            intrvtested = false;
            for (i = 1; i < nbvtx; ++i)
            {
                firstp = GeomInt_LineTool.Vertex(L, i).ParameterOnLine();
                lastp = GeomInt_LineTool.Vertex(L, i + 1).ParameterOnLine();
                if (Math.Abs(firstp - lastp) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    intrvtested = true;
                    double pmid = (firstp + lastp) * 0.5;
                    GLinePoint(typl, GLine, pmid, out Point3D Pmid);
                    //
                    Parameters(myHS1!, myHS2!, Pmid, out u1, out v1, out u2, out v2);
                    AdjustPeriodic(myHS1!, myHS2!, ref u1, ref v1, ref u2, ref v2);
                    TopAbs_State in1 = myDom1!.Classify(new Point2D(u1, v1), Tol);
                    if (in1 != TopAbs_OUT)
                    {
                        TopAbs_State in2 = myDom2!.Classify(new Point2D(u2, v2), Tol);
                        if (in2 != TopAbs_OUT)
                        {
                            seqp.Append(firstp);
                            seqp.Append(lastp);
                        }
                    }
                }
            }
            //
            if (!intrvtested)
            {
                // Keep a priori. A point 2d on each
                // surface is required to make the decision. Will be done in the caller
                seqp.Append(GeomInt_LineTool.FirstParameter(L));
                seqp.Append(GeomInt_LineTool.LastParameter(L));
            }
            //
            done = true;
            return;
        } // else if (typl != IntPatch_Restriction)  {

        done = false;
        seqp.Clear();
        nbvtx = GeomInt_LineTool.NbVertex(L);
        if (nbvtx == 0)
        { // Keep a priori. Point 2d is required on each
            // surface to make the decision. Will be done in the caller
            seqp.Append(GeomInt_LineTool.FirstParameter(L));
            seqp.Append(GeomInt_LineTool.LastParameter(L));
            done = true;
            return;
        }

        NCollection_Sequence<GeomInt_ParameterAndOrientation> seqpss = new();
        TopAbs_Orientation or1 = TopAbs_FORWARD, or2 = TopAbs_FORWARD;

        for (i = 1; i <= nbvtx; i++)
        {
            IntPatch_Point thevtx = GeomInt_LineTool.Vertex(L, i);
            double prm = thevtx.ParameterOnLine();
            if (thevtx.IsOnDomS1())
            {
                switch (thevtx.TransitionLineArc1().TransitionType())
                {
                    case IntSurf_In:
                        or1 = TopAbs_FORWARD;
                        break;
                    case IntSurf_Out:
                        or1 = TopAbs_REVERSED;
                        break;
                    case IntSurf_Touch:
                        or1 = TopAbs_INTERNAL;
                        break;
                    case IntSurf_Undecided:
                        or1 = TopAbs_INTERNAL;
                        break;
                }
            }
            else
            {
                or1 = TopAbs_INTERNAL;
            }

            if (thevtx.IsOnDomS2())
            {
                switch (thevtx.TransitionLineArc2().TransitionType())
                {
                    case IntSurf_In:
                        or2 = TopAbs_FORWARD;
                        break;
                    case IntSurf_Out:
                        or2 = TopAbs_REVERSED;
                        break;
                    case IntSurf_Touch:
                        or2 = TopAbs_INTERNAL;
                        break;
                    case IntSurf_Undecided:
                        or2 = TopAbs_INTERNAL;
                        break;
                }
            }
            else
            {
                or2 = TopAbs_INTERNAL;
            }
            //
            int nbinserted = seqpss.Length();
            bool inserted = false;
            for (int j = 1; j <= nbinserted; j++)
            {
                if (Math.Abs(prm - seqpss[j].Parameter()) <= Tol)
                {
                    // accumulate
                    ref GeomInt_ParameterAndOrientation valj = ref seqpss.ChangeValue(j);
                    if (or1 != TopAbs_INTERNAL)
                    {
                        if (valj.Orientation1() != TopAbs_INTERNAL)
                        {
                            if (or1 != valj.Orientation1())
                            {
                                valj.SetOrientation1(TopAbs_INTERNAL);
                            }
                        }
                        else
                        {
                            valj.SetOrientation1(or1);
                        }
                    }

                    if (or2 != TopAbs_INTERNAL)
                    {
                        if (valj.Orientation2() != TopAbs_INTERNAL)
                        {
                            if (or2 != valj.Orientation2())
                            {
                                valj.SetOrientation2(TopAbs_INTERNAL);
                            }
                        }
                        else
                        {
                            valj.SetOrientation2(or2);
                        }
                    }
                    inserted = true;
                    break;
                }

                if (prm < seqpss[j].Parameter() - Tol)
                {
                    // insert before position j
                    seqpss.InsertBefore(j, new GeomInt_ParameterAndOrientation(prm, or1, or2));
                    inserted = true;
                    break;
                }
            }
            if (!inserted)
            {
                seqpss.Append(new GeomInt_ParameterAndOrientation(prm, or1, or2));
            }
        }

        // determine the state at the beginning of line
        bool trim = false;
        bool dansS1 = false;
        bool dansS2 = false;

        nbvtx = seqpss.Length();
        for (i = 1; i <= nbvtx; i++)
        {
            or1 = seqpss[i].Orientation1();
            if (or1 != TopAbs_INTERNAL)
            {
                trim = true;
                dansS1 = (or1 != TopAbs_FORWARD);
                break;
            }
        }

        if (i > nbvtx)
        {
            double U, V;
            for (i = 1; i <= GeomInt_LineTool.NbVertex(L); i++)
            {
                if (!GeomInt_LineTool.Vertex(L, i).IsOnDomS1())
                {
                    GeomInt_LineTool.Vertex(L, i).ParametersOnS1(out U, out V);
                    Point2D PPCC = new(U, V);
                    if (myDom1!.Classify(PPCC, Tol) == TopAbs_OUT)
                    {
                        done = true;
                        return;
                    }
                    break;
                }
            }
            dansS1 = true; // Keep in doubt
        }
        //
        for (i = 1; i <= nbvtx; i++)
        {
            or2 = seqpss[i].Orientation2();
            if (or2 != TopAbs_INTERNAL)
            {
                trim = true;
                dansS2 = (or2 != TopAbs_FORWARD);
                break;
            }
        }

        if (i > nbvtx)
        {
            double U, V;
            for (i = 1; i <= GeomInt_LineTool.NbVertex(L); i++)
            {
                if (!GeomInt_LineTool.Vertex(L, i).IsOnDomS2())
                {
                    GeomInt_LineTool.Vertex(L, i).ParametersOnS2(out U, out V);
                    if (myDom2!.Classify(new Point2D(U, V), Tol) == TopAbs_OUT)
                    {
                        done = true;
                        return;
                    }
                    break;
                }
            }
            dansS2 = true; //  Keep in doubt
        }

        if (!trim)
        { // necessarily dansS1 == dansS2 == true
            seqp.Append(GeomInt_LineTool.FirstParameter(L));
            seqp.Append(GeomInt_LineTool.LastParameter(L));
            done = true;
            return;
        }

        // sequence seqpss is peeled to create valid ends
        // and store them in seqp(2*i+1) and seqp(2*i+2)
        double thefirst = GeomInt_LineTool.FirstParameter(L);
        double thelast = GeomInt_LineTool.LastParameter(L);
        firstp = thefirst;

        for (i = 1; i <= nbvtx; i++)
        {
            or1 = seqpss[i].Orientation1();
            or2 = seqpss[i].Orientation2();
            if (dansS1 && dansS2)
            {
                if (or1 == TopAbs_REVERSED)
                {
                    dansS1 = false;
                }

                if (or2 == TopAbs_REVERSED)
                {
                    dansS2 = false;
                }
                if (!dansS1 || !dansS2)
                {
                    lastp = seqpss[i].Parameter();
                    double stofirst = Max(firstp, thefirst);
                    double stolast = Min(lastp, thelast);

                    if (stolast > stofirst)
                    {
                        seqp.Append(stofirst);
                        seqp.Append(stolast);
                    }
                    if (lastp > thelast)
                    {
                        break;
                    }
                }
            }
            else
            {
                if (dansS1)
                {
                    if (or1 == TopAbs_REVERSED)
                    {
                        dansS1 = false;
                    }
                }
                else
                {
                    if (or1 == TopAbs_FORWARD)
                    {
                        dansS1 = true;
                    }
                }
                if (dansS2)
                {
                    if (or2 == TopAbs_REVERSED)
                    {
                        dansS2 = false;
                    }
                }
                else
                {
                    if (or2 == TopAbs_FORWARD)
                    {
                        dansS2 = true;
                    }
                }
                if (dansS1 && dansS2)
                {
                    firstp = seqpss[i].Parameter();
                }
            }
        }
        //
        // finally to add
        if (dansS1 && dansS2)
        {
            lastp = thelast;
            firstp = Max(firstp, thefirst);
            if (lastp > firstp)
            {
                seqp.Append(firstp);
                seqp.Append(lastp);
            }
        }
        done = true;
    }

    /// <summary>Splits a circle or an ellipse at its vertices (sorted in [0, 2π[) into the arcs inside both
    /// domains.</summary>
    protected void TreatCircle(IntPatch_Line theLine, double theTol)
    {
        IntPatch_IType aType = theLine.ArcType();
        IntPatch_GLine aGLine = (theLine as IntPatch_GLine)!;
        if (RejectMicroCircle(aGLine, aType, theTol))
        {
            return;
        }
        //----------------------------------------
        int aNbVtx = aGLine.NbVertex();
        NCollection_Array1<GeomInt_Vertex> aVtxArr = new(1, aNbVtx + 1);
        for (int i = 1; i <= aNbVtx; i++)
        {
            aVtxArr[i].SetVertex(aGLine.Vertex(i));
        }

        Sorting.Sort(aVtxArr.AsSpan()[..aNbVtx], new VertexLess());

        // Create last vertex
        double aMinPrm = aVtxArr.First().Getvertex().ParameterOnLine() + TwoPI;
        aVtxArr.ChangeLast().SetParameter(aMinPrm);

        RejectDuplicates(aVtxArr);

        Sorting.Sort(aVtxArr.AsSpan(), new VertexLess());

        double aU1, aV1, aU2, aV2;
        Point2D aP2D = new();
        for (int i = aVtxArr.Lower(); i <= aVtxArr.Upper() - 1; i++)
        {
            double aT1 = aVtxArr[i].Getvertex().ParameterOnLine();
            double aT2 = aVtxArr[i + 1].Getvertex().ParameterOnLine();

            if (aT2 == RealLast())
            {
                break;
            }

            double aTmid = (aT1 + aT2) * 0.5;
            GLinePoint(aType, aGLine, aTmid, out Point3D aPmid);
            //
            Parameters(myHS1!, myHS2!, aPmid, out aU1, out aV1, out aU2, out aV2);
            AdjustPeriodic(myHS1!, myHS2!, ref aU1, ref aV1, ref aU2, ref aV2);
            //
            aP2D.SetCoord(aU1, aV1);
            TopAbs_State aState = myDom1!.Classify(aP2D, theTol);
            if (aState != TopAbs_OUT)
            {
                aP2D.SetCoord(aU2, aV2);
                aState = myDom2!.Classify(aP2D, theTol);
                if (aState != TopAbs_OUT)
                {
                    seqp.Append(aT1);
                    seqp.Append(aT2);
                }
            }
        }
    }

    private static void AdjustPeriodic(GeomAdaptor_Surface myHS1, GeomAdaptor_Surface myHS2, ref double u1,
                                       ref double v1, ref double u2, ref double v2)
    {
        bool myHS1IsUPeriodic, myHS1IsVPeriodic;
        SurfaceKind typs1 = myHS1.GetType();
        switch (typs1)
        {
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            {
                myHS1IsUPeriodic = true;
                myHS1IsVPeriodic = false;
                break;
            }
            case SurfaceKind.Torus:
            {
                myHS1IsUPeriodic = myHS1IsVPeriodic = true;
                break;
            }
            default:
            {
                //-- Case of periodic biparameters is processed upstream
                myHS1IsUPeriodic = myHS1IsVPeriodic = false;
                break;
            }
        }
        bool myHS2IsUPeriodic, myHS2IsVPeriodic;
        SurfaceKind typs2 = myHS2.GetType();
        switch (typs2)
        {
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            {
                myHS2IsUPeriodic = true;
                myHS2IsVPeriodic = false;
                break;
            }
            case SurfaceKind.Torus:
            {
                myHS2IsUPeriodic = myHS2IsVPeriodic = true;
                break;
            }
            default:
            {
                //-- Case of periodic biparameters is processed upstream
                myHS2IsUPeriodic = myHS2IsVPeriodic = false;
                break;
            }
        }
        //
        if (myHS1IsUPeriodic)
        {
            double lmf = M_PI + M_PI; //-- myHS1->UPeriod();
            double f = myHS1.FirstUParameter();
            double l = myHS1.LastUParameter();
            GeomInt.AdjustPeriodic(u1, f, l, lmf, out u1, out _);
        }
        if (myHS1IsVPeriodic)
        {
            double lmf = M_PI + M_PI; //-- myHS1->VPeriod();
            double f = myHS1.FirstVParameter();
            double l = myHS1.LastVParameter();
            GeomInt.AdjustPeriodic(v1, f, l, lmf, out v1, out _);
        }
        if (myHS2IsUPeriodic)
        {
            double lmf = M_PI + M_PI; //-- myHS2->UPeriod();
            double f = myHS2.FirstUParameter();
            double l = myHS2.LastUParameter();
            GeomInt.AdjustPeriodic(u2, f, l, lmf, out u2, out _);
        }
        if (myHS2IsVPeriodic)
        {
            double lmf = M_PI + M_PI; //-- myHS2->VPeriod();
            double f = myHS2.FirstVParameter();
            double l = myHS2.LastVParameter();
            GeomInt.AdjustPeriodic(v2, f, l, lmf, out v2, out _);
        }
    }

    private static void Parameters(GeomAdaptor_Surface myHS1, GeomAdaptor_Surface myHS2, in Point3D Ptref,
                                   out double U1, out double V1, out double U2, out double V2)
    {
        Parameters(myHS1, Ptref, out U1, out V1);
        Parameters(myHS2, Ptref, out U2, out V2);
    }

    private static void Parameters(GeomAdaptor_Surface myHS1, in Point3D Ptref, out double U1, out double V1)
    {
        IntSurf_Quadric quad1 = new();
        //
        switch (myHS1.GetType())
        {
            case Plane:
                quad1.SetValue(myHS1.Plane());
                break;
            case SurfaceKind.Cylinder:
                quad1.SetValue(myHS1.Cylinder());
                break;
            case SurfaceKind.Cone:
                quad1.SetValue(myHS1.Cone());
                break;
            case SurfaceKind.Sphere:
                quad1.SetValue(myHS1.Sphere());
                break;
            case SurfaceKind.Torus:
                quad1.SetValue(myHS1.Torus());
                break;
            default:
                throw new ConstructionErrorException("GeomInt_LineConstructor::Parameters");
        }
        U1 = 0.0;
        V1 = 0.0;
        quad1.Parameters(Ptref, ref U1, ref V1);
    }

    private static void GLinePoint(IntPatch_IType typl, IntPatch_GLine GLine, double aT, out Point3D aP)
    {
        switch (typl)
        {
            case IntPatch_Lin:
                aP = global::Kerf.Engine.Geometry.ElCLib.Value(aT, GLine.Line());
                break;
            case IntPatch_Circle:
                aP = global::Kerf.Engine.Geometry.ElCLib.Value(aT, GLine.Circle());
                break;
            case IntPatch_Ellipse:
                aP = global::Kerf.Engine.Geometry.ElCLib.Value(aT, GLine.Ellipse());
                break;
            case IntPatch_Hyperbola:
                aP = global::Kerf.Engine.Geometry.ElCLib.Value(aT, GLine.Hyperbola());
                break;
            case IntPatch_Parabola:
                aP = global::Kerf.Engine.Geometry.ElCLib.Value(aT, GLine.Parabola());
                break;
            default:
                throw new ConstructionErrorException("GeomInt_LineConstructor::Parameters");
        }
    }

    private static bool RejectMicroCircle(IntPatch_GLine aGLine, IntPatch_IType aType, double aTol3D)
    {
        bool bRet;
        double aR;
        //
        bRet = false;
        //
        if (aType == IntPatch_Circle)
        {
            aR = aGLine.Circle().Radius();
            bRet = (aR < aTol3D);
        }
        else if (aType == IntPatch_Ellipse)
        {
            aR = aGLine.Ellipse().MajorRadius();
            bRet = (aR < aTol3D);
        }
        return bRet;
    }

    // Finds two coincident IntPatch_Points (if they exist) and sets Parameter-On-Line fore one such point to DBL_MAX
    // (i.e. its use in the future is forbidden). The source array must be sorted in ascending order.
    private static void RejectDuplicates(NCollection_Array1<GeomInt_Vertex> theVtxArr)
    {
        // About the value aTolPC=1000.*Precision::PConfusion(),
        // see IntPatch_GLine::ComputeVertexParameters(...)
        // for more details;
        double aTolPC = 1000.0 * global::Kerf.Engine.Foundation.Precision.PConfusion();

        // Find duplicates in a slice of the array [LowerBound, UpperBound-1].
        // If a duplicate has been found, the element with greater index will be rejected.
        for (int i = theVtxArr.Lower(); i <= theVtxArr.Upper() - 2; i++)
        {
            IntPatch_Point aVi = theVtxArr[i].Getvertex();
            double aPrmi = aVi.ParameterOnLine();

            if (aPrmi == RealLast())
            {
                continue;
            }

            for (int j = i + 1; j <= theVtxArr.Upper() - 1; j++)
            {
                IntPatch_Point aVj = theVtxArr[j].Getvertex();
                double aPrmj = aVj.ParameterOnLine();

                if (aPrmj - aPrmi < aTolPC)
                {
                    theVtxArr[j].SetParameter(RealLast());
                }
                else
                {
                    break;
                }
            }
        }

        // Find duplicates with the last element of the array.
        // If a duplicate has been found, the found element will be rejected.
        double aMaxPrm = theVtxArr.Last().Getvertex().ParameterOnLine();
        for (int i = theVtxArr.Upper() - 1; i > theVtxArr.Lower(); i--)
        {
            IntPatch_Point aVi = theVtxArr[i].Getvertex();
            double aPrmi = aVi.ParameterOnLine();

            if (aPrmi == RealLast())
            {
                continue;
            }

            if ((aMaxPrm - aPrmi) < aTolPC)
            {
                theVtxArr[i].SetParameter(RealLast());
            }
            else
            {
                break;
            }
        }
    }
}
