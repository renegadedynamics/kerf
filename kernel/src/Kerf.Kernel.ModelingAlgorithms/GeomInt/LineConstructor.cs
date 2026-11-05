// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_LineConstructor.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_LineConstructor.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_LineConstructor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Foundation.Std;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Surfaces;

/// <summary>
/// Splits an intersection line into the parts that lie inside both surfaces' domains: <see cref="Perform"/> keeps
/// the parameter intervals between consecutive vertices whose middle point both domains classify as not out.
/// </summary>
/// <remarks>
/// A working object, as OCCT's: <see cref="Load"/> the domains and surfaces, then <see cref="Perform"/> for each
/// line. OCCT's file-local GeomInt_Vertex is a private nested struct, sorted as MSVC's <c>std::sort</c> does
/// (decision 621); the file-static functions are private static methods.
/// </remarks>
internal class LineConstructor
{
    private static readonly double TwoPI = M_PI + M_PI;

    /// <summary>A vertex sortable by its parameter on the line (OCCT's file-local class).</summary>
    private struct GeomInt_Vertex
    {
        private PatchPoint _vertex;

        public GeomInt_Vertex() => _vertex = new PatchPoint();

        //! Initializes this class by IntPatch_Point
        public void SetVertex(in PatchPoint other)
        {
            _vertex = other;
            double newParam = global::Kerf.Engine.Geometry.ElCLib.InPeriod(other.ParameterOnLine(), 0.0, TwoPI);
            SetParameter(newParam);
        }

        //! Sets Parameter on Line
        public void SetParameter(double param) => _vertex.SetParameter(param);

        //! Returns IntPatch_Point
        public readonly PatchPoint Getvertex() => _vertex;
    }

    //! To provide sort
    private readonly struct VertexLess : Heap.ILess<GeomInt_Vertex>
    {
        public bool Less(GeomInt_Vertex left, GeomInt_Vertex right)
            => left.Getvertex().ParameterOnLine() < right.Getvertex().ParameterOnLine();
    }

    private bool _done;
    private readonly NCollection_Sequence<double> _seqp = new();
    private TopolTool? _dom1;
    private TopolTool? _dom2;
    private SurfaceAdaptor? _hs1;
    private SurfaceAdaptor? _hs2;

    public LineConstructor()
    {
        _done = false;
    }

    /// <summary>Gives the domains and the surfaces of the intersection.</summary>
    public void Load(TopolTool d1, TopolTool d2, SurfaceAdaptor? s1, SurfaceAdaptor? s2)
    {
        _dom1 = d1;
        _dom2 = d2;
        _hs1 = s1;
        _hs2 = s2;
    }

    /// <summary>True if <see cref="Perform"/> succeeded.</summary>
    public bool IsDone => _done;

    /// <summary>The number of parts kept. Throws StdFail_NotDone if not done.</summary>
    public int PartCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return (_seqp.Length() / 2);
        }
    }

    /// <summary>The parameters on the line of the ends of part I. Throws StdFail_NotDone if not done.</summary>
    public void Part(int i, out double wFirst, out double wLast)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        wFirst = _seqp[2 * i - 1];
        wLast = _seqp[2 * i];
    }

    /// <summary>Splits the line L into the parts inside both domains.</summary>
    public void Perform(PatchLine l)
    {
        int i, nbvtx;
        double firstp, lastp;
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion() * 35.0;

        PatchLineType typl = l.ArcType();
        if (typl == PatchLineType.Analytic)
        {
            double u1, v1, u2, v2;
            ALine aLine = (l as ALine)!;
            _seqp.Clear();
            nbvtx = LineTool.NbVertex(l);
            for (i = 1; i < nbvtx; i++)
            {
                firstp = LineTool.Vertex(l, i).ParameterOnLine();
                lastp = LineTool.Vertex(l, i + 1).ParameterOnLine();
                if (firstp != lastp)
                {
                    double pmid = (firstp + lastp) * 0.5;
                    Point3D Pmid = aLine.Value(pmid);
                    Parameters(_hs1!, _hs2!, Pmid, out u1, out v1, out u2, out v2);
                    AdjustPeriodic(_hs1!, _hs2!, ref u1, ref v1, ref u2, ref v2);
                    Containment in1 = _dom1!.Classify(new Point2D(u1, v1), tol);
                    if (in1 != global::Kerf.Topology.Containment.Out)
                    {
                        Containment in2 = _dom2!.Classify(new Point2D(u2, v2), tol);
                        if (in2 != global::Kerf.Topology.Containment.Out)
                        {
                            _seqp.Append(firstp);
                            _seqp.Append(lastp);
                        }
                    }
                }
            }
            _done = true;
            return;
        } // if(typl == IntPatch_Analytic)  {
        else if (typl == PatchLineType.Walking)
        {
            double u1, v1, u2, v2;
            WLine wLine = (l as WLine)!;
            _seqp.Clear();
            nbvtx = LineTool.NbVertex(l);
            for (i = 1; i < nbvtx; i++)
            {
                firstp = LineTool.Vertex(l, i).ParameterOnLine();
                lastp = LineTool.Vertex(l, i + 1).ParameterOnLine();
                if (firstp != lastp)
                {
                    if (lastp != firstp + 1)
                    {
                        int pmid = (int)((firstp + lastp) / 2);
                        PntOn2S Pmid = wLine.Point(pmid);
                        Pmid.Parameters(out u1, out v1, out u2, out v2);
                        AdjustPeriodic(_hs1!, _hs2!, ref u1, ref v1, ref u2, ref v2);
                        Containment in1 = _dom1!.Classify(new Point2D(u1, v1), tol);
                        if (in1 != global::Kerf.Topology.Containment.Out)
                        {
                            Containment in2 = _dom2!.Classify(new Point2D(u2, v2), tol);
                            if (in2 != global::Kerf.Topology.Containment.Out)
                            {
                                _seqp.Append(firstp);
                                _seqp.Append(lastp);
                            }
                        }
                    }
                    else
                    {
                        if (wLine.GetCreatingWay() == global::Kerf.Engine.Intersection.Patches.WLine.IntPatch_WLType.WLImpPrm)
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

                            double u21, v21, u22, v22;
                            PntOn2S aPfirst = wLine.Point((int)(firstp));
                            PntOn2S aPlast = wLine.Point((int)(lastp));
                            aPfirst.Parameters(out u1, out v1, out u2, out v2);
                            AdjustPeriodic(_hs1!, _hs2!, ref u1, ref v1, ref u2, ref v2);
                            aPlast.Parameters(out u21, out v21, out u22, out v22);
                            AdjustPeriodic(_hs1!, _hs2!, ref u21, ref v21, ref u22, ref v22);

                            u1 = 0.5 * (u1 + u21);
                            v1 = 0.5 * (v1 + v21);
                            u2 = 0.5 * (u2 + u22);
                            v2 = 0.5 * (v2 + v22);

                            Containment in1 = _dom1!.Classify(new Point2D(u1, v1), tol);
                            if (in1 != global::Kerf.Topology.Containment.Out)
                            {
                                Containment in2 = _dom2!.Classify(new Point2D(u2, v2), tol);
                                if (in2 != global::Kerf.Topology.Containment.Out)
                                {
                                    _seqp.Append(firstp);
                                    _seqp.Append(lastp);
                                }
                            }
                        }
                        else
                        {
                            PntOn2S pfirst = wLine.Point((int)(firstp));
                            pfirst.Parameters(out u1, out v1, out u2, out v2);
                            AdjustPeriodic(_hs1!, _hs2!, ref u1, ref v1, ref u2, ref v2);
                            Containment in1 = _dom1!.Classify(new Point2D(u1, v1), tol);
                            if (in1 != global::Kerf.Topology.Containment.Out)
                            {
                                Containment in2 = _dom2!.Classify(new Point2D(u2, v2), tol);
                                if (in2 != global::Kerf.Topology.Containment.Out)
                                {
                                    PntOn2S plast = wLine.Point((int)(lastp));
                                    plast.Parameters(out u1, out v1, out u2, out v2);
                                    AdjustPeriodic(_hs1!, _hs2!, ref u1, ref v1, ref u2, ref v2);
                                    in1 = _dom1.Classify(new Point2D(u1, v1), tol);
                                    if (in1 != global::Kerf.Topology.Containment.Out)
                                    {
                                        in2 = _dom2.Classify(new Point2D(u2, v2), tol);
                                        if (in2 != global::Kerf.Topology.Containment.Out)
                                        {
                                            _seqp.Append(firstp);
                                            _seqp.Append(lastp);
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
            int nbParts;
            //
            nbParts = _seqp.Length() / 2;
            if (nbParts > 1)
            {
                bool bCond;
                SurfaceKind st1, st2;
                st1 = _hs1!.GetType();
                st2 = _hs2!.GetType();
                //
                bCond = false;
                if (st1 == Plane)
                {
                    if (st2 == SurfaceOfExtrusion || st2 == SurfaceOfRevolution)
                    { //+zft
                        bCond = !bCond;
                    }
                }
                else if (st2 == Plane)
                {
                    if (st1 == SurfaceOfExtrusion || st1 == SurfaceOfRevolution)
                    { //+zft
                        bCond = !bCond;
                    }
                }
                //
                if (bCond)
                {
                    int nb, index, nbTmp, jx;
                    IndexedSet<int> map = new();
                    NCollection_Sequence<double> seqTmp = new();
                    //
                    nb = _seqp.Length();
                    for (i = 1; i <= nb; ++i)
                    {
                        lastp = _seqp[i];
                        index = (int)lastp;
                        if (!map.Contains(index))
                        {
                            map.Add(index);
                            seqTmp.Append(lastp);
                        }
                        else
                        {
                            nbTmp = seqTmp.Length();
                            seqTmp.Remove(nbTmp);
                        }
                    }
                    //
                    _seqp.Clear();
                    //
                    nb = seqTmp.Length() / 2;
                    for (i = 1; i <= nb; ++i)
                    {
                        jx = 2 * i;
                        firstp = seqTmp[jx - 1];
                        lastp = seqTmp[jx];
                        _seqp.Append(firstp);
                        _seqp.Append(lastp);
                    }
                } // if (bCond) {
            }
            _done = true;
            return;
        } // else if(typl == IntPatch_Walking)  {
        //
        //-----------------------------------------------------------
        else if (typl != PatchLineType.Restriction)
        {
            _seqp.Clear();
            //
            GLine gLine = (l as GLine)!;
            //
            if (typl == PatchLineType.Circle || typl == PatchLineType.Ellipse)
            {
                TreatCircle(l, tol);
                _done = true;
                return;
            }
            //----------------------------
            bool intrvtested;
            double u1, v1, u2, v2;
            //
            nbvtx = LineTool.NbVertex(l);
            intrvtested = false;
            for (i = 1; i < nbvtx; ++i)
            {
                firstp = LineTool.Vertex(l, i).ParameterOnLine();
                lastp = LineTool.Vertex(l, i + 1).ParameterOnLine();
                if (Math.Abs(firstp - lastp) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    intrvtested = true;
                    double pmid = (firstp + lastp) * 0.5;
                    GLinePoint(typl, gLine, pmid, out Point3D Pmid);
                    //
                    Parameters(_hs1!, _hs2!, Pmid, out u1, out v1, out u2, out v2);
                    AdjustPeriodic(_hs1!, _hs2!, ref u1, ref v1, ref u2, ref v2);
                    Containment in1 = _dom1!.Classify(new Point2D(u1, v1), tol);
                    if (in1 != global::Kerf.Topology.Containment.Out)
                    {
                        Containment in2 = _dom2!.Classify(new Point2D(u2, v2), tol);
                        if (in2 != global::Kerf.Topology.Containment.Out)
                        {
                            _seqp.Append(firstp);
                            _seqp.Append(lastp);
                        }
                    }
                }
            }
            //
            if (!intrvtested)
            {
                // Keep a priori. A point 2d on each
                // surface is required to make the decision. Will be done in the caller
                _seqp.Append(LineTool.FirstParameter(l));
                _seqp.Append(LineTool.LastParameter(l));
            }
            //
            _done = true;
            return;
        } // else if (typl != IntPatch_Restriction)  {

        _done = false;
        _seqp.Clear();
        nbvtx = LineTool.NbVertex(l);
        if (nbvtx == 0)
        { // Keep a priori. Point 2d is required on each
            // surface to make the decision. Will be done in the caller
            _seqp.Append(LineTool.FirstParameter(l));
            _seqp.Append(LineTool.LastParameter(l));
            _done = true;
            return;
        }

        NCollection_Sequence<ParameterAndOrientation> seqpss = new();
        Orientation or1 = Forward, or2 = Forward;

        for (i = 1; i <= nbvtx; i++)
        {
            PatchPoint thevtx = LineTool.Vertex(l, i);
            double prm = thevtx.ParameterOnLine();
            if (thevtx.IsOnDomS1)
            {
                switch (thevtx.TransitionLineArc1().TransitionType())
                {
                    case WalkingTypeTrans.In:
                        or1 = Forward;
                        break;
                    case WalkingTypeTrans.Out:
                        or1 = Reversed;
                        break;
                    case WalkingTypeTrans.Touch:
                        or1 = Internal;
                        break;
                    case WalkingTypeTrans.Undecided:
                        or1 = Internal;
                        break;
                }
            }
            else
            {
                or1 = Internal;
            }

            if (thevtx.IsOnDomS2)
            {
                switch (thevtx.TransitionLineArc2().TransitionType())
                {
                    case WalkingTypeTrans.In:
                        or2 = Forward;
                        break;
                    case WalkingTypeTrans.Out:
                        or2 = Reversed;
                        break;
                    case WalkingTypeTrans.Touch:
                        or2 = Internal;
                        break;
                    case WalkingTypeTrans.Undecided:
                        or2 = Internal;
                        break;
                }
            }
            else
            {
                or2 = Internal;
            }
            //
            int nbinserted = seqpss.Length();
            bool inserted = false;
            for (int j = 1; j <= nbinserted; j++)
            {
                if (Math.Abs(prm - seqpss[j].Parameter()) <= tol)
                {
                    // accumulate
                    ref ParameterAndOrientation valj = ref seqpss.ChangeValue(j);
                    if (or1 != Internal)
                    {
                        if (valj.Orientation1() != Internal)
                        {
                            if (or1 != valj.Orientation1())
                            {
                                valj.SetOrientation1(Internal);
                            }
                        }
                        else
                        {
                            valj.SetOrientation1(or1);
                        }
                    }

                    if (or2 != Internal)
                    {
                        if (valj.Orientation2() != Internal)
                        {
                            if (or2 != valj.Orientation2())
                            {
                                valj.SetOrientation2(Internal);
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

                if (prm < seqpss[j].Parameter() - tol)
                {
                    // insert before position j
                    seqpss.InsertBefore(j, new ParameterAndOrientation(prm, or1, or2));
                    inserted = true;
                    break;
                }
            }
            if (!inserted)
            {
                seqpss.Append(new ParameterAndOrientation(prm, or1, or2));
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
            if (or1 != Internal)
            {
                trim = true;
                dansS1 = (or1 != Forward);
                break;
            }
        }

        if (i > nbvtx)
        {
            double u, v;
            for (i = 1; i <= LineTool.NbVertex(l); i++)
            {
                if (!LineTool.Vertex(l, i).IsOnDomS1)
                {
                    LineTool.Vertex(l, i).ParametersOnS1(out u, out v);
                    Point2D ppcc = new(u, v);
                    if (_dom1!.Classify(ppcc, tol) == global::Kerf.Topology.Containment.Out)
                    {
                        _done = true;
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
            if (or2 != Internal)
            {
                trim = true;
                dansS2 = (or2 != Forward);
                break;
            }
        }

        if (i > nbvtx)
        {
            double u, v;
            for (i = 1; i <= LineTool.NbVertex(l); i++)
            {
                if (!LineTool.Vertex(l, i).IsOnDomS2)
                {
                    LineTool.Vertex(l, i).ParametersOnS2(out u, out v);
                    if (_dom2!.Classify(new Point2D(u, v), tol) == global::Kerf.Topology.Containment.Out)
                    {
                        _done = true;
                        return;
                    }
                    break;
                }
            }
            dansS2 = true; //  Keep in doubt
        }

        if (!trim)
        { // necessarily dansS1 == dansS2 == true
            _seqp.Append(LineTool.FirstParameter(l));
            _seqp.Append(LineTool.LastParameter(l));
            _done = true;
            return;
        }

        // sequence seqpss is peeled to create valid ends
        // and store them in seqp(2*i+1) and seqp(2*i+2)
        double thefirst = LineTool.FirstParameter(l);
        double thelast = LineTool.LastParameter(l);
        firstp = thefirst;

        for (i = 1; i <= nbvtx; i++)
        {
            or1 = seqpss[i].Orientation1();
            or2 = seqpss[i].Orientation2();
            if (dansS1 && dansS2)
            {
                if (or1 == Reversed)
                {
                    dansS1 = false;
                }

                if (or2 == Reversed)
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
                        _seqp.Append(stofirst);
                        _seqp.Append(stolast);
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
                    if (or1 == Reversed)
                    {
                        dansS1 = false;
                    }
                }
                else
                {
                    if (or1 == Forward)
                    {
                        dansS1 = true;
                    }
                }
                if (dansS2)
                {
                    if (or2 == Reversed)
                    {
                        dansS2 = false;
                    }
                }
                else
                {
                    if (or2 == Forward)
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
                _seqp.Append(firstp);
                _seqp.Append(lastp);
            }
        }
        _done = true;
    }

    /// <summary>Splits a circle or an ellipse at its vertices (sorted in [0, 2π[) into the arcs inside both
    /// domains.</summary>
    protected void TreatCircle(PatchLine line, double tol)
    {
        PatchLineType type = line.ArcType();
        GLine gLine = (line as GLine)!;
        if (RejectMicroCircle(gLine, type, tol))
        {
            return;
        }
        //----------------------------------------
        int nbVtx = gLine.VertexCount;
        NCollection_Array1<GeomInt_Vertex> vtxArr = new(1, nbVtx + 1);
        for (int i = 1; i <= nbVtx; i++)
        {
            vtxArr[i].SetVertex(gLine.Vertex(i));
        }

        Sorting.Sort(vtxArr.AsSpan()[..nbVtx], new VertexLess());

        // Create last vertex
        double minPrm = vtxArr.First().Getvertex().ParameterOnLine() + TwoPI;
        vtxArr.ChangeLast().SetParameter(minPrm);

        RejectDuplicates(vtxArr);

        Sorting.Sort(vtxArr.AsSpan(), new VertexLess());

        double u1, v1, u2, v2;
        Point2D p2D = new();
        for (int i = vtxArr.Lower(); i <= vtxArr.Upper() - 1; i++)
        {
            double t1 = vtxArr[i].Getvertex().ParameterOnLine();
            double t2 = vtxArr[i + 1].Getvertex().ParameterOnLine();

            if (t2 == RealLast())
            {
                break;
            }

            double tmid = (t1 + t2) * 0.5;
            GLinePoint(type, gLine, tmid, out Point3D pmid);
            //
            Parameters(_hs1!, _hs2!, pmid, out u1, out v1, out u2, out v2);
            AdjustPeriodic(_hs1!, _hs2!, ref u1, ref v1, ref u2, ref v2);
            //
            p2D = p2D.WithCoord(u1, v1);
            Containment state = _dom1!.Classify(p2D, tol);
            if (state != global::Kerf.Topology.Containment.Out)
            {
                p2D = p2D.WithCoord(u2, v2);
                state = _dom2!.Classify(p2D, tol);
                if (state != global::Kerf.Topology.Containment.Out)
                {
                    _seqp.Append(t1);
                    _seqp.Append(t2);
                }
            }
        }
    }

    private static void AdjustPeriodic(SurfaceAdaptor myHS1, SurfaceAdaptor myHS2, ref double u1,
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
            global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(u1, f, l, lmf, out u1, out _);
        }
        if (myHS1IsVPeriodic)
        {
            double lmf = M_PI + M_PI; //-- myHS1->VPeriod();
            double f = myHS1.FirstVParameter();
            double l = myHS1.LastVParameter();
            global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(v1, f, l, lmf, out v1, out _);
        }
        if (myHS2IsUPeriodic)
        {
            double lmf = M_PI + M_PI; //-- myHS2->UPeriod();
            double f = myHS2.FirstUParameter();
            double l = myHS2.LastUParameter();
            global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(u2, f, l, lmf, out u2, out _);
        }
        if (myHS2IsVPeriodic)
        {
            double lmf = M_PI + M_PI; //-- myHS2->VPeriod();
            double f = myHS2.FirstVParameter();
            double l = myHS2.LastVParameter();
            global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(v2, f, l, lmf, out v2, out _);
        }
    }

    private static void Parameters(SurfaceAdaptor myHS1, SurfaceAdaptor myHS2, in Point3D ptref,
                                   out double u1, out double v1, out double u2, out double v2)
    {
        Parameters(myHS1, ptref, out u1, out v1);
        Parameters(myHS2, ptref, out u2, out v2);
    }

    private static void Parameters(SurfaceAdaptor myHS1, in Point3D ptref, out double u1, out double v1)
    {
        SurfaceQuadric quad1 = new();
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
        u1 = 0.0;
        v1 = 0.0;
        quad1.Parameters(ptref, ref u1, ref v1);
    }

    private static void GLinePoint(PatchLineType typl, GLine gLine, double aT, out Point3D aP)
    {
        switch (typl)
        {
            case PatchLineType.Lin:
                aP = global::Kerf.Engine.Geometry.ElCLib.Value(aT, gLine.Line());
                break;
            case PatchLineType.Circle:
                aP = global::Kerf.Engine.Geometry.ElCLib.Value(aT, gLine.Circle());
                break;
            case PatchLineType.Ellipse:
                aP = global::Kerf.Engine.Geometry.ElCLib.Value(aT, gLine.Ellipse());
                break;
            case PatchLineType.Hyperbola:
                aP = global::Kerf.Engine.Geometry.ElCLib.Value(aT, gLine.Hyperbola());
                break;
            case PatchLineType.Parabola:
                aP = global::Kerf.Engine.Geometry.ElCLib.Value(aT, gLine.Parabola());
                break;
            default:
                throw new ConstructionErrorException("GeomInt_LineConstructor::Parameters");
        }
    }

    private static bool RejectMicroCircle(GLine aGLine, PatchLineType aType, double aTol3D)
    {
        bool bRet;
        double r;
        //
        bRet = false;
        //
        if (aType == PatchLineType.Circle)
        {
            r = aGLine.Circle().Radius;
            bRet = (r < aTol3D);
        }
        else if (aType == PatchLineType.Ellipse)
        {
            r = aGLine.Ellipse().MajorRadius;
            bRet = (r < aTol3D);
        }
        return bRet;
    }

    // Finds two coincident IntPatch_Points (if they exist) and sets Parameter-On-Line fore one such point to DBL_MAX
    // (i.e. its use in the future is forbidden). The source array must be sorted in ascending order.
    private static void RejectDuplicates(NCollection_Array1<GeomInt_Vertex> vtxArr)
    {
        // About the value aTolPC=1000.*Precision::PConfusion(),
        // see IntPatch_GLine::ComputeVertexParameters(...)
        // for more details;
        double tolPC = 1000.0 * global::Kerf.Engine.Foundation.Precision.PConfusion();

        // Find duplicates in a slice of the array [LowerBound, UpperBound-1].
        // If a duplicate has been found, the element with greater index will be rejected.
        for (int i = vtxArr.Lower(); i <= vtxArr.Upper() - 2; i++)
        {
            PatchPoint vi = vtxArr[i].Getvertex();
            double prmi = vi.ParameterOnLine();

            if (prmi == RealLast())
            {
                continue;
            }

            for (int j = i + 1; j <= vtxArr.Upper() - 1; j++)
            {
                PatchPoint vj = vtxArr[j].Getvertex();
                double prmj = vj.ParameterOnLine();

                if (prmj - prmi < tolPC)
                {
                    vtxArr[j].SetParameter(RealLast());
                }
                else
                {
                    break;
                }
            }
        }

        // Find duplicates with the last element of the array.
        // If a duplicate has been found, the found element will be rejected.
        double maxPrm = vtxArr.Last().Getvertex().ParameterOnLine();
        for (int i = vtxArr.Upper() - 1; i > vtxArr.Lower(); i--)
        {
            PatchPoint vi = vtxArr[i].Getvertex();
            double prmi = vi.ParameterOnLine();

            if (prmi == RealLast())
            {
                continue;
            }

            if ((maxPrm - prmi) < tolPC)
            {
                vtxArr[i].SetParameter(RealLast());
            }
            else
            {
                break;
            }
        }
    }
}
