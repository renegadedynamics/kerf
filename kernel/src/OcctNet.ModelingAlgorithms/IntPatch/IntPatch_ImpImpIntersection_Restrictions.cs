// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntPatch_IType;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

// The restriction lines (OCCT 7.x's IntPatch_ImpImpIntersection_0.gxx, continued): the segments of the boundaries
// that lie on the other surface become RLines (ProcessSegments), and the RLines take the multiple points of the
// other lines, or are removed where they duplicate a GLine (ProcessRLine).
public sealed partial class IntPatch_ImpImpIntersection
{
    // The vertex count, a vertex and the replacement of a vertex of an ALine, RLine or GLine.
    private static int NbVertexOfARG(IntPatch_Line theLine, IntPatch_IType theType) =>
        theType == IntPatch_Analytic ? ((IntPatch_ALine)theLine).NbVertex()
        : theType == IntPatch_Restriction ? ((IntPatch_RLine)theLine).NbVertex()
        : ((IntPatch_GLine)theLine).NbVertex();

    private static IntPatch_Point VertexOfARG(IntPatch_Line theLine, IntPatch_IType theType, int theIndex) =>
        theType == IntPatch_Analytic ? ((IntPatch_ALine)theLine).Vertex(theIndex)
        : theType == IntPatch_Restriction ? ((IntPatch_RLine)theLine).Vertex(theIndex)
        : ((IntPatch_GLine)theLine).Vertex(theIndex);

    private static void ReplaceVertexOfARG(IntPatch_Line theLine, IntPatch_IType theType, int theIndex,
                                           in IntPatch_Point thePnt)
    {
        if (theType == IntPatch_Analytic)
        {
            ((IntPatch_ALine)theLine).Replace(theIndex, thePnt);
        }
        else if (theType == IntPatch_Restriction)
        {
            ((IntPatch_RLine)theLine).Replace(theIndex, thePnt);
        }
        else
        {
            ((IntPatch_GLine)theLine).Replace(theIndex, thePnt);
        }
    }

    // Recomputes the transition of newptvtx on the arc of ptvtx on one surface (ProcessSegments' repeated block).
    private static void SetArcTransition(ref IntPatch_Point newptvtx, in IntPatch_Point ptvtx, bool OnS1,
                                         in IntSurf_Quadric Quad, in gp_Vec tgline, ref gp_Pnt valpt,
                                         ref gp_Vec d1u, ref gp_Vec d1v, ref gp_Vec tgarc,
                                         ref IntSurf_Transition TRest, ref IntSurf_Transition TArc)
    {
        Adaptor2d_Curve2d thearc = OnS1 ? ptvtx.ArcOnS1() : ptvtx.ArcOnS2();
        double aParOnArc = OnS1 ? ptvtx.ParameterOnArc1() : ptvtx.ParameterOnArc2();
        thearc.D1(aParOnArc, out gp_Pnt2d p2d, out gp_Vec2d d2d);
        Quad.D1(p2d.X(), p2d.Y(), ref valpt, ref d1u, ref d1v);
        tgarc.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);
        gp_Vec norm = d1u.Crossed(d1v);
        if (norm.SquareMagnitude() < 1e-16)
        {
            TRest.SetValue(true, IntSurf_Undecided);
            TArc.SetValue(true, IntSurf_Undecided);
        }
        else
        {
            IntSurf.MakeTransition(tgline, tgarc, new gp_Dir(norm), ref TRest, ref TArc);
        }
        newptvtx.SetArc(OnS1, thearc, aParOnArc, TRest, TArc);
    }

    private static void ProcessSegments(NCollection_Sequence<IntPatch_TheSegmentOfTheSOnBounds> listedg,
                                        NCollection_Sequence<IntPatch_Line> slin,
                                        in IntSurf_Quadric Quad1,
                                        in IntSurf_Quadric Quad2,
                                        bool OnFirst,
                                        double TolArc)
    {
        int i, j, k;
        int nbedg = listedg.Length();
        int Nblines, Nbpts;

        Adaptor2d_Curve2d arcRef;
        // OCCT: ptvtx lives across the segments and the vertex loop: an end point that no line carries is built
        // from the last vertex read, keeping that vertex's arcs, vertices and flags where it sets none.
        IntPatch_Point ptvtx = new(), newptvtx;

        IntPatch_RLine rline; //-- On fait rline = new ... par la suite

        IntPatch_TheSegmentOfTheSOnBounds thesegsol;
        IntPatch_ThePathPointOfTheSOnBounds PStartf = new(), PStartl = new();
        bool dofirst, dolast, procf, procl;

        double paramf = 0.0, paraml = 0.0, U1 = 0.0, V1 = 0.0, U2 = 0.0, V2 = 0.0;

        IntPatch_IType typ;
        IntSurf_TypeTrans trans1, trans2;
        IntSurf_Transition TRest = new(), TArc = new();
        gp_Vec tgline = new(), norm1, norm2, tgarc = new();
        gp_Pnt valpt = new();

        gp_Vec d1u = new(), d1v = new();
        gp_Pnt2d p2d;
        gp_Vec2d d2d;

        for (i = 1; i <= nbedg; i++)
        {
            bool EdgeDegenere = false;
            thesegsol = listedg.Value(i);
            arcRef = thesegsol.Curve();

            rline = new IntPatch_RLine(false);
            if (OnFirst)
            {
                rline.SetArcOnS1(arcRef);
            }
            else
            {
                rline.SetArcOnS2(arcRef);
            }

            // Traitement des points debut/fin du segment solution.

            dofirst = false;
            dolast = false;
            procf = false;
            procl = false;

            if (thesegsol.HasFirstPoint())
            {
                dofirst = true;
                PStartf = thesegsol.FirstPoint();
                paramf = PStartf.Parameter();
            }
            if (thesegsol.HasLastPoint())
            {
                dolast = true;
                PStartl = thesegsol.LastPoint();
                paraml = PStartl.Parameter();
            }

            if (dofirst && dolast)
            { // determination de la transition de la ligne
                arcRef.D1(0.5 * (paramf + paraml), out p2d, out d2d);
                if (OnFirst)
                {
                    Quad1.D1(p2d.X(), p2d.Y(), ref valpt, ref d1u, ref d1v);
                }
                else
                {
                    Quad2.D1(p2d.X(), p2d.Y(), ref valpt, ref d1u, ref d1v);
                }
                tgline.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);

                if (d1u.Magnitude() < 1e-7)
                { //-- edge degenere ?
                    EdgeDegenere = true;
                    for (int edg = 0; edg <= 10; edg++)
                    {
                        arcRef.D1(paramf + (paraml - paramf) * edg * 0.1, out p2d, out d2d);
                        if (OnFirst)
                        {
                            Quad1.D1(p2d.X(), p2d.Y(), ref valpt, ref d1u, ref d1v);
                        }
                        else
                        {
                            Quad2.D1(p2d.X(), p2d.Y(), ref valpt, ref d1u, ref d1v);
                        }

                        if (d1u.Magnitude() > 1e-7)
                        {
                            EdgeDegenere = false;
                        }
                    }
                    rline = new IntPatch_RLine(false);
                    if (OnFirst)
                    {
                        rline.SetArcOnS1(arcRef);
                    }
                    else
                    {
                        rline.SetArcOnS2(arcRef);
                    }
                }
                else
                {
                    norm2 = Quad2.Normale(valpt);
                    norm1 = Quad1.Normale(valpt);

                    if (tgline.DotCross(norm2, norm1) > 0.000000001)
                    {
                        trans1 = IntSurf_Out;
                        trans2 = IntSurf_In;
                    }
                    else if (tgline.DotCross(norm2, norm1) < -0.000000001)
                    {
                        trans1 = IntSurf_In;
                        trans2 = IntSurf_Out;
                    }
                    else
                    {
                        trans1 = trans2 = IntSurf_Undecided;
                    }
                    rline = new IntPatch_RLine(false, trans1, trans2);
                    if (OnFirst)
                    {
                        rline.SetArcOnS1(arcRef);
                    }
                    else
                    {
                        rline.SetArcOnS2(arcRef);
                    }
                }
            }
            else
            {
                rline = new IntPatch_RLine(false);
                if (OnFirst)
                {
                    rline.SetArcOnS1(arcRef);
                }
                else
                {
                    rline.SetArcOnS2(arcRef);
                }
            }

            if (dofirst || dolast)
            {
                Nblines = slin.Length();
                for (j = 1; j <= Nblines; j++)
                {
                    IntPatch_Line slinj = slin[j];
                    typ = slinj.ArcType();
                    Nbpts = NbVertexOfARG(slinj, typ);
                    for (k = 1; k <= Nbpts; k++)
                    {
                        ptvtx = VertexOfARG(slinj, typ, k);

                        if (!EdgeDegenere && dofirst)
                        {
                            if (ptvtx.Value().Distance(PStartf.Value()) <= TolArc)
                            {
                                ptvtx.SetMultiple(true);
                                ptvtx.SetTolerance(TolArc);
                                ReplaceVertexOfARG(slinj, typ, k, ptvtx);
                                newptvtx = ptvtx;
                                newptvtx.SetParameter(paramf);
                                // Recalcul des  transitions si point sur restriction

                                arcRef.D1(paramf, out p2d, out d2d);
                                if (OnFirst)
                                {
                                    Quad1.D1(p2d.X(), p2d.Y(), ref valpt, ref d1u, ref d1v);
                                }
                                else
                                {
                                    Quad2.D1(p2d.X(), p2d.Y(), ref valpt, ref d1u, ref d1v);
                                }
                                tgline.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);
                                if (ptvtx.IsOnDomS1())
                                {
                                    SetArcTransition(ref newptvtx, ptvtx, true, Quad1, tgline, ref valpt, ref d1u,
                                                     ref d1v, ref tgarc, ref TRest, ref TArc);
                                }
                                if (ptvtx.IsOnDomS2())
                                {
                                    SetArcTransition(ref newptvtx, ptvtx, false, Quad2, tgline, ref valpt, ref d1u,
                                                     ref d1v, ref tgarc, ref TRest, ref TArc);
                                }

                                rline.AddVertex(newptvtx);
                                if (!procf)
                                {
                                    procf = true;
                                    rline.SetFirstPoint(rline.NbVertex());
                                }
                            }
                        }
                        if (!EdgeDegenere && dolast)
                        {
                            if (ptvtx.Value().Distance(PStartl.Value()) <= TolArc)
                            {
                                ptvtx.SetMultiple(true);
                                ptvtx.SetTolerance(TolArc);
                                ReplaceVertexOfARG(slinj, typ, k, ptvtx);

                                newptvtx = ptvtx;
                                newptvtx.SetParameter(paraml);
                                // Recalcul des  transitions si point sur restriction

                                arcRef.D1(paraml, out p2d, out d2d);
                                if (OnFirst)
                                {
                                    Quad1.D1(p2d.X(), p2d.Y(), ref valpt, ref d1u, ref d1v);
                                }
                                else
                                {
                                    Quad2.D1(p2d.X(), p2d.Y(), ref valpt, ref d1u, ref d1v);
                                }
                                tgline.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);
                                if (ptvtx.IsOnDomS1())
                                {
                                    SetArcTransition(ref newptvtx, ptvtx, true, Quad1, tgline, ref valpt, ref d1u,
                                                     ref d1v, ref tgarc, ref TRest, ref TArc);
                                }
                                if (ptvtx.IsOnDomS2())
                                {
                                    SetArcTransition(ref newptvtx, ptvtx, false, Quad2, tgline, ref valpt, ref d1u,
                                                     ref d1v, ref tgarc, ref TRest, ref TArc);
                                }

                                rline.AddVertex(newptvtx);
                                if (!procl)
                                {
                                    procl = true;
                                    rline.SetLastPoint(rline.NbVertex());
                                }
                            }
                        }
                    }
                    // Si on a traite le pt debut et/ou fin, on ne doit pas recommencer si
                    // il (ils) correspond(ent) a un point multiple.

                    if (procf)
                    {
                        dofirst = false;
                    }
                    if (procl)
                    {
                        dolast = false;
                    }
                }
            }
            // Si on n a pas trouve le point debut et./ou fin sur une des lignes
            // d intersection, il faut quand-meme le placer sur la restriction solution

            if (dofirst)
            {
                ptvtx.SetValue(PStartf.Value(), PStartf.Tolerance(), false);
                Quad1.Parameters(PStartf.Value(), ref U1, ref V1);
                Quad2.Parameters(PStartf.Value(), ref U2, ref V2);
                ptvtx.SetParameters(U1, V1, U2, V2);
                ptvtx.SetParameter(paramf);
                if (!PStartf.IsNew())
                {
                    IntSurf_Transition Transline = new();
                    IntSurf_Transition Transarc = new();
                    ptvtx.SetVertex(OnFirst, PStartf.Vertex());
                    ptvtx.SetArc(OnFirst, PStartf.Arc(), PStartf.Parameter(), Transline, Transarc);
                }

                rline.AddVertex(ptvtx);
                rline.SetFirstPoint(rline.NbVertex());
            }
            if (dolast)
            {
                ptvtx.SetValue(PStartl.Value(), PStartl.Tolerance(), false);
                Quad1.Parameters(PStartl.Value(), ref U1, ref V1);
                Quad2.Parameters(PStartl.Value(), ref U2, ref V2);
                ptvtx.SetParameters(U1, V1, U2, V2);
                ptvtx.SetParameter(paraml);
                if (!PStartl.IsNew())
                {
                    IntSurf_Transition Transline = new();
                    IntSurf_Transition Transarc = new();

                    ptvtx.SetVertex(OnFirst, PStartl.Vertex());
                    ptvtx.SetArc(OnFirst, PStartl.Arc(), PStartl.Parameter(), Transline, Transarc);
                }

                rline.AddVertex(ptvtx);
                rline.SetLastPoint(rline.NbVertex());
            }
            slin.Append(rline);
        }
    }

    // OCCT's PointValue and VertexValue, chosen through a function pointer in IsRLineGood.
    private static gp_Pnt PointValue(IntPatch_RLine theRLine, int theIndex) => theRLine.Point(theIndex).Value();

    private static gp_Pnt VertexValue(IntPatch_RLine theRLine, int theIndex) => theRLine.Vertex(theIndex).Value();

    private static double SquareDistance(IntPatch_GLine theGLine, in gp_Pnt theP, Extrema_ExtPC theExtr)
    {
        double aSQDist = RealLast();
        switch (theGLine.ArcType())
        {
            case IntPatch_Lin:
                aSQDist = theGLine.Line().SquareDistance(theP);
                break;
            case IntPatch_Circle:
                aSQDist = theGLine.Circle().SquareDistance(theP);
                break;
            default:
                theExtr.Perform(theP);
                if (!theExtr.IsDone() || theExtr.NbExt() == 0)
                {
                    // Lines are not overlapped
                    return aSQDist;
                }

                aSQDist = theExtr.SquareDistance(1);
                int aNbExtr = theExtr.NbExt();
                for (int i = 2; i <= aNbExtr; i++)
                {
                    double aSQD = theExtr.SquareDistance(i);
                    if (aSQD < aSQDist)
                    {
                        aSQDist = aSQD;
                    }
                }
                break;
        }

        return aSQDist;
    }

    private static bool IsRLineGood(in IntSurf_Quadric Quad1,
                                    in IntSurf_Quadric Quad2,
                                    IntPatch_GLine theGLine,
                                    IntPatch_RLine theRLine,
                                    double theTol)
    {
        double aSQTol = theTol * theTol;
        IntPatch_IType aGType = theGLine.ArcType();
        int aNbPntsM1 = 0;

        Func<IntPatch_RLine, int, gp_Pnt> Value;

        if (theRLine.HasPolygon())
        {
            aNbPntsM1 = theRLine.NbPnts() - 1;
            Value = PointValue;
        }
        else
        {
            aNbPntsM1 = theRLine.NbVertex() - 1;
            Value = VertexValue;
        }

        if (aNbPntsM1 < 1)
        {
            return false;
        }

        Extrema_ExtPC anExtr = new();
        GeomAdaptor_Curve anAC = new();
        Geom_Curve? aCurv = null;

        if (aGType == IntPatch_Ellipse)
        {
            aCurv = new Geom_Ellipse(theGLine.Ellipse());
        }
        else if (aGType == IntPatch_Parabola)
        {
            aCurv = new Geom_Parabola(theGLine.Parabola());
        }
        else if (aGType == IntPatch_Hyperbola)
        {
            aCurv = new Geom_Hyperbola(theGLine.Hyperbola());
        }

        if (aCurv is not null)
        {
            double anUinf = aCurv.FirstParameter(), anUsup = aCurv.LastParameter();
            anAC.Load(aCurv, anUinf, anUsup);
            anExtr.Initialize(anAC, anUinf, anUsup);
        }

        if (aNbPntsM1 == 1)
        {
            gp_Pnt aP1 = Value(theRLine, 1), aP2 = Value(theRLine, 2);

            if (aP1.SquareDistance(aP2) < aSQTol)
            {
                // RLine is degenerated
                return false;
            }

            gp_Pnt aPMid;
            if (theRLine.IsArcOnS1())
            {
                Adaptor2d_Curve2d anAC2d = theRLine.ArcOnS1()!;
                double aParF = anAC2d.FirstParameter(), aParL = anAC2d.LastParameter();
                gp_Pnt2d aP2d = anAC2d.Value(0.5 * (aParF + aParL));
                aPMid = Quad1.Value(aP2d.X(), aP2d.Y());
            }
            else
            {
                Adaptor2d_Curve2d anAC2d = theRLine.ArcOnS2()!;
                double aParF = anAC2d.FirstParameter(), aParL = anAC2d.LastParameter();
                gp_Pnt2d aP2d = anAC2d.Value(0.5 * (aParF + aParL));
                aPMid = Quad2.Value(aP2d.X(), aP2d.Y());
            }

            double aSQDist1 = SquareDistance(theGLine, aPMid, anExtr);
            return (aSQDist1 > aSQTol);
        }

        for (int i = 2; i <= aNbPntsM1; i++)
        {
            gp_Pnt aP = Value(theRLine, i);
            double aSQDist = SquareDistance(theGLine, aP, anExtr);

            if (aSQDist > aSQTol)
            {
                return true;
            }
        }

        return false;
    }

    private static void ProcessRLine(NCollection_Sequence<IntPatch_Line> slin,
                                     in IntSurf_Quadric Quad1,
                                     in IntSurf_Quadric Quad2,
                                     double _TolArc,
                                     bool theIsReqToKeepRLine)
    {
        // On cherche a placer sur les restrictions solutions les points "multiples"
        // des autres lignes d intersection
        // Pas forcemment le plus efficace : on rique de projeter plusieurs fois
        // le meme point sur la meme restriction...

        double TolArc = 100.0 * _TolArc;
        if (TolArc > 0.1)
        {
            TolArc = 0.1;
        }

        int i, j, k;
        int Nblin, Nbvtx, Nbpt;

        bool OnFirst = false, project = false, keeppoint = false;

        Adaptor2d_Curve2d? arcref = null;
        double paramproj, paramf, paraml;

        NCollection_Sequence<gp_Pnt> seq_Pnt3d = new();
        NCollection_Sequence<double> seq_Real = new();

        gp_Pnt ptproj, toproj, valpt = new();

        gp_Pnt2d p2d;
        gp_Vec2d d2d;
        gp_Vec d1u = new(), d1v = new(), tgrest = new(), tgarc = new(), norm;
        IntSurf_Transition TRest = new(), TArc = new();
        double U = 0.0, V = 0.0;
        IntPatch_Point Ptvtx, newptvtx = new();

        IntPatch_IType typ1, typ2;

        Nblin = slin.Length();
        for (i = 1; i <= Nblin; i++)
        {
            IntPatch_Line slini = slin[i];
            typ1 = slini.ArcType();

            bool HasToDeleteRLine = false;
            if (typ1 == IntPatch_Restriction)
            {
                IntPatch_RLine rlini = (IntPatch_RLine)slini;
                seq_Pnt3d.Clear();
                seq_Real.Clear();

                for (j = 1; j <= Nblin; j++)
                {
                    IntPatch_Line slinj = slin[j];
                    Nbpt = seq_Pnt3d.Length(); // important que ce soit ici
                    typ2 = slinj.ArcType();
                    if (typ2 != IntPatch_Restriction)
                    {
                        //-- DES CHOSES A FAIRE ICI
                        if (rlini.IsArcOnS1())
                        {
                            OnFirst = true;
                            arcref = rlini.ArcOnS1();
                        }
                        else if (rlini.IsArcOnS2())
                        {
                            arcref = rlini.ArcOnS2();
                            OnFirst = false;
                        }
                        if (rlini.HasFirstPoint())
                        {
                            paramf = rlini.FirstPoint().ParameterOnLine();
                        }
                        else
                        {
                            paramf = RealFirst();
                        }
                        if (rlini.HasLastPoint())
                        {
                            paraml = rlini.LastPoint().ParameterOnLine();
                        }
                        else
                        {
                            paraml = RealLast();
                        }

                        Nbvtx = NbVertexOf(slinj, typ2);

                        bool EdgeDegenere = true;
                        for (int edg = 0; EdgeDegenere && edg <= 10; edg++)
                        {
                            arcref!.D1(paramf + (paraml - paramf) * edg * 0.1, out p2d, out d2d);
                            if (OnFirst)
                            {
                                Quad1.D1(p2d.X(), p2d.Y(), ref valpt, ref d1u, ref d1v);
                            }
                            else
                            {
                                Quad2.D1(p2d.X(), p2d.Y(), ref valpt, ref d1u, ref d1v);
                            }
                            if (d1u.Magnitude() > 1e-7)
                            {
                                EdgeDegenere = false;
                            }
                        }

                        for (k = 1; !EdgeDegenere && k <= Nbvtx; k++)
                        {
                            Ptvtx = VertexOf(slinj, typ2, k);
                            if ((OnFirst && !Ptvtx.IsOnDomS1()) || (!OnFirst && !Ptvtx.IsOnDomS2()))
                            {
                                // Si OnFirst && OnDomS1, c est qu on est a une extremite
                                // ca doit etre traite par Process Segment...
                                project = true;
                                keeppoint = false;
                                toproj = Ptvtx.Value();

                                int jj;
                                for (jj = 1; jj <= Nbpt; jj++)
                                {
                                    if (toproj.Distance(seq_Pnt3d[jj]) < _TolArc)
                                    {
                                        project = false;
                                        break;
                                    }
                                }
                                if (project)
                                { //-- il faut projeter pour trouver le point sur la rline.
                                    if (OnFirst)
                                    {
                                        Ptvtx.ParametersOnS1(out U, out V);
                                    }
                                    else
                                    {
                                        Ptvtx.ParametersOnS2(out U, out V);
                                    }

                                    project = IntPatch_HInterTool.Project(arcref!, new gp_Pnt2d(U, V), out paramproj,
                                                                          out p2d);

                                    if (project)
                                    {
                                        if (OnFirst)
                                        {
                                            ptproj = Quad1.Value(p2d.X(), p2d.Y());
                                        }
                                        else
                                        {
                                            ptproj = Quad2.Value(p2d.X(), p2d.Y());
                                        }
                                        if ((toproj.Distance(ptproj) <= 100 * TolArc) && (paramproj >= paramf)
                                            && (paramproj <= paraml))
                                        {
                                            newptvtx = Ptvtx;
                                            newptvtx.SetParameter(paramproj);
                                            keeppoint = true;
                                            seq_Pnt3d.Append(toproj);
                                            seq_Real.Append(paramproj);

                                            //-- verifier que si la restriction arcref est trouvee, elle porte ce vertex
                                            for (int ri = 1; ri <= Nblin; ri++)
                                            {
                                                IntPatch_Line slinri = slin[ri];
                                                if (slinri.ArcType() == IntPatch_Restriction)
                                                {
                                                    IntPatch_RLine rlinri = (IntPatch_RLine)slinri;
                                                    if (OnFirst && rlinri.IsArcOnS1())
                                                    {
                                                        if (arcref == rlinri.ArcOnS1())
                                                        {
                                                            rlinri.AddVertex(newptvtx);
                                                        }
                                                    }
                                                    else if (!OnFirst && rlinri.IsArcOnS2())
                                                    {
                                                        if (arcref == rlinri.ArcOnS2())
                                                        {
                                                            rlinri.AddVertex(newptvtx);
                                                        }
                                                    }
                                                }
                                            }
                                            // -- --------------------------------------------------
                                        }
                                    }
                                }
                                else
                                {
                                    keeppoint = true;
                                    newptvtx = Ptvtx;
                                    newptvtx.SetParameter(seq_Real[jj]);
                                }
                                if (keeppoint)
                                {
                                    Ptvtx.SetMultiple(true);
                                    Ptvtx.SetTolerance(_TolArc);
                                    newptvtx.SetMultiple(true);

                                    ReplaceVertexOf(slinj, typ2, k, Ptvtx);

                                    if (Ptvtx.IsOnDomS1() || Ptvtx.IsOnDomS2())
                                    {
                                        arcref!.D1(newptvtx.ParameterOnLine(), out p2d, out d2d);

                                        if (OnFirst)
                                        { // donc OnDomS2
                                            Quad1.D1(p2d.X(), p2d.Y(), ref valpt, ref d1u, ref d1v);
                                            tgrest.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);

                                            Adaptor2d_Curve2d thearc = Ptvtx.ArcOnS2();
                                            thearc.D1(Ptvtx.ParameterOnArc2(), out p2d, out d2d);
                                            Quad2.D1(p2d.X(), p2d.Y(), ref valpt, ref d1u, ref d1v);
                                            tgarc.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);
                                            norm = d1u.Crossed(d1v); // Quad2.Normale(valpt);
                                            if (norm.SquareMagnitude() < 1e-16)
                                            {
                                                TRest.SetValue(true, IntSurf_Undecided);
                                                TArc.SetValue(true, IntSurf_Undecided);
                                            }
                                            else
                                            {
                                                IntSurf.MakeTransition(tgrest, tgarc, new gp_Dir(norm), ref TRest,
                                                                       ref TArc);
                                            }
                                            newptvtx.SetArc(false, thearc, Ptvtx.ParameterOnArc2(), TRest, TArc);
                                        }
                                        else
                                        { // donc OnDomS1
                                            Quad2.D1(p2d.X(), p2d.Y(), ref valpt, ref d1u, ref d1v);
                                            tgrest.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);

                                            Adaptor2d_Curve2d thearc = Ptvtx.ArcOnS1();
                                            thearc.D1(Ptvtx.ParameterOnArc1(), out p2d, out d2d);
                                            Quad1.D1(p2d.X(), p2d.Y(), ref valpt, ref d1u, ref d1v);
                                            tgarc.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);
                                            norm = d1u.Crossed(d1v); // Quad1.Normale(valpt);
                                            if (norm.SquareMagnitude() < 1e-16)
                                            {
                                                TRest.SetValue(true, IntSurf_Undecided);
                                                TArc.SetValue(true, IntSurf_Undecided);
                                            }
                                            else
                                            {
                                                IntSurf.MakeTransition(tgrest, tgarc, new gp_Dir(norm), ref TRest,
                                                                       ref TArc);
                                            }
                                            newptvtx.SetArc(true, thearc, Ptvtx.ParameterOnArc1(), TRest, TArc);
                                        }
                                    } //-- if (Ptvtx.IsOnDomS1() || Ptvtx.IsOnDomS2())

                                    rlini.AddVertex(newptvtx);

                                } //-- if (keeppoint)
                            } //-- if ((OnFirst && !Ptvtx.IsOnDomS1())||(!OnFirst && !Ptvtx.IsOnDomS2()))
                        } //-- boucle sur les vertex

                        if (!theIsReqToKeepRLine)
                        {
                            if (slinj is IntPatch_GLine aGL)
                            {
                                HasToDeleteRLine = !IsRLineGood(Quad1, Quad2, aGL, rlini, TolArc);
                            }

                            if (HasToDeleteRLine)
                            {
                                break;
                            }
                        }
                    } //-- if (typ2 != IntPatch_Restriction)
                } //-- for (j=1; j<=Nblin; j++)
            } //-- if (typ1 == IntPatch_Restriction)

            if (HasToDeleteRLine)
            {
                slin.Remove(i);
                i--;
                Nblin = slin.Length();
                continue;
            }
        } //-- for (i=1; i<=Nblin; i++)
    }
}
