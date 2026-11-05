// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.Intersection.Patches;

// The restriction lines (OCCT 7.x's IntPatch_ImpImpIntersection_0.gxx, continued): the segments of the boundaries
// that lie on the other surface become RLines (ProcessSegments), and the RLines take the multiple points of the
// other lines, or are removed where they duplicate a GLine (ProcessRLine).
internal sealed partial class ImpImpIntersection
{
    // The vertex count, a vertex and the replacement of a vertex of an ALine, RLine or GLine.
    private static int NbVertexOfARG(PatchLine line, PatchLineType type) =>
        type == PatchLineType.Analytic ? ((ALine)line).VertexCount
        : type == PatchLineType.Restriction ? ((RLine)line).VertexCount
        : ((GLine)line).VertexCount;

    private static PatchPoint VertexOfARG(PatchLine line, PatchLineType type, int index) =>
        type == PatchLineType.Analytic ? ((ALine)line).Vertex(index)
        : type == PatchLineType.Restriction ? ((RLine)line).Vertex(index)
        : ((GLine)line).Vertex(index);

    private static void ReplaceVertexOfARG(PatchLine line, PatchLineType type, int index,
                                           in PatchPoint pnt)
    {
        if (type == PatchLineType.Analytic)
        {
            ((ALine)line).Replace(index, pnt);
        }
        else if (type == PatchLineType.Restriction)
        {
            ((RLine)line).Replace(index, pnt);
        }
        else
        {
            ((GLine)line).Replace(index, pnt);
        }
    }

    // Recomputes the transition of newptvtx on the arc of ptvtx on one surface (ProcessSegments' repeated block).
    private static void SetArcTransition(ref PatchPoint newptvtx, in PatchPoint ptvtx, bool onS1,
                                         in SurfaceQuadric quad, in Vector3D tgline, ref Point3D valpt,
                                         ref Vector3D d1u, ref Vector3D d1v, ref Vector3D tgarc,
                                         ref WalkingTransition tRest, ref WalkingTransition tArc)
    {
        Curve2DAdaptorBase thearc = onS1 ? ptvtx.ArcOnS1() : ptvtx.ArcOnS2();
        double parOnArc = onS1 ? ptvtx.ParameterOnArc1() : ptvtx.ParameterOnArc2();
        thearc.D1(parOnArc, out Point2D p2d, out Vector2D d2d);
        quad.D1(p2d.X, p2d.Y, ref valpt, ref d1u, ref d1v);
        tgarc = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
        Vector3D norm = d1u.Crossed(d1v);
        if (norm.SquareMagnitude < 1e-16)
        {
            tRest.SetValue(true, WalkingTypeTrans.Undecided);
            tArc.SetValue(true, WalkingTypeTrans.Undecided);
        }
        else
        {
            global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(tgline, tgarc, new Direction3D(norm), ref tRest, ref tArc);
        }
        newptvtx.SetArc(onS1, thearc, parOnArc, tRest, tArc);
    }

    private static void ProcessSegments(NCollection_Sequence<TheSegmentOfTheSOnBounds> listedg,
                                        NCollection_Sequence<PatchLine> slin,
                                        in SurfaceQuadric quad1,
                                        in SurfaceQuadric quad2,
                                        bool onFirst,
                                        double tolArc)
    {
        int i, j, k;
        int nbedg = listedg.Length();
        int nblines, nbpts;

        Curve2DAdaptorBase arcRef;
        // OCCT: ptvtx lives across the segments and the vertex loop: an end point that no line carries is built
        // from the last vertex read, keeping that vertex's arcs, vertices and flags where it sets none.
        PatchPoint ptvtx = new(), newptvtx;

        RLine rline; //-- On fait rline = new ... par la suite

        TheSegmentOfTheSOnBounds thesegsol;
        ThePathPointOfTheSOnBounds pStartf = new(), pStartl = new();
        bool dofirst, dolast, procf, procl;

        double paramf = 0.0, paraml = 0.0, u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;

        PatchLineType typ;
        WalkingTypeTrans trans1, trans2;
        WalkingTransition tRest = new(), tArc = new();
        Vector3D tgline = new(), norm1, norm2, tgarc = new();
        Point3D valpt = new();

        Vector3D d1u = new(), d1v = new();
        Point2D p2d;
        Vector2D d2d;

        for (i = 1; i <= nbedg; i++)
        {
            bool edgeDegenere = false;
            thesegsol = listedg.Value(i);
            arcRef = thesegsol.Curve();

            rline = new RLine(false);
            if (onFirst)
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

            if (thesegsol.HasFirstPoint)
            {
                dofirst = true;
                pStartf = thesegsol.FirstPoint();
                paramf = pStartf.Parameter();
            }
            if (thesegsol.HasLastPoint)
            {
                dolast = true;
                pStartl = thesegsol.LastPoint();
                paraml = pStartl.Parameter();
            }

            if (dofirst && dolast)
            { // determination de la transition de la ligne
                arcRef.D1(0.5 * (paramf + paraml), out p2d, out d2d);
                if (onFirst)
                {
                    quad1.D1(p2d.X, p2d.Y, ref valpt, ref d1u, ref d1v);
                }
                else
                {
                    quad2.D1(p2d.X, p2d.Y, ref valpt, ref d1u, ref d1v);
                }
                tgline = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);

                if (d1u.Magnitude < 1e-7)
                { //-- edge degenere ?
                    edgeDegenere = true;
                    for (int edg = 0; edg <= 10; edg++)
                    {
                        arcRef.D1(paramf + (paraml - paramf) * edg * 0.1, out p2d, out d2d);
                        if (onFirst)
                        {
                            quad1.D1(p2d.X, p2d.Y, ref valpt, ref d1u, ref d1v);
                        }
                        else
                        {
                            quad2.D1(p2d.X, p2d.Y, ref valpt, ref d1u, ref d1v);
                        }

                        if (d1u.Magnitude > 1e-7)
                        {
                            edgeDegenere = false;
                        }
                    }
                    rline = new RLine(false);
                    if (onFirst)
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
                    norm2 = quad2.Normale(valpt);
                    norm1 = quad1.Normale(valpt);

                    if (tgline.DotCross(norm2, norm1) > 0.000000001)
                    {
                        trans1 = WalkingTypeTrans.Out;
                        trans2 = WalkingTypeTrans.In;
                    }
                    else if (tgline.DotCross(norm2, norm1) < -0.000000001)
                    {
                        trans1 = WalkingTypeTrans.In;
                        trans2 = WalkingTypeTrans.Out;
                    }
                    else
                    {
                        trans1 = trans2 = WalkingTypeTrans.Undecided;
                    }
                    rline = new RLine(false, trans1, trans2);
                    if (onFirst)
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
                rline = new RLine(false);
                if (onFirst)
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
                nblines = slin.Length();
                for (j = 1; j <= nblines; j++)
                {
                    PatchLine slinj = slin[j];
                    typ = slinj.ArcType();
                    nbpts = NbVertexOfARG(slinj, typ);
                    for (k = 1; k <= nbpts; k++)
                    {
                        ptvtx = VertexOfARG(slinj, typ, k);

                        if (!edgeDegenere && dofirst)
                        {
                            if (ptvtx.Value().DistanceTo(pStartf.Value()) <= tolArc)
                            {
                                ptvtx.SetMultiple(true);
                                ptvtx.SetTolerance(tolArc);
                                ReplaceVertexOfARG(slinj, typ, k, ptvtx);
                                newptvtx = ptvtx;
                                newptvtx.SetParameter(paramf);
                                // Recalcul des  transitions si point sur restriction

                                arcRef.D1(paramf, out p2d, out d2d);
                                if (onFirst)
                                {
                                    quad1.D1(p2d.X, p2d.Y, ref valpt, ref d1u, ref d1v);
                                }
                                else
                                {
                                    quad2.D1(p2d.X, p2d.Y, ref valpt, ref d1u, ref d1v);
                                }
                                tgline = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
                                if (ptvtx.IsOnDomS1)
                                {
                                    SetArcTransition(ref newptvtx, ptvtx, true, quad1, tgline, ref valpt, ref d1u,
                                                     ref d1v, ref tgarc, ref tRest, ref tArc);
                                }
                                if (ptvtx.IsOnDomS2)
                                {
                                    SetArcTransition(ref newptvtx, ptvtx, false, quad2, tgline, ref valpt, ref d1u,
                                                     ref d1v, ref tgarc, ref tRest, ref tArc);
                                }

                                rline.AddVertex(newptvtx);
                                if (!procf)
                                {
                                    procf = true;
                                    rline.SetFirstPoint(rline.VertexCount);
                                }
                            }
                        }
                        if (!edgeDegenere && dolast)
                        {
                            if (ptvtx.Value().DistanceTo(pStartl.Value()) <= tolArc)
                            {
                                ptvtx.SetMultiple(true);
                                ptvtx.SetTolerance(tolArc);
                                ReplaceVertexOfARG(slinj, typ, k, ptvtx);

                                newptvtx = ptvtx;
                                newptvtx.SetParameter(paraml);
                                // Recalcul des  transitions si point sur restriction

                                arcRef.D1(paraml, out p2d, out d2d);
                                if (onFirst)
                                {
                                    quad1.D1(p2d.X, p2d.Y, ref valpt, ref d1u, ref d1v);
                                }
                                else
                                {
                                    quad2.D1(p2d.X, p2d.Y, ref valpt, ref d1u, ref d1v);
                                }
                                tgline = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
                                if (ptvtx.IsOnDomS1)
                                {
                                    SetArcTransition(ref newptvtx, ptvtx, true, quad1, tgline, ref valpt, ref d1u,
                                                     ref d1v, ref tgarc, ref tRest, ref tArc);
                                }
                                if (ptvtx.IsOnDomS2)
                                {
                                    SetArcTransition(ref newptvtx, ptvtx, false, quad2, tgline, ref valpt, ref d1u,
                                                     ref d1v, ref tgarc, ref tRest, ref tArc);
                                }

                                rline.AddVertex(newptvtx);
                                if (!procl)
                                {
                                    procl = true;
                                    rline.SetLastPoint(rline.VertexCount);
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
                ptvtx.SetValue(pStartf.Value(), pStartf.Tolerance(), false);
                quad1.Parameters(pStartf.Value(), ref u1, ref v1);
                quad2.Parameters(pStartf.Value(), ref u2, ref v2);
                ptvtx.SetParameters(u1, v1, u2, v2);
                ptvtx.SetParameter(paramf);
                if (!pStartf.IsNew())
                {
                    WalkingTransition transline = new();
                    WalkingTransition transarc = new();
                    ptvtx.SetVertex(onFirst, pStartf.Vertex());
                    ptvtx.SetArc(onFirst, pStartf.Arc(), pStartf.Parameter(), transline, transarc);
                }

                rline.AddVertex(ptvtx);
                rline.SetFirstPoint(rline.VertexCount);
            }
            if (dolast)
            {
                ptvtx.SetValue(pStartl.Value(), pStartl.Tolerance(), false);
                quad1.Parameters(pStartl.Value(), ref u1, ref v1);
                quad2.Parameters(pStartl.Value(), ref u2, ref v2);
                ptvtx.SetParameters(u1, v1, u2, v2);
                ptvtx.SetParameter(paraml);
                if (!pStartl.IsNew())
                {
                    WalkingTransition transline = new();
                    WalkingTransition transarc = new();

                    ptvtx.SetVertex(onFirst, pStartl.Vertex());
                    ptvtx.SetArc(onFirst, pStartl.Arc(), pStartl.Parameter(), transline, transarc);
                }

                rline.AddVertex(ptvtx);
                rline.SetLastPoint(rline.VertexCount);
            }
            slin.Append(rline);
        }
    }

    // OCCT's PointValue and VertexValue, chosen through a function pointer in IsRLineGood.
    private static Point3D PointValue(RLine rLine, int index) => rLine.Point(index).Value();

    private static Point3D VertexValue(RLine rLine, int index) => rLine.Vertex(index).Value();

    private static double SquareDistance(GLine gLine, in Point3D p, CurvePointExtrema extr)
    {
        double sqDist = RealLast();
        switch (gLine.ArcType())
        {
            case PatchLineType.Lin:
                sqDist = gLine.Line().SquareDistanceTo(p);
                break;
            case PatchLineType.Circle:
                sqDist = gLine.Circle().SquareDistanceTo(p);
                break;
            default:
                extr.Perform(p);
                if (!extr.IsDone || extr.ExtCount == 0)
                {
                    // Lines are not overlapped
                    return sqDist;
                }

                sqDist = extr.SquareDistance(1);
                int nbExtr = extr.ExtCount;
                for (int i = 2; i <= nbExtr; i++)
                {
                    double sqd = extr.SquareDistance(i);
                    if (sqd < sqDist)
                    {
                        sqDist = sqd;
                    }
                }
                break;
        }

        return sqDist;
    }

    private static bool IsRLineGood(in SurfaceQuadric quad1,
                                    in SurfaceQuadric quad2,
                                    GLine gLine,
                                    RLine rLine,
                                    double tol)
    {
        double sqTol = tol * tol;
        PatchLineType gType = gLine.ArcType();
        int nbPntsM1 = 0;

        Func<RLine, int, Point3D> value;

        if (rLine.HasPolygon)
        {
            nbPntsM1 = rLine.NbPnts() - 1;
            value = PointValue;
        }
        else
        {
            nbPntsM1 = rLine.VertexCount - 1;
            value = VertexValue;
        }

        if (nbPntsM1 < 1)
        {
            return false;
        }

        CurvePointExtrema extr = new();
        CurveAdaptor ac = new();
        Curve3D? curv = null;

        if (gType == PatchLineType.Ellipse)
        {
            curv = new EllipseCurve3D(gLine.Ellipse());
        }
        else if (gType == PatchLineType.Parabola)
        {
            curv = new ParabolaCurve3D(gLine.Parabola());
        }
        else if (gType == PatchLineType.Hyperbola)
        {
            curv = new HyperbolaCurve3D(gLine.Hyperbola());
        }

        if (curv is not null)
        {
            double uinf = curv.FirstParameter, usup = curv.LastParameter;
            ac.Load(curv, uinf, usup);
            extr.Initialize(ac, uinf, usup);
        }

        if (nbPntsM1 == 1)
        {
            Point3D p1 = value(rLine, 1), p2 = value(rLine, 2);

            if (p1.SquareDistanceTo(p2) < sqTol)
            {
                // RLine is degenerated
                return false;
            }

            Point3D pMid;
            if (rLine.IsArcOnS1)
            {
                Curve2DAdaptorBase ac2d = rLine.ArcOnS1()!;
                double parF = ac2d.FirstParameter(), parL = ac2d.LastParameter();
                Point2D p2d = ac2d.Value(0.5 * (parF + parL));
                pMid = quad1.Value(p2d.X, p2d.Y);
            }
            else
            {
                Curve2DAdaptorBase ac2d = rLine.ArcOnS2()!;
                double parF = ac2d.FirstParameter(), parL = ac2d.LastParameter();
                Point2D p2d = ac2d.Value(0.5 * (parF + parL));
                pMid = quad2.Value(p2d.X, p2d.Y);
            }

            double sqDist1 = SquareDistance(gLine, pMid, extr);
            return (sqDist1 > sqTol);
        }

        for (int i = 2; i <= nbPntsM1; i++)
        {
            Point3D p = value(rLine, i);
            double sqDist = SquareDistance(gLine, p, extr);

            if (sqDist > sqTol)
            {
                return true;
            }
        }

        return false;
    }

    private static void ProcessRLine(NCollection_Sequence<PatchLine> slin,
                                     in SurfaceQuadric quad1,
                                     in SurfaceQuadric quad2,
                                     double _TolArc,
                                     bool isReqToKeepRLine)
    {
        // On cherche a placer sur les restrictions solutions les points "multiples"
        // des autres lignes d intersection
        // Pas forcemment le plus efficace : on rique de projeter plusieurs fois
        // le meme point sur la meme restriction...

        double tolArc = 100.0 * _TolArc;
        if (tolArc > 0.1)
        {
            tolArc = 0.1;
        }

        int i, j, k;
        int nblin, nbvtx, nbpt;

        bool onFirst = false, project = false, keeppoint = false;

        Curve2DAdaptorBase? arcref = null;
        double paramproj, paramf, paraml;

        NCollection_Sequence<Point3D> seq_Pnt3d = new();
        NCollection_Sequence<double> seq_Real = new();

        Point3D ptproj, toproj, valpt = new();

        Point2D p2d;
        Vector2D d2d;
        Vector3D d1u = new(), d1v = new(), tgrest = new(), tgarc = new(), norm;
        WalkingTransition tRest = new(), tArc = new();
        double u = 0.0, v = 0.0;
        PatchPoint ptvtx, newptvtx = new();

        PatchLineType typ1, typ2;

        nblin = slin.Length();
        for (i = 1; i <= nblin; i++)
        {
            PatchLine slini = slin[i];
            typ1 = slini.ArcType();

            bool hasToDeleteRLine = false;
            if (typ1 == PatchLineType.Restriction)
            {
                RLine rlini = (RLine)slini;
                seq_Pnt3d.Clear();
                seq_Real.Clear();

                for (j = 1; j <= nblin; j++)
                {
                    PatchLine slinj = slin[j];
                    nbpt = seq_Pnt3d.Length(); // important que ce soit ici
                    typ2 = slinj.ArcType();
                    if (typ2 != PatchLineType.Restriction)
                    {
                        //-- DES CHOSES A FAIRE ICI
                        if (rlini.IsArcOnS1)
                        {
                            onFirst = true;
                            arcref = rlini.ArcOnS1();
                        }
                        else if (rlini.IsArcOnS2)
                        {
                            arcref = rlini.ArcOnS2();
                            onFirst = false;
                        }
                        if (rlini.HasFirstPoint)
                        {
                            paramf = rlini.FirstPoint().ParameterOnLine();
                        }
                        else
                        {
                            paramf = RealFirst();
                        }
                        if (rlini.HasLastPoint)
                        {
                            paraml = rlini.LastPoint().ParameterOnLine();
                        }
                        else
                        {
                            paraml = RealLast();
                        }

                        nbvtx = NbVertexOf(slinj, typ2);

                        bool edgeDegenere = true;
                        for (int edg = 0; edgeDegenere && edg <= 10; edg++)
                        {
                            arcref!.D1(paramf + (paraml - paramf) * edg * 0.1, out p2d, out d2d);
                            if (onFirst)
                            {
                                quad1.D1(p2d.X, p2d.Y, ref valpt, ref d1u, ref d1v);
                            }
                            else
                            {
                                quad2.D1(p2d.X, p2d.Y, ref valpt, ref d1u, ref d1v);
                            }
                            if (d1u.Magnitude > 1e-7)
                            {
                                edgeDegenere = false;
                            }
                        }

                        for (k = 1; !edgeDegenere && k <= nbvtx; k++)
                        {
                            ptvtx = VertexOf(slinj, typ2, k);
                            if ((onFirst && !ptvtx.IsOnDomS1) || (!onFirst && !ptvtx.IsOnDomS2))
                            {
                                // Si OnFirst && OnDomS1, c est qu on est a une extremite
                                // ca doit etre traite par Process Segment...
                                project = true;
                                keeppoint = false;
                                toproj = ptvtx.Value();

                                int jj;
                                for (jj = 1; jj <= nbpt; jj++)
                                {
                                    if (toproj.DistanceTo(seq_Pnt3d[jj]) < _TolArc)
                                    {
                                        project = false;
                                        break;
                                    }
                                }
                                if (project)
                                { //-- il faut projeter pour trouver le point sur la rline.
                                    if (onFirst)
                                    {
                                        ptvtx.ParametersOnS1(out u, out v);
                                    }
                                    else
                                    {
                                        ptvtx.ParametersOnS2(out u, out v);
                                    }

                                    project = HInterTool.Project(arcref!, new Point2D(u, v), out paramproj,
                                                                          out p2d);

                                    if (project)
                                    {
                                        if (onFirst)
                                        {
                                            ptproj = quad1.Value(p2d.X, p2d.Y);
                                        }
                                        else
                                        {
                                            ptproj = quad2.Value(p2d.X, p2d.Y);
                                        }
                                        if ((toproj.DistanceTo(ptproj) <= 100 * tolArc) && (paramproj >= paramf)
                                            && (paramproj <= paraml))
                                        {
                                            newptvtx = ptvtx;
                                            newptvtx.SetParameter(paramproj);
                                            keeppoint = true;
                                            seq_Pnt3d.Append(toproj);
                                            seq_Real.Append(paramproj);

                                            //-- verifier que si la restriction arcref est trouvee, elle porte ce vertex
                                            for (int ri = 1; ri <= nblin; ri++)
                                            {
                                                PatchLine slinri = slin[ri];
                                                if (slinri.ArcType() == PatchLineType.Restriction)
                                                {
                                                    RLine rlinri = (RLine)slinri;
                                                    if (onFirst && rlinri.IsArcOnS1)
                                                    {
                                                        if (arcref == rlinri.ArcOnS1())
                                                        {
                                                            rlinri.AddVertex(newptvtx);
                                                        }
                                                    }
                                                    else if (!onFirst && rlinri.IsArcOnS2)
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
                                    newptvtx = ptvtx;
                                    newptvtx.SetParameter(seq_Real[jj]);
                                }
                                if (keeppoint)
                                {
                                    ptvtx.SetMultiple(true);
                                    ptvtx.SetTolerance(_TolArc);
                                    newptvtx.SetMultiple(true);

                                    ReplaceVertexOf(slinj, typ2, k, ptvtx);

                                    if (ptvtx.IsOnDomS1 || ptvtx.IsOnDomS2)
                                    {
                                        arcref!.D1(newptvtx.ParameterOnLine(), out p2d, out d2d);

                                        if (onFirst)
                                        { // donc OnDomS2
                                            quad1.D1(p2d.X, p2d.Y, ref valpt, ref d1u, ref d1v);
                                            tgrest = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);

                                            Curve2DAdaptorBase thearc = ptvtx.ArcOnS2();
                                            thearc.D1(ptvtx.ParameterOnArc2(), out p2d, out d2d);
                                            quad2.D1(p2d.X, p2d.Y, ref valpt, ref d1u, ref d1v);
                                            tgarc = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
                                            norm = d1u.Crossed(d1v); // Quad2.Normale(valpt);
                                            if (norm.SquareMagnitude < 1e-16)
                                            {
                                                tRest.SetValue(true, WalkingTypeTrans.Undecided);
                                                tArc.SetValue(true, WalkingTypeTrans.Undecided);
                                            }
                                            else
                                            {
                                                global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(tgrest, tgarc, new Direction3D(norm), ref tRest,
                                                                       ref tArc);
                                            }
                                            newptvtx.SetArc(false, thearc, ptvtx.ParameterOnArc2(), tRest, tArc);
                                        }
                                        else
                                        { // donc OnDomS1
                                            quad2.D1(p2d.X, p2d.Y, ref valpt, ref d1u, ref d1v);
                                            tgrest = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);

                                            Curve2DAdaptorBase thearc = ptvtx.ArcOnS1();
                                            thearc.D1(ptvtx.ParameterOnArc1(), out p2d, out d2d);
                                            quad1.D1(p2d.X, p2d.Y, ref valpt, ref d1u, ref d1v);
                                            tgarc = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
                                            norm = d1u.Crossed(d1v); // Quad1.Normale(valpt);
                                            if (norm.SquareMagnitude < 1e-16)
                                            {
                                                tRest.SetValue(true, WalkingTypeTrans.Undecided);
                                                tArc.SetValue(true, WalkingTypeTrans.Undecided);
                                            }
                                            else
                                            {
                                                global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(tgrest, tgarc, new Direction3D(norm), ref tRest,
                                                                       ref tArc);
                                            }
                                            newptvtx.SetArc(true, thearc, ptvtx.ParameterOnArc1(), tRest, tArc);
                                        }
                                    } //-- if (Ptvtx.IsOnDomS1() || Ptvtx.IsOnDomS2())

                                    rlini.AddVertex(newptvtx);

                                } //-- if (keeppoint)
                            } //-- if ((OnFirst && !Ptvtx.IsOnDomS1())||(!OnFirst && !Ptvtx.IsOnDomS2()))
                        } //-- boucle sur les vertex

                        if (!isReqToKeepRLine)
                        {
                            if (slinj is GLine gl)
                            {
                                hasToDeleteRLine = !IsRLineGood(quad1, quad2, gl, rlini, tolArc);
                            }

                            if (hasToDeleteRLine)
                            {
                                break;
                            }
                        }
                    } //-- if (typ2 != IntPatch_Restriction)
                } //-- for (j=1; j<=Nblin; j++)
            } //-- if (typ1 == IntPatch_Restriction)

            if (hasToDeleteRLine)
            {
                slin.Remove(i);
                i--;
                nblin = slin.Length();
                continue;
            }
        } //-- for (i=1; i<=Nblin; i++)
    }
}
