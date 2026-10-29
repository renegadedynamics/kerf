// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_TransferParametersProj.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_TransferParametersProj.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// Transfers parameters between an edge's 3d curve and its pcurve by projection, where ShapeAnalysis_TransferParameters
/// does it linearly. Used when an edge isn't same-parameter (or its tolerance is at least the maximal tolerance, or
/// projection is forced).
/// </summary>
/// <remarks>
/// A working object. <see cref="TransferRange"/> writes into the edge it is given and is internal (ported code calls
/// it on edges it builds; it throws on a frozen edge). The public <see cref="CopyNMVertex(in TopoDS_Vertex, in
/// TopoDS_Edge, in TopoDS_Edge)"/> and its face form return new frozen vertices; ported code that goes on building
/// calls the internal non-freezing forms.
/// </remarks>
public class ShapeAnalysis_TransferParametersProj : ShapeAnalysis_TransferParameters
{
    private Geom_Curve?              myCurve;
    private Geom2d_Curve?            myCurve2d;
    private Adaptor3d_CurveOnSurface myAC3d = new Adaptor3d_CurveOnSurface();
    private double                   myPrecision;
    private TopLoc_Location          myLocation = new TopLoc_Location();
    private bool                     myForceProj;
    private bool                     myInitOK;

    /// <summary>Empty constructor.</summary>
    public ShapeAnalysis_TransferParametersProj()
    {
        myPrecision    = 0.0;
        myMaxTolerance = 1; // Precision::Infinite(); ?? pdn
        myForceProj    = false;
        myInitOK       = false;
    }

    /// <summary>Creates the tool for an edge and a face (the face may be null: 3d curve only).</summary>
    public ShapeAnalysis_TransferParametersProj(in TopoDS_Edge E, in TopoDS_Face F)
    {
        myMaxTolerance = 1; // Precision::Infinite(); ?? pdn
        myForceProj    = false;
        Init(E, F);
    }

    /// <summary>Initializes the tool with an edge and a face.</summary>
    public override void Init(in TopoDS_Edge E, in TopoDS_Face F)
    {
        myInitOK = false;
        base.Init(E, F);
        myEdge      = E;
        myPrecision = BRep_Tool.Tolerance(E); // it is better - skl OCC2851
        // myPrecision = Precision::Confusion();

        myCurve = BRep_Tool.Curve(E, out myFirst, out myLast);
        if (myCurve is null)
        {
            myFirst = 0.0;
            myLast  = 1.0;
            return;
        }

        if (F.IsNull())
        {
            return;
        }

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        if (sae.PCurve(E, F, out myCurve2d, out double f2d, out double l2d, false))
        {
            Geom2dAdaptor_Curve AC2d     = new Geom2dAdaptor_Curve(myCurve2d!, f2d, l2d);
            Geom_Surface        aSurface = BRep_Tool.Surface(F, out myLocation)!;
            GeomAdaptor_Surface AdS      = new GeomAdaptor_Surface(aSurface);

            Adaptor3d_CurveOnSurface Ad1 = new Adaptor3d_CurveOnSurface(AC2d, AdS);
            myAC3d   = Ad1; // new Adaptor3d_CurveOnSurface(AC2d,AdS);
            myInitOK = true;
        }
    }

    /// <summary>Transfers the parameters (in increasing order) from 3d to 2d or back, by projection.</summary>
    public override NCollection_HSequence<double> Perform(NCollection_HSequence<double> Knots, bool To2d)
    {
        // pdn
        if (!myInitOK || (!myForceProj && myPrecision < myMaxTolerance && BRep_Tool.SameParameter(myEdge)))
        {
            return base.Perform(Knots, To2d);
        }

        NCollection_HSequence<double> resKnots = new NCollection_HSequence<double>();

        int    len   = Knots.Length();
        double preci = 2 * Occt.Precision.PConfusion();

        double first   = (To2d ? myAC3d.FirstParameter() : myFirst);
        double last    = (To2d ? myAC3d.LastParameter() : myLast);
        double maxPar  = first;
        double lastPar = last;
        double prevPar = maxPar;

        int j; // svv Jan 10 2000 : porting on DEC
        for (j = 1; j <= len; j++)
        {
            double par = PreformSegment(Knots.Value(j), To2d, prevPar, lastPar);
            prevPar    = par;
            if (prevPar > lastPar)
            {
                prevPar -= preci;
            }

            resKnots.Append(par);
            if (par > maxPar)
            {
                maxPar = par;
            }
        }

        // pdn correcting on periodic
        if (myCurve!.IsClosed())
        {
            for (j = len; j >= 1; j--)
            {
                if (resKnots.Value(j) < maxPar)
                {
                    resKnots.SetValue(j, (To2d ? myAC3d.LastParameter() : myCurve.LastParameter()) - (len - j) * preci);
                }
                else
                {
                    break;
                }
            }
        }

        // pdn correction on range
        for (j = 1; j <= len; j++)
        {
            if (resKnots.Value(j) < first)
            {
                resKnots.SetValue(j, first);
            }

            if (resKnots.Value(j) > last)
            {
                resKnots.SetValue(j, last);
            }
        }

        return resKnots;
    }

    private double PreformSegment(double Param, bool To2d, double First, double Last)
    {
        double linPar = base.Perform(Param, To2d);
        if (!myInitOK || (!myForceProj && myPrecision < myMaxTolerance && BRep_Tool.SameParameter(myEdge)))
        {
            return linPar;
        }

        double              linDev, projDev;
        ShapeAnalysis_Curve sac = new ShapeAnalysis_Curve();
        double              ppar;
        if (To2d)
        {
            gp_Pnt                   p1   = myCurve!.Value(Param).Transformed(myLocation.Inverted().Transformation());
            Adaptor3d_Surface        AdS  = myAC3d.GetSurface()!;
            Geom2dAdaptor_Curve      AC2d = new Geom2dAdaptor_Curve(myCurve2d!, First, Last);
            Adaptor3d_CurveOnSurface Ad1  = new Adaptor3d_CurveOnSurface(AC2d, AdS);
            projDev = sac.Project(Ad1, p1, myPrecision, out _, out ppar); // pdn
            linDev  = p1.Distance(Ad1.Value(linPar));
        }
        else
        {
            gp_Pnt p1 = myAC3d.Value(Param).Transformed(myLocation.Transformation());
            projDev   = sac.Project(myCurve!, p1, myPrecision, out _, out ppar, First, Last, false);
            linDev    = p1.Distance(myCurve!.Value(linPar));
        }

        if (linDev <= projDev || (linDev < myPrecision && linDev <= 2 * projDev))
        {
            ppar = linPar;
        }

        return ppar;
    }

    /// <summary>Transfers one parameter from 3d to 2d or back, by projection.</summary>
    public override double Perform(double Knot, bool To2d)
    {
        if (!myInitOK || (!myForceProj && myPrecision < myMaxTolerance && BRep_Tool.SameParameter(myEdge)))
        {
            return base.Perform(Knot, To2d);
        }

        double res;
        if (To2d)
        {
            res = PreformSegment(Knot, To2d, myAC3d.FirstParameter(), myAC3d.LastParameter());
        }
        else
        {
            res = PreformSegment(Knot, To2d, myFirst, myLast);
        }

        // pdn correction on range
        double first = (To2d ? myAC3d.FirstParameter() : myFirst);
        double last  = (To2d ? myAC3d.LastParameter() : myLast);
        if (res < first)
        {
            res = first;
        }

        if (res > last)
        {
            res = last;
        }

        return res;
    }

    private static double CorrectParameter(Geom2d_Curve crv, double param)
    {
        if (crv is Geom2d_TrimmedCurve tmpT)
        {
            return CorrectParameter(tmpT.BasisCurve(), param);
        }
        else if (crv is Geom2d_OffsetCurve tmpO)
        {
            return CorrectParameter(tmpO.BasisCurve(), param);
        }
        else if (crv is Geom2d_BSplineCurve bspline)
        {
            for (int j = bspline.FirstUKnotIndex(); j <= bspline.LastUKnotIndex(); j++)
            {
                double valknot = bspline.Knot(j);
                if (Math.Abs(valknot - param) < Occt.Precision.PConfusion())
                {
                    return valknot;
                }
            }
        }

        return param;
    }

    /// <summary>
    /// Sets the ranges of the curves of <paramref name="newEdge"/> (a part of the tool's edge between
    /// <paramref name="prevPar"/> and <paramref name="currPar"/>, on the 3d curve or the pcurve) by projection, and its
    /// SameRange flag. Writes into <paramref name="newEdge"/>: it throws on a frozen edge.
    /// </summary>
    internal override void TransferRange(in TopoDS_Edge newEdge, double prevPar, double currPar, bool Is2d)
    {
        if (!myInitOK || (!myForceProj && myPrecision < myMaxTolerance && BRep_Tool.SameParameter(myEdge)))
        {
            base.TransferRange(newEdge, prevPar, currPar, Is2d);
            return;
        }

        BRep_Builder    B         = new BRep_Builder();
        bool            samerange = true;
        ShapeBuild_Edge sbe       = new ShapeBuild_Edge();
        sbe.CopyRanges(newEdge, myEdge);
        gp_Pnt p1;
        gp_Pnt p2;
        double alpha = 0, beta = 1;
        double preci = Occt.Precision.PConfusion();
        double firstPar, lastPar;
        if (prevPar < currPar)
        {
            firstPar = prevPar;
            lastPar  = currPar;
        }
        else
        {
            firstPar = currPar;
            lastPar  = prevPar;
        }

        if (Is2d)
        {
            p1 = myAC3d.Value(firstPar).Transformed(myLocation.Transformation());
            if (Occt.Precision.IsInfinite(p1.X()) || Occt.Precision.IsInfinite(p1.Y()) || Occt.Precision.IsInfinite(p1.Z()))
            {
                B.SameRange(newEdge, false);
                return;
            }

            p2 = myAC3d.Value(lastPar).Transformed(myLocation.Transformation());
            if (Occt.Precision.IsInfinite(p2.X()) || Occt.Precision.IsInfinite(p2.Y()) || Occt.Precision.IsInfinite(p2.Z()))
            {
                B.SameRange(newEdge, false);
                return;
            }

            double fact = myAC3d.LastParameter() - myAC3d.FirstParameter();
            if (fact > Epsilon(myAC3d.LastParameter()))
            {
                alpha = (firstPar - myAC3d.FirstParameter()) / fact;
                beta  = (lastPar - myAC3d.FirstParameter()) / fact;
            }
        }
        else
        {
            p1 = myCurve!.Value(firstPar);
            if (Occt.Precision.IsInfinite(p1.X()) || Occt.Precision.IsInfinite(p1.Y()) || Occt.Precision.IsInfinite(p1.Z()))
            {
                B.SameRange(newEdge, false);
                return;
            }

            p2 = myCurve.Value(lastPar);
            if (Occt.Precision.IsInfinite(p2.X()) || Occt.Precision.IsInfinite(p2.Y()) || Occt.Precision.IsInfinite(p2.Z()))
            {
                B.SameRange(newEdge, false);
                return;
            }

            double fact = myLast - myFirst;
            if (fact > Epsilon(myLast))
            {
                alpha = (firstPar - myFirst) / fact;
                beta  = (lastPar - myFirst) / fact;
            }
        }

        bool                useLinearFirst = (alpha < preci);
        bool                useLinearLast  = (1 - beta < preci);
        TopLoc_Location     EdgeLoc        = myEdge.Location();
        ShapeAnalysis_Curve sac            = new ShapeAnalysis_Curve();
        double              ppar1, ppar2;
        // OCCT.NET: frozen: ChangeCurves() throws on a frozen edge, as BRep_Builder does
        NCollection_List<BRep_CurveRepresentation> tolist = ((BRep_TEdge)newEdge.TShape()!).ChangeCurves();
        BRep_GCurve? toGC;
        for (NCollection_List<BRep_CurveRepresentation>.Iterator toitcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(tolist);
             toitcr.More();
             toitcr.Next())
        {
            toGC = toitcr.Value() as BRep_GCurve;
            if (toGC is null)
            {
                continue;
            }

            TopLoc_Location loc = (EdgeLoc * toGC.Location()).Inverted();
            if (toGC.IsCurve3D())
            {
                if (!Is2d)
                {
                    ppar1 = firstPar;
                    ppar2 = lastPar;
                }
                else
                {
                    Geom_Curve? C3d = toGC.Curve3D();
                    if (C3d is null)
                    {
                        continue;
                    }

                    double            first = toGC.First();
                    double            last  = toGC.Last();
                    double            len   = last - first;
                    gp_Pnt            ploc1 = p1.Transformed(loc.Transformation());
                    gp_Pnt            ploc2 = p2.Transformed(loc.Transformation());
                    GeomAdaptor_Curve GAC   = new GeomAdaptor_Curve(C3d, first, last);
                    // CATIA bplseitli.model FAC1155 - Copy: protection for degenerated edges(3d case for
                    // symmetry)
                    double linFirst  = first + alpha * len;
                    double linLast   = first + beta * len;
                    double dist1     = sac.NextProject(linFirst, GAC, ploc1, myPrecision, out _, out ppar1);
                    double dist2     = sac.NextProject(linLast, GAC, ploc2, myPrecision, out _, out ppar2);
                    bool   useLinear = Math.Abs(ppar1 - ppar2) < preci;

                    gp_Pnt pos1 = C3d.Value(linFirst);
                    gp_Pnt pos2 = C3d.Value(linLast);
                    double d01  = pos1.Distance(ploc1);
                    double d02  = pos2.Distance(ploc2);
                    if (useLinearFirst || useLinear || d01 <= dist1 || (d01 < myPrecision && d01 <= 2 * dist1))
                    {
                        ppar1 = linFirst;
                    }

                    if (useLinearLast || useLinear || d02 <= dist2 || (d02 < myPrecision && d02 <= 2 * dist2))
                    {
                        ppar2 = linLast;
                    }
                }

                if (ppar1 > ppar2)
                {
                    double tmpP = ppar2;
                    ppar2       = ppar1;
                    ppar1       = tmpP;
                }

                if (ppar2 - ppar1 < preci)
                {
                    if (ppar1 - toGC.First() < preci)
                    {
                        ppar2 += 2 * preci;
                    }
                    else if (toGC.Last() - ppar2 < preci)
                    {
                        ppar1 -= 2 * preci;
                    }
                    else
                    {
                        ppar1 -= preci;
                        ppar2 += preci;
                    }
                }

                toGC.SetRange(ppar1, ppar2);
                // if(fabs(ppar1- firstPar) > Precision::PConfusion() ||
                //     fabs(ppar2 - lastPar) >Precision::PConfusion()) // by LSS
                if (ppar1 != firstPar || ppar2 != lastPar)
                {
                    samerange = false;
                }
            }
            else if (toGC.IsCurveOnSurface())
            { // continue;  ||

                bool          localLinearFirst = useLinearFirst;
                bool          localLinearLast  = useLinearLast;
                Geom2d_Curve  C2d              = toGC.PCurve()!;
                double        first            = toGC.First();
                double        last             = toGC.Last();
                double        len              = last - first;
                Geom2dAdaptor_Curve      AC2d  = new Geom2dAdaptor_Curve(toGC.PCurve()!, first, last);
                GeomAdaptor_Surface      AdS   = new GeomAdaptor_Surface(toGC.Surface()!);
                Adaptor3d_CurveOnSurface Ad1   = new Adaptor3d_CurveOnSurface(AC2d, AdS);
                ShapeAnalysis_Curve      sac1  = new ShapeAnalysis_Curve();

                // gp_Pnt p1 = Ad1.Value(prevPar);
                // gp_Pnt p2 = Ad1.Value(currPar);
                gp_Pnt ploc1 = p1.Transformed(loc.Transformation());
                gp_Pnt ploc2 = p2.Transformed(loc.Transformation());
                // CATIA bplseitli.model FAC1155 - Copy: protection for degenerated edges
                double linFirst = first + alpha * len;
                double linLast  = first + beta * len;
                double dist1    = sac1.NextProject(linFirst, Ad1, ploc1, myPrecision, out _, out ppar1);
                double dist2    = sac1.NextProject(linLast, Ad1, ploc2, myPrecision, out _, out ppar2);

                bool isFirstOnEnd = (ppar1 - first) / len < Occt.Precision.PConfusion();
                bool isLastOnEnd  = (last - ppar2) / len < Occt.Precision.PConfusion();
                bool useLinear    = Math.Abs(ppar1 - ppar2) < Occt.Precision.PConfusion();
                if (isFirstOnEnd && !localLinearFirst)
                {
                    localLinearFirst = true;
                }

                if (isLastOnEnd && !localLinearLast)
                {
                    localLinearLast = true;
                }

                gp_Pnt pos1 = Ad1.Value(linFirst);
                gp_Pnt pos2 = Ad1.Value(linLast);
                double d01  = pos1.Distance(ploc1);
                double d02  = pos2.Distance(ploc2);
                if (localLinearFirst || useLinear || d01 <= dist1 || (d01 < myPrecision && d01 <= 2 * dist1))
                {
                    ppar1 = linFirst;
                }

                if (localLinearLast || useLinear || d02 <= dist2 || (d02 < myPrecision && d02 <= 2 * dist2))
                {
                    ppar2 = linLast;
                }

                if (ppar1 > ppar2)
                {
                    double tmpP = ppar2;
                    ppar2       = ppar1;
                    ppar1       = tmpP;
                }

                ppar1 = CorrectParameter(C2d, ppar1);
                ppar2 = CorrectParameter(C2d, ppar2);
                if (ppar2 - ppar1 < preci)
                {
                    if (ppar1 - toGC.First() < preci)
                    {
                        ppar2 += 2 * preci;
                    }
                    else if (toGC.Last() - ppar2 < preci)
                    {
                        ppar1 -= 2 * preci;
                    }
                    else
                    {
                        ppar1 -= preci;
                        ppar2 += preci;
                    }
                }

                toGC.SetRange(ppar1, ppar2);
                // if(fabs(ppar1 - firstPar) > Precision::PConfusion() ||
                //    fabs(ppar2 -lastPar) > Precision::PConfusion())// by LSS
                if (ppar1 != firstPar || ppar2 != lastPar)
                {
                    samerange = false;
                }
            }
        }

        B.SameRange(newEdge, samerange);
    }

    /// <summary>Returns false when the transfer is a projection, else ShapeAnalysis_TransferParameters' answer.</summary>
    public override bool IsSameRange()
    {
        if (!myInitOK || (!myForceProj && myPrecision < myMaxTolerance && BRep_Tool.SameParameter(myEdge)))
        {
            return base.IsSameRange();
        }
        else
        {
            return false;
        }
    }

    /// <summary>The flag forcing the projection (by default only edges that aren't same-parameter are projected).</summary>
    public ref bool ForceProjection()
    {
        return ref myForceProj;
    }

    /// <summary>
    /// Makes a copy of an INTERNAL or EXTERNAL vertex of <paramref name="fromedge"/> for <paramref name="toedge"/>, with
    /// its parameter on <paramref name="toedge"/>'s curve (projected when the ranges differ); a null vertex for any other
    /// orientation. The new vertex is frozen.
    /// </summary>
    public static TopoDS_Vertex CopyNMVertex(in TopoDS_Vertex theV, in TopoDS_Edge toedge, in TopoDS_Edge fromedge)
    {
        TopoDS_Vertex anewV = InternalCopyNMVertex(theV, toedge, fromedge);
        if (!anewV.IsNull())
        {
            anewV.TShape()!.FreezeGraph();
        }

        return anewV;
    }

    /// <summary>
    /// Makes a copy of an INTERNAL or EXTERNAL vertex of <paramref name="fromFace"/> for <paramref name="toFace"/>, with
    /// its parameters on <paramref name="toFace"/>'s surface; a null vertex for any other orientation. The new vertex is
    /// frozen.
    /// </summary>
    public static TopoDS_Vertex CopyNMVertex(in TopoDS_Vertex theV, in TopoDS_Face toFace, in TopoDS_Face fromFace)
    {
        TopoDS_Vertex anewV = InternalCopyNMVertex(theV, toFace, fromFace);
        if (!anewV.IsNull())
        {
            anewV.TShape()!.FreezeGraph();
        }

        return anewV;
    }

    /// <summary>OCCT's CopyNMVertex for edges: the new vertex isn't frozen, for ported code that goes on building.</summary>
    internal static TopoDS_Vertex InternalCopyNMVertex(in TopoDS_Vertex theV, in TopoDS_Edge toedge, in TopoDS_Edge fromedge)
    {
        TopoDS_Vertex anewV = new TopoDS_Vertex();
        if (theV.Orientation() != TopAbs_INTERNAL && theV.Orientation() != TopAbs_EXTERNAL)
        {
            return anewV;
        }

        Geom_Curve? C1 = BRep_Tool.Curve(fromedge, out TopLoc_Location fromLoc, out double f1, out double l1);
        fromLoc        = fromLoc.Predivided(theV.Location());

        Geom_Curve? C2 = BRep_Tool.Curve(toedge, out double f2, out double l2);

        anewV      = TopoDS.Vertex(theV.EmptyCopied());
        gp_Pnt apv = BRep_Tool.Pnt(anewV);

        NCollection_List<BRep_PointRepresentation> alistrep = ((BRep_TVertex)anewV.TShape()!).ChangePoints();

        NCollection_List<BRep_PointRepresentation>.Iterator itpr =
            new NCollection_List<BRep_PointRepresentation>.Iterator(((BRep_TVertex)theV.TShape()!).Points());
        double aOldPar = RealLast();
        bool   hasRepr = false;
        for (; itpr.More(); itpr.Next())
        {
            BRep_PointRepresentation? pr = itpr.Value();
            if (pr is null)
            {
                continue;
            }

            if (pr.IsPointOnCurve(C1, fromLoc))
            {
                aOldPar = pr.Parameter();
                hasRepr = true;
                continue;
            }
            else if (pr.IsPointOnSurface())
            {
                BRep_PointOnSurface aOld = (BRep_PointOnSurface)pr;
                BRep_PointOnSurface aPS  = new BRep_PointOnSurface(aOld.Parameter(), aOld.Parameter2(), aOld.Surface(), aOld.Location());
                alistrep.Append(aPS);
                continue;
            }
            else if (pr.IsPointOnCurveOnSurface())
            {
                bool found = false;
                // OCCT.NET: frozen: OCCT iterates fromedge's ChangeCurves(); it only reads them
                NCollection_List<BRep_CurveRepresentation>.Iterator fromitcr =
                    new NCollection_List<BRep_CurveRepresentation>.Iterator(((BRep_TEdge)fromedge.TShape()!).Curves());
                for (; fromitcr.More() && !found; fromitcr.Next())
                {
                    BRep_GCurve? fromGC = fromitcr.Value() as BRep_GCurve;
                    if (fromGC is null || !fromGC.IsCurveOnSurface())
                    {
                        continue;
                    }

                    TopLoc_Location aL       = fromGC.Location().Predivided(theV.Location());
                    Geom_Surface?   surface1 = fromGC.Surface();
                    Geom2d_Curve?   ac2d1    = fromGC.PCurve();
                    if (pr.IsPointOnCurveOnSurface(ac2d1, surface1, aL))
                    {
                        found = true;
                        if (!hasRepr)
                        {
                            aOldPar = pr.Parameter();
                        }
                    }
                }

                if (found)
                {
                    continue;
                }
            }

            if (pr.IsPointOnCurve())
            {
                BRep_PointOnCurve aPRep = new BRep_PointOnCurve(pr.Parameter(), pr.Curve(), pr.Location());
                alistrep.Append(aPRep);
            }
            else if (pr.IsPointOnCurveOnSurface())
            {
                BRep_PointOnCurveOnSurface aPonCS =
                    new BRep_PointOnCurveOnSurface(pr.Parameter(), pr.PCurve(), pr.Surface(), pr.Location());
                alistrep.Append(aPonCS);
            }
        }

        double apar = aOldPar;
        double aTol = BRep_Tool.Tolerance(theV);
        if (!hasRepr || (Math.Abs(f1 - f2) > Occt.Precision.PConfusion() || Math.Abs(l1 - l2) > Occt.Precision.PConfusion()))
        {
            ShapeAnalysis_Curve sae   = new ShapeAnalysis_Curve();
            double              adist = sae.Project(C2!, apv, Occt.Precision.Confusion(), out _, out apar);
            if (aTol < adist)
            {
                aTol = adist;
            }
        }

        BRep_Builder aB = new BRep_Builder();
        aB.UpdateVertex(anewV, apar, toedge, aTol);

        // update tolerance
        bool            needUpdate = false;
        gp_Pnt          aPV        = ((BRep_TVertex)anewV.TShape()!).Pnt();
        TopLoc_Location toLoc      = toedge.Location();
        // OCCT.NET: frozen: OCCT iterates toedge's ChangeCurves(); it only reads them
        NCollection_List<BRep_CurveRepresentation>.Iterator toitcr =
            new NCollection_List<BRep_CurveRepresentation>.Iterator(((BRep_TEdge)toedge.TShape()!).Curves());
        for (; toitcr.More(); toitcr.Next())
        {
            BRep_GCurve? toGC = toitcr.Value() as BRep_GCurve;
            if (toGC is null || !toGC.IsCurveOnSurface())
            {
                continue;
            }

            TopLoc_Location aL = (toLoc * toGC.Location()).Predivided(theV.Location());
            // aL.Predivided(theV.Location());
            Geom_Surface surface1 = toGC.Surface()!;
            Geom2d_Curve ac2d1    = toGC.PCurve()!;
            gp_Pnt2d     aP2d     = ac2d1.Value(apar);
            gp_Pnt       aP3d     = surface1.Value(aP2d.X(), aP2d.Y());
            aP3d.Transform(aL.Transformation());
            double adist = aPV.Distance(aP3d);
            if (adist > aTol)
            {
                aTol       = adist;
                needUpdate = true;
            }
        }

        if (needUpdate)
        {
            aB.UpdateVertex(anewV, aTol);
        }

        return anewV;
    }

    /// <summary>OCCT's CopyNMVertex for faces: the new vertex isn't frozen, for ported code that goes on building.</summary>
    internal static TopoDS_Vertex InternalCopyNMVertex(in TopoDS_Vertex theV, in TopoDS_Face toFace, in TopoDS_Face fromFace)
    {
        TopoDS_Vertex anewV = new TopoDS_Vertex();
        if (theV.Orientation() != TopAbs_INTERNAL && theV.Orientation() != TopAbs_EXTERNAL)
        {
            return anewV;
        }

        Geom_Surface? fromSurf = BRep_Tool.Surface(fromFace, out TopLoc_Location fromLoc);
        Geom_Surface? toSurf   = BRep_Tool.Surface(toFace, out TopLoc_Location toLoc);
        fromLoc                = fromLoc.Predivided(theV.Location());

        anewV      = TopoDS.Vertex(theV.EmptyCopied());
        gp_Pnt apv = BRep_Tool.Pnt(anewV);

        NCollection_List<BRep_PointRepresentation> alistrep = ((BRep_TVertex)anewV.TShape()!).ChangePoints();

        NCollection_List<BRep_PointRepresentation>.Iterator itpr =
            new NCollection_List<BRep_PointRepresentation>.Iterator(((BRep_TVertex)theV.TShape()!).Points());
        bool   hasRepr = false;
        double apar1 = 0.0, apar2 = 0.0;
        for (; itpr.More(); itpr.Next())
        {
            BRep_PointRepresentation? pr = itpr.Value();
            if (pr is null)
            {
                continue;
            }

            TopLoc_Location aLoc = pr.Location();
            if (pr.IsPointOnCurveOnSurface())
            {
                BRep_PointOnCurveOnSurface aPonCS = new BRep_PointOnCurveOnSurface(pr.Parameter(), pr.PCurve(), pr.Surface(), aLoc);
                alistrep.Append(aPonCS);
            }
            else if (pr.IsPointOnCurve())
            {
                BRep_PointOnCurve aPRep = new BRep_PointOnCurve(pr.Parameter(), pr.Curve(), aLoc);
                alistrep.Append(aPRep);
            }
            else if (pr.IsPointOnSurface())
            {
                BRep_PointOnSurface aOld = (BRep_PointOnSurface)pr;

                if (pr.IsPointOnSurface(fromSurf, fromLoc))
                {
                    apar1   = aOld.Parameter();
                    apar2   = aOld.Parameter2();
                    hasRepr = true;
                }
                else
                {
                    BRep_PointOnSurface aPS = new BRep_PointOnSurface(aOld.Parameter(), aOld.Parameter2(), aOld.Surface(), aOld.Location());
                    alistrep.Append(aPS);
                }
            }
        }

        double aTol = BRep_Tool.Tolerance(anewV);
        if (!hasRepr || (fromSurf != toSurf || fromLoc != toLoc))
        {
            Geom_Surface?         aS        = BRep_Tool.Surface(toFace);
            ShapeAnalysis_Surface aSurfTool = new ShapeAnalysis_Surface(aS);
            gp_Pnt2d              aP2d      = aSurfTool.ValueOfUV(apv, Occt.Precision.Confusion());
            apar1                           = aP2d.X();
            apar2                           = aP2d.Y();

            if (aTol < aSurfTool.Gap())
            {
                aTol = aSurfTool.Gap() + 0.1 * Occt.Precision.Confusion();
            }
            // occ::handle<BRep_PointOnSurface> aPS = new
            // BRep_PointOnSurface(aP2d.X(),aP2d.Y(),toSurf,toLoc); alistrep.Append(aPS);
        }

        BRep_Builder aB = new BRep_Builder();
        aB.UpdateVertex(anewV, apar1, apar2, toFace, aTol);
        return anewV;
    }
}
