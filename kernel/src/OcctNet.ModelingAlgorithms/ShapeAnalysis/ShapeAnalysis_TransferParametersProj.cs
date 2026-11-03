// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_TransferParametersProj.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_TransferParametersProj.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>
/// Transfers parameters between an edge's 3d curve and its pcurve by projection, where ShapeAnalysis_TransferParameters
/// does it linearly. Used when an edge isn't same-parameter (or its tolerance is at least the maximal tolerance, or
/// projection is forced).
/// </summary>
/// <remarks>
/// A working object. <see cref="TransferRange"/> writes into the edge it is given and is internal (ported code calls
/// it on edges it builds; it throws on a frozen edge). The public <see cref="CopyNMVertex(in Vertex, in
/// Edge, in Edge)"/> and its face form return new frozen vertices; ported code that goes on building
/// calls the internal non-freezing forms.
/// </remarks>
public class ShapeAnalysis_TransferParametersProj : ShapeAnalysis_TransferParameters
{
    private Curve3D?              myCurve;
    private Curve2D?            myCurve2d;
    private CurveOnSurface myAC3d = new CurveOnSurface();
    private double                   myPrecision;
    private Placement          myLocation = new Placement();
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
    public ShapeAnalysis_TransferParametersProj(in Edge E, in Face F)
    {
        myMaxTolerance = 1; // Precision::Infinite(); ?? pdn
        myForceProj    = false;
        Init(E, F);
    }

    /// <summary>Initializes the tool with an edge and a face.</summary>
    public override void Init(in Edge E, in Face F)
    {
        myInitOK = false;
        base.Init(E, F);
        myEdge      = E;
        myPrecision = Tool.Tolerance(E); // it is better - skl OCC2851
        // myPrecision = Precision::Confusion();

        myCurve = Tool.Curve(E, out myFirst, out myLast);
        if (myCurve is null)
        {
            myFirst = 0.0;
            myLast  = 1.0;
            return;
        }

        if (F.IsNull)
        {
            return;
        }

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        if (sae.PCurve(E, F, out myCurve2d, out double f2d, out double l2d, false))
        {
            CurveAdaptor2D AC2d     = new CurveAdaptor2D(myCurve2d!, f2d, l2d);
            Surface        aSurface = Tool.Surface(F, out myLocation)!;
            SurfaceAdaptor AdS      = new SurfaceAdaptor(aSurface);

            CurveOnSurface Ad1 = new CurveOnSurface(AC2d, AdS);
            myAC3d   = Ad1; // new Adaptor3d_CurveOnSurface(AC2d,AdS);
            myInitOK = true;
        }
    }

    /// <summary>Transfers the parameters (in increasing order) from 3d to 2d or back, by projection.</summary>
    public override NCollection_HSequence<double> Perform(NCollection_HSequence<double> Knots, bool To2d)
    {
        // pdn
        if (!myInitOK || (!myForceProj && myPrecision < myMaxTolerance && Tool.SameParameter(myEdge)))
        {
            return base.Perform(Knots, To2d);
        }

        NCollection_HSequence<double> resKnots = new NCollection_HSequence<double>();

        int    len   = Knots.Length();
        double preci = 2 * Kerf.Engine.Foundation.Precision.PConfusion();

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
        if (myCurve!.IsClosed)
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
        if (!myInitOK || (!myForceProj && myPrecision < myMaxTolerance && Tool.SameParameter(myEdge)))
        {
            return linPar;
        }

        double              linDev, projDev;
        ShapeAnalysis_Curve sac = new ShapeAnalysis_Curve();
        double              ppar;
        if (To2d)
        {
            Point3D                   p1   = myCurve!.Value(Param).Transformed(myLocation.Inverted().Transformation());
            SurfaceAdaptorBase        AdS  = myAC3d.GetSurface()!;
            CurveAdaptor2D      AC2d = new CurveAdaptor2D(myCurve2d!, First, Last);
            CurveOnSurface Ad1  = new CurveOnSurface(AC2d, AdS);
            projDev = sac.Project(Ad1, p1, myPrecision, out _, out ppar); // pdn
            linDev  = p1.Distance(Ad1.Value(linPar));
        }
        else
        {
            Point3D p1 = myAC3d.Value(Param).Transformed(myLocation.Transformation());
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
        if (!myInitOK || (!myForceProj && myPrecision < myMaxTolerance && Tool.SameParameter(myEdge)))
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

    private static double CorrectParameter(Curve2D crv, double param)
    {
        if (crv is TrimmedCurve2D tmpT)
        {
            return CorrectParameter(tmpT.BasisCurve(), param);
        }
        else if (crv is OffsetCurve2D tmpO)
        {
            return CorrectParameter(tmpO.BasisCurve(), param);
        }
        else if (crv is BSplineCurve2D bspline)
        {
            for (int j = bspline.FirstUKnotIndex(); j <= bspline.LastUKnotIndex(); j++)
            {
                double valknot = bspline.Knot(j);
                if (Math.Abs(valknot - param) < Kerf.Engine.Foundation.Precision.PConfusion())
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
    internal override void TransferRange(in Edge newEdge, double prevPar, double currPar, bool Is2d)
    {
        if (!myInitOK || (!myForceProj && myPrecision < myMaxTolerance && Tool.SameParameter(myEdge)))
        {
            base.TransferRange(newEdge, prevPar, currPar, Is2d);
            return;
        }

        BrepBuilder    B         = new BrepBuilder();
        bool            samerange = true;
        ShapeBuild_Edge sbe       = new ShapeBuild_Edge();
        sbe.CopyRanges(newEdge, myEdge);
        Point3D p1;
        Point3D p2;
        double alpha = 0, beta = 1;
        double preci = Kerf.Engine.Foundation.Precision.PConfusion();
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
            if (Kerf.Engine.Foundation.Precision.IsInfinite(p1.X()) || Kerf.Engine.Foundation.Precision.IsInfinite(p1.Y()) || Kerf.Engine.Foundation.Precision.IsInfinite(p1.Z()))
            {
                B.SameRange(newEdge, false);
                return;
            }

            p2 = myAC3d.Value(lastPar).Transformed(myLocation.Transformation());
            if (Kerf.Engine.Foundation.Precision.IsInfinite(p2.X()) || Kerf.Engine.Foundation.Precision.IsInfinite(p2.Y()) || Kerf.Engine.Foundation.Precision.IsInfinite(p2.Z()))
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
            if (Kerf.Engine.Foundation.Precision.IsInfinite(p1.X()) || Kerf.Engine.Foundation.Precision.IsInfinite(p1.Y()) || Kerf.Engine.Foundation.Precision.IsInfinite(p1.Z()))
            {
                B.SameRange(newEdge, false);
                return;
            }

            p2 = myCurve.Value(lastPar);
            if (Kerf.Engine.Foundation.Precision.IsInfinite(p2.X()) || Kerf.Engine.Foundation.Precision.IsInfinite(p2.Y()) || Kerf.Engine.Foundation.Precision.IsInfinite(p2.Z()))
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
        Placement     EdgeLoc        = myEdge.Location();
        ShapeAnalysis_Curve sac            = new ShapeAnalysis_Curve();
        double              ppar1, ppar2;
        // OCCT.NET: frozen: ChangeCurves() throws on a frozen edge, as BRep_Builder does
        NCollection_List<CurveRepresentation> tolist = ((BrepTEdge)newEdge.TShape()!).ChangeCurves();
        GCurve? toGC;
        for (NCollection_List<CurveRepresentation>.Iterator toitcr = new NCollection_List<CurveRepresentation>.Iterator(tolist);
             toitcr.More();
             toitcr.Next())
        {
            toGC = toitcr.Value() as GCurve;
            if (toGC is null)
            {
                continue;
            }

            Placement loc = (EdgeLoc * toGC.Location()).Inverted();
            if (toGC.IsCurve3D)
            {
                if (!Is2d)
                {
                    ppar1 = firstPar;
                    ppar2 = lastPar;
                }
                else
                {
                    Curve3D? C3d = toGC.Curve3D();
                    if (C3d is null)
                    {
                        continue;
                    }

                    double            first = toGC.First();
                    double            last  = toGC.Last();
                    double            len   = last - first;
                    Point3D            ploc1 = p1.Transformed(loc.Transformation());
                    Point3D            ploc2 = p2.Transformed(loc.Transformation());
                    CurveAdaptor GAC   = new CurveAdaptor(C3d, first, last);
                    // CATIA bplseitli.model FAC1155 - Copy: protection for degenerated edges(3d case for
                    // symmetry)
                    double linFirst  = first + alpha * len;
                    double linLast   = first + beta * len;
                    double dist1     = sac.NextProject(linFirst, GAC, ploc1, myPrecision, out _, out ppar1);
                    double dist2     = sac.NextProject(linLast, GAC, ploc2, myPrecision, out _, out ppar2);
                    bool   useLinear = Math.Abs(ppar1 - ppar2) < preci;

                    Point3D pos1 = C3d.Value(linFirst);
                    Point3D pos2 = C3d.Value(linLast);
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
                Curve2D  C2d              = toGC.PCurve()!;
                double        first            = toGC.First();
                double        last             = toGC.Last();
                double        len              = last - first;
                CurveAdaptor2D      AC2d  = new CurveAdaptor2D(toGC.PCurve()!, first, last);
                SurfaceAdaptor      AdS   = new SurfaceAdaptor(toGC.Surface()!);
                CurveOnSurface Ad1   = new CurveOnSurface(AC2d, AdS);
                ShapeAnalysis_Curve      sac1  = new ShapeAnalysis_Curve();

                // gp_Pnt p1 = Ad1.Value(prevPar);
                // gp_Pnt p2 = Ad1.Value(currPar);
                Point3D ploc1 = p1.Transformed(loc.Transformation());
                Point3D ploc2 = p2.Transformed(loc.Transformation());
                // CATIA bplseitli.model FAC1155 - Copy: protection for degenerated edges
                double linFirst = first + alpha * len;
                double linLast  = first + beta * len;
                double dist1    = sac1.NextProject(linFirst, Ad1, ploc1, myPrecision, out _, out ppar1);
                double dist2    = sac1.NextProject(linLast, Ad1, ploc2, myPrecision, out _, out ppar2);

                bool isFirstOnEnd = (ppar1 - first) / len < Kerf.Engine.Foundation.Precision.PConfusion();
                bool isLastOnEnd  = (last - ppar2) / len < Kerf.Engine.Foundation.Precision.PConfusion();
                bool useLinear    = Math.Abs(ppar1 - ppar2) < Kerf.Engine.Foundation.Precision.PConfusion();
                if (isFirstOnEnd && !localLinearFirst)
                {
                    localLinearFirst = true;
                }

                if (isLastOnEnd && !localLinearLast)
                {
                    localLinearLast = true;
                }

                Point3D pos1 = Ad1.Value(linFirst);
                Point3D pos2 = Ad1.Value(linLast);
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
        if (!myInitOK || (!myForceProj && myPrecision < myMaxTolerance && Tool.SameParameter(myEdge)))
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
    public static Vertex CopyNMVertex(in Vertex theV, in Edge toedge, in Edge fromedge)
    {
        Vertex anewV = InternalCopyNMVertex(theV, toedge, fromedge);
        if (!anewV.IsNull)
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
    public static Vertex CopyNMVertex(in Vertex theV, in Face toFace, in Face fromFace)
    {
        Vertex anewV = InternalCopyNMVertex(theV, toFace, fromFace);
        if (!anewV.IsNull)
        {
            anewV.TShape()!.FreezeGraph();
        }

        return anewV;
    }

    /// <summary>OCCT's CopyNMVertex for edges: the new vertex isn't frozen, for ported code that goes on building.</summary>
    internal static Vertex InternalCopyNMVertex(in Vertex theV, in Edge toedge, in Edge fromedge)
    {
        Vertex anewV = new Vertex();
        if (theV.Orientation() != Internal && theV.Orientation() != External)
        {
            return anewV;
        }

        Curve3D? C1 = Tool.Curve(fromedge, out Placement fromLoc, out double f1, out double l1);
        fromLoc        = fromLoc.Predivided(theV.Location());

        Curve3D? C2 = Tool.Curve(toedge, out double f2, out double l2);

        anewV      = ShapeCasts.Vertex(theV.EmptyCopied());
        Point3D apv = Tool.Pnt(anewV);

        NCollection_List<PointRepresentation> alistrep = ((BrepTVertex)anewV.TShape()!).ChangePoints();

        NCollection_List<PointRepresentation>.Iterator itpr =
            new NCollection_List<PointRepresentation>.Iterator(((BrepTVertex)theV.TShape()!).Points());
        double aOldPar = RealLast();
        bool   hasRepr = false;
        for (; itpr.More(); itpr.Next())
        {
            PointRepresentation? pr = itpr.Value();
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
                PointOnSurface aOld = (PointOnSurface)pr;
                PointOnSurface aPS  = new PointOnSurface(aOld.Parameter(), aOld.Parameter2(), aOld.Surface(), aOld.Location());
                alistrep.Append(aPS);
                continue;
            }
            else if (pr.IsPointOnCurveOnSurface())
            {
                bool found = false;
                // OCCT.NET: frozen: OCCT iterates fromedge's ChangeCurves(); it only reads them
                NCollection_List<CurveRepresentation>.Iterator fromitcr =
                    new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)fromedge.TShape()!).Curves());
                for (; fromitcr.More() && !found; fromitcr.Next())
                {
                    GCurve? fromGC = fromitcr.Value() as GCurve;
                    if (fromGC is null || !fromGC.IsCurveOnSurface())
                    {
                        continue;
                    }

                    Placement aL       = fromGC.Location().Predivided(theV.Location());
                    Surface?   surface1 = fromGC.Surface();
                    Curve2D?   ac2d1    = fromGC.PCurve();
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
                PointOnCurve aPRep = new PointOnCurve(pr.Parameter(), pr.Curve(), pr.Location());
                alistrep.Append(aPRep);
            }
            else if (pr.IsPointOnCurveOnSurface())
            {
                PointOnCurveOnSurface aPonCS =
                    new PointOnCurveOnSurface(pr.Parameter(), pr.PCurve(), pr.Surface(), pr.Location());
                alistrep.Append(aPonCS);
            }
        }

        double apar = aOldPar;
        double aTol = Tool.Tolerance(theV);
        if (!hasRepr || (Math.Abs(f1 - f2) > Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(l1 - l2) > Kerf.Engine.Foundation.Precision.PConfusion()))
        {
            ShapeAnalysis_Curve sae   = new ShapeAnalysis_Curve();
            double              adist = sae.Project(C2!, apv, Kerf.Engine.Foundation.Precision.Confusion(), out _, out apar);
            if (aTol < adist)
            {
                aTol = adist;
            }
        }

        BrepBuilder aB = new BrepBuilder();
        aB.UpdateVertex(anewV, apar, toedge, aTol);

        // update tolerance
        bool            needUpdate = false;
        Point3D          aPV        = ((BrepTVertex)anewV.TShape()!).Pnt();
        Placement toLoc      = toedge.Location();
        // OCCT.NET: frozen: OCCT iterates toedge's ChangeCurves(); it only reads them
        NCollection_List<CurveRepresentation>.Iterator toitcr =
            new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)toedge.TShape()!).Curves());
        for (; toitcr.More(); toitcr.Next())
        {
            GCurve? toGC = toitcr.Value() as GCurve;
            if (toGC is null || !toGC.IsCurveOnSurface())
            {
                continue;
            }

            Placement aL = (toLoc * toGC.Location()).Predivided(theV.Location());
            // aL.Predivided(theV.Location());
            Surface surface1 = toGC.Surface()!;
            Curve2D ac2d1    = toGC.PCurve()!;
            Point2D     aP2d     = ac2d1.Value(apar);
            Point3D       aP3d     = surface1.Value(aP2d.X(), aP2d.Y());
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
    internal static Vertex InternalCopyNMVertex(in Vertex theV, in Face toFace, in Face fromFace)
    {
        Vertex anewV = new Vertex();
        if (theV.Orientation() != Internal && theV.Orientation() != External)
        {
            return anewV;
        }

        Surface? fromSurf = Tool.Surface(fromFace, out Placement fromLoc);
        Surface? toSurf   = Tool.Surface(toFace, out Placement toLoc);
        fromLoc                = fromLoc.Predivided(theV.Location());

        anewV      = ShapeCasts.Vertex(theV.EmptyCopied());
        Point3D apv = Tool.Pnt(anewV);

        NCollection_List<PointRepresentation> alistrep = ((BrepTVertex)anewV.TShape()!).ChangePoints();

        NCollection_List<PointRepresentation>.Iterator itpr =
            new NCollection_List<PointRepresentation>.Iterator(((BrepTVertex)theV.TShape()!).Points());
        bool   hasRepr = false;
        double apar1 = 0.0, apar2 = 0.0;
        for (; itpr.More(); itpr.Next())
        {
            PointRepresentation? pr = itpr.Value();
            if (pr is null)
            {
                continue;
            }

            Placement aLoc = pr.Location();
            if (pr.IsPointOnCurveOnSurface())
            {
                PointOnCurveOnSurface aPonCS = new PointOnCurveOnSurface(pr.Parameter(), pr.PCurve(), pr.Surface(), aLoc);
                alistrep.Append(aPonCS);
            }
            else if (pr.IsPointOnCurve())
            {
                PointOnCurve aPRep = new PointOnCurve(pr.Parameter(), pr.Curve(), aLoc);
                alistrep.Append(aPRep);
            }
            else if (pr.IsPointOnSurface())
            {
                PointOnSurface aOld = (PointOnSurface)pr;

                if (pr.IsPointOnSurface(fromSurf, fromLoc))
                {
                    apar1   = aOld.Parameter();
                    apar2   = aOld.Parameter2();
                    hasRepr = true;
                }
                else
                {
                    PointOnSurface aPS = new PointOnSurface(aOld.Parameter(), aOld.Parameter2(), aOld.Surface(), aOld.Location());
                    alistrep.Append(aPS);
                }
            }
        }

        double aTol = Tool.Tolerance(anewV);
        if (!hasRepr || (fromSurf != toSurf || fromLoc != toLoc))
        {
            Surface?         aS        = Tool.Surface(toFace);
            ShapeAnalysis_Surface aSurfTool = new ShapeAnalysis_Surface(aS);
            Point2D              aP2d      = aSurfTool.ValueOfUV(apv, Kerf.Engine.Foundation.Precision.Confusion());
            apar1                           = aP2d.X();
            apar2                           = aP2d.Y();

            if (aTol < aSurfTool.Gap())
            {
                aTol = aSurfTool.Gap() + 0.1 * Kerf.Engine.Foundation.Precision.Confusion();
            }
            // occ::handle<BRep_PointOnSurface> aPS = new
            // BRep_PointOnSurface(aP2d.X(),aP2d.Y(),toSurf,toLoc); alistrep.Append(aPS);
        }

        BrepBuilder aB = new BrepBuilder();
        aB.UpdateVertex(anewV, apar1, apar2, toFace, aTol);
        return anewV;
    }
}
