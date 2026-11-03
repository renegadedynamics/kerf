// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_TransferParametersProj.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_TransferParametersProj.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Healing.Analysis;

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
public class TransferParametersProjAnalysis : TransferParametersAnalysis
{
    private Curve3D?              _curve;
    private Curve2D?            _curve2d;
    private CurveOnSurface _ac3d = new CurveOnSurface();
    private double                   _precision;
    private Placement          _location = new Placement();
    private bool                     _forceProj;
    private bool                     _initOK;

    /// <summary>Empty constructor.</summary>
    public TransferParametersProjAnalysis()
    {
        _precision    = 0.0;
        _maxTolerance = 1; // Precision::Infinite(); ?? pdn
        _forceProj    = false;
        _initOK       = false;
    }

    /// <summary>Creates the tool for an edge and a face (the face may be null: 3d curve only).</summary>
    public TransferParametersProjAnalysis(in Edge e, in Face f)
    {
        _maxTolerance = 1; // Precision::Infinite(); ?? pdn
        _forceProj    = false;
        Init(e, f);
    }

    /// <summary>Initializes the tool with an edge and a face.</summary>
    public override void Init(in Edge e, in Face f)
    {
        _initOK = false;
        base.Init(e, f);
        _edge      = e;
        _precision = Tool.Tolerance(e); // it is better - skl OCC2851
        // myPrecision = Precision::Confusion();

        _curve = Tool.Curve(e, out _first, out _last);
        if (_curve is null)
        {
            _first = 0.0;
            _last  = 1.0;
            return;
        }

        if (f.IsNull)
        {
            return;
        }

        EdgeAnalysis sae = new EdgeAnalysis();
        if (sae.PCurve(e, f, out _curve2d, out double f2d, out double l2d, false))
        {
            CurveAdaptor2D ac2d     = new CurveAdaptor2D(_curve2d!, f2d, l2d);
            Surface        surface = Tool.Surface(f, out _location)!;
            SurfaceAdaptor adS      = new SurfaceAdaptor(surface);

            CurveOnSurface ad1 = new CurveOnSurface(ac2d, adS);
            _ac3d   = ad1; // new Adaptor3d_CurveOnSurface(AC2d,AdS);
            _initOK = true;
        }
    }

    /// <summary>Transfers the parameters (in increasing order) from 3d to 2d or back, by projection.</summary>
    public override NCollection_HSequence<double> Perform(NCollection_HSequence<double> knots, bool to2d)
    {
        // pdn
        if (!_initOK || (!_forceProj && _precision < _maxTolerance && Tool.SameParameter(_edge)))
        {
            return base.Perform(knots, to2d);
        }

        NCollection_HSequence<double> resKnots = new NCollection_HSequence<double>();

        int    len   = knots.Length();
        double preci = 2 * Kerf.Engine.Foundation.Precision.PConfusion();

        double first   = (to2d ? _ac3d.FirstParameter() : _first);
        double last    = (to2d ? _ac3d.LastParameter() : _last);
        double maxPar  = first;
        double lastPar = last;
        double prevPar = maxPar;

        int j; // svv Jan 10 2000 : porting on DEC
        for (j = 1; j <= len; j++)
        {
            double par = PreformSegment(knots.Value(j), to2d, prevPar, lastPar);
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
        if (_curve!.IsClosed)
        {
            for (j = len; j >= 1; j--)
            {
                if (resKnots.Value(j) < maxPar)
                {
                    resKnots.SetValue(j, (to2d ? _ac3d.LastParameter() : _curve.LastParameter) - (len - j) * preci);
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

    private double PreformSegment(double param, bool to2d, double first, double last)
    {
        double linPar = base.Perform(param, to2d);
        if (!_initOK || (!_forceProj && _precision < _maxTolerance && Tool.SameParameter(_edge)))
        {
            return linPar;
        }

        double              linDev, projDev;
        CurveAnalysis sac = new CurveAnalysis();
        double              ppar;
        if (to2d)
        {
            Point3D                   p1   = _curve!.Value(param).Transformed(_location.Inverted().Transformation);
            SurfaceAdaptorBase        adS  = _ac3d.GetSurface()!;
            CurveAdaptor2D      ac2d = new CurveAdaptor2D(_curve2d!, first, last);
            CurveOnSurface ad1  = new CurveOnSurface(ac2d, adS);
            projDev = sac.Project(ad1, p1, _precision, out _, out ppar); // pdn
            linDev  = p1.DistanceTo(ad1.Value(linPar));
        }
        else
        {
            Point3D p1 = _ac3d.Value(param).Transformed(_location.Transformation);
            projDev   = sac.Project(_curve!, p1, _precision, out _, out ppar, first, last, false);
            linDev    = p1.DistanceTo(_curve!.Value(linPar));
        }

        if (linDev <= projDev || (linDev < _precision && linDev <= 2 * projDev))
        {
            ppar = linPar;
        }

        return ppar;
    }

    /// <summary>Transfers one parameter from 3d to 2d or back, by projection.</summary>
    public override double Perform(double knot, bool to2d)
    {
        if (!_initOK || (!_forceProj && _precision < _maxTolerance && Tool.SameParameter(_edge)))
        {
            return base.Perform(knot, to2d);
        }

        double res;
        if (to2d)
        {
            res = PreformSegment(knot, to2d, _ac3d.FirstParameter(), _ac3d.LastParameter());
        }
        else
        {
            res = PreformSegment(knot, to2d, _first, _last);
        }

        // pdn correction on range
        double first = (to2d ? _ac3d.FirstParameter() : _first);
        double last  = (to2d ? _ac3d.LastParameter() : _last);
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
            return CorrectParameter(tmpT.BasisCurve, param);
        }
        else if (crv is OffsetCurve2D tmpO)
        {
            return CorrectParameter(tmpO.BasisCurve, param);
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
    internal override void TransferRange(in Edge newEdge, double prevPar, double currPar, bool is2d)
    {
        if (!_initOK || (!_forceProj && _precision < _maxTolerance && Tool.SameParameter(_edge)))
        {
            base.TransferRange(newEdge, prevPar, currPar, is2d);
            return;
        }

        BrepBuilder    b         = new BrepBuilder();
        bool            samerange = true;
        BuildEdgeBuilder sbe       = new BuildEdgeBuilder();
        sbe.CopyRanges(newEdge, _edge);
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

        if (is2d)
        {
            p1 = _ac3d.Value(firstPar).Transformed(_location.Transformation);
            if (Kerf.Engine.Foundation.Precision.IsInfinite(p1.X) || Kerf.Engine.Foundation.Precision.IsInfinite(p1.Y) || Kerf.Engine.Foundation.Precision.IsInfinite(p1.Z))
            {
                b.SameRange(newEdge, false);
                return;
            }

            p2 = _ac3d.Value(lastPar).Transformed(_location.Transformation);
            if (Kerf.Engine.Foundation.Precision.IsInfinite(p2.X) || Kerf.Engine.Foundation.Precision.IsInfinite(p2.Y) || Kerf.Engine.Foundation.Precision.IsInfinite(p2.Z))
            {
                b.SameRange(newEdge, false);
                return;
            }

            double fact = _ac3d.LastParameter() - _ac3d.FirstParameter();
            if (fact > Epsilon(_ac3d.LastParameter()))
            {
                alpha = (firstPar - _ac3d.FirstParameter()) / fact;
                beta  = (lastPar - _ac3d.FirstParameter()) / fact;
            }
        }
        else
        {
            p1 = _curve!.Value(firstPar);
            if (Kerf.Engine.Foundation.Precision.IsInfinite(p1.X) || Kerf.Engine.Foundation.Precision.IsInfinite(p1.Y) || Kerf.Engine.Foundation.Precision.IsInfinite(p1.Z))
            {
                b.SameRange(newEdge, false);
                return;
            }

            p2 = _curve.Value(lastPar);
            if (Kerf.Engine.Foundation.Precision.IsInfinite(p2.X) || Kerf.Engine.Foundation.Precision.IsInfinite(p2.Y) || Kerf.Engine.Foundation.Precision.IsInfinite(p2.Z))
            {
                b.SameRange(newEdge, false);
                return;
            }

            double fact = _last - _first;
            if (fact > Epsilon(_last))
            {
                alpha = (firstPar - _first) / fact;
                beta  = (lastPar - _first) / fact;
            }
        }

        bool                useLinearFirst = (alpha < preci);
        bool                useLinearLast  = (1 - beta < preci);
        Placement     edgeLoc        = _edge.Placement;
        CurveAnalysis sac            = new CurveAnalysis();
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

            Placement loc = (edgeLoc * toGC.Location()).Inverted();
            if (toGC.IsCurve3D)
            {
                if (!is2d)
                {
                    ppar1 = firstPar;
                    ppar2 = lastPar;
                }
                else
                {
                    Curve3D? c3d = toGC.Curve3D();
                    if (c3d is null)
                    {
                        continue;
                    }

                    double            first = toGC.First();
                    double            last  = toGC.Last();
                    double            len   = last - first;
                    Point3D            ploc1 = p1.Transformed(loc.Transformation);
                    Point3D            ploc2 = p2.Transformed(loc.Transformation);
                    CurveAdaptor gac   = new CurveAdaptor(c3d, first, last);
                    // CATIA bplseitli.model FAC1155 - Copy: protection for degenerated edges(3d case for
                    // symmetry)
                    double linFirst  = first + alpha * len;
                    double linLast   = first + beta * len;
                    double dist1     = sac.NextProject(linFirst, gac, ploc1, _precision, out _, out ppar1);
                    double dist2     = sac.NextProject(linLast, gac, ploc2, _precision, out _, out ppar2);
                    bool   useLinear = Math.Abs(ppar1 - ppar2) < preci;

                    Point3D pos1 = c3d.Value(linFirst);
                    Point3D pos2 = c3d.Value(linLast);
                    double d01  = pos1.DistanceTo(ploc1);
                    double d02  = pos2.DistanceTo(ploc2);
                    if (useLinearFirst || useLinear || d01 <= dist1 || (d01 < _precision && d01 <= 2 * dist1))
                    {
                        ppar1 = linFirst;
                    }

                    if (useLinearLast || useLinear || d02 <= dist2 || (d02 < _precision && d02 <= 2 * dist2))
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
                Curve2D  c2d              = toGC.PCurve()!;
                double        first            = toGC.First();
                double        last             = toGC.Last();
                double        len              = last - first;
                CurveAdaptor2D      ac2d  = new CurveAdaptor2D(toGC.PCurve()!, first, last);
                SurfaceAdaptor      adS   = new SurfaceAdaptor(toGC.Surface()!);
                CurveOnSurface ad1   = new CurveOnSurface(ac2d, adS);
                CurveAnalysis      sac1  = new CurveAnalysis();

                // gp_Pnt p1 = Ad1.Value(prevPar);
                // gp_Pnt p2 = Ad1.Value(currPar);
                Point3D ploc1 = p1.Transformed(loc.Transformation);
                Point3D ploc2 = p2.Transformed(loc.Transformation);
                // CATIA bplseitli.model FAC1155 - Copy: protection for degenerated edges
                double linFirst = first + alpha * len;
                double linLast  = first + beta * len;
                double dist1    = sac1.NextProject(linFirst, ad1, ploc1, _precision, out _, out ppar1);
                double dist2    = sac1.NextProject(linLast, ad1, ploc2, _precision, out _, out ppar2);

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

                Point3D pos1 = ad1.Value(linFirst);
                Point3D pos2 = ad1.Value(linLast);
                double d01  = pos1.DistanceTo(ploc1);
                double d02  = pos2.DistanceTo(ploc2);
                if (localLinearFirst || useLinear || d01 <= dist1 || (d01 < _precision && d01 <= 2 * dist1))
                {
                    ppar1 = linFirst;
                }

                if (localLinearLast || useLinear || d02 <= dist2 || (d02 < _precision && d02 <= 2 * dist2))
                {
                    ppar2 = linLast;
                }

                if (ppar1 > ppar2)
                {
                    double tmpP = ppar2;
                    ppar2       = ppar1;
                    ppar1       = tmpP;
                }

                ppar1 = CorrectParameter(c2d, ppar1);
                ppar2 = CorrectParameter(c2d, ppar2);
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

        b.SameRange(newEdge, samerange);
    }

    /// <summary>Returns false when the transfer is a projection, else ShapeAnalysis_TransferParameters' answer.</summary>
    public override bool IsSameRange
    {
        get
        {
            if (!_initOK || (!_forceProj && _precision < _maxTolerance && Tool.SameParameter(_edge)))
            {
                return base.IsSameRange;
            }
            else
            {
                return false;
            }
        }
    }

    /// <summary>The flag forcing the projection (by default only edges that aren't same-parameter are projected).</summary>
    public ref bool ForceProjection()
    {
        return ref _forceProj;
    }

    /// <summary>
    /// Makes a copy of an INTERNAL or EXTERNAL vertex of <paramref name="fromedge"/> for <paramref name="toedge"/>, with
    /// its parameter on <paramref name="toedge"/>'s curve (projected when the ranges differ); a null vertex for any other
    /// orientation. The new vertex is frozen.
    /// </summary>
    public static Vertex CopyNMVertex(in Vertex v, in Edge toedge, in Edge fromedge)
    {
        Vertex anewV = InternalCopyNMVertex(v, toedge, fromedge);
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
    public static Vertex CopyNMVertex(in Vertex v, in Face toFace, in Face fromFace)
    {
        Vertex anewV = InternalCopyNMVertex(v, toFace, fromFace);
        if (!anewV.IsNull)
        {
            anewV.TShape()!.FreezeGraph();
        }

        return anewV;
    }

    /// <summary>OCCT's CopyNMVertex for edges: the new vertex isn't frozen, for ported code that goes on building.</summary>
    internal static Vertex InternalCopyNMVertex(in Vertex v, in Edge toedge, in Edge fromedge)
    {
        Vertex anewV = new Vertex();
        if (v.Orientation != Internal && v.Orientation != External)
        {
            return anewV;
        }

        Curve3D? c1 = Tool.Curve(fromedge, out Placement fromLoc, out double f1, out double l1);
        fromLoc        = fromLoc.Predivided(v.Placement);

        Curve3D? c2 = Tool.Curve(toedge, out double f2, out double l2);

        anewV      = ShapeCasts.Vertex(v.EmptyCopied());
        Point3D apv = Tool.Pnt(anewV);

        NCollection_List<PointRepresentation> alistrep = ((BrepTVertex)anewV.TShape()!).ChangePoints();

        NCollection_List<PointRepresentation>.Iterator itpr =
            new NCollection_List<PointRepresentation>.Iterator(((BrepTVertex)v.TShape()!).Points());
        double oldPar = RealLast();
        bool   hasRepr = false;
        for (; itpr.More(); itpr.Next())
        {
            PointRepresentation? pr = itpr.Value();
            if (pr is null)
            {
                continue;
            }

            if (pr.IsPointOnCurve(c1, fromLoc))
            {
                oldPar = pr.Parameter();
                hasRepr = true;
                continue;
            }
            else if (pr.IsPointOnSurface())
            {
                PointOnSurface old = (PointOnSurface)pr;
                PointOnSurface ps  = new PointOnSurface(old.Parameter(), old.Parameter2(), old.Surface(), old.Location());
                alistrep.Append(ps);
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

                    Placement l       = fromGC.Location().Predivided(v.Placement);
                    Surface?   surface1 = fromGC.Surface();
                    Curve2D?   ac2d1    = fromGC.PCurve();
                    if (pr.IsPointOnCurveOnSurface(ac2d1, surface1, l))
                    {
                        found = true;
                        if (!hasRepr)
                        {
                            oldPar = pr.Parameter();
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
                PointOnCurve pRep = new PointOnCurve(pr.Parameter(), pr.Curve(), pr.Location());
                alistrep.Append(pRep);
            }
            else if (pr.IsPointOnCurveOnSurface())
            {
                PointOnCurveOnSurface ponCS =
                    new PointOnCurveOnSurface(pr.Parameter(), pr.PCurve(), pr.Surface(), pr.Location());
                alistrep.Append(ponCS);
            }
        }

        double apar = oldPar;
        double tol = Tool.Tolerance(v);
        if (!hasRepr || (Math.Abs(f1 - f2) > Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(l1 - l2) > Kerf.Engine.Foundation.Precision.PConfusion()))
        {
            CurveAnalysis sae   = new CurveAnalysis();
            double              adist = sae.Project(c2!, apv, Kerf.Engine.Foundation.Precision.Confusion(), out _, out apar);
            if (tol < adist)
            {
                tol = adist;
            }
        }

        BrepBuilder b = new BrepBuilder();
        b.UpdateVertex(anewV, apar, toedge, tol);

        // update tolerance
        bool            needUpdate = false;
        Point3D          pv        = ((BrepTVertex)anewV.TShape()!).Pnt();
        Placement toLoc      = toedge.Placement;
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

            Placement l = (toLoc * toGC.Location()).Predivided(v.Placement);
            // aL.Predivided(theV.Location());
            Surface surface1 = toGC.Surface()!;
            Curve2D ac2d1    = toGC.PCurve()!;
            Point2D     p2d     = ac2d1.Value(apar);
            Point3D       p3d     = surface1.Value(p2d.X, p2d.Y);
            p3d = p3d.Transformed(l.Transformation);
            double adist = pv.DistanceTo(p3d);
            if (adist > tol)
            {
                tol       = adist;
                needUpdate = true;
            }
        }

        if (needUpdate)
        {
            b.UpdateVertex(anewV, tol);
        }

        return anewV;
    }

    /// <summary>OCCT's CopyNMVertex for faces: the new vertex isn't frozen, for ported code that goes on building.</summary>
    internal static Vertex InternalCopyNMVertex(in Vertex v, in Face toFace, in Face fromFace)
    {
        Vertex anewV = new Vertex();
        if (v.Orientation != Internal && v.Orientation != External)
        {
            return anewV;
        }

        Surface? fromSurf = Tool.Surface(fromFace, out Placement fromLoc);
        Surface? toSurf   = Tool.Surface(toFace, out Placement toLoc);
        fromLoc                = fromLoc.Predivided(v.Placement);

        anewV      = ShapeCasts.Vertex(v.EmptyCopied());
        Point3D apv = Tool.Pnt(anewV);

        NCollection_List<PointRepresentation> alistrep = ((BrepTVertex)anewV.TShape()!).ChangePoints();

        NCollection_List<PointRepresentation>.Iterator itpr =
            new NCollection_List<PointRepresentation>.Iterator(((BrepTVertex)v.TShape()!).Points());
        bool   hasRepr = false;
        double apar1 = 0.0, apar2 = 0.0;
        for (; itpr.More(); itpr.Next())
        {
            PointRepresentation? pr = itpr.Value();
            if (pr is null)
            {
                continue;
            }

            Placement loc = pr.Location();
            if (pr.IsPointOnCurveOnSurface())
            {
                PointOnCurveOnSurface ponCS = new PointOnCurveOnSurface(pr.Parameter(), pr.PCurve(), pr.Surface(), loc);
                alistrep.Append(ponCS);
            }
            else if (pr.IsPointOnCurve())
            {
                PointOnCurve pRep = new PointOnCurve(pr.Parameter(), pr.Curve(), loc);
                alistrep.Append(pRep);
            }
            else if (pr.IsPointOnSurface())
            {
                PointOnSurface old = (PointOnSurface)pr;

                if (pr.IsPointOnSurface(fromSurf, fromLoc))
                {
                    apar1   = old.Parameter();
                    apar2   = old.Parameter2();
                    hasRepr = true;
                }
                else
                {
                    PointOnSurface ps = new PointOnSurface(old.Parameter(), old.Parameter2(), old.Surface(), old.Location());
                    alistrep.Append(ps);
                }
            }
        }

        double tol = Tool.Tolerance(anewV);
        if (!hasRepr || (fromSurf != toSurf || !fromLoc.IsEqual(toLoc)))
        {
            Surface?         s        = Tool.Surface(toFace);
            SurfaceAnalysis surfTool = new SurfaceAnalysis(s);
            Point2D              p2d      = surfTool.ValueOfUV(apv, Kerf.Engine.Foundation.Precision.Confusion());
            apar1                           = p2d.X;
            apar2                           = p2d.Y;

            if (tol < surfTool.Gap())
            {
                tol = surfTool.Gap() + 0.1 * Kerf.Engine.Foundation.Precision.Confusion();
            }
            // occ::handle<BRep_PointOnSurface> aPS = new
            // BRep_PointOnSurface(aP2d.X(),aP2d.Y(),toSurf,toLoc); alistrep.Append(aPS);
        }

        BrepBuilder b = new BrepBuilder();
        b.UpdateVertex(anewV, apar1, apar2, toFace, tol);
        return anewV;
    }
}
