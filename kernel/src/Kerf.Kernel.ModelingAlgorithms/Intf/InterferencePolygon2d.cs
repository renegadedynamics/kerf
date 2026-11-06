// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_InterferencePolygon2d.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_InterferencePolygon2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Buffers;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;

namespace Kerf.Engine.Intersection.Polygons;

/// <summary>The interference between two polygons, or the self-intersection of a polygon, in two dimensions.</summary>
internal class InterferencePolygon2d : PolygonInterference
{
    // Angular precision (sinus) below that value two right segments
    // are considered as having a potential zone of tangency.
    private static readonly double Prcang = global::Kerf.Engine.Foundation.Precision.Angular();

    private bool _oClos;
    private bool _tClos;
    private int _nbso;
    private NCollection_Sequence<SectionPoint>? _sectionWork; // Intersect's, as most pairs of segments it's asked about give no point

    /// <summary>Constructs an empty interference of polygons.</summary>
    public InterferencePolygon2d()
        : base(false)
    {
        _oClos = false;
        _tClos = false;
        _nbso = 0;
    }

    /// <summary>Constructs and computes an interference between two polygons.</summary>
    public InterferencePolygon2d(Polygon2d obje1, Polygon2d obje2)
        : base(false)
    {
        _oClos = false;
        _tClos = false;
        _nbso = 0;
        if (!obje1.Bounding().IsOut(obje2.Bounding()))
        {
            _tolerance = obje1.DeflectionOverEstimation() + obje2.DeflectionOverEstimation();
            if (_tolerance == 0.0)
            {
                _tolerance = Epsilon(1000.0);
            }

            _nbso = obje1.SegmentCount;
            _oClos = obje1.Closed();
            _tClos = obje2.Closed();
            Interference(obje1, obje2);
            Clean();
        }
    }

    /// <summary>Constructs and computes the self-interference of a polygon.</summary>
    public InterferencePolygon2d(Polygon2d obje)
        : base(true)
    {
        _oClos = false;
        _tClos = false;
        _nbso = 0;
        _tolerance = obje.DeflectionOverEstimation() * 2;
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }

        _oClos = obje.Closed();
        _tClos = _oClos;
        Interference(obje);
        Clean();
    }

    /// <summary>Computes an interference between two polygons.</summary>
    public void Perform(Polygon2d obje1, Polygon2d obje2)
    {
        SelfInterference(false);
        if (!obje1.Bounding().IsOut(obje2.Bounding()))
        {
            _tolerance = obje1.DeflectionOverEstimation() + obje2.DeflectionOverEstimation();
            if (_tolerance == 0.0)
            {
                _tolerance = Epsilon(1000.0);
            }

            _nbso = obje1.SegmentCount;
            _oClos = obje1.Closed();
            _tClos = obje2.Closed();
            Interference(obje1, obje2);
            Clean();
        }
    }

    /// <summary>Computes the self-interference of a polygon.</summary>
    public void Perform(Polygon2d obje)
    {
        SelfInterference(true);
        _tolerance = obje.DeflectionOverEstimation() * 2;
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }

        _oClos = obje.Closed();
        _tClos = _oClos;
        Interference(obje);
        Clean();
    }

    /// <summary>The 2D point of the intersection point of address <paramref name="index"/> in the interference.</summary>
    public Point2D Pnt2dValue(int index)
    {
        return new Point2D((_sPoins[index]).Pnt().X, (_sPoins[index]).Pnt().Y);
    }

    private void Interference(Polygon2d obje1, Polygon2d obje2)
    {
        Box2d bSO = new();
        Box2d bST = new();

        int iObje1, iObje2, n1 = _nbso, n2 = obje2.SegmentCount;
        double d1 = obje1.DeflectionOverEstimation(), d2 = obje2.DeflectionOverEstimation();

        // As for one polygon, below: most pairs are far apart, and the extents answer them. Its margin only ever leaves a pair to the exact test
        double[] extents = ArrayPool<double>.Shared.Rent(4 * (n2 + 1));
        Point2D p1b, p1e, p2b, p2e;
        for (iObje2 = 1; iObje2 <= n2; iObje2++)
        {
            obje2.Segment(iObje2, out p2b, out p2e);
            extents[4 * iObje2] = Math.Min(p2b.X, p2e.X);
            extents[4 * iObje2 + 1] = Math.Max(p2b.X, p2e.X);
            extents[4 * iObje2 + 2] = Math.Min(p2b.Y, p2e.Y);
            extents[4 * iObje2 + 3] = Math.Max(p2b.Y, p2e.Y);
        }

        double farApart = (Math.Abs(d1) + Math.Abs(d2)) * (1 + 1e-9) + 1e-9;
        for (iObje1 = 1; iObje1 <= n1; iObje1++)
        {
            bSO.SetVoid();
            obje1.Segment(iObje1, out p1b, out p1e);
            bSO.Add(p1b);
            bSO.Add(p1e);
            bSO.Enlarge(d1);
            if (!obje2.Bounding().IsOut(bSO))
            {
                double xMin = Math.Min(p1b.X, p1e.X) - farApart, xMax = Math.Max(p1b.X, p1e.X) + farApart;
                double yMin = Math.Min(p1b.Y, p1e.Y) - farApart, yMax = Math.Max(p1b.Y, p1e.Y) + farApart;
                for (iObje2 = 1; iObje2 <= n2; iObje2++)
                {
                    int at = 4 * iObje2;
                    if (extents[at] > xMax || extents[at + 1] < xMin || extents[at + 2] > yMax || extents[at + 3] < yMin)
                    {
                        continue;
                    }

                    bST.SetVoid();
                    obje2.Segment(iObje2, out p2b, out p2e);
                    bST.Add(p2b);
                    bST.Add(p2e);
                    bST.Enlarge(d2);
                    if (!bSO.IsOut(bST))
                    {
                        Intersect(iObje1, iObje2, p1b, p1e, p2b, p2e);
                    }
                }
            }
        }

        ArrayPool<double>.Shared.Return(extents);
    }

    private void Interference(Polygon2d obje)
    {
        Box2d bSO = new();
        Box2d bST = new();

        int iObje1, iObje2, n = obje.SegmentCount;
        double d = obje.DeflectionOverEstimation();

        // Most pairs of a polygon's segments are far apart, and each one's box is only built for the exact test below: this
        // answers them from the extents of the segments alone. Its margin only ever leaves a pair to the exact test
        double[] extents = ArrayPool<double>.Shared.Rent(4 * (n + 1));
        Point2D p1b, p1e, p2b, p2e;
        for (iObje1 = 1; iObje1 <= n; iObje1++)
        {
            obje.Segment(iObje1, out p1b, out p1e);
            extents[4 * iObje1] = Math.Min(p1b.X, p1e.X);
            extents[4 * iObje1 + 1] = Math.Max(p1b.X, p1e.X);
            extents[4 * iObje1 + 2] = Math.Min(p1b.Y, p1e.Y);
            extents[4 * iObje1 + 3] = Math.Max(p1b.Y, p1e.Y);
        }

        double farApart = 2 * Math.Abs(d) * (1 + 1e-9) + 1e-9;
        for (iObje1 = 1; iObje1 <= n; iObje1++)
        {
            bSO.SetVoid();
            obje.Segment(iObje1, out p1b, out p1e);
            bSO.Add(p1b);
            bSO.Add(p1e);
            bSO.Enlarge(d);
            if (!obje.Bounding().IsOut(bSO))
            {
                double xMin = extents[4 * iObje1] - farApart, xMax = extents[4 * iObje1 + 1] + farApart;
                double yMin = extents[4 * iObje1 + 2] - farApart, yMax = extents[4 * iObje1 + 3] + farApart;
                for (iObje2 = iObje1 + 1; iObje2 <= n; iObje2++)
                {
                    int at = 4 * iObje2;
                    if (extents[at] > xMax || extents[at + 1] < xMin || extents[at + 2] > yMax || extents[at + 3] < yMin)
                    {
                        continue;
                    }

                    bST.SetVoid();
                    obje.Segment(iObje2, out p2b, out p2e);
                    bST.Add(p2b);
                    bST.Add(p2e);
                    bST.Enlarge(d);
                    if (!bSO.IsOut(bST))
                    {
                        Intersect(iObje1, iObje2, p1b, p1e, p2b, p2e);
                    }
                }
            }
        }

        ArrayPool<double>.Shared.Return(extents);
    }

    private void Clean()
    {
        // The zones of tangency that concerns only one couple of segments are
        // conserved if the angle between the segments is less than <PRCANG> and
        // if there is no real point of intersection EDGE/EDGE:
        int nbIt = _tZones.Length();
        int decal = 0;
        int addr1, addr2;
        PIType dim1, dim2;
        double par;
        int tsp, tsps;
        int lpi, ltz;
        // OCCT: set once, before the loop, so a zone keeps the value left by the previous one
        bool only1Seg = false;

        for (ltz = 1; ltz <= nbIt; ltz++)
        {
            tsp = tsps = 0;
            double pr1mi, pr1ma, pr2mi, pr2ma, delta1, delta2;
            _tZones[ltz - decal].ParamOnFirst(out pr1mi, out pr1ma);
            delta1 = pr1ma - pr1mi;
            _tZones[ltz - decal].ParamOnSecond(out pr2mi, out pr2ma);
            delta2 = pr2ma - pr2mi;
            if (delta1 < 1.0 && delta2 < 1.0)
            {
                only1Seg = true;
            }

            if (delta1 == 0.0 || delta2 == 0.0)
            {
                only1Seg = true;
            }

            for (lpi = 1; lpi <= _tZones[ltz - decal].NumberOfPoints(); lpi++)
            {
                SectionPoint pi1 = _tZones[ltz - decal].GetPoint(lpi);
                if (pi1.Incidence() <= Prcang)
                {
                    tsp = tsps = 0;
                    break;
                }

                pi1.InfoFirst(out dim1, out addr1, out par);
                pi1.InfoSecond(out dim2, out addr2, out par);
                if (dim1 == PIType.Edge && dim2 == PIType.Edge)
                {
                    tsps = 0;
                    if (tsp > 0)
                    {
                        tsp = 0;
                        only1Seg = false;
                        break;
                    }

                    tsp = lpi;
                }
                else if (dim1 != PIType.External && dim2 != PIType.External)
                {
                    tsps = lpi;
                }
            }

            if (tsp > 0)
            {
                _sPoins.Append(_tZones[ltz - decal].GetPoint(tsp));
                _tZones.Remove(ltz - decal);
                decal++;
            }
            else if (only1Seg && tsps != 0)
            {
                _sPoins.Append(_tZones[ltz - decal].GetPoint(tsps));
                _tZones.Remove(ltz - decal);
                decal++;
            }
        }

        // The points of intersection located in the tangency zone are
        // removed from the list :
        nbIt = _sPoins.Length();
        decal = 0;

        for (lpi = 1; lpi <= nbIt; lpi++)
        {
            for (ltz = 1; ltz <= _tZones.Length(); ltz++)
            {
                if (_tZones[ltz].RangeContains(_sPoins[lpi - decal]))
                {
                    _sPoins.Remove(lpi - decal);
                    decal++;
                    break;
                }
            }
        }
    }

    /// <summary>Computes the intersection between the two segments <paramref name="begO"/> <paramref name="endO"/> and <paramref name="begT"/> <paramref name="endT"/>.</summary>
    private void Intersect(int iObje1, int iObje2, in Point2D begO, in Point2D endO, in Point2D begT,
                           in Point2D endT)
    {
        if (_selfIntf)
        {
            if (Math.Abs(iObje1 - iObje2) <= 1)
            {
                return; //-- Ajout du 15 jan 98
            }
        }

        int nbpi = 0;
        Span<double> parO = stackalloc double[8];
        Span<double> parT = stackalloc double[8];
        NCollection_Sequence<SectionPoint> thePi = _sectionWork ??= new();
        thePi.Clear();
        Xy segT = endT.XY() - begT.XY();
        Xy segO = endO.XY() - begO.XY();

        // If the length of segment is zero, nothing is done
        double lgT = Math.Sqrt(segT * segT);
        if (lgT <= 0.0)
        {
            return;
        }

        double lgO = Math.Sqrt(segO * segO);
        if (lgO <= 0.0)
        {
            return;
        }

        // Direction of parsing of segments
        double sigPS = (segO * segT) > 0.0 ? 1.0 : -1.0;

        // Precision of calculation
        double floatgap = Epsilon(lgO + lgT);

        // Angle between two straight lines and radius of interference
        double sinTeta = (segO.CrossMagnitude(segT) / lgO) / lgT;
        double rayIntf = 0.0;
        if (sinTeta > 0.0)
        {
            rayIntf = _tolerance / sinTeta;
        }

        // Interference <begO> <segT>
        double dbOT = ((begO.XY() - begT.XY()) ^ segT) / lgT;
        double dbObT = begO.DistanceTo(begT);
        double dbOeT = begO.DistanceTo(endT);
        if (Math.Abs(dbOT) <= _tolerance)
        {
            if (dbObT <= _tolerance)
            {
                nbpi++;
                parO[nbpi] = 0.0;
                parT[nbpi] = 0.0;
                thePi.Append(
                    new SectionPoint(begO, PIType.Vertex, iObje1, 0.0, PIType.Vertex, iObje2, 0.0, sinTeta));
            }

            if (dbOeT <= _tolerance)
            {
                nbpi++;
                parO[nbpi] = 0.0;
                parT[nbpi] = 1.0;
                thePi.Append(
                    new SectionPoint(begO, PIType.Vertex, iObje1, 0.0, PIType.Vertex, iObje2 + 1, 0.0, sinTeta));
            }

            if (dbObT > _tolerance && dbOeT > _tolerance && dbObT + dbOeT <= (lgT + _tolerance))
            {
                nbpi++;
                parO[nbpi] = 0.0;
                parT[nbpi] = dbObT / lgT;
                thePi.Append(
                    new SectionPoint(begO, PIType.Vertex, iObje1, 0.0, PIType.Edge, iObje2, parT[nbpi], sinTeta));
            }
        }

        // Interference <endO> <segT>
        double deOT = ((endO.XY() - begT.XY()) ^ segT) / lgT;
        double deObT = endO.DistanceTo(begT);
        double deOeT = endO.DistanceTo(endT);
        if (Math.Abs(deOT) <= _tolerance)
        {
            if (deObT <= _tolerance)
            {
                nbpi++;
                parO[nbpi] = 1.0;
                parT[nbpi] = 0.0;
                thePi.Append(
                    new SectionPoint(endO, PIType.Vertex, iObje1 + 1, 0.0, PIType.Vertex, iObje2, 0.0, sinTeta));
            }

            if (deOeT <= _tolerance)
            {
                nbpi++;
                parO[nbpi] = 1.0;
                parT[nbpi] = 1.0;
                thePi.Append(
                    new SectionPoint(endO, PIType.Vertex, iObje1 + 1, 0.0, PIType.Vertex, iObje2 + 1, 0.0, sinTeta));
            }

            if (deObT > _tolerance && deOeT > _tolerance && deObT + deOeT <= (lgT + _tolerance))
            {
                nbpi++;
                parO[nbpi] = 1.0;
                parT[nbpi] = deObT / lgT;
                thePi.Append(new SectionPoint(endO,
                                                   PIType.Vertex,
                                                   iObje1 + 1,
                                                   0.0,
                                                   PIType.Edge,
                                                   iObje2,
                                                   parT[nbpi],
                                                   sinTeta));
            }
        }

        // Interference <begT> <segO>
        double dbTO = ((begT.XY() - begO.XY()) ^ segO) / lgO;
        if (Math.Abs(dbTO) <= _tolerance)
        {
            if (dbObT > _tolerance && deObT > _tolerance && dbObT + deObT <= (lgO + _tolerance))
            {
                nbpi++;
                parO[nbpi] = dbObT / lgO;
                parT[nbpi] = 0.0;
                thePi.Append(
                    new SectionPoint(begT, PIType.Edge, iObje1, parO[nbpi], PIType.Vertex, iObje2, 0.0, sinTeta));
            }
        }

        // Interference <endT> <segO>
        double deTO = ((endT.XY() - begO.XY()) ^ segO) / lgO;
        if (Math.Abs(deTO) <= _tolerance)
        {
            if (dbOeT > _tolerance && deOeT > _tolerance && dbOeT + deOeT <= (lgO + _tolerance))
            {
                nbpi++;
                parO[nbpi] = dbOeT / lgO;
                parT[nbpi] = 1.0;
                thePi.Append(new SectionPoint(endT,
                                                   PIType.Edge,
                                                   iObje1,
                                                   parO[nbpi],
                                                   PIType.Vertex,
                                                   iObje2 + 1,
                                                   0.0,
                                                   sinTeta));
            }
        }

        bool edgeSP = false;
        double parOSP = 0, parTSP = 0;

        if (Math.Abs(dbOT - deOT) > floatgap && Math.Abs(dbTO - deTO) > floatgap)
        {
            parOSP = dbOT / (dbOT - deOT);
            parTSP = dbTO / (dbTO - deTO);
            if (dbOT * deOT <= 0.0 && dbTO * deTO <= 0.0)
            {
                edgeSP = true;
            }
            else if (nbpi == 0)
            {
                return;
            }

            // If there is no interference it is necessary to take the points segment by segment
            if (nbpi == 0 && sinTeta > Prcang)
            {
                nbpi++;
                parO[nbpi] = parOSP;
                parT[nbpi] = parTSP;
                thePi.Append(
                    new SectionPoint(new Point2D(begO.X + (segO.X() * parOSP), begO.Y + (segO.Y() * parOSP)),
                                          PIType.Edge,
                                          iObje1,
                                          parOSP,
                                          PIType.Edge,
                                          iObje2,
                                          parTSP,
                                          sinTeta));
            }

            // Otherwise it is required to check if there is no other
            else if (rayIntf >= _tolerance)
            {
                double deltaO = rayIntf / lgO;
                double deltaT = rayIntf / lgT;
                double x, y;
                double parOdeb = parOSP - deltaO;
                double parOfin = parOSP + deltaO;
                double parTdeb = parTSP - sigPS * deltaT;
                double parTfin = parTSP + sigPS * deltaT;
                if (nbpi == 0)
                {
                    parO[1] = parOdeb;
                    parO[2] = parOfin;
                    parT[1] = parTdeb;
                    parT[2] = parTfin;
                    while (nbpi < 2)
                    {
                        nbpi++;
                        x = begO.X + (segO.X() * parO[nbpi]);
                        y = begO.Y + (segO.Y() * parO[nbpi]);
                        thePi.Append(new SectionPoint(new Point2D(x, y),
                                                           PIType.External,
                                                           iObje1,
                                                           parO[nbpi],
                                                           PIType.External,
                                                           iObje2,
                                                           parT[nbpi],
                                                           sinTeta));
                    }
                }
                else
                { // nbpi>0
                    if (nbpi == 1)
                    {
                        bool ok = true;
                        if (0.0 < parOdeb && parOdeb < 1.0 && 0.0 < parTdeb && parTdeb < 1.0)
                        {
                            parO[nbpi + 1] = parOdeb;
                            parT[nbpi + 1] = parTdeb;
                        }
                        else if (0.0 < parOfin && parOfin < 1.0 && 0.0 < parTfin && parTfin < 1.0)
                        {
                            parO[nbpi + 1] = parOfin;
                            parT[nbpi + 1] = parTfin;
                        }
                        else
                        {
                            ok = false;
                        }

                        if (ok)
                        {
                            x = begO.X + (segO.X() * parO[nbpi + 1]);
                            y = begO.Y + (segO.Y() * parO[nbpi + 1]);
                            if (thePi[1].Pnt().DistanceTo(new Point3D(x, y, 0)) >= (_tolerance / 4.0))
                            {
                                nbpi++;
                                thePi.Append(new SectionPoint(new Point2D(x, y),
                                                                   PIType.External,
                                                                   iObje1,
                                                                   parO[nbpi],
                                                                   PIType.External,
                                                                   iObje2,
                                                                   parT[nbpi],
                                                                   sinTeta));
                            }
                        }
                    }
                    else
                    { // plus d une singularite
                        double parOmin = parO[1];
                        double parOmax = parO[1];
                        double parTmin = parT[1];
                        double parTmax = parT[1];
                        for (int i = 2; i <= nbpi; i++)
                        {
                            parOmin = Min(parOmin, parO[i]);
                            parOmax = Max(parOmax, parO[i]);
                            parTmin = Min(parTmin, parT[i]);
                            parTmax = Max(parTmax, parT[i]);
                        }

                        double delta;
                        if (parOdeb < 0.0)
                        {
                            delta = -parOdeb;
                            parOdeb = 0.0;
                            parTdeb = parTdeb + sigPS * (delta * (deltaT / deltaO));
                        }

                        if (parOfin > 1.0)
                        {
                            delta = parOfin - 1.0;
                            parOfin = 1.0;
                            parTfin = parTfin - sigPS * (delta * (deltaT / deltaO));
                        }

                        if (sigPS > 0.0)
                        {
                            if (parTdeb < 0.0)
                            {
                                delta = -parTdeb;
                                parTdeb = 0.0;
                                parOdeb = parOdeb + delta * (deltaO / deltaT);
                            }

                            if (parTfin > 1.0)
                            {
                                delta = parTfin - 1.0;
                                parTfin = 1.0;
                                parOfin = parOfin - delta * (deltaO / deltaT);
                            }
                        }
                        else
                        {
                            if (parTdeb > 1.0)
                            {
                                delta = parTdeb - 1.0;
                                parTdeb = 1.0;
                                parOdeb = parOdeb + delta * (deltaO / deltaT);
                            }

                            if (parTfin < 0.0)
                            {
                                delta = -parTfin;
                                parTfin = 0.0;
                                parOfin = parOfin - delta * (deltaO / deltaT);
                            }
                        }

                        if ((parOdeb < parOmin && parOmin > 0.0)
                            || (sigPS > 0.0 && parTdeb < parTmin && parTmin > 0.0)
                            || (sigPS < 0.0 && parTdeb > parTmax && parTmax < 1.0))
                        {
                            nbpi++;
                            parO[nbpi] = Max(0.0, Min(1.0, parOdeb));
                            parT[nbpi] = Max(0.0, Min(1.0, parTdeb));
                            x = begO.X + (segO.X() * parO[nbpi]);
                            y = begO.Y + (segO.Y() * parO[nbpi]);
                            thePi.Append(new SectionPoint(new Point2D(x, y),
                                                               PIType.External,
                                                               iObje1,
                                                               parO[nbpi],
                                                               PIType.External,
                                                               iObje2,
                                                               parT[nbpi],
                                                               sinTeta));
                        }

                        if ((parOfin > parOmax && parOmax < 1.0)
                            || (sigPS < 0.0 && parTfin < parTmin && parTmin > 0.0)
                            || (sigPS > 0.0 && parTfin > parTmax && parTmax < 1.0))
                        {
                            nbpi++;
                            parO[nbpi] = Min(1.0, Max(0.0, parOfin));
                            parT[nbpi] = Min(1.0, Max(0.0, parTfin));
                            x = begO.X + (segO.X() * parO[nbpi]);
                            y = begO.Y + (segO.Y() * parO[nbpi]);
                            thePi.Append(new SectionPoint(new Point2D(x, y),
                                                               PIType.External,
                                                               iObje1,
                                                               parO[nbpi],
                                                               PIType.External,
                                                               iObje2,
                                                               parT[nbpi],
                                                               sinTeta));
                        }
                    }
                }
            }
        }

        //-- lbr : The points too close to each other are suspended
        bool suppr;
        do
        {
            suppr = false;
            for (int i = 2; !suppr && i <= nbpi; i++)
            {
                Point3D pim1 = thePi[i - 1].Pnt();
                Point3D pi = thePi[i].Pnt();
                double d = pi.DistanceTo(pim1);
                d *= 50.0;
                if (d < lgT && d < lgO)
                {
                    for (int j = i; j < nbpi; j++)
                    {
                        thePi[j] = thePi[j + 1];
                    }

                    nbpi--;
                    suppr = true;
                }
            }
        } while (suppr);

        if (nbpi == 1)
        {
            if (edgeSP)
            {
                thePi[1] =
                    new SectionPoint(new Point2D(begO.X + (segO.X() * parOSP), begO.Y + (segO.Y() * parOSP)),
                                          PIType.Edge,
                                          iObje1,
                                          parOSP,
                                          PIType.Edge,
                                          iObje2,
                                          parTSP,
                                          sinTeta);
                parO[1] = parOSP;
                parT[1] = parTSP;
            }

            if (!_selfIntf)
            {
                bool contains = false;
                for (int i = 1; i <= _sPoins.Length(); i++)
                {
                    if (thePi[1].IsEqual(_sPoins[i]))
                    {
                        contains = true;
                        break;
                    }
                }

                if (!contains)
                {
                    _sPoins.Append(thePi[1]);
                }
            }
            else if (iObje2 - iObje1 != 1 && (!_oClos || (iObje1 != 1 && iObje2 != _nbso)))
            {
                _sPoins.Append(thePi[1]);
            }
        }
        else if (nbpi >= 2)
        {
            TangentZone theTZ = new();
            if (nbpi == 2)
            {
                theTZ.PolygonInsert(thePi[1]);
                theTZ.PolygonInsert(thePi[2]);
            }
            else
            {
                int lpj;
                int lmin = 1;
                int lmax = 1;
                for (lpj = 2; lpj <= nbpi; lpj++)
                {
                    if (parO[lpj] < parO[lmin])
                    {
                        lmin = lpj;
                    }
                    else if (parO[lpj] > parO[lmax])
                    {
                        lmax = lpj;
                    }
                }

                theTZ.PolygonInsert(thePi[lmin]);
                theTZ.PolygonInsert(thePi[lmax]);

                int ltmin = 1;
                int ltmax = 1;
                for (lpj = 2; lpj <= nbpi; lpj++)
                {
                    if (parT[lpj] < parT[ltmin])
                    {
                        ltmin = lpj;
                    }
                    else if (parT[lpj] > parT[ltmax])
                    {
                        ltmax = lpj;
                    }
                }

                if (ltmin != lmin && ltmin != lmax)
                {
                    theTZ.PolygonInsert(thePi[ltmin]);
                }

                if (ltmax != lmin && ltmax != lmax)
                {
                    theTZ.PolygonInsert(thePi[ltmax]);
                }
            }

            if (edgeSP)
            {
                theTZ.PolygonInsert(
                    new SectionPoint(new Point2D(begO.X + (segO.X() * parOSP), begO.Y + (segO.Y() * parOSP)),
                                          PIType.Edge,
                                          iObje1,
                                          parOSP,
                                          PIType.Edge,
                                          iObje2,
                                          parTSP,
                                          sinTeta));
            }

            int nbtz = _tZones.Length();
            NCollection_List<int> lIndex = new();
            for (int ltz = 1; ltz <= nbtz; ltz++)
            {
                if (theTZ.HasCommonRange(_tZones[ltz]))
                {
                    lIndex.Append(ltz);
                }
            }

            //------------------------------------------------------------------------
            //--   The list is parsed in ascending order by index, zone and tg
            //--
            if (lIndex.IsEmpty())
            {
                _tZones.Append(theTZ);
            }
            else
            {
                int indexfirst = lIndex.First();
                lIndex.RemoveFirst();
                int decal = 0;
                _tZones[indexfirst].Append(theTZ);
                while (!lIndex.IsEmpty())
                {
                    int index = lIndex.First();
                    lIndex.RemoveFirst();
                    _tZones[indexfirst].Append(_tZones[index - decal]);
                    _tZones.Remove(index - decal);
                    decal++;
                }
            }
        }
    }
}
