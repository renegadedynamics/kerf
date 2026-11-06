// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Hatcher.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Hatcher.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Hatcher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Classification.Hatching;

/// <summary>
/// Trims 2D curves, the hatchings, by a boundary made of 2D curves, the elements: computes the points where each
/// hatching meets the elements (<c>Trim</c>), then the domains of each hatching inside the boundary
/// (<c>ComputeDomains</c>).
/// </summary>
/// <remarks>
/// A working object (decisions 21 and 666): OCCT's API, with the elements and hatchings in maps indexed from 1 that
/// iterate in OCCT's order (<see cref="NCollection_IntDataMap{TValue}"/>). The intersector given to the constructor
/// is copied, as OCCT copies it. Results (<see cref="Point"/>, <see cref="Domain"/>) are the hatcher's own points and
/// domains, which callers can't change. OCCT's <c>IsDone()</c> without an index is declared but never defined: left
/// out.
/// </remarks>
internal class Hatcher2D
{
    private readonly Intersector2D _intersector;
    private double _confusion2d;
    private double _confusion3d;
    private bool _keepPoints;
    private bool _keepSegments;
    private int _nbElements;
    private readonly Elements2D _elements = new();
    private int _nbHatchings;
    private readonly NCollection_IntDataMap<Hatching2D> _hatchings = new();

    //=======================================================================
    //  Category : General use.
    //=======================================================================

    /// <summary>
    /// An empty hatcher with (a copy of) the intersector <paramref name="intersector"/>, the 2D and 3D confusion
    /// tolerances, and whether the domains keep isolated points (<paramref name="keepPnt"/>) and segments
    /// (<paramref name="keepSeg"/>).
    /// </summary>
    public Hatcher2D(Intersector2D intersector,
                               double confusion2d,
                               double confusion3d,
                               bool keepPnt = false,
                               bool keepSeg = false)
    {
        _intersector = new Intersector2D(intersector);
        _confusion2d = confusion2d;
        _confusion3d = confusion3d;
        _keepPoints = keepPnt;
        _keepSegments = keepSeg;
        _nbElements = 0;
        _nbHatchings = 0;
    }

    /// <summary>Sets the intersector (a copy of <paramref name="intersector"/>); the trimmings are cleared.</summary>
    public void Intersector(Intersector2D intersector)
    {
        _intersector.Assign(intersector);
        for (int indH = 1; indH <= _nbHatchings; indH++)
        {
            if (_hatchings.IsBound(indH))
            {
                Hatching2D hatching = _hatchings.ChangeFind(indH);
                hatching.ClrPoints();
            }
        }
    }

    /// <summary>The intersector of the hatcher.</summary>
    public Intersector2D Intersector() => _intersector;

    /// <summary>The intersector of the hatcher, for changing its tolerances.</summary>
    public Intersector2D ChangeIntersector() => _intersector;

    /// <summary>Sets the 2D confusion tolerance; the trimmings are cleared.</summary>
    public void Confusion2d(double confusion)
    {
        _confusion2d = confusion;
        for (int indH = 1; indH <= _nbHatchings; indH++)
        {
            if (_hatchings.IsBound(indH))
            {
                Hatching2D hatching = _hatchings.ChangeFind(indH);
                hatching.ClrPoints();
            }
        }
    }

    /// <summary>The 2D confusion tolerance.</summary>
    public double Confusion2d() => _confusion2d;

    /// <summary>Sets the 3D confusion tolerance; the trimmings are cleared.</summary>
    public void Confusion3d(double confusion)
    {
        _confusion3d = confusion;
        for (int indH = 1; indH <= _nbHatchings; indH++)
        {
            if (_hatchings.IsBound(indH))
            {
                Hatching2D hatching = _hatchings.ChangeFind(indH);
                hatching.ClrPoints();
            }
        }
    }

    /// <summary>The 3D confusion tolerance.</summary>
    public double Confusion3d() => _confusion3d;

    /// <summary>Sets whether the domains keep isolated points; the domains are cleared.</summary>
    public void KeepPoints(bool keep)
    {
        _keepPoints = keep;
        for (int indH = 1; indH <= _nbHatchings; indH++)
        {
            if (_hatchings.IsBound(indH))
            {
                Hatching2D hatching = _hatchings.ChangeFind(indH);
                hatching.ClrDomains();
            }
        }
    }

    /// <summary>True if the domains keep isolated points.</summary>
    public bool KeepPoints() => _keepPoints;

    /// <summary>Sets whether the domains keep segments; the domains are cleared.</summary>
    public void KeepSegments(bool keep)
    {
        _keepSegments = keep;
        for (int indH = 1; indH <= _nbHatchings; indH++)
        {
            if (_hatchings.IsBound(indH))
            {
                Hatching2D hatching = _hatchings.ChangeFind(indH);
                hatching.ClrDomains();
            }
        }
    }

    /// <summary>True if the domains keep segments.</summary>
    public bool KeepSegments() => _keepSegments;

    /// <summary>Removes all the hatchings and all the elements.</summary>
    public void Clear()
    {
        if (_nbHatchings != 0)
        {
            ClrHatchings();
        }
        if (_nbElements != 0)
        {
            ClrElements();
        }
    }

    //=======================================================================
    //  Category : Element.
    //=======================================================================

    /// <summary>The <paramref name="indE"/>-th element, for writing.</summary>
    protected ref Element2D Element(int indE)
    {
        ref Element2D element = ref _elements.ChangeFind(indE);
        return ref element;
    }

    /// <summary>The curve of the <paramref name="indE"/>-th element.</summary>
    public CurveAdaptor2D ElementCurve(int indE)
    {
        Element2D element = _elements.Find(indE);
        return element.Curve();
    }

    /// <summary>
    /// Adds an element (a copy of <paramref name="curve"/>, with its orientation) and returns its index: the lowest
    /// free one. The trimmings are cleared.
    /// </summary>
    public int AddElement(CurveAdaptor2D curve, Orientation orientation = Forward)
    {
        int indE;
        for (indE = 1; indE <= _nbElements && _elements.IsBound(indE); indE++)
        {
        }
        if (indE > _nbElements)
        {
            _nbElements++;
            indE = _nbElements;
        }
        Element2D element = new(curve, orientation);
        _elements.Bind(indE, element);
        for (int indH = 1; indH <= _nbHatchings; indH++)
        {
            if (_hatchings.IsBound(indH))
            {
                Hatching2D hatching = _hatchings.ChangeFind(indH);
                hatching.ClrPoints();
            }
        }
        return indE;
    }

    /// <summary>Adds an element on the curve <paramref name="curve"/>, with its orientation, and returns its index.</summary>
    public int AddElement(Curve2D curve, Orientation orientation = Forward)
    {
        CurveAdaptor2D gac = new(curve);
        return AddElement(gac, orientation);
    }

    /// <summary>Removes the <paramref name="indE"/>-th element.</summary>
    public void RemElement(int indE)
    {
        for (int indH = 1; indH <= _nbHatchings; indH++)
        {
            if (_hatchings.IsBound(indH))
            {
                Hatching2D hatching = _hatchings.ChangeFind(indH);
                bool domainsToClear = false;
                for (int iPntH = hatching.PointCount; iPntH > 0; iPntH--)
                {
                    // OCCT copies the point, so the removals below don't reach the hatching's points
                    // (occt-issues.md): only a point left with no point on element is removed.
                    PointOnHatching pntH = new(hatching.ChangePoint(iPntH));
                    for (int iPntE = pntH.PointCount; iPntE > 0; iPntE--)
                    {
                        if (pntH.Point(iPntE).Index() == indE)
                        {
                            pntH.RemPoint(iPntE);
                            domainsToClear = true;
                        }
                    }
                    if (pntH.PointCount == 0)
                    {
                        hatching.RemPoint(iPntH);
                    }
                }
                if (domainsToClear)
                {
                    hatching.ClrDomains();
                }
            }
        }
        _elements.UnBind(indE);
        if (indE == _nbElements)
        {
            _nbElements--;
        }
    }

    /// <summary>Removes all the elements; the trimmings are cleared.</summary>
    public void ClrElements()
    {
        if (_nbElements != 0)
        {
            if (_nbHatchings != 0)
            {
                for (int indH = 1; indH <= _nbHatchings; indH++)
                {
                    if (_hatchings.IsBound(indH))
                    {
                        Hatching2D hatching = _hatchings.ChangeFind(indH);
                        hatching.ClrPoints();
                    }
                }
            }
            _elements.Clear();
            _nbElements = 0;
        }
    }

    //=======================================================================
    //  Category : Hatching.
    //=======================================================================

    /// <summary>The <paramref name="indH"/>-th hatching.</summary>
    protected Hatching2D Hatching(int indH)
    {
        Hatching2D hatching = _hatchings.ChangeFind(indH);
        return hatching;
    }

    /// <summary>The curve of the <paramref name="indH"/>-th hatching.</summary>
    public CurveAdaptor2D HatchingCurve(int indH)
    {
        Hatching2D hatching = _hatchings.Find(indH);
        return hatching.Curve();
    }

    /// <summary>Adds a hatching on (a copy of) <paramref name="curve"/> and returns its index: the lowest free one.</summary>
    public int AddHatching(CurveAdaptor2D curve)
    {
        int indH;
        for (indH = 1; indH <= _nbHatchings && _hatchings.IsBound(indH); indH++)
        {
        }
        if (indH > _nbHatchings)
        {
            _nbHatchings++;
            indH = _nbHatchings;
        }
        Hatching2D hatching = new(curve);
        _hatchings.Bind(indH, hatching);
        return indH;
    }

    /// <summary>Removes the <paramref name="indH"/>-th hatching.</summary>
    public void RemHatching(int indH)
    {
        Hatching2D hatching = _hatchings.ChangeFind(indH);
        hatching.ClrPoints();
        _hatchings.UnBind(indH);
        if (indH == _nbHatchings)
        {
            _nbHatchings--;
        }
    }

    /// <summary>Removes all the hatchings.</summary>
    public void ClrHatchings()
    {
        if (_nbHatchings != 0)
        {
            for (int indH = 1; indH <= _nbHatchings; indH++)
            {
                if (_hatchings.IsBound(indH))
                {
                    Hatching2D hatching = _hatchings.ChangeFind(indH);
                    hatching.ClrPoints();
                }
            }
            _hatchings.Clear();
            _nbHatchings = 0;
        }
    }

    /// <summary>The number of intersection points of the <paramref name="indH"/>-th hatching.</summary>
    public int NbPoints(int indH)
    {
        Hatching2D hatching = _hatchings.Find(indH);
        return hatching.PointCount;
    }

    /// <summary>The <paramref name="indP"/>-th intersection point of the <paramref name="indH"/>-th hatching.</summary>
    public PointOnHatching Point(int indH, int indP)
    {
        Hatching2D hatching = _hatchings.Find(indH);
        PointOnHatching pntH = hatching.Point(indP);
        return pntH;
    }

    //=======================================================================
    //  Category : Computation - Trimming
    //=======================================================================

    /// <summary>Trims all the hatchings by all the elements.</summary>
    public void Trim()
    {
        for (int indH = 1; indH <= _nbHatchings; indH++)
        {
            if (_hatchings.IsBound(indH))
            {
                Trim(indH);
            }
        }
    }

    /// <summary>Adds a hatching on <paramref name="curve"/>, trims it by the elements and returns its index.</summary>
    public int Trim(CurveAdaptor2D curve)
    {
        int indH = AddHatching(curve);
        Trim(indH);
        return indH;
    }

    /// <summary>Trims the <paramref name="indH"/>-th hatching by the elements, then computes its points' transitions.</summary>
    public void Trim(int indH)
    {
        Hatching2D hatching = _hatchings.ChangeFind(indH);

        hatching.ClrPoints();

        bool ok, allOK;

        allOK = true;
        for (int indE = 1; indE <= _nbElements; indE++)
        {
            if (_elements.IsBound(indE))
            {
                ok = Trim(indH, indE);
                allOK = allOK && ok;
            }
        }
        hatching.TrimDone(true);
        hatching.TrimFailed(!allOK);

        if (allOK)
        {
            for (int iPnt = 1; iPnt <= hatching.PointCount; iPnt++)
            {
                PointOnHatching pntH = hatching.ChangePoint(iPnt);
                ok = GlobalTransition(pntH);
                allOK = allOK && ok;
            }
            hatching.Status(allOK ? HatchingErrorStatus.NoProblem : HatchingErrorStatus.TransitionFailure);
        }
    }

    // Trims the IndH-th hatching by the IndE-th element.
    private bool Trim(int indH, int indE)
    {
        Hatching2D Hatching = _hatchings.ChangeFind(indH);
        ref Element2D Element = ref _elements.ChangeFind(indE);

        // OCCT intersects copies of the two adaptors, which share their curves and evaluation caches with the
        // originals; the port passes the adaptors themselves (decision 666)
        CurveAdaptor2D hatching = Hatching.ChangeCurve();
        CurveAdaptor2D element = Element.ChangeCurve();

        _intersector.Intersect(hatching, element);

        if (!_intersector.IsDone)
        {
            // OCCT also writes " Intersector -> Done = False " to std::cout (left out, occt-issues.md)
            return false;
        }

        if (_intersector.IsEmpty)
        {
            return true;
        }

        //-----------------------------------------------------------------------
        // Traitement des points d intersection.
        //-----------------------------------------------------------------------

        for (int iPntI = 1; iPntI <= _intersector.PointCount; iPntI++)
        {
            IntersectionPoint2D pntI = _intersector.Point(iPntI);

            PointOnElement pntE = new(pntI);
            pntE.SetIndex(indE);

            PointOnHatching pntH = new(pntI);
            pntH.SetIndex(indH);
            pntH.AddPoint(pntE, _confusion2d);

            Hatching.AddPoint(pntH, _confusion2d);
        }

        //-----------------------------------------------------------------------
        // Traitement des segments d intersection.
        //-----------------------------------------------------------------------

        for (int iSeg = 1; iSeg <= _intersector.SegmentCount; iSeg++)
        {
            IntersectionSegment2D seg = _intersector.Segment(iSeg);

            bool firstPoint = seg.HasFirstPoint;
            bool lastPoint = seg.HasLastPoint;

            //-----------------------------------------------------------------------
            // Les deux points peuvent etre confondus.
            //-----------------------------------------------------------------------

            if (firstPoint && lastPoint)
            {
                IntersectionPoint2D pnt1 = seg.FirstPoint();
                IntersectionPoint2D pnt2 = seg.LastPoint();

                Transition trsPnt1H = pnt1.TransitionOfFirst();
                Transition trsPnt1E = pnt1.TransitionOfSecond();
                Transition trsPnt2H = pnt2.TransitionOfFirst();
                Transition trsPnt2E = pnt2.TransitionOfSecond();

                TypeTrans typePnt1H = trsPnt1H.TransitionType();
                TypeTrans typePnt1E = trsPnt1E.TransitionType();
                TypeTrans typePnt2H = trsPnt2H.TransitionType();
                TypeTrans typePnt2E = trsPnt2E.TransitionType();

                //-----------------------------------------------------------------------
                // Les deux points peuvent etre confondus au regard de la precision du
                // `hatcher'.
                //-----------------------------------------------------------------------

                bool conf2d = Math.Abs(pnt1.ParamOnFirst() - pnt2.ParamOnFirst()) <= _confusion2d;

                //-----------------------------------------------------------------------
                // Les deux points peuvent etre `confondus' au regard des intersections.
                //-----------------------------------------------------------------------

                bool conf3d = false;

                if (!conf2d)
                {
                    conf3d = true;
                    if (conf3d)
                    {
                        conf3d = typePnt1H != TypeTrans.Touch && typePnt1H != TypeTrans.Undecided;
                    }
                    if (conf3d)
                    {
                        conf3d = typePnt1E != TypeTrans.Touch && typePnt1E != TypeTrans.Undecided;
                    }
                    if (conf3d)
                    {
                        conf3d = typePnt2H != TypeTrans.Touch && typePnt2H != TypeTrans.Undecided;
                    }
                    if (conf3d)
                    {
                        conf3d = typePnt2E != TypeTrans.Touch && typePnt2E != TypeTrans.Undecided;
                    }
                    if (conf3d)
                    {
                        conf3d = typePnt1H == typePnt2H && typePnt1E == typePnt2E;
                    }
                    if (conf3d)
                    {
                        conf3d = pnt1.Value().DistanceTo(pnt2.Value()) <= _confusion3d;
                    }
                }

                if (conf2d || conf3d)
                {
                    PointOnElement pntE = new();
                    pntE.SetIndex(indE);
                    pntE.SetParameter((pnt1.ParamOnSecond() + pnt2.ParamOnSecond()) / 2.0);
                    switch (trsPnt1E.PositionOnCurve())
                    {
                        case Position2D.Head:
                        {
                            pntE.SetPosition(Forward);
                            break;
                        }
                        case Position2D.Middle:
                        {
                            switch (trsPnt2E.PositionOnCurve())
                            {
                                case Position2D.Head:
                                {
                                    pntE.SetPosition(Forward);
                                    break;
                                }
                                case Position2D.Middle:
                                {
                                    pntE.SetPosition(Internal);
                                    break;
                                }
                                case Position2D.End:
                                {
                                    pntE.SetPosition(Reversed);
                                    break;
                                }
                                default:
                                {
                                    break;
                                }
                            }
                            break;
                        }
                        case Position2D.End:
                        {
                            pntE.SetPosition(Reversed);
                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }
                    pntE.SetIntersectionType((pntE.Position() == Internal) ? IntersectionType.True : IntersectionType.Touch);
                    pntE.SetStateBefore((typePnt1H == TypeTrans.In) ? global::Kerf.Topology.Containment.Out : global::Kerf.Topology.Containment.In);
                    pntE.SetStateAfter((typePnt2H == TypeTrans.In) ? global::Kerf.Topology.Containment.Out : global::Kerf.Topology.Containment.In);

                    PointOnHatching pntH = new();
                    pntH.SetIndex(indH);
                    pntH.SetParameter((pnt1.ParamOnFirst() + pnt2.ParamOnFirst()) / 2.0);
                    switch (trsPnt1H.PositionOnCurve())
                    {
                        case Position2D.Head:
                        {
                            pntH.SetPosition(Forward);
                            break;
                        }
                        case Position2D.Middle:
                        {
                            switch (trsPnt2H.PositionOnCurve())
                            {
                                case Position2D.Head:
                                {
                                    pntH.SetPosition(Forward);
                                    break;
                                }
                                case Position2D.Middle:
                                {
                                    pntH.SetPosition(Internal);
                                    break;
                                }
                                case Position2D.End:
                                {
                                    pntH.SetPosition(Reversed);
                                    break;
                                }
                                default:
                                {
                                    break;
                                }
                            }
                            break;
                        }
                        case Position2D.End:
                        {
                            pntH.SetPosition(Reversed);
                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }

                    pntH.AddPoint(pntE, _confusion2d);
                    Hatching.AddPoint(pntH, _confusion2d);

                    continue;
                }

                //-----------------------------------------------------------------------
                // Traitement du premier point du segment.
                //-----------------------------------------------------------------------

                if (firstPoint)
                {
                    IntersectionPoint2D pntI = seg.FirstPoint();

                    PointOnElement pntE = new(pntI);
                    pntE.SetIndex(indE);
                    pntE.SetSegmentBeginning(true);
                    pntE.SetSegmentEnd(false);

                    PointOnHatching pntH = new(pntI);
                    pntH.SetIndex(indH);
                    pntH.AddPoint(pntE, _confusion2d);

                    Hatching.AddPoint(pntH, _confusion2d);
                }

                //-----------------------------------------------------------------------
                // Traitement du deuxieme point du segment.
                //-----------------------------------------------------------------------

                if (lastPoint)
                {
                    IntersectionPoint2D pntI = seg.LastPoint();

                    PointOnElement pntE = new(pntI);
                    pntE.SetIndex(indE);
                    pntE.SetSegmentBeginning(false);
                    pntE.SetSegmentEnd(true);

                    PointOnHatching pntH = new(pntI);
                    pntH.SetIndex(indH);
                    pntH.AddPoint(pntE, _confusion2d);

                    Hatching.AddPoint(pntH, _confusion2d);
                }
            }
        }
        return true;
    }

    //=======================================================================
    //  Category : Computation - Domains
    //=======================================================================

    // The states before and after the complex transition at an intersection point of a hatching, from the local
    // geometry of the hatching and of each element there.
    private bool GlobalTransition(PointOnHatching point)
    {
        Containment stateBefore = Unknown;
        Containment stateAfter = Unknown;
        bool segmentBegin = false;
        bool segmentEnd = false;

        Direction2D tangente2d, normale2d;
        Direction3D tangente = new(), normale = new();
        double courbure;

        CurveAdaptor2D curveH = HatchingCurve(point.Index());

        // OCCT passes CurveH.Curve(), converted to a new adaptor on the whole curve; LocalGeometry only reads the
        // adaptor's Curve(), which is the same
        _intersector.LocalGeometry(curveH, point.Parameter(), out tangente2d, out normale2d, out courbure);

        tangente = tangente.WithCoord(tangente2d.X, tangente2d.Y, 0.0);
        if (courbure < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            normale = normale.WithCoord(-tangente2d.Y, tangente2d.X, 0.0);
        }
        else
        {
            normale = normale.WithCoord(normale2d.X, normale2d.Y, 0.0);
        }

        CurveTransition complexTransition = new();
        complexTransition.Reset(tangente, normale, courbure);

        for (int iPntE = 1; iPntE <= point.PointCount; iPntE++)
        {
            PointOnElement pntE = point.Point(iPntE);

            segmentBegin = segmentBegin || pntE.SegmentBeginning();
            segmentEnd = segmentEnd || pntE.SegmentEnd();

            Element2D element = _elements.Find(pntE.Index());
            CurveAdaptor2D curveE = element.Curve();

            Orientation elementOrientation = element.Orientation();
            bool toReverse = (elementOrientation == Reversed);
            double param;
            switch (pntE.Position())
            {
                case Forward:
                    param = toReverse ? curveE.LastParameter() : curveE.FirstParameter();
                    break;

                case Internal:
                    param = pntE.Parameter();
                    break;

                case Reversed:
                    param = toReverse ? curveE.FirstParameter() : curveE.LastParameter();
                    break;

                default:
                    param = 0.0; // uninitialised in OCCT, and overwritten below
                    break;
            }

            // This overwrites the parameter the switch above computed,
            // making the position-based logic dead code. Removing this line caused
            // blend regressions. The switch logic may need endpoint-specific investigation.
            _ = param;
            param = pntE.Parameter(); // OCCT: overwrites the parameter chosen above (occt-issues.md)

            // As for the hatching above, OCCT passes CurveE.Curve()
            _intersector.LocalGeometry(curveE, param, out tangente2d, out normale2d, out courbure);

            //-----------------------------------------------------------------------
            // Calcul de la transition locale. On suppose les relations suivantes :
            //  - Si l orientation de l element est INTERNAL ==> INTERNAL
            //  - Si l orientation de l element est EXTERNAL ==> EXTERNAL
            //  - Si tangence, on a IN-IN  ou OUT-OUT ==> INTERNAL/EXTERNAL
            //  - Sinon,       on a IN-OUT ou OUT-IN  ==> REVERSED/FORWARD
            // Les deux dernieres conditions avec l element vu en FORWARD.
            //-----------------------------------------------------------------------
            Orientation localTransition = External;

            if (elementOrientation == Internal)
            {
                localTransition = Internal;
            }
            else if (elementOrientation == External)
            {
                localTransition = External;
            }
            else if (pntE.IntersectionType() == IntersectionType.Tangent)
            {
                if (pntE.Position() == Internal)
                {
                    switch (pntE.StateBefore())
                    {
                        case global::Kerf.Topology.Containment.In:
                            localTransition = toReverse ? External : Internal;
                            break;
                        case global::Kerf.Topology.Containment.Out:
                            localTransition = toReverse ? Internal : External;
                            break;
                        default:
                            break;
                    }
                }
                else
                {
                    switch (pntE.StateBefore())
                    {
                        case global::Kerf.Topology.Containment.In:
                            localTransition = toReverse ? Forward : Reversed;
                            break;
                        case global::Kerf.Topology.Containment.Out:
                            localTransition = toReverse ? Reversed : Forward;
                            break;
                        default:
                            break;
                    }
                }
            }
            else
            {
                switch (pntE.StateBefore())
                {
                    case global::Kerf.Topology.Containment.In:
                        localTransition = toReverse ? Forward : Reversed;
                        break;
                    case global::Kerf.Topology.Containment.Out:
                        localTransition = toReverse ? Reversed : Forward;
                        break;
                    default:
                        break;
                }
            }

            //-----------------------------------------------------------------------
            // Orientation de la tangente au point d interference.
            //-----------------------------------------------------------------------
            Orientation tangenteOrientation = Forward;
            switch (pntE.Position())
            {
                case Forward:
                    tangenteOrientation = toReverse ? Reversed : Forward;
                    break;
                case Internal:
                    tangenteOrientation = Internal;
                    break;
                case Reversed:
                    tangenteOrientation = toReverse ? Forward : Reversed;
                    break;

                default:
                    break;
            }

            //-----------------------------------------------------------------------
            // Proprietes geometriques.
            //-----------------------------------------------------------------------

            if (toReverse)
            {
                tangente = tangente.WithCoord(-tangente2d.X, -tangente2d.Y, 0.0);
            }
            else
            {
                tangente = tangente.WithCoord(tangente2d.X, tangente2d.Y, 0.0);
            }
            normale = normale.WithCoord(normale2d.X, normale2d.Y, 0.0);

            complexTransition.Compare(global::Kerf.Engine.Foundation.Precision.Angular(),
                                      tangente,
                                      normale,
                                      courbure,
                                      localTransition,
                                      tangenteOrientation);
        }

        switch (complexTransition.StateBefore())
        {
            case global::Kerf.Topology.Containment.In:
                stateBefore = global::Kerf.Topology.Containment.In;
                break;
            case global::Kerf.Topology.Containment.Out:
                stateBefore = global::Kerf.Topology.Containment.Out;
                break;
            case On:
                return false;
            case Unknown:
                return false;
        }
        switch (complexTransition.StateAfter())
        {
            case global::Kerf.Topology.Containment.In:
                stateAfter = global::Kerf.Topology.Containment.In;
                break;
            case global::Kerf.Topology.Containment.Out:
                stateAfter = global::Kerf.Topology.Containment.Out;
                break;
            case On:
                return false;
            case Unknown:
                return false;
        }

        point.SetStateBefore(stateBefore);
        point.SetStateAfter(stateAfter);
        point.SetSegmentBeginning(segmentBegin);
        point.SetSegmentEnd(segmentEnd);
        return true;
    }

    /// <summary>Computes the domains of all the hatchings.</summary>
    public void ComputeDomains()
    {
        for (int indH = 1; indH <= _nbHatchings; indH++)
        {
            if (_hatchings.IsBound(indH))
            {
                ComputeDomains(indH);
            }
        }
    }

    /// <summary>
    /// Computes the domains of the <paramref name="indH"/>-th hatching, trimming it first if needed. A hatching with
    /// no intersection point is one domain or none, by the classification of a point of it.
    /// </summary>
    public void ComputeDomains(int indH)
    {
        Hatching2D hatching = _hatchings.ChangeFind(indH);
        hatching.ClrDomains();

        hatching.IsDone(false);

        if (!hatching.TrimDone())
        {
            Trim(indH);
        }

        if (hatching.Status() != HatchingErrorStatus.NoProblem)
        {
            return;
        }

        bool points = _keepPoints;
        bool segments = _keepSegments;
        int iSav = 0;
        bool savPnt = false;
        int nbOpenedSegments = 0;
        int nbPnt = hatching.PointCount;
        int iPnt = 1;

        if (nbPnt == 0)
        {
            //-- std::cout << "The hatching # " << std::setw(3) << IndH << " has to be classified" <<
            // std::endl ;
            Classifier2D classifier = new(_elements, hatching.ClassificationPoint(), 0.0000001);
            if (classifier.State() == global::Kerf.Topology.Containment.In)
            {
                HatchingDomain domain0 = new();
                hatching.AddDomain(domain0);
            }

            hatching.IsDone(true);
            return;
        }

        // for (int IPnt = 1 ; IPnt <= NbPnt ; IPnt++) {
        for (iPnt = 1; iPnt <= nbPnt; iPnt++)
        {
            bool noDomain = hatching.DomainCount == 0;
            bool firstPoint = iPnt == 1;
            bool lastPoint = iPnt == nbPnt;

            PointOnHatching curPnt = hatching.Point(iPnt);

            //-----------------------------------------------------------------------
            // Calcul des domaines.
            //-----------------------------------------------------------------------

            Containment stateBefore = curPnt.StateBefore();
            Containment stateAfter = curPnt.StateAfter();
            bool segmentBegin = curPnt.SegmentBeginning();
            bool segmentEnd = curPnt.SegmentEnd();

            HatchingDomain domain = new();

            //-----------------------------------------------------------------------
            // Initialisations dues au premier point.
            //-----------------------------------------------------------------------

            if (firstPoint)
            {
                savPnt = false;
                iSav = 0;
                nbOpenedSegments = 0;
                if (segmentEnd && segmentBegin)
                {
                    if (stateAfter == Unknown)
                    {
                        stateAfter = global::Kerf.Topology.Containment.In;
                    }
                    if (stateBefore == Unknown)
                    {
                        stateBefore = global::Kerf.Topology.Containment.In;
                    }

                    if (segments)
                    {
                        savPnt = true;
                        iSav = 0;
                    }
                }
                else if (segmentEnd)
                {
                    if (stateAfter == Unknown)
                    {
                        stateAfter = global::Kerf.Topology.Containment.In;
                    }

                    if (segments)
                    {
                        savPnt = true;
                        iSav = 0;
                    }
                }
                else if (segmentBegin)
                {
                    if (stateBefore == Unknown)
                    {
                        stateBefore = global::Kerf.Topology.Containment.In;
                    }
                    if (stateBefore == global::Kerf.Topology.Containment.In)
                    {
                        savPnt = true;
                        iSav = 0;
                    }
                }
                else
                {
                    if (stateBefore == global::Kerf.Topology.Containment.In)
                    {
                        savPnt = true;
                        iSav = 0;
                    }
                }
            }

            //-----------------------------------------------------------------------
            // Initialisations dues au dernier point.
            //-----------------------------------------------------------------------

            if (lastPoint)
            {
                if (segmentEnd && segmentBegin)
                {
                    if (stateAfter == Unknown)
                    {
                        stateAfter = global::Kerf.Topology.Containment.In;
                    }

                    if (stateBefore == Unknown)
                    {
                        stateBefore = global::Kerf.Topology.Containment.In;
                    }
                }
                else if (segmentEnd)
                {
                    if (stateAfter == Unknown)
                    {
                        stateAfter = global::Kerf.Topology.Containment.In;
                    }
                }
                else if (segmentBegin)
                {
                    if (stateBefore == Unknown)
                    {
                        stateBefore = global::Kerf.Topology.Containment.In;
                    }
                }
                else
                {
                }
            }

            //-----------------------------------------------------------------------
            // Cas general.
            //-----------------------------------------------------------------------

            bool toAppend = false;

            if (segmentEnd && segmentBegin)
            {
                if (stateBefore != global::Kerf.Topology.Containment.In && stateAfter != global::Kerf.Topology.Containment.In)
                {
                    hatching.Status(HatchingErrorStatus.IncompatibleStates);
                    return;
                }

                if (points)
                {
                    if (segments)
                    {
                        if (!savPnt)
                        {
                            if (noDomain)
                            {
                                hatching.Status(HatchingErrorStatus.IncoherentParity);
                            }
                            else
                            {
                                hatching.IsDone(true);
                            }
                            return;
                        }

                        if (iSav != 0)
                        {
                            domain.SetFirstPoint(hatching.Point(iSav));
                        }

                        domain.SetSecondPoint(curPnt);
                        toAppend = true;
                        savPnt = true;
                        iSav = iPnt;
                    }
                    else
                    {
                        bool isININ = (stateBefore == global::Kerf.Topology.Containment.In && stateAfter == global::Kerf.Topology.Containment.In);
                        if (savPnt && !isININ)
                        {
                            if (noDomain)
                            {
                                hatching.Status(HatchingErrorStatus.IncoherentParity);
                            }
                            else
                            {
                                hatching.IsDone(true);
                            }

                            return;
                        }

                        domain.SetPoints(curPnt, curPnt);
                        toAppend = true;
                        savPnt = false;
                        iSav = 0;
                    }
                }
            }
            else if (segmentEnd)
            {
                if (segments)
                {
                    if (stateAfter == global::Kerf.Topology.Containment.Out)
                    {
                        if (!savPnt)
                        {
                            if (noDomain)
                            {
                                hatching.Status(HatchingErrorStatus.IncoherentParity);
                            }
                            else
                            {
                                hatching.IsDone(true);
                            }
                            return;
                        }

                        if (iSav != 0)
                        {
                            domain.SetFirstPoint(hatching.Point(iSav));
                        }

                        domain.SetSecondPoint(curPnt);
                        toAppend = true;
                    }
                    else
                    {
                        if (points)
                        {
                            if (iSav != 0)
                            {
                                domain.SetFirstPoint(hatching.Point(iSav));
                            }

                            domain.SetSecondPoint(curPnt);
                            toAppend = true;
                            savPnt = true;
                            iSav = iPnt;
                        }
                    }
                }
                else
                {
                    if (stateAfter == global::Kerf.Topology.Containment.In)
                    {
                        savPnt = true;
                        iSav = iPnt;
                    }
                }

                nbOpenedSegments--;
            }
            else if (segmentBegin)
            {
                if (segments)
                {
                    if (stateBefore == global::Kerf.Topology.Containment.Out)
                    {
                        savPnt = true;
                        iSav = iPnt;
                    }
                    else
                    {
                        if (points)
                        {
                            if (!savPnt)
                            {
                                if (noDomain)
                                {
                                    hatching.Status(HatchingErrorStatus.IncoherentParity);
                                }
                                else
                                {
                                    hatching.IsDone(true);
                                }

                                return;
                            }

                            if (iSav != 0)
                            {
                                domain.SetFirstPoint(hatching.Point(iSav));
                            }

                            domain.SetSecondPoint(curPnt);
                            toAppend = true;
                            savPnt = true;
                            iSav = iPnt;
                        }
                    }
                }
                else
                {
                    if (stateBefore == global::Kerf.Topology.Containment.In)
                    {
                        if (!savPnt)
                        {
                            if (noDomain)
                            {
                                hatching.Status(HatchingErrorStatus.IncoherentParity);
                            }
                            else
                            {
                                hatching.IsDone(true);
                            }

                            return;
                        }

                        if (iSav != 0)
                        {
                            domain.SetFirstPoint(hatching.Point(iSav));
                        }

                        domain.SetSecondPoint(curPnt);
                        toAppend = true;

                        // Modified by Sergey KHROMOV - Fri Jan  5 12:05:30 2001
                        // SavPnt = false ;
                        // ISav = 0 ;

                        savPnt = true;
                        iSav = iPnt;
                        // Modified by Sergey KHROMOV - Fri Jan  5 12:05:31 2001
                    }
                }

                nbOpenedSegments++;
            }
            else
            {
                //-- ???????????????????????????????????????????????????????????????????????????
                //-- Solution provisoire (lbr le 11 Aout 97 )
                //-- si On a 2 points dont des points OUT OUT ou IN IN qui delimitent une isos
                //-- on transforme les transitions
                if (stateBefore == global::Kerf.Topology.Containment.Out && stateAfter == global::Kerf.Topology.Containment.Out)
                {
                    if (nbPnt == 2)
                    {
                        if (firstPoint)
                        {
                            stateAfter = global::Kerf.Topology.Containment.In;
                        }
                        else
                        {
                            stateBefore = global::Kerf.Topology.Containment.In;
                        }
                    }
                }
                //-- ???????????????????????????????????????????????????????????????????????????
                if (stateBefore == global::Kerf.Topology.Containment.Out && stateAfter == global::Kerf.Topology.Containment.Out)
                {
                    if (savPnt)
                    {
                        if (noDomain)
                        {
                            hatching.Status(HatchingErrorStatus.IncoherentParity);
                        }
                        else
                        {
                            hatching.IsDone(true);
                        }

                        return;
                    }

                    if (points)
                    {
                        domain.SetPoints(curPnt, curPnt);
                        toAppend = true;
                        savPnt = true;
                        iSav = iPnt;
                    }
                }
                else if (stateBefore == global::Kerf.Topology.Containment.Out && stateAfter == global::Kerf.Topology.Containment.In)
                {
                    savPnt = true;
                    iSav = iPnt;
                }
                else if (stateBefore == global::Kerf.Topology.Containment.In && stateAfter == global::Kerf.Topology.Containment.Out)
                {
                    if (!savPnt)
                    {
                        if (noDomain)
                        {
                            hatching.Status(HatchingErrorStatus.IncoherentParity);
                        }
                        else
                        {
                            hatching.IsDone(true);
                        }

                        return;
                    }

                    if (iSav != 0)
                    {
                        domain.SetFirstPoint(hatching.Point(iSav));
                    }

                    domain.SetSecondPoint(curPnt);
                    toAppend = true;
                    savPnt = false;
                    iSav = 0;
                }
                else if (stateBefore == global::Kerf.Topology.Containment.In && stateAfter == global::Kerf.Topology.Containment.In)
                {
                    if (points)
                    {
                        if (nbOpenedSegments == 0)
                        {
                            if (!savPnt)
                            {
                                if (noDomain)
                                {
                                    hatching.Status(HatchingErrorStatus.IncoherentParity);
                                }
                                else
                                {
                                    hatching.IsDone(true);
                                }

                                // return;
                                continue;
                            }

                            if (iSav != 0)
                            {
                                domain.SetFirstPoint(hatching.Point(iSav));
                            }

                            domain.SetSecondPoint(curPnt);
                            toAppend = true;
                            savPnt = true;
                            iSav = iPnt;
                        }
                        else
                        {
                            if (segments)
                            {
                                if (!savPnt)
                                {
                                    if (noDomain)
                                    {
                                        hatching.Status(HatchingErrorStatus.IncoherentParity);
                                    }
                                    else
                                    {
                                        hatching.IsDone(true);
                                    }

                                    return;
                                }

                                if (iSav != 0)
                                {
                                    domain.SetFirstPoint(hatching.Point(iSav));
                                }

                                domain.SetSecondPoint(curPnt);
                                toAppend = true;
                                savPnt = true;
                                iSav = iPnt;
                            }
                            else
                            {
                                if (savPnt)
                                {
                                    if (noDomain)
                                    {
                                        hatching.Status(HatchingErrorStatus.IncoherentParity);
                                    }
                                    else
                                    {
                                        hatching.IsDone(true);
                                    }

                                    return;
                                }

                                domain.SetPoints(curPnt, curPnt);
                                toAppend = true;
                                savPnt = false;
                                iSav = 0;
                            }
                        }
                    }
                }
                else
                {
                    hatching.Status(HatchingErrorStatus.IncompatibleStates);
                    return;
                }
            }

            //-----------------------------------------------------------------------
            // Ajout du domaine.
            //-----------------------------------------------------------------------

            if (toAppend)
            {
                hatching.AddDomain(domain);
            }

            //-----------------------------------------------------------------------
            // Traitement lie au dernier point.
            //-----------------------------------------------------------------------

            if (lastPoint)
            {
                domain.SetPoints();
                toAppend = false;

                if (segmentEnd && segmentBegin)
                {
                    if (segments)
                    {
                        if (!savPnt)
                        {
                            if (noDomain)
                            {
                                hatching.Status(HatchingErrorStatus.IncoherentParity);
                            }
                            else
                            {
                                hatching.IsDone(true);
                            }

                            return;
                        }

                        if (iSav != 0)
                        {
                            domain.SetFirstPoint(hatching.Point(iSav));
                        }

                        toAppend = true;
                    }
                }
                else if (segmentEnd)
                {
                    if (stateAfter == global::Kerf.Topology.Containment.In)
                    {
                        if (!savPnt)
                        {
                            if (noDomain)
                            {
                                hatching.Status(HatchingErrorStatus.IncoherentParity);
                            }
                            else
                            {
                                hatching.IsDone(true);
                            }

                            return;
                        }

                        if (iSav != 0)
                        {
                            domain.SetFirstPoint(hatching.Point(iSav));
                        }

                        toAppend = true;
                    }
                }
                else if (segmentBegin)
                {
                    if (segments)
                    {
                        if (!savPnt)
                        {
                            if (noDomain)
                            {
                                hatching.Status(HatchingErrorStatus.IncoherentParity);
                            }
                            else
                            {
                                hatching.IsDone(true);
                            }

                            return;
                        }

                        if (iSav != 0)
                        {
                            domain.SetFirstPoint(hatching.Point(iSav));
                        }

                        toAppend = true;
                    }
                }
                else
                {
                    if (stateAfter == global::Kerf.Topology.Containment.In)
                    {
                        if (!savPnt)
                        {
                            if (noDomain)
                            {
                                hatching.Status(HatchingErrorStatus.IncoherentParity);
                            }
                            else
                            {
                                hatching.IsDone(true);
                            }

                            return;
                        }

                        if (iSav != 0)
                        {
                            domain.SetFirstPoint(hatching.Point(iSav));
                        }

                        toAppend = true;
                    }
                }

                if (toAppend)
                {
                    hatching.AddDomain(domain);
                }
            }
        }

        hatching.IsDone(true);
    }

    //=======================================================================
    //  Category : Results.
    //=======================================================================

    /// <summary>True if the intersections of the <paramref name="indH"/>-th hatching were computed.</summary>
    public bool TrimDone(int indH)
    {
        Hatching2D hatching = _hatchings.Find(indH);
        return hatching.TrimDone();
    }

    /// <summary>True if an intersection of the <paramref name="indH"/>-th hatching failed.</summary>
    public bool TrimFailed(int indH)
    {
        Hatching2D hatching = _hatchings.Find(indH);
        return hatching.TrimFailed();
    }

    /// <summary>True if the domains of the <paramref name="indH"/>-th hatching were computed.</summary>
    public bool IsDone(int indH)
    {
        Hatching2D hatching = _hatchings.Find(indH);
        return hatching.IsDone();
    }

    /// <summary>The status of the <paramref name="indH"/>-th hatching.</summary>
    public HatchingErrorStatus Status(int indH)
    {
        Hatching2D hatching = _hatchings.Find(indH);
        return hatching.Status();
    }

    /// <summary>The number of domains of the <paramref name="indH"/>-th hatching.</summary>
    /// <exception cref="NotDoneException">The domains are not computed.</exception>
    public int NbDomains(int indH)
    {
        Hatching2D hatching = _hatchings.Find(indH);
        StdFail_NotDone_Raise_if(!hatching.IsDone(), "Geom2dHatch_Hatcher::NbDomains");
        return hatching.DomainCount;
    }

    /// <summary>The <paramref name="iDom"/>-th domain of the <paramref name="indH"/>-th hatching.</summary>
    /// <exception cref="NotDoneException">The domains are not computed.</exception>
    public HatchingDomain Domain(int indH, int iDom)
    {
        Hatching2D hatching = _hatchings.Find(indH);
        StdFail_NotDone_Raise_if(!hatching.IsDone(), "Geom2dHatch_Hatcher::Domain");
        HatchingDomain domain = hatching.Domain(iDom);
        return domain;
    }

    /// <summary>The <paramref name="iDom"/>-th domain of the <paramref name="indH"/>-th hatching, for writing (Kerf:
    /// ChFi3d_Builder_SpKP's <c>Tri</c> casts the constness of <see cref="Domain"/> away and edits it in place).</summary>
    internal ref HatchingDomain ChangeDomain(int indH, int iDom)
    {
        Hatching2D hatching = _hatchings.Find(indH);
        StdFail_NotDone_Raise_if(!hatching.IsDone(), "Geom2dHatch_Hatcher::Domain");
        return ref hatching.ChangeDomain(iDom);
    }

    //=======================================================================
    //  Category : Dump.
    //=======================================================================

    /// <summary>Writes the hatcher (its tolerances, hatchings and elements) to <paramref name="os"/>, as OCCT writes it to std::cout.</summary>
    public void Dump(TextWriter os)
    {
        os.Write('\n');
        os.Write("========================================================\n");
        os.Write("=== Dump of the hatcher ================================\n");
        os.Write("========================================================\n");
        os.Write('\n');

        os.Write($"The points   are {(_keepPoints ? "    " : "not ")}considered.\n");
        os.Write($"The segments are {(_keepSegments ? "    " : "not ")}considered.\n");
        os.Write($"2D Confusion tolerance : {IntersectionPoint.Number(_confusion2d)}\n");
        os.Write($"3D Confusion tolerance : {IntersectionPoint.Number(_confusion3d)}\n");

        os.Write($"{_nbHatchings} hatching{((_nbHatchings == 1) ? "" : "s")}\n");
        os.Write($"{_nbElements} element{((_nbElements == 1) ? "" : "s")}\n");

        os.Write('\n');
        os.Write("========================================================\n");
        os.Write("=== Hatchings ==========================================\n");
        os.Write("========================================================\n");
        os.Write('\n');

        for (int indH = 1; indH <= _nbHatchings; indH++)
        {
            os.Write($"Hatching # {indH}");
            if (!_hatchings.IsBound(indH))
            {
                os.Write(" is not bound\n");
            }
            else
            {
                Hatching2D hatching = _hatchings.Find(indH);
                int nbPnt = hatching.PointCount;
                os.Write($" contains {nbPnt} restriction points :\n");
                for (int iPnt = 1; iPnt <= nbPnt; iPnt++)
                {
                    PointOnHatching pntH = hatching.Point(iPnt);
                    pntH.Dump(os, iPnt);
                }
                os.Write("----------------------------------------------\n");
            }
        }

        os.Write('\n');
        os.Write("========================================================\n");
        os.Write("=== Elements ===========================================\n");
        os.Write("========================================================\n");
        os.Write('\n');

        for (int indE = 1; indE <= _nbElements; indE++)
        {
            os.Write($"Element # {indE}");
            if (!_elements.IsBound(indE))
            {
                os.Write(" is not bound\n");
            }
            else
            {
                Element2D element = _elements.Find(indE);
                switch (element.Orientation())
                {
                    case Forward:
                        os.Write(" is FORWARD\n");
                        break;
                    case Reversed:
                        os.Write(" is REVERSED\n");
                        break;
                    case Internal:
                        os.Write(" is INTERNAL\n");
                        break;
                    case External:
                        os.Write(" is EXTERNAL\n");
                        break;
                }
            }
        }

        os.Write('\n');
    }
}
