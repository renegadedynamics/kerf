// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Hatcher.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Hatcher.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Hatcher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.HatchGen_ErrorStatus;
using static Occt.HatchGen_IntersectionType;
using static Occt.IntRes2d_Position;
using static Occt.IntRes2d_TypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Occt;

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
public class Geom2dHatch_Hatcher
{
    private readonly Geom2dHatch_Intersector myIntersector;
    private double myConfusion2d;
    private double myConfusion3d;
    private bool myKeepPoints;
    private bool myKeepSegments;
    private int myNbElements;
    private readonly Geom2dHatch_Elements myElements = new();
    private int myNbHatchings;
    private readonly NCollection_IntDataMap<Geom2dHatch_Hatching> myHatchings = new();

    //=======================================================================
    //  Category : General use.
    //=======================================================================

    /// <summary>
    /// An empty hatcher with (a copy of) the intersector <paramref name="Intersector"/>, the 2D and 3D confusion
    /// tolerances, and whether the domains keep isolated points (<paramref name="KeepPnt"/>) and segments
    /// (<paramref name="KeepSeg"/>).
    /// </summary>
    public Geom2dHatch_Hatcher(Geom2dHatch_Intersector Intersector,
                               double Confusion2d,
                               double Confusion3d,
                               bool KeepPnt = false,
                               bool KeepSeg = false)
    {
        myIntersector = new Geom2dHatch_Intersector(Intersector);
        myConfusion2d = Confusion2d;
        myConfusion3d = Confusion3d;
        myKeepPoints = KeepPnt;
        myKeepSegments = KeepSeg;
        myNbElements = 0;
        myNbHatchings = 0;
    }

    /// <summary>Sets the intersector (a copy of <paramref name="Intersector"/>); the trimmings are cleared.</summary>
    public void Intersector(Geom2dHatch_Intersector Intersector)
    {
        myIntersector.Assign(Intersector);
        for (int IndH = 1; IndH <= myNbHatchings; IndH++)
        {
            if (myHatchings.IsBound(IndH))
            {
                Geom2dHatch_Hatching Hatching = myHatchings.ChangeFind(IndH);
                Hatching.ClrPoints();
            }
        }
    }

    /// <summary>The intersector of the hatcher.</summary>
    public Geom2dHatch_Intersector Intersector() => myIntersector;

    /// <summary>The intersector of the hatcher, for changing its tolerances.</summary>
    public Geom2dHatch_Intersector ChangeIntersector() => myIntersector;

    /// <summary>Sets the 2D confusion tolerance; the trimmings are cleared.</summary>
    public void Confusion2d(double Confusion)
    {
        myConfusion2d = Confusion;
        for (int IndH = 1; IndH <= myNbHatchings; IndH++)
        {
            if (myHatchings.IsBound(IndH))
            {
                Geom2dHatch_Hatching Hatching = myHatchings.ChangeFind(IndH);
                Hatching.ClrPoints();
            }
        }
    }

    /// <summary>The 2D confusion tolerance.</summary>
    public double Confusion2d() => myConfusion2d;

    /// <summary>Sets the 3D confusion tolerance; the trimmings are cleared.</summary>
    public void Confusion3d(double Confusion)
    {
        myConfusion3d = Confusion;
        for (int IndH = 1; IndH <= myNbHatchings; IndH++)
        {
            if (myHatchings.IsBound(IndH))
            {
                Geom2dHatch_Hatching Hatching = myHatchings.ChangeFind(IndH);
                Hatching.ClrPoints();
            }
        }
    }

    /// <summary>The 3D confusion tolerance.</summary>
    public double Confusion3d() => myConfusion3d;

    /// <summary>Sets whether the domains keep isolated points; the domains are cleared.</summary>
    public void KeepPoints(bool Keep)
    {
        myKeepPoints = Keep;
        for (int IndH = 1; IndH <= myNbHatchings; IndH++)
        {
            if (myHatchings.IsBound(IndH))
            {
                Geom2dHatch_Hatching Hatching = myHatchings.ChangeFind(IndH);
                Hatching.ClrDomains();
            }
        }
    }

    /// <summary>True if the domains keep isolated points.</summary>
    public bool KeepPoints() => myKeepPoints;

    /// <summary>Sets whether the domains keep segments; the domains are cleared.</summary>
    public void KeepSegments(bool Keep)
    {
        myKeepSegments = Keep;
        for (int IndH = 1; IndH <= myNbHatchings; IndH++)
        {
            if (myHatchings.IsBound(IndH))
            {
                Geom2dHatch_Hatching Hatching = myHatchings.ChangeFind(IndH);
                Hatching.ClrDomains();
            }
        }
    }

    /// <summary>True if the domains keep segments.</summary>
    public bool KeepSegments() => myKeepSegments;

    /// <summary>Removes all the hatchings and all the elements.</summary>
    public void Clear()
    {
        if (myNbHatchings != 0)
        {
            ClrHatchings();
        }
        if (myNbElements != 0)
        {
            ClrElements();
        }
    }

    //=======================================================================
    //  Category : Element.
    //=======================================================================

    /// <summary>The <paramref name="IndE"/>-th element, for writing.</summary>
    protected ref Geom2dHatch_Element Element(int IndE)
    {
        ref Geom2dHatch_Element Element = ref myElements.ChangeFind(IndE);
        return ref Element;
    }

    /// <summary>The curve of the <paramref name="IndE"/>-th element.</summary>
    public CurveAdaptor2D ElementCurve(int IndE)
    {
        Geom2dHatch_Element Element = myElements.Find(IndE);
        return Element.Curve();
    }

    /// <summary>
    /// Adds an element (a copy of <paramref name="Curve"/>, with its orientation) and returns its index: the lowest
    /// free one. The trimmings are cleared.
    /// </summary>
    public int AddElement(CurveAdaptor2D Curve, Orientation Orientation = Forward)
    {
        int IndE;
        for (IndE = 1; IndE <= myNbElements && myElements.IsBound(IndE); IndE++)
        {
        }
        if (IndE > myNbElements)
        {
            myNbElements++;
            IndE = myNbElements;
        }
        Geom2dHatch_Element Element = new(Curve, Orientation);
        myElements.Bind(IndE, Element);
        for (int IndH = 1; IndH <= myNbHatchings; IndH++)
        {
            if (myHatchings.IsBound(IndH))
            {
                Geom2dHatch_Hatching Hatching = myHatchings.ChangeFind(IndH);
                Hatching.ClrPoints();
            }
        }
        return IndE;
    }

    /// <summary>Adds an element on the curve <paramref name="Curve"/>, with its orientation, and returns its index.</summary>
    public int AddElement(Curve2D Curve, Orientation Orientation = Forward)
    {
        CurveAdaptor2D aGAC = new(Curve);
        return AddElement(aGAC, Orientation);
    }

    /// <summary>Removes the <paramref name="IndE"/>-th element.</summary>
    public void RemElement(int IndE)
    {
        for (int IndH = 1; IndH <= myNbHatchings; IndH++)
        {
            if (myHatchings.IsBound(IndH))
            {
                Geom2dHatch_Hatching Hatching = myHatchings.ChangeFind(IndH);
                bool DomainsToClear = false;
                for (int IPntH = Hatching.NbPoints(); IPntH > 0; IPntH--)
                {
                    // OCCT copies the point, so the removals below don't reach the hatching's points
                    // (occt-issues.md): only a point left with no point on element is removed.
                    HatchGen_PointOnHatching PntH = new(Hatching.ChangePoint(IPntH));
                    for (int IPntE = PntH.NbPoints(); IPntE > 0; IPntE--)
                    {
                        if (PntH.Point(IPntE).Index() == IndE)
                        {
                            PntH.RemPoint(IPntE);
                            DomainsToClear = true;
                        }
                    }
                    if (PntH.NbPoints() == 0)
                    {
                        Hatching.RemPoint(IPntH);
                    }
                }
                if (DomainsToClear)
                {
                    Hatching.ClrDomains();
                }
            }
        }
        myElements.UnBind(IndE);
        if (IndE == myNbElements)
        {
            myNbElements--;
        }
    }

    /// <summary>Removes all the elements; the trimmings are cleared.</summary>
    public void ClrElements()
    {
        if (myNbElements != 0)
        {
            if (myNbHatchings != 0)
            {
                for (int IndH = 1; IndH <= myNbHatchings; IndH++)
                {
                    if (myHatchings.IsBound(IndH))
                    {
                        Geom2dHatch_Hatching Hatching = myHatchings.ChangeFind(IndH);
                        Hatching.ClrPoints();
                    }
                }
            }
            myElements.Clear();
            myNbElements = 0;
        }
    }

    //=======================================================================
    //  Category : Hatching.
    //=======================================================================

    /// <summary>The <paramref name="IndH"/>-th hatching.</summary>
    protected Geom2dHatch_Hatching Hatching(int IndH)
    {
        Geom2dHatch_Hatching Hatching = myHatchings.ChangeFind(IndH);
        return Hatching;
    }

    /// <summary>The curve of the <paramref name="IndH"/>-th hatching.</summary>
    public CurveAdaptor2D HatchingCurve(int IndH)
    {
        Geom2dHatch_Hatching Hatching = myHatchings.Find(IndH);
        return Hatching.Curve();
    }

    /// <summary>Adds a hatching on (a copy of) <paramref name="Curve"/> and returns its index: the lowest free one.</summary>
    public int AddHatching(CurveAdaptor2D Curve)
    {
        int IndH;
        for (IndH = 1; IndH <= myNbHatchings && myHatchings.IsBound(IndH); IndH++)
        {
        }
        if (IndH > myNbHatchings)
        {
            myNbHatchings++;
            IndH = myNbHatchings;
        }
        Geom2dHatch_Hatching Hatching = new(Curve);
        myHatchings.Bind(IndH, Hatching);
        return IndH;
    }

    /// <summary>Removes the <paramref name="IndH"/>-th hatching.</summary>
    public void RemHatching(int IndH)
    {
        Geom2dHatch_Hatching Hatching = myHatchings.ChangeFind(IndH);
        Hatching.ClrPoints();
        myHatchings.UnBind(IndH);
        if (IndH == myNbHatchings)
        {
            myNbHatchings--;
        }
    }

    /// <summary>Removes all the hatchings.</summary>
    public void ClrHatchings()
    {
        if (myNbHatchings != 0)
        {
            for (int IndH = 1; IndH <= myNbHatchings; IndH++)
            {
                if (myHatchings.IsBound(IndH))
                {
                    Geom2dHatch_Hatching Hatching = myHatchings.ChangeFind(IndH);
                    Hatching.ClrPoints();
                }
            }
            myHatchings.Clear();
            myNbHatchings = 0;
        }
    }

    /// <summary>The number of intersection points of the <paramref name="IndH"/>-th hatching.</summary>
    public int NbPoints(int IndH)
    {
        Geom2dHatch_Hatching Hatching = myHatchings.Find(IndH);
        return Hatching.NbPoints();
    }

    /// <summary>The <paramref name="IndP"/>-th intersection point of the <paramref name="IndH"/>-th hatching.</summary>
    public HatchGen_PointOnHatching Point(int IndH, int IndP)
    {
        Geom2dHatch_Hatching Hatching = myHatchings.Find(IndH);
        HatchGen_PointOnHatching PntH = Hatching.Point(IndP);
        return PntH;
    }

    //=======================================================================
    //  Category : Computation - Trimming
    //=======================================================================

    /// <summary>Trims all the hatchings by all the elements.</summary>
    public void Trim()
    {
        for (int IndH = 1; IndH <= myNbHatchings; IndH++)
        {
            if (myHatchings.IsBound(IndH))
            {
                Trim(IndH);
            }
        }
    }

    /// <summary>Adds a hatching on <paramref name="Curve"/>, trims it by the elements and returns its index.</summary>
    public int Trim(CurveAdaptor2D Curve)
    {
        int IndH = AddHatching(Curve);
        Trim(IndH);
        return IndH;
    }

    /// <summary>Trims the <paramref name="IndH"/>-th hatching by the elements, then computes its points' transitions.</summary>
    public void Trim(int IndH)
    {
        Geom2dHatch_Hatching Hatching = myHatchings.ChangeFind(IndH);

        Hatching.ClrPoints();

        bool OK, AllOK;

        AllOK = true;
        for (int IndE = 1; IndE <= myNbElements; IndE++)
        {
            if (myElements.IsBound(IndE))
            {
                OK = Trim(IndH, IndE);
                AllOK = AllOK && OK;
            }
        }
        Hatching.TrimDone(true);
        Hatching.TrimFailed(!AllOK);

        if (AllOK)
        {
            for (int IPnt = 1; IPnt <= Hatching.NbPoints(); IPnt++)
            {
                HatchGen_PointOnHatching PntH = Hatching.ChangePoint(IPnt);
                OK = GlobalTransition(PntH);
                AllOK = AllOK && OK;
            }
            Hatching.Status(AllOK ? HatchGen_NoProblem : HatchGen_TransitionFailure);
        }
    }

    // Trims the IndH-th hatching by the IndE-th element.
    private bool Trim(int IndH, int IndE)
    {
        Geom2dHatch_Hatching Hatching = myHatchings.ChangeFind(IndH);
        ref Geom2dHatch_Element Element = ref myElements.ChangeFind(IndE);

        // OCCT intersects copies of the two adaptors, which share their curves and evaluation caches with the
        // originals; the port passes the adaptors themselves (decision 666)
        CurveAdaptor2D hatching = Hatching.ChangeCurve();
        CurveAdaptor2D element = Element.ChangeCurve();

        myIntersector.Intersect(hatching, element);

        if (!myIntersector.IsDone())
        {
            // OCCT also writes " Intersector -> Done = False " to std::cout (left out, occt-issues.md)
            return false;
        }

        if (myIntersector.IsEmpty())
        {
            return true;
        }

        //-----------------------------------------------------------------------
        // Traitement des points d intersection.
        //-----------------------------------------------------------------------

        for (int IPntI = 1; IPntI <= myIntersector.NbPoints(); IPntI++)
        {
            IntRes2d_IntersectionPoint PntI = myIntersector.Point(IPntI);

            HatchGen_PointOnElement PntE = new(PntI);
            PntE.SetIndex(IndE);

            HatchGen_PointOnHatching PntH = new(PntI);
            PntH.SetIndex(IndH);
            PntH.AddPoint(PntE, myConfusion2d);

            Hatching.AddPoint(PntH, myConfusion2d);
        }

        //-----------------------------------------------------------------------
        // Traitement des segments d intersection.
        //-----------------------------------------------------------------------

        for (int ISeg = 1; ISeg <= myIntersector.NbSegments(); ISeg++)
        {
            IntRes2d_IntersectionSegment Seg = myIntersector.Segment(ISeg);

            bool FirstPoint = Seg.HasFirstPoint();
            bool LastPoint = Seg.HasLastPoint();

            //-----------------------------------------------------------------------
            // Les deux points peuvent etre confondus.
            //-----------------------------------------------------------------------

            if (FirstPoint && LastPoint)
            {
                IntRes2d_IntersectionPoint Pnt1 = Seg.FirstPoint();
                IntRes2d_IntersectionPoint Pnt2 = Seg.LastPoint();

                IntRes2d_Transition TrsPnt1H = Pnt1.TransitionOfFirst();
                IntRes2d_Transition TrsPnt1E = Pnt1.TransitionOfSecond();
                IntRes2d_Transition TrsPnt2H = Pnt2.TransitionOfFirst();
                IntRes2d_Transition TrsPnt2E = Pnt2.TransitionOfSecond();

                IntRes2d_TypeTrans TypePnt1H = TrsPnt1H.TransitionType();
                IntRes2d_TypeTrans TypePnt1E = TrsPnt1E.TransitionType();
                IntRes2d_TypeTrans TypePnt2H = TrsPnt2H.TransitionType();
                IntRes2d_TypeTrans TypePnt2E = TrsPnt2E.TransitionType();

                //-----------------------------------------------------------------------
                // Les deux points peuvent etre confondus au regard de la precision du
                // `hatcher'.
                //-----------------------------------------------------------------------

                bool Conf2d = Math.Abs(Pnt1.ParamOnFirst() - Pnt2.ParamOnFirst()) <= myConfusion2d;

                //-----------------------------------------------------------------------
                // Les deux points peuvent etre `confondus' au regard des intersections.
                //-----------------------------------------------------------------------

                bool Conf3d = false;

                if (!Conf2d)
                {
                    Conf3d = true;
                    if (Conf3d)
                    {
                        Conf3d = TypePnt1H != IntRes2d_Touch && TypePnt1H != IntRes2d_Undecided;
                    }
                    if (Conf3d)
                    {
                        Conf3d = TypePnt1E != IntRes2d_Touch && TypePnt1E != IntRes2d_Undecided;
                    }
                    if (Conf3d)
                    {
                        Conf3d = TypePnt2H != IntRes2d_Touch && TypePnt2H != IntRes2d_Undecided;
                    }
                    if (Conf3d)
                    {
                        Conf3d = TypePnt2E != IntRes2d_Touch && TypePnt2E != IntRes2d_Undecided;
                    }
                    if (Conf3d)
                    {
                        Conf3d = TypePnt1H == TypePnt2H && TypePnt1E == TypePnt2E;
                    }
                    if (Conf3d)
                    {
                        Conf3d = Pnt1.Value().Distance(Pnt2.Value()) <= myConfusion3d;
                    }
                }

                if (Conf2d || Conf3d)
                {
                    HatchGen_PointOnElement PntE = new();
                    PntE.SetIndex(IndE);
                    PntE.SetParameter((Pnt1.ParamOnSecond() + Pnt2.ParamOnSecond()) / 2.0);
                    switch (TrsPnt1E.PositionOnCurve())
                    {
                        case IntRes2d_Head:
                        {
                            PntE.SetPosition(Forward);
                            break;
                        }
                        case IntRes2d_Middle:
                        {
                            switch (TrsPnt2E.PositionOnCurve())
                            {
                                case IntRes2d_Head:
                                {
                                    PntE.SetPosition(Forward);
                                    break;
                                }
                                case IntRes2d_Middle:
                                {
                                    PntE.SetPosition(Internal);
                                    break;
                                }
                                case IntRes2d_End:
                                {
                                    PntE.SetPosition(Reversed);
                                    break;
                                }
                                default:
                                {
                                    break;
                                }
                            }
                            break;
                        }
                        case IntRes2d_End:
                        {
                            PntE.SetPosition(Reversed);
                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }
                    PntE.SetIntersectionType((PntE.Position() == Internal) ? HatchGen_TRUE : HatchGen_TOUCH);
                    PntE.SetStateBefore((TypePnt1H == IntRes2d_In) ? Out : In);
                    PntE.SetStateAfter((TypePnt2H == IntRes2d_In) ? Out : In);

                    HatchGen_PointOnHatching PntH = new();
                    PntH.SetIndex(IndH);
                    PntH.SetParameter((Pnt1.ParamOnFirst() + Pnt2.ParamOnFirst()) / 2.0);
                    switch (TrsPnt1H.PositionOnCurve())
                    {
                        case IntRes2d_Head:
                        {
                            PntH.SetPosition(Forward);
                            break;
                        }
                        case IntRes2d_Middle:
                        {
                            switch (TrsPnt2H.PositionOnCurve())
                            {
                                case IntRes2d_Head:
                                {
                                    PntH.SetPosition(Forward);
                                    break;
                                }
                                case IntRes2d_Middle:
                                {
                                    PntH.SetPosition(Internal);
                                    break;
                                }
                                case IntRes2d_End:
                                {
                                    PntH.SetPosition(Reversed);
                                    break;
                                }
                                default:
                                {
                                    break;
                                }
                            }
                            break;
                        }
                        case IntRes2d_End:
                        {
                            PntH.SetPosition(Reversed);
                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }

                    PntH.AddPoint(PntE, myConfusion2d);
                    Hatching.AddPoint(PntH, myConfusion2d);

                    continue;
                }

                //-----------------------------------------------------------------------
                // Traitement du premier point du segment.
                //-----------------------------------------------------------------------

                if (FirstPoint)
                {
                    IntRes2d_IntersectionPoint PntI = Seg.FirstPoint();

                    HatchGen_PointOnElement PntE = new(PntI);
                    PntE.SetIndex(IndE);
                    PntE.SetSegmentBeginning(true);
                    PntE.SetSegmentEnd(false);

                    HatchGen_PointOnHatching PntH = new(PntI);
                    PntH.SetIndex(IndH);
                    PntH.AddPoint(PntE, myConfusion2d);

                    Hatching.AddPoint(PntH, myConfusion2d);
                }

                //-----------------------------------------------------------------------
                // Traitement du deuxieme point du segment.
                //-----------------------------------------------------------------------

                if (LastPoint)
                {
                    IntRes2d_IntersectionPoint PntI = Seg.LastPoint();

                    HatchGen_PointOnElement PntE = new(PntI);
                    PntE.SetIndex(IndE);
                    PntE.SetSegmentBeginning(false);
                    PntE.SetSegmentEnd(true);

                    HatchGen_PointOnHatching PntH = new(PntI);
                    PntH.SetIndex(IndH);
                    PntH.AddPoint(PntE, myConfusion2d);

                    Hatching.AddPoint(PntH, myConfusion2d);
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
    private bool GlobalTransition(HatchGen_PointOnHatching Point)
    {
        Containment StateBefore = Unknown;
        Containment StateAfter = Unknown;
        bool SegmentBegin = false;
        bool SegmentEnd = false;

        Direction2D Tangente2d, Normale2d;
        Direction3D Tangente = new(), Normale = new();
        double Courbure;

        CurveAdaptor2D CurveH = HatchingCurve(Point.Index());

        // OCCT passes CurveH.Curve(), converted to a new adaptor on the whole curve; LocalGeometry only reads the
        // adaptor's Curve(), which is the same
        myIntersector.LocalGeometry(CurveH, Point.Parameter(), out Tangente2d, out Normale2d, out Courbure);

        Tangente.SetCoord(Tangente2d.X(), Tangente2d.Y(), 0.0);
        if (Courbure < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Normale.SetCoord(-Tangente2d.Y(), Tangente2d.X(), 0.0);
        }
        else
        {
            Normale.SetCoord(Normale2d.X(), Normale2d.Y(), 0.0);
        }

        TopTrans_CurveTransition ComplexTransition = new();
        ComplexTransition.Reset(Tangente, Normale, Courbure);

        for (int IPntE = 1; IPntE <= Point.NbPoints(); IPntE++)
        {
            HatchGen_PointOnElement PntE = Point.Point(IPntE);

            SegmentBegin = SegmentBegin || PntE.SegmentBeginning();
            SegmentEnd = SegmentEnd || PntE.SegmentEnd();

            Geom2dHatch_Element Element = myElements.Find(PntE.Index());
            CurveAdaptor2D CurveE = Element.Curve();

            Orientation ElementOrientation = Element.Orientation();
            bool ToReverse = (ElementOrientation == Reversed);
            double Param;
            switch (PntE.Position())
            {
                case Forward:
                    Param = ToReverse ? CurveE.LastParameter() : CurveE.FirstParameter();
                    break;

                case Internal:
                    Param = PntE.Parameter();
                    break;

                case Reversed:
                    Param = ToReverse ? CurveE.FirstParameter() : CurveE.LastParameter();
                    break;

                default:
                    Param = 0.0; // uninitialised in OCCT, and overwritten below
                    break;
            }

            // TODO: finding #25 - this overwrites Param computed by the switch above,
            // making the position-based logic dead code. Removing this line caused
            // blend regressions. The switch logic may need endpoint-specific investigation.
            _ = Param;
            Param = PntE.Parameter(); // OCCT: overwrites the parameter chosen above (occt-issues.md)

            // As for the hatching above, OCCT passes CurveE.Curve()
            myIntersector.LocalGeometry(CurveE, Param, out Tangente2d, out Normale2d, out Courbure);

            //-----------------------------------------------------------------------
            // Calcul de la transition locale. On suppose les relations suivantes :
            //  - Si l orientation de l element est INTERNAL ==> INTERNAL
            //  - Si l orientation de l element est EXTERNAL ==> EXTERNAL
            //  - Si tangence, on a IN-IN  ou OUT-OUT ==> INTERNAL/EXTERNAL
            //  - Sinon,       on a IN-OUT ou OUT-IN  ==> REVERSED/FORWARD
            // Les deux dernieres conditions avec l element vu en FORWARD.
            //-----------------------------------------------------------------------
            Orientation LocalTransition = External;

            if (ElementOrientation == Internal)
            {
                LocalTransition = Internal;
            }
            else if (ElementOrientation == External)
            {
                LocalTransition = External;
            }
            else if (PntE.IntersectionType() == HatchGen_TANGENT)
            {
                if (PntE.Position() == Internal)
                {
                    switch (PntE.StateBefore())
                    {
                        case In:
                            LocalTransition = ToReverse ? External : Internal;
                            break;
                        case Out:
                            LocalTransition = ToReverse ? Internal : External;
                            break;
                        default:
                            break;
                    }
                }
                else
                {
                    switch (PntE.StateBefore())
                    {
                        case In:
                            LocalTransition = ToReverse ? Forward : Reversed;
                            break;
                        case Out:
                            LocalTransition = ToReverse ? Reversed : Forward;
                            break;
                        default:
                            break;
                    }
                }
            }
            else
            {
                switch (PntE.StateBefore())
                {
                    case In:
                        LocalTransition = ToReverse ? Forward : Reversed;
                        break;
                    case Out:
                        LocalTransition = ToReverse ? Reversed : Forward;
                        break;
                    default:
                        break;
                }
            }

            //-----------------------------------------------------------------------
            // Orientation de la tangente au point d interference.
            //-----------------------------------------------------------------------
            Orientation TangenteOrientation = Forward;
            switch (PntE.Position())
            {
                case Forward:
                    TangenteOrientation = ToReverse ? Reversed : Forward;
                    break;
                case Internal:
                    TangenteOrientation = Internal;
                    break;
                case Reversed:
                    TangenteOrientation = ToReverse ? Forward : Reversed;
                    break;

                default:
                    break;
            }

            //-----------------------------------------------------------------------
            // Proprietes geometriques.
            //-----------------------------------------------------------------------

            if (ToReverse)
            {
                Tangente.SetCoord(-Tangente2d.X(), -Tangente2d.Y(), 0.0);
            }
            else
            {
                Tangente.SetCoord(Tangente2d.X(), Tangente2d.Y(), 0.0);
            }
            Normale.SetCoord(Normale2d.X(), Normale2d.Y(), 0.0);

            ComplexTransition.Compare(global::Kerf.Engine.Foundation.Precision.Angular(),
                                      Tangente,
                                      Normale,
                                      Courbure,
                                      LocalTransition,
                                      TangenteOrientation);
        }

        switch (ComplexTransition.StateBefore())
        {
            case In:
                StateBefore = In;
                break;
            case Out:
                StateBefore = Out;
                break;
            case On:
                return false;
            case Unknown:
                return false;
        }
        switch (ComplexTransition.StateAfter())
        {
            case In:
                StateAfter = In;
                break;
            case Out:
                StateAfter = Out;
                break;
            case On:
                return false;
            case Unknown:
                return false;
        }

        Point.SetStateBefore(StateBefore);
        Point.SetStateAfter(StateAfter);
        Point.SetSegmentBeginning(SegmentBegin);
        Point.SetSegmentEnd(SegmentEnd);
        return true;
    }

    /// <summary>Computes the domains of all the hatchings.</summary>
    public void ComputeDomains()
    {
        for (int IndH = 1; IndH <= myNbHatchings; IndH++)
        {
            if (myHatchings.IsBound(IndH))
            {
                ComputeDomains(IndH);
            }
        }
    }

    /// <summary>
    /// Computes the domains of the <paramref name="IndH"/>-th hatching, trimming it first if needed. A hatching with
    /// no intersection point is one domain or none, by the classification of a point of it.
    /// </summary>
    public void ComputeDomains(int IndH)
    {
        Geom2dHatch_Hatching Hatching = myHatchings.ChangeFind(IndH);
        Hatching.ClrDomains();

        Hatching.IsDone(false);

        if (!Hatching.TrimDone())
        {
            Trim(IndH);
        }

        if (Hatching.Status() != HatchGen_NoProblem)
        {
            return;
        }

        bool Points = myKeepPoints;
        bool Segments = myKeepSegments;
        int ISav = 0;
        bool SavPnt = false;
        int NbOpenedSegments = 0;
        int NbPnt = Hatching.NbPoints();
        int IPnt = 1;

        if (NbPnt == 0)
        {
            //-- std::cout << "The hatching # " << std::setw(3) << IndH << " has to be classified" <<
            // std::endl ;
            Geom2dHatch_Classifier Classifier = new(myElements, Hatching.ClassificationPoint(), 0.0000001);
            if (Classifier.State() == In)
            {
                HatchGen_Domain domain0 = new();
                Hatching.AddDomain(domain0);
            }

            Hatching.IsDone(true);
            return;
        }

        // for (int IPnt = 1 ; IPnt <= NbPnt ; IPnt++) {
        for (IPnt = 1; IPnt <= NbPnt; IPnt++)
        {
            bool NoDomain = Hatching.NbDomains() == 0;
            bool FirstPoint = IPnt == 1;
            bool LastPoint = IPnt == NbPnt;

            HatchGen_PointOnHatching CurPnt = Hatching.Point(IPnt);

            //-----------------------------------------------------------------------
            // Calcul des domaines.
            //-----------------------------------------------------------------------

            Containment StateBefore = CurPnt.StateBefore();
            Containment StateAfter = CurPnt.StateAfter();
            bool SegmentBegin = CurPnt.SegmentBeginning();
            bool SegmentEnd = CurPnt.SegmentEnd();

            HatchGen_Domain domain = new();

            //-----------------------------------------------------------------------
            // Initialisations dues au premier point.
            //-----------------------------------------------------------------------

            if (FirstPoint)
            {
                SavPnt = false;
                ISav = 0;
                NbOpenedSegments = 0;
                if (SegmentEnd && SegmentBegin)
                {
                    if (StateAfter == Unknown)
                    {
                        StateAfter = In;
                    }
                    if (StateBefore == Unknown)
                    {
                        StateBefore = In;
                    }

                    if (Segments)
                    {
                        SavPnt = true;
                        ISav = 0;
                    }
                }
                else if (SegmentEnd)
                {
                    if (StateAfter == Unknown)
                    {
                        StateAfter = In;
                    }

                    if (Segments)
                    {
                        SavPnt = true;
                        ISav = 0;
                    }
                }
                else if (SegmentBegin)
                {
                    if (StateBefore == Unknown)
                    {
                        StateBefore = In;
                    }
                    if (StateBefore == In)
                    {
                        SavPnt = true;
                        ISav = 0;
                    }
                }
                else
                {
                    if (StateBefore == In)
                    {
                        SavPnt = true;
                        ISav = 0;
                    }
                }
            }

            //-----------------------------------------------------------------------
            // Initialisations dues au dernier point.
            //-----------------------------------------------------------------------

            if (LastPoint)
            {
                if (SegmentEnd && SegmentBegin)
                {
                    if (StateAfter == Unknown)
                    {
                        StateAfter = In;
                    }

                    if (StateBefore == Unknown)
                    {
                        StateBefore = In;
                    }
                }
                else if (SegmentEnd)
                {
                    if (StateAfter == Unknown)
                    {
                        StateAfter = In;
                    }
                }
                else if (SegmentBegin)
                {
                    if (StateBefore == Unknown)
                    {
                        StateBefore = In;
                    }
                }
                else
                {
                }
            }

            //-----------------------------------------------------------------------
            // Cas general.
            //-----------------------------------------------------------------------

            bool ToAppend = false;

            if (SegmentEnd && SegmentBegin)
            {
                if (StateBefore != In && StateAfter != In)
                {
                    Hatching.Status(HatchGen_IncompatibleStates);
                    return;
                }

                if (Points)
                {
                    if (Segments)
                    {
                        if (!SavPnt)
                        {
                            if (NoDomain)
                            {
                                Hatching.Status(HatchGen_IncoherentParity);
                            }
                            else
                            {
                                Hatching.IsDone(true);
                            }
                            return;
                        }

                        if (ISav != 0)
                        {
                            domain.SetFirstPoint(Hatching.Point(ISav));
                        }

                        domain.SetSecondPoint(CurPnt);
                        ToAppend = true;
                        SavPnt = true;
                        ISav = IPnt;
                    }
                    else
                    {
                        bool isININ = (StateBefore == In && StateAfter == In);
                        if (SavPnt && !isININ)
                        {
                            if (NoDomain)
                            {
                                Hatching.Status(HatchGen_IncoherentParity);
                            }
                            else
                            {
                                Hatching.IsDone(true);
                            }

                            return;
                        }

                        domain.SetPoints(CurPnt, CurPnt);
                        ToAppend = true;
                        SavPnt = false;
                        ISav = 0;
                    }
                }
            }
            else if (SegmentEnd)
            {
                if (Segments)
                {
                    if (StateAfter == Out)
                    {
                        if (!SavPnt)
                        {
                            if (NoDomain)
                            {
                                Hatching.Status(HatchGen_IncoherentParity);
                            }
                            else
                            {
                                Hatching.IsDone(true);
                            }
                            return;
                        }

                        if (ISav != 0)
                        {
                            domain.SetFirstPoint(Hatching.Point(ISav));
                        }

                        domain.SetSecondPoint(CurPnt);
                        ToAppend = true;
                    }
                    else
                    {
                        if (Points)
                        {
                            if (ISav != 0)
                            {
                                domain.SetFirstPoint(Hatching.Point(ISav));
                            }

                            domain.SetSecondPoint(CurPnt);
                            ToAppend = true;
                            SavPnt = true;
                            ISav = IPnt;
                        }
                    }
                }
                else
                {
                    if (StateAfter == In)
                    {
                        SavPnt = true;
                        ISav = IPnt;
                    }
                }

                NbOpenedSegments--;
            }
            else if (SegmentBegin)
            {
                if (Segments)
                {
                    if (StateBefore == Out)
                    {
                        SavPnt = true;
                        ISav = IPnt;
                    }
                    else
                    {
                        if (Points)
                        {
                            if (!SavPnt)
                            {
                                if (NoDomain)
                                {
                                    Hatching.Status(HatchGen_IncoherentParity);
                                }
                                else
                                {
                                    Hatching.IsDone(true);
                                }

                                return;
                            }

                            if (ISav != 0)
                            {
                                domain.SetFirstPoint(Hatching.Point(ISav));
                            }

                            domain.SetSecondPoint(CurPnt);
                            ToAppend = true;
                            SavPnt = true;
                            ISav = IPnt;
                        }
                    }
                }
                else
                {
                    if (StateBefore == In)
                    {
                        if (!SavPnt)
                        {
                            if (NoDomain)
                            {
                                Hatching.Status(HatchGen_IncoherentParity);
                            }
                            else
                            {
                                Hatching.IsDone(true);
                            }

                            return;
                        }

                        if (ISav != 0)
                        {
                            domain.SetFirstPoint(Hatching.Point(ISav));
                        }

                        domain.SetSecondPoint(CurPnt);
                        ToAppend = true;

                        // Modified by Sergey KHROMOV - Fri Jan  5 12:05:30 2001
                        // SavPnt = false ;
                        // ISav = 0 ;

                        SavPnt = true;
                        ISav = IPnt;
                        // Modified by Sergey KHROMOV - Fri Jan  5 12:05:31 2001
                    }
                }

                NbOpenedSegments++;
            }
            else
            {
                //-- ???????????????????????????????????????????????????????????????????????????
                //-- Solution provisoire (lbr le 11 Aout 97 )
                //-- si On a 2 points dont des points OUT OUT ou IN IN qui delimitent une isos
                //-- on transforme les transitions
                if (StateBefore == Out && StateAfter == Out)
                {
                    if (NbPnt == 2)
                    {
                        if (FirstPoint)
                        {
                            StateAfter = In;
                        }
                        else
                        {
                            StateBefore = In;
                        }
                    }
                }
                //-- ???????????????????????????????????????????????????????????????????????????
                if (StateBefore == Out && StateAfter == Out)
                {
                    if (SavPnt)
                    {
                        if (NoDomain)
                        {
                            Hatching.Status(HatchGen_IncoherentParity);
                        }
                        else
                        {
                            Hatching.IsDone(true);
                        }

                        return;
                    }

                    if (Points)
                    {
                        domain.SetPoints(CurPnt, CurPnt);
                        ToAppend = true;
                        SavPnt = true;
                        ISav = IPnt;
                    }
                }
                else if (StateBefore == Out && StateAfter == In)
                {
                    SavPnt = true;
                    ISav = IPnt;
                }
                else if (StateBefore == In && StateAfter == Out)
                {
                    if (!SavPnt)
                    {
                        if (NoDomain)
                        {
                            Hatching.Status(HatchGen_IncoherentParity);
                        }
                        else
                        {
                            Hatching.IsDone(true);
                        }

                        return;
                    }

                    if (ISav != 0)
                    {
                        domain.SetFirstPoint(Hatching.Point(ISav));
                    }

                    domain.SetSecondPoint(CurPnt);
                    ToAppend = true;
                    SavPnt = false;
                    ISav = 0;
                }
                else if (StateBefore == In && StateAfter == In)
                {
                    if (Points)
                    {
                        if (NbOpenedSegments == 0)
                        {
                            if (!SavPnt)
                            {
                                if (NoDomain)
                                {
                                    Hatching.Status(HatchGen_IncoherentParity);
                                }
                                else
                                {
                                    Hatching.IsDone(true);
                                }

                                // return;
                                continue;
                            }

                            if (ISav != 0)
                            {
                                domain.SetFirstPoint(Hatching.Point(ISav));
                            }

                            domain.SetSecondPoint(CurPnt);
                            ToAppend = true;
                            SavPnt = true;
                            ISav = IPnt;
                        }
                        else
                        {
                            if (Segments)
                            {
                                if (!SavPnt)
                                {
                                    if (NoDomain)
                                    {
                                        Hatching.Status(HatchGen_IncoherentParity);
                                    }
                                    else
                                    {
                                        Hatching.IsDone(true);
                                    }

                                    return;
                                }

                                if (ISav != 0)
                                {
                                    domain.SetFirstPoint(Hatching.Point(ISav));
                                }

                                domain.SetSecondPoint(CurPnt);
                                ToAppend = true;
                                SavPnt = true;
                                ISav = IPnt;
                            }
                            else
                            {
                                if (SavPnt)
                                {
                                    if (NoDomain)
                                    {
                                        Hatching.Status(HatchGen_IncoherentParity);
                                    }
                                    else
                                    {
                                        Hatching.IsDone(true);
                                    }

                                    return;
                                }

                                domain.SetPoints(CurPnt, CurPnt);
                                ToAppend = true;
                                SavPnt = false;
                                ISav = 0;
                            }
                        }
                    }
                }
                else
                {
                    Hatching.Status(HatchGen_IncompatibleStates);
                    return;
                }
            }

            //-----------------------------------------------------------------------
            // Ajout du domaine.
            //-----------------------------------------------------------------------

            if (ToAppend)
            {
                Hatching.AddDomain(domain);
            }

            //-----------------------------------------------------------------------
            // Traitement lie au dernier point.
            //-----------------------------------------------------------------------

            if (LastPoint)
            {
                domain.SetPoints();
                ToAppend = false;

                if (SegmentEnd && SegmentBegin)
                {
                    if (Segments)
                    {
                        if (!SavPnt)
                        {
                            if (NoDomain)
                            {
                                Hatching.Status(HatchGen_IncoherentParity);
                            }
                            else
                            {
                                Hatching.IsDone(true);
                            }

                            return;
                        }

                        if (ISav != 0)
                        {
                            domain.SetFirstPoint(Hatching.Point(ISav));
                        }

                        ToAppend = true;
                    }
                }
                else if (SegmentEnd)
                {
                    if (StateAfter == In)
                    {
                        if (!SavPnt)
                        {
                            if (NoDomain)
                            {
                                Hatching.Status(HatchGen_IncoherentParity);
                            }
                            else
                            {
                                Hatching.IsDone(true);
                            }

                            return;
                        }

                        if (ISav != 0)
                        {
                            domain.SetFirstPoint(Hatching.Point(ISav));
                        }

                        ToAppend = true;
                    }
                }
                else if (SegmentBegin)
                {
                    if (Segments)
                    {
                        if (!SavPnt)
                        {
                            if (NoDomain)
                            {
                                Hatching.Status(HatchGen_IncoherentParity);
                            }
                            else
                            {
                                Hatching.IsDone(true);
                            }

                            return;
                        }

                        if (ISav != 0)
                        {
                            domain.SetFirstPoint(Hatching.Point(ISav));
                        }

                        ToAppend = true;
                    }
                }
                else
                {
                    if (StateAfter == In)
                    {
                        if (!SavPnt)
                        {
                            if (NoDomain)
                            {
                                Hatching.Status(HatchGen_IncoherentParity);
                            }
                            else
                            {
                                Hatching.IsDone(true);
                            }

                            return;
                        }

                        if (ISav != 0)
                        {
                            domain.SetFirstPoint(Hatching.Point(ISav));
                        }

                        ToAppend = true;
                    }
                }

                if (ToAppend)
                {
                    Hatching.AddDomain(domain);
                }
            }
        }

        Hatching.IsDone(true);
    }

    //=======================================================================
    //  Category : Results.
    //=======================================================================

    /// <summary>True if the intersections of the <paramref name="IndH"/>-th hatching were computed.</summary>
    public bool TrimDone(int IndH)
    {
        Geom2dHatch_Hatching Hatching = myHatchings.Find(IndH);
        return Hatching.TrimDone();
    }

    /// <summary>True if an intersection of the <paramref name="IndH"/>-th hatching failed.</summary>
    public bool TrimFailed(int IndH)
    {
        Geom2dHatch_Hatching Hatching = myHatchings.Find(IndH);
        return Hatching.TrimFailed();
    }

    /// <summary>True if the domains of the <paramref name="IndH"/>-th hatching were computed.</summary>
    public bool IsDone(int IndH)
    {
        Geom2dHatch_Hatching Hatching = myHatchings.Find(IndH);
        return Hatching.IsDone();
    }

    /// <summary>The status of the <paramref name="IndH"/>-th hatching.</summary>
    public HatchGen_ErrorStatus Status(int IndH)
    {
        Geom2dHatch_Hatching Hatching = myHatchings.Find(IndH);
        return Hatching.Status();
    }

    /// <summary>The number of domains of the <paramref name="IndH"/>-th hatching.</summary>
    /// <exception cref="NotDoneException">The domains are not computed.</exception>
    public int NbDomains(int IndH)
    {
        Geom2dHatch_Hatching Hatching = myHatchings.Find(IndH);
        StdFail_NotDone_Raise_if(!Hatching.IsDone(), "Geom2dHatch_Hatcher::NbDomains");
        return Hatching.NbDomains();
    }

    /// <summary>The <paramref name="IDom"/>-th domain of the <paramref name="IndH"/>-th hatching.</summary>
    /// <exception cref="NotDoneException">The domains are not computed.</exception>
    public HatchGen_Domain Domain(int IndH, int IDom)
    {
        Geom2dHatch_Hatching Hatching = myHatchings.Find(IndH);
        StdFail_NotDone_Raise_if(!Hatching.IsDone(), "Geom2dHatch_Hatcher::Domain");
        HatchGen_Domain Domain = Hatching.Domain(IDom);
        return Domain;
    }

    /// <summary>The <paramref name="IDom"/>-th domain of the <paramref name="IndH"/>-th hatching, for writing (OCCT.NET:
    /// ChFi3d_Builder_SpKP's <c>Tri</c> casts the constness of <see cref="Domain"/> away and edits it in place).</summary>
    internal ref HatchGen_Domain ChangeDomain(int IndH, int IDom)
    {
        Geom2dHatch_Hatching Hatching = myHatchings.Find(IndH);
        StdFail_NotDone_Raise_if(!Hatching.IsDone(), "Geom2dHatch_Hatcher::Domain");
        return ref Hatching.ChangeDomain(IDom);
    }

    //=======================================================================
    //  Category : Dump.
    //=======================================================================

    /// <summary>Writes the hatcher (its tolerances, hatchings and elements) to <paramref name="theOS"/>, as OCCT writes it to std::cout.</summary>
    public void Dump(TextWriter theOS)
    {
        theOS.Write('\n');
        theOS.Write("========================================================\n");
        theOS.Write("=== Dump of the hatcher ================================\n");
        theOS.Write("========================================================\n");
        theOS.Write('\n');

        theOS.Write($"The points   are {(myKeepPoints ? "    " : "not ")}considered.\n");
        theOS.Write($"The segments are {(myKeepSegments ? "    " : "not ")}considered.\n");
        theOS.Write($"2D Confusion tolerance : {HatchGen_IntersectionPoint.Number(myConfusion2d)}\n");
        theOS.Write($"3D Confusion tolerance : {HatchGen_IntersectionPoint.Number(myConfusion3d)}\n");

        theOS.Write($"{myNbHatchings} hatching{((myNbHatchings == 1) ? "" : "s")}\n");
        theOS.Write($"{myNbElements} element{((myNbElements == 1) ? "" : "s")}\n");

        theOS.Write('\n');
        theOS.Write("========================================================\n");
        theOS.Write("=== Hatchings ==========================================\n");
        theOS.Write("========================================================\n");
        theOS.Write('\n');

        for (int IndH = 1; IndH <= myNbHatchings; IndH++)
        {
            theOS.Write($"Hatching # {IndH}");
            if (!myHatchings.IsBound(IndH))
            {
                theOS.Write(" is not bound\n");
            }
            else
            {
                Geom2dHatch_Hatching Hatching = myHatchings.Find(IndH);
                int NbPnt = Hatching.NbPoints();
                theOS.Write($" contains {NbPnt} restriction points :\n");
                for (int IPnt = 1; IPnt <= NbPnt; IPnt++)
                {
                    HatchGen_PointOnHatching PntH = Hatching.Point(IPnt);
                    PntH.Dump(theOS, IPnt);
                }
                theOS.Write("----------------------------------------------\n");
            }
        }

        theOS.Write('\n');
        theOS.Write("========================================================\n");
        theOS.Write("=== Elements ===========================================\n");
        theOS.Write("========================================================\n");
        theOS.Write('\n');

        for (int IndE = 1; IndE <= myNbElements; IndE++)
        {
            theOS.Write($"Element # {IndE}");
            if (!myElements.IsBound(IndE))
            {
                theOS.Write(" is not bound\n");
            }
            else
            {
                Geom2dHatch_Element Element = myElements.Find(IndE);
                switch (Element.Orientation())
                {
                    case Forward:
                        theOS.Write(" is FORWARD\n");
                        break;
                    case Reversed:
                        theOS.Write(" is REVERSED\n");
                        break;
                    case Internal:
                        theOS.Write(" is INTERNAL\n");
                        break;
                    case External:
                        theOS.Write(" is EXTERNAL\n");
                        break;
                }
            }
        }

        theOS.Write('\n');
    }
}
