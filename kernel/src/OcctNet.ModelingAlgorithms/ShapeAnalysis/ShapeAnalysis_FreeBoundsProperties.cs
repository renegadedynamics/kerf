// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBoundsProperties.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBoundsProperties.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_FreeBoundsProperties.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The properties of the free bounds of a shape (found by <see cref="ShapeAnalysis_FreeBounds"/>): for each contour,
/// its area and perimeter, the ratio of its average length to its average width, its average width, and its notches
/// (narrow V-like sub-contours) with their maximum widths, kept in <see cref="ShapeAnalysis_FreeBoundData"/>.
/// </summary>
/// <remarks>
/// A public working object (decision 21; OCCT uses it by value). It only reads the shape; the free bounds and the
/// notches it hands out are frozen.
/// </remarks>
public sealed class ShapeAnalysis_FreeBoundsProperties
{
    private const int NbControl = 23;

    private TopoDS_Shape myShape = new();
    private double myTolerance;
    private bool mySplitClosed;
    private bool mySplitOpen;
    private readonly NCollection_HSequence<ShapeAnalysis_FreeBoundData> myClosedFreeBounds;
    private readonly NCollection_HSequence<ShapeAnalysis_FreeBoundData> myOpenFreeBounds;

    private static void ContourProperties(in TopoDS_Wire wire, out double countourArea, out double countourLength)
    {
        int nbe = 0;
        double length = 0.0;
        gp_XYZ area = new(0.0, 0.0, 0.0);
        gp_XYZ prev = new(), cont = new();
        for (var exp = new BRepTools_WireExplorer(wire); exp.More(); exp.Next())
        {
            TopoDS_Edge Edge = exp.Current();
            nbe++;
            var sae = new ShapeAnalysis_Edge();
            if (!sae.Curve3d(Edge, out Geom_Curve? c3d, out double First, out double Last))
            {
                continue;
            }
            int ibeg = 0;
            if (nbe == 1)
            {
                gp_Pnt pntIni = c3d!.Value(First);
                prev = pntIni.XYZ();
                cont = prev;
                ibeg = 1;
            }
            for (int i = ibeg; i < NbControl; i++)
            {
                double prm = ((NbControl - 1 - i) * First + i * Last) / (NbControl - 1);
                gp_Pnt pntCurr = c3d!.Value(prm);
                gp_XYZ curr = pntCurr.XYZ();
                gp_XYZ delta = curr - prev;
                length += delta.Modulus();
                area += curr ^ prev;
                prev = curr;
            }
        }
        area += cont ^ prev;
        countourArea = area.Modulus() / 2;
        countourLength = length;
    }

    /// <summary>An empty analyser.</summary>
    public ShapeAnalysis_FreeBoundsProperties()
    {
        myClosedFreeBounds = new NCollection_HSequence<ShapeAnalysis_FreeBoundData>();
        myOpenFreeBounds = new NCollection_HSequence<ShapeAnalysis_FreeBoundData>();
        myTolerance = 0.0;
    }

    /// <summary>An analyser of the free bounds of a compound of faces, sewn with the tolerance (see <see cref="Init(in TopoDS_Shape, double, bool, bool)"/>).</summary>
    public ShapeAnalysis_FreeBoundsProperties(in TopoDS_Shape shape, double tolerance, bool splitclosed = false, bool splitopen = false)
    {
        myClosedFreeBounds = new NCollection_HSequence<ShapeAnalysis_FreeBoundData>();
        myOpenFreeBounds = new NCollection_HSequence<ShapeAnalysis_FreeBoundData>();
        Init(shape, tolerance, splitclosed, splitopen);
    }

    /// <summary>An analyser of the free bounds of a compound of shells (see <see cref="Init(in TopoDS_Shape, bool, bool)"/>).</summary>
    public ShapeAnalysis_FreeBoundsProperties(in TopoDS_Shape shape, bool splitclosed = false, bool splitopen = false)
    {
        myClosedFreeBounds = new NCollection_HSequence<ShapeAnalysis_FreeBoundData>();
        myOpenFreeBounds = new NCollection_HSequence<ShapeAnalysis_FreeBoundData>();
        myTolerance = 0.0;
        Init(shape, splitclosed, splitopen);
    }

    /// <summary>Initializes with a compound of faces, sewn with the tolerance (the options as <see cref="ShapeAnalysis_FreeBounds"/>'s).</summary>
    public void Init(in TopoDS_Shape shape, double tolerance, bool splitclosed = false, bool splitopen = false)
    {
        Init(shape, splitclosed, splitopen);
        myTolerance = tolerance;
    }

    /// <summary>Initializes with a compound of shells (the options as <see cref="ShapeAnalysis_FreeBounds"/>'s).</summary>
    public void Init(in TopoDS_Shape shape, bool splitclosed = false, bool splitopen = false)
    {
        myShape = shape;
        mySplitClosed = splitclosed;
        mySplitOpen = splitopen;
    }

    /// <summary>
    /// Builds the free bounds and computes each one's properties (area, perimeter, ratio, average width, notches and
    /// their widths). True if no step failed and free bounds were found.
    /// </summary>
    public bool Perform()
    {
        bool result = false;
        result |= DispatchBounds();
        result |= CheckNotches();
        result |= CheckContours();
        return result;
    }

    /// <summary>True if a shape is loaded.</summary>
    public bool IsLoaded() => !(myShape.IsNull());

    /// <summary>The shape.</summary>
    public TopoDS_Shape Shape() => myShape;

    /// <summary>The tolerance (0 for a compound of shells).</summary>
    public double Tolerance() => myTolerance;

    /// <summary>The number of free bounds.</summary>
    public int NbFreeBounds() => (myClosedFreeBounds.Length() + myOpenFreeBounds.Length());

    /// <summary>The number of closed free bounds.</summary>
    public int NbClosedFreeBounds() => myClosedFreeBounds.Length();

    /// <summary>The number of open free bounds.</summary>
    public int NbOpenFreeBounds() => myOpenFreeBounds.Length();

    /// <summary>The closed free bounds (the object's own sequence, as OCCT's handle).</summary>
    public NCollection_HSequence<ShapeAnalysis_FreeBoundData> ClosedFreeBounds() => myClosedFreeBounds;

    /// <summary>The open free bounds (the object's own sequence, as OCCT's handle).</summary>
    public NCollection_HSequence<ShapeAnalysis_FreeBoundData> OpenFreeBounds() => myOpenFreeBounds;

    /// <summary>The closed free bound of the given rank.</summary>
    public ShapeAnalysis_FreeBoundData ClosedFreeBound(int index) => myClosedFreeBounds.Value(index);

    /// <summary>The open free bound of the given rank.</summary>
    public ShapeAnalysis_FreeBoundData OpenFreeBound(int index) => myOpenFreeBounds.Value(index);

    /// <summary>Builds the free bounds and records the closed and open ones. False if no shape is loaded.</summary>
    public bool DispatchBounds()
    {
        if (!IsLoaded())
        {
            return false;
        }

        TopoDS_Compound tmpClosedBounds, tmpOpenBounds;
        if (myTolerance > 0.0)
        {
            var safb = new ShapeAnalysis_FreeBounds(myShape, myTolerance, mySplitClosed, mySplitOpen);
            tmpClosedBounds = safb.GetClosedWires();
            tmpOpenBounds = safb.GetOpenWires();
        }
        else
        {
            var safb = new ShapeAnalysis_FreeBounds(myShape, mySplitClosed, mySplitOpen);
            tmpClosedBounds = safb.GetClosedWires();
            tmpOpenBounds = safb.GetOpenWires();
        }

        var shexpl = new ShapeExtend_Explorer();
        NCollection_HSequence<TopoDS_Shape> tmpSeq = shexpl.SeqFromCompound(tmpClosedBounds, false);
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= tmpSeq.Length(); i++)
        {
            TopoDS_Wire wire = TopoDS.Wire(tmpSeq.Value(i));
            var fbData = new ShapeAnalysis_FreeBoundData();
            fbData.SetFreeBound(wire);
            myClosedFreeBounds.Append(fbData);
        }

        NCollection_HSequence<TopoDS_Shape> tmpSeq2 = shexpl.SeqFromCompound(tmpOpenBounds, false);
        for (i = 1; i <= tmpSeq2.Length(); i++)
        {
            TopoDS_Wire wire = TopoDS.Wire(tmpSeq2.Value(i));
            var fbData = new ShapeAnalysis_FreeBoundData();
            fbData.SetFreeBound(wire);
            myOpenFreeBounds.Append(fbData);
        }

        return true;
    }

    /// <summary>Finds the notches of every free bound.</summary>
    public bool CheckNotches(double prec = 0.0)
    {
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= myClosedFreeBounds.Length(); i++)
        {
            ShapeAnalysis_FreeBoundData fbData = myClosedFreeBounds.Value(i);
            CheckNotches(fbData, prec);
        }
        for (i = 1; i <= myOpenFreeBounds.Length(); i++)
        {
            ShapeAnalysis_FreeBoundData fbData = myOpenFreeBounds.Value(i);
            CheckNotches(fbData, prec);
        }
        return true;
    }

    /// <summary>Finds the notches of a free bound and adds them to its data.</summary>
    public bool CheckNotches(ShapeAnalysis_FreeBoundData fbData, double prec = 0.0)
    {
        var swd = new ShapeExtend_WireData(fbData.FreeBound());
        if (swd.NbEdges() > 1)
        {
            for (int j = 1; j <= swd.NbEdges(); j++)
            {
                if (CheckNotches(fbData.FreeBound(), j, out TopoDS_Wire notch, out double dMax, prec))
                {
                    fbData.AddNotch(notch, dMax);
                }
            }
        }
        return true;
    }

    /// <summary>Computes the properties of every free bound.</summary>
    public bool CheckContours(double prec = 0.0)
    {
        bool status = false;
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= myClosedFreeBounds.Length(); i++)
        {
            ShapeAnalysis_FreeBoundData fbData = myClosedFreeBounds.Value(i);
            status |= FillProperties(fbData, prec);
        }
        for (i = 1; i <= myOpenFreeBounds.Length(); i++)
        {
            ShapeAnalysis_FreeBoundData fbData = myOpenFreeBounds.Value(i);
            status |= FillProperties(fbData, prec);
        }
        return status;
    }

    /// <summary>
    /// True if the edge <paramref name="num"/> of the wire and the next one (skipping a small edge between them)
    /// make a notch: they turn back by more than 0.95 pi. <paramref name="notch"/> is the wire of those edges (frozen;
    /// always built), <paramref name="distMax"/> the largest distance from 23 points of the first to the second (0
    /// when false).
    /// </summary>
    public bool CheckNotches(in TopoDS_Wire wire, int num, out TopoDS_Wire notch, out double distMax, double prec = 0.0)
    {
        distMax = 0.0; // OCCT.NET: OCCT leaves it unset when it returns false
        bool aRes = checkNotches(wire, num, out notch, ref distMax);
        notch.TShape()?.FreezeGraph();
        return aRes;
    }

    private bool checkNotches(in TopoDS_Wire wire, int num, out TopoDS_Wire notch, ref double distMax)
    {
        double tol = Max(myTolerance, Occt.Precision.Confusion());
        var wdt = new ShapeExtend_WireData(wire);
        BRep_Builder B = new();
        notch = new TopoDS_Wire();
        B.MakeWire(ref notch);

        if ((num <= 0) || (num > wdt.NbEdges()))
        {
            return false;
        }

        int n1 = (num > 0 ? num : wdt.NbEdges());
        int n2 = (n1 < wdt.NbEdges() ? n1 + 1 : 1);

        TopoDS_Edge E1 = wdt.Edge(n1);
        B.Add(notch, E1);

        var saw = new ShapeAnalysis_Wire();
        saw.Load(wdt);
        saw.SetPrecision(myTolerance);
        if (saw.CheckSmall(n2, tol))
        {
            B.Add(notch, wdt.Edge(n2));
            n2 = (n2 < wdt.NbEdges() ? n2 + 1 : 1);
        }

        TopoDS_Edge E2 = wdt.Edge(n2);
        B.Add(notch, E2);

        var sae = new ShapeAnalysis_Edge();
        // szv#4:S4163:12Mar99 optimized
        if (!sae.Curve3d(E1, out Geom_Curve? c3d1, out double First1, out double Last1)
            || !sae.Curve3d(E2, out Geom_Curve? c3d2, out double First2, out double Last2))
        {
            return false;
        }

        c3d1!.D1(Last1, out _, out gp_Vec vec1);
        c3d2!.D1(First2, out _, out gp_Vec vec2);
        if (E1.Orientation() == TopAbs_REVERSED)
        {
            vec1.Reverse();
        }
        if (E2.Orientation() == TopAbs_REVERSED)
        {
            vec2.Reverse();
        }

        double angl = Math.Abs(vec1.Angle(vec2));
        if (angl > 0.95 * M_PI)
        {
            distMax = 0.0;
            for (int i = 0; i < NbControl; i++)
            {
                double prm = ((NbControl - 1 - i) * First1 + i * Last1) / (NbControl - 1);
                gp_Pnt pntCurr = c3d1.Value(prm);

                double p1, p2;
                if (First2 < Last2)
                {
                    p1 = First2;
                    p2 = Last2;
                }
                else
                {
                    p1 = Last2;
                    p2 = First2;
                }

                // szv#4:S4163:12Mar99 warning
                var ppc = new GeomAPI_ProjectPointOnCurve(pntCurr, c3d2, p1, p2);
                double newDist = (ppc.NbPoints() != 0 ? ppc.LowerDistance() : 0);
                if (newDist > distMax)
                {
                    distMax = newDist;
                }
            }

            return true;
        }

        return false;
    }

    /// <summary>Computes the area, perimeter, ratio and average width of a free bound into its data.</summary>
    public bool FillProperties(ShapeAnalysis_FreeBoundData fbData, double prec = 0.0)
    {
        ContourProperties(fbData.FreeBound(), out double area, out double length);

        double r = 0;
        double aver = 0;

        if (length != 0.0)
        {
            // szv#4:S4163:12Mar99 anti-exception
            double k = area / (length * length); // szv#4:S4163:12Mar99
            // szv#4:S4163:12Mar99 optimized
            if (k != 0.0)
            {
                // szv#4:S4163:12Mar99 anti-exception
                double aux = 1.0 - 16.0 * k;
                if (aux >= 0.0)
                {
                    r = (1.0 + Math.Sqrt(aux)) / (8.0 * k);
                    aver = length / (2.0 * r);
                    r -= 1.0;
                }
            }
        }

        fbData.SetArea(area);
        fbData.SetPerimeter(length);
        fbData.SetRatio(r);
        fbData.SetWidth(aver);

        return true;
    }
}
