// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_InterferencePolygonPolyhedron.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Intersection.Polygons.PIType;

namespace Kerf.Engine.Intersection.Polygons;

/// <summary>
/// The static tool over a 3D polygon that the generic <c>Intf_InterferencePolygonPolyhedron</c> uses (OCCT's
/// ToolPolygon3d macro: IntCurveSurface_ThePolygonToolOfHInter, HLRBRep_ThePolygonToolOfInterCSurf).
/// </summary>
internal interface IToolPolygon3d<Polygon3d>
{
    static abstract Box Bounding(in Polygon3d polygon);

    static abstract double DeflectionOverEstimation(in Polygon3d polygon);

    static abstract bool Closed(in Polygon3d polygon);

    static abstract int NbSegments(in Polygon3d polygon);

    static abstract Point3D BeginOfSeg(in Polygon3d polygon, int index);

    static abstract Point3D EndOfSeg(in Polygon3d polygon, int index);
}

/// <summary>
/// The static tool over a polyhedron (a triangulated surface) that the generic
/// <c>Intf_InterferencePolygonPolyhedron</c> uses (OCCT's ToolPolyh macro: IntCurveSurface_ThePolyhedronToolOfHInter,
/// HLRBRep_ThePolyhedronToolOfInterCSurf).
/// </summary>
internal interface IToolPolyhedron<Polyhedron>
{
    static abstract Box Bounding(in Polyhedron polyh);

    static abstract NCollection_HArray1<Box> ComponentsBounding(in Polyhedron polyh);

    static abstract double DeflectionOverEstimation(in Polyhedron polyh);

    static abstract int NbTriangles(in Polyhedron polyh);

    static abstract void Triangle(in Polyhedron polyh, int index, out int p1, out int p2, out int p3);

    static abstract Point3D Point(in Polyhedron polyh, int index);

    static abstract int TriConnex(in Polyhedron polyh, int triang, int pivot, int pedge, out int triCon,
                                  out int otherP);

    static abstract bool IsOnBound(in Polyhedron polyh, int index1, int index2);

    static abstract double GetBorderDeflection(in Polyhedron polyh);
}

/// <summary>
/// The interference between a polygon (or straight lines) and a polyhedron: section points where the segments
/// cross the triangles, or pass within the tolerance (the sum of the deflections) of their edges.
/// </summary>
/// <remarks>
/// OCCT's generic file Intf_InterferencePolygonPolyhedron.gxx, ported once as a generic class (decision 202's
/// approach). OCCT's instantiations (IntCurveSurface_TheInterferenceOfHInter, and HLRBRep's later) are sealed
/// subclasses with OCCT's constructors forwarded. The section points use OCCT's addressing: the first element is the
/// segment index on the polygon (0 for a line, the line's index for an array of lines), the second the vertex, the
/// edge (its two vertices) or the triangle of the polyhedron. The <c>Intersect</c> variant OCCT compiles out
/// (<c>#if 0</c>) is not ported.
/// </remarks>
internal class InterferencePolygonPolyhedron<Polygon3d, ToolPolygon3d, Polyhedron, ToolPolyh> : PolygonInterference
    where ToolPolygon3d : IToolPolygon3d<Polygon3d>
    where ToolPolyh : IToolPolyhedron<Polyhedron>
{
    private static readonly int[] Pourcent3 = [0, 1, 2, 0];

    private bool _beginOfClosedPolygon;
    private int _iLin;

    private static bool IsInSegment(in Vector3D p1P2, in Vector3D p1P, double np1P2, out double param, double tolerance)
    {
        param = 0.0;
        if (np1P2 < global::Kerf.Engine.Foundation.Precision.Computational())
        {
            return false;
        }
        param = p1P2.Dot(p1P);
        param /= np1P2;
        if (param > (np1P2 + tolerance))
        {
            return (false);
        }
        if (param < (-tolerance))
        {
            return (false);
        }
        param /= np1P2;
        if (param < 0.0)
        {
            param = 0.0;
        }
        if (param > 1.0)
        {
            param = 1.0;
        }
        return (true);
    }

    public InterferencePolygonPolyhedron()
        : base(false)
    {
        _beginOfClosedPolygon = false;
        _iLin = 0;
    }

    /// <summary>Constructs and computes an interference between a polygon and a polyhedron.</summary>
    public InterferencePolygonPolyhedron(in Polygon3d polyg, in Polyhedron polyh)
        : base(false)
    {
        _beginOfClosedPolygon = false;
        _iLin = 0;
        _tolerance = ToolPolygon3d.DeflectionOverEstimation(polyg) + ToolPolyh.DeflectionOverEstimation(polyh);
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }

        if (!ToolPolygon3d.Bounding(polyg).IsOut(ToolPolyh.Bounding(polyh)))
        {
            Interference(polyg, polyh);
        }
    }

    /// <summary>
    /// Constructs and computes an interference between a polygon and a polyhedron whose triangles' boxes are
    /// sorted in <paramref name="polyhGrid"/>.
    /// </summary>
    public InterferencePolygonPolyhedron(in Polygon3d polyg, in Polyhedron polyh, BoundSortBox polyhGrid)
        : base(false)
    {
        _beginOfClosedPolygon = false;
        _iLin = 0;
        _tolerance = ToolPolygon3d.DeflectionOverEstimation(polyg) + ToolPolyh.DeflectionOverEstimation(polyh);
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }

        if (!ToolPolygon3d.Bounding(polyg).IsOut(ToolPolyh.Bounding(polyh)))
        {
            Interference(polyg, polyh, polyhGrid);
        }
    }

    /// <summary>Constructs and computes an interference between a straight line and a polyhedron.</summary>
    public InterferencePolygonPolyhedron(in Line3D lin, in Polyhedron polyh)
        : base(false)
    {
        _beginOfClosedPolygon = false;
        _iLin = 0;
        _tolerance = ToolPolyh.DeflectionOverEstimation(polyh);
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }

        _beginOfClosedPolygon = false;

        BoundSortBox polyhGrid = new();
        polyhGrid.Initialize(ToolPolyh.Bounding(polyh), ToolPolyh.ComponentsBounding(polyh));
        int indTri;

        _iLin = 0;

        Box bofLin = new();
        PolygonTool btoo = new();
        btoo.LinBox(lin, ToolPolyh.Bounding(polyh), ref bofLin);

        NCollection_List<int>.Iterator iCl = new(polyhGrid.Compare(bofLin));
        while (iCl.More())
        {
            indTri = iCl.Value();
            Intersect(lin.Location, lin.Location.Translated(new Vector3D(lin.Direction)), true, indTri,
                      polyh);
            iCl.Next();
        }
    }

    /// <summary>Constructs and computes an interference between the straight lines and a polyhedron.</summary>
    public InterferencePolygonPolyhedron(NCollection_Array1<Line3D> lins, in Polyhedron polyh)
        : base(false)
    {
        _beginOfClosedPolygon = false;
        _iLin = 0;
        _tolerance = ToolPolyh.DeflectionOverEstimation(polyh);
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }

        Box bofLin = new();
        PolygonTool bToo = new();
        _beginOfClosedPolygon = false;

        BoundSortBox polyhGrid = new();
        polyhGrid.Initialize(ToolPolyh.Bounding(polyh), ToolPolyh.ComponentsBounding(polyh));

        int indTri;

        for (_iLin = 1; _iLin <= lins.Length(); _iLin++)
        {
            bToo.LinBox(lins[_iLin], ToolPolyh.Bounding(polyh), ref bofLin);

            NCollection_List<int>.Iterator ilC = new(polyhGrid.Compare(bofLin));

            while (ilC.More())
            {
                indTri = ilC.Value();
                Intersect(lins[_iLin].Location,
                          lins[_iLin].Location.Translated(new Vector3D(lins[_iLin].Direction)), true, indTri,
                          polyh);
                ilC.Next();
            }
        }
    }

    public void Perform(in Polygon3d polyg, in Polyhedron polyh)
    {
        SelfInterference(false);
        _tolerance = ToolPolygon3d.DeflectionOverEstimation(polyg) + ToolPolyh.DeflectionOverEstimation(polyh);
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }

        if (!ToolPolygon3d.Bounding(polyg).IsOut(ToolPolyh.Bounding(polyh)))
        {
            Interference(polyg, polyh);
        }
    }

    public void Perform(in Line3D lin, in Polyhedron polyh)
    {
        SelfInterference(false);
        _tolerance = ToolPolyh.DeflectionOverEstimation(polyh);
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }

        _beginOfClosedPolygon = false;

        BoundSortBox polyhGrid = new();
        polyhGrid.Initialize(ToolPolyh.Bounding(polyh), ToolPolyh.ComponentsBounding(polyh));

        int indTri;

        _iLin = 0;

        Box bofLin = new();
        PolygonTool btoo = new();
        btoo.LinBox(lin, ToolPolyh.Bounding(polyh), ref bofLin);

        NCollection_List<int>.Iterator lCi = new(polyhGrid.Compare(bofLin));
        while (lCi.More())
        {
            indTri = lCi.Value();
            Intersect(lin.Location, lin.Location.Translated(new Vector3D(lin.Direction)), true, indTri,
                      polyh);
            lCi.Next();
        }
    }

    /// <summary>Computes an interference between the straight lines and the polyhedron.</summary>
    public void Perform(NCollection_Array1<Line3D> lins, in Polyhedron polyh)
    {
        SelfInterference(false);
        _tolerance = ToolPolyh.DeflectionOverEstimation(polyh);
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }

        Box bofLin = new();
        PolygonTool btoo = new();
        _beginOfClosedPolygon = false;

        BoundSortBox polyhGrid = new();
        polyhGrid.Initialize(ToolPolyh.Bounding(polyh), ToolPolyh.ComponentsBounding(polyh));

        int indTri;

        for (_iLin = 1; _iLin <= lins.Length(); _iLin++)
        {
            btoo.LinBox(lins[_iLin], ToolPolyh.Bounding(polyh), ref bofLin);

            NCollection_List<int>.Iterator tlC = new(polyhGrid.Compare(bofLin));

            while (tlC.More())
            {
                indTri = tlC.Value();
                Intersect(lins[_iLin].Location,
                          lins[_iLin].Location.Translated(new Vector3D(lins[_iLin].Direction)), true, indTri,
                          polyh);
                tlC.Next();
            }
        }
    }

    /// <summary>Compares the boxes of the segments of the polygon with those of the triangles of the polyhedron.</summary>
    public void Interference(in Polygon3d polyg, in Polyhedron polyh)
    {
        Box bofSeg = new();

        BoundSortBox polyhGrid = new();
        polyhGrid.Initialize(ToolPolyh.Bounding(polyh), ToolPolyh.ComponentsBounding(polyh));

        int indTri;
        _beginOfClosedPolygon = ToolPolygon3d.Closed(polyg);

        double defPh = ToolPolyh.DeflectionOverEstimation(polyh);

        for (_iLin = 1; _iLin <= ToolPolygon3d.NbSegments(polyg); _iLin++)
        {
            bofSeg.SetVoid();
            bofSeg.Add(ToolPolygon3d.BeginOfSeg(polyg, _iLin));
            bofSeg.Add(ToolPolygon3d.EndOfSeg(polyg, _iLin));
            bofSeg.Enlarge(ToolPolygon3d.DeflectionOverEstimation(polyg));

            NCollection_List<int> maliste;
            maliste = polyhGrid.Compare(bofSeg);
            NCollection_List<int>.Iterator clt = new(maliste);
            for (; clt.More(); clt.Next())
            {
                indTri = clt.Value();
                Point3D p1 = ToolPolygon3d.BeginOfSeg(polyg, _iLin);
                Point3D p2 = ToolPolygon3d.EndOfSeg(polyg, _iLin);
                ToolPolyh.Triangle(polyh, indTri, out int pTri0, out int pTri1, out int pTri2);
                Point3D pa = ToolPolyh.Point(polyh, pTri0);
                Point3D pb = ToolPolyh.Point(polyh, pTri1);
                Point3D pc = ToolPolyh.Point(polyh, pTri2);
                Vector3D paPb = new(pa, pb);
                Vector3D paPc = new(pa, pc);
                Vector3D normale = paPb.Crossed(paPc);
                double Norm_Normale = normale.Magnitude;
                if (Norm_Normale < 1e-14)
                {
                    continue;
                }
                normale = normale.Multiplied(defPh / Norm_Normale);
                Point3D p1m = p1.Translated(-normale);
                Point3D p1p = p1.Translated(normale);
                Point3D p2m = p2.Translated(-normale);
                Point3D p2p = p2.Translated(normale);
                Intersect(p1m, p2p, false, indTri, polyh);
                Intersect(p1p, p2m, false, indTri, polyh);
            }
            _beginOfClosedPolygon = false;
        }
    }

    /// <summary>
    /// Constructs and computes an interference between a straight line and a polyhedron whose triangles' boxes are
    /// sorted in <paramref name="polyhGrid"/>.
    /// </summary>
    public InterferencePolygonPolyhedron(in Line3D lin, in Polyhedron polyh, BoundSortBox polyhGrid)
        : base(false)
    {
        _tolerance = ToolPolyh.DeflectionOverEstimation(polyh);
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }

        _beginOfClosedPolygon = false;

        int indTri;

        _iLin = 0;

        Box bofLin = new();
        PolygonTool btoo = new();
        btoo.LinBox(lin, ToolPolyh.Bounding(polyh), ref bofLin);

        NCollection_List<int>.Iterator iCl = new(polyhGrid.Compare(bofLin));
        while (iCl.More())
        {
            indTri = iCl.Value();
            Intersect(lin.Location, lin.Location.Translated(new Vector3D(lin.Direction)), true, indTri,
                      polyh);
            iCl.Next();
        }
    }

    /// <summary>
    /// Constructs and computes an interference between the straight lines and a polyhedron whose triangles' boxes
    /// are sorted in <paramref name="polyhGrid"/>.
    /// </summary>
    public InterferencePolygonPolyhedron(NCollection_Array1<Line3D> lins, in Polyhedron polyh,
                                              BoundSortBox polyhGrid)
        : base(false)
    {
        _tolerance = ToolPolyh.DeflectionOverEstimation(polyh);
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }

        Box bofLin = new();
        PolygonTool bToo = new();
        _beginOfClosedPolygon = false;

        int indTri;

        for (_iLin = 1; _iLin <= lins.Length(); _iLin++)
        {
            bToo.LinBox(lins[_iLin], ToolPolyh.Bounding(polyh), ref bofLin);

            NCollection_List<int>.Iterator ilC = new(polyhGrid.Compare(bofLin));

            while (ilC.More())
            {
                indTri = ilC.Value();
                Intersect(lins[_iLin].Location,
                          lins[_iLin].Location.Translated(new Vector3D(lins[_iLin].Direction)), true, indTri,
                          polyh);
                ilC.Next();
            }
        }
    }

    public void Perform(in Polygon3d polyg, in Polyhedron polyh, BoundSortBox polyhGrid)
    {
        SelfInterference(false);
        _tolerance = ToolPolygon3d.DeflectionOverEstimation(polyg) + ToolPolyh.DeflectionOverEstimation(polyh);
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }

        if (!ToolPolygon3d.Bounding(polyg).IsOut(ToolPolyh.Bounding(polyh)))
        {
            Interference(polyg, polyh, polyhGrid);
        }
    }

    public void Perform(in Line3D lin, in Polyhedron polyh, BoundSortBox polyhGrid)
    {
        SelfInterference(false);
        _tolerance = ToolPolyh.DeflectionOverEstimation(polyh);
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }

        _beginOfClosedPolygon = false;

        int indTri;

        _iLin = 0;

        Box bofLin = new();
        PolygonTool btoo = new();
        btoo.LinBox(lin, ToolPolyh.Bounding(polyh), ref bofLin);

        NCollection_List<int>.Iterator lCi = new(polyhGrid.Compare(bofLin));
        while (lCi.More())
        {
            indTri = lCi.Value();
            Intersect(lin.Location, lin.Location.Translated(new Vector3D(lin.Direction)), true, indTri,
                      polyh);
            lCi.Next();
        }
    }

    /// <summary>Computes an interference between the straight lines and the polyhedron.</summary>
    public void Perform(NCollection_Array1<Line3D> lins, in Polyhedron polyh, BoundSortBox polyhGrid)
    {
        SelfInterference(false);
        _tolerance = ToolPolyh.DeflectionOverEstimation(polyh);
        if (_tolerance == 0.0)
        {
            _tolerance = Epsilon(1000.0);
        }

        Box bofLin = new();
        PolygonTool btoo = new();
        _beginOfClosedPolygon = false;

        int indTri;

        for (_iLin = 1; _iLin <= lins.Length(); _iLin++)
        {
            btoo.LinBox(lins[_iLin], ToolPolyh.Bounding(polyh), ref bofLin);

            NCollection_List<int>.Iterator tlC = new(polyhGrid.Compare(bofLin));

            while (tlC.More())
            {
                indTri = tlC.Value();
                Intersect(lins[_iLin].Location,
                          lins[_iLin].Location.Translated(new Vector3D(lins[_iLin].Direction)), true, indTri,
                          polyh);
                tlC.Next();
            }
        }
    }

    /// <summary>
    /// Compares the boxes of the segments of the polygon with those of the triangles of the polyhedron, sorted in
    /// <paramref name="polyhGrid"/>.
    /// </summary>
    public void Interference(in Polygon3d polyg, in Polyhedron polyh, BoundSortBox polyhGrid)
    {
        Box bofSeg = new();

        int indTri;
        Span<int> pTri = stackalloc int[3]; // declared in the loop in OCCT
        _beginOfClosedPolygon = ToolPolygon3d.Closed(polyg);

        for (_iLin = 1; _iLin <= ToolPolygon3d.NbSegments(polyg); _iLin++)
        {
            bofSeg.SetVoid();
            bofSeg.Add(ToolPolygon3d.BeginOfSeg(polyg, _iLin));
            bofSeg.Add(ToolPolygon3d.EndOfSeg(polyg, _iLin));
            bofSeg.Enlarge(ToolPolygon3d.DeflectionOverEstimation(polyg));

            //  Modified by MKK - Thu Oct  25 12:40:11 2007
            double defPh = ToolPolyh.DeflectionOverEstimation(polyh);
            NCollection_List<int> maliste;
            maliste = polyhGrid.Compare(bofSeg);
            NCollection_List<int>.Iterator clt = new(maliste);
            //  Modified by MKK - Thu Oct  25 12:40:11 2007 Begin
            Point3D p1 = new(), beg0 = new();
            Point3D p2 = new(), end0 = new();
            if (!maliste.IsEmpty())
            {
                p1 = ToolPolygon3d.BeginOfSeg(polyg, _iLin);
                p2 = ToolPolygon3d.EndOfSeg(polyg, _iLin);
                beg0 = p1;
                end0 = p2;
            }
            //  Modified by MKK - Thu Oct  25 12:40:11 2007 End
            while (clt.More())
            {
                indTri = clt.Value();
                //  Modified by MKK - Thu Oct  25 12:40:11 2007 Begin

                ToolPolyh.Triangle(polyh, indTri, out pTri[0], out pTri[1], out pTri[2]);
                // Normal vector and polar distance.
                global::Kerf.Engine.Intersection.Polygons.Intf.PlaneEquation(ToolPolyh.Point(polyh, pTri[0]), ToolPolyh.Point(polyh, pTri[1]),
                                   ToolPolyh.Point(polyh, pTri[2]), out Xyz triNor, out double triDp);

                // enlarge boundary segment
                if (_iLin == 1)
                {
                    Xyz dif = p1.XYZ() - p2.XYZ();
                    double dist = dif.Modulus();
                    if (dist > Constants.Resolution())
                    {
                        dif /= dist;
                        double cos = dif * triNor;
                        cos = Math.Abs(cos);
                        if (cos > Constants.Resolution())
                        {
                            double shift = defPh / cos;
                            beg0 = beg0.WithXYZ(p1.XYZ() + dif * shift);
                        }
                    }
                }
                else if (_iLin == ToolPolygon3d.NbSegments(polyg))
                {
                    Xyz dif = p2.XYZ() - p1.XYZ();
                    double dist = dif.Modulus();
                    if (dist > Constants.Resolution())
                    {
                        dif /= dist;
                        double cos = dif * triNor;
                        cos = Math.Abs(cos);
                        if (cos > Constants.Resolution())
                        {
                            double shift = defPh / cos;
                            end0 = end0.WithXYZ(p2.XYZ() + dif * shift);
                        }
                    }
                }
                double dBegTri = (triNor * beg0.XYZ()) - triDp; // Distance <p1> plane
                double dEndTri = (triNor * end0.XYZ()) - triDp; // Distance <p2> plane

                Intersect(beg0, end0, false, indTri, polyh, triNor, triDp, dBegTri, dEndTri);

                //  Modified by MKK - Thu Oct  25 12:40:11 2007 End
                clt.Next();
            }
            _beginOfClosedPolygon = false;
        }
    }

    /// <summary>
    /// Computes the intersection between the segment (or the line, if <paramref name="infinite"/>) and the
    /// triangle <paramref name="tTri"/>.
    /// </summary>
    private void Intersect(in Point3D begO, in Point3D endO, bool infinite, int tTri, in Polyhedron polyh)
    {
        Span<int> pTri = stackalloc int[3];
        ToolPolyh.Triangle(polyh, tTri, out pTri[0], out pTri[1], out pTri[2]);
        // Normal vector and polar distance.
        global::Kerf.Engine.Intersection.Polygons.Intf.PlaneEquation(ToolPolyh.Point(polyh, pTri[0]), ToolPolyh.Point(polyh, pTri[1]),
                           ToolPolyh.Point(polyh, pTri[2]), out Xyz triNor, out double triDp);

        double dBegTri = (triNor * begO.XYZ()) - triDp; // Distance <BegO> plan
        double dEndTri = (triNor * endO.XYZ()) - triDp; // Distance <EndO> plan

        IntersectWithPlane(begO, endO, infinite, tTri, polyh, pTri, triNor, dBegTri, dEndTri);
    }

    /// <summary>
    /// Computes the intersection between the segment (or the line, if <paramref name="infinite"/>) and the
    /// triangle <paramref name="tTri"/>, whose plane equation and the distances of the segment's ends to it are
    /// given.
    /// </summary>
    private void Intersect(in Point3D begO, in Point3D endO, bool infinite, int tTri, in Polyhedron polyh,
                           in Xyz triNormal, double triDp, double dBegTri, double dEndTri)
    {
        _ = triDp;
        Span<int> pTri = stackalloc int[3];
        ToolPolyh.Triangle(polyh, tTri, out pTri[0], out pTri[1], out pTri[2]);
        Xyz triNor = triNormal; // Normal vector.

        IntersectWithPlane(begO, endO, infinite, tTri, polyh, pTri, triNor, dBegTri, dEndTri);
    }

    /// <summary>
    /// The body OCCT writes out twice, identically, in both <c>Intersect</c> methods (after their plane equation).
    /// </summary>
    private void IntersectWithPlane(in Point3D begO, in Point3D endO, bool infinite, int tTri, in Polyhedron polyh,
                                    ReadOnlySpan<int> pTri, in Xyz triNor, double dBegTri, double dEndTri)
    {
        PIType typOnG = PIType.Edge;
        double t;
        bool noIntersectionWithTriangle = false;

        double param;
        t = dBegTri - dEndTri;
        if (t >= 1.0e-16 || t <= -1.0e-16)
        {
            param = dBegTri / t;
        }
        else
        {
            param = dBegTri;
        }
        double floatgap = Epsilon(1000.0);

        if (!infinite)
        {
            if (dBegTri <= floatgap && dBegTri >= -floatgap)
            {
                param = 0.0;
                typOnG = PIType.Vertex;
                if (_beginOfClosedPolygon)
                {
                    noIntersectionWithTriangle = false;
                }
            }
            else if (dEndTri <= floatgap && dEndTri >= -floatgap)
            {
                param = 1.0;
                typOnG = PIType.Vertex;
                noIntersectionWithTriangle = false;
            }
            if (param < 0.0 || param > 1.0)
            {
                noIntersectionWithTriangle = true;
            }
        }
        if (noIntersectionWithTriangle == false)
        {
            Xyz spLieu = begO.XYZ() + ((endO.XYZ() - begO.XYZ()) * param);
            Span<double> dPiE = stackalloc double[3] { 0.0, 0.0, 0.0 };
            Span<double> dPtPi = stackalloc double[3];
            double sigd;
            int @is = 0;
            int sEdge = -1;
            int sVertex = -1;
            int tbreak = 0;
            { //-- is = 0
                Xyz segT = ToolPolyh.Point(polyh, pTri[1]).XYZ() - ToolPolyh.Point(polyh, pTri[0]).XYZ();
                Xyz vecP = spLieu - ToolPolyh.Point(polyh, pTri[0]).XYZ();
                dPtPi[0] = vecP.Modulus();
                if (dPtPi[0] <= floatgap)
                {
                    sVertex = 0;
                    @is = 0;
                    tbreak = 1;
                }
                else
                {
                    Xyz segT_x_vecP = segT ^ vecP;
                    double Modulus_segT_x_vecP = segT_x_vecP.Modulus();
                    sigd = segT_x_vecP * triNor;
                    if (sigd > floatgap)
                    {
                        sigd = 1.0;
                    }
                    else if (sigd < -floatgap)
                    {
                        sigd = -1.0;
                    }
                    else
                    {
                        sigd = 0.0;
                    }
                    dPiE[0] = sigd * (Modulus_segT_x_vecP / segT.Modulus());
                    if (dPiE[0] <= floatgap && dPiE[0] >= -floatgap)
                    {
                        sEdge = 0;
                        @is = 0;
                        tbreak = 1;
                    }
                }
            }

            if (tbreak == 0)
            { //-- is = 1
                Xyz segT = ToolPolyh.Point(polyh, pTri[2]).XYZ() - ToolPolyh.Point(polyh, pTri[1]).XYZ();
                Xyz vecP = spLieu - ToolPolyh.Point(polyh, pTri[1]).XYZ();
                dPtPi[1] = vecP.Modulus();
                if (dPtPi[1] <= floatgap)
                {
                    sVertex = 1;
                    @is = 1;
                    tbreak = 1;
                }
                else
                {
                    Xyz segT_x_vecP = segT ^ vecP;
                    double Modulus_segT_x_vecP = segT_x_vecP.Modulus();
                    sigd = segT_x_vecP * triNor;
                    if (sigd > floatgap)
                    {
                        sigd = 1.0;
                    }
                    else if (sigd < -floatgap)
                    {
                        sigd = -1.0;
                    }
                    else
                    {
                        sigd = 0.0;
                    }
                    dPiE[1] = sigd * (Modulus_segT_x_vecP / segT.Modulus());
                    if (dPiE[1] <= floatgap && dPiE[1] >= -floatgap)
                    {
                        sEdge = 1;
                        @is = 1;
                        tbreak = 1;
                    }
                }
            }
            if (tbreak == 0)
            { //-- is = 2
                Xyz segT = ToolPolyh.Point(polyh, pTri[0]).XYZ() - ToolPolyh.Point(polyh, pTri[2]).XYZ();
                Xyz vecP = spLieu - ToolPolyh.Point(polyh, pTri[2]).XYZ();
                dPtPi[2] = vecP.Modulus();
                if (dPtPi[2] <= floatgap)
                {
                    sVertex = 2;
                    @is = 2;
                }
                Xyz segT_x_vecP = segT ^ vecP;
                double Modulus_segT_x_vecP = segT_x_vecP.Modulus();
                sigd = segT_x_vecP * triNor;
                if (sigd > floatgap)
                {
                    sigd = 1.0;
                }
                else if (sigd < -floatgap)
                {
                    sigd = -1.0;
                }
                else
                {
                    sigd = 0.0;
                }
                dPiE[2] = sigd * (Modulus_segT_x_vecP / segT.Modulus());
                if (dPiE[2] <= floatgap && dPiE[2] >= -floatgap)
                {
                    sEdge = 2;
                    @is = 2;
                }
            }
            //-- end for i=0 to 2

            if (sVertex > -1)
            {
                SectionPoint sp = new(new Point3D(spLieu), typOnG, 0, _iLin, param, PIType.Vertex, pTri[@is], 0, 0.0,
                                           1.0);
                _sPoins.Append(sp);
            }
            else if (sEdge > -1)
            {
                ToolPolyh.TriConnex(polyh, tTri, pTri[sEdge], pTri[Pourcent3[sEdge + 1]], out _, out _);
                SectionPoint sp = new(new Point3D(spLieu), typOnG, 0, _iLin, param, PIType.Edge,
                                           Min(pTri[sEdge], pTri[Pourcent3[sEdge + 1]]),
                                           Max(pTri[sEdge], pTri[Pourcent3[sEdge + 1]]), 0.0, 1.0);
                _sPoins.Append(sp);
            }
            else if (dPiE[0] > 0.0 && dPiE[1] > 0.0 && dPiE[2] > 0.0)
            {
                SectionPoint sp = new(new Point3D(spLieu), typOnG, 0, _iLin, param, PIType.Face, tTri, 0, 0.0, 1.0);
                _sPoins.Append(sp);
            }
            //  Modified by Sergey KHROMOV - Fri Dec  7 14:40:11 2001 Begin
            // Sometimes triangulation doesn't cover whole the face. In this
            // case it is necessary to take into account the deflection between boundary
            // isolines of the surface and boundary trianles. Computed value of this
            // deflection is contained in thePolyh.
            else
            {
                int i;

                for (i = 1; i <= 3; i++)
                {
                    int indP1 = (i == 3) ? pTri[0] : pTri[i];
                    int indP2 = pTri[i - 1];

                    if (ToolPolyh.IsOnBound(polyh, indP1, indP2))
                    {
                        // For boundary line it is necessary to check the border deflection.
                        double deflection = ToolPolyh.GetBorderDeflection(polyh);
                        Point3D begP = ToolPolyh.Point(polyh, indP1);
                        Point3D endP = ToolPolyh.Point(polyh, indP2);
                        Vector3D vecTri = new(begP, endP);
                        Direction3D dirTri = new(vecTri);
                        Line3D linTri = new(begP, dirTri);
                        Point3D pOnE = new(spLieu);
                        double dist = linTri.DistanceTo(pOnE);

                        if (dist <= deflection)
                        {
                            Vector3D vLocPOnE = new(begP, pOnE);
                            Vector3D vecDirTri = new(dirTri);
                            double par = vLocPOnE * vecDirTri;
                            double maxPar = vecTri.Magnitude;

                            if (par >= 0 && par <= maxPar)
                            {
                                SectionPoint sp = new(new Point3D(spLieu), typOnG, 0, _iLin, param, PIType.Face, tTri,
                                                           0, 0.0, 1.0);
                                _sPoins.Append(sp);
                            }
                        }
                    }
                }
            }
            //  Modified by Sergey KHROMOV - Fri Dec  7 14:40:29 2001 End
        } //---- if(NoIntersectionWithTriangle == false)

        //---------------------------------------------------------------------------
        //-- Test the distance between the sides of the triangle and the polygon.
        //--
        //-- If this distance is less than Tolerance, a section point is created.
        {
            Vector3D vecPol = new(begO, endO);
            double nVecPol = vecPol.Magnitude;
            if (nVecPol < global::Kerf.Engine.Foundation.Precision.Computational())
            {
                return;
            }
            Direction3D dirPol = new(vecPol);
            Line3D linPol = new(begO, dirPol);
            double dist2;

            for (int i = 0; i < 3; i++)
            {
                int pTri_ip1pc3 = pTri[Pourcent3[i + 1]];
                int pTri_i = pTri[i];
                Point3D begT = ToolPolyh.Point(polyh, pTri_ip1pc3);
                Point3D endT = ToolPolyh.Point(polyh, pTri_i);
                Vector3D vecTri = new(begT, endT);
                double nVecTri = vecTri.Magnitude;
                if (nVecTri < global::Kerf.Engine.Foundation.Precision.Computational())
                {
                    continue;
                }
                Direction3D dirTri = new(vecTri);
                Line3D linTri = new(begT, dirTri);
                ExtElC extrema = new(linPol, linTri, 0.00000001);
                if (extrema.IsDone)
                {
                    if (extrema.IsParallel == false)
                    {
                        if (extrema.ExtCount != 0)
                        {
                            dist2 = extrema.SquareDistance();
                            if (dist2 <= _tolerance * _tolerance)
                            {
                                extrema.Points(1, out POnCurv pOnC1, out POnCurv pOnC2);
                                Point3D po = pOnC1.Value();
                                Point3D pt = pOnC2.Value();
                                if (IsInSegment(vecPol, new Vector3D(begO, po), nVecPol, out double paramOnO, _tolerance))
                                {
                                    if (IsInSegment(vecTri, new Vector3D(begT, pt), nVecTri, out double paramOnT,
                                                    _tolerance))
                                    {
                                        Xyz spLieu = begT.XYZ() + ((endT.XYZ() - begT.XYZ()) * paramOnT);
                                        int tmin, tmax;
                                        if (pTri_i > pTri_ip1pc3)
                                        {
                                            tmin = pTri_ip1pc3;
                                            tmax = pTri_i;
                                        }
                                        else
                                        {
                                            tmax = pTri_ip1pc3;
                                            tmin = pTri_i;
                                        }
                                        SectionPoint sp = new(new Point3D(spLieu), typOnG, 0, _iLin, paramOnO,
                                                                   PIType.Edge, tmin, tmax, 0.0, 1.0);
                                        _sPoins.Append(sp);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
