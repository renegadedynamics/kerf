// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_InterferencePolygonPolyhedron.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Occt.Intf_PIType;

namespace Occt;

/// <summary>
/// The static tool over a 3D polygon that the generic <c>Intf_InterferencePolygonPolyhedron</c> uses (OCCT's
/// ToolPolygon3d macro: IntCurveSurface_ThePolygonToolOfHInter, HLRBRep_ThePolygonToolOfInterCSurf).
/// </summary>
public interface IIntf_ToolPolygon3d<Polygon3d>
{
    static abstract Box Bounding(in Polygon3d thePolygon);

    static abstract double DeflectionOverEstimation(in Polygon3d thePolygon);

    static abstract bool Closed(in Polygon3d thePolygon);

    static abstract int NbSegments(in Polygon3d thePolygon);

    static abstract Point3D BeginOfSeg(in Polygon3d thePolygon, int Index);

    static abstract Point3D EndOfSeg(in Polygon3d thePolygon, int Index);
}

/// <summary>
/// The static tool over a polyhedron (a triangulated surface) that the generic
/// <c>Intf_InterferencePolygonPolyhedron</c> uses (OCCT's ToolPolyh macro: IntCurveSurface_ThePolyhedronToolOfHInter,
/// HLRBRep_ThePolyhedronToolOfInterCSurf).
/// </summary>
public interface IIntf_ToolPolyhedron<Polyhedron>
{
    static abstract Box Bounding(in Polyhedron thePolyh);

    static abstract NCollection_HArray1<Box> ComponentsBounding(in Polyhedron thePolyh);

    static abstract double DeflectionOverEstimation(in Polyhedron thePolyh);

    static abstract int NbTriangles(in Polyhedron thePolyh);

    static abstract void Triangle(in Polyhedron thePolyh, int Index, out int P1, out int P2, out int P3);

    static abstract Point3D Point(in Polyhedron thePolyh, int Index);

    static abstract int TriConnex(in Polyhedron thePolyh, int Triang, int Pivot, int Pedge, out int TriCon,
                                  out int OtherP);

    static abstract bool IsOnBound(in Polyhedron thePolyh, int Index1, int Index2);

    static abstract double GetBorderDeflection(in Polyhedron thePolyh);
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
public class Intf_InterferencePolygonPolyhedron<Polygon3d, ToolPolygon3d, Polyhedron, ToolPolyh> : Intf_Interference
    where ToolPolygon3d : IIntf_ToolPolygon3d<Polygon3d>
    where ToolPolyh : IIntf_ToolPolyhedron<Polyhedron>
{
    private static readonly int[] Pourcent3 = [0, 1, 2, 0];

    private bool BeginOfClosedPolygon;
    private int iLin;

    private static bool IsInSegment(in Vector3D P1P2, in Vector3D P1P, double NP1P2, out double Param, double Tolerance)
    {
        Param = 0.0;
        if (NP1P2 < global::Kerf.Engine.Foundation.Precision.Computational())
        {
            return false;
        }
        Param = P1P2.Dot(P1P);
        Param /= NP1P2;
        if (Param > (NP1P2 + Tolerance))
        {
            return (false);
        }
        if (Param < (-Tolerance))
        {
            return (false);
        }
        Param /= NP1P2;
        if (Param < 0.0)
        {
            Param = 0.0;
        }
        if (Param > 1.0)
        {
            Param = 1.0;
        }
        return (true);
    }

    public Intf_InterferencePolygonPolyhedron()
        : base(false)
    {
        BeginOfClosedPolygon = false;
        iLin = 0;
    }

    /// <summary>Constructs and computes an interference between a polygon and a polyhedron.</summary>
    public Intf_InterferencePolygonPolyhedron(in Polygon3d thePolyg, in Polyhedron thePolyh)
        : base(false)
    {
        BeginOfClosedPolygon = false;
        iLin = 0;
        Tolerance = ToolPolygon3d.DeflectionOverEstimation(thePolyg) + ToolPolyh.DeflectionOverEstimation(thePolyh);
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }

        if (!ToolPolygon3d.Bounding(thePolyg).IsOut(ToolPolyh.Bounding(thePolyh)))
        {
            Interference(thePolyg, thePolyh);
        }
    }

    /// <summary>
    /// Constructs and computes an interference between a polygon and a polyhedron whose triangles' boxes are
    /// sorted in <paramref name="PolyhGrid"/>.
    /// </summary>
    public Intf_InterferencePolygonPolyhedron(in Polygon3d thePolyg, in Polyhedron thePolyh, BoundSortBox PolyhGrid)
        : base(false)
    {
        BeginOfClosedPolygon = false;
        iLin = 0;
        Tolerance = ToolPolygon3d.DeflectionOverEstimation(thePolyg) + ToolPolyh.DeflectionOverEstimation(thePolyh);
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }

        if (!ToolPolygon3d.Bounding(thePolyg).IsOut(ToolPolyh.Bounding(thePolyh)))
        {
            Interference(thePolyg, thePolyh, PolyhGrid);
        }
    }

    /// <summary>Constructs and computes an interference between a straight line and a polyhedron.</summary>
    public Intf_InterferencePolygonPolyhedron(in Line3D theLin, in Polyhedron thePolyh)
        : base(false)
    {
        BeginOfClosedPolygon = false;
        iLin = 0;
        Tolerance = ToolPolyh.DeflectionOverEstimation(thePolyh);
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }

        BeginOfClosedPolygon = false;

        BoundSortBox PolyhGrid = new();
        PolyhGrid.Initialize(ToolPolyh.Bounding(thePolyh), ToolPolyh.ComponentsBounding(thePolyh));
        int indTri;

        iLin = 0;

        Box bofLin = new();
        Intf_Tool btoo = new();
        btoo.LinBox(theLin, ToolPolyh.Bounding(thePolyh), ref bofLin);

        NCollection_List<int>.Iterator iCl = new(PolyhGrid.Compare(bofLin));
        while (iCl.More())
        {
            indTri = iCl.Value();
            Intersect(theLin.Location(), theLin.Location().Translated(new Vector3D(theLin.Direction())), true, indTri,
                      thePolyh);
            iCl.Next();
        }
    }

    /// <summary>Constructs and computes an interference between the straight lines and a polyhedron.</summary>
    public Intf_InterferencePolygonPolyhedron(NCollection_Array1<Line3D> theLins, in Polyhedron thePolyh)
        : base(false)
    {
        BeginOfClosedPolygon = false;
        iLin = 0;
        Tolerance = ToolPolyh.DeflectionOverEstimation(thePolyh);
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }

        Box bofLin = new();
        Intf_Tool bToo = new();
        BeginOfClosedPolygon = false;

        BoundSortBox PolyhGrid = new();
        PolyhGrid.Initialize(ToolPolyh.Bounding(thePolyh), ToolPolyh.ComponentsBounding(thePolyh));

        int indTri;

        for (iLin = 1; iLin <= theLins.Length(); iLin++)
        {
            bToo.LinBox(theLins[iLin], ToolPolyh.Bounding(thePolyh), ref bofLin);

            NCollection_List<int>.Iterator ilC = new(PolyhGrid.Compare(bofLin));

            while (ilC.More())
            {
                indTri = ilC.Value();
                Intersect(theLins[iLin].Location(),
                          theLins[iLin].Location().Translated(new Vector3D(theLins[iLin].Direction())), true, indTri,
                          thePolyh);
                ilC.Next();
            }
        }
    }

    public void Perform(in Polygon3d thePolyg, in Polyhedron thePolyh)
    {
        SelfInterference(false);
        Tolerance = ToolPolygon3d.DeflectionOverEstimation(thePolyg) + ToolPolyh.DeflectionOverEstimation(thePolyh);
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }

        if (!ToolPolygon3d.Bounding(thePolyg).IsOut(ToolPolyh.Bounding(thePolyh)))
        {
            Interference(thePolyg, thePolyh);
        }
    }

    public void Perform(in Line3D theLin, in Polyhedron thePolyh)
    {
        SelfInterference(false);
        Tolerance = ToolPolyh.DeflectionOverEstimation(thePolyh);
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }

        BeginOfClosedPolygon = false;

        BoundSortBox PolyhGrid = new();
        PolyhGrid.Initialize(ToolPolyh.Bounding(thePolyh), ToolPolyh.ComponentsBounding(thePolyh));

        int indTri;

        iLin = 0;

        Box bofLin = new();
        Intf_Tool btoo = new();
        btoo.LinBox(theLin, ToolPolyh.Bounding(thePolyh), ref bofLin);

        NCollection_List<int>.Iterator lCi = new(PolyhGrid.Compare(bofLin));
        while (lCi.More())
        {
            indTri = lCi.Value();
            Intersect(theLin.Location(), theLin.Location().Translated(new Vector3D(theLin.Direction())), true, indTri,
                      thePolyh);
            lCi.Next();
        }
    }

    /// <summary>Computes an interference between the straight lines and the polyhedron.</summary>
    public void Perform(NCollection_Array1<Line3D> theLins, in Polyhedron thePolyh)
    {
        SelfInterference(false);
        Tolerance = ToolPolyh.DeflectionOverEstimation(thePolyh);
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }

        Box bofLin = new();
        Intf_Tool Btoo = new();
        BeginOfClosedPolygon = false;

        BoundSortBox PolyhGrid = new();
        PolyhGrid.Initialize(ToolPolyh.Bounding(thePolyh), ToolPolyh.ComponentsBounding(thePolyh));

        int indTri;

        for (iLin = 1; iLin <= theLins.Length(); iLin++)
        {
            Btoo.LinBox(theLins[iLin], ToolPolyh.Bounding(thePolyh), ref bofLin);

            NCollection_List<int>.Iterator tlC = new(PolyhGrid.Compare(bofLin));

            while (tlC.More())
            {
                indTri = tlC.Value();
                Intersect(theLins[iLin].Location(),
                          theLins[iLin].Location().Translated(new Vector3D(theLins[iLin].Direction())), true, indTri,
                          thePolyh);
                tlC.Next();
            }
        }
    }

    /// <summary>Compares the boxes of the segments of the polygon with those of the triangles of the polyhedron.</summary>
    public void Interference(in Polygon3d thePolyg, in Polyhedron thePolyh)
    {
        Box bofSeg = new();

        BoundSortBox PolyhGrid = new();
        PolyhGrid.Initialize(ToolPolyh.Bounding(thePolyh), ToolPolyh.ComponentsBounding(thePolyh));

        int indTri;
        BeginOfClosedPolygon = ToolPolygon3d.Closed(thePolyg);

        double defPh = ToolPolyh.DeflectionOverEstimation(thePolyh);

        for (iLin = 1; iLin <= ToolPolygon3d.NbSegments(thePolyg); iLin++)
        {
            bofSeg.SetVoid();
            bofSeg.Add(ToolPolygon3d.BeginOfSeg(thePolyg, iLin));
            bofSeg.Add(ToolPolygon3d.EndOfSeg(thePolyg, iLin));
            bofSeg.Enlarge(ToolPolygon3d.DeflectionOverEstimation(thePolyg));

            NCollection_List<int> maliste;
            maliste = PolyhGrid.Compare(bofSeg);
            NCollection_List<int>.Iterator clt = new(maliste);
            for (; clt.More(); clt.Next())
            {
                indTri = clt.Value();
                Point3D p1 = ToolPolygon3d.BeginOfSeg(thePolyg, iLin);
                Point3D p2 = ToolPolygon3d.EndOfSeg(thePolyg, iLin);
                ToolPolyh.Triangle(thePolyh, indTri, out int pTri0, out int pTri1, out int pTri2);
                Point3D Pa = ToolPolyh.Point(thePolyh, pTri0);
                Point3D Pb = ToolPolyh.Point(thePolyh, pTri1);
                Point3D Pc = ToolPolyh.Point(thePolyh, pTri2);
                Vector3D PaPb = new(Pa, Pb);
                Vector3D PaPc = new(Pa, Pc);
                Vector3D Normale = PaPb.Crossed(PaPc);
                double Norm_Normale = Normale.Magnitude();
                if (Norm_Normale < 1e-14)
                {
                    continue;
                }
                Normale.Multiply(defPh / Norm_Normale);
                Point3D p1m = p1.Translated(-Normale);
                Point3D p1p = p1.Translated(Normale);
                Point3D p2m = p2.Translated(-Normale);
                Point3D p2p = p2.Translated(Normale);
                Intersect(p1m, p2p, false, indTri, thePolyh);
                Intersect(p1p, p2m, false, indTri, thePolyh);
            }
            BeginOfClosedPolygon = false;
        }
    }

    /// <summary>
    /// Constructs and computes an interference between a straight line and a polyhedron whose triangles' boxes are
    /// sorted in <paramref name="PolyhGrid"/>.
    /// </summary>
    public Intf_InterferencePolygonPolyhedron(in Line3D theLin, in Polyhedron thePolyh, BoundSortBox PolyhGrid)
        : base(false)
    {
        Tolerance = ToolPolyh.DeflectionOverEstimation(thePolyh);
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }

        BeginOfClosedPolygon = false;

        int indTri;

        iLin = 0;

        Box bofLin = new();
        Intf_Tool btoo = new();
        btoo.LinBox(theLin, ToolPolyh.Bounding(thePolyh), ref bofLin);

        NCollection_List<int>.Iterator iCl = new(PolyhGrid.Compare(bofLin));
        while (iCl.More())
        {
            indTri = iCl.Value();
            Intersect(theLin.Location(), theLin.Location().Translated(new Vector3D(theLin.Direction())), true, indTri,
                      thePolyh);
            iCl.Next();
        }
    }

    /// <summary>
    /// Constructs and computes an interference between the straight lines and a polyhedron whose triangles' boxes
    /// are sorted in <paramref name="PolyhGrid"/>.
    /// </summary>
    public Intf_InterferencePolygonPolyhedron(NCollection_Array1<Line3D> theLins, in Polyhedron thePolyh,
                                              BoundSortBox PolyhGrid)
        : base(false)
    {
        Tolerance = ToolPolyh.DeflectionOverEstimation(thePolyh);
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }

        Box bofLin = new();
        Intf_Tool bToo = new();
        BeginOfClosedPolygon = false;

        int indTri;

        for (iLin = 1; iLin <= theLins.Length(); iLin++)
        {
            bToo.LinBox(theLins[iLin], ToolPolyh.Bounding(thePolyh), ref bofLin);

            NCollection_List<int>.Iterator ilC = new(PolyhGrid.Compare(bofLin));

            while (ilC.More())
            {
                indTri = ilC.Value();
                Intersect(theLins[iLin].Location(),
                          theLins[iLin].Location().Translated(new Vector3D(theLins[iLin].Direction())), true, indTri,
                          thePolyh);
                ilC.Next();
            }
        }
    }

    public void Perform(in Polygon3d thePolyg, in Polyhedron thePolyh, BoundSortBox PolyhGrid)
    {
        SelfInterference(false);
        Tolerance = ToolPolygon3d.DeflectionOverEstimation(thePolyg) + ToolPolyh.DeflectionOverEstimation(thePolyh);
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }

        if (!ToolPolygon3d.Bounding(thePolyg).IsOut(ToolPolyh.Bounding(thePolyh)))
        {
            Interference(thePolyg, thePolyh, PolyhGrid);
        }
    }

    public void Perform(in Line3D theLin, in Polyhedron thePolyh, BoundSortBox PolyhGrid)
    {
        SelfInterference(false);
        Tolerance = ToolPolyh.DeflectionOverEstimation(thePolyh);
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }

        BeginOfClosedPolygon = false;

        int indTri;

        iLin = 0;

        Box bofLin = new();
        Intf_Tool btoo = new();
        btoo.LinBox(theLin, ToolPolyh.Bounding(thePolyh), ref bofLin);

        NCollection_List<int>.Iterator lCi = new(PolyhGrid.Compare(bofLin));
        while (lCi.More())
        {
            indTri = lCi.Value();
            Intersect(theLin.Location(), theLin.Location().Translated(new Vector3D(theLin.Direction())), true, indTri,
                      thePolyh);
            lCi.Next();
        }
    }

    /// <summary>Computes an interference between the straight lines and the polyhedron.</summary>
    public void Perform(NCollection_Array1<Line3D> theLins, in Polyhedron thePolyh, BoundSortBox PolyhGrid)
    {
        SelfInterference(false);
        Tolerance = ToolPolyh.DeflectionOverEstimation(thePolyh);
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }

        Box bofLin = new();
        Intf_Tool Btoo = new();
        BeginOfClosedPolygon = false;

        int indTri;

        for (iLin = 1; iLin <= theLins.Length(); iLin++)
        {
            Btoo.LinBox(theLins[iLin], ToolPolyh.Bounding(thePolyh), ref bofLin);

            NCollection_List<int>.Iterator tlC = new(PolyhGrid.Compare(bofLin));

            while (tlC.More())
            {
                indTri = tlC.Value();
                Intersect(theLins[iLin].Location(),
                          theLins[iLin].Location().Translated(new Vector3D(theLins[iLin].Direction())), true, indTri,
                          thePolyh);
                tlC.Next();
            }
        }
    }

    /// <summary>
    /// Compares the boxes of the segments of the polygon with those of the triangles of the polyhedron, sorted in
    /// <paramref name="PolyhGrid"/>.
    /// </summary>
    public void Interference(in Polygon3d thePolyg, in Polyhedron thePolyh, BoundSortBox PolyhGrid)
    {
        Box bofSeg = new();

        int indTri;
        Span<int> pTri = stackalloc int[3]; // declared in the loop in OCCT
        BeginOfClosedPolygon = ToolPolygon3d.Closed(thePolyg);

        for (iLin = 1; iLin <= ToolPolygon3d.NbSegments(thePolyg); iLin++)
        {
            bofSeg.SetVoid();
            bofSeg.Add(ToolPolygon3d.BeginOfSeg(thePolyg, iLin));
            bofSeg.Add(ToolPolygon3d.EndOfSeg(thePolyg, iLin));
            bofSeg.Enlarge(ToolPolygon3d.DeflectionOverEstimation(thePolyg));

            //  Modified by MKK - Thu Oct  25 12:40:11 2007
            double defPh = ToolPolyh.DeflectionOverEstimation(thePolyh);
            NCollection_List<int> maliste;
            maliste = PolyhGrid.Compare(bofSeg);
            NCollection_List<int>.Iterator clt = new(maliste);
            //  Modified by MKK - Thu Oct  25 12:40:11 2007 Begin
            Point3D p1 = new(), Beg0 = new();
            Point3D p2 = new(), End0 = new();
            if (!maliste.IsEmpty())
            {
                p1 = ToolPolygon3d.BeginOfSeg(thePolyg, iLin);
                p2 = ToolPolygon3d.EndOfSeg(thePolyg, iLin);
                Beg0 = p1;
                End0 = p2;
            }
            //  Modified by MKK - Thu Oct  25 12:40:11 2007 End
            while (clt.More())
            {
                indTri = clt.Value();
                //  Modified by MKK - Thu Oct  25 12:40:11 2007 Begin

                ToolPolyh.Triangle(thePolyh, indTri, out pTri[0], out pTri[1], out pTri[2]);
                // Normal vector and polar distance.
                Intf.PlaneEquation(ToolPolyh.Point(thePolyh, pTri[0]), ToolPolyh.Point(thePolyh, pTri[1]),
                                   ToolPolyh.Point(thePolyh, pTri[2]), out Xyz triNor, out double triDp);

                // enlarge boundary segment
                if (iLin == 1)
                {
                    Xyz dif = p1.XYZ() - p2.XYZ();
                    double dist = dif.Modulus();
                    if (dist > Constants.Resolution())
                    {
                        dif /= dist;
                        double aCos = dif * triNor;
                        aCos = Math.Abs(aCos);
                        if (aCos > Constants.Resolution())
                        {
                            double shift = defPh / aCos;
                            Beg0.SetXYZ(p1.XYZ() + dif * shift);
                        }
                    }
                }
                else if (iLin == ToolPolygon3d.NbSegments(thePolyg))
                {
                    Xyz dif = p2.XYZ() - p1.XYZ();
                    double dist = dif.Modulus();
                    if (dist > Constants.Resolution())
                    {
                        dif /= dist;
                        double aCos = dif * triNor;
                        aCos = Math.Abs(aCos);
                        if (aCos > Constants.Resolution())
                        {
                            double shift = defPh / aCos;
                            End0.SetXYZ(p2.XYZ() + dif * shift);
                        }
                    }
                }
                double dBegTri = (triNor * Beg0.XYZ()) - triDp; // Distance <p1> plane
                double dEndTri = (triNor * End0.XYZ()) - triDp; // Distance <p2> plane

                Intersect(Beg0, End0, false, indTri, thePolyh, triNor, triDp, dBegTri, dEndTri);

                //  Modified by MKK - Thu Oct  25 12:40:11 2007 End
                clt.Next();
            }
            BeginOfClosedPolygon = false;
        }
    }

    /// <summary>
    /// Computes the intersection between the segment (or the line, if <paramref name="Infinite"/>) and the
    /// triangle <paramref name="TTri"/>.
    /// </summary>
    private void Intersect(in Point3D BegO, in Point3D EndO, bool Infinite, int TTri, in Polyhedron thePolyh)
    {
        Span<int> pTri = stackalloc int[3];
        ToolPolyh.Triangle(thePolyh, TTri, out pTri[0], out pTri[1], out pTri[2]);
        // Normal vector and polar distance.
        Intf.PlaneEquation(ToolPolyh.Point(thePolyh, pTri[0]), ToolPolyh.Point(thePolyh, pTri[1]),
                           ToolPolyh.Point(thePolyh, pTri[2]), out Xyz triNor, out double triDp);

        double dBegTri = (triNor * BegO.XYZ()) - triDp; // Distance <BegO> plan
        double dEndTri = (triNor * EndO.XYZ()) - triDp; // Distance <EndO> plan

        IntersectWithPlane(BegO, EndO, Infinite, TTri, thePolyh, pTri, triNor, dBegTri, dEndTri);
    }

    /// <summary>
    /// Computes the intersection between the segment (or the line, if <paramref name="Infinite"/>) and the
    /// triangle <paramref name="TTri"/>, whose plane equation and the distances of the segment's ends to it are
    /// given.
    /// </summary>
    private void Intersect(in Point3D BegO, in Point3D EndO, bool Infinite, int TTri, in Polyhedron thePolyh,
                           in Xyz TriNormal, double TriDp, double dBegTri, double dEndTri)
    {
        _ = TriDp;
        Span<int> pTri = stackalloc int[3];
        ToolPolyh.Triangle(thePolyh, TTri, out pTri[0], out pTri[1], out pTri[2]);
        Xyz triNor = TriNormal; // Normal vector.

        IntersectWithPlane(BegO, EndO, Infinite, TTri, thePolyh, pTri, triNor, dBegTri, dEndTri);
    }

    /// <summary>
    /// The body OCCT writes out twice, identically, in both <c>Intersect</c> methods (after their plane equation).
    /// </summary>
    private void IntersectWithPlane(in Point3D BegO, in Point3D EndO, bool Infinite, int TTri, in Polyhedron thePolyh,
                                    ReadOnlySpan<int> pTri, in Xyz triNor, double dBegTri, double dEndTri)
    {
        Intf_PIType typOnG = Intf_EDGE;
        double t;
        bool NoIntersectionWithTriangle = false;

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

        if (!Infinite)
        {
            if (dBegTri <= floatgap && dBegTri >= -floatgap)
            {
                param = 0.0;
                typOnG = Intf_VERTEX;
                if (BeginOfClosedPolygon)
                {
                    NoIntersectionWithTriangle = false;
                }
            }
            else if (dEndTri <= floatgap && dEndTri >= -floatgap)
            {
                param = 1.0;
                typOnG = Intf_VERTEX;
                NoIntersectionWithTriangle = false;
            }
            if (param < 0.0 || param > 1.0)
            {
                NoIntersectionWithTriangle = true;
            }
        }
        if (NoIntersectionWithTriangle == false)
        {
            Xyz spLieu = BegO.XYZ() + ((EndO.XYZ() - BegO.XYZ()) * param);
            Span<double> dPiE = stackalloc double[3] { 0.0, 0.0, 0.0 };
            Span<double> dPtPi = stackalloc double[3];
            double sigd;
            int @is = 0;
            int sEdge = -1;
            int sVertex = -1;
            int tbreak = 0;
            { //-- is = 0
                Xyz segT = ToolPolyh.Point(thePolyh, pTri[1]).XYZ() - ToolPolyh.Point(thePolyh, pTri[0]).XYZ();
                Xyz vecP = spLieu - ToolPolyh.Point(thePolyh, pTri[0]).XYZ();
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
                Xyz segT = ToolPolyh.Point(thePolyh, pTri[2]).XYZ() - ToolPolyh.Point(thePolyh, pTri[1]).XYZ();
                Xyz vecP = spLieu - ToolPolyh.Point(thePolyh, pTri[1]).XYZ();
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
                Xyz segT = ToolPolyh.Point(thePolyh, pTri[0]).XYZ() - ToolPolyh.Point(thePolyh, pTri[2]).XYZ();
                Xyz vecP = spLieu - ToolPolyh.Point(thePolyh, pTri[2]).XYZ();
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
                Intf_SectionPoint SP = new(new Point3D(spLieu), typOnG, 0, iLin, param, Intf_VERTEX, pTri[@is], 0, 0.0,
                                           1.0);
                mySPoins.Append(SP);
            }
            else if (sEdge > -1)
            {
                ToolPolyh.TriConnex(thePolyh, TTri, pTri[sEdge], pTri[Pourcent3[sEdge + 1]], out _, out _);
                Intf_SectionPoint SP = new(new Point3D(spLieu), typOnG, 0, iLin, param, Intf_EDGE,
                                           Min(pTri[sEdge], pTri[Pourcent3[sEdge + 1]]),
                                           Max(pTri[sEdge], pTri[Pourcent3[sEdge + 1]]), 0.0, 1.0);
                mySPoins.Append(SP);
            }
            else if (dPiE[0] > 0.0 && dPiE[1] > 0.0 && dPiE[2] > 0.0)
            {
                Intf_SectionPoint SP = new(new Point3D(spLieu), typOnG, 0, iLin, param, Intf_FACE, TTri, 0, 0.0, 1.0);
                mySPoins.Append(SP);
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

                    if (ToolPolyh.IsOnBound(thePolyh, indP1, indP2))
                    {
                        // For boundary line it is necessary to check the border deflection.
                        double Deflection = ToolPolyh.GetBorderDeflection(thePolyh);
                        Point3D BegP = ToolPolyh.Point(thePolyh, indP1);
                        Point3D EndP = ToolPolyh.Point(thePolyh, indP2);
                        Vector3D VecTri = new(BegP, EndP);
                        Direction3D DirTri = new(VecTri);
                        Line3D LinTri = new(BegP, DirTri);
                        Point3D aPOnE = new(spLieu);
                        double aDist = LinTri.Distance(aPOnE);

                        if (aDist <= Deflection)
                        {
                            Vector3D aVLocPOnE = new(BegP, aPOnE);
                            Vector3D aVecDirTri = new(DirTri);
                            double aPar = aVLocPOnE * aVecDirTri;
                            double aMaxPar = VecTri.Magnitude();

                            if (aPar >= 0 && aPar <= aMaxPar)
                            {
                                Intf_SectionPoint SP = new(new Point3D(spLieu), typOnG, 0, iLin, param, Intf_FACE, TTri,
                                                           0, 0.0, 1.0);
                                mySPoins.Append(SP);
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
            Vector3D VecPol = new(BegO, EndO);
            double NVecPol = VecPol.Magnitude();
            if (NVecPol < global::Kerf.Engine.Foundation.Precision.Computational())
            {
                return;
            }
            Direction3D DirPol = new(VecPol);
            Line3D LinPol = new(BegO, DirPol);
            double dist2;

            for (int i = 0; i < 3; i++)
            {
                int pTri_ip1pc3 = pTri[Pourcent3[i + 1]];
                int pTri_i = pTri[i];
                Point3D BegT = ToolPolyh.Point(thePolyh, pTri_ip1pc3);
                Point3D EndT = ToolPolyh.Point(thePolyh, pTri_i);
                Vector3D VecTri = new(BegT, EndT);
                double NVecTri = VecTri.Magnitude();
                if (NVecTri < global::Kerf.Engine.Foundation.Precision.Computational())
                {
                    continue;
                }
                Direction3D DirTri = new(VecTri);
                Line3D LinTri = new(BegT, DirTri);
                ExtElC Extrema = new(LinPol, LinTri, 0.00000001);
                if (Extrema.IsDone)
                {
                    if (Extrema.IsParallel == false)
                    {
                        if (Extrema.ExtCount != 0)
                        {
                            dist2 = Extrema.SquareDistance();
                            if (dist2 <= Tolerance * Tolerance)
                            {
                                Extrema.Points(1, out POnCurv POnC1, out POnCurv POnC2);
                                Point3D PO = POnC1.Value();
                                Point3D PT = POnC2.Value();
                                if (IsInSegment(VecPol, new Vector3D(BegO, PO), NVecPol, out double ParamOnO, Tolerance))
                                {
                                    if (IsInSegment(VecTri, new Vector3D(BegT, PT), NVecTri, out double ParamOnT,
                                                    Tolerance))
                                    {
                                        Xyz spLieu = BegT.XYZ() + ((EndT.XYZ() - BegT.XYZ()) * ParamOnT);
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
                                        Intf_SectionPoint SP = new(new Point3D(spLieu), typOnG, 0, iLin, ParamOnO,
                                                                   Intf_EDGE, tmin, tmax, 0.0, 1.0);
                                        mySPoins.Append(SP);
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
