// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_MaillageAffinage.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_MaillageAffinage.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using BVH_Vec3d = Occt.NCollection_Vec3<double>;
using IntPolyh_BoxBndTree = Occt.BVH_BoxSet<double, Occt.NCollection_Vec3<double>, int>;

namespace Occt;

/// <summary>
/// The low-level algorithm that intersects two surfaces by intersecting their triangulations: sampling, refinement
/// of the triangles by deflection, the interfering couples of triangles and the start points chained into section lines.
/// </summary>
/// <remarks>
/// A working object (decision 21), used by <see cref="IntPolyh_Intersection"/>; its arrays and couples are handed out
/// as OCCT does. Debug prints are left out. Calling the triangulation steps (<c>FillArrayOfTriangles</c>, ...) a second
/// time on the same object keeps the items the first pass left in the arrays beyond what it rewrites, as OCCT's does
/// (the enlarged-zone retry of <see cref="IntPolyh_Intersection"/> relies on it; see occt-issues.md, unit 5E).
/// </remarks>
public sealed partial class IntPolyh_MaillageAffinage
{
    private const double MyTolerance = 10.0e-7;
    private const double MyConfusionPrecision = 10.0e-12;
    private const double SquareMyConfusionPrecision = 10.0e-24;

    private readonly Adaptor3d_Surface MaSurface1;
    private readonly Adaptor3d_Surface MaSurface2;
    private Bnd_Box MyBox1 = new();
    private Bnd_Box MyBox2 = new();
    private readonly int NbSamplesU1;
    private readonly int NbSamplesU2;
    private readonly int NbSamplesV1;
    private readonly int NbSamplesV2;
    private double FlecheMax1;
    private double FlecheMax2;
    private double FlecheMin1;
    private double FlecheMin2;
    // For the arrays of Points, Edges and Triangles we need instant access to the items.
    // Moreover, we might add new items during refinement process in case the deflection
    // is too big, thus the vectors should be used.
    private readonly IntPolyh_Array<IntPolyh_Point> TPoints1 = new();
    private readonly IntPolyh_Array<IntPolyh_Point> TPoints2 = new();
    private readonly IntPolyh_Array<IntPolyh_Edge> TEdges1 = new();
    private readonly IntPolyh_Array<IntPolyh_Edge> TEdges2 = new();
    private readonly IntPolyh_Array<IntPolyh_Triangle> TTriangles1 = new();
    private readonly IntPolyh_Array<IntPolyh_Triangle> TTriangles2 = new();
    // The intersecting triangles are just filled and then
    // sequentially analyzed, thus we might use the list.
    private readonly NCollection_List<IntPolyh_Couple> TTrianglesContacts = new();
    private bool myEnlargeZone;

    /// <summary>The algorithm on two surfaces with 10 x 10 samples each (the last argument, OCCT's print level, is
    /// ignored).</summary>
    public IntPolyh_MaillageAffinage(Adaptor3d_Surface Surface1, Adaptor3d_Surface Surface2, int PRINT)
    {
        _ = PRINT;
        MaSurface1 = Surface1;
        MaSurface2 = Surface2;
        NbSamplesU1 = 10;
        NbSamplesU2 = 10;
        NbSamplesV1 = 10;
        NbSamplesV2 = 10;
        FlecheMax1 = 0.0;
        FlecheMax2 = 0.0;
        FlecheMin1 = 0.0;
        FlecheMin2 = 0.0;
        myEnlargeZone = false;
    }

    /// <summary>The algorithm on two surfaces with the given numbers of samples (the last argument, OCCT's print level,
    /// is ignored).</summary>
    public IntPolyh_MaillageAffinage(Adaptor3d_Surface Surface1, int NbSU1, int NbSV1, Adaptor3d_Surface Surface2,
                                     int NbSU2, int NbSV2, int PRINT)
    {
        _ = PRINT;
        MaSurface1 = Surface1;
        MaSurface2 = Surface2;
        NbSamplesU1 = NbSU1;
        NbSamplesU2 = NbSU2;
        NbSamplesV1 = NbSV1;
        NbSamplesV2 = NbSV2;
        FlecheMax1 = 0.0;
        FlecheMax2 = 0.0;
        FlecheMin1 = 0.0;
        FlecheMin2 = 0.0;
        myEnlargeZone = false;
    }

    //=================================================================================================
    // The BVH of the triangles' boxes

    /// <summary>Selects the pairs of elements of two BVH box sets whose boxes interfere.</summary>
    private sealed class IntPolyh_BoxBndTreeSelector : BVH_PairTraverse<double, BVH_Vec3d, IntPolyh_BoxBndTree, double>
    {
        /// <summary>A pair of indices, ordered by the first then the second.</summary>
        internal readonly record struct PairIDs(int ID1, int ID2) : IComparable<PairIDs>
        {
            public int CompareTo(PairIDs theOther)
                => ID1 < theOther.ID1 || (ID1 == theOther.ID1 && ID2 < theOther.ID2) ? -1
                   : (ID1 == theOther.ID1 && ID2 == theOther.ID2 ? 0 : 1);
        }

        private readonly List<PairIDs> myPairs = [];

        public override bool RejectNode(in BVH_Vec3d theCMin1, in BVH_Vec3d theCMax1, in BVH_Vec3d theCMin2,
                                        in BVH_Vec3d theCMax2, out double theMetric)
        {
            theMetric = 0.0;
            return new BVH_Box<double, BVH_Vec3d>(theCMin1, theCMax1).IsOut(theCMin2, theCMax2);
        }

        public override bool Accept(int theID1, int theID2)
        {
            if (!myBVHSet1!.Box(theID1).IsOut(myBVHSet2!.Box(theID2)))
            {
                myPairs.Add(new PairIDs(myBVHSet1.Element(theID1), myBVHSet2.Element(theID2)));
                return true;
            }

            return false;
        }

        /// <summary>The selected pairs.</summary>
        public List<PairIDs> Pairs() => myPairs;

        /// <summary>
        /// Sorts the pairs (OCCT's <c>std::sort</c>). The pairs are distinct (each pair of leaves is visited once and
        /// the elements of a set are distinct triangles), so there are no ties and any sort gives MSVC's order
        /// (decision 5502).
        /// </summary>
        public void Sort() => myPairs.Sort();
    }

    // The triangles with interfering bounding boxes: for each triangle of the first array (in increasing order), the
    // triangles of the second (in increasing order)
    private static void GetInterferingTriangles(IntPolyh_Array<IntPolyh_Triangle> theTriangles1,
                                                IntPolyh_Array<IntPolyh_Point> thePoints1,
                                                IntPolyh_Array<IntPolyh_Triangle> theTriangles2,
                                                IntPolyh_Array<IntPolyh_Point> thePoints2,
                                                NCollection_IndexedDataMap<int, NCollection_List<int>> theCouples)
    {
        // Use linear builder for BVH construction
        BVH_LinearBuilder<double, BVH_Vec3d> aLBuilder = new(10);

        // To find the triangles with interfering bounding boxes
        // use the BVH structure
        IntPolyh_BoxBndTree aBBTree1 = new(aLBuilder), aBBTree2 = new(aLBuilder);

        // 1. Fill the trees with the boxes of the surfaces triangles
        for (int i = 0; i < 2; ++i)
        {
            IntPolyh_BoxBndTree aBBTree = i == 0 ? aBBTree1 : aBBTree2;
            IntPolyh_Array<IntPolyh_Triangle> aTriangles = i == 0 ? theTriangles1 : theTriangles2;
            IntPolyh_Array<IntPolyh_Point> aPoints = i == 0 ? thePoints1 : thePoints2;

            int aNbT = aTriangles.NbItems();
            aBBTree.SetSize(aNbT);
            for (int j = 0; j < aNbT; ++j)
            {
                ref IntPolyh_Triangle aT = ref aTriangles[j];
                if (!aT.IsIntersectionPossible() || aT.IsDegenerated())
                {
                    continue;
                }

                aBBTree.Add(j, Bnd_Tools.Bnd2BVH(aT.BoundingBox(aPoints)));
            }

            if (aBBTree.Size() == 0)
            {
                return;
            }
        }

        // 2. Construct BVH trees
        aBBTree1.Build();
        aBBTree2.Build();

        // 3. Perform selection of the interfering triangles
        IntPolyh_BoxBndTreeSelector aSelector = new();
        aSelector.SetBVHSets(aBBTree1, aBBTree2);
        aSelector.Select();
        aSelector.Sort();

        List<IntPolyh_BoxBndTreeSelector.PairIDs> aPairs = aSelector.Pairs();
        int aNbPairs = aPairs.Count;

        for (int i = 0; i < aNbPairs; ++i)
        {
            IntPolyh_BoxBndTreeSelector.PairIDs aPair = aPairs[i];
            if (!theCouples.FindFromKey(aPair.ID1, out NCollection_List<int>? pTriangles2))
            {
                pTriangles2 = new NCollection_List<int>();
                theCouples.Add(aPair.ID1, pTriangles2);
            }

            pTriangles2!.Append(aPair.ID2);
        }
    }

    //=================================================================================================
    // Sampling

    /// <summary>Makes the sampling of the surface: the parameters of the triangulation's nodes.</summary>
    public void MakeSampling(int SurfID, NCollection_Array1<double> theUPars, NCollection_Array1<double> theVPars)
    {
        if (SurfID == 1)
        {
            IntPolyh_Tools.MakeSampling(MaSurface1, NbSamplesU1, NbSamplesV1, myEnlargeZone, theUPars, theVPars);
        }
        else
        {
            IntPolyh_Tools.MakeSampling(MaSurface2, NbSamplesU2, NbSamplesV2, myEnlargeZone, theUPars, theVPars);
        }
    }

    /// <summary>Computes the points of one surface at its own sampling (the standard method).</summary>
    public void FillArrayOfPnt(int SurfID)
    {
        // Make sampling
        NCollection_Array1<double> aUpars = new(), aVpars = new();
        MakeSampling(SurfID, aUpars, aVpars);
        // Fill array of points
        FillArrayOfPnt(SurfID, aUpars, aVpars);
    }

    /// <summary>Computes the points of one surface at its own sampling, shifted along the normal by the deflection
    /// (forward or reversed: the advanced method).</summary>
    public void FillArrayOfPnt(int SurfID, bool isShiftFwd)
    {
        // Make sampling
        NCollection_Array1<double> aUpars = new(), aVpars = new();
        MakeSampling(SurfID, aUpars, aVpars);
        // Fill array of points
        FillArrayOfPnt(SurfID, isShiftFwd, aUpars, aVpars);
    }

    /// <summary>
    /// Computes the points of one surface at the given sampling and its box, enlarged by 1.2 times the deflection
    /// tolerance (<paramref name="theDeflTol"/> if given, else computed).
    /// </summary>
    public void FillArrayOfPnt(int SurfID, NCollection_Array1<double> Upars, NCollection_Array1<double> Vpars,
                               double? theDeflTol = null)
    {
        bool bDegI, bDeg;
        int aNbU, aNbV, iCnt, i, j;
        int aID1, aID2, aJD1, aJD2;
        double aTol, aU, aV, aX, aY, aZ;
        gp_Pnt aP;
        //
        aNbU = (SurfID == 1) ? NbSamplesU1 : NbSamplesU2;
        aNbV = (SurfID == 1) ? NbSamplesV1 : NbSamplesV2;
        ref Bnd_Box aBox = ref (SurfID == 1) ? ref MyBox1 : ref MyBox2;
        Adaptor3d_Surface aS = (SurfID == 1) ? MaSurface1 : MaSurface2;
        IntPolyh_Array<IntPolyh_Point> TPoints = (SurfID == 1) ? TPoints1 : TPoints2;
        //
        DegeneratedIndex(Vpars, aNbV, aS, 1, out aJD1, out aJD2);
        aID1 = 0;
        aID2 = 0;
        if (!(aJD1 != 0 || aJD2 != 0))
        {
            DegeneratedIndex(Upars, aNbU, aS, 2, out aID1, out aID2);
        }
        //
        TPoints.Init(aNbU * aNbV);
        iCnt = 0;
        for (i = 1; i <= aNbU; ++i)
        {
            bDegI = (aID1 == i || aID2 == i);
            aU = Upars[i];
            for (j = 1; j <= aNbV; ++j)
            {
                aV = Vpars[j];
                aP = aS.Value(aU, aV);
                aP.Coord(out aX, out aY, out aZ);
                ref IntPolyh_Point aIP = ref TPoints[iCnt];
                aIP.Set(aX, aY, aZ, aU, aV);
                //
                bDeg = bDegI || (aJD1 == j || aJD2 == j);
                if (bDeg)
                {
                    aIP.SetDegenerated(bDeg);
                }

                ++iCnt;
                aBox.Add(aP);
            }
        }
        //
        TPoints.SetNbItems(iCnt);
        //
        aTol = theDeflTol is null ? IntPolyh_Tools.ComputeDeflection(aS, Upars, Vpars) : theDeflTol.Value;
        aTol *= 1.2;

        aBox.Get(out double a1, out double a2, out double a3, out double b1, out double b2, out double b3);
        aBox.Update(a1 - aTol, a2 - aTol, a3 - aTol, b1 + aTol, b2 + aTol, b3 + aTol);
        aBox.Enlarge(MyTolerance);
    }

    /// <summary>
    /// Computes the points of one surface from the given points and normals, shifted along the normal by 1.5 times the
    /// deflection tolerance (forward or reversed), and its box.
    /// </summary>
    public void FillArrayOfPnt(int SurfID, bool isShiftFwd, IntPolyh_Array<IntPolyh_PointNormal> thePointsNorm,
                               NCollection_Array1<double> theUPars, NCollection_Array1<double> theVPars,
                               double theDeflTol)
    {
        Adaptor3d_Surface aS = (SurfID == 1) ? MaSurface1 : MaSurface2;
        IntPolyh_Array<IntPolyh_Point> TPoints = (SurfID == 1) ? TPoints1 : TPoints2;
        int aNbU = (SurfID == 1) ? NbSamplesU1 : NbSamplesU2;
        int aNbV = (SurfID == 1) ? NbSamplesV1 : NbSamplesV2;
        ref Bnd_Box aBox = ref (SurfID == 1) ? ref MyBox1 : ref MyBox2;

        int aID1 = 0, aID2 = 0;
        DegeneratedIndex(theVPars, aNbV, aS, 1, out int aJD1, out int aJD2);
        if (!(aJD1 != 0 || aJD2 != 0))
        {
            DegeneratedIndex(theUPars, aNbU, aS, 2, out aID1, out aID2);
        }

        bool bDegI, bDeg;
        int iCnt = 0, i, j;
        double aX, aY, aZ, aU, aV;

        TPoints.Init(thePointsNorm.NbItems());

        for (i = 1; i <= aNbU; ++i)
        {
            aU = theUPars[i];
            bDegI = (aID1 == i || aID2 == i);
            for (j = 1; j <= aNbV; ++j)
            {
                aV = theVPars[j];

                ref readonly IntPolyh_PointNormal aPN = ref thePointsNorm.Value(iCnt);
                gp_Vec aNorm = aPN.Normal.Multiplied(1.5 * theDeflTol);
                if (!isShiftFwd)
                {
                    aNorm.Reverse();
                }

                gp_Pnt aP = aPN.Point.Translated(aNorm);

                ref IntPolyh_Point aIP = ref TPoints[iCnt];
                aP.Coord(out aX, out aY, out aZ);
                aIP.Set(aX, aY, aZ, aU, aV);
                bDeg = bDegI || (aJD1 == j || aJD2 == j);
                if (bDeg)
                {
                    aIP.SetDegenerated(bDeg);
                }

                ++iCnt;
                aBox.Add(aP);
            }
        }

        TPoints.SetNbItems(iCnt);

        // Update box
        double Tol = theDeflTol * 1.2;
        aBox.Get(out double a1, out double a2, out double a3, out double b1, out double b2, out double b3);
        aBox.Update(a1 - Tol, a2 - Tol, a3 - Tol, b1 + Tol, b2 + Tol, b3 + Tol);
        aBox.Enlarge(MyTolerance);
    }

    /// <summary>
    /// Computes the points of one surface at the given sampling, shifted along the normal (the advanced method).
    /// </summary>
    /// <remarks>OCCT fills the first surface's points whatever <paramref name="SurfID"/> is (occt-issues.md, unit 5E).</remarks>
    public void FillArrayOfPnt(int SurfID, bool isShiftFwd, NCollection_Array1<double> Upars,
                               NCollection_Array1<double> Vpars, double? theDeflTol = null)
    {
        Adaptor3d_Surface aS = (SurfID == 1) ? MaSurface1 : MaSurface2;
        // Compute the tolerance
        double aTol = theDeflTol is not null ? theDeflTol.Value : IntPolyh_Tools.ComputeDeflection(aS, Upars, Vpars);
        // Fill array of point normal
        IntPolyh_Array<IntPolyh_PointNormal> aPoints = new();
        IntPolyh_Tools.FillArrayOfPointNormal(aS, Upars, Vpars, aPoints);

        // Fill array of points
        FillArrayOfPnt(1, isShiftFwd, aPoints, Upars, Vpars, aTol); // OCCT: 1, not SurfID
    }

    //=================================================================================================
    // The common box

    /// <summary>Looks for the common box of the surfaces and marks the points inside it.</summary>
    public void CommonBox()
    {
        CommonBox(GetBox(1), GetBox(2), out _, out _, out _, out _, out _, out _);
    }

    /// <summary>
    /// Computes the common box (the intersection of the two surfaces' boxes, enlarged by a tenth), and marks the points
    /// of both surfaces by their position against it (<c>PartOfCommon</c>: 0 inside, else bits of the sides they are
    /// out of). The box arguments are not used: the surfaces' own boxes are.
    /// </summary>
    public void CommonBox(in Bnd_Box B1, in Bnd_Box B2, out double XMin, out double YMin, out double ZMin,
                          out double XMax, out double YMax, out double ZMax)
    {
        _ = B1; // OCCT: the box arguments are not used
        _ = B2;
        MyBox1.Get(out double x10, out double y10, out double z10, out double x11, out double y11, out double z11);
        MyBox2.Get(out double x20, out double y20, out double z20, out double x21, out double y21, out double z21);
        XMin = 0.0;
        YMin = 0.0;
        ZMin = 0.0;
        XMax = 0.0;
        YMax = 0.0;
        ZMax = 0.0;

        if ((x10 > x21) || (x20 > x11) || (y10 > y21) || (y20 > y11) || (z10 > z21) || (z20 > z11))
        {
        }
        else
        {
            if (x11 <= x21)
            {
                XMax = x11;
            }
            else
            {
                if (x21 <= x11)
                {
                    XMax = x21;
                }
            }

            if (x20 <= x10)
            {
                XMin = x10;
            }
            else
            {
                if (x10 <= x20)
                {
                    XMin = x20;
                }
            }

            if (y11 <= y21)
            {
                YMax = y11;
            }
            else
            {
                if (y21 <= y11)
                {
                    YMax = y21;
                }
            }

            if (y20 <= y10)
            {
                YMin = y10;
            }
            else
            {
                if (y10 <= y20)
                {
                    YMin = y20;
                }
            }

            if (z11 <= z21)
            {
                ZMax = z11;
            }
            else
            {
                if (z21 <= z11)
                {
                    ZMax = z21;
                }
            }

            if (z20 <= z10)
            {
                ZMin = z10;
            }
            else
            {
                if (z10 <= z20)
                {
                    ZMin = z20;
                }
            }
        }
        //
        double X, Y, Z;
        X = XMax - XMin;
        Y = YMax - YMin;
        Z = ZMax - ZMin;
        // extension of the box
        if ((X == 0) && (Y != 0))
        {
            X = Y * 0.1;
        }
        else if ((X == 0) && (Z != 0))
        {
            X = Z * 0.1;
        }
        else
        {
            X *= 0.1;
        }

        if ((Y == 0) && (X != 0))
        {
            Y = X * 0.1;
        }
        else if ((Y == 0) && (Z != 0))
        {
            Y = Z * 0.1;
        }
        else
        {
            Y *= 0.1;
        }

        if ((Z == 0) && (X != 0))
        {
            Z = X * 0.1;
        }
        else if ((Z == 0) && (Y != 0))
        {
            Z = Y * 0.1;
        }
        else
        {
            Z *= 0.1;
        }

        XMin -= X;
        XMax += X;
        YMin -= Y;
        YMax += Y;
        ZMin -= Z;
        ZMax += Z;

        // Marking of points included in the common
        int FinTP1 = TPoints1.NbItems();
        int i;
        for (i = 0; i < FinTP1; i++)
        {
            ref IntPolyh_Point Pt1 = ref TPoints1[i];
            int r;
            if (Pt1.X() < XMin)
            {
                r = 1;
            }
            else
            {
                if (Pt1.X() > XMax)
                {
                    r = 2;
                }
                else
                {
                    r = 0;
                }
            }

            if (Pt1.Y() < YMin)
            {
                r |= 4;
            }
            else
            {
                if (Pt1.Y() > YMax)
                {
                    r |= 8;
                }
            }

            if (Pt1.Z() < ZMin)
            {
                r |= 16;
            }
            else
            {
                if (Pt1.Z() > ZMax)
                {
                    r |= 32;
                }
            }

            Pt1.SetPartOfCommon(r);
        }

        int FinTP2 = TPoints2.NbItems();
        for (int ii = 0; ii < FinTP2; ii++)
        {
            ref IntPolyh_Point Pt2 = ref TPoints2[ii];
            int rr;
            if (Pt2.X() < XMin)
            {
                rr = 1;
            }
            else
            {
                if (Pt2.X() > XMax)
                {
                    rr = 2;
                }
                else
                {
                    rr = 0;
                }
            }

            if (Pt2.Y() < YMin)
            {
                rr |= 4;
            }
            else
            {
                if (Pt2.Y() > YMax)
                {
                    rr |= 8;
                }
            }

            if (Pt2.Z() < ZMin)
            {
                rr |= 16;
            }
            else
            {
                if (Pt2.Z() > ZMax)
                {
                    rr |= 32;
                }
            }

            Pt2.SetPartOfCommon(rr);
        }
    }

    //=================================================================================================
    // The triangulation

    /// <summary>Computes the edges of the triangulation of one surface, and links them to its triangles.</summary>
    public void FillArrayOfEdges(int SurfID)
    {
        IntPolyh_Array<IntPolyh_Edge> TEdges = (SurfID == 1) ? TEdges1 : TEdges2;
        IntPolyh_Array<IntPolyh_Triangle> TTriangles = (SurfID == 1) ? TTriangles1 : TTriangles2;
        int NbSamplesU = (SurfID == 1) ? NbSamplesU1 : NbSamplesU2;
        int NbSamplesV = (SurfID == 1) ? NbSamplesV1 : NbSamplesV2;

        // NbEdges = 3 + 3*(NbSamplesV-2) + 3*(NbSamplesU-2) +
        //         + 3*(NbSamplesU-2)*(NbSamplesV-2) + (NbSamplesV-1) + (NbSamplesU-1);
        // NbSamplesU and NbSamples cannot be less than 2, so
        int NbEdges = 3 * NbSamplesU * NbSamplesV - 2 * (NbSamplesU + NbSamplesV) + 1;
        TEdges.Init(NbEdges);

        int CpteurTabEdges = 0;

        // mesh u0 v0
        TEdges[CpteurTabEdges].SetFirstPoint(0);  // U V
        TEdges[CpteurTabEdges].SetSecondPoint(1); // U V+1
        TEdges[CpteurTabEdges].SetSecondTriangle(0);
        TTriangles[0].SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
        CpteurTabEdges++;

        TEdges[CpteurTabEdges].SetFirstPoint(0);           // U V
        TEdges[CpteurTabEdges].SetSecondPoint(NbSamplesV); // U+1 V
        TEdges[CpteurTabEdges].SetFirstTriangle(1);
        TTriangles[1].SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
        CpteurTabEdges++;

        TEdges[CpteurTabEdges].SetFirstPoint(0);               // U V
        TEdges[CpteurTabEdges].SetSecondPoint(NbSamplesV + 1); // U+1 V+1
        TEdges[CpteurTabEdges].SetFirstTriangle(0);
        TTriangles[0].SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
        TEdges[CpteurTabEdges].SetSecondTriangle(1);
        TTriangles[1].SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
        CpteurTabEdges++;

        // mesh on U=u0
        int PntInit = 1;
        int BoucleMeshV;
        for (BoucleMeshV = 1; BoucleMeshV < NbSamplesV - 1; BoucleMeshV++)
        {
            TEdges[CpteurTabEdges].SetFirstPoint(PntInit);      // U V
            TEdges[CpteurTabEdges].SetSecondPoint(PntInit + 1); // U V+1
            TEdges[CpteurTabEdges].SetSecondTriangle(BoucleMeshV * 2);
            TTriangles[BoucleMeshV * 2].SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
            CpteurTabEdges++;

            TEdges[CpteurTabEdges].SetFirstPoint(PntInit);                   // U V
            TEdges[CpteurTabEdges].SetSecondPoint(PntInit + NbSamplesV + 1); // U+1 V+1
            TEdges[CpteurTabEdges].SetFirstTriangle(BoucleMeshV * 2);
            TTriangles[BoucleMeshV * 2].SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
            TEdges[CpteurTabEdges].SetSecondTriangle(BoucleMeshV * 2 + 1);
            TTriangles[BoucleMeshV * 2 + 1].SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
            CpteurTabEdges++;

            TEdges[CpteurTabEdges].SetFirstPoint(PntInit);               // U V
            TEdges[CpteurTabEdges].SetSecondPoint(PntInit + NbSamplesV); // U+1 V
            TEdges[CpteurTabEdges].SetFirstTriangle(BoucleMeshV * 2 + 1);
            TTriangles[BoucleMeshV * 2 + 1].SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
            TEdges[CpteurTabEdges].SetSecondTriangle(BoucleMeshV * 2 - 2);
            TTriangles[BoucleMeshV * 2 - 2].SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
            CpteurTabEdges++;
            PntInit++;
        }

        // mesh on V=v0
        PntInit = NbSamplesV;
        for (BoucleMeshV = 1; BoucleMeshV < NbSamplesU - 1; BoucleMeshV++)
        {
            TEdges[CpteurTabEdges].SetFirstPoint(PntInit);      // U V
            TEdges[CpteurTabEdges].SetSecondPoint(PntInit + 1); // U V+1
            TEdges[CpteurTabEdges].SetFirstTriangle((BoucleMeshV - 1) * (NbSamplesV - 1) * 2 + 1);
            TTriangles[(BoucleMeshV - 1) * (NbSamplesV - 1) * 2 + 1].SetEdgeAndOrientation(TEdges[CpteurTabEdges],
                                                                                           CpteurTabEdges);
            TEdges[CpteurTabEdges].SetSecondTriangle(BoucleMeshV * (NbSamplesV - 1) * 2);
            TTriangles[BoucleMeshV * (NbSamplesV - 1) * 2].SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
            CpteurTabEdges++;

            TEdges[CpteurTabEdges].SetFirstPoint(PntInit);                   // U V
            TEdges[CpteurTabEdges].SetSecondPoint(PntInit + NbSamplesV + 1); // U+1 V+1
            TEdges[CpteurTabEdges].SetFirstTriangle(BoucleMeshV * (NbSamplesV - 1) * 2);
            TTriangles[BoucleMeshV * (NbSamplesV - 1) * 2].SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
            TEdges[CpteurTabEdges].SetSecondTriangle(BoucleMeshV * (NbSamplesV - 1) * 2 + 1);
            TTriangles[BoucleMeshV * (NbSamplesV - 1) * 2 + 1].SetEdgeAndOrientation(TEdges[CpteurTabEdges],
                                                                                     CpteurTabEdges);
            CpteurTabEdges++;

            TEdges[CpteurTabEdges].SetFirstPoint(PntInit);               // U V
            TEdges[CpteurTabEdges].SetSecondPoint(PntInit + NbSamplesV); // U+1 V
            TEdges[CpteurTabEdges].SetFirstTriangle(BoucleMeshV * (NbSamplesV - 1) * 2 + 1);
            TTriangles[BoucleMeshV * (NbSamplesV - 1) * 2 + 1].SetEdgeAndOrientation(TEdges[CpteurTabEdges],
                                                                                     CpteurTabEdges);
            CpteurTabEdges++;
            PntInit += NbSamplesV;
        }

        PntInit = NbSamplesV + 1;
        // To provide recursion I associate a point with three edges
        for (int BoucleMeshU = 1; BoucleMeshU < NbSamplesU - 1; BoucleMeshU++)
        {
            for (BoucleMeshV = 1; BoucleMeshV < NbSamplesV - 1; BoucleMeshV++)
            {
                TEdges[CpteurTabEdges].SetFirstPoint(PntInit);      // U V
                TEdges[CpteurTabEdges].SetSecondPoint(PntInit + 1); // U V+1
                TEdges[CpteurTabEdges].SetFirstTriangle((NbSamplesV - 1) * 2 * (BoucleMeshU - 1) + BoucleMeshV * 2 + 1);
                TTriangles[(NbSamplesV - 1) * 2 * (BoucleMeshU - 1) + BoucleMeshV * 2 + 1]
                    .SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
                TEdges[CpteurTabEdges].SetSecondTriangle((NbSamplesV - 1) * 2 * BoucleMeshU + BoucleMeshV * 2);
                TTriangles[(NbSamplesV - 1) * 2 * BoucleMeshU + BoucleMeshV * 2]
                    .SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
                CpteurTabEdges++;

                TEdges[CpteurTabEdges].SetFirstPoint(PntInit);                   // U V
                TEdges[CpteurTabEdges].SetSecondPoint(PntInit + NbSamplesV + 1); // U+1 V+1
                TEdges[CpteurTabEdges].SetFirstTriangle((NbSamplesV - 1) * 2 * BoucleMeshU + BoucleMeshV * 2);
                TTriangles[(NbSamplesV - 1) * 2 * BoucleMeshU + BoucleMeshV * 2]
                    .SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
                TEdges[CpteurTabEdges].SetSecondTriangle((NbSamplesV - 1) * 2 * BoucleMeshU + BoucleMeshV * 2 + 1);
                TTriangles[(NbSamplesV - 1) * 2 * BoucleMeshU + BoucleMeshV * 2 + 1]
                    .SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
                CpteurTabEdges++;

                TEdges[CpteurTabEdges].SetFirstPoint(PntInit);               // U V
                TEdges[CpteurTabEdges].SetSecondPoint(PntInit + NbSamplesV); // U+1 V
                TEdges[CpteurTabEdges].SetFirstTriangle((NbSamplesV - 1) * 2 * BoucleMeshU + BoucleMeshV * 2 + 1);
                TTriangles[(NbSamplesV - 1) * 2 * BoucleMeshU + BoucleMeshV * 2 + 1]
                    .SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
                TEdges[CpteurTabEdges].SetSecondTriangle((NbSamplesV - 1) * 2 * BoucleMeshU + BoucleMeshV * 2 - 2);
                TTriangles[(NbSamplesV - 1) * 2 * BoucleMeshU + BoucleMeshV * 2 - 2]
                    .SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
                CpteurTabEdges++;
                PntInit++; // Pass to the next point
            }

            PntInit++; // Pass the last point of the column
            PntInit++; // Pass the first point of the next column
        }

        // close mesh on U=u1
        PntInit = (NbSamplesU - 1) * NbSamplesV; // point U=u1 V=0
        for (BoucleMeshV = 0; BoucleMeshV < NbSamplesV - 1; BoucleMeshV++)
        {
            TEdges[CpteurTabEdges].SetFirstPoint(PntInit);      // U=u1 V
            TEdges[CpteurTabEdges].SetSecondPoint(PntInit + 1); // U=u1 V+1
            TEdges[CpteurTabEdges].SetFirstTriangle((NbSamplesU - 2) * (NbSamplesV - 1) * 2 + BoucleMeshV * 2 + 1);
            TTriangles[(NbSamplesU - 2) * (NbSamplesV - 1) * 2 + BoucleMeshV * 2 + 1]
                .SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
            CpteurTabEdges++;
            PntInit++;
        }

        // close mesh on V=v1
        for (BoucleMeshV = 0; BoucleMeshV < NbSamplesU - 1; BoucleMeshV++)
        {
            TEdges[CpteurTabEdges].SetFirstPoint(NbSamplesV - 1 + BoucleMeshV * NbSamplesV);         // U V=v1
            TEdges[CpteurTabEdges].SetSecondPoint(NbSamplesV - 1 + (BoucleMeshV + 1) * NbSamplesV);  // U+1 V=v1
            TEdges[CpteurTabEdges].SetSecondTriangle(BoucleMeshV * 2 * (NbSamplesV - 1) + (NbSamplesV - 2) * 2);
            TTriangles[BoucleMeshV * 2 * (NbSamplesV - 1) + (NbSamplesV - 2) * 2]
                .SetEdgeAndOrientation(TEdges[CpteurTabEdges], CpteurTabEdges);
            CpteurTabEdges++;
        }

        TEdges.SetNbItems(CpteurTabEdges);
    }

    /// <summary>
    /// Computes the triangles of one surface from its points (two per cell of the sampling), and marks the triangles
    /// whose points are all on one side out of the common box as not intersecting.
    /// </summary>
    public void FillArrayOfTriangles(int SurfID)
    {
        int CpteurTabTriangles = 0;
        int PntInit = 0;

        IntPolyh_Array<IntPolyh_Point> TPoints = (SurfID == 1) ? TPoints1 : TPoints2;
        IntPolyh_Array<IntPolyh_Triangle> TTriangles = (SurfID == 1) ? TTriangles1 : TTriangles2;
        int NbSamplesU = (SurfID == 1) ? NbSamplesU1 : NbSamplesU2;
        int NbSamplesV = (SurfID == 1) ? NbSamplesV1 : NbSamplesV2;

        TTriangles.Init(2 * (NbSamplesU - 1) * (NbSamplesV - 1));
        // To provide recursion, I associate a point with two triangles
        for (int BoucleMeshU = 0; BoucleMeshU < NbSamplesU - 1; BoucleMeshU++)
        {
            for (int BoucleMeshV = 0; BoucleMeshV < NbSamplesV - 1; BoucleMeshV++)
            {
                // FIRST TRIANGLE
                TTriangles[CpteurTabTriangles].SetFirstPoint(PntInit);                  // U V
                TTriangles[CpteurTabTriangles].SetSecondPoint(PntInit + 1);             // U V+1
                TTriangles[CpteurTabTriangles].SetThirdPoint(PntInit + NbSamplesV + 1); // U+1 V+1

                // IF ITS EDGE CONTACTS WITH THE COMMON BOX IP REMAINS = A 1
                if (((TPoints[PntInit].PartOfCommon() & TPoints[PntInit + 1].PartOfCommon()) != 0)
                    && ((TPoints[PntInit + 1].PartOfCommon() & TPoints[PntInit + NbSamplesV + 1].PartOfCommon()) != 0)
                    && ((TPoints[PntInit + NbSamplesV + 1].PartOfCommon() & TPoints[PntInit].PartOfCommon()) != 0))
                {
                    // IF NOT IP=0
                    TTriangles[CpteurTabTriangles].SetIntersectionPossible(false);
                }

                CpteurTabTriangles++;

                // SECOND TRIANGLE
                TTriangles[CpteurTabTriangles].SetFirstPoint(PntInit);                   // U V
                TTriangles[CpteurTabTriangles].SetSecondPoint(PntInit + NbSamplesV + 1); // U+1 V+1
                TTriangles[CpteurTabTriangles].SetThirdPoint(PntInit + NbSamplesV);      // U+1 V

                if (((TPoints[PntInit].PartOfCommon() & TPoints[PntInit + NbSamplesV + 1].PartOfCommon()) != 0)
                    && ((TPoints[PntInit + NbSamplesV + 1].PartOfCommon() & TPoints[PntInit + NbSamplesV].PartOfCommon()) != 0)
                    && ((TPoints[PntInit + NbSamplesV].PartOfCommon() & TPoints[PntInit].PartOfCommon()) != 0))
                {
                    TTriangles[CpteurTabTriangles].SetIntersectionPossible(false);
                }

                CpteurTabTriangles++;

                PntInit++; // Pass to the next point
            }

            PntInit++; // Pass the last point of the column
        }

        TTriangles.SetNbItems(CpteurTabTriangles);
    }

    //=================================================================================================
    // Refinement

    /// <summary>Refines all the marked triangles of both surfaces once.</summary>
    public void CommonPartRefinement()
    {
        int FinInit1 = TTriangles1.NbItems();
        for (int i = 0; i < FinInit1; i++)
        {
            if (TTriangles1[i].IsIntersectionPossible())
            {
                TTriangles1[i].MiddleRefinement(i, MaSurface1, TPoints1, TTriangles1, TEdges1);
            }
        }

        int FinInit2 = TTriangles2.NbItems();
        for (int ii = 0; ii < FinInit2; ii++)
        {
            if (TTriangles2[ii].IsIntersectionPossible())
            {
                TTriangles2[ii].MiddleRefinement(ii, MaSurface2, TPoints2, TTriangles2, TEdges2);
            }
        }
    }

    /// <summary>Refines all the marked triangles of one surface once.</summary>
    public void LocalSurfaceRefinement(int SurfID)
    {
        // refine locally, but systematically the chosen surface
        if (SurfID == 1)
        {
            int FinInit1 = TTriangles1.NbItems();
            for (int i = 0; i < FinInit1; i++)
            {
                if (TTriangles1[i].IsIntersectionPossible())
                {
                    TTriangles1[i].MiddleRefinement(i, MaSurface1, TPoints1, TTriangles1, TEdges1);
                }
            }
        }
        //
        if (SurfID == 2)
        {
            int FinInit2 = TTriangles2.NbItems();
            for (int ii = 0; ii < FinInit2; ii++)
            {
                if (TTriangles2[ii].IsIntersectionPossible())
                {
                    TTriangles2[ii].MiddleRefinement(ii, MaSurface2, TPoints2, TTriangles2, TEdges2);
                }
            }
        }
    }

    /// <summary>Computes the deflection of every triangle of one surface, and their minimum and maximum.</summary>
    public void ComputeDeflections(int SurfID)
    {
        Adaptor3d_Surface aSurface = (SurfID == 1) ? MaSurface1 : MaSurface2;
        IntPolyh_Array<IntPolyh_Point> TPoints = (SurfID == 1) ? TPoints1 : TPoints2;
        IntPolyh_Array<IntPolyh_Triangle> TTriangles = (SurfID == 1) ? TTriangles1 : TTriangles2;
        ref double FlecheMin = ref (SurfID == 1) ? ref FlecheMin1 : ref FlecheMin2;
        ref double FlecheMax = ref (SurfID == 1) ? ref FlecheMax1 : ref FlecheMax2;

        FlecheMax = -RealLast();
        FlecheMin = RealLast();
        int FinTT = TTriangles.NbItems();

        for (int i = 0; i < FinTT; i++)
        {
            ref IntPolyh_Triangle aTriangle = ref TTriangles[i];
            double Fleche = aTriangle.ComputeDeflection(aSurface, TPoints);
            if (Fleche > FlecheMax)
            {
                FlecheMax = Fleche;
            }

            if (Fleche < FlecheMin)
            {
                FlecheMin = Fleche;
            }
        }
    }

    // Refinement of the triangles depending on the deflection
    private static void TrianglesDeflectionsRefinement(Adaptor3d_Surface theS1,
                                                       IntPolyh_Array<IntPolyh_Triangle> theTriangles1,
                                                       IntPolyh_Array<IntPolyh_Edge> theEdges1,
                                                       IntPolyh_Array<IntPolyh_Point> thePoints1,
                                                       double theFlecheCritique1,
                                                       Adaptor3d_Surface theS2,
                                                       IntPolyh_Array<IntPolyh_Triangle> theTriangles2,
                                                       IntPolyh_Array<IntPolyh_Edge> theEdges2,
                                                       IntPolyh_Array<IntPolyh_Point> thePoints2,
                                                       double theFlecheCritique2)
    {
        // Find the intersecting triangles
        NCollection_IndexedDataMap<int, NCollection_List<int>> aDMILI = new();
        GetInterferingTriangles(theTriangles1, thePoints1, theTriangles2, thePoints2, aDMILI);
        //
        // Interfering triangles of second surface
        NCollection_Map<int> aMIntS2 = new();
        // Since the number of the triangles may change during refinement,
        // save the number of triangles before refinement
        int FinTT1 = theTriangles1.NbItems();
        int FinTT2 = theTriangles2.NbItems();
        //
        // Analyze interfering triangles
        for (int i_S1 = 0; i_S1 < FinTT1; i_S1++)
        {
            ref IntPolyh_Triangle aTriangle1 = ref theTriangles1[i_S1];
            if (!aTriangle1.IsIntersectionPossible())
            {
                continue;
            }
            //
            if (!aDMILI.FindFromKey(i_S1, out NCollection_List<int>? pLI) || pLI is null || pLI.IsEmpty())
            {
                // Mark non-interfering triangles of S1 to avoid their repeated usage
                aTriangle1.SetIntersectionPossible(false);
                continue;
            }
            //
            if (aTriangle1.Deflection() > theFlecheCritique1)
            {
                aTriangle1.MiddleRefinement(i_S1, theS1, thePoints1, theTriangles1, theEdges1);
            }
            //
            for (NCollection_List<int>.Iterator Iter = new(pLI); Iter.More(); Iter.Next())
            {
                int i_S2 = Iter.Value();
                if (aMIntS2.Add(i_S2))
                {
                    ref IntPolyh_Triangle aTriangle2 = ref theTriangles2[i_S2];
                    if (aTriangle2.Deflection() > theFlecheCritique2)
                    {
                        // Refinement of the larger triangles
                        aTriangle2.MiddleRefinement(i_S2, theS2, thePoints2, theTriangles2, theEdges2);
                    }
                }
            }
        }
        //
        // Mark non-interfering triangles of S2 to avoid their repeated usage
        if (aMIntS2.Extent() < FinTT2)
        {
            for (int i_S2 = 0; i_S2 < FinTT2; i_S2++)
            {
                if (!aMIntS2.Contains(i_S2))
                {
                    theTriangles2[i_S2].SetIntersectionPossible(false);
                }
            }
        }
    }

    // Refinement of the large triangles in case one surface is much smaller then the other
    private static void LargeTrianglesDeflectionsRefinement(Adaptor3d_Surface theS,
                                                            IntPolyh_Array<IntPolyh_Triangle> theTriangles,
                                                            IntPolyh_Array<IntPolyh_Edge> theEdges,
                                                            IntPolyh_Array<IntPolyh_Point> thePoints,
                                                            in Bnd_Box theOppositeBox)
    {
        // Find all triangles of the bigger surface with bounding boxes
        // overlapping the bounding box the other surface
        NCollection_List<int> aLIT = new();
        int i, aNbT = theTriangles.NbItems();
        for (i = 0; i < aNbT; ++i)
        {
            ref IntPolyh_Triangle aTriangle = ref theTriangles[i];
            if (!aTriangle.IsIntersectionPossible() || aTriangle.IsDegenerated())
            {
                continue;
            }
            //
            Bnd_Box aBox = aTriangle.BoundingBox(thePoints);
            if (aBox.IsVoid())
            {
                continue;
            }
            //
            if (aBox.IsOut(theOppositeBox))
            {
                aTriangle.SetIntersectionPossible(false);
                continue;
            }
            //
            aLIT.Append(i);
        }
        //
        if (aLIT.IsEmpty())
        {
            return;
        }
        //
        // One surface is very small relatively to the other.
        // The criterion of refining for large surface depends on the size of
        // the bounding box of the other - since the criterion should be minimized,
        // the smallest side of the bounding box is taken
        theOppositeBox.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
        double dx = Math.Abs(x1 - x0);
        double dy = Math.Abs(y1 - y0);
        double diag = Math.Abs(z1 - z0);
        double dd = (dx > dy) ? dy : dx;
        if (diag > dd)
        {
            diag = dd;
        }

        // calculation of the criterion of refining
        double CritereAffinage = 0.0;
        double DiagPonderation = 0.5;
        CritereAffinage = diag * DiagPonderation;
        //
        // The deflection of the greater is compared to the size of the smaller
        for (NCollection_List<int>.Iterator Iter = new(aLIT); Iter.More(); Iter.Next())
        {
            i = Iter.Value();
            ref IntPolyh_Triangle aTriangle = ref theTriangles[i];
            if (aTriangle.Deflection() > CritereAffinage)
            {
                aTriangle.MultipleMiddleRefinement(CritereAffinage, theOppositeBox, i, theS, thePoints, theTriangles,
                                                   theEdges);
            }
            else
            {
                aTriangle.MiddleRefinement(i, theS, thePoints, theTriangles, theEdges);
            }
        }
    }

    /// <summary>
    /// Refines both surfaces, with a BVH of the triangles' boxes as rejection. The criteria to refine a triangle are its
    /// deflection and the size of the boxes (one surface may be very small compared to the other).
    /// </summary>
    public void TrianglesDeflectionsRefinementBSB()
    {
        // To estimate a surface in general it can be interesting
        // to calculate all deflections
        ComputeDeflections(1);
        // Check deflection at output
        double FlecheCritique1;
        if (FlecheMin1 > FlecheMax1)
        {
            return;
        }
        else
        {
            // fleche min + (flechemax-flechemin) * 80/100
            FlecheCritique1 = FlecheMin1 * 0.2 + FlecheMax1 * 0.8;
        }

        // Compute deflections for the second surface
        ComputeDeflections(2);

        //-- Check arrows at output
        double FlecheCritique2;
        if (FlecheMin2 > FlecheMax2)
        {
            return;
        }
        else
        {
            // fleche min + (flechemax-flechemin) * 80/100
            FlecheCritique2 = FlecheMin2 * 0.2 + FlecheMax2 * 0.8;
        }

        // The greatest of two bounding boxes created in FillArrayOfPoints is found.
        // Then this value is weighted depending on the discretization
        // (NbSamplesU and NbSamplesV)
        double diag1, diag2;

        MyBox1.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
        x0 -= x1;
        y0 -= y1;
        z0 -= z1;
        diag1 = x0 * x0 + y0 * y0 + z0 * z0;
        double NbSamplesUV1 = (double)NbSamplesU1 * (double)NbSamplesV1;
        diag1 /= NbSamplesUV1;

        MyBox2.Get(out x0, out y0, out z0, out x1, out y1, out z1);
        x0 -= x1;
        y0 -= y1;
        z0 -= z1;
        diag2 = x0 * x0 + y0 * y0 + z0 * z0;
        double NbSamplesUV2 = (double)NbSamplesU2 * (double)NbSamplesV2;
        diag2 /= NbSamplesUV2;

        // The surface with the greatest bounding box is "discretized"
        if (diag1 < diag2)
        {
            // second is discretized
            if (FlecheCritique2 < diag1)
            {
                // The corresponding sizes are not too disproportional
                TrianglesDeflectionsRefinement(MaSurface1, TTriangles1, TEdges1, TPoints1, FlecheCritique1,
                                               MaSurface2, TTriangles2, TEdges2, TPoints2, FlecheCritique2);
            }
            else
            {
                // second surface is much larger then the first
                LargeTrianglesDeflectionsRefinement(MaSurface2, TTriangles2, TEdges2, TPoints2, MyBox1);
            }
        }
        else
        {
            // first is discretized
            if (FlecheCritique1 < diag2)
            {
                // The corresponding sizes are not too disproportional
                TrianglesDeflectionsRefinement(MaSurface2, TTriangles2, TEdges2, TPoints2, FlecheCritique2,
                                               MaSurface1, TTriangles1, TEdges1, TPoints1, FlecheCritique1);
            }
            else
            {
                // first surface is much larger then the second
                LargeTrianglesDeflectionsRefinement(MaSurface1, TTriangles1, TEdges1, TPoints1, MyBox2);
            }
        }
    }

    //=================================================================================================
    // Contact of triangles

    // Used by project6
    private static double maxSR(double a, double b, double c)
    {
        double t = a;
        if (b > t)
        {
            t = b;
        }

        if (c > t)
        {
            t = c;
        }

        return t;
    }

    // Used by project6
    private static double minSR(double a, double b, double c)
    {
        double t = a;
        if (b < t)
        {
            t = b;
        }

        if (c < t)
        {
            t = c;
        }

        return t;
    }

    // Used by TriContact: 0 if the projections of the triangles on the axis are disjoint
    private static int project6(in IntPolyh_Point ax, in IntPolyh_Point p1, in IntPolyh_Point p2, in IntPolyh_Point p3,
                                in IntPolyh_Point q1, in IntPolyh_Point q2, in IntPolyh_Point q3)
    {
        double P1 = ax.Dot(p1);
        double P2 = ax.Dot(p2);
        double P3 = ax.Dot(p3);
        double Q1 = ax.Dot(q1);
        double Q2 = ax.Dot(q2);
        double Q3 = ax.Dot(q3);

        double mx1 = maxSR(P1, P2, P3);
        double mn1 = minSR(P1, P2, P3);
        double mx2 = maxSR(Q1, Q2, Q3);
        double mn2 = minSR(Q1, Q2, Q3);

        if (mn1 > mx2)
        {
            return 0;
        }

        if (mn2 > mx1)
        {
            return 0;
        }

        return 1;
    }

    /// <summary>
    /// 1 if the triangles (P1, P2, P3) and (Q1, Q2, Q3) are in contact (separating axes), else 0. When in contact and
    /// neither normal vanishes, <paramref name="Angle"/> is the cosine of the angle between their normals (else it is
    /// left unchanged).
    /// </summary>
    public int TriContact(in IntPolyh_Point P1, in IntPolyh_Point P2, in IntPolyh_Point P3, in IntPolyh_Point Q1,
                          in IntPolyh_Point Q2, in IntPolyh_Point Q3, ref double Angle)
    {
        // The first triangle is (p1,p2,p3).  The other is (q1,q2,q3).
        // The edges are (e1,e2,e3) and (f1,f2,f3).
        // The normals are n1 and m1
        // Outwards are (g1,g2,g3) and (h1,h2,h3).

        if (maxSR(P1.X(), P2.X(), P3.X()) < minSR(Q1.X(), Q2.X(), Q3.X()))
        {
            return 0;
        }

        if (maxSR(P1.Y(), P2.Y(), P3.Y()) < minSR(Q1.Y(), Q2.Y(), Q3.Y()))
        {
            return 0;
        }

        if (maxSR(P1.Z(), P2.Z(), P3.Z()) < minSR(Q1.Z(), Q2.Z(), Q3.Z()))
        {
            return 0;
        }

        if (minSR(P1.X(), P2.X(), P3.X()) > maxSR(Q1.X(), Q2.X(), Q3.X()))
        {
            return 0;
        }

        if (minSR(P1.Y(), P2.Y(), P3.Y()) > maxSR(Q1.Y(), Q2.Y(), Q3.Y()))
        {
            return 0;
        }

        if (minSR(P1.Z(), P2.Z(), P3.Z()) > maxSR(Q1.Z(), Q2.Z(), Q3.Z()))
        {
            return 0;
        }

        IntPolyh_Point p1 = new(), p2 = new(), p3 = new();
        IntPolyh_Point q1 = new(), q2 = new(), q3 = new();
        IntPolyh_Point e1 = new(), e2 = new(), e3 = new();
        IntPolyh_Point f1 = new(), f2 = new(), f3 = new();
        IntPolyh_Point g1 = new(), g2 = new(), g3 = new();
        IntPolyh_Point h1 = new(), h2 = new(), h3 = new();
        IntPolyh_Point n1 = new(), m1 = new();
        IntPolyh_Point z = new();

        IntPolyh_Point ef11 = new(), ef12 = new(), ef13 = new();
        IntPolyh_Point ef21 = new(), ef22 = new(), ef23 = new();
        IntPolyh_Point ef31 = new(), ef32 = new(), ef33 = new();

        z.SetX(0.0);
        z.SetY(0.0);
        z.SetZ(0.0);

        p1.SetX(P1.X() - P1.X());
        p1.SetY(P1.Y() - P1.Y());
        p1.SetZ(P1.Z() - P1.Z());
        p2.SetX(P2.X() - P1.X());
        p2.SetY(P2.Y() - P1.Y());
        p2.SetZ(P2.Z() - P1.Z());
        p3.SetX(P3.X() - P1.X());
        p3.SetY(P3.Y() - P1.Y());
        p3.SetZ(P3.Z() - P1.Z());

        q1.SetX(Q1.X() - P1.X());
        q1.SetY(Q1.Y() - P1.Y());
        q1.SetZ(Q1.Z() - P1.Z());
        q2.SetX(Q2.X() - P1.X());
        q2.SetY(Q2.Y() - P1.Y());
        q2.SetZ(Q2.Z() - P1.Z());
        q3.SetX(Q3.X() - P1.X());
        q3.SetY(Q3.Y() - P1.Y());
        q3.SetZ(Q3.Z() - P1.Z());

        e1.SetX(p2.X() - p1.X());
        e1.SetY(p2.Y() - p1.Y());
        e1.SetZ(p2.Z() - p1.Z());
        e2.SetX(p3.X() - p2.X());
        e2.SetY(p3.Y() - p2.Y());
        e2.SetZ(p3.Z() - p2.Z());
        e3.SetX(p1.X() - p3.X());
        e3.SetY(p1.Y() - p3.Y());
        e3.SetZ(p1.Z() - p3.Z());

        f1.SetX(q2.X() - q1.X());
        f1.SetY(q2.Y() - q1.Y());
        f1.SetZ(q2.Z() - q1.Z());
        f2.SetX(q3.X() - q2.X());
        f2.SetY(q3.Y() - q2.Y());
        f2.SetZ(q3.Z() - q2.Z());
        f3.SetX(q1.X() - q3.X());
        f3.SetY(q1.Y() - q3.Y());
        f3.SetZ(q1.Z() - q3.Z());

        n1.Cross(e1, e2); // normal to the first triangle
        m1.Cross(f1, f2); // normal to the second triangle

        g1.Cross(e1, n1);
        g2.Cross(e2, n1);
        g3.Cross(e3, n1);
        h1.Cross(f1, m1);
        h2.Cross(f2, m1);
        h3.Cross(f3, m1);

        ef11.Cross(e1, f1);
        ef12.Cross(e1, f2);
        ef13.Cross(e1, f3);
        ef21.Cross(e2, f1);
        ef22.Cross(e2, f2);
        ef23.Cross(e2, f3);
        ef31.Cross(e3, f1);
        ef32.Cross(e3, f2);
        ef33.Cross(e3, f3);

        // Now the testing is done

        if (project6(n1, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T2 is not higher or lower than T1
        }

        if (project6(m1, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T1 is not higher of lower than T2
        }

        if (project6(ef11, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (project6(ef12, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (project6(ef13, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (project6(ef21, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (project6(ef22, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (project6(ef23, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (project6(ef31, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (project6(ef32, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (project6(ef33, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (project6(g1, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T2 is outside of T1 in the plane of T1
        }

        if (project6(g2, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T2 is outside of T1 in the plane of T1
        }

        if (project6(g3, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T2 is outside of T1 in the plane of T1
        }

        if (project6(h1, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T1 is outside of T2 in the plane of T2
        }

        if (project6(h2, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T1 is outside of T2 in the plane of T2
        }

        if (project6(h3, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T1 is outside of T2 in the plane of T2
        }

        // Calculation of cosinus angle between two normals
        double SqModn1 = -1.0;
        double SqModm1 = -1.0;
        SqModn1 = n1.SquareModulus();
        if (SqModn1 > SquareMyConfusionPrecision)
        {
            SqModm1 = m1.SquareModulus();
        }

        // OCCT: Angle is left unchanged when a normal vanishes (TriangleCompare then keeps the previous couple's)
        if (SqModm1 > SquareMyConfusionPrecision)
        {
            Angle = (n1.Dot(m1)) / (Math.Sqrt(SqModn1) * Math.Sqrt(SqModm1));
        }

        return 1;
    }

    // Used by StartingPointsResearch() to control the number of points found and keep the result in conformity
    // (1 or 2 points)
    private static void TestNbPoints(int TriSurfID, ref int NbPoints, ref int NbPointsTotal, in IntPolyh_StartPoint Pt1,
                                     in IntPolyh_StartPoint Pt2, ref IntPolyh_StartPoint SP1,
                                     ref IntPolyh_StartPoint SP2)
    {
        _ = TriSurfID;
        // already checked in TriangleEdgeContact
        //  if( (NbPoints==2)&&(Pt1.CheckSameSP(Pt2)) ) NbPoints=1;

        if (NbPoints > 2)
        {
        }
        else
        {
            if ((NbPoints == 1) && (NbPointsTotal == 0))
            {
                SP1 = Pt1;
                NbPointsTotal = 1;
            }
            else if ((NbPoints == 1) && (NbPointsTotal == 1))
            {
                if (Pt1.CheckSameSP(SP1) != 1)
                {
                    SP2 = Pt1;
                    NbPointsTotal = 2;
                }
            }
            else if ((NbPoints == 1) && (NbPointsTotal == 2))
            {
                if ((SP1.CheckSameSP(Pt1) != 0) || (SP2.CheckSameSP(Pt1) != 0))
                {
                    NbPointsTotal = 2;
                }
                else
                {
                    NbPointsTotal = 3;
                }
            }
            else if ((NbPoints == 2) && (NbPointsTotal == 0))
            {
                SP1 = Pt1;
                SP2 = Pt2;
                NbPointsTotal = 2;
            }
            else if ((NbPoints == 2) && (NbPointsTotal == 1))
            {
                // there is also Pt1 != Pt2
                if (SP1.CheckSameSP(Pt1) != 0)
                {
                    SP2 = Pt2;
                    NbPointsTotal = 2;
                }
                else if (SP1.CheckSameSP(Pt2) != 0)
                {
                    SP2 = Pt1;
                    NbPointsTotal = 2;
                }
                else
                {
                    NbPointsTotal = 3; // case SP1!=Pt1 && SP1!=Pt2!
                }
            }
            else if ((NbPoints == 2) && (NbPointsTotal == 2))
            {
                // there is also SP1!=SP2
                if ((SP1.CheckSameSP(Pt1) != 0) || (SP1.CheckSameSP(Pt2) != 0))
                {
                    if ((SP2.CheckSameSP(Pt1) != 0) || (SP2.CheckSameSP(Pt2) != 0))
                    {
                        NbPointsTotal = 2;
                    }
                    else
                    {
                        NbPointsTotal = 3;
                    }
                }
                else
                {
                    NbPointsTotal = 3;
                }
            }
        }
    }

    /// <summary>
    /// The intersection points of two triangles (by index into the surfaces' triangles): returns their number; more
    /// than two means the triangles are coplanar.
    /// </summary>
    public int StartingPointsResearch(int T1, int T2, ref IntPolyh_StartPoint SP1, ref IntPolyh_StartPoint SP2)
    {
        ref readonly IntPolyh_Triangle Tri1 = ref TTriangles1.Value(T1);
        ref readonly IntPolyh_Triangle Tri2 = ref TTriangles2.Value(T2);

        ref readonly IntPolyh_Point P1 = ref TPoints1.Value(Tri1.FirstPoint());
        ref readonly IntPolyh_Point P2 = ref TPoints1.Value(Tri1.SecondPoint());
        ref readonly IntPolyh_Point P3 = ref TPoints1.Value(Tri1.ThirdPoint());
        ref readonly IntPolyh_Point Q1 = ref TPoints2.Value(Tri2.FirstPoint());
        ref readonly IntPolyh_Point Q2 = ref TPoints2.Value(Tri2.SecondPoint());
        ref readonly IntPolyh_Point Q3 = ref TPoints2.Value(Tri2.ThirdPoint());

        // The first triangle is (p1,p2,p3).  The other is (q1,q2,q3).
        // The sides are (e1,e2,e3) and (f1,f2,f3).
        // The normals are n1 and m1

        IntPolyh_Point e1 = P2 - P1;
        IntPolyh_Point e2 = P3 - P2;
        IntPolyh_Point e3 = P1 - P3;

        IntPolyh_Point f1 = Q2 - Q1;
        IntPolyh_Point f2 = Q3 - Q2;
        IntPolyh_Point f3 = Q1 - Q3;

        IntPolyh_Point nn1 = new(), mm1 = new();
        nn1.Cross(e1, e2); // normal to the first triangle
        mm1.Cross(f1, f2); // normal to the second triangle

        double nn1modulus, mm1modulus;
        nn1modulus = Math.Sqrt(nn1.SquareModulus());
        mm1modulus = Math.Sqrt(mm1.SquareModulus());

        //-------------------------------------------------
        // calculation of intersections points between triangles
        //-------------------------------------------------
        int NbPoints = 0;
        int NbPointsTotal = 0;

        // check T1 normal
        if (Math.Abs(nn1modulus) < MyConfusionPrecision)
        {
        }
        else
        {
            IntPolyh_Point n1 = nn1.Divide(nn1modulus);
            // T2 edges with T1
            if (NbPointsTotal < 3)
            {
                IntPolyh_StartPoint Pt1 = new(), Pt2 = new();
                NbPoints = TriangleEdgeContact(1, 1, Tri1, Tri2, P1, P2, P3, e1, e2, e3, Q1, Q2, f1, n1, ref Pt1, ref Pt2);
                TestNbPoints(1, ref NbPoints, ref NbPointsTotal, Pt1, Pt2, ref SP1, ref SP2);
            }

            if (NbPointsTotal < 3)
            {
                IntPolyh_StartPoint Pt1 = new(), Pt2 = new();
                NbPoints = TriangleEdgeContact(1, 2, Tri1, Tri2, P1, P2, P3, e1, e2, e3, Q2, Q3, f2, n1, ref Pt1, ref Pt2);
                TestNbPoints(1, ref NbPoints, ref NbPointsTotal, Pt1, Pt2, ref SP1, ref SP2);
            }

            if (NbPointsTotal < 3)
            {
                IntPolyh_StartPoint Pt1 = new(), Pt2 = new();
                NbPoints = TriangleEdgeContact(1, 3, Tri1, Tri2, P1, P2, P3, e1, e2, e3, Q3, Q1, f3, n1, ref Pt1, ref Pt2);
                TestNbPoints(1, ref NbPoints, ref NbPointsTotal, Pt1, Pt2, ref SP1, ref SP2);
            }
        }

        // check T2 normal
        if (Math.Abs(mm1modulus) < MyConfusionPrecision)
        {
        }
        else
        {
            IntPolyh_Point m1 = mm1.Divide(mm1modulus);
            // T1 edges with T2
            if (NbPointsTotal < 3)
            {
                IntPolyh_StartPoint Pt1 = new(), Pt2 = new();
                NbPoints = TriangleEdgeContact(2, 1, Tri1, Tri2, Q1, Q2, Q3, f1, f2, f3, P1, P2, e1, m1, ref Pt1, ref Pt2);
                TestNbPoints(2, ref NbPoints, ref NbPointsTotal, Pt1, Pt2, ref SP1, ref SP2);
            }

            if (NbPointsTotal < 3)
            {
                IntPolyh_StartPoint Pt1 = new(), Pt2 = new();
                NbPoints = TriangleEdgeContact(2, 2, Tri1, Tri2, Q1, Q2, Q3, f1, f2, f3, P2, P3, e2, m1, ref Pt1, ref Pt2);
                TestNbPoints(2, ref NbPoints, ref NbPointsTotal, Pt1, Pt2, ref SP1, ref SP2);
            }

            if (NbPointsTotal < 3)
            {
                IntPolyh_StartPoint Pt1 = new(), Pt2 = new();
                NbPoints = TriangleEdgeContact(2, 3, Tri1, Tri2, Q1, Q2, Q3, f1, f2, f3, P3, P1, e3, m1, ref Pt1, ref Pt2);
                TestNbPoints(2, ref NbPoints, ref NbPointsTotal, Pt1, Pt2, ref SP1, ref SP2);
            }
        }

        if (NbPointsTotal == 2)
        {
            SP1.SetCoupleValue(T1, T2);
            SP2.SetCoupleValue(T1, T2);
        }
        else if (NbPointsTotal == 1)
        {
            SP1.SetCoupleValue(T1, T2);
        }
        else if (NbPointsTotal == 3)
        {
            SP1.SetCoupleValue(T1, T2);
        }

        return NbPointsTotal;
    }

    /// <summary>
    /// From two triangles and an intersection point, searches the other intersection point of the triangles (if it
    /// exists), skipping the edges the first point lies on. Used by <see cref="StartPointsChain"/>.
    /// </summary>
    public int NextStartingPointsResearch(int T1, int T2, in IntPolyh_StartPoint SPInit, ref IntPolyh_StartPoint SPNext)
    {
        int NbPointsTotal = 0;
        int EdgeInit1 = SPInit.E1();
        int EdgeInit2 = SPInit.E2();
        if ((T1 < 0) || (T2 < 0))
        {
            NbPointsTotal = 0;
        }
        else
        {
            ref readonly IntPolyh_Triangle Tri1 = ref TTriangles1.Value(T1);
            ref readonly IntPolyh_Triangle Tri2 = ref TTriangles2.Value(T2);

            ref readonly IntPolyh_Point P1 = ref TPoints1.Value(Tri1.FirstPoint());
            ref readonly IntPolyh_Point P2 = ref TPoints1.Value(Tri1.SecondPoint());
            ref readonly IntPolyh_Point P3 = ref TPoints1.Value(Tri1.ThirdPoint());
            ref readonly IntPolyh_Point Q1 = ref TPoints2.Value(Tri2.FirstPoint());
            ref readonly IntPolyh_Point Q2 = ref TPoints2.Value(Tri2.SecondPoint());
            ref readonly IntPolyh_Point Q3 = ref TPoints2.Value(Tri2.ThirdPoint());

            // The first triangle is (p1,p2,p3).  The other is (q1,q2,q3).
            // The edges are (e1,e2,e3) and (f1,f2,f3).
            // The normals are n1 and m1

            IntPolyh_Point e1 = P2 - P1;
            IntPolyh_Point e2 = P3 - P2;
            IntPolyh_Point e3 = P1 - P3;

            IntPolyh_Point f1 = Q2 - Q1;
            IntPolyh_Point f2 = Q3 - Q2;
            IntPolyh_Point f3 = Q1 - Q3;

            IntPolyh_Point nn1 = new(), mm1 = new();
            nn1.Cross(e1, e2); // normal to the first triangle
            mm1.Cross(f1, f2); // normal to the second triangle

            double nn1modulus, mm1modulus;
            nn1modulus = Math.Sqrt(nn1.SquareModulus());
            mm1modulus = Math.Sqrt(mm1.SquareModulus());

            //-------------------------------------------------
            // calculation of intersections points between triangles
            //-------------------------------------------------

            int NbPoints = 0;
            IntPolyh_StartPoint SP1 = new(), SP2 = new();

            // check T1 normal
            if (Math.Abs(nn1modulus) < MyConfusionPrecision)
            {
            }
            else
            {
                IntPolyh_Point n1 = nn1.Divide(nn1modulus);
                // T2 edges with T1
                if ((NbPointsTotal < 3) && (EdgeInit2 != Tri2.FirstEdge()))
                {
                    IntPolyh_StartPoint Pt1 = new(), Pt2 = new();
                    NbPoints = TriangleEdgeContact(1, 1, Tri1, Tri2, P1, P2, P3, e1, e2, e3, Q1, Q2, f1, n1, ref Pt1, ref Pt2);
                    TestNbPoints(1, ref NbPoints, ref NbPointsTotal, Pt1, Pt2, ref SP1, ref SP2);
                }

                if ((NbPointsTotal < 3) && (EdgeInit2 != Tri2.SecondEdge()))
                {
                    IntPolyh_StartPoint Pt1 = new(), Pt2 = new();
                    NbPoints = TriangleEdgeContact(1, 2, Tri1, Tri2, P1, P2, P3, e1, e2, e3, Q2, Q3, f2, n1, ref Pt1, ref Pt2);
                    TestNbPoints(1, ref NbPoints, ref NbPointsTotal, Pt1, Pt2, ref SP1, ref SP2);
                }

                if ((NbPointsTotal < 3) && (EdgeInit2 != Tri2.ThirdEdge()))
                {
                    IntPolyh_StartPoint Pt1 = new(), Pt2 = new();
                    NbPoints = TriangleEdgeContact(1, 3, Tri1, Tri2, P1, P2, P3, e1, e2, e3, Q3, Q1, f3, n1, ref Pt1, ref Pt2);
                    TestNbPoints(1, ref NbPoints, ref NbPointsTotal, Pt1, Pt2, ref SP1, ref SP2);
                }
            }

            // check T2 normal
            if (Math.Abs(mm1modulus) < MyConfusionPrecision)
            {
            }
            else
            {
                IntPolyh_Point m1 = mm1.Divide(mm1modulus);
                // T1 edges with T2
                if ((NbPointsTotal < 3) && (EdgeInit1 != Tri1.FirstEdge()))
                {
                    IntPolyh_StartPoint Pt1 = new(), Pt2 = new();
                    NbPoints = TriangleEdgeContact(2, 1, Tri1, Tri2, Q1, Q2, Q3, f1, f2, f3, P1, P2, e1, m1, ref Pt1, ref Pt2);
                    TestNbPoints(2, ref NbPoints, ref NbPointsTotal, Pt1, Pt2, ref SP1, ref SP2);
                }

                if ((NbPointsTotal < 3) && (EdgeInit1 != Tri1.SecondEdge()))
                {
                    IntPolyh_StartPoint Pt1 = new(), Pt2 = new();
                    NbPoints = TriangleEdgeContact(2, 2, Tri1, Tri2, Q1, Q2, Q3, f1, f2, f3, P2, P3, e2, m1, ref Pt1, ref Pt2);
                    TestNbPoints(2, ref NbPoints, ref NbPointsTotal, Pt1, Pt2, ref SP1, ref SP2);
                }

                if ((NbPointsTotal < 3) && (EdgeInit1 != Tri1.ThirdEdge()))
                {
                    IntPolyh_StartPoint Pt1 = new(), Pt2 = new();
                    NbPoints = TriangleEdgeContact(2, 3, Tri1, Tri2, Q1, Q2, Q3, f1, f2, f3, P3, P1, e3, m1, ref Pt1, ref Pt2);
                    TestNbPoints(2, ref NbPoints, ref NbPointsTotal, Pt1, Pt2, ref SP1, ref SP2);
                }
            }

            if (NbPointsTotal == 1)
            {
                if (SP1.CheckSameSP(SPInit) != 0)
                {
                    NbPointsTotal = 0;
                }
                else
                {
                    SPNext = SP1;
                }
            }
            else if ((NbPointsTotal == 2) && (SP1.CheckSameSP(SPInit) != 0))
            {
                NbPointsTotal = 1; // SP1 and SPInit are the same
                SPNext = SP2;
            }
            else if ((NbPointsTotal == 2) && (SP2.CheckSameSP(SPInit) != 0))
            {
                NbPointsTotal = 1; // SP2 and SPInit are the same
                SPNext = SP1;
            }
            else if (NbPointsTotal > 1)
            {
            }
        }

        SPNext.SetCoupleValue(T1, T2);
        return NbPointsTotal;
    }

    // The intersection points of a side of a triangle and an edge of the other, coplanar with it
    private static void CalculPtsInterTriEdgeCoplanaires(int TriSurfID, in IntPolyh_Point NormaleTri,
                                                         in IntPolyh_Triangle Tri1, in IntPolyh_Triangle Tri2,
                                                         in IntPolyh_Point PE1, in IntPolyh_Point PE2,
                                                         in IntPolyh_Point Edge, int EdgeIndex, in IntPolyh_Point PT1,
                                                         in IntPolyh_Point PT2, in IntPolyh_Point Cote, int CoteIndex,
                                                         ref IntPolyh_StartPoint SP1, ref IntPolyh_StartPoint SP2,
                                                         ref int NbPoints)
    {
        double aDE, aDC;
        //
        gp_Vec aVE = new(Edge.X(), Edge.Y(), Edge.Z());
        gp_Vec aVC = new(Cote.X(), Cote.Y(), Cote.Z());
        //
        aDE = aVE.SquareMagnitude();
        aDC = aVC.SquareMagnitude();
        //
        if (aDE > SquareMyConfusionPrecision)
        {
            aVE.Divide(aDE);
        }
        //
        if (aDC > SquareMyConfusionPrecision)
        {
            aVC.Divide(aDC);
        }
        //
        if (!aVE.IsParallel(aVC, MyConfusionPrecision))
        {
            // Edge and side are not parallel
            IntPolyh_Point Per = new();
            Per.Cross(NormaleTri, Cote);
            double p1p = Per.Dot(PE1);
            double p2p = Per.Dot(PE2);
            double p0p = Per.Dot(PT1);
            // The edge are PT1 are projected on the perpendicular of the side in the plane of the triangle
            if ((((p1p >= p0p) && (p0p >= p2p)) || ((p1p <= p0p) && (p0p <= p2p)))
                && (Math.Abs(p1p - p2p) > MyConfusionPrecision))
            {
                double lambda = (p1p - p0p) / (p1p - p2p);
                IntPolyh_Point PIE;
                if (Math.Abs(lambda) < MyConfusionPrecision)
                {
                    // lambda=0
                    PIE = PE1;
                }
                else if (Math.Abs(lambda) > 1.0 - MyConfusionPrecision)
                {
                    // lambda=1
                    PIE = PE2;
                }
                else
                {
                    PIE = PE1 + Edge * lambda;
                }

                double alpha = RealLast();
                if (Cote.X() != 0)
                {
                    alpha = (PIE.X() - PT1.X()) / Cote.X();
                }
                else if (Cote.Y() != 0)
                {
                    alpha = (PIE.Y() - PT1.Y()) / Cote.Y();
                }
                else if (Cote.Z() != 0)
                {
                    alpha = (PIE.Z() - PT1.Z()) / Cote.Z();
                }

                if (alpha < -MyConfusionPrecision)
                {
                }
                else
                {
                    if (NbPoints == 0)
                    {
                        SP1.SetXYZ(PIE.X(), PIE.Y(), PIE.Z());
                        if (TriSurfID == 1)
                        {
                            if (Math.Abs(alpha) < MyConfusionPrecision)
                            {
                                // alpha=0
                                SP1.SetUV1(PT1.U(), PT1.V());
                                SP1.SetUV2(PIE.U(), PIE.V());
                                SP1.SetEdge1(-1);
                            }

                            if (Math.Abs(alpha) > 1.0 - MyConfusionPrecision)
                            {
                                // alpha=1
                                SP1.SetUV1(PT2.U(), PT2.V());
                                SP1.SetUV2(PIE.U(), PIE.V());
                                SP1.SetEdge1(-1);
                            }
                            else
                            {
                                SP1.SetUV1(PT1.U() + Cote.U() * alpha, PT1.V() + Cote.V() * alpha);
                                SP1.SetUV2(PIE.U(), PIE.V());
                                SP1.SetEdge1(Tri1.GetEdgeNumber(CoteIndex));
                                if (Tri1.GetEdgeOrientation(CoteIndex) > 0)
                                {
                                    SP1.SetLambda1(alpha);
                                }
                                else
                                {
                                    SP1.SetLambda1(1.0 - alpha);
                                }
                            }

                            NbPoints++;
                        }
                        else if (TriSurfID == 2)
                        {
                            if (Math.Abs(alpha) < MyConfusionPrecision)
                            {
                                // alpha=0
                                SP1.SetUV2(PT1.U(), PT1.V());
                                SP1.SetUV1(PIE.U(), PIE.V());
                                SP1.SetEdge2(-1);
                            }

                            if (Math.Abs(alpha) > 1.0 - MyConfusionPrecision)
                            {
                                // alpha=1
                                SP1.SetUV2(PT2.U(), PT2.V());
                                SP1.SetUV1(PIE.U(), PIE.V());
                                SP1.SetEdge2(-1);
                            }
                            else
                            {
                                SP1.SetUV1(PIE.U(), PIE.V());
                                SP1.SetUV2(PT1.U() + Cote.U() * alpha, PT1.V() + Cote.V() * alpha);
                                SP1.SetEdge2(Tri2.GetEdgeNumber(CoteIndex));
                                if (Tri2.GetEdgeOrientation(CoteIndex) > 0)
                                {
                                    SP1.SetLambda2(alpha);
                                }
                                else
                                {
                                    SP1.SetLambda2(1.0 - alpha);
                                }
                            }

                            NbPoints++;
                        }
                    }
                    else if (NbPoints == 1)
                    {
                        SP2.SetXYZ(PIE.X(), PIE.Y(), PIE.Z());
                        if (TriSurfID == 1)
                        {
                            if (Math.Abs(alpha) < MyConfusionPrecision)
                            {
                                // alpha=0
                                SP2.SetUV1(PT1.U(), PT1.V());
                                SP2.SetUV2(PIE.U(), PIE.V());
                                SP2.SetEdge1(-1);
                            }

                            if (Math.Abs(alpha) > 1.0 - MyConfusionPrecision)
                            {
                                // alpha=1
                                SP2.SetUV1(PT2.U(), PT2.V());
                                SP2.SetUV2(PIE.U(), PIE.V());
                                SP2.SetEdge1(-1);
                            }
                            else
                            {
                                SP2.SetUV1(PT1.U() + Cote.U() * alpha, PT1.V() + Cote.V() * alpha);
                                SP2.SetUV2(PIE.U(), PIE.V());
                                SP2.SetEdge1(Tri1.GetEdgeNumber(CoteIndex));
                                if (Tri1.GetEdgeOrientation(CoteIndex) > 0)
                                {
                                    SP2.SetLambda1(alpha);
                                }
                                else
                                {
                                    SP2.SetLambda1(1.0 - alpha);
                                }
                            }

                            NbPoints++;
                        }
                        else if (TriSurfID == 2)
                        {
                            if (Math.Abs(alpha) < MyConfusionPrecision)
                            {
                                // alpha=0
                                SP2.SetUV2(PT1.U(), PT1.V());
                                SP2.SetUV1(PIE.U(), PIE.V());
                                SP2.SetEdge2(-1);
                            }

                            if (Math.Abs(alpha) > 1.0 - MyConfusionPrecision)
                            {
                                // alpha=1
                                SP2.SetUV2(PT2.U(), PT2.V());
                                SP2.SetUV1(PIE.U(), PIE.V());
                                SP2.SetEdge2(-1);
                            }
                            else
                            {
                                SP2.SetUV1(PIE.U(), PIE.V());
                                SP2.SetUV2(PT1.U() + Cote.U() * alpha, PT1.V() + Cote.V() * alpha);
                                SP2.SetEdge2(Tri2.GetEdgeNumber(CoteIndex));
                                if (Tri1.GetEdgeOrientation(CoteIndex) > 0) // OCCT: Tri1, where SP1's branch reads Tri2
                                {
                                    SP2.SetLambda2(alpha);
                                }
                                else
                                {
                                    SP2.SetLambda2(1.0 - alpha);
                                }
                            }

                            NbPoints++;
                        }
                    }
                    else if ((NbPoints > 2) || (NbPoints < 0))
                    {
                    }
                }
            }
        }
        else
        {
            // Side and Edge are parallel, with previous
            // rejections they are at the same side
            // The points are projected on that side
            double pe1p = Cote.Dot(PE1);
            double pe2p = Cote.Dot(PE2);
            double pt1p = Cote.Dot(PT1);
            double pt2p = Cote.Dot(PT2);
            double lambda1 = 0.0, lambda2 = 0.0, alpha1 = 0.0, alpha2 = 0.0;
            IntPolyh_Point PEP1 = new(), PTP1 = new(), PEP2 = new(), PTP2 = new();

            if (pe1p > pe2p)
            {
                if ((pt1p < pe1p) && (pe1p <= pt2p))
                {
                    lambda1 = 0.0;
                    PEP1 = PE1;
                    alpha1 = ((pe1p - pt1p) / (pt2p - pt1p));
                    PTP1 = PT1 + Cote * alpha1;
                    NbPoints = 1;
                    if (pt1p <= pe2p)
                    {
                        lambda2 = 1.0;
                        PEP2 = PE2;
                        alpha2 = ((pe2p - pt1p) / (pt2p - pt1p));
                        PTP2 = PT1 + Cote * alpha2;
                        NbPoints = 2;
                    }
                    else
                    {
                        lambda2 = ((pt1p - pe1p) / (pe2p - pe1p));
                        PEP2 = PE1 + Edge * lambda2;
                        alpha2 = 0.0;
                        PTP2 = PT1;
                        NbPoints = 2;
                    }
                }
                else if ((pt2p < pe1p) && (pe1p <= pt1p))
                {
                    lambda1 = 0.0;
                    PEP1 = PE1;
                    alpha1 = ((pt1p - pe1p) / (pt1p - pt2p));
                    PTP1 = PT1 + Cote * alpha1;
                    NbPoints = 1;
                    if (pt2p <= pe2p)
                    {
                        lambda2 = 1.0;
                        PEP2 = PE2;
                        alpha2 = ((pe2p - pt1p) / (pt2p - pt1p));
                        PTP2 = PT1 + Cote * alpha2;
                        NbPoints = 2;
                    }
                    else
                    {
                        lambda2 = ((pt2p - pe1p) / (pe2p - pe1p));
                        PEP2 = PE1 + Edge * lambda2;
                        alpha2 = 1.0;
                        PTP2 = PT2;
                        NbPoints = 2;
                    }
                }
            }

            if (pe1p < pe2p)
            {
                if ((pt1p < pe2p) && (pe2p <= pt2p))
                {
                    lambda1 = 1.0;
                    PEP1 = PE2;
                    alpha1 = ((pe2p - pt1p) / (pt2p - pt1p));
                    PTP1 = PT1 + Cote * alpha1;
                    NbPoints = 1;
                    if (pt1p <= pe1p)
                    {
                        lambda2 = 0.0;
                        PEP2 = PE1;
                        alpha2 = ((pe1p - pt1p) / (pt2p - pt1p));
                        PTP2 = PT1 + Cote * alpha2;
                        NbPoints = 2;
                    }
                    else
                    {
                        lambda2 = ((pt1p - pe1p) / (pe2p - pe1p));
                        PEP2 = PE2 + Edge * lambda2; // OCCT: PE2, where the other branches start from PE1
                        alpha2 = 0.0;
                        PTP2 = PT1;
                        NbPoints = 2;
                    }
                }
                else if ((pt2p < pe2p) && (pe2p <= pt1p))
                {
                    lambda1 = 1.0;
                    PEP1 = PE2;
                    alpha1 = ((pt1p - pe2p) / (pt1p - pt2p));
                    PTP1 = PT1 + Cote * alpha1;
                    NbPoints = 1;
                    if (pt2p <= pe1p)
                    {
                        lambda2 = 0.0;
                        PEP2 = PE1;
                        alpha2 = ((pe1p - pt1p) / (pt2p - pt1p));
                        PTP2 = PT1 + Cote * alpha2;
                        NbPoints = 2;
                    }
                    else
                    {
                        lambda2 = ((pt2p - pe1p) / (pe2p - pe1p));
                        PEP2 = PE1 + Edge * lambda2;
                        alpha2 = 1.0;
                        PTP2 = PT2;
                        NbPoints = 2;
                    }
                }
            }

            if (NbPoints != 0)
            {
                SP1.SetXYZ(PEP1.X(), PEP1.Y(), PEP1.Z());
                if (TriSurfID == 1)
                {
                    // the side belongs to Tri1
                    SP1.SetUV1(PTP1.U(), PTP1.V());
                    SP1.SetUV2(PEP1.U(), PEP1.V());
                    SP1.SetEdge1(Tri1.GetEdgeNumber(CoteIndex));

                    if (Tri1.GetEdgeOrientation(CoteIndex) > 0)
                    {
                        SP1.SetLambda1(alpha1);
                    }
                    else
                    {
                        SP1.SetLambda1(1.0 - alpha1);
                    }

                    if (Tri2.GetEdgeOrientation(EdgeIndex) > 0)
                    {
                        SP1.SetLambda2(lambda1);
                    }
                    else
                    {
                        SP1.SetLambda2(1.0 - lambda1);
                    }
                }

                if (TriSurfID == 2)
                {
                    // the side belongs to Tri2
                    SP1.SetUV1(PEP1.U(), PTP1.V()); // OCCT: U of PEP1 with V of PTP1, and below
                    SP1.SetUV2(PTP1.U(), PEP1.V());
                    SP1.SetEdge2(Tri2.GetEdgeNumber(CoteIndex));

                    if (Tri2.GetEdgeOrientation(CoteIndex) > 0)
                    {
                        SP1.SetLambda1(alpha1);
                    }
                    else
                    {
                        SP1.SetLambda1(1.0 - alpha1);
                    }

                    if (Tri1.GetEdgeOrientation(EdgeIndex) > 0)
                    {
                        SP1.SetLambda2(lambda1);
                    }
                    else
                    {
                        SP1.SetLambda2(1.0 - lambda1);
                    }
                }

                // It is checked if PEP1!=PEP2
                if ((NbPoints == 2) && (Math.Abs(PEP1.U() - PEP2.U()) < MyConfusionPrecision)
                    && (Math.Abs(PEP1.V() - PEP2.V()) < MyConfusionPrecision))
                {
                    NbPoints = 1;
                }

                if (NbPoints == 2)
                {
                    SP2.SetXYZ(PEP2.X(), PEP2.Y(), PEP2.Z());
                    if (TriSurfID == 1)
                    {
                        SP2.SetUV1(PTP2.U(), PTP2.V());
                        SP2.SetUV2(PEP2.U(), PEP2.V());
                        SP2.SetEdge1(Tri1.GetEdgeNumber(CoteIndex));

                        // OCCT: alpha1 and lambda1, not alpha2 and lambda2, for the second point (and below)
                        if (Tri1.GetEdgeOrientation(CoteIndex) > 0)
                        {
                            SP2.SetLambda1(alpha1);
                        }
                        else
                        {
                            SP2.SetLambda1(1.0 - alpha1);
                        }

                        if (Tri2.GetEdgeOrientation(EdgeIndex) > 0)
                        {
                            SP2.SetLambda2(lambda1);
                        }
                        else
                        {
                            SP2.SetLambda2(1.0 - lambda1);
                        }
                    }

                    if (TriSurfID == 2)
                    {
                        SP2.SetUV1(PEP2.U(), PTP2.V());
                        SP2.SetUV2(PTP2.U(), PEP2.V());
                        SP2.SetEdge2(Tri2.GetEdgeNumber(CoteIndex));

                        if (Tri1.GetEdgeOrientation(CoteIndex) > 0) // OCCT: Tri1 for the side of Tri2
                        {
                            SP2.SetLambda1(alpha1);
                        }
                        else
                        {
                            SP2.SetLambda1(1.0 - alpha1);
                        }

                        if (Tri2.GetEdgeOrientation(EdgeIndex) > 0) // OCCT: Tri2 for the edge of Tri1
                        {
                            SP2.SetLambda2(lambda1);
                        }
                        else
                        {
                            SP2.SetLambda2(1.0 - lambda1);
                        }
                    }
                }
            }
        }

        // Filter if the point is placed on top, the edge is set  to -1
        if (NbPoints > 0)
        {
            if (Math.Abs(SP1.Lambda1()) < MyConfusionPrecision)
            {
                SP1.SetEdge1(-1);
            }

            if (Math.Abs(SP1.Lambda1() - 1) < MyConfusionPrecision)
            {
                SP1.SetEdge1(-1);
            }

            if (Math.Abs(SP1.Lambda2()) < MyConfusionPrecision)
            {
                SP1.SetEdge2(-1);
            }

            if (Math.Abs(SP1.Lambda2() - 1) < MyConfusionPrecision)
            {
                SP1.SetEdge2(-1);
            }
        }

        if (NbPoints == 2)
        {
            if (Math.Abs(SP2.Lambda1()) < MyConfusionPrecision)
            {
                SP2.SetEdge1(-1);
            }

            if (Math.Abs(SP2.Lambda1() - 1) < MyConfusionPrecision)
            {
                SP2.SetEdge1(-1);
            }

            if (Math.Abs(SP2.Lambda2()) < MyConfusionPrecision)
            {
                SP2.SetEdge2(-1);
            }

            if (Math.Abs(SP2.Lambda2() - 1) < MyConfusionPrecision)
            {
                SP2.SetEdge2(-1);
            }
        }
    }

    /// <summary>
    /// The intersection points (0 to 2) of an edge (PE1, PE2, Edge = PE2 - PE1) of one triangle with the other triangle
    /// (PT1, PT2, PT3, its sides and unit normal). <paramref name="TriSurfID"/> is the surface of the triangle (PT1, PT2,
    /// PT3); <paramref name="EdgeIndex"/> the edge's index (1 to 3) in the other.
    /// </summary>
    public int TriangleEdgeContact(int TriSurfID, int EdgeIndex, in IntPolyh_Triangle Tri1, in IntPolyh_Triangle Tri2,
                                   in IntPolyh_Point PT1, in IntPolyh_Point PT2, in IntPolyh_Point PT3,
                                   in IntPolyh_Point Cote12, in IntPolyh_Point Cote23, in IntPolyh_Point Cote31,
                                   in IntPolyh_Point PE1, in IntPolyh_Point PE2, in IntPolyh_Point Edge,
                                   in IntPolyh_Point NormaleT, ref IntPolyh_StartPoint SP1, ref IntPolyh_StartPoint SP2)
    {
        double lambda = 0.0, alpha = 0.0, beta = 0.0;

        // One of edges on which the point is located is known
        if (TriSurfID == 1)
        {
            SP1.SetEdge2(Tri2.GetEdgeNumber(EdgeIndex));
            SP2.SetEdge2(Tri2.GetEdgeNumber(EdgeIndex));
        }
        else if (TriSurfID == 2)
        {
            SP1.SetEdge1(Tri1.GetEdgeNumber(EdgeIndex));
            SP2.SetEdge1(Tri1.GetEdgeNumber(EdgeIndex));
        }

        // The edge is projected on the normal in the triangle if yes
        // --> a free intersection (point I)--> Start point is found
        int NbPoints = 0;
        if (NormaleT.SquareModulus() == 0)
        {
        }
        else if ((Cote12.SquareModulus() == 0) || (Cote23.SquareModulus() == 0) || (Cote31.SquareModulus() == 0))
        {
        }
        else if (Edge.SquareModulus() == 0)
        {
        }
        else
        {
            double pe1 = NormaleT.Dot(PE1);
            double pe2 = NormaleT.Dot(PE2);
            double pt1 = NormaleT.Dot(PT1);

            // PE1I = lambda.Edge

            if ((Math.Abs(pe1 - pt1) < MyConfusionPrecision) && (Math.Abs(pe2 - pt1) < MyConfusionPrecision))
            {
                // edge and triangle are coplanar (two contact points at maximum)

                // the tops of the triangle are projected on the perpendicular to the edge
                IntPolyh_Point PerpEdge = new();
                PerpEdge.Cross(NormaleT, Edge);
                double pp1 = PerpEdge.Dot(PT1);
                double pp2 = PerpEdge.Dot(PT2);
                double pp3 = PerpEdge.Dot(PT3);
                double ppe1 = PerpEdge.Dot(PE1);

                if ((Math.Abs(pp1 - pp2) < MyConfusionPrecision) && (Math.Abs(pp1 - pp3) < MyConfusionPrecision))
                {
                }
                else
                {
                    if (((pp1 >= ppe1) && (pp2 <= ppe1) && (pp3 <= ppe1))
                        || ((pp1 <= ppe1) && (pp2 >= ppe1) && (pp3 >= ppe1)))
                    {
                        // there are two sides (common top PT1) that can cut the edge

                        // first side
                        CalculPtsInterTriEdgeCoplanaires(TriSurfID, NormaleT, Tri1, Tri2, PE1, PE2, Edge, EdgeIndex,
                                                         PT1, PT2, Cote12, 1, ref SP1, ref SP2, ref NbPoints);

                        if ((NbPoints > 1) && (Math.Abs(SP2.U1() - SP1.U1()) < MyConfusionPrecision)
                            && (Math.Abs(SP1.V1() - SP2.V1()) < MyConfusionPrecision))
                        {
                            NbPoints = 1;
                        }

                        // second side
                        if (NbPoints < 2)
                        {
                            CalculPtsInterTriEdgeCoplanaires(TriSurfID, NormaleT, Tri1, Tri2, PE1, PE2, Edge, EdgeIndex,
                                                             PT3, PT1, Cote31, 3, ref SP1, ref SP2, ref NbPoints);
                        }
                    }

                    if ((NbPoints > 1) && (Math.Abs(SP1.U1() - SP2.U1()) < MyConfusionPrecision)
                        && (Math.Abs(SP1.V2() - SP2.V1()) < MyConfusionPrecision)) // OCCT: V2 against V1
                    {
                        NbPoints = 1;
                    }

                    if (NbPoints >= 2)
                    {
                        return NbPoints;
                    }
                    else if ((((pp2 >= ppe1) && (pp1 <= ppe1) && (pp3 <= ppe1))
                              || ((pp2 <= ppe1) && (pp1 >= ppe1) && (pp3 >= ppe1)))
                             && (NbPoints < 2))
                    {
                        // there are two sides (common top PT2) that can cut the edge

                        // first side
                        CalculPtsInterTriEdgeCoplanaires(TriSurfID, NormaleT, Tri1, Tri2, PE1, PE2, Edge, EdgeIndex,
                                                         PT1, PT2, Cote12, 1, ref SP1, ref SP2, ref NbPoints);

                        if ((NbPoints > 1) && (Math.Abs(SP2.U1() - SP1.U1()) < MyConfusionPrecision)
                            && (Math.Abs(SP1.V1() - SP2.V1()) < MyConfusionPrecision))
                        {
                            NbPoints = 1;
                        }

                        // second side
                        if (NbPoints < 2)
                        {
                            CalculPtsInterTriEdgeCoplanaires(TriSurfID, NormaleT, Tri1, Tri2, PE1, PE2, Edge, EdgeIndex,
                                                             PT2, PT3, Cote23, 2, ref SP1, ref SP2, ref NbPoints);
                        }
                    }

                    if ((NbPoints > 1) && (Math.Abs(SP2.U1() - SP1.U1()) < MyConfusionPrecision)
                        && (Math.Abs(SP1.V1() - SP2.V1()) < MyConfusionPrecision))
                    {
                        NbPoints = 1;
                    }

                    if (NbPoints >= 2)
                    {
                        return NbPoints;
                    }
                    else if ((((pp3 >= ppe1) && (pp1 <= ppe1) && (pp2 <= ppe1))
                              || ((pp3 <= ppe1) && (pp1 >= ppe1) && (pp2 >= ppe1)))
                             && (NbPoints < 2))
                    {
                        // there are two sides (common top PT3) that can cut the edge

                        // first side
                        CalculPtsInterTriEdgeCoplanaires(TriSurfID, NormaleT, Tri1, Tri2, PE1, PE2, Edge, EdgeIndex,
                                                         PT3, PT1, Cote31, 3, ref SP1, ref SP2, ref NbPoints);

                        if ((NbPoints > 1) && (Math.Abs(SP2.U1() - SP1.U1()) < MyConfusionPrecision)
                            && (Math.Abs(SP1.V1() - SP2.V1()) < MyConfusionPrecision))
                        {
                            NbPoints = 1;
                        }

                        // second side
                        if (NbPoints < 2)
                        {
                            CalculPtsInterTriEdgeCoplanaires(TriSurfID, NormaleT, Tri1, Tri2, PE1, PE2, Edge, EdgeIndex,
                                                             PT2, PT3, Cote23, 2, ref SP1, ref SP2, ref NbPoints);
                        }
                    }

                    if ((NbPoints > 1) && (Math.Abs(SP2.U1() - SP1.U1()) < MyConfusionPrecision)
                        && (Math.Abs(SP2.V1() - SP1.V1()) < MyConfusionPrecision))
                    {
                        NbPoints = 1;
                    }

                    if (NbPoints >= 2)
                    {
                        return NbPoints;
                    }
                }
            }

            //------------------------------------------------------
            // NON COPLANAR edge and triangle (a contact point)
            //------------------------------------------------------
            else if (((pe1 >= pt1) && (pt1 >= pe2)) || ((pe1 <= pt1) && (pt1 <= pe2)))
            {
                lambda = (pe1 - pt1) / (pe1 - pe2);
                IntPolyh_Point PI = new();
                if (lambda < -MyConfusionPrecision)
                {
                }
                else if (Math.Abs(lambda) < MyConfusionPrecision)
                {
                    // lambda==0
                    PI = PE1;
                    if (TriSurfID == 1)
                    {
                        SP1.SetEdge2(-1);
                    }
                    else
                    {
                        SP1.SetEdge1(-1);
                    }
                }
                else if (Math.Abs(lambda - 1.0) < MyConfusionPrecision)
                {
                    // lambda==1
                    PI = PE2;
                    if (TriSurfID == 1)
                    {
                        SP1.SetEdge2(-1);
                    }
                    else
                    {
                        SP1.SetEdge1(-1);
                    }
                }
                else
                {
                    PI = PE1 + Edge * lambda;
                    if (TriSurfID == 1)
                    {
                        if (Tri2.GetEdgeOrientation(EdgeIndex) > 0)
                        {
                            SP1.SetLambda2(lambda);
                        }
                        else
                        {
                            SP1.SetLambda2(1.0 - lambda);
                        }
                    }

                    if (TriSurfID == 2)
                    {
                        if (Tri1.GetEdgeOrientation(EdgeIndex) > 0)
                        {
                            SP1.SetLambda1(lambda);
                        }
                        else
                        {
                            SP1.SetLambda1(1.0 - lambda);
                        }
                    }
                }

                double Cote23X = Cote23.X();
                double D1 = 0.0;
                double D3, D4;

                // Combination Eq1 Eq2
                if (Math.Abs(Cote23X) > MyConfusionPrecision)
                {
                    D1 = Cote12.Y() - Cote12.X() * Cote23.Y() / Cote23X;
                }

                if (Math.Abs(D1) > MyConfusionPrecision)
                {
                    alpha = (PI.Y() - PT1.Y() - (PI.X() - PT1.X()) * Cote23.Y() / Cote23X) / D1;

                    // It is checked if 1.0>=alpha>=0.0
                    if ((alpha < -MyConfusionPrecision) || (alpha > (1.0 + MyConfusionPrecision)))
                    {
                        return 0;
                    }
                    else
                    {
                        beta = (PI.X() - PT1.X() - alpha * Cote12.X()) / Cote23X;
                    }
                }
                // Combination Eq1 and Eq2 with Cote23.X()==0
                else if ((Math.Abs(Cote12.X()) > MyConfusionPrecision) && (Math.Abs(Cote23X) < MyConfusionPrecision))
                {
                    // There is Cote23.X()==0
                    alpha = (PI.X() - PT1.X()) / Cote12.X();

                    if ((alpha < -MyConfusionPrecision) || (alpha > (1.0 + MyConfusionPrecision)))
                    {
                        return 0;
                    }
                    else if (Math.Abs(Cote23.Y()) > MyConfusionPrecision)
                    {
                        beta = (PI.Y() - PT1.Y() - alpha * Cote12.Y()) / Cote23.Y();
                    }
                    else if (Math.Abs(Cote23.Z()) > MyConfusionPrecision)
                    {
                        beta = (PI.Z() - PT1.Z() - alpha * Cote12.Z()) / Cote23.Z();
                    }
                }
                // Combination Eq1 and Eq3
                else if ((Math.Abs(Cote23.X()) > MyConfusionPrecision)
                         && (Math.Abs(D3 = (Cote12.Z() - Cote12.X() * Cote23.Z() / Cote23.X())) > MyConfusionPrecision))
                {
                    alpha = (PI.Z() - PT1.Z() - (PI.X() - PT1.X()) * Cote23.Z() / Cote23.X()) / D3;

                    if ((alpha < -MyConfusionPrecision) || (alpha > (1.0 + MyConfusionPrecision)))
                    {
                        return 0;
                    }
                    else
                    {
                        beta = (PI.X() - PT1.X() - alpha * Cote12.X()) / Cote23.X();
                    }
                }
                // Combination Eq2 and Eq3
                else if ((Math.Abs(Cote23.Y()) > MyConfusionPrecision)
                         && (Math.Abs(D4 = (Cote12.Z() - Cote12.Y() * Cote23.Z() / Cote23.Y())) > MyConfusionPrecision))
                {
                    alpha = (PI.Z() - PT1.Z() - (PI.Y() - PT1.Y()) * Cote23.Z() / Cote23.Y()) / D4;

                    if ((alpha < -MyConfusionPrecision) || (alpha > (1.0 + MyConfusionPrecision)))
                    {
                        return 0;
                    }
                    else
                    {
                        beta = (PI.Y() - PT1.Y() - alpha * Cote12.Y()) / Cote23.Y();
                    }
                }
                // Combination Eq2 and Eq3 with Cote23.Y()==0
                else if ((Math.Abs(Cote12.Y()) > MyConfusionPrecision) && (Math.Abs(Cote23.Y()) < MyConfusionPrecision))
                {
                    alpha = (PI.Y() - PT1.Y()) / Cote12.Y();

                    if ((alpha < -MyConfusionPrecision) || (alpha > (1.0 + MyConfusionPrecision)))
                    {
                        return 0;
                    }
                    else if (Math.Abs(Cote23.Z()) > MyConfusionPrecision)
                    {
                        beta = (PI.Z() - PT1.Z() - alpha * Cote12.Z()) / Cote23.Z();
                    }
                    else
                    {
                        // OCCT: printf("\nCote PT2PT3 nul1\n"); PT2.Dump(2004); PT3.Dump(3004); (a debug print, left out)
                    }
                }
                // Combination Eq1 and Eq3 with Cote23.Z()==0
                else if ((Math.Abs(Cote12.Z()) > MyConfusionPrecision) && (Math.Abs(Cote23.Z()) < MyConfusionPrecision))
                {
                    alpha = (PI.Z() - PT1.Z()) / Cote12.Z();

                    if ((alpha < -MyConfusionPrecision) || (alpha > (1.0 + MyConfusionPrecision)))
                    {
                        return 0;
                    }
                    else if (Math.Abs(Cote23.X()) > MyConfusionPrecision)
                    {
                        beta = (PI.X() - PT1.X() - alpha * Cote12.X()) / Cote23.X();
                    }
                }
                else
                {
                    // Particular case not processed ?
                    alpha = RealLast();
                    beta = RealLast();
                }

                if ((beta < -MyConfusionPrecision) || (beta > (alpha + MyConfusionPrecision)))
                {
                    return 0;
                }
                else
                {
                    SP1.SetXYZ(PI.X(), PI.Y(), PI.Z());

                    if (TriSurfID == 1)
                    {
                        SP1.SetUV2(PI.U(), PI.V());
                        SP1.SetUV1(PT1.U() + Cote12.U() * alpha + Cote23.U() * beta,
                                   PT1.V() + Cote12.V() * alpha + Cote23.V() * beta);
                        NbPoints++;
                        if (alpha < MyConfusionPrecision)
                        {
                            // alpha=0 --> beta==0
                            SP1.SetXYZ(PT1.X(), PT1.Y(), PT1.Z());
                            SP1.SetUV1(PT1.U(), PT1.V());
                            SP1.SetEdge1(-1);
                        }
                        else if ((beta < MyConfusionPrecision) && (Math.Abs(1 - alpha) < MyConfusionPrecision))
                        {
                            // beta==0 alpha==1
                            SP1.SetXYZ(PT2.X(), PT2.Y(), PT2.Z());
                            SP1.SetUV1(PT2.U(), PT2.V());
                            SP1.SetEdge1(-1);
                        }
                        else if ((Math.Abs(beta - 1) < MyConfusionPrecision) && (Math.Abs(1 - alpha) < MyConfusionPrecision))
                        {
                            // beta==1 alpha==1
                            SP1.SetXYZ(PT3.X(), PT3.Y(), PT3.Z());
                            SP1.SetUV1(PT3.U(), PT3.V());
                            SP1.SetEdge1(-1);
                        }
                        else if (beta < MyConfusionPrecision)
                        {
                            // beta==0
                            SP1.SetEdge1(Tri1.GetEdgeNumber(1));
                            if (Tri1.GetEdgeOrientation(1) > 0)
                            {
                                SP1.SetLambda1(alpha);
                            }
                            else
                            {
                                SP1.SetLambda1(1.0 - alpha);
                            }
                        }
                        else if (Math.Abs(beta - alpha) < MyConfusionPrecision)
                        {
                            // beta==alpha
                            SP1.SetEdge1(Tri1.GetEdgeNumber(3));
                            if (Tri1.GetEdgeOrientation(3) > 0)
                            {
                                SP1.SetLambda1(1.0 - alpha);
                            }
                            else
                            {
                                SP1.SetLambda1(alpha);
                            }
                        }
                        else if (Math.Abs(alpha - 1) < MyConfusionPrecision)
                        {
                            // alpha==1
                            SP1.SetEdge1(Tri1.GetEdgeNumber(2));
                            if (Tri1.GetEdgeOrientation(2) > 0)
                            {
                                SP1.SetLambda1(beta);
                            }
                            else
                            {
                                SP1.SetLambda1(1.0 - beta);
                            }
                        }
                    }
                    else if (TriSurfID == 2)
                    {
                        SP1.SetUV1(PI.U(), PI.V());
                        SP1.SetUV2(PT1.U() + Cote12.U() * alpha + Cote23.U() * beta,
                                   PT1.V() + Cote12.V() * alpha + Cote23.V() * beta);
                        NbPoints++;
                        if (alpha < MyConfusionPrecision)
                        {
                            // alpha=0 --> beta==0
                            SP1.SetXYZ(PT1.X(), PT1.Y(), PT1.Z());
                            SP1.SetUV2(PT1.U(), PT1.V());
                            SP1.SetEdge2(-1);
                        }
                        else if ((beta < MyConfusionPrecision) && (Math.Abs(1 - alpha) < MyConfusionPrecision))
                        {
                            // beta==0 alpha==1
                            SP1.SetXYZ(PT2.X(), PT2.Y(), PT2.Z());
                            SP1.SetUV2(PT2.U(), PT2.V());
                            SP1.SetEdge2(-1);
                        }
                        else if ((Math.Abs(beta - 1) < MyConfusionPrecision) && (Math.Abs(1 - alpha) < MyConfusionPrecision))
                        {
                            // beta==1 alpha==1
                            SP1.SetXYZ(PT3.X(), PT3.Y(), PT3.Z());
                            SP1.SetUV2(PT3.U(), PT3.V());
                            SP1.SetEdge2(-1);
                        }
                        else if (beta < MyConfusionPrecision)
                        {
                            // beta==0
                            SP1.SetEdge2(Tri2.GetEdgeNumber(1));
                            if (Tri2.GetEdgeOrientation(1) > 0)
                            {
                                SP1.SetLambda2(alpha);
                            }
                            else
                            {
                                SP1.SetLambda2(1.0 - alpha);
                            }
                        }
                        else if (Math.Abs(beta - alpha) < MyConfusionPrecision)
                        {
                            // beta==alpha
                            SP1.SetEdge2(Tri2.GetEdgeNumber(3));
                            if (Tri2.GetEdgeOrientation(3) > 0)
                            {
                                SP1.SetLambda2(1.0 - alpha);
                            }
                            else
                            {
                                SP1.SetLambda2(alpha);
                            }
                        }
                        else if (Math.Abs(alpha - 1) < MyConfusionPrecision)
                        {
                            // alpha==1
                            // OCCT: lambda from alpha, where the first surface's branch uses beta
                            SP1.SetEdge2(Tri2.GetEdgeNumber(2));
                            if (Tri2.GetEdgeOrientation(2) > 0)
                            {
                                SP1.SetLambda2(alpha);
                            }
                            else
                            {
                                SP1.SetLambda2(1.0 - alpha);
                            }
                        }
                    }
                }
            }
            else
            {
                return 0;
            }
        }

        return NbPoints;
    }

    /// <summary>
    /// Finds the couples of interfering triangles (boxes through a BVH, then <see cref="TriContact"/>) with the cosine of
    /// their angle, appends them to the couples and marks their triangles; returns the number of couples.
    /// </summary>
    public int TriangleCompare()
    {
        // Find couples with interfering bounding boxes
        NCollection_IndexedDataMap<int, NCollection_List<int>> aDMILI = new();
        GetInterferingTriangles(TTriangles1, TPoints1, TTriangles2, TPoints2, aDMILI);
        if (aDMILI.IsEmpty())
        {
            return 0;
        }
        //
        double CoupleAngle = -2.0;
        //
        // Intersection of the triangles
        int i, aNb = aDMILI.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            int i_S1 = aDMILI.FindKey(i);
            ref IntPolyh_Triangle Triangle1 = ref TTriangles1[i_S1];
            ref readonly IntPolyh_Point P1 = ref TPoints1.Value(Triangle1.FirstPoint());
            ref readonly IntPolyh_Point P2 = ref TPoints1.Value(Triangle1.SecondPoint());
            ref readonly IntPolyh_Point P3 = ref TPoints1.Value(Triangle1.ThirdPoint());
            //
            NCollection_List<int> aLI2 = aDMILI.FindFromIndex(i);
            for (NCollection_List<int>.Iterator aItLI = new(aLI2); aItLI.More(); aItLI.Next())
            {
                int i_S2 = aItLI.Value();
                ref IntPolyh_Triangle Triangle2 = ref TTriangles2[i_S2];
                ref readonly IntPolyh_Point Q1 = ref TPoints2.Value(Triangle2.FirstPoint());
                ref readonly IntPolyh_Point Q2 = ref TPoints2.Value(Triangle2.SecondPoint());
                ref readonly IntPolyh_Point Q3 = ref TPoints2.Value(Triangle2.ThirdPoint());
                //
                if (TriContact(P1, P2, P3, Q1, Q2, Q3, ref CoupleAngle) != 0)
                {
                    IntPolyh_Couple aCouple = new(i_S1, i_S2, CoupleAngle);
                    TTrianglesContacts.Append(aCouple);
                    //
                    Triangle1.SetIntersection(true);
                    Triangle2.SetIntersection(true);
                }
            }
        }

        return TTrianglesContacts.Extent();
    }

    // Marks the first not-analyzed couple (T1, T2) analyzed and gets its angle; false if there is none
    private static bool CheckCoupleAndGetAngle(int T1, int T2, ref double Angle,
                                               NCollection_List<IntPolyh_Couple> TTrianglesContacts)
    {
        for (NCollection_List<IntPolyh_Couple>.Iterator aIt = new(TTrianglesContacts); aIt.More(); aIt.Next())
        {
            ref IntPolyh_Couple TestCouple = ref aIt.ChangeValue();
            if (!TestCouple.IsAnalyzed())
            {
                if (TestCouple.FirstValue() == T1 && TestCouple.SecondValue() == T2)
                {
                    TestCouple.SetAnalyzed(true);
                    Angle = TestCouple.Angle();
                    return true;
                }
            }
        }

        return false;
    }

    // Couple T1 T2 is looked for in the list. T11 and T22 are the two other triangles of the edge-edge contact: the
    // couples CT11 (T1, T22) and CT22 (T11, T2) are found too, to be marked if there is a start point
    private static bool CheckCoupleAndGetAngle2(int T1, int T2, int T11, int T22,
                                                NCollection_List<IntPolyh_Couple>.Iterator theItCT11,
                                                NCollection_List<IntPolyh_Couple>.Iterator theItCT22, ref double Angle,
                                                NCollection_List<IntPolyh_Couple> TTrianglesContacts)
    {
        bool Test1, Test2, Test3;
        Test1 = Test2 = Test3 = false;
        //
        for (NCollection_List<IntPolyh_Couple>.Iterator aIt = new(TTrianglesContacts); aIt.More(); aIt.Next())
        {
            ref IntPolyh_Couple TestCouple = ref aIt.ChangeValue();
            if (TestCouple.IsAnalyzed())
            {
                continue;
            }
            //
            if (TestCouple.FirstValue() == T1)
            {
                if (TestCouple.SecondValue() == T2)
                {
                    Test1 = true;
                    TestCouple.SetAnalyzed(true);
                    Angle = TestCouple.Angle();
                }
                else if (TestCouple.SecondValue() == T22)
                {
                    Test2 = true;
                    theItCT11.Reset(TTrianglesContacts, aIt.Index); // theItCT11 = aIt
                    Angle = TestCouple.Angle();
                }
            }
            else if (TestCouple.FirstValue() == T11)
            {
                if (TestCouple.SecondValue() == T2)
                {
                    Test3 = true;
                    theItCT22.Reset(TTrianglesContacts, aIt.Index); // theItCT22 = aIt
                    Angle = TestCouple.Angle();
                }
            }
            //
            if (Test1 && Test2 && Test3)
            {
                break;
            }
        }

        return Test1;
    }

    // If the point is a vertex of a triangle (an edge -1), it is stored in the tangent zones unless it is there already,
    // and 0 is returned; else it is added to the section line (at its end, or its beginning if Prepend) and 1 returned
    private static int CheckNextStartPoint(IntPolyh_SectionLine SectionLine,
                                           IntPolyh_Array<IntPolyh_StartPoint> TTangentZones,
                                           ref IntPolyh_StartPoint SP, bool Prepend = false)
    {
        int Test = 1;
        if ((SP.E1() == -1) || (SP.E2() == -1))
        {
            // The tops of triangle are analyzed
            // It is checked if they are not in the array TTangentZones
            int FinTTZ = TTangentZones.NbItems();
            for (int uiui = 0; uiui < FinTTZ; uiui++)
            {
                IntPolyh_StartPoint TestSP = TTangentZones[uiui];
                if ((Math.Abs(SP.U1() - TestSP.U1()) < MyConfusionPrecision)
                    && (Math.Abs(SP.V1() - TestSP.V1()) < MyConfusionPrecision))
                {
                    if ((Math.Abs(SP.U2() - TestSP.U2()) < MyConfusionPrecision)
                        && (Math.Abs(SP.V2() - TestSP.V2()) < MyConfusionPrecision))
                    {
                        Test = 0; // SP is already in the list of  tops
                        uiui = FinTTZ;
                    }
                }
            }

            if (Test != 0)
            {
                // the top does not belong to the list of TangentZones
                SP.SetChainList(-1);
                TTangentZones[FinTTZ] = SP;
                TTangentZones.IncrementNbItems();
                Test = 0; // the examined point is a top
            }
        }
        else if (Test != 0)
        {
            if (Prepend)
            {
                SectionLine.Prepend(SP);
            }
            else
            {
                SectionLine[SectionLine.NbStartPoints()] = SP;
                SectionLine.IncrementNbStartPoints();
            }
        }

        // if the point is not a top Test=1
        // The chain is continued
        return Test;
    }

    /// <summary>
    /// Loops on the couples of triangles: computes their start points and chains them into section lines, or puts them
    /// in the tangent zones where chaining is not possible.
    /// </summary>
    public int StartPointsChain(IntPolyh_Array<IntPolyh_SectionLine> TSectionLines,
                                IntPolyh_Array<IntPolyh_StartPoint> TTangentZones)
    {
        // Loop on the array of couples filled in the function COMPARE()
        for (NCollection_List<IntPolyh_Couple>.Iterator aIt = new(TTrianglesContacts); aIt.More(); aIt.Next())
        {
            // Check if the couple of triangles has not been already examined.
            if (!aIt.Value().IsAnalyzed())
            {
                int SectionLineIndex = TSectionLines.NbItems();
                // fill last section line if still empty (eap)
                if (SectionLineIndex > 0 && TSectionLines[SectionLineIndex - 1].NbStartPoints() == 0)
                {
                    SectionLineIndex -= 1;
                }
                else
                {
                    TSectionLines.IncrementNbItems();
                }

                IntPolyh_SectionLine MySectionLine = TSectionLines[SectionLineIndex];
                if (MySectionLine.GetN() == 0)
                {
                    // eap
                    MySectionLine.Init(10000); // Initialisation of array of StartPoint
                }

                int NbPoints = -1;
                int T1I, T2I;
                T1I = aIt.Value().FirstValue();
                T2I = aIt.Value().SecondValue();

                // Start points for the current couple are found
                IntPolyh_StartPoint SP1 = new(), SP2 = new();
                NbPoints = StartingPointsResearch(T1I, T2I, ref SP1, ref SP2); // first calculation
                aIt.ChangeValue().SetAnalyzed(true);                           // the couple is marked
                double aCoupleAngle = aIt.Value().Angle();

                if (NbPoints == 1)
                {
                    // particular case top/triangle or edge/edge
                    // the start point is input in the array
                    SP1.SetChainList(SectionLineIndex);
                    SP1.SetAngle(aCoupleAngle);
                    // it is checked if the point is not atop of the triangle
                    if (CheckNextStartPoint(MySectionLine, TTangentZones, ref SP1) != 0)
                    {
                        IntPolyh_StartPoint SPNext1 = new();
                        int TestSP1 = 0;

                        // chain of a side
                        IntPolyh_StartPoint SP11 = new(); //=SP1;
                        if (SP1.E1() >= 0)
                        {
                            //&&(SP1.E2()!=-1) already tested if the point is not a top
                            int NextTriangle1 = 0;
                            if (TEdges1[SP1.E1()].FirstTriangle() != T1I)
                            {
                                NextTriangle1 = TEdges1[SP1.E1()].FirstTriangle();
                            }
                            else
                            {
                                NextTriangle1 = TEdges1[SP1.E1()].SecondTriangle();
                            }

                            double Angle = -2.0;
                            if (CheckCoupleAndGetAngle(NextTriangle1, T2I, ref Angle, TTrianglesContacts))
                            {
                                // it is checked if the couple exists and is marked
                                int NbPoints11 = 0;
                                NbPoints11 = NextStartingPointsResearch(NextTriangle1, T2I, SP1, ref SP11);
                                if (NbPoints11 == 1)
                                {
                                    SP11.SetChainList(SectionLineIndex);
                                    SP11.SetAngle(Angle);

                                    if (CheckNextStartPoint(MySectionLine, TTangentZones, ref SP11) != 0)
                                    {
                                        int EndChainList = 1;
                                        while (EndChainList != 0)
                                        {
                                            TestSP1 = GetNextChainStartPoint(SP11, ref SPNext1, MySectionLine, TTangentZones);
                                            if (TestSP1 == 1)
                                            {
                                                SPNext1.SetChainList(SectionLineIndex);
                                                if (CheckNextStartPoint(MySectionLine, TTangentZones, ref SPNext1) != 0)
                                                {
                                                    SP11 = SPNext1;
                                                }
                                                else
                                                {
                                                    EndChainList = 0;
                                                }
                                            }
                                            else
                                            {
                                                EndChainList = 0; // There is no next point
                                            }
                                        }
                                    }
                                }
                                else
                                {
                                    if (NbPoints11 > 1)
                                    {
                                        // The point is input in the array TTangentZones
                                        TTangentZones[TTangentZones.NbItems()] = SP11; // default list number = -1
                                        TTangentZones.IncrementNbItems();
                                    }
                                }
                            }
                        }

                        // chain of the other side
                        IntPolyh_StartPoint SP12 = new(); //=SP1;
                        if (SP1.E2() >= 0)
                        {
                            //&&(SP1.E1()!=-1) already tested
                            int NextTriangle2;
                            if (TEdges2[SP1.E2()].FirstTriangle() != T2I)
                            {
                                NextTriangle2 = TEdges2[SP1.E2()].FirstTriangle();
                            }
                            else
                            {
                                NextTriangle2 = TEdges2[SP1.E2()].SecondTriangle();
                            }

                            double Angle = -2.0;
                            if (CheckCoupleAndGetAngle(T1I, NextTriangle2, ref Angle, TTrianglesContacts))
                            {
                                int NbPoints12 = 0;
                                NbPoints12 = NextStartingPointsResearch(T1I, NextTriangle2, SP1, ref SP12);
                                if (NbPoints12 == 1)
                                {
                                    SP12.SetChainList(SectionLineIndex);
                                    SP12.SetAngle(Angle);
                                    bool Prepend = true; // eap

                                    if (CheckNextStartPoint(MySectionLine, TTangentZones, ref SP12, Prepend) != 0)
                                    {
                                        int EndChainList = 1;
                                        while (EndChainList != 0)
                                        {
                                            TestSP1 = GetNextChainStartPoint(SP12, ref SPNext1, MySectionLine,
                                                                             TTangentZones, Prepend); // eap
                                            if (TestSP1 == 1)
                                            {
                                                SPNext1.SetChainList(SectionLineIndex);
                                                if (CheckNextStartPoint(MySectionLine, TTangentZones, ref SPNext1, Prepend) != 0)
                                                {
                                                    SP12 = SPNext1;
                                                }
                                                else
                                                {
                                                    EndChainList = 0;
                                                }
                                            }
                                            else
                                            {
                                                EndChainList = 0; // there is no next point
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if (NbPoints12 > 1)
                                        {
                                            // The points are input in the array TTangentZones
                                            TTangentZones[TTangentZones.NbItems()] = SP12; // default list number = -1
                                            TTangentZones.IncrementNbItems();
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                else if (NbPoints == 2)
                {
                    // the start points are input in the array
                    IntPolyh_StartPoint SPNext2 = new();
                    int TestSP2 = 0;
                    int EndChainList = 1;

                    SP1.SetChainList(SectionLineIndex);
                    SP1.SetAngle(aCoupleAngle);
                    if (CheckNextStartPoint(MySectionLine, TTangentZones, ref SP1) != 0)
                    {
                        // chain of a side
                        while (EndChainList != 0)
                        {
                            TestSP2 = GetNextChainStartPoint(SP1, ref SPNext2, MySectionLine, TTangentZones);
                            if (TestSP2 == 1)
                            {
                                SPNext2.SetChainList(SectionLineIndex);
                                if (CheckNextStartPoint(MySectionLine, TTangentZones, ref SPNext2) != 0)
                                {
                                    SP1 = SPNext2;
                                }
                                else
                                {
                                    EndChainList = 0;
                                }
                            }
                            else
                            {
                                EndChainList = 0; // there is no next point
                            }
                        }
                    }

                    SP2.SetChainList(SectionLineIndex);
                    SP2.SetAngle(aCoupleAngle);
                    bool Prepend = true; // eap

                    if (CheckNextStartPoint(MySectionLine, TTangentZones, ref SP2, Prepend) != 0)
                    {
                        // chain of the other side
                        EndChainList = 1;
                        while (EndChainList != 0)
                        {
                            TestSP2 = GetNextChainStartPoint(SP2, ref SPNext2, MySectionLine, TTangentZones, Prepend); // eap
                            if (TestSP2 == 1)
                            {
                                SPNext2.SetChainList(SectionLineIndex);
                                if (CheckNextStartPoint(MySectionLine, TTangentZones, ref SPNext2, Prepend) != 0)
                                {
                                    SP2 = SPNext2;
                                }
                                else
                                {
                                    EndChainList = 0;
                                }
                            }
                            else
                            {
                                EndChainList = 0; // there is no next point
                            }
                        }
                    }
                }
                else if ((NbPoints > 2) && (NbPoints < 7))
                {
                    // More than two start points
                    // the start point is input in the table
                    SP1.SetChainList(SectionLineIndex);
                    CheckNextStartPoint(MySectionLine, TTangentZones, ref SP1);
                }
            }
        }

        return 1;
    }

    /// <summary>
    /// Tries to compute the next start point of a chain, through the triangle on the other side of the edge(s) the
    /// point lies on. Used by <see cref="StartPointsChain"/>, when it is known that there are 2 contact points.
    /// </summary>
    public int GetNextChainStartPoint(in IntPolyh_StartPoint SP, ref IntPolyh_StartPoint SPNext,
                                      IntPolyh_SectionLine MySectionLine,
                                      IntPolyh_Array<IntPolyh_StartPoint> TTangentZones, bool Prepend = false)
    {
        int NbPoints = 0;
        if ((SP.E1() >= 0) && (SP.E2() == -2))
        {
            // case if the point is on edge of T1
            int NextTriangle1;
            if (TEdges1[SP.E1()].FirstTriangle() != SP.T1())
            {
                NextTriangle1 = TEdges1[SP.E1()].FirstTriangle();
            }
            else
            {
                NextTriangle1 = TEdges1[SP.E1()].SecondTriangle();
            }

            // If is checked if two triangles intersect
            double Angle = -2.0;
            if (CheckCoupleAndGetAngle(NextTriangle1, SP.T2(), ref Angle, TTrianglesContacts))
            {
                NbPoints = NextStartingPointsResearch(NextTriangle1, SP.T2(), SP, ref SPNext);
                if (NbPoints != 1)
                {
                    if (NbPoints > 1)
                    {
                        CheckNextStartPoint(MySectionLine, TTangentZones, ref SPNext, Prepend);
                    }
                    else
                    {
                        NbPoints = 0;
                    }
                }
                else
                {
                    SPNext.SetAngle(Angle);
                }
            }
            else
            {
                NbPoints = 0; // this couple does not intersect
            }
        }
        else if ((SP.E1() == -2) && (SP.E2() >= 0))
        {
            // case if the point is on edge of T2
            int NextTriangle2;
            if (TEdges2[SP.E2()].FirstTriangle() != SP.T2())
            {
                NextTriangle2 = TEdges2[SP.E2()].FirstTriangle();
            }
            else
            {
                NextTriangle2 = TEdges2[SP.E2()].SecondTriangle();
            }

            double Angle = -2.0;
            if (CheckCoupleAndGetAngle(SP.T1(), NextTriangle2, ref Angle, TTrianglesContacts))
            {
                NbPoints = NextStartingPointsResearch(SP.T1(), NextTriangle2, SP, ref SPNext);
                if (NbPoints != 1)
                {
                    if (NbPoints > 1)
                    {
                        CheckNextStartPoint(MySectionLine, TTangentZones, ref SPNext, Prepend);
                    }
                    else
                    {
                        NbPoints = 0;
                    }
                }
                else
                {
                    SPNext.SetAngle(Angle);
                }
            }
            else
            {
                NbPoints = 0;
            }
        }
        else if ((SP.E1() == -2) && (SP.E2() == -2))
        {
            // no edge is touched or cut
            NbPoints = 0;
        }
        else if ((SP.E1() >= 0) && (SP.E2() >= 0))
        {
            // the point is located on two edges
            int NextTriangle1;
            if (TEdges1[SP.E1()].FirstTriangle() != SP.T1())
            {
                NextTriangle1 = TEdges1[SP.E1()].FirstTriangle();
            }
            else
            {
                NextTriangle1 = TEdges1[SP.E1()].SecondTriangle();
            }

            int NextTriangle2;
            if (TEdges2[SP.E2()].FirstTriangle() != SP.T2())
            {
                NextTriangle2 = TEdges2[SP.E2()].FirstTriangle();
            }
            else
            {
                NextTriangle2 = TEdges2[SP.E2()].SecondTriangle();
            }

            double Angle = -2.0;

            NCollection_List<IntPolyh_Couple>.Iterator aItCT11 = new(), aItCT22 = new();
            if (CheckCoupleAndGetAngle2(NextTriangle1, NextTriangle2, SP.T1(), SP.T2(), aItCT11, aItCT22, ref Angle,
                                        TTrianglesContacts))
            {
                NbPoints = NextStartingPointsResearch(NextTriangle1, NextTriangle2, SP, ref SPNext);
                if (NbPoints != 1)
                {
                    if (NbPoints > 1)
                    {
                        // The new point is checked
                        CheckNextStartPoint(MySectionLine, TTangentZones, ref SPNext, Prepend);
                    }

                    NbPoints = 0;
                }
                else
                {
                    // NbPoints==1
                    SPNext.SetAngle(Angle);
                    if (aItCT11.More())
                    {
                        aItCT11.ChangeValue().SetAnalyzed(true);
                    }

                    if (aItCT22.More())
                    {
                        aItCT22.ChangeValue().SetAnalyzed(true);
                    }
                }
            }
            else
            {
                NbPoints = 0;
            }
        }
        else if ((SP.E1() == -1) || (SP.E2() == -1))
        {
            // the points are tops of triangle
            // the point is stored in an intermediary array
        }

        return NbPoints;
    }

    //=================================================================================================
    // Accessors

    /// <summary>The points of one surface.</summary>
    public IntPolyh_Array<IntPolyh_Point> GetArrayOfPoints(int SurfID) => SurfID == 1 ? TPoints1 : TPoints2;

    /// <summary>The edges of one surface.</summary>
    public IntPolyh_Array<IntPolyh_Edge> GetArrayOfEdges(int SurfID) => SurfID == 1 ? TEdges1 : TEdges2;

    /// <summary>The triangles of one surface.</summary>
    public IntPolyh_Array<IntPolyh_Triangle> GetArrayOfTriangles(int SurfID) => SurfID == 1 ? TTriangles1 : TTriangles2;

    // GetFinTE and GetFinTT are left out: OCCT declares them and never defines them.

    /// <summary>The box of one surface's points.</summary>
    public Bnd_Box GetBox(int SurfID) => SurfID == 1 ? MyBox1 : MyBox2;

    /// <summary>The couples of interfering triangles.</summary>
    public NCollection_List<IntPolyh_Couple> GetCouples() => TTrianglesContacts;

    /// <summary>Whether the sampling zone is enlarged (B-spline and Bézier surfaces).</summary>
    public void SetEnlargeZone(bool EnlargeZone) => myEnlargeZone = EnlargeZone;

    public bool GetEnlargeZone() => myEnlargeZone;

    /// <summary>The minimum deflection of the triangles of one surface (FlecheMin).</summary>
    public double GetMinDeflection(int SurfID) => (SurfID == 1) ? FlecheMin1 : FlecheMin2;

    /// <summary>The maximum deflection of the triangles of one surface (FlecheMax).</summary>
    public double GetMaxDeflection(int SurfID) => (SurfID == 1) ? FlecheMax1 : FlecheMax2;

    //=================================================================================================
    // Degenerated isolines

    // The indices of the parameters (1 to aNbX) of the degenerated isolines at the bounds of the surface, 0 if none
    private static void DegeneratedIndex(NCollection_Array1<double> aXpars, int aNbX, Adaptor3d_Surface aS,
                                         int aIsoDirection, out int aI1, out int aI2)
    {
        int i;
        bool bDegX1, bDegX2;
        double aDegX1, aDegX2, aTol2, aX;
        //
        aI1 = 0;
        aI2 = 0;
        aTol2 = MyTolerance * MyTolerance;
        //
        if (aIsoDirection == 1)
        {
            // V=const
            bDegX1 = IsDegenerated(aS, 1, aTol2, out aDegX1);
            bDegX2 = IsDegenerated(aS, 2, aTol2, out aDegX2);
        }
        else if (aIsoDirection == 2)
        {
            // U=const
            bDegX1 = IsDegenerated(aS, 3, aTol2, out aDegX1);
            bDegX2 = IsDegenerated(aS, 4, aTol2, out aDegX2);
        }
        else
        {
            return;
        }
        //
        if (!(bDegX1 || bDegX2))
        {
            return;
        }
        //
        for (i = 1; i <= aNbX; ++i)
        {
            aX = aXpars[i];
            if (bDegX1)
            {
                if (Math.Abs(aX - aDegX1) < MyTolerance)
                {
                    aI1 = i;
                }
            }

            if (bDegX2)
            {
                if (Math.Abs(aX - aDegX2) < MyTolerance)
                {
                    aI2 = i;
                }
            }
        }
    }

    // Whether the bound isoline (1, 2: V first and last; 3, 4: U first and last) is degenerated: three points on it
    // within the tolerance. OCCT samples at i * dU and i * dV, not from the first parameter (occt-issues.md, unit 5E).
    private static bool IsDegenerated(Adaptor3d_Surface aS, int aIndex, double aTol2, out double aDegX)
    {
        bool bRet;
        int i, aNbP;
        double aU, dU, aU1, aU2, aV, dV, aV1, aV2, aD2;
        gp_Pnt aP1, aP2;
        //
        bRet = false;
        aNbP = 3;
        aDegX = 99;
        //
        aU1 = aS.FirstUParameter();
        aU2 = aS.LastUParameter();
        aV1 = aS.FirstVParameter();
        aV2 = aS.LastVParameter();
        //
        if (aIndex < 3)
        {
            // V=const
            aV = aV1;
            if (aIndex == 2)
            {
                aV = aV2;
            }

            dU = (aU2 - aU1) / (aNbP - 1);
            aU = aU1;
            aP1 = aS.Value(aU, aV);
            for (i = 1; i < aNbP; ++i)
            {
                aU = i * dU; // OCCT: not aU1 + i * dU
                if (i == aNbP - 1)
                {
                    aU = aU2;
                }

                aP2 = aS.Value(aU, aV);
                aD2 = aP1.SquareDistance(aP2);
                if (aD2 > aTol2)
                {
                    return bRet;
                }

                aP1 = aP2;
            }

            aDegX = aV;
            bRet = !bRet;
        }
        else
        {
            // U=const
            aU = aU1;
            if (aIndex == 4)
            {
                aU = aU2;
            }

            dV = (aV2 - aV1) / (aNbP - 1);
            aV = aV1;
            aP1 = aS.Value(aU, aV);
            for (i = 1; i < aNbP; ++i)
            {
                aV = i * dV; // OCCT: not aV1 + i * dV
                if (i == aNbP - 1)
                {
                    aV = aV2;
                }

                aP2 = aS.Value(aU, aV);
                aD2 = aP1.SquareDistance(aP2);
                if (aD2 > aTol2)
                {
                    return bRet;
                }

                aP1 = aP2;
            }

            bRet = !bRet;
            aDegX = aU;
        }
        //
        return bRet;
    }
}
