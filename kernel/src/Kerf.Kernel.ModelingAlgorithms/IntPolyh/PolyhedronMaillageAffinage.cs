// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_MaillageAffinage.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_MaillageAffinage.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Bvh;
using Kerf.Geometry;
using Occt;
using BVH_Vec3d = Occt.NCollection_Vec3<double>;
using IntPolyh_BoxBndTree = Kerf.Engine.Geometry.Bvh.BoxSet<double, Occt.NCollection_Vec3<double>, int>;

namespace Kerf.Engine.Intersection.Polyhedra;

/// <summary>
/// The low-level algorithm that intersects two surfaces by intersecting their triangulations: sampling, refinement
/// of the triangles by deflection, the interfering couples of triangles and the start points chained into section lines.
/// </summary>
/// <remarks>
/// A working object (decision 21), used by <see cref="PolyhedronIntersection"/>; its arrays and couples are handed out
/// as OCCT does. Debug prints are left out. Calling the triangulation steps (<c>FillArrayOfTriangles</c>, ...) a second
/// time on the same object keeps the items the first pass left in the arrays beyond what it rewrites, as OCCT's does
/// (the enlarged-zone retry of <see cref="PolyhedronIntersection"/> relies on it; see occt-issues.md, unit 5E).
/// </remarks>
public sealed partial class PolyhedronMaillageAffinage
{
    private const double MyTolerance = 10.0e-7;
    private const double MyConfusionPrecision = 10.0e-12;
    private const double SquareMyConfusionPrecision = 10.0e-24;

    private readonly SurfaceAdaptorBase _maSurface1;
    private readonly SurfaceAdaptorBase _maSurface2;
    private Box _myBox1 = new();
    private Box _myBox2 = new();
    private readonly int _nbSamplesU1;
    private readonly int _nbSamplesU2;
    private readonly int _nbSamplesV1;
    private readonly int _nbSamplesV2;
    private double _flecheMax1;
    private double _flecheMax2;
    private double _flecheMin1;
    private double _flecheMin2;
    // For the arrays of Points, Edges and Triangles we need instant access to the items.
    // Moreover, we might add new items during refinement process in case the deflection
    // is too big, thus the vectors should be used.
    private readonly PolyhedronArray<PolyhedronPoint> _tPoints1 = new();
    private readonly PolyhedronArray<PolyhedronPoint> _tPoints2 = new();
    private readonly PolyhedronArray<PolyhedronEdge> _tEdges1 = new();
    private readonly PolyhedronArray<PolyhedronEdge> _tEdges2 = new();
    private readonly PolyhedronArray<PolyhedronTriangle> _tTriangles1 = new();
    private readonly PolyhedronArray<PolyhedronTriangle> _tTriangles2 = new();
    // The intersecting triangles are just filled and then
    // sequentially analyzed, thus we might use the list.
    private readonly NCollection_List<PolyhedronCouple> _tTrianglesContacts = new();
    private bool _enlargeZone;

    /// <summary>The algorithm on two surfaces with 10 x 10 samples each (the last argument, OCCT's print level, is
    /// ignored).</summary>
    public PolyhedronMaillageAffinage(SurfaceAdaptorBase surface1, SurfaceAdaptorBase surface2, int print)
    {
        _ = print;
        _maSurface1 = surface1;
        _maSurface2 = surface2;
        _nbSamplesU1 = 10;
        _nbSamplesU2 = 10;
        _nbSamplesV1 = 10;
        _nbSamplesV2 = 10;
        _flecheMax1 = 0.0;
        _flecheMax2 = 0.0;
        _flecheMin1 = 0.0;
        _flecheMin2 = 0.0;
        _enlargeZone = false;
    }

    /// <summary>The algorithm on two surfaces with the given numbers of samples (the last argument, OCCT's print level,
    /// is ignored).</summary>
    public PolyhedronMaillageAffinage(SurfaceAdaptorBase surface1, int nbSU1, int nbSV1, SurfaceAdaptorBase surface2,
                                     int nbSU2, int nbSV2, int print)
    {
        _ = print;
        _maSurface1 = surface1;
        _maSurface2 = surface2;
        _nbSamplesU1 = nbSU1;
        _nbSamplesU2 = nbSU2;
        _nbSamplesV1 = nbSV1;
        _nbSamplesV2 = nbSV2;
        _flecheMax1 = 0.0;
        _flecheMax2 = 0.0;
        _flecheMin1 = 0.0;
        _flecheMin2 = 0.0;
        _enlargeZone = false;
    }

    //=================================================================================================
    // The BVH of the triangles' boxes

    /// <summary>Selects the pairs of elements of two BVH box sets whose boxes interfere.</summary>
    private sealed class IntPolyh_BoxBndTreeSelector : PairTraverse<double, BVH_Vec3d, IntPolyh_BoxBndTree, double>
    {
        /// <summary>A pair of indices, ordered by the first then the second.</summary>
        internal readonly record struct PairIDs(int ID1, int ID2) : IComparable<PairIDs>
        {
            public int CompareTo(PairIDs other)
                => ID1 < other.ID1 || (ID1 == other.ID1 && ID2 < other.ID2) ? -1
                   : (ID1 == other.ID1 && ID2 == other.ID2 ? 0 : 1);
        }

        private readonly List<PairIDs> _pairs = [];

        public override bool RejectNode(in BVH_Vec3d cMin1, in BVH_Vec3d cMax1, in BVH_Vec3d cMin2,
                                        in BVH_Vec3d cMax2, out double metric)
        {
            metric = 0.0;
            return new Box<double, BVH_Vec3d>(cMin1, cMax1).IsOut(cMin2, cMax2);
        }

        public override bool Accept(int id1, int id2)
        {
            if (!_bvhSet1!.Box(id1).IsOut(_bvhSet2!.Box(id2)))
            {
                _pairs.Add(new PairIDs(_bvhSet1.Element(id1), _bvhSet2.Element(id2)));
                return true;
            }

            return false;
        }

        /// <summary>The selected pairs.</summary>
        public List<PairIDs> Pairs() => _pairs;

        /// <summary>
        /// Sorts the pairs (OCCT's <c>std::sort</c>). The pairs are distinct (each pair of leaves is visited once and
        /// the elements of a set are distinct triangles), so there are no ties and any sort gives MSVC's order
        /// (decision 592).
        /// </summary>
        public void Sort() => _pairs.Sort();
    }

    // The triangles with interfering bounding boxes: for each triangle of the first array (in increasing order), the
    // triangles of the second (in increasing order)
    private static void GetInterferingTriangles(PolyhedronArray<PolyhedronTriangle> triangles1,
                                                PolyhedronArray<PolyhedronPoint> points1,
                                                PolyhedronArray<PolyhedronTriangle> triangles2,
                                                PolyhedronArray<PolyhedronPoint> points2,
                                                NCollection_IndexedDataMap<int, NCollection_List<int>> couples)
    {
        // Use linear builder for BVH construction
        LinearBuilder<double, BVH_Vec3d> lBuilder = new(10);

        // To find the triangles with interfering bounding boxes
        // use the BVH structure
        IntPolyh_BoxBndTree bbTree1 = new(lBuilder), bbTree2 = new(lBuilder);

        // 1. Fill the trees with the boxes of the surfaces triangles
        for (int i = 0; i < 2; ++i)
        {
            IntPolyh_BoxBndTree bbTree = i == 0 ? bbTree1 : bbTree2;
            PolyhedronArray<PolyhedronTriangle> triangles = i == 0 ? triangles1 : triangles2;
            PolyhedronArray<PolyhedronPoint> points = i == 0 ? points1 : points2;

            int nbT = triangles.ItemCount;
            bbTree.SetSize(nbT);
            for (int j = 0; j < nbT; ++j)
            {
                ref PolyhedronTriangle t = ref triangles[j];
                if (!t.IsIntersectionPossible || t.IsDegenerated)
                {
                    continue;
                }

                bbTree.Add(j, global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(t.BoundingBox(points)));
            }

            if (bbTree.Size() == 0)
            {
                return;
            }
        }

        // 2. Construct BVH trees
        bbTree1.Build();
        bbTree2.Build();

        // 3. Perform selection of the interfering triangles
        IntPolyh_BoxBndTreeSelector selector = new();
        selector.SetBVHSets(bbTree1, bbTree2);
        selector.Select();
        selector.Sort();

        List<IntPolyh_BoxBndTreeSelector.PairIDs> pairs = selector.Pairs();
        int nbPairs = pairs.Count;

        for (int i = 0; i < nbPairs; ++i)
        {
            IntPolyh_BoxBndTreeSelector.PairIDs pair = pairs[i];
            if (!couples.FindFromKey(pair.ID1, out NCollection_List<int>? pTriangles2))
            {
                pTriangles2 = new NCollection_List<int>();
                couples.Add(pair.ID1, pTriangles2);
            }

            pTriangles2!.Append(pair.ID2);
        }
    }

    //=================================================================================================
    // Sampling

    /// <summary>Makes the sampling of the surface: the parameters of the triangulation's nodes.</summary>
    public void MakeSampling(int surfID, NCollection_Array1<double> uPars, NCollection_Array1<double> vPars)
    {
        if (surfID == 1)
        {
            PolyhedronTools.MakeSampling(_maSurface1, _nbSamplesU1, _nbSamplesV1, _enlargeZone, uPars, vPars);
        }
        else
        {
            PolyhedronTools.MakeSampling(_maSurface2, _nbSamplesU2, _nbSamplesV2, _enlargeZone, uPars, vPars);
        }
    }

    /// <summary>Computes the points of one surface at its own sampling (the standard method).</summary>
    public void FillArrayOfPnt(int surfID)
    {
        // Make sampling
        NCollection_Array1<double> upars = new(), vpars = new();
        MakeSampling(surfID, upars, vpars);
        // Fill array of points
        FillArrayOfPnt(surfID, upars, vpars);
    }

    /// <summary>Computes the points of one surface at its own sampling, shifted along the normal by the deflection
    /// (forward or reversed: the advanced method).</summary>
    public void FillArrayOfPnt(int surfID, bool isShiftFwd)
    {
        // Make sampling
        NCollection_Array1<double> upars = new(), vpars = new();
        MakeSampling(surfID, upars, vpars);
        // Fill array of points
        FillArrayOfPnt(surfID, isShiftFwd, upars, vpars);
    }

    /// <summary>
    /// Computes the points of one surface at the given sampling and its box, enlarged by 1.2 times the deflection
    /// tolerance (<paramref name="deflTol"/> if given, else computed).
    /// </summary>
    public void FillArrayOfPnt(int surfID, NCollection_Array1<double> upars, NCollection_Array1<double> vpars,
                               double? deflTol = null)
    {
        bool bDegI, bDeg;
        int nbU, nbV, iCnt, i, j;
        int id1, id2, jd1, jd2;
        double tol, u, v, x, y, z;
        Point3D p;
        //
        nbU = (surfID == 1) ? _nbSamplesU1 : _nbSamplesU2;
        nbV = (surfID == 1) ? _nbSamplesV1 : _nbSamplesV2;
        ref Box box = ref (surfID == 1) ? ref _myBox1 : ref _myBox2;
        SurfaceAdaptorBase s = (surfID == 1) ? _maSurface1 : _maSurface2;
        PolyhedronArray<PolyhedronPoint> tPoints = (surfID == 1) ? _tPoints1 : _tPoints2;
        //
        DegeneratedIndex(vpars, nbV, s, 1, out jd1, out jd2);
        id1 = 0;
        id2 = 0;
        if (!(jd1 != 0 || jd2 != 0))
        {
            DegeneratedIndex(upars, nbU, s, 2, out id1, out id2);
        }
        //
        tPoints.Init(nbU * nbV);
        iCnt = 0;
        for (i = 1; i <= nbU; ++i)
        {
            bDegI = (id1 == i || id2 == i);
            u = upars[i];
            for (j = 1; j <= nbV; ++j)
            {
                v = vpars[j];
                p = s.Value(u, v);
                p.Coord(out x, out y, out z);
                ref PolyhedronPoint ip = ref tPoints[iCnt];
                ip.Set(x, y, z, u, v);
                //
                bDeg = bDegI || (jd1 == j || jd2 == j);
                if (bDeg)
                {
                    ip.SetDegenerated(bDeg);
                }

                ++iCnt;
                box.Add(p);
            }
        }
        //
        tPoints.SetNbItems(iCnt);
        //
        tol = deflTol is null ? PolyhedronTools.ComputeDeflection(s, upars, vpars) : deflTol.Value;
        tol *= 1.2;

        box.Get(out double a1, out double a2, out double a3, out double b1, out double b2, out double b3);
        box.Update(a1 - tol, a2 - tol, a3 - tol, b1 + tol, b2 + tol, b3 + tol);
        box.Enlarge(MyTolerance);
    }

    /// <summary>
    /// Computes the points of one surface from the given points and normals, shifted along the normal by 1.5 times the
    /// deflection tolerance (forward or reversed), and its box.
    /// </summary>
    public void FillArrayOfPnt(int surfID, bool isShiftFwd, PolyhedronArray<PolyhedronPointNormal> pointsNorm,
                               NCollection_Array1<double> uPars, NCollection_Array1<double> vPars,
                               double deflTol)
    {
        SurfaceAdaptorBase s = (surfID == 1) ? _maSurface1 : _maSurface2;
        PolyhedronArray<PolyhedronPoint> tPoints = (surfID == 1) ? _tPoints1 : _tPoints2;
        int nbU = (surfID == 1) ? _nbSamplesU1 : _nbSamplesU2;
        int nbV = (surfID == 1) ? _nbSamplesV1 : _nbSamplesV2;
        ref Box box = ref (surfID == 1) ? ref _myBox1 : ref _myBox2;

        int id1 = 0, id2 = 0;
        DegeneratedIndex(vPars, nbV, s, 1, out int jd1, out int jd2);
        if (!(jd1 != 0 || jd2 != 0))
        {
            DegeneratedIndex(uPars, nbU, s, 2, out id1, out id2);
        }

        bool bDegI, bDeg;
        int iCnt = 0, i, j;
        double x, y, z, u, v;

        tPoints.Init(pointsNorm.ItemCount);

        for (i = 1; i <= nbU; ++i)
        {
            u = uPars[i];
            bDegI = (id1 == i || id2 == i);
            for (j = 1; j <= nbV; ++j)
            {
                v = vPars[j];

                ref readonly PolyhedronPointNormal pn = ref pointsNorm.Value(iCnt);
                Vector3D norm = pn.Normal.Multiplied(1.5 * deflTol);
                if (!isShiftFwd)
                {
                    norm = norm.Reversed();
                }

                Point3D p = pn.Point.Translated(norm);

                ref PolyhedronPoint ip = ref tPoints[iCnt];
                p.Coord(out x, out y, out z);
                ip.Set(x, y, z, u, v);
                bDeg = bDegI || (jd1 == j || jd2 == j);
                if (bDeg)
                {
                    ip.SetDegenerated(bDeg);
                }

                ++iCnt;
                box.Add(p);
            }
        }

        tPoints.SetNbItems(iCnt);

        // Update box
        double tol = deflTol * 1.2;
        box.Get(out double a1, out double a2, out double a3, out double b1, out double b2, out double b3);
        box.Update(a1 - tol, a2 - tol, a3 - tol, b1 + tol, b2 + tol, b3 + tol);
        box.Enlarge(MyTolerance);
    }

    /// <summary>
    /// Computes the points of one surface at the given sampling, shifted along the normal (the advanced method).
    /// </summary>
    /// <remarks>OCCT fills the first surface's points whatever <paramref name="surfID"/> is (occt-issues.md, unit 5E).</remarks>
    public void FillArrayOfPnt(int surfID, bool isShiftFwd, NCollection_Array1<double> upars,
                               NCollection_Array1<double> vpars, double? deflTol = null)
    {
        SurfaceAdaptorBase s = (surfID == 1) ? _maSurface1 : _maSurface2;
        // Compute the tolerance
        double tol = deflTol is not null ? deflTol.Value : PolyhedronTools.ComputeDeflection(s, upars, vpars);
        // Fill array of point normal
        PolyhedronArray<PolyhedronPointNormal> points = new();
        PolyhedronTools.FillArrayOfPointNormal(s, upars, vpars, points);

        // Fill array of points
        FillArrayOfPnt(1, isShiftFwd, points, upars, vpars, tol); // OCCT: 1, not SurfID
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
    public void CommonBox(in Box b1, in Box b2, out double xMin, out double yMin, out double zMin,
                          out double xMax, out double yMax, out double zMax)
    {
        _ = b1; // OCCT: the box arguments are not used
        _ = b2;
        _myBox1.Get(out double x10, out double y10, out double z10, out double x11, out double y11, out double z11);
        _myBox2.Get(out double x20, out double y20, out double z20, out double x21, out double y21, out double z21);
        xMin = 0.0;
        yMin = 0.0;
        zMin = 0.0;
        xMax = 0.0;
        yMax = 0.0;
        zMax = 0.0;

        if ((x10 > x21) || (x20 > x11) || (y10 > y21) || (y20 > y11) || (z10 > z21) || (z20 > z11))
        {
        }
        else
        {
            if (x11 <= x21)
            {
                xMax = x11;
            }
            else
            {
                if (x21 <= x11)
                {
                    xMax = x21;
                }
            }

            if (x20 <= x10)
            {
                xMin = x10;
            }
            else
            {
                if (x10 <= x20)
                {
                    xMin = x20;
                }
            }

            if (y11 <= y21)
            {
                yMax = y11;
            }
            else
            {
                if (y21 <= y11)
                {
                    yMax = y21;
                }
            }

            if (y20 <= y10)
            {
                yMin = y10;
            }
            else
            {
                if (y10 <= y20)
                {
                    yMin = y20;
                }
            }

            if (z11 <= z21)
            {
                zMax = z11;
            }
            else
            {
                if (z21 <= z11)
                {
                    zMax = z21;
                }
            }

            if (z20 <= z10)
            {
                zMin = z10;
            }
            else
            {
                if (z10 <= z20)
                {
                    zMin = z20;
                }
            }
        }
        //
        double x, y, z;
        x = xMax - xMin;
        y = yMax - yMin;
        z = zMax - zMin;
        // extension of the box
        if ((x == 0) && (y != 0))
        {
            x = y * 0.1;
        }
        else if ((x == 0) && (z != 0))
        {
            x = z * 0.1;
        }
        else
        {
            x *= 0.1;
        }

        if ((y == 0) && (x != 0))
        {
            y = x * 0.1;
        }
        else if ((y == 0) && (z != 0))
        {
            y = z * 0.1;
        }
        else
        {
            y *= 0.1;
        }

        if ((z == 0) && (x != 0))
        {
            z = x * 0.1;
        }
        else if ((z == 0) && (y != 0))
        {
            z = y * 0.1;
        }
        else
        {
            z *= 0.1;
        }

        xMin -= x;
        xMax += x;
        yMin -= y;
        yMax += y;
        zMin -= z;
        zMax += z;

        // Marking of points included in the common
        int finTP1 = _tPoints1.ItemCount;
        int i;
        for (i = 0; i < finTP1; i++)
        {
            ref PolyhedronPoint pt1 = ref _tPoints1[i];
            int r;
            if (pt1.X() < xMin)
            {
                r = 1;
            }
            else
            {
                if (pt1.X() > xMax)
                {
                    r = 2;
                }
                else
                {
                    r = 0;
                }
            }

            if (pt1.Y() < yMin)
            {
                r |= 4;
            }
            else
            {
                if (pt1.Y() > yMax)
                {
                    r |= 8;
                }
            }

            if (pt1.Z() < zMin)
            {
                r |= 16;
            }
            else
            {
                if (pt1.Z() > zMax)
                {
                    r |= 32;
                }
            }

            pt1.SetPartOfCommon(r);
        }

        int finTP2 = _tPoints2.ItemCount;
        for (int ii = 0; ii < finTP2; ii++)
        {
            ref PolyhedronPoint pt2 = ref _tPoints2[ii];
            int rr;
            if (pt2.X() < xMin)
            {
                rr = 1;
            }
            else
            {
                if (pt2.X() > xMax)
                {
                    rr = 2;
                }
                else
                {
                    rr = 0;
                }
            }

            if (pt2.Y() < yMin)
            {
                rr |= 4;
            }
            else
            {
                if (pt2.Y() > yMax)
                {
                    rr |= 8;
                }
            }

            if (pt2.Z() < zMin)
            {
                rr |= 16;
            }
            else
            {
                if (pt2.Z() > zMax)
                {
                    rr |= 32;
                }
            }

            pt2.SetPartOfCommon(rr);
        }
    }

    //=================================================================================================
    // The triangulation

    /// <summary>Computes the edges of the triangulation of one surface, and links them to its triangles.</summary>
    public void FillArrayOfEdges(int surfID)
    {
        PolyhedronArray<PolyhedronEdge> tEdges = (surfID == 1) ? _tEdges1 : _tEdges2;
        PolyhedronArray<PolyhedronTriangle> tTriangles = (surfID == 1) ? _tTriangles1 : _tTriangles2;
        int nbSamplesU = (surfID == 1) ? _nbSamplesU1 : _nbSamplesU2;
        int nbSamplesV = (surfID == 1) ? _nbSamplesV1 : _nbSamplesV2;

        // NbEdges = 3 + 3*(NbSamplesV-2) + 3*(NbSamplesU-2) +
        //         + 3*(NbSamplesU-2)*(NbSamplesV-2) + (NbSamplesV-1) + (NbSamplesU-1);
        // NbSamplesU and NbSamples cannot be less than 2, so
        int nbEdges = 3 * nbSamplesU * nbSamplesV - 2 * (nbSamplesU + nbSamplesV) + 1;
        tEdges.Init(nbEdges);

        int cpteurTabEdges = 0;

        // mesh u0 v0
        tEdges[cpteurTabEdges].SetFirstPoint(0);  // U V
        tEdges[cpteurTabEdges].SetSecondPoint(1); // U V+1
        tEdges[cpteurTabEdges].SetSecondTriangle(0);
        tTriangles[0].SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
        cpteurTabEdges++;

        tEdges[cpteurTabEdges].SetFirstPoint(0);           // U V
        tEdges[cpteurTabEdges].SetSecondPoint(nbSamplesV); // U+1 V
        tEdges[cpteurTabEdges].SetFirstTriangle(1);
        tTriangles[1].SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
        cpteurTabEdges++;

        tEdges[cpteurTabEdges].SetFirstPoint(0);               // U V
        tEdges[cpteurTabEdges].SetSecondPoint(nbSamplesV + 1); // U+1 V+1
        tEdges[cpteurTabEdges].SetFirstTriangle(0);
        tTriangles[0].SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
        tEdges[cpteurTabEdges].SetSecondTriangle(1);
        tTriangles[1].SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
        cpteurTabEdges++;

        // mesh on U=u0
        int pntInit = 1;
        int boucleMeshV;
        for (boucleMeshV = 1; boucleMeshV < nbSamplesV - 1; boucleMeshV++)
        {
            tEdges[cpteurTabEdges].SetFirstPoint(pntInit);      // U V
            tEdges[cpteurTabEdges].SetSecondPoint(pntInit + 1); // U V+1
            tEdges[cpteurTabEdges].SetSecondTriangle(boucleMeshV * 2);
            tTriangles[boucleMeshV * 2].SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
            cpteurTabEdges++;

            tEdges[cpteurTabEdges].SetFirstPoint(pntInit);                   // U V
            tEdges[cpteurTabEdges].SetSecondPoint(pntInit + nbSamplesV + 1); // U+1 V+1
            tEdges[cpteurTabEdges].SetFirstTriangle(boucleMeshV * 2);
            tTriangles[boucleMeshV * 2].SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
            tEdges[cpteurTabEdges].SetSecondTriangle(boucleMeshV * 2 + 1);
            tTriangles[boucleMeshV * 2 + 1].SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
            cpteurTabEdges++;

            tEdges[cpteurTabEdges].SetFirstPoint(pntInit);               // U V
            tEdges[cpteurTabEdges].SetSecondPoint(pntInit + nbSamplesV); // U+1 V
            tEdges[cpteurTabEdges].SetFirstTriangle(boucleMeshV * 2 + 1);
            tTriangles[boucleMeshV * 2 + 1].SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
            tEdges[cpteurTabEdges].SetSecondTriangle(boucleMeshV * 2 - 2);
            tTriangles[boucleMeshV * 2 - 2].SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
            cpteurTabEdges++;
            pntInit++;
        }

        // mesh on V=v0
        pntInit = nbSamplesV;
        for (boucleMeshV = 1; boucleMeshV < nbSamplesU - 1; boucleMeshV++)
        {
            tEdges[cpteurTabEdges].SetFirstPoint(pntInit);      // U V
            tEdges[cpteurTabEdges].SetSecondPoint(pntInit + 1); // U V+1
            tEdges[cpteurTabEdges].SetFirstTriangle((boucleMeshV - 1) * (nbSamplesV - 1) * 2 + 1);
            tTriangles[(boucleMeshV - 1) * (nbSamplesV - 1) * 2 + 1].SetEdgeAndOrientation(tEdges[cpteurTabEdges],
                                                                                           cpteurTabEdges);
            tEdges[cpteurTabEdges].SetSecondTriangle(boucleMeshV * (nbSamplesV - 1) * 2);
            tTriangles[boucleMeshV * (nbSamplesV - 1) * 2].SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
            cpteurTabEdges++;

            tEdges[cpteurTabEdges].SetFirstPoint(pntInit);                   // U V
            tEdges[cpteurTabEdges].SetSecondPoint(pntInit + nbSamplesV + 1); // U+1 V+1
            tEdges[cpteurTabEdges].SetFirstTriangle(boucleMeshV * (nbSamplesV - 1) * 2);
            tTriangles[boucleMeshV * (nbSamplesV - 1) * 2].SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
            tEdges[cpteurTabEdges].SetSecondTriangle(boucleMeshV * (nbSamplesV - 1) * 2 + 1);
            tTriangles[boucleMeshV * (nbSamplesV - 1) * 2 + 1].SetEdgeAndOrientation(tEdges[cpteurTabEdges],
                                                                                     cpteurTabEdges);
            cpteurTabEdges++;

            tEdges[cpteurTabEdges].SetFirstPoint(pntInit);               // U V
            tEdges[cpteurTabEdges].SetSecondPoint(pntInit + nbSamplesV); // U+1 V
            tEdges[cpteurTabEdges].SetFirstTriangle(boucleMeshV * (nbSamplesV - 1) * 2 + 1);
            tTriangles[boucleMeshV * (nbSamplesV - 1) * 2 + 1].SetEdgeAndOrientation(tEdges[cpteurTabEdges],
                                                                                     cpteurTabEdges);
            cpteurTabEdges++;
            pntInit += nbSamplesV;
        }

        pntInit = nbSamplesV + 1;
        // To provide recursion I associate a point with three edges
        for (int boucleMeshU = 1; boucleMeshU < nbSamplesU - 1; boucleMeshU++)
        {
            for (boucleMeshV = 1; boucleMeshV < nbSamplesV - 1; boucleMeshV++)
            {
                tEdges[cpteurTabEdges].SetFirstPoint(pntInit);      // U V
                tEdges[cpteurTabEdges].SetSecondPoint(pntInit + 1); // U V+1
                tEdges[cpteurTabEdges].SetFirstTriangle((nbSamplesV - 1) * 2 * (boucleMeshU - 1) + boucleMeshV * 2 + 1);
                tTriangles[(nbSamplesV - 1) * 2 * (boucleMeshU - 1) + boucleMeshV * 2 + 1]
                    .SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
                tEdges[cpteurTabEdges].SetSecondTriangle((nbSamplesV - 1) * 2 * boucleMeshU + boucleMeshV * 2);
                tTriangles[(nbSamplesV - 1) * 2 * boucleMeshU + boucleMeshV * 2]
                    .SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
                cpteurTabEdges++;

                tEdges[cpteurTabEdges].SetFirstPoint(pntInit);                   // U V
                tEdges[cpteurTabEdges].SetSecondPoint(pntInit + nbSamplesV + 1); // U+1 V+1
                tEdges[cpteurTabEdges].SetFirstTriangle((nbSamplesV - 1) * 2 * boucleMeshU + boucleMeshV * 2);
                tTriangles[(nbSamplesV - 1) * 2 * boucleMeshU + boucleMeshV * 2]
                    .SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
                tEdges[cpteurTabEdges].SetSecondTriangle((nbSamplesV - 1) * 2 * boucleMeshU + boucleMeshV * 2 + 1);
                tTriangles[(nbSamplesV - 1) * 2 * boucleMeshU + boucleMeshV * 2 + 1]
                    .SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
                cpteurTabEdges++;

                tEdges[cpteurTabEdges].SetFirstPoint(pntInit);               // U V
                tEdges[cpteurTabEdges].SetSecondPoint(pntInit + nbSamplesV); // U+1 V
                tEdges[cpteurTabEdges].SetFirstTriangle((nbSamplesV - 1) * 2 * boucleMeshU + boucleMeshV * 2 + 1);
                tTriangles[(nbSamplesV - 1) * 2 * boucleMeshU + boucleMeshV * 2 + 1]
                    .SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
                tEdges[cpteurTabEdges].SetSecondTriangle((nbSamplesV - 1) * 2 * boucleMeshU + boucleMeshV * 2 - 2);
                tTriangles[(nbSamplesV - 1) * 2 * boucleMeshU + boucleMeshV * 2 - 2]
                    .SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
                cpteurTabEdges++;
                pntInit++; // Pass to the next point
            }

            pntInit++; // Pass the last point of the column
            pntInit++; // Pass the first point of the next column
        }

        // close mesh on U=u1
        pntInit = (nbSamplesU - 1) * nbSamplesV; // point U=u1 V=0
        for (boucleMeshV = 0; boucleMeshV < nbSamplesV - 1; boucleMeshV++)
        {
            tEdges[cpteurTabEdges].SetFirstPoint(pntInit);      // U=u1 V
            tEdges[cpteurTabEdges].SetSecondPoint(pntInit + 1); // U=u1 V+1
            tEdges[cpteurTabEdges].SetFirstTriangle((nbSamplesU - 2) * (nbSamplesV - 1) * 2 + boucleMeshV * 2 + 1);
            tTriangles[(nbSamplesU - 2) * (nbSamplesV - 1) * 2 + boucleMeshV * 2 + 1]
                .SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
            cpteurTabEdges++;
            pntInit++;
        }

        // close mesh on V=v1
        for (boucleMeshV = 0; boucleMeshV < nbSamplesU - 1; boucleMeshV++)
        {
            tEdges[cpteurTabEdges].SetFirstPoint(nbSamplesV - 1 + boucleMeshV * nbSamplesV);         // U V=v1
            tEdges[cpteurTabEdges].SetSecondPoint(nbSamplesV - 1 + (boucleMeshV + 1) * nbSamplesV);  // U+1 V=v1
            tEdges[cpteurTabEdges].SetSecondTriangle(boucleMeshV * 2 * (nbSamplesV - 1) + (nbSamplesV - 2) * 2);
            tTriangles[boucleMeshV * 2 * (nbSamplesV - 1) + (nbSamplesV - 2) * 2]
                .SetEdgeAndOrientation(tEdges[cpteurTabEdges], cpteurTabEdges);
            cpteurTabEdges++;
        }

        tEdges.SetNbItems(cpteurTabEdges);
    }

    /// <summary>
    /// Computes the triangles of one surface from its points (two per cell of the sampling), and marks the triangles
    /// whose points are all on one side out of the common box as not intersecting.
    /// </summary>
    public void FillArrayOfTriangles(int surfID)
    {
        int cpteurTabTriangles = 0;
        int pntInit = 0;

        PolyhedronArray<PolyhedronPoint> tPoints = (surfID == 1) ? _tPoints1 : _tPoints2;
        PolyhedronArray<PolyhedronTriangle> tTriangles = (surfID == 1) ? _tTriangles1 : _tTriangles2;
        int nbSamplesU = (surfID == 1) ? _nbSamplesU1 : _nbSamplesU2;
        int nbSamplesV = (surfID == 1) ? _nbSamplesV1 : _nbSamplesV2;

        tTriangles.Init(2 * (nbSamplesU - 1) * (nbSamplesV - 1));
        // To provide recursion, I associate a point with two triangles
        for (int boucleMeshU = 0; boucleMeshU < nbSamplesU - 1; boucleMeshU++)
        {
            for (int boucleMeshV = 0; boucleMeshV < nbSamplesV - 1; boucleMeshV++)
            {
                // FIRST TRIANGLE
                tTriangles[cpteurTabTriangles].SetFirstPoint(pntInit);                  // U V
                tTriangles[cpteurTabTriangles].SetSecondPoint(pntInit + 1);             // U V+1
                tTriangles[cpteurTabTriangles].SetThirdPoint(pntInit + nbSamplesV + 1); // U+1 V+1

                // IF ITS EDGE CONTACTS WITH THE COMMON BOX IP REMAINS = A 1
                if (((tPoints[pntInit].PartOfCommon() & tPoints[pntInit + 1].PartOfCommon()) != 0)
                    && ((tPoints[pntInit + 1].PartOfCommon() & tPoints[pntInit + nbSamplesV + 1].PartOfCommon()) != 0)
                    && ((tPoints[pntInit + nbSamplesV + 1].PartOfCommon() & tPoints[pntInit].PartOfCommon()) != 0))
                {
                    // IF NOT IP=0
                    tTriangles[cpteurTabTriangles].SetIntersectionPossible(false);
                }

                cpteurTabTriangles++;

                // SECOND TRIANGLE
                tTriangles[cpteurTabTriangles].SetFirstPoint(pntInit);                   // U V
                tTriangles[cpteurTabTriangles].SetSecondPoint(pntInit + nbSamplesV + 1); // U+1 V+1
                tTriangles[cpteurTabTriangles].SetThirdPoint(pntInit + nbSamplesV);      // U+1 V

                if (((tPoints[pntInit].PartOfCommon() & tPoints[pntInit + nbSamplesV + 1].PartOfCommon()) != 0)
                    && ((tPoints[pntInit + nbSamplesV + 1].PartOfCommon() & tPoints[pntInit + nbSamplesV].PartOfCommon()) != 0)
                    && ((tPoints[pntInit + nbSamplesV].PartOfCommon() & tPoints[pntInit].PartOfCommon()) != 0))
                {
                    tTriangles[cpteurTabTriangles].SetIntersectionPossible(false);
                }

                cpteurTabTriangles++;

                pntInit++; // Pass to the next point
            }

            pntInit++; // Pass the last point of the column
        }

        tTriangles.SetNbItems(cpteurTabTriangles);
    }

    //=================================================================================================
    // Refinement

    /// <summary>Refines all the marked triangles of both surfaces once.</summary>
    public void CommonPartRefinement()
    {
        int finInit1 = _tTriangles1.ItemCount;
        for (int i = 0; i < finInit1; i++)
        {
            if (_tTriangles1[i].IsIntersectionPossible)
            {
                _tTriangles1[i].MiddleRefinement(i, _maSurface1, _tPoints1, _tTriangles1, _tEdges1);
            }
        }

        int finInit2 = _tTriangles2.ItemCount;
        for (int ii = 0; ii < finInit2; ii++)
        {
            if (_tTriangles2[ii].IsIntersectionPossible)
            {
                _tTriangles2[ii].MiddleRefinement(ii, _maSurface2, _tPoints2, _tTriangles2, _tEdges2);
            }
        }
    }

    /// <summary>Refines all the marked triangles of one surface once.</summary>
    public void LocalSurfaceRefinement(int surfID)
    {
        // refine locally, but systematically the chosen surface
        if (surfID == 1)
        {
            int finInit1 = _tTriangles1.ItemCount;
            for (int i = 0; i < finInit1; i++)
            {
                if (_tTriangles1[i].IsIntersectionPossible)
                {
                    _tTriangles1[i].MiddleRefinement(i, _maSurface1, _tPoints1, _tTriangles1, _tEdges1);
                }
            }
        }
        //
        if (surfID == 2)
        {
            int finInit2 = _tTriangles2.ItemCount;
            for (int ii = 0; ii < finInit2; ii++)
            {
                if (_tTriangles2[ii].IsIntersectionPossible)
                {
                    _tTriangles2[ii].MiddleRefinement(ii, _maSurface2, _tPoints2, _tTriangles2, _tEdges2);
                }
            }
        }
    }

    /// <summary>Computes the deflection of every triangle of one surface, and their minimum and maximum.</summary>
    public void ComputeDeflections(int surfID)
    {
        SurfaceAdaptorBase surface = (surfID == 1) ? _maSurface1 : _maSurface2;
        PolyhedronArray<PolyhedronPoint> tPoints = (surfID == 1) ? _tPoints1 : _tPoints2;
        PolyhedronArray<PolyhedronTriangle> tTriangles = (surfID == 1) ? _tTriangles1 : _tTriangles2;
        ref double flecheMin = ref (surfID == 1) ? ref _flecheMin1 : ref _flecheMin2;
        ref double flecheMax = ref (surfID == 1) ? ref _flecheMax1 : ref _flecheMax2;

        flecheMax = -RealLast();
        flecheMin = RealLast();
        int finTT = tTriangles.ItemCount;

        for (int i = 0; i < finTT; i++)
        {
            ref PolyhedronTriangle triangle = ref tTriangles[i];
            double fleche = triangle.ComputeDeflection(surface, tPoints);
            if (fleche > flecheMax)
            {
                flecheMax = fleche;
            }

            if (fleche < flecheMin)
            {
                flecheMin = fleche;
            }
        }
    }

    // Refinement of the triangles depending on the deflection
    private static void TrianglesDeflectionsRefinement(SurfaceAdaptorBase s1,
                                                       PolyhedronArray<PolyhedronTriangle> triangles1,
                                                       PolyhedronArray<PolyhedronEdge> edges1,
                                                       PolyhedronArray<PolyhedronPoint> points1,
                                                       double flecheCritique1,
                                                       SurfaceAdaptorBase s2,
                                                       PolyhedronArray<PolyhedronTriangle> triangles2,
                                                       PolyhedronArray<PolyhedronEdge> edges2,
                                                       PolyhedronArray<PolyhedronPoint> points2,
                                                       double flecheCritique2)
    {
        // Find the intersecting triangles
        NCollection_IndexedDataMap<int, NCollection_List<int>> dmili = new();
        GetInterferingTriangles(triangles1, points1, triangles2, points2, dmili);
        //
        // Interfering triangles of second surface
        NCollection_Map<int> mIntS2 = new();
        // Since the number of the triangles may change during refinement,
        // save the number of triangles before refinement
        int finTT1 = triangles1.ItemCount;
        int finTT2 = triangles2.ItemCount;
        //
        // Analyze interfering triangles
        for (int i_S1 = 0; i_S1 < finTT1; i_S1++)
        {
            ref PolyhedronTriangle triangle1 = ref triangles1[i_S1];
            if (!triangle1.IsIntersectionPossible)
            {
                continue;
            }
            //
            if (!dmili.FindFromKey(i_S1, out NCollection_List<int>? pLI) || pLI is null || pLI.IsEmpty())
            {
                // Mark non-interfering triangles of S1 to avoid their repeated usage
                triangle1.SetIntersectionPossible(false);
                continue;
            }
            //
            if (triangle1.Deflection() > flecheCritique1)
            {
                triangle1.MiddleRefinement(i_S1, s1, points1, triangles1, edges1);
            }
            //
            for (NCollection_List<int>.Iterator iter = new(pLI); iter.More(); iter.Next())
            {
                int i_S2 = iter.Value();
                if (mIntS2.Add(i_S2))
                {
                    ref PolyhedronTriangle triangle2 = ref triangles2[i_S2];
                    if (triangle2.Deflection() > flecheCritique2)
                    {
                        // Refinement of the larger triangles
                        triangle2.MiddleRefinement(i_S2, s2, points2, triangles2, edges2);
                    }
                }
            }
        }
        //
        // Mark non-interfering triangles of S2 to avoid their repeated usage
        if (mIntS2.Extent() < finTT2)
        {
            for (int i_S2 = 0; i_S2 < finTT2; i_S2++)
            {
                if (!mIntS2.Contains(i_S2))
                {
                    triangles2[i_S2].SetIntersectionPossible(false);
                }
            }
        }
    }

    // Refinement of the large triangles in case one surface is much smaller then the other
    private static void LargeTrianglesDeflectionsRefinement(SurfaceAdaptorBase s,
                                                            PolyhedronArray<PolyhedronTriangle> triangles,
                                                            PolyhedronArray<PolyhedronEdge> edges,
                                                            PolyhedronArray<PolyhedronPoint> points,
                                                            in Box oppositeBox)
    {
        // Find all triangles of the bigger surface with bounding boxes
        // overlapping the bounding box the other surface
        NCollection_List<int> lit = new();
        int i, nbT = triangles.ItemCount;
        for (i = 0; i < nbT; ++i)
        {
            ref PolyhedronTriangle triangle = ref triangles[i];
            if (!triangle.IsIntersectionPossible || triangle.IsDegenerated)
            {
                continue;
            }
            //
            Box box = triangle.BoundingBox(points);
            if (box.IsVoid)
            {
                continue;
            }
            //
            if (box.IsOut(oppositeBox))
            {
                triangle.SetIntersectionPossible(false);
                continue;
            }
            //
            lit.Append(i);
        }
        //
        if (lit.IsEmpty())
        {
            return;
        }
        //
        // One surface is very small relatively to the other.
        // The criterion of refining for large surface depends on the size of
        // the bounding box of the other - since the criterion should be minimized,
        // the smallest side of the bounding box is taken
        oppositeBox.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
        double dx = Math.Abs(x1 - x0);
        double dy = Math.Abs(y1 - y0);
        double diag = Math.Abs(z1 - z0);
        double dd = (dx > dy) ? dy : dx;
        if (diag > dd)
        {
            diag = dd;
        }

        // calculation of the criterion of refining
        double critereAffinage = 0.0;
        double diagPonderation = 0.5;
        critereAffinage = diag * diagPonderation;
        //
        // The deflection of the greater is compared to the size of the smaller
        for (NCollection_List<int>.Iterator iter = new(lit); iter.More(); iter.Next())
        {
            i = iter.Value();
            ref PolyhedronTriangle triangle = ref triangles[i];
            if (triangle.Deflection() > critereAffinage)
            {
                triangle.MultipleMiddleRefinement(critereAffinage, oppositeBox, i, s, points, triangles,
                                                   edges);
            }
            else
            {
                triangle.MiddleRefinement(i, s, points, triangles, edges);
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
        double flecheCritique1;
        if (_flecheMin1 > _flecheMax1)
        {
            return;
        }
        else
        {
            // fleche min + (flechemax-flechemin) * 80/100
            flecheCritique1 = _flecheMin1 * 0.2 + _flecheMax1 * 0.8;
        }

        // Compute deflections for the second surface
        ComputeDeflections(2);

        //-- Check arrows at output
        double flecheCritique2;
        if (_flecheMin2 > _flecheMax2)
        {
            return;
        }
        else
        {
            // fleche min + (flechemax-flechemin) * 80/100
            flecheCritique2 = _flecheMin2 * 0.2 + _flecheMax2 * 0.8;
        }

        // The greatest of two bounding boxes created in FillArrayOfPoints is found.
        // Then this value is weighted depending on the discretization
        // (NbSamplesU and NbSamplesV)
        double diag1, diag2;

        _myBox1.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
        x0 -= x1;
        y0 -= y1;
        z0 -= z1;
        diag1 = x0 * x0 + y0 * y0 + z0 * z0;
        double nbSamplesUV1 = (double)_nbSamplesU1 * (double)_nbSamplesV1;
        diag1 /= nbSamplesUV1;

        _myBox2.Get(out x0, out y0, out z0, out x1, out y1, out z1);
        x0 -= x1;
        y0 -= y1;
        z0 -= z1;
        diag2 = x0 * x0 + y0 * y0 + z0 * z0;
        double nbSamplesUV2 = (double)_nbSamplesU2 * (double)_nbSamplesV2;
        diag2 /= nbSamplesUV2;

        // The surface with the greatest bounding box is "discretized"
        if (diag1 < diag2)
        {
            // second is discretized
            if (flecheCritique2 < diag1)
            {
                // The corresponding sizes are not too disproportional
                TrianglesDeflectionsRefinement(_maSurface1, _tTriangles1, _tEdges1, _tPoints1, flecheCritique1,
                                               _maSurface2, _tTriangles2, _tEdges2, _tPoints2, flecheCritique2);
            }
            else
            {
                // second surface is much larger then the first
                LargeTrianglesDeflectionsRefinement(_maSurface2, _tTriangles2, _tEdges2, _tPoints2, _myBox1);
            }
        }
        else
        {
            // first is discretized
            if (flecheCritique1 < diag2)
            {
                // The corresponding sizes are not too disproportional
                TrianglesDeflectionsRefinement(_maSurface2, _tTriangles2, _tEdges2, _tPoints2, flecheCritique2,
                                               _maSurface1, _tTriangles1, _tEdges1, _tPoints1, flecheCritique1);
            }
            else
            {
                // first surface is much larger then the second
                LargeTrianglesDeflectionsRefinement(_maSurface1, _tTriangles1, _tEdges1, _tPoints1, _myBox2);
            }
        }
    }

    //=================================================================================================
    // Contact of triangles

    // Used by project6
    private static double MaxSR(double a, double b, double c)
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
    private static double MinSR(double a, double b, double c)
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
    private static int Project6(in PolyhedronPoint ax, in PolyhedronPoint p1, in PolyhedronPoint p2, in PolyhedronPoint p3,
                                in PolyhedronPoint q1, in PolyhedronPoint q2, in PolyhedronPoint q3)
    {
        double P1 = ax.Dot(p1);
        double P2 = ax.Dot(p2);
        double P3 = ax.Dot(p3);
        double Q1 = ax.Dot(q1);
        double Q2 = ax.Dot(q2);
        double Q3 = ax.Dot(q3);

        double mx1 = MaxSR(P1, P2, P3);
        double mn1 = MinSR(P1, P2, P3);
        double mx2 = MaxSR(Q1, Q2, Q3);
        double mn2 = MinSR(Q1, Q2, Q3);

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
    /// neither normal vanishes, <paramref name="angle"/> is the cosine of the angle between their normals (else it is
    /// left unchanged).
    /// </summary>
    public int TriContact(in PolyhedronPoint P1, in PolyhedronPoint P2, in PolyhedronPoint P3, in PolyhedronPoint Q1,
                          in PolyhedronPoint Q2, in PolyhedronPoint Q3, ref double angle)
    {
        // The first triangle is (p1,p2,p3).  The other is (q1,q2,q3).
        // The edges are (e1,e2,e3) and (f1,f2,f3).
        // The normals are n1 and m1
        // Outwards are (g1,g2,g3) and (h1,h2,h3).

        if (MaxSR(P1.X(), P2.X(), P3.X()) < MinSR(Q1.X(), Q2.X(), Q3.X()))
        {
            return 0;
        }

        if (MaxSR(P1.Y(), P2.Y(), P3.Y()) < MinSR(Q1.Y(), Q2.Y(), Q3.Y()))
        {
            return 0;
        }

        if (MaxSR(P1.Z(), P2.Z(), P3.Z()) < MinSR(Q1.Z(), Q2.Z(), Q3.Z()))
        {
            return 0;
        }

        if (MinSR(P1.X(), P2.X(), P3.X()) > MaxSR(Q1.X(), Q2.X(), Q3.X()))
        {
            return 0;
        }

        if (MinSR(P1.Y(), P2.Y(), P3.Y()) > MaxSR(Q1.Y(), Q2.Y(), Q3.Y()))
        {
            return 0;
        }

        if (MinSR(P1.Z(), P2.Z(), P3.Z()) > MaxSR(Q1.Z(), Q2.Z(), Q3.Z()))
        {
            return 0;
        }

        PolyhedronPoint p1 = new(), p2 = new(), p3 = new();
        PolyhedronPoint q1 = new(), q2 = new(), q3 = new();
        PolyhedronPoint e1 = new(), e2 = new(), e3 = new();
        PolyhedronPoint f1 = new(), f2 = new(), f3 = new();
        PolyhedronPoint g1 = new(), g2 = new(), g3 = new();
        PolyhedronPoint h1 = new(), h2 = new(), h3 = new();
        PolyhedronPoint n1 = new(), m1 = new();
        PolyhedronPoint z = new();

        PolyhedronPoint ef11 = new(), ef12 = new(), ef13 = new();
        PolyhedronPoint ef21 = new(), ef22 = new(), ef23 = new();
        PolyhedronPoint ef31 = new(), ef32 = new(), ef33 = new();

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

        if (Project6(n1, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T2 is not higher or lower than T1
        }

        if (Project6(m1, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T1 is not higher of lower than T2
        }

        if (Project6(ef11, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (Project6(ef12, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (Project6(ef13, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (Project6(ef21, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (Project6(ef22, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (Project6(ef23, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (Project6(ef31, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (Project6(ef32, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (Project6(ef33, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0;
        }

        if (Project6(g1, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T2 is outside of T1 in the plane of T1
        }

        if (Project6(g2, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T2 is outside of T1 in the plane of T1
        }

        if (Project6(g3, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T2 is outside of T1 in the plane of T1
        }

        if (Project6(h1, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T1 is outside of T2 in the plane of T2
        }

        if (Project6(h2, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T1 is outside of T2 in the plane of T2
        }

        if (Project6(h3, p1, p2, p3, q1, q2, q3) == 0)
        {
            return 0; // T1 is outside of T2 in the plane of T2
        }

        // Calculation of cosinus angle between two normals
        double sqModn1 = -1.0;
        double sqModm1 = -1.0;
        sqModn1 = n1.SquareModulus();
        if (sqModn1 > SquareMyConfusionPrecision)
        {
            sqModm1 = m1.SquareModulus();
        }

        // OCCT: Angle is left unchanged when a normal vanishes (TriangleCompare then keeps the previous couple's)
        if (sqModm1 > SquareMyConfusionPrecision)
        {
            angle = (n1.Dot(m1)) / (Math.Sqrt(sqModn1) * Math.Sqrt(sqModm1));
        }

        return 1;
    }

    // Used by StartingPointsResearch() to control the number of points found and keep the result in conformity
    // (1 or 2 points)
    private static void TestNbPoints(int triSurfID, ref int nbPoints, ref int nbPointsTotal, in PolyhedronStartPoint pt1,
                                     in PolyhedronStartPoint pt2, ref PolyhedronStartPoint sp1,
                                     ref PolyhedronStartPoint sp2)
    {
        _ = triSurfID;
        // already checked in TriangleEdgeContact
        //  if( (NbPoints==2)&&(Pt1.CheckSameSP(Pt2)) ) NbPoints=1;

        if (nbPoints > 2)
        {
        }
        else
        {
            if ((nbPoints == 1) && (nbPointsTotal == 0))
            {
                sp1 = pt1;
                nbPointsTotal = 1;
            }
            else if ((nbPoints == 1) && (nbPointsTotal == 1))
            {
                if (pt1.CheckSameSP(sp1) != 1)
                {
                    sp2 = pt1;
                    nbPointsTotal = 2;
                }
            }
            else if ((nbPoints == 1) && (nbPointsTotal == 2))
            {
                if ((sp1.CheckSameSP(pt1) != 0) || (sp2.CheckSameSP(pt1) != 0))
                {
                    nbPointsTotal = 2;
                }
                else
                {
                    nbPointsTotal = 3;
                }
            }
            else if ((nbPoints == 2) && (nbPointsTotal == 0))
            {
                sp1 = pt1;
                sp2 = pt2;
                nbPointsTotal = 2;
            }
            else if ((nbPoints == 2) && (nbPointsTotal == 1))
            {
                // there is also Pt1 != Pt2
                if (sp1.CheckSameSP(pt1) != 0)
                {
                    sp2 = pt2;
                    nbPointsTotal = 2;
                }
                else if (sp1.CheckSameSP(pt2) != 0)
                {
                    sp2 = pt1;
                    nbPointsTotal = 2;
                }
                else
                {
                    nbPointsTotal = 3; // case SP1!=Pt1 && SP1!=Pt2!
                }
            }
            else if ((nbPoints == 2) && (nbPointsTotal == 2))
            {
                // there is also SP1!=SP2
                if ((sp1.CheckSameSP(pt1) != 0) || (sp1.CheckSameSP(pt2) != 0))
                {
                    if ((sp2.CheckSameSP(pt1) != 0) || (sp2.CheckSameSP(pt2) != 0))
                    {
                        nbPointsTotal = 2;
                    }
                    else
                    {
                        nbPointsTotal = 3;
                    }
                }
                else
                {
                    nbPointsTotal = 3;
                }
            }
        }
    }

    /// <summary>
    /// The intersection points of two triangles (by index into the surfaces' triangles): returns their number; more
    /// than two means the triangles are coplanar.
    /// </summary>
    public int StartingPointsResearch(int t1, int t2, ref PolyhedronStartPoint sp1, ref PolyhedronStartPoint sp2)
    {
        ref readonly PolyhedronTriangle tri1 = ref _tTriangles1.Value(t1);
        ref readonly PolyhedronTriangle tri2 = ref _tTriangles2.Value(t2);

        ref readonly PolyhedronPoint p1 = ref _tPoints1.Value(tri1.FirstPoint());
        ref readonly PolyhedronPoint p2 = ref _tPoints1.Value(tri1.SecondPoint());
        ref readonly PolyhedronPoint p3 = ref _tPoints1.Value(tri1.ThirdPoint());
        ref readonly PolyhedronPoint q1 = ref _tPoints2.Value(tri2.FirstPoint());
        ref readonly PolyhedronPoint q2 = ref _tPoints2.Value(tri2.SecondPoint());
        ref readonly PolyhedronPoint q3 = ref _tPoints2.Value(tri2.ThirdPoint());

        // The first triangle is (p1,p2,p3).  The other is (q1,q2,q3).
        // The sides are (e1,e2,e3) and (f1,f2,f3).
        // The normals are n1 and m1

        PolyhedronPoint e1 = p2 - p1;
        PolyhedronPoint e2 = p3 - p2;
        PolyhedronPoint e3 = p1 - p3;

        PolyhedronPoint f1 = q2 - q1;
        PolyhedronPoint f2 = q3 - q2;
        PolyhedronPoint f3 = q1 - q3;

        PolyhedronPoint nn1 = new(), mm1 = new();
        nn1.Cross(e1, e2); // normal to the first triangle
        mm1.Cross(f1, f2); // normal to the second triangle

        double nn1modulus, mm1modulus;
        nn1modulus = Math.Sqrt(nn1.SquareModulus());
        mm1modulus = Math.Sqrt(mm1.SquareModulus());

        //-------------------------------------------------
        // calculation of intersections points between triangles
        //-------------------------------------------------
        int nbPoints = 0;
        int nbPointsTotal = 0;

        // check T1 normal
        if (Math.Abs(nn1modulus) < MyConfusionPrecision)
        {
        }
        else
        {
            PolyhedronPoint n1 = nn1.Divide(nn1modulus);
            // T2 edges with T1
            if (nbPointsTotal < 3)
            {
                PolyhedronStartPoint pt1 = new(), pt2 = new();
                nbPoints = TriangleEdgeContact(1, 1, tri1, tri2, p1, p2, p3, e1, e2, e3, q1, q2, f1, n1, ref pt1, ref pt2);
                TestNbPoints(1, ref nbPoints, ref nbPointsTotal, pt1, pt2, ref sp1, ref sp2);
            }

            if (nbPointsTotal < 3)
            {
                PolyhedronStartPoint pt1 = new(), pt2 = new();
                nbPoints = TriangleEdgeContact(1, 2, tri1, tri2, p1, p2, p3, e1, e2, e3, q2, q3, f2, n1, ref pt1, ref pt2);
                TestNbPoints(1, ref nbPoints, ref nbPointsTotal, pt1, pt2, ref sp1, ref sp2);
            }

            if (nbPointsTotal < 3)
            {
                PolyhedronStartPoint pt1 = new(), pt2 = new();
                nbPoints = TriangleEdgeContact(1, 3, tri1, tri2, p1, p2, p3, e1, e2, e3, q3, q1, f3, n1, ref pt1, ref pt2);
                TestNbPoints(1, ref nbPoints, ref nbPointsTotal, pt1, pt2, ref sp1, ref sp2);
            }
        }

        // check T2 normal
        if (Math.Abs(mm1modulus) < MyConfusionPrecision)
        {
        }
        else
        {
            PolyhedronPoint m1 = mm1.Divide(mm1modulus);
            // T1 edges with T2
            if (nbPointsTotal < 3)
            {
                PolyhedronStartPoint pt1 = new(), pt2 = new();
                nbPoints = TriangleEdgeContact(2, 1, tri1, tri2, q1, q2, q3, f1, f2, f3, p1, p2, e1, m1, ref pt1, ref pt2);
                TestNbPoints(2, ref nbPoints, ref nbPointsTotal, pt1, pt2, ref sp1, ref sp2);
            }

            if (nbPointsTotal < 3)
            {
                PolyhedronStartPoint pt1 = new(), pt2 = new();
                nbPoints = TriangleEdgeContact(2, 2, tri1, tri2, q1, q2, q3, f1, f2, f3, p2, p3, e2, m1, ref pt1, ref pt2);
                TestNbPoints(2, ref nbPoints, ref nbPointsTotal, pt1, pt2, ref sp1, ref sp2);
            }

            if (nbPointsTotal < 3)
            {
                PolyhedronStartPoint pt1 = new(), pt2 = new();
                nbPoints = TriangleEdgeContact(2, 3, tri1, tri2, q1, q2, q3, f1, f2, f3, p3, p1, e3, m1, ref pt1, ref pt2);
                TestNbPoints(2, ref nbPoints, ref nbPointsTotal, pt1, pt2, ref sp1, ref sp2);
            }
        }

        if (nbPointsTotal == 2)
        {
            sp1.SetCoupleValue(t1, t2);
            sp2.SetCoupleValue(t1, t2);
        }
        else if (nbPointsTotal == 1)
        {
            sp1.SetCoupleValue(t1, t2);
        }
        else if (nbPointsTotal == 3)
        {
            sp1.SetCoupleValue(t1, t2);
        }

        return nbPointsTotal;
    }

    /// <summary>
    /// From two triangles and an intersection point, searches the other intersection point of the triangles (if it
    /// exists), skipping the edges the first point lies on. Used by <see cref="StartPointsChain"/>.
    /// </summary>
    public int NextStartingPointsResearch(int t1, int t2, in PolyhedronStartPoint spInit, ref PolyhedronStartPoint spNext)
    {
        int nbPointsTotal = 0;
        int edgeInit1 = spInit.E1();
        int edgeInit2 = spInit.E2();
        if ((t1 < 0) || (t2 < 0))
        {
            nbPointsTotal = 0;
        }
        else
        {
            ref readonly PolyhedronTriangle tri1 = ref _tTriangles1.Value(t1);
            ref readonly PolyhedronTriangle tri2 = ref _tTriangles2.Value(t2);

            ref readonly PolyhedronPoint p1 = ref _tPoints1.Value(tri1.FirstPoint());
            ref readonly PolyhedronPoint p2 = ref _tPoints1.Value(tri1.SecondPoint());
            ref readonly PolyhedronPoint p3 = ref _tPoints1.Value(tri1.ThirdPoint());
            ref readonly PolyhedronPoint q1 = ref _tPoints2.Value(tri2.FirstPoint());
            ref readonly PolyhedronPoint q2 = ref _tPoints2.Value(tri2.SecondPoint());
            ref readonly PolyhedronPoint q3 = ref _tPoints2.Value(tri2.ThirdPoint());

            // The first triangle is (p1,p2,p3).  The other is (q1,q2,q3).
            // The edges are (e1,e2,e3) and (f1,f2,f3).
            // The normals are n1 and m1

            PolyhedronPoint e1 = p2 - p1;
            PolyhedronPoint e2 = p3 - p2;
            PolyhedronPoint e3 = p1 - p3;

            PolyhedronPoint f1 = q2 - q1;
            PolyhedronPoint f2 = q3 - q2;
            PolyhedronPoint f3 = q1 - q3;

            PolyhedronPoint nn1 = new(), mm1 = new();
            nn1.Cross(e1, e2); // normal to the first triangle
            mm1.Cross(f1, f2); // normal to the second triangle

            double nn1modulus, mm1modulus;
            nn1modulus = Math.Sqrt(nn1.SquareModulus());
            mm1modulus = Math.Sqrt(mm1.SquareModulus());

            //-------------------------------------------------
            // calculation of intersections points between triangles
            //-------------------------------------------------

            int nbPoints = 0;
            PolyhedronStartPoint sp1 = new(), sp2 = new();

            // check T1 normal
            if (Math.Abs(nn1modulus) < MyConfusionPrecision)
            {
            }
            else
            {
                PolyhedronPoint n1 = nn1.Divide(nn1modulus);
                // T2 edges with T1
                if ((nbPointsTotal < 3) && (edgeInit2 != tri2.FirstEdge()))
                {
                    PolyhedronStartPoint pt1 = new(), pt2 = new();
                    nbPoints = TriangleEdgeContact(1, 1, tri1, tri2, p1, p2, p3, e1, e2, e3, q1, q2, f1, n1, ref pt1, ref pt2);
                    TestNbPoints(1, ref nbPoints, ref nbPointsTotal, pt1, pt2, ref sp1, ref sp2);
                }

                if ((nbPointsTotal < 3) && (edgeInit2 != tri2.SecondEdge()))
                {
                    PolyhedronStartPoint pt1 = new(), pt2 = new();
                    nbPoints = TriangleEdgeContact(1, 2, tri1, tri2, p1, p2, p3, e1, e2, e3, q2, q3, f2, n1, ref pt1, ref pt2);
                    TestNbPoints(1, ref nbPoints, ref nbPointsTotal, pt1, pt2, ref sp1, ref sp2);
                }

                if ((nbPointsTotal < 3) && (edgeInit2 != tri2.ThirdEdge()))
                {
                    PolyhedronStartPoint pt1 = new(), pt2 = new();
                    nbPoints = TriangleEdgeContact(1, 3, tri1, tri2, p1, p2, p3, e1, e2, e3, q3, q1, f3, n1, ref pt1, ref pt2);
                    TestNbPoints(1, ref nbPoints, ref nbPointsTotal, pt1, pt2, ref sp1, ref sp2);
                }
            }

            // check T2 normal
            if (Math.Abs(mm1modulus) < MyConfusionPrecision)
            {
            }
            else
            {
                PolyhedronPoint m1 = mm1.Divide(mm1modulus);
                // T1 edges with T2
                if ((nbPointsTotal < 3) && (edgeInit1 != tri1.FirstEdge()))
                {
                    PolyhedronStartPoint pt1 = new(), pt2 = new();
                    nbPoints = TriangleEdgeContact(2, 1, tri1, tri2, q1, q2, q3, f1, f2, f3, p1, p2, e1, m1, ref pt1, ref pt2);
                    TestNbPoints(2, ref nbPoints, ref nbPointsTotal, pt1, pt2, ref sp1, ref sp2);
                }

                if ((nbPointsTotal < 3) && (edgeInit1 != tri1.SecondEdge()))
                {
                    PolyhedronStartPoint pt1 = new(), pt2 = new();
                    nbPoints = TriangleEdgeContact(2, 2, tri1, tri2, q1, q2, q3, f1, f2, f3, p2, p3, e2, m1, ref pt1, ref pt2);
                    TestNbPoints(2, ref nbPoints, ref nbPointsTotal, pt1, pt2, ref sp1, ref sp2);
                }

                if ((nbPointsTotal < 3) && (edgeInit1 != tri1.ThirdEdge()))
                {
                    PolyhedronStartPoint pt1 = new(), pt2 = new();
                    nbPoints = TriangleEdgeContact(2, 3, tri1, tri2, q1, q2, q3, f1, f2, f3, p3, p1, e3, m1, ref pt1, ref pt2);
                    TestNbPoints(2, ref nbPoints, ref nbPointsTotal, pt1, pt2, ref sp1, ref sp2);
                }
            }

            if (nbPointsTotal == 1)
            {
                if (sp1.CheckSameSP(spInit) != 0)
                {
                    nbPointsTotal = 0;
                }
                else
                {
                    spNext = sp1;
                }
            }
            else if ((nbPointsTotal == 2) && (sp1.CheckSameSP(spInit) != 0))
            {
                nbPointsTotal = 1; // SP1 and SPInit are the same
                spNext = sp2;
            }
            else if ((nbPointsTotal == 2) && (sp2.CheckSameSP(spInit) != 0))
            {
                nbPointsTotal = 1; // SP2 and SPInit are the same
                spNext = sp1;
            }
            else if (nbPointsTotal > 1)
            {
            }
        }

        spNext.SetCoupleValue(t1, t2);
        return nbPointsTotal;
    }

    // The intersection points of a side of a triangle and an edge of the other, coplanar with it
    private static void CalculPtsInterTriEdgeCoplanaires(int triSurfID, in PolyhedronPoint normaleTri,
                                                         in PolyhedronTriangle tri1, in PolyhedronTriangle tri2,
                                                         in PolyhedronPoint pe1, in PolyhedronPoint pe2,
                                                         in PolyhedronPoint edge, int edgeIndex, in PolyhedronPoint pt1,
                                                         in PolyhedronPoint pt2, in PolyhedronPoint cote, int coteIndex,
                                                         ref PolyhedronStartPoint sp1, ref PolyhedronStartPoint sp2,
                                                         ref int nbPoints)
    {
        double de, dc;
        //
        Vector3D ve = new(edge.X(), edge.Y(), edge.Z());
        Vector3D vc = new(cote.X(), cote.Y(), cote.Z());
        //
        de = ve.SquareMagnitude();
        dc = vc.SquareMagnitude();
        //
        if (de > SquareMyConfusionPrecision)
        {
            ve = ve.Divided(de);
        }
        //
        if (dc > SquareMyConfusionPrecision)
        {
            vc = vc.Divided(dc);
        }
        //
        if (!ve.IsParallel(vc, MyConfusionPrecision))
        {
            // Edge and side are not parallel
            PolyhedronPoint per = new();
            per.Cross(normaleTri, cote);
            double p1p = per.Dot(pe1);
            double p2p = per.Dot(pe2);
            double p0p = per.Dot(pt1);
            // The edge are PT1 are projected on the perpendicular of the side in the plane of the triangle
            if ((((p1p >= p0p) && (p0p >= p2p)) || ((p1p <= p0p) && (p0p <= p2p)))
                && (Math.Abs(p1p - p2p) > MyConfusionPrecision))
            {
                double lambda = (p1p - p0p) / (p1p - p2p);
                PolyhedronPoint pie;
                if (Math.Abs(lambda) < MyConfusionPrecision)
                {
                    // lambda=0
                    pie = pe1;
                }
                else if (Math.Abs(lambda) > 1.0 - MyConfusionPrecision)
                {
                    // lambda=1
                    pie = pe2;
                }
                else
                {
                    pie = pe1 + edge * lambda;
                }

                double alpha = RealLast();
                if (cote.X() != 0)
                {
                    alpha = (pie.X() - pt1.X()) / cote.X();
                }
                else if (cote.Y() != 0)
                {
                    alpha = (pie.Y() - pt1.Y()) / cote.Y();
                }
                else if (cote.Z() != 0)
                {
                    alpha = (pie.Z() - pt1.Z()) / cote.Z();
                }

                if (alpha < -MyConfusionPrecision)
                {
                }
                else
                {
                    if (nbPoints == 0)
                    {
                        sp1.SetXYZ(pie.X(), pie.Y(), pie.Z());
                        if (triSurfID == 1)
                        {
                            if (Math.Abs(alpha) < MyConfusionPrecision)
                            {
                                // alpha=0
                                sp1.SetUV1(pt1.U(), pt1.V());
                                sp1.SetUV2(pie.U(), pie.V());
                                sp1.SetEdge1(-1);
                            }

                            if (Math.Abs(alpha) > 1.0 - MyConfusionPrecision)
                            {
                                // alpha=1
                                sp1.SetUV1(pt2.U(), pt2.V());
                                sp1.SetUV2(pie.U(), pie.V());
                                sp1.SetEdge1(-1);
                            }
                            else
                            {
                                sp1.SetUV1(pt1.U() + cote.U() * alpha, pt1.V() + cote.V() * alpha);
                                sp1.SetUV2(pie.U(), pie.V());
                                sp1.SetEdge1(tri1.GetEdgeNumber(coteIndex));
                                if (tri1.GetEdgeOrientation(coteIndex) > 0)
                                {
                                    sp1.SetLambda1(alpha);
                                }
                                else
                                {
                                    sp1.SetLambda1(1.0 - alpha);
                                }
                            }

                            nbPoints++;
                        }
                        else if (triSurfID == 2)
                        {
                            if (Math.Abs(alpha) < MyConfusionPrecision)
                            {
                                // alpha=0
                                sp1.SetUV2(pt1.U(), pt1.V());
                                sp1.SetUV1(pie.U(), pie.V());
                                sp1.SetEdge2(-1);
                            }

                            if (Math.Abs(alpha) > 1.0 - MyConfusionPrecision)
                            {
                                // alpha=1
                                sp1.SetUV2(pt2.U(), pt2.V());
                                sp1.SetUV1(pie.U(), pie.V());
                                sp1.SetEdge2(-1);
                            }
                            else
                            {
                                sp1.SetUV1(pie.U(), pie.V());
                                sp1.SetUV2(pt1.U() + cote.U() * alpha, pt1.V() + cote.V() * alpha);
                                sp1.SetEdge2(tri2.GetEdgeNumber(coteIndex));
                                if (tri2.GetEdgeOrientation(coteIndex) > 0)
                                {
                                    sp1.SetLambda2(alpha);
                                }
                                else
                                {
                                    sp1.SetLambda2(1.0 - alpha);
                                }
                            }

                            nbPoints++;
                        }
                    }
                    else if (nbPoints == 1)
                    {
                        sp2.SetXYZ(pie.X(), pie.Y(), pie.Z());
                        if (triSurfID == 1)
                        {
                            if (Math.Abs(alpha) < MyConfusionPrecision)
                            {
                                // alpha=0
                                sp2.SetUV1(pt1.U(), pt1.V());
                                sp2.SetUV2(pie.U(), pie.V());
                                sp2.SetEdge1(-1);
                            }

                            if (Math.Abs(alpha) > 1.0 - MyConfusionPrecision)
                            {
                                // alpha=1
                                sp2.SetUV1(pt2.U(), pt2.V());
                                sp2.SetUV2(pie.U(), pie.V());
                                sp2.SetEdge1(-1);
                            }
                            else
                            {
                                sp2.SetUV1(pt1.U() + cote.U() * alpha, pt1.V() + cote.V() * alpha);
                                sp2.SetUV2(pie.U(), pie.V());
                                sp2.SetEdge1(tri1.GetEdgeNumber(coteIndex));
                                if (tri1.GetEdgeOrientation(coteIndex) > 0)
                                {
                                    sp2.SetLambda1(alpha);
                                }
                                else
                                {
                                    sp2.SetLambda1(1.0 - alpha);
                                }
                            }

                            nbPoints++;
                        }
                        else if (triSurfID == 2)
                        {
                            if (Math.Abs(alpha) < MyConfusionPrecision)
                            {
                                // alpha=0
                                sp2.SetUV2(pt1.U(), pt1.V());
                                sp2.SetUV1(pie.U(), pie.V());
                                sp2.SetEdge2(-1);
                            }

                            if (Math.Abs(alpha) > 1.0 - MyConfusionPrecision)
                            {
                                // alpha=1
                                sp2.SetUV2(pt2.U(), pt2.V());
                                sp2.SetUV1(pie.U(), pie.V());
                                sp2.SetEdge2(-1);
                            }
                            else
                            {
                                sp2.SetUV1(pie.U(), pie.V());
                                sp2.SetUV2(pt1.U() + cote.U() * alpha, pt1.V() + cote.V() * alpha);
                                sp2.SetEdge2(tri2.GetEdgeNumber(coteIndex));
                                if (tri1.GetEdgeOrientation(coteIndex) > 0) // OCCT: Tri1, where SP1's branch reads Tri2
                                {
                                    sp2.SetLambda2(alpha);
                                }
                                else
                                {
                                    sp2.SetLambda2(1.0 - alpha);
                                }
                            }

                            nbPoints++;
                        }
                    }
                    else if ((nbPoints > 2) || (nbPoints < 0))
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
            double pe1p = cote.Dot(pe1);
            double pe2p = cote.Dot(pe2);
            double pt1p = cote.Dot(pt1);
            double pt2p = cote.Dot(pt2);
            double lambda1 = 0.0, lambda2 = 0.0, alpha1 = 0.0, alpha2 = 0.0;
            PolyhedronPoint pep1 = new(), ptp1 = new(), pep2 = new(), ptp2 = new();

            if (pe1p > pe2p)
            {
                if ((pt1p < pe1p) && (pe1p <= pt2p))
                {
                    lambda1 = 0.0;
                    pep1 = pe1;
                    alpha1 = ((pe1p - pt1p) / (pt2p - pt1p));
                    ptp1 = pt1 + cote * alpha1;
                    nbPoints = 1;
                    if (pt1p <= pe2p)
                    {
                        lambda2 = 1.0;
                        pep2 = pe2;
                        alpha2 = ((pe2p - pt1p) / (pt2p - pt1p));
                        ptp2 = pt1 + cote * alpha2;
                        nbPoints = 2;
                    }
                    else
                    {
                        lambda2 = ((pt1p - pe1p) / (pe2p - pe1p));
                        pep2 = pe1 + edge * lambda2;
                        alpha2 = 0.0;
                        ptp2 = pt1;
                        nbPoints = 2;
                    }
                }
                else if ((pt2p < pe1p) && (pe1p <= pt1p))
                {
                    lambda1 = 0.0;
                    pep1 = pe1;
                    alpha1 = ((pt1p - pe1p) / (pt1p - pt2p));
                    ptp1 = pt1 + cote * alpha1;
                    nbPoints = 1;
                    if (pt2p <= pe2p)
                    {
                        lambda2 = 1.0;
                        pep2 = pe2;
                        alpha2 = ((pe2p - pt1p) / (pt2p - pt1p));
                        ptp2 = pt1 + cote * alpha2;
                        nbPoints = 2;
                    }
                    else
                    {
                        lambda2 = ((pt2p - pe1p) / (pe2p - pe1p));
                        pep2 = pe1 + edge * lambda2;
                        alpha2 = 1.0;
                        ptp2 = pt2;
                        nbPoints = 2;
                    }
                }
            }

            if (pe1p < pe2p)
            {
                if ((pt1p < pe2p) && (pe2p <= pt2p))
                {
                    lambda1 = 1.0;
                    pep1 = pe2;
                    alpha1 = ((pe2p - pt1p) / (pt2p - pt1p));
                    ptp1 = pt1 + cote * alpha1;
                    nbPoints = 1;
                    if (pt1p <= pe1p)
                    {
                        lambda2 = 0.0;
                        pep2 = pe1;
                        alpha2 = ((pe1p - pt1p) / (pt2p - pt1p));
                        ptp2 = pt1 + cote * alpha2;
                        nbPoints = 2;
                    }
                    else
                    {
                        lambda2 = ((pt1p - pe1p) / (pe2p - pe1p));
                        pep2 = pe2 + edge * lambda2; // OCCT: PE2, where the other branches start from PE1
                        alpha2 = 0.0;
                        ptp2 = pt1;
                        nbPoints = 2;
                    }
                }
                else if ((pt2p < pe2p) && (pe2p <= pt1p))
                {
                    lambda1 = 1.0;
                    pep1 = pe2;
                    alpha1 = ((pt1p - pe2p) / (pt1p - pt2p));
                    ptp1 = pt1 + cote * alpha1;
                    nbPoints = 1;
                    if (pt2p <= pe1p)
                    {
                        lambda2 = 0.0;
                        pep2 = pe1;
                        alpha2 = ((pe1p - pt1p) / (pt2p - pt1p));
                        ptp2 = pt1 + cote * alpha2;
                        nbPoints = 2;
                    }
                    else
                    {
                        lambda2 = ((pt2p - pe1p) / (pe2p - pe1p));
                        pep2 = pe1 + edge * lambda2;
                        alpha2 = 1.0;
                        ptp2 = pt2;
                        nbPoints = 2;
                    }
                }
            }

            if (nbPoints != 0)
            {
                sp1.SetXYZ(pep1.X(), pep1.Y(), pep1.Z());
                if (triSurfID == 1)
                {
                    // the side belongs to Tri1
                    sp1.SetUV1(ptp1.U(), ptp1.V());
                    sp1.SetUV2(pep1.U(), pep1.V());
                    sp1.SetEdge1(tri1.GetEdgeNumber(coteIndex));

                    if (tri1.GetEdgeOrientation(coteIndex) > 0)
                    {
                        sp1.SetLambda1(alpha1);
                    }
                    else
                    {
                        sp1.SetLambda1(1.0 - alpha1);
                    }

                    if (tri2.GetEdgeOrientation(edgeIndex) > 0)
                    {
                        sp1.SetLambda2(lambda1);
                    }
                    else
                    {
                        sp1.SetLambda2(1.0 - lambda1);
                    }
                }

                if (triSurfID == 2)
                {
                    // the side belongs to Tri2
                    sp1.SetUV1(pep1.U(), ptp1.V()); // OCCT: U of PEP1 with V of PTP1, and below
                    sp1.SetUV2(ptp1.U(), pep1.V());
                    sp1.SetEdge2(tri2.GetEdgeNumber(coteIndex));

                    if (tri2.GetEdgeOrientation(coteIndex) > 0)
                    {
                        sp1.SetLambda1(alpha1);
                    }
                    else
                    {
                        sp1.SetLambda1(1.0 - alpha1);
                    }

                    if (tri1.GetEdgeOrientation(edgeIndex) > 0)
                    {
                        sp1.SetLambda2(lambda1);
                    }
                    else
                    {
                        sp1.SetLambda2(1.0 - lambda1);
                    }
                }

                // It is checked if PEP1!=PEP2
                if ((nbPoints == 2) && (Math.Abs(pep1.U() - pep2.U()) < MyConfusionPrecision)
                    && (Math.Abs(pep1.V() - pep2.V()) < MyConfusionPrecision))
                {
                    nbPoints = 1;
                }

                if (nbPoints == 2)
                {
                    sp2.SetXYZ(pep2.X(), pep2.Y(), pep2.Z());
                    if (triSurfID == 1)
                    {
                        sp2.SetUV1(ptp2.U(), ptp2.V());
                        sp2.SetUV2(pep2.U(), pep2.V());
                        sp2.SetEdge1(tri1.GetEdgeNumber(coteIndex));

                        // OCCT: alpha1 and lambda1, not alpha2 and lambda2, for the second point (and below)
                        if (tri1.GetEdgeOrientation(coteIndex) > 0)
                        {
                            sp2.SetLambda1(alpha1);
                        }
                        else
                        {
                            sp2.SetLambda1(1.0 - alpha1);
                        }

                        if (tri2.GetEdgeOrientation(edgeIndex) > 0)
                        {
                            sp2.SetLambda2(lambda1);
                        }
                        else
                        {
                            sp2.SetLambda2(1.0 - lambda1);
                        }
                    }

                    if (triSurfID == 2)
                    {
                        sp2.SetUV1(pep2.U(), ptp2.V());
                        sp2.SetUV2(ptp2.U(), pep2.V());
                        sp2.SetEdge2(tri2.GetEdgeNumber(coteIndex));

                        if (tri1.GetEdgeOrientation(coteIndex) > 0) // OCCT: Tri1 for the side of Tri2
                        {
                            sp2.SetLambda1(alpha1);
                        }
                        else
                        {
                            sp2.SetLambda1(1.0 - alpha1);
                        }

                        if (tri2.GetEdgeOrientation(edgeIndex) > 0) // OCCT: Tri2 for the edge of Tri1
                        {
                            sp2.SetLambda2(lambda1);
                        }
                        else
                        {
                            sp2.SetLambda2(1.0 - lambda1);
                        }
                    }
                }
            }
        }

        // Filter if the point is placed on top, the edge is set  to -1
        if (nbPoints > 0)
        {
            if (Math.Abs(sp1.Lambda1()) < MyConfusionPrecision)
            {
                sp1.SetEdge1(-1);
            }

            if (Math.Abs(sp1.Lambda1() - 1) < MyConfusionPrecision)
            {
                sp1.SetEdge1(-1);
            }

            if (Math.Abs(sp1.Lambda2()) < MyConfusionPrecision)
            {
                sp1.SetEdge2(-1);
            }

            if (Math.Abs(sp1.Lambda2() - 1) < MyConfusionPrecision)
            {
                sp1.SetEdge2(-1);
            }
        }

        if (nbPoints == 2)
        {
            if (Math.Abs(sp2.Lambda1()) < MyConfusionPrecision)
            {
                sp2.SetEdge1(-1);
            }

            if (Math.Abs(sp2.Lambda1() - 1) < MyConfusionPrecision)
            {
                sp2.SetEdge1(-1);
            }

            if (Math.Abs(sp2.Lambda2()) < MyConfusionPrecision)
            {
                sp2.SetEdge2(-1);
            }

            if (Math.Abs(sp2.Lambda2() - 1) < MyConfusionPrecision)
            {
                sp2.SetEdge2(-1);
            }
        }
    }

    /// <summary>
    /// The intersection points (0 to 2) of an edge (PE1, PE2, Edge = PE2 - PE1) of one triangle with the other triangle
    /// (PT1, PT2, PT3, its sides and unit normal). <paramref name="triSurfID"/> is the surface of the triangle (PT1, PT2,
    /// PT3); <paramref name="edgeIndex"/> the edge's index (1 to 3) in the other.
    /// </summary>
    public int TriangleEdgeContact(int triSurfID, int edgeIndex, in PolyhedronTriangle tri1, in PolyhedronTriangle tri2,
                                   in PolyhedronPoint PT1, in PolyhedronPoint pt2, in PolyhedronPoint pt3,
                                   in PolyhedronPoint cote12, in PolyhedronPoint cote23, in PolyhedronPoint cote31,
                                   in PolyhedronPoint PE1, in PolyhedronPoint PE2, in PolyhedronPoint edge,
                                   in PolyhedronPoint normaleT, ref PolyhedronStartPoint sp1, ref PolyhedronStartPoint sp2)
    {
        double lambda = 0.0, alpha = 0.0, beta = 0.0;

        // One of edges on which the point is located is known
        if (triSurfID == 1)
        {
            sp1.SetEdge2(tri2.GetEdgeNumber(edgeIndex));
            sp2.SetEdge2(tri2.GetEdgeNumber(edgeIndex));
        }
        else if (triSurfID == 2)
        {
            sp1.SetEdge1(tri1.GetEdgeNumber(edgeIndex));
            sp2.SetEdge1(tri1.GetEdgeNumber(edgeIndex));
        }

        // The edge is projected on the normal in the triangle if yes
        // --> a free intersection (point I)--> Start point is found
        int nbPoints = 0;
        if (normaleT.SquareModulus() == 0)
        {
        }
        else if ((cote12.SquareModulus() == 0) || (cote23.SquareModulus() == 0) || (cote31.SquareModulus() == 0))
        {
        }
        else if (edge.SquareModulus() == 0)
        {
        }
        else
        {
            double pe1 = normaleT.Dot(PE1);
            double pe2 = normaleT.Dot(PE2);
            double pt1 = normaleT.Dot(PT1);

            // PE1I = lambda.Edge

            if ((Math.Abs(pe1 - pt1) < MyConfusionPrecision) && (Math.Abs(pe2 - pt1) < MyConfusionPrecision))
            {
                // edge and triangle are coplanar (two contact points at maximum)

                // the tops of the triangle are projected on the perpendicular to the edge
                PolyhedronPoint perpEdge = new();
                perpEdge.Cross(normaleT, edge);
                double pp1 = perpEdge.Dot(PT1);
                double pp2 = perpEdge.Dot(pt2);
                double pp3 = perpEdge.Dot(pt3);
                double ppe1 = perpEdge.Dot(PE1);

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
                        CalculPtsInterTriEdgeCoplanaires(triSurfID, normaleT, tri1, tri2, PE1, PE2, edge, edgeIndex,
                                                         PT1, pt2, cote12, 1, ref sp1, ref sp2, ref nbPoints);

                        if ((nbPoints > 1) && (Math.Abs(sp2.U1() - sp1.U1()) < MyConfusionPrecision)
                            && (Math.Abs(sp1.V1() - sp2.V1()) < MyConfusionPrecision))
                        {
                            nbPoints = 1;
                        }

                        // second side
                        if (nbPoints < 2)
                        {
                            CalculPtsInterTriEdgeCoplanaires(triSurfID, normaleT, tri1, tri2, PE1, PE2, edge, edgeIndex,
                                                             pt3, PT1, cote31, 3, ref sp1, ref sp2, ref nbPoints);
                        }
                    }

                    if ((nbPoints > 1) && (Math.Abs(sp1.U1() - sp2.U1()) < MyConfusionPrecision)
                        && (Math.Abs(sp1.V2() - sp2.V1()) < MyConfusionPrecision)) // OCCT: V2 against V1
                    {
                        nbPoints = 1;
                    }

                    if (nbPoints >= 2)
                    {
                        return nbPoints;
                    }
                    else if ((((pp2 >= ppe1) && (pp1 <= ppe1) && (pp3 <= ppe1))
                              || ((pp2 <= ppe1) && (pp1 >= ppe1) && (pp3 >= ppe1)))
                             && (nbPoints < 2))
                    {
                        // there are two sides (common top PT2) that can cut the edge

                        // first side
                        CalculPtsInterTriEdgeCoplanaires(triSurfID, normaleT, tri1, tri2, PE1, PE2, edge, edgeIndex,
                                                         PT1, pt2, cote12, 1, ref sp1, ref sp2, ref nbPoints);

                        if ((nbPoints > 1) && (Math.Abs(sp2.U1() - sp1.U1()) < MyConfusionPrecision)
                            && (Math.Abs(sp1.V1() - sp2.V1()) < MyConfusionPrecision))
                        {
                            nbPoints = 1;
                        }

                        // second side
                        if (nbPoints < 2)
                        {
                            CalculPtsInterTriEdgeCoplanaires(triSurfID, normaleT, tri1, tri2, PE1, PE2, edge, edgeIndex,
                                                             pt2, pt3, cote23, 2, ref sp1, ref sp2, ref nbPoints);
                        }
                    }

                    if ((nbPoints > 1) && (Math.Abs(sp2.U1() - sp1.U1()) < MyConfusionPrecision)
                        && (Math.Abs(sp1.V1() - sp2.V1()) < MyConfusionPrecision))
                    {
                        nbPoints = 1;
                    }

                    if (nbPoints >= 2)
                    {
                        return nbPoints;
                    }
                    else if ((((pp3 >= ppe1) && (pp1 <= ppe1) && (pp2 <= ppe1))
                              || ((pp3 <= ppe1) && (pp1 >= ppe1) && (pp2 >= ppe1)))
                             && (nbPoints < 2))
                    {
                        // there are two sides (common top PT3) that can cut the edge

                        // first side
                        CalculPtsInterTriEdgeCoplanaires(triSurfID, normaleT, tri1, tri2, PE1, PE2, edge, edgeIndex,
                                                         pt3, PT1, cote31, 3, ref sp1, ref sp2, ref nbPoints);

                        if ((nbPoints > 1) && (Math.Abs(sp2.U1() - sp1.U1()) < MyConfusionPrecision)
                            && (Math.Abs(sp1.V1() - sp2.V1()) < MyConfusionPrecision))
                        {
                            nbPoints = 1;
                        }

                        // second side
                        if (nbPoints < 2)
                        {
                            CalculPtsInterTriEdgeCoplanaires(triSurfID, normaleT, tri1, tri2, PE1, PE2, edge, edgeIndex,
                                                             pt2, pt3, cote23, 2, ref sp1, ref sp2, ref nbPoints);
                        }
                    }

                    if ((nbPoints > 1) && (Math.Abs(sp2.U1() - sp1.U1()) < MyConfusionPrecision)
                        && (Math.Abs(sp2.V1() - sp1.V1()) < MyConfusionPrecision))
                    {
                        nbPoints = 1;
                    }

                    if (nbPoints >= 2)
                    {
                        return nbPoints;
                    }
                }
            }

            //------------------------------------------------------
            // NON COPLANAR edge and triangle (a contact point)
            //------------------------------------------------------
            else if (((pe1 >= pt1) && (pt1 >= pe2)) || ((pe1 <= pt1) && (pt1 <= pe2)))
            {
                lambda = (pe1 - pt1) / (pe1 - pe2);
                PolyhedronPoint pi = new();
                if (lambda < -MyConfusionPrecision)
                {
                }
                else if (Math.Abs(lambda) < MyConfusionPrecision)
                {
                    // lambda==0
                    pi = PE1;
                    if (triSurfID == 1)
                    {
                        sp1.SetEdge2(-1);
                    }
                    else
                    {
                        sp1.SetEdge1(-1);
                    }
                }
                else if (Math.Abs(lambda - 1.0) < MyConfusionPrecision)
                {
                    // lambda==1
                    pi = PE2;
                    if (triSurfID == 1)
                    {
                        sp1.SetEdge2(-1);
                    }
                    else
                    {
                        sp1.SetEdge1(-1);
                    }
                }
                else
                {
                    pi = PE1 + edge * lambda;
                    if (triSurfID == 1)
                    {
                        if (tri2.GetEdgeOrientation(edgeIndex) > 0)
                        {
                            sp1.SetLambda2(lambda);
                        }
                        else
                        {
                            sp1.SetLambda2(1.0 - lambda);
                        }
                    }

                    if (triSurfID == 2)
                    {
                        if (tri1.GetEdgeOrientation(edgeIndex) > 0)
                        {
                            sp1.SetLambda1(lambda);
                        }
                        else
                        {
                            sp1.SetLambda1(1.0 - lambda);
                        }
                    }
                }

                double cote23X = cote23.X();
                double d1 = 0.0;
                double d3, d4;

                // Combination Eq1 Eq2
                if (Math.Abs(cote23X) > MyConfusionPrecision)
                {
                    d1 = cote12.Y() - cote12.X() * cote23.Y() / cote23X;
                }

                if (Math.Abs(d1) > MyConfusionPrecision)
                {
                    alpha = (pi.Y() - PT1.Y() - (pi.X() - PT1.X()) * cote23.Y() / cote23X) / d1;

                    // It is checked if 1.0>=alpha>=0.0
                    if ((alpha < -MyConfusionPrecision) || (alpha > (1.0 + MyConfusionPrecision)))
                    {
                        return 0;
                    }
                    else
                    {
                        beta = (pi.X() - PT1.X() - alpha * cote12.X()) / cote23X;
                    }
                }
                // Combination Eq1 and Eq2 with Cote23.X()==0
                else if ((Math.Abs(cote12.X()) > MyConfusionPrecision) && (Math.Abs(cote23X) < MyConfusionPrecision))
                {
                    // There is Cote23.X()==0
                    alpha = (pi.X() - PT1.X()) / cote12.X();

                    if ((alpha < -MyConfusionPrecision) || (alpha > (1.0 + MyConfusionPrecision)))
                    {
                        return 0;
                    }
                    else if (Math.Abs(cote23.Y()) > MyConfusionPrecision)
                    {
                        beta = (pi.Y() - PT1.Y() - alpha * cote12.Y()) / cote23.Y();
                    }
                    else if (Math.Abs(cote23.Z()) > MyConfusionPrecision)
                    {
                        beta = (pi.Z() - PT1.Z() - alpha * cote12.Z()) / cote23.Z();
                    }
                }
                // Combination Eq1 and Eq3
                else if ((Math.Abs(cote23.X()) > MyConfusionPrecision)
                         && (Math.Abs(d3 = (cote12.Z() - cote12.X() * cote23.Z() / cote23.X())) > MyConfusionPrecision))
                {
                    alpha = (pi.Z() - PT1.Z() - (pi.X() - PT1.X()) * cote23.Z() / cote23.X()) / d3;

                    if ((alpha < -MyConfusionPrecision) || (alpha > (1.0 + MyConfusionPrecision)))
                    {
                        return 0;
                    }
                    else
                    {
                        beta = (pi.X() - PT1.X() - alpha * cote12.X()) / cote23.X();
                    }
                }
                // Combination Eq2 and Eq3
                else if ((Math.Abs(cote23.Y()) > MyConfusionPrecision)
                         && (Math.Abs(d4 = (cote12.Z() - cote12.Y() * cote23.Z() / cote23.Y())) > MyConfusionPrecision))
                {
                    alpha = (pi.Z() - PT1.Z() - (pi.Y() - PT1.Y()) * cote23.Z() / cote23.Y()) / d4;

                    if ((alpha < -MyConfusionPrecision) || (alpha > (1.0 + MyConfusionPrecision)))
                    {
                        return 0;
                    }
                    else
                    {
                        beta = (pi.Y() - PT1.Y() - alpha * cote12.Y()) / cote23.Y();
                    }
                }
                // Combination Eq2 and Eq3 with Cote23.Y()==0
                else if ((Math.Abs(cote12.Y()) > MyConfusionPrecision) && (Math.Abs(cote23.Y()) < MyConfusionPrecision))
                {
                    alpha = (pi.Y() - PT1.Y()) / cote12.Y();

                    if ((alpha < -MyConfusionPrecision) || (alpha > (1.0 + MyConfusionPrecision)))
                    {
                        return 0;
                    }
                    else if (Math.Abs(cote23.Z()) > MyConfusionPrecision)
                    {
                        beta = (pi.Z() - PT1.Z() - alpha * cote12.Z()) / cote23.Z();
                    }
                    else
                    {
                        // OCCT: printf("\nCote PT2PT3 nul1\n"); PT2.Dump(2004); PT3.Dump(3004); (a debug print, left out)
                    }
                }
                // Combination Eq1 and Eq3 with Cote23.Z()==0
                else if ((Math.Abs(cote12.Z()) > MyConfusionPrecision) && (Math.Abs(cote23.Z()) < MyConfusionPrecision))
                {
                    alpha = (pi.Z() - PT1.Z()) / cote12.Z();

                    if ((alpha < -MyConfusionPrecision) || (alpha > (1.0 + MyConfusionPrecision)))
                    {
                        return 0;
                    }
                    else if (Math.Abs(cote23.X()) > MyConfusionPrecision)
                    {
                        beta = (pi.X() - PT1.X() - alpha * cote12.X()) / cote23.X();
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
                    sp1.SetXYZ(pi.X(), pi.Y(), pi.Z());

                    if (triSurfID == 1)
                    {
                        sp1.SetUV2(pi.U(), pi.V());
                        sp1.SetUV1(PT1.U() + cote12.U() * alpha + cote23.U() * beta,
                                   PT1.V() + cote12.V() * alpha + cote23.V() * beta);
                        nbPoints++;
                        if (alpha < MyConfusionPrecision)
                        {
                            // alpha=0 --> beta==0
                            sp1.SetXYZ(PT1.X(), PT1.Y(), PT1.Z());
                            sp1.SetUV1(PT1.U(), PT1.V());
                            sp1.SetEdge1(-1);
                        }
                        else if ((beta < MyConfusionPrecision) && (Math.Abs(1 - alpha) < MyConfusionPrecision))
                        {
                            // beta==0 alpha==1
                            sp1.SetXYZ(pt2.X(), pt2.Y(), pt2.Z());
                            sp1.SetUV1(pt2.U(), pt2.V());
                            sp1.SetEdge1(-1);
                        }
                        else if ((Math.Abs(beta - 1) < MyConfusionPrecision) && (Math.Abs(1 - alpha) < MyConfusionPrecision))
                        {
                            // beta==1 alpha==1
                            sp1.SetXYZ(pt3.X(), pt3.Y(), pt3.Z());
                            sp1.SetUV1(pt3.U(), pt3.V());
                            sp1.SetEdge1(-1);
                        }
                        else if (beta < MyConfusionPrecision)
                        {
                            // beta==0
                            sp1.SetEdge1(tri1.GetEdgeNumber(1));
                            if (tri1.GetEdgeOrientation(1) > 0)
                            {
                                sp1.SetLambda1(alpha);
                            }
                            else
                            {
                                sp1.SetLambda1(1.0 - alpha);
                            }
                        }
                        else if (Math.Abs(beta - alpha) < MyConfusionPrecision)
                        {
                            // beta==alpha
                            sp1.SetEdge1(tri1.GetEdgeNumber(3));
                            if (tri1.GetEdgeOrientation(3) > 0)
                            {
                                sp1.SetLambda1(1.0 - alpha);
                            }
                            else
                            {
                                sp1.SetLambda1(alpha);
                            }
                        }
                        else if (Math.Abs(alpha - 1) < MyConfusionPrecision)
                        {
                            // alpha==1
                            sp1.SetEdge1(tri1.GetEdgeNumber(2));
                            if (tri1.GetEdgeOrientation(2) > 0)
                            {
                                sp1.SetLambda1(beta);
                            }
                            else
                            {
                                sp1.SetLambda1(1.0 - beta);
                            }
                        }
                    }
                    else if (triSurfID == 2)
                    {
                        sp1.SetUV1(pi.U(), pi.V());
                        sp1.SetUV2(PT1.U() + cote12.U() * alpha + cote23.U() * beta,
                                   PT1.V() + cote12.V() * alpha + cote23.V() * beta);
                        nbPoints++;
                        if (alpha < MyConfusionPrecision)
                        {
                            // alpha=0 --> beta==0
                            sp1.SetXYZ(PT1.X(), PT1.Y(), PT1.Z());
                            sp1.SetUV2(PT1.U(), PT1.V());
                            sp1.SetEdge2(-1);
                        }
                        else if ((beta < MyConfusionPrecision) && (Math.Abs(1 - alpha) < MyConfusionPrecision))
                        {
                            // beta==0 alpha==1
                            sp1.SetXYZ(pt2.X(), pt2.Y(), pt2.Z());
                            sp1.SetUV2(pt2.U(), pt2.V());
                            sp1.SetEdge2(-1);
                        }
                        else if ((Math.Abs(beta - 1) < MyConfusionPrecision) && (Math.Abs(1 - alpha) < MyConfusionPrecision))
                        {
                            // beta==1 alpha==1
                            sp1.SetXYZ(pt3.X(), pt3.Y(), pt3.Z());
                            sp1.SetUV2(pt3.U(), pt3.V());
                            sp1.SetEdge2(-1);
                        }
                        else if (beta < MyConfusionPrecision)
                        {
                            // beta==0
                            sp1.SetEdge2(tri2.GetEdgeNumber(1));
                            if (tri2.GetEdgeOrientation(1) > 0)
                            {
                                sp1.SetLambda2(alpha);
                            }
                            else
                            {
                                sp1.SetLambda2(1.0 - alpha);
                            }
                        }
                        else if (Math.Abs(beta - alpha) < MyConfusionPrecision)
                        {
                            // beta==alpha
                            sp1.SetEdge2(tri2.GetEdgeNumber(3));
                            if (tri2.GetEdgeOrientation(3) > 0)
                            {
                                sp1.SetLambda2(1.0 - alpha);
                            }
                            else
                            {
                                sp1.SetLambda2(alpha);
                            }
                        }
                        else if (Math.Abs(alpha - 1) < MyConfusionPrecision)
                        {
                            // alpha==1
                            // OCCT: lambda from alpha, where the first surface's branch uses beta
                            sp1.SetEdge2(tri2.GetEdgeNumber(2));
                            if (tri2.GetEdgeOrientation(2) > 0)
                            {
                                sp1.SetLambda2(alpha);
                            }
                            else
                            {
                                sp1.SetLambda2(1.0 - alpha);
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

        return nbPoints;
    }

    /// <summary>
    /// Finds the couples of interfering triangles (boxes through a BVH, then <see cref="TriContact"/>) with the cosine of
    /// their angle, appends them to the couples and marks their triangles; returns the number of couples.
    /// </summary>
    public int TriangleCompare()
    {
        // Find couples with interfering bounding boxes
        NCollection_IndexedDataMap<int, NCollection_List<int>> dmili = new();
        GetInterferingTriangles(_tTriangles1, _tPoints1, _tTriangles2, _tPoints2, dmili);
        if (dmili.IsEmpty())
        {
            return 0;
        }
        //
        double coupleAngle = -2.0;
        //
        // Intersection of the triangles
        int i, nb = dmili.Extent();
        for (i = 1; i <= nb; ++i)
        {
            int i_S1 = dmili.FindKey(i);
            ref PolyhedronTriangle triangle1 = ref _tTriangles1[i_S1];
            ref readonly PolyhedronPoint p1 = ref _tPoints1.Value(triangle1.FirstPoint());
            ref readonly PolyhedronPoint p2 = ref _tPoints1.Value(triangle1.SecondPoint());
            ref readonly PolyhedronPoint p3 = ref _tPoints1.Value(triangle1.ThirdPoint());
            //
            NCollection_List<int> li2 = dmili.FindFromIndex(i);
            for (NCollection_List<int>.Iterator itLI = new(li2); itLI.More(); itLI.Next())
            {
                int i_S2 = itLI.Value();
                ref PolyhedronTriangle triangle2 = ref _tTriangles2[i_S2];
                ref readonly PolyhedronPoint q1 = ref _tPoints2.Value(triangle2.FirstPoint());
                ref readonly PolyhedronPoint q2 = ref _tPoints2.Value(triangle2.SecondPoint());
                ref readonly PolyhedronPoint q3 = ref _tPoints2.Value(triangle2.ThirdPoint());
                //
                if (TriContact(p1, p2, p3, q1, q2, q3, ref coupleAngle) != 0)
                {
                    PolyhedronCouple couple = new(i_S1, i_S2, coupleAngle);
                    _tTrianglesContacts.Append(couple);
                    //
                    triangle1.SetIntersection(true);
                    triangle2.SetIntersection(true);
                }
            }
        }

        return _tTrianglesContacts.Extent();
    }

    // Marks the first not-analyzed couple (T1, T2) analyzed and gets its angle; false if there is none
    private static bool CheckCoupleAndGetAngle(int t1, int t2, ref double angle,
                                               NCollection_List<PolyhedronCouple> tTrianglesContacts)
    {
        for (NCollection_List<PolyhedronCouple>.Iterator it = new(tTrianglesContacts); it.More(); it.Next())
        {
            ref PolyhedronCouple testCouple = ref it.ChangeValue();
            if (!testCouple.IsAnalyzed)
            {
                if (testCouple.FirstValue() == t1 && testCouple.SecondValue() == t2)
                {
                    testCouple.SetAnalyzed(true);
                    angle = testCouple.Angle();
                    return true;
                }
            }
        }

        return false;
    }

    // Couple T1 T2 is looked for in the list. T11 and T22 are the two other triangles of the edge-edge contact: the
    // couples CT11 (T1, T22) and CT22 (T11, T2) are found too, to be marked if there is a start point
    private static bool CheckCoupleAndGetAngle2(int t1, int t2, int t11, int t22,
                                                NCollection_List<PolyhedronCouple>.Iterator itCT11,
                                                NCollection_List<PolyhedronCouple>.Iterator itCT22, ref double angle,
                                                NCollection_List<PolyhedronCouple> tTrianglesContacts)
    {
        bool test1, test2, test3;
        test1 = test2 = test3 = false;
        //
        for (NCollection_List<PolyhedronCouple>.Iterator it = new(tTrianglesContacts); it.More(); it.Next())
        {
            ref PolyhedronCouple testCouple = ref it.ChangeValue();
            if (testCouple.IsAnalyzed)
            {
                continue;
            }
            //
            if (testCouple.FirstValue() == t1)
            {
                if (testCouple.SecondValue() == t2)
                {
                    test1 = true;
                    testCouple.SetAnalyzed(true);
                    angle = testCouple.Angle();
                }
                else if (testCouple.SecondValue() == t22)
                {
                    test2 = true;
                    itCT11.Reset(tTrianglesContacts, it.Index); // theItCT11 = aIt
                    angle = testCouple.Angle();
                }
            }
            else if (testCouple.FirstValue() == t11)
            {
                if (testCouple.SecondValue() == t2)
                {
                    test3 = true;
                    itCT22.Reset(tTrianglesContacts, it.Index); // theItCT22 = aIt
                    angle = testCouple.Angle();
                }
            }
            //
            if (test1 && test2 && test3)
            {
                break;
            }
        }

        return test1;
    }

    // If the point is a vertex of a triangle (an edge -1), it is stored in the tangent zones unless it is there already,
    // and 0 is returned; else it is added to the section line (at its end, or its beginning if Prepend) and 1 returned
    private static int CheckNextStartPoint(PolyhedronSectionLine sectionLine,
                                           PolyhedronArray<PolyhedronStartPoint> tTangentZones,
                                           ref PolyhedronStartPoint sp, bool prepend = false)
    {
        int test = 1;
        if ((sp.E1() == -1) || (sp.E2() == -1))
        {
            // The tops of triangle are analyzed
            // It is checked if they are not in the array TTangentZones
            int finTTZ = tTangentZones.ItemCount;
            for (int uiui = 0; uiui < finTTZ; uiui++)
            {
                PolyhedronStartPoint testSP = tTangentZones[uiui];
                if ((Math.Abs(sp.U1() - testSP.U1()) < MyConfusionPrecision)
                    && (Math.Abs(sp.V1() - testSP.V1()) < MyConfusionPrecision))
                {
                    if ((Math.Abs(sp.U2() - testSP.U2()) < MyConfusionPrecision)
                        && (Math.Abs(sp.V2() - testSP.V2()) < MyConfusionPrecision))
                    {
                        test = 0; // SP is already in the list of  tops
                        uiui = finTTZ;
                    }
                }
            }

            if (test != 0)
            {
                // the top does not belong to the list of TangentZones
                sp.SetChainList(-1);
                tTangentZones[finTTZ] = sp;
                tTangentZones.IncrementNbItems();
                test = 0; // the examined point is a top
            }
        }
        else if (test != 0)
        {
            if (prepend)
            {
                sectionLine.Prepend(sp);
            }
            else
            {
                sectionLine[sectionLine.StartPointCount] = sp;
                sectionLine.IncrementNbStartPoints();
            }
        }

        // if the point is not a top Test=1
        // The chain is continued
        return test;
    }

    /// <summary>
    /// Loops on the couples of triangles: computes their start points and chains them into section lines, or puts them
    /// in the tangent zones where chaining is not possible.
    /// </summary>
    public int StartPointsChain(PolyhedronArray<PolyhedronSectionLine> tSectionLines,
                                PolyhedronArray<PolyhedronStartPoint> tTangentZones)
    {
        // Loop on the array of couples filled in the function COMPARE()
        for (NCollection_List<PolyhedronCouple>.Iterator it = new(_tTrianglesContacts); it.More(); it.Next())
        {
            // Check if the couple of triangles has not been already examined.
            if (!it.Value().IsAnalyzed)
            {
                int sectionLineIndex = tSectionLines.ItemCount;
                // fill last section line if still empty (eap)
                if (sectionLineIndex > 0 && tSectionLines[sectionLineIndex - 1].StartPointCount == 0)
                {
                    sectionLineIndex -= 1;
                }
                else
                {
                    tSectionLines.IncrementNbItems();
                }

                PolyhedronSectionLine mySectionLine = tSectionLines[sectionLineIndex];
                if (mySectionLine.GetN() == 0)
                {
                    // eap
                    mySectionLine.Init(10000); // Initialisation of array of StartPoint
                }

                int nbPoints = -1;
                int t1I, t2I;
                t1I = it.Value().FirstValue();
                t2I = it.Value().SecondValue();

                // Start points for the current couple are found
                PolyhedronStartPoint sp1 = new(), sp2 = new();
                nbPoints = StartingPointsResearch(t1I, t2I, ref sp1, ref sp2); // first calculation
                it.ChangeValue().SetAnalyzed(true);                           // the couple is marked
                double coupleAngle = it.Value().Angle();

                if (nbPoints == 1)
                {
                    // particular case top/triangle or edge/edge
                    // the start point is input in the array
                    sp1.SetChainList(sectionLineIndex);
                    sp1.SetAngle(coupleAngle);
                    // it is checked if the point is not atop of the triangle
                    if (CheckNextStartPoint(mySectionLine, tTangentZones, ref sp1) != 0)
                    {
                        PolyhedronStartPoint spNext1 = new();
                        int testSP1 = 0;

                        // chain of a side
                        PolyhedronStartPoint sp11 = new(); //=SP1;
                        if (sp1.E1() >= 0)
                        {
                            //&&(SP1.E2()!=-1) already tested if the point is not a top
                            int nextTriangle1 = 0;
                            if (_tEdges1[sp1.E1()].FirstTriangle() != t1I)
                            {
                                nextTriangle1 = _tEdges1[sp1.E1()].FirstTriangle();
                            }
                            else
                            {
                                nextTriangle1 = _tEdges1[sp1.E1()].SecondTriangle();
                            }

                            double angle = -2.0;
                            if (CheckCoupleAndGetAngle(nextTriangle1, t2I, ref angle, _tTrianglesContacts))
                            {
                                // it is checked if the couple exists and is marked
                                int nbPoints11 = 0;
                                nbPoints11 = NextStartingPointsResearch(nextTriangle1, t2I, sp1, ref sp11);
                                if (nbPoints11 == 1)
                                {
                                    sp11.SetChainList(sectionLineIndex);
                                    sp11.SetAngle(angle);

                                    if (CheckNextStartPoint(mySectionLine, tTangentZones, ref sp11) != 0)
                                    {
                                        int endChainList = 1;
                                        while (endChainList != 0)
                                        {
                                            testSP1 = GetNextChainStartPoint(sp11, ref spNext1, mySectionLine, tTangentZones);
                                            if (testSP1 == 1)
                                            {
                                                spNext1.SetChainList(sectionLineIndex);
                                                if (CheckNextStartPoint(mySectionLine, tTangentZones, ref spNext1) != 0)
                                                {
                                                    sp11 = spNext1;
                                                }
                                                else
                                                {
                                                    endChainList = 0;
                                                }
                                            }
                                            else
                                            {
                                                endChainList = 0; // There is no next point
                                            }
                                        }
                                    }
                                }
                                else
                                {
                                    if (nbPoints11 > 1)
                                    {
                                        // The point is input in the array TTangentZones
                                        tTangentZones[tTangentZones.ItemCount] = sp11; // default list number = -1
                                        tTangentZones.IncrementNbItems();
                                    }
                                }
                            }
                        }

                        // chain of the other side
                        PolyhedronStartPoint sp12 = new(); //=SP1;
                        if (sp1.E2() >= 0)
                        {
                            //&&(SP1.E1()!=-1) already tested
                            int nextTriangle2;
                            if (_tEdges2[sp1.E2()].FirstTriangle() != t2I)
                            {
                                nextTriangle2 = _tEdges2[sp1.E2()].FirstTriangle();
                            }
                            else
                            {
                                nextTriangle2 = _tEdges2[sp1.E2()].SecondTriangle();
                            }

                            double angle = -2.0;
                            if (CheckCoupleAndGetAngle(t1I, nextTriangle2, ref angle, _tTrianglesContacts))
                            {
                                int nbPoints12 = 0;
                                nbPoints12 = NextStartingPointsResearch(t1I, nextTriangle2, sp1, ref sp12);
                                if (nbPoints12 == 1)
                                {
                                    sp12.SetChainList(sectionLineIndex);
                                    sp12.SetAngle(angle);
                                    bool prepend = true; // eap

                                    if (CheckNextStartPoint(mySectionLine, tTangentZones, ref sp12, prepend) != 0)
                                    {
                                        int endChainList = 1;
                                        while (endChainList != 0)
                                        {
                                            testSP1 = GetNextChainStartPoint(sp12, ref spNext1, mySectionLine,
                                                                             tTangentZones, prepend); // eap
                                            if (testSP1 == 1)
                                            {
                                                spNext1.SetChainList(sectionLineIndex);
                                                if (CheckNextStartPoint(mySectionLine, tTangentZones, ref spNext1, prepend) != 0)
                                                {
                                                    sp12 = spNext1;
                                                }
                                                else
                                                {
                                                    endChainList = 0;
                                                }
                                            }
                                            else
                                            {
                                                endChainList = 0; // there is no next point
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if (nbPoints12 > 1)
                                        {
                                            // The points are input in the array TTangentZones
                                            tTangentZones[tTangentZones.ItemCount] = sp12; // default list number = -1
                                            tTangentZones.IncrementNbItems();
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                else if (nbPoints == 2)
                {
                    // the start points are input in the array
                    PolyhedronStartPoint spNext2 = new();
                    int testSP2 = 0;
                    int endChainList = 1;

                    sp1.SetChainList(sectionLineIndex);
                    sp1.SetAngle(coupleAngle);
                    if (CheckNextStartPoint(mySectionLine, tTangentZones, ref sp1) != 0)
                    {
                        // chain of a side
                        while (endChainList != 0)
                        {
                            testSP2 = GetNextChainStartPoint(sp1, ref spNext2, mySectionLine, tTangentZones);
                            if (testSP2 == 1)
                            {
                                spNext2.SetChainList(sectionLineIndex);
                                if (CheckNextStartPoint(mySectionLine, tTangentZones, ref spNext2) != 0)
                                {
                                    sp1 = spNext2;
                                }
                                else
                                {
                                    endChainList = 0;
                                }
                            }
                            else
                            {
                                endChainList = 0; // there is no next point
                            }
                        }
                    }

                    sp2.SetChainList(sectionLineIndex);
                    sp2.SetAngle(coupleAngle);
                    bool prepend = true; // eap

                    if (CheckNextStartPoint(mySectionLine, tTangentZones, ref sp2, prepend) != 0)
                    {
                        // chain of the other side
                        endChainList = 1;
                        while (endChainList != 0)
                        {
                            testSP2 = GetNextChainStartPoint(sp2, ref spNext2, mySectionLine, tTangentZones, prepend); // eap
                            if (testSP2 == 1)
                            {
                                spNext2.SetChainList(sectionLineIndex);
                                if (CheckNextStartPoint(mySectionLine, tTangentZones, ref spNext2, prepend) != 0)
                                {
                                    sp2 = spNext2;
                                }
                                else
                                {
                                    endChainList = 0;
                                }
                            }
                            else
                            {
                                endChainList = 0; // there is no next point
                            }
                        }
                    }
                }
                else if ((nbPoints > 2) && (nbPoints < 7))
                {
                    // More than two start points
                    // the start point is input in the table
                    sp1.SetChainList(sectionLineIndex);
                    CheckNextStartPoint(mySectionLine, tTangentZones, ref sp1);
                }
            }
        }

        return 1;
    }

    /// <summary>
    /// Tries to compute the next start point of a chain, through the triangle on the other side of the edge(s) the
    /// point lies on. Used by <see cref="StartPointsChain"/>, when it is known that there are 2 contact points.
    /// </summary>
    public int GetNextChainStartPoint(in PolyhedronStartPoint sp, ref PolyhedronStartPoint spNext,
                                      PolyhedronSectionLine mySectionLine,
                                      PolyhedronArray<PolyhedronStartPoint> tTangentZones, bool prepend = false)
    {
        int nbPoints = 0;
        if ((sp.E1() >= 0) && (sp.E2() == -2))
        {
            // case if the point is on edge of T1
            int nextTriangle1;
            if (_tEdges1[sp.E1()].FirstTriangle() != sp.T1())
            {
                nextTriangle1 = _tEdges1[sp.E1()].FirstTriangle();
            }
            else
            {
                nextTriangle1 = _tEdges1[sp.E1()].SecondTriangle();
            }

            // If is checked if two triangles intersect
            double angle = -2.0;
            if (CheckCoupleAndGetAngle(nextTriangle1, sp.T2(), ref angle, _tTrianglesContacts))
            {
                nbPoints = NextStartingPointsResearch(nextTriangle1, sp.T2(), sp, ref spNext);
                if (nbPoints != 1)
                {
                    if (nbPoints > 1)
                    {
                        CheckNextStartPoint(mySectionLine, tTangentZones, ref spNext, prepend);
                    }
                    else
                    {
                        nbPoints = 0;
                    }
                }
                else
                {
                    spNext.SetAngle(angle);
                }
            }
            else
            {
                nbPoints = 0; // this couple does not intersect
            }
        }
        else if ((sp.E1() == -2) && (sp.E2() >= 0))
        {
            // case if the point is on edge of T2
            int nextTriangle2;
            if (_tEdges2[sp.E2()].FirstTriangle() != sp.T2())
            {
                nextTriangle2 = _tEdges2[sp.E2()].FirstTriangle();
            }
            else
            {
                nextTriangle2 = _tEdges2[sp.E2()].SecondTriangle();
            }

            double angle = -2.0;
            if (CheckCoupleAndGetAngle(sp.T1(), nextTriangle2, ref angle, _tTrianglesContacts))
            {
                nbPoints = NextStartingPointsResearch(sp.T1(), nextTriangle2, sp, ref spNext);
                if (nbPoints != 1)
                {
                    if (nbPoints > 1)
                    {
                        CheckNextStartPoint(mySectionLine, tTangentZones, ref spNext, prepend);
                    }
                    else
                    {
                        nbPoints = 0;
                    }
                }
                else
                {
                    spNext.SetAngle(angle);
                }
            }
            else
            {
                nbPoints = 0;
            }
        }
        else if ((sp.E1() == -2) && (sp.E2() == -2))
        {
            // no edge is touched or cut
            nbPoints = 0;
        }
        else if ((sp.E1() >= 0) && (sp.E2() >= 0))
        {
            // the point is located on two edges
            int nextTriangle1;
            if (_tEdges1[sp.E1()].FirstTriangle() != sp.T1())
            {
                nextTriangle1 = _tEdges1[sp.E1()].FirstTriangle();
            }
            else
            {
                nextTriangle1 = _tEdges1[sp.E1()].SecondTriangle();
            }

            int nextTriangle2;
            if (_tEdges2[sp.E2()].FirstTriangle() != sp.T2())
            {
                nextTriangle2 = _tEdges2[sp.E2()].FirstTriangle();
            }
            else
            {
                nextTriangle2 = _tEdges2[sp.E2()].SecondTriangle();
            }

            double angle = -2.0;

            NCollection_List<PolyhedronCouple>.Iterator itCT11 = new(), itCT22 = new();
            if (CheckCoupleAndGetAngle2(nextTriangle1, nextTriangle2, sp.T1(), sp.T2(), itCT11, itCT22, ref angle,
                                        _tTrianglesContacts))
            {
                nbPoints = NextStartingPointsResearch(nextTriangle1, nextTriangle2, sp, ref spNext);
                if (nbPoints != 1)
                {
                    if (nbPoints > 1)
                    {
                        // The new point is checked
                        CheckNextStartPoint(mySectionLine, tTangentZones, ref spNext, prepend);
                    }

                    nbPoints = 0;
                }
                else
                {
                    // NbPoints==1
                    spNext.SetAngle(angle);
                    if (itCT11.More())
                    {
                        itCT11.ChangeValue().SetAnalyzed(true);
                    }

                    if (itCT22.More())
                    {
                        itCT22.ChangeValue().SetAnalyzed(true);
                    }
                }
            }
            else
            {
                nbPoints = 0;
            }
        }
        else if ((sp.E1() == -1) || (sp.E2() == -1))
        {
            // the points are tops of triangle
            // the point is stored in an intermediary array
        }

        return nbPoints;
    }

    //=================================================================================================
    // Accessors

    /// <summary>The points of one surface.</summary>
    public PolyhedronArray<PolyhedronPoint> GetArrayOfPoints(int surfID) => surfID == 1 ? _tPoints1 : _tPoints2;

    /// <summary>The edges of one surface.</summary>
    public PolyhedronArray<PolyhedronEdge> GetArrayOfEdges(int surfID) => surfID == 1 ? _tEdges1 : _tEdges2;

    /// <summary>The triangles of one surface.</summary>
    public PolyhedronArray<PolyhedronTriangle> GetArrayOfTriangles(int surfID) => surfID == 1 ? _tTriangles1 : _tTriangles2;

    // GetFinTE and GetFinTT are left out: OCCT declares them and never defines them.

    /// <summary>The box of one surface's points.</summary>
    public Box GetBox(int surfID) => surfID == 1 ? _myBox1 : _myBox2;

    /// <summary>The couples of interfering triangles.</summary>
    public NCollection_List<PolyhedronCouple> GetCouples() => _tTrianglesContacts;

    /// <summary>Whether the sampling zone is enlarged (B-spline and Bézier surfaces).</summary>
    public void SetEnlargeZone(bool enlargeZone) => _enlargeZone = enlargeZone;

    public bool GetEnlargeZone() => _enlargeZone;

    /// <summary>The minimum deflection of the triangles of one surface (FlecheMin).</summary>
    public double GetMinDeflection(int surfID) => (surfID == 1) ? _flecheMin1 : _flecheMin2;

    /// <summary>The maximum deflection of the triangles of one surface (FlecheMax).</summary>
    public double GetMaxDeflection(int surfID) => (surfID == 1) ? _flecheMax1 : _flecheMax2;

    //=================================================================================================
    // Degenerated isolines

    // The indices of the parameters (1 to aNbX) of the degenerated isolines at the bounds of the surface, 0 if none
    private static void DegeneratedIndex(NCollection_Array1<double> aXpars, int aNbX, SurfaceAdaptorBase aS,
                                         int aIsoDirection, out int aI1, out int aI2)
    {
        int i;
        bool bDegX1, bDegX2;
        double degX1, degX2, tol2, x;
        //
        aI1 = 0;
        aI2 = 0;
        tol2 = MyTolerance * MyTolerance;
        //
        if (aIsoDirection == 1)
        {
            // V=const
            bDegX1 = IsDegenerated(aS, 1, tol2, out degX1);
            bDegX2 = IsDegenerated(aS, 2, tol2, out degX2);
        }
        else if (aIsoDirection == 2)
        {
            // U=const
            bDegX1 = IsDegenerated(aS, 3, tol2, out degX1);
            bDegX2 = IsDegenerated(aS, 4, tol2, out degX2);
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
            x = aXpars[i];
            if (bDegX1)
            {
                if (Math.Abs(x - degX1) < MyTolerance)
                {
                    aI1 = i;
                }
            }

            if (bDegX2)
            {
                if (Math.Abs(x - degX2) < MyTolerance)
                {
                    aI2 = i;
                }
            }
        }
    }

    // Whether the bound isoline (1, 2: V first and last; 3, 4: U first and last) is degenerated: three points on it
    // within the tolerance. OCCT samples at i * dU and i * dV, not from the first parameter (occt-issues.md, unit 5E).
    private static bool IsDegenerated(SurfaceAdaptorBase aS, int aIndex, double aTol2, out double aDegX)
    {
        bool bRet;
        int i, nbP;
        double u, dU, u1, u2, v, dV, v1, v2, d2;
        Point3D p1, p2;
        //
        bRet = false;
        nbP = 3;
        aDegX = 99;
        //
        u1 = aS.FirstUParameter();
        u2 = aS.LastUParameter();
        v1 = aS.FirstVParameter();
        v2 = aS.LastVParameter();
        //
        if (aIndex < 3)
        {
            // V=const
            v = v1;
            if (aIndex == 2)
            {
                v = v2;
            }

            dU = (u2 - u1) / (nbP - 1);
            u = u1;
            p1 = aS.Value(u, v);
            for (i = 1; i < nbP; ++i)
            {
                u = i * dU; // OCCT: not aU1 + i * dU
                if (i == nbP - 1)
                {
                    u = u2;
                }

                p2 = aS.Value(u, v);
                d2 = p1.SquareDistance(p2);
                if (d2 > aTol2)
                {
                    return bRet;
                }

                p1 = p2;
            }

            aDegX = v;
            bRet = !bRet;
        }
        else
        {
            // U=const
            u = u1;
            if (aIndex == 4)
            {
                u = u2;
            }

            dV = (v2 - v1) / (nbP - 1);
            v = v1;
            p1 = aS.Value(u, v);
            for (i = 1; i < nbP; ++i)
            {
                v = i * dV; // OCCT: not aV1 + i * dV
                if (i == nbP - 1)
                {
                    v = v2;
                }

                p2 = aS.Value(u, v);
                d2 = p1.SquareDistance(p2);
                if (d2 > aTol2)
                {
                    return bRet;
                }

                p1 = p2;
            }

            bRet = !bRet;
            aDegX = u;
        }
        //
        return bRet;
    }
}
