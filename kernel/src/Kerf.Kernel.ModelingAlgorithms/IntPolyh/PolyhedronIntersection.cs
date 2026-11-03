// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Intersection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Intersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Occt;

namespace Kerf.Engine.Intersection.Polyhedra;

/// <summary>
/// Intersects two surfaces by intersecting their triangulations, with a given sampling or a 10 x 10 one. The
/// intersection is done by the constructors; check <see cref="IsDone"/>, then read the section lines (chains of
/// points) and the tangent zones (isolated points).
/// </summary>
/// <remarks>
/// When the standard intersection finds no interfering triangles, or few with nearly parallel normals, it runs four
/// more with the points shifted along the normals (forward and reversed on each surface) and keeps their couples
/// without duplicates. Serial: OCCT has no parallel loop here, and the result is the same every run (decision 593).
/// </remarks>
public sealed class PolyhedronIntersection
{
    // Inputs
    private readonly SurfaceAdaptorBase _surf1; // First surface
    private readonly SurfaceAdaptorBase _surf2; // Second surface
    private readonly int _nbSU1; // Number of samples in U direction for first surface
    private readonly int _nbSV1; // Number of samples in V direction for first surface
    private readonly int _nbSU2; // Number of samples in U direction for second surface
    private readonly int _nbSV2; // Number of samples in V direction for second surface
    // Results
    private bool _isDone; // State of the operation
    private readonly PolyhedronArray<PolyhedronSectionLine> _sectionLines = new(); // Section lines
    private readonly PolyhedronArray<PolyhedronStartPoint> _tangentZones = new();  // Tangent zones
    private bool _isParallel;

    /// <summary>Intersects two surfaces with 10 x 10 samples each.</summary>
    public PolyhedronIntersection(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2)
    {
        _surf1 = s1;
        _surf2 = s2;
        _nbSU1 = 10;
        _nbSV1 = 10;
        _nbSU2 = 10;
        _nbSV2 = 10;
        _isDone = false;
        _isParallel = false;
        _sectionLines.Init(1000);
        _tangentZones.Init(10000);
        Perform();
    }

    /// <summary>Intersects two surfaces with <paramref name="nbSU1"/> x <paramref name="nbSV1"/> samples on the
    /// first and <paramref name="nbSU2"/> x <paramref name="nbSV2"/> on the second.</summary>
    public PolyhedronIntersection(SurfaceAdaptorBase s1, int nbSU1, int nbSV1, SurfaceAdaptorBase s2,
                                 int nbSU2, int nbSV2)
    {
        _surf1 = s1;
        _surf2 = s2;
        _nbSU1 = nbSU1;
        _nbSV1 = nbSV1;
        _nbSU2 = nbSU2;
        _nbSV2 = nbSV2;
        _isDone = false;
        _isParallel = false;
        _sectionLines.Init(1000);
        _tangentZones.Init(10000);
        Perform();
    }

    /// <summary>Intersects two surfaces at the given samplings (parameters from 1).</summary>
    public PolyhedronIntersection(SurfaceAdaptorBase s1, NCollection_Array1<double> uPars1,
                                 NCollection_Array1<double> vPars1, SurfaceAdaptorBase s2,
                                 NCollection_Array1<double> uPars2, NCollection_Array1<double> vPars2)
    {
        _surf1 = s1;
        _surf2 = s2;
        _nbSU1 = uPars1.Length();
        _nbSV1 = vPars1.Length();
        _nbSU2 = uPars2.Length();
        _nbSV2 = vPars2.Length();
        _isDone = false;
        _isParallel = false;
        _sectionLines.Init(1000);
        _tangentZones.Init(10000);
        Perform(uPars1, vPars1, uPars2, vPars2);
    }

    /// <summary>The state of the operation.</summary>
    public bool IsDone => _isDone;

    /// <summary>True if the surfaces were found parallel (too many interfering triangles with parallel normals).</summary>
    public bool IsParallel => _isParallel;

    /// <summary>The number of section lines.</summary>
    public int SectionLineCount => _sectionLines.ItemCount;

    /// <summary>The number of points in the given line (from 1).</summary>
    public int NbPointsInLine(int indexLine) => _sectionLines.Value(indexLine - 1).StartPointCount;

    /// <summary>The number of tangent zones.</summary>
    public int TangentZoneCount => _tangentZones.ItemCount;

    /// <summary>The number of points in a tangent zone: always 1.</summary>
    public int NbPointsInTangentZone(int index)
    {
        _ = index;
        return 1;
    }

    /// <summary>The point of a section line (both from 1): its 3D point, its parameters on both surfaces and the cosine
    /// of the angle of the triangles there.</summary>
    public void GetLinePoint(int indexl, int indexp, out double x, out double y, out double z, out double u1,
                             out double v1, out double u2, out double v2, out double incidence)
    {
        PolyhedronSectionLine msl = _sectionLines.Value(indexl - 1);
        PolyhedronStartPoint sp = msl.Value(indexp - 1);
        x = sp.X();
        y = sp.Y();
        z = sp.Z();
        u1 = sp.U1();
        v1 = sp.V1();
        u2 = sp.U2();
        v2 = sp.V2();
        incidence = sp.GetAngle();
    }

    /// <summary>The point of a tangent zone (from 1; the point index is ignored).</summary>
    public void GetTangentZonePoint(int indexz, int indexp, out double x, out double y, out double z, out double u1,
                                    out double v1, out double u2, out double v2)
    {
        _ = indexp;
        ref readonly PolyhedronStartPoint sp = ref _tangentZones.Value(indexz - 1);
        x = sp.X();
        y = sp.Y();
        z = sp.Z();
        u1 = sp.U1();
        v1 = sp.V1();
        u2 = sp.U2();
        v2 = sp.V2();
    }

    // Computes the intersection by first making the sampling of the surfaces
    private void Perform()
    {
        // Prepare the sampling of the surfaces - UV parameters of the triangulation nodes
        NCollection_Array1<double> uPars1 = new(), vPars1 = new(), uPars2 = new(), vPars2 = new();
        PolyhedronTools.MakeSampling(_surf1, _nbSU1, _nbSV1, false, uPars1, vPars1);
        PolyhedronTools.MakeSampling(_surf2, _nbSU2, _nbSV2, false, uPars2, vPars2);

        // Perform intersection
        Perform(uPars1, vPars1, uPars2, vPars2);
    }

    // Computes the intersection on the given sampling
    private void Perform(NCollection_Array1<double> uPars1, NCollection_Array1<double> vPars1,
                         NCollection_Array1<double> uPars2, NCollection_Array1<double> vPars2)
    {
        _isDone = true;

        // Compute the deflection of the given sampling if it is not set
        double deflTol1 = PolyhedronTools.ComputeDeflection(_surf1, uPars1, vPars1);
        double deflTol2 = PolyhedronTools.ComputeDeflection(_surf2, uPars2, vPars2);

        // Perform standard intersection
        bool isStdDone = PerformStd(uPars1, vPars1, uPars2, vPars2, deflTol1, deflTol2,
                                    out PolyhedronMaillageAffinage? pMaillageStd, out int nbCouplesStd);

        if (!isStdDone)
        {
            // Intersection not done
            _isDone = false;
            return;
        }

        if (!IsAdvRequired(pMaillageStd))
        {
            // Default interference done well, use it
            pMaillageStd!.StartPointsChain(_sectionLines, _tangentZones);
        }
        else
        {
            // Default intersection is done, but too few interferences found.
            // Perform advanced intersection - perform intersection four times with different shifts.
            bool isAdvDone = PerformAdv(uPars1, vPars1, uPars2, vPars2, deflTol1, deflTol2,
                                        out PolyhedronMaillageAffinage? pMaillageFF,
                                        out PolyhedronMaillageAffinage? pMaillageFR,
                                        out PolyhedronMaillageAffinage? pMaillageRF,
                                        out PolyhedronMaillageAffinage? pMaillageRR, out int nbCouplesAdv);

            if (isAdvDone && nbCouplesAdv > 0)
            {
                // Advanced interference found
                pMaillageFF!.StartPointsChain(_sectionLines, _tangentZones);
                pMaillageFR!.StartPointsChain(_sectionLines, _tangentZones);
                pMaillageRF!.StartPointsChain(_sectionLines, _tangentZones);
                pMaillageRR!.StartPointsChain(_sectionLines, _tangentZones);
            }
            else
            {
                // Advanced intersection not done or no intersection is found -> use standard intersection
                if (nbCouplesStd > 0)
                {
                    pMaillageStd!.StartPointsChain(_sectionLines, _tangentZones);
                }
            }
        }
    }

    // Performs the default (standard) intersection of the triangles
    private bool PerformStd(NCollection_Array1<double> uPars1, NCollection_Array1<double> vPars1,
                            NCollection_Array1<double> uPars2, NCollection_Array1<double> vPars2,
                            double deflTol1, double deflTol2, out PolyhedronMaillageAffinage? maillageS,
                            out int nbCouples)
    {
        bool isDone = PerformMaillage(uPars1, vPars1, uPars2, vPars2, deflTol1, deflTol2,
                                      out maillageS);
        nbCouples = isDone ? maillageS!.GetCouples().Extent() : 0;
        return isDone;
    }

    // Performs the advanced intersection of the triangles - four intersections with different shifts of the sampling
    // points. As in C++'s &&, a failed intersection stops the next ones (their outputs stay null).
    private bool PerformAdv(NCollection_Array1<double> uPars1, NCollection_Array1<double> vPars1,
                            NCollection_Array1<double> uPars2, NCollection_Array1<double> vPars2,
                            double deflTol1, double deflTol2, out PolyhedronMaillageAffinage? maillageFF,
                            out PolyhedronMaillageAffinage? maillageFR, out PolyhedronMaillageAffinage? maillageRF,
                            out PolyhedronMaillageAffinage? maillageRR, out int nbCouples)
    {
        maillageFF = null;
        maillageRF = null;
        maillageRR = null;
        nbCouples = 0;

        // Compute the points on the surface and normal directions in these points
        PolyhedronArray<PolyhedronPointNormal> points1 = new(), points2 = new();
        PolyhedronTools.FillArrayOfPointNormal(_surf1, uPars1, vPars1, points1);
        PolyhedronTools.FillArrayOfPointNormal(_surf2, uPars2, vPars2, points2);

        // Perform intersection with the different shifts of the triangles
        bool isDone = PerformMaillage(uPars1, vPars1, uPars2, vPars2, // sampling
                                      deflTol1, deflTol2,                   // deflection tolerance
                                      points1, points2,                         // points and normals
                                      true, false,                                // shift
                                      out maillageFR)
                      && PerformMaillage(uPars1, vPars1, uPars2, vPars2, deflTol1, deflTol2,
                                         points1, points2, false, true, out maillageRF)
                      && PerformMaillage(uPars1, vPars1, uPars2, vPars2, deflTol1, deflTol2,
                                         points1, points2, true, true, out maillageFF)
                      && PerformMaillage(uPars1, vPars1, uPars2, vPars2, deflTol1, deflTol2,
                                         points1, points2, false, false, out maillageRR);

        if (isDone)
        {
            nbCouples = maillageFF!.GetCouples().Extent() + maillageFR!.GetCouples().Extent()
                           + maillageRF!.GetCouples().Extent() + maillageRR!.GetCouples().Extent();

            // Merge couples
            if (nbCouples > 0)
            {
                MergeCouples(maillageFF.GetCouples(), maillageFR.GetCouples(), maillageRF.GetCouples(),
                             maillageRR.GetCouples());
            }
        }

        return isDone;
    }

    // Computes the standard MaillageAffinage (without shift); if no interfering triangles are found, tries again on
    // the enlarged surfaces
    private bool PerformMaillage(NCollection_Array1<double> uPars1, NCollection_Array1<double> vPars1,
                                 NCollection_Array1<double> uPars2, NCollection_Array1<double> vPars2,
                                 double deflTol1, double deflTol2, out PolyhedronMaillageAffinage? maillage)
    {
        maillage = new PolyhedronMaillageAffinage(_surf1, uPars1.Length(), vPars1.Length(), _surf2,
                                                    uPars2.Length(), vPars2.Length(), 0);

        maillage.FillArrayOfPnt(1, uPars1, vPars1, deflTol1);
        maillage.FillArrayOfPnt(2, uPars2, vPars2, deflTol2);

        int finTTC = ComputeIntersection(maillage);

        // If no intersecting triangles are found, try enlarged surfaces
        if (finTTC == 0)
        {
            // Check if enlarge for the surfaces is possible
            PolyhedronTools.IsEnlargePossible(_surf1, out bool isEnlargeU1, out bool isEnlargeV1);
            PolyhedronTools.IsEnlargePossible(_surf2, out bool isEnlargeU2, out bool isEnlargeV2);

            if (isEnlargeU1 || isEnlargeV1 || isEnlargeU2 || isEnlargeV2)
            {
                // OCCT: the retry runs on the same object, whose triangles keep the first pass's flags and boxes
                maillage.SetEnlargeZone(true);
                // Make new points on the enlarged surface
                maillage.FillArrayOfPnt(1);
                maillage.FillArrayOfPnt(2);
                // Compute intersection
                ComputeIntersection(maillage);
                maillage.SetEnlargeZone(false);
            }
        }

        // if too many intersections, consider surfaces parallel
        return AnalyzeIntersection(maillage);
    }

    // Computes the MaillageAffinage with the points shifted along the normals
    private bool PerformMaillage(NCollection_Array1<double> uPars1, NCollection_Array1<double> vPars1,
                                 NCollection_Array1<double> uPars2, NCollection_Array1<double> vPars2,
                                 double deflTol1, double deflTol2,
                                 PolyhedronArray<PolyhedronPointNormal> points1,
                                 PolyhedronArray<PolyhedronPointNormal> points2, bool isFirstFwd,
                                 bool isSecondFwd, out PolyhedronMaillageAffinage? maillage)
    {
        maillage = new PolyhedronMaillageAffinage(_surf1, uPars1.Length(), vPars1.Length(), _surf2,
                                                    uPars2.Length(), vPars2.Length(), 0);

        maillage.FillArrayOfPnt(1, isFirstFwd, points1, uPars1, vPars1, deflTol1);
        maillage.FillArrayOfPnt(2, isSecondFwd, points2, uPars2, vPars2, deflTol2);

        ComputeIntersection(maillage);

        return AnalyzeIntersection(maillage);
    }

    // Leaves each couple found by several of the four shifted intersections in the first of the lists only
    private static void MergeCouples(NCollection_List<PolyhedronCouple> anArrayFF,
                                     NCollection_List<PolyhedronCouple> anArrayFR,
                                     NCollection_List<PolyhedronCouple> anArrayRF,
                                     NCollection_List<PolyhedronCouple> anArrayRR)
    {
        // Fence map to remove from the lists the duplicating elements.
        HashSet<PolyhedronCouple> fenceMap = new(PolyhedronCouple.Hasher.Instance);
        //
        NCollection_List<PolyhedronCouple>[] pLists = [anArrayFF, anArrayFR, anArrayRF, anArrayRR];
        for (int i = 0; i < 4; ++i)
        {
            NCollection_List<PolyhedronCouple>.Iterator it = new(pLists[i]);
            for (; it.More();)
            {
                if (!fenceMap.Add(it.Value()))
                {
                    pLists[i].Remove(it);
                    continue;
                }

                it.Next();
            }
        }
    }

    // Whether the advanced intersection is required: no interfering triangles (and the surfaces not parallel), or few
    // (10 or less) with one couple at less than about 5 degrees
    private bool IsAdvRequired(PolyhedronMaillageAffinage? maillage)
    {
        if (maillage is null)
        {
            return true;
        }

        // Interfering triangles
        NCollection_List<PolyhedronCouple> couples = maillage.GetCouples();
        // Number of interfering pairs
        int nbCouples = couples.Extent();
        // Flag to define whether advanced intersection is required or not
        bool isAdvReq = (nbCouples == 0) && !IsParallel;
        if (isAdvReq)
        {
            // No interfering triangles are found -> perform advanced intersection
            return isAdvReq;
        }

        if (nbCouples > 10)
        {
            // Enough interfering triangles are found -> no need to perform advanced intersection
            return isAdvReq;
        }

        const double eps = .996; //~ cos of 5 deg
        for (NCollection_List<PolyhedronCouple>.Iterator it = new(couples); it.More(); it.Next())
        {
            if (Math.Abs(it.Value().Angle()) > eps)
            {
                // The angle between interfering triangles is small -> perform advanced
                // intersection to make intersection more precise
                isAdvReq = true;
                break;
            }
        }

        return isAdvReq;
    }

    // Computes the intersection of the triangles
    private static int ComputeIntersection(PolyhedronMaillageAffinage? maillage)
    {
        if (maillage is null)
        {
            return 0;
        }

        // Compute common box and mark the points inside that box
        maillage.CommonBox();

        // Make triangles
        maillage.FillArrayOfTriangles(1);
        maillage.FillArrayOfTriangles(2);

        // Make edges
        maillage.FillArrayOfEdges(1);
        maillage.FillArrayOfEdges(2);

        // Deflection refinement
        maillage.TrianglesDeflectionsRefinementBSB();

        return maillage.TriangleCompare();
    }

    // Analyzes the intersection on the number of interfering triangles: more than 200 couples, as many nearly parallel
    // ones as triangles on a surface, means parallel surfaces (the couples are cleared)
    private bool AnalyzeIntersection(PolyhedronMaillageAffinage? maillage)
    {
        if (maillage is null)
        {
            return false;
        }

        NCollection_List<PolyhedronCouple> couples = maillage.GetCouples();
        int finTTC = couples.Extent();
        if (finTTC > 200)
        {
            const double eps = .996; //~ cos of 5deg.
            int npara = 0;
            for (NCollection_List<PolyhedronCouple>.Iterator it = new(couples); it.More(); it.Next())
            {
                double cosa = Math.Abs(it.Value().Angle());
                if (cosa > eps)
                {
                    ++npara;
                }
            }

            if (npara >= maillage.GetArrayOfTriangles(1).ItemCount
                || npara >= maillage.GetArrayOfTriangles(2).ItemCount)
            {
                couples.Clear();
                _isParallel = true;
                return true;
            }
        }

        return true;
    }
}
