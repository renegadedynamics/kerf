// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Intersection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Intersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
public sealed class IntPolyh_Intersection
{
    // Inputs
    private readonly Adaptor3d_Surface mySurf1; // First surface
    private readonly Adaptor3d_Surface mySurf2; // Second surface
    private readonly int myNbSU1; // Number of samples in U direction for first surface
    private readonly int myNbSV1; // Number of samples in V direction for first surface
    private readonly int myNbSU2; // Number of samples in U direction for second surface
    private readonly int myNbSV2; // Number of samples in V direction for second surface
    // Results
    private bool myIsDone; // State of the operation
    private readonly IntPolyh_Array<IntPolyh_SectionLine> mySectionLines = new(); // Section lines
    private readonly IntPolyh_Array<IntPolyh_StartPoint> myTangentZones = new();  // Tangent zones
    private bool myIsParallel;

    /// <summary>Intersects two surfaces with 10 x 10 samples each.</summary>
    public IntPolyh_Intersection(Adaptor3d_Surface theS1, Adaptor3d_Surface theS2)
    {
        mySurf1 = theS1;
        mySurf2 = theS2;
        myNbSU1 = 10;
        myNbSV1 = 10;
        myNbSU2 = 10;
        myNbSV2 = 10;
        myIsDone = false;
        myIsParallel = false;
        mySectionLines.Init(1000);
        myTangentZones.Init(10000);
        Perform();
    }

    /// <summary>Intersects two surfaces with <paramref name="theNbSU1"/> x <paramref name="theNbSV1"/> samples on the
    /// first and <paramref name="theNbSU2"/> x <paramref name="theNbSV2"/> on the second.</summary>
    public IntPolyh_Intersection(Adaptor3d_Surface theS1, int theNbSU1, int theNbSV1, Adaptor3d_Surface theS2,
                                 int theNbSU2, int theNbSV2)
    {
        mySurf1 = theS1;
        mySurf2 = theS2;
        myNbSU1 = theNbSU1;
        myNbSV1 = theNbSV1;
        myNbSU2 = theNbSU2;
        myNbSV2 = theNbSV2;
        myIsDone = false;
        myIsParallel = false;
        mySectionLines.Init(1000);
        myTangentZones.Init(10000);
        Perform();
    }

    /// <summary>Intersects two surfaces at the given samplings (parameters from 1).</summary>
    public IntPolyh_Intersection(Adaptor3d_Surface theS1, NCollection_Array1<double> theUPars1,
                                 NCollection_Array1<double> theVPars1, Adaptor3d_Surface theS2,
                                 NCollection_Array1<double> theUPars2, NCollection_Array1<double> theVPars2)
    {
        mySurf1 = theS1;
        mySurf2 = theS2;
        myNbSU1 = theUPars1.Length();
        myNbSV1 = theVPars1.Length();
        myNbSU2 = theUPars2.Length();
        myNbSV2 = theVPars2.Length();
        myIsDone = false;
        myIsParallel = false;
        mySectionLines.Init(1000);
        myTangentZones.Init(10000);
        Perform(theUPars1, theVPars1, theUPars2, theVPars2);
    }

    /// <summary>The state of the operation.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>True if the surfaces were found parallel (too many interfering triangles with parallel normals).</summary>
    public bool IsParallel() => myIsParallel;

    /// <summary>The number of section lines.</summary>
    public int NbSectionLines() => mySectionLines.NbItems();

    /// <summary>The number of points in the given line (from 1).</summary>
    public int NbPointsInLine(int IndexLine) => mySectionLines.Value(IndexLine - 1).NbStartPoints();

    /// <summary>The number of tangent zones.</summary>
    public int NbTangentZones() => myTangentZones.NbItems();

    /// <summary>The number of points in a tangent zone: always 1.</summary>
    public int NbPointsInTangentZone(int theIndex)
    {
        _ = theIndex;
        return 1;
    }

    /// <summary>The point of a section line (both from 1): its 3D point, its parameters on both surfaces and the cosine
    /// of the angle of the triangles there.</summary>
    public void GetLinePoint(int Indexl, int Indexp, out double x, out double y, out double z, out double u1,
                             out double v1, out double u2, out double v2, out double incidence)
    {
        IntPolyh_SectionLine msl = mySectionLines.Value(Indexl - 1);
        IntPolyh_StartPoint sp = msl.Value(Indexp - 1);
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
    public void GetTangentZonePoint(int Indexz, int Indexp, out double x, out double y, out double z, out double u1,
                                    out double v1, out double u2, out double v2)
    {
        _ = Indexp;
        ref readonly IntPolyh_StartPoint sp = ref myTangentZones.Value(Indexz - 1);
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
        NCollection_Array1<double> UPars1 = new(), VPars1 = new(), UPars2 = new(), VPars2 = new();
        IntPolyh_Tools.MakeSampling(mySurf1, myNbSU1, myNbSV1, false, UPars1, VPars1);
        IntPolyh_Tools.MakeSampling(mySurf2, myNbSU2, myNbSV2, false, UPars2, VPars2);

        // Perform intersection
        Perform(UPars1, VPars1, UPars2, VPars2);
    }

    // Computes the intersection on the given sampling
    private void Perform(NCollection_Array1<double> theUPars1, NCollection_Array1<double> theVPars1,
                         NCollection_Array1<double> theUPars2, NCollection_Array1<double> theVPars2)
    {
        myIsDone = true;

        // Compute the deflection of the given sampling if it is not set
        double aDeflTol1 = IntPolyh_Tools.ComputeDeflection(mySurf1, theUPars1, theVPars1);
        double aDeflTol2 = IntPolyh_Tools.ComputeDeflection(mySurf2, theUPars2, theVPars2);

        // Perform standard intersection
        bool isStdDone = PerformStd(theUPars1, theVPars1, theUPars2, theVPars2, aDeflTol1, aDeflTol2,
                                    out IntPolyh_MaillageAffinage? pMaillageStd, out int nbCouplesStd);

        if (!isStdDone)
        {
            // Intersection not done
            myIsDone = false;
            return;
        }

        if (!IsAdvRequired(pMaillageStd))
        {
            // Default interference done well, use it
            pMaillageStd!.StartPointsChain(mySectionLines, myTangentZones);
        }
        else
        {
            // Default intersection is done, but too few interferences found.
            // Perform advanced intersection - perform intersection four times with different shifts.
            bool isAdvDone = PerformAdv(theUPars1, theVPars1, theUPars2, theVPars2, aDeflTol1, aDeflTol2,
                                        out IntPolyh_MaillageAffinage? pMaillageFF,
                                        out IntPolyh_MaillageAffinage? pMaillageFR,
                                        out IntPolyh_MaillageAffinage? pMaillageRF,
                                        out IntPolyh_MaillageAffinage? pMaillageRR, out int nbCouplesAdv);

            if (isAdvDone && nbCouplesAdv > 0)
            {
                // Advanced interference found
                pMaillageFF!.StartPointsChain(mySectionLines, myTangentZones);
                pMaillageFR!.StartPointsChain(mySectionLines, myTangentZones);
                pMaillageRF!.StartPointsChain(mySectionLines, myTangentZones);
                pMaillageRR!.StartPointsChain(mySectionLines, myTangentZones);
            }
            else
            {
                // Advanced intersection not done or no intersection is found -> use standard intersection
                if (nbCouplesStd > 0)
                {
                    pMaillageStd!.StartPointsChain(mySectionLines, myTangentZones);
                }
            }
        }
    }

    // Performs the default (standard) intersection of the triangles
    private bool PerformStd(NCollection_Array1<double> theUPars1, NCollection_Array1<double> theVPars1,
                            NCollection_Array1<double> theUPars2, NCollection_Array1<double> theVPars2,
                            double theDeflTol1, double theDeflTol2, out IntPolyh_MaillageAffinage? theMaillageS,
                            out int theNbCouples)
    {
        bool isDone = PerformMaillage(theUPars1, theVPars1, theUPars2, theVPars2, theDeflTol1, theDeflTol2,
                                      out theMaillageS);
        theNbCouples = isDone ? theMaillageS!.GetCouples().Extent() : 0;
        return isDone;
    }

    // Performs the advanced intersection of the triangles - four intersections with different shifts of the sampling
    // points. As in C++'s &&, a failed intersection stops the next ones (their outputs stay null).
    private bool PerformAdv(NCollection_Array1<double> theUPars1, NCollection_Array1<double> theVPars1,
                            NCollection_Array1<double> theUPars2, NCollection_Array1<double> theVPars2,
                            double theDeflTol1, double theDeflTol2, out IntPolyh_MaillageAffinage? theMaillageFF,
                            out IntPolyh_MaillageAffinage? theMaillageFR, out IntPolyh_MaillageAffinage? theMaillageRF,
                            out IntPolyh_MaillageAffinage? theMaillageRR, out int theNbCouples)
    {
        theMaillageFF = null;
        theMaillageRF = null;
        theMaillageRR = null;
        theNbCouples = 0;

        // Compute the points on the surface and normal directions in these points
        IntPolyh_Array<IntPolyh_PointNormal> aPoints1 = new(), aPoints2 = new();
        IntPolyh_Tools.FillArrayOfPointNormal(mySurf1, theUPars1, theVPars1, aPoints1);
        IntPolyh_Tools.FillArrayOfPointNormal(mySurf2, theUPars2, theVPars2, aPoints2);

        // Perform intersection with the different shifts of the triangles
        bool isDone = PerformMaillage(theUPars1, theVPars1, theUPars2, theVPars2, // sampling
                                      theDeflTol1, theDeflTol2,                   // deflection tolerance
                                      aPoints1, aPoints2,                         // points and normals
                                      true, false,                                // shift
                                      out theMaillageFR)
                      && PerformMaillage(theUPars1, theVPars1, theUPars2, theVPars2, theDeflTol1, theDeflTol2,
                                         aPoints1, aPoints2, false, true, out theMaillageRF)
                      && PerformMaillage(theUPars1, theVPars1, theUPars2, theVPars2, theDeflTol1, theDeflTol2,
                                         aPoints1, aPoints2, true, true, out theMaillageFF)
                      && PerformMaillage(theUPars1, theVPars1, theUPars2, theVPars2, theDeflTol1, theDeflTol2,
                                         aPoints1, aPoints2, false, false, out theMaillageRR);

        if (isDone)
        {
            theNbCouples = theMaillageFF!.GetCouples().Extent() + theMaillageFR!.GetCouples().Extent()
                           + theMaillageRF!.GetCouples().Extent() + theMaillageRR!.GetCouples().Extent();

            // Merge couples
            if (theNbCouples > 0)
            {
                MergeCouples(theMaillageFF.GetCouples(), theMaillageFR.GetCouples(), theMaillageRF.GetCouples(),
                             theMaillageRR.GetCouples());
            }
        }

        return isDone;
    }

    // Computes the standard MaillageAffinage (without shift); if no interfering triangles are found, tries again on
    // the enlarged surfaces
    private bool PerformMaillage(NCollection_Array1<double> theUPars1, NCollection_Array1<double> theVPars1,
                                 NCollection_Array1<double> theUPars2, NCollection_Array1<double> theVPars2,
                                 double theDeflTol1, double theDeflTol2, out IntPolyh_MaillageAffinage? theMaillage)
    {
        theMaillage = new IntPolyh_MaillageAffinage(mySurf1, theUPars1.Length(), theVPars1.Length(), mySurf2,
                                                    theUPars2.Length(), theVPars2.Length(), 0);

        theMaillage.FillArrayOfPnt(1, theUPars1, theVPars1, theDeflTol1);
        theMaillage.FillArrayOfPnt(2, theUPars2, theVPars2, theDeflTol2);

        int FinTTC = ComputeIntersection(theMaillage);

        // If no intersecting triangles are found, try enlarged surfaces
        if (FinTTC == 0)
        {
            // Check if enlarge for the surfaces is possible
            IntPolyh_Tools.IsEnlargePossible(mySurf1, out bool isEnlargeU1, out bool isEnlargeV1);
            IntPolyh_Tools.IsEnlargePossible(mySurf2, out bool isEnlargeU2, out bool isEnlargeV2);

            if (isEnlargeU1 || isEnlargeV1 || isEnlargeU2 || isEnlargeV2)
            {
                // OCCT: the retry runs on the same object, whose triangles keep the first pass's flags and boxes
                theMaillage.SetEnlargeZone(true);
                // Make new points on the enlarged surface
                theMaillage.FillArrayOfPnt(1);
                theMaillage.FillArrayOfPnt(2);
                // Compute intersection
                ComputeIntersection(theMaillage);
                theMaillage.SetEnlargeZone(false);
            }
        }

        // if too many intersections, consider surfaces parallel
        return AnalyzeIntersection(theMaillage);
    }

    // Computes the MaillageAffinage with the points shifted along the normals
    private bool PerformMaillage(NCollection_Array1<double> theUPars1, NCollection_Array1<double> theVPars1,
                                 NCollection_Array1<double> theUPars2, NCollection_Array1<double> theVPars2,
                                 double theDeflTol1, double theDeflTol2,
                                 IntPolyh_Array<IntPolyh_PointNormal> thePoints1,
                                 IntPolyh_Array<IntPolyh_PointNormal> thePoints2, bool theIsFirstFwd,
                                 bool theIsSecondFwd, out IntPolyh_MaillageAffinage? theMaillage)
    {
        theMaillage = new IntPolyh_MaillageAffinage(mySurf1, theUPars1.Length(), theVPars1.Length(), mySurf2,
                                                    theUPars2.Length(), theVPars2.Length(), 0);

        theMaillage.FillArrayOfPnt(1, theIsFirstFwd, thePoints1, theUPars1, theVPars1, theDeflTol1);
        theMaillage.FillArrayOfPnt(2, theIsSecondFwd, thePoints2, theUPars2, theVPars2, theDeflTol2);

        ComputeIntersection(theMaillage);

        return AnalyzeIntersection(theMaillage);
    }

    // Leaves each couple found by several of the four shifted intersections in the first of the lists only
    private static void MergeCouples(NCollection_List<IntPolyh_Couple> anArrayFF,
                                     NCollection_List<IntPolyh_Couple> anArrayFR,
                                     NCollection_List<IntPolyh_Couple> anArrayRF,
                                     NCollection_List<IntPolyh_Couple> anArrayRR)
    {
        // Fence map to remove from the lists the duplicating elements.
        NCollection_Map<IntPolyh_Couple> aFenceMap = new(IntPolyh_Couple.Hasher.Instance);
        //
        NCollection_List<IntPolyh_Couple>[] pLists = [anArrayFF, anArrayFR, anArrayRF, anArrayRR];
        for (int i = 0; i < 4; ++i)
        {
            NCollection_List<IntPolyh_Couple>.Iterator aIt = new(pLists[i]);
            for (; aIt.More();)
            {
                if (!aFenceMap.Add(aIt.Value()))
                {
                    pLists[i].Remove(aIt);
                    continue;
                }

                aIt.Next();
            }
        }
    }

    // Whether the advanced intersection is required: no interfering triangles (and the surfaces not parallel), or few
    // (10 or less) with one couple at less than about 5 degrees
    private bool IsAdvRequired(IntPolyh_MaillageAffinage? theMaillage)
    {
        if (theMaillage is null)
        {
            return true;
        }

        // Interfering triangles
        NCollection_List<IntPolyh_Couple> Couples = theMaillage.GetCouples();
        // Number of interfering pairs
        int aNbCouples = Couples.Extent();
        // Flag to define whether advanced intersection is required or not
        bool isAdvReq = (aNbCouples == 0) && !IsParallel();
        if (isAdvReq)
        {
            // No interfering triangles are found -> perform advanced intersection
            return isAdvReq;
        }

        if (aNbCouples > 10)
        {
            // Enough interfering triangles are found -> no need to perform advanced intersection
            return isAdvReq;
        }

        const double anEps = .996; //~ cos of 5 deg
        for (NCollection_List<IntPolyh_Couple>.Iterator aIt = new(Couples); aIt.More(); aIt.Next())
        {
            if (Math.Abs(aIt.Value().Angle()) > anEps)
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
    private static int ComputeIntersection(IntPolyh_MaillageAffinage? theMaillage)
    {
        if (theMaillage is null)
        {
            return 0;
        }

        // Compute common box and mark the points inside that box
        theMaillage.CommonBox();

        // Make triangles
        theMaillage.FillArrayOfTriangles(1);
        theMaillage.FillArrayOfTriangles(2);

        // Make edges
        theMaillage.FillArrayOfEdges(1);
        theMaillage.FillArrayOfEdges(2);

        // Deflection refinement
        theMaillage.TrianglesDeflectionsRefinementBSB();

        return theMaillage.TriangleCompare();
    }

    // Analyzes the intersection on the number of interfering triangles: more than 200 couples, as many nearly parallel
    // ones as triangles on a surface, means parallel surfaces (the couples are cleared)
    private bool AnalyzeIntersection(IntPolyh_MaillageAffinage? theMaillage)
    {
        if (theMaillage is null)
        {
            return false;
        }

        NCollection_List<IntPolyh_Couple> Couples = theMaillage.GetCouples();
        int FinTTC = Couples.Extent();
        if (FinTTC > 200)
        {
            const double eps = .996; //~ cos of 5deg.
            int npara = 0;
            for (NCollection_List<IntPolyh_Couple>.Iterator aIt = new(Couples); aIt.More(); aIt.Next())
            {
                double cosa = Math.Abs(aIt.Value().Angle());
                if (cosa > eps)
                {
                    ++npara;
                }
            }

            if (npara >= theMaillage.GetArrayOfTriangles(1).NbItems()
                || npara >= theMaillage.GetArrayOfTriangles(2).NbItems())
            {
                Couples.Clear();
                myIsParallel = true;
                return true;
            }
        }

        return true;
    }
}
