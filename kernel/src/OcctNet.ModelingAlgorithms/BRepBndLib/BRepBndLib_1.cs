// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBndLib/BRepBndLib_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;
using static Occt.gp_TrsfForm;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

public static partial class BRepBndLib
{
    // Returns TRUE if theC is line-like.
    private static bool IsLinear(Adaptor3d_Curve theC)
    {
        GeomAbs_CurveType aCT = theC.GetType();
        if (aCT == GeomAbs_OffsetCurve)
        {
            return IsLinear(new GeomAdaptor_Curve(theC.OffsetCurve().BasisCurve()));
        }

        if ((aCT == GeomAbs_BSplineCurve) || (aCT == GeomAbs_BezierCurve))
        {
            // Indeed, curves with C0-continuity and degree==1, may be
            // represented with set of points. It will be possible made
            // in the future.

            return ((theC.Degree() == 1) && (theC.Continuity() != GeomAbs_C0));
        }

        if (aCT == GeomAbs_Line)
        {
            return true;
        }

        return false;
    }

    // Returns TRUE if theS is plane-like.
    private static bool IsPlanar(Adaptor3d_Surface theS)
    {
        GeomAbs_SurfaceType aST = theS.GetType();
        if (aST == GeomAbs_OffsetSurface)
        {
            return IsPlanar(theS.BasisSurface());
        }

        if (aST == GeomAbs_SurfaceOfExtrusion)
        {
            return IsLinear(theS.BasisCurve());
        }

        if ((aST == GeomAbs_BSplineSurface) || (aST == GeomAbs_BezierSurface))
        {
            if ((theS.UDegree() != 1) || (theS.VDegree() != 1))
            {
                return false;
            }

            // Indeed, surfaces with C0-continuity and degree==1, may be
            // represented with set of points. It will be possible made
            // in the future.

            return ((theS.UContinuity() != GeomAbs_C0) && (theS.VContinuity() != GeomAbs_C0));
        }

        if (aST == GeomAbs_Plane)
        {
            return true;
        }

        return false;
    }

    // Returns number of points for array.
    //
    // Attention!!!
    //  1. Start index for thePts must be 0 strictly.
    //  2. Currently, infinite edges/faces (e.g. half-space) are not
    //      processed correctly because computation of UV-bounds is a costly operation.
    private static int PointsForOBB(in TopoDS_Shape theS,
                                    bool theIsTriangulationUsed,
                                    NCollection_Array1<gp_Pnt>? thePts = null,
                                    NCollection_Array1<double>? theArrOfToler = null)
    {
        int aRetVal = 0;
        var anExpF = new TopExp_Explorer();
        var anExpE = new TopExp_Explorer();

        // get all vertices from the shape
        for (anExpF.Init(theS, TopAbs_VERTEX); anExpF.More(); anExpF.Next())
        {
            TopoDS_Vertex aVert = TopoDS.Vertex(anExpF.Current());
            if (thePts is not null)
            {
                gp_Pnt aP = BRep_Tool.Pnt(aVert);
                thePts[aRetVal] = aP;
            }

            if (theArrOfToler is not null)
            {
                theArrOfToler[aRetVal] = BRep_Tool.Tolerance(aVert);
            }

            ++aRetVal;
        }

        if (aRetVal == 0)
        {
            return 0;
        }

        // analyze the faces of the shape on planarity and existence of triangulation
        TopLoc_Location aLoc;
        for (anExpF.Init(theS, TopAbs_FACE); anExpF.More(); anExpF.Next())
        {
            TopoDS_Face aF = TopoDS.Face(anExpF.Current());
            var anAS = new BRepAdaptor_Surface(aF, false);

            if (!IsPlanar(anAS))
            {
                if (!theIsTriangulationUsed)
                {
                    // not planar and triangulation usage disabled
                    return 0;
                }
            }
            else
            {
                // planar face
                for (anExpE.Init(aF, TopAbs_EDGE); anExpE.More(); anExpE.Next())
                {
                    TopoDS_Edge anE = TopoDS.Edge(anExpE.Current());
                    if (BRep_Tool.IsGeometric(anE))
                    {
                        var anAC = new BRepAdaptor_Curve(anE);
                        if (!IsLinear(anAC))
                        {
                            if (!theIsTriangulationUsed)
                            {
                                // not linear and triangulation usage disabled
                                return 0;
                            }

                            break;
                        }
                    }
                }

                if (!anExpE.More())
                {
                    // skip planar face with linear edges as its vertices have already been added
                    continue;
                }
            }

            // Use triangulation of the face
            Poly_Triangulation? aTrng = BRep_Tool.Triangulation(aF, out aLoc);
            if (aTrng is null)
            {
                // no triangulation on the face
                return 0;
            }

            int aCNode = aTrng.NbNodes();
            gp_Trsf aTrsf = aLoc.Transformation();
            for (int i = 1; i <= aCNode; i++)
            {
                if (thePts is not null)
                {
                    gp_Pnt aP = aTrsf.Form() == gp_Identity ? aTrng.Node(i) : aTrng.Node(i).Transformed(aTrsf);
                    thePts[aRetVal] = aP;
                }

                if (theArrOfToler is not null)
                {
                    theArrOfToler[aRetVal] = aTrng.Deflection();
                }

                ++aRetVal;
            }
        }

        // Consider edges without faces

        for (anExpE.Init(theS, TopAbs_EDGE, TopAbs_FACE); anExpE.More(); anExpE.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anExpE.Current());
            if (BRep_Tool.IsGeometric(anE))
            {
                var anAC = new BRepAdaptor_Curve(anE);
                if (IsLinear(anAC))
                {
                    // skip linear edge as its vertices have already been added
                    continue;
                }
            }

            if (!theIsTriangulationUsed)
            {
                // not linear and triangulation usage disabled
                return 0;
            }

            Poly_Polygon3D? aPolygon = BRep_Tool.Polygon3D(anE, out aLoc);
            if (aPolygon is null)
            {
                return 0;
            }

            int aCNode = aPolygon.NbNodes();
            NCollection_Array1<gp_Pnt> aNodesArr = aPolygon.Nodes();
            for (int i = 1; i <= aCNode; i++)
            {
                if (thePts is not null)
                {
                    gp_Pnt aP = aLoc.IsIdentity() ? aNodesArr[i] : aNodesArr[i].Transformed(aLoc.Transformation());
                    thePts[aRetVal] = aP;
                }

                if (theArrOfToler is not null)
                {
                    theArrOfToler[aRetVal] = aPolygon.Deflection();
                }

                ++aRetVal;
            }
        }

        return aRetVal;
    }

    // Returns 0 if the theDir does not match any axis of WCS.
    // Otherwise, returns the index of correspond axis.
    private static int IsWCS(in gp_Dir theDir)
    {
        double aToler = Precision.Angular() * Precision.Angular();

        double aX = theDir.X(), aY = theDir.Y(), aZ = theDir.Z();

        double aVx = aY * aY + aZ * aZ, aVy = aX * aX + aZ * aZ, aVz = aX * aX + aY * aY;

        if (aVz < aToler)
        {
            return 3; // Z-axis
        }

        if (aVy < aToler)
        {
            return 2; // Y-axis
        }

        if (aVx < aToler)
        {
            return 1; // X-axis
        }

        return 0;
    }

    // Collects points for DiTO algorithm for OBB construction on
    // linear/planar shapes and shapes having triangulation
    // (http://www.idt.mdh.se/~tla/publ/FastOBBs.pdf).
    private static bool CheckPoints(in TopoDS_Shape theS,
                                    bool theIsTriangulationUsed,
                                    bool theIsOptimal,
                                    bool theIsShapeToleranceUsed,
                                    ref Bnd_OBB theOBB)
    {
        int aNbPnts = PointsForOBB(theS, theIsTriangulationUsed);

        if (aNbPnts < 1)
        {
            return false;
        }

        var anArrPnts = new NCollection_Array1<gp_Pnt>(0, theOBB.IsVoid() ? aNbPnts - 1 : aNbPnts + 7);
        NCollection_Array1<double> anArrOfTolerances = new();
        if (theIsShapeToleranceUsed)
        {
            anArrOfTolerances.Resize(anArrPnts.Lower(), anArrPnts.Upper(), false);
            anArrOfTolerances.Init(0.0);
        }

        NCollection_Array1<double>? aPtrArrTol = theIsShapeToleranceUsed ? anArrOfTolerances : null;

        PointsForOBB(theS, theIsTriangulationUsed, anArrPnts, aPtrArrTol);

        if (!theOBB.IsVoid())
        {
            // All points of old OBB have zero-tolerance
            theOBB.GetVertex(anArrPnts.AsSpan(aNbPnts));
        }

        theOBB.ReBuild(anArrPnts, aPtrArrTol, theIsOptimal);

        return (!theOBB.IsVoid());
    }

    private static void ComputeProperties(in TopoDS_Shape theS, GProp_GProps theGCommon)
    {
        var anExp = new TopExp_Explorer();
        for (anExp.Init(theS, TopAbs_SOLID); anExp.More(); anExp.Next())
        {
            var aG = new GProp_GProps();
            BRepGProp.VolumeProperties(anExp.Current(), aG, true);
            theGCommon.Add(aG);
        }

        for (anExp.Init(theS, TopAbs_FACE, TopAbs_SOLID); anExp.More(); anExp.Next())
        {
            var aG = new GProp_GProps();
            BRepGProp.SurfaceProperties(anExp.Current(), aG, true);
            theGCommon.Add(aG);
        }

        for (anExp.Init(theS, TopAbs_EDGE, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            var aG = new GProp_GProps();
            BRepGProp.LinearProperties(anExp.Current(), aG, true);
            theGCommon.Add(aG);
        }

        for (anExp.Init(theS, TopAbs_VERTEX, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            var aG = new GProp_GProps(BRep_Tool.Pnt(TopoDS.Vertex(anExp.Current())));
            theGCommon.Add(aG);
        }
    }

    // Creates OBB with axes of inertia.
    private static void ComputePCA(in TopoDS_Shape theS,
                                   ref Bnd_OBB theOBB,
                                   bool theIsTriangulationUsed,
                                   bool theIsOptimal,
                                   bool theIsShapeToleranceUsed)
    {
        // Compute the transformation matrix to obtain more tight bounding box
        var aGCommon = new GProp_GProps();
        ComputeProperties(theS, aGCommon);

        // Transform the shape to the local coordinate system
        gp_Trsf aTrsf = new();

        // OCCT converts the axes of inertia (gp_Vec) to gp_Dir implicitly
        int anIdx1 = IsWCS(new gp_Dir(aGCommon.PrincipalProperties().FirstAxisOfInertia()));
        int anIdx2 = IsWCS(new gp_Dir(aGCommon.PrincipalProperties().SecondAxisOfInertia()));

        if ((anIdx1 == 0) || (anIdx2 == 0))
        {
            // Coordinate system in which the shape will have the optimal bounding box
            var aLocCoordSys = new gp_Ax3(aGCommon.CentreOfMass(),
                                          new gp_Dir(aGCommon.PrincipalProperties().ThirdAxisOfInertia()),
                                          new gp_Dir(aGCommon.PrincipalProperties().FirstAxisOfInertia()));
            aTrsf.SetTransformation(aLocCoordSys);
        }

        TopoDS_Shape aST = (aTrsf.Form() == gp_Identity) ? theS : theS.Moved(new TopLoc_Location(aTrsf));

        // Initial axis-aligned BndBox
        Bnd_Box aShapeBox = new();
        if (theIsOptimal)
        {
            AddOptimal(aST, ref aShapeBox, theIsTriangulationUsed, theIsShapeToleranceUsed);
        }
        else
        {
            Add(aST, ref aShapeBox);
        }

        if (aShapeBox.IsVoid())
        {
            return;
        }

        gp_Pnt aPMin = aShapeBox.CornerMin();
        gp_Pnt aPMax = aShapeBox.CornerMax();

        gp_XYZ aXDir = new(1, 0, 0);
        gp_XYZ aYDir = new(0, 1, 0);
        gp_XYZ aZDir = new(0, 0, 1);

        // Compute the center of the box
        gp_XYZ aCenter = (aPMin.XYZ() + aPMax.XYZ()) / 2.0;

        // Compute the half diagonal size of the box.
        // It takes into account the gap.
        gp_XYZ anOBBHSize = (aPMax.XYZ() - aPMin.XYZ()) / 2.0;

        // Apply transformation if necessary
        if (aTrsf.Form() != gp_Identity)
        {
            aTrsf.Invert();
            aTrsf.Transforms(ref aCenter);

            // Make transformation
            gp_Mat aMat = aTrsf.HVectorialPart();

            // Compute axes directions of the box
            aXDir = new gp_XYZ(aMat.Value(1, 1), aMat.Value(2, 1), aMat.Value(3, 1));
            aYDir = new gp_XYZ(aMat.Value(1, 2), aMat.Value(2, 2), aMat.Value(3, 2));
            aZDir = new gp_XYZ(aMat.Value(1, 3), aMat.Value(2, 3), aMat.Value(3, 3));
        }

        if (theOBB.IsVoid())
        {
            // Create the OBB box

            // Set parameters to the OBB
            theOBB.SetCenter(new gp_Pnt(aCenter));

            // OCCT converts the gp_XYZ axes to gp_Dir implicitly
            theOBB.SetXComponent(new gp_Dir(aXDir), anOBBHSize.X());
            theOBB.SetYComponent(new gp_Dir(aYDir), anOBBHSize.Y());
            theOBB.SetZComponent(new gp_Dir(aZDir), anOBBHSize.Z());
            theOBB.SetAABox(aTrsf.Form() == gp_Identity);
        }
        else
        {
            // Recreate the OBB box

            var aListOfPnts = new NCollection_Array1<gp_Pnt>(0, 15);
            theOBB.GetVertex(aListOfPnts.AsSpan(0));

            double aX = anOBBHSize.X();
            double aY = anOBBHSize.Y();
            double aZ = anOBBHSize.Z();

            gp_XYZ aXext = aX * aXDir, aYext = aY * aYDir, aZext = aZ * aZDir;

            int aPntIdx = 8;
            aListOfPnts[aPntIdx++] = new gp_Pnt(aCenter - aXext - aYext - aZext);
            aListOfPnts[aPntIdx++] = new gp_Pnt(aCenter + aXext - aYext - aZext);
            aListOfPnts[aPntIdx++] = new gp_Pnt(aCenter - aXext + aYext - aZext);
            aListOfPnts[aPntIdx++] = new gp_Pnt(aCenter + aXext + aYext - aZext);
            aListOfPnts[aPntIdx++] = new gp_Pnt(aCenter - aXext - aYext + aZext);
            aListOfPnts[aPntIdx++] = new gp_Pnt(aCenter + aXext - aYext + aZext);
            aListOfPnts[aPntIdx++] = new gp_Pnt(aCenter - aXext + aYext + aZext);
            aListOfPnts[aPntIdx++] = new gp_Pnt(aCenter + aXext + aYext + aZext);

            theOBB.ReBuild(aListOfPnts);
        }
    }

    /// <summary>
    /// Adds the shape <paramref name="theS"/> to the oriented box <paramref name="theOBB"/>. From the shape's points
    /// (DiTO) when it is made of planar faces and linear edges, or carries triangulations and polygons (used unless
    /// <paramref name="theIsTriangulationUsed"/> is false); otherwise along the shape's axes of inertia.
    /// <paramref name="theIsShapeToleranceUsed"/> enlarges the box by the shapes' tolerances;
    /// <paramref name="theIsOptimal"/> looks for a tighter box, more slowly.
    /// </summary>
    public static void AddOBB(in TopoDS_Shape theS,
                              ref Bnd_OBB theOBB,
                              bool theIsTriangulationUsed = true,
                              bool theIsOptimal = false,
                              bool theIsShapeToleranceUsed = true)
    {
        if (CheckPoints(theS, theIsTriangulationUsed, theIsOptimal, theIsShapeToleranceUsed, ref theOBB))
        {
            return;
        }

        ComputePCA(theS, ref theOBB, theIsTriangulationUsed, theIsOptimal, theIsShapeToleranceUsed);
    }

    /// <summary>The oriented box of the shape: a new box given to <see cref="AddOBB"/> (not in OCCT).</summary>
    public static Bnd_OBB OBB(in TopoDS_Shape theS,
                              bool theIsTriangulationUsed = true,
                              bool theIsOptimal = false,
                              bool theIsShapeToleranceUsed = true)
    {
        Bnd_OBB anOBB = new();
        AddOBB(theS, ref anOBB, theIsTriangulationUsed, theIsOptimal, theIsShapeToleranceUsed);
        return anOBB;
    }
}
