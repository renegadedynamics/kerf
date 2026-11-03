// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBndLib/BRepBndLib_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Geometry.TransformForm;
using static Kerf.Topology.ShapeType;

namespace Occt;

public static partial class BRepBndLib
{
    // Returns TRUE if theC is line-like.
    private static bool IsLinear(CurveAdaptorBase theC)
    {
        CurveKind aCT = theC.GetType();
        if (aCT == OffsetCurve)
        {
            return IsLinear(new CurveAdaptor(theC.OffsetCurve().BasisCurve()));
        }

        if ((aCT == BSplineCurve) || (aCT == BezierCurve))
        {
            // Indeed, curves with C0-continuity and degree==1, may be
            // represented with set of points. It will be possible made
            // in the future.

            return ((theC.Degree() == 1) && (theC.Continuity() != C0));
        }

        if (aCT == Line)
        {
            return true;
        }

        return false;
    }

    // Returns TRUE if theS is plane-like.
    private static bool IsPlanar(SurfaceAdaptorBase theS)
    {
        SurfaceKind aST = theS.GetType();
        if (aST == SurfaceKind.OffsetSurface)
        {
            return IsPlanar(theS.BasisSurface());
        }

        if (aST == SurfaceOfExtrusion)
        {
            return IsLinear(theS.BasisCurve());
        }

        if ((aST == SurfaceKind.BSplineSurface) || (aST == SurfaceKind.BezierSurface))
        {
            if ((theS.UDegree() != 1) || (theS.VDegree() != 1))
            {
                return false;
            }

            // Indeed, surfaces with C0-continuity and degree==1, may be
            // represented with set of points. It will be possible made
            // in the future.

            return ((theS.UContinuity() != C0) && (theS.VContinuity() != C0));
        }

        if (aST == Plane)
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
    private static int PointsForOBB(in Shape theS,
                                    bool theIsTriangulationUsed,
                                    NCollection_Array1<Point3D>? thePts = null,
                                    NCollection_Array1<double>? theArrOfToler = null)
    {
        int aRetVal = 0;
        var anExpF = new Explorer();
        var anExpE = new Explorer();

        // get all vertices from the shape
        for (anExpF.Init(theS, ShapeType.Vertex); anExpF.More(); anExpF.Next())
        {
            Vertex aVert = ShapeCasts.Vertex(anExpF.Current());
            if (thePts is not null)
            {
                Point3D aP = Tool.Pnt(aVert);
                thePts[aRetVal] = aP;
            }

            if (theArrOfToler is not null)
            {
                theArrOfToler[aRetVal] = Tool.Tolerance(aVert);
            }

            ++aRetVal;
        }

        if (aRetVal == 0)
        {
            return 0;
        }

        // analyze the faces of the shape on planarity and existence of triangulation
        Placement aLoc;
        for (anExpF.Init(theS, ShapeType.Face); anExpF.More(); anExpF.Next())
        {
            Face aF = ShapeCasts.Face(anExpF.Current());
            var anAS = new FaceSurfaceAdaptor(aF, false);

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
                for (anExpE.Init(aF, ShapeType.Edge); anExpE.More(); anExpE.Next())
                {
                    Edge anE = ShapeCasts.Edge(anExpE.Current());
                    if (Tool.IsGeometric(anE))
                    {
                        var anAC = new EdgeCurveAdaptor(anE);
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
            Triangulation? aTrng = Tool.Triangulation(aF, out aLoc);
            if (aTrng is null)
            {
                // no triangulation on the face
                return 0;
            }

            int aCNode = aTrng.NodeCount;
            Transform3D aTrsf = aLoc.Transformation();
            for (int i = 1; i <= aCNode; i++)
            {
                if (thePts is not null)
                {
                    Point3D aP = aTrsf.Form() == Identity ? aTrng.Node(i) : aTrng.Node(i).Transformed(aTrsf);
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

        for (anExpE.Init(theS, ShapeType.Edge, ShapeType.Face); anExpE.More(); anExpE.Next())
        {
            Edge anE = ShapeCasts.Edge(anExpE.Current());
            if (Tool.IsGeometric(anE))
            {
                var anAC = new EdgeCurveAdaptor(anE);
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

            Polygon3D? aPolygon = Tool.Polygon3D(anE, out aLoc);
            if (aPolygon is null)
            {
                return 0;
            }

            int aCNode = aPolygon.NodeCount;
            NCollection_Array1<Point3D> aNodesArr = aPolygon.Nodes();
            for (int i = 1; i <= aCNode; i++)
            {
                if (thePts is not null)
                {
                    Point3D aP = aLoc.IsIdentity ? aNodesArr[i] : aNodesArr[i].Transformed(aLoc.Transformation());
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
    private static int IsWCS(in Direction3D theDir)
    {
        double aToler = global::Kerf.Engine.Foundation.Precision.Angular() * global::Kerf.Engine.Foundation.Precision.Angular();

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
    private static bool CheckPoints(in Shape theS,
                                    bool theIsTriangulationUsed,
                                    bool theIsOptimal,
                                    bool theIsShapeToleranceUsed,
                                    ref OrientedBox theOBB)
    {
        int aNbPnts = PointsForOBB(theS, theIsTriangulationUsed);

        if (aNbPnts < 1)
        {
            return false;
        }

        var anArrPnts = new NCollection_Array1<Point3D>(0, theOBB.IsVoid ? aNbPnts - 1 : aNbPnts + 7);
        NCollection_Array1<double> anArrOfTolerances = new();
        if (theIsShapeToleranceUsed)
        {
            anArrOfTolerances.Resize(anArrPnts.Lower(), anArrPnts.Upper(), false);
            anArrOfTolerances.Init(0.0);
        }

        NCollection_Array1<double>? aPtrArrTol = theIsShapeToleranceUsed ? anArrOfTolerances : null;

        PointsForOBB(theS, theIsTriangulationUsed, anArrPnts, aPtrArrTol);

        if (!theOBB.IsVoid)
        {
            // All points of old OBB have zero-tolerance
            theOBB.GetVertex(anArrPnts.AsSpan(aNbPnts));
        }

        theOBB.ReBuild(anArrPnts, aPtrArrTol, theIsOptimal);

        return (!theOBB.IsVoid);
    }

    private static void ComputeProperties(in Shape theS, Properties theGCommon)
    {
        var anExp = new Explorer();
        for (anExp.Init(theS, ShapeType.Solid); anExp.More(); anExp.Next())
        {
            var aG = new Properties();
            BRepGProp.VolumeProperties(anExp.Current(), aG, true);
            theGCommon.Add(aG);
        }

        for (anExp.Init(theS, ShapeType.Face, ShapeType.Solid); anExp.More(); anExp.Next())
        {
            var aG = new Properties();
            BRepGProp.SurfaceProperties(anExp.Current(), aG, true);
            theGCommon.Add(aG);
        }

        for (anExp.Init(theS, ShapeType.Edge, ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aG = new Properties();
            BRepGProp.LinearProperties(anExp.Current(), aG, true);
            theGCommon.Add(aG);
        }

        for (anExp.Init(theS, ShapeType.Vertex, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            var aG = new Properties(Tool.Pnt(ShapeCasts.Vertex(anExp.Current())));
            theGCommon.Add(aG);
        }
    }

    // Creates OBB with axes of inertia.
    private static void ComputePCA(in Shape theS,
                                   ref OrientedBox theOBB,
                                   bool theIsTriangulationUsed,
                                   bool theIsOptimal,
                                   bool theIsShapeToleranceUsed)
    {
        // Compute the transformation matrix to obtain more tight bounding box
        var aGCommon = new Properties();
        ComputeProperties(theS, aGCommon);

        // Transform the shape to the local coordinate system
        Transform3D aTrsf = new();

        // C++ converts the axes of inertia (gp_Vec) to gp_Dir implicitly
        int anIdx1 = IsWCS(new Direction3D(aGCommon.PrincipalProperties().FirstAxisOfInertia()));
        int anIdx2 = IsWCS(new Direction3D(aGCommon.PrincipalProperties().SecondAxisOfInertia()));

        if ((anIdx1 == 0) || (anIdx2 == 0))
        {
            // Coordinate system in which the shape will have the optimal bounding box
            var aLocCoordSys = new CoordinateSystem(aGCommon.CentreOfMass(),
                                          new Direction3D(aGCommon.PrincipalProperties().ThirdAxisOfInertia()),
                                          new Direction3D(aGCommon.PrincipalProperties().FirstAxisOfInertia()));
            aTrsf.SetTransformation(aLocCoordSys);
        }

        Shape aST = (aTrsf.Form() == Identity) ? theS : theS.Moved(new Placement(aTrsf));

        // Initial axis-aligned BndBox
        Box aShapeBox = new();
        if (theIsOptimal)
        {
            AddOptimal(aST, ref aShapeBox, theIsTriangulationUsed, theIsShapeToleranceUsed);
        }
        else
        {
            Add(aST, ref aShapeBox);
        }

        if (aShapeBox.IsVoid)
        {
            return;
        }

        Point3D aPMin = aShapeBox.CornerMin();
        Point3D aPMax = aShapeBox.CornerMax();

        Xyz aXDir = new(1, 0, 0);
        Xyz aYDir = new(0, 1, 0);
        Xyz aZDir = new(0, 0, 1);

        // Compute the center of the box
        Xyz aCenter = (aPMin.XYZ() + aPMax.XYZ()) / 2.0;

        // Compute the half diagonal size of the box.
        // It takes into account the gap.
        Xyz anOBBHSize = (aPMax.XYZ() - aPMin.XYZ()) / 2.0;

        // Apply transformation if necessary
        if (aTrsf.Form() != Identity)
        {
            aTrsf.Invert();
            aTrsf.Transforms(ref aCenter);

            // Make transformation
            Matrix3x3 aMat = aTrsf.HVectorialPart();

            // Compute axes directions of the box
            aXDir = new Xyz(aMat.Value(1, 1), aMat.Value(2, 1), aMat.Value(3, 1));
            aYDir = new Xyz(aMat.Value(1, 2), aMat.Value(2, 2), aMat.Value(3, 2));
            aZDir = new Xyz(aMat.Value(1, 3), aMat.Value(2, 3), aMat.Value(3, 3));
        }

        if (theOBB.IsVoid)
        {
            // Create the OBB box

            // Set parameters to the OBB
            theOBB.SetCenter(new Point3D(aCenter));

            // C++ converts the gp_XYZ axes to gp_Dir implicitly
            theOBB.SetXComponent(new Direction3D(aXDir), anOBBHSize.X());
            theOBB.SetYComponent(new Direction3D(aYDir), anOBBHSize.Y());
            theOBB.SetZComponent(new Direction3D(aZDir), anOBBHSize.Z());
            theOBB.SetAABox(aTrsf.Form() == Identity);
        }
        else
        {
            // Recreate the OBB box

            var aListOfPnts = new NCollection_Array1<Point3D>(0, 15);
            theOBB.GetVertex(aListOfPnts.AsSpan(0));

            double aX = anOBBHSize.X();
            double aY = anOBBHSize.Y();
            double aZ = anOBBHSize.Z();

            Xyz aXext = aX * aXDir, aYext = aY * aYDir, aZext = aZ * aZDir;

            int aPntIdx = 8;
            aListOfPnts[aPntIdx++] = new Point3D(aCenter - aXext - aYext - aZext);
            aListOfPnts[aPntIdx++] = new Point3D(aCenter + aXext - aYext - aZext);
            aListOfPnts[aPntIdx++] = new Point3D(aCenter - aXext + aYext - aZext);
            aListOfPnts[aPntIdx++] = new Point3D(aCenter + aXext + aYext - aZext);
            aListOfPnts[aPntIdx++] = new Point3D(aCenter - aXext - aYext + aZext);
            aListOfPnts[aPntIdx++] = new Point3D(aCenter + aXext - aYext + aZext);
            aListOfPnts[aPntIdx++] = new Point3D(aCenter - aXext + aYext + aZext);
            aListOfPnts[aPntIdx++] = new Point3D(aCenter + aXext + aYext + aZext);

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
    public static void AddOBB(in Shape theS,
                              ref OrientedBox theOBB,
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
    public static OrientedBox OBB(in Shape theS,
                              bool theIsTriangulationUsed = true,
                              bool theIsOptimal = false,
                              bool theIsShapeToleranceUsed = true)
    {
        OrientedBox anOBB = new();
        AddOBB(theS, ref anOBB, theIsTriangulationUsed, theIsOptimal, theIsShapeToleranceUsed);
        return anOBB;
    }
}
