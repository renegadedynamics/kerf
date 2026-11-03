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
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Geometry.TransformForm;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Measurement;

public static partial class Bounds
{
    // Returns TRUE if theC is line-like.
    private static bool IsLinear(CurveAdaptorBase c)
    {
        CurveKind ct = c.GetType();
        if (ct == OffsetCurve)
        {
            return IsLinear(new CurveAdaptor(c.OffsetCurve().BasisCurve()));
        }

        if ((ct == BSplineCurve) || (ct == BezierCurve))
        {
            // Indeed, curves with C0-continuity and degree==1, may be
            // represented with set of points. It will be possible made
            // in the future.

            return ((c.Degree() == 1) && (c.Continuity() != C0));
        }

        if (ct == Line)
        {
            return true;
        }

        return false;
    }

    // Returns TRUE if theS is plane-like.
    private static bool IsPlanar(SurfaceAdaptorBase s)
    {
        SurfaceKind st = s.GetType();
        if (st == SurfaceKind.OffsetSurface)
        {
            return IsPlanar(s.BasisSurface());
        }

        if (st == SurfaceOfExtrusion)
        {
            return IsLinear(s.BasisCurve());
        }

        if ((st == SurfaceKind.BSplineSurface) || (st == SurfaceKind.BezierSurface))
        {
            if ((s.UDegree() != 1) || (s.VDegree() != 1))
            {
                return false;
            }

            // Indeed, surfaces with C0-continuity and degree==1, may be
            // represented with set of points. It will be possible made
            // in the future.

            return ((s.UContinuity() != C0) && (s.VContinuity() != C0));
        }

        if (st == Plane)
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
    private static int PointsForOBB(in Shape s,
                                    bool isTriangulationUsed,
                                    NCollection_Array1<Point3D>? pts = null,
                                    NCollection_Array1<double>? arrOfToler = null)
    {
        int retVal = 0;
        var expF = new Explorer();
        var expE = new Explorer();

        // get all vertices from the shape
        for (expF.Init(s, ShapeType.Vertex); expF.More(); expF.Next())
        {
            Vertex vert = ShapeCasts.Vertex(expF.Current());
            if (pts is not null)
            {
                Point3D p = Tool.Pnt(vert);
                pts[retVal] = p;
            }

            if (arrOfToler is not null)
            {
                arrOfToler[retVal] = Tool.Tolerance(vert);
            }

            ++retVal;
        }

        if (retVal == 0)
        {
            return 0;
        }

        // analyze the faces of the shape on planarity and existence of triangulation
        Placement loc;
        for (expF.Init(s, ShapeType.Face); expF.More(); expF.Next())
        {
            Face f = ShapeCasts.Face(expF.Current());
            var anAS = new FaceSurfaceAdaptor(f, false);

            if (!IsPlanar(anAS))
            {
                if (!isTriangulationUsed)
                {
                    // not planar and triangulation usage disabled
                    return 0;
                }
            }
            else
            {
                // planar face
                for (expE.Init(f, ShapeType.Edge); expE.More(); expE.Next())
                {
                    Edge e = ShapeCasts.Edge(expE.Current());
                    if (Tool.IsGeometric(e))
                    {
                        var ac = new EdgeCurveAdaptor(e);
                        if (!IsLinear(ac))
                        {
                            if (!isTriangulationUsed)
                            {
                                // not linear and triangulation usage disabled
                                return 0;
                            }

                            break;
                        }
                    }
                }

                if (!expE.More())
                {
                    // skip planar face with linear edges as its vertices have already been added
                    continue;
                }
            }

            // Use triangulation of the face
            Triangulation? trng = Tool.Triangulation(f, out loc);
            if (trng is null)
            {
                // no triangulation on the face
                return 0;
            }

            int cNode = trng.NodeCount;
            Transform3D trsf = loc.Transformation();
            for (int i = 1; i <= cNode; i++)
            {
                if (pts is not null)
                {
                    Point3D p = trsf.Form() == Identity ? trng.Node(i) : trng.Node(i).Transformed(trsf);
                    pts[retVal] = p;
                }

                if (arrOfToler is not null)
                {
                    arrOfToler[retVal] = trng.Deflection();
                }

                ++retVal;
            }
        }

        // Consider edges without faces

        for (expE.Init(s, ShapeType.Edge, ShapeType.Face); expE.More(); expE.Next())
        {
            Edge e = ShapeCasts.Edge(expE.Current());
            if (Tool.IsGeometric(e))
            {
                var ac = new EdgeCurveAdaptor(e);
                if (IsLinear(ac))
                {
                    // skip linear edge as its vertices have already been added
                    continue;
                }
            }

            if (!isTriangulationUsed)
            {
                // not linear and triangulation usage disabled
                return 0;
            }

            Polygon3D? polygon = Tool.Polygon3D(e, out loc);
            if (polygon is null)
            {
                return 0;
            }

            int cNode = polygon.NodeCount;
            NCollection_Array1<Point3D> nodesArr = polygon.Nodes();
            for (int i = 1; i <= cNode; i++)
            {
                if (pts is not null)
                {
                    Point3D p = loc.IsIdentity ? nodesArr[i] : nodesArr[i].Transformed(loc.Transformation());
                    pts[retVal] = p;
                }

                if (arrOfToler is not null)
                {
                    arrOfToler[retVal] = polygon.Deflection();
                }

                ++retVal;
            }
        }

        return retVal;
    }

    // Returns 0 if the theDir does not match any axis of WCS.
    // Otherwise, returns the index of correspond axis.
    private static int IsWCS(in Direction3D dir)
    {
        double toler = global::Kerf.Engine.Foundation.Precision.Angular() * global::Kerf.Engine.Foundation.Precision.Angular();

        double x = dir.X(), y = dir.Y(), z = dir.Z();

        double vx = y * y + z * z, vy = x * x + z * z, vz = x * x + y * y;

        if (vz < toler)
        {
            return 3; // Z-axis
        }

        if (vy < toler)
        {
            return 2; // Y-axis
        }

        if (vx < toler)
        {
            return 1; // X-axis
        }

        return 0;
    }

    // Collects points for DiTO algorithm for OBB construction on
    // linear/planar shapes and shapes having triangulation
    // (http://www.idt.mdh.se/~tla/publ/FastOBBs.pdf).
    private static bool CheckPoints(in Shape s,
                                    bool isTriangulationUsed,
                                    bool isOptimal,
                                    bool isShapeToleranceUsed,
                                    ref OrientedBox obb)
    {
        int nbPnts = PointsForOBB(s, isTriangulationUsed);

        if (nbPnts < 1)
        {
            return false;
        }

        var arrPnts = new NCollection_Array1<Point3D>(0, obb.IsVoid ? nbPnts - 1 : nbPnts + 7);
        NCollection_Array1<double> arrOfTolerances = new();
        if (isShapeToleranceUsed)
        {
            arrOfTolerances.Resize(arrPnts.Lower(), arrPnts.Upper(), false);
            arrOfTolerances.Init(0.0);
        }

        NCollection_Array1<double>? ptrArrTol = isShapeToleranceUsed ? arrOfTolerances : null;

        PointsForOBB(s, isTriangulationUsed, arrPnts, ptrArrTol);

        if (!obb.IsVoid)
        {
            // All points of old OBB have zero-tolerance
            obb.GetVertex(arrPnts.AsSpan(nbPnts));
        }

        obb.ReBuild(arrPnts, ptrArrTol, isOptimal);

        return (!obb.IsVoid);
    }

    private static void ComputeProperties(in Shape s, Properties gCommon)
    {
        var exp = new Explorer();
        for (exp.Init(s, ShapeType.Solid); exp.More(); exp.Next())
        {
            var g = new Properties();
            MassPropertiesBuilder.VolumeProperties(exp.Current(), g, true);
            gCommon.Add(g);
        }

        for (exp.Init(s, ShapeType.Face, ShapeType.Solid); exp.More(); exp.Next())
        {
            var g = new Properties();
            MassPropertiesBuilder.SurfaceProperties(exp.Current(), g, true);
            gCommon.Add(g);
        }

        for (exp.Init(s, ShapeType.Edge, ShapeType.Face); exp.More(); exp.Next())
        {
            var g = new Properties();
            MassPropertiesBuilder.LinearProperties(exp.Current(), g, true);
            gCommon.Add(g);
        }

        for (exp.Init(s, ShapeType.Vertex, ShapeType.Edge); exp.More(); exp.Next())
        {
            var g = new Properties(Tool.Pnt(ShapeCasts.Vertex(exp.Current())));
            gCommon.Add(g);
        }
    }

    // Creates OBB with axes of inertia.
    private static void ComputePCA(in Shape s,
                                   ref OrientedBox obb,
                                   bool isTriangulationUsed,
                                   bool isOptimal,
                                   bool isShapeToleranceUsed)
    {
        // Compute the transformation matrix to obtain more tight bounding box
        var gCommon = new Properties();
        ComputeProperties(s, gCommon);

        // Transform the shape to the local coordinate system
        Transform3D trsf = new();

        // C++ converts the axes of inertia (gp_Vec) to gp_Dir implicitly
        int idx1 = IsWCS(new Direction3D(gCommon.PrincipalProperties().FirstAxisOfInertia()));
        int idx2 = IsWCS(new Direction3D(gCommon.PrincipalProperties().SecondAxisOfInertia()));

        if ((idx1 == 0) || (idx2 == 0))
        {
            // Coordinate system in which the shape will have the optimal bounding box
            var locCoordSys = new CoordinateSystem(gCommon.CentreOfMass(),
                                          new Direction3D(gCommon.PrincipalProperties().ThirdAxisOfInertia()),
                                          new Direction3D(gCommon.PrincipalProperties().FirstAxisOfInertia()));
            trsf.SetTransformation(locCoordSys);
        }

        Shape st = (trsf.Form() == Identity) ? s : s.Moved(new Placement(trsf));

        // Initial axis-aligned BndBox
        Box shapeBox = new();
        if (isOptimal)
        {
            AddOptimal(st, ref shapeBox, isTriangulationUsed, isShapeToleranceUsed);
        }
        else
        {
            Add(st, ref shapeBox);
        }

        if (shapeBox.IsVoid)
        {
            return;
        }

        Point3D pMin = shapeBox.CornerMin();
        Point3D pMax = shapeBox.CornerMax();

        Xyz xDir = new(1, 0, 0);
        Xyz yDir = new(0, 1, 0);
        Xyz zDir = new(0, 0, 1);

        // Compute the center of the box
        Xyz center = (pMin.XYZ() + pMax.XYZ()) / 2.0;

        // Compute the half diagonal size of the box.
        // It takes into account the gap.
        Xyz obbhSize = (pMax.XYZ() - pMin.XYZ()) / 2.0;

        // Apply transformation if necessary
        if (trsf.Form() != Identity)
        {
            trsf.Invert();
            trsf.Transforms(ref center);

            // Make transformation
            Matrix3x3 mat = trsf.HVectorialPart();

            // Compute axes directions of the box
            xDir = new Xyz(mat.Value(1, 1), mat.Value(2, 1), mat.Value(3, 1));
            yDir = new Xyz(mat.Value(1, 2), mat.Value(2, 2), mat.Value(3, 2));
            zDir = new Xyz(mat.Value(1, 3), mat.Value(2, 3), mat.Value(3, 3));
        }

        if (obb.IsVoid)
        {
            // Create the OBB box

            // Set parameters to the OBB
            obb.SetCenter(new Point3D(center));

            // C++ converts the gp_XYZ axes to gp_Dir implicitly
            obb.SetXComponent(new Direction3D(xDir), obbhSize.X());
            obb.SetYComponent(new Direction3D(yDir), obbhSize.Y());
            obb.SetZComponent(new Direction3D(zDir), obbhSize.Z());
            obb.SetAABox(trsf.Form() == Identity);
        }
        else
        {
            // Recreate the OBB box

            var listOfPnts = new NCollection_Array1<Point3D>(0, 15);
            obb.GetVertex(listOfPnts.AsSpan(0));

            double x = obbhSize.X();
            double y = obbhSize.Y();
            double z = obbhSize.Z();

            Xyz xext = x * xDir, yext = y * yDir, zext = z * zDir;

            int pntIdx = 8;
            listOfPnts[pntIdx++] = new Point3D(center - xext - yext - zext);
            listOfPnts[pntIdx++] = new Point3D(center + xext - yext - zext);
            listOfPnts[pntIdx++] = new Point3D(center - xext + yext - zext);
            listOfPnts[pntIdx++] = new Point3D(center + xext + yext - zext);
            listOfPnts[pntIdx++] = new Point3D(center - xext - yext + zext);
            listOfPnts[pntIdx++] = new Point3D(center + xext - yext + zext);
            listOfPnts[pntIdx++] = new Point3D(center - xext + yext + zext);
            listOfPnts[pntIdx++] = new Point3D(center + xext + yext + zext);

            obb.ReBuild(listOfPnts);
        }
    }

    /// <summary>
    /// Adds the shape <paramref name="s"/> to the oriented box <paramref name="obb"/>. From the shape's points
    /// (DiTO) when it is made of planar faces and linear edges, or carries triangulations and polygons (used unless
    /// <paramref name="isTriangulationUsed"/> is false); otherwise along the shape's axes of inertia.
    /// <paramref name="isShapeToleranceUsed"/> enlarges the box by the shapes' tolerances;
    /// <paramref name="isOptimal"/> looks for a tighter box, more slowly.
    /// </summary>
    public static void AddOBB(in Shape s,
                              ref OrientedBox obb,
                              bool isTriangulationUsed = true,
                              bool isOptimal = false,
                              bool isShapeToleranceUsed = true)
    {
        if (CheckPoints(s, isTriangulationUsed, isOptimal, isShapeToleranceUsed, ref obb))
        {
            return;
        }

        ComputePCA(s, ref obb, isTriangulationUsed, isOptimal, isShapeToleranceUsed);
    }

    /// <summary>The oriented box of the shape: a new box given to <see cref="AddOBB"/> (not in OCCT).</summary>
    public static OrientedBox OBB(in Shape s,
                              bool isTriangulationUsed = true,
                              bool isOptimal = false,
                              bool isShapeToleranceUsed = true)
    {
        OrientedBox obb = new();
        AddOBB(s, ref obb, isTriangulationUsed, isOptimal, isShapeToleranceUsed);
        return obb;
    }
}
