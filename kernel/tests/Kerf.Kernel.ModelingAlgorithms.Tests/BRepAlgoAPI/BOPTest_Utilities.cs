// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepAlgoAPI;

/// <summary>
/// OCCT's GTest helpers of the Boolean tests (src/ModelingAlgorithms/TKBO/GTests/BOPTest_Utilities.pxx): the shape
/// makers and properties of <c>BOPTest_Utilities</c>, shared by the BRepAlgoAPI GTests (unit 6P). OCCT's
/// <c>BRepAlgoAPI_TestBase</c> fixture is <see cref="BRepAlgoAPI_TestBase"/>.
/// </summary>
/// <remarks>
/// <c>CreateBlend</c> needs BRepFilletAPI_MakeFillet (phase 7): left out, and the tests that call it are skipped with
/// that reason. The makers' results are published (frozen) shapes, as a caller's are.
/// </remarks>
public static class BOPTest_Utilities
{
    /// <summary>Profile command types (equivalent to TCL profile commands).</summary>
    public enum ProfileCmd
    {
        O,  // Set origin (O X Y Z)
        P,  // Set plane (P nx ny nz dx dy dz)
        F,  // Set first point
        X,  // Translate along X
        Y,  // Translate along Y
        L,  // Translate along direction
        XX, // Set X coordinate
        YY, // Set Y coordinate
        T,  // Translate by vector
        TT, // Set point
        R,  // Rotate direction
        RR, // Set direction angle
        D,  // Set direction vector
        C,  // Arc (circle)
        W,  // Make closed wire
        WW, // Make open wire
    }

    /// <summary>Profile command structure.</summary>
    public sealed class ProfileOperation(ProfileCmd c, params double[] p)
    {
        public ProfileCmd cmd = c;
        public List<double> @params = [.. p];
    }

    /// <summary>Default tolerance values.</summary>
    public static double DefaultTolerance() => 1.0e-6;

    public static double DefaultFuzzyValue() => 1.0e-8;

    /// <summary>Calculate surface area of a shape.</summary>
    public static double GetSurfaceArea(in Shape theShape)
    {
        if (theShape.IsNull)
        {
            return 0.0;
        }
        Properties aProps = new();
        MassPropertiesBuilder.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    /// <summary>Calculate volume of a shape.</summary>
    public static double GetVolume(in Shape theShape)
    {
        if (theShape.IsNull)
        {
            return 0.0;
        }
        Properties aProps = new();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    /// <summary>Check if shape is effectively empty (very small surface area).</summary>
    public static bool IsEmpty(in Shape theShape, double theTolerance = 1.0e-6) =>
        GetSurfaceArea(theShape) <= theTolerance;

    /// <summary>Create unit box at origin (1x1x1).</summary>
    public static Shape CreateUnitBox() => new MakeBox(1.0, 1.0, 1.0).Shape();

    /// <summary>Create box at specific location.</summary>
    public static Shape CreateBox(in Point3D theCorner, double theX, double theY, double theZ) =>
        new MakeBox(theCorner, theX, theY, theZ).Shape();

    /// <summary>Create unit sphere (radius = 1.0).</summary>
    public static Shape CreateUnitSphere() => new MakeSphere(1.0).Shape();

    /// <summary>Create sphere at location with radius.</summary>
    public static Shape CreateSphere(in Point3D theCenter, double theRadius) =>
        new MakeSphere(theCenter, theRadius).Shape();

    /// <summary>Create cylinder.</summary>
    public static Shape CreateCylinder(double theRadius, double theHeight) =>
        new ApiMakeCylinder(theRadius, theHeight).Shape();

    /// <summary>Create cone.</summary>
    public static Shape CreateCone(double theR1, double theR2, double theHeight) =>
        new ApiMakeCone(theR1, theR2, theHeight).Shape();

    /// <summary>Convert shape to NURBS.</summary>
    public static Shape ConvertToNurbs(in Shape theShape)
    {
        NurbsConvert aNurbsConverter = new(theShape);
        if (!aNurbsConverter.IsDone)
        {
            return new Shape(); // Return null shape on failure
        }
        return aNurbsConverter.Shape();
    }

    /// <summary>Create rectangular polygon face.</summary>
    public static Shape CreatePolygonFace(in Point3D theP1, in Point3D theP2, in Point3D theP3, in Point3D theP4)
    {
        MakePolygon aPolygonMaker = new();
        aPolygonMaker.Add(theP1);
        aPolygonMaker.Add(theP2);
        aPolygonMaker.Add(theP3);
        aPolygonMaker.Add(theP4);
        aPolygonMaker.Close();

        if (!aPolygonMaker.IsDone)
        {
            return new Shape();
        }

        MakeFace aFaceMaker = new(aPolygonMaker.Wire());
        return aFaceMaker.Shape();
    }

    /// <summary>Rotate shape around axis.</summary>
    public static Shape RotateShape(in Shape theShape, in Axis3D theAxis, double theAngle)
    {
        Transform3D aTrsf = new Transform3D().WithRotation(theAxis, theAngle);
        Transform aTransformer = new(theShape, aTrsf);
        return aTransformer.Shape();
    }

    /// <summary>Translate shape by vector.</summary>
    public static Shape TranslateShape(in Shape theShape, in Vector3D theVector)
    {
        Transform3D aTrsf = new Transform3D().WithTranslation(theVector);
        Transform aTransformer = new(theShape, aTrsf);
        return aTransformer.Shape();
    }

    /// <summary>Create vertex from point.</summary>
    public static Vertex CreateVertex(in Point3D thePoint) => new MakeVertex(thePoint).Vertex();

    /// <summary>Create edge between two points.</summary>
    public static Edge CreateEdge(in Point3D theP1, in Point3D theP2) => new MakeEdge(theP1, theP2).Edge();

    /// <summary>Create wire from list of points (closed polygon).</summary>
    public static Wire CreatePolygonWire(IReadOnlyList<Point3D> thePoints, bool theClose = true)
    {
        MakePolygon aPolygonMaker = new();
        foreach (Point3D aPt in thePoints)
        {
            aPolygonMaker.Add(aPt);
        }
        if (theClose)
        {
            aPolygonMaker.Close();
        }
        return aPolygonMaker.Wire();
    }

    /// <summary>
    /// Create complex profile using simplified approach (similar to TCL profile command). Simulates "profile rev S face
    /// F x y [commands...]" by creating a rectangular wire.
    /// </summary>
    public static Wire CreateProfileWire(in Plane3D thePlane, in Point2D theStartPt, IReadOnlyList<string> theCommands)
    {
        // Simplified profile creation - create a rectangular wire based on commands
        // This approximates the TCL profile behavior for basic geometric operations
        List<Point2D> aPoints = [theStartPt];

        Point2D aCurrentPt = theStartPt;
        Vector2D aCurrentDir = new(1.0, 0.0); // Default direction

        // Process simplified command set
        for (int i = 0; i < theCommands.Count; ++i)
        {
            string aCmd = theCommands[i];
            if (aCmd == "Y" && i + 1 < theCommands.Count)
            {
                // Y command: translate along Y
                double aDY = double.Parse(theCommands[i + 1], System.Globalization.CultureInfo.InvariantCulture);
                aCurrentPt = aCurrentPt.WithY(aCurrentPt.Y + aDY);
                aPoints.Add(aCurrentPt);
                i++; // Skip next parameter
            }
            else if (aCmd == "C" && i + 2 < theCommands.Count)
            {
                // C command: arc (simplified as straight line for now)
                double aRadius = double.Parse(theCommands[i + 1], System.Globalization.CultureInfo.InvariantCulture);
                // Simplified: just move in current direction
                Vector2D aMoveVec = aCurrentDir * aRadius;
                aCurrentPt = aCurrentPt.Translated(aMoveVec);
                aPoints.Add(aCurrentPt);
                i += 2; // Skip next two parameters
            }
        }

        // Convert 2D points to 3D points on the plane
        List<Point3D> a3DPoints = [];
        foreach (Point2D aPt2d in aPoints)
        {
            a3DPoints.Add(global::Kerf.Engine.Geometry.ElSLib.Value(aPt2d.X, aPt2d.Y, thePlane));
        }

        return CreatePolygonWire(a3DPoints, true);
    }

    /// <summary>
    /// Create profile using typed commands (equivalent to TCL profile). plane: the reference plane/face (equivalent to
    /// "S face" in TCL); operations: list of profile operations.
    /// </summary>
    public static Shape CreateProfile(in Plane3D thePlane, IReadOnlyList<ProfileOperation> theOperations)
    {
        MakeWire aMakeWire = new();

        Point2D aCurrentPt = new(0, 0);  // Current point in 2D
        Vector2D aCurrentDir = new(1, 0); // Current direction in 2D
        bool aFirstSet = false;
        Point2D aFirstPt = new(0, 0);
        Plane3D aWorkingPlane = thePlane; // Working plane that can be modified by P command

        foreach (ProfileOperation op in theOperations)
        {
            switch (op.cmd)
            {
                case ProfileCmd.O: // Set origin (affects the plane, not the starting point)
                    if (op.@params.Count >= 3)
                    {
                        // O command sets the origin of the plane coordinate system
                        Point3D aNewOrigin = new(op.@params[0], op.@params[1], op.@params[2]);
                        CoordinateSystem aNewAx3 = new(aNewOrigin, aWorkingPlane.Axis.Direction, aWorkingPlane.XAxis.Direction);
                        aWorkingPlane = new Plane3D(aNewAx3);

                        // Profile still starts at (0,0) in the new plane coordinate system
                        if (!aFirstSet)
                        {
                            aCurrentPt = aCurrentPt.WithCoord(0.0, 0.0); // TCL default: start at (0,0)
                            aFirstPt = aCurrentPt;
                            aFirstSet = true;
                        }
                    }
                    break;

                case ProfileCmd.P: // Set plane (P nx ny nz dx dy dz)
                    if (op.@params.Count >= 6)
                    {
                        // P command sets plane normal and X direction vectors
                        Vector3D aNormal = new(op.@params[0], op.@params[1], op.@params[2]);
                        Vector3D aXDir = new(op.@params[3], op.@params[4], op.@params[5]);

                        // Create new coordinate system
                        Direction3D aNormalDir = new(aNormal);
                        Direction3D aXDirection = new(aXDir);

                        // Use the current plane's origin but new orientation
                        CoordinateSystem aNewAx3 = new(aWorkingPlane.Location, aNormalDir, aXDirection);
                        aWorkingPlane = new Plane3D(aNewAx3);
                    }
                    break;

                case ProfileCmd.F: // Set first point
                    if (op.@params.Count >= 2)
                    {
                        aCurrentPt = aCurrentPt.WithCoord(op.@params[0], op.@params[1]);
                        aFirstPt = aCurrentPt;
                        aFirstSet = true;
                    }
                    break;

                case ProfileCmd.X: // Translate along X
                    if (op.@params.Count != 0)
                    {
                        // If first point not set, implicitly start at (0,0)
                        if (!aFirstSet)
                        {
                            aCurrentPt = aCurrentPt.WithCoord(0.0, 0.0);
                            aFirstPt = aCurrentPt;
                            aFirstSet = true;
                        }

                        Point2D aNewPt = new(aCurrentPt.X + op.@params[0], aCurrentPt.Y);
                        // Add line segment
                        Point3D aPt1 = global::Kerf.Engine.Geometry.ElSLib.Value(aCurrentPt.X, aCurrentPt.Y, aWorkingPlane);
                        Point3D aPt2 = global::Kerf.Engine.Geometry.ElSLib.Value(aNewPt.X, aNewPt.Y, aWorkingPlane);
                        aMakeWire.Add(new MakeEdge(aPt1, aPt2).Edge());
                        aCurrentPt = aNewPt;
                        aCurrentDir = new Vector2D(op.@params[0] > 0 ? 1 : -1, 0);
                    }
                    break;

                case ProfileCmd.Y: // Translate along Y
                    if (op.@params.Count != 0)
                    {
                        // If first point not set, implicitly start at (0,0)
                        if (!aFirstSet)
                        {
                            aCurrentPt = aCurrentPt.WithCoord(0.0, 0.0);
                            aFirstPt = aCurrentPt;
                            aFirstSet = true;
                        }

                        Point2D aNewPt = new(aCurrentPt.X, aCurrentPt.Y + op.@params[0]);
                        // Add line segment
                        Point3D aPt1 = global::Kerf.Engine.Geometry.ElSLib.Value(aCurrentPt.X, aCurrentPt.Y, aWorkingPlane);
                        Point3D aPt2 = global::Kerf.Engine.Geometry.ElSLib.Value(aNewPt.X, aNewPt.Y, aWorkingPlane);
                        aMakeWire.Add(new MakeEdge(aPt1, aPt2).Edge());
                        aCurrentPt = aNewPt;
                        aCurrentDir = new Vector2D(0, op.@params[0] > 0 ? 1 : -1);
                    }
                    break;

                case ProfileCmd.C: // Arc
                    if (op.@params.Count >= 2)
                    {
                        double aRadius = Math.Abs(op.@params[0]);
                        double aAngleDeg = op.@params[1];
                        double aAngleRad = aAngleDeg * M_PI / 180.0;

                        // Handle full circle case (360 degrees)
                        if (Math.Abs(aAngleDeg) >= 360.0)
                        {
                            // Create a full circle centered at current point (0,0 if not set)
                            Point2D aCenter2D = aFirstSet ? aCurrentPt : new Point2D(0, 0);
                            Point3D aCenter3D = global::Kerf.Engine.Geometry.ElSLib.Value(aCenter2D.X, aCenter2D.Y, aWorkingPlane);

                            // Create full circle
                            Circle3D aCirc = new(new Frame3D(aCenter3D, aWorkingPlane.Axis.Direction), aRadius);
                            aMakeWire.Add(new MakeEdge(aCirc).Edge());

                            // Set current point to a point on the circle
                            aCurrentPt = new Point2D(aCenter2D.X + aRadius, aCenter2D.Y);
                            aCurrentDir = new Vector2D(0, 1); // Tangent direction
                            aFirstSet = true;
                        }
                        else
                        {
                            // If first point not set, start at origin with default direction
                            if (!aFirstSet)
                            {
                                aCurrentPt = aCurrentPt.WithCoord(0.0, 0.0);
                                aCurrentDir = new Vector2D(1, 0); // Default direction: positive X
                                aFirstPt = aCurrentPt;
                                aFirstSet = true;
                            }

                            // Create arc from current point
                            Vector2D aNormal = new(-aCurrentDir.Y, aCurrentDir.X);
                            Point2D aCenter = aCurrentPt.Translated(aNormal * aRadius);

                            // Calculate end point
                            double aStartAngle = Math.Atan2(aCurrentPt.Y - aCenter.Y, aCurrentPt.X - aCenter.X);
                            double aEndAngle = aStartAngle + aAngleRad;
                            Point2D aEndPt = new(aCenter.X + aRadius * Math.Cos(aEndAngle), aCenter.Y + aRadius * Math.Sin(aEndAngle));

                            // Create 3D arc
                            Point3D aPt1 = global::Kerf.Engine.Geometry.ElSLib.Value(aCurrentPt.X, aCurrentPt.Y, aWorkingPlane);
                            Point3D aPt2 = global::Kerf.Engine.Geometry.ElSLib.Value(aEndPt.X, aEndPt.Y, aWorkingPlane);
                            Point3D aCenter3D = global::Kerf.Engine.Geometry.ElSLib.Value(aCenter.X, aCenter.Y, aWorkingPlane);

                            Circle3D aCirc = new(new Frame3D(aCenter3D, aWorkingPlane.Axis.Direction), aRadius);
                            aMakeWire.Add(new MakeEdge(aCirc, aPt1, aPt2).Edge());

                            aCurrentPt = aEndPt;
                            aCurrentDir = new Vector2D(Math.Cos(aEndAngle + M_PI / 2), Math.Sin(aEndAngle + M_PI / 2));
                        }
                    }
                    break;

                case ProfileCmd.D: // Set direction vector
                    if (op.@params.Count >= 2)
                    {
                        aCurrentDir = new Vector2D(op.@params[0], op.@params[1]);
                        aCurrentDir = aCurrentDir.Normalized();
                    }
                    break;

                case ProfileCmd.W: // Make closed wire
                    // W command closes the current wire - handled at end of function
                    break;

                default:
                    // Other commands can be added as needed
                    break;
            }
        }

        // Close the wire if needed (TCL profiles are typically closed by default)
        if (aFirstSet && !aCurrentPt.IsEqual(aFirstPt, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            // Add closing edge back to start
            Point3D aPt1 = global::Kerf.Engine.Geometry.ElSLib.Value(aCurrentPt.X, aCurrentPt.Y, aWorkingPlane);
            Point3D aPt2 = global::Kerf.Engine.Geometry.ElSLib.Value(aFirstPt.X, aFirstPt.Y, aWorkingPlane);
            aMakeWire.Add(new MakeEdge(aPt1, aPt2).Edge());
        }

        Assert.True(aMakeWire.IsDone, "Profile wire creation failed");
        Wire aWire = aMakeWire.Wire();

        // Create face from wire
        MakeFace aFaceMaker = new(aWire);
        Assert.True(aFaceMaker.IsDone, "Profile face creation failed");
        return aFaceMaker.Face();
    }

    /// <summary>Create profile from operations list (simplified version).</summary>
    public static Shape CreateProfileFromOperations(IReadOnlyList<ProfileOperation> theOperations)
    {
        // Use default plane (XY plane)
        Plane3D aPlane = new(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z));
        return CreateProfile(aPlane, theOperations);
    }

    /// <summary>Create rectangular profile from corner and dimensions.</summary>
    public static Wire CreateRectangularProfile(in Point3D theCorner, double theX, double theY)
    {
        List<Point3D> aPoints =
        [
            theCorner,
            new Point3D(theCorner.X + theX, theCorner.Y, theCorner.Z),
            new Point3D(theCorner.X + theX, theCorner.Y + theY, theCorner.Z),
            new Point3D(theCorner.X, theCorner.Y + theY, theCorner.Z),
        ];
        return CreatePolygonWire(aPoints, true);
    }

    /// <summary>Create face from wire.</summary>
    public static Shape CreateFaceFromWire(in Wire theWire)
    {
        MakeFace aFaceMaker = new(theWire);
        if (!aFaceMaker.IsDone)
        {
            return new Shape();
        }
        return aFaceMaker.Shape();
    }

    /// <summary>Create prism by extruding shape along vector.</summary>
    public static Shape CreatePrism(in Shape theProfile, in Vector3D theDirection)
    {
        ApiMakePrism aPrismMaker = new(theProfile, theDirection);
        if (!aPrismMaker.IsDone)
        {
            return new Shape();
        }
        return aPrismMaker.Shape();
    }

    /// <summary>Create simple rectangular prism (like TCL profile operations).</summary>
    public static Shape CreateRectangularPrism(in Point3D theCorner, double theX, double theY, double theZ)
    {
        Wire aWire = CreateRectangularProfile(theCorner, theX, theY);
        Shape aFace = CreateFaceFromWire(aWire);
        return CreatePrism(aFace, new Vector3D(0, 0, theZ));
    }

    /// <summary>Create revolution of a profile around an axis.</summary>
    public static Shape CreateRevolution(in Shape theProfile, in Axis3D theAxis, double theAngle)
    {
        ApiMakeRevol aRevolMaker = new(theProfile, theAxis, theAngle);
        Assert.True(aRevolMaker.IsDone, "Revolution operation failed");
        return aRevolMaker.Shape();
    }

    /// <summary>Get a face from a shape by index (for nexplode simulation).</summary>
    public static Face GetFaceByIndex(in Shape theShape, int theIndex)
    {
        Explorer anExp = new(theShape, ShapeType.Face);
        int aCurrentIndex = 1;

        while (anExp.More())
        {
            if (aCurrentIndex == theIndex)
            {
                return ShapeCasts.Face(anExp.Current());
            }
            aCurrentIndex++;
            anExp.Next();
        }
        return new Face(); // Return empty face if not found
    }

    /// <summary>Apply rotation around Z-axis (equivalent to "trotate shape 0 0 0 0 0 1 angle").</summary>
    public static Shape RotateZ(in Shape theShape, double theAngleDeg)
    {
        Transform3D aRotation = new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), theAngleDeg * M_PI / 180.0);
        Transform aTransform = new(theShape, aRotation);
        return aTransform.Shape();
    }

    /// <summary>Apply rotation around Y-axis (equivalent to "trotate shape 0 0 0 0 1 0 angle").</summary>
    public static Shape RotateY(in Shape theShape, double theAngleDeg)
    {
        Transform3D aRotation = new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Y)), theAngleDeg * M_PI / 180.0);
        Transform aTransform = new(theShape, aRotation);
        return aTransform.Shape();
    }

    /// <summary>Apply rotation around X-axis (equivalent to "trotate shape 0 0 0 1 0 0 angle").</summary>
    public static Shape RotateX(in Shape theShape, double theAngleDeg)
    {
        Transform3D aRotation = new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.X)), theAngleDeg * M_PI / 180.0);
        Transform aTransform = new(theShape, aRotation);
        return aTransform.Shape();
    }

    /// <summary>Apply common test rotation: Z(-90deg) then Y(-45deg) - used in many TCL tests.</summary>
    public static Shape RotateStandard(in Shape theShape)
    {
        Shape aResult = RotateZ(theShape, -90.0);
        return RotateY(aResult, -45.0);
    }

    /// <summary>Apply translation (equivalent to "ttranslate shape dx dy dz").</summary>
    public static Shape Translate(in Shape theShape, double theDx, double theDy, double theDz)
    {
        Transform3D aTranslation = new Transform3D().WithTranslation(new Vector3D(theDx, theDy, theDz));
        Transform aTransform = new(theShape, aTranslation);
        return aTransform.Shape();
    }

    /// <summary>Apply Y-axis 90-degree rotation (common pattern: "trotate shape 0 0 1 0 1 0 90").</summary>
    public static Shape RotateY90(in Shape theShape)
    {
        Transform3D aRotation = new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 1), new Direction3D(Direction3D.D.Y)), 90.0 * M_PI / 180.0);
        Transform aTransform = new(theShape, aRotation);
        return aTransform.Shape();
    }

    /// <summary>Create unit sphere and box pair (most common test setup).</summary>
    public static void CreateSphereAndBox(out Shape theSphere, out Shape theBox)
    {
        theSphere = CreateUnitSphere();
        theBox = CreateUnitBox();
    }

    /// <summary>Create two identical unit boxes at origin (common BOP test setup).</summary>
    public static void CreateIdenticalBoxes(out Shape theBox1, out Shape theBox2)
    {
        theBox1 = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
        theBox2 = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
    }

    /// <summary>Create NURBS box and regular box pair (common B-series test pattern).</summary>
    public static void CreateNurbsAndRegularBox(out Shape theNurbsBox, out Shape theRegularBox,
                                                Point3D? theNurbsCorner = null, Point3D? theRegularCorner = null,
                                                double theNurbsX = 1.0, double theNurbsY = 1.0, double theNurbsZ = 1.0,
                                                double theRegularX = 1.0, double theRegularY = 0.5, double theRegularZ = 1.0)
    {
        theNurbsBox = CreateBox(theNurbsCorner ?? new Point3D(0, 0, 0), theNurbsX, theNurbsY, theNurbsZ);
        theNurbsBox = ConvertToNurbs(theNurbsBox);

        theRegularBox = CreateBox(theRegularCorner ?? new Point3D(0, 1, 0), theRegularX, theRegularY, theRegularZ);
    }

    // CreateBlend (BRepFilletAPI_MakeFillet with DRAW's blend parameters) is left out: the 3D fillets are phase 7's.
    internal const string NoFillets = "BOPTest_Utilities::CreateBlend needs BRepFilletAPI_MakeFillet (phase 7)";

    /// <summary>Create a cylinder on a specified plane (like TCL pcylinder command).</summary>
    public static Shape CreateCylinderOnPlane(in Plane3D thePlane, double theRadius, double theHeight)
    {
        // Use DRAW pcylinder approach exactly:
        // S = BRepPrimAPI_MakeCylinder(P->Pln().Position().Ax2(), radius, height);
        Frame3D anAx2 = thePlane.Position.Frame;

        ApiMakeCylinder aCylinderMaker = new(anAx2, theRadius, theHeight);
        aCylinderMaker.Build();

        if (!aCylinderMaker.IsDone)
        {
            return new Shape();
        }

        return aCylinderMaker.Shape();
    }
}

/// <summary>OCCT's base test class for BRepAlgoAPI operations (BOPTest_Utilities.pxx's <c>BRepAlgoAPI_TestBase</c>).</summary>
public abstract class BRepAlgoAPI_TestBase
{
    protected double myTolerance = BOPTest_Utilities.DefaultTolerance();

    /// <summary>Perform BRepAlgoAPI Cut operation.</summary>
    protected static Shape PerformCut(in Shape theObject, in Shape theTool)
    {
        Cut aCutter = new(theObject, theTool);
        Assert.True(aCutter.IsDone, "BRepAlgoAPI_Cut operation failed");
        return aCutter.Shape();
    }

    /// <summary>Perform BRepAlgoAPI Fuse operation.</summary>
    protected static Shape PerformFuse(in Shape theShape1, in Shape theShape2)
    {
        Fuse aFuser = new(theShape1, theShape2);
        Assert.True(aFuser.IsDone, "BRepAlgoAPI_Fuse operation failed");
        return aFuser.Shape();
    }

    /// <summary>Perform BRepAlgoAPI Common operation.</summary>
    protected static Shape PerformCommon(in Shape theShape1, in Shape theShape2)
    {
        Common aCommoner = new(theShape1, theShape2);
        Assert.True(aCommoner.IsDone, "BRepAlgoAPI_Common operation failed");
        return aCommoner.Shape();
    }

    /// <summary>Validate result properties against expected values.</summary>
    protected void ValidateResult(in Shape theResult, double theExpectedSurfaceArea = -1.0,
                                  double theExpectedVolume = -1.0, bool theExpectedEmpty = false)
    {
        if (theExpectedEmpty)
        {
            Assert.True(BOPTest_Utilities.IsEmpty(theResult, myTolerance), "Result should be empty");
            return;
        }

        Assert.False(theResult.IsNull, "Result shape should not be null");

        if (theExpectedSurfaceArea >= 0.0)
        {
            double aSurfaceArea = BOPTest_Utilities.GetSurfaceArea(theResult);
            Assert.Equal(theExpectedSurfaceArea, aSurfaceArea, 5000.0); // Surface area mismatch
        }

        if (theExpectedVolume >= 0.0)
        {
            double aVolume = BOPTest_Utilities.GetVolume(theResult);
            Assert.Equal(theExpectedVolume, aVolume, myTolerance); // Volume mismatch
        }
    }
}
