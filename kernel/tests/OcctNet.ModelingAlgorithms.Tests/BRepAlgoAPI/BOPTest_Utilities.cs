// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests.BRepAlgoAPI;

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
    public static double GetSurfaceArea(in TopoDS_Shape theShape)
    {
        if (theShape.IsNull())
        {
            return 0.0;
        }
        GProp_GProps aProps = new();
        BRepGProp.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    /// <summary>Calculate volume of a shape.</summary>
    public static double GetVolume(in TopoDS_Shape theShape)
    {
        if (theShape.IsNull())
        {
            return 0.0;
        }
        GProp_GProps aProps = new();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    /// <summary>Check if shape is effectively empty (very small surface area).</summary>
    public static bool IsEmpty(in TopoDS_Shape theShape, double theTolerance = 1.0e-6) =>
        GetSurfaceArea(theShape) <= theTolerance;

    /// <summary>Create unit box at origin (1x1x1).</summary>
    public static TopoDS_Shape CreateUnitBox() => new BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

    /// <summary>Create box at specific location.</summary>
    public static TopoDS_Shape CreateBox(in gp_Pnt theCorner, double theX, double theY, double theZ) =>
        new BRepPrimAPI_MakeBox(theCorner, theX, theY, theZ).Shape();

    /// <summary>Create unit sphere (radius = 1.0).</summary>
    public static TopoDS_Shape CreateUnitSphere() => new BRepPrimAPI_MakeSphere(1.0).Shape();

    /// <summary>Create sphere at location with radius.</summary>
    public static TopoDS_Shape CreateSphere(in gp_Pnt theCenter, double theRadius) =>
        new BRepPrimAPI_MakeSphere(theCenter, theRadius).Shape();

    /// <summary>Create cylinder.</summary>
    public static TopoDS_Shape CreateCylinder(double theRadius, double theHeight) =>
        new BRepPrimAPI_MakeCylinder(theRadius, theHeight).Shape();

    /// <summary>Create cone.</summary>
    public static TopoDS_Shape CreateCone(double theR1, double theR2, double theHeight) =>
        new BRepPrimAPI_MakeCone(theR1, theR2, theHeight).Shape();

    /// <summary>Convert shape to NURBS.</summary>
    public static TopoDS_Shape ConvertToNurbs(in TopoDS_Shape theShape)
    {
        BRepBuilderAPI_NurbsConvert aNurbsConverter = new(theShape);
        if (!aNurbsConverter.IsDone())
        {
            return new TopoDS_Shape(); // Return null shape on failure
        }
        return aNurbsConverter.Shape();
    }

    /// <summary>Create rectangular polygon face.</summary>
    public static TopoDS_Shape CreatePolygonFace(in gp_Pnt theP1, in gp_Pnt theP2, in gp_Pnt theP3, in gp_Pnt theP4)
    {
        BRepBuilderAPI_MakePolygon aPolygonMaker = new();
        aPolygonMaker.Add(theP1);
        aPolygonMaker.Add(theP2);
        aPolygonMaker.Add(theP3);
        aPolygonMaker.Add(theP4);
        aPolygonMaker.Close();

        if (!aPolygonMaker.IsDone())
        {
            return new TopoDS_Shape();
        }

        BRepBuilderAPI_MakeFace aFaceMaker = new(aPolygonMaker.Wire());
        return aFaceMaker.Shape();
    }

    /// <summary>Rotate shape around axis.</summary>
    public static TopoDS_Shape RotateShape(in TopoDS_Shape theShape, in gp_Ax1 theAxis, double theAngle)
    {
        gp_Trsf aTrsf = new();
        aTrsf.SetRotation(theAxis, theAngle);
        BRepBuilderAPI_Transform aTransformer = new(theShape, aTrsf);
        return aTransformer.Shape();
    }

    /// <summary>Translate shape by vector.</summary>
    public static TopoDS_Shape TranslateShape(in TopoDS_Shape theShape, in gp_Vec theVector)
    {
        gp_Trsf aTrsf = new();
        aTrsf.SetTranslation(theVector);
        BRepBuilderAPI_Transform aTransformer = new(theShape, aTrsf);
        return aTransformer.Shape();
    }

    /// <summary>Create vertex from point.</summary>
    public static TopoDS_Vertex CreateVertex(in gp_Pnt thePoint) => new BRepBuilderAPI_MakeVertex(thePoint).Vertex();

    /// <summary>Create edge between two points.</summary>
    public static TopoDS_Edge CreateEdge(in gp_Pnt theP1, in gp_Pnt theP2) => new BRepBuilderAPI_MakeEdge(theP1, theP2).Edge();

    /// <summary>Create wire from list of points (closed polygon).</summary>
    public static TopoDS_Wire CreatePolygonWire(IReadOnlyList<gp_Pnt> thePoints, bool theClose = true)
    {
        BRepBuilderAPI_MakePolygon aPolygonMaker = new();
        foreach (gp_Pnt aPt in thePoints)
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
    public static TopoDS_Wire CreateProfileWire(in gp_Pln thePlane, in gp_Pnt2d theStartPt, IReadOnlyList<string> theCommands)
    {
        // Simplified profile creation - create a rectangular wire based on commands
        // This approximates the TCL profile behavior for basic geometric operations
        List<gp_Pnt2d> aPoints = [theStartPt];

        gp_Pnt2d aCurrentPt = theStartPt;
        gp_Vec2d aCurrentDir = new(1.0, 0.0); // Default direction

        // Process simplified command set
        for (int i = 0; i < theCommands.Count; ++i)
        {
            string aCmd = theCommands[i];
            if (aCmd == "Y" && i + 1 < theCommands.Count)
            {
                // Y command: translate along Y
                double aDY = double.Parse(theCommands[i + 1], System.Globalization.CultureInfo.InvariantCulture);
                aCurrentPt.SetY(aCurrentPt.Y() + aDY);
                aPoints.Add(aCurrentPt);
                i++; // Skip next parameter
            }
            else if (aCmd == "C" && i + 2 < theCommands.Count)
            {
                // C command: arc (simplified as straight line for now)
                double aRadius = double.Parse(theCommands[i + 1], System.Globalization.CultureInfo.InvariantCulture);
                // Simplified: just move in current direction
                gp_Vec2d aMoveVec = aCurrentDir * aRadius;
                aCurrentPt.Translate(aMoveVec);
                aPoints.Add(aCurrentPt);
                i += 2; // Skip next two parameters
            }
        }

        // Convert 2D points to 3D points on the plane
        List<gp_Pnt> a3DPoints = [];
        foreach (gp_Pnt2d aPt2d in aPoints)
        {
            a3DPoints.Add(ElSLib.Value(aPt2d.X(), aPt2d.Y(), thePlane));
        }

        return CreatePolygonWire(a3DPoints, true);
    }

    /// <summary>
    /// Create profile using typed commands (equivalent to TCL profile). plane: the reference plane/face (equivalent to
    /// "S face" in TCL); operations: list of profile operations.
    /// </summary>
    public static TopoDS_Shape CreateProfile(in gp_Pln thePlane, IReadOnlyList<ProfileOperation> theOperations)
    {
        BRepBuilderAPI_MakeWire aMakeWire = new();

        gp_Pnt2d aCurrentPt = new(0, 0);  // Current point in 2D
        gp_Vec2d aCurrentDir = new(1, 0); // Current direction in 2D
        bool aFirstSet = false;
        gp_Pnt2d aFirstPt = new(0, 0);
        gp_Pln aWorkingPlane = thePlane; // Working plane that can be modified by P command

        foreach (ProfileOperation op in theOperations)
        {
            switch (op.cmd)
            {
                case ProfileCmd.O: // Set origin (affects the plane, not the starting point)
                    if (op.@params.Count >= 3)
                    {
                        // O command sets the origin of the plane coordinate system
                        gp_Pnt aNewOrigin = new(op.@params[0], op.@params[1], op.@params[2]);
                        gp_Ax3 aNewAx3 = new(aNewOrigin, aWorkingPlane.Axis().Direction(), aWorkingPlane.XAxis().Direction());
                        aWorkingPlane = new gp_Pln(aNewAx3);

                        // Profile still starts at (0,0) in the new plane coordinate system
                        if (!aFirstSet)
                        {
                            aCurrentPt.SetCoord(0.0, 0.0); // TCL default: start at (0,0)
                            aFirstPt = aCurrentPt;
                            aFirstSet = true;
                        }
                    }
                    break;

                case ProfileCmd.P: // Set plane (P nx ny nz dx dy dz)
                    if (op.@params.Count >= 6)
                    {
                        // P command sets plane normal and X direction vectors
                        gp_Vec aNormal = new(op.@params[0], op.@params[1], op.@params[2]);
                        gp_Vec aXDir = new(op.@params[3], op.@params[4], op.@params[5]);

                        // Create new coordinate system
                        gp_Dir aNormalDir = new(aNormal);
                        gp_Dir aXDirection = new(aXDir);

                        // Use the current plane's origin but new orientation
                        gp_Ax3 aNewAx3 = new(aWorkingPlane.Location(), aNormalDir, aXDirection);
                        aWorkingPlane = new gp_Pln(aNewAx3);
                    }
                    break;

                case ProfileCmd.F: // Set first point
                    if (op.@params.Count >= 2)
                    {
                        aCurrentPt.SetCoord(op.@params[0], op.@params[1]);
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
                            aCurrentPt.SetCoord(0.0, 0.0);
                            aFirstPt = aCurrentPt;
                            aFirstSet = true;
                        }

                        gp_Pnt2d aNewPt = new(aCurrentPt.X() + op.@params[0], aCurrentPt.Y());
                        // Add line segment
                        gp_Pnt aPt1 = ElSLib.Value(aCurrentPt.X(), aCurrentPt.Y(), aWorkingPlane);
                        gp_Pnt aPt2 = ElSLib.Value(aNewPt.X(), aNewPt.Y(), aWorkingPlane);
                        aMakeWire.Add(new BRepBuilderAPI_MakeEdge(aPt1, aPt2).Edge());
                        aCurrentPt = aNewPt;
                        aCurrentDir = new gp_Vec2d(op.@params[0] > 0 ? 1 : -1, 0);
                    }
                    break;

                case ProfileCmd.Y: // Translate along Y
                    if (op.@params.Count != 0)
                    {
                        // If first point not set, implicitly start at (0,0)
                        if (!aFirstSet)
                        {
                            aCurrentPt.SetCoord(0.0, 0.0);
                            aFirstPt = aCurrentPt;
                            aFirstSet = true;
                        }

                        gp_Pnt2d aNewPt = new(aCurrentPt.X(), aCurrentPt.Y() + op.@params[0]);
                        // Add line segment
                        gp_Pnt aPt1 = ElSLib.Value(aCurrentPt.X(), aCurrentPt.Y(), aWorkingPlane);
                        gp_Pnt aPt2 = ElSLib.Value(aNewPt.X(), aNewPt.Y(), aWorkingPlane);
                        aMakeWire.Add(new BRepBuilderAPI_MakeEdge(aPt1, aPt2).Edge());
                        aCurrentPt = aNewPt;
                        aCurrentDir = new gp_Vec2d(0, op.@params[0] > 0 ? 1 : -1);
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
                            gp_Pnt2d aCenter2D = aFirstSet ? aCurrentPt : new gp_Pnt2d(0, 0);
                            gp_Pnt aCenter3D = ElSLib.Value(aCenter2D.X(), aCenter2D.Y(), aWorkingPlane);

                            // Create full circle
                            gp_Circ aCirc = new(new gp_Ax2(aCenter3D, aWorkingPlane.Axis().Direction()), aRadius);
                            aMakeWire.Add(new BRepBuilderAPI_MakeEdge(aCirc).Edge());

                            // Set current point to a point on the circle
                            aCurrentPt = new gp_Pnt2d(aCenter2D.X() + aRadius, aCenter2D.Y());
                            aCurrentDir = new gp_Vec2d(0, 1); // Tangent direction
                            aFirstSet = true;
                        }
                        else
                        {
                            // If first point not set, start at origin with default direction
                            if (!aFirstSet)
                            {
                                aCurrentPt.SetCoord(0.0, 0.0);
                                aCurrentDir = new gp_Vec2d(1, 0); // Default direction: positive X
                                aFirstPt = aCurrentPt;
                                aFirstSet = true;
                            }

                            // Create arc from current point
                            gp_Vec2d aNormal = new(-aCurrentDir.Y(), aCurrentDir.X());
                            gp_Pnt2d aCenter = aCurrentPt.Translated(aNormal * aRadius);

                            // Calculate end point
                            double aStartAngle = Math.Atan2(aCurrentPt.Y() - aCenter.Y(), aCurrentPt.X() - aCenter.X());
                            double aEndAngle = aStartAngle + aAngleRad;
                            gp_Pnt2d aEndPt = new(aCenter.X() + aRadius * Math.Cos(aEndAngle), aCenter.Y() + aRadius * Math.Sin(aEndAngle));

                            // Create 3D arc
                            gp_Pnt aPt1 = ElSLib.Value(aCurrentPt.X(), aCurrentPt.Y(), aWorkingPlane);
                            gp_Pnt aPt2 = ElSLib.Value(aEndPt.X(), aEndPt.Y(), aWorkingPlane);
                            gp_Pnt aCenter3D = ElSLib.Value(aCenter.X(), aCenter.Y(), aWorkingPlane);

                            gp_Circ aCirc = new(new gp_Ax2(aCenter3D, aWorkingPlane.Axis().Direction()), aRadius);
                            aMakeWire.Add(new BRepBuilderAPI_MakeEdge(aCirc, aPt1, aPt2).Edge());

                            aCurrentPt = aEndPt;
                            aCurrentDir = new gp_Vec2d(Math.Cos(aEndAngle + M_PI / 2), Math.Sin(aEndAngle + M_PI / 2));
                        }
                    }
                    break;

                case ProfileCmd.D: // Set direction vector
                    if (op.@params.Count >= 2)
                    {
                        aCurrentDir = new gp_Vec2d(op.@params[0], op.@params[1]);
                        aCurrentDir.Normalize();
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
        if (aFirstSet && !aCurrentPt.IsEqual(aFirstPt, Precision.Confusion()))
        {
            // Add closing edge back to start
            gp_Pnt aPt1 = ElSLib.Value(aCurrentPt.X(), aCurrentPt.Y(), aWorkingPlane);
            gp_Pnt aPt2 = ElSLib.Value(aFirstPt.X(), aFirstPt.Y(), aWorkingPlane);
            aMakeWire.Add(new BRepBuilderAPI_MakeEdge(aPt1, aPt2).Edge());
        }

        Assert.True(aMakeWire.IsDone(), "Profile wire creation failed");
        TopoDS_Wire aWire = aMakeWire.Wire();

        // Create face from wire
        BRepBuilderAPI_MakeFace aFaceMaker = new(aWire);
        Assert.True(aFaceMaker.IsDone(), "Profile face creation failed");
        return aFaceMaker.Face();
    }

    /// <summary>Create profile from operations list (simplified version).</summary>
    public static TopoDS_Shape CreateProfileFromOperations(IReadOnlyList<ProfileOperation> theOperations)
    {
        // Use default plane (XY plane)
        gp_Pln aPlane = new(new gp_Pnt(0, 0, 0), new gp_Dir(gp_Dir.D.Z));
        return CreateProfile(aPlane, theOperations);
    }

    /// <summary>Create rectangular profile from corner and dimensions.</summary>
    public static TopoDS_Wire CreateRectangularProfile(in gp_Pnt theCorner, double theX, double theY)
    {
        List<gp_Pnt> aPoints =
        [
            theCorner,
            new gp_Pnt(theCorner.X() + theX, theCorner.Y(), theCorner.Z()),
            new gp_Pnt(theCorner.X() + theX, theCorner.Y() + theY, theCorner.Z()),
            new gp_Pnt(theCorner.X(), theCorner.Y() + theY, theCorner.Z()),
        ];
        return CreatePolygonWire(aPoints, true);
    }

    /// <summary>Create face from wire.</summary>
    public static TopoDS_Shape CreateFaceFromWire(in TopoDS_Wire theWire)
    {
        BRepBuilderAPI_MakeFace aFaceMaker = new(theWire);
        if (!aFaceMaker.IsDone())
        {
            return new TopoDS_Shape();
        }
        return aFaceMaker.Shape();
    }

    /// <summary>Create prism by extruding shape along vector.</summary>
    public static TopoDS_Shape CreatePrism(in TopoDS_Shape theProfile, in gp_Vec theDirection)
    {
        BRepPrimAPI_MakePrism aPrismMaker = new(theProfile, theDirection);
        if (!aPrismMaker.IsDone())
        {
            return new TopoDS_Shape();
        }
        return aPrismMaker.Shape();
    }

    /// <summary>Create simple rectangular prism (like TCL profile operations).</summary>
    public static TopoDS_Shape CreateRectangularPrism(in gp_Pnt theCorner, double theX, double theY, double theZ)
    {
        TopoDS_Wire aWire = CreateRectangularProfile(theCorner, theX, theY);
        TopoDS_Shape aFace = CreateFaceFromWire(aWire);
        return CreatePrism(aFace, new gp_Vec(0, 0, theZ));
    }

    /// <summary>Create revolution of a profile around an axis.</summary>
    public static TopoDS_Shape CreateRevolution(in TopoDS_Shape theProfile, in gp_Ax1 theAxis, double theAngle)
    {
        BRepPrimAPI_MakeRevol aRevolMaker = new(theProfile, theAxis, theAngle);
        Assert.True(aRevolMaker.IsDone(), "Revolution operation failed");
        return aRevolMaker.Shape();
    }

    /// <summary>Get a face from a shape by index (for nexplode simulation).</summary>
    public static TopoDS_Face GetFaceByIndex(in TopoDS_Shape theShape, int theIndex)
    {
        TopExp_Explorer anExp = new(theShape, TopAbs_FACE);
        int aCurrentIndex = 1;

        while (anExp.More())
        {
            if (aCurrentIndex == theIndex)
            {
                return TopoDS.Face(anExp.Current());
            }
            aCurrentIndex++;
            anExp.Next();
        }
        return new TopoDS_Face(); // Return empty face if not found
    }

    /// <summary>Apply rotation around Z-axis (equivalent to "trotate shape 0 0 0 0 0 1 angle").</summary>
    public static TopoDS_Shape RotateZ(in TopoDS_Shape theShape, double theAngleDeg)
    {
        gp_Trsf aRotation = new();
        aRotation.SetRotation(new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(gp_Dir.D.Z)), theAngleDeg * M_PI / 180.0);
        BRepBuilderAPI_Transform aTransform = new(theShape, aRotation);
        return aTransform.Shape();
    }

    /// <summary>Apply rotation around Y-axis (equivalent to "trotate shape 0 0 0 0 1 0 angle").</summary>
    public static TopoDS_Shape RotateY(in TopoDS_Shape theShape, double theAngleDeg)
    {
        gp_Trsf aRotation = new();
        aRotation.SetRotation(new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(gp_Dir.D.Y)), theAngleDeg * M_PI / 180.0);
        BRepBuilderAPI_Transform aTransform = new(theShape, aRotation);
        return aTransform.Shape();
    }

    /// <summary>Apply rotation around X-axis (equivalent to "trotate shape 0 0 0 1 0 0 angle").</summary>
    public static TopoDS_Shape RotateX(in TopoDS_Shape theShape, double theAngleDeg)
    {
        gp_Trsf aRotation = new();
        aRotation.SetRotation(new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(gp_Dir.D.X)), theAngleDeg * M_PI / 180.0);
        BRepBuilderAPI_Transform aTransform = new(theShape, aRotation);
        return aTransform.Shape();
    }

    /// <summary>Apply common test rotation: Z(-90deg) then Y(-45deg) - used in many TCL tests.</summary>
    public static TopoDS_Shape RotateStandard(in TopoDS_Shape theShape)
    {
        TopoDS_Shape aResult = RotateZ(theShape, -90.0);
        return RotateY(aResult, -45.0);
    }

    /// <summary>Apply translation (equivalent to "ttranslate shape dx dy dz").</summary>
    public static TopoDS_Shape Translate(in TopoDS_Shape theShape, double theDx, double theDy, double theDz)
    {
        gp_Trsf aTranslation = new();
        aTranslation.SetTranslation(new gp_Vec(theDx, theDy, theDz));
        BRepBuilderAPI_Transform aTransform = new(theShape, aTranslation);
        return aTransform.Shape();
    }

    /// <summary>Apply Y-axis 90-degree rotation (common pattern: "trotate shape 0 0 1 0 1 0 90").</summary>
    public static TopoDS_Shape RotateY90(in TopoDS_Shape theShape)
    {
        gp_Trsf aRotation = new();
        aRotation.SetRotation(new gp_Ax1(new gp_Pnt(0, 0, 1), new gp_Dir(gp_Dir.D.Y)), 90.0 * M_PI / 180.0);
        BRepBuilderAPI_Transform aTransform = new(theShape, aRotation);
        return aTransform.Shape();
    }

    /// <summary>Create unit sphere and box pair (most common test setup).</summary>
    public static void CreateSphereAndBox(out TopoDS_Shape theSphere, out TopoDS_Shape theBox)
    {
        theSphere = CreateUnitSphere();
        theBox = CreateUnitBox();
    }

    /// <summary>Create two identical unit boxes at origin (common BOP test setup).</summary>
    public static void CreateIdenticalBoxes(out TopoDS_Shape theBox1, out TopoDS_Shape theBox2)
    {
        theBox1 = CreateBox(new gp_Pnt(0, 0, 0), 1.0, 1.0, 1.0);
        theBox2 = CreateBox(new gp_Pnt(0, 0, 0), 1.0, 1.0, 1.0);
    }

    /// <summary>Create NURBS box and regular box pair (common B-series test pattern).</summary>
    public static void CreateNurbsAndRegularBox(out TopoDS_Shape theNurbsBox, out TopoDS_Shape theRegularBox,
                                                gp_Pnt? theNurbsCorner = null, gp_Pnt? theRegularCorner = null,
                                                double theNurbsX = 1.0, double theNurbsY = 1.0, double theNurbsZ = 1.0,
                                                double theRegularX = 1.0, double theRegularY = 0.5, double theRegularZ = 1.0)
    {
        theNurbsBox = CreateBox(theNurbsCorner ?? new gp_Pnt(0, 0, 0), theNurbsX, theNurbsY, theNurbsZ);
        theNurbsBox = ConvertToNurbs(theNurbsBox);

        theRegularBox = CreateBox(theRegularCorner ?? new gp_Pnt(0, 1, 0), theRegularX, theRegularY, theRegularZ);
    }

    // CreateBlend (BRepFilletAPI_MakeFillet with DRAW's blend parameters) is left out: the 3D fillets are phase 7's.
    internal const string NoFillets = "BOPTest_Utilities::CreateBlend needs BRepFilletAPI_MakeFillet (phase 7)";

    /// <summary>Create a cylinder on a specified plane (like TCL pcylinder command).</summary>
    public static TopoDS_Shape CreateCylinderOnPlane(in gp_Pln thePlane, double theRadius, double theHeight)
    {
        // Use DRAW pcylinder approach exactly:
        // S = BRepPrimAPI_MakeCylinder(P->Pln().Position().Ax2(), radius, height);
        gp_Ax2 anAx2 = thePlane.Position().Ax2();

        BRepPrimAPI_MakeCylinder aCylinderMaker = new(anAx2, theRadius, theHeight);
        aCylinderMaker.Build();

        if (!aCylinderMaker.IsDone())
        {
            return new TopoDS_Shape();
        }

        return aCylinderMaker.Shape();
    }
}

/// <summary>OCCT's base test class for BRepAlgoAPI operations (BOPTest_Utilities.pxx's <c>BRepAlgoAPI_TestBase</c>).</summary>
public abstract class BRepAlgoAPI_TestBase
{
    protected double myTolerance = BOPTest_Utilities.DefaultTolerance();

    /// <summary>Perform BRepAlgoAPI Cut operation.</summary>
    protected static TopoDS_Shape PerformCut(in TopoDS_Shape theObject, in TopoDS_Shape theTool)
    {
        BRepAlgoAPI_Cut aCutter = new(theObject, theTool);
        Assert.True(aCutter.IsDone(), "BRepAlgoAPI_Cut operation failed");
        return aCutter.Shape();
    }

    /// <summary>Perform BRepAlgoAPI Fuse operation.</summary>
    protected static TopoDS_Shape PerformFuse(in TopoDS_Shape theShape1, in TopoDS_Shape theShape2)
    {
        BRepAlgoAPI_Fuse aFuser = new(theShape1, theShape2);
        Assert.True(aFuser.IsDone(), "BRepAlgoAPI_Fuse operation failed");
        return aFuser.Shape();
    }

    /// <summary>Perform BRepAlgoAPI Common operation.</summary>
    protected static TopoDS_Shape PerformCommon(in TopoDS_Shape theShape1, in TopoDS_Shape theShape2)
    {
        BRepAlgoAPI_Common aCommoner = new(theShape1, theShape2);
        Assert.True(aCommoner.IsDone(), "BRepAlgoAPI_Common operation failed");
        return aCommoner.Shape();
    }

    /// <summary>Validate result properties against expected values.</summary>
    protected void ValidateResult(in TopoDS_Shape theResult, double theExpectedSurfaceArea = -1.0,
                                  double theExpectedVolume = -1.0, bool theExpectedEmpty = false)
    {
        if (theExpectedEmpty)
        {
            Assert.True(BOPTest_Utilities.IsEmpty(theResult, myTolerance), "Result should be empty");
            return;
        }

        Assert.False(theResult.IsNull(), "Result shape should not be null");

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
