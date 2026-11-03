// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepOffsetAPI_ThruSections_Test.cxx (OCCT V8_0_1), with OCCT.NET's
// checks of frozen inputs, the history, BRepFill_Generator and BRepProj_Projection added.

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Schematiq.Collections;
using static Occt.BRepFill_ThruSectionErrorStatus;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepOffsetAPI_ThruSections, BRepFill_Generator and BRepProj_Projection (unit 5T): OCCT's GTests, and the port's
/// frozen inputs and history.
/// </summary>
public class BRepOffsetAPI_ThruSectionsTests
{
    private static string Text(in TopoDS_Shape theShape)
    {
        using StringWriter aW = new();
        BRepTools.Write(theShape, aW);
        return aW.ToString();
    }

    private static TopoDS_Wire Polygon(params Point3D[] thePoints)
    {
        BRepBuilderAPI_MakePolygon aPolygon = new();
        foreach (Point3D aP in thePoints)
        {
            aPolygon.Add(aP);
        }
        aPolygon.Close();
        return aPolygon.Wire();
    }

    private static TopoDS_Wire Square(double theZ, double theHalf) =>
        Polygon(new Point3D(-theHalf, -theHalf, theZ), new Point3D(theHalf, -theHalf, theZ), new Point3D(theHalf, theHalf, theZ),
                new Point3D(-theHalf, theHalf, theZ));

    private static TopoDS_Wire Circle(double theZ, double theRadius) =>
        new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(new Circle3D(new Frame3D(new Point3D(0, 0, theZ), new Direction3D(0, 0, 1)), theRadius)).Edge()).Wire();

    private static double Area(in TopoDS_Shape theShape)
    {
        GProp_GProps aProps = new();
        BRepGProp.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static double Volume(in TopoDS_Shape theShape)
    {
        GProp_GProps aProps = new();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        NCollection_IndexedMap<TopoDS_Shape> aMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    // ---- OCCT's GTests ----

    // Test OCC10006: BRepOffsetAPI_ThruSections loft operation with Boolean fusion
    [Fact]
    public void OCC10006_LoftAndFusion()
    {
        // Define bottom and top polygon coordinates for first loft
        double[] aBottomPoints1 = [10, -10, 0, 100, -10, 0, 100, -100, 0, 10, -100, 0];
        double[] aTopPoints1 = [0, 0, 10, 100, 0, 10, 100, -100, 10, 0, -100, 10];

        // Define bottom and top polygon coordinates for second loft
        double[] aBottomPoints2 = [0, 0, 10.00, 100, 0, 10.00, 100, -100, 10.00, 0, -100, 10.00];
        double[] aTopPoints2 = [0, 0, 250, 100, 0, 250, 100, -100, 250, 0, -100, 250];

        // Create polygons
        BRepBuilderAPI_MakePolygon aBottomPolygon1 = new(), aTopPolygon1 = new(), aBottomPolygon2 = new(), aTopPolygon2 = new();
        for (int i = 0; i < 4; i++)
        {
            aBottomPolygon1.Add(new Point3D(aBottomPoints1[3 * i], aBottomPoints1[3 * i + 1], aBottomPoints1[3 * i + 2]));
            aTopPolygon1.Add(new Point3D(aTopPoints1[3 * i], aTopPoints1[3 * i + 1], aTopPoints1[3 * i + 2]));
            aBottomPolygon2.Add(new Point3D(aBottomPoints2[3 * i], aBottomPoints2[3 * i + 1], aBottomPoints2[3 * i + 2]));
            aTopPolygon2.Add(new Point3D(aTopPoints2[3 * i], aTopPoints2[3 * i + 1], aTopPoints2[3 * i + 2]));
        }

        // Close polygons
        aBottomPolygon1.Close();
        aTopPolygon1.Close();
        aBottomPolygon2.Close();
        aTopPolygon2.Close();

        // Create first loft (ThruSections)
        BRepOffsetAPI_ThruSections aLoft1 = new(true, true);
        aLoft1.AddWire(aBottomPolygon1.Wire());
        aLoft1.AddWire(aTopPolygon1.Wire());
        aLoft1.Build();

        // Create second loft (ThruSections)
        BRepOffsetAPI_ThruSections aLoft2 = new(true, true);
        aLoft2.AddWire(aBottomPolygon2.Wire());
        aLoft2.AddWire(aTopPolygon2.Wire());
        aLoft2.Build();

        // Verify that loft operations succeeded
        Assert.False(aLoft1.Shape().IsNull());
        Assert.False(aLoft2.Shape().IsNull());

        // Perform Boolean fusion of the two lofted shapes
        BRepAlgoAPI_Fuse aFusion = new(aLoft1.Shape(), aLoft2.Shape());

        // Verify that fusion operation succeeded
        Assert.False(aFusion.Shape().IsNull());
    }

    //! Helper function to create a B-spline curve from poles and knot sequence.
    private static Geom_BSplineCurve createBSplineCurve(double[] thePoles, double[] theKnots)
    {
        int aKnotSeqSize = theKnots.Length;
        const int aDegree = 3; // cubic spline
        int aNbPoles = aKnotSeqSize - aDegree - 1;

        NCollection_Array1<Point3D> aPoles = new(1, aNbPoles);
        int anIdx = 0;
        for (int i = 1; i <= aNbPoles; ++i)
        {
            aPoles[i] = new Point3D(thePoles[anIdx], thePoles[anIdx + 1], thePoles[anIdx + 2]);
            anIdx += 3;
        }

        NCollection_Array1<double> aKnotSeq = new(1, aKnotSeqSize);
        for (int i = 1; i <= aKnotSeqSize; ++i)
        {
            aKnotSeq[i] = theKnots[i - 1];
        }

        NCollection_Array1<double> aKnots = new(1, global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotsLength(aKnotSeq, false));
        NCollection_Array1<int> aMults = new(1, aKnots.Upper());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Knots(aKnotSeq, aKnots, aMults);

        return new Geom_BSplineCurve(aPoles, aKnots, aMults, aDegree, false);
    }

    private static readonly double[] aPoles1 =
    [
        0.90194, -0.49457, 0, 1.10106097848166, -0.688963767263419, 0,
        1.04668209568152, -1.01716787534971, 0, 1.56914438377061, -1.59514757777645, 0,
        2.07302273729763, -3.97023193652984, 0, 2.13699279206564, -4.57936580650492, 0,
        2.13184140893145, -7.00174300027487, 0, 1.23454863419269, -8.49296048646617, 0,
        0.718149935438171, -9.43440624002067, 0, -2.02215956293932, -12.0782514208969, 0,
        -3.44515644568264, -13.0526565862391, 0, -8.61587484541011, -15.7788974963508, 0,
        -12.8309565197945, -17.0127003407937, 0, -18.9761857527559, -18.432946752008, 0,
        -20.6360623744536, -18.7727324951302, 0, -21.2130359371451, -19.0009746915956, 0,
        -22.2663547655952, -18.9176136571682, 0, -22.1384066610996, -17.5929745686915, 0,
        -21.7954793416826, -18.0629411635879, 0, -14.4780597089423, -15.0594484542493, 0,
        -10.8386889762912, -13.1323128331196, 0, -6.48208180266811, -9.4594741816569, 0,
        -5.28863405147604, -8.21363647074186, 0, -2.96204585602755, -5.08386133056186, 0,
        -2.54058250866771, -4.20341306384795, 0, -1.61736353367643, -2.66854172662003, 0,
        -1.36012191071392, -0.569755235503059, 0, -1.02752582768388, -0.306835878255118, 0,
        0.166466683785465, 0.162094918019513, 0, 0.0600379384518613, 0.327344971788155, 0,
        0.90194, -0.49457, 0,
    ];

    private static readonly double[] aKnots1 =
    [
        0, 0, 0, 0, 0.0161348522840521, 0.0807608132035811,
        0.131338840314679, 0.147844505833154, 0.16533568293713, 0.250607290759724, 0.267449419858552, 0.322628298230969,
        0.341027430676258, 0.36078046495376, 0.420324490838906, 0.440253145913864, 0.458870520105636, 0.492435831240754,
        0.509078502457839, 0.525715578369451, 0.577446072383669, 0.597241246236863, 0.656453700919606, 0.676155157762965,
        0.694541842118733, 0.749834839741989, 0.766726765513781, 0.834662561102041, 0.868900689429001, 0.88456999851039,
        0.931780340251946, 1, 1, 1, 1,
    ];

    private static readonly double[] aPoles2 =
    [
        1.01818, -0.45183, 14, 1.23027550376089, -0.635831979821499, 14,
        1.22298178591092, -0.993247942942908, 14, 1.9247594565888, -1.68138513003473, 14,
        2.78453675046566, -4.24062036175799, 14, 2.92887039780498, -4.88871135922063, 14,
        3.21530512178939, -7.44536648753893, 14, 2.43464600049467, -9.01717535389148, 14,
        1.99774717699481, -10.0099731707865, 14, -0.580430194841712, -12.811667114046, 14,
        -1.96280131040106, -13.8497247564759, 14, -7.0770142802295, -16.7706677743417, 14,
        -11.3526244425869, -18.1132238752247, 14, -17.6322830947984, -19.667828036594, 14,
        -19.3359506088904, -20.0397164345371, 14, -19.9363407564968, -20.2618454245171, 14,
        -20.9958867740652, -20.2431659322596, 14, -20.9472778652509, -18.9170304109807, 14,
        -20.5705540974871, -19.3684485145849, 14, -13.1692875695181, -16.2161494354514, 14,
        -9.51921634418429, -14.2082484638036, 14, -5.35937404690226, -10.3089201842049, 14,
        -4.47798441471647, -9.23875034192191, 14, -3.21879123246707, -7.6110600629437, 14,
        -2.18689883513949, -5.31367859264728, 14, -1.97348761746698, -4.58783119756755, 14,
        -1.33574873445926, -3.01168469104978, 14, -1.71865526796731, -0.0846344433001956, 14,
        0.269132653252561, 0.122297343391581, 14, 0.122686280710628, 0.325049350788693, 14,
        1.01818, -0.45183, 14,
    ];

    private static readonly double[] aKnots2 =
    [
        0, 0, 0, 0, 0.0161348522840521, 0.0807608132035811,
        0.131338840314679, 0.147844505833154, 0.16533568293713, 0.250607290759724, 0.267449419858552, 0.322628298230969,
        0.341027430676258, 0.36078046495376, 0.420324490838906, 0.440253145913864, 0.458870520105636, 0.492435831240754,
        0.509078502457839, 0.525715578369451, 0.577446072383669, 0.597241246236863, 0.656453700919606, 0.676155157762965,
        0.694541842118733, 0.749834839741989, 0.766726765513781, 0.834662561102041, 0.868900689429001, 0.88456999851039,
        0.931780340251946, 1, 1, 1, 1,
    ];

    private static readonly double[] aPoles3 =
    [
        1.1315, -0.412, 28, 1.35620967405692, -0.586767016287049, 28,
        1.40346173744982, -0.904848944467879, 28, 2.42203198448737, -2.18621379588629, 28,
        3.32033694639709, -3.92656269181969, 28, 3.51212155543691, -4.32855460347163, 28,
        4.24437254399706, -7.7411305703627, 28, 3.58837706744425, -9.38948673956861, 28,
        3.23381768225783, -10.4288340403835, 28, 0.827297282546986, -13.3806847722879, 28,
        -0.511092431459457, -14.4804531232202, 28, -5.56121110916677, -17.5918940081576, 28,
        -9.89839343105239, -19.0417230070618, 28, -16.3191628184718, -20.7281818875723, 28,
        -18.069202037559, -21.1317454056832, 28, -18.6952658436501, -21.3501485512683, 28,
        -19.7594163372563, -21.3890021828607, 28, -19.7825452812777, -20.0662589434646, 28,
        -19.3742511072047, -20.4997648812408, 28, -11.8843871512175, -17.1947066102013, 28,
        -8.22814484532529, -15.0978242272864, 28, -4.27266763395487, -10.9862396235634, 28,
        -3.46908164394296, -9.8527181618549, 28, -2.33410525489297, -8.12592678971571, 28,
        -1.5676761966055, -5.65380588039358, 28, -1.45500694833621, -4.86219190572022, 28,
        -1.20491779902222, -2.99088321180665, 28, -1.71473210788581, -0.11550194997251, 28,
        0.339193684817586, 0.130842094688643, 28, 0.185038304272247, 0.324106655338832, 28,
        1.1315, -0.411999999999999, 28,
    ];

    private static readonly double[] aKnots3 =
    [
        0, 0, 0, 0, 0.0161348522840521, 0.0807608132035811,
        0.131338840314679, 0.147844505833154, 0.16533568293713, 0.250607290759724, 0.267449419858552, 0.322628298230969,
        0.341027430676258, 0.36078046495376, 0.420324490838906, 0.440253145913864, 0.458870520105636, 0.492435831240754,
        0.509078502457839, 0.525715578369451, 0.577446072383669, 0.597241246236863, 0.656453700919606, 0.676155157762965,
        0.694541842118733, 0.749834839741989, 0.766726765513781, 0.834662561102041, 0.868900689429001, 0.88456999851039,
        0.931780340251946, 1, 1, 1, 1,
    ];

    private static readonly double[] aPoles4 =
    [
        1.23665, -0.38035, 42, 1.47168196581999, -0.547085468398438, 42,
        1.59588093039332, -0.976910285968177, 42, 2.37359942123034, -1.62852397162461, 42,
        3.88462097653375, -4.06747989117973, 42, 4.08076351079016, -4.41252552269171, 42,
        5.13530795520565, -7.91791038549566, 42, 4.5942068788067, -9.62828143393584, 42,
        4.31440651520895, -10.7043545807689, 42, 2.06395892933396, -13.7788094086647, 42,
        0.765564553906658, -14.9299915868171, 42, -4.22675671524901, -18.2024292050444, 42,
        -8.62128212326746, -19.7458598662774, 42, -15.1728828553279, -21.5499125868338, 42,
        -16.9659291749178, -21.9822055955299, 42, -17.6161839495155, -22.1994311185674, 42,
        -18.6836955455801, -22.2848113284216, 42, -18.7654460877162, -20.9681980423141, 42,
        -18.33029903925, -21.3862202927921, 42, -10.7621667068909, -17.9386762111952, 42,
        -7.10729220900608, -15.7553189784139, 42, -3.34515330946479, -11.4622305098615, 42,
        -2.61347538922171, -10.2767882581575, 42, -1.59293712529107, -8.46932612376041, 42,
        -1.06722790049871, -5.86343234527305, 42, -1.05069435531815, -5.16187378699568, 42,
        -0.946257221489481, -3.39477994610666, 42, -1.94338925491643, -0.26811859733049, 42,
        0.402165057047551, 0.139888420742186, 42, 0.248044898503808, 0.320982408483093, 42,
        1.23665, -0.380349999999999, 42,
    ];

    private static readonly double[] aKnots4 =
    [
        0, 0, 0, 0, 0.0161348522840521, 0.0807608132035811,
        0.131338840314679, 0.147844505833154, 0.16533568293713, 0.250607290759724, 0.267449419858552, 0.322628298230969,
        0.341027430676258, 0.36078046495376, 0.420324490838906, 0.440253145913864, 0.458870520105636, 0.492435831240754,
        0.509078502457839, 0.525715578369451, 0.577446072383669, 0.597241246236863, 0.656453700919606, 0.676155157762965,
        0.694541842118733, 0.749834839741989, 0.766726765513781, 0.834662561102041, 0.868900689429001, 0.88456999851039,
        0.931780340251946, 1, 1, 1, 1,
    ];

    private static readonly double[] aPoles5 =
    [
        1.38958, -0.30093, 56, 2.43615912287256, -0.909728724567382, 56,
        3.0281955825442, -1.8572582903678, 56, 3.65762773796813, -2.54529310782012, 56,
        4.96146733383277, -4.12983817908431, 56, 5.23021328351262, -4.48294463637656, 56,
        6.97411718250412, -8.12140889198172, 56, 6.70106429787142, -9.9518367568647, 56.0000000000001,
        6.61799594372847, -10.831499083977, 56, 5.83042424901388, -13.1305423440654, 56,
        4.4856942305868, -14.6753503685142, 56, 3.526218495628, -15.6885280970105, 56,
        -1.30359786741575, -19.3108973511269, 56, -5.83186375110243, -21.0521342727528, 56,
        -12.7369662411789, -23.0728272909214, 56, -14.6499746822398, -23.5541553891022, 56,
        -15.5393756779425, -23.8164232092995, 56, -16.7857626486778, -22.971990212193, 56,
        -15.8148711149702, -22.7381053237965, 56, -14.1582028215773, -22.0884170074466, 56,
        -8.2848005862047, -19.3490481837665, 56, -4.53645198453342, -17.0504864772276, 56,
        -1.10904288834323, -12.471539082827, 56, -0.522426290934783, -11.1995642443973, 56,
        0.262420398047911, -9.25463182654568, 56, 0.213867071381995, -6.39113351333029, 56,
        0.0369897362998156, -5.60398557321822, 56, -0.344006372425831, -3.85342683471391, 56,
        -1.68576438677131, -1.32767756558911, 56, -1.05109637156753, -0.635077640089276, 56,
        0.456139976122501, 0.188083742774127, 56, 0.318806380192129, 0.321942747786321, 56,
        1.38958, -0.30093, 56,
    ];

    private static readonly double[] aKnots5 =
    [
        0, 0, 0, 0, 0.0668554777677272, 0.0834666756318599,
        0.100984720713562, 0.151909931618998, 0.169593414001346, 0.254704249388902, 0.27136091562454, 0.289520682572814,
        0.325734361487981, 0.343874098775056, 0.363388151948866, 0.422534909356546, 0.442420378784176, 0.461023580422867,
        0.494592323781788, 0.527437648618005, 0.560332403705056, 0.578743404285102, 0.598421903816054, 0.65700259163225,
        0.676406731087284, 0.694519638867019, 0.730942734487404, 0.749396334888904, 0.766418578227312, 0.818448937788905,
        0.836068871481316, 0.8848958398016, 0.931598977689477, 1, 1, 1,
        1,
    ];

    // Test case: ThruSections with B-spline profiles having different numbers of poles.
    [Fact]
    public void BSplineProfilesWithDifferentPoleCount()
    {
        (double[], double[])[] aSections = [(aPoles1, aKnots1), (aPoles2, aKnots2), (aPoles3, aKnots3), (aPoles4, aKnots4), (aPoles5, aKnots5)];

        BRepOffsetAPI_ThruSections aThruSections = new(true, false, 1.0e-06);

        foreach ((double[] aP, double[] aK) in aSections)
        {
            Geom_BSplineCurve aCurve = createBSplineCurve(aP, aK);

            BRepBuilderAPI_MakeEdge aMakeEdge = new(aCurve);
            Assert.True(aMakeEdge.IsDone()); // Failed to create edge from B-spline curve

            BRepBuilderAPI_MakeWire aMakeWire = new(aMakeEdge.Edge());
            Assert.True(aMakeWire.IsDone()); // Failed to create wire from edge

            aThruSections.AddWire(aMakeWire.Wire());
        }

        // Build should not throw - this is the regression test
        aThruSections.Build();

        // Verify the result
        Assert.True(aThruSections.IsDone());
        Assert.False(aThruSections.Shape().IsNull());
    }

    // Test OCC895: BRepOffsetAPI_ThruSections with two circular arc wires.
    [Fact]
    public void OCC895_TwoCircularArcWires_NoTwist()
    {
        double aRad = 1.0;
        double aAngle = 5.0 * Math.PI / 180.0;

        // Wire 1: arc from a circle whose axis is rotated 5 degrees around Z
        Point3D aCenter1 = new(0, 10, 0);
        Frame3D aAxis1 = new Frame3D(aCenter1, Constants.DY().Reversed(), Constants.DX()).Rotated(new Axis3D(aCenter1, Constants.DZ()), aAngle);

        gce_MakeCirc aMakeCirc1 = new(aAxis1, aRad);
        Assert.True(aMakeCirc1.IsDone());
        GC_MakeArcOfCircle aMakeArc1 = new(aMakeCirc1.Value(), 0, Math.PI / 2, true);
        Assert.True(aMakeArc1.IsDone());
        Geom_TrimmedCurve aArc1 = aMakeArc1.Value();

        BRepBuilderAPI_MakeEdge aMakeEdge1 = new(aArc1, aArc1.StartPoint(), aArc1.EndPoint());
        Assert.True(aMakeEdge1.IsDone());
        BRepBuilderAPI_MakeWire aMakeWire1 = new(aMakeEdge1.Edge());
        Assert.True(aMakeWire1.IsDone());
        TopoDS_Wire aWire1 = aMakeWire1.Wire();

        // Wire 2: arc from a circle at a different center with fixed axis
        Point3D aCenter2 = new(10, 0, 0);
        Frame3D aAxis2 = new(aCenter2, Constants.DX().Reversed(), Constants.DZ());

        gce_MakeCirc aMakeCirc2 = new(aAxis2, aRad);
        Assert.True(aMakeCirc2.IsDone());
        GC_MakeArcOfCircle aMakeArc2 = new(aMakeCirc2.Value(), 0, Math.PI / 2, true);
        Assert.True(aMakeArc2.IsDone());
        Geom_TrimmedCurve aArc2 = aMakeArc2.Value();

        BRepBuilderAPI_MakeEdge aMakeEdge2 = new(aArc2, aArc2.StartPoint(), aArc2.EndPoint());
        Assert.True(aMakeEdge2.IsDone());
        BRepBuilderAPI_MakeWire aMakeWire2 = new(aMakeEdge2.Edge());
        Assert.True(aMakeWire2.IsDone());
        TopoDS_Wire aWire2 = aMakeWire2.Wire();

        // Build ThruSections shell with order=0: wire2 first, then wire1
        BRepOffsetAPI_ThruSections aThruSect = new(false, true);
        aThruSect.AddWire(aWire2);
        aThruSect.AddWire(aWire1);
        aThruSect.Build();

        Assert.True(aThruSect.IsDone());
        Assert.False(aThruSect.Shape().IsNull());

        // Verify surface area is approximately 18.1614 (reference value from DRAW test)
        Assert.Equal(18.1614, Area(aThruSect.Shape()), 0.01);
    }

    // ---- OCCT.NET: frozen inputs, the history ----

    [Theory]
    [InlineData(false, true)]
    [InlineData(true, true)]
    [InlineData(false, false)]
    [InlineData(true, false)]
    public void Frozen_sections_are_never_written_and_the_result_is_frozen(bool theSolid, bool theRuled)
    {
        TopoDS_Wire[] aWires = [Square(0, 2), Circle(5, 1.5), Square(10, 1)];
        string[] aTexts = [.. aWires.Select(w => Text(w))];
        BRepOffsetAPI_ThruSections aLoft = new(theSolid, theRuled);
        foreach (TopoDS_Wire aW in aWires)
        {
            Assert.True(aW.TShape()!.IsFrozen);
            aLoft.AddWire(aW);
        }
        aLoft.Build();
        Assert.True(aLoft.IsDone());
        TopoDS_Shape aResult = aLoft.Shape();
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(theSolid ? TopAbs_SOLID : TopAbs_SHELL, aResult.ShapeType());
        // CompatibleWires cuts the sections into 5 edges (the circle's seam and the squares' corners)
        Assert.Equal(theRuled ? 10 : 5, Count(aResult, TopAbs_FACE) - (theSolid ? 2 : 0));
        for (int i = 0; i < aWires.Length; i++)
        {
            Assert.Equal(aTexts[i], Text(aWires[i]));
        }

        // the history answers for the caller's edges and vertices, with shapes of the result
        NCollection_IndexedMap<TopoDS_Shape> aResultMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aResult, aResultMap);
        int aNbFaces = 0;
        for (TopExp_Explorer anExp = new(aWires[0], TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            ISchematiqList<TopoDS_Shape> aFaces = aLoft.Generated(anExp.Current());
            Assert.NotEmpty(aFaces); // one face per piece of the edge (per section and piece when ruled)
            Assert.All(aFaces, f => Assert.True(aResultMap.Contains(f)));
            TopoDS_Shape aFace = aLoft.GeneratedFace(anExp.Current()); // null for an edge CompatibleWires cut, as natively
            Assert.True(aFace.IsNull() || (aResultMap.Contains(aFace) && aFace.TShape()!.IsFrozen));
            aNbFaces += aFace.IsNull() ? 0 : 1;
        }
        Assert.Equal(3, aNbFaces);
        for (TopExp_Explorer anExp = new(aWires[0], TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            ISchematiqList<TopoDS_Shape> anEdges = aLoft.Generated(anExp.Current());
            Assert.Equal(theRuled ? 2 : 1, anEdges.Count);
            Assert.All(anEdges, e => Assert.True(aResultMap.Contains(e) && e.ShapeType() == TopAbs_EDGE));
        }
        if (theSolid)
        {
            Assert.True(aResultMap.Contains(aLoft.FirstShape()));
            Assert.True(aResultMap.Contains(aLoft.LastShape()));
            Assert.True(Volume(aResult) > 0);
        }
    }

    [Fact]
    public void Point_sections_and_their_history()
    {
        TopoDS_Vertex aV1 = new BRepBuilderAPI_MakeVertex(new Point3D(0, 0, -4)).Vertex();
        TopoDS_Vertex aV2 = new BRepBuilderAPI_MakeVertex(new Point3D(0, 0, 4)).Vertex();
        foreach (bool aRuled in new[] { true, false })
        {
            BRepOffsetAPI_ThruSections aLoft = new(true, aRuled);
            aLoft.AddVertex(aV1);
            aLoft.AddWire(Square(0, 2));
            aLoft.AddVertex(aV2);
            aLoft.Build();
            Assert.True(aLoft.IsDone());
            Assert.Equal(BRepFill_ThruSectionErrorStatus_Done, aLoft.GetStatus());
            Assert.True(Volume(aLoft.Shape()) > 0);
            Assert.Equal(aRuled ? 8 : 4, aLoft.Generated(aV1).Count); // ruled: the chain of lateral edges through the square
            Assert.Equal(3, aLoft.Wires().Count);
        }
    }

    [Fact]
    public void A_point_between_sections_is_wrong_usage()
    {
        BRepOffsetAPI_ThruSections aLoft = new();
        aLoft.AddWire(Square(0, 1));
        aLoft.AddVertex(new BRepBuilderAPI_MakeVertex(new Point3D(0, 0, 1)).Vertex());
        aLoft.AddWire(Square(2, 1));
        aLoft.Build();
        Assert.False(aLoft.IsDone());
        Assert.Equal(BRepFill_ThruSectionErrorStatus_WrongUsage, aLoft.GetStatus());
        Assert.Throws<NotDoneException>(() => aLoft.Shape());
    }

    [Fact]
    public void Options_keep_OCCTs_defaults()
    {
        BRepOffsetAPI_ThruSections aLoft = new();
        Assert.Equal(8, aLoft.MaxDegree());
        Assert.Equal(Continuity.C2, aLoft.Continuity());
        Assert.Equal(Approx_ParametrizationType.Approx_ChordLength, aLoft.ParType());
        Assert.False(aLoft.UseSmoothing());
        Assert.True(aLoft.IsMutableInput());
        aLoft.CriteriumWeight(out double aW1, out double aW2, out double aW3);
        Assert.Equal((0.4, 0.2, 0.4), (aW1, aW2, aW3));
        aLoft.SetCriteriumWeight(-1, 0, 0);
        Assert.Equal(BRepFill_ThruSectionErrorStatus_Failed, aLoft.GetStatus());
        aLoft.Init();
        Assert.Equal(6, aLoft.MaxDegree()); // OCCT: Init sets 6
    }

    [Fact]
    public void Generator_copies_frozen_sections_and_keys_faces_as_OCCT_does()
    {
        TopoDS_Wire aW1 = Square(0, 1), aW2 = Square(2, 2);
        string aT1 = Text(aW1), aT2 = Text(aW2);
        BRepFill_Generator aGen = new();
        aGen.AddWire(aW1);
        aGen.AddWire(aW2);
        aGen.Perform();
        Assert.Equal(BRepFill_ThruSectionErrorStatus_Done, aGen.GetStatus());
        TopoDS_Shell aShell = aGen.Shell();
        Assert.True(aShell.TShape()!.IsFrozen);
        Assert.Equal(4, Count(aShell, TopAbs_FACE));
        Assert.Equal(aT1, Text(aW1));
        Assert.Equal(aT2, Text(aW2));

        // OCCT keys each face by the edge after the one it was built on (a null edge for the last; occt-issues.md)
        BRepTools_WireExplorer anExp = new(aW1);
        TopoDS_Edge aFirst = anExp.Current();
        Assert.Empty(aGen.GeneratedShapes(aFirst));
        anExp.Next();
        Assert.Single(aGen.GeneratedShapes(anExp.Current()));
        Assert.Equal(4, aGen.Generated().Count);
    }

    // ---- BRepProj_Projection ----

    [Fact]
    public void Projection_along_a_direction_and_from_a_point()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new Point3D(-5, -5, 0), 10, 10, 2).Shape();
        TopoDS_Wire aCircle = Circle(10, 2);
        string aText = Text(aCircle);

        BRepProj_Projection aCyl = new(aCircle, aBox, new Direction3D(0, 0, -1));
        Assert.True(aCyl.IsDone());
        int aNb = 0;
        for (aCyl.Init(); aCyl.More(); aCyl.Next())
        {
            Assert.True(aCyl.Current().TShape()!.IsFrozen);
            aNb++;
        }
        Assert.Equal(2, aNb); // on the top and bottom faces
        Assert.True(aCyl.Shape().TShape()!.IsFrozen);

        BRepProj_Projection aCone = new(aCircle, aBox, new Point3D(0, 0, 20));
        Assert.True(aCone.IsDone());
        Assert.Equal(aText, Text(aCircle));
    }

    [Fact]
    public void Projection_checks_its_input()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(new Point3D(1, 1, 1)).Vertex();
        Assert.Throws<NullObjectException>(() => new BRepProj_Projection(new TopoDS_Shape(), aBox, new Direction3D(0, 0, 1)));
        Assert.Throws<ConstructionErrorException>(() => new BRepProj_Projection(aV, aBox, new Direction3D(0, 0, 1)));
        TopoDS_Wire aSquare = Square(20, 1);
        TopExp_Explorer anExp = new(aSquare, TopAbs_VERTEX);
        Point3D aFirst = BRep_Tool.Pnt(TopoDS.Vertex(anExp.Current()));
        Assert.Throws<ConstructionErrorException>(() => new BRepProj_Projection(aSquare, aBox, aFirst));
        Assert.Throws<ConstructionErrorException>(() => new BRepProj_Projection(aSquare, aV, new Direction3D(0, 0, 1)));
    }
}
