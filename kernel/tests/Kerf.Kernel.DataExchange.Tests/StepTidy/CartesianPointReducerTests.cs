// Port of: src/DataExchange/TKDESTEP/GTests/StepTidy_CartesianPointReducer_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Tidy;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;

namespace Kerf.Kernel.DataExchange.Tests.StepTidy;

public class CartesianPointReducerTests : BaseTestFixture
{
    // Perform removal of duplicate Cartesian points.
    HashSet<Transient> replaceDuplicateCartesianPoints()
    {
        var aReducer = new CartesianPointReducer(myWS);
        for (int anIndex = 1; anIndex <= myWS.Model()!.EntityCount; ++anIndex)
        {
            aReducer.ProcessEntity(myWS.Model()!.Value(anIndex)!);
        }

        var aRemovedEntities = new HashSet<Transient>(ReferenceEqualityComparer.Instance);
        aReducer.Perform(aRemovedEntities);
        return aRemovedEntities;
    }

    [Fact]
    public void DifferentNames()
    {
        // Creating Cartesian points.
        var aPt1 = addCartesianPoint("FirstPt");
        var aPt2 = addCartesianPoint("SecondPt");

        // Creating direction.
        var aDirCoords = new NCollection_HArray1<double>(1, 3);
        aDirCoords.SetValue(1, 0.0);
        aDirCoords.SetValue(2, 0.0);
        aDirCoords.SetValue(3, 1.0);
        var aDir = new StepDirection();
        aDir.Init(new TCollection_HAsciiString(), aDirCoords);
        addToModel(aDir);

        // Creating axis containing the first Cartesian point.
        var aFirstAxis = new StepAxis2Placement3d();
        aFirstAxis.Init(new TCollection_HAsciiString(), aPt1, true, aDir, false, null!);
        addToModel(aFirstAxis);

        // Creating axis containing the second Cartesian point.
        var aSecondAxis = new StepAxis2Placement3d();
        aSecondAxis.Init(new TCollection_HAsciiString(), aPt2, true, aDir, false, null!);
        addToModel(aSecondAxis);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateCartesianPoints();

        // Check that nothing was removed.
        Assert.Empty(aRemovedEntities);
    }

    [Fact]
    public void Axis2Placement3d()
    {
        // Creating Cartesian points.
        var aPt1 = addCartesianPoint();
        var aPt2 = addCartesianPoint();

        // Creating direction.
        var aDirCoords = new NCollection_HArray1<double>(1, 3);
        aDirCoords.SetValue(1, 0.0);
        aDirCoords.SetValue(2, 0.0);
        aDirCoords.SetValue(3, 1.0);
        var aDir = new StepDirection();
        aDir.Init(new TCollection_HAsciiString(), aDirCoords);
        addToModel(aDir);

        // Creating axis containing the first Cartesian point.
        var aFirstAxis = new StepAxis2Placement3d();
        aFirstAxis.Init(new TCollection_HAsciiString(), aPt1, true, aDir, false, null!);
        addToModel(aFirstAxis);

        // Creating axis containing the second Cartesian point.
        var aSecondAxis = new StepAxis2Placement3d();
        aSecondAxis.Init(new TCollection_HAsciiString(), aPt2, true, aDir, false, null!);
        addToModel(aSecondAxis);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateCartesianPoints();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aPt1) || aRemovedEntities.Contains(aPt2));
    }

    [Fact]
    public void VertexPoint()
    {
        // Creating Cartesian points.
        var aPt1 = addCartesianPoint();
        var aPt2 = addCartesianPoint();

        // Creating vertex containing the first Cartesian point.
        var aFirstVertex = new StepVertexPoint();
        aFirstVertex.Init(new TCollection_HAsciiString(), aPt1);
        addToModel(aFirstVertex);

        // Creating vertex containing the second Cartesian point.
        var aSecondVertex = new StepVertexPoint();
        aSecondVertex.Init(new TCollection_HAsciiString(), aPt2);
        addToModel(aSecondVertex);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateCartesianPoints();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aPt1) || aRemovedEntities.Contains(aPt2));
    }

    [Fact]
    public void GeometricCurveSet()
    {
        // Creating Cartesian points.
        var aPt1 = addCartesianPoint();
        var aPt2 = addCartesianPoint();

        // Creating curve set containing the first Cartesian point.
        var aFirstElements =
            new NCollection_HArray1<StepGeometricSetSelect>(1, 1);
        var aFirstSelect = new StepGeometricSetSelect();
        aFirstSelect.SetValue(aPt1);
        aFirstElements.SetValue(1, aFirstSelect);
        addToModel(aFirstElements);
        var aFirstCurveSet = new StepGeometricCurveSet();
        aFirstCurveSet.Init(new TCollection_HAsciiString(), aFirstElements);
        addToModel(aFirstCurveSet);

        // Creating curve set containing the second Cartesian point.
        var aSecondElements =
            new NCollection_HArray1<StepGeometricSetSelect>(1, 1);
        var aSecondSelect = new StepGeometricSetSelect();
        aSecondSelect.SetValue(aPt2);
        aSecondElements.SetValue(1, aSecondSelect);
        addToModel(aSecondElements);
        var aSecondCurveSet = new StepGeometricCurveSet();
        aSecondCurveSet.Init(new TCollection_HAsciiString(), aSecondElements);
        addToModel(aSecondCurveSet);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateCartesianPoints();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aPt1) || aRemovedEntities.Contains(aPt2));
    }

    [Fact]
    public void PresentationLayerAssignment()
    {
        // Creating Cartesian points.
        var aPt1 = addCartesianPoint();
        var aPt2 = addCartesianPoint();

        // Creating presentation layer assignment containing the first Cartesian point.
        var aFirstAssignedItems =
            new NCollection_HArray1<LayeredItem>(1, 1);
        var aFirstLayeredItem = new LayeredItem();
        aFirstLayeredItem.SetValue(aPt1);
        aFirstAssignedItems.SetValue(1, aFirstLayeredItem);
        addToModel(aFirstAssignedItems);
        var aFirstAssignment =
            new PresentationLayerAssignment();
        aFirstAssignment.Init(new TCollection_HAsciiString(),
                                                      new TCollection_HAsciiString(),
                                                      aFirstAssignedItems);
        addToModel(aFirstAssignment);

        // Creating presentation layer assignment containing the second Cartesian point.
        var aSecondAssignedItems =
            new NCollection_HArray1<LayeredItem>(1, 1);
        var aSecondLayeredItem = new LayeredItem();
        aSecondLayeredItem.SetValue(aPt2);
        aSecondAssignedItems.SetValue(1, aSecondLayeredItem);
        addToModel(aSecondAssignedItems);
        var aSecondAssignment =
            new PresentationLayerAssignment();
        aSecondAssignment.Init(new TCollection_HAsciiString(),
                                                        new TCollection_HAsciiString(),
                                                        aSecondAssignedItems);
        addToModel(aSecondAssignment);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateCartesianPoints();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aPt1) || aRemovedEntities.Contains(aPt2));
    }

    [Fact]
    public void StyledItem()
    {
        // Creating Cartesian points.
        var aPt1 = addCartesianPoint();
        var aPt2 = addCartesianPoint();

        // Creating styled item containing the first Cartesian point.
        var aFirstAssignedItems =
                new NCollection_HArray1<PresentationStyleAssignment>(1, 1);
        var aFirstStyledItem = new StyledItem();
        aFirstStyledItem.Init(new TCollection_HAsciiString(), aFirstAssignedItems, aPt1);
        addToModel(aFirstStyledItem);

        // Creating styled item containing the second Cartesian point.
        var aSecondAssignedItems =
                new NCollection_HArray1<PresentationStyleAssignment>(1, 1);
        var aSecondStyledItem = new StyledItem();
        aSecondStyledItem.Init(new TCollection_HAsciiString(), aSecondAssignedItems, aPt2);
        addToModel(aSecondStyledItem);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateCartesianPoints();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aPt1) || aRemovedEntities.Contains(aPt2));
    }

    [Fact]
    public void BSplineCurveWithKnots()
    {
        // Creating Cartesian points.
        var aPt1 = addCartesianPoint();
        var aPt2 = addCartesianPoint();

        // Creating curve containing the first Cartesian point.
        var aFirstControlPoints =
            new NCollection_HArray1<StepCartesianPoint>(1, 1);
        aFirstControlPoints.SetValue(1, aPt1);
        var aFirstCurve = new StepBSplineCurveWithKnots();
        aFirstCurve.Init(new TCollection_HAsciiString(),
                                            1,
                                            aFirstControlPoints,
                                            StepBSplineCurveForm.BscfUnspecified,
                                            Logical.LUnknown,
                                            Logical.LUnknown,
                                            new NCollection_HArray1<int>(),
                                            new NCollection_HArray1<double>(),
                                            StepKnotType.KtUnspecified);
        addToModel(aFirstCurve);

        // Creating curve containing the second Cartesian point.
        var aSecondControlPoints =
            new NCollection_HArray1<StepCartesianPoint>(1, 1);
        aSecondControlPoints.SetValue(1, aPt2);
        var aSecondCurve = new StepBSplineCurveWithKnots();
        aSecondCurve.Init(new TCollection_HAsciiString(),
                                              1,
                                              aSecondControlPoints,
                                              StepBSplineCurveForm.BscfUnspecified,
                                              Logical.LUnknown,
                                              Logical.LUnknown,
                                              new NCollection_HArray1<int>(),
                                              new NCollection_HArray1<double>(),
                                              StepKnotType.KtUnspecified);
        addToModel(aSecondCurve);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateCartesianPoints();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aPt1) || aRemovedEntities.Contains(aPt2));
    }

    [Fact]
    public void Line()
    {
        // Creating Cartesian points.
        var aPt1 = addCartesianPoint();
        var aPt2 = addCartesianPoint();

        // Creating line containing the first Cartesian point.
        var aFirstLine = new StepLine();
        aFirstLine.Init(new TCollection_HAsciiString(), aPt1, new StepVector());
        addToModel(aFirstLine);

        // Creating line containing the second Cartesian point.
        var aSecondLine = new StepLine();
        aSecondLine.Init(new TCollection_HAsciiString(), aPt2, new StepVector());
        addToModel(aSecondLine);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateCartesianPoints();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aPt1) || aRemovedEntities.Contains(aPt2));
    }

    [Fact]
    public void BSplineSurfaceWithKnots()
    {
        // Creating Cartesian points.
        var aPt1 = addCartesianPoint();
        var aPt2 = addCartesianPoint();

        // Creating surface containing the first Cartesian point.
        var aFirstControlPoints =
            new NCollection_HArray2<StepCartesianPoint>(1, 1, 1, 1);
        aFirstControlPoints.SetValue(1, 1, aPt1);
        var aFirstSurface =
            new StepBSplineSurfaceWithKnots();
        aFirstSurface.Init(new TCollection_HAsciiString(),
                                                1,
                                                1,
                                                aFirstControlPoints,
                                                StepBSplineSurfaceForm.BssfUnspecified,
                                                Logical.LUnknown,
                                                Logical.LUnknown,
                                                Logical.LUnknown,
                                                new NCollection_HArray1<int>(),
                                                new NCollection_HArray1<int>(),
                                                new NCollection_HArray1<double>(),
                                                new NCollection_HArray1<double>(),
                                                StepKnotType.KtUnspecified);
        addToModel(aFirstSurface);

        // Creating surface containing the second Cartesian point.
        var aSecondControlPoints =
            new NCollection_HArray2<StepCartesianPoint>(1, 1, 1, 1);
        aSecondControlPoints.SetValue(1, 1, aPt2);
        var aSecondSurface =
            new StepBSplineSurfaceWithKnots();
        aSecondSurface.Init(new TCollection_HAsciiString(),
                                                  1,
                                                  1,
                                                  aSecondControlPoints,
                                                  StepBSplineSurfaceForm.BssfUnspecified,
                                                  Logical.LUnknown,
                                                  Logical.LUnknown,
                                                  Logical.LUnknown,
                                                  new NCollection_HArray1<int>(),
                                                  new NCollection_HArray1<int>(),
                                                  new NCollection_HArray1<double>(),
                                                  new NCollection_HArray1<double>(),
                                                  StepKnotType.KtUnspecified);
        addToModel(aSecondSurface);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateCartesianPoints();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aPt1) || aRemovedEntities.Contains(aPt2));
    }

    [Fact]
    public void Axis1Placement()
    {
        // Creating Cartesian points.
        var aPt1 = addCartesianPoint();
        var aPt2 = addCartesianPoint();

        // Creating axis containing the first Cartesian point.
        var aFirstAxis = new StepAxis1Placement();
        aFirstAxis.Init(new TCollection_HAsciiString(), aPt1, false, new StepDirection());
        addToModel(aFirstAxis);

        // Creating axis containing the second Cartesian point.
        var aSecondAxis = new StepAxis1Placement();
        aSecondAxis.Init(new TCollection_HAsciiString(), aPt2, false, new StepDirection());
        addToModel(aSecondAxis);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateCartesianPoints();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aPt1) || aRemovedEntities.Contains(aPt2));
    }

    [Fact]
    public void Representation()
    {
        // Creating Cartesian points.
        var aPt1 = addCartesianPoint();
        var aPt2 = addCartesianPoint();

        // Creating representation containing the first Cartesian point.
        var aFirstItems =
            new NCollection_HArray1<RepresentationItem>(1, 1);
        aFirstItems.SetValue(1, aPt1);
        var aFirstRepresentation = new StepRepresentation();
        aFirstRepresentation.Init(new TCollection_HAsciiString(),
                                                              aFirstItems,
                                                              new RepresentationContext());
        addToModel(aFirstRepresentation);

        // Creating representation containing the second Cartesian point.
        var aSecondItems =
            new NCollection_HArray1<RepresentationItem>(1, 1);
        aSecondItems.SetValue(1, aPt2);
        var aSecondRepresentation = new StepRepresentation();
        aSecondRepresentation.Init(new TCollection_HAsciiString(),
                                                                aSecondItems,
                                                                new RepresentationContext());
        addToModel(aSecondRepresentation);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateCartesianPoints();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aPt1) || aRemovedEntities.Contains(aPt2));
    }

    [Fact]
    public void BSplineCurveWithKnotsAndRationalBSplineCurve()
    {
        // Creating Cartesian points.
        var aPt1 = addCartesianPoint();
        var aPt2 = addCartesianPoint();

        // Creating curve containing the first Cartesian point.
        var aFirstControlPoints =
            new NCollection_HArray1<StepCartesianPoint>(1, 1);
        aFirstControlPoints.SetValue(1, aPt1);
        var aFirstCurve =
            new StepBSplineCurveWithKnotsAndRationalBSplineCurve();
        aFirstCurve.Init(new TCollection_HAsciiString(),
                                            1,
                                            aFirstControlPoints,
                                            StepBSplineCurveForm.BscfUnspecified,
                                            Logical.LUnknown,
                                            Logical.LUnknown,
                                            new NCollection_HArray1<int>(),
                                            new NCollection_HArray1<double>(),
                                            StepKnotType.KtUnspecified,
                                            new NCollection_HArray1<double>());
        addToModel(aFirstCurve);

        // Creating curve containing the second Cartesian point.
        var aSecondControlPoints =
            new NCollection_HArray1<StepCartesianPoint>(1, 1);
        aSecondControlPoints.SetValue(1, aPt2);
        var aSecondCurve =
            new StepBSplineCurveWithKnotsAndRationalBSplineCurve();
        aSecondCurve.Init(new TCollection_HAsciiString(),
                                              1,
                                              aSecondControlPoints,
                                              StepBSplineCurveForm.BscfUnspecified,
                                              Logical.LUnknown,
                                              Logical.LUnknown,
                                              new NCollection_HArray1<int>(),
                                              new NCollection_HArray1<double>(),
                                              StepKnotType.KtUnspecified,
                                              new NCollection_HArray1<double>());
        addToModel(aSecondCurve);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities =
            replaceDuplicateCartesianPoints();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aPt1) || aRemovedEntities.Contains(aPt2));
    }

    [Fact]
    public void BSplineSurfaceWithKnotsAndRationalBSplineSurface()
    {
        // Creating Cartesian points.
        var aPt1 = addCartesianPoint();
        var aPt2 = addCartesianPoint();

        // Creating rational BSpline surface to use.
        var aRationalBSplineSurface =
        new StepRationalBSplineSurface();
        aRationalBSplineSurface.Init(
        new TCollection_HAsciiString(),
        1,
        1,
        new NCollection_HArray2<StepCartesianPoint>(1, 1, 1, 1),
        StepBSplineSurfaceForm.BssfUnspecified,
        Logical.LUnknown,
        Logical.LUnknown,
        Logical.LUnknown,
        new NCollection_HArray2<double>(1, 1, 1, 1));

        // Creating surface containing the first Cartesian point.
        var aFirstControlPoints =
        new NCollection_HArray2<StepCartesianPoint>(1, 1, 1, 1);
        aFirstControlPoints.SetValue(1, 1, aPt1);
        var aFirstBSSWN = new StepBSplineSurfaceWithKnots();
        aFirstBSSWN.Init(new TCollection_HAsciiString(),
                                        1,
                                        1,
                                        aFirstControlPoints,
                                        StepBSplineSurfaceForm.BssfUnspecified,
                                        Logical.LUnknown,
                                        Logical.LUnknown,
                                        Logical.LUnknown,
                                        new NCollection_HArray1<int>(),
                                        new NCollection_HArray1<int>(),
                                        new NCollection_HArray1<double>(),
                                        new NCollection_HArray1<double>(),
                                        StepKnotType.KtUnspecified);
        addToModel(aFirstBSSWN);
        var aFirstSurface =
        new StepBSplineSurfaceWithKnotsAndRationalBSplineSurface();
        aFirstSurface.Init(new TCollection_HAsciiString(),
                                            1,
                                            1,
                                            aFirstControlPoints,
                                            StepBSplineSurfaceForm.BssfUnspecified,
                                            Logical.LUnknown,
                                            Logical.LUnknown,
                                            Logical.LUnknown,
                                            aFirstBSSWN,
                                            aRationalBSplineSurface);
        addToModel(aFirstSurface);

        // Creating surface containing the second Cartesian point.
        var aSecondControlPoints =
        new NCollection_HArray2<StepCartesianPoint>(1, 1, 1, 1);
        aSecondControlPoints.SetValue(1, 1, aPt2);
        var aSecondBSSWN = new StepBSplineSurfaceWithKnots();
        aSecondBSSWN.Init(new TCollection_HAsciiString(),
                                          1,
                                          1,
                                          aSecondControlPoints,
                                          StepBSplineSurfaceForm.BssfUnspecified,
                                          Logical.LUnknown,
                                          Logical.LUnknown,
                                          Logical.LUnknown,
                                          new NCollection_HArray1<int>(),
                                          new NCollection_HArray1<int>(),
                                          new NCollection_HArray1<double>(),
                                          new NCollection_HArray1<double>(),
                                          StepKnotType.KtUnspecified);
        addToModel(aSecondBSSWN);
        var aSecondSurface =
        new StepBSplineSurfaceWithKnotsAndRationalBSplineSurface();
        aSecondSurface.Init(new TCollection_HAsciiString(),
                                              1,
                                              1,
                                              aSecondControlPoints,
                                              StepBSplineSurfaceForm.BssfUnspecified,
                                              Logical.LUnknown,
                                              Logical.LUnknown,
                                              Logical.LUnknown,
                                              aSecondBSSWN,
                                              aRationalBSplineSurface);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities =
        replaceDuplicateCartesianPoints();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aPt1) || aRemovedEntities.Contains(aPt2));
    }
}
