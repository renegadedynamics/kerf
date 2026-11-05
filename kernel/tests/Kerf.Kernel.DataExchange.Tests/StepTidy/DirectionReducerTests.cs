// Port of: src/DataExchange/TKDESTEP/GTests/StepTidy_DirectionReducer_Test.cxx
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

public class DirectionReducerTests : BaseTestFixture
{
    // Perform removal of duplicate entities.
    HashSet<Transient> replaceDuplicateDirections()
    {
        var aReducer = new DirectionReducer(myWS);
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
        // Creating directions.
        var aDir1 = addDirection("dir1");
        var aDir2 = addDirection("dir2");

        // Creating vector containing the first direction.
        var aFirstVector = new StepVector();
        aFirstVector.Init(new TCollection_HAsciiString(), aDir1, 1.0);
        addToModel(aFirstVector);

        // Creating vector containing the second direction.
        var aSecondVector = new StepVector();
        aSecondVector.Init(new TCollection_HAsciiString(), aDir2, 1.0);
        addToModel(aSecondVector);

        // Performing removal of duplicate directions.
        HashSet<Transient> aRemovedEntities = replaceDuplicateDirections();

        // Check that nothing was removed.
        Assert.Empty(aRemovedEntities);
    }

    [Fact]
    public void Axis1Placement()
    {
        // Creating directions.
        var aDir1 = addDirection();
        var aDir2 = addDirection();

        // Creating Cartesian point for the location.
        var aLocation = new StepCartesianPoint();
        var aLocationCoords = new NCollection_HArray1<double>(1, 3);
        aLocationCoords.SetValue(1, 0.0);
        aLocationCoords.SetValue(2, 0.0);
        aLocationCoords.SetValue(3, 0.0);
        aLocation.Init(new TCollection_HAsciiString(), aLocationCoords);
        addToModel(aLocation);

        // Creating axis containing the first direction.
        var aFirstAxis = new StepAxis1Placement();
        aFirstAxis.Init(new TCollection_HAsciiString(), aLocation, true, aDir1);
        addToModel(aFirstAxis);

        // Creating axis containing the second direction.
        var aSecondAxis = new StepAxis1Placement();
        aSecondAxis.Init(new TCollection_HAsciiString(), aLocation, true, aDir2);
        addToModel(aSecondAxis);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities = replaceDuplicateDirections();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aDir1) || aRemovedEntities.Contains(aDir2));
    }

    [Fact]
    public void Axis2Placement()
    {
        // Creating directions.
        var aDir1 = addDirection();
        var aDir2 = addDirection();

        // Creating Cartesian point for the location.
        var aLocation = new StepCartesianPoint();
        var aLocationCoords = new NCollection_HArray1<double>(1, 3);
        aLocationCoords.SetValue(1, 0.0);
        aLocationCoords.SetValue(2, 0.0);
        aLocationCoords.SetValue(3, 0.0);
        aLocation.Init(new TCollection_HAsciiString(), aLocationCoords);
        addToModel(aLocation);

        // Creating axis containing the first direction.
        var aFirstAxis = new StepAxis2Placement3d();
        aFirstAxis.Init(new TCollection_HAsciiString(), aLocation, true, aDir1, false, null!);
        addToModel(aFirstAxis);

        // Creating axis containing the second direction.
        var aSecondAxis = new StepAxis2Placement3d();
        aSecondAxis.Init(new TCollection_HAsciiString(), aLocation, true, aDir2, false, null!);
        addToModel(aSecondAxis);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities = replaceDuplicateDirections();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aDir1) || aRemovedEntities.Contains(aDir2));
    }

    [Fact]
    public void Vector()
    {
        // Creating directions.
        var aDir1 = addDirection();
        var aDir2 = addDirection();

        // Creating vector containing the first direction.
        var aFirstVector = new StepVector();
        aFirstVector.Init(new TCollection_HAsciiString(), aDir1, 1.0);
        addToModel(aFirstVector);

        // Creating vector containing the second direction.
        var aSecondVector = new StepVector();
        aSecondVector.Init(new TCollection_HAsciiString(), aDir2, 1.0);
        addToModel(aSecondVector);

        // Performing removal of duplicate Cartesian points.
        HashSet<Transient> aRemovedEntities = replaceDuplicateDirections();

        // Check that duplicate was removed.
        Assert.Single(aRemovedEntities);
        Assert.True(aRemovedEntities.Contains(aDir1) || aRemovedEntities.Contains(aDir2));
    }
}
