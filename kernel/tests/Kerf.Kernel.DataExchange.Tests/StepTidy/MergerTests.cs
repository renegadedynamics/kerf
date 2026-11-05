// Port of: src/DataExchange/TKDESTEP/GTests/StepTidy_Merger_Test.cxx
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

public class DuplicateCleanerTests : BaseTestFixture
{
    // Get the number of entities of the specified type.
    // @param theType the type of entities to count.
    // @return the number of entities of the specified type.
    int getEntitiesCount(Type theType)
    {
        int aCount = 0;
        for (int i = 1; i <= myWS.Model()!.EntityCount; i++)
        {
            if (myWS.Model()!.Value(i)!.GetType().IsAssignableTo(theType))
            {
                aCount++;
            }
        }
        return aCount;
    }

    // Perform removal of duplicate entities points.
    void performRemoval()
    {
        var aMerger = new DuplicateCleaner(myWS);
        aMerger.Perform();
    }

    [Fact]
    public void DifferentEntities()
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

        int aDirectionCountBefore = getEntitiesCount(typeof(StepDirection));

        // Performing removal of duplicate directions.
        performRemoval();

        int aDirectionCountAfter = getEntitiesCount(typeof(StepDirection));

        // Check that nothing was removed.
        Assert.Equal(2, aDirectionCountBefore);
        Assert.Equal(aDirectionCountAfter, aDirectionCountBefore);
    }

    [Fact]
    public void EqualEntities()
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

        int aDirectionCountBefore = getEntitiesCount(typeof(StepDirection));

        // Performing removal of duplicate directions.
        performRemoval();

        int aDirectionCountAfter = getEntitiesCount(typeof(StepDirection));

        // Check that one direction was removed.
        Assert.Equal(2, aDirectionCountBefore);
        Assert.Equal(1, aDirectionCountAfter);
    }
}
