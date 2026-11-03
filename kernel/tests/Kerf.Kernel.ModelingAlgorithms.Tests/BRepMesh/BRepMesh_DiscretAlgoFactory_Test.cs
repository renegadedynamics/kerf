// Port of: src/ModelingAlgorithms/TKMesh/GTests/BRepMesh_DiscretAlgoFactory_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Meshing;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

// Test fixture for BRepMesh_DiscretAlgoFactory tests
public class BRepMesh_DiscretAlgoFactoryTest
{
    private readonly Shape myBox;

    public BRepMesh_DiscretAlgoFactoryTest()
    {
        // Create a simple box shape for testing
        MakeBox aBoxMaker = new(10.0, 10.0, 10.0);
        myBox = aBoxMaker.Shape();
    }

    // Test that at least one factory is registered (FastDiscret from BRepMesh_IncrementalMeshFactory)
    [Fact]
    public void Factories_AtLeastOneRegistered()
    {
        NCollection_List<DiscretAlgoFactory> aFactories = DiscretAlgoFactory.Factories();

        Assert.False(aFactories.IsEmpty(), "No factories registered");
    }

    // Test that DefaultFactory returns a valid factory
    [Fact]
    public void DefaultFactory_ReturnsValid()
    {
        DiscretAlgoFactory? aFactory = DiscretAlgoFactory.DefaultFactory();

        Assert.NotNull(aFactory);
    }

    // Test that FastDiscret factory is registered and can be found
    [Fact]
    public void FindFactory_FastDiscret()
    {
        DiscretAlgoFactory? aFactory = DiscretAlgoFactory.FindFactory("FastDiscret");

        Assert.NotNull(aFactory);
        Assert.Equal("FastDiscret", aFactory.Name());
    }

    // Test that FindFactory returns null for non-existent factory
    [Fact]
    public void FindFactory_NonExistent_ReturnsNull()
    {
        DiscretAlgoFactory? aFactory = DiscretAlgoFactory.FindFactory("NonExistentFactory");

        Assert.Null(aFactory);
    }

    // Test that CreateAlgorithm creates a valid algorithm
    [Fact]
    public void CreateAlgorithm_ReturnsValid()
    {
        DiscretAlgoFactory? aFactory = DiscretAlgoFactory.DefaultFactory();
        Assert.NotNull(aFactory);

        DiscretRoot anAlgo = aFactory.CreateAlgorithm(myBox, 0.1, 0.5);

        Assert.NotNull(anAlgo);
    }

    // Test that created algorithm can mesh a shape
    [Fact]
    public void CreateAlgorithm_CanMesh()
    {
        DiscretAlgoFactory? aFactory = DiscretAlgoFactory.DefaultFactory();
        Assert.NotNull(aFactory);

        DiscretRoot anAlgo = aFactory.CreateAlgorithm(myBox, 0.1, 0.5);
        Assert.NotNull(anAlgo);

        anAlgo.Perform();

        Assert.True(anAlgo.IsDone, "Meshing failed");
    }

    // Test that BRepMesh_DiscretFactory::Discret uses the new registry
    [Fact]
    public void DiscretFactory_UsesRegistry()
    {
        DiscretFactory aFactory = DiscretFactory.Get();

        DiscretRoot? anAlgo = aFactory.Discret(myBox, 0.1, 0.5);

        Assert.NotNull(anAlgo);
        anAlgo.Perform();
        Assert.True(anAlgo.IsDone, "Meshing failed");
    }

    // Test that SetDefaultName works with registry-based factories
    [Fact]
    public void DiscretFactory_SetDefaultName()
    {
        DiscretFactory aFactory = DiscretFactory.Get();

        bool isSuccess = aFactory.SetDefaultName("FastDiscret");
        Assert.True(isSuccess, "SetDefaultName(FastDiscret) failed");
        Assert.Equal("FastDiscret", aFactory.DefaultName());
    }

    // Test factory name uniqueness - registering same factory twice should not duplicate
    [Fact]
    public void RegisterFactory_Uniqueness()
    {
        NCollection_List<DiscretAlgoFactory> aFactories = DiscretAlgoFactory.Factories();

        // Count factories with name "FastDiscret"
        int aCount = 0;
        for (NCollection_List<DiscretAlgoFactory>.Iterator anIter = new(aFactories); anIter.More(); anIter.Next())
        {
            if (anIter.Value().Name() == "FastDiscret")
            {
                ++aCount;
            }
        }

        Assert.Equal(1, aCount);
    }

    // OCCT.NET: the two factories OCCT registers when TKMesh and TKXMesh load, in that order (decision 492)
    [Fact]
    public void Registry_holds_FastDiscret_then_XBRepMesh()
    {
        NCollection_List<DiscretAlgoFactory> aFactories = DiscretAlgoFactory.Factories();
        Assert.Equal(2, aFactories.Extent());
        Assert.IsType<IncrementalMeshFactory>(aFactories.First());
        Assert.IsType<Factory>(aFactories.Last());
        Assert.Same(aFactories.Last(), DiscretAlgoFactory.FindFactory("xbrepmesh"));

        DiscretRoot anAlgo = aFactories.Last().CreateAlgorithm(myBox, 0.2, 0.4);
        IncrementalMesh anIncMesh = Assert.IsType<IncrementalMesh>(anAlgo);
        Assert.Equal(0.2, anIncMesh.Parameters().Deflection);
        Assert.Equal(0.4, anIncMesh.Parameters().Angle);
        Assert.False(anIncMesh.Parameters().InParallel);

        Assert.False(DiscretFactory.Get().SetDefaultName("NoSuchMesher"));
        Assert.Equal("FastDiscret", DiscretFactory.Get().DefaultName());
    }
}
