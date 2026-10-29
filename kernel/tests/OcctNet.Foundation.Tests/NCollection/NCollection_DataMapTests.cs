// Port of: src/FoundationClasses/TKernel/GTests/NCollection_DataMap_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

/// <summary>
/// NCollection_DataMap's binding methods: Bind and Bound replace the value of a bound key, TryBind and TryBound keep
/// it (decision 370). OCCT's GTests Rebind, TryBind_NewKey/ExistingKey and TryBound_NewKey/ExistingKey (the others test
/// C++ move semantics, Emplace and iterators the port doesn't have), and the port's own for Bound.
/// </summary>
public class NCollection_DataMapTests
{
    [Fact]
    public void Rebind()
    {
        var aMap = new NCollection_DataMap<int, string>();

        // First binding
        Assert.True(aMap.Bind(1, "One"));
        Assert.Equal("One", aMap.Find(1));

        // Re-bind the same key with a different value
        Assert.False(aMap.Bind(1, "New One"));
        Assert.Equal("New One", aMap.Find(1));

        // Size should still be 1
        Assert.Equal(1, aMap.Size());
    }

    [Fact]
    public void Bound_NewKey()
    {
        var aMap = new NCollection_DataMap<int, string>();
        ref string aRef = ref aMap.Bound(1, "One");
        Assert.Equal("One", aRef);
        Assert.Equal(1, aMap.Size());

        // Modify through reference
        aRef = "Modified One";
        Assert.Equal("Modified One", aMap.Find(1));
    }

    [Fact]
    public void Bound_ExistingKey()
    {
        var aMap = new NCollection_DataMap<int, string>();
        aMap.Bind(1, "One");

        // Bound on an existing key replaces the value (OCCT: "overrides value previously bound to the key")
        ref string aRef = ref aMap.Bound(1, "New One");
        Assert.Equal("New One", aRef);
        Assert.Equal("New One", aMap.Find(1));
        Assert.Equal(1, aMap.Size());

        // Modify through reference
        aRef = "Modified One";
        Assert.Equal("Modified One", aMap.Find(1));
    }

    [Fact]
    public void Bound_replaces_a_reference_value_with_the_new_object()
    {
        // BRepCheck_Shell.Minimum binds a new status list for its shape and fills the one Bound returns
        var aMap = new NCollection_DataMap<string, NCollection_List<int>>();
        var anOld = new NCollection_List<int>();
        anOld.Append(1);
        aMap.Bind("shell", anOld);
        var aNew = new NCollection_List<int>();
        NCollection_List<int> aList = aMap.Bound("shell", aNew);
        Assert.Same(aNew, aList);
        Assert.Same(aNew, aMap.Find("shell"));
        Assert.Equal(1, anOld.Extent());
    }

    [Fact]
    public void TryBind_NewKey()
    {
        var aMap = new NCollection_DataMap<int, string>();

        // TryBind on new key should succeed
        bool aResult = aMap.TryBind(1, "One");
        Assert.True(aResult);
        Assert.Equal(1, aMap.Size());
        Assert.Equal("One", aMap.Find(1));

        // TryBind another new key
        aResult = aMap.TryBind(2, "Two");
        Assert.True(aResult);
        Assert.Equal(2, aMap.Size());
        Assert.Equal("Two", aMap.Find(2));
    }

    [Fact]
    public void TryBind_ExistingKey()
    {
        var aMap = new NCollection_DataMap<int, string>();

        // First bind a key
        aMap.Bind(1, "One");
        Assert.Equal("One", aMap.Find(1));

        // TryBind on existing key should fail and NOT replace the value
        bool aResult = aMap.TryBind(1, "New One");
        Assert.False(aResult);
        Assert.Equal(1, aMap.Size());
        Assert.Equal("One", aMap.Find(1)); // Value should be unchanged
    }

    [Fact]
    public void TryBound_NewKey()
    {
        var aMap = new NCollection_DataMap<int, string>();

        // TryBound on new key should add and return reference to new item
        ref string aRef = ref aMap.TryBound(1, "One");
        Assert.Equal("One", aRef);
        Assert.Equal("One", aMap.Find(1));
        Assert.Equal(1, aMap.Size());

        // Modify through reference
        aRef = "Modified One";
        Assert.Equal("Modified One", aMap.Find(1));
    }

    [Fact]
    public void TryBound_ExistingKey()
    {
        var aMap = new NCollection_DataMap<int, string>();

        // First bind a key
        aMap.Bind(1, "One");

        // TryBound on existing key should return reference to existing item (not replace)
        ref string aRef = ref aMap.TryBound(1, "New One");
        Assert.Equal("One", aRef);         // Should be the original value
        Assert.Equal("One", aMap.Find(1)); // Map should have original value
        Assert.Equal(1, aMap.Size());

        // Modify through reference
        aRef = "Modified One";
        Assert.Equal("Modified One", aMap.Find(1));
    }
}
