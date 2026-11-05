// Port of: src/FoundationClasses/TKernel/GTests/NCollection_DataMap_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// NCollection_DataMap's binding methods, now Dictionary's and DictionaryExtensions' (unit 10J2): Bind (the indexer)
/// and Bound (SetValueRef) replace the value of a bound key, TryBind (TryAdd) and TryBound (GetOrAddValueRef) keep it
/// (decision 370). OCCT's GTests Rebind, TryBind_NewKey/ExistingKey and TryBound_NewKey/ExistingKey (the others test
/// C++ move semantics, Emplace and iterators the port doesn't have), and the port's own for Bound.
/// </summary>
public class NCollection_DataMapTests
{
    [Fact]
    public void Rebind()
    {
        var aMap = new Dictionary<int, string>();

        // First binding
        Assert.False(aMap.ContainsKey(1));
        aMap[1] = "One";
        Assert.Equal("One", aMap.GetValue(1));

        // Re-bind the same key with a different value
        Assert.True(aMap.ContainsKey(1));
        aMap[1] = "New One";
        Assert.Equal("New One", aMap.GetValue(1));

        // Size should still be 1
        Assert.Single(aMap);
    }

    [Fact]
    public void Bound_NewKey()
    {
        var aMap = new Dictionary<int, string>();
        ref string aRef = ref aMap.SetValueRef(1, "One");
        Assert.Equal("One", aRef);
        Assert.Single(aMap);

        // Modify through reference
        aRef = "Modified One";
        Assert.Equal("Modified One", aMap.GetValue(1));
    }

    [Fact]
    public void Bound_ExistingKey()
    {
        var aMap = new Dictionary<int, string>();
        aMap[1] = "One";

        // Bound on an existing key replaces the value (OCCT: "overrides value previously bound to the key")
        ref string aRef = ref aMap.SetValueRef(1, "New One");
        Assert.Equal("New One", aRef);
        Assert.Equal("New One", aMap.GetValue(1));
        Assert.Single(aMap);

        // Modify through reference
        aRef = "Modified One";
        Assert.Equal("Modified One", aMap.GetValue(1));
    }

    [Fact]
    public void Bound_replaces_a_reference_value_with_the_new_object()
    {
        // BRepCheck_Shell.Minimum binds a new status list for its shape and fills the one Bound returns
        var aMap = new Dictionary<string, NCollection_List<int>>();
        var anOld = new NCollection_List<int>();
        anOld.Append(1);
        aMap["shell"] = anOld;
        var aNew = new NCollection_List<int>();
        NCollection_List<int> aList = aMap.SetValueRef("shell", aNew);
        Assert.Same(aNew, aList);
        Assert.Same(aNew, aMap.GetValue("shell"));
        Assert.Equal(1, anOld.Extent());
    }

    [Fact]
    public void TryBind_NewKey()
    {
        var aMap = new Dictionary<int, string>();

        // TryBind on new key should succeed
        bool aResult = aMap.TryAdd(1, "One");
        Assert.True(aResult);
        Assert.Single(aMap);
        Assert.Equal("One", aMap.GetValue(1));

        // TryBind another new key
        aResult = aMap.TryAdd(2, "Two");
        Assert.True(aResult);
        Assert.Equal(2, aMap.Count);
        Assert.Equal("Two", aMap.GetValue(2));
    }

    [Fact]
    public void TryBind_ExistingKey()
    {
        var aMap = new Dictionary<int, string>();

        // First bind a key
        aMap[1] = "One";
        Assert.Equal("One", aMap.GetValue(1));

        // TryBind on existing key should fail and NOT replace the value
        bool aResult = aMap.TryAdd(1, "New One");
        Assert.False(aResult);
        Assert.Single(aMap);
        Assert.Equal("One", aMap.GetValue(1)); // Value should be unchanged
    }

    [Fact]
    public void TryBound_NewKey()
    {
        var aMap = new Dictionary<int, string>();

        // TryBound on new key should add and return reference to new item
        ref string aRef = ref aMap.GetOrAddValueRef(1, "One");
        Assert.Equal("One", aRef);
        Assert.Equal("One", aMap.GetValue(1));
        Assert.Single(aMap);

        // Modify through reference
        aRef = "Modified One";
        Assert.Equal("Modified One", aMap.GetValue(1));
    }

    [Fact]
    public void TryBound_ExistingKey()
    {
        var aMap = new Dictionary<int, string>();

        // First bind a key
        aMap[1] = "One";

        // TryBound on existing key should return reference to existing item (not replace)
        ref string aRef = ref aMap.GetOrAddValueRef(1, "New One");
        Assert.Equal("One", aRef);         // Should be the original value
        Assert.Equal("One", aMap.GetValue(1)); // Map should have original value
        Assert.Single(aMap);

        // Modify through reference
        aRef = "Modified One";
        Assert.Equal("Modified One", aMap.GetValue(1));
    }
}
