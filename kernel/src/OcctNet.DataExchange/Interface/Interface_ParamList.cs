// Port of: src/DataExchange/TKXSBase/Interface/Interface_ParamList.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_ParamList.lxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_ParamList.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2008-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A list of file parameters, numbered from 1, which grows as values are set (OCCT's NCollection_DynamicArray of
/// <see cref="Interface_FileParameter"/>).
/// </summary>
public class Interface_ParamList : Standard_Transient
{
    private readonly List<Interface_FileParameter> myVector;

    /// <summary>Creates a list; <paramref name="theIncrement"/> is OCCT's memory block size (a capacity here).</summary>
    public Interface_ParamList(int theIncrement = 256)
    {
        myVector = new List<Interface_FileParameter>(Math.Max(theIncrement, 0));
    }

    /// <summary>The number of elements.</summary>
    public int Length() => myVector.Count;

    /// <summary>The lower bound: 1.</summary>
    public int Lower() => 1;

    /// <summary>The upper bound: the number of elements.</summary>
    public int Upper() => myVector.Count;

    /// <summary>
    /// Assigns a copy of the value to the <paramref name="Index"/>-th item, growing the list (with empty parameters)
    /// if needed.
    /// </summary>
    public void SetValue(int Index, Interface_FileParameter Value)
    {
        int ind = Index - 1;
        Grow(ind);
        myVector[ind] = new Interface_FileParameter(Value);
    }

    /// <summary>The <paramref name="Index"/>-th element.</summary>
    /// <exception cref="Standard_OutOfRange">Out of 1 to Length.</exception>
    public Interface_FileParameter Value(int Index)
    {
        int ind = Index - 1;
        if (ind < 0 || ind >= myVector.Count)
        {
            throw new Standard_OutOfRange("NCollection_DynamicArray::Value");
        }
        return myVector[ind];
    }

    /// <summary>The <paramref name="Index"/>-th element, to be changed in place; the list grows if needed.</summary>
    public Interface_FileParameter ChangeValue(int Index)
    {
        int ind = Index - 1;
        if (ind >= myVector.Count)
        {
            Interface_FileParameter aFP = new Interface_FileParameter();
            Grow(ind);
            myVector[ind] = aFP;
        }
        if (ind < 0)
        {
            throw new Standard_OutOfRange("NCollection_DynamicArray::ChangeValue");
        }
        return myVector[ind];
    }

    /// <summary>OCCT's <c>operator()</c>: <see cref="ChangeValue"/>.</summary>
    public Interface_FileParameter this[int Index] => ChangeValue(Index);

    /// <summary>Removes every element.</summary>
    public void Clear() => myVector.Clear();

    // NCollection_DynamicArray::SetValue beyond the end: the elements between are default-constructed
    private void Grow(int theIndex)
    {
        if (theIndex < 0)
        {
            throw new Standard_OutOfRange("NCollection_DynamicArray::SetValue");
        }
        while (myVector.Count <= theIndex)
        {
            myVector.Add(new Interface_FileParameter());
        }
    }
}
