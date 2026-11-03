// Port of: src/DataExchange/TKXSBase/Interface/Interface_ParamList.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_ParamList.lxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_ParamList.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2008-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// A list of file parameters, numbered from 1, which grows as values are set (OCCT's NCollection_DynamicArray of
/// <see cref="FileParameter"/>).
/// </summary>
public class ParamList : Transient
{
    private readonly List<FileParameter> _vector;

    /// <summary>Creates a list; <paramref name="increment"/> is OCCT's memory block size (a capacity here).</summary>
    public ParamList(int increment = 256)
    {
        _vector = new List<FileParameter>(Math.Max(increment, 0));
    }

    /// <summary>The number of elements.</summary>
    public int Length() => _vector.Count;

    /// <summary>The lower bound: 1.</summary>
    public int Lower() => 1;

    /// <summary>The upper bound: the number of elements.</summary>
    public int Upper() => _vector.Count;

    /// <summary>
    /// Assigns a copy of the value to the <paramref name="index"/>-th item, growing the list (with empty parameters)
    /// if needed.
    /// </summary>
    public void SetValue(int index, FileParameter value)
    {
        int ind = index - 1;
        Grow(ind);
        _vector[ind] = new FileParameter(value);
    }

    /// <summary>The <paramref name="index"/>-th element.</summary>
    /// <exception cref="OutOfRangeException">Out of 1 to Length.</exception>
    public FileParameter Value(int index)
    {
        int ind = index - 1;
        if (ind < 0 || ind >= _vector.Count)
        {
            throw new OutOfRangeException("NCollection_DynamicArray::Value");
        }
        return _vector[ind];
    }

    /// <summary>The <paramref name="index"/>-th element, to be changed in place; the list grows if needed.</summary>
    public FileParameter ChangeValue(int index)
    {
        int ind = index - 1;
        if (ind >= _vector.Count)
        {
            FileParameter fp = new FileParameter();
            Grow(ind);
            _vector[ind] = fp;
        }
        if (ind < 0)
        {
            throw new OutOfRangeException("NCollection_DynamicArray::ChangeValue");
        }
        return _vector[ind];
    }

    /// <summary>OCCT's <c>operator()</c>: <see cref="ChangeValue"/>.</summary>
    public FileParameter this[int index] => ChangeValue(index);

    /// <summary>Removes every element.</summary>
    public void Clear() => _vector.Clear();

    // NCollection_DynamicArray::SetValue beyond the end: the elements between are default-constructed
    private void Grow(int index)
    {
        if (index < 0)
        {
            throw new OutOfRangeException("NCollection_DynamicArray::SetValue");
        }
        while (_vector.Count <= index)
        {
            _vector.Add(new FileParameter());
        }
    }
}
