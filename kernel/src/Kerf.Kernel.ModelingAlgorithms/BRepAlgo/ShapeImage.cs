// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_Image.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_Image.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans.Algo;

/// <summary>Stores the links between shapes and the shapes obtained from them (their images).</summary>
/// <remarks>
/// <para>
/// A working object of the offset algorithms, copied by value in OCCT (<see cref="Assign"/>). The maps iterate in
/// insertion order; <see cref="Filter"/> iterates one, and its result doesn't depend on the order.
/// </para>
/// <para>
/// OCCT's <see cref="Image"/> of a shape without images appends the shape to a function-level static list and returns
/// it, so the list grows with every such call (marked <c>// OCCT</c>, occt-issues.md). The port's list is the image
/// object's own (unit 10I; per thread before), so what one operation appended never reaches another.
/// </para>
/// </remarks>
public sealed class ShapeImage
{
    private readonly NCollection_List<Shape> _roots = new();
    private readonly NCollection_DataMap<Shape, Shape> _up = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> _down = new(ShapeHasher.Instance);

    private NCollection_List<Shape>? _noImage;

    /// <summary>An empty image.</summary>
    public ShapeImage()
    {
    }

    /// <summary>OCCT.NET: makes this a copy of <paramref name="other"/> (OCCT's assignment).</summary>
    public void Assign(ShapeImage other)
    {
        if (ReferenceEquals(this, other))
        {
            return;
        }
        _roots.Assign(other._roots);
        _up.Clear();
        foreach (KeyValuePair<Shape, Shape> item in other._up)
        {
            _up.Bind(item.Key, item.Value);
        }
        _down.Clear();
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> item in other._down)
        {
            _down.Bind(item.Key, new NCollection_List<Shape>(item.Value));
        }
    }

    /// <summary>Adds <paramref name="s"/> to the roots.</summary>
    public void SetRoot(in Shape s)
    {
        _roots.Append(s);
    }

    /// <summary>Links <paramref name="newS"/> as the image of <paramref name="oldS"/>.</summary>
    /// <exception cref="ConstructionErrorException">OldS already has images.</exception>
    public void Bind(in Shape oldS, in Shape newS)
    {
        if (_down.IsBound(oldS))
        {
            throw new ConstructionErrorException(" BRepAlgo_Image::Bind");
        }
        NCollection_List<Shape> l = new();
        _down.Bind(oldS, l);
        _down[oldS].Append(newS);
        _up.Bind(newS, oldS);
    }

    /// <summary>Links the shapes of <paramref name="l"/> as the images of <paramref name="oldS"/>.</summary>
    /// <exception cref="ConstructionErrorException">OldS already has images.</exception>
    public void Bind(in Shape oldS, NCollection_List<Shape> l)
    {
        if (HasImage(oldS))
        {
            throw new ConstructionErrorException(" BRepAlgo_Image::Bind");
        }
        NCollection_List<Shape>.Iterator it = new(l);
        for (; it.More(); it.Next())
        {
            if (!HasImage(oldS))
            {
                Bind(oldS, it.Value());
            }
            else
            {
                Add(oldS, it.Value());
            }
        }
    }

    /// <summary>Removes everything.</summary>
    public void Clear()
    {
        _roots.Clear();
        _up.Clear();
        _down.Clear();
    }

    /// <summary>Adds <paramref name="newS"/> to the images of <paramref name="oldS"/>.</summary>
    /// <exception cref="ConstructionErrorException">OldS has no images.</exception>
    public void Add(in Shape oldS, in Shape newS)
    {
        if (!HasImage(oldS))
        {
            throw new ConstructionErrorException(" BRepAlgo_Image::Add");
        }
        _down[oldS].Append(newS);
        _up.Bind(newS, oldS);
    }

    /// <summary>Adds the shapes of <paramref name="l"/> to the images of <paramref name="oldS"/>.</summary>
    public void Add(in Shape oldS, NCollection_List<Shape> l)
    {
        NCollection_List<Shape>.Iterator it = new(l);
        for (; it.More(); it.Next())
        {
            Add(oldS, it.Value());
        }
    }

    /// <summary>Removes the image <paramref name="s"/>.</summary>
    /// <exception cref="ConstructionErrorException">S isn't an image.</exception>
    public void Remove(in Shape s)
    {
        if (!_up.IsBound(s))
        {
            throw new ConstructionErrorException(" BRepAlgo_Image::Remove");
        }
        Shape oldS = _up[s];
        NCollection_List<Shape> l = _down[oldS];
        NCollection_List<Shape>.Iterator it = new(l);
        while (it.More())
        {
            if (it.Value().IsSame(s))
            {
                l.Remove(it);
                break;
            }
            it.Next();
        }
        if (l.IsEmpty())
        {
            _down.UnBind(oldS);
        }
        _up.UnBind(s);
    }

    /// <summary>The roots.</summary>
    public NCollection_List<Shape> Roots() => _roots;

    /// <summary>Whether <paramref name="s"/> is an image.</summary>
    public bool IsImage(in Shape s) => _up.IsBound(s);

    /// <summary>The shape <paramref name="s"/> is an image of.</summary>
    /// <exception cref="ConstructionErrorException">S isn't an image.</exception>
    public Shape ImageFrom(in Shape s)
    {
        if (!_up.IsBound(s))
        {
            throw new ConstructionErrorException(" BRepAlgo_Image::ImageFrom");
        }
        return _up[s];
    }

    /// <summary>The first generator of <paramref name="s"/>.</summary>
    /// <exception cref="ConstructionErrorException">S isn't an image.</exception>
    public Shape Root(in Shape s)
    {
        if (!_up.IsBound(s))
        {
            throw new ConstructionErrorException(" BRepAlgo_Image::FirstImageFrom");
        }

        Shape s1 = _up[s];
        Shape s2 = s;

        if (s1.IsSame(s2))
        {
            return _up[s];
        }

        while (_up.IsBound(s1))
        {
            s2 = s1;
            s1 = _up[s1];
            if (s1.IsSame(s2))
            {
                break;
            }
        }
        return _up[s2];
    }

    /// <summary>Whether <paramref name="s"/> has images.</summary>
    public bool HasImage(in Shape s) => _down.IsBound(s);

    /// <summary>The images of <paramref name="s"/>; for a shape without images, a list holding it (see the remarks).</summary>
    public NCollection_List<Shape> Image(in Shape s)
    {
        if (!HasImage(s))
        {
            // OCCT: the static list keeps every shape appended by earlier calls
            _noImage ??= new NCollection_List<Shape>();
            _noImage.Append(s);
            return _noImage;
        }
        return _down[s];
    }

    /// <summary>
    /// Appends to <paramref name="l"/> the images of the images ... of <paramref name="s"/> (S itself if it has none).
    /// </summary>
    public void LastImage(in Shape s, NCollection_List<Shape> l)
    {
        if (!_down.IsBound(s))
        {
            l.Append(s);
        }
        else
        {
            NCollection_List<Shape>.Iterator it = new(_down[s]);
            for (; it.More(); it.Next())
            {
                if (it.Value().IsSame(s))
                {
                    l.Append(s);
                }
                else
                {
                    LastImage(it.Value(), l);
                }
            }
        }
    }

    /// <summary>Keeps only the links between the roots and their last images.</summary>
    public void Compact()
    {
        NCollection_DataMap<Shape, NCollection_List<Shape>> m = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator it = new(_roots);
        for (; it.More(); it.Next())
        {
            Shape s = it.Value();
            NCollection_List<Shape> li = new();
            if (HasImage(s))
            {
                LastImage(s, li);
            }
            m.Bind(s, li);
        }
        _up.Clear();
        _down.Clear();
        for (it.Initialize(_roots); it.More(); it.Next())
        {
            if (m.IsBound(it.Value()))
            {
                Bind(it.Value(), m[it.Value()]);
            }
        }
    }

    /// <summary>
    /// Removes the images of type <paramref name="t"/> that aren't sub-shapes of <paramref name="s"/>; call
    /// <see cref="Compact"/> first.
    /// </summary>
    public void Filter(in Shape s, ShapeType t)
    {
        Explorer exp = new(s, t);
        NCollection_Map<Shape> m = new(ShapeHasher.Instance);
        for (; exp.More(); exp.Next())
        {
            m.Add(exp.Current());
        }
        bool change = true;
        while (change)
        {
            change = false;
            foreach (KeyValuePair<Shape, Shape> mit in _up)
            {
                Shape aS = mit.Key;
                if (aS.ShapeType() == t && !m.Contains(aS))
                {
                    Remove(aS);
                    change = true;
                    break;
                }
            }
        }
    }

    /// <summary>Removes the root <paramref name="root"/> from the roots, and its images.</summary>
    public void RemoveRoot(in Shape root)
    {
        bool isRemoved = false;
        for (NCollection_List<Shape>.Iterator it = new(_roots); it.More(); it.Next())
        {
            if (root.IsSame(it.Value()))
            {
                _roots.Remove(it);
                isRemoved = true;
                break;
            }
        }

        if (!isRemoved)
        {
            return;
        }

        if (_down.Find(root, out NCollection_List<Shape> pNewS))
        {
            for (NCollection_List<Shape>.Iterator it = new(pNewS); it.More(); it.Next())
            {
                if (_up.Find(it.Value(), out Shape pOldS) && pOldS.IsSame(root))
                {
                    _up.UnBind(it.Value());
                }
            }
            _down.UnBind(root);
        }
    }

    /// <summary>
    /// Replaces the root <paramref name="oldRoot"/> by <paramref name="newRoot"/>: the images of the old one become
    /// images of the new one, and the old one is removed.
    /// </summary>
    public void ReplaceRoot(in Shape oldRoot, in Shape newRoot)
    {
        if (!HasImage(oldRoot))
        {
            return;
        }

        NCollection_List<Shape> lImage = Image(oldRoot);
        if (HasImage(newRoot))
        {
            Add(newRoot, lImage);
        }
        else
        {
            Bind(newRoot, lImage);
        }

        SetRoot(newRoot);
        RemoveRoot(oldRoot);
    }
}
