// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_Image.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_Image.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Stores the links between shapes and the shapes obtained from them (their images).</summary>
/// <remarks>
/// <para>
/// A working object of the offset algorithms, copied by value in OCCT (<see cref="Assign"/>). The maps iterate in
/// insertion order; <see cref="Filter"/> iterates one, and its result doesn't depend on the order.
/// </para>
/// <para>
/// OCCT's <see cref="Image"/> of a shape without images appends the shape to a function-level static list and returns
/// it, so the list grows with every such call (marked <c>// OCCT</c>, occt-issues.md). The port's list is per thread.
/// </para>
/// </remarks>
public sealed class BRepAlgo_Image
{
    private readonly NCollection_List<TopoDS_Shape> roots = new();
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> up = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> down = new(TopTools_ShapeMapHasher.Instance);

    [ThreadStatic]
    private static NCollection_List<TopoDS_Shape>? theNoImage;

    /// <summary>An empty image.</summary>
    public BRepAlgo_Image()
    {
    }

    /// <summary>OCCT.NET: makes this a copy of <paramref name="theOther"/> (OCCT's assignment).</summary>
    public void Assign(BRepAlgo_Image theOther)
    {
        if (ReferenceEquals(this, theOther))
        {
            return;
        }
        roots.Assign(theOther.roots);
        up.Clear();
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> anItem in theOther.up)
        {
            up.Bind(anItem.Key, anItem.Value);
        }
        down.Clear();
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anItem in theOther.down)
        {
            down.Bind(anItem.Key, new NCollection_List<TopoDS_Shape>(anItem.Value));
        }
    }

    /// <summary>Adds <paramref name="S"/> to the roots.</summary>
    public void SetRoot(in TopoDS_Shape S)
    {
        roots.Append(S);
    }

    /// <summary>Links <paramref name="NewS"/> as the image of <paramref name="OldS"/>.</summary>
    /// <exception cref="Standard_ConstructionError">OldS already has images.</exception>
    public void Bind(in TopoDS_Shape OldS, in TopoDS_Shape NewS)
    {
        if (down.IsBound(OldS))
        {
            throw new Standard_ConstructionError(" BRepAlgo_Image::Bind");
        }
        NCollection_List<TopoDS_Shape> L = new();
        down.Bind(OldS, L);
        down[OldS].Append(NewS);
        up.Bind(NewS, OldS);
    }

    /// <summary>Links the shapes of <paramref name="L"/> as the images of <paramref name="OldS"/>.</summary>
    /// <exception cref="Standard_ConstructionError">OldS already has images.</exception>
    public void Bind(in TopoDS_Shape OldS, NCollection_List<TopoDS_Shape> L)
    {
        if (HasImage(OldS))
        {
            throw new Standard_ConstructionError(" BRepAlgo_Image::Bind");
        }
        NCollection_List<TopoDS_Shape>.Iterator it = new(L);
        for (; it.More(); it.Next())
        {
            if (!HasImage(OldS))
            {
                Bind(OldS, it.Value());
            }
            else
            {
                Add(OldS, it.Value());
            }
        }
    }

    /// <summary>Removes everything.</summary>
    public void Clear()
    {
        roots.Clear();
        up.Clear();
        down.Clear();
    }

    /// <summary>Adds <paramref name="NewS"/> to the images of <paramref name="OldS"/>.</summary>
    /// <exception cref="Standard_ConstructionError">OldS has no images.</exception>
    public void Add(in TopoDS_Shape OldS, in TopoDS_Shape NewS)
    {
        if (!HasImage(OldS))
        {
            throw new Standard_ConstructionError(" BRepAlgo_Image::Add");
        }
        down[OldS].Append(NewS);
        up.Bind(NewS, OldS);
    }

    /// <summary>Adds the shapes of <paramref name="L"/> to the images of <paramref name="OldS"/>.</summary>
    public void Add(in TopoDS_Shape OldS, NCollection_List<TopoDS_Shape> L)
    {
        NCollection_List<TopoDS_Shape>.Iterator it = new(L);
        for (; it.More(); it.Next())
        {
            Add(OldS, it.Value());
        }
    }

    /// <summary>Removes the image <paramref name="S"/>.</summary>
    /// <exception cref="Standard_ConstructionError">S isn't an image.</exception>
    public void Remove(in TopoDS_Shape S)
    {
        if (!up.IsBound(S))
        {
            throw new Standard_ConstructionError(" BRepAlgo_Image::Remove");
        }
        TopoDS_Shape OldS = up[S];
        NCollection_List<TopoDS_Shape> L = down[OldS];
        NCollection_List<TopoDS_Shape>.Iterator it = new(L);
        while (it.More())
        {
            if (it.Value().IsSame(S))
            {
                L.Remove(it);
                break;
            }
            it.Next();
        }
        if (L.IsEmpty())
        {
            down.UnBind(OldS);
        }
        up.UnBind(S);
    }

    /// <summary>The roots.</summary>
    public NCollection_List<TopoDS_Shape> Roots() => roots;

    /// <summary>Whether <paramref name="S"/> is an image.</summary>
    public bool IsImage(in TopoDS_Shape S) => up.IsBound(S);

    /// <summary>The shape <paramref name="S"/> is an image of.</summary>
    /// <exception cref="Standard_ConstructionError">S isn't an image.</exception>
    public TopoDS_Shape ImageFrom(in TopoDS_Shape S)
    {
        if (!up.IsBound(S))
        {
            throw new Standard_ConstructionError(" BRepAlgo_Image::ImageFrom");
        }
        return up[S];
    }

    /// <summary>The first generator of <paramref name="S"/>.</summary>
    /// <exception cref="Standard_ConstructionError">S isn't an image.</exception>
    public TopoDS_Shape Root(in TopoDS_Shape S)
    {
        if (!up.IsBound(S))
        {
            throw new Standard_ConstructionError(" BRepAlgo_Image::FirstImageFrom");
        }

        TopoDS_Shape S1 = up[S];
        TopoDS_Shape S2 = S;

        if (S1.IsSame(S2))
        {
            return up[S];
        }

        while (up.IsBound(S1))
        {
            S2 = S1;
            S1 = up[S1];
            if (S1.IsSame(S2))
            {
                break;
            }
        }
        return up[S2];
    }

    /// <summary>Whether <paramref name="S"/> has images.</summary>
    public bool HasImage(in TopoDS_Shape S) => down.IsBound(S);

    /// <summary>The images of <paramref name="S"/>; for a shape without images, a list holding it (see the remarks).</summary>
    public NCollection_List<TopoDS_Shape> Image(in TopoDS_Shape S)
    {
        if (!HasImage(S))
        {
            // OCCT: the static list keeps every shape appended by earlier calls
            theNoImage ??= new NCollection_List<TopoDS_Shape>();
            theNoImage.Append(S);
            return theNoImage;
        }
        return down[S];
    }

    /// <summary>
    /// Appends to <paramref name="L"/> the images of the images ... of <paramref name="S"/> (S itself if it has none).
    /// </summary>
    public void LastImage(in TopoDS_Shape S, NCollection_List<TopoDS_Shape> L)
    {
        if (!down.IsBound(S))
        {
            L.Append(S);
        }
        else
        {
            NCollection_List<TopoDS_Shape>.Iterator it = new(down[S]);
            for (; it.More(); it.Next())
            {
                if (it.Value().IsSame(S))
                {
                    L.Append(S);
                }
                else
                {
                    LastImage(it.Value(), L);
                }
            }
        }
    }

    /// <summary>Keeps only the links between the roots and their last images.</summary>
    public void Compact()
    {
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> M = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator it = new(roots);
        for (; it.More(); it.Next())
        {
            TopoDS_Shape S = it.Value();
            NCollection_List<TopoDS_Shape> LI = new();
            if (HasImage(S))
            {
                LastImage(S, LI);
            }
            M.Bind(S, LI);
        }
        up.Clear();
        down.Clear();
        for (it.Initialize(roots); it.More(); it.Next())
        {
            if (M.IsBound(it.Value()))
            {
                Bind(it.Value(), M[it.Value()]);
            }
        }
    }

    /// <summary>
    /// Removes the images of type <paramref name="T"/> that aren't sub-shapes of <paramref name="S"/>; call
    /// <see cref="Compact"/> first.
    /// </summary>
    public void Filter(in TopoDS_Shape S, TopAbs_ShapeEnum T)
    {
        TopExp_Explorer exp = new(S, T);
        NCollection_Map<TopoDS_Shape> M = new(TopTools_ShapeMapHasher.Instance);
        for (; exp.More(); exp.Next())
        {
            M.Add(exp.Current());
        }
        bool Change = true;
        while (Change)
        {
            Change = false;
            foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> mit in up)
            {
                TopoDS_Shape aS = mit.Key;
                if (aS.ShapeType() == T && !M.Contains(aS))
                {
                    Remove(aS);
                    Change = true;
                    break;
                }
            }
        }
    }

    /// <summary>Removes the root <paramref name="Root"/> from the roots, and its images.</summary>
    public void RemoveRoot(in TopoDS_Shape Root)
    {
        bool isRemoved = false;
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(roots); it.More(); it.Next())
        {
            if (Root.IsSame(it.Value()))
            {
                roots.Remove(it);
                isRemoved = true;
                break;
            }
        }

        if (!isRemoved)
        {
            return;
        }

        if (down.Find(Root, out NCollection_List<TopoDS_Shape> pNewS))
        {
            for (NCollection_List<TopoDS_Shape>.Iterator it = new(pNewS); it.More(); it.Next())
            {
                if (up.Find(it.Value(), out TopoDS_Shape pOldS) && pOldS.IsSame(Root))
                {
                    up.UnBind(it.Value());
                }
            }
            down.UnBind(Root);
        }
    }

    /// <summary>
    /// Replaces the root <paramref name="OldRoot"/> by <paramref name="NewRoot"/>: the images of the old one become
    /// images of the new one, and the old one is removed.
    /// </summary>
    public void ReplaceRoot(in TopoDS_Shape OldRoot, in TopoDS_Shape NewRoot)
    {
        if (!HasImage(OldRoot))
        {
            return;
        }

        NCollection_List<TopoDS_Shape> aLImage = Image(OldRoot);
        if (HasImage(NewRoot))
        {
            Add(NewRoot, aLImage);
        }
        else
        {
            Bind(NewRoot, aLImage);
        }

        SetRoot(NewRoot);
        RemoveRoot(OldRoot);
    }
}
