// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Substitution.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Substitution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Topology.Tools;

/// <summary>
/// Replaces sub-shapes by lists of shapes (possibly empty) and rebuilds the shapes containing them.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Copy"/> hands out the shapes built here, so it freezes them
/// (docs/phase3-design.md, section 4.2); ported code that keeps building on them reads <see cref="InternalCopy"/>.
/// </remarks>
internal sealed class Substitution
{
    private readonly Dictionary<Shape, NCollection_List<Shape>> _map = new(ShapeHasher.Instance);

    /// <summary>An empty substitution.</summary>
    public Substitution()
    {
    }

    /// <summary>Resets all the fields.</summary>
    public void Clear()
    {
        _map.Clear();
    }

    /// <summary>
    /// Records that <paramref name="os"/> is to be replaced by <paramref name="ns"/>, which may be empty (the shape then
    /// disappears from its ancestors). A FORWARD item of <paramref name="ns"/> takes the orientation of
    /// <paramref name="os"/> in its ancestors; another is reversed.
    /// </summary>
    /// <exception cref="ConstructionErrorException">The shape is already replaced.</exception>
    public void Substitute(in Shape os, NCollection_List<Shape> ns)
    {
        Standard_ConstructionError_Raise_if(IsCopied(os), "BRepTools_CutClue::Substitute");
        // Kerf: OCCT's map stores a copy of the list
        _map[os] = new NCollection_List<Shape>(ns);
    }

    /// <summary>Builds the new shape of <paramref name="s"/> if its sub-shapes have been replaced.</summary>
    public void Build(in Shape s)
    {
        if (IsCopied(s))
        {
            return;
        }

        BrepBuilder b = new();
        var iteS = new Iterator(s.Oriented(Forward));
        bool isModified = false;
        bool hasSubShape = false;

        //------------------------------------------
        // look S is modified and build subshapes.
        //------------------------------------------
        for (; iteS.More(); iteS.Next())
        {
            Shape ss = iteS.Value();
            Build(ss);
            if (IsCopied(ss))
            {
                isModified = true;
            }
        }

        Shape newS = s.Oriented(Forward);
        if (isModified)
        {
            //----------------------------------------
            // Rebuild S.
            //------------------------------------------
            newS.EmptyCopy();

            if (newS.Type == ShapeType.Edge)
            {
                Tool.Range(ShapeCasts.Edge(s), out double f, out double l);
                b.Range(ShapeCasts.Edge(newS), f, l);
            }

            iteS.Initialize(s.Oriented(Forward));
            //------------------------------------------
            // Add the copy of subshapes of S to NewS.
            //------------------------------------------
            for (; iteS.More(); iteS.Next())
            {
                Orientation os = iteS.Value().Orientation;
                var L = new NCollection_List<Shape>(_map.GetValue(iteS.Value()));
                var iteL = new NCollection_List<Shape>.Iterator(L);

                for (; iteL.More(); iteL.Next())
                {
                    Shape nss = iteL.Value();
                    //------------------------------------------
                    // Rebuild NSS and add its copy to NewS.
                    //------------------------------------------
                    Build(nss);

                    NCollection_List<Shape> nl = _map.GetValue(nss);
                    Orientation newOr = global::Kerf.Engine.Topology.TopAbs.Compose(os, nss.Orientation);
                    var iteNL = new NCollection_List<Shape>.Iterator(nl);

                    for (; iteNL.More(); iteNL.Next())
                    {
                        b.Add(newS, iteNL.Value().Oriented(newOr));
                        hasSubShape = true;
                    }
                }
            }
            if (!hasSubShape)
            {
                if (newS.Type == ShapeType.Wire || newS.Type == ShapeType.Shell
                    || newS.Type == ShapeType.Solid || newS.Type == ShapeType.Compound)
                {
                    //-----------------------------------------------------------------
                    // Wire,Solid,Shell,Compound must have subshape else they disappear
                    //-----------------------------------------------------------------
                    newS.Nullify();
                }
            }
        }
        var l2 = new NCollection_List<Shape>();
        //-------------------------------------------------------
        // NewS has the same orientation than S in its ancestors
        // so NewS is bound with orientation FORWARD.
        //-------------------------------------------------------
        if (!newS.IsNull)
        {
            l2.Append(newS.Oriented(Forward));
        }
        Substitute(s, l2);
    }

    /// <summary>True if <paramref name="s"/> has been replaced.</summary>
    public bool IsCopied(in Shape s)
    {
        if (_map.ContainsKey(s))
        {
            if (_map.GetValue(s).IsEmpty())
            {
                return true;
            }
            else
            {
                return !s.IsSame(_map.GetValue(s).First());
            }
        }
        else
        {
            return false;
        }
    }

    /// <summary>The shapes substituted for <paramref name="s"/>, frozen.</summary>
    /// <exception cref="NoSuchObjectException">The shape hasn't been replaced.</exception>
    public IReadOnlyList<Shape> Copy(in Shape s)
    {
        NCollection_List<Shape> list = InternalCopy(s);
        foreach (Shape aS in list)
        {
            aS.TShape()?.FreezeGraph();
        }
        return ImmutableArrays.Own([.. list]);
    }

    /// <summary>The shapes substituted for <paramref name="s"/> (the map's own list), without freezing (ported code).</summary>
    internal NCollection_List<Shape> InternalCopy(in Shape s)
    {
        Standard_NoSuchObject_Raise_if(!IsCopied(s), "BRepTools_Substitution::Copy");
        return _map.GetValue(s);
    }
}
