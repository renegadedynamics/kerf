// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Substitution.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Substitution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Schematiq.Collections;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Replaces sub-shapes by lists of shapes (possibly empty) and rebuilds the shapes containing them.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Copy"/> hands out the shapes built here, so it freezes them
/// (docs/phase3-design.md, section 4.2); ported code that keeps building on them reads <see cref="InternalCopy"/>.
/// </remarks>
public sealed class BRepTools_Substitution
{
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMap = new(TopTools_ShapeMapHasher.Instance);

    /// <summary>An empty substitution.</summary>
    public BRepTools_Substitution()
    {
    }

    /// <summary>Resets all the fields.</summary>
    public void Clear()
    {
        myMap.Clear();
    }

    /// <summary>
    /// Records that <paramref name="OS"/> is to be replaced by <paramref name="NS"/>, which may be empty (the shape then
    /// disappears from its ancestors). A FORWARD item of <paramref name="NS"/> takes the orientation of
    /// <paramref name="OS"/> in its ancestors; another is reversed.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">The shape is already replaced.</exception>
    public void Substitute(in TopoDS_Shape OS, NCollection_List<TopoDS_Shape> NS)
    {
        Standard_ConstructionError_Raise_if(IsCopied(OS), "BRepTools_CutClue::Substitute");
        // OCCT.NET: OCCT's map stores a copy of the list
        myMap.Bind(OS, new NCollection_List<TopoDS_Shape>(NS));
    }

    /// <summary>Builds the new shape of <paramref name="S"/> if its sub-shapes have been replaced.</summary>
    public void Build(in TopoDS_Shape S)
    {
        if (IsCopied(S))
        {
            return;
        }

        BRep_Builder B = new();
        var iteS = new TopoDS_Iterator(S.Oriented(TopAbs_FORWARD));
        bool IsModified = false;
        bool HasSubShape = false;

        //------------------------------------------
        // look S is modified and build subshapes.
        //------------------------------------------
        for (; iteS.More(); iteS.Next())
        {
            TopoDS_Shape SS = iteS.Value();
            Build(SS);
            if (IsCopied(SS))
            {
                IsModified = true;
            }
        }

        TopoDS_Shape NewS = S.Oriented(TopAbs_FORWARD);
        if (IsModified)
        {
            //----------------------------------------
            // Rebuild S.
            //------------------------------------------
            NewS.EmptyCopy();

            if (NewS.ShapeType() == TopAbs_EDGE)
            {
                BRep_Tool.Range(TopoDS.Edge(S), out double f, out double l);
                B.Range(TopoDS.Edge(NewS), f, l);
            }

            iteS.Initialize(S.Oriented(TopAbs_FORWARD));
            //------------------------------------------
            // Add the copy of subshapes of S to NewS.
            //------------------------------------------
            for (; iteS.More(); iteS.Next())
            {
                TopAbs_Orientation OS = iteS.Value().Orientation();
                var L = new NCollection_List<TopoDS_Shape>(myMap[iteS.Value()]);
                var iteL = new NCollection_List<TopoDS_Shape>.Iterator(L);

                for (; iteL.More(); iteL.Next())
                {
                    TopoDS_Shape NSS = iteL.Value();
                    //------------------------------------------
                    // Rebuild NSS and add its copy to NewS.
                    //------------------------------------------
                    Build(NSS);

                    NCollection_List<TopoDS_Shape> NL = myMap[NSS];
                    TopAbs_Orientation NewOr = TopAbs.Compose(OS, NSS.Orientation());
                    var iteNL = new NCollection_List<TopoDS_Shape>.Iterator(NL);

                    for (; iteNL.More(); iteNL.Next())
                    {
                        B.Add(NewS, iteNL.Value().Oriented(NewOr));
                        HasSubShape = true;
                    }
                }
            }
            if (!HasSubShape)
            {
                if (NewS.ShapeType() == TopAbs_WIRE || NewS.ShapeType() == TopAbs_SHELL
                    || NewS.ShapeType() == TopAbs_SOLID || NewS.ShapeType() == TopAbs_COMPOUND)
                {
                    //-----------------------------------------------------------------
                    // Wire,Solid,Shell,Compound must have subshape else they disappear
                    //-----------------------------------------------------------------
                    NewS.Nullify();
                }
            }
        }
        var L2 = new NCollection_List<TopoDS_Shape>();
        //-------------------------------------------------------
        // NewS has the same orientation than S in its ancestors
        // so NewS is bound with orientation FORWARD.
        //-------------------------------------------------------
        if (!NewS.IsNull())
        {
            L2.Append(NewS.Oriented(TopAbs_FORWARD));
        }
        Substitute(S, L2);
    }

    /// <summary>True if <paramref name="S"/> has been replaced.</summary>
    public bool IsCopied(in TopoDS_Shape S)
    {
        if (myMap.IsBound(S))
        {
            if (myMap[S].IsEmpty())
            {
                return true;
            }
            else
            {
                return !S.IsSame(myMap[S].First());
            }
        }
        else
        {
            return false;
        }
    }

    /// <summary>The shapes substituted for <paramref name="S"/>, frozen.</summary>
    /// <exception cref="Standard_NoSuchObject">The shape hasn't been replaced.</exception>
    public ISchematiqList<TopoDS_Shape> Copy(in TopoDS_Shape S)
    {
        NCollection_List<TopoDS_Shape> aList = InternalCopy(S);
        foreach (TopoDS_Shape aS in aList)
        {
            aS.TShape()?.FreezeGraph();
        }
        return PackedList.Own([.. aList]);
    }

    /// <summary>The shapes substituted for <paramref name="S"/> (the map's own list), without freezing (ported code).</summary>
    internal NCollection_List<TopoDS_Shape> InternalCopy(in TopoDS_Shape S)
    {
        Standard_NoSuchObject_Raise_if(!IsCopied(S), "BRepTools_Substitution::Copy");
        return myMap[S];
    }
}
