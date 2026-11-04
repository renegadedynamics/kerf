// Port of: src/DataExchange/TKXSBase/Interface/Interface_Category.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_Category.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// Manages categories: a category is defined by a name and a number, a rough classification of entities, less precise
/// than a type (Shape, Drawing, Structure, Description, Auxiliary, Professional, FEA, Kinematics, Piping). Also
/// computes then quickly queries the category numbers of a whole model.
/// </summary>
/// <remarks>
/// OCCT keeps the categories in a process-wide list, filled once by <see cref="Init"/> and extensible by
/// <c>AddCategory</c>. Only <c>Init</c> adds to it, so the port keeps that list as a constant and leaves
/// <c>AddCategory</c> out (decision 1183). As OCCT's list is a dynamic array numbered from 0, "Shape" has the number
/// 0, which <see cref="Number"/> also returns for an unknown name: reproduced.
/// </remarks>
internal class Category
{
    // OCCT's theCats() after Init(): a 0-based NCollection_DynamicArray
    private static readonly string[] Cats =
        ["Shape", "Drawing", "Structure", "Description", "Auxiliary", "Professional", "FEA", "Kinematics", "Piping"];

    private const string Unspec = "unspecified";

    /// <summary>Creates a category tool, with no protocol yet.</summary>
    public Category()
    {
        _gTool = new GTool();
        Init();
    }

    /// <summary>Creates a category tool with a given protocol.</summary>
    public Category(Protocol? protocol)
    {
        _gTool = new GTool(protocol);
        Init();
    }

    /// <summary>Creates a category tool with a given GTool.</summary>
    public Category(GTool? gTool)
    {
        _gTool = gTool;
        Init();
    }

    /// <summary>Sets or changes the protocol.</summary>
    public void SetProtocol(Protocol? protocol)
    {
        _gTool!.SetProtocol(protocol);
    }

    /// <summary>Determines the category number of an entity in its context, by the general service CategoryNumber.</summary>
    public int CatNum(Transient? ent, ShareTool shares)
    {
        if (ent is null)
        {
            return 0;
        }
        int cn;
        GeneralModule? module;
        if (!_gTool!.Select(ent, out module, out cn))
        {
            return 0;
        }
        return module!.CategoryNumber(cn, ent, shares);
    }

    /// <summary>Clears the recorded list of category numbers.</summary>
    public void ClearNums()
    {
        _num = null;
    }

    /// <summary>
    /// Computes the category number of each entity of a model and records it (entity number to category number), to
    /// be queried by <see cref="Num"/>. The model itself is not recorded.
    /// </summary>
    public void Compute(InterfaceModel? model, ShareTool shares)
    {
        ClearNums();
        if (model is null)
        {
            return;
        }
        int cn, i, nb = model.EntityCount;
        _gTool!.Reservate(nb);
        if (nb == 0)
        {
            return;
        }
        _num = new NCollection_HArray1<int>(1, nb);
        _num.Init(0);
        for (i = 1; i <= nb; i++)
        {
            Transient? ent = model.Value(i);
            if (ent is null)
            {
                continue;
            }
            GeneralModule? module;
            if (!_gTool!.Select(ent, out module, out cn))
            {
                continue;
            }
            _num.SetValue(i, module!.CategoryNumber(cn, ent, shares));
        }
    }

    /// <summary>Returns the category number recorded for an entity number, 0 if out of range.</summary>
    public int Num(int numEnt)
    {
        if (_num is null)
        {
            return 0;
        }
        if (numEnt < 1 || numEnt > _num.Length())
        {
            return 0;
        }
        return _num.Value(numEnt);
    }

    // OCCT.NET: AddCategory, which adds a category to the process-wide list, is left out (decision 1183)

    /// <summary>Returns the count of recorded categories.</summary>
    public static int CategoryCount => Cats.Length;

    /// <summary>Returns the name of a category from its number ("unspecified" out of range, "" if negative).</summary>
    public static string Name(int num)
    {
        if (num < 0)
        {
            return "";
        }
        if (num < 0 || num > Cats.Length - 1)
        {
            return Unspec;
        }
        return Cats[num];
    }

    /// <summary>
    /// Returns the number of a category from its name: 0 if unknown, and also 0 for "Shape" (OCCT: the list is numbered
    /// from 0).
    /// </summary>
    public static int Number(string name)
    {
        int i;
        for (i = 0; i <= Cats.Length - 1; i++)
        {
            if (Cats[i] == name)
            {
                return i;
            }
        }
        return 0;
    }

    /// <summary>Default initialisation: the standard categories (a constant list in the port, so nothing to do).</summary>
    public static void Init()
    {
    }

    private readonly GTool? _gTool;
    private NCollection_HArray1<int>? _num;
}
