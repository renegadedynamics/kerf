// Port of: src/DataExchange/TKXSBase/Interface/Interface_Category.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_Category.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
public class Interface_Category
{
    // OCCT's theCats() after Init(): a 0-based NCollection_DynamicArray
    private static readonly string[] theCats =
        ["Shape", "Drawing", "Structure", "Description", "Auxiliary", "Professional", "FEA", "Kinematics", "Piping"];

    private const string unspec = "unspecified";

    /// <summary>Creates a category tool, with no protocol yet.</summary>
    public Interface_Category()
    {
        myGTool = new Interface_GTool();
        Init();
    }

    /// <summary>Creates a category tool with a given protocol.</summary>
    public Interface_Category(Interface_Protocol? theProtocol)
    {
        myGTool = new Interface_GTool(theProtocol);
        Init();
    }

    /// <summary>Creates a category tool with a given GTool.</summary>
    public Interface_Category(Interface_GTool? theGTool)
    {
        myGTool = theGTool;
        Init();
    }

    /// <summary>Sets or changes the protocol.</summary>
    public void SetProtocol(Interface_Protocol? theProtocol)
    {
        myGTool!.SetProtocol(theProtocol);
    }

    /// <summary>Determines the category number of an entity in its context, by the general service CategoryNumber.</summary>
    public int CatNum(Standard_Transient? theEnt, Interface_ShareTool theShares)
    {
        if (theEnt is null)
        {
            return 0;
        }
        int CN;
        Interface_GeneralModule? aModule;
        if (!myGTool!.Select(theEnt, out aModule, out CN))
        {
            return 0;
        }
        return aModule!.CategoryNumber(CN, theEnt, theShares);
    }

    /// <summary>Clears the recorded list of category numbers.</summary>
    public void ClearNums()
    {
        myNum = null;
    }

    /// <summary>
    /// Computes the category number of each entity of a model and records it (entity number to category number), to
    /// be queried by <see cref="Num"/>. The model itself is not recorded.
    /// </summary>
    public void Compute(Interface_InterfaceModel? theModel, Interface_ShareTool theShares)
    {
        ClearNums();
        if (theModel is null)
        {
            return;
        }
        int CN, i, nb = theModel.NbEntities();
        myGTool!.Reservate(nb);
        if (nb == 0)
        {
            return;
        }
        myNum = new NCollection_HArray1<int>(1, nb);
        myNum.Init(0);
        for (i = 1; i <= nb; i++)
        {
            Standard_Transient? anEnt = theModel.Value(i);
            if (anEnt is null)
            {
                continue;
            }
            Interface_GeneralModule? aModule;
            if (!myGTool!.Select(anEnt, out aModule, out CN))
            {
                continue;
            }
            myNum.SetValue(i, aModule!.CategoryNumber(CN, anEnt, theShares));
        }
    }

    /// <summary>Returns the category number recorded for an entity number, 0 if out of range.</summary>
    public int Num(int theNumEnt)
    {
        if (myNum is null)
        {
            return 0;
        }
        if (theNumEnt < 1 || theNumEnt > myNum.Length())
        {
            return 0;
        }
        return myNum.Value(theNumEnt);
    }

    // OCCT.NET: AddCategory, which adds a category to the process-wide list, is left out (decision 1183)

    /// <summary>Returns the count of recorded categories.</summary>
    public static int NbCategories() => theCats.Length;

    /// <summary>Returns the name of a category from its number ("unspecified" out of range, "" if negative).</summary>
    public static string Name(int theNum)
    {
        if (theNum < 0)
        {
            return "";
        }
        if (theNum < 0 || theNum > theCats.Length - 1)
        {
            return unspec;
        }
        return theCats[theNum];
    }

    /// <summary>
    /// Returns the number of a category from its name: 0 if unknown, and also 0 for "Shape" (OCCT: the list is numbered
    /// from 0).
    /// </summary>
    public static int Number(string theName)
    {
        int i;
        for (i = 0; i <= theCats.Length - 1; i++)
        {
            if (theCats[i] == theName)
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

    private readonly Interface_GTool? myGTool;
    private NCollection_HArray1<int>? myNum;
}
