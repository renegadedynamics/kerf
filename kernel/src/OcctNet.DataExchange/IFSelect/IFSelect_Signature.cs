// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Signature.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Signature.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// Signature provides the basic service used by the classes SelectSignature and Counter (i.e. Name, Value),
/// which is : - for an entity in a model, give a characteristic string, its signature This string has not to
/// be unique in the model, but gives a value for such or such important feature. Examples : Dynamic Type;
/// Category; etc
/// </summary>
public abstract class IFSelect_Signature : Interface_SignType
{

    /// <summary>
    /// Sets the information data to tell "integer cases" with possible min and max values To be called when
    /// creating
    /// </summary>
    public void SetIntCase(bool hasmin, int valmin, bool hasmax, int valmax)
    {
        thecasi[0] = 1;
        if (hasmin)
        {
            thecasi[0] += 2;
            thecasi[1] = valmin;
        }
        if (hasmax)
        {
            thecasi[0] += 4;
            thecasi[2] = valmax;
        }
    }

    /// <summary>
    /// Tells if this Signature gives integer values and returns values from SetIntCase if True
    /// </summary>
    public bool IsIntCase(out bool hasmin, out int valmin, out bool hasmax, out int valmax)
    {
        hasmin = hasmax = false;
        valmin = valmax = 0;
        if (thecasi[0] == 0)
        {
            return false;
        }
        if ((thecasi[0] & 2) != 0)
        {
            hasmin = true;
            valmin = thecasi[1];
        }
        if ((thecasi[0] & 4) != 0)
        {
            hasmax = true;
            valmax = thecasi[2];
        }
        return true;
    }

    /// <summary>
    /// Adds a possible case To be called when creating, IF the list of possible cases for Value is known when
    /// starting For instance, for CDL types, rather do not fill this, but for a specific enumeration (such as a
    /// status), can be used
    /// </summary>
    public void AddCase(string acase)
    {
        if (thecasl is null)
        {
            thecasl = new NCollection_HSequence<string>();
        }
        string scase = acase;
        thecasl.Append(scase);
    }

    /// <summary>
    /// Returns the predefined list of possible cases, filled by AddCase Null Handle if no predefined list
    /// (hence, to be counted) Useful to filter on really possible vase, for instance, or for a help
    /// </summary>
    public NCollection_HSequence<string>? CaseList()
    {
        return thecasl;
    }

    /// <summary>
    /// Returns an identification of the Signature (a word), given at initialization time Returns the Signature
    /// for a Transient object. It is specific of each sub-class of Signature. For a Null Handle, it should
    /// provide "" It can work with the model which contains the entity
    /// </summary>
    public override string Name()
    {
        return thename;
    }

    /// <summary>
    /// The label of a Signature uses its name as follow : "Signature : &lt;name&gt;"
    /// </summary>
    public string Label()
    {
        string label = "Signature : ";
        label += thename;
        return label;
    }

    /// <summary>
    /// Tells if the value for &lt;ent&gt; in &lt;model&gt; matches a text, with a criterium &lt;exact&gt;. The
    /// default definition calls MatchValue Can be redefined
    /// </summary>
    public virtual bool Matches(Standard_Transient? ent, Interface_InterfaceModel? model, string text, bool exact)
    {
        return IFSelect_Signature.MatchValue(Value(ent, model), text, exact);
    }

    /// <summary>
    /// Default procedure to tell if a value &lt;val&gt; matches a text with a criterium &lt;exact&gt;.
    /// &lt;exact&gt; = True requires equality, else only contained (no reg-exp)
    /// </summary>
    public static bool MatchValue(string val, string text, bool exact)
    {
        if (exact)
        {
            return text == val;
        }
        // NB: no regexp
        if (text.Length == 0)
        {
            throw new Standard_OutOfRange(); // OCCT: TCollection_AsciiString::Value(1) of an empty text
        }
        char cardeb = text[0];
        int ln, lnt, i, j;
        ln = text.Length;
        lnt = val.Length - ln;
        for (i = 0; i <= lnt; i++)
        {
            if (val[i] == cardeb)
            {
                //    a candidate
                bool res = true;
                for (j = 1; j < ln; j++)
                {
                    if (val[i + j] != text[j])
                    {
                        res = false;
                        break;
                    }
                }
                if (res)
                {
                    return res;
                }
            }
        }
        return false;
    }

    /// <summary>
    /// This procedure converts an Integer to a CString It is a convenient way when the value of a signature has
    /// the form of a simple integer value The value is to be used immediately (one buffer only, no copy)
    /// </summary>
    public static string IntValue(int val)
    {
        switch (val)
        {
            case 0:
                return "0";
            case 1:
                return "1";
            case 2:
                return "2";
            case 3:
                return "3";
            case 4:
                return "4";
            case 5:
                return "5";
            case 6:
                return "6";
            case 7:
                return "7";
            case 8:
                return "8";
            case 9:
                return "9";
            default:
                break;
        }
        return Sprintf("%d", val);
    }

    /// <summary>
    /// Initializes a Signature with its name
    /// </summary>
    protected IFSelect_Signature(string name)
    {
        thename = name;
        thecasi[0] = thecasi[1] = thecasi[2] = 0; // OCCT.NET: a C array of three
    }

    protected string thename = "";
    private readonly int[] thecasi = new int[3];
    private NCollection_HSequence<string>? thecasl;
}
