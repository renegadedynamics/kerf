// Port of: src/DataExchange/TKXSBase/Interface/Interface_Static.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_Static.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Interface_ParamType;

namespace Occt;

/// <summary>
/// A named parameter of data exchange ("read.precision.mode", "write.step.schema"...): a typed value (its type,
/// constraints if any, and a value, held as a string and readable as an integer, a real or an enumerative string),
/// with a family and a status for reinitialisation.
/// </summary>
/// <remarks>
/// <para>
/// In OCCT, statics are "global" parameters recorded in one process-wide dictionary, and the static methods
/// <c>Interface_Static::Init</c>, <c>IVal</c>, <c>SetCVal</c>... read and write it. The port has no such dictionary
/// (plan 2.7, decision 12101): the statics are recorded in an <see cref="Interface_StaticSet"/>, a parameter object
/// passed to each read and write, whose instance methods have OCCT's names, so <c>Interface_Static::IVal("x")</c>
/// becomes <c>theStatics.IVal("x")</c>. This class keeps the part of OCCT's class that is about one parameter.
/// </para>
/// </remarks>
public class Interface_Static : Interface_TypedValue
{
    /// <summary>
    /// Creates a static, with a family and a name. <paramref name="family"/> can report to a name of resource or to a
    /// system or internal definition. <paramref name="type"/> gives the type of the parameter, default is free text.
    /// <paramref name="init"/> gives an initial value; if it is not given, the static begins as "not set".
    /// </summary>
    /// <remarks>OCCT's constructor doesn't record the static: <see cref="Interface_StaticSet.Init(string, string, Interface_ParamType, string)"/> does.</remarks>
    public Interface_Static(string family, string name, Interface_ParamType type = Interface_ParamText, string init = "")
        : base(name, type, init)
    {
        thefamily = family;
        theupdate = true;
    }

    /// <summary>
    /// Creates a new static with the same definition as another one (the value is copied, except for an entity: it
    /// remains null).
    /// </summary>
    public Interface_Static(string family, string name, Interface_Static other) : base(name, other.Type(), "")
    {
        thefamily = family;
        theupdate = true;
        switch (Type())
        {
            case Interface_ParamInteger:
            {
                int lim;
                if (other.IntegerLimit(true, out lim))
                {
                    SetIntegerLimit(true, lim);
                }
                if (other.IntegerLimit(false, out lim))
                {
                    SetIntegerLimit(false, lim);
                }
            }
            break;
            case Interface_ParamReal:
            {
                double lim;
                if (other.RealLimit(true, out lim))
                {
                    SetRealLimit(true, lim);
                }
                if (other.RealLimit(false, out lim))
                {
                    SetRealLimit(false, lim);
                }
                SetUnitDef(other.UnitDef());
            }
            break;
            case Interface_ParamEnum:
            {
                bool match;
                int e0, e1, i;
                other.EnumDef(out e0, out e1, out match);
                StartEnum(e0, match);
                //      if (e1 >= e0) theenums = new NCollection_HArray1<TCollection_AsciiString>(e0,e1);
                for (i = e0; i <= e1; i++)
                {
                    AddEnum(other.EnumVal(i));
                }
            }
            break;
            case Interface_ParamIdent:
                SetObjectType(other.ObjectType());
                break;
            default:
                break;
        }

        if (other.IsSetValue())
        {
            SetCStringValue(other.CStringValue());
        }
    }

    /// <summary>
    /// An exact copy of another static (OCCT.NET, for <see cref="Interface_StaticSet"/>'s copies): definition, value,
    /// family and status; the wild-card is set by the set, which maps it to its own copy.
    /// </summary>
    internal Interface_Static(Interface_Static other, bool theExact) : base(other, theExact)
    {
        thefamily = other.thefamily;
        theupdate = other.theupdate;
    }

    /// <summary>
    /// Writes the properties of a parameter in the diagnostic file: its name, its family, its wild-card (if it has
    /// one), its current status (empty string if it was updated or if it is the original one) and its value.
    /// </summary>
    public void PrintStatic(TextWriter S)
    {
        S.Write("--- Static Value : ");
        S.Write(Name());
        S.Write("  Family:");
        S.Write(Family());
        Print(S);
        if (thewild is not null)
        {
            S.Write(" -- Attached to wild-card : ");
            S.Write(thewild.Name());
            S.Write('\n');
        }
        S.Write("--- Actual status : ");
        S.Write(theupdate ? "" : "original");
        S.Write("  Value : ");

        // OCCT: Interface_Static declares its own thesatisf and thesatisn, which nothing sets (the base class's are the
        // ones SetSatisfies sets), so this never prints
        if (thesatisf is not null)
        {
            S.Write(" -- Specific Function for Satisfies : ");
            S.Write(thesatisn);
            S.Write('\n');
        }
    }

    /// <summary>
    /// Returns the family. It can be: a resource name for applis, an internal name between: $e (environment
    /// variables), $l (other, purely local).
    /// </summary>
    public string Family() => thefamily;

    /// <summary>Returns the wild-card static, which can be (is most often) null.</summary>
    public Interface_Static? Wild() => thewild;

    /// <summary>
    /// Sets a "wild-card" static: its value will be considered if this one is not properly set (reset by setting a
    /// null one).
    /// </summary>
    public void SetWild(Interface_Static? wildcard) => thewild = wildcard;

    /// <summary>
    /// Records a static as "uptodate", i.e. its value has been taken into account by a reinitialisation procedure.
    /// This flag is reset at each successful SetValue.
    /// </summary>
    public void SetUptodate() => theupdate = true;

    /// <summary>Returns the status "uptodate".</summary>
    public bool UpdatedStatus() => theupdate;

    private readonly string thefamily;
    private Interface_Static? thewild;
    private bool theupdate;

    // OCCT: these shadow the base class's satisfies function and name, and are never set (see PrintStatic)
    private readonly Interface_StaticSatisfies? thesatisf = null;
    private readonly string thesatisn = "";
}
