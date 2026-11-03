// Port of: src/DataExchange/TKXSBase/Interface/Interface_Static.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_Static.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Exchange.Framework.ParamType;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// A named parameter of data exchange ("read.precision.mode", "write.step.schema"...): a typed value (its type,
/// constraints if any, and a value, held as a string and readable as an integer, a real or an enumerative string),
/// with a family and a status for reinitialisation.
/// </summary>
/// <remarks>
/// <para>
/// In OCCT, statics are "global" parameters recorded in one process-wide dictionary, and the static methods
/// <c>Interface_Static::Init</c>, <c>IVal</c>, <c>SetCVal</c>... read and write it. The port has no such dictionary
/// (plan 2.7, decision 1181): the statics are recorded in an <see cref="StaticSet"/>, a parameter object
/// passed to each read and write, whose instance methods have OCCT's names, so <c>Interface_Static::IVal("x")</c>
/// becomes <c>theStatics.IVal("x")</c>. This class keeps the part of OCCT's class that is about one parameter.
/// </para>
/// </remarks>
public class Static : InterfaceTypedValue
{
    /// <summary>
    /// Creates a static, with a family and a name. <paramref name="family"/> can report to a name of resource or to a
    /// system or internal definition. <paramref name="type"/> gives the type of the parameter, default is free text.
    /// <paramref name="init"/> gives an initial value; if it is not given, the static begins as "not set".
    /// </summary>
    /// <remarks>OCCT's constructor doesn't record the static: <see cref="StaticSet.Init(string, string, ParamType, string)"/> does.</remarks>
    public Static(string family, string name, ParamType type = ParamType.ParamText, string init = "")
        : base(name, type, init)
    {
        _thefamily = family;
        _theupdate = true;
    }

    /// <summary>
    /// Creates a new static with the same definition as another one (the value is copied, except for an entity: it
    /// remains null).
    /// </summary>
    public Static(string family, string name, Static other) : base(name, other.Type(), "")
    {
        _thefamily = family;
        _theupdate = true;
        switch (Type())
        {
            case ParamType.ParamInteger:
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
            case ParamType.ParamReal:
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
            case ParamType.ParamEnum:
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
            case ParamType.ParamIdent:
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
    /// An exact copy of another static (OCCT.NET, for <see cref="StaticSet"/>'s copies): definition, value,
    /// family and status; the wild-card is set by the set, which maps it to its own copy.
    /// </summary>
    internal Static(Static other, bool exact) : base(other, exact)
    {
        _thefamily = other._thefamily;
        _theupdate = other._theupdate;
    }

    /// <summary>
    /// Writes the properties of a parameter in the diagnostic file: its name, its family, its wild-card (if it has
    /// one), its current status (empty string if it was updated or if it is the original one) and its value.
    /// </summary>
    public void PrintStatic(TextWriter s)
    {
        s.Write("--- Static Value : ");
        s.Write(Name());
        s.Write("  Family:");
        s.Write(Family());
        Print(s);
        if (_thewild is not null)
        {
            s.Write(" -- Attached to wild-card : ");
            s.Write(_thewild.Name());
            s.Write('\n');
        }
        s.Write("--- Actual status : ");
        s.Write(_theupdate ? "" : "original");
        s.Write("  Value : ");

        // OCCT: Interface_Static declares its own thesatisf and thesatisn, which nothing sets (the base class's are the
        // ones SetSatisfies sets), so this never prints
        if (_thesatisf is not null)
        {
            s.Write(" -- Specific Function for Satisfies : ");
            s.Write(_thesatisn);
            s.Write('\n');
        }
    }

    /// <summary>
    /// Returns the family. It can be: a resource name for applis, an internal name between: $e (environment
    /// variables), $l (other, purely local).
    /// </summary>
    public string Family() => _thefamily;

    /// <summary>Returns the wild-card static, which can be (is most often) null.</summary>
    public Static? Wild() => _thewild;

    /// <summary>
    /// Sets a "wild-card" static: its value will be considered if this one is not properly set (reset by setting a
    /// null one).
    /// </summary>
    public void SetWild(Static? wildcard) => _thewild = wildcard;

    /// <summary>
    /// Records a static as "uptodate", i.e. its value has been taken into account by a reinitialisation procedure.
    /// This flag is reset at each successful SetValue.
    /// </summary>
    public void SetUptodate() => _theupdate = true;

    /// <summary>Returns the status "uptodate".</summary>
    public bool UpdatedStatus() => _theupdate;

    private readonly string _thefamily;
    private Static? _thewild;
    private bool _theupdate;

    // OCCT: these shadow the base class's satisfies function and name, and are never set (see PrintStatic)
    private readonly StaticSatisfies? _thesatisf = null;
    private readonly string _thesatisn = "";
}
