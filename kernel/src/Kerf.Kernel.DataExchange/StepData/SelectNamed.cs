// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectNamed.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectNamed.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Occt;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// This select member can be of any kind, and be named. But it takes more memory than some specialised ones. This
/// class allows one name for the instance.
/// </summary>
public class SelectNamed : SelectMember
{
    private string _thename;
    private readonly Field _theval = new();

    public SelectNamed()
    {
        _theval.Clear();
        _thename = "";
    }

    public override bool HasName
    {
        get
        {
            return (_thename.Length > 0);
        }
    }

    public override string Name()
    {
        return _thename;
    }

    public override bool SetName(string name)
    {
        _thename = name ?? "";
        return true;
    }

    public Field Field()
    {
        return _theval;
    }

    /// <summary>The field, to be changed in place (C++'s non-const reference).</summary>
    public Field CField()
    {
        return _theval;
    }

    public override int Kind()
    {
        return _theval.Kind();
    }

    public override void SetKind(int kind)
    {
        _theval.Clear(kind);
    }

    public override int Int()
    {
        return _theval.Int();
    }

    public override void SetInt(int val)
    {
        _theval.SetInt(val);
    }

    public override double Real()
    {
        return _theval.Real();
    }

    public override void SetReal(double val)
    {
        _theval.SetReal(val);
    }

    public override string String()
    {
        return _theval.String();
    }

    public override void SetString(string val)
    {
        _theval.SetString(val);
    }
}
