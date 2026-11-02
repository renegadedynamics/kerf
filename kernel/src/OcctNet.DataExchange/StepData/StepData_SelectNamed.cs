// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectNamed.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectNamed.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// This select member can be of any kind, and be named. But it takes more memory than some specialised ones. This
/// class allows one name for the instance.
/// </summary>
public class StepData_SelectNamed : StepData_SelectMember
{
    private string thename;
    private readonly StepData_Field theval = new();

    public StepData_SelectNamed()
    {
        theval.Clear();
        thename = "";
    }

    public override bool HasName()
    {
        return (thename.Length > 0);
    }

    public override string Name()
    {
        return thename;
    }

    public override bool SetName(string name)
    {
        thename = name ?? "";
        return true;
    }

    public StepData_Field Field()
    {
        return theval;
    }

    /// <summary>The field, to be changed in place (C++'s non-const reference).</summary>
    public StepData_Field CField()
    {
        return theval;
    }

    public override int Kind()
    {
        return theval.Kind();
    }

    public override void SetKind(int kind)
    {
        theval.Clear(kind);
    }

    public override int Int()
    {
        return theval.Int();
    }

    public override void SetInt(int val)
    {
        theval.SetInt(val);
    }

    public override double Real()
    {
        return theval.Real();
    }

    public override void SetReal(double val)
    {
        theval.SetReal(val);
    }

    public override string String()
    {
        return theval.String();
    }

    public override void SetString(string val)
    {
        theval.SetString(val);
    }
}
