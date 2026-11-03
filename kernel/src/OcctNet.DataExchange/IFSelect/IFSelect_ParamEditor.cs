// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ParamEditor.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ParamEditor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using static Occt.IFSelect_EditValue;

namespace Occt;

/// <summary>
/// A ParamEditor gives access for edition to a list of TypedValue (i.e. of Static too) Its definition is
/// made of the TypedValue to edit themselves, and can add some constants, which can then be displayed but
/// not changed (for instance, system name, processor version ...) I.E. it gives a way of editing or at least
/// displaying parameters as global
/// </summary>
public class IFSelect_ParamEditor : IFSelect_Editor
{

    /// <summary>
    /// Creates a ParamEditor, empty, with a maximum count of params (default is 100) And a label, by default it
    /// will be "Param Editor"
    /// </summary>
    public IFSelect_ParamEditor(int nbmax = 100, string label = "") : base(nbmax)
    {
        thelabel = label;
        SetNbValues(0);
        if (thelabel.Length == 0)
        {
            thelabel += "Param Editor";
        }
    }

    /// <summary>
    /// Adds a TypedValue By default, its short name equates its complete name, it can be made explicit
    /// </summary>
    public void AddValue(Interface_TypedValue? val, string shortname = "")
    {
        SetNbValues(NbValues() + 1);
        SetValue(NbValues(), val, shortname);
    }

    /// <summary>
    /// Adds a Constant Text, it will be Read Only By default, its long name equates its shortname
    /// </summary>
    public void AddConstantText(string val, string shortname, string longname = "")
    {
        Interface_TypedValue tv =
            new Interface_TypedValue(longname.Length == 0 ? shortname : longname);
        tv.SetCStringValue(val);
        SetNbValues(NbValues() + 1);
        SetValue(NbValues(), tv, shortname, IFSelect_EditRead);
    }

    public override string Label()
    {
        return thelabel;
    }

    public override bool Recognize(IFSelect_EditForm? form)
    {
        return true;
    }

    public override TCollection_HAsciiString? StringValue(IFSelect_EditForm? form, int num)
    {
        return TypedValue(num).HStringValue();
    }

    public override bool Load(IFSelect_EditForm? form, Transient? ent, Interface_InterfaceModel? model)
    {
        int i, nb = NbValues();
        for (i = 1; i <= nb; i++)
        {
            form.LoadValue(i, TypedValue(i).HStringValue());
        }

        return true;
    }

    public override bool Apply(IFSelect_EditForm? form, Transient? ent, Interface_InterfaceModel? model)
    {
        int i, nb = NbValues();
        for (i = 1; i <= nb; i++)
        {
            if (form.IsModified(i))
            {
                TypedValue(i).SetHStringValue(form.EditedValue(i));
            }
        }

        return true;
    }

    /// <summary>
    /// Returns a ParamEditor to work on the Static Parameters of which names are listed in &lt;list&gt; Null
    /// Handle if &lt;list&gt; is null or empty
    /// </summary>
    /// <remarks>
    /// OCCT.NET: the statics are named in the parameter set <paramref name="statics"/> (OCCT: the process-wide
    /// statics; decision 1181). The editor edits that set's statics themselves.
    /// </remarks>
    public static IFSelect_ParamEditor? StaticEditor(Interface_StaticSet statics, NCollection_HSequence<TCollection_HAsciiString>? list, string label = "")
    {
        IFSelect_ParamEditor editor = null;
        if (list is null)
        {
            return editor;
        }
        int i, nb = list.Length();
        //  if (nb == 0) return editor;
        editor = new IFSelect_ParamEditor(nb + 10, label);
        for (i = 1; i <= nb; i++)
        {
            Interface_Static val = statics.Static(list.Value(i).ToCString());
            if (val is not null)
            {
                editor.AddValue(val);
            }
        }
        return editor;
    }

    private string thelabel = "";
}
