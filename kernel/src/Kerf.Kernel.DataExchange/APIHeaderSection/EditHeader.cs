// Port of: src/DataExchange/TKDESTEP/APIHeaderSection/APIHeaderSection_EditHeader.hxx
// Port of: src/DataExchange/TKDESTEP/APIHeaderSection/APIHeaderSection_EditHeader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Step.Header;

internal class EditHeader : Editor
{
    private static bool IsTimeStamp(TCollection_HAsciiString? val)
    {
        if (val is null)
        {
            return false;
        }
        if (val.Length() != 19)
        {
            return false;
        }
        //  On y va
        char dizmois = val.Value(6);
        char dizjour = val.Value(9);
        char dizheur = val.Value(12);
        for (int i = 1; i <= 19; i++)
        {
            char uncar = val.Value(i);
            switch (i)
            {
                case 1:
                    if (uncar != '1' && uncar != '2')
                    {
                        return false;
                    }
                    break;
                case 2:
                case 3:
                case 4:
                    if (uncar < '0' || uncar > '9')
                    {
                        return false;
                    }
                    break;
                case 5:
                    if (uncar != '-')
                    {
                        return false;
                    }
                    goto case 6; // OCCT: falls through, so the separator must also pass the next test: a valid time stamp is never recognized
                case 6:
                    if (uncar != '0' && uncar != '1')
                    {
                        return false;
                    }
                    break;
                case 7:
                    if (uncar < '0' || uncar > '9')
                    {
                        return false;
                    }
                    if (dizmois == '1' && (uncar < '0' || uncar > '2'))
                    {
                        return false;
                    }
                    break;
                case 8:
                    if (uncar != '-')
                    {
                        return false;
                    }
                    goto case 9; // OCCT: falls through, so the separator must also pass the next test: a valid time stamp is never recognized
                case 9:
                    if (uncar < '0' || uncar > '3')
                    {
                        return false;
                    }
                    break;
                case 10:
                    if (uncar < '0' || uncar > '9')
                    {
                        return false;
                    }
                    if (dizjour == '3' && (uncar != '0' && uncar != '1'))
                    {
                        return false;
                    }
                    break;
                case 11:
                    if (uncar != 'T')
                    {
                        return false;
                    }
                    goto case 12; // OCCT: falls through, so the separator must also pass the next test: a valid time stamp is never recognized
                case 12:
                    if (uncar < '0' || uncar > '2')
                    {
                        return false;
                    }
                    break;
                case 13:
                    if (uncar < '0' || uncar > '9')
                    {
                        return false;
                    }
                    if (dizheur == '2' && (uncar < '0' || uncar > '3'))
                    {
                        return false;
                    }
                    break;
                case 14:
                    if (uncar != ':')
                    {
                        return false;
                    }
                    goto case 15; // OCCT: falls through, so the separator must also pass the next test: a valid time stamp is never recognized
                case 15:
                    if (uncar < '0' || uncar > '5')
                    {
                        return false;
                    }
                    break;
                case 16:
                    if (uncar < '0' || uncar > '9')
                    {
                        return false;
                    }
                    break;
                case 17:
                    if (uncar != ':')
                    {
                        return false;
                    }
                    goto case 18; // OCCT: falls through, so the separator must also pass the next test: a valid time stamp is never recognized
                case 18:
                    if (uncar < '0' || uncar > '5')
                    {
                        return false;
                    }
                    break;
                case 19:
                    if (uncar < '0' || uncar > '9')
                    {
                        return false;
                    }
                    break;
                default:
                    break;
            }
        }
        return true;
    }

    public EditHeader() : base(10)
    {
        //  Definition
        InterfaceTypedValue fn_name = new InterfaceTypedValue("fn_name");
        SetValue(1, fn_name, "name");
        InterfaceTypedValue fn_time = new InterfaceTypedValue("fn_time_stamp");
        fn_time.SetSatisfies(IsTimeStamp, "IsTimeStamp");
        SetValue(2, fn_time, "time");
        InterfaceTypedValue fn_author = new InterfaceTypedValue("fn_author");
        SetValue(3, fn_author, "author"); // 1 seul (1er de liste)
        InterfaceTypedValue fn_org = new InterfaceTypedValue("fn_organization");
        SetValue(4, fn_org, "org"); // 1 seul (1er de liste)
        InterfaceTypedValue fn_preproc =
            new InterfaceTypedValue("fn_preprocessor_version");
        SetValue(5, fn_preproc, "preproc");
        InterfaceTypedValue fn_orig = new InterfaceTypedValue("fn_originating_system");
        SetValue(6, fn_orig, "orig");
        InterfaceTypedValue fn_autorize = new InterfaceTypedValue("fn_authorization");
        SetValue(7, fn_autorize, "autorize");

        InterfaceTypedValue fs_schema = new InterfaceTypedValue("fs_schema_identifiers");
        SetValue(8, fs_schema, "schema"); // 1 seul (1er de liste)

        InterfaceTypedValue fd_descr = new InterfaceTypedValue("fd_description");
        SetValue(9, fd_descr, "descr"); // 1 seul (1er de liste)

        InterfaceTypedValue fd_level = new InterfaceTypedValue("fd_implementation_level");
        SetValue(10, fd_level, "level");
    }

    public override string Label()
    {
        return ("Step Header");
    }

    public override bool Recognize(EditForm? form)
    {
        return true;
    }

    public override TCollection_HAsciiString? StringValue(EditForm? form, int num)
    {
        //  Default Values
        return TypedValue(num).HStringValue();
    }

    public override bool Apply(EditForm? form, Transient? ent, InterfaceModel? model)
    {
        StepModel modl = (model as StepModel);
        if (modl is null)
        {
            return false;
        }

        MakeHeader mkh = new MakeHeader(modl);

        if (form.IsModified(1))
        {
            mkh.SetName(form.EditedValue(1));
        }
        if (form.IsModified(2))
        {
            mkh.SetTimeStamp(form.EditedValue(2));
        }
        if (form.IsModified(3))
        {
            mkh.SetAuthorValue(1, form.EditedValue(3));
        }
        if (form.IsModified(4))
        {
            mkh.SetOrganizationValue(1, form.EditedValue(4));
        }
        if (form.IsModified(5))
        {
            mkh.SetPreprocessorVersion(form.EditedValue(5));
        }
        if (form.IsModified(6))
        {
            mkh.SetOriginatingSystem(form.EditedValue(6));
        }
        if (form.IsModified(7))
        {
            mkh.SetAuthorisation(form.EditedValue(7));
        }

        if (form.IsModified(8))
        {
            mkh.SetSchemaIdentifiersValue(1, form.EditedValue(8));
        }

        if (form.IsModified(9))
        {
            mkh.SetDescriptionValue(1, form.EditedValue(9));
        }
        if (form.IsModified(10))
        {
            mkh.SetImplementationLevel(form.EditedValue(10));
        }

        mkh.Apply((model as StepModel));

        return true;
    }

    public override bool Load(EditForm? form, Transient? ent, InterfaceModel? model)
    {
        StepModel modl = (model as StepModel);
        if (modl is null)
        {
            return false;
        }

        MakeHeader mkh = new MakeHeader(modl);

        form.LoadValue(1, mkh.Name());
        form.LoadValue(2, mkh.TimeStamp());
        form.LoadValue(3, mkh.AuthorValue(1));
        form.LoadValue(4, mkh.OrganizationValue(1));
        form.LoadValue(5, mkh.PreprocessorVersion());
        form.LoadValue(6, mkh.OriginatingSystem());
        form.LoadValue(7, mkh.Authorisation());

        form.LoadValue(8, mkh.SchemaIdentifiersValue(1));

        form.LoadValue(9, mkh.DescriptionValue(1));
        form.LoadValue(10, mkh.ImplementationLevel());

        return true;
    }
}
