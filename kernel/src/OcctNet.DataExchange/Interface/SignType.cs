// Port of: src/DataExchange/TKXSBase/Interface/Interface_SignType.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_SignType.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Monitoring;
using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// Provides the basic service to get a type name, according to a norm It can be used for other classes
/// (general signatures ...)
/// </summary>
public abstract class SignType : SignText
{

    /// <summary>
    /// Returns an identification of the Signature (a word), given at initialization time Specialised to consider
    /// context as an InterfaceModel
    /// </summary>
    public override string Text(Transient? ent, Transient? context)
    {
        string atext = "";
        InterfaceModel model = (context as InterfaceModel);
        if (ent is null || model is null)
        {
            return atext;
        }
        atext += Value(ent, model);
        return atext;
    }

    /// <summary>
    /// Returns the Signature for a Transient object. It is specific of each sub-class of Signature. For a Null
    /// Handle, it should provide "" It can work with the model which contains the entity
    /// </summary>
    public abstract string Value(Transient? ent, InterfaceModel? model);

    /// <summary>
    /// From a CDL Type Name, returns the Class part (package dropped) WARNING : buffered, to be immediately
    /// copied or printed
    /// </summary>
    public static string ClassName(string typnam)
    {
        string tn = typnam;
        for (int i = 0; i < tn.Length; i++)
        {
            if (tn[i] == '_')
            {
                return tn.Substring(i + 1);
            }
        }
        return tn;
    }
}
