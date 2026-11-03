// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_StepType.hxx
// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_StepType.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Foundation;
using Occt;
using System.Text;

namespace Kerf.Engine.Exchange.Step.Selections;

/// <summary>
/// StepType is a Signature specific to Step definitions : it considers the type as defined in STEP Schemas, the same
/// which is used in files. For a Complex Type, if its definition is known, StepType produces the list of basic
/// types, separated by commas, the whole between brackets : "(TYPE1,TYPE2..)". If its precise definition is not
/// known (simply it is known as Complex, it can be recognised, but the list is produced at Write time only),
/// StepType produces : "(..COMPLEX TYPE..)"
/// </summary>
public class StepType : Signature
{
    /// <summary>
    /// Creates a Signature for Step Type. Protocol is undefined here, hence no Signature may yet be produced. The
    /// StepType signature requires a Protocol before working
    /// </summary>
    public StepType() : base("Step Type")
    {
    }

    /// <summary>Sets the StepType signature to work with a Protocol : this initialises the library</summary>
    public void SetProtocol(Protocol proto)
    {
        StepProtocol newproto = proto as StepProtocol;
        if (newproto is null)
        {
            throw new InterfaceErrorException("StepSelect_StepType");
        }
        _theproto = newproto;
        _thelib.Clear();
        _thelib.AddProtocol(_theproto);
        _thename = "Step Type (Schema " + _theproto.SchemaName(null) + ")";
    }

    /// <summary>
    /// Returns the Step Type defined from the Protocol (see above). If &lt;ent&gt; is not recognised, produces
    /// "..NOT FROM SCHEMA &lt;name&gt;.."
    /// </summary>
    public override string Value(Transient ent, InterfaceModel model)
    {
        lock (_mutex)
        {
            if (!_thelib.Select(ent, out ReadWriteModule module, out int cn))
            {
                // Build error message for unrecognized entity
                return "..NOT FROM SCHEMA " + _theproto.SchemaName(model) + "..";
            }

            // Handle simple (non-complex) type - return direct reference from module
            if (!module.IsComplex(cn))
            {
                return module.StepType(cn);
            }

            // Handle complex type from module
            NCollection_Sequence<string> list = new NCollection_Sequence<string>();
            if (module.ComplexType(cn, list))
            {
                int nb = list.Length();
                if (nb == 0)
                {
                    return "(..COMPLEX TYPE..)";
                }
                StringBuilder value = new StringBuilder("(");
                for (int i = 1; i <= nb; i++)
                {
                    if (i > 1)
                    {
                        value.Append(',');
                    }
                    value.Append(list.Value(i));
                }
                value.Append(')');
                return value.ToString();
            }

            // Fallback: check for undefined entity
            UndefinedEntity und = ent as UndefinedEntity;
            if (und is null)
            {
                return "";
            }

            if (!und.IsComplex)
            {
                return und.StepType(); // Direct return from entity's internal storage
            }

            // Build complex type from undefined entity
            StringBuilder lastValue = new StringBuilder("(");
            bool isFirst = true;
            while (und is not null)
            {
                if (!isFirst)
                {
                    lastValue.Append(',');
                }
                lastValue.Append(und.StepType());
                und = und.Next();
                isFirst = false;
            }
            lastValue.Append(')');
            return lastValue.ToString();
        }
    }

    protected WriterLib _thelib = new WriterLib();
    private StepProtocol _theproto;
    private readonly object _mutex = new object();
}
