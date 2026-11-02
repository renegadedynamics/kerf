// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_StepType.hxx
// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_StepType.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using System.Text;

namespace Occt;

/// <summary>
/// StepType is a Signature specific to Step definitions : it considers the type as defined in STEP Schemas, the same
/// which is used in files. For a Complex Type, if its definition is known, StepType produces the list of basic
/// types, separated by commas, the whole between brackets : "(TYPE1,TYPE2..)". If its precise definition is not
/// known (simply it is known as Complex, it can be recognised, but the list is produced at Write time only),
/// StepType produces : "(..COMPLEX TYPE..)"
/// </summary>
public class StepSelect_StepType : IFSelect_Signature
{
    /// <summary>
    /// Creates a Signature for Step Type. Protocol is undefined here, hence no Signature may yet be produced. The
    /// StepType signature requires a Protocol before working
    /// </summary>
    public StepSelect_StepType() : base("Step Type")
    {
    }

    /// <summary>Sets the StepType signature to work with a Protocol : this initialises the library</summary>
    public void SetProtocol(Interface_Protocol proto)
    {
        StepData_Protocol newproto = proto as StepData_Protocol;
        if (newproto is null)
        {
            throw new Interface_InterfaceError("StepSelect_StepType");
        }
        theproto = newproto;
        thelib.Clear();
        thelib.AddProtocol(theproto);
        thename = "Step Type (Schema " + theproto.SchemaName(null) + ")";
    }

    /// <summary>
    /// Returns the Step Type defined from the Protocol (see above). If &lt;ent&gt; is not recognised, produces
    /// "..NOT FROM SCHEMA &lt;name&gt;.."
    /// </summary>
    public override string Value(Standard_Transient ent, Interface_InterfaceModel model)
    {
        lock (myMutex)
        {
            if (!thelib.Select(ent, out StepData_ReadWriteModule aModule, out int aCN))
            {
                // Build error message for unrecognized entity
                return "..NOT FROM SCHEMA " + theproto.SchemaName(model) + "..";
            }

            // Handle simple (non-complex) type - return direct reference from module
            if (!aModule.IsComplex(aCN))
            {
                return aModule.StepType(aCN);
            }

            // Handle complex type from module
            NCollection_Sequence<string> aList = new NCollection_Sequence<string>();
            if (aModule.ComplexType(aCN, aList))
            {
                int aNb = aList.Length();
                if (aNb == 0)
                {
                    return "(..COMPLEX TYPE..)";
                }
                StringBuilder aValue = new StringBuilder("(");
                for (int i = 1; i <= aNb; i++)
                {
                    if (i > 1)
                    {
                        aValue.Append(',');
                    }
                    aValue.Append(aList.Value(i));
                }
                aValue.Append(')');
                return aValue.ToString();
            }

            // Fallback: check for undefined entity
            StepData_UndefinedEntity anUnd = ent as StepData_UndefinedEntity;
            if (anUnd is null)
            {
                return "";
            }

            if (!anUnd.IsComplex())
            {
                return anUnd.StepType(); // Direct return from entity's internal storage
            }

            // Build complex type from undefined entity
            StringBuilder aLastValue = new StringBuilder("(");
            bool isFirst = true;
            while (anUnd is not null)
            {
                if (!isFirst)
                {
                    aLastValue.Append(',');
                }
                aLastValue.Append(anUnd.StepType());
                anUnd = anUnd.Next();
                isFirst = false;
            }
            aLastValue.Append(')');
            return aLastValue.ToString();
        }
    }

    protected StepData_WriterLib thelib = new StepData_WriterLib();
    private StepData_Protocol theproto;
    private readonly object myMutex = new object();
}
