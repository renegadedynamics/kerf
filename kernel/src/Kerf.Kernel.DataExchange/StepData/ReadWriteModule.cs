// Port of: src/DataExchange/TKDESTEP/StepData/StepData_ReadWriteModule.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_ReadWriteModule.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// Defines basic File Access Module (Recognize, Read, Write). That is: ReaderModule (Recognize and Read) but also
/// WriterModule for StepWriter, through the Protocol. Each module has a protocol. StepData uses it in its
/// "Reader" and "Writer" tools.
/// </summary>
public abstract class ReadWriteModule : ReaderModule
{
    /// <summary>
    /// Translate the Type of record num in data to a positive Case Number, or 0 if failed. Works with a
    /// StepReaderData, in which the Type of an Entity is defined as a String: Reads the RecordType num then calls
    /// CaseNum (this type). Warning: The methods CaseStep, StepType and Recognize must be in phase (triplets
    /// CaseNum-StepType-Type of Object).
    /// </summary>
    public override int CaseNum(FileReaderData? data, int num)
    {
        if (data is not StepReaderData stepdat)
        {
            return 0;
        }
        if (stepdat.IsComplex(num))
        {
            NCollection_Sequence<string> types = new NCollection_Sequence<string>();
            stepdat.ComplexType(num, types);
            if (types.IsEmpty())
            {
                return 0;
            }
            if (types.Length() == 1)
            {
                return CaseStep(types.Value(1));
            }
            else
            {
                return CaseStep(types);
            }
        }
        return CaseStep(stepdat.RecordType(num));
    }

    /// <summary>Defines Case Numbers corresponding to the recognized Types. Called by CaseNum (data, num) above for a Simple Type Entity. Warning: CaseStep must give the same Value as Protocol does for the Entity type which corresponds to this Type given as a String.</summary>
    public abstract int CaseStep(string atype);

    /// <summary>Same a above but for a Complex Type Entity ("Plex"). The provided Default recognizes nothing.</summary>
    public virtual int CaseStep(NCollection_Sequence<string> types)
    {
        return 0;
    } // default

    /// <summary>Returns True if the Case Number corresponds to a Complex Type ("Plex"). Remember that all possible combinations must be acknowledged to be processed. Default is False for all cases. For a Protocol which defines possible Plexes, this method must be redefined.</summary>
    public virtual bool IsComplex(int cn)
    {
        return false;
    } // default

    /// <summary>Function specific to STEP, which delivers the StepType as it is recorded in and read from a File compliant with STEP. This method is symmetric to the method CaseStep. StepType can be different from Dynamic Type's name, but belongs to the same class of Object. Returns an empty String if CN is zero. Warning: For a Complex Type Entity, returns an Empty String (Complex Type must be managed by users).</summary>
    public abstract string StepType(int cn);

    /// <summary>Function specific to STEP. Some STEP Types have a short form. This method can be redefined to fill it. By default, returns an empty string, which is then interpreted to take normal form from StepType.</summary>
    public virtual string ShortType(int cn)
    {
        return "";
    } // default empty

    /// <summary>Function specific to STEP, which delivers the list of types which corresponds to a complex type. If CN is not for a complex type, this method returns False. Else it returns True and fills the list in alphabetic order. The default returns False. To be redefined as required.</summary>
    public virtual bool ComplexType(int cn, NCollection_Sequence<string> types)
    {
        return false;
    }

    /// <summary>General Read Function, calls ReadStep.</summary>
    public override void Read(int cn, FileReaderData? data, int num, ref FrameworkCheck? ach, Transient? ent)
    {
        if (data is not StepReaderData stepdat)
        {
            return;
        }
        ReadStep(cn, stepdat, num, ach, ent);
    }

    /// <summary>Specific Read Function. Works with StepReaderData.</summary>
    public abstract void ReadStep(int cn, StepReaderData data, int num, FrameworkCheck ach, Transient? ent);

    /// <summary>Write Function, switched by CaseNum.</summary>
    public abstract void WriteStep(int cn, StepWriter sw, Transient? ent);
}
