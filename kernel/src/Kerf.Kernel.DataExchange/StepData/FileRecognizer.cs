// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FileRecognizer.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FileRecognizer_0.cxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_Recognizer.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// A recognizer: from a starting key (a STEP type name), it attempts to associate a result (an empty entity of the
/// type). Each particular class provides a specific Eval method, which calls SetOK(result) when it recognizes the
/// key. Recognizers can be chained (Add): Evaluate then asks the next one when this one fails.
/// </summary>
public abstract class FileRecognizer : Transient
{
    //  WARNING: TransRecognizer has exactly the same code ...
    //  But produces a Transient instead of a Persistent

    private Transient _theres;
    private bool _hasnext;
    private FileRecognizer _thenext;

    /// <summary>Assumes that no result has yet been recognized.</summary>
    protected FileRecognizer()
    {
        _hasnext = false;
    }

    /// <summary>
    /// Evaluates if recognition has a result, returns it if yes. In case of success, returns True and puts result in
    /// res. In case of failure, simply returns False. Works by calling deferred method Eval, and in case of failure,
    /// looks for Added Recognizers to work.
    /// </summary>
    public bool Evaluate(string akey, ref Transient? res)
    {
        _theres = null;
        Eval(akey);
        if (_theres is not null)
        {
            res = _theres;
            return true;
        }
        else if (_hasnext)
        {
            return _thenext.Evaluate(akey, ref res);
        }
        return false;
    }

    /// <summary>Returns result of last recognition (call of Evaluate).</summary>
    /// <exception cref="NoSuchObjectException">The evaluation has failed.</exception>
    public Transient Result()
    {
        if (_theres is not null)
        {
            return _theres;
        }
        if (_hasnext)
        {
            return _thenext.Result();
        }
        throw new NoSuchObjectException("Recognizer evaluation has failed");
    }

    /// <summary>Adds a new Recognizer to the Compound, at the end. Several calls to Add work by adding in the order of calls: hence, when Eval has failed to recognize, Evaluate will call Evaluate from the first added Recognizer if there is one, and to the second if there is still no result, and so on.</summary>
    public void Add(FileRecognizer reco)
    {
        _thenext = reco;
        _hasnext = true;
    }

    /// <summary>Records the result of the recognition. Called by specific method Eval to record a result: after calling it, Eval has finished and can return.</summary>
    protected void SetOK(Transient aresult)
    {
        _theres = aresult;
    }

    /// <summary>Records that recognition gives no result.</summary>
    protected void SetKO()
    {
        _theres = null;
    }

    /// <summary>This method is called by Evaluate. Once work has been done, it can call SetOK to record a result, or SetKO to record a failure (default).</summary>
    protected abstract void Eval(string akey);
}
