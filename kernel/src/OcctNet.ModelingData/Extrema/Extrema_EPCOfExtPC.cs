// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GGenExtPC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_EPCOfExtPC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_EPCOfELPCOfLocateExtPC.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// All the extremum distances between a point and a 3D curve on a parameter range: the roots of
/// <see cref="Extrema_PCFOfEPCOfExtPC"/> found by math_FunctionRoots on a sampling of the range.
/// </summary>
/// <remarks>
/// The 3D instantiation of OCCT's template Extrema_GGenExtPC, which OCCT names Extrema_EPCOfExtPC and
/// Extrema_EPCOfELPCOfLocateExtPC (one type). An algorithm object (decision 117): initialize it, then
/// <c>Perform</c>. Keeps a reference to the curve, as OCCT does.
/// </remarks>
public class Extrema_EPCOfExtPC
{
    private bool myDone;
#pragma warning disable CS0414 // OCCT sets myInit and never reads it
    private bool myInit;
#pragma warning restore CS0414
    private int mynbsample;
    private double myumin;
    private double myusup;
    private double mytolu;
    private double mytolF;
    private readonly Extrema_PCFOfEPCOfExtPC myF;

    public Extrema_EPCOfExtPC()
    {
        myDone = false;
        myInit = false;
        mynbsample = 0;
        myumin = 0.0;
        myusup = 0.0;
        mytolu = 0.0;
        mytolF = 0.0;
        myF = new Extrema_PCFOfEPCOfExtPC();
    }

    /// <summary>All the extrema between theP and theC, from theNbSample samples of the curve's whole range.</summary>
    public Extrema_EPCOfExtPC(in gp_Pnt theP, Adaptor3d_Curve theC, int theNbSample, double theTolU, double theTolF)
    {
        myF = new Extrema_PCFOfEPCOfExtPC(theP, theC);
        Initialize(theC, theNbSample, theTolU, theTolF);
        Perform(theP);
    }

    /// <summary>All the extrema between theP and theC on [theUmin, theUsup], from theNbSample samples.</summary>
    public Extrema_EPCOfExtPC(in gp_Pnt theP, Adaptor3d_Curve theC, int theNbSample, double theUmin, double theUsup,
        double theTolU, double theTolF)
    {
        myF = new Extrema_PCFOfEPCOfExtPC(theP, theC);
        Initialize(theC, theNbSample, theUmin, theUsup, theTolU, theTolF);
        Perform(theP);
    }

    /// <summary>Sets the curve, the number of samples and the tolerances, over the curve's whole range.</summary>
    public void Initialize(Adaptor3d_Curve theC, int theNbU, double theTolU, double theTolF)
    {
        myInit = true;
        mynbsample = theNbU;
        mytolu = theTolU;
        mytolF = theTolF;
        myF.Initialize(theC);
        myumin = Extrema_CurveTool.FirstParameter(theC);
        myusup = Extrema_CurveTool.LastParameter(theC);
    }

    /// <summary>Sets the curve, the number of samples, the range and the tolerances.</summary>
    public void Initialize(Adaptor3d_Curve theC, int theNbU, double theUmin, double theUsup, double theTolU,
        double theTolF)
    {
        myInit = true;
        mynbsample = theNbU;
        mytolu = theTolU;
        mytolF = theTolF;
        myF.Initialize(theC);
        myumin = theUmin;
        myusup = theUsup;
    }

    /// <summary>Sets only the number of samples, the range and the tolerances.</summary>
    public void Initialize(int theNbU, double theUmin, double theUsup, double theTolU, double theTolF)
    {
        mynbsample = theNbU;
        mytolu = theTolU;
        mytolF = theTolF;
        myumin = theUmin;
        myusup = theUsup;
    }

    /// <summary>Sets the curve of the function.</summary>
    public void Initialize(Adaptor3d_Curve theC) => myF.Initialize(theC);

    /// <summary>Searches the extrema for the point theP.</summary>
    public void Perform(in gp_Pnt theP)
    {
        myF.SetPoint(theP);
        myF.SubIntervalInitialize(myumin, myusup);
        myDone = false;

        math_FunctionRoots S = new(myF, myumin, myusup, mynbsample, mytolu, mytolF, mytolF);
        if (!S.IsDone() || S.IsAllNull())
        {
            return;
        }

        myDone = true;
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myDone;

    /// <summary>The number of extremum distances.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public int NbExt()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        return myF.NbExt();
    }

    /// <summary>The Nth extremum square distance.</summary>
    /// <exception cref="Standard_OutOfRange">theN is not in 1..NbExt().</exception>
    public double SquareDistance(int theN)
    {
        if ((theN < 1) || (theN > NbExt()))
        {
            throw new Standard_OutOfRange();
        }
        return myF.SquareDistance(theN);
    }

    /// <summary>True if the Nth extremum distance is a minimum.</summary>
    /// <exception cref="Standard_OutOfRange">theN is not in 1..NbExt().</exception>
    public bool IsMin(int theN)
    {
        if ((theN < 1) || (theN > NbExt()))
        {
            throw new Standard_OutOfRange();
        }
        return myF.IsMin(theN);
    }

    /// <summary>The point of the Nth extremum distance.</summary>
    /// <exception cref="Standard_OutOfRange">theN is not in 1..NbExt().</exception>
    public Extrema_POnCurv Point(int theN)
    {
        if ((theN < 1) || (theN > NbExt()))
        {
            throw new Standard_OutOfRange();
        }
        return myF.Point(theN);
    }
}
