// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_SurfData.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_SurfData.lxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_SurfData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The data of one fillet surface and its two faces: the indices of the faces, the fillet surface and the curves in
/// the data structure, the interferences with the faces, the four corner points and the spine parameters.
/// </summary>
/// <remarks>
/// A working object of the fillet builders (decision 21): they fill it while they build the stripes, through the
/// internal mutators and the <c>Change*</c> accessors that return the stored values by reference. The public
/// members only read.
/// </remarks>
public sealed class ChFiDS_SurfData : Standard_Transient
{
    private ChFiDS_CommonPoint pfirstOnS1 = new();
    private ChFiDS_CommonPoint plastOnS1 = new();
    private ChFiDS_CommonPoint pfirstOnS2 = new();
    private ChFiDS_CommonPoint plastOnS2 = new();
    private ChFiDS_FaceInterference intf1 = new();
    private ChFiDS_FaceInterference intf2 = new();
    private gp_Pnt2d p2df1 = new();
    private gp_Pnt2d p2dl1 = new();
    private gp_Pnt2d p2df2 = new();
    private gp_Pnt2d p2dl2 = new();
    private double ufspine;
    private double ulspine;
    private double myfirstextend;
    private double mylastextend;
    private Standard_Transient? simul;
    private int indexOfS1;
    private int indexOfC1;
    private int indexOfS2;
    private int indexOfC2;
    private int indexOfConge;
    private bool isoncurv1;
    private bool isoncurv2;
    private bool twistons1;
    private bool twistons2;
    private TopAbs_Orientation orientation;

    public ChFiDS_SurfData()
    {
        ufspine = 0.0;
        ulspine = 0.0;
        myfirstextend = 0.0;
        mylastextend = 0.0;
        indexOfS1 = 0;
        indexOfC1 = 0;
        indexOfS2 = 0;
        indexOfC2 = 0;
        indexOfConge = 0;
        isoncurv1 = false;
        isoncurv2 = false;
        twistons1 = false;
        twistons2 = false;
        orientation = TopAbs_FORWARD;
    }

    /// <summary>
    /// Copies <paramref name="Other"/>'s data into this one, except (as OCCT) the indices of the curves and the
    /// flags that the interferences are on curves.
    /// </summary>
    internal void Copy(ChFiDS_SurfData Other)
    {
        indexOfS1 = Other.indexOfS1;
        indexOfS2 = Other.indexOfS2;
        indexOfConge = Other.indexOfConge;
        orientation = Other.orientation;
        intf1 = Other.intf1;
        intf2 = Other.intf2;
        pfirstOnS1 = Other.pfirstOnS1;
        plastOnS1 = Other.plastOnS1;
        pfirstOnS2 = Other.pfirstOnS2;
        plastOnS2 = Other.plastOnS2;

        ufspine = Other.ufspine;
        ulspine = Other.ulspine;

        simul = Other.simul;
        p2df1 = Other.p2df1;
        p2dl1 = Other.p2dl1;
        p2df2 = Other.p2df2;
        p2dl2 = Other.p2dl2;

        myfirstextend = Other.myfirstextend;
        mylastextend = Other.mylastextend;

        twistons1 = Other.twistons1;
        twistons2 = Other.twistons2;
    }

    public int IndexOfS1()
    {
        return indexOfS1;
    }

    public int IndexOfS2()
    {
        return indexOfS2;
    }

    public bool IsOnCurve1()
    {
        return isoncurv1;
    }

    public bool IsOnCurve2()
    {
        return isoncurv2;
    }

    /// <summary>The index of the curve on the first face; throws <see cref="Standard_Failure"/> if not on a curve.</summary>
    public int IndexOfC1()
    {
        if (!isoncurv1)
        {
            throw new Standard_Failure("Interference pas sur courbe");
        }

        return indexOfC1;
    }

    /// <summary>The index of the curve on the second face; throws <see cref="Standard_Failure"/> if not on a curve.</summary>
    public int IndexOfC2()
    {
        if (!isoncurv2)
        {
            throw new Standard_Failure("Interference pas sur courbe");
        }

        return indexOfC2;
    }

    /// <summary>The index of the fillet surface in the data structure.</summary>
    public int Surf()
    {
        return indexOfConge;
    }

    public TopAbs_Orientation Orientation()
    {
        return orientation;
    }

    public ChFiDS_FaceInterference InterferenceOnS1()
    {
        return intf1;
    }

    public ChFiDS_FaceInterference InterferenceOnS2()
    {
        return intf2;
    }

    public ChFiDS_CommonPoint VertexFirstOnS1()
    {
        return pfirstOnS1;
    }

    public ChFiDS_CommonPoint VertexFirstOnS2()
    {
        return pfirstOnS2;
    }

    public ChFiDS_CommonPoint VertexLastOnS1()
    {
        return plastOnS1;
    }

    public ChFiDS_CommonPoint VertexLastOnS2()
    {
        return plastOnS2;
    }

    internal void ChangeIndexOfS1(int Index)
    {
        indexOfS1 = Index;
    }

    internal void ChangeIndexOfS2(int Index)
    {
        indexOfS2 = Index;
    }

    internal void ChangeSurf(int Index)
    {
        indexOfConge = Index;
    }

    internal void SetIndexOfC1(int theIndex)
    {
        indexOfC1 = theIndex;
        isoncurv1 = (theIndex != 0);
    }

    internal void SetIndexOfC2(int theIndex)
    {
        indexOfC2 = theIndex;
        isoncurv2 = (theIndex != 0);
    }

    internal ref TopAbs_Orientation ChangeOrientation() => ref orientation;

    internal ref ChFiDS_FaceInterference ChangeInterferenceOnS1() => ref intf1;

    internal ref ChFiDS_FaceInterference ChangeInterferenceOnS2() => ref intf2;

    internal ref ChFiDS_CommonPoint ChangeVertexFirstOnS1() => ref pfirstOnS1;

    internal ref ChFiDS_CommonPoint ChangeVertexFirstOnS2() => ref pfirstOnS2;

    internal ref ChFiDS_CommonPoint ChangeVertexLastOnS1() => ref plastOnS1;

    internal ref ChFiDS_CommonPoint ChangeVertexLastOnS2() => ref plastOnS2;

    /// <summary>The interference on face <paramref name="OnS"/> (1 or 2).</summary>
    public ChFiDS_FaceInterference Interference(int OnS)
    {
        if (OnS == 1)
        {
            return intf1;
        }
        else
        {
            return intf2;
        }
    }

    internal ref ChFiDS_FaceInterference ChangeInterference(int OnS)
    {
        if (OnS == 1)
        {
            return ref intf1;
        }
        else
        {
            return ref intf2;
        }
    }

    /// <summary>The index of face <paramref name="OfS"/> (1 or 2).</summary>
    public int Index(int OfS)
    {
        if (OfS == 1)
        {
            return indexOfS1;
        }
        else
        {
            return indexOfS2;
        }
    }

    /// <summary>The first or last point on face <paramref name="OnS"/>.</summary>
    public ChFiDS_CommonPoint Vertex(bool First, int OnS)
    {
        if (First && OnS == 1)
        {
            return pfirstOnS1;
        }
        else if (First && OnS == 2)
        {
            return pfirstOnS2;
        }
        else if (!First && OnS == 1)
        {
            return plastOnS1;
        }
        else
        {
            return plastOnS2;
        }
    }

    internal ref ChFiDS_CommonPoint ChangeVertex(bool First, int OnS)
    {
        if (First && OnS == 1)
        {
            return ref pfirstOnS1;
        }
        else if (First && OnS == 2)
        {
            return ref pfirstOnS2;
        }
        else if (!First && OnS == 1)
        {
            return ref plastOnS1;
        }
        else
        {
            return ref plastOnS2;
        }
    }

    public bool IsOnCurve(int OnS)
    {
        if (OnS == 1)
        {
            return isoncurv1;
        }

        return isoncurv2;
    }

    public int IndexOfC(int OnS)
    {
        if (OnS == 1)
        {
            if (!isoncurv1)
            {
                throw new Standard_Failure("Interference pas sur courbe");
            }

            return indexOfC1;
        }

        if (!isoncurv2)
        {
            throw new Standard_Failure("Interference pas sur courbe");
        }

        return indexOfC2;
    }

    public double FirstSpineParam()
    {
        return ufspine;
    }

    public double LastSpineParam()
    {
        return ulspine;
    }

    internal void FirstSpineParam(double Par)
    {
        ufspine = Par;
    }

    internal void LastSpineParam(double Par)
    {
        ulspine = Par;
    }

    public double FirstExtensionValue()
    {
        return myfirstextend;
    }

    public double LastExtensionValue()
    {
        return mylastextend;
    }

    internal void FirstExtensionValue(double Extend)
    {
        myfirstextend = Extend;
    }

    internal void LastExtensionValue(double Extend)
    {
        mylastextend = Extend;
    }

    /// <summary>The simulated sections of the fillet (ChFiDS_SecHArray1, set by the simulation), or null.</summary>
    public Standard_Transient? Simul()
    {
        return simul;
    }

    internal void SetSimul(Standard_Transient? S)
    {
        simul = S;
    }

    internal void ResetSimul()
    {
        simul = null;
    }

    /// <summary>The first or last 2d point on face <paramref name="OnS"/>.</summary>
    public gp_Pnt2d Get2dPoints(bool First, int OnS)
    {
        if (First && OnS == 1)
        {
            return p2df1;
        }
        else if (!First && OnS == 1)
        {
            return p2dl1;
        }
        else if (First && OnS == 2)
        {
            return p2df2;
        }

        return p2dl2;
    }

    public void Get2dPoints(out gp_Pnt2d P2df1, out gp_Pnt2d P2dl1, out gp_Pnt2d P2df2, out gp_Pnt2d P2dl2)
    {
        P2df1 = p2df1;
        P2dl1 = p2dl1;
        P2df2 = p2df2;
        P2dl2 = p2dl2;
    }

    internal void Set2dPoints(in gp_Pnt2d P2df1, in gp_Pnt2d P2dl1, in gp_Pnt2d P2df2, in gp_Pnt2d P2dl2)
    {
        p2df1 = P2df1;
        p2dl1 = P2dl1;
        p2df2 = P2df2;
        p2dl2 = P2dl2;
    }

    public bool TwistOnS1()
    {
        return twistons1;
    }

    public bool TwistOnS2()
    {
        return twistons2;
    }

    internal void TwistOnS1(bool T)
    {
        twistons1 = T;
    }

    internal void TwistOnS2(bool T)
    {
        twistons2 = T;
    }
}
