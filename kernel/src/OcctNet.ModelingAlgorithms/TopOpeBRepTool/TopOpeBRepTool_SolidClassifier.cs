// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_SolidClassifier.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_SolidClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Classifies points against solids and shells, keeping one <see cref="BRepClass3d_SolidClassifier"/> per solid or
/// shell met.
/// </summary>
/// <remarks>A working object. OCCT stores raw pointers to the classifiers and deletes them in its destructor; the port
/// stores references.</remarks>
public sealed class TopOpeBRepTool_SolidClassifier
{
    private BRepClass3d_SolidClassifier? myPClassifier;
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, BRepClass3d_SolidClassifier> myShapeClassifierMap =
        new(TopTools_ShapeMapHasher.Instance);
    private TopAbs_State myState;
    private TopoDS_Shell myShell = new();
    private TopoDS_Solid mySolid = new();
    private readonly BRep_Builder myBuilder = new();

    /// <summary>An empty classifier.</summary>
    public TopOpeBRepTool_SolidClassifier()
    {
        Clear();
    }

    /// <summary>Forgets the solids and shells met.</summary>
    public void Clear()
    {
        myPClassifier = null;
        // modified by NIZNHY-PKV Mon Dec 16 10:46:04 2002 f
        myShapeClassifierMap.Clear();
        // modified by NIZNHY-PKV Mon Dec 16 10:46:06 2002 t
        myState = TopAbs_UNKNOWN;
        myShell.Nullify();
        mySolid.Nullify();
    }

    /// <summary>Makes <paramref name="SOL"/>'s classifier current, creating it on first use.</summary>
    public void LoadSolid(in TopoDS_Solid SOL)
    {
        bool found = myShapeClassifierMap.Contains(SOL);
        if (!found)
        {
            myPClassifier = new BRepClass3d_SolidClassifier(SOL);
            myShapeClassifierMap.Add(SOL, myPClassifier);
        }
        else
        {
            myPClassifier = myShapeClassifierMap.ChangeFromKey(SOL);
        }
    }

    /// <summary>
    /// The state of <paramref name="P"/> against the solid <paramref name="SOL"/>, with the state of an EXTERNAL or
    /// INTERNAL face found by the classifier taken into account.
    /// </summary>
    public TopAbs_State Classify(in TopoDS_Solid SOL, in Point3D P, double Tol)
    {
        myPClassifier = null;
        myState = TopAbs_UNKNOWN;
        LoadSolid(SOL);
        if (myPClassifier is null)
        {
            return myState;
        }
        myPClassifier.Perform(P, Tol);
        myState = myPClassifier.State();
        TopoDS_Shape fres = myPClassifier.Face();
        if (fres.IsNull())
        {
            // NYI: in case of elimination of EXTERNAL and INTERNAL faces by the
            // BRepClass3d_SolidClassifier classifier, still handle these faces
            // to generate ON/Solid state when the point is IN/face INTERNAL or EXTERNAL
            return myState;
        }
        TopAbs_Orientation ofres = fres.Orientation();
        if (ofres == TopAbs_EXTERNAL)
        {
            if (myState == TopAbs_IN)
            {
                myState = TopAbs_OUT;
            }
            else if (myState == TopAbs_OUT)
            {
                myState = TopAbs_OUT;
            }
            else if (myState == TopAbs_ON)
            {
                myState = TopAbs_ON;
            }
            else if (myState == TopAbs_UNKNOWN)
            {
                myState = TopAbs_OUT;
            }
        }
        else if (ofres == TopAbs_INTERNAL)
        {
            if (myState == TopAbs_IN)
            {
                myState = TopAbs_IN;
            }
            else if (myState == TopAbs_OUT)
            {
                myState = TopAbs_IN;
            }
            else if (myState == TopAbs_ON)
            {
                myState = TopAbs_ON;
            }
            else if (myState == TopAbs_UNKNOWN)
            {
                myState = TopAbs_IN;
            }
        }
        return myState;
    }

    /// <summary>
    /// Makes <paramref name="SHE"/>'s classifier current, creating it on first use on a solid made of the shell.
    /// </summary>
    public void LoadShell(in TopoDS_Shell SHE)
    {
        bool found = myShapeClassifierMap.Contains(SHE);
        if (!found)
        {
            myBuilder.MakeSolid(ref mySolid);
            myBuilder.Add(mySolid, SHE);
            myPClassifier = new BRepClass3d_SolidClassifier(mySolid);
            myShapeClassifierMap.Add(SHE, myPClassifier);
        }
        else
        {
            myPClassifier = myShapeClassifierMap.ChangeFromKey(SHE);
        }
    }

    /// <summary>The state of <paramref name="P"/> against the solid bounded by the shell <paramref name="SHE"/>.</summary>
    public TopAbs_State Classify(in TopoDS_Shell SHE, in Point3D P, double Tol)
    {
        myPClassifier = null;
        myState = TopAbs_UNKNOWN;
        LoadShell(SHE);
        if (myPClassifier is null)
        {
            return myState;
        }
        myPClassifier.Perform(P, Tol);
        myState = myPClassifier.State();
        return myState;
    }

    /// <summary>The state found by the last classification.</summary>
    public TopAbs_State State() => myState;
}
