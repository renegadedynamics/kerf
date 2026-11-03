// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeCylindricalHole.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeCylindricalHole.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeCylindricalHole.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepFeat_Status;
using static Kerf.Engine.Geometry.NormalStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Drills a cylindrical hole in a shape along an axis: through all, through the next faces, up to the end, between two
/// parameters along the axis, or blind for a length.
/// </summary>
/// <remarks>
/// <para>A BRepFeat_Builder: <see cref="BRepFeat_Builder.Init(in Shape)"/> or <see cref="Init(in Shape, in Axis3D)"/>
/// gives the shape, a <c>Perform</c> method the hole, <see cref="Build"/> the result
/// (<see cref="BOPAlgo_BuilderShape.Shape"/>, frozen) and <see cref="Status"/> its status. OCCT makes the Boolean's
/// <c>Perform</c> protected; C# can't narrow it, so BOPAlgo_BOP's stays public.</para>
/// <para>The shape is read, never written: the Booleans are non-destructive (decision 274).</para>
/// </remarks>
public sealed class BRepFeat_MakeCylindricalHole : BRepFeat_Builder
{
    private Axis3D myAxis = new();
    private bool myAxDef;
    private BRepFeat_Status myStatus;
    private bool myIsBlind;
    private bool myValidate;
    private Face myTopFace = new();
    private Face myBotFace = new();

    /// <summary>Empty constructor.</summary>
    public BRepFeat_MakeCylindricalHole()
    {
        myAxDef = false;
        myStatus = BRepFeat_NoError;
        myIsBlind = false;
        myValidate = false;
    }

    /// <summary>Sets the axis of the hole.</summary>
    public void Init(in Axis3D Axis)
    {
        myAxis = Axis;
        myAxDef = true;
    }

    /// <summary>Sets the shape and the axis of the hole.</summary>
    public void Init(in Shape S, in Axis3D Axis)
    {
        base.Init(S);
        myAxis = Axis;
        myAxDef = true;
    }

    /// <summary>Performs a hole of radius <paramref name="Radius"/> through all the shape.</summary>
    /// <exception cref="ConstructionErrorException">No shape or no axis.</exception>
    public void Perform(double Radius)
    {
        Shape aObject = myArguments.First();
        if (aObject.IsNull || !myAxDef)
        {
            throw new ConstructionErrorException();
        }

        myIsBlind = false;
        myStatus = BRepFeat_NoError;

        LocOpe_CurveShapeIntersector theASI = new(myAxis, aObject);
        if (!theASI.IsDone() || theASI.NbPoints() <= 0)
        {
            myStatus = BRepFeat_InvalidPlacement;
            return;
        }

        // It is not possible to use infinite cylinder for topological operations.
        BoxParameters(aObject, myAxis, out double PMin, out double PMax);
        double Heigth = 2.0 * (PMax - PMin);
        Xyz theOrig = myAxis.Location().XYZ();
        theOrig += ((3.0 * PMin - PMax) / 2.0) * myAxis.Direction().XYZ();
        Point3D p1_ao1 = new(theOrig);
        Frame3D a1_ao1 = new(p1_ao1, myAxis.Direction());
        BRepPrim_Cylinder theCylinder = new(a1_ao1, Radius, Heigth);

        // Probably it is better to make cut directly

        BrepBuilder B = new();
        Solid theTool = new();
        B.MakeSolid(ref theTool);
        B.Add(theTool, theCylinder.Shell());

        myTopFace = theCylinder.TopFace();
        myBotFace = theCylinder.BottomFace();
        myValidate = false;

        bool Fuse = false;
        //
        AddTool(theTool);
        SetOperation(Fuse ? 1 : 0);
        base.Perform();
    }

    /// <summary>Performs a hole of radius <paramref name="Radius"/> through the next faces met along the axis.</summary>
    /// <exception cref="ConstructionErrorException">No shape or no axis.</exception>
    public void PerformThruNext(double Radius, bool Cont = true)
    {
        //
        Shape aObject = myArguments.First();
        if (aObject.IsNull || !myAxDef)
        {
            throw new ConstructionErrorException();
        }

        myIsBlind = false;
        myValidate = Cont;
        myStatus = BRepFeat_NoError;

        LocOpe_CurveShapeIntersector theASI = new(myAxis, aObject);
        if (!theASI.IsDone())
        {
            myStatus = BRepFeat_InvalidPlacement;
            return;
        }

        int IndFrom = 0, IndTo = 0;
        Orientation theOr = Forward;
        LocOpe_PntFace PntInfoFirst = new(), PntInfoLast = new();
        bool ok = theASI.LocalizeAfter(0.0, ref theOr, ref IndFrom, ref IndTo);
        if (ok)
        {
            if (theOr == Forward)
            {
                PntInfoFirst = theASI.Point(IndFrom);
                ok = theASI.LocalizeAfter(IndTo, ref theOr, ref IndFrom, ref IndTo);
                if (ok)
                {
                    if (theOr != Reversed)
                    {
                        ok = false;
                    }
                    else
                    {
                        PntInfoLast = theASI.Point(IndTo);
                    }
                }
            }
            else
            { // TopAbs_REVERSED
                PntInfoLast = theASI.Point(IndTo);
                ok = theASI.LocalizeBefore(IndFrom, ref theOr, ref IndFrom, ref IndTo);
                if (ok)
                {
                    if (theOr != Forward)
                    {
                        ok = false;
                    }
                    else
                    {
                        PntInfoFirst = theASI.Point(IndFrom);
                    }
                }
            }
        }
        if (!ok)
        {
            myStatus = BRepFeat_InvalidPlacement;
            return;
        }

        CreateCyl(PntInfoFirst, PntInfoLast, Radius, myAxis, out Shell Cyl, out myTopFace, out myBotFace);

        BrepBuilder B = new();
        Solid theTool = new();
        B.MakeSolid(ref theTool);
        B.Add(theTool, Cyl);

        bool Fuse = false;
        AddTool(theTool);
        SetOperation(Fuse ? 1 : 0);
        base.Perform();
        NCollection_List<Shape> parts = new();
        PartsOfTool(parts);

        int nbparts = 0;
        NCollection_List<Shape>.Iterator its = new(parts);
        for (; its.More(); its.Next())
        {
            nbparts++;
        }
        if (nbparts == 0)
        {
            myStatus = BRepFeat_InvalidPlacement;
            return;
        }

        if (nbparts >= 2)
        { // preserve the smallest as parameter
            // along the axis
            double First = PntInfoFirst.Parameter();
            double Last = PntInfoLast.Parameter();
            Shape tokeep = new();
            double parbar, parmin = Last;
            for (its.Initialize(parts); its.More(); its.Next())
            {
                Baryc(its.Value(), out Point3D Barycentre);
                parbar = global::Kerf.Engine.Geometry.ElCLib.LineParameter(myAxis, Barycentre);
                if (parbar >= First && parbar <= Last && parbar <= parmin)
                {
                    parmin = parbar;
                    tokeep = its.Value();
                }
            }

            if (tokeep.IsNull)
            { // preserve the closest interval

                double dmin = RealLast();
                for (its.Initialize(parts); its.More(); its.Next())
                {
                    Baryc(its.Value(), out Point3D Barycentre);
                    parbar = global::Kerf.Engine.Geometry.ElCLib.LineParameter(myAxis, Barycentre);
                    if (parbar < First)
                    {
                        if (First - parbar < dmin)
                        {
                            dmin = First - parbar;
                            tokeep = its.Value();
                        }
                        else
                        { // parbar > Last (OCCT: the else hangs on the wrong if)
                            if (parbar - Last < dmin)
                            {
                                dmin = parbar - Last;
                                tokeep = its.Value();
                            }
                        }
                    }
                }
            }
            for (its.Initialize(parts); its.More(); its.Next())
            {
                if (tokeep.IsSame(its.Value()))
                {
                    KeepPart(its.Value());
                    break;
                }
            }
        }
    }

    /// <summary>Performs a hole of radius <paramref name="Radius"/> from the first face met up to the end.</summary>
    /// <exception cref="ConstructionErrorException">No shape or no axis.</exception>
    public void PerformUntilEnd(double Radius, bool Cont = true)
    {
        //
        Shape aObject = myArguments.First();
        if (aObject.IsNull || !myAxDef)
        {
            throw new ConstructionErrorException();
        }

        myIsBlind = false;
        myValidate = Cont;
        myStatus = BRepFeat_NoError;

        LocOpe_CurveShapeIntersector theASI = new(myAxis, aObject);
        if (!theASI.IsDone())
        {
            myStatus = BRepFeat_InvalidPlacement;
            return;
        }

        int IndFrom = 0, IndTo = 0;
        Orientation theOr = Forward;
        bool ok = theASI.LocalizeAfter(0.0, ref theOr, ref IndFrom, ref IndTo);
        LocOpe_PntFace PntInfoFirst = new(), PntInfoLast = new();

        if (ok)
        {
            if (theOr == Reversed)
            {
                ok = theASI.LocalizeBefore(IndFrom, ref theOr, ref IndFrom, ref IndTo); // on reset
                // It is possible to search for the next.
            }
            if (ok && theOr == Forward)
            {
                PntInfoFirst = theASI.Point(IndFrom);
                ok = theASI.LocalizeBefore(theASI.NbPoints() + 1, ref theOr, ref IndFrom, ref IndTo);
                if (ok)
                {
                    if (theOr != Reversed)
                    {
                        ok = false;
                    }
                    else
                    {
                        PntInfoLast = theASI.Point(IndTo);
                    }
                }
            }
        }
        if (!ok)
        {
            myStatus = BRepFeat_InvalidPlacement;
            return;
        }

        CreateCyl(PntInfoFirst, PntInfoLast, Radius, myAxis, out Shell Cyl, out myTopFace, out myBotFace);

        BrepBuilder B = new();
        Solid theTool = new();
        B.MakeSolid(ref theTool);
        B.Add(theTool, Cyl);

        bool Fuse = false;
        AddTool(theTool);
        SetOperation(Fuse ? 1 : 0);
        base.Perform();
        NCollection_List<Shape> parts = new();
        PartsOfTool(parts);

        int nbparts = 0;
        NCollection_List<Shape>.Iterator its = new(parts);
        for (; its.More(); its.Next())
        {
            nbparts++;
        }
        if (nbparts == 0)
        {
            myStatus = BRepFeat_InvalidPlacement;
            return;
        }

        if (nbparts >= 2)
        { // preserve everything above the First
            double parbar;
            for (its.Initialize(parts); its.More(); its.Next())
            {
                Baryc(its.Value(), out Point3D Barycentre);
                parbar = global::Kerf.Engine.Geometry.ElCLib.LineParameter(myAxis, Barycentre);
                if (parbar > PntInfoFirst.Parameter())
                {
                    KeepPart(its.Value());
                }
            }
        }
    }

    /// <summary>
    /// Performs a hole of radius <paramref name="Radius"/> between the parameters <paramref name="PFrom"/> and
    /// <paramref name="PTo"/> along the axis.
    /// </summary>
    /// <exception cref="ConstructionErrorException">No shape or no axis.</exception>
    public void Perform(double Radius, double PFrom, double PTo, bool Cont = true)
    {
        //
        Shape aObject = myArguments.First();
        if (aObject.IsNull || !myAxDef)
        {
            throw new ConstructionErrorException();
        }

        myIsBlind = false;
        myValidate = Cont;
        myStatus = BRepFeat_NoError;

        LocOpe_CurveShapeIntersector theASI = new(myAxis, aObject);
        if (!theASI.IsDone())
        {
            myStatus = BRepFeat_InvalidPlacement;
            return;
        }

        double thePFrom, thePTo;
        if (PFrom < PTo)
        {
            thePFrom = PFrom;
            thePTo = PTo;
        }
        else
        {
            thePFrom = PTo;
            thePTo = PFrom;
        }

        LocOpe_PntFace PntInfoFirst = new(), PntInfoLast = new();
        int IndFrom = 0, IndTo = 0;
        Orientation theOr = Forward;
        bool ok = theASI.LocalizeAfter(thePFrom, ref theOr, ref IndFrom, ref IndTo);
        if (ok)
        {
            if (theOr == Reversed)
            {
                ok = theASI.LocalizeBefore(IndFrom, ref theOr, ref IndFrom, ref IndTo); // reset
                // It is possible to find the next.
            }
            if (ok && theOr == Forward)
            {
                PntInfoFirst = theASI.Point(IndFrom);
                ok = theASI.LocalizeBefore(thePTo, ref theOr, ref IndFrom, ref IndTo);
                if (ok)
                {
                    if (theOr == Forward)
                    {
                        ok = theASI.LocalizeAfter(IndTo, ref theOr, ref IndFrom, ref IndTo);
                    }
                    if (ok && theOr == Reversed)
                    {
                        PntInfoLast = theASI.Point(IndTo);
                    }
                }
            }
        }

        if (!ok)
        {
            myStatus = BRepFeat_InvalidPlacement;
            return;
        }

        CreateCyl(PntInfoFirst, PntInfoLast, Radius, myAxis, out Shell Cyl, out myTopFace, out myBotFace);

        BrepBuilder B = new();
        Solid theTool = new();
        B.MakeSolid(ref theTool);
        B.Add(theTool, Cyl);

        bool Fuse = false;
        AddTool(theTool);
        SetOperation(Fuse ? 1 : 0);
        base.Perform();
        NCollection_List<Shape> parts = new();
        PartsOfTool(parts);

        int nbparts = 0;
        NCollection_List<Shape>.Iterator its = new(parts);
        for (; its.More(); its.Next())
        {
            nbparts++;
        }
        if (nbparts == 0)
        {
            myStatus = BRepFeat_InvalidPlacement;
            return;
        }

        if (nbparts >= 2)
        { // preserve parts between First and Last
            double parbar;
            for (its.Initialize(parts); its.More(); its.Next())
            {
                Baryc(its.Value(), out Point3D Barycentre);
                parbar = global::Kerf.Engine.Geometry.ElCLib.LineParameter(myAxis, Barycentre);
                if (parbar >= PntInfoFirst.Parameter() && parbar <= PntInfoLast.Parameter())
                {
                    KeepPart(its.Value());
                }
            }
        }
    }

    /// <summary>Performs a blind hole of radius <paramref name="Radius"/> and length <paramref name="Length"/>.</summary>
    /// <exception cref="ConstructionErrorException">No shape, no axis, or a length not positive.</exception>
    public void PerformBlind(double Radius, double Length, bool Cont = true)
    {
        //
        Shape aObject = myArguments.First();
        if (aObject.IsNull || !myAxDef || Length <= 0.0)
        {
            throw new ConstructionErrorException();
        }

        myIsBlind = true;
        myValidate = Cont;
        myStatus = BRepFeat_NoError;

        LocOpe_CurveShapeIntersector theASI = new(myAxis, aObject);
        if (!theASI.IsDone())
        {
            myStatus = BRepFeat_InvalidPlacement;
            return;
        }

        double First;
        int IndFrom = 0, IndTo = 0;
        Orientation theOr = Forward;
        bool ok = theASI.LocalizeAfter(0.0, ref theOr, ref IndFrom, ref IndTo);

        if (ok)
        {
            if (theOr == Reversed)
            {
                ok = theASI.LocalizeBefore(IndFrom, ref theOr, ref IndFrom, ref IndTo); // reset
                // it is possible to find the next
            }
            ok = ok && theOr == Forward;
        }
        if (!ok)
        {
            myStatus = BRepFeat_InvalidPlacement;
            return;
        }

        // check a priori the length of the hole
        int IFNext = 0, ITNext = 0;
        ok = theASI.LocalizeAfter(IndTo, ref theOr, ref IFNext, ref ITNext);
        if (!ok)
        {
            myStatus = BRepFeat_InvalidPlacement;
            return;
        }
        if (theASI.Point(IFNext).Parameter() <= Length)
        {
            myStatus = BRepFeat_HoleTooLong;
            return;
        }

        NCollection_List<Shape> theList = new();
        // version for advanced control
        for (int i = IndFrom; i <= ITNext; i++)
        {
            theList.Append(theASI.Point(i).Face());
        }

        First = theASI.Point(IndFrom).Parameter();

        //// It is not possible to use infinite cylinder for topological operations.
        BoxParameters(aObject, myAxis, out double PMin, out double PMax);
        if (PMin > Length)
        {
            myStatus = BRepFeat_InvalidPlacement;
            return;
        }

        double Heigth = 3.0 * (Length - PMin) / 2.0;
        Xyz theOrig = myAxis.Location().XYZ();
        theOrig += ((3.0 * PMin - Length) / 2.0) * myAxis.Direction().XYZ();
        Point3D p5_ao1 = new(theOrig);
        Frame3D a5_ao1 = new(p5_ao1, myAxis.Direction());
        BRepPrim_Cylinder theCylinder = new(a5_ao1, Radius, Heigth);

        BrepBuilder B = new();
        Solid theTool = new();
        B.MakeSolid(ref theTool);
        B.Add(theTool, theCylinder.Shell());

        myTopFace = theCylinder.TopFace();
        myBotFace.Nullify();

        bool Fuse = false;
        AddTool(theTool);
        SetOperation(Fuse ? 1 : 0);
        base.Perform();
        NCollection_List<Shape> parts = new();
        PartsOfTool(parts);

        int nbparts = 0;
        NCollection_List<Shape>.Iterator its = new(parts);
        for (; its.More(); its.Next())
        {
            nbparts++;
        }
        if (nbparts == 0)
        {
            myStatus = BRepFeat_InvalidPlacement;
            return;
        }

        if (nbparts >= 2)
        { // preserve the smallest as parameter along the axis
            Shape tokeep = new();
            double parbar, parmin = RealLast();
            for (its.Initialize(parts); its.More(); its.Next())
            {
                Baryc(its.Value(), out Point3D Barycentre);
                parbar = global::Kerf.Engine.Geometry.ElCLib.LineParameter(myAxis, Barycentre);
                if (parbar >= First && parbar <= parmin)
                {
                    parmin = parbar;
                    tokeep = its.Value();
                }
            }

            if (tokeep.IsNull)
            { // preserve the closest interval

                double dmin = RealLast();
                for (its.Initialize(parts); its.More(); its.Next())
                {
                    Baryc(its.Value(), out Point3D Barycentre);
                    parbar = global::Kerf.Engine.Geometry.ElCLib.LineParameter(myAxis, Barycentre);
                    if (Math.Abs(First - parbar) < dmin)
                    {
                        dmin = Math.Abs(First - parbar);
                        tokeep = its.Value();
                    }
                }
            }
            for (its.Initialize(parts); its.More(); its.Next())
            {
                if (tokeep.IsSame(its.Value()))
                {
                    KeepPart(its.Value());
                    break;
                }
            }
        }
    }

    /// <summary>The status of the last construction.</summary>
    public BRepFeat_Status Status() => myStatus;

    /// <summary>Builds the result of the hole (then <see cref="BOPAlgo_BuilderShape.Shape"/>) and checks it.</summary>
    public void Build()
    {
        if (myStatus == BRepFeat_NoError)
        {
            PerformResult();
            if (!HasErrors())
            {
                myStatus = myValidate ? Validate() : BRepFeat_NoError;
                if (myStatus == BRepFeat_NoError)
                {
                    myShape = InternalShape();
                }
            }
            else
            {
                myStatus = BRepFeat_InvalidPlacement; // why not
            }
        }
    }

    private BRepFeat_Status Validate()
    {
        BRepFeat_Status thestat = BRepFeat_NoError;
        Explorer ex = new(InternalShape(), ShapeType.Face);
        if (myIsBlind)
        { // limit of the hole
            for (; ex.More(); ex.Next())
            {
                if (ex.Current().IsSame(myTopFace))
                {
                    break;
                }
            }
            if (!ex.More())
            {
                thestat = BRepFeat_HoleTooLong;
            }
        }
        else
        {
            for (; ex.More(); ex.Next())
            {
                if (ex.Current().IsSame(myTopFace))
                {
                    return BRepFeat_InvalidPlacement;
                }
            }
            for (ex.ReInit(); ex.More(); ex.Next())
            {
                if (ex.Current().IsSame(myBotFace))
                {
                    return BRepFeat_InvalidPlacement;
                }
            }
        }
        return thestat;
    }

    private static void Baryc(in Shape S, out Point3D B)
    {
        Explorer exp = new(S, ShapeType.Edge);
        Xyz Bar = new(0.0, 0.0, 0.0);
        Curve3D C;
        double prm;
        int i, nbp = 0;

        for (; exp.More(); exp.Next())
        {
            // Calculate points by non-degenerated edges
            Edge E = ShapeCasts.Edge(exp.Current());
            if (!Tool.Degenerated(E))
            {
                C = Tool.Curve(E, out Placement L, out double First, out double Last)!;
                C = (Curve3D)C.Transformed(L.Transformation());

                for (i = 1; i <= 11; i++)
                {
                    prm = ((11 - i) * First + (i - 1) * Last) / 10.0;
                    Bar += C.Value(prm).XYZ();
                    nbp++;
                }
            }
        }
        Bar.Divide((double)nbp);
        B = new Point3D();
        B.SetXYZ(Bar);
    }

    private static void BoxParameters(in Shape S, in Axis3D Axis, out double parmin, out double parmax)
    {
        // calculate the parameters of a bounding box in the direction of the axis of the hole
        Box B = new();
        BRepBndLib.Add(S, ref B);
        double[] c = new double[6];
        B.Get(out c[0], out c[2], out c[4], out c[1], out c[3], out c[5]);
        Point3D P = new();
        int i, j, k;
        parmin = RealLast();
        parmax = RealFirst();
        double param;
        for (i = 0; i <= 1; i++)
        {
            P.SetX(c[i]);
            for (j = 2; j <= 3; j++)
            {
                P.SetY(c[j]);
                for (k = 4; k <= 5; k++)
                {
                    P.SetZ(c[k]);
                    param = global::Kerf.Engine.Geometry.ElCLib.LineParameter(Axis, P);
                    parmin = Min(param, parmin);
                    parmax = Max(param, parmax);
                }
            }
        }
    }

    private static bool GetOffset(in LocOpe_PntFace PntInfo, double Radius, in Axis3D Axis, out double outOff)
    {
        outOff = 0.0; // OCCT leaves it unchanged when false
        Face FF = PntInfo.Face();
        FaceSurfaceAdaptor FFA = new(FF);

        double Up = PntInfo.UParameter();
        double Vp = PntInfo.VParameter();
        FFA.D1(Up, Vp, out Point3D PP, out Vector3D D1U, out Vector3D D1V);
        Direction3D NormF = new();
        global::Kerf.Engine.Geometry.CSLib.Normal(D1U, D1V, global::Kerf.Engine.Foundation.Precision.Angular(), out NormalStatus stat, ref NormF);
        if (stat != Defined)
        {
            return false;
        }
        double angle = Axis.Direction().Angle(NormF);
        if (Math.Abs(M_PI / 2.0 - angle) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return false;
        }
        outOff = Radius * Math.Abs(Math.Tan(angle));
        return true;
    }

    private static void CreateCyl(in LocOpe_PntFace PntInfoFirst, in LocOpe_PntFace PntInfoLast, double Radius,
                                  in Axis3D Axis, out Shell Cyl, out Face CylTopF,
                                  out Face CylBottF)
    {
        double First = 0, Last = 0;
        double offF = 0.0, offL = 0.0;
        Last = PntInfoLast.Parameter();
        First = PntInfoFirst.Parameter();
        double Heigth = Last - First;

        if (!GetOffset(PntInfoFirst, Radius, Axis, out offF))
        {
            offF = Radius;
        }
        if (!GetOffset(PntInfoLast, Radius, Axis, out offL))
        {
            offL = Radius;
        }

        // create cylinder along the axis (myAxis);
        // from 'First - offF' to 'Last + offL' params
        Xyz theOrig = PntInfoFirst.Pnt().XYZ() - offF * Axis.Direction().XYZ();
        Point3D p2_ao1 = new(theOrig);
        Frame3D a2_ao1 = new(p2_ao1, Axis.Direction());
        BRepPrim_Cylinder theCylinder = new(a2_ao1, Radius, Heigth + offF + offL);
        Cyl = theCylinder.Shell();
        CylTopF = theCylinder.TopFace();
        CylBottF = theCylinder.BottomFace();
    }
}
