// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeCylindricalHole.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeCylindricalHole.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeCylindricalHole.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Construction.Primitives;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Features.FeatureStatus;
using static Kerf.Engine.Geometry.NormalStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features;

/// <summary>
/// Drills a cylindrical hole in a shape along an axis: through all, through the next faces, up to the end, between two
/// parameters along the axis, or blind for a length.
/// </summary>
/// <remarks>
/// <para>A BRepFeat_Builder: <see cref="FeatureBuilder.Init(in Shape)"/> or <see cref="Init(in Shape, in Axis3D)"/>
/// gives the shape, a <c>Perform</c> method the hole, <see cref="Build"/> the result
/// (<see cref="BuilderShape.Shape"/>, frozen) and <see cref="Status"/> its status. OCCT makes the Boolean's
/// <c>Perform</c> protected; C# can't narrow it, so BOPAlgo_BOP's stays public.</para>
/// <para>The shape is read, never written: the Booleans are non-destructive (decision 274).</para>
/// </remarks>
public sealed class MakeCylindricalHole : FeatureBuilder
{
    private Axis3D _axis = new();
    private bool _axDef;
    private FeatureStatus _status;
    private bool _isBlind;
    private bool _validate;
    private Face _topFace = new();
    private Face _botFace = new();

    /// <summary>Empty constructor.</summary>
    public MakeCylindricalHole()
    {
        _axDef = false;
        _status = FeatureStatus.NoError;
        _isBlind = false;
        _validate = false;
    }

    /// <summary>Sets the axis of the hole.</summary>
    public void Init(in Axis3D axis)
    {
        _axis = axis;
        _axDef = true;
    }

    /// <summary>Sets the shape and the axis of the hole.</summary>
    public void Init(in Shape s, in Axis3D axis)
    {
        base.Init(s);
        _axis = axis;
        _axDef = true;
    }

    /// <summary>Performs a hole of radius <paramref name="radius"/> through all the shape.</summary>
    /// <exception cref="ConstructionErrorException">No shape or no axis.</exception>
    public void Perform(double radius)
    {
        Shape aObject = _arguments.First();
        if (aObject.IsNull || !_axDef)
        {
            throw new ConstructionErrorException();
        }

        _isBlind = false;
        _status = FeatureStatus.NoError;

        LocalCurveShapeIntersector theASI = new(_axis, aObject);
        if (!theASI.IsDone || theASI.PointCount <= 0)
        {
            _status = FeatureStatus.InvalidPlacement;
            return;
        }

        // It is not possible to use infinite cylinder for topological operations.
        BoxParameters(aObject, _axis, out double pMin, out double pMax);
        double heigth = 2.0 * (pMax - pMin);
        Xyz theOrig = _axis.Location.XYZ();
        theOrig += ((3.0 * pMin - pMax) / 2.0) * _axis.Direction.XYZ();
        Point3D p1_ao1 = new(theOrig);
        Frame3D a1_ao1 = new(p1_ao1, _axis.Direction);
        PrimitiveCylinder theCylinder = new(a1_ao1, radius, heigth);

        // Probably it is better to make cut directly

        BrepBuilder b = new();
        Solid theTool = new();
        b.MakeSolid(ref theTool);
        b.Add(theTool, theCylinder.Shell());

        _topFace = theCylinder.TopFace();
        _botFace = theCylinder.BottomFace();
        _validate = false;

        bool fuse = false;
        //
        AddTool(theTool);
        SetOperation(fuse ? 1 : 0);
        base.Perform();
    }

    /// <summary>Performs a hole of radius <paramref name="radius"/> through the next faces met along the axis.</summary>
    /// <exception cref="ConstructionErrorException">No shape or no axis.</exception>
    public void PerformThruNext(double radius, bool cont = true)
    {
        //
        Shape aObject = _arguments.First();
        if (aObject.IsNull || !_axDef)
        {
            throw new ConstructionErrorException();
        }

        _isBlind = false;
        _validate = cont;
        _status = FeatureStatus.NoError;

        LocalCurveShapeIntersector theASI = new(_axis, aObject);
        if (!theASI.IsDone)
        {
            _status = FeatureStatus.InvalidPlacement;
            return;
        }

        int indFrom = 0, indTo = 0;
        Orientation theOr = Forward;
        PntFace pntInfoFirst = new(), pntInfoLast = new();
        bool ok = theASI.LocalizeAfter(0.0, ref theOr, ref indFrom, ref indTo);
        if (ok)
        {
            if (theOr == Forward)
            {
                pntInfoFirst = theASI.Point(indFrom);
                ok = theASI.LocalizeAfter(indTo, ref theOr, ref indFrom, ref indTo);
                if (ok)
                {
                    if (theOr != Reversed)
                    {
                        ok = false;
                    }
                    else
                    {
                        pntInfoLast = theASI.Point(indTo);
                    }
                }
            }
            else
            { // TopAbs_REVERSED
                pntInfoLast = theASI.Point(indTo);
                ok = theASI.LocalizeBefore(indFrom, ref theOr, ref indFrom, ref indTo);
                if (ok)
                {
                    if (theOr != Forward)
                    {
                        ok = false;
                    }
                    else
                    {
                        pntInfoFirst = theASI.Point(indFrom);
                    }
                }
            }
        }
        if (!ok)
        {
            _status = FeatureStatus.InvalidPlacement;
            return;
        }

        CreateCyl(pntInfoFirst, pntInfoLast, radius, _axis, out Shell cyl, out _topFace, out _botFace);

        BrepBuilder b = new();
        Solid theTool = new();
        b.MakeSolid(ref theTool);
        b.Add(theTool, cyl);

        bool fuse = false;
        AddTool(theTool);
        SetOperation(fuse ? 1 : 0);
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
            _status = FeatureStatus.InvalidPlacement;
            return;
        }

        if (nbparts >= 2)
        { // preserve the smallest as parameter
            // along the axis
            double first = pntInfoFirst.Parameter();
            double last = pntInfoLast.Parameter();
            Shape tokeep = new();
            double parbar, parmin = last;
            for (its.Initialize(parts); its.More(); its.Next())
            {
                Baryc(its.Value(), out Point3D barycentre);
                parbar = global::Kerf.Engine.Geometry.ElCLib.LineParameter(_axis, barycentre);
                if (parbar >= first && parbar <= last && parbar <= parmin)
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
                    Baryc(its.Value(), out Point3D barycentre);
                    parbar = global::Kerf.Engine.Geometry.ElCLib.LineParameter(_axis, barycentre);
                    if (parbar < first)
                    {
                        if (first - parbar < dmin)
                        {
                            dmin = first - parbar;
                            tokeep = its.Value();
                        }
                        else
                        { // parbar > Last (OCCT: the else hangs on the wrong if)
                            if (parbar - last < dmin)
                            {
                                dmin = parbar - last;
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

    /// <summary>Performs a hole of radius <paramref name="radius"/> from the first face met up to the end.</summary>
    /// <exception cref="ConstructionErrorException">No shape or no axis.</exception>
    public void PerformUntilEnd(double radius, bool cont = true)
    {
        //
        Shape aObject = _arguments.First();
        if (aObject.IsNull || !_axDef)
        {
            throw new ConstructionErrorException();
        }

        _isBlind = false;
        _validate = cont;
        _status = FeatureStatus.NoError;

        LocalCurveShapeIntersector theASI = new(_axis, aObject);
        if (!theASI.IsDone)
        {
            _status = FeatureStatus.InvalidPlacement;
            return;
        }

        int indFrom = 0, indTo = 0;
        Orientation theOr = Forward;
        bool ok = theASI.LocalizeAfter(0.0, ref theOr, ref indFrom, ref indTo);
        PntFace pntInfoFirst = new(), pntInfoLast = new();

        if (ok)
        {
            if (theOr == Reversed)
            {
                ok = theASI.LocalizeBefore(indFrom, ref theOr, ref indFrom, ref indTo); // on reset
                // It is possible to search for the next.
            }
            if (ok && theOr == Forward)
            {
                pntInfoFirst = theASI.Point(indFrom);
                ok = theASI.LocalizeBefore(theASI.PointCount + 1, ref theOr, ref indFrom, ref indTo);
                if (ok)
                {
                    if (theOr != Reversed)
                    {
                        ok = false;
                    }
                    else
                    {
                        pntInfoLast = theASI.Point(indTo);
                    }
                }
            }
        }
        if (!ok)
        {
            _status = FeatureStatus.InvalidPlacement;
            return;
        }

        CreateCyl(pntInfoFirst, pntInfoLast, radius, _axis, out Shell cyl, out _topFace, out _botFace);

        BrepBuilder b = new();
        Solid theTool = new();
        b.MakeSolid(ref theTool);
        b.Add(theTool, cyl);

        bool fuse = false;
        AddTool(theTool);
        SetOperation(fuse ? 1 : 0);
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
            _status = FeatureStatus.InvalidPlacement;
            return;
        }

        if (nbparts >= 2)
        { // preserve everything above the First
            double parbar;
            for (its.Initialize(parts); its.More(); its.Next())
            {
                Baryc(its.Value(), out Point3D barycentre);
                parbar = global::Kerf.Engine.Geometry.ElCLib.LineParameter(_axis, barycentre);
                if (parbar > pntInfoFirst.Parameter())
                {
                    KeepPart(its.Value());
                }
            }
        }
    }

    /// <summary>
    /// Performs a hole of radius <paramref name="radius"/> between the parameters <paramref name="pFrom"/> and
    /// <paramref name="pTo"/> along the axis.
    /// </summary>
    /// <exception cref="ConstructionErrorException">No shape or no axis.</exception>
    public void Perform(double radius, double pFrom, double pTo, bool cont = true)
    {
        //
        Shape aObject = _arguments.First();
        if (aObject.IsNull || !_axDef)
        {
            throw new ConstructionErrorException();
        }

        _isBlind = false;
        _validate = cont;
        _status = FeatureStatus.NoError;

        LocalCurveShapeIntersector theASI = new(_axis, aObject);
        if (!theASI.IsDone)
        {
            _status = FeatureStatus.InvalidPlacement;
            return;
        }

        double thePFrom, thePTo;
        if (pFrom < pTo)
        {
            thePFrom = pFrom;
            thePTo = pTo;
        }
        else
        {
            thePFrom = pTo;
            thePTo = pFrom;
        }

        PntFace pntInfoFirst = new(), pntInfoLast = new();
        int indFrom = 0, indTo = 0;
        Orientation theOr = Forward;
        bool ok = theASI.LocalizeAfter(thePFrom, ref theOr, ref indFrom, ref indTo);
        if (ok)
        {
            if (theOr == Reversed)
            {
                ok = theASI.LocalizeBefore(indFrom, ref theOr, ref indFrom, ref indTo); // reset
                // It is possible to find the next.
            }
            if (ok && theOr == Forward)
            {
                pntInfoFirst = theASI.Point(indFrom);
                ok = theASI.LocalizeBefore(thePTo, ref theOr, ref indFrom, ref indTo);
                if (ok)
                {
                    if (theOr == Forward)
                    {
                        ok = theASI.LocalizeAfter(indTo, ref theOr, ref indFrom, ref indTo);
                    }
                    if (ok && theOr == Reversed)
                    {
                        pntInfoLast = theASI.Point(indTo);
                    }
                }
            }
        }

        if (!ok)
        {
            _status = FeatureStatus.InvalidPlacement;
            return;
        }

        CreateCyl(pntInfoFirst, pntInfoLast, radius, _axis, out Shell cyl, out _topFace, out _botFace);

        BrepBuilder b = new();
        Solid theTool = new();
        b.MakeSolid(ref theTool);
        b.Add(theTool, cyl);

        bool fuse = false;
        AddTool(theTool);
        SetOperation(fuse ? 1 : 0);
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
            _status = FeatureStatus.InvalidPlacement;
            return;
        }

        if (nbparts >= 2)
        { // preserve parts between First and Last
            double parbar;
            for (its.Initialize(parts); its.More(); its.Next())
            {
                Baryc(its.Value(), out Point3D barycentre);
                parbar = global::Kerf.Engine.Geometry.ElCLib.LineParameter(_axis, barycentre);
                if (parbar >= pntInfoFirst.Parameter() && parbar <= pntInfoLast.Parameter())
                {
                    KeepPart(its.Value());
                }
            }
        }
    }

    /// <summary>Performs a blind hole of radius <paramref name="radius"/> and length <paramref name="length"/>.</summary>
    /// <exception cref="ConstructionErrorException">No shape, no axis, or a length not positive.</exception>
    public void PerformBlind(double radius, double length, bool cont = true)
    {
        //
        Shape aObject = _arguments.First();
        if (aObject.IsNull || !_axDef || length <= 0.0)
        {
            throw new ConstructionErrorException();
        }

        _isBlind = true;
        _validate = cont;
        _status = FeatureStatus.NoError;

        LocalCurveShapeIntersector theASI = new(_axis, aObject);
        if (!theASI.IsDone)
        {
            _status = FeatureStatus.InvalidPlacement;
            return;
        }

        double first;
        int indFrom = 0, indTo = 0;
        Orientation theOr = Forward;
        bool ok = theASI.LocalizeAfter(0.0, ref theOr, ref indFrom, ref indTo);

        if (ok)
        {
            if (theOr == Reversed)
            {
                ok = theASI.LocalizeBefore(indFrom, ref theOr, ref indFrom, ref indTo); // reset
                // it is possible to find the next
            }
            ok = ok && theOr == Forward;
        }
        if (!ok)
        {
            _status = FeatureStatus.InvalidPlacement;
            return;
        }

        // check a priori the length of the hole
        int ifNext = 0, itNext = 0;
        ok = theASI.LocalizeAfter(indTo, ref theOr, ref ifNext, ref itNext);
        if (!ok)
        {
            _status = FeatureStatus.InvalidPlacement;
            return;
        }
        if (theASI.Point(ifNext).Parameter() <= length)
        {
            _status = FeatureStatus.HoleTooLong;
            return;
        }

        NCollection_List<Shape> theList = new();
        // version for advanced control
        for (int i = indFrom; i <= itNext; i++)
        {
            theList.Append(theASI.Point(i).Face());
        }

        first = theASI.Point(indFrom).Parameter();

        //// It is not possible to use infinite cylinder for topological operations.
        BoxParameters(aObject, _axis, out double pMin, out double pMax);
        if (pMin > length)
        {
            _status = FeatureStatus.InvalidPlacement;
            return;
        }

        double heigth = 3.0 * (length - pMin) / 2.0;
        Xyz theOrig = _axis.Location.XYZ();
        theOrig += ((3.0 * pMin - length) / 2.0) * _axis.Direction.XYZ();
        Point3D p5_ao1 = new(theOrig);
        Frame3D a5_ao1 = new(p5_ao1, _axis.Direction);
        PrimitiveCylinder theCylinder = new(a5_ao1, radius, heigth);

        BrepBuilder b = new();
        Solid theTool = new();
        b.MakeSolid(ref theTool);
        b.Add(theTool, theCylinder.Shell());

        _topFace = theCylinder.TopFace();
        _botFace.Nullify();

        bool fuse = false;
        AddTool(theTool);
        SetOperation(fuse ? 1 : 0);
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
            _status = FeatureStatus.InvalidPlacement;
            return;
        }

        if (nbparts >= 2)
        { // preserve the smallest as parameter along the axis
            Shape tokeep = new();
            double parbar, parmin = RealLast();
            for (its.Initialize(parts); its.More(); its.Next())
            {
                Baryc(its.Value(), out Point3D barycentre);
                parbar = global::Kerf.Engine.Geometry.ElCLib.LineParameter(_axis, barycentre);
                if (parbar >= first && parbar <= parmin)
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
                    Baryc(its.Value(), out Point3D barycentre);
                    parbar = global::Kerf.Engine.Geometry.ElCLib.LineParameter(_axis, barycentre);
                    if (Math.Abs(first - parbar) < dmin)
                    {
                        dmin = Math.Abs(first - parbar);
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
    public FeatureStatus Status() => _status;

    /// <summary>Builds the result of the hole (then <see cref="BuilderShape.Shape"/>) and checks it.</summary>
    public void Build()
    {
        if (_status == FeatureStatus.NoError)
        {
            PerformResult();
            if (!HasErrors)
            {
                _status = _validate ? Validate() : FeatureStatus.NoError;
                if (_status == FeatureStatus.NoError)
                {
                    _shape = InternalShape();
                }
            }
            else
            {
                _status = FeatureStatus.InvalidPlacement; // why not
            }
        }
    }

    private FeatureStatus Validate()
    {
        FeatureStatus thestat = FeatureStatus.NoError;
        Explorer ex = new(InternalShape(), ShapeType.Face);
        if (_isBlind)
        { // limit of the hole
            for (; ex.More(); ex.Next())
            {
                if (ex.Current().IsSame(_topFace))
                {
                    break;
                }
            }
            if (!ex.More())
            {
                thestat = FeatureStatus.HoleTooLong;
            }
        }
        else
        {
            for (; ex.More(); ex.Next())
            {
                if (ex.Current().IsSame(_topFace))
                {
                    return FeatureStatus.InvalidPlacement;
                }
            }
            for (ex.ReInit(); ex.More(); ex.Next())
            {
                if (ex.Current().IsSame(_botFace))
                {
                    return FeatureStatus.InvalidPlacement;
                }
            }
        }
        return thestat;
    }

    private static void Baryc(in Shape s, out Point3D b)
    {
        Explorer exp = new(s, ShapeType.Edge);
        Xyz bar = new(0.0, 0.0, 0.0);
        Curve3D c;
        double prm;
        int i, nbp = 0;

        for (; exp.More(); exp.Next())
        {
            // Calculate points by non-degenerated edges
            Edge e = ShapeCasts.Edge(exp.Current());
            if (!Tool.Degenerated(e))
            {
                c = Tool.Curve(e, out Placement l, out double first, out double last)!;
                c = (Curve3D)c.Transformed(l.Transformation);

                for (i = 1; i <= 11; i++)
                {
                    prm = ((11 - i) * first + (i - 1) * last) / 10.0;
                    bar += c.Value(prm).XYZ();
                    nbp++;
                }
            }
        }
        bar.Divide((double)nbp);
        b = new Point3D();
        b = b.WithXYZ(bar);
    }

    private static void BoxParameters(in Shape s, in Axis3D axis, out double parmin, out double parmax)
    {
        // calculate the parameters of a bounding box in the direction of the axis of the hole
        Box b = new();
        Bounds.Add(s, ref b);
        double[] c = new double[6];
        b.Get(out c[0], out c[2], out c[4], out c[1], out c[3], out c[5]);
        Point3D p = new();
        int i, j, k;
        parmin = RealLast();
        parmax = RealFirst();
        double param;
        for (i = 0; i <= 1; i++)
        {
            p = p.WithX(c[i]);
            for (j = 2; j <= 3; j++)
            {
                p = p.WithY(c[j]);
                for (k = 4; k <= 5; k++)
                {
                    p = p.WithZ(c[k]);
                    param = global::Kerf.Engine.Geometry.ElCLib.LineParameter(axis, p);
                    parmin = Min(param, parmin);
                    parmax = Max(param, parmax);
                }
            }
        }
    }

    private static bool GetOffset(in PntFace pntInfo, double radius, in Axis3D axis, out double outOff)
    {
        outOff = 0.0; // OCCT leaves it unchanged when false
        Face ff = pntInfo.Face();
        FaceSurfaceAdaptor ffa = new(ff);

        double up = pntInfo.UParameter();
        double vp = pntInfo.VParameter();
        ffa.D1(up, vp, out Point3D pp, out Vector3D d1U, out Vector3D d1V);
        Direction3D normF = new();
        global::Kerf.Engine.Geometry.CSLib.Normal(d1U, d1V, global::Kerf.Engine.Foundation.Precision.Angular(), out NormalStatus stat, ref normF);
        if (stat != Defined)
        {
            return false;
        }
        double angle = axis.Direction.AngleTo(normF);
        if (Math.Abs(M_PI / 2.0 - angle) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return false;
        }
        outOff = radius * Math.Abs(Math.Tan(angle));
        return true;
    }

    private static void CreateCyl(in PntFace pntInfoFirst, in PntFace pntInfoLast, double radius,
                                  in Axis3D axis, out Shell cyl, out Face cylTopF,
                                  out Face cylBottF)
    {
        double first = 0, last = 0;
        double offF = 0.0, offL = 0.0;
        last = pntInfoLast.Parameter();
        first = pntInfoFirst.Parameter();
        double heigth = last - first;

        if (!GetOffset(pntInfoFirst, radius, axis, out offF))
        {
            offF = radius;
        }
        if (!GetOffset(pntInfoLast, radius, axis, out offL))
        {
            offL = radius;
        }

        // create cylinder along the axis (myAxis);
        // from 'First - offF' to 'Last + offL' params
        Xyz theOrig = pntInfoFirst.Pnt().XYZ() - offF * axis.Direction.XYZ();
        Point3D p2_ao1 = new(theOrig);
        Frame3D a2_ao1 = new(p2_ao1, axis.Direction);
        PrimitiveCylinder theCylinder = new(a2_ao1, radius, heigth + offF + offL);
        cyl = theCylinder.Shell();
        cylTopF = theCylinder.TopFace();
        cylBottF = theCylinder.BottomFace();
    }
}
