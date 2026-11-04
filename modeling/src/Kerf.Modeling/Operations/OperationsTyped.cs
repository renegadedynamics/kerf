namespace Kerf.Modeling;

// The operations whose result follows the dimension of what they are given, typed for the algebra's classes: a mirrored
// Part is a Part, a scaled Sketch a Sketch. The forms that take any shapes (and a mix of dimensions) return a Compound,
// which is a Part, Sketch or Curve at run time when the shapes share a dimension.
public static partial class Operations
{
    /// <summary>The part mirrored about a plane (by default Plane.XZ).</summary>
    public static Part Mirror(Part obj, Plane? about = null) => Mirror([(Shape)obj], about).As<Part>();

    /// <summary>The sketch mirrored about a plane (by default Plane.XZ).</summary>
    public static Sketch Mirror(Sketch obj, Plane? about = null) => Mirror([(Shape)obj], about).As<Sketch>();

    /// <summary>The curve mirrored about a plane (by default Plane.XZ).</summary>
    public static Curve Mirror(Curve obj, Plane? about = null) => Mirror([(Shape)obj], about).As<Curve>();

    /// <summary>The part scaled by a factor about <paramref name="about"/> or, by default, its location.</summary>
    public static Part Scale(Part obj, double by, Vector? about = null) => Scale([(Shape)obj], by, about).As<Part>();

    /// <summary>The sketch scaled by a factor about <paramref name="about"/> or, by default, its location.</summary>
    public static Sketch Scale(Sketch obj, double by, Vector? about = null) => Scale([(Shape)obj], by, about).As<Sketch>();

    /// <summary>The curve scaled by a factor about <paramref name="about"/> or, by default, its location.</summary>
    public static Curve Scale(Curve obj, double by, Vector? about = null) => Scale([(Shape)obj], by, about).As<Curve>();

    /// <summary>The part scaled by a factor along each axis.</summary>
    public static Part Scale(Part obj, Vector by, Vector? about = null) => Scale([(Shape)obj], by, about).As<Part>();

    /// <summary>The sketch scaled by a factor along each axis.</summary>
    public static Sketch Scale(Sketch obj, Vector by, Vector? about = null) => Scale([(Shape)obj], by, about).As<Sketch>();

    /// <summary>The curve scaled by a factor along each axis.</summary>
    public static Curve Scale(Curve obj, Vector by, Vector? about = null) => Scale([(Shape)obj], by, about).As<Curve>();

    /// <summary>The part split by a plane (by default Plane.XZ), keeping the pieces <paramref name="keep"/> says.</summary>
    public static Part Split(Part obj, Plane? bisectBy = null, Keep keep = Keep.Top) => Split([(Shape)obj], bisectBy, keep).As<Part>();

    /// <summary>The sketch split by a plane (by default Plane.XZ), keeping the pieces <paramref name="keep"/> says.</summary>
    public static Sketch Split(Sketch obj, Plane? bisectBy = null, Keep keep = Keep.Top) => Split([(Shape)obj], bisectBy, keep).As<Sketch>();

    /// <summary>The curve split by a plane (by default Plane.XZ), keeping the pieces <paramref name="keep"/> says.</summary>
    public static Curve Split(Curve obj, Plane? bisectBy = null, Keep keep = Keep.Top) => Split([(Shape)obj], bisectBy, keep).As<Curve>();

    /// <summary>The part split by a face or shell, keeping the pieces <paramref name="keep"/> says.</summary>
    public static Part Split(Part obj, Shape2D bisectBy, Keep keep = Keep.Top) => Split([(Shape)obj], bisectBy, keep).As<Part>();

    /// <summary>The sketch split by a face or shell, keeping the pieces <paramref name="keep"/> says.</summary>
    public static Sketch Split(Sketch obj, Shape2D bisectBy, Keep keep = Keep.Top) => Split([(Shape)obj], bisectBy, keep).As<Sketch>();

    /// <summary>The curve split by a face or shell, keeping the pieces <paramref name="keep"/> says.</summary>
    public static Curve Split(Curve obj, Shape2D bisectBy, Keep keep = Keep.Top) => Split([(Shape)obj], bisectBy, keep).As<Curve>();
}
